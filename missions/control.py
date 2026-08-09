"""Mission sequencing state machine — drives navigate's existing HTTP
API (load/start/stop/status), never drive or the control loop directly.
Mirrors navigate/control.py's PathRunner shape: no networking of its
own (load_path/start_path/stop_path/navigate_status/pump_on/
waterbutt_go/waterbutt_stop are injected, so this is testable without a
real navigate/waterbutt running — see test_control.py), driven by
repeated tick() calls from app.py's background loop. See
missions-prd.md's "Mission execution".

Four step types: `run_path` (unchanged - watches navigate's own status
until stopped_ok/aborted), and three timed steps added 2026-08-08 for
single-plant watering - `pause` (wait, nothing else), `water` (pump on,
stationary, for duration_s), `fill` (waterbutt's valve open for
duration_s). The timed steps don't have anything external to poll like
navigate's state, so completion is tracked with an injected `now`
clock instead (defaults to time.monotonic, overridable in tests so they
don't need to sleep in wall-clock time).
"""

import threading
import time

TIMED_STEP_TYPES = ("pause", "water", "fill")


class MissionRunner:
    def __init__(self, load_path, start_path, stop_path, navigate_status,
                 pump_on, waterbutt_go, waterbutt_stop, now=time.monotonic):
        self.load_path = load_path              # (path_name) -> {"ok": bool, "reason": ...}
        self.start_path = start_path            # () -> {"ok": bool, "reason": ...}
        self.stop_path = stop_path              # () -> None
        self.navigate_status = navigate_status  # () -> {"state": ..., "abort_reason": ...}
        self.pump_on = pump_on                  # (bool) -> {"ok": bool, "reason": ...}
        self.waterbutt_go = waterbutt_go        # (duration_s) -> {"ok": bool, "reason": ...}
        self.waterbutt_stop = waterbutt_stop    # () -> None
        self.now = now
        self.lock = threading.RLock()
        self._reset()

    def _reset(self):
        self.mission_name = None
        self.steps = []
        self.state = "idle"  # idle | running | stopped_ok | aborted
        self.current_step_index = None
        self.abort_reason = None
        self.step_log = []  # [{"index", "type", "path"/"duration_s", "outcome", "reason"}, ...] - this run only
        self.step_deadline = None  # self.now() value a pause/water/fill step completes at

    def go(self, mission_name: str, steps: list, start_index: int = 0):
        """Starts (or resumes, from start_index) a mission."""
        with self.lock:
            self.mission_name = mission_name
            self.steps = steps
            self.current_step_index = start_index
            self.state = "running"
            self.abort_reason = None
            self.step_log = []
            self.step_deadline = None
        self._start_current_step(start_index)

    def stop(self):
        """Operator-requested stop — same as PathRunner.stop(): leaves
        state "idle", not "aborted", and always stops navigate
        regardless of what it thinks it's doing. Also switches off
        whatever a timed step turned on - an operator stop must not
        leave the pump or the water butt's valve running unattended."""
        with self.lock:
            step_type = self._current_step_type()
            self.state = "idle"
            self.abort_reason = None
            self.step_deadline = None
        self.stop_path()
        if step_type == "water":
            self.pump_on(False)
        elif step_type == "fill":
            self.waterbutt_stop()

    def _current_step_type(self):
        """Call while holding self.lock."""
        if self.current_step_index is None or self.current_step_index >= len(self.steps):
            return None
        return self.steps[self.current_step_index].get("type", "run_path")

    def _step_summary(self, step_index: int) -> dict:
        step = self.steps[step_index]
        step_type = step.get("type", "run_path")
        if step_type == "run_path":
            return {"type": "run_path", "path": step["path"]}
        return {"type": step_type, "duration_s": step.get("duration_s")}

    def _start_current_step(self, step_index: int):
        step = self.steps[step_index]
        step_type = step.get("type", "run_path")

        if step_type == "run_path":
            self._start_run_path_step(step_index, step)
        elif step_type == "pause":
            with self.lock:
                self.step_deadline = self.now() + step["duration_s"]
        elif step_type == "water":
            result = self.pump_on(True)
            if not result.get("ok"):
                self._fail_step(step_index, "failed_to_start", result.get("reason", "couldn't start the pump"))
                return
            with self.lock:
                self.step_deadline = self.now() + step["duration_s"]
        elif step_type == "fill":
            result = self.waterbutt_go(step["duration_s"])
            if not result.get("ok"):
                self._fail_step(step_index, "failed_to_start", result.get("reason", "couldn't start filling"))
                return
            with self.lock:
                self.step_deadline = self.now() + step["duration_s"]

    def _start_run_path_step(self, step_index: int, step: dict):
        path_name = step["path"]

        load_result = self.load_path(path_name)
        if not load_result.get("ok"):
            self._fail_step(step_index, "failed_to_load", load_result.get("reason", "couldn't load path"))
            return

        result = self.start_path()
        if not result.get("ok"):
            self._fail_step(step_index, "failed_to_start", result.get("reason", "couldn't start path"))

    def _fail_step(self, step_index: int, outcome: str, reason: str):
        with self.lock:
            # A concurrent stop() may already have moved the mission on
            # while the step's own start call was in flight - don't
            # clobber that.
            if self.state != "running" or self.current_step_index != step_index:
                return
            self.state = "aborted"
            self.abort_reason = reason
            self.step_log.append({"index": step_index, **self._step_summary(step_index), "outcome": outcome, "reason": reason})

    def tick(self):
        """Call periodically (see app.py) while a mission might be
        running. No-op if idle/finished."""
        with self.lock:
            if self.state != "running":
                return
            step_index = self.current_step_index
            step_type = self._current_step_type()

        if step_type == "run_path":
            self._tick_run_path(step_index)
        else:
            self._tick_timed(step_index, step_type)

    def _tick_run_path(self, step_index: int):
        nav = self.navigate_status()
        nav_state = nav.get("state")
        if nav_state == "running":
            return  # still going - nothing to do yet

        next_step_index = None
        with self.lock:
            if self.state != "running" or self.current_step_index != step_index:
                return  # stop() (or another tick) already handled this
            if nav_state == "stopped_ok":
                self.step_log.append({"index": step_index, **self._step_summary(step_index), "outcome": "ok"})
                next_step_index = self._advance(step_index)
            else:  # aborted, or left "idle" unexpectedly (e.g. stopped from elsewhere)
                reason = nav.get("abort_reason") or f"navigate is unexpectedly {nav_state!r}"
                self.state = "aborted"
                self.abort_reason = reason
                self.step_log.append({"index": step_index, **self._step_summary(step_index), "outcome": "aborted", "reason": reason})

        if next_step_index is not None:
            self._start_current_step(next_step_index)

    def _tick_timed(self, step_index: int, step_type: str):
        with self.lock:
            if self.state != "running" or self.current_step_index != step_index:
                return
            deadline = self.step_deadline

        if deadline is None or self.now() < deadline:
            if step_type == "water":
                # Resend every tick, not just once at step start - drive's
                # own firmware watchdog turns the pump off after 2000ms of
                # silence (see navigate-prd.md's own pump-resend fix,
                # navigate/control.py's step()), and mission_status_hz's
                # tick period is comfortably under that.
                self.pump_on(True)
            return

        if step_type == "water":
            self.pump_on(False)
        # fill needs no explicit stop here - waterbutt's own valve
        # controller self-terminates at the same duration (see
        # ValveController.tick()); an explicit stop only matters for an
        # operator-requested stop() mid-fill, handled there.

        next_step_index = None
        with self.lock:
            if self.state != "running" or self.current_step_index != step_index:
                return
            self.step_deadline = None
            self.step_log.append({"index": step_index, **self._step_summary(step_index), "outcome": "ok"})
            next_step_index = self._advance(step_index)

        if next_step_index is not None:
            self._start_current_step(next_step_index)

    def _advance(self, step_index: int):
        """Call while holding self.lock. Moves current_step_index to the
        next step, or marks the mission stopped_ok if that was the last
        one. Returns the new step index to actually start, or None if
        the mission just finished (so the caller knows not to call
        _start_current_step)."""
        if step_index + 1 >= len(self.steps):
            self.state = "stopped_ok"
            self.current_step_index = None
            return None
        self.current_step_index = step_index + 1
        return self.current_step_index

    def status(self) -> dict:
        with self.lock:
            return {
                "mission_name": self.mission_name,
                "state": self.state,
                "current_step_index": self.current_step_index,
                "step_count": len(self.steps),
                "abort_reason": self.abort_reason,
                "step_log": list(self.step_log),
            }
