"""Job sequencing state machine — drives navigate's existing HTTP
API (load/start/stop/turn/status), never drive or the control loop
directly. Mirrors navigate/control.py's PathRunner shape: no
networking of its own (load_path/start_path/stop_path/start_turn/
navigate_status/pump_on/waterbutt_go/waterbutt_stop are injected, so
this is testable without a real navigate/waterbutt running — see
test_control.py), driven by repeated tick() calls from app.py's
background loop. See jobs-prd.md's "Job execution".

Five step types: `run_path` and `turn_to_heading` (added 2026-08-18 -
both watch navigate's own status until stopped_ok/aborted, since
navigate's own /control/stop stops whichever of the two it's actually
doing - see navigate-prd.md's "Turn in place"), and three timed steps
added 2026-08-08 for single-plant watering - `pause` (wait, nothing
else), `water` (pump on, stationary, for duration_s), `fill`
(waterbutt's valve open for duration_s). The timed steps don't have
anything external to poll like navigate's state, so completion is
tracked with an injected `now` clock instead (defaults to
time.monotonic, overridable in tests so they don't need to sleep in
wall-clock time).
"""

import threading
import time

TIMED_STEP_TYPES = ("pause", "water", "fill")
NAVIGATE_STEP_TYPES = ("run_path", "turn_to_heading")  # both polled via _tick_navigate_step

NAVIGATE_STEP_FEED_GRACE_S = 1.0  # see _tick_navigate_step's comment

# On/off phases run before every "water" step's own duration_s, to clear
# air from the pump (always emptied between uses) before real watering
# starts - found live 2026-09-01, without this the water lands almost
# randomly for the first couple of seconds. Fixed at 4 phases/8s total
# regardless of duration_s: negligible next to a 50-60s water (the only
# duration actually in use so far), acknowledged as a real ~4s of extra
# watering time on a short one (duration_s + the two "on" phases here) -
# see jobs-prd.md's "Water priming".
WATER_PRIME_PHASES = ((True, 2.0), (False, 2.0), (True, 2.0), (False, 2.0))


class JobRunner:
    def __init__(self, load_path, start_path, stop_path, start_turn, navigate_status,
                 pump_on, waterbutt_go, waterbutt_stop, now=time.monotonic):
        self.load_path = load_path              # (path_name) -> {"ok": bool, "reason": ...}
        self.start_path = start_path            # () -> {"ok": bool, "reason": ...}
        self.stop_path = stop_path              # () -> None - also stops a turn_to_heading step, see navigate's /control/stop
        self.start_turn = start_turn            # (heading_deg, tolerance_deg) -> {"ok": bool, "reason": ...}
        self.navigate_status = navigate_status  # () -> {"state": ..., "abort_reason": ...}
        self.pump_on = pump_on                  # (bool) -> {"ok": bool, "reason": ...}
        self.waterbutt_go = waterbutt_go        # (duration_s) -> {"ok": bool, "reason": ...}
        self.waterbutt_stop = waterbutt_stop    # () -> None
        self.now = now
        self.lock = threading.RLock()
        self._reset()

    def _reset(self):
        self.job_name = None
        self.steps = []
        self.state = "idle"  # idle | running | stopped_ok | aborted
        self.current_step_index = None
        self.abort_reason = None
        self.step_log = []  # [{"index", "type", "path"/"duration_s"/"heading_deg", "outcome", "reason"}, ...] - this run only
        self.step_deadline = None  # self.now() value a pause/water/fill step completes at
        self._navigate_step_seen_running = False  # see _tick_navigate_step's comment
        self._navigate_step_started_at = None
        self._water_phases = None  # the current "water" step's full (pump_on, duration_s) phase list - WATER_PRIME_PHASES then (True, duration_s) - see _start_current_step/_tick_timed
        self._water_phase_index = None

    def go(self, job_name: str, steps: list, start_index: int = 0):
        """Starts (or resumes, from start_index) a job."""
        with self.lock:
            self.job_name = job_name
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
            self._water_phases = None
            self._water_phase_index = None
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
        if step_type == "turn_to_heading":
            return {"type": "turn_to_heading", "heading_deg": step["heading_deg"], "tolerance_deg": step.get("tolerance_deg")}
        return {"type": step_type, "duration_s": step.get("duration_s")}

    def _start_current_step(self, step_index: int):
        step = self.steps[step_index]
        step_type = step.get("type", "run_path")

        if step_type == "run_path":
            self._start_run_path_step(step_index, step)
        elif step_type == "turn_to_heading":
            self._start_turn_step(step_index, step)
        elif step_type == "pause":
            with self.lock:
                self.step_deadline = self.now() + step["duration_s"]
        elif step_type == "water":
            phases = (*WATER_PRIME_PHASES, (True, step["duration_s"]))
            result = self.pump_on(phases[0][0])
            if not result.get("ok"):
                self._fail_step(step_index, "failed_to_start", result.get("reason", "couldn't start the pump"))
                return
            with self.lock:
                self._water_phases = phases
                self._water_phase_index = 0
                self.step_deadline = self.now() + phases[0][1]
        elif step_type == "fill":
            result = self.waterbutt_go(step["duration_s"])
            if not result.get("ok"):
                if result.get("refused"):
                    self._skip_refused_fill(step_index, result.get("reason"))
                else:
                    self._fail_step(step_index, "failed_to_start", result.get("reason", "couldn't start filling"))
                return
            with self.lock:
                self.step_deadline = self.now() + step["duration_s"]

    def _start_run_path_step(self, step_index: int, step: dict):
        # Stamped up front, before the (blocking, real network I/O)
        # load_path/start_path calls below - not after they return.
        # self.state is already "running" by the time we're called (set
        # by go()/_tick_navigate_step before dispatching here), so the
        # background tick loop can call _tick_navigate_step concurrently
        # while load_path/start_path are still in flight. Stamping late
        # left a real window where _navigate_step_started_at was still
        # None while state was already "running", crashing tick()
        # outright (seen live 2026-08-09, right after this race fix was
        # added - killed the tick thread entirely, so the job never
        # progressed again until the service was restarted).
        with self.lock:
            self._navigate_step_seen_running = False
            self._navigate_step_started_at = self.now()

        path_name = step["path"]

        load_result = self.load_path(path_name)
        if not load_result.get("ok"):
            self._fail_step(step_index, "failed_to_load", load_result.get("reason", "couldn't load path"))
            return

        result = self.start_path()
        if not result.get("ok"):
            self._fail_step(step_index, "failed_to_start", result.get("reason", "couldn't start path"))
            return

    def _start_turn_step(self, step_index: int, step: dict):
        # Same up-front stamping as _start_run_path_step, same reason -
        # see its comment.
        with self.lock:
            self._navigate_step_seen_running = False
            self._navigate_step_started_at = self.now()

        result = self.start_turn(step["heading_deg"], step.get("tolerance_deg"))
        if not result.get("ok"):
            self._fail_step(step_index, "failed_to_start", result.get("reason", "couldn't start turning"))
            return

    def _fail_step(self, step_index: int, outcome: str, reason: str):
        with self.lock:
            # A concurrent stop() may already have moved the job on
            # while the step's own start call was in flight - don't
            # clobber that.
            if self.state != "running" or self.current_step_index != step_index:
                return
            self.state = "aborted"
            self.abort_reason = reason
            self.step_log.append({"index": step_index, **self._step_summary(step_index), "outcome": outcome, "reason": reason})

    def _skip_refused_fill(self, step_index: int, reason: str):
        """A fill refused by waterbutt itself - the QC marker wasn't
        visible (2026-08-15), or (2026-08-18) the tank-level estimate
        isn't confident the butt is actually empty - keep the job (and,
        via missions, the whole round) going without water rather than
        aborting over it. Distinct from _fail_step: still logged (so a
        dry bed shows up in the step log/history), just doesn't stop
        anything."""
        next_step_index = None
        with self.lock:
            if self.state != "running" or self.current_step_index != step_index:
                return  # a concurrent stop() already handled this
            self.step_log.append({
                "index": step_index, **self._step_summary(step_index),
                "outcome": "skipped_no_water", "reason": reason,
            })
            next_step_index = self._advance(step_index)
        if next_step_index is not None:
            self._start_current_step(next_step_index)

    def tick(self):
        """Call periodically (see app.py) while a job might be
        running. No-op if idle/finished."""
        with self.lock:
            if self.state != "running":
                return
            step_index = self.current_step_index
            step_type = self._current_step_type()

        if step_type in NAVIGATE_STEP_TYPES:
            self._tick_navigate_step(step_index)
        else:
            self._tick_timed(step_index, step_type)

    def _tick_navigate_step(self, step_index: int):
        """Polls navigate's own status until it's left "running" -
        shared by run_path and turn_to_heading (added 2026-08-18): both
        just ask navigate to do something and wait, and navigate's own
        /control/stop already stops whichever of the two is actually in
        progress, so the polling/race-guard logic below doesn't need to
        know which one this run actually is."""
        nav = self.navigate_status()
        nav_state = nav.get("state")

        next_step_index = None
        with self.lock:
            if self.state != "running" or self.current_step_index != step_index:
                return  # stop() (or another tick) already handled this

            if nav_state == "running":
                self._navigate_step_seen_running = True
                return  # still going - nothing to do yet

            if not self._navigate_step_seen_running and self.now() - self._navigate_step_started_at < NAVIGATE_STEP_FEED_GRACE_S:
                # navigate's feed pushes on its own fixed timer (see
                # navigate/feed.py), independent of when its internal
                # state actually changes - right after start_path()
                # returns, the feed can still be reporting the
                # *previous* run's terminal state for up to one push
                # period. Without this grace window, a step could be
                # marked complete before navigate had even started
                # driving it (seen live 2026-08-09: the job moved on to
                # the next step's load, which navigate correctly
                # refused since the first path was still actually
                # running).
                return

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
                # Resend every tick, not just once at the current phase's
                # start - drive's own firmware watchdog turns the pump
                # off after 2000ms of silence (see navigate-prd.md's own
                # pump-resend fix, navigate/control.py's step()), and
                # job_status_hz's tick period is comfortably under that.
                # Whichever phase we're actually in right now (priming
                # on/off, or the real watering duration) - see
                # WATER_PRIME_PHASES/_start_current_step.
                with self.lock:
                    phase_on = self._water_phases[self._water_phase_index][0]
                self.pump_on(phase_on)
            return

        if step_type == "water":
            next_phase_index = None
            with self.lock:
                if self.state == "running" and self.current_step_index == step_index:
                    next_phase_index = self._water_phase_index + 1
            if next_phase_index is not None and next_phase_index < len(self._water_phases):
                # Priming (or the real watering duration - same
                # mechanism either way) isn't finished - move to the
                # next phase rather than ending the step.
                pump_state, phase_duration = self._water_phases[next_phase_index]
                self.pump_on(pump_state)
                with self.lock:
                    if self.state != "running" or self.current_step_index != step_index:
                        return
                    self._water_phase_index = next_phase_index
                    self.step_deadline = self.now() + phase_duration
                return
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
            self._water_phases = None
            self._water_phase_index = None
            self.step_log.append({"index": step_index, **self._step_summary(step_index), "outcome": "ok"})
            next_step_index = self._advance(step_index)

        if next_step_index is not None:
            self._start_current_step(next_step_index)

    def _advance(self, step_index: int):
        """Call while holding self.lock. Moves current_step_index to the
        next step, or marks the job stopped_ok if that was the last
        one. Returns the new step index to actually start, or None if
        the job just finished (so the caller knows not to call
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
                "job_name": self.job_name,
                "state": self.state,
                "current_step_index": self.current_step_index,
                "step_count": len(self.steps),
                "abort_reason": self.abort_reason,
                "step_log": list(self.step_log),
            }
