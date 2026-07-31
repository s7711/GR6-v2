"""Mission sequencing state machine — drives navigate's existing HTTP
API (load/start/stop/status), never drive or the control loop directly.
Mirrors navigate/control.py's PathRunner shape: no networking of its
own (load_path/start_path/stop_path/navigate_status are injected, so
this is testable without a real navigate running — see
test_control.py), driven by repeated tick() calls from app.py's
background loop. See missions-prd.md's "Mission execution".
"""

import threading


class MissionRunner:
    def __init__(self, load_path, start_path, stop_path, navigate_status):
        self.load_path = load_path              # (path_name) -> None
        self.start_path = start_path            # () -> {"ok": bool, "reason": ...}
        self.stop_path = stop_path              # () -> None
        self.navigate_status = navigate_status  # () -> {"state": ..., "abort_reason": ...}
        self.lock = threading.RLock()
        self._reset()

    def _reset(self):
        self.mission_name = None
        self.steps = []
        self.state = "idle"  # idle | running | stopped_ok | aborted
        self.current_step_index = None
        self.abort_reason = None
        self.step_log = []  # [{"index", "path", "outcome", "reason"}, ...] - this run only

    def go(self, mission_name: str, steps: list, start_index: int = 0):
        """Starts (or resumes, from start_index) a mission."""
        with self.lock:
            self.mission_name = mission_name
            self.steps = steps
            self.current_step_index = start_index
            self.state = "running"
            self.abort_reason = None
            self.step_log = []
        self._start_current_step(start_index)

    def stop(self):
        """Operator-requested stop — same as PathRunner.stop(): leaves
        state "idle", not "aborted", and always stops navigate
        regardless of what it thinks it's doing."""
        with self.lock:
            self.state = "idle"
            self.abort_reason = None
        self.stop_path()

    def _start_current_step(self, step_index: int):
        path_name = self.steps[step_index]["path"]
        self.load_path(path_name)
        result = self.start_path()
        if result.get("ok"):
            return
        with self.lock:
            # A concurrent stop() may already have moved the mission on
            # while start_path() was in flight - don't clobber that.
            if self.state != "running" or self.current_step_index != step_index:
                return
            reason = result.get("reason", "couldn't start path")
            self.state = "aborted"
            self.abort_reason = reason
            self.step_log.append({"index": step_index, "path": path_name, "outcome": "failed_to_start", "reason": reason})

    def tick(self):
        """Call periodically (see app.py) while a mission might be
        running - checks navigate's real status and advances/finishes/
        aborts accordingly. No-op if idle/finished."""
        with self.lock:
            if self.state != "running":
                return
            step_index = self.current_step_index

        nav = self.navigate_status()
        nav_state = nav.get("state")
        if nav_state == "running":
            return  # still going - nothing to do yet

        next_step_index = None
        with self.lock:
            if self.state != "running" or self.current_step_index != step_index:
                return  # stop() (or another tick) already handled this
            path_name = self.steps[step_index]["path"]
            if nav_state == "stopped_ok":
                self.step_log.append({"index": step_index, "path": path_name, "outcome": "ok"})
                if step_index + 1 >= len(self.steps):
                    self.state = "stopped_ok"
                    self.current_step_index = None
                else:
                    next_step_index = step_index + 1
                    self.current_step_index = next_step_index
            else:  # aborted, or left "idle" unexpectedly (e.g. stopped from elsewhere)
                reason = nav.get("abort_reason") or f"navigate is unexpectedly {nav_state!r}"
                self.state = "aborted"
                self.abort_reason = reason
                self.step_log.append({"index": step_index, "path": path_name, "outcome": "aborted", "reason": reason})

        if next_step_index is not None:
            self._start_current_step(next_step_index)

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
