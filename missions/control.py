"""Mission sequencing state machine — drives jobs' existing HTTP API
(start/stop/status), never navigate or drive directly. Mirrors
jobs/control.py's JobRunner shape one level up: no networking of its
own (start_job/stop_job/job_status are injected, so this is testable
without a real jobs service running — see test_control.py), driven by
repeated tick() calls from app.py's background loop. See
missions-prd.md's "Mission execution".

One step type so far: `run_job` (runs a saved job to completion,
watching jobs' own status). A `pause` step type could be added later
the same way jobs added timed steps on top of navigate's run_path, but
isn't needed yet - each job can already carry its own trailing pause if
one's wanted between two jobs.

Unlike JobRunner watching navigate (whose status arrives via a
continuously-pushed feed with its own fixed timer - see navigate/
feed.py and jobs/control.py's RUN_PATH_FEED_GRACE_S comment), jobs has
no such push feed: job_status() is a plain synchronous poll of jobs'
own /control/status, which always reflects jobs' true current state at
the moment it's called, not a cached snapshot. So the specific
feed-staleness race JobRunner guards against doesn't apply here.

A different race *does* apply though, for the same reason it always
does when one state machine's tick() thread runs concurrently with
another call still driving that same step forward: go()/
_start_current_step() sets self.state = "running" *before* the
blocking start_job() HTTP call to jobs returns. If the background tick
loop's own thread calls tick() in that window, it's a no-op-looking
poll of *jobs'* current status - which, depending on scheduling, could
still reflect the *previous* job run's terminal state if jobs hasn't
processed the start call yet. Guarded the same way JobRunner guards
its own analogous window: stamp a start time before the blocking call,
and don't trust a "finished" status until either it's actually been
seen "running" at least once, or a short grace period has passed.
"""

import threading
import time

RUN_JOB_STATUS_GRACE_S = 1.0  # see _tick_run_job's comment, and control.py's own module docstring


class MissionRunner:
    def __init__(self, start_job, stop_job, job_status, now=time.monotonic):
        self.start_job = start_job    # (job_name) -> {"ok": bool, "reason": ...}
        self.stop_job = stop_job      # () -> None
        self.job_status = job_status  # () -> {"state": ..., "abort_reason": ...}
        self.now = now
        self.lock = threading.RLock()
        self._reset()

    def _reset(self):
        self.mission_name = None
        self.steps = []
        self.state = "idle"  # idle | running | stopped_ok | aborted
        self.current_step_index = None
        self.abort_reason = None
        self.step_log = []  # [{"index", "type", "job", "outcome", "reason"}, ...] - this run only
        self._run_job_seen_running = False  # see module docstring
        self._run_job_started_at = None

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
        """Operator-requested stop — same as JobRunner.stop(): leaves
        state "idle", not "aborted", and always stops jobs regardless
        of what it thinks it's doing."""
        with self.lock:
            self.state = "idle"
            self.abort_reason = None
        self.stop_job()

    def _step_summary(self, step_index: int) -> dict:
        step = self.steps[step_index]
        return {"type": "run_job", "job": step["job"]}

    def _start_current_step(self, step_index: int):
        step = self.steps[step_index]
        # Stamped up front, before the blocking start_job() call below -
        # not after it returns. See module docstring for why.
        with self.lock:
            self._run_job_seen_running = False
            self._run_job_started_at = self.now()

        result = self.start_job(step["job"])
        if not result.get("ok"):
            self._fail_step(step_index, "failed_to_start", result.get("reason", "couldn't start job"))

    def _fail_step(self, step_index: int, outcome: str, reason: str):
        with self.lock:
            # A concurrent stop() may already have moved the mission on
            # while this step's own start call was in flight - don't
            # clobber that (same reasoning as JobRunner._fail_step).
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
        self._tick_run_job(step_index)

    def _tick_run_job(self, step_index: int):
        status = self.job_status()
        job_state = status.get("state")

        next_step_index = None
        with self.lock:
            if self.state != "running" or self.current_step_index != step_index:
                return  # stop() (or another tick) already handled this

            if job_state == "running":
                self._run_job_seen_running = True
                return  # still going - nothing to do yet

            if not self._run_job_seen_running and self.now() - self._run_job_started_at < RUN_JOB_STATUS_GRACE_S:
                return  # too soon to trust this as the current job's own outcome - see module docstring

            if job_state == "stopped_ok":
                self.step_log.append({"index": step_index, **self._step_summary(step_index), "outcome": "ok"})
                next_step_index = self._advance(step_index)
            else:  # aborted, or left "idle" unexpectedly (e.g. stopped from elsewhere)
                reason = status.get("abort_reason") or f"jobs is unexpectedly {job_state!r}"
                self.state = "aborted"
                self.abort_reason = reason
                self.step_log.append({"index": step_index, **self._step_summary(step_index), "outcome": "aborted", "reason": reason})

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
