"""Turn-in-place control: rotate to a target heading, no forward
motion. Added 2026-08-18 for job steps that need to face a heading a
saved path can't reach any other way (e.g. a plant only approachable
from one direction). Heading-only, deliberately — the motors are
"sticky" enough that one wheel often does most of the turning, so
holding a fixed centre of rotation would need a lever arm from the
navigation frame to the robot's true body-centre plus a genuine two-DOF
position+heading controller. Started with the simpler version instead:
accept wherever the robot ends up, see if that's actually a problem in
practice before building the harder one (see navigate-prd.md's "Turn
in place").

TurnRunner does no networking of its own — driven by step()/start()/
stop() calls, with send_velocity injected, same shape as PathRunner
(see control.py) so it's testable without drive or oxts-nav running.
"""

import logging
import math
import threading
import time

import geometry
from stall import StallGuard


class TurnRunner:
    def __init__(self, config: dict, send_velocity, now=time.monotonic):
        self.config = config
        self.send_velocity = send_velocity
        self.now = now
        self.lock = threading.RLock()
        self._reset()

    def _reset(self):
        self.target_heading_deg = None
        self.tolerance_deg = None
        self.state = "idle"  # idle | running | stopped_ok | aborted
        self.abort_reason = None
        self.last_status = {}
        self._stall_guard = StallGuard(self.config["stall_check_window_s"], self.config["turn_stall_min_deg"])

    def start(self, target_heading_deg: float, tolerance_deg: float, robot_heading_deg: float) -> dict:
        with self.lock:
            if self.state == "running":
                return {"ok": False, "reason": "already turning"}
            self._reset()
            self.target_heading_deg = target_heading_deg
            self.tolerance_deg = tolerance_deg
            self.state = "running"
            self._stall_guard.reset(robot_heading_deg, self.now())
            return {"ok": True}

    def stop(self):
        """Operator-requested stop — same immediate zero-command as an
        abort, but leaves state as "idle" rather than "aborted"."""
        with self.lock:
            self.state = "idle"
            self.abort_reason = None
        self.send_velocity(0.0, 0.0)

    def abort_if_running(self, reason):
        """For a caller outside the normal step() loop (app.py, when
        oxts-nav's feed has gone stale and there's no heading to step()
        with at all) - a no-op unless actually running, so it's safe to
        call every tick regardless of state."""
        with self.lock:
            if self.state != "running":
                return
            self._abort(reason)

    def step(self, robot_heading_deg: float):
        """Call at control_hz while running. No-op if not running."""
        with self.lock:
            if self.state != "running":
                return

            heading_err = geometry.angle_diff(self.target_heading_deg, robot_heading_deg)
            if abs(heading_err) <= self.tolerance_deg:
                self._finish()
                return

            stalled = self._stall_guard.stalled(
                robot_heading_deg, self.now(),
                lambda a, b: abs(geometry.angle_diff(b, a)),
            )
            if stalled is not None:
                window_s = self.config["stall_check_window_s"]
                min_deg = self.config["turn_stall_min_deg"]
                self._abort(f"turned only {stalled:.1f}deg in {window_s:.1f}s (limit {min_deg:.1f}deg) - stuck?")
                return

            # Shortest-way proportional heading control, no forward
            # speed - differential_drive(0.0, turn, ...) is then exactly
            # equal-and-opposite wheel speeds, a pure spin.
            turn_rate = self.config["turn_gain"] * math.radians(heading_err)
            max_rate = self.config["turn_max_rate_rad_s"]
            turn_rate = max(-max_rate, min(max_rate, turn_rate))
            left, right = geometry.differential_drive(
                0.0, turn_rate, self.config["wheel_base_m"], self.config["turn_max_mps"]
            )
            self.send_velocity(left, right)

            self.last_status = {
                "target_heading_deg": self.target_heading_deg,
                "heading_error_deg": heading_err,
                "left_mps": left,
                "right_mps": right,
            }

    def _abort(self, reason):
        self.state = "aborted"
        self.abort_reason = reason
        self.send_velocity(0.0, 0.0)
        logging.warning("[navigate] Turn aborted: %s", reason)

    def _finish(self):
        self.state = "stopped_ok"
        self.send_velocity(0.0, 0.0)

    def status(self) -> dict:
        with self.lock:
            return {
                "state": self.state,
                "abort_reason": self.abort_reason,
                **self.last_status,
            }
