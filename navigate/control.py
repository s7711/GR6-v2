"""Path-following control loop orchestration: loads a path, runs the
entry-scan, and (via repeated step() calls at control_hz) tracks it with
the pure-pursuit math in geometry.py, aborting cleanly per
navigate-prd.md's "Variable tolerance" and "Ownership of tolerance/
accuracy enforcement" — this is the sole enforcer of clearance/accuracy
limits for now.

PathRunner itself does no networking/threading — it's driven by
step()/start()/stop() calls, with `send_velocity`/`send_pump` injected
so it can be tested without drive or oxts-nav actually running (see
test_control.py). The real control-loop thread (pulling live position
from oxts-nav's feed at control_hz and calling step()) lives in app.py.
"""

import logging
import math
import threading
import time

import geometry
from stall import StallGuard


class PathRunner:
    def __init__(self, config: dict, send_velocity, send_pump, now=time.monotonic):
        self.config = config
        self.send_velocity = send_velocity
        self.send_pump = send_pump
        self.now = now
        self.lock = threading.RLock()
        self._reset()

    def _reset(self):
        self.path = []
        self.path_ref = None
        self.tracked_index = 0
        self.state = "idle"  # idle | running | stopped_ok | aborted
        self.abort_reason = None
        self.distance_travelled_m = 0.0
        self._last_robot_local = None
        self.last_status = {}
        self._stall_guard = StallGuard(self.config["stall_check_window_s"], self.config["stall_min_distance_m"])

    def load_path(self, points: list) -> dict:
        """points: stored path dicts (lat/lon/speed_mps/pump/clearance_m),
        as loaded from paths.py. Refuses ({"ok": False, "reason": ...})
        while a run is already in progress, rather than silently
        abandoning it — found live 2026-07-31: start() already refused
        to run while "running", but load_path() reset straight to idle
        unconditionally, so a second caller (jobs, another browser
        tab) loading a different path mid-run wiped out that state
        before start()'s own check ever saw it, abandoning the run with
        no explicit stop (the robot kept coasting at its last commanded
        speed until drive's own firmware watchdog zeroed it out).
        "Loading a path is a deliberate operator action" still holds —
        it just isn't one that gets to preempt a run already under way;
        stop first, then load."""
        with self.lock:
            if self.state == "running":
                return {"ok": False, "reason": "another path is already running - stop it first"}
            self._reset()
            if points:
                ref_lat, ref_lon = geometry.path_reference(points)
                self.path_ref = (ref_lat, ref_lon)
                self.path = geometry.path_to_local(points, ref_lat, ref_lon)
            return {"ok": True}

    def entry_check(self, robot_lat, robot_lon, robot_heading_deg) -> dict:
        """{"ok": True, "index": i} if a valid entry segment exists, else
        {"ok": False, "reason": ..., "nearest_index"/"distance_m"/
        "heading_error_deg": ...} describing the closest candidate, so
        the operator can be shown distance/angle to drive towards —
        never silently proceeds (see navigate-prd.md's fix over GR6-v1's
        silent-failure behaviour)."""
        with self.lock:
            if not self.path:
                return {"ok": False, "reason": "no path loaded"}
            robot_north, robot_east = geometry.to_local(robot_lat, robot_lon, *self.path_ref)
            index = geometry.find_entry_segment(
                self.path, robot_north, robot_east, robot_heading_deg,
                self.config["entry_max_distance_m"], self.config["entry_max_heading_deg"],
                self.config["lookahead_distance_m"],
            )
            if index is not None:
                return {"ok": True, "index": index}

            best = None
            for i in range(len(self.path) - 1):
                a, b = self.path[i], self.path[i + 1]
                _px, _py, _t, dist = geometry.project_onto_segment(
                    robot_north, robot_east, a.north, a.east, b.north, b.east
                )
                if best is None or dist < best[1]:
                    # Heading error against the *lookahead point* (what
                    # find_entry_segment actually checks), not the raw
                    # projected point on the segment - close to the path,
                    # bearing to a point almost beside you is dominated by
                    # the cross-track direction and swings towards +/-90deg
                    # regardless of true heading alignment (see
                    # find_entry_segment's docstring). Using the same
                    # lookahead-based measure here keeps this diagnostic
                    # message consistent with what actually gated entry.
                    lookahead = geometry.find_lookahead_point(
                        self.path, i, robot_north, robot_east, self.config["lookahead_distance_m"]
                    )
                    if lookahead is None:
                        continue
                    heading_err = geometry.heading_error_deg(
                        robot_heading_deg, robot_north, robot_east, lookahead.north, lookahead.east
                    )
                    best = (i, dist, heading_err)
            return {
                "ok": False,
                "reason": "no segment within entry tolerance",
                "nearest_index": best[0],
                "distance_m": best[1],
                "heading_error_deg": best[2],
            }

    def start(self, robot_lat, robot_lon, robot_heading_deg) -> dict:
        with self.lock:
            if self.state == "running":
                return {"ok": False, "reason": "already running"}
            check = self.entry_check(robot_lat, robot_lon, robot_heading_deg)
            if not check["ok"]:
                return check
            self.tracked_index = check["index"]
            self.state = "running"
            self.abort_reason = None
            self.distance_travelled_m = 0.0
            self._last_robot_local = None
            robot_north, robot_east = geometry.to_local(robot_lat, robot_lon, *self.path_ref)
            self._stall_guard.reset((robot_north, robot_east), self.now())
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
        oxts-nav's feed has gone stale and there's no position to step()
        with at all) - a no-op unless actually running, so it's safe to
        call every tick regardless of state."""
        with self.lock:
            if self.state != "running":
                return
            self._abort(reason)

    def preview(self, robot_lat, robot_lon, robot_heading_deg):
        """Live cross-track/heading error against the loaded path, for
        the Run page's display to stay meaningful even with no run in
        progress (before starting, or once one's finished/aborted) —
        call this whenever step() is a no-op. No-op itself if running
        (step()'s own values are authoritative then) or no path loaded.

        Deliberately mirrors start()+step()'s own entry-segment lookup
        (find_entry_segment, then find_lookahead_point from that index)
        rather than an independent "nearest segment overall" search —
        found live 2026-07-30 that those can disagree on a path that
        loops back near itself (e.g. Waterstablebed): the preview showed
        a small, misleadingly-low error from whichever segment was
        geometrically closest, while the real run — which only ever
        tracks forward from the actual entry segment — aborted almost
        immediately against a much larger one. This now shows exactly
        what step()'s first tick would compute, not an approximation of
        it. Only falls back to the nearest-segment-overall search (same
        one entry_check's own rejection message already uses) when no
        valid entry segment exists at all, so there's still something
        meaningful to show."""
        with self.lock:
            if self.state == "running" or len(self.path) < 2:
                return
            robot_north, robot_east = geometry.to_local(robot_lat, robot_lon, *self.path_ref)
            index = geometry.find_entry_segment(
                self.path, robot_north, robot_east, robot_heading_deg,
                self.config["entry_max_distance_m"], self.config["entry_max_heading_deg"],
                self.config["lookahead_distance_m"],
            )
            if index is None:
                offset = geometry.nearest_segment_offset(self.path, robot_north, robot_east, robot_heading_deg)
                self.last_status["cross_track_error_m"] = offset["cross_track_error_m"]
                self.last_status["heading_error_deg"] = offset["heading_error_deg"]
                return
            result = geometry.find_lookahead_point(
                self.path, index, robot_north, robot_east, self.config["lookahead_distance_m"]
            )
            if result is None:
                return
            self.last_status["cross_track_error_m"] = result.cross_track_error_m
            self.last_status["heading_error_deg"] = geometry.heading_error_deg(
                robot_heading_deg, robot_north, robot_east, result.north, result.east
            )

    def step(self, robot_lat, robot_lon, robot_heading_deg, horizontal_accuracy_m):
        """Call at control_hz while running. No-op if not running."""
        with self.lock:
            if self.state != "running":
                return

            robot_north, robot_east = geometry.to_local(robot_lat, robot_lon, *self.path_ref)
            if self._last_robot_local is not None:
                self.distance_travelled_m += math.hypot(
                    robot_north - self._last_robot_local[0], robot_east - self._last_robot_local[1]
                )
            self._last_robot_local = (robot_north, robot_east)

            limit = self.config["localisation_accuracy_limit_m"]
            if horizontal_accuracy_m is not None and horizontal_accuracy_m > limit:
                self._abort(f"localisation accuracy {horizontal_accuracy_m:.2f}m exceeds limit {limit:.2f}m")
                return

            stalled = self._stall_guard.stalled(
                (robot_north, robot_east), self.now(),
                lambda a, b: math.hypot(b[0] - a[0], b[1] - a[1]),
            )
            if stalled is not None:
                window_s = self.config["stall_check_window_s"]
                min_m = self.config["stall_min_distance_m"]
                self._abort(f"moved only {stalled:.2f}m in {window_s:.1f}s (limit {min_m:.2f}m) - stuck?")
                return

            result = geometry.find_lookahead_point(
                self.path, self.tracked_index, robot_north, robot_east,
                self.config["lookahead_distance_m"],
            )
            if result is None or result.path_complete:
                self._finish()
                return
            self.tracked_index = result.tracked_index

            clearance = self.path[self.tracked_index].clearance_m
            if abs(result.cross_track_error_m) > clearance:
                self._abort(
                    f"cross-track error {result.cross_track_error_m:.2f}m exceeds "
                    f"this segment's clearance {clearance:.2f}m"
                )
                return

            heading_err = geometry.heading_error_deg(
                robot_heading_deg, robot_north, robot_east, result.north, result.east
            )
            max_heading = self.config["max_heading_correction_deg"]
            if abs(heading_err) > max_heading:
                self._abort(f"heading error {heading_err:.1f}deg exceeds limit {max_heading:.1f}deg")
                return

            turn = geometry.turn_command(
                heading_err, result.cross_track_error_m, result.speed_mps,
                self.config["heading_gain"], self.config["cte_gain"],
                self.config["lookahead_distance_m"],
            )
            left, right = geometry.differential_drive(
                result.speed_mps, turn, self.config["wheel_base_m"], self.config["max_speed_mps"]
            )
            self.send_velocity(left, right)
            # Every step, not just on change — the firmware's WP watchdog
            # turns the pump off if no WP command arrives within 2000ms
            # (see drive-prd.md), same reasoning as send_velocity above.
            # Found live 2026-07-30: navigation completed fine but the
            # pump had silently switched itself off partway through,
            # because this used to only resend on a state change.
            self.send_pump(result.pump)

            self.last_status = {
                "tracked_index": self.tracked_index,
                "cross_track_error_m": result.cross_track_error_m,
                "clearance_m": clearance,
                "clearance_headroom_m": clearance - abs(result.cross_track_error_m),
                "heading_error_deg": heading_err,
                "distance_travelled_m": self.distance_travelled_m,
                "target_speed_mps": result.speed_mps,
                "left_mps": left,
                "right_mps": right,
            }

    def _abort(self, reason):
        self.state = "aborted"
        self.abort_reason = reason
        self.send_velocity(0.0, 0.0)
        # One place, covers both callers (the real run's shared runner
        # and /record/forward's throwaway mini-path runner) — quicker to
        # check via `journalctl -u robot-navigate` than the debug log,
        # per navigate-prd.md.
        logging.warning("[navigate] Aborted: %s", reason)

    def _finish(self):
        self.state = "stopped_ok"
        self.send_velocity(0.0, 0.0)

    def status(self) -> dict:
        with self.lock:
            return {
                "state": self.state,
                "abort_reason": self.abort_reason,
                "path_points": len(self.path),
                **self.last_status,
            }
