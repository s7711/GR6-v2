"""End-to-end tests against the Flask app — routes, page rendering,
recording flow, control wiring. Never makes a real HTTP call to drive or
a real socket connection to oxts-nav: `app.nav_client` is replaced with a
fake exposing `.latest()`, and `app.runner` with a fresh PathRunner built
over recording send_velocity/send_pump stubs (see test_control.py for
PathRunner's own behaviour). `app.PATHS_DIR` points at a temp directory
so tests never touch this robot's real recorded paths.
"""

import json
import math
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import requests

import app
import paths as paths_module
from control import PathRunner

SAMPLE_POINTS = [
    {"lat": 52.2, "lon": -1.5, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
    {"lat": 52.2005, "lon": -1.5, "speed_mps": 0.5, "pump": True, "clearance_m": 0.5},
]


class FakeNavClient:
    def __init__(self):
        self.payload = {"nav": {}, "status": {}, "connection": {}}
        self._sequence = None
        self._sequence_index = 0

    def latest(self):
        if self._sequence is not None:
            payload = self._sequence[min(self._sequence_index, len(self._sequence) - 1)]
            self._sequence_index += 1
            return payload
        return self.payload

    def queue_sequence(self, payloads):
        """Return each payload in order on successive .latest() calls
        (repeating the last one once exhausted) — for simulating the
        robot's position/heading changing partway through a maneuver,
        which a single static `self.payload` can't do."""
        self._sequence = payloads
        self._sequence_index = 0


class Recorder:
    def __init__(self):
        self.velocity_calls = []
        self.pump_calls = []

    def send_velocity(self, left, right):
        self.velocity_calls.append((left, right))

    def send_pump(self, on):
        self.pump_calls.append(on)


class RecordingPost:
    """Stand-in for requests.post that just records every call (url,
    kwargs) and returns a fake 204 response - for asserting on calls to
    oxts-nav's /gnss/... routes without a real oxts-nav to hit."""

    def __init__(self):
        self.calls = []

    def __call__(self, url, **kwargs):
        self.calls.append((url, kwargs))
        resp = requests.Response()
        resp.status_code = 204
        return resp


class NavigateAppTestCase(unittest.TestCase):
    def setUp(self):
        app.PATHS_DIR = Path(tempfile.mkdtemp())
        # LOGS_DIR is computed once from PATHS_DIR at import time, not
        # re-derived when PATHS_DIR is reassigned above — redirect it too,
        # so tests never touch this robot's real debug logs.
        app.LOGS_DIR = app.PATHS_DIR / "logs"
        app.WATERBUTT_LOGS_DIR = app.PATHS_DIR / "waterbutt-logs"
        app._debug_log_path = None
        self.recorder = Recorder()
        app.runner = PathRunner(app.CONTROL_CONFIG, self.recorder.send_velocity, self.recorder.send_pump)
        app._current_path_name = None
        app._aruco_priority_active_for_run = False
        app._control_loop_last_state = "idle"
        app.nav_client = FakeNavClient()
        self.client = app.app.test_client()

    def _set_position(self, lat_deg, lon_deg, heading_deg, north_acc=0.05, east_acc=0.05):
        app.nav_client.payload = {
            "nav": {"Lat": math.radians(lat_deg), "Lon": math.radians(lon_deg), "Heading": heading_deg},
            "status": {"NorthAcc": north_acc, "EastAcc": east_acc},
            "connection": {},
        }

    def test_list_paths_empty(self):
        self.assertEqual(self.client.get("/api/paths").get_json(), [])

    def test_get_and_delete_path(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)

        names = [p["name"] for p in self.client.get("/api/paths").get_json()]
        self.assertEqual(names, ["loop"])

        self.assertEqual(self.client.get("/api/paths/loop").get_json(), SAMPLE_POINTS)

        resp = self.client.delete("/api/paths/loop")
        self.assertEqual(resp.status_code, 204)
        self.assertEqual(self.client.get("/api/paths").get_json(), [])

    def test_get_missing_path_404(self):
        self.assertEqual(self.client.get("/api/paths/does-not-exist").status_code, 404)

    def test_save_path_creates_new_file(self):
        resp = self.client.post("/api/paths/new-loop", json={"points": SAMPLE_POINTS})
        self.assertEqual(resp.status_code, 204)
        self.assertEqual(self.client.get("/api/paths/new-loop").get_json(), SAMPLE_POINTS)

    def test_save_path_overwrites_existing_file(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        edited = [dict(p) for p in SAMPLE_POINTS]
        edited[0]["speed_mps"] = 0.2
        resp = self.client.post("/api/paths/loop", json={"points": edited})
        self.assertEqual(resp.status_code, 204)
        self.assertEqual(self.client.get("/api/paths/loop").get_json()[0]["speed_mps"], 0.2)

    def test_save_path_rejects_fewer_than_two_points(self):
        resp = self.client.post("/api/paths/too-short", json={"points": SAMPLE_POINTS[:1]})
        self.assertEqual(resp.status_code, 400)

    def test_project_endpoint_matches_geometry_module(self):
        import geometry
        resp = self.client.post(
            "/api/project", json={"lat": 52.2, "lon": -1.5, "bearing_deg": 0, "distance_m": 0.1}
        )
        expected_lat, expected_lon = geometry.project_forward(52.2, -1.5, 0, 0.1)
        result = resp.get_json()
        self.assertAlmostEqual(result["lat"], expected_lat)
        self.assertAlmostEqual(result["lon"], expected_lon)

    def test_drop_point_without_position_fix_is_conflict(self):
        resp = self.client.post("/record/drop", json={"speed_mps": 0.5, "pump": False, "clearance_m": 0.5})
        self.assertEqual(resp.status_code, 409)

    def test_record_new_drop_current_save(self):
        self._set_position(52.2, -1.5, 0)
        self.client.post("/record/new")
        resp = self.client.post("/record/drop", json={"speed_mps": 0.5, "pump": False, "clearance_m": 0.5})
        self.assertEqual(resp.get_json()["point_count"], 1)

        self._set_position(52.2005, -1.5, 0)
        self.client.post("/record/drop", json={"speed_mps": 0.6, "pump": True, "clearance_m": 1.0})

        self.assertEqual(len(self.client.get("/record/current").get_json()), 2)

        resp = self.client.post("/record/save", json={"name": "test-path"})
        self.assertEqual(resp.status_code, 204)
        self.assertEqual([p["name"] for p in self.client.get("/api/paths").get_json()], ["test-path"])

    def test_record_new_clears_in_progress_points(self):
        self._set_position(52.2, -1.5, 0)
        self.client.post("/record/drop", json={"speed_mps": 0.5, "pump": False, "clearance_m": 0.5})
        self.client.post("/record/new")
        self.assertEqual(self.client.get("/record/current").get_json(), [])

    def test_record_save_rejects_fewer_than_two_points(self):
        self._set_position(52.2, -1.5, 0)
        self.client.post("/record/new")
        self.client.post("/record/drop", json={"speed_mps": 0.5, "pump": False, "clearance_m": 0.5})
        resp = self.client.post("/record/save", json={"name": "too-short"})
        self.assertEqual(resp.status_code, 400)

    def test_record_forward_without_position_fix(self):
        resp = self.client.post("/record/forward", json={"distance_m": 1.0, "speed_mps": 0.5})
        self.assertEqual(resp.get_json(), {"ok": False, "reason": "no position fix yet"})

    def test_record_forward_drives_then_times_out_and_stops(self):
        # The fake nav client's position never actually moves in this
        # test (no real robot to simulate), so the maneuver can never
        # reach path_complete on its own — force a short timeout so the
        # test exercises the "still running, force-stop" path quickly
        # rather than waiting the real MOVE_FORWARD_TIMEOUT_S out. Also
        # patch send_velocity/send_pump directly: /record/forward's
        # ephemeral nudge_runner uses the real module-level functions
        # (correct in production — it really does need to reach drive),
        # not self.recorder, so those must be patched here rather than
        # relying on app.runner's wiring from setUp.
        self._set_position(52.2, -1.5, 0)
        with patch.object(app, "MOVE_FORWARD_TIMEOUT_S", 0.05), \
             patch.object(app, "send_velocity", self.recorder.send_velocity), \
             patch.object(app, "send_pump", self.recorder.send_pump):
            resp = self.client.post("/record/forward", json={"distance_m": 1.0, "speed_mps": 0.5})
        result = resp.get_json()
        self.assertEqual(result["state"], "idle")  # forced stop, not "aborted"
        self.assertEqual(self.recorder.velocity_calls[-1], (0.0, 0.0))
        # At least one real forward command was sent before the timeout —
        # confirms the loop actually stepped the controller, not just
        # immediately timing out without ever driving.
        self.assertTrue(any(left > 0 for left, _right in self.recorder.velocity_calls))

    def test_record_forward_does_not_overwrite_a_real_abort_with_forced_stop(self):
        def payload_for(heading):
            return {
                "nav": {"Lat": math.radians(52.2), "Lon": math.radians(-1.5), "Heading": heading},
                "status": {"NorthAcc": 0.05, "EastAcc": 0.05},
                "connection": {},
            }

        # The route's initial read (used to build the synthetic path and
        # to call start()) sees heading 0 — the path points due north.
        # Every later read (inside the step loop) sees heading 170 —
        # wildly misaligned with that path — forcing a real heading-
        # error abort well within the timeout. The final result must
        # reflect that real abort, not get clobbered by the "timed out"
        # force-stop path (which resets state to "idle").
        app.nav_client.queue_sequence([payload_for(0)] + [payload_for(170)] * 5)

        with patch.object(app, "send_velocity", self.recorder.send_velocity), \
             patch.object(app, "send_pump", self.recorder.send_pump):
            resp = self.client.post("/record/forward", json={"distance_m": 1.0, "speed_mps": 0.5})
        result = resp.get_json()
        self.assertEqual(result["state"], "aborted")
        self.assertIn("heading error", result["abort_reason"])

    def test_control_load_start_stop(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.assertEqual(self.client.post("/control/load/loop").get_json(), {"ok": True})

        self._set_position(52.2, -1.5, 0)  # at the first point, facing along the path
        resp = self.client.post("/control/start")
        self.assertEqual(resp.get_json(), {"ok": True})

        self.assertEqual(self.client.post("/control/stop").status_code, 204)
        self.assertEqual(self.recorder.velocity_calls[-1], (0.0, 0.0))

    def test_control_load_refused_while_a_path_is_already_running(self):
        # Found live 2026-07-31: a second caller (jobs, another
        # browser tab) loading a different path mid-run used to reset
        # straight to idle with no explicit stop, abandoning the run.
        other_points = [
            {"lat": 53.0, "lon": -2.0, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
            {"lat": 53.0005, "lon": -2.0, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
        ]
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        paths_module.save_path(app.PATHS_DIR, "other", other_points)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")

        resp = self.client.post("/control/load/other")
        self.assertEqual(resp.get_json(), {"ok": False, "reason": "another path is already running - stop it first"})
        self.assertEqual(app.runner.status()["state"], "running")  # untouched

    def test_control_load_allowed_again_once_stopped(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        paths_module.save_path(app.PATHS_DIR, "other", SAMPLE_POINTS)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")
        self.client.post("/control/stop")

        resp = self.client.post("/control/load/other")
        self.assertEqual(resp.get_json(), {"ok": True})

    def test_snapshot_reports_the_currently_loaded_path_name(self):
        # So the Run page can stay in sync with a path loaded a
        # different way - the Paths page's own Run button, or a job
        # driving navigate directly - neither of which go through this
        # page's own Load button. See navigate-prd.md's "Path entry".
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.assertIsNone(app._snapshot()["path_name"])
        self.client.post("/control/load/loop")
        self.assertEqual(app._snapshot()["path_name"], "loop")

    def test_snapshot_path_name_unchanged_by_a_refused_load(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        paths_module.save_path(app.PATHS_DIR, "other", SAMPLE_POINTS)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")

        self.client.post("/control/load/other")  # refused - a path is already running
        self.assertEqual(app._snapshot()["path_name"], "loop")

    def test_pump_manual_turns_pump_on_and_off(self):
        with patch.object(app, "send_pump", self.recorder.send_pump):
            resp_on = self.client.post("/pump/manual", json={"on": True})
            resp_off = self.client.post("/pump/manual", json={"on": False})
        self.assertEqual(resp_on.get_json(), {"ok": True})
        self.assertEqual(resp_off.get_json(), {"ok": True})
        self.assertEqual(self.recorder.pump_calls, [True, False])

    def test_pump_manual_refused_while_a_path_is_running(self):
        # jobs' "water" step sends this between path steps - it must
        # not be able to race an interactively-started path's own
        # per-tick pump commands (see navigate/control.py's step()).
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")

        with patch.object(app, "send_pump", self.recorder.send_pump):
            resp = self.client.post("/pump/manual", json={"on": True})
        self.assertEqual(resp.get_json(), {"ok": False, "reason": "a path is already running"})
        self.assertEqual(self.recorder.pump_calls, [])

    def test_successful_start_creates_a_fresh_debug_log(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")
        self.assertIsNotNone(app._debug_log_path)
        self.assertEqual(app._debug_log_path.read_text(), "")

    def test_failed_start_does_not_create_a_debug_log(self):
        # No path loaded -> entry_check fails -> start() returns ok: False
        self.client.post("/control/start", json={})
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")
        self.assertIsNone(app._debug_log_path)

    # --- Aruco priority (path flags + GNSS mode calls) ---

    def test_path_flags_default_to_aruco_priority_false(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        resp = self.client.get("/api/paths/loop/flags")
        self.assertEqual(resp.get_json(), {"aruco_priority": False})

    def test_path_flags_save_and_load_round_trip(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.client.post("/api/paths/loop/flags", json={"aruco_priority": True})
        resp = self.client.get("/api/paths/loop/flags")
        self.assertEqual(resp.get_json(), {"aruco_priority": True})

    def test_starting_an_aruco_priority_path_enters_aruco_priority_mode(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        paths_module.save_flags(app.PATHS_DIR, "loop", {"aruco_priority": True})
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        post = RecordingPost()
        with patch.object(app.requests, "post", post):
            self.client.post("/control/start")
        urls = [url for url, kwargs in post.calls]
        self.assertIn(f"{app.OXTSNAV_BASE_URL}/gnss/aruco-priority", urls)
        self.assertTrue(app._aruco_priority_active_for_run)

    def test_starting_an_ordinary_path_does_not_touch_gnss_mode(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        post = RecordingPost()
        with patch.object(app.requests, "post", post):
            self.client.post("/control/start")
        urls = [url for url, kwargs in post.calls]
        self.assertNotIn(f"{app.OXTSNAV_BASE_URL}/gnss/aruco-priority", urls)
        self.assertFalse(app._aruco_priority_active_for_run)

    def test_stopping_an_aruco_priority_run_restores_normal_gnss(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        paths_module.save_flags(app.PATHS_DIR, "loop", {"aruco_priority": True})
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        with patch.object(app.requests, "post", RecordingPost()):
            self.client.post("/control/start")
        post = RecordingPost()
        with patch.object(app.requests, "post", post):
            self.client.post("/control/stop")
        urls = [url for url, kwargs in post.calls]
        self.assertIn(f"{app.OXTSNAV_BASE_URL}/gnss/normal", urls)
        self.assertFalse(app._aruco_priority_active_for_run)

    def test_stopping_an_ordinary_run_does_not_call_gnss_normal(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")
        post = RecordingPost()
        with patch.object(app.requests, "post", post):
            self.client.post("/control/stop")
        self.assertEqual(post.calls, [])

    def test_control_tick_restores_gnss_when_run_aborts_on_its_own(self):
        # No /control/stop call at all - runner.step() aborts internally
        # (heading error breach), which _control_tick's own edge
        # detection must catch since there's no HTTP call marking the
        # moment otherwise.
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        paths_module.save_flags(app.PATHS_DIR, "loop", {"aruco_priority": True})
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        with patch.object(app.requests, "post", RecordingPost()):
            self.client.post("/control/start")
        self.assertTrue(app._aruco_priority_active_for_run)
        # The real control loop always ticks at least once while state
        # is genuinely "running" before anything else can happen to it
        # (that's how _control_loop_last_state gets to "running" in the
        # first place) - one clean tick first, matching that, rather
        # than jumping straight from start() to an abort on tick one.
        app._control_tick()
        self.assertEqual(app._control_loop_last_state, "running")

        self._set_position(52.2, -1.5, 170)  # a huge heading error -> abort
        post = RecordingPost()
        with patch.object(app.requests, "post", post):
            app._control_tick()
        urls = [url for url, kwargs in post.calls]
        self.assertIn(f"{app.OXTSNAV_BASE_URL}/gnss/normal", urls)
        self.assertFalse(app._aruco_priority_active_for_run)
        self.assertEqual(app.runner.status()["state"], "aborted")

    def test_two_successful_starts_leave_both_logs_on_disk(self):
        # Each run gets its own retained file, unlike the old
        # single-overwritten-file behaviour — see navigate-prd.md.
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")
        first_log_path = app._debug_log_path
        self.client.post("/control/stop")
        self.client.post("/control/start")
        second_log_path = app._debug_log_path
        self.assertNotEqual(first_log_path, second_log_path)
        self.assertTrue(first_log_path.exists())
        self.assertTrue(second_log_path.exists())

    def test_append_debug_log_writes_one_json_line_with_position_and_status(self):
        app._start_new_debug_log()
        app._append_debug_log({"lat": 52.2, "lon": -1.5, "heading_deg": 0, "horizontal_accuracy_m": 0.1})
        lines = app._debug_log_path.read_text().splitlines()
        self.assertEqual(len(lines), 1)
        entry = json.loads(lines[0])
        self.assertEqual(entry["lat"], 52.2)
        self.assertIn("state", entry)  # from runner.status()
        self.assertIn("t", entry)

    def test_debug_log_filename_and_lines_include_the_loaded_path_name(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")
        self.assertIn("loop", app._debug_log_path.name)
        app._append_debug_log({"lat": 52.2, "lon": -1.5, "heading_deg": 0, "horizontal_accuracy_m": 0.1})
        entry = json.loads(app._debug_log_path.read_text().splitlines()[-1])
        self.assertEqual(entry["path_name"], "loop")

    def test_append_debug_log_before_any_run_is_a_no_op(self):
        app._append_debug_log({"lat": 52.2, "lon": -1.5, "heading_deg": 0, "horizontal_accuracy_m": 0.1})
        self.assertFalse(app.LOGS_DIR.exists())

    def test_sweep_old_debug_logs_deletes_only_files_past_retention(self):
        app.LOGS_DIR.mkdir(parents=True)
        old_file = app.LOGS_DIR / "250101_000000.jsonl"
        new_file = app.LOGS_DIR / "260101_000000.jsonl"
        old_file.write_text("{}\n")
        new_file.write_text("{}\n")
        old_time = time.time() - (app.service_cfg["log_retention_days"] + 1) * 86400
        os.utime(old_file, (old_time, old_time))

        app._sweep_old_debug_logs()

        self.assertFalse(old_file.exists())
        self.assertTrue(new_file.exists())

    def test_control_start_without_position_fix(self):
        resp = self.client.post("/control/start")
        self.assertEqual(resp.get_json(), {"ok": False, "reason": "no position fix yet"})

    def test_control_entry_check(self):
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        self.assertEqual(self.client.get("/control/entry-check").get_json()["ok"], True)

    def test_jog_manual_proxies_to_drive(self):
        fake_response = requests.Response()
        fake_response.status_code = 200
        fake_response._content = b'{"accepted": true}'
        with patch.object(app.requests, "post", return_value=fake_response) as mock_post:
            resp = self.client.post("/jog/manual", json={"left_mps": 0.3, "right_mps": -0.3})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.get_json(), {"accepted": True})
        mock_post.assert_called_once_with(
            f"{app.DRIVE_BASE_URL}/command/manual",
            json={"left_mps": 0.3, "right_mps": -0.3},
            timeout=app.DRIVE_TIMEOUT_S,
        )

    def test_jog_manual_returns_502_if_drive_unreachable(self):
        with patch.object(app.requests, "post", side_effect=requests.exceptions.ConnectionError):
            resp = self.client.post("/jog/manual", json={"left_mps": 0.0, "right_mps": 0.0})
        self.assertEqual(resp.status_code, 502)

    def test_pages_render(self):
        for path in ["/", "/pages/create-path", "/pages/paths", "/pages/config", "/pages/edit-path", "/pages/logs"]:
            resp = self.client.get(path)
            self.assertEqual(resp.status_code, 200, path)

    def test_api_logs_empty_when_no_logs_dirs_exist(self):
        resp = self.client.get("/api/logs")
        self.assertEqual(resp.get_json(), {"navigate": [], "waterbutt": []})

    def test_api_logs_summarises_navigate_and_waterbutt_logs(self):
        app.LOGS_DIR.mkdir(parents=True)
        (app.LOGS_DIR / "260812_100000_loop.jsonl").write_text(
            '{"t": 100.0, "path_name": "loop"}\n{"t": 101.0, "path_name": "loop"}\n'
        )
        app.WATERBUTT_LOGS_DIR.mkdir(parents=True)
        (app.WATERBUTT_LOGS_DIR / "260812_090000.jsonl").write_text(
            '{"t": 90.0, "state": "not_visible"}\n'
        )

        resp = self.client.get("/api/logs").get_json()

        self.assertEqual(len(resp["navigate"]), 1)
        nav_entry = resp["navigate"][0]
        self.assertEqual(nav_entry["path_name"], "loop")
        self.assertEqual(nav_entry["start_t"], 100.0)
        self.assertEqual(nav_entry["end_t"], 101.0)
        self.assertEqual(nav_entry["line_count"], 2)

        self.assertEqual(len(resp["waterbutt"]), 1)
        self.assertEqual(resp["waterbutt"][0]["start_t"], 90.0)
        self.assertIsNone(resp["waterbutt"][0]["path_name"])

    def test_api_logs_ignores_an_empty_just_started_file(self):
        app.LOGS_DIR.mkdir(parents=True)
        (app.LOGS_DIR / "260812_100000_loop.jsonl").write_text("")

        resp = self.client.get("/api/logs").get_json()

        self.assertEqual(resp["navigate"], [{
            "filename": "260812_100000_loop.jsonl", "start_t": None, "end_t": None,
            "line_count": 0, "path_name": None,
        }])

    def test_api_log_file_serves_raw_content(self):
        app.LOGS_DIR.mkdir(parents=True)
        (app.LOGS_DIR / "260812_100000_loop.jsonl").write_text('{"t": 100.0}\n')

        resp = self.client.get("/api/logs/navigate/260812_100000_loop.jsonl")

        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.get_data(as_text=True), '{"t": 100.0}\n')

    def test_api_log_file_404s_for_an_unknown_filename(self):
        app.LOGS_DIR.mkdir(parents=True)
        resp = self.client.get("/api/logs/navigate/does-not-exist.jsonl")
        self.assertEqual(resp.status_code, 404)

    def test_api_log_file_404s_for_path_traversal(self):
        resp = self.client.get("/api/logs/navigate/..%2F..%2Fetc%2Fpasswd")
        self.assertEqual(resp.status_code, 404)

    def test_api_log_file_404s_for_an_unknown_source(self):
        app.LOGS_DIR.mkdir(parents=True)
        (app.LOGS_DIR / "260812_100000_loop.jsonl").write_text('{"t": 100.0}\n')
        resp = self.client.get("/api/logs/oxts-nav/260812_100000_loop.jsonl")
        self.assertEqual(resp.status_code, 404)


if __name__ == "__main__":
    unittest.main()
