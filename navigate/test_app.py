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
import tempfile
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


class NavigateAppTestCase(unittest.TestCase):
    def setUp(self):
        app.PATHS_DIR = Path(tempfile.mkdtemp())
        # DEBUG_LOG_PATH is computed once from PATHS_DIR at import time,
        # not re-derived when PATHS_DIR is reassigned above — redirect it
        # too, so tests never touch this robot's real debug log.
        app.DEBUG_LOG_PATH = app.PATHS_DIR / "last_run_debug.jsonl"
        self.recorder = Recorder()
        app.runner = PathRunner(app.CONTROL_CONFIG, self.recorder.send_velocity, self.recorder.send_pump)
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
        self.assertEqual(self.client.post("/control/load/loop").status_code, 204)

        self._set_position(52.2, -1.5, 0)  # at the first point, facing along the path
        resp = self.client.post("/control/start")
        self.assertEqual(resp.get_json(), {"ok": True})

        self.assertEqual(self.client.post("/control/stop").status_code, 204)
        self.assertEqual(self.recorder.velocity_calls[-1], (0.0, 0.0))

    def test_successful_start_resets_the_debug_log(self):
        app.DEBUG_LOG_PATH.write_text('{"stale": "entry from a previous run"}\n')
        paths_module.save_path(app.PATHS_DIR, "loop", SAMPLE_POINTS)
        self.client.post("/control/load/loop")
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")
        self.assertEqual(app.DEBUG_LOG_PATH.read_text(), "")

    def test_failed_start_does_not_touch_the_debug_log(self):
        app.DEBUG_LOG_PATH.write_text('{"kept": true}\n')
        # No path loaded -> entry_check fails -> start() returns ok: False
        self.client.post("/control/start", json={})
        self._set_position(52.2, -1.5, 0)
        self.client.post("/control/start")
        self.assertIn("kept", app.DEBUG_LOG_PATH.read_text())

    def test_append_debug_log_writes_one_json_line_with_position_and_status(self):
        app._reset_debug_log()
        app._append_debug_log({"lat": 52.2, "lon": -1.5, "heading_deg": 0, "horizontal_accuracy_m": 0.1})
        lines = app.DEBUG_LOG_PATH.read_text().splitlines()
        self.assertEqual(len(lines), 1)
        entry = json.loads(lines[0])
        self.assertEqual(entry["lat"], 52.2)
        self.assertIn("state", entry)  # from runner.status()
        self.assertIn("t", entry)

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
        for path in ["/", "/pages/create-path", "/pages/paths", "/pages/config"]:
            resp = self.client.get(path)
            self.assertEqual(resp.status_code, 200, path)


if __name__ == "__main__":
    unittest.main()
