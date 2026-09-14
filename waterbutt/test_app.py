"""End-to-end tests against the Flask app's /go route — specifically
the QC gating added 2026-08-11 (see waterbutt-prd.md's "QC gating on
fill"). Never touches the real ESP8266: app.valve is replaced with a
stub, and app._set_qc_reading() drives the QC state directly rather
than needing a real aruco feed.
"""

import json
import os
import tempfile
import time
import unittest
import unittest.mock
from pathlib import Path

import app


class ValveStub:
    def __init__(self):
        self.go_calls = []

    def go(self, duration_s):
        self.go_calls.append(duration_s)

    def status(self):
        return {"state": "idle", "duration_s": 0.0, "remaining_s": 0.0}


class WaterbuttAppTestCase(unittest.TestCase):
    def setUp(self):
        self.valve_stub = ValveStub()
        app.valve = self.valve_stub
        # LOGS_DIR/_qc_log_path are computed/set once at import/startup,
        # not re-derived here - redirect both so tests never touch this
        # robot's real QC logs.
        app.LOGS_DIR = Path(tempfile.mkdtemp()) / "logs"
        app._qc_log_path = None
        app._set_qc_reading({"state": "not_configured"})
        app.QC_SETTLE_S = 0  # don't actually wait out the settle delay in tests
        # These tests are about QC gating specifically - start each one
        # already past the separate tank-level gate (see
        # TestTankLevelGating below for that gate's own tests) so it
        # doesn't interfere.
        app.level.mark_empty()
        self.client = app.app.test_client()

    def test_go_refused_when_no_qc_marker_configured(self):
        resp = self.client.post("/go", json={"duration_s": 5})
        self.assertEqual(resp.status_code, 409)
        self.assertIn("no QC marker configured", resp.get_json()["reason"])
        self.assertEqual(self.valve_stub.go_calls, [])

    def test_go_refused_when_marker_not_visible(self):
        app._set_qc_reading({"state": "not_visible", "marker_id": 12})
        resp = self.client.post("/go", json={"duration_s": 5})
        self.assertEqual(resp.status_code, 409)
        self.assertIn("not visible", resp.get_json()["reason"])
        self.assertEqual(self.valve_stub.go_calls, [])

    def test_go_refused_when_aruco_unreachable(self):
        app._set_qc_reading({"state": "aruco_unreachable"})
        resp = self.client.post("/go", json={"duration_s": 5})
        self.assertEqual(resp.status_code, 409)
        self.assertEqual(self.valve_stub.go_calls, [])

    def test_go_refused_when_beyond_threshold(self):
        app._set_qc_reading({"state": "ok", "marker_id": 12, "distance_m": 0.15})
        resp = self.client.post("/go", json={"duration_s": 5})
        self.assertEqual(resp.status_code, 409)
        self.assertIn("15.0cm", resp.get_json()["reason"])
        self.assertIn("8cm", resp.get_json()["reason"])  # app.QC_THRESHOLD_M
        self.assertEqual(self.valve_stub.go_calls, [])

    def test_go_allowed_within_threshold(self):
        app._set_qc_reading({"state": "ok", "marker_id": 12, "distance_m": 0.04})
        resp = self.client.post("/go", json={"duration_s": 5})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(self.valve_stub.go_calls, [5])

    def test_go_allowed_exactly_at_threshold(self):
        app._set_qc_reading({"state": "ok", "marker_id": 12, "distance_m": 0.08})
        resp = self.client.post("/go", json={"duration_s": 5})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(self.valve_stub.go_calls, [5])

    def test_go_rejects_a_duration_outside_the_allow_list(self):
        app._set_qc_reading({"state": "ok", "marker_id": 12, "distance_m": 0.01})
        resp = self.client.post("/go", json={"duration_s": 7})
        self.assertEqual(resp.status_code, 400)
        self.assertEqual(self.valve_stub.go_calls, [])

    def test_go_waits_out_the_qc_settle_delay_before_checking(self):
        # Added 2026-09-14: used to be a "pause" step individual jobs had
        # to remember to add before their fill step; now /go enforces it
        # itself for every caller. QC_SETTLE_S is forced to 0 in setUp()
        # so the other tests here don't actually sleep - undo that just
        # for this one test.
        app.QC_SETTLE_S = 3
        app._set_qc_reading({"state": "ok", "marker_id": 12, "distance_m": 0.01})
        with unittest.mock.patch.object(app.time, "sleep") as mock_sleep:
            resp = self.client.post("/go", json={"duration_s": 5})
        mock_sleep.assert_called_once_with(3)
        self.assertEqual(resp.status_code, 200)

    def test_set_qc_reading_before_a_log_started_is_a_no_op(self):
        app._set_qc_reading({"state": "ok", "marker_id": 12, "distance_m": 0.01})
        self.assertFalse(app.LOGS_DIR.exists())

    def test_set_qc_reading_appends_to_the_current_log(self):
        app._start_new_qc_log()
        app._set_qc_reading({"state": "not_visible", "marker_id": 12})
        app._set_qc_reading({"state": "ok", "marker_id": 12, "distance_m": 0.03})
        lines = app._qc_log_path.read_text().splitlines()
        self.assertEqual(len(lines), 2)
        first, second = (json.loads(line) for line in lines)
        self.assertEqual(first["state"], "not_visible")
        self.assertIn("t", first)
        self.assertEqual(second["distance_m"], 0.03)

    def test_set_qc_reading_rotates_to_a_new_file_past_the_rotate_interval(self):
        app._start_new_qc_log()
        first_path = app._qc_log_path
        first_path.write_text('{"state": "not_visible"}\n')  # as if a reading had already been logged to it
        app._qc_log_opened_at = time.monotonic() - (app.service_cfg["qc_log_rotate_s"] + 1)

        app._set_qc_reading({"state": "not_visible", "marker_id": 12})

        self.assertNotEqual(app._qc_log_path, first_path)
        self.assertTrue(first_path.exists())  # rotation doesn't delete the old file, only starts a new one
        self.assertEqual(len(app._qc_log_path.read_text().splitlines()), 1)

    def test_set_qc_reading_does_not_rotate_before_the_interval_elapses(self):
        app._start_new_qc_log()
        first_path = app._qc_log_path

        app._set_qc_reading({"state": "not_visible", "marker_id": 12})

        self.assertEqual(app._qc_log_path, first_path)

    def test_sweep_old_qc_logs_deletes_only_files_past_retention(self):
        app.LOGS_DIR.mkdir(parents=True)
        old_file = app.LOGS_DIR / "250101_000000.jsonl"
        new_file = app.LOGS_DIR / "260101_000000.jsonl"
        old_file.write_text("{}\n")
        new_file.write_text("{}\n")
        old_time = time.time() - (app.service_cfg["log_retention_days"] + 1) * 86400
        os.utime(old_file, (old_time, old_time))

        app._sweep_old_qc_logs()

        self.assertFalse(old_file.exists())
        self.assertTrue(new_file.exists())


class TestTankLevelGating(unittest.TestCase):
    """/go's second gate (added 2026-08-18) - see level.py/waterbutt-
    prd.md's "Tank level estimate". QC is set to always pass here so
    only the level gate is under test."""

    def setUp(self):
        self.valve_stub = ValveStub()
        app.valve = self.valve_stub
        app.LOGS_DIR = Path(tempfile.mkdtemp()) / "logs"
        app._qc_log_path = None
        app._set_qc_reading({"state": "ok", "distance_m": 0.0})
        app.QC_SETTLE_S = 0  # don't actually wait out the settle delay in tests
        self.client = app.app.test_client()

    def test_go_refused_when_not_believed_empty(self):
        resp = self.client.post("/go", json={"duration_s": 5})
        self.assertEqual(resp.status_code, 409)
        self.assertIn("not confident the butt is empty", resp.get_json()["reason"])
        self.assertEqual(self.valve_stub.go_calls, [])

    def test_go_allowed_once_believed_empty(self):
        app.level.mark_empty()
        resp = self.client.post("/go", json={"duration_s": 5})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(self.valve_stub.go_calls, [5])

    def test_successful_go_marks_full_again(self):
        app.level.mark_empty()
        self.client.post("/go", json={"duration_s": 5})
        self.assertFalse(app.level.believed_empty())

    def test_mark_empty_endpoint_allows_a_subsequent_go(self):
        self.client.post("/level/mark-empty")
        resp = self.client.post("/go", json={"duration_s": 5})
        self.assertEqual(resp.status_code, 200)

    def test_mark_full_endpoint_refuses_a_subsequent_go(self):
        app.level.mark_empty()
        self.client.post("/level/mark-full")
        resp = self.client.post("/go", json={"duration_s": 5})
        self.assertEqual(resp.status_code, 409)


if __name__ == "__main__":
    unittest.main()
