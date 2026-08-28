"""End-to-end-ish tests against the Flask app and its tick logic. Never
makes a real socket connection to oxts-nav/drive: app.nav_client and
app.drive_client are replaced with fakes exposing .latest(), same
pattern as navigate/test_app.py's FakeNavClient."""

import json
import math
import tempfile
import unittest
from pathlib import Path

import app


class FakeFeedClient:
    def __init__(self, payload):
        self.payload = payload

    def latest(self):
        return self.payload


def make_nav_client(lat_deg=0.0, lon_deg=0.0, heading_deg=0.0, north_acc=0.01, east_acc=0.01, alt_acc=0.02):
    status = {}
    if north_acc is not None:
        status["NorthAcc"] = north_acc
    if east_acc is not None:
        status["EastAcc"] = east_acc
    if alt_acc is not None:
        status["AltAcc"] = alt_acc
    return FakeFeedClient({"nav": {"Lat": math.radians(lat_deg), "Lon": math.radians(lon_deg), "Heading": heading_deg}, "status": status, "connection": {}})


class TestEligibility(unittest.TestCase):
    def setUp(self):
        app.set_logging_enabled(False)
        app.set_raw_capture_enabled(False)
        app.grid._cells = {}
        app.grid._overrides = {}
        app.MAP_ORIGIN_LAT = 0.0
        app.MAP_ORIGIN_LON = 0.0
        app.nav_client = make_nav_client()
        app.drive_client = FakeFeedClient({})

    def test_logging_off_by_default_blocks_updates(self):
        app._tick()
        self.assertEqual(app.grid.stats()["cells"], 0)

    def test_enabling_logging_with_good_accuracy_updates_the_grid(self):
        app.set_logging_enabled(True)
        app.drive_client = FakeFeedClient({"ultrasonic_0_mm": 500})
        app._tick()
        self.assertGreater(app.grid.stats()["cells"], 0)

    def test_poor_horizontal_accuracy_blocks_updates(self):
        app.set_logging_enabled(True)
        app.nav_client = make_nav_client(north_acc=1.0, east_acc=1.0)
        app.drive_client = FakeFeedClient({"ultrasonic_0_mm": 500})
        app._tick()
        self.assertEqual(app.grid.stats()["cells"], 0)

    def test_no_fix_blocks_updates(self):
        app.set_logging_enabled(True)
        app.nav_client = FakeFeedClient({"nav": {}, "status": {}, "connection": {}})
        app.drive_client = FakeFeedClient({"ultrasonic_0_mm": 500})
        app._tick()
        self.assertEqual(app.grid.stats()["cells"], 0)

    def test_eligibility_reason_reported_when_logging_off(self):
        self.assertEqual(app._eligibility(app._current_pose()), "logging is off")

    def test_eligibility_none_when_everything_checks_out(self):
        app.set_logging_enabled(True)
        self.assertIsNone(app._eligibility(app._current_pose()))


class TestLoggingEndpoint(unittest.TestCase):
    def setUp(self):
        app.set_logging_enabled(False)
        self.client = app.app.test_client()

    def test_post_logging_enabled_true_turns_it_on(self):
        resp = self.client.post("/logging", json={"enabled": True})
        self.assertEqual(resp.get_json(), {"enabled": True})
        self.assertTrue(app.logging_enabled())

    def test_post_logging_enabled_false_turns_it_off(self):
        app.set_logging_enabled(True)
        resp = self.client.post("/logging", json={"enabled": False})
        self.assertEqual(resp.get_json(), {"enabled": False})
        self.assertFalse(app.logging_enabled())


class TestRawCapture(unittest.TestCase):
    def setUp(self):
        app.set_logging_enabled(False)
        app.set_raw_capture_enabled(False)
        app.RAW_LOGS_DIR = Path(tempfile.mkdtemp())
        app.MAP_ORIGIN_LAT = 0.0
        app.MAP_ORIGIN_LON = 0.0
        app.nav_client = make_nav_client()
        app.drive_client = FakeFeedClient({"ultrasonic_0_mm": 500})

    def test_off_by_default_writes_nothing(self):
        app._tick()
        self.assertEqual(list(app.RAW_LOGS_DIR.glob("*.jsonl")), [])

    def test_enabling_starts_a_new_file_and_records_a_reading(self):
        app.set_raw_capture_enabled(True)
        app._tick()
        files = list(app.RAW_LOGS_DIR.glob("*.jsonl"))
        self.assertEqual(len(files), 1)
        record = json.loads(files[0].read_text().splitlines()[0])
        self.assertEqual(record["tag"], 0)
        self.assertEqual(record["range_mm"], 500)

    def test_independent_of_the_live_grid_switch(self):
        # Raw capture on, live grid logging off - only the raw file
        # should get anything.
        app.set_raw_capture_enabled(True)
        app._tick()
        self.assertEqual(app.grid.stats()["cells"], 0)
        self.assertEqual(len(list(app.RAW_LOGS_DIR.glob("*.jsonl"))), 1)

    def test_poor_accuracy_blocks_raw_capture_too(self):
        app.set_raw_capture_enabled(True)
        app.nav_client = make_nav_client(north_acc=1.0, east_acc=1.0)
        app._tick()
        # A session file is opened as soon as the switch flips on, but
        # nothing's ever appended to it while accuracy stays poor.
        self.assertFalse(any(f.stat().st_size > 0 for f in app.RAW_LOGS_DIR.glob("*.jsonl")))

    def test_raw_capture_endpoint_toggles(self):
        client = app.app.test_client()
        resp = client.post("/raw-capture", json={"enabled": True})
        self.assertEqual(resp.get_json(), {"enabled": True})
        self.assertTrue(app.raw_capture_enabled())


class TestReprocessEndpoint(unittest.TestCase):
    def setUp(self):
        self.raw_dir = Path(tempfile.mkdtemp())
        self.grids_dir = Path(tempfile.mkdtemp())
        app.RAW_LOGS_DIR = self.raw_dir
        app.GRIDS_DIR = self.grids_dir
        (self.raw_dir / "a.jsonl").write_text(
            json.dumps({"t": 1.0, "north": 0.0, "east": 0.0, "heading_deg": 0.0, "tag": 0, "range_mm": 500}) + "\n"
        )
        self.client = app.app.test_client()

    def test_api_raw_logs_lists_available_files(self):
        resp = self.client.get("/api/raw-logs")
        entries = resp.get_json()
        self.assertEqual(len(entries), 1)
        self.assertEqual(entries[0]["filename"], "a.jsonl")

    def test_reprocess_writes_a_snapshot_and_reports_it(self):
        resp = self.client.post("/reprocess", json={
            "files": ["a.jsonl"], "p_hit": 0.7, "p_miss": 0.3, "p_min": 0.02, "p_max": 0.98,
            "cell_size_m": 0.1, "max_range_m": 1.2, "beam_half_angle_deg": 7.5,
        })
        result = resp.get_json()
        self.assertTrue(result["ok"])
        self.assertTrue((self.grids_dir / result["grid_filename"]).exists())
        self.assertTrue((self.grids_dir / result["settings_filename"]).exists())
        self.assertGreater(result["stats"]["cells"], 0)

    def test_reprocess_with_no_files_is_refused(self):
        resp = self.client.post("/reprocess", json={
            "files": [], "p_hit": 0.7, "p_miss": 0.3, "p_min": 0.02, "p_max": 0.98,
            "cell_size_m": 0.1, "max_range_m": 1.2, "beam_half_angle_deg": 7.5,
        })
        self.assertEqual(resp.status_code, 400)


if __name__ == "__main__":
    unittest.main()
