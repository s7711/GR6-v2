"""End-to-end-ish tests against the Flask app and its tick logic. Never
makes a real socket connection to oxts-nav/drive: app.nav_client and
app.drive_client are replaced with fakes exposing .latest(), same
pattern as navigate/test_app.py's FakeNavClient."""

import math
import unittest

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


if __name__ == "__main__":
    unittest.main()
