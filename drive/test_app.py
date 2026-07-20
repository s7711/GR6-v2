"""End-to-end tests against the Flask app — routes, page rendering,
control arbitration wiring. Never touches real hardware: `app.link` is
replaced with a `SerialLink` built over a fake serial object (see
test_serial_link.py's FakeSerial), and `app.arbiter` is replaced with a
stub so tests don't depend on real wall-clock timing.
"""

import json
import time
import unittest

import app
from serial_link import SerialLink
from test_serial_link import FakeSerial, wait_until


class StubArbiter:
    """Deterministic stand-in for ControlArbiter — avoids real-time waits
    in tests that only care about *whether* a command gets forwarded,
    not the exact hold-timer behaviour (that's covered by test_control.py)."""

    def __init__(self, accept=True):
        self.accept = accept
        self.calls = []

    def try_command(self, source):
        self.calls.append(source)
        return self.accept

    def status(self):
        return {"controller": "test", "manual_lock_until": None}


class DriveAppTestCase(unittest.TestCase):
    def setUp(self):
        self.fake_serial = FakeSerial("ignored", 115200, 0.05)
        app.link = SerialLink("ignored-port", 115200, serial_factory=lambda port, baudrate, timeout: self.fake_serial)
        app.link.start()
        app.arbiter = StubArbiter(accept=True)
        self.client = app.app.test_client()

    def test_manual_command_accepted_and_forwarded(self):
        resp = self.client.post("/command/manual", json={"left_mps": 0.2, "right_mps": -0.2})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.get_json(), {"accepted": True})
        # 0.2 m/s * 250 counts/m = 50 counts/s
        self.assertEqual(self.fake_serial.written, [b"SV 50 -50\n"])
        self.assertEqual(app.arbiter.calls, ["manual"])

    def test_auto_command_rejected_is_not_forwarded(self):
        app.arbiter = StubArbiter(accept=False)
        resp = self.client.post("/command/auto", json={"left_mps": 0.2, "right_mps": 0.2})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.get_json(), {"accepted": False})
        self.assertEqual(self.fake_serial.written, [])

    def test_pump_on_off(self):
        self.client.post("/pump", json={"on": True})
        self.client.post("/pump", json={"on": False})
        self.assertEqual(self.fake_serial.written, [b"WP 1\n", b"WP 0\n"])

    def test_tuning_valid_param(self):
        resp = self.client.post("/tuning", json={"name": "Kp", "left": 1.5, "right": 2.0})
        self.assertEqual(resp.status_code, 204)
        self.assertEqual(self.fake_serial.written, [b"Kp 150 200\n"])

    def test_tuning_unknown_param_rejected(self):
        resp = self.client.post("/tuning", json={"name": "Nope", "left": 1.0, "right": 1.0})
        self.assertEqual(resp.status_code, 400)
        self.assertEqual(self.fake_serial.written, [])

    def test_snapshot_converts_to_physical_units(self):
        self.fake_serial.queue.put(b"EN 250 500\n")  # counts_per_metre=250 in config -> 1m, 2m
        self.fake_serial.queue.put(b"SV 100 -100\n")  # counts/s x100 -> 1.0, -1.0 counts/s -> /250 m/s
        self.assertTrue(wait_until(lambda: app.link.snapshot().get("LM_position") == 250))
        snap = app._snapshot()
        self.assertEqual(snap["LM_position_m"], 1.0)
        self.assertEqual(snap["RM_position_m"], 2.0)
        self.assertAlmostEqual(snap["LM_setvel_mps"], 0.004)  # 1.0 counts/s / 250
        self.assertIn("control", snap)
        self.assertIn("firmware", snap)

    def test_firmware_status_computed_live_from_current_state(self):
        # Regression test: firmware status used to be captured once by a
        # blocking startup check and then frozen — if the Version line
        # arrived late (it's one of several telemetry tags on a slow
        # round-robin cycle), the Home page banner got stuck reporting
        # "no version received" forever, even after a real version showed
        # up. It must now be recomputed from live state on every call.
        self.assertIsNone(app._snapshot()["firmware"]["actual"])
        self.fake_serial.queue.put(b"Version 1.0\n")
        self.assertTrue(wait_until(lambda: app.link.snapshot().get("firmware_version") == "1.0"))
        status = app._snapshot()["firmware"]
        self.assertEqual(status["actual"], "1.0")
        self.assertEqual(status["expected"], app.service_cfg.get("expected_firmware_version"))

    def test_home_page_renders(self):
        resp = self.client.get("/")
        self.assertEqual(resp.status_code, 200)
        self.assertIn(b"joystick", resp.data)

    def test_tuning_page_lists_params(self):
        resp = self.client.get("/pages/tuning")
        self.assertEqual(resp.status_code, 200)
        self.assertIn(b"Kp", resp.data)

    def test_ultrasonics_page_renders(self):
        resp = self.client.get("/pages/ultrasonics")
        self.assertEqual(resp.status_code, 200)
        self.assertIn(b"wedge-0", resp.data)

    def test_config_page_shows_serial_port(self):
        resp = self.client.get("/pages/config")
        self.assertEqual(resp.status_code, 200)
        self.assertIn(app.service_cfg["serial_port"].encode(), resp.data)


if __name__ == "__main__":
    unittest.main()
