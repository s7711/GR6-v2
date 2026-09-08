"""End-to-end tests against the Flask app — routes, page rendering,
control arbitration wiring. Never touches real hardware: `app.link` is
replaced with a `SerialLink` built over a fake serial object (see
test_serial_link.py's FakeSerial), and `app.arbiter` is replaced with a
stub so tests don't depend on real wall-clock timing.
"""

import json
import shutil
import tempfile
import unittest
from pathlib import Path

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
        # Derived from the live counts_per_metre, not hardcoded - this
        # went stale twice before (120 -> 250 -> ...) as the real config
        # got recalibrated, see drive-prd.md's counts_per_metre history.
        counts_s = round(app.mps_to_counts_s(0.2))
        self.assertEqual(self.fake_serial.written, [f"SV {counts_s} {-counts_s}\n".encode()])
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

    def test_power_off(self):
        resp = self.client.post("/power-off", json={"delay_s": 20})
        self.assertEqual(resp.status_code, 204)
        self.assertEqual(self.fake_serial.written, [b"P_OFF 20\n"])

    def test_snapshot_converts_to_physical_units(self):
        # Derived from the live counts_per_metre, not hardcoded - see
        # test_manual_command_accepted_and_forwarded's own note.
        self.fake_serial.queue.put(b"EN 250 500\n")
        self.fake_serial.queue.put(b"SV 100 -100\n")  # SV is wire-scaled x100 (see protocol.py) -> 1.0, -1.0 counts/s
        self.assertTrue(wait_until(lambda: app.link.snapshot().get("LM_position") == 250))
        snap = app._snapshot()
        self.assertAlmostEqual(snap["LM_position_m"], app.counts_s_to_mps(250))
        self.assertAlmostEqual(snap["RM_position_m"], app.counts_s_to_mps(500))
        self.assertAlmostEqual(snap["LM_setvel_mps"], app.counts_s_to_mps(1.0))
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


class DriveLoggingTestCase(unittest.TestCase):
    """Continuous logging (see _log_loop) is now just app's own
    RotatingJsonlLog instance (`debug_log`) - the rotation/retention/
    write mechanics themselves are shared/test_rotating_jsonl_log.py's
    job to cover, not re-tested per consumer service. These tests only
    confirm drive wires it up correctly: pointed at LOGS_DIR, and
    _log_loop's body actually appends a real snapshot to it."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        app.LOGS_DIR = self.tmp / "logs"
        app.debug_log = app.RotatingJsonlLog(
            app.LOGS_DIR, app.service_cfg["log_rotate_s"], app.service_cfg["log_retention_days"]
        )
        app.debug_log.start()

    def test_log_loop_body_appends_a_real_snapshot(self):
        app.debug_log.append(app._snapshot())
        files = list(app.LOGS_DIR.glob("*.jsonl"))
        self.assertEqual(len(files), 1)
        entry = json.loads(files[0].read_text().splitlines()[0])
        self.assertIn("t", entry)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
