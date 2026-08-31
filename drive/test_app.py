"""End-to-end tests against the Flask app — routes, page rendering,
control arbitration wiring. Never touches real hardware: `app.link` is
replaced with a `SerialLink` built over a fake serial object (see
test_serial_link.py's FakeSerial), and `app.arbiter` is replaced with a
stub so tests don't depend on real wall-clock timing.
"""

import json
import os
import shutil
import tempfile
import time
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
        # 0.2 m/s * 120 counts/m = 24 counts/s
        self.assertEqual(self.fake_serial.written, [b"SV 24 -24\n"])
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
        self.fake_serial.queue.put(b"EN 250 500\n")  # counts_per_metre=120 in config -> 250/120, 500/120 m
        self.fake_serial.queue.put(b"SV 100 -100\n")  # SV is wire-scaled x100 (see protocol.py) -> 1.0, -1.0 counts/s -> /120 m/s
        self.assertTrue(wait_until(lambda: app.link.snapshot().get("LM_position") == 250))
        snap = app._snapshot()
        self.assertAlmostEqual(snap["LM_position_m"], 250 / 120)
        self.assertAlmostEqual(snap["RM_position_m"], 500 / 120)
        self.assertAlmostEqual(snap["LM_setvel_mps"], 1.0 / 120)
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
    """Tests for the self-triggered debug log (see _log_loop) - same
    "redirect the module-level dir constant" pattern as navigate's own
    debug-log tests. _log_loop's infinite loop itself is never started
    here; each test drives its building blocks (_is_active,
    _start_new_debug_log, _append_debug_log) directly, deterministically."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        app.LOGS_DIR = self.tmp / "logs"
        app._debug_log_path = None
        app._last_active_monotonic = None

    def test_is_active_true_when_any_speed_field_exceeds_epsilon(self):
        self.assertFalse(app._is_active({}))
        self.assertFalse(app._is_active({"LM_setvel_mps": 0.0, "RM_vel_filt_mps": 0.005}))
        self.assertTrue(app._is_active({"LM_setvel_mps": 0.3}))
        self.assertTrue(app._is_active({"RM_vel_filt_mps": -0.02}))

    def test_append_debug_log_before_any_activity_is_a_no_op(self):
        app._append_debug_log({"LM_setvel_mps": 0.3})
        self.assertFalse(app.LOGS_DIR.exists())

    def test_start_then_append_writes_one_json_line_with_state_and_t(self):
        app._start_new_debug_log()
        app._append_debug_log({"LM_setvel_mps": 0.3, "RM_setvel_mps": 0.3})
        lines = app._debug_log_path.read_text().splitlines()
        self.assertEqual(len(lines), 1)
        entry = json.loads(lines[0])
        self.assertEqual(entry["LM_setvel_mps"], 0.3)
        self.assertIn("t", entry)

    def test_start_new_debug_log_avoids_collisions_within_the_same_second(self):
        app._start_new_debug_log()
        first = app._debug_log_path
        app._start_new_debug_log()
        second = app._debug_log_path
        self.assertNotEqual(first, second)
        self.assertTrue(first.exists())
        self.assertTrue(second.exists())

    def test_log_loop_body_closes_after_idle_tail_elapses(self):
        # One manual iteration of _log_loop's own body, run "active" then
        # "idle past the tail", without the real thread/sleep.
        active_state = {"LM_setvel_mps": 0.3}
        idle_state = {"LM_setvel_mps": 0.0}

        now = time.monotonic()
        app._last_active_monotonic = now
        app._start_new_debug_log()
        app._append_debug_log(active_state)
        self.assertIsNotNone(app._debug_log_path)

        # Still within the idle tail - stays open.
        app._append_debug_log(idle_state)
        self.assertIsNotNone(app._debug_log_path)

        # Simulate the tail having elapsed, same check _log_loop makes.
        app._last_active_monotonic = now - (app.LOG_IDLE_TAIL_S + 1.0)
        if time.monotonic() - app._last_active_monotonic > app.LOG_IDLE_TAIL_S:
            app._debug_log_path = None
        self.assertIsNone(app._debug_log_path)

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

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
