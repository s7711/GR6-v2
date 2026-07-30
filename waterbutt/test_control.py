import unittest
import unittest.mock as mock

import control
from control import ValveController


class Recorder:
    def __init__(self):
        self.open_calls = 0
        self.close_calls = 0

    def send_open(self):
        self.open_calls += 1

    def send_close(self):
        self.close_calls += 1


def make_valve():
    recorder = Recorder()
    valve = ValveController(recorder.send_open, recorder.send_close)
    return valve, recorder


class TestGo(unittest.TestCase):
    def test_go_opens_immediately_and_sets_state(self):
        valve, recorder = make_valve()
        valve.go(10)
        self.assertEqual(recorder.open_calls, 1)
        self.assertEqual(valve.status()["state"], "watering")
        self.assertEqual(valve.status()["duration_s"], 10)

    def test_restarting_while_already_watering_resets_the_duration(self):
        valve, recorder = make_valve()
        valve.go(10)
        valve.go(5)
        self.assertEqual(valve.status()["duration_s"], 5)
        self.assertEqual(recorder.open_calls, 2)


class TestTick(unittest.TestCase):
    def test_tick_is_a_noop_while_idle(self):
        valve, recorder = make_valve()
        valve.tick()
        self.assertEqual(recorder.open_calls, 0)
        self.assertEqual(recorder.close_calls, 0)

    def test_tick_repulses_open_before_the_firmware_failsafe_elapses(self):
        valve, recorder = make_valve()
        with mock.patch("control.time.monotonic") as fake_time:
            fake_time.return_value = 0.0
            valve.go(10)
            self.assertEqual(recorder.open_calls, 1)

            fake_time.return_value = control.REOPEN_INTERVAL_S - 0.1
            valve.tick()
            self.assertEqual(recorder.open_calls, 1, "too early to need a repulse yet")

            fake_time.return_value = control.REOPEN_INTERVAL_S + 0.1
            valve.tick()
            self.assertEqual(recorder.open_calls, 2)

    def test_tick_closes_and_returns_to_idle_once_duration_elapses(self):
        valve, recorder = make_valve()
        with mock.patch("control.time.monotonic") as fake_time:
            fake_time.return_value = 0.0
            valve.go(10)

            fake_time.return_value = 10.1
            valve.tick()
            self.assertEqual(recorder.close_calls, 1)
            self.assertEqual(valve.status()["state"], "idle")
            self.assertEqual(valve.status()["remaining_s"], 0.0)

    def test_tick_does_not_repulse_after_closing(self):
        valve, recorder = make_valve()
        with mock.patch("control.time.monotonic") as fake_time:
            fake_time.return_value = 0.0
            valve.go(1)
            fake_time.return_value = 1.1
            valve.tick()
            fake_time.return_value = 100.0
            valve.tick()
        self.assertEqual(recorder.close_calls, 1)
        self.assertEqual(recorder.open_calls, 1)

    def test_remaining_s_counts_down(self):
        valve, _recorder = make_valve()
        with mock.patch("control.time.monotonic") as fake_time:
            fake_time.return_value = 0.0
            valve.go(10)
            fake_time.return_value = 4.0
            valve.tick()
        self.assertEqual(valve.status()["remaining_s"], 6.0)


class TestStop(unittest.TestCase):
    def test_stop_closes_and_returns_to_idle(self):
        valve, recorder = make_valve()
        valve.go(10)
        valve.stop()
        self.assertEqual(recorder.close_calls, 1)
        self.assertEqual(valve.status()["state"], "idle")
        self.assertEqual(valve.status()["remaining_s"], 0.0)

    def test_stop_is_safe_to_call_while_already_idle(self):
        valve, recorder = make_valve()
        valve.stop()
        self.assertEqual(recorder.close_calls, 1)
        self.assertEqual(valve.status()["state"], "idle")

    def test_stop_prevents_a_pending_tick_from_reopening(self):
        valve, recorder = make_valve()
        with mock.patch("control.time.monotonic") as fake_time:
            fake_time.return_value = 0.0
            valve.go(10)
            valve.stop()
            fake_time.return_value = control.REOPEN_INTERVAL_S + 0.1
            valve.tick()
        self.assertEqual(recorder.open_calls, 1)


if __name__ == "__main__":
    unittest.main()
