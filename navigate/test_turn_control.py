import unittest

from turn_control import TurnRunner

CONFIG = {
    "wheel_base_m": 0.42,
    "turn_gain": 1.0,
    "turn_max_rate_rad_s": 0.5,
    "turn_max_mps": 0.15,
    "stall_check_window_s": 5.0,
    "turn_stall_min_deg": 5.0,
}


class Recorder:
    def __init__(self):
        self.velocity_calls = []

    def send_velocity(self, left, right):
        self.velocity_calls.append((left, right))


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_runner(clock=None):
    recorder = Recorder()
    kwargs = {"now": clock} if clock is not None else {}
    return TurnRunner(CONFIG, recorder.send_velocity, **kwargs), recorder


class TestStartStop(unittest.TestCase):
    def test_start_succeeds_and_sets_running(self):
        runner, _ = make_runner()
        result = runner.start(target_heading_deg=90, tolerance_deg=10, robot_heading_deg=0)
        self.assertTrue(result["ok"])
        self.assertEqual(runner.status()["state"], "running")

    def test_start_refused_while_already_running(self):
        runner, _ = make_runner()
        runner.start(target_heading_deg=90, tolerance_deg=10, robot_heading_deg=0)
        result = runner.start(target_heading_deg=45, tolerance_deg=10, robot_heading_deg=0)
        self.assertFalse(result["ok"])

    def test_stop_sends_zero_velocity_and_sets_idle(self):
        runner, recorder = make_runner()
        runner.start(target_heading_deg=90, tolerance_deg=10, robot_heading_deg=0)
        runner.stop()
        self.assertEqual(runner.status()["state"], "idle")
        self.assertEqual(recorder.velocity_calls[-1], (0.0, 0.0))


class TestStep(unittest.TestCase):
    def test_step_is_noop_when_idle(self):
        runner, recorder = make_runner()
        runner.step(robot_heading_deg=0)
        self.assertEqual(recorder.velocity_calls, [])

    def test_turning_right_speeds_up_left_wheel_slows_right(self):
        # Target is clockwise (right) of current heading - matches
        # geometry.differential_drive's sign convention (see its own
        # docstring): turning right means left wheel speeds up, right
        # wheel slows down/reverses.
        runner, recorder = make_runner()
        runner.start(target_heading_deg=90, tolerance_deg=1, robot_heading_deg=0)
        runner.step(robot_heading_deg=0)
        left, right = recorder.velocity_calls[0]
        self.assertGreater(left, 0)
        self.assertLess(right, left)

    def test_shortest_way_round_for_a_heading_near_the_0_360_wrap(self):
        # Current heading 350, target 10 - shortest turn is +20 (right),
        # not the long way round through 180.
        runner, recorder = make_runner()
        runner.start(target_heading_deg=10, tolerance_deg=1, robot_heading_deg=350)
        runner.step(robot_heading_deg=350)
        left, right = recorder.velocity_calls[0]
        self.assertGreater(left, right)  # turning right

    def test_finishes_once_within_tolerance(self):
        runner, recorder = make_runner()
        runner.start(target_heading_deg=90, tolerance_deg=10, robot_heading_deg=85)
        runner.step(robot_heading_deg=85)
        status = runner.status()
        self.assertEqual(status["state"], "stopped_ok")
        self.assertEqual(recorder.velocity_calls[-1], (0.0, 0.0))

    def test_turn_rate_is_capped(self):
        runner, recorder = make_runner()
        runner.start(target_heading_deg=180, tolerance_deg=1, robot_heading_deg=0)
        runner.step(robot_heading_deg=0)
        left, right = recorder.velocity_calls[0]
        max_mps = CONFIG["turn_max_mps"]
        self.assertLessEqual(abs(left), max_mps + 1e-9)
        self.assertLessEqual(abs(right), max_mps + 1e-9)

    def test_stall_aborts_when_heading_barely_changes_within_the_window(self):
        clock = FakeClock()
        runner, recorder = make_runner(clock)
        runner.start(target_heading_deg=90, tolerance_deg=1, robot_heading_deg=0)
        runner.step(robot_heading_deg=0)
        clock.advance(CONFIG["stall_check_window_s"] + 0.1)
        # Heading hasn't actually moved - stuck (sticky motor, wedged, etc).
        runner.step(robot_heading_deg=0)
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("stuck", status["abort_reason"])
        self.assertEqual(recorder.velocity_calls[-1], (0.0, 0.0))

    def test_stall_does_not_abort_while_genuinely_turning(self):
        clock = FakeClock()
        runner, recorder = make_runner(clock)
        runner.start(target_heading_deg=90, tolerance_deg=1, robot_heading_deg=0)
        runner.step(robot_heading_deg=0)
        clock.advance(CONFIG["stall_check_window_s"] + 0.1)
        runner.step(robot_heading_deg=30)  # comfortably over turn_stall_min_deg
        self.assertEqual(runner.status()["state"], "running")

    def test_step_after_finish_is_noop(self):
        runner, recorder = make_runner()
        runner.start(target_heading_deg=90, tolerance_deg=10, robot_heading_deg=85)
        runner.step(robot_heading_deg=85)
        calls_after_finish = len(recorder.velocity_calls)
        runner.step(robot_heading_deg=85)
        self.assertEqual(len(recorder.velocity_calls), calls_after_finish)


if __name__ == "__main__":
    unittest.main()
