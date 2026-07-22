import unittest

from control import PathRunner

CONFIG = {
    "entry_max_distance_m": 1.0,
    "entry_max_heading_deg": 45,
    "lookahead_distance_m": 0.4,
    "heading_gain": 2.0,
    "cte_gain": 0.6,
    "localisation_accuracy_limit_m": 1.0,
    "max_heading_correction_deg": 70,
    "wheel_base_m": 0.42,
}

# A short straight path running due north from a fixed lat/lon, generated
# with small enough steps that lat/lon <-> local metres round-trips
# predictably for these tests (see geometry.py's own tests for the
# conversion math itself).
STRAIGHT_NORTH_PATH = [
    {"lat": 52.200000, "lon": -1.500000, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
    {"lat": 52.200090, "lon": -1.500000, "speed_mps": 0.5, "pump": True, "clearance_m": 0.5},
    {"lat": 52.200180, "lon": -1.500000, "speed_mps": 0.6, "pump": True, "clearance_m": 1.5},
]


class Recorder:
    def __init__(self):
        self.velocity_calls = []
        self.pump_calls = []

    def send_velocity(self, left, right):
        self.velocity_calls.append((left, right))

    def send_pump(self, on):
        self.pump_calls.append(on)


def make_runner():
    recorder = Recorder()
    runner = PathRunner(CONFIG, recorder.send_velocity, recorder.send_pump)
    runner.load_path(STRAIGHT_NORTH_PATH)
    return runner, recorder


class TestEntryCheck(unittest.TestCase):
    def test_close_and_aligned_succeeds(self):
        runner, _ = make_runner()
        result = runner.entry_check(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        self.assertEqual(result, {"ok": True, "index": 0})

    def test_far_away_reports_nearest_candidate(self):
        runner, _ = make_runner()
        result = runner.entry_check(robot_lat=52.205, robot_lon=-1.500000, robot_heading_deg=0)
        self.assertFalse(result["ok"])
        self.assertIn("distance_m", result)
        self.assertIn("heading_error_deg", result)

    def test_no_path_loaded(self):
        recorder = Recorder()
        runner = PathRunner(CONFIG, recorder.send_velocity, recorder.send_pump)
        result = runner.entry_check(robot_lat=52.2, robot_lon=-1.5, robot_heading_deg=0)
        self.assertEqual(result, {"ok": False, "reason": "no path loaded"})


class TestStartStop(unittest.TestCase):
    def test_start_succeeds_when_close_and_aligned(self):
        runner, _ = make_runner()
        result = runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        self.assertEqual(result, {"ok": True})
        self.assertEqual(runner.status()["state"], "running")

    def test_start_fails_when_too_far(self):
        runner, _ = make_runner()
        result = runner.start(robot_lat=52.205, robot_lon=-1.500000, robot_heading_deg=0)
        self.assertFalse(result["ok"])
        self.assertEqual(runner.status()["state"], "idle")

    def test_start_fails_when_already_running(self):
        runner, _ = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        result = runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        self.assertEqual(result, {"ok": False, "reason": "already running"})

    def test_stop_sends_zero_velocity_and_sets_idle(self):
        runner, recorder = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.stop()
        self.assertEqual(runner.status()["state"], "idle")
        self.assertEqual(recorder.velocity_calls[-1], (0.0, 0.0))


class TestStep(unittest.TestCase):
    def test_step_is_noop_when_idle(self):
        runner, recorder = make_runner()
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        self.assertEqual(recorder.velocity_calls, [])

    def test_step_while_on_path_sends_forward_velocity(self):
        runner, recorder = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        self.assertEqual(runner.status()["state"], "running")
        self.assertEqual(len(recorder.velocity_calls), 1)
        left, right = recorder.velocity_calls[0]
        # On-path, facing the right way: both wheels close to the target
        # speed, no significant turn.
        self.assertAlmostEqual(left, 0.5, delta=0.05)
        self.assertAlmostEqual(right, 0.5, delta=0.05)

    def test_pump_command_sent_once_on_change_not_every_step(self):
        runner, recorder = make_runner()
        # Start near the first point (pump False) and step forward past
        # the second point (pump True) to cross a pump-state boundary.
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        self.assertEqual(recorder.pump_calls, [False])  # explicit sync on the first step
        runner.step(robot_lat=52.200090, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        runner.step(robot_lat=52.200090, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        self.assertEqual(recorder.pump_calls, [False, True])  # then once more, only when it actually changed

    def test_distance_travelled_accumulates(self):
        runner, _ = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        runner.step(robot_lat=52.200045, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        self.assertGreater(runner.status()["distance_travelled_m"], 0)

    def test_localisation_accuracy_breach_aborts(self):
        runner, recorder = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=5.0)
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("localisation accuracy", status["abort_reason"])
        self.assertEqual(recorder.velocity_calls[-1], (0.0, 0.0))

    def test_cross_track_breach_aborts(self):
        runner, recorder = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        # Well east of the path — first point's clearance is 0.5m.
        with self.assertLogs(level="WARNING") as logs:
            runner.step(robot_lat=52.200000, robot_lon=-1.499900, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("cross-track", status["abort_reason"])
        # An abort must be visible in the journal (journalctl -u
        # robot-navigate), not just the debug log — quicker to check.
        self.assertTrue(any("Aborted" in message for message in logs.output))
        self.assertEqual(recorder.velocity_calls[-1], (0.0, 0.0))

    def test_heading_breach_aborts(self):
        runner, recorder = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=170, horizontal_accuracy_m=0.1)
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("heading error", status["abort_reason"])
        self.assertEqual(recorder.velocity_calls[-1], (0.0, 0.0))

    def test_reaching_the_end_of_path_finishes_cleanly(self):
        runner, recorder = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        # Step from right at the final point — the robot's projection
        # onto the last segment clamps to that segment's far end, which
        # is what "path complete" means (see geometry.py).
        for _ in range(3):
            runner.step(robot_lat=52.200180, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        status = runner.status()
        self.assertEqual(status["state"], "stopped_ok")
        self.assertEqual(recorder.velocity_calls[-1], (0.0, 0.0))

    def test_step_after_abort_is_noop(self):
        runner, recorder = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=170, horizontal_accuracy_m=0.1)
        calls_after_abort = len(recorder.velocity_calls)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        self.assertEqual(len(recorder.velocity_calls), calls_after_abort)


if __name__ == "__main__":
    unittest.main()
