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
    "stall_check_window_s": 5.0,
    "stall_min_distance_m": 0.10,
    "path_max_speed_mps": None,
    "motor_max_speed_mps": None,
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


class FakeClock:
    """Injected as PathRunner's/TurnRunner's `now` - lets stall-abort
    tests advance time explicitly instead of sleeping in real
    wall-clock time."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_runner(clock=None):
    recorder = Recorder()
    kwargs = {"now": clock} if clock is not None else {}
    runner = PathRunner(CONFIG, recorder.send_velocity, recorder.send_pump, **kwargs)
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


class TestLoadPath(unittest.TestCase):
    def test_refused_while_running(self):
        runner, _ = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        result = runner.load_path(STRAIGHT_NORTH_PATH)
        self.assertEqual(result, {"ok": False, "reason": "another path is already running - stop it first"})
        self.assertEqual(runner.status()["state"], "running")  # untouched, not reset to idle

    def test_allowed_while_idle(self):
        runner, _ = make_runner()
        result = runner.load_path(STRAIGHT_NORTH_PATH)
        self.assertEqual(result, {"ok": True})

    def test_allowed_again_once_stopped(self):
        runner, _ = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.stop()
        result = runner.load_path(STRAIGHT_NORTH_PATH)
        self.assertEqual(result, {"ok": True})

    def test_allowed_once_finished_or_aborted(self):
        runner, _ = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner._abort("test abort")
        result = runner.load_path(STRAIGHT_NORTH_PATH)
        self.assertEqual(result, {"ok": True})


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

    def test_stall_aborts_when_barely_moving_within_the_window(self):
        clock = FakeClock()
        runner, recorder = make_runner(clock)
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        self.assertEqual(runner.status()["state"], "running")
        clock.advance(CONFIG["stall_check_window_s"] + 0.1)
        # Same position - stuck.
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("stuck", status["abort_reason"])
        self.assertEqual(recorder.velocity_calls[-1], (0.0, 0.0))

    def test_stall_does_not_abort_when_making_real_progress(self):
        clock = FakeClock()
        runner, recorder = make_runner(clock)
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        clock.advance(CONFIG["stall_check_window_s"] + 0.1)
        # ~10m further north - comfortably over stall_min_distance_m.
        runner.step(robot_lat=52.200090, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        self.assertEqual(runner.status()["state"], "running")

    def test_stall_window_resets_each_check_not_from_the_original_start(self):
        # Small genuine movement every window should never trip the
        # guard, even though the *cumulative* distance since start()
        # would - the checkpoint must slide forward each time, not stay
        # pinned to the run's original position.
        clock = FakeClock()
        runner, recorder = make_runner(clock)
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        for lat in (52.200005, 52.200010, 52.200015):
            clock.advance(CONFIG["stall_check_window_s"] + 0.1)
            runner.step(robot_lat=lat, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
            self.assertEqual(runner.status()["state"], "running")

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

    def test_motor_max_speed_mps_clamps_both_wheels_preserving_turn_ratio(self):
        recorder = Recorder()
        config = {**CONFIG, "motor_max_speed_mps": 0.2}
        runner = PathRunner(config, recorder.send_velocity, recorder.send_pump)
        runner.load_path(STRAIGHT_NORTH_PATH)
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        left, right = recorder.velocity_calls[0]
        # Uncapped this step would be ~0.5/0.5 (straight, on-path) - both
        # wheels scaled down to the 0.2 cap rather than just clipped, which
        # would otherwise turn the robot by distorting the L/R ratio.
        self.assertLessEqual(max(abs(left), abs(right)), 0.2 + 1e-9)
        self.assertAlmostEqual(left, right, delta=0.01)

    def test_path_max_speed_mps_clamps_the_input_not_the_output(self):
        # Unlike motor_max_speed_mps above, this clamps speed_mps *before*
        # turn_command()/differential_drive() run - so a turn's
        # differential rides on top of the clamped baseline rather than
        # eating into it. Start a little off-heading (but still within
        # entry tolerance) so turn_command() actually returns something
        # nonzero - see navigate-prd.md's "Split into two caps".
        recorder = Recorder()
        config = {**CONFIG, "path_max_speed_mps": 0.2}
        runner = PathRunner(config, recorder.send_velocity, recorder.send_pump)
        runner.load_path(STRAIGHT_NORTH_PATH)
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=20)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=20, horizontal_accuracy_m=0.1)
        left, right = recorder.velocity_calls[0]
        # The turn differential is symmetric around the clamped speed, so
        # the average stays pinned at the cap regardless of how much
        # correction is applied - this is the property motor_max_speed_mps
        # (scaling both wheels down together) does NOT preserve.
        self.assertAlmostEqual((left + right) / 2, 0.2, delta=1e-9)
        # And with no motor_max_speed_mps to catch it, the outer wheel is
        # allowed to ride above the 0.2 cap - the deliberate trade-off.
        self.assertGreater(max(left, right), 0.2)

    def test_pump_command_resent_every_step_not_just_on_change(self):
        # The firmware's WP watchdog turns the pump off if no WP command
        # arrives within 2000ms (drive-prd.md) — found live 2026-07-30
        # that only resending on a state *change* let the pump silently
        # switch itself off mid-run. Every step must resend the current
        # state, same as send_velocity already does.
        runner, recorder = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        self.assertEqual(recorder.pump_calls, [False])
        runner.step(robot_lat=52.200090, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        runner.step(robot_lat=52.200090, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        self.assertEqual(recorder.pump_calls, [False, True, True])

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


class TestPreview(unittest.TestCase):
    def test_updates_cross_track_and_heading_error_while_idle(self):
        # No run started at all — the Run page's live display should
        # still show a real number, not stay blank until Start is
        # pressed (found live 2026-07-30 asking for this).
        runner, _ = make_runner()
        runner.preview(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=90)
        status = runner.status()
        self.assertIn("cross_track_error_m", status)
        self.assertIn("heading_error_deg", status)

    def test_noop_without_a_loaded_path(self):
        recorder = Recorder()
        runner = PathRunner(CONFIG, recorder.send_velocity, recorder.send_pump)
        runner.preview(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        self.assertNotIn("cross_track_error_m", runner.status())

    def test_does_not_override_steps_own_values_while_running(self):
        # step()'s values are authoritative during an actual run — see
        # navigate-prd.md's "Ownership of tolerance/accuracy
        # enforcement" — preview() must not fight them.
        runner, _ = make_runner()
        runner.start(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0)
        runner.step(robot_lat=52.200000, robot_lon=-1.500000, robot_heading_deg=0, horizontal_accuracy_m=0.1)
        from_step = runner.status()["cross_track_error_m"]
        # A wildly different position/heading than the real one above -
        # if preview() ran, this would visibly change the reported value.
        runner.preview(robot_lat=52.205000, robot_lon=-1.505000, robot_heading_deg=180)
        self.assertEqual(runner.status()["cross_track_error_m"], from_step)

    def test_matches_exactly_what_starting_would_immediately_show(self):
        # The actual bug found live 2026-07-30: preview() used to search
        # for whichever segment was geometrically closest overall, which
        # can differ from the entry segment start()+step() actually
        # track on a path that loops back near itself - showing a
        # falsely-reassuring low error right before an immediate real
        # abort. Same position/heading fed to both must now produce
        # identical numbers.
        lat, lon, heading = 52.200030, -1.500000, 2  # near the first point, slightly off both axes
        runner, _ = make_runner()
        runner.preview(robot_lat=lat, robot_lon=lon, robot_heading_deg=heading)
        from_preview = dict(runner.status())

        runner.start(robot_lat=lat, robot_lon=lon, robot_heading_deg=heading)
        runner.step(robot_lat=lat, robot_lon=lon, robot_heading_deg=heading, horizontal_accuracy_m=0.1)
        from_step = runner.status()

        self.assertAlmostEqual(from_preview["cross_track_error_m"], from_step["cross_track_error_m"])
        self.assertAlmostEqual(from_preview["heading_error_deg"], from_step["heading_error_deg"])


if __name__ == "__main__":
    unittest.main()
