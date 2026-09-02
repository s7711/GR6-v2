import unittest

from control import NAVIGATE_STEP_FEED_GRACE_S, JobRunner

STEPS = [{"path": "A"}, {"path": "B"}, {"path": "C"}]


class NavigateStub:
    def __init__(self):
        self.loaded = []
        self.start_calls = 0
        self.stop_calls = 0
        self.turn_calls = []  # [(heading_deg, tolerance_deg), ...]
        self.load_result = {"ok": True}
        self.start_result = {"ok": True}
        self.turn_result = {"ok": True}
        self._status = {"state": "idle", "abort_reason": None}

    def load_path(self, name):
        self.loaded.append(name)
        return self.load_result

    def start_path(self):
        self.start_calls += 1
        return self.start_result

    def start_turn(self, heading_deg, tolerance_deg):
        self.turn_calls.append((heading_deg, tolerance_deg))
        return self.turn_result

    def stop_path(self):
        self.stop_calls += 1

    def navigate_status(self):
        return self._status

    def set_status(self, state, abort_reason=None):
        self._status = {"state": state, "abort_reason": abort_reason}


class HardwareStub:
    """Fakes the pump (via navigate's /pump/manual) and the water butt's
    valve (waterbutt's own /go, /stop) - the two pieces of hardware the
    `water`/`fill` step types drive outside of any path."""

    def __init__(self):
        self.pump_calls = []       # [True, False, True, ...] in call order
        self.waterbutt_go_calls = []
        self.waterbutt_stop_calls = 0
        self.pump_result = {"ok": True}
        self.waterbutt_go_result = {"ok": True}

    def pump_on(self, on):
        self.pump_calls.append(on)
        return self.pump_result

    def waterbutt_go(self, duration_s):
        self.waterbutt_go_calls.append(duration_s)
        return self.waterbutt_go_result

    def waterbutt_stop(self):
        self.waterbutt_stop_calls += 1


class FakeClock:
    """Injected as JobRunner's `now` - lets timed-step tests advance
    time explicitly instead of sleeping in real wall-clock time."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_runner(clock=None):
    stub = NavigateStub()
    hw = HardwareStub()
    kwargs = {"now": clock} if clock is not None else {}
    runner = JobRunner(
        stub.load_path, stub.start_path, stub.stop_path, stub.start_turn, stub.navigate_status,
        hw.pump_on, hw.waterbutt_go, hw.waterbutt_stop, **kwargs,
    )
    return runner, stub, hw


class TestGo(unittest.TestCase):
    def test_go_loads_and_starts_the_first_step(self):
        runner, stub, hw = make_runner()
        runner.go("test-job", STEPS)
        self.assertEqual(stub.loaded, ["A"])
        self.assertEqual(stub.start_calls, 1)
        status = runner.status()
        self.assertEqual(status["state"], "running")
        self.assertEqual(status["current_step_index"], 0)
        self.assertEqual(status["job_name"], "test-job")

    def test_go_can_resume_from_a_later_step(self):
        runner, stub, hw = make_runner()
        runner.go("test-job", STEPS, start_index=1)
        self.assertEqual(stub.loaded, ["B"])
        self.assertEqual(runner.status()["current_step_index"], 1)

    def test_failed_start_aborts_immediately(self):
        runner, stub, hw = make_runner()
        stub.start_result = {"ok": False, "reason": "no segment within entry tolerance"}
        runner.go("test-job", STEPS)
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("entry tolerance", status["abort_reason"])

    def test_failed_load_aborts_immediately_without_calling_start(self):
        # e.g. navigate refusing because a path is already running -
        # see navigate/control.py's load_path() guard.
        runner, stub, hw = make_runner()
        stub.load_result = {"ok": False, "reason": "another path is already running - stop it first"}
        runner.go("test-job", STEPS)
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("already running", status["abort_reason"])
        self.assertEqual(stub.start_calls, 0)


class TestTurnToHeadingStep(unittest.TestCase):
    # turn_to_heading polls navigate exactly like run_path does (shared
    # _tick_navigate_step - see control.py) - the grace-period/race
    # tests already cover that machinery via run_path, this just checks
    # the turn-specific bits: starting it, its own failure/step-log
    # shape, and that it defaults tolerance_deg the same way navigate's
    # own /control/turn does.
    def test_go_starts_a_turn_with_its_own_heading_and_tolerance(self):
        runner, stub, hw = make_runner()
        runner.go("test-job", [{"type": "turn_to_heading", "heading_deg": 245, "tolerance_deg": 10}])
        self.assertEqual(stub.turn_calls, [(245, 10)])
        self.assertEqual(runner.status()["state"], "running")

    def test_go_omits_tolerance_when_the_step_does_not_specify_one(self):
        runner, stub, hw = make_runner()
        runner.go("test-job", [{"type": "turn_to_heading", "heading_deg": 245}])
        self.assertEqual(stub.turn_calls, [(245, None)])

    def test_failed_turn_start_aborts_immediately(self):
        runner, stub, hw = make_runner()
        stub.turn_result = {"ok": False, "reason": "a path is already running - stop it first"}
        runner.go("test-job", [{"type": "turn_to_heading", "heading_deg": 245}])
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("already running", status["abort_reason"])

    def test_advances_once_navigate_reports_stopped_ok(self):
        runner, stub, hw = make_runner()
        runner.go("test-job", [
            {"type": "turn_to_heading", "heading_deg": 245, "tolerance_deg": 10},
            {"path": "A"},
        ])
        stub.set_status("running")  # navigate's feed catches up to the step actually running
        runner.tick()
        stub.set_status("stopped_ok")
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 1)
        self.assertEqual(stub.loaded, ["A"])

    def test_navigate_abort_mid_turn_aborts_the_job(self):
        runner, stub, hw = make_runner()
        runner.go("test-job", [{"type": "turn_to_heading", "heading_deg": 245}])
        stub.set_status("running")
        runner.tick()
        stub.set_status("aborted", abort_reason="turned only 0.0deg in 5.0s (limit 5.0deg) - stuck?")
        runner.tick()
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("stuck", status["abort_reason"])

    def test_stop_during_turn_stops_navigate(self):
        # No pump/waterbutt equivalent for a turn - stop_path() alone
        # covers it, since navigate's own /control/stop stops whichever
        # of run_path/turn_to_heading is actually in progress.
        runner, stub, hw = make_runner()
        runner.go("test-job", [{"type": "turn_to_heading", "heading_deg": 245}])
        runner.stop()
        self.assertEqual(stub.stop_calls, 1)
        self.assertEqual(runner.status()["state"], "idle")


class TestTick(unittest.TestCase):
    def test_tick_is_noop_while_navigate_still_running(self):
        runner, stub, hw = make_runner()
        runner.go("m", STEPS)
        stub.set_status("running")
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 0)
        self.assertEqual(stub.start_calls, 1)

    def test_tick_advances_to_next_step_on_stopped_ok(self):
        runner, stub, hw = make_runner()
        runner.go("m", STEPS)
        stub.set_status("running")  # navigate's feed catches up to the step actually running
        runner.tick()
        stub.set_status("stopped_ok")
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 1)
        self.assertEqual(stub.loaded, ["A", "B"])
        self.assertEqual(stub.start_calls, 2)

    def test_tick_finishes_job_after_last_step(self):
        runner, stub, hw = make_runner()
        runner.go("m", STEPS)
        for _ in range(len(STEPS)):
            stub.set_status("running")
            runner.tick()
            stub.set_status("stopped_ok")
            runner.tick()
        status = runner.status()
        self.assertEqual(status["state"], "stopped_ok")
        self.assertIsNone(status["current_step_index"])
        self.assertEqual(stub.loaded, ["A", "B", "C"])

    def test_tick_aborts_job_on_navigate_abort(self):
        runner, stub, hw = make_runner()
        runner.go("m", STEPS)
        stub.set_status("running")
        runner.tick()
        stub.set_status("aborted", "heading error 80.0deg exceeds limit 70.0deg")
        runner.tick()
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("heading error", status["abort_reason"])
        self.assertEqual(stub.loaded, ["A"])  # never proceeded to B

    def test_tick_is_noop_once_job_already_finished(self):
        runner, stub, hw = make_runner()
        runner.go("m", [{"path": "A"}])
        stub.set_status("running")
        runner.tick()
        stub.set_status("stopped_ok")
        runner.tick()
        self.assertEqual(runner.status()["state"], "stopped_ok")
        stub.set_status("running")  # something odd happening on navigate's side now
        runner.tick()
        self.assertEqual(runner.status()["state"], "stopped_ok")  # unaffected


class TestRunPathFeedRace(unittest.TestCase):
    """navigate's own feed (navigate/feed.py) pushes on a fixed timer,
    independent of when its internal state actually changes - so right
    after start_path() returns, navigate_status() can still report the
    *previous* run's terminal state for up to one push period. Seen
    live 2026-08-09: a job read that stale "stopped_ok" immediately
    after starting, treated it as the just-started step already having
    finished, and moved on to loading the next step while the first
    path was actually still running - which navigate correctly refused."""

    def test_stale_terminal_status_right_after_start_does_not_advance(self):
        runner, stub, hw = make_runner()
        stub.set_status("stopped_ok")  # leftover from a previous run of this same path
        runner.go("m", STEPS)
        runner.tick()  # lands before navigate's feed has caught up
        self.assertEqual(runner.status()["current_step_index"], 0)
        self.assertEqual(stub.loaded, ["A"])  # never proceeded to B

    def test_advances_once_navigate_actually_reports_running_then_finishes(self):
        runner, stub, hw = make_runner()
        stub.set_status("stopped_ok")  # stale, as above
        runner.go("m", STEPS)
        runner.tick()
        stub.set_status("running")  # feed catches up to the real state
        runner.tick()
        stub.set_status("stopped_ok")  # genuine completion this time
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 1)

    def test_tick_does_not_crash_if_it_lands_while_load_path_is_still_in_flight(self):
        # go()/_advance() set state to "running" before _start_run_path_step
        # makes its (blocking, real network I/O in production) load_path/
        # start_path calls - so the background tick loop can genuinely
        # call tick() while a step is still starting, before its
        # started-at timestamp used to get stamped. Simulated here by
        # having load_path itself trigger a reentrant tick() call, same
        # as a concurrent thread landing mid-flight would.
        runner, stub, hw = make_runner()
        real_load_path = runner.load_path

        def load_path_that_races_a_concurrent_tick(name):
            runner.tick()  # must not crash - _run_path_started_at may not be stamped yet
            return real_load_path(name)

        runner.load_path = load_path_that_races_a_concurrent_tick
        runner.go("m", STEPS)  # must not raise
        self.assertEqual(runner.status()["current_step_index"], 0)

    def test_trusts_terminal_status_once_grace_period_elapses_even_without_seeing_running(self):
        # Safety net for a real (not stale) near-instant completion that
        # the feed's push timer happened to never report as "running" -
        # this must not wait forever.
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        stub.set_status("stopped_ok")
        runner.go("m", STEPS)
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 0)  # still within the grace window
        clock.advance(NAVIGATE_STEP_FEED_GRACE_S + 0.1)
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 1)  # now trusted


class TestStop(unittest.TestCase):
    def test_stop_returns_to_idle_and_stops_navigate(self):
        runner, stub, hw = make_runner()
        runner.go("m", STEPS)
        runner.stop()
        self.assertEqual(runner.status()["state"], "idle")
        self.assertEqual(stub.stop_calls, 1)

    def test_stop_prevents_a_pending_ticks_stale_success_from_advancing(self):
        runner, stub, hw = make_runner()
        runner.go("m", STEPS)
        runner.stop()
        stub.set_status("stopped_ok")  # a stale success for the step that was just stopped
        runner.tick()
        self.assertEqual(runner.status()["state"], "idle")
        self.assertEqual(stub.loaded, ["A"])  # never loaded B


class TestStepLog(unittest.TestCase):
    def test_step_log_records_a_successful_step(self):
        runner, stub, hw = make_runner()
        runner.go("m", STEPS)
        stub.set_status("running")
        runner.tick()
        stub.set_status("stopped_ok")
        runner.tick()
        self.assertEqual(
            runner.status()["step_log"],
            [{"index": 0, "type": "run_path", "path": "A", "outcome": "ok"}],
        )

    def test_step_log_records_a_failed_start(self):
        runner, stub, hw = make_runner()
        stub.start_result = {"ok": False, "reason": "no segment within entry tolerance"}
        runner.go("m", STEPS)
        log = runner.status()["step_log"]
        self.assertEqual(len(log), 1)
        self.assertEqual(log[0]["outcome"], "failed_to_start")

    def test_step_log_records_a_failed_load(self):
        runner, stub, hw = make_runner()
        stub.load_result = {"ok": False, "reason": "another path is already running - stop it first"}
        runner.go("m", STEPS)
        log = runner.status()["step_log"]
        self.assertEqual(len(log), 1)
        self.assertEqual(log[0]["outcome"], "failed_to_load")


class TestPauseStep(unittest.TestCase):
    def test_pause_advances_only_once_duration_elapses(self):
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        runner.go("m", [{"type": "pause", "duration_s": 30}, {"type": "run_path", "path": "A"}])
        clock.advance(29)
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 0)  # not yet
        clock.advance(2)
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 1)
        self.assertEqual(stub.loaded, ["A"])

    def test_pause_never_touches_pump_or_waterbutt(self):
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        runner.go("m", [{"type": "pause", "duration_s": 5}])
        clock.advance(5)
        runner.tick()
        self.assertEqual(hw.pump_calls, [])
        self.assertEqual(hw.waterbutt_go_calls, [])


class TestWaterStep(unittest.TestCase):
    def test_water_turns_pump_on_immediately(self):
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        runner.go("m", [{"type": "water", "duration_s": 60}])
        self.assertEqual(hw.pump_calls, [True])

    def test_water_resends_pump_on_each_tick_within_a_phase(self):
        # Drive's own firmware watchdog turns the pump off after 2000ms
        # of silence - the pump command must be resent well within that,
        # not just fired once at a phase's start. Stays inside the first
        # priming phase's 2s (see WATER_PRIME_PHASES) - phase transitions
        # are covered by test_priming_sequence_runs_before_the_real_duration.
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        runner.go("m", [{"type": "water", "duration_s": 60}])
        clock.advance(0.5)
        runner.tick()
        clock.advance(0.5)
        runner.tick()
        self.assertEqual(hw.pump_calls, [True, True, True])

    def test_priming_sequence_runs_before_the_real_duration(self):
        # Found live 2026-09-01: the pump always has air in it (emptied
        # between uses), which makes the first couple of seconds of real
        # watering land almost randomly - an on/off/on/off priming cycle
        # clears it. See WATER_PRIME_PHASES.
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        runner.go("m", [{"type": "water", "duration_s": 60}, {"type": "pause", "duration_s": 1}])

        # go() above already fired the first phase's pump_on(True) - each
        # of these 4 ticks crosses one more phase boundary: the 3
        # remaining priming phases (off/on/off), then into the real
        # 60s duration (on).
        for on in (False, True, False, True):
            clock.advance(2)
            runner.tick()
        self.assertEqual(hw.pump_calls, [True, False, True, False, True])
        self.assertEqual(runner.status()["current_step_index"], 0)  # still on the water step - now in the real duration, not priming

        clock.advance(60)
        runner.tick()
        self.assertEqual(hw.pump_calls[-1], False)
        self.assertEqual(runner.status()["current_step_index"], 1)

    def test_water_step_fails_if_pump_cannot_be_started(self):
        # e.g. a path is currently running - see navigate's /pump/manual guard.
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        hw.pump_result = {"ok": False, "reason": "a path is already running"}
        runner.go("m", [{"type": "water", "duration_s": 60}])
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("already running", status["abort_reason"])


class TestFillStep(unittest.TestCase):
    def test_fill_starts_waterbutt_with_the_step_duration(self):
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        runner.go("m", [{"type": "fill", "duration_s": 20}])
        self.assertEqual(hw.waterbutt_go_calls, [20])

    def test_fill_advances_without_an_explicit_stop_call(self):
        # waterbutt's own valve controller self-terminates at the same
        # duration - no explicit stop needed on the success path.
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        runner.go("m", [{"type": "fill", "duration_s": 20}, {"type": "pause", "duration_s": 1}])
        clock.advance(20)
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 1)
        self.assertEqual(hw.waterbutt_stop_calls, 0)

    def test_fill_step_fails_if_waterbutt_cannot_be_started(self):
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        hw.waterbutt_go_result = {"ok": False, "reason": "couldn't reach waterbutt"}
        runner.go("m", [{"type": "fill", "duration_s": 20}])
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("waterbutt", status["abort_reason"])

    def test_fill_step_qc_refusal_skips_without_water_rather_than_aborting(self):
        # 2026-08-15: keep going without water rather than abort the
        # whole job (and, via missions, the whole round) over one
        # obscured QC marker.
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        hw.waterbutt_go_result = {"ok": False, "reason": "QC marker 7 not visible", "refused": True}
        runner.go("m", [{"type": "fill", "duration_s": 20}, {"type": "pause", "duration_s": 1}])
        status = runner.status()
        self.assertEqual(status["state"], "running")
        self.assertEqual(status["current_step_index"], 1)
        self.assertEqual(status["step_log"][-1]["outcome"], "skipped_no_water")
        self.assertEqual(status["step_log"][-1]["reason"], "QC marker 7 not visible")

    def test_fill_step_qc_refusal_on_last_step_finishes_the_job_ok(self):
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        hw.waterbutt_go_result = {"ok": False, "reason": "QC marker 7 not visible", "refused": True}
        runner.go("m", [{"type": "fill", "duration_s": 20}])
        status = runner.status()
        self.assertEqual(status["state"], "stopped_ok")


class TestStopDuringTimedStep(unittest.TestCase):
    def test_stop_during_water_turns_pump_off(self):
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        runner.go("m", [{"type": "water", "duration_s": 60}])
        runner.stop()
        self.assertEqual(hw.pump_calls, [True, False])
        self.assertEqual(runner.status()["state"], "idle")

    def test_stop_during_fill_stops_the_waterbutt(self):
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        runner.go("m", [{"type": "fill", "duration_s": 60}])
        runner.stop()
        self.assertEqual(hw.waterbutt_stop_calls, 1)

    def test_stop_during_pause_touches_neither(self):
        clock = FakeClock()
        runner, stub, hw = make_runner(clock)
        runner.go("m", [{"type": "pause", "duration_s": 60}])
        runner.stop()
        self.assertEqual(hw.pump_calls, [])
        self.assertEqual(hw.waterbutt_stop_calls, 0)


if __name__ == "__main__":
    unittest.main()
