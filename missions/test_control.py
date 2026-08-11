import unittest

from control import RUN_JOB_STATUS_GRACE_S, MissionRunner

STEPS = [{"job": "A"}, {"job": "B"}, {"job": "C"}]


class JobsStub:
    def __init__(self):
        self.started = []
        self.stop_calls = 0
        self.start_result = {"ok": True}
        self._status = {"state": "idle", "abort_reason": None}

    def start_job(self, name):
        self.started.append(name)
        return self.start_result

    def stop_job(self):
        self.stop_calls += 1

    def job_status(self):
        return self._status

    def set_status(self, state, abort_reason=None):
        self._status = {"state": state, "abort_reason": abort_reason}


class FakeClock:
    """Injected as MissionRunner's `now` — lets grace-window tests
    advance time explicitly instead of sleeping in real wall-clock
    time."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_runner(clock=None):
    stub = JobsStub()
    kwargs = {"now": clock} if clock is not None else {}
    runner = MissionRunner(stub.start_job, stub.stop_job, stub.job_status, **kwargs)
    return runner, stub


class TestGo(unittest.TestCase):
    def test_go_starts_the_first_step(self):
        runner, stub = make_runner()
        runner.go("test-mission", STEPS)
        self.assertEqual(stub.started, ["A"])
        status = runner.status()
        self.assertEqual(status["state"], "running")
        self.assertEqual(status["current_step_index"], 0)
        self.assertEqual(status["mission_name"], "test-mission")

    def test_go_can_resume_from_a_later_step(self):
        runner, stub = make_runner()
        runner.go("test-mission", STEPS, start_index=1)
        self.assertEqual(stub.started, ["B"])
        self.assertEqual(runner.status()["current_step_index"], 1)

    def test_failed_start_aborts_immediately(self):
        runner, stub = make_runner()
        stub.start_result = {"ok": False, "reason": "mission has no steps"}
        runner.go("test-mission", STEPS)
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("no steps", status["abort_reason"])


class TestTick(unittest.TestCase):
    def test_tick_is_noop_while_job_still_running(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        stub.set_status("running")
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 0)
        self.assertEqual(stub.started, ["A"])

    def test_tick_advances_to_next_step_on_stopped_ok(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        stub.set_status("running")
        runner.tick()
        stub.set_status("stopped_ok")
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 1)
        self.assertEqual(stub.started, ["A", "B"])

    def test_tick_finishes_mission_after_last_step(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        for _ in range(len(STEPS)):
            stub.set_status("running")
            runner.tick()
            stub.set_status("stopped_ok")
            runner.tick()
        status = runner.status()
        self.assertEqual(status["state"], "stopped_ok")
        self.assertIsNone(status["current_step_index"])
        self.assertEqual(stub.started, ["A", "B", "C"])

    def test_tick_aborts_mission_on_job_abort(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        stub.set_status("running")
        runner.tick()
        stub.set_status("aborted", "no segment within entry tolerance")
        runner.tick()
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("entry tolerance", status["abort_reason"])
        self.assertEqual(stub.started, ["A"])  # never proceeded to B

    def test_tick_is_noop_once_mission_already_finished(self):
        runner, stub = make_runner()
        runner.go("m", [{"job": "A"}])
        stub.set_status("running")
        runner.tick()
        stub.set_status("stopped_ok")
        runner.tick()
        self.assertEqual(runner.status()["state"], "stopped_ok")
        stub.set_status("running")  # something odd happening on jobs' side now
        runner.tick()
        self.assertEqual(runner.status()["state"], "stopped_ok")  # unaffected


class TestRunJobRace(unittest.TestCase):
    """Unlike navigate's own feed (see jobs/control.py's
    RUN_PATH_FEED_GRACE_S), jobs has no push feed of its own -
    job_status() is a plain synchronous poll, always reflecting jobs'
    true current state. But a related race still exists: go()/
    _start_current_step() sets self.state = "running" *before* the
    blocking start_job() HTTP call returns, so the background tick
    loop's own thread could call tick() -> job_status() in that window
    and see whatever jobs' state happened to be *before* jobs had
    processed the start call - e.g. the previous mission run's
    terminal state. Guarded the same way, and for the same underlying
    reason, as JobRunner's own fix."""

    def test_stale_terminal_status_right_after_start_does_not_advance(self):
        runner, stub = make_runner()
        stub.set_status("stopped_ok")  # leftover from a previous run of this same job
        runner.go("m", STEPS)
        runner.tick()  # lands before jobs has actually processed the start call
        self.assertEqual(runner.status()["current_step_index"], 0)
        self.assertEqual(stub.started, ["A"])  # never proceeded to B

    def test_advances_once_jobs_actually_reports_running_then_finishes(self):
        runner, stub = make_runner()
        stub.set_status("stopped_ok")  # stale, as above
        runner.go("m", STEPS)
        runner.tick()
        stub.set_status("running")  # jobs catches up to the real state
        runner.tick()
        stub.set_status("stopped_ok")  # genuine completion this time
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 1)

    def test_tick_does_not_crash_if_it_lands_while_start_job_is_still_in_flight(self):
        # Reentrantly calls tick() from inside start_job() itself, same
        # as a concurrent thread landing mid-flight would - must not
        # crash on a not-yet-stamped _run_job_started_at (see
        # jobs/test_control.py's identical regression test for the
        # exact live crash this class of bug caused there).
        runner, stub = make_runner()
        real_start_job = runner.start_job

        def start_job_that_races_a_concurrent_tick(name):
            runner.tick()  # must not crash - _run_job_started_at may not be stamped yet
            return real_start_job(name)

        runner.start_job = start_job_that_races_a_concurrent_tick
        runner.go("m", STEPS)  # must not raise
        self.assertEqual(runner.status()["current_step_index"], 0)

    def test_trusts_terminal_status_once_grace_period_elapses_even_without_seeing_running(self):
        # Safety net for a real (not stale) near-instant completion
        # that a concurrent poll happened to never catch as "running" -
        # this must not wait forever.
        clock = FakeClock()
        runner, stub = make_runner(clock)
        stub.set_status("stopped_ok")
        runner.go("m", STEPS)
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 0)  # still within the grace window
        clock.advance(RUN_JOB_STATUS_GRACE_S + 0.1)
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 1)  # now trusted


class TestStop(unittest.TestCase):
    def test_stop_returns_to_idle_and_stops_jobs(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        runner.stop()
        self.assertEqual(runner.status()["state"], "idle")
        self.assertEqual(stub.stop_calls, 1)

    def test_stop_prevents_a_pending_ticks_stale_success_from_advancing(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        runner.stop()
        stub.set_status("stopped_ok")  # a stale success for the step that was just stopped
        runner.tick()
        self.assertEqual(runner.status()["state"], "idle")
        self.assertEqual(stub.started, ["A"])  # never started B


class TestStepLog(unittest.TestCase):
    def test_step_log_records_a_successful_step(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        stub.set_status("running")
        runner.tick()
        stub.set_status("stopped_ok")
        runner.tick()
        self.assertEqual(
            runner.status()["step_log"],
            [{"index": 0, "type": "run_job", "job": "A", "outcome": "ok"}],
        )

    def test_step_log_records_a_failed_start(self):
        runner, stub = make_runner()
        stub.start_result = {"ok": False, "reason": "no segment within entry tolerance"}
        runner.go("m", STEPS)
        log = runner.status()["step_log"]
        self.assertEqual(len(log), 1)
        self.assertEqual(log[0]["outcome"], "failed_to_start")


if __name__ == "__main__":
    unittest.main()
