import unittest

from control import MissionRunner

STEPS = [{"path": "A"}, {"path": "B"}, {"path": "C"}]


class NavigateStub:
    def __init__(self):
        self.loaded = []
        self.start_calls = 0
        self.stop_calls = 0
        self.start_result = {"ok": True}
        self._status = {"state": "idle", "abort_reason": None}

    def load_path(self, name):
        self.loaded.append(name)

    def start_path(self):
        self.start_calls += 1
        return self.start_result

    def stop_path(self):
        self.stop_calls += 1

    def navigate_status(self):
        return self._status

    def set_status(self, state, abort_reason=None):
        self._status = {"state": state, "abort_reason": abort_reason}


def make_runner():
    stub = NavigateStub()
    runner = MissionRunner(stub.load_path, stub.start_path, stub.stop_path, stub.navigate_status)
    return runner, stub


class TestGo(unittest.TestCase):
    def test_go_loads_and_starts_the_first_step(self):
        runner, stub = make_runner()
        runner.go("test-mission", STEPS)
        self.assertEqual(stub.loaded, ["A"])
        self.assertEqual(stub.start_calls, 1)
        status = runner.status()
        self.assertEqual(status["state"], "running")
        self.assertEqual(status["current_step_index"], 0)
        self.assertEqual(status["mission_name"], "test-mission")

    def test_go_can_resume_from_a_later_step(self):
        runner, stub = make_runner()
        runner.go("test-mission", STEPS, start_index=1)
        self.assertEqual(stub.loaded, ["B"])
        self.assertEqual(runner.status()["current_step_index"], 1)

    def test_failed_start_aborts_immediately(self):
        runner, stub = make_runner()
        stub.start_result = {"ok": False, "reason": "no segment within entry tolerance"}
        runner.go("test-mission", STEPS)
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("entry tolerance", status["abort_reason"])


class TestTick(unittest.TestCase):
    def test_tick_is_noop_while_navigate_still_running(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        stub.set_status("running")
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 0)
        self.assertEqual(stub.start_calls, 1)

    def test_tick_advances_to_next_step_on_stopped_ok(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        stub.set_status("stopped_ok")
        runner.tick()
        self.assertEqual(runner.status()["current_step_index"], 1)
        self.assertEqual(stub.loaded, ["A", "B"])
        self.assertEqual(stub.start_calls, 2)

    def test_tick_finishes_mission_after_last_step(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        for _ in range(len(STEPS)):
            stub.set_status("stopped_ok")
            runner.tick()
        status = runner.status()
        self.assertEqual(status["state"], "stopped_ok")
        self.assertIsNone(status["current_step_index"])
        self.assertEqual(stub.loaded, ["A", "B", "C"])

    def test_tick_aborts_mission_on_navigate_abort(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        stub.set_status("aborted", "heading error 80.0deg exceeds limit 70.0deg")
        runner.tick()
        status = runner.status()
        self.assertEqual(status["state"], "aborted")
        self.assertIn("heading error", status["abort_reason"])
        self.assertEqual(stub.loaded, ["A"])  # never proceeded to B

    def test_tick_is_noop_once_mission_already_finished(self):
        runner, stub = make_runner()
        runner.go("m", [{"path": "A"}])
        stub.set_status("stopped_ok")
        runner.tick()
        self.assertEqual(runner.status()["state"], "stopped_ok")
        stub.set_status("running")  # something odd happening on navigate's side now
        runner.tick()
        self.assertEqual(runner.status()["state"], "stopped_ok")  # unaffected


class TestStop(unittest.TestCase):
    def test_stop_returns_to_idle_and_stops_navigate(self):
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
        self.assertEqual(stub.loaded, ["A"])  # never loaded B


class TestStepLog(unittest.TestCase):
    def test_step_log_records_a_successful_step(self):
        runner, stub = make_runner()
        runner.go("m", STEPS)
        stub.set_status("stopped_ok")
        runner.tick()
        self.assertEqual(runner.status()["step_log"], [{"index": 0, "path": "A", "outcome": "ok"}])

    def test_step_log_records_a_failed_start(self):
        runner, stub = make_runner()
        stub.start_result = {"ok": False, "reason": "no segment within entry tolerance"}
        runner.go("m", STEPS)
        log = runner.status()["step_log"]
        self.assertEqual(len(log), 1)
        self.assertEqual(log[0]["outcome"], "failed_to_start")


if __name__ == "__main__":
    unittest.main()
