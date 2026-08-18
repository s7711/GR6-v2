import unittest

from level import LevelEstimate


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_level(clock=None, drain_confirm_s=60):
    return LevelEstimate(drain_confirm_s, now=clock or FakeClock())


class TestStartupDefault(unittest.TestCase):
    def test_starts_believed_full_not_empty(self):
        level = make_level()
        self.assertFalse(level.believed_empty())
        self.assertEqual(level.status()["pump_seconds_since_full"], 0.0)


class TestAccumulation(unittest.TestCase):
    def test_pump_on_then_off_accumulates_elapsed_time(self):
        clock = FakeClock()
        level = make_level(clock)
        level.set_pump_on(True)
        clock.advance(30)
        level.set_pump_on(False)
        self.assertEqual(level.status()["pump_seconds_since_full"], 30.0)
        self.assertFalse(level.believed_empty())

    def test_reaching_drain_confirm_s_marks_believed_empty(self):
        clock = FakeClock()
        level = make_level(clock, drain_confirm_s=60)
        level.set_pump_on(True)
        clock.advance(60)
        level.set_pump_on(False)
        self.assertTrue(level.believed_empty())

    def test_still_running_counts_live_without_needing_pump_off(self):
        clock = FakeClock()
        level = make_level(clock, drain_confirm_s=60)
        level.set_pump_on(True)
        clock.advance(61)
        self.assertTrue(level.believed_empty())

    def test_multiple_separate_runs_accumulate(self):
        clock = FakeClock()
        level = make_level(clock, drain_confirm_s=60)
        level.set_pump_on(True)
        clock.advance(20)
        level.set_pump_on(False)
        clock.advance(500)  # long gap with pump off - shouldn't count
        level.set_pump_on(True)
        clock.advance(20)
        level.set_pump_on(False)
        self.assertEqual(level.status()["pump_seconds_since_full"], 40.0)
        self.assertFalse(level.believed_empty())

    def test_redundant_set_pump_on_calls_dont_double_count(self):
        clock = FakeClock()
        level = make_level(clock)
        level.set_pump_on(True)
        level.set_pump_on(True)  # e.g. two feed ticks in a row while still on
        clock.advance(10)
        level.set_pump_on(False)
        self.assertEqual(level.status()["pump_seconds_since_full"], 10.0)


class TestManualMarks(unittest.TestCase):
    def test_mark_full_resets_accumulator(self):
        clock = FakeClock()
        level = make_level(clock, drain_confirm_s=60)
        level.set_pump_on(True)
        clock.advance(60)
        level.set_pump_on(False)
        self.assertTrue(level.believed_empty())
        level.mark_full()
        self.assertFalse(level.believed_empty())
        self.assertEqual(level.status()["pump_seconds_since_full"], 0.0)

    def test_mark_empty_immediately_satisfies_threshold(self):
        level = make_level(drain_confirm_s=60)
        self.assertFalse(level.believed_empty())
        level.mark_empty()
        self.assertTrue(level.believed_empty())

    def test_mark_full_does_not_lose_an_in_progress_pump_run(self):
        clock = FakeClock()
        level = make_level(clock, drain_confirm_s=60)
        level.set_pump_on(True)
        clock.advance(10)
        level.mark_full()  # e.g. operator corrects mid-drain
        clock.advance(10)
        level.set_pump_on(False)
        # The 10s before mark_full is discarded (accumulator reset to
        # zero), but the pump keeps counting from its own real start -
        # so 20s total elapsed since set_pump_on(True), not 10.
        self.assertEqual(level.status()["pump_seconds_since_full"], 20.0)


class TestFillStartsResetIt(unittest.TestCase):
    def test_on_fill_start_equivalent_mark_full_resets(self):
        level = make_level(drain_confirm_s=60)
        level.mark_empty()
        self.assertTrue(level.believed_empty())
        level.mark_full()  # app.py calls this when a fill actually starts
        self.assertFalse(level.believed_empty())


if __name__ == "__main__":
    unittest.main()
