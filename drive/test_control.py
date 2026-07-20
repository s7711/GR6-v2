import unittest

from control import AUTO, MANUAL, ControlArbiter


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class TestControlArbiter(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.arbiter = ControlArbiter(hold_seconds=0.5, clock=self.clock)

    def test_auto_accepted_when_no_manual_activity(self):
        self.assertTrue(self.arbiter.try_command(AUTO))
        self.assertEqual(self.arbiter.status()["controller"], AUTO)

    def test_manual_always_accepted(self):
        self.assertTrue(self.arbiter.try_command(MANUAL))
        self.assertEqual(self.arbiter.status()["controller"], MANUAL)

    def test_auto_rejected_during_manual_hold_window(self):
        self.arbiter.try_command(MANUAL)
        self.clock.advance(0.3)  # within the 0.5s hold
        self.assertFalse(self.arbiter.try_command(AUTO))
        self.assertEqual(self.arbiter.status()["controller"], MANUAL)

    def test_auto_accepted_once_hold_expires(self):
        self.arbiter.try_command(MANUAL)
        self.clock.advance(0.6)  # past the 0.5s hold
        self.assertTrue(self.arbiter.try_command(AUTO))
        self.assertEqual(self.arbiter.status()["controller"], AUTO)

    def test_repeated_manual_commands_keep_extending_the_hold(self):
        self.arbiter.try_command(MANUAL)
        self.clock.advance(0.3)
        self.arbiter.try_command(MANUAL)  # e.g. the jog page's 300ms repeat
        self.clock.advance(0.3)  # 0.6s since the first, but only 0.3s since the second
        self.assertFalse(self.arbiter.try_command(AUTO))

    def test_status_reports_no_lock_until_when_unlocked(self):
        self.arbiter.try_command(MANUAL)
        self.clock.advance(0.6)
        status = self.arbiter.status()
        self.assertIsNone(status["manual_lock_until"])

    def test_status_before_any_command(self):
        status = self.arbiter.status()
        self.assertEqual(status["controller"], "none")
        self.assertIsNone(status["manual_lock_until"])


if __name__ == "__main__":
    unittest.main()
