import unittest

from gnss_hold import CLEAR, REJECTING, SETTLING, GnssHold


def make():
    return GnssHold(threshold=12, settle_s=30.0)


class TestGnssHold(unittest.TestCase):
    def test_below_threshold_never_starts(self):
        hold = make()
        for t, reject in enumerate([0, 3, 8, 11, 4, 0]):
            self.assertIsNone(hold.update(reject, float(t)))
        self.assertFalse(hold.active)

    def test_reaching_threshold_starts_an_event(self):
        hold = make()
        self.assertIsNone(hold.update(11, 0.0))
        self.assertEqual(hold.update(12, 1.0), "started")
        self.assertTrue(hold.active)
        self.assertEqual(hold.state, REJECTING)
        self.assertEqual(hold.started_at, 1.0)

    def test_settle_timer_starts_only_when_reject_returns_to_zero(self):
        hold = make()
        hold.update(15, 0.0)
        hold.update(20, 10.0)
        hold.update(4, 40.0)   # still rejecting, however long it's been
        self.assertEqual(hold.state, REJECTING)
        hold.update(0, 41.0)
        self.assertEqual(hold.state, SETTLING)
        self.assertIsNone(hold.update(0, 70.9))
        self.assertTrue(hold.active)
        self.assertEqual(hold.update(0, 71.0), "ended")
        self.assertEqual(hold.state, CLEAR)
        self.assertFalse(hold.active)

    def test_small_rejects_while_settling_do_not_restart_it(self):
        hold = make()
        hold.update(12, 0.0)
        hold.update(0, 1.0)
        hold.update(5, 10.0)
        hold.update(0, 11.0)
        self.assertEqual(hold.update(0, 31.0), "ended")

    def test_a_relapse_while_settling_waits_for_the_next_reset(self):
        hold = make()
        hold.update(12, 0.0)
        hold.update(0, 1.0)
        hold.update(14, 20.0)  # the u-blox refixes, the INS rejects again
        self.assertEqual(hold.state, REJECTING)
        self.assertEqual(hold.relapses, 1)
        self.assertIsNone(hold.update(0, 31.0))  # 30s after the FIRST reset - not settled
        self.assertEqual(hold.state, SETTLING)
        self.assertIsNone(hold.update(0, 60.9))
        self.assertEqual(hold.update(0, 61.0), "ended")
        self.assertEqual(hold.peak_reject, 14)
        self.assertEqual(hold.started_at, 0.0)  # one event throughout

    def test_missing_reject_leaves_state_unchanged(self):
        hold = make()
        self.assertIsNone(hold.update(None, 0.0))
        self.assertFalse(hold.active)
        hold.update(12, 1.0)
        hold.update(None, 2.0)
        self.assertEqual(hold.state, REJECTING)

    def test_settle_remaining_and_duration(self):
        hold = make()
        self.assertIsNone(hold.duration_s(5.0))
        hold.update(12, 0.0)
        self.assertIsNone(hold.settle_remaining_s(1.0))
        hold.update(0, 2.0)
        self.assertAlmostEqual(hold.settle_remaining_s(12.0), 20.0)
        self.assertAlmostEqual(hold.duration_s(12.0), 12.0)

    def test_a_new_event_after_one_ends_starts_fresh(self):
        hold = make()
        hold.update(20, 0.0)
        hold.update(14, 1.0)
        hold.update(0, 2.0)
        hold.update(0, 32.0)
        self.assertEqual(hold.update(12, 100.0), "started")
        self.assertEqual(hold.peak_reject, 12)
        self.assertEqual(hold.relapses, 0)
        self.assertEqual(hold.started_at, 100.0)


if __name__ == "__main__":
    unittest.main()
