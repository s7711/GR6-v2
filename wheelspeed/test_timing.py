import unittest

from timing import midpoint_time


class TestMidpointTime(unittest.TestCase):
    def test_midpoint_of_two_timestamps(self):
        self.assertAlmostEqual(midpoint_time(10.0, 20.0), 15.0)

    def test_zero_interval(self):
        self.assertAlmostEqual(midpoint_time(5.0, 5.0), 5.0)

    def test_order_does_not_matter_for_the_arithmetic(self):
        # Not expected to happen in practice (current_timestamp should
        # always be >= prev_timestamp), but the arithmetic itself is
        # symmetric — no special-casing needed either way.
        self.assertAlmostEqual(midpoint_time(20.0, 10.0), 15.0)


if __name__ == "__main__":
    unittest.main()
