import unittest

from velocity import forward_velocity


class TestForwardVelocity(unittest.TestCase):
    def test_heading_north_is_pure_vn(self):
        self.assertAlmostEqual(forward_velocity(vn=2.0, ve=0.0, heading_deg=0.0), 2.0)

    def test_heading_east_is_pure_ve(self):
        self.assertAlmostEqual(forward_velocity(vn=0.0, ve=3.0, heading_deg=90.0), 3.0)

    def test_heading_south_is_negative_vn(self):
        self.assertAlmostEqual(forward_velocity(vn=2.0, ve=0.0, heading_deg=180.0), -2.0)

    def test_heading_west_is_negative_ve(self):
        self.assertAlmostEqual(forward_velocity(vn=0.0, ve=3.0, heading_deg=270.0), -3.0)

    def test_45_degrees_combines_both(self):
        result = forward_velocity(vn=1.0, ve=1.0, heading_deg=45.0)
        self.assertAlmostEqual(result, 2.0 ** 0.5)


if __name__ == "__main__":
    unittest.main()
