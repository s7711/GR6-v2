import unittest

from scale_factor_gate import gnss_velocity_trustworthy


class TestGnssVelocityTrustworthy(unittest.TestCase):
    def test_good_status_passes(self):
        status = {"GnssVelReject": 0, "InnVelXFilt": 0.3, "InnVelYFilt": -0.2}
        self.assertTrue(gnss_velocity_trustworthy(status, max_innovation=3.5))

    def test_nonzero_reject_count_fails(self):
        status = {"GnssVelReject": 2, "InnVelXFilt": 0.1, "InnVelYFilt": 0.1}
        self.assertFalse(gnss_velocity_trustworthy(status, max_innovation=3.5))

    def test_large_innovation_fails(self):
        status = {"GnssVelReject": 0, "InnVelXFilt": 5.0, "InnVelYFilt": 0.1}
        self.assertFalse(gnss_velocity_trustworthy(status, max_innovation=3.5))

    def test_innovation_exactly_at_threshold_passes(self):
        status = {"GnssVelReject": 0, "InnVelXFilt": 3.5, "InnVelYFilt": 3.5}
        self.assertTrue(gnss_velocity_trustworthy(status, max_innovation=3.5))

    def test_missing_innovation_field_fails_rather_than_guessing(self):
        status = {"GnssVelReject": 0}
        self.assertFalse(gnss_velocity_trustworthy(status, max_innovation=3.5))

    def test_missing_reject_count_defaults_to_zero(self):
        status = {"InnVelXFilt": 0.1, "InnVelYFilt": 0.1}
        self.assertTrue(gnss_velocity_trustworthy(status, max_innovation=3.5))


if __name__ == "__main__":
    unittest.main()
