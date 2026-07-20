"""Regression tests for coords.py — the safety net promised in
aruco-prd.md against a silent sign-flip/transpose error in the rotation
chain, independent of how carefully the code was first written.

Run with: python -m unittest aruco.test_coords -v  (from the repo root)
"""

import math
import random
import unittest

import numpy as np

import coords


def assert_rotation_matrix(test, m, msg=""):
    test.assertTrue(np.allclose(m @ m.T, np.eye(3), atol=1e-9), f"{msg}: not orthogonal\n{m}")
    test.assertAlmostEqual(np.linalg.det(m), 1.0, places=9, msg=f"{msg}: det != 1")


class TestFixedMatrices(unittest.TestCase):
    def test_C_Cc_is_a_valid_rotation(self):
        assert_rotation_matrix(self, coords.C_Cc, "C_Cc")

    def test_C_mM_is_a_valid_rotation(self):
        assert_rotation_matrix(self, coords.C_mM, "C_mM")

    def test_C_Cc_matches_frame_definitions(self):
        # V_C = C_Cc . V_c. c is (forward, right, down); C is (right, down, forward).
        forward_c = np.array([1.0, 0.0, 0.0])
        right_c = np.array([0.0, 1.0, 0.0])
        down_c = np.array([0.0, 0.0, 1.0])
        np.testing.assert_allclose(coords.C_Cc @ forward_c, [0, 0, 1])  # forward -> Z_C (boresight)
        np.testing.assert_allclose(coords.C_Cc @ right_c, [1, 0, 0])  # right -> X_C (right)
        np.testing.assert_allclose(coords.C_Cc @ down_c, [0, 1, 0])  # down -> Y_C (down)

    def test_C_mM_matches_worked_zero_angle_example(self):
        # From the module/PRD docstring, at Hm=Pm=Rm=0:
        #   X_M points east,  X_m points north
        #   Y_M points up,    Y_m points east
        #   Z_M points south, Z_m points down
        # so X_M's direction (east) equals Y_m's direction (east), etc.
        x_M = np.array([1.0, 0.0, 0.0])
        y_M = np.array([0.0, 1.0, 0.0])
        z_M = np.array([0.0, 0.0, 1.0])
        np.testing.assert_allclose(coords.C_mM @ x_M, [0, 1, 0])  # X_M (east) -> Y_m (east)
        np.testing.assert_allclose(coords.C_mM @ y_M, [0, 0, -1])  # Y_M (up) -> -Z_m (up = -down)
        np.testing.assert_allclose(coords.C_mM @ z_M, [-1, 0, 0])  # Z_M (south) -> -X_m (south = -north)


class TestHprDcm(unittest.TestCase):
    def test_zero_is_identity(self):
        np.testing.assert_allclose(coords.hpr_to_dcm(0, 0, 0), np.eye(3), atol=1e-12)

    def test_heading_90_rotates_north_to_east(self):
        # North (X_n) should map to East (Y_n) at heading=90, matching compass sense.
        dcm = coords.hpr_to_dcm(90, 0, 0)
        np.testing.assert_allclose(dcm @ [1, 0, 0], [0, 1, 0], atol=1e-9)

    def test_round_trip_random_angles(self):
        random.seed(0)
        for _ in range(200):
            h = random.uniform(-180, 180)
            p = random.uniform(-89, 89)  # avoid gimbal lock at +-90
            r = random.uniform(-180, 180)
            dcm = coords.hpr_to_dcm(h, p, r)
            assert_rotation_matrix(self, dcm, f"hpr_to_dcm({h},{p},{r})")
            h2, p2, r2 = coords.dcm_to_hpr(dcm)
            self.assertAlmostEqual(h, h2, places=6)
            self.assertAlmostEqual(p, p2, places=6)
            self.assertAlmostEqual(r, r2, places=6)


class TestRvecConversion(unittest.TestCase):
    def test_zero_rvec_is_identity(self):
        np.testing.assert_allclose(coords.rvec_to_C_CM(np.zeros(3)), np.eye(3), atol=1e-12)

    def test_straight_on_marker_is_180_about_x(self):
        # Camera looking straight at an untilted marker: right stays right,
        # but the camera's forward (into the marker) is opposite the
        # marker's "towards camera" axis, and down is opposite up.
        # That's C_CM = diag(1, -1, -1), i.e. rvec = (pi, 0, 0).
        rvec = np.array([math.pi, 0.0, 0.0])
        np.testing.assert_allclose(coords.rvec_to_C_CM(rvec), np.diag([1.0, -1.0, -1.0]), atol=1e-9)


class TestChainRoundTrip(unittest.TestCase):
    """The real safety net: if either composed function has a wrong
    transpose or wrong multiplication order, this fails, regardless of
    whether the underlying physical interpretation is "right" — it
    checks that marker_dcm_from_vehicle_attitude and
    vehicle_dcm_from_marker_detection are true inverses of each other
    for the same single observation, as the algebra in coords.py's
    docstring requires."""

    def test_round_trip_random_cases(self):
        random.seed(1)
        for _ in range(200):
            vehicle_hpr = (random.uniform(-180, 180), random.uniform(-89, 89), random.uniform(-89, 89))
            hpr_cb = (random.uniform(-180, 180), random.uniform(-89, 89), random.uniform(-89, 89))
            rvec = np.random.default_rng().normal(size=3)
            rvec = rvec / np.linalg.norm(rvec) * random.uniform(0, math.pi)

            vehicle_dcm = coords.hpr_to_dcm(*vehicle_hpr)
            marker_dcm = coords.marker_dcm_from_vehicle_attitude(vehicle_dcm, rvec, hpr_cb)
            assert_rotation_matrix(self, marker_dcm, "marker_dcm_from_vehicle_attitude")

            recovered_vehicle_dcm = coords.vehicle_dcm_from_marker_detection(marker_dcm, rvec, hpr_cb)
            np.testing.assert_allclose(recovered_vehicle_dcm, vehicle_dcm, atol=1e-9)

    def test_displacement_camera_to_body_straight_ahead(self):
        # Zero mount offset: camera frame IS body frame (relabelled).
        # A displacement straight down the camera's boresight (Z_C) is,
        # in body-frame terms, straight ahead (X_b).
        v_C = np.array([0.0, 0.0, 2.5])  # 2.5m along the boresight
        v_b = coords.displacement_camera_to_body(v_C, (0.0, 0.0, 0.0))
        np.testing.assert_allclose(v_b, [2.5, 0.0, 0.0], atol=1e-9)

    def test_zero_case_end_to_end(self):
        # Fully zeroed inputs: vehicle facing north/level, no mount offset,
        # marker facing north/level, camera looking straight at it.
        vehicle_dcm = np.eye(3)
        hpr_cb = (0.0, 0.0, 0.0)
        rvec = np.array([math.pi, 0.0, 0.0])  # straight-on

        marker_dcm = coords.marker_dcm_from_vehicle_attitude(vehicle_dcm, rvec, hpr_cb)
        recovered_vehicle_dcm = coords.vehicle_dcm_from_marker_detection(marker_dcm, rvec, hpr_cb)
        np.testing.assert_allclose(recovered_vehicle_dcm, vehicle_dcm, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
