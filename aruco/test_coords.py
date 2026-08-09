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


class TestCameraPositionInMarkerFrame(unittest.TestCase):
    def test_recovers_the_true_camera_position(self):
        # Forward model: a point at the camera's own location, expressed
        # in marker-frame terms, must map to the camera origin (X_C=0)
        # under X_C = C_CM . X_M + tvec - so tvec = -C_CM @ true_position_m.
        rng = np.random.default_rng(2)
        for _ in range(50):
            true_position_m = rng.normal(size=3)
            rvec = rng.normal(size=3)
            rvec = rvec / np.linalg.norm(rvec) * random.uniform(0, math.pi)
            c_cm = coords.rvec_to_C_CM(rvec)
            tvec = -c_cm @ true_position_m
            recovered = coords.camera_position_in_marker_frame(rvec, tvec)
            np.testing.assert_allclose(recovered, true_position_m, atol=1e-9)

    def test_pure_rotation_at_the_same_position_gives_the_same_result(self):
        # The property the QC marker check depends on: two readings of
        # the same fixed marker from the same physical camera position
        # but a different orientation must agree here - even though the
        # raw tvec for the two readings is quite different.
        true_position_m = np.array([0.5, -0.2, 1.0])
        rvec_a = np.array([0.1, 0.2, 0.05])
        rvec_b = np.array([0.4, -0.3, 0.2])
        tvec_a = -coords.rvec_to_C_CM(rvec_a) @ true_position_m
        tvec_b = -coords.rvec_to_C_CM(rvec_b) @ true_position_m
        self.assertFalse(np.allclose(tvec_a, tvec_b))  # sanity: the raw readings really do differ
        np.testing.assert_allclose(
            coords.camera_position_in_marker_frame(rvec_a, tvec_a),
            coords.camera_position_in_marker_frame(rvec_b, tvec_b),
            atol=1e-9,
        )


class TestQcMarkerDeltaBodyFrame(unittest.TestCase):
    def test_zero_for_identical_readings(self):
        rvec = np.array([0.1, 0.2, 0.05])
        tvec = np.array([0.3, -0.1, 0.9])
        delta = coords.qc_marker_delta_body_frame(rvec, tvec, rvec, tvec, (0.0, 0.0, 0.0))
        np.testing.assert_allclose(delta, [0.0, 0.0, 0.0], atol=1e-9)

    def test_zero_for_a_pure_rotation_even_though_tvec_and_rvec_both_change(self):
        # The actual bug report: Ben moved much closer to the ideal spot,
        # but a rotation-only difference in how the robot was facing
        # still showed up as an error under the old (plain body-frame)
        # comparison. This must come out as ~zero.
        true_position_m = np.array([0.5, -0.2, 1.0])
        rvec_ideal = np.array([0.1, 0.2, 0.05])
        rvec_live = np.array([0.4, -0.3, 0.2])
        tvec_ideal = -coords.rvec_to_C_CM(rvec_ideal) @ true_position_m
        tvec_live = -coords.rvec_to_C_CM(rvec_live) @ true_position_m
        delta = coords.qc_marker_delta_body_frame(
            rvec_ideal, tvec_ideal, rvec_live, tvec_live, (0.0, 0.0, 0.0)
        )
        np.testing.assert_allclose(delta, [0.0, 0.0, 0.0], atol=1e-9)

    def test_zero_offset_matches_the_camera_only_case(self):
        rng = np.random.default_rng(4)
        rvec_ideal, rvec_live = rng.normal(size=3), rng.normal(size=3)
        tvec_ideal, tvec_live = rng.normal(size=3), rng.normal(size=3)
        hpr_cb = (5.0, 0.0, 0.0)
        camera_only = coords.qc_marker_delta_body_frame(rvec_ideal, tvec_ideal, rvec_live, tvec_live, hpr_cb)
        zero_offset = coords.qc_marker_delta_body_frame(
            rvec_ideal, tvec_ideal, rvec_live, tvec_live, hpr_cb, target_offset_c=(0.0, 0.0, 0.0)
        )
        np.testing.assert_allclose(zero_offset, camera_only, atol=1e-12)

    def test_zero_for_a_pure_rotation_with_a_real_offset_behind_the_camera(self):
        # The waterbutt funnel case: a point ~23cm behind the camera on
        # the same rigid mount must ALSO be rotation-invariant, not just
        # the camera itself - the whole point of rotating the offset by
        # each reading's own orientation before differencing.
        true_funnel_position_m = np.array([0.5, -0.2, 1.0])
        funnel_offset_c = (-0.228, 0.0, 0.0)
        rvec_ideal = np.array([0.1, 0.2, 0.05])
        rvec_live = np.array([0.4, -0.3, 0.2])

        def tvec_for(rvec):
            # Place the CAMERA such that the FUNNEL ends up exactly at
            # true_funnel_position_m, for this reading's orientation.
            c_cm = coords.rvec_to_C_CM(rvec)
            offset_C = coords.C_Cc @ np.array(funnel_offset_c)
            offset_M = c_cm.T @ offset_C
            camera_position_m = true_funnel_position_m - offset_M
            return -c_cm @ camera_position_m

        delta = coords.qc_marker_delta_body_frame(
            rvec_ideal, tvec_for(rvec_ideal), rvec_live, tvec_for(rvec_live), (0.0, 0.0, 0.0), funnel_offset_c
        )
        np.testing.assert_allclose(delta, [0.0, 0.0, 0.0], atol=1e-9)

    def test_matches_a_manual_composition_of_the_underlying_primitives(self):
        # Checks qc_marker_delta_body_frame is wired up as documented -
        # a faithful composition of the (separately tested) primitives
        # it's built from, for a real camera-mount offset this time.
        rng = np.random.default_rng(3)
        rvec_ideal = rng.normal(size=3)
        rvec_live = rng.normal(size=3)
        tvec_ideal = rng.normal(size=3)
        tvec_live = rng.normal(size=3)
        hpr_cb = (12.0, -3.0, 1.5)

        delta_m = coords.camera_position_in_marker_frame(
            rvec_live, tvec_live
        ) - coords.camera_position_in_marker_frame(rvec_ideal, tvec_ideal)
        expected = coords.displacement_camera_to_body(coords.rvec_to_C_CM(rvec_ideal) @ delta_m, hpr_cb)

        actual = coords.qc_marker_delta_body_frame(rvec_ideal, tvec_ideal, rvec_live, tvec_live, hpr_cb)
        np.testing.assert_allclose(actual, expected, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
