"""Tests for the bore-sight forward model.

The important ones here aren't "does the arithmetic run" — they're the
round-trips that pin this module as the exact *inverse* of the shipping
detection path (coords.py / survey.py). If the forward model and the
service disagree about corner ordering or a frame convention, the solve
will still converge happily to a confidently wrong hpr_cb, so the
convention is pinned against cv2.aruco itself and against coords.py
rather than against my own comments.
"""

import math
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import coords  # noqa: E402
import model  # noqa: E402

# Real values from shared/camera-cal.yaml (1280x960 Pi camera) — using the
# live intrinsics rather than a made-up pinhole keeps the pixel scales in
# these tests representative of what the solver will actually see.
CAMERA_MATRIX = np.array([
    [1166.0314860010476, 0.0, 633.90600534139935],
    [0.0, 1168.2026009458311, 526.17034280566315],
    [0.0, 0.0, 1.0],
])
DIST_COEFFS = np.array([
    0.091577794655201383, -0.41358268148786193,
    0.00087352266508141057, 7.2164044943823303e-05, 0.36489243164727858,
])
NO_DIST = np.zeros(5)

MARKER_SIZE = 0.097
DXC_B = np.array([0.0775, 0.002, -0.07])


class HprToDcmBatchTestCase(unittest.TestCase):
    def test_matches_coords_hpr_to_dcm_elementwise(self):
        rng = np.random.default_rng(0)
        hpr = rng.uniform(-180, 180, size=(25, 3))
        hpr[:, 1] = rng.uniform(-80, 80, size=25)  # keep pitch clear of gimbal lock
        batch = model.hpr_to_dcm_batch(hpr)
        for i, (h, p, r) in enumerate(hpr):
            np.testing.assert_allclose(batch[i], coords.hpr_to_dcm(h, p, r), atol=1e-12)

    def test_preserves_leading_shape(self):
        self.assertEqual(model.hpr_to_dcm_batch(np.zeros((7, 3))).shape, (7, 3, 3))
        self.assertEqual(model.hpr_to_dcm_batch(np.zeros(3)).shape, (3, 3))


class CornerOrderTestCase(unittest.TestCase):
    """Pins marker_corners_M against cv2.aruco's own object-point order:
    project synthetic corners, hand them back to estimatePoseSingleMarkers,
    and require the pose it recovers to be the one we started from. A
    corner-order mistake shows up here as a multiple-of-90-degree rotation
    error, which is exactly the failure that would otherwise go unnoticed."""

    def test_projected_corners_round_trip_through_estimate_pose(self):
        # A marker 2.5m in front of the camera, tilted a little so the
        # round-trip isn't testing a degenerate fronto-parallel case.
        rvec_true = np.array([0.12, -0.25, 0.07])
        tvec_true = np.array([0.15, -0.08, 2.5])
        c_cm = coords.rvec_to_C_CM(rvec_true)
        x_C = (marker := model.marker_corners_M(MARKER_SIZE)[0]) @ c_cm.T + tvec_true
        self.assertEqual(marker.shape, (4, 3))

        pixels, _ = cv2.projectPoints(x_C, np.zeros(3), np.zeros(3), CAMERA_MATRIX, NO_DIST)
        pixels = pixels.reshape(1, 4, 2).astype(np.float32)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [pixels], MARKER_SIZE, CAMERA_MATRIX, NO_DIST
        )
        # Tolerances are set by estimatePoseSingleMarkers' own precision,
        # not by anything this module does: detectMarkers hands it float32
        # pixels, so that's the input precision its IPPE solve works from.
        # A corner-order or frame mistake would land here as a
        # multiple-of-90-degree error, nowhere near these tolerances.
        np.testing.assert_allclose(tvecs[0][0], tvec_true, atol=1e-5)
        # Compare as rotation matrices, not raw Rodrigues vectors — the
        # same rotation has several valid rvec spellings.
        np.testing.assert_allclose(
            coords.rvec_to_C_CM(rvecs[0][0]), c_cm, atol=1e-5
        )


class ForwardModelInvertsTheServiceTestCase(unittest.TestCase):
    """The real check: run the forward model to get corner pixels, then
    put those corners through exactly what aruco/app.py does to a live
    detection (estimatePoseSingleMarkers -> coords.*) and require the
    marker pose and position to come back out."""

    def setUp(self):
        self.hpr_cb = (3.5, -1.25, 2.0)
        self.nav_pos = np.array([[0.0, 0.0, 0.0]])
        self.nav_hpr = np.array([[37.0, 1.5, -2.25]])
        self.marker_pos = np.array([[2.1, 1.4, -0.35]])  # NED: 2.1m north, 1.4m east, 0.35m up
        self.marker_hpr = np.array([[-140.0, 4.0, -3.0]])

    def _detect(self):
        pixels = model.project(
            self.nav_pos, self.nav_hpr, self.marker_pos, self.marker_hpr,
            np.array([MARKER_SIZE]), self.hpr_cb, DXC_B, CAMERA_MATRIX, NO_DIST,
        )
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [pixels[0].astype(np.float32).reshape(1, 4, 2)], MARKER_SIZE, CAMERA_MATRIX, NO_DIST
        )
        return rvecs[0][0], tvecs[0][0]

    def test_recovers_the_marker_position_the_way_survey_does(self):
        _rvec, tvec = self._detect()
        # aruco/app.py: body-frame displacement to the marker
        dxm_b = np.array(DXC_B) + coords.displacement_camera_to_body(tvec, self.hpr_cb)
        # survey.py: rotate into NED and add to the vehicle position
        c_nb = coords.hpr_to_dcm(*self.nav_hpr[0])
        recovered = self.nav_pos[0] + c_nb @ dxm_b
        np.testing.assert_allclose(recovered, self.marker_pos[0], atol=1e-6)

    def test_recovers_the_marker_attitude_the_way_survey_does(self):
        rvec, _tvec = self._detect()
        c_nb = coords.hpr_to_dcm(*self.nav_hpr[0])
        c_nm = coords.marker_dcm_from_vehicle_attitude(c_nb, rvec, self.hpr_cb)
        heading, pitch, roll = coords.dcm_to_hpr(c_nm)
        # 1e-3 deg: estimatePoseSingleMarkers' float32-pixel precision
        # again (see CornerOrderTestCase), ~4000x finer than the 0.1 deg
        # this whole exercise is trying to measure.
        np.testing.assert_allclose([heading, pitch, roll], self.marker_hpr[0], atol=1e-3)

    def test_wrong_hpr_cb_moves_the_answer(self):
        """Sanity floor: if a 1-degree hpr_cb error didn't shift the
        recovered marker position, none of this could calibrate anything."""
        _rvec, tvec = self._detect()
        wrong = (self.hpr_cb[0] + 1.0, self.hpr_cb[1], self.hpr_cb[2])
        dxm_b = np.array(DXC_B) + coords.displacement_camera_to_body(tvec, wrong)
        c_nb = coords.hpr_to_dcm(*self.nav_hpr[0])
        recovered = self.nav_pos[0] + c_nb @ dxm_b
        moved = np.linalg.norm(recovered - self.marker_pos[0])
        # ~2.5m range x 1 degree ~= 44mm
        self.assertGreater(moved, 0.02)
        self.assertLess(moved, 0.08)


class ProjectBatchingTestCase(unittest.TestCase):
    def test_batch_matches_one_at_a_time(self):
        rng = np.random.default_rng(3)
        n = 12
        nav_pos = rng.uniform(-4, 4, size=(n, 3)) * [1, 1, 0]
        nav_hpr = np.column_stack([rng.uniform(-180, 180, n), rng.uniform(-3, 3, n), rng.uniform(-3, 3, n)])
        marker_pos = np.tile([1.0, 0.5, -0.4], (n, 1))
        marker_hpr = np.tile([20.0, 2.0, -1.0], (n, 1))
        sizes = np.full(n, MARKER_SIZE)
        hpr_cb = (2.0, 1.0, -0.5)

        batch = model.project(nav_pos, nav_hpr, marker_pos, marker_hpr, sizes, hpr_cb, DXC_B, CAMERA_MATRIX, DIST_COEFFS)
        for i in range(n):
            one = model.project(
                nav_pos[i:i + 1], nav_hpr[i:i + 1], marker_pos[i:i + 1], marker_hpr[i:i + 1],
                sizes[i:i + 1], hpr_cb, DXC_B, CAMERA_MATRIX, DIST_COEFFS,
            )
            np.testing.assert_allclose(batch[i], one[0], atol=1e-9)

    def test_marker_normal_points_out_of_the_printed_face(self):
        # Heading 0 in the m convention: X_m points north out the *back*,
        # so the face looks south.
        normal = model.marker_normal_n(np.array([[0.0, 0.0, 0.0]]))
        np.testing.assert_allclose(normal[0], [-1.0, 0.0, 0.0], atol=1e-12)
        normal = model.marker_normal_n(np.array([[90.0, 0.0, 0.0]]))
        np.testing.assert_allclose(normal[0], [0.0, -1.0, 0.0], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
