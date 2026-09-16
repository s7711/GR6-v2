"""Forward projection model for camera bore-sight estimation: given a
vehicle pose, a marker's pose, and a candidate `hpr_cb`, predict where
the marker's four corners land in the image.

This is the whole basis of the bore-sight solve. Rather than build
residuals out of ArUco's own rvec/tvec, everything is done in *pixels* —
the solve minimises corner reprojection error. That matters because rvec
and tvec are wildly different in quality: the bearing to a marker corner
is good to ~0.05 px / ~0.01 deg (camera-cal.yaml's RMS reprojection error
is 0.18 px), whereas a small planar marker's out-of-plane rotation is the
classic ill-conditioned case and can be degrees out. Reprojection error
weights all of that correctly and automatically; hand-picked rvec/tvec
residual weights would not.

Frames and the rotation chain are aruco/coords.py's, not a second
convention - see its module docstring (n/b/C/c/M/m). This module only
adds the two things coords.py has no reason to provide:

  * the *forward* direction (pose -> pixels); coords.py is written for
    the inverse problem (a detection -> a pose), and
  * batched evaluation over thousands of observations at once, which the
    solver needs and a per-detection service loop does not.

hpr_to_dcm_batch is checked against coords.hpr_to_dcm element-by-element
in test_model.py - coords.py stays the authority on the convention, this
is only a faster way to evaluate the same thing.
"""

import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import coords  # noqa: E402


def hpr_to_dcm_batch(hpr_deg):
    """(..., 3) heading/pitch/roll in degrees -> (..., 3, 3) DCMs.

    Identical convention to coords.hpr_to_dcm: C = Rz(h) . Ry(p) . Rx(r),
    which is scipy's intrinsic "ZYX" with the angles supplied in that
    same order.
    """
    hpr = np.asarray(hpr_deg, dtype=float)
    flat = hpr.reshape(-1, 3)
    dcms = Rotation.from_euler("ZYX", flat, degrees=True).as_matrix()
    return dcms.reshape(hpr.shape[:-1] + (3, 3))


def marker_corners_M(size):
    """The four marker corners in the raw ArUco marker frame M (X right,
    Y up, Z out of the printed face), in the same order
    cv2.aruco.estimatePoseSingleMarkers uses for its object points and
    detectMarkers returns for its image points: top-left, top-right,
    bottom-right, bottom-left as seen face-on.

    Order matters — a mismatch here doesn't fail loudly, it quietly
    rotates every marker by a multiple of 90 degrees. test_model.py pins
    it by round-tripping through estimatePoseSingleMarkers rather than
    trusting this comment.
    """
    h = np.asarray(size, dtype=float) / 2.0
    signs = np.array([[-1.0, 1.0], [1.0, 1.0], [1.0, -1.0], [-1.0, -1.0]])
    h = np.atleast_1d(h)[..., None, None]
    out = np.zeros(h.shape[:-2] + (4, 3))
    out[..., :, :2] = signs * h
    return out


def corners_in_camera_frame(nav_pos_n, nav_hpr, marker_pos_n, marker_hpr, marker_size, hpr_cb, dxc_b):
    """Marker corners expressed in the raw camera frame C, batched over
    observations. All array arguments are (N, ...) with N observations;
    hpr_cb and dxc_b are single (3,) values shared by every observation —
    they're the calibration constants being solved for / held.

    Returns (N, 4, 3).

    The chain, following coords.py:
        X_m = C_mM . X_M                      (definitional relabel)
        X_n = marker_pos + C_nm . X_m         (marker pose)
        X_b = C_nb^T . (X_n - nav_pos)        (vehicle pose; nav frame -> body)
        X_b_cam = X_b - dxc_b                 (body origin -> camera origin)
        X_C = C_Cc . C_cb . X_b_cam           (inverse of coords.displacement_camera_to_body)
    """
    nav_pos_n = np.asarray(nav_pos_n, dtype=float)
    marker_pos_n = np.asarray(marker_pos_n, dtype=float)

    c_nb = hpr_to_dcm_batch(nav_hpr)  # (N, 3, 3)
    c_nm = hpr_to_dcm_batch(marker_hpr)  # (N, 3, 3)
    c_cb = coords.hpr_to_dcm(*hpr_cb)  # (3, 3)

    x_M = marker_corners_M(marker_size)  # (N, 4, 3)
    x_m = x_M @ coords.C_mM.T  # (N, 4, 3); (C_mM . v) for each row v
    x_n = marker_pos_n[:, None, :] + x_m @ np.swapaxes(c_nm, -1, -2)
    x_b = (x_n - nav_pos_n[:, None, :]) @ c_nb  # C_nb^T . v  ==  v . C_nb, per row
    x_b_cam = x_b - np.asarray(dxc_b, dtype=float)
    return x_b_cam @ (coords.C_Cc @ c_cb).T


def project(nav_pos_n, nav_hpr, marker_pos_n, marker_hpr, marker_size, hpr_cb, dxc_b, camera_matrix, dist_coeffs):
    """Predicted corner pixels, (N, 4, 2).

    The camera-frame transform above is done in numpy for every
    observation at once, so cv2.projectPoints only has to apply the
    intrinsics + distortion — one call with an identity pose over all
    4N stacked points, rather than N calls.
    """
    x_C = corners_in_camera_frame(
        nav_pos_n, nav_hpr, marker_pos_n, marker_hpr, marker_size, hpr_cb, dxc_b
    )
    flat = x_C.reshape(-1, 3)
    zero = np.zeros(3)
    pixels, _ = cv2.projectPoints(flat, zero, zero, camera_matrix, dist_coeffs)
    return pixels.reshape(x_C.shape[:-1] + (2,))


def marker_normal_n(marker_hpr):
    """Unit vector out of the *front* (printed) face of each marker, in
    the nav frame — (N, 3). In the m convention X points out the *back*,
    so the front face is -X_m. Used for the obliquity visibility gate:
    ArUco stops detecting reliably once the viewing direction gets too
    far off this normal.
    """
    c_nm = hpr_to_dcm_batch(marker_hpr)
    return -c_nm[..., :, 0]
