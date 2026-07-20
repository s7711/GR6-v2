"""Coordinate-frame conventions and the rotation chain relating them.

See aruco-prd.md ("Coordinate frame convention") for the full write-up —
this is the part most worth reading before touching this file. Summary
of the six frames involved:

    n  (nav)             X north, Y east, Z down.
    b  (body/vehicle)    X forward, Y right, Z down.
                         V_n = C_nb(HPR) . V_b,  C_nb = C_Heading . C_Pitch . C_Roll
    C  (raw camera)      X right (behind the camera looking in), Y down,
                         Z along the camera's boresight into the scene.
                         Fixed by OpenCV's own convention.
    c  (HPR-friendly camera)
                         X forward out of the camera, Y right, Z down —
                         chosen only so the camera's mount angle can be
                         written as an ordinary HPR triple (`hpr_cb`).
    M  (raw marker, as cv2.aruco returns it)
                         X right (facing the marker from the front),
                         Y up, Z towards the camera (out of the printed face).
    m  (HPR-friendly marker, as stored in the marker map)
                         X out the back of the marker, Y to the marker's
                         right, Z down — chosen only so a marker's pose
                         can be written as an ordinary HPR triple
                         (heading/pitch/roll in the map file).

Worked check (Hm=Pm=Rm=0): X_M points east while X_m points north — two
different physical directions for "the marker's X axis" under the two
labelling conventions for one physical object. Not a typo.

The full chain (read right-to-left — "a vector's components in the
rightmost frame, expressed in the leftmost frame"):

    C_nb = C_nm . C_mM . C_MC . C_Cc . C_cb

    V_b --(C_cb)--> V_c --(C_Cc)--> V_C --(C_MC)--> V_M --(C_mM)--> V_m --(C_nm)--> V_n

  C_cb - fixed camera-mount HPR (config `camera_extrinsics.hpr_cb`).
         NOT properly bore-sighted — currently [0,0,0], an assumption,
         not a measurement. Likely the dominant error source.
  C_Cc - fixed, definitional, relates the two camera-frame conventions.
  C_MC - from ArUco's own pose estimate: C_MC = (C_CM).T, where
         C_CM = cv2.Rodrigues(rvec)[0] is what estimatePoseSingleMarkers
         actually returns (the one live measurement in the whole chain).
  C_mM - fixed, definitional, relates the two marker-frame conventions.
  C_nm - built from the map file's marker heading/pitch/roll, the same
         way C_nb is built from vehicle HPR.

A note on naming `hpr_cb`: strictly, heading/pitch/roll describe the
nav<->body relationship; a fixed camera mount offset is really a plain
ZYX Euler triple. Keeping the HPR name anyway (see aruco-prd.md) — it
communicates more to a reader than "ZYX" would, so long as this comment
exists: "heading" here has no compass meaning, it's just the Z-axis term
in the same rotation-composition convention, applied to a fixed offset
instead of the vehicle's live attitude.
"""

import math

import cv2
import numpy as np

# C_Cc: raw-OpenCV-camera-frame <- HPR-friendly-camera-frame.
# c is (forward, right, down); C is (right, down, forward) — a fixed
# axis relabelling of the same physical camera, not a measurement.
C_Cc = np.array(
    [
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
    ]
)

# C_mM: HPR-friendly-marker-frame <- raw-ArUco-marker-frame.
# M is (right, up, towards camera / out of the front face);
# m is (out the back, right, down) — a fixed axis relabelling of the
# same physical marker, not a measurement. Derived from, and checked
# against, the worked Hm=Pm=Rm=0 example in the module docstring:
#   X_m (out the back) = -Z_M (out the front)
#   Y_m (right)         =  X_M (right)
#   Z_m (down)          = -Y_M (up)
C_mM = np.array(
    [
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)


def hpr_to_dcm(heading_deg, pitch_deg, roll_deg):
    """C_nb(HPR): V_n = C_nb . V_b. Ported unchanged (in effect — same
    convention, re-derived and checked against it) from GR6-v1's
    ncomrx.RbnHPR. Same formula builds C_nm (marker HPR -> nav) and C_cb
    (camera mount HPR -> body) — it's the identical HPR-to-DCM
    relationship in every case, only the two frames it relates change."""
    h = math.radians(heading_deg)
    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)
    c_heading = np.array([[math.cos(h), -math.sin(h), 0.0], [math.sin(h), math.cos(h), 0.0], [0.0, 0.0, 1.0]])
    c_pitch = np.array([[math.cos(p), 0.0, math.sin(p)], [0.0, 1.0, 0.0], [-math.sin(p), 0.0, math.cos(p)]])
    c_roll = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(r), -math.sin(r)], [0.0, math.sin(r), math.cos(r)]])
    return c_heading @ c_pitch @ c_roll


def dcm_to_hpr(dcm):
    """Inverse of hpr_to_dcm: recover (heading_deg, pitch_deg, roll_deg)
    from a C_nb-style rotation matrix built as C_Heading . C_Pitch . C_Roll."""
    heading = math.atan2(dcm[1, 0], dcm[0, 0])
    pitch = math.asin(-dcm[2, 0])
    roll = math.atan2(dcm[2, 1], dcm[2, 2])
    return math.degrees(heading), math.degrees(pitch), math.degrees(roll)


def rvec_to_C_CM(rvec):
    """C_CM (raw marker frame -> raw camera frame) from ArUco's own
    rvec output, as cv2.Rodrigues defines it: a marker-frame point X_M
    maps to camera-frame X_C via X_C = C_CM @ X_M + tvec."""
    dcm, _ = cv2.Rodrigues(rvec)
    return dcm


def vehicle_dcm_from_marker_detection(marker_dcm_nm, rvec, hpr_cb_deg):
    """GAD case: given a KNOWN marker's C_nm (from its map HPR) and a
    fresh detection's rvec, return the vehicle's C_nb as implied by this
    one observation — the "measurement" a heading-GAD update is built
    from.

        C_nb = C_nm . C_mM . C_MC . C_Cc . C_cb
    """
    c_mc = rvec_to_C_CM(rvec).T  # C_MC = (C_CM).T
    c_cb = hpr_to_dcm(*hpr_cb_deg)
    return marker_dcm_nm @ C_mM @ c_mc @ C_Cc @ c_cb


def marker_dcm_from_vehicle_attitude(vehicle_dcm_nb, rvec, hpr_cb_deg):
    """Add Marker case: given the vehicle's current C_nb (from nav) and a
    fresh detection's rvec, return the surveyed marker's C_nm — solving
    the same chain for C_nm instead of C_nb. All the rotation matrices
    on the right are orthogonal, so "invert the product" is just
    "reverse the order and transpose each one":

        C_nm = C_nb . (C_mM . C_MC . C_Cc . C_cb)^-1
             = C_nb . C_cb^T . C_Cc^T . C_MC^T . C_mM^T
             = C_nb . C_cb^T . C_Cc^T . C_CM . C_mM^T   (C_MC^T = C_CM)
    """
    c_cm = rvec_to_C_CM(rvec)  # C_MC^T = C_CM
    c_cb = hpr_to_dcm(*hpr_cb_deg)
    return vehicle_dcm_nb @ c_cb.T @ C_Cc.T @ c_cm @ C_mM.T


def displacement_camera_to_body(v_C, hpr_cb_deg):
    """A displacement vector measured in the raw camera frame C — e.g.
    ArUco's tvec, camera-to-marker — re-expressed in body-frame
    components: V_b = C_cb^T . C_Cc^T . V_C. Used to build the GAD
    lever-arm vector (see gad.py) and the Add Marker survey position."""
    c_cb = hpr_to_dcm(*hpr_cb_deg)
    return c_cb.T @ C_Cc.T @ v_C
