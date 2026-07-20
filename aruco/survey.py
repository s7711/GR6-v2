"""Add Marker: survey a new marker's global pose from the vehicle's
current nav fix and a live detection. This is new to GR6-v2 — v1 never
computed a marker's position this way; map.csv was tape-measured/
GPS-surveyed by hand and typed in.

Correction (found via a real field test, marker ~61cm in front of the
camera showing up ~7cm away and in the wrong direction on the Map page):
this module originally assumed nav['Heading']/['Pitch']/['Roll'] were
NCOM's IMU-referenced primary attitude fields, needing an extra rotation
through HPR_ib to reach body frame — the same relationship gad.py's
lever-arm math uses for a position vector, applied to an attitude DCM
instead. That was wrong. oxts-nav/app.py downloads `mobile.vat` ("Vehicle
attitude") onto the xNAV650 — that's OxTS's mechanism for giving the
device its own IMU-to-vehicle alignment once, so its *primary* NCOM
output already reports vehicle/body-frame attitude directly afterward.
So nav['Heading']/['Pitch']/['Roll'] are already C_nb — no further
rotation through HPR_ib needed or wanted here (HPR_ib is still correctly
used in gad.py, since the GAD API specifically wants lever-arm/alignment
in IMU-frame terms regardless of what mobile.vat does to the main output
— that's a different, internal-EKF-facing API, not the same question).
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.geodesy import ned_to_lla  # noqa: E402

import coords  # noqa: E402


def survey_marker(nav, detection, hpr_cb_deg, dxc_b, hpr_ib_deg, debug=None):
    """nav: the 'nav' dict from oxts-nav's nav_feed (needs Lat/Lon/Alt/
    Heading/Pitch/Roll — note Lat/Lon are in radians there, per
    oxts-nav/ncomrx.py's raw NCOM decode, while Heading/Pitch/Roll are
    already in degrees, and already vehicle/body-frame — see module
    docstring). detection: one item from detection.detect_markers()
    (rvec/tvec in the raw camera frame). hpr_ib_deg is unused here (see
    module docstring) but kept as a parameter for a consistent call
    signature with gad.py's send(), which does need it.

    If `debug` is a dict, it's filled in with the intermediate values
    (vehicle attitude used, body-frame and NED displacement) so a caller
    can surface them for hand-verification against a real measurement —
    see aruco/app.py's /ws/aruco "debug" field.

    Returns {lat, lon, alt, heading, pitch, roll} for the marker map —
    id and size are the caller's to fill in from the operator's input,
    not something a detection can tell you (see aruco-prd.md, Add
    Marker workflow)."""
    nav_lat_deg = math.degrees(nav["Lat"])
    nav_lon_deg = math.degrees(nav["Lon"])

    vehicle_dcm_nb = coords.hpr_to_dcm(nav["Heading"], nav["Pitch"], nav["Roll"])

    dxm_b = np.array(dxc_b) + coords.displacement_camera_to_body(detection["tvec"], hpr_cb_deg)
    dxm_n = vehicle_dcm_nb @ dxm_b  # NED displacement: xNAV location -> marker

    lat, lon, alt = ned_to_lla(dxm_n[0], dxm_n[1], dxm_n[2], nav_lat_deg, nav_lon_deg, nav["Alt"])

    marker_dcm_nm = coords.marker_dcm_from_vehicle_attitude(vehicle_dcm_nb, detection["rvec"], hpr_cb_deg)
    heading, pitch, roll = coords.dcm_to_hpr(marker_dcm_nm)

    if debug is not None:
        debug["vehicle_heading_pitch_roll"] = [nav["Heading"], nav["Pitch"], nav["Roll"]]
        debug["tvec_camera_frame"] = list(detection["tvec"])
        debug["marker_size_used"] = detection["size"]
        debug["displacement_body_frame"] = dxm_b.tolist()
        debug["displacement_ned"] = dxm_n.tolist()
        debug["range_m"] = float(np.linalg.norm(dxm_n))

    return {"lat": lat, "lon": lon, "alt": alt, "heading": heading, "pitch": pitch, "roll": roll}
