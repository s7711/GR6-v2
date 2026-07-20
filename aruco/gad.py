"""Sends known-marker detections to the xNAV650 as GAD (Generic Aiding
Data) position + heading updates. Ported from GR6-v1's gad_aruco.py, with
the "orientation"/"orientation2"/"orientation3" variants dropped — those
were roll/pitch attempts, explicitly out of scope (see aruco-prd.md, GAD
scope: position + heading, not roll/pitch — non-white ArUco innovations
corrupt the accelerometer bias estimate, and there's no equivalent payoff
for roll/pitch to justify that risk the way there is for heading drift).

Key GAD design point carried forward from v1 (not obvious from the
xNAV650 docs on first read): a position update does NOT require computing
the vehicle's own position. It reports the *marker's* known geodetic
position, plus a lever-arm vector (IMU -> marker, in the IMU frame) at
the moment of detection — the device's own filter combines those with
its current attitude estimate to correct vehicle position. No lat/lon
maths needed here (see shared/geodesy.py instead for the Add Marker
survey direction, where a new marker's position *is* computed from the
vehicle's).
"""

import logging

import numpy as np
import oxts_sdk

import coords

# Stream IDs, unchanged from GR6-v1 — arbitrary OxTS aiding-stream identifiers.
POSITION_STREAM_ID = 129
HEADING_STREAM_ID = 131

POSITION_VAR = [0.001, 0.001, 0.001, 0.0, 0.0, 0.0]  # matches v1 (comment there says ~1cm; note: using 3 terms did not appear to work)
HEADING_VAR = 100.0  # ~10 degrees; fine since the update is correlated and frequent (v1 comment)
HEADING_ALIGNMENT_VAR = [5.0, 5.0, 5.0]  # "not very aligned" (v1 comment) — HPR_ib itself isn't precisely known either


def _lever_arm_var(dxm_i):
    """Empirical formula from v1: at 1m lever arm, ~3cm accuracy is
    achievable; 2m ~10cm; 3m ~30cm; 4m ~1m. Not a real accuracy model,
    just what was observed to work — see aruco-prd.md's "per-marker
    accuracy estimates" future-enhancement note."""
    d = np.clip(np.linalg.norm(dxm_i, 2), 1.0, 3.0)
    a = 0.00001 * 30**d
    return [a, a, a]


class GadSender:
    def __init__(self, xnav_ip):
        self._handler = oxts_sdk.GadHandler()
        self._handler.set_encoder_to_bin()
        self._handler.set_output_mode_to_udp(xnav_ip)

    def send(self, detection, marker_record, gps_week, gps_seconds, hpr_cb_deg, dxc_b, hpr_ib_deg, send_heading=True):
        """detection: one item from detection.detect_markers() (rvec/tvec
        in the raw camera frame). marker_record: the matching entry from
        marker_map.py (must be the marker this detection's id resolves
        to — caller's job to look that up)."""
        dxm_b = np.array(dxc_b) + coords.displacement_camera_to_body(detection["tvec"], hpr_cb_deg)
        c_ib = coords.hpr_to_dcm(*hpr_ib_deg)
        dxm_i = c_ib @ dxm_b

        try:
            gp = oxts_sdk.GadPosition(POSITION_STREAM_ID)
            gp.pos_geodetic = [marker_record["lat"], marker_record["lon"], marker_record["alt"]]
            gp.pos_geodetic_var = POSITION_VAR
            gp.time_gps = [gps_week, gps_seconds]
            gp.aiding_lever_arm_fixed = dxm_i.tolist()
            gp.aiding_lever_arm_var = _lever_arm_var(dxm_i)
            self._handler.send_packet(gp)

            if send_heading:
                marker_dcm_nm = coords.hpr_to_dcm(
                    marker_record["heading"], marker_record["pitch"], marker_record["roll"]
                )
                vehicle_dcm_nb = coords.vehicle_dcm_from_marker_detection(marker_dcm_nm, detection["rvec"], hpr_cb_deg)
                vehicle_heading, _pitch, _roll = coords.dcm_to_hpr(vehicle_dcm_nb)

                gh = oxts_sdk.GadHeading(HEADING_STREAM_ID)
                gh.heading = vehicle_heading
                gh.heading_var = HEADING_VAR
                gh.time_gps = [gps_week, gps_seconds]
                gh.aiding_alignment_fixed = list(hpr_ib_deg)
                gh.aiding_alignment_var = HEADING_ALIGNMENT_VAR
                self._handler.send_packet(gh)
        except Exception:
            logging.exception("[gad] Failed to send GAD update for marker %s", marker_record.get("id"))
