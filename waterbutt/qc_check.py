"""Live comparison for the QC marker check: given the saved "ideal"
reading and a live per-marker debug entry from aruco's /ws/aruco feed,
returns the rotation-aware distance from ideal, or the reason there's
nothing to compare yet. See waterbutt-prd.md's "QC marker" section and
aruco/coords.py's qc_marker_delta_body_frame for why this needs the raw
tvec/rvec, not just the derived body-frame position - a robot facing a
different way than when the ideal was saved must not show up as a false
positional error. target_offset_c (config.yaml's
waterbutt_funnel_offset_c) shifts the point being tracked from the
camera itself to the funnel, rigidly offset behind it on the same
mount - see aruco/coords.py's target_position_in_marker_frame.
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent / "aruco"))
import coords  # noqa: E402


def compare(saved: dict, live_debug_by_id: dict, hpr_cb, target_offset_c=(0.0, 0.0, 0.0)) -> dict:
    if saved is None:
        return {"state": "not_configured"}

    debug = (live_debug_by_id or {}).get(str(saved["marker_id"]))
    if debug is None or "tvec_camera_frame" not in debug:
        return {"state": "not_visible", "marker_id": saved["marker_id"]}

    delta = coords.qc_marker_delta_body_frame(
        saved["rvec_camera_frame"], saved["tvec_camera_frame"],
        debug["rvec_camera_frame"], debug["tvec_camera_frame"],
        hpr_cb, target_offset_c,
    )
    return {
        "state": "ok",
        "marker_id": saved["marker_id"],
        "forward_m": float(delta[0]),
        "right_m": float(delta[1]),
        "down_m": float(delta[2]),
        "distance_m": float((delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2) ** 0.5),
    }
