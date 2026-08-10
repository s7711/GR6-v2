"""Save-time check: would navigate's own entry logic accept path B's
start, coming straight off the end of path A? Reuses navigate's own
geometry primitives (find_entry_segment, the same thing /control/start
checks live against the robot's real position) rather than a second
implementation of the same tolerance rules — see jobs-prd.md's
"Path continuity".

Path A's final heading/position stand in for "wherever the robot is"
here, since there's no live robot for this check (it runs at job
save time, not run time). The equirectangular local-frame conversion
is per navigate/geometry.py's own convention: fine at garden scale,
and translating between two paths' different reference points doesn't
introduce a rotation, so a heading computed after conversion is still
the same real-world heading.
"""

import sys
from pathlib import Path


# Appended, not inserted at 0 - this directory (jobs/) must keep
# priority for same-named local modules (control.py, app.py, ...);
# navigate/ is only a fallback for names not found here. Found live
# 2026-07-31: an insert(0, ...) here made a same-process test run of
# multiple modules resolve "test_app"/"control" to navigate/'s own
# files instead of jobs/'s, once this module's import had already
# pushed navigate/ ahead of jobs/ in sys.path.
sys.path.append(str(Path(__file__).resolve().parent.parent / "navigate"))
import geometry  # noqa: E402


def check(
    points_a: list, points_b: list, entry_max_distance_m: float, entry_max_heading_deg: float,
    lookahead_distance_m: float,
) -> dict:
    """{"ok": True} if path B's start would be enterable straight off
    the end of path A, else {"ok": False, "reason": ..., "distance_m":
    ..., "heading_error_deg": ...} (matching navigate's own entry_check
    shape). {"ok": True} trivially if either path is too short to
    define a heading/entry segment — nothing meaningful to check."""
    if len(points_a) < 2 or len(points_b) < 2:
        return {"ok": True}

    ref_lat, ref_lon = geometry.path_reference(points_b)
    path_b_local = geometry.path_to_local(points_b, ref_lat, ref_lon)

    prev_north, prev_east = geometry.to_local(points_a[-2]["lat"], points_a[-2]["lon"], ref_lat, ref_lon)
    last_north, last_east = geometry.to_local(points_a[-1]["lat"], points_a[-1]["lon"], ref_lat, ref_lon)
    approach_heading_deg = geometry.bearing(prev_north, prev_east, last_north, last_east)

    index = geometry.find_entry_segment(
        path_b_local, last_north, last_east, approach_heading_deg,
        entry_max_distance_m, entry_max_heading_deg, lookahead_distance_m,
    )
    if index is not None:
        return {"ok": True}

    best = None
    for i in range(len(path_b_local) - 1):
        a, b = path_b_local[i], path_b_local[i + 1]
        _px, _py, _t, dist = geometry.project_onto_segment(last_north, last_east, a.north, a.east, b.north, b.east)
        if best is None or dist < best[1]:
            segment_heading = geometry.bearing(a.north, a.east, b.north, b.east)
            heading_err = geometry.angle_diff(segment_heading, approach_heading_deg)
            best = (i, dist, heading_err)
    return {
        "ok": False,
        "reason": "no segment within entry tolerance",
        "nearest_index": best[0],
        "distance_m": best[1],
        "heading_error_deg": best[2],
    }
