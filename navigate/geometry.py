"""Pure-pursuit path-following geometry: local-frame conversion, path
entry, lookahead-point/cross-track/heading-error computation, and
differential-drive conversion. See navigate-prd.md ("Path-following
control logic", "Path entry", "Path storage") for the GR6-v1 prior art
this reproduces, and what's deliberately changed.

Conventions:
- Local positions are (north, east) metres, from shared/geodesy.py's
  lla_to_ned, using a path's own first point as the reference (see
  path_reference/path_to_local) — never a persisted origin.
- Headings/bearings are compass degrees, 0-360, 0=North, 90=East,
  clockwise — matching oxts-nav's own Heading field, so no separate
  math-vs-compass convention needs to be tracked or converted.
"""

import math
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.geodesy import lla_to_ned, ned_to_lla  # noqa: E402


@dataclass
class PathPoint:
    north: float
    east: float
    speed_mps: float
    pump: bool
    clearance_m: float


@dataclass
class LookaheadResult:
    north: float
    east: float
    speed_mps: float
    pump: bool
    proj_north: float
    proj_east: float
    cross_track_error_m: float
    tracked_index: int
    path_complete: bool


def to_local(lat, lon, ref_lat, ref_lon):
    """(north, east) metres of (lat, lon) relative to (ref_lat, ref_lon) —
    a thin, altitude-free wrapper around shared/geodesy.py's lla_to_ned."""
    north, east, _down = lla_to_ned(lat, lon, 0.0, ref_lat, ref_lon, 0.0)
    return north, east


def project_forward(lat, lon, heading_deg, distance_m):
    """(lat, lon) of the point distance_m ahead of (lat, lon) along
    heading_deg (compass bearing) — used by the create-path page's "move
    forward" helper to build a synthetic mini-path (see navigate-prd.md).
    The inverse of `bearing`, via shared/geodesy.py's ned_to_lla."""
    heading_rad = math.radians(heading_deg)
    north = distance_m * math.cos(heading_rad)
    east = distance_m * math.sin(heading_rad)
    target_lat, target_lon, _alt = ned_to_lla(north, east, 0.0, lat, lon, 0.0)
    return target_lat, target_lon


def path_reference(points: list) -> tuple:
    """The lat/lon reference used for a path's local-frame conversion —
    always its own first point (see navigate-prd.md's "Path storage")."""
    return points[0]["lat"], points[0]["lon"]


def path_to_local(points: list, ref_lat: float, ref_lon: float) -> list:
    """Convert a path's stored point dicts (lat/lon/speed_mps/pump/
    clearance_m, as loaded from a path YAML file) into local-frame
    PathPoints, all relative to the same fixed reference for a run."""
    local = []
    for p in points:
        north, east = to_local(p["lat"], p["lon"], ref_lat, ref_lon)
        local.append(PathPoint(north, east, p["speed_mps"], p["pump"], p["clearance_m"]))
    return local


def bearing(from_north, from_east, to_north, to_east) -> float:
    """Compass bearing (degrees, 0-360) from one local point to another."""
    d_north = to_north - from_north
    d_east = to_east - from_east
    return math.degrees(math.atan2(d_east, d_north)) % 360


def angle_diff(a_deg, b_deg) -> float:
    """Smallest signed difference a-b, wrapped to (-180, 180]."""
    return (a_deg - b_deg + 180) % 360 - 180


def heading_error_deg(robot_heading_deg, from_north, from_east, to_north, to_east) -> float:
    """Signed heading error (degrees) between the robot's current
    heading and the bearing from (from_north, from_east) to
    (to_north, to_east)."""
    target_bearing = bearing(from_north, from_east, to_north, to_east)
    return angle_diff(target_bearing, robot_heading_deg)


def project_onto_segment(px, py, ax, ay, bx, by):
    """Project point P onto segment AB (clamped to the segment, t in
    [0,1]). Returns (proj_x, proj_y, t, perpendicular_distance)."""
    ab_x, ab_y = bx - ax, by - ay
    seg_len_sq = ab_x * ab_x + ab_y * ab_y
    if seg_len_sq == 0:
        t = 0.0
    else:
        t = ((px - ax) * ab_x + (py - ay) * ab_y) / seg_len_sq
        t = max(0.0, min(1.0, t))
    proj_x = ax + t * ab_x
    proj_y = ay + t * ab_y
    dist = math.hypot(px - proj_x, py - proj_y)
    return proj_x, proj_y, t, dist


def find_entry_segment(path, robot_north, robot_east, robot_heading_deg,
                        max_distance_m, max_heading_deg):
    """Scan forward through the path from its start; return the index of
    the first segment within max_distance_m of the robot's position AND
    within max_heading_deg of the robot's current heading. None if no
    segment qualifies — the caller must surface this to the operator
    (distance/angle to the nearest candidate), not silently proceed, per
    navigate-prd.md's fix over GR6-v1's silent-failure behaviour."""
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        _px, _py, _t, dist = project_onto_segment(robot_north, robot_east, a.north, a.east, b.north, b.east)
        if dist > max_distance_m:
            continue
        segment_heading = bearing(a.north, a.east, b.north, b.east)
        if abs(angle_diff(robot_heading_deg, segment_heading)) > max_heading_deg:
            continue
        return i
    return None


def find_lookahead_point(path, start_index, robot_north, robot_east, lookahead_distance_m):
    """Starting from start_index, find the closest forward projection of
    the robot's position onto the path (the tracking index only ever
    moves forward, matching GR6-v1), then walk forward by
    lookahead_distance_m along the path from that projection to get the
    actual lookahead target. Returns None once start_index is already at
    the last point (path complete)."""
    if start_index >= len(path) - 1:
        return None

    best_index, best_dist, best_proj, best_t = start_index, None, None, None
    for i in range(start_index, len(path) - 1):
        a, b = path[i], path[i + 1]
        proj_x, proj_y, t, dist = project_onto_segment(robot_north, robot_east, a.north, a.east, b.north, b.east)
        if best_dist is not None and dist > best_dist:
            break  # getting worse — matches GR6-v1's early-stop
        best_index, best_dist, best_proj, best_t = i, dist, (proj_x, proj_y), t
    proj_north, proj_east = best_proj
    # The robot has reached (or passed) the final waypoint once its
    # projection onto the last segment clamps to that segment's far end —
    # this, not a lookahead-distance shortfall, is what "path complete"
    # means (the lookahead walk below can't tell the difference between
    # "near the end" and "plenty of straight path left but short on
    # lookahead", so it isn't used for this).
    path_complete = best_index == len(path) - 2 and best_t >= 0.999999

    a, b = path[best_index], path[best_index + 1]
    seg_north, seg_east = b.north - a.north, b.east - a.east
    to_robot_north, to_robot_east = robot_north - proj_north, robot_east - proj_east
    cross = seg_north * to_robot_east - seg_east * to_robot_north
    seg_len = math.hypot(seg_north, seg_east)
    cte = cross / seg_len if seg_len > 0 else 0.0

    remaining = lookahead_distance_m
    idx = best_index
    cur_north, cur_east = proj_north, proj_east
    last_dir = None
    while remaining > 0 and idx < len(path) - 1:
        a, b = path[idx], path[idx + 1]
        # The segment's own direction (a->b), not (b - cur) - cur can
        # already equal b (clamped there by the projection above), which
        # would make that vector zero even though the segment itself has
        # a perfectly good direction to extrapolate along below.
        full_seg_north, full_seg_east = b.north - a.north, b.east - a.east
        full_seg_len = math.hypot(full_seg_north, full_seg_east)
        if full_seg_len > 0:
            last_dir = (full_seg_north / full_seg_len, full_seg_east / full_seg_len)
        seg_north, seg_east = b.north - cur_north, b.east - cur_east
        seg_len = math.hypot(seg_north, seg_east)
        if seg_len <= 0:
            idx += 1
            continue
        if seg_len >= remaining:
            frac = remaining / seg_len
            cur_north += seg_north * frac
            cur_east += seg_east * frac
            remaining = 0.0
        else:
            remaining -= seg_len
            cur_north, cur_east = b.north, b.east
            idx += 1

    if remaining > 0 and last_dir is not None:
        # Ran out of path before covering the full lookahead distance
        # (near the very end, or on a path shorter than
        # lookahead_distance_m outright) - extend virtually past the
        # last point along its own final direction, rather than leaving
        # the target sitting right on top of wherever the robot now is.
        # Found live 2026-08-08: with the target coincident with the
        # robot, heading_error_deg's bearing calculation becomes a
        # near-zero vector dominated by GNSS/INS noise, not a real
        # steering signal - a stationary robot sitting on a just-
        # finished path showed heading errors swinging wildly (~170+
        # degrees) purely from centimetre-scale position jitter. Mostly
        # visible via preview() (which keeps evaluating a finished path
        # for the Run page's display - see its own docstring) rather
        # than live steering, since step() has already called _finish()
        # by the time a real run reaches this point - but it's a
        # genuine instability in the lookahead maths itself, not just a
        # display quirk, so fixed at the source.
        cur_north += last_dir[0] * remaining
        cur_east += last_dir[1] * remaining

    # Speed/pump: the segment currently being tracked (best_index) governs
    # both — pump in particular is deliberately tied to the tracked point,
    # not the lookahead point, fixing a GR6-v1 quirk (see navigate-prd.md).
    tracked_point = path[best_index]
    return LookaheadResult(
        north=cur_north,
        east=cur_east,
        speed_mps=tracked_point.speed_mps,
        pump=tracked_point.pump,
        proj_north=proj_north,
        proj_east=proj_east,
        cross_track_error_m=cte,
        tracked_index=best_index,
        path_complete=path_complete,
    )


def nearest_segment_offset(path, robot_north, robot_east, robot_heading_deg) -> dict:
    """Signed cross-track error + heading error against whichever
    segment of `path` the robot is currently closest to — searches
    every segment (unlike find_lookahead_point, which only tracks
    forward from a given index), so this stays meaningful with no run
    in progress: before starting, or once one has finished/aborted. For
    the Run page's live "how far off/misaligned am I" display —
    heading error here is against the *segment's own bearing*, not a
    lookahead point ahead on the path (what step()'s heading_error_deg
    uses while actually running) — a different, simpler question ("am
    I roughly aligned with the path here"), not a steering target."""
    best = None
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        proj_north, proj_east, _t, dist = project_onto_segment(robot_north, robot_east, a.north, a.east, b.north, b.east)
        if best is None or dist < best[0]:
            seg_north, seg_east = b.north - a.north, b.east - a.east
            to_robot_north, to_robot_east = robot_north - proj_north, robot_east - proj_east
            cross = seg_north * to_robot_east - seg_east * to_robot_north
            seg_len = math.hypot(seg_north, seg_east)
            cte = cross / seg_len if seg_len > 0 else 0.0
            segment_heading = bearing(a.north, a.east, b.north, b.east)
            heading_err = angle_diff(segment_heading, robot_heading_deg)
            best = (dist, i, cte, heading_err)
    _dist, index, cte, heading_err = best
    return {"index": index, "cross_track_error_m": cte, "heading_error_deg": heading_err}


def differential_drive(forward_mps, turn, wheel_base_m, max_mps=None):
    """Convert a forward speed + turn command into (left_mps, right_mps).
    `turn` is positive when the robot needs to turn right/clockwise
    (matching heading_error_deg's sign convention — see turn_command) —
    turning right means the right (inside) wheel slows down and the left
    (outside) wheel speeds up. (This is the same sign a real jog-stick
    bug turned up in drive's home.html: pushing the stick right must
    increase the left wheel and decrease the right, not the other way
    round — confirmed on real hardware, see navigate-prd.md.)
    If max_mps is given and either wheel would exceed it, both are scaled
    down symmetrically to preserve the commanded turn ratio (matches
    GR6-v1's clamp behaviour)."""
    left = forward_mps + turn * wheel_base_m / 2
    right = forward_mps - turn * wheel_base_m / 2
    if max_mps is not None:
        largest = max(abs(left), abs(right))
        if largest > max_mps and largest > 0:
            scale = max_mps / largest
            left *= scale
            right *= scale
    return left, right


def turn_command(heading_error_deg, cross_track_error_m, forward_mps, heading_gain, cte_gain, lookahead_distance_m):
    """Blend heading error (degrees) and cross-track error (metres) into
    a single turn command (an angular velocity, rad/s — differential_drive
    multiplies it by wheel_base_m/2, not this function). Positive = turn
    right (matches differential_drive's convention). heading_error_deg is
    positive when the target is to the robot's right, so its contribution
    is added directly. cross_track_error_m is positive when the robot is
    east of a path heading north (see find_lookahead_point) — being east
    of the path means it needs to turn LEFT to correct, so this term is
    subtracted, not added. GR6-v1's own equivalent formula added it
    instead — its code comment admitted the cte sign was "not... even
    verified"; this is that verification, done from first principles
    rather than copied unverified (see navigate-prd.md).

    Curvature-based (real pure-pursuit), not a fixed angular rate: the
    heading term is the standard pure-pursuit curvature 2*sin(alpha)/L_d
    (alpha = heading_error_deg, L_d = lookahead_distance_m), and the whole
    curvature (heading term + cte term) is multiplied by forward_mps to
    get an actual angular velocity. This is a deliberate fix over an
    earlier version that computed the angular velocity directly from
    heading_error_deg with no speed term at all — since curvature (path
    bend per metre travelled) is what actually matters for staying on the
    path, and a fixed angular-RATE response to a given heading error gets
    proportionally sharper (more curvature) the slower the robot is going,
    that version oscillated more at low speed, not less — the opposite of
    what you'd want approaching a tight corner. Multiplying by speed here
    (not dividing anything by it) means forward_mps=0 just yields turn=0,
    not a division error — see navigate-prd.md's "Speed-scaled pure
    pursuit" section for the field-testing story behind this.
    Use the *commanded* speed (the tracked point's speed_mps), not a
    measured one — this is a geometry function and shouldn't depend on
    how well the robot is currently hitting its target speed."""
    curvature = heading_gain * 2.0 * math.sin(math.radians(heading_error_deg)) / lookahead_distance_m
    curvature -= cte_gain * cross_track_error_m
    return curvature * forward_mps
