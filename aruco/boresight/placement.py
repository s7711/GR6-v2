"""Where to plant the markers: validate the recorded boundary path is a
box, then work out the panel's position and facing and each marker's
target spot.

Ben can only place a marker approximately, and lat/lon digits are no use
to someone standing in a field — so the output here is meant to be
*guided to*, with the robot itself as the measuring instrument (it has
RTK to a centimetre). The page built on this reports live range and
bearing from wherever the robot is to each target.

How accurate does placement need to be? Barely at all. Every marker's
pose is a free parameter in the solve (see solve.py), so planting error
costs nothing directly. What matters is only that the *geometry* comes
out roughly as designed:

  * panel near the box centre, so the fan/past legs stay inside the
    boundary and within ArUco's ~4m range — a few tens of centimetres is
    plenty,
  * facing roughly along the long axis, within maybe +/-15 deg, so the
    approach sector isn't cut short by viewing obliquity,
  * markers about 0.5m apart, which a tape measure on the rail settles.

So this is guidance, not survey.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from shared.geodesy import lla_to_ned, ned_to_lla  # noqa: E402

# A hand-driven boundary is never square to the millimetre; these are
# "did the operator actually drive a rectangle" checks, not tolerances on
# the calibration. The recorded "Boresight limits" box sits at 86.8-95.3
# deg corners with opposite sides differing by 4.7% and 11%, so these
# leave real headroom while still rejecting a triangle or a figure-eight.
CORNER_TOLERANCE_DEG = 15.0
OPPOSITE_SIDE_TOLERANCE = 0.25  # fractional difference between opposite sides
MIN_SIDE_M = 3.0  # below this there is no room to drive the pattern at all
# How much roomier one facing direction must be before it's preferred
# over the other — see marker_targets().
TIE_BREAK_CLEARANCE_M = 0.5


def _local(points):
    """Path points (dicts with lat/lon) -> (centroid_lat, centroid_lon,
    Nx2 local north/east metres)."""
    lat0 = sum(p["lat"] for p in points) / len(points)
    lon0 = sum(p["lon"] for p in points) / len(points)
    local = np.array([lla_to_ned(p["lat"], p["lon"], 0.0, lat0, lon0, 0.0)[:2] for p in points])
    return lat0, lon0, local


def _bearing(from_ne, to_ne):
    return math.degrees(math.atan2(to_ne[1] - from_ne[1], to_ne[0] - from_ne[0]))


def check_box(points):
    """Is this recorded path a box?

    Returns a dict with `is_box`, a list of human-readable `problems`,
    and the measured geometry either way — the geometry is worth showing
    even on a rejection, since that's what tells the operator *how* it's
    wrong rather than just that it is.

    The path is a bare list of corner points and is NOT closed (Ben
    records four corners and leaves the closing edge implied), so the
    fourth edge is corner 3 -> corner 0.
    """
    problems = []
    if len(points) != 4:
        return {"is_box": False, "problems": [f"expected 4 corners, got {len(points)}"],
                "corners_deg": [], "side_lengths_m": [], "centroid": None,
                "long_axis_bearing_deg": None}

    lat0, lon0, local = _local(points)

    sides = []
    for i in range(4):
        a, b = local[i], local[(i + 1) % 4]
        sides.append(float(math.hypot(b[0] - a[0], b[1] - a[1])))

    corners = []
    for i in range(4):
        prev, cur, nxt = local[(i - 1) % 4], local[i], local[(i + 1) % 4]
        v1, v2 = prev - cur, nxt - cur
        cosang = float(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        corners.append(math.degrees(math.acos(max(-1.0, min(1.0, cosang)))))

    for i, ang in enumerate(corners):
        if abs(ang - 90.0) > CORNER_TOLERANCE_DEG:
            problems.append(f"corner {i} is {ang:.1f} deg, not a right angle")
    for i in (0, 1):
        a, b = sides[i], sides[i + 2]
        if abs(a - b) / max(a, b) > OPPOSITE_SIDE_TOLERANCE:
            problems.append(f"opposite sides {a:.1f}m and {b:.1f}m differ by more than "
                            f"{OPPOSITE_SIDE_TOLERANCE * 100:.0f}%")
    for i, s in enumerate(sides):
        if s < MIN_SIDE_M:
            problems.append(f"side {i} is only {s:.1f}m — too small to drive the pattern in")

    # Long axis: the direction of the longer pair of sides. Averaged over
    # both of them rather than taken from one, so a sloppily-driven
    # corner doesn't tilt the panel's facing.
    pair = (0, 2) if (sides[0] + sides[2]) >= (sides[1] + sides[3]) else (1, 3)
    vectors = []
    for i in pair:
        a, b = local[i], local[(i + 1) % 4]
        v = np.array([b[0] - a[0], b[1] - a[1]])
        if vectors and np.dot(v, vectors[0]) < 0:
            v = -v  # opposite sides run in opposite directions round the loop
        vectors.append(v)
    mean_v = np.mean(vectors, axis=0)
    long_axis = math.degrees(math.atan2(mean_v[1], mean_v[0]))

    return {
        "is_box": not problems,
        "problems": problems,
        "corners_deg": corners,
        "side_lengths_m": sides,
        "centroid": (lat0, lon0),
        "long_axis_bearing_deg": long_axis,
        "local": local,
    }


def _clearance(local, point_ne, bearing_deg):
    """Distance from `point_ne` to the boundary along `bearing_deg`."""
    direction = np.array([math.cos(math.radians(bearing_deg)), math.sin(math.radians(bearing_deg))])
    best = float("inf")
    for i in range(len(local)):
        a, b = local[i], local[(i + 1) % len(local)]
        edge = b - a
        denom = direction[0] * (-edge[1]) + direction[1] * edge[0]
        if abs(denom) < 1e-9:
            continue
        diff = a - point_ne
        t = (diff[0] * (-edge[1]) + diff[1] * edge[0]) / denom
        u = (diff[0] * direction[1] - diff[1] * direction[0]) / -denom
        if t > 0 and 0.0 <= u <= 1.0:
            best = min(best, t)
    return best


def marker_targets(points, ids, height_m, spacing_m, ins_height_m, along_axis_fraction=0.9):
    """Where each marker goes, given a validated box.

    Returns (info, targets) where info is check_box()'s output plus the
    chosen facing, and targets is a list of dicts with lat/lon, the local
    north/east, the height above ground, and the marker-map `heading`.

    The panel sits at the box centroid facing along the long axis, in
    whichever of the two directions has more room in front of it — with
    a hand-driven boundary the centroid isn't exactly central, and the
    approach fan wants the longer run.
    """
    info = check_box(points)
    if not info["is_box"]:
        return info, []

    local = info["local"]
    centre = np.array([0.0, 0.0])  # the centroid is the local origin by construction
    long_axis = info["long_axis_bearing_deg"]
    # Which way along the long axis to face. More room in front is
    # better, but on a symmetric box the two are a near-tie (5.87m vs
    # 5.86m on the real recorded boundary), and a bare max() would then
    # flip between runs on rounding alone. That flip matters: the markers
    # are already in the ground by the time the capture path is
    # generated, and a path fanning round the wrong end of the box would
    # be looking at the backs of them. So only prefer a direction when it
    # is meaningfully roomier, and otherwise break the tie deterministically.
    a_bearing = (long_axis + 180.0) % 360.0 - 180.0
    b_bearing = (long_axis + 360.0) % 360.0 - 180.0
    a_clear, b_clear = _clearance(local, centre, a_bearing), _clearance(local, centre, b_bearing)
    if abs(a_clear - b_clear) > TIE_BREAK_CLEARANCE_M:
        face_bearing = a_bearing if a_clear > b_clear else b_bearing
    else:
        # Deterministic: the easterly-pointing one. Arbitrary, but stable.
        face_bearing = a_bearing if math.sin(math.radians(a_bearing)) >= 0 else b_bearing

    # In the m convention X points out the marker's BACK, so a marker
    # whose face looks along `face_bearing` has map heading 180 from it.
    heading = (face_bearing + 180.0 + 180.0) % 360.0 - 180.0
    # The row runs perpendicular to the facing direction.
    across = math.radians(face_bearing - 90.0)
    across_vec = np.array([math.cos(across), math.sin(across)])

    # Push the panel back towards the rear edge instead of sitting it in
    # the middle. You can't see a marker from behind it, so with the panel
    # at the centroid roughly half the box is dead space. Backing it up to
    # `along_axis_fraction` of the way to the rear edge converts almost
    # all of that into usable ground — which matters less for range (the
    # ~4m ArUco limit binds long before the boundary does) than for
    # turning room, which the rounded capture path needs. Ben's
    # suggestion, 2026-09-16.
    behind = _clearance(local, centre, face_bearing + 180.0)
    face_vec = np.array([math.cos(math.radians(face_bearing)),
                         math.sin(math.radians(face_bearing))])
    panel_centre = centre - face_vec * (behind * along_axis_fraction)

    lat0, lon0 = info["centroid"]
    offsets = (np.arange(len(ids)) - (len(ids) - 1) / 2.0) * spacing_m
    targets = []
    for marker_id, offset in zip(ids, offsets):
        ne = panel_centre + across_vec * offset
        down = -(height_m - ins_height_m)
        lat, lon, _alt = ned_to_lla(float(ne[0]), float(ne[1]), down, lat0, lon0, 0.0)
        targets.append({
            "id": int(marker_id),
            "lat": lat, "lon": lon,
            "north": float(ne[0]), "east": float(ne[1]),
            "height_m": float(height_m),
            "heading_deg": float(heading),
        })

    info = dict(info)
    info["face_bearing_deg"] = float(face_bearing)
    info["clearance_m"] = float(_clearance(local, panel_centre, face_bearing))
    info["clearance_behind_m"] = float(_clearance(local, panel_centre, face_bearing + 180.0))
    info["panel_centre_ne"] = [float(panel_centre[0]), float(panel_centre[1])]
    return info, targets


def move_guidance(robot_heading_deg, actual, target):
    """How to physically move a marker from where it actually is to where
    it should be — for someone standing at the marker with the robot in
    sight.

    "Range and bearing from the robot" (guidance_from) can't be acted on
    once the marker exists: you can't stand the robot on the spot and put
    the marker there too (Ben, 2026-09-16). What's actionable is a nudge,
    so the correction is resolved in the ROBOT's frame, which is the one
    thing both the operator and the robot can see:

      towards_robot_m   move the marker this much closer to the robot
                        (negative: further away)
      to_robot_right_m  move it this much to the robot's right
                        (negative: to the robot's left)
      rotate_deg        turn the marker's face this much clockwise seen
                        from above (negative: anticlockwise)

    The two distances are an orthogonal basis (the robot's own forward
    and right axes), not a mix of line-of-sight and body axes, so they
    can be applied one after the other without interacting.
    """
    north, east, _ = lla_to_ned(target["lat"], target["lon"], 0.0,
                                actual["lat"], actual["lon"], 0.0)
    heading = math.radians(robot_heading_deg)
    # The robot is looking at the marker, so the marker sits at +forward;
    # moving it towards the robot therefore decreases the forward component.
    forward = north * math.cos(heading) + east * math.sin(heading)
    right = -north * math.sin(heading) + east * math.cos(heading)
    return {
        "id": target["id"],
        "towards_robot_m": float(-forward),
        "to_robot_right_m": float(right),
        "move_m": float(math.hypot(north, east)),
        "rotate_deg": float((target["heading_deg"] - actual["heading"] + 180.0) % 360.0 - 180.0),
    }


def guidance_from(robot_lat, robot_lon, target):
    """Range and bearing from the robot's current position to a target —
    what the placement page shows live, and the only form of this
    information that's any use to someone standing in the garden."""
    north, east, _ = lla_to_ned(target["lat"], target["lon"], 0.0, robot_lat, robot_lon, 0.0)
    return {
        "id": target["id"],
        "range_m": float(math.hypot(north, east)),
        "bearing_deg": float(math.degrees(math.atan2(east, north))),
    }
