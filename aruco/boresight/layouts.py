"""Candidate marker layouts and capture paths for the "Boresight limits"
box, plus the box itself.

Kept rather than thrown away after the design decision: the same
machinery re-answers the question if the box moves, if bigger markers get
printed, or if a second calibration is ever wanted somewhere else.

The box, as recorded by driving its four corners (navigate/data/
"Boresight limits.yaml") and converted to local NED about its own
centroid: 11.7m x 6.7m, interior angles 86.8-95.3 deg (a genuine
rectangle), long axis on bearing 73 deg. Critically, it is 3.35m from the
centroid to the near (long) edges — so a marker station at the centre can
be driven round at 2-3m all the way about, inside the 4m ArUco visibility
limit for a 97mm marker. That fit is what makes a central station the
natural choice.
"""

import numpy as np

import sim

# Local NED metres about the box centroid (52.235464288, -1.460508331).
BOX_CORNERS = np.array([
    [4.53, 4.66],
    [1.91, -6.51],
    [-4.97, -4.82],
    [-1.47, 6.67],
])
LONG_AXIS_BEARING_DEG = 73.0

# The panel faces along the box's long axis, where there's most room
# (5.9m to the short edges vs 3.35m to the long ones). In the m
# convention X points out the *back* of a marker, so a marker whose face
# looks along bearing B has heading B - 180.
PANEL_FACE_BEARING_DEG = LONG_AXIS_BEARING_DEG + 180.0  # 253 deg, looking WSW
PANEL_HEADING_DEG = LONG_AXIS_BEARING_DEG               # 73 deg

# --- heights -----------------------------------------------------------
#
# Everything the operator sees is a height ABOVE GROUND, in metres; the
# simulator works in `down` about the INS reference point's plane (see
# sim.py's note). This is the one place the two are related.
CAMERA_HEIGHT_M = 0.133          # CAD, Ben 2026-09-14
DXC_B_DOWN_M = -0.07             # config.yaml aruco.camera_extrinsics.d_xc_b[2]; camera is ABOVE the INS
INS_HEIGHT_M = CAMERA_HEIGHT_M + DXC_B_DOWN_M  # 0.063m above ground


def height_to_down(height_above_ground_m):
    """Ground height -> the `down` coordinate the simulator and marker
    map use. Negative down is above the INS plane, as usual."""
    return -(np.asarray(height_above_ground_m, dtype=float) - INS_HEIGHT_M)


def down_to_height(down_m):
    return INS_HEIGHT_M - np.asarray(down_m, dtype=float)


# The camera is only 133mm up, so there is almost no room to put a marker
# *below* it — every marker is above the boresight, and the higher it is
# the further away the robot must be before it fits in the 45 deg
# vertical FOV. See evaluate.py's height comparison: within 0.15-0.8m it
# makes no measurable difference, so these are chosen for buildability.
#
# Lowest at 0.20m, not 0.15m: these are marker *centres*, and a 97mm
# marker centred at 0.15m has its bottom black edge at ~0.10m, with the
# printed sheet's white quiet zone lower again — close enough for grass
# to reach the bottom corners and spoil the detection (Ben, 2026-09-16).
LOWEST_HEIGHT_M = 0.20
HEIGHTS_ABOVE_GROUND = (0.20, 0.40, 0.60)

# The recommended build: a FLAT row, all three at one height.
#
# Staggered heights and a triangle were both measured against it (see
# evaluate.py) and neither is distinguishable from flat at 50 trials.
# Ben's reasoning for why is the right one: the robot's own motion
# already sweeps the markers across the image in two dimensions —
# horizontally as it drives past, vertically as the elevation angle
# changes with range — so instantaneous 2D spread adds nothing the path
# hasn't already provided. Flat is much the easiest to build, so flat
# wins on buildability alone.
#
# 0.25m rather than Ben's suggested 0.20m: his markers carry a 5cm
# border all round, making the printed sheet ~197mm, so a centre at
# 0.20m puts the bottom of the white quiet zone at 0.10m. 0.25m keeps
# 0.15m of clearance under the border for grass. Height is free
# (see the height table), so the margin costs nothing.
RECOMMENDED_HEIGHT_M = 0.25
HEIGHTS = tuple(height_to_down(HEIGHTS_ABOVE_GROUND))

INSIDE_MARGIN_M = 0.4  # keep the simulated path this far inside the recorded boundary


def inside_box(points_ne, margin=INSIDE_MARGIN_M):
    """Point-in-polygon against the recorded boundary, shrunk by
    `margin`. Used to reject candidate path legs that would take the
    robot outside the area Ben actually measured — a simulated result
    from an undriveable path is worthless."""
    pts = np.atleast_2d(points_ne)
    centre = BOX_CORNERS.mean(axis=0)
    shrunk = centre + (BOX_CORNERS - centre) * (1.0 - margin / 3.0)
    inside = np.ones(len(pts), dtype=bool)
    for i in range(len(shrunk)):
        a, b = shrunk[i], shrunk[(i + 1) % len(shrunk)]
        edge = b - a
        # Corners are recorded clockwise in NED; a point is inside when it
        # is on the same side of every edge.
        cross = edge[0] * (pts[:, 1] - a[1]) - edge[1] * (pts[:, 0] - a[0])
        inside &= cross <= 0
    return inside  # always an array, even for one point — callers use .all() or [0]


# --- layouts ------------------------------------------------------------

def _panel(ids, heights, easts, heading, name, size=0.097):
    """One flat panel of markers. `easts` are offsets along the panel's
    own width; the panel is built perpendicular to its facing direction."""
    face = np.radians(heading + 180.0)
    # Panel width runs perpendicular to the facing direction.
    across = np.array([-np.sin(face), np.cos(face)])
    positions = [[across[0] * e, across[1] * e, h] for e, h in zip(easts, heights)]
    return sim.Layout(ids, positions, [[heading, 2.0, -1.5]] * len(ids), size=size, name=name)


def single(size=0.097):
    return sim.Layout([20], [[0.0, 0.0, float(height_to_down(0.35))]],
                      [[PANEL_HEADING_DEG, 2.0, -1.5]], size=size, name="1 marker")


def stack3(size=0.097):
    """Three markers stacked vertically on one post — Ben's "easier than a
    slope" option."""
    return _panel([20, 21, 22], HEIGHTS, [0.0, 0.0, 0.0], PANEL_HEADING_DEG, "3 stacked", size)


def row3(size=0.097, height=RECOMMENDED_HEIGHT_M):
    """Three in a horizontal row, all the same height."""
    return _panel([20, 21, 22], [float(height_to_down(height))] * 3, [-0.5, 0.0, 0.5],
                  PANEL_HEADING_DEG, "3 in a row", size)


def recommended(size=0.097):
    """The layout to actually build — see RECOMMENDED_HEIGHT_M."""
    lay = row3(size=size, height=RECOMMENDED_HEIGHT_M)
    lay.name = "recommended: 3 in a flat row"
    return lay


def row3h(size=0.097, heights=HEIGHTS_ABOVE_GROUND):
    """Three in a horizontal row, each at a different height — a single
    rail with the markers staggered. Gives horizontal *and* vertical
    image spread for the same build effort as a plain row."""
    return _panel([20, 21, 22], height_to_down(heights), [-0.5, 0.0, 0.5],
                  PANEL_HEADING_DEG, "3 staggered", size)


def triangle3(size=0.097, base=1.0, apex_height=0.60, base_height=LOWEST_HEIGHT_M):
    """Two markers low and wide, one raised in the middle.

    Ben's intuition (2026-09-16): a triangle is "more 2D" than a line, so
    if marker geometry mattered a triangle ought to beat a row. Worth
    measuring rather than arguing about — the counter-argument is that
    the robot's own motion already sweeps the markers across the image in
    two dimensions (horizontally as it drives past, vertically as the
    elevation angle changes with range), so the instantaneous spread may
    add nothing the path hasn't already provided.
    """
    half = base / 2.0
    return _panel([20, 21, 22],
                  height_to_down([base_height, base_height, apex_height]),
                  [-half, half, 0.0], PANEL_HEADING_DEG, "3 in a triangle", size)


def row3h_back_to_back(size=0.097):
    """Two staggered rails back-to-back, so the station is visible from
    both sides and the robot can work all the way round it."""
    front = row3h(size)
    back = _panel([23, 24, 25], HEIGHTS, [-0.5, 0.0, 0.5], PANEL_HEADING_DEG + 180.0, "", size)
    # (both panels share HEIGHTS — same rail heights, opposite faces)
    return sim.Layout(
        front.ids + back.ids,
        np.vstack([front.positions, back.positions]),
        np.vstack([front.hprs, back.hprs]),
        size=size, name="3 staggered, back-to-back",
    )


# --- capture paths ------------------------------------------------------

def _clip(legs):
    """Drop any leg with an end outside the boundary."""
    return [leg for leg in legs if inside_box(np.array(leg)).all()]


def fan_legs(sector_centre_deg=PANEL_FACE_BEARING_DEG, half_width_deg=55.0, count=7,
             r_far=3.5, r_near=1.3):
    """Radial in-and-out legs across a sector in front of the panel — the
    robot drives straight at the station, so the markers sit near the
    image centre. This is what gives azimuth spread, and so heading."""
    legs = []
    for brg in np.linspace(sector_centre_deg - half_width_deg,
                           sector_centre_deg + half_width_deg, count):
        a = np.radians(brg)
        far = (r_far * np.cos(a), r_far * np.sin(a))
        near = (r_near * np.cos(a), r_near * np.sin(a))
        legs.append((far, near))
        legs.append((near, far))  # and back out again — range diversity, and it has to return anyway
    return _clip(legs)


def past_legs(sector_centre_deg=PANEL_FACE_BEARING_DEG, half_width_deg=55.0, count=6,
              r_start=3.4, r_end=1.6, swing_deg=45.0):
    """Legs that start aimed near the station and end off to one side, so
    the markers sweep out towards the image edge instead of sitting in
    the middle. This is the roll signal: camera roll displaces an image
    point by rho.phi, so content at rho~600px is worth more than twice
    what the panel's own 1m baseline provides.

    Generated in mirrored pairs (swinging left and right by turns), on
    the theory that distortion-model error at the image edge also looks
    like roll and should cancel if the edge is visited symmetrically.
    That theory is UNTESTED — it is a reason to keep the symmetry, not a
    result.

    What *is* tested: this symmetry does NOT cancel a capture-timestamp
    latency. Mirroring the fan made the latency bias worse, not better
    (-0.170 deg one-directional vs +0.326 deg mirrored, at 50ms). An
    earlier version of this comment was cited as if it covered latency
    too; it never did. Latency is dealt with by interpolating nav to the
    frame's own GPS time — see boresight-prd.md's capture section.
    """
    legs = []
    for i, brg in enumerate(np.linspace(sector_centre_deg - half_width_deg,
                                        sector_centre_deg + half_width_deg, count)):
        swing = swing_deg if i % 2 == 0 else -swing_deg
        a0, a1 = np.radians(brg), np.radians(brg + swing)
        start = (r_start * np.cos(a0), r_start * np.sin(a0))
        end = (r_end * np.cos(a1), r_end * np.sin(a1))
        legs.append((start, end))
        legs.append((end, start))  # the mirror of each swing, for the same cancellation reason
    return _clip(legs)


def both_sides_legs(**kwargs):
    """Fan + past legs in BOTH sectors, for a back-to-back layout.

    Note what this replaced: an "orbit" — a closed polygon driven around
    the station. That is useless here and the simulator caught it, by
    finding no trial in which every marker was seen. The camera is fixed
    and forward-facing, so driving a circle around a station points it
    tangentially, 90 degrees away from the very thing it is orbiting.
    Approaching from many directions is what gives azimuth spread; going
    *round* does not.
    """
    front = fan_legs(**kwargs) + past_legs(**kwargs)
    back = (fan_legs(sector_centre_deg=PANEL_FACE_BEARING_DEG + 180.0, **kwargs)
            + past_legs(sector_centre_deg=PANEL_FACE_BEARING_DEG + 180.0, **kwargs))
    return front + back


def path_fn(legs, **kwargs):
    """Wrap a leg list into the pose_fn(rng) the Monte Carlo wants."""
    def fn(rng):
        return sim.legs_to_poses(legs, rng=rng, **kwargs)
    return fn
