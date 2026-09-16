"""Bore-sight capture simulator: generate synthetic observations for a
proposed marker layout and capture path, run the real solver on them, and
report how accurately hpr_cb comes back.

The point is to design the physical setup numerically *before* anything
gets printed or planted — how many markers, where, at what heights, and
what the robot has to drive — and to answer "is 97mm enough?" with a
number instead of an opinion.

It is deliberately not a toy. The visibility gates, the noise model and
the solver are the same ones the real capture will face:

  * Observations are generated from the *true* vehicle pose and handed to
    the solver with the *reported* pose, the two differing by a modelled
    INS error. That distinction is the whole experiment — the camera is
    roughly ten times better than the reference it's being calibrated
    against, so INS error, not corner noise, sets the answer.
  * INS attitude error is modelled as a constant bias plus an
    exponentially-correlated (Ornstein-Uhlenbeck) process, not white
    noise. This matters enormously: white noise over 2500 frames would
    average down by a factor of 50 and promise an accuracy that a real
    30-second-correlated GNSS attitude error will never deliver.
  * The constant part of the INS attitude bias is very nearly degenerate
    with hpr_cb (see solve.py) and is absorbed rather than measured, so
    results are reported both ways: total error, and error relative to
    truth-plus-bias, which is the part geometry and averaging control.

Real numbers behind the defaults, measured off the 2026-09-14 mission
logs (oxts-nav/data/logs/260914_1*.jsonl, RTK-fixed, GnssPosMode 6):
NorthAcc median 0.009m; HeadingAcc median 0.18 deg moving, 0.28 deg
stationary.
"""

import numpy as np

import model

# --- camera, from shared/camera-cal.yaml -------------------------------

IMAGE_W, IMAGE_H = 1280, 960
CAMERA_MATRIX = np.array([
    [1166.0314860010476, 0.0, 633.90600534139935],
    [0.0, 1168.2026009458311, 526.17034280566315],
    [0.0, 0.0, 1.0],
])
DIST_COEFFS = np.array([
    0.091577794655201383, -0.41358268148786193,
    0.00087352266508141057, 7.2164044943823303e-05, 0.36489243164727858,
])

# --- physical / detection limits ---------------------------------------

MAX_RANGE_M = 4.0        # Ben's measured ArUco visibility limit for a 97mm marker
MIN_RANGE_M = 0.8        # below this the marker starts leaving the frame entirely
MAX_OBLIQUE_DEG = 60.0   # viewing angle off the marker normal past which detection gets unreliable
MIN_MARKER_PX = 18.0     # apparent marker side length needed for a 6x6 marker to decode
EDGE_MARGIN_PX = 12.0    # keep corners this far inside the image border

# NOTE ON HEIGHTS: this module's local NED has down=0 at the INS
# reference point's plane, not at the ground — because that is what
# nav positions actually are. A marker's `down` is therefore its height
# above the INS reference point, NOT above the ground, and the camera
# sits 0.07m above that plane via d_xc_b's z of -0.07.
#
# Don't put a "camera height above ground" constant here. There was one,
# set to an assumed 0.30m, and it was never read by anything — purely
# misleading, and it did mislead: the recommended marker heights were
# reported to Ben as heights above ground when they were heights above
# the INS plane. Ground heights are layouts.py's business, where the real
# CAD figure lives and the conversion is done once.


class Layout:
    """A physical marker arrangement. positions are local NED metres
    (north, east, down — so a marker 0.5m above the ground is down=-0.5),
    hpr is each marker's pose in the m convention aruco/coords.py uses."""

    def __init__(self, ids, positions, hprs, size=0.097, name=""):
        self.ids = list(ids)
        self.positions = np.asarray(positions, dtype=float)
        self.hprs = np.asarray(hprs, dtype=float)
        self.size = size
        self.name = name

    def __len__(self):
        return len(self.ids)


def ou_process(n, dt, sigma, tau, rng):
    """Exponentially-correlated noise: stationary with standard deviation
    `sigma` and correlation time `tau`. tau -> 0 gives white noise, tau
    -> inf gives a constant offset."""
    if tau <= 0:
        return rng.normal(0.0, sigma, size=n)
    a = np.exp(-dt / tau)
    out = np.empty(n)
    out[0] = rng.normal(0.0, sigma)
    step = sigma * np.sqrt(1.0 - a * a)
    noise = rng.normal(0.0, step, size=n)
    for i in range(1, n):
        out[i] = a * out[i - 1] + noise[i]
    return out


def legs_to_poses(legs, speed_mps=0.2, fps=5.0, bump_deg=1.5, bump_tau_s=1.5, rng=None):
    """Vehicle poses sampled along a list of straight legs.

    legs: [((n0, e0), (n1, e1)), ...] in local NED metres. The vehicle's
    heading is the leg's bearing — a path-following robot points where
    it's going — plus a little cross-track wobble.

    bump_deg is *real* pitch/roll from the bumpy ground, not an error:
    the INS measures it correctly and it genuinely helps, by adding
    attitude diversity the flat-ground case wouldn't have.

    Returns (t, pos_n, hpr) with pos_n (N,3) and hpr (N,3) degrees.
    """
    rng = rng or np.random.default_rng()
    dt = 1.0 / fps
    step = speed_mps * dt
    pos, hdg = [], []
    for (n0, e0), (n1, e1) in legs:
        dn, de = n1 - n0, e1 - e0
        length = np.hypot(dn, de)
        if length < step:
            continue
        # round, not truncate: a 2.0m leg at exactly 0.04m/frame comes out
        # as 49.9999... in floating point and would silently lose a frame.
        count = int(round(length / step))
        f = np.arange(count) / count
        pos.append(np.column_stack([n0 + f * dn, e0 + f * de]))
        hdg.append(np.full(count, np.degrees(np.arctan2(de, dn))))
    pos = np.vstack(pos)
    hdg = np.concatenate(hdg)
    n = len(hdg)
    t = np.arange(n) * dt

    # Path-following wobble: the robot doesn't track a line perfectly, and
    # the heading variation that comes with it is real signal, not noise.
    hdg = hdg + ou_process(n, dt, 2.0, 2.0, rng)
    pitch = ou_process(n, dt, bump_deg, bump_tau_s, rng)
    roll = ou_process(n, dt, bump_deg, bump_tau_s, rng)
    pos_n = np.column_stack([pos[:, 0], pos[:, 1], np.zeros(n)])
    return t, pos_n, np.column_stack([hdg, pitch, roll])


def stations_to_poses(stations, dwell_s=3.0, fps=5.0, bump_deg=1.5, rng=None):
    """Poses for a stop-and-stare capture: each station is
    ((north, east), heading_deg) and contributes dwell_s of frames.
    Stationary capture removes motion blur and any camera-timestamp
    latency, at the cost of a slightly worse GNSS heading (0.28 vs
    0.18 deg) and fewer frames per minute."""
    rng = rng or np.random.default_rng()
    dt = 1.0 / fps
    per = max(int(dwell_s * fps), 1)
    pos, hpr, t = [], [], []
    clock = 0.0
    for (north, east), heading in stations:
        pos.append(np.tile([north, east, 0.0], (per, 1)))
        tilt = rng.normal(0.0, bump_deg, size=2)  # fixed attitude while parked on one bump
        hpr.append(np.tile([heading, tilt[0], tilt[1]], (per, 1)))
        t.append(clock + np.arange(per) * dt)
        clock += dwell_s + 6.0  # allow for driving between stations
    return np.concatenate(t), np.vstack(pos), np.vstack(hpr)


def visible(pos_n, hpr, layout, marker_index, hpr_cb, dxc_b):
    """Boolean mask: can this marker actually be detected from each pose?

    Applies the four gates that decide it in practice — range, viewing
    obliquity, apparent size, and whether all four corners are actually
    inside the frame. The last is the one that bites hardest with a
    narrow (57 x 45 deg) FOV and is exactly why a marker 1m up can't be
    seen from 1m away.
    """
    n = len(pos_n)
    marker_pos = np.tile(layout.positions[marker_index], (n, 1))
    marker_hpr = np.tile(layout.hprs[marker_index], (n, 1))
    sizes = np.full(n, layout.size)

    to_marker = marker_pos - pos_n
    rng_m = np.linalg.norm(to_marker, axis=1)
    ok = (rng_m >= MIN_RANGE_M) & (rng_m <= MAX_RANGE_M)

    normal = model.marker_normal_n(marker_hpr)
    view_dir = -to_marker / rng_m[:, None]  # marker -> camera, unit
    cos_oblique = np.sum(normal * view_dir, axis=1)
    ok &= cos_oblique >= np.cos(np.radians(MAX_OBLIQUE_DEG))

    corners = model.project(pos_n, hpr, marker_pos, marker_hpr, sizes, hpr_cb, dxc_b,
                            CAMERA_MATRIX, DIST_COEFFS)
    # In front of the camera at all? projectPoints happily projects points
    # behind it, which would otherwise pass the in-frame test as a ghost.
    x_C = model.corners_in_camera_frame(pos_n, hpr, marker_pos, marker_hpr, sizes, hpr_cb, dxc_b)
    ok &= np.all(x_C[..., 2] > 0.05, axis=1)

    ok &= np.all(corners[..., 0] > EDGE_MARGIN_PX, axis=1)
    ok &= np.all(corners[..., 0] < IMAGE_W - EDGE_MARGIN_PX, axis=1)
    ok &= np.all(corners[..., 1] > EDGE_MARGIN_PX, axis=1)
    ok &= np.all(corners[..., 1] < IMAGE_H - EDGE_MARGIN_PX, axis=1)

    side_px = np.linalg.norm(corners[:, 0] - corners[:, 1], axis=1)
    ok &= side_px >= MIN_MARKER_PX
    return ok, corners


class NoiseModel:
    """INS and camera error. Defaults are the measured 2026-09-14
    figures; `bias_*` are the constant components that hpr_cb absorbs
    rather than measures."""

    def __init__(self, sigma_px=0.2, pos_sigma_m=0.01,
                 heading_sigma_deg=0.18, heading_tau_s=30.0, heading_bias_deg=0.15,
                 tilt_sigma_deg=0.04, tilt_tau_s=30.0, tilt_bias_deg=0.03,
                 timestamp_latency_s=0.0):
        self.sigma_px = sigma_px
        self.pos_sigma_m = pos_sigma_m
        self.heading_sigma_deg = heading_sigma_deg
        self.heading_tau_s = heading_tau_s
        self.heading_bias_deg = heading_bias_deg
        self.tilt_sigma_deg = tilt_sigma_deg
        self.tilt_tau_s = tilt_tau_s
        self.tilt_bias_deg = tilt_bias_deg
        # Lag between the instant the shutter actually opened and the nav
        # sample paired with it — exposure, the camera pipeline, and the
        # 20Hz nav feed having no interpolation hook. SYSTEMATIC, not
        # random: the same lag every frame, so it biases rather than
        # averaging away, which is exactly what makes it dangerous and
        # why it needs modelling before stationary-vs-moving capture can
        # be decided. Zero while stationary, by construction.
        self.timestamp_latency_s = timestamp_latency_s


def apply_timestamp_latency(t, pos, hpr, latency_s):
    """Where the nav solution *was*, `latency_s` before each frame was
    actually exposed — i.e. the pose a capture would wrongly attach to
    that frame.

    First-order (pose minus rate times lag) rather than a resampling of
    the trajectory: the lag is tens of milliseconds against 0.2s frames,
    so the curvature term is negligible, and it keeps this honest about
    being a small correction.

    The rates are clipped because legs_to_poses concatenates straight
    legs end to end, and consecutive legs meet at a heading
    discontinuity. Differentiating across that join gives a meaningless
    spike, which would otherwise manufacture a huge fake latency error
    at a handful of frames.
    """
    if not latency_s:
        return pos, hpr
    vel = np.clip(np.gradient(pos, t, axis=0), -2.0, 2.0)
    unwrapped = hpr.copy()
    unwrapped[:, 0] = np.degrees(np.unwrap(np.radians(hpr[:, 0])))
    rate = np.clip(np.gradient(unwrapped, t, axis=0), -60.0, 60.0)
    return pos - vel * latency_s, hpr - rate * latency_s


def simulate(layout, t, pos_true, hpr_true, hpr_cb_true, dxc_b, noise, rng, assumed_size=None):
    """Build an Observations set for one realisation.

    Corner pixels come from the TRUE vehicle pose; the poses attached to
    the observations are the INS's REPORTED ones. Returns
    (Observations, absorbed_bias_deg) — the bias being the constant part
    of the attitude error, which no geometry can separate from hpr_cb.

    assumed_size: the marker size the *solver* will be told, when that
    differs from the layout's true size — i.e. a mis-configured marker
    size. Images are always generated at the true size. This is the real
    situation found live on 2026-09-16: markers are 100.2mm and config
    says 97mm.
    """
    import solve as solve_mod

    n = len(t)
    dt = float(np.median(np.diff(t))) if n > 1 else 0.2

    bias = np.array([
        rng.normal(0.0, noise.heading_bias_deg),
        rng.normal(0.0, noise.tilt_bias_deg),
        rng.normal(0.0, noise.tilt_bias_deg),
    ])
    hpr_err = np.column_stack([
        bias[0] + ou_process(n, dt, noise.heading_sigma_deg, noise.heading_tau_s, rng),
        bias[1] + ou_process(n, dt, noise.tilt_sigma_deg, noise.tilt_tau_s, rng),
        bias[2] + ou_process(n, dt, noise.tilt_sigma_deg, noise.tilt_tau_s, rng),
    ])
    pos_err = rng.normal(0.0, noise.pos_sigma_m, size=(n, 3))

    # Latency first (it's a property of when the pose was sampled), then
    # the INS's own errors on top of that sampled pose.
    pos_lagged, hpr_lagged = apply_timestamp_latency(t, pos_true, hpr_true, noise.timestamp_latency_s)
    hpr_reported = hpr_lagged + hpr_err
    pos_reported = pos_lagged + pos_err

    rows_t, rows_pos, rows_hpr, rows_idx, rows_corners = [], [], [], [], []
    for k in range(len(layout)):
        ok, corners = visible(pos_true, hpr_true, layout, k, hpr_cb_true, dxc_b)
        if not ok.any():
            continue
        noisy = corners[ok] + rng.normal(0.0, noise.sigma_px, size=corners[ok].shape)
        rows_t.append(t[ok])
        rows_pos.append(pos_reported[ok])
        rows_hpr.append(hpr_reported[ok])
        rows_idx.append(np.full(ok.sum(), k))
        rows_corners.append(noisy)

    if not rows_idx:
        return None, bias
    obs = solve_mod.Observations(
        np.concatenate(rows_t), np.vstack(rows_pos), np.vstack(rows_hpr),
        np.concatenate(rows_idx), np.vstack(rows_corners),
        np.full(sum(len(r) for r in rows_idx),
                layout.size if assumed_size is None else assumed_size),
        layout.ids,
    )
    return obs, bias


def monte_carlo(layout, pose_fn, hpr_cb_true, dxc_b, noise=None, trials=20, seed=0):
    """Run `trials` independent realisations and report how well hpr_cb
    comes back.

    pose_fn(rng) -> (t, pos, hpr), so each trial re-draws the bumps and
    wobble as well as the noise.

    "absorbed" is error measured against truth-plus-INS-bias: the part
    the capture design can actually influence. "total" includes the bias,
    which is what you'd see if you compared against a perfect external
    reference — and which is harmless for aruco's purposes, since GAD
    wants the camera consistent with the frame the xNAV reports.
    """
    import solve as solve_mod

    noise = noise or NoiseModel()
    rng = np.random.default_rng(seed)
    total_errs, absorbed_errs, counts, rms = [], [], [], []
    for _ in range(trials):
        t, pos, hpr = pose_fn(rng)
        obs, bias = simulate(layout, t, pos, hpr, hpr_cb_true, dxc_b, noise, rng)
        if obs is None or len(obs) < 50 or len(np.unique(obs.marker_idx)) < len(layout):
            counts.append(0 if obs is None else len(obs))
            continue
        out = solve_mod.solve(obs, hpr_cb_true, dxc_b, CAMERA_MATRIX, DIST_COEFFS,
                              sigma_px=noise.sigma_px)
        if not out["success"]:
            continue
        total_errs.append(out["hpr_cb"] - np.asarray(hpr_cb_true))
        absorbed_errs.append(out["hpr_cb"] - (np.asarray(hpr_cb_true) + bias))
        counts.append(len(obs))
        rms.append(out["rms_px"])

    total_errs = np.array(total_errs)
    absorbed_errs = np.array(absorbed_errs)
    return {
        "name": layout.name,
        "trials": len(total_errs),
        "obs_median": float(np.median(counts)) if counts else 0.0,
        "rms_px": float(np.mean(rms)) if rms else float("nan"),
        "total_rms_deg": np.sqrt((total_errs ** 2).mean(axis=0)) if len(total_errs) else np.full(3, np.nan),
        "absorbed_rms_deg": np.sqrt((absorbed_errs ** 2).mean(axis=0)) if len(absorbed_errs) else np.full(3, np.nan),
        "bias_deg": total_errs.mean(axis=0) if len(total_errs) else np.full(3, np.nan),
    }
