"""Bore-sight solve: least-squares estimate of hpr_cb from a set of
marker observations, with each marker's own 6-DOF pose carried as a free
nuisance parameter.

Why the marker poses are free rather than surveyed: Ben can only plant a
marker approximately, and can't measure the spacing between markers to
better than a centimetre or two. Estimating them costs 6 parameters per
marker and buys complete independence from how accurately anything was
planted or measured. The markers end up surveyed as a *by-product* of
the calibration, to far better than a hand measurement — which is also
the cross-check: their solved attitudes can be compared against a
spirit-level reading.

Observability, which is what drives the capture design (see sim.py):

  * hpr_cb heading is exactly degenerate with each marker's own heading
    if you use rotations alone — yaw the camera +5 deg and every marker
    -5 deg and no rvec changes. It's broken by *position* consistency:
    a camera yaw error psi puts the estimated marker at ~r.psi sideways
    in the body frame, which points in a different nav-frame direction
    for every vehicle heading. So azimuth spread per marker is the
    single most valuable thing the capture path can provide.
  * hpr_cb roll is near-degenerate with a marker's own roll while that
    marker sits at the image centre. Camera roll displaces an image
    point at radius rho from the principal point by rho.phi, so it's
    broken by observations well off-centre - which the path provides by
    driving *past* a marker rather than always straight at it.
  * hpr_cb pitch trades against marker height as r.pitch, so range
    diversity separates it.

A constant INS heading bias is very nearly degenerate with hpr_cb
heading, and is absorbed into it rather than measured. That's the right
outcome rather than a flaw: aruco's GAD updates want the camera
consistent with the frame the xNAV650 actually reports, not with true
north.

"Very nearly", not "exactly" — measured, in test_solve.py:

  * With the camera at the INS origin and a zero mount angle, absorption
    is exact. C_nb(h+b,p,r) = Rz(b).C_nb, and on level ground that
    nav-frame rotation passes straight through to the camera.
  * The real 0.105m lever arm leaks ~1.3% of it. A heading bias swings
    the camera *position* by |d_xc_b|.b, and no camera rotation
    reproduces a translation.
  * A non-zero mount angle leaks a little into pitch/roll instead, since
    an hpr_cb heading change is a rotation about the c-frame vertical
    while the INS bias is one about the body vertical.

Vehicle tilt, which might look like the obvious thing to break it, turns
out not to matter measurably. The practical consequence is small — a
0.15 deg INS bias leaks ~0.002 deg — but it's why the numbers don't come
out at exactly the bias.
"""

import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import coords  # noqa: E402
import model  # noqa: E402

DEFAULT_SIGMA_PX = 0.2  # per corner coordinate; camera-cal.yaml's RMS reprojection error is 0.18px


class Observations:
    """A capture session's worth of marker sightings, in the flat
    array-of-columns shape the solver wants.

    t           (N,)       capture time, seconds
    nav_pos_n   (N, 3)     vehicle position, local NED metres
    nav_hpr     (N, 3)     vehicle attitude, degrees (already body-frame — see survey.py)
    marker_idx  (N,)       index into marker_ids
    corners     (N, 4, 2)  detected corner pixels
    sizes       (N,)       marker size in metres

    Rows are in no particular order — two markers seen in the same frame
    are two rows sharing a timestamp, and nothing guarantees the set
    arrives sorted. `t` is carried explicitly for that reason: split_check
    needs real capture times to group by, and inferring them from row
    order silently gives per-marker blocks instead of time slices.
    """

    def __init__(self, t, nav_pos_n, nav_hpr, marker_idx, corners, sizes, marker_ids):
        self.t = np.asarray(t, dtype=float)
        self.nav_pos_n = np.asarray(nav_pos_n, dtype=float)
        self.nav_hpr = np.asarray(nav_hpr, dtype=float)
        self.marker_idx = np.asarray(marker_idx, dtype=int)
        self.corners = np.asarray(corners, dtype=float)
        self.sizes = np.asarray(sizes, dtype=float)
        self.marker_ids = list(marker_ids)

    def __len__(self):
        return len(self.marker_idx)

    @property
    def n_markers(self):
        return len(self.marker_ids)

    def subset(self, mask):
        mask = np.asarray(mask)
        return Observations(
            self.t[mask], self.nav_pos_n[mask], self.nav_hpr[mask], self.marker_idx[mask],
            self.corners[mask], self.sizes[mask], self.marker_ids,
        )


def _pack(hpr_cb, marker_pos, marker_hpr, size_scale=None):
    parts = [np.asarray(hpr_cb, float), np.asarray(marker_pos, float).ravel(),
             np.asarray(marker_hpr, float).ravel()]
    if size_scale is not None:
        parts.append(np.array([size_scale], dtype=float))
    return np.concatenate(parts)


def _unpack(params, n_markers, with_size_scale=False):
    """Size scale goes LAST so adding it leaves every other index alone."""
    hpr_cb = params[:3]
    marker_pos = params[3:3 + 3 * n_markers].reshape(n_markers, 3)
    marker_hpr = params[3 + 3 * n_markers:3 + 6 * n_markers].reshape(n_markers, 3)
    size_scale = params[3 + 6 * n_markers] if with_size_scale else 1.0
    return hpr_cb, marker_pos, marker_hpr, size_scale


def initial_marker_poses(obs, hpr_cb, dxc_b, camera_matrix, dist_coeffs, max_per_marker=60):
    """Rough per-marker starting pose, from the same single-shot maths
    survey.py already uses on a live detection — run over every
    observation of each marker and reduced with a median, so one bad
    detection can't set the starting point.

    Medians are taken per-component. For position that's unimpeachable;
    for heading it's only safe because these markers are static and
    viewed over a modest angular range, so the values never wrap. The
    solve that follows doesn't care about small errors here anyway — it
    only needs a basin, and hpr_cb is already approximately right.
    """
    import cv2  # local: only needed for the bootstrap, not the solve itself

    n = obs.n_markers
    pos = np.zeros((n, 3))
    hpr = np.zeros((n, 3))
    for k in range(n):
        rows = np.flatnonzero(obs.marker_idx == k)
        # estimatePoseSingleMarkers is one Python-level cv2 call per
        # observation, and a median over a few dozen spread-out views is
        # every bit as good a starting point as one over thousands — this
        # only has to land the solve in the right basin.
        if len(rows) > max_per_marker:
            rows = rows[np.linspace(0, len(rows) - 1, max_per_marker).astype(int)]
        est_pos, est_hpr = [], []
        for i in rows:
            pixels = obs.corners[i].astype(np.float32).reshape(1, 4, 2)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [pixels], float(obs.sizes[i]), camera_matrix, dist_coeffs
            )
            rvec, tvec = rvecs[0][0], tvecs[0][0]
            c_nb = coords.hpr_to_dcm(*obs.nav_hpr[i])
            dxm_b = np.asarray(dxc_b) + coords.displacement_camera_to_body(tvec, hpr_cb)
            est_pos.append(obs.nav_pos_n[i] + c_nb @ dxm_b)
            est_hpr.append(coords.dcm_to_hpr(coords.marker_dcm_from_vehicle_attitude(c_nb, rvec, hpr_cb)))
        pos[k] = np.median(np.array(est_pos), axis=0)
        hpr[k] = np.median(np.array(est_hpr), axis=0)
    return pos, hpr


def residuals(params, obs, dxc_b, camera_matrix, dist_coeffs, sigma_px, with_size_scale=False):
    """Corner reprojection error in units of sigma, flattened."""
    hpr_cb, marker_pos, marker_hpr, size_scale = _unpack(params, obs.n_markers, with_size_scale)
    predicted = model.project(
        obs.nav_pos_n, obs.nav_hpr,
        marker_pos[obs.marker_idx], marker_hpr[obs.marker_idx],
        obs.sizes * size_scale, hpr_cb, dxc_b, camera_matrix, dist_coeffs,
    )
    return ((predicted - obs.corners) / sigma_px).ravel()


def solve(obs, hpr_cb0, dxc_b, camera_matrix, dist_coeffs, sigma_px=DEFAULT_SIGMA_PX,
          marker_pos0=None, marker_hpr0=None, huber=None, estimate_size_scale=False):
    """Estimate hpr_cb (and every marker's pose) from `obs`.

    huber: robust-loss scale in sigma units, or None for plain
    least-squares. A real capture wants it on (a mis-detected or
    half-occluded marker shouldn't drag the answer); the simulator's
    clean data doesn't need it, and leaving it off there keeps the
    covariance interpretation simple.

    estimate_size_scale: carry one extra unknown, a global multiplier on
    every marker's assumed size.

    NOT because a wrong size would otherwise corrupt the answer — it
    would not, and an earlier version of this docstring wrongly said so.
    Measured in test_solve.py: a 3.3% size error with the scale held
    fixed shifts hpr_cb by 0.0035 deg and the solved marker positions by
    0mm, because hpr_cb is fixed by the *bearings* to the corners while
    size only affects *range*. The two are orthogonal, and with many
    views the bearings pin the positions hard enough that the size error
    has nowhere to go but the residual.

    It is carried because it *measures* the marker size, as a free
    by-product of a calibration run — useful in its own right, and an
    independent check on the hand measurement. (Note the live GAD path,
    unlike this solve, is NOT immune: it uses a single-shot tvec whose
    magnitude scales directly with the assumed size. Markers measured
    100.2mm on 2026-09-16 while config says 97mm, and that 3.3% is what
    pulls navigation towards the marker at the waterbutt.)

    Well observed here because the vehicle positions are RTK-known, so a
    range scale error cannot hide.

    Returns a dict — hpr_cb, its formal 1-sigma, the solved marker poses,
    the size scale, and residual statistics.

    The formal sigma here reflects corner noise and geometry ONLY. It
    does NOT include the INS's own attitude/position error, which is the
    dominant term in reality (HeadingAcc ~0.2 deg vs ~0.01 deg of camera
    bearing noise), so it will read optimistically small. Use sim.py's
    Monte Carlo, or split_check() on real data, for an honest number.
    """
    if marker_pos0 is None or marker_hpr0 is None:
        marker_pos0, marker_hpr0 = initial_marker_poses(obs, hpr_cb0, dxc_b, camera_matrix, dist_coeffs)

    x0 = _pack(hpr_cb0, marker_pos0, marker_hpr0, 1.0 if estimate_size_scale else None)
    kwargs = {"loss": "huber", "f_scale": huber} if huber else {}
    args = (obs, dxc_b, camera_matrix, dist_coeffs, sigma_px, estimate_size_scale)
    result = least_squares(residuals, x0, args=args, method="trf", x_scale="jac", **kwargs)
    hpr_cb, marker_pos, marker_hpr, size_scale = _unpack(result.x, obs.n_markers, estimate_size_scale)

    resid_px = residuals(result.x, *args) * sigma_px
    dof = max(len(resid_px) - len(result.x), 1)
    # Scale the covariance by the achieved reduced chi-square rather than
    # trusting sigma_px outright — if the real corner noise is bigger than
    # assumed (it usually is: mounting flex, marker non-flatness, a
    # distortion model that isn't perfect at the edges), this notices.
    chi2_red = float(np.sum((resid_px / sigma_px) ** 2) / dof)
    try:
        cov = np.linalg.inv(result.jac.T @ result.jac) * chi2_red
        sigma = np.sqrt(np.diag(cov)[:3])
    except np.linalg.LinAlgError:
        cov, sigma = None, np.full(3, np.nan)

    return {
        "hpr_cb": hpr_cb,
        "hpr_cb_sigma_formal": sigma,
        "marker_pos": marker_pos,
        "marker_hpr": marker_hpr,
        "size_scale": float(size_scale),
        "size_scale_sigma_formal": (
            float(np.sqrt(cov[-1, -1])) if (cov is not None and estimate_size_scale) else float("nan")
        ),
        "marker_ids": list(obs.marker_ids),
        "n_obs": len(obs),
        "rms_px": float(np.sqrt(np.mean(resid_px ** 2))),
        "max_px": float(np.max(np.abs(resid_px))),
        "chi2_reduced": chi2_red,
        "success": bool(result.success),
        "covariance": cov,
    }


# Minimum viewpoint diversity before an answer means anything. These are
# "is this a real dataset" floors, far below what the design study says
# is needed (a full pattern gives ~110 deg of azimuth spread and metres
# of position spread) — they exist to catch a run that never happened,
# not to judge a marginal one.
MIN_POSITION_SPREAD_M = 1.5
MIN_HEADING_SPREAD_DEG = 25.0
MIN_RANGE_SPREAD_M = 0.5


def geometry_check(obs):
    """Does this data have enough viewpoint diversity to determine
    hpr_cb at all?

    Emphatically not optional. A stationary capture — 318 detections of
    all three markers, residual 0.42px, `success: True` — solved to a
    marker size of 11.25 METRES with the markers 130m away and hpr_cb at
    [76, 0.8, 75]. Nothing in the residuals hinted at it: with one
    viewpoint the model can trade marker distance against marker size
    almost freely, so it fits the pixels beautifully while being
    meaningless.

    Observed 2026-09-16 on a real stationary session, which is exactly
    what an aborted or never-started run would leave behind.
    """
    problems = {}
    extent = obs.nav_pos_n.max(axis=0) - obs.nav_pos_n.min(axis=0)
    position_spread = float(np.hypot(extent[0], extent[1]))
    headings = np.radians(obs.nav_hpr[:, 0])
    # Circular spread, so a run either side of north isn't read as 360.
    mean_vector = np.hypot(np.cos(headings).mean(), np.sin(headings).mean())
    heading_spread = float(np.degrees(np.arccos(np.clip(mean_vector, -1.0, 1.0)) * 2.0))

    if position_spread < MIN_POSITION_SPREAD_M:
        problems["position_spread_m"] = position_spread
    if heading_spread < MIN_HEADING_SPREAD_DEG:
        problems["heading_spread_deg"] = heading_spread

    ranges = {}
    for k, marker_id in enumerate(obs.marker_ids):
        rows = obs.marker_idx == k
        if not rows.any():
            problems[f"marker_{marker_id}"] = "never seen"
            continue
        seen = obs.nav_pos_n[rows]
        ranges[marker_id] = float(np.hypot(*(seen.max(axis=0) - seen.min(axis=0))[:2]))
    if ranges and max(ranges.values()) < MIN_RANGE_SPREAD_M:
        problems["viewpoint_spread_per_marker_m"] = max(ranges.values())

    return {
        "ok": not problems,
        "problems": problems,
        "position_spread_m": round(position_spread, 3),
        "heading_spread_deg": round(heading_spread, 2),
        "n_obs": len(obs),
    }


def split_check(obs, hpr_cb0, dxc_b, camera_matrix, dist_coeffs, n_groups=4, **kwargs):
    """Solve independently on `n_groups` contiguous time slices and
    return the per-group hpr_cb estimates plus their scatter.

    This is the honest uncertainty estimate for real data. The formal
    covariance assumes independent corner noise and a perfect INS; the
    real error budget is dominated by INS attitude error that is
    correlated over tens of seconds. Splitting by time and looking at
    the spread measures whatever is actually there — mount flex, a
    slowly-varying heading error, a marker that shifted — without
    needing a model for any of it.

    Contiguous slices of *time*, not interleaved: interleaving would put
    highly correlated neighbouring epochs in different groups and report a
    reassuringly tiny scatter that means nothing. Grouping by row order
    would do the same, or worse — rows arrive grouped by marker as often
    as by time — hence splitting on obs.t.
    """
    edges = np.quantile(obs.t, np.linspace(0.0, 1.0, n_groups + 1))
    edges[-1] = np.nextafter(edges[-1], np.inf)  # make the last slice closed
    estimates = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        mask = (obs.t >= lo) & (obs.t < hi)
        if not mask.any():
            continue
        # A group that happens to miss a marker entirely would leave that
        # marker's 6 parameters unconstrained; drop such groups rather
        # than report a singular solve.
        if len(np.unique(obs.marker_idx[mask])) < obs.n_markers:
            continue
        estimates.append(solve(obs.subset(mask), hpr_cb0, dxc_b, camera_matrix, dist_coeffs, **kwargs)["hpr_cb"])
    estimates = np.array(estimates)
    if len(estimates) < 2:
        return {"estimates": estimates, "mean": np.full(3, np.nan), "scatter": np.full(3, np.nan)}
    return {
        "estimates": estimates,
        "mean": estimates.mean(axis=0),
        # Standard error of the mean — the spread of the groups tells us
        # about a single group; the answer we quote uses all the data.
        "scatter": estimates.std(axis=0, ddof=1) / np.sqrt(len(estimates)),
    }
