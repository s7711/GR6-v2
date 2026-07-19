"""Camera calibration: checkerboard target positions, per-frame detection,
and the calibration session state machine behind the Calibrate page.

Ported from `temp-cam-cal/main.py` (see camera-prd.md "Calibration
procedure") — the geometry/detection logic is carried over close to
unchanged; what's new for GR6-v2 is the session being a server-side
singleton (see camera-prd.md "Calibration page (Calibrate): session
lifecycle") instead of a one-shot blocking script.
"""

import csv
import json
import logging
import math
import shutil
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import matplotlib.path as mpltPath
import numpy as np
from scipy.spatial import ConvexHull

from bg_camera import RESOLUTION

DATA_DIR = Path(__file__).resolve().parent / "data"
ACTIVE_CAL_PATH = Path(__file__).resolve().parent.parent / "shared" / "camera-cal.yaml"
CAPTURE_LOG_NAME = "capture_log.csv"
CAPTURE_LOG_FIELDS = [
    "timestamp", "position", "message", "fit", "position_error", "shape_error", "alignment_error",
    "corners", "polygon",
]

CHECKERBOARD = (4, 6)  # Internal corners (columns, rows) — specific to the physical board used
CHECKERBOARD_SQUARE = 0.030  # metres
CORNER_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def generate_checkerboard_positions(
    imgx=RESOLUTION[0], imgy=RESOLUTION[1],  # Image size in pixels
    dx=0.0, dy=0.0, dz=0.5,  # Camera to checkerboard
    H=0.0, P=0.0, R=0.0,     # Heading, Pitch, Roll of checkerboard
    width=0.21, height=0.15,  # Size of checkerboard
    f=1500,                  # Focal length (pixels) — tuned for 1280x960
):
    """Returns the 4 corner points (image pixels) of where a checkerboard
    of the given size should appear for a given displacement/rotation
    from the camera — used both to draw the target polygon on the
    calibration page and to check a detected board against it."""
    cb = np.matrix([[dx - width / 2.0, dy + height / 2.0, 0.0],
                    [dx + width / 2.0, dy + height / 2.0, 0.0],
                    [dx + width / 2.0, dy - height / 2.0, 0.0],
                    [dx - width / 2.0, dy - height / 2.0, 0.0]]).transpose()

    cosH, sinH = math.cos(H * math.pi / 180.0), math.sin(H * math.pi / 180.0)
    Ry = np.matrix([[cosH, 0, -sinH], [0, 1, 0], [sinH, 0, cosH]])
    cosP, sinP = math.cos(P * math.pi / 180.0), math.sin(P * math.pi / 180.0)
    Rx = np.matrix([[1, 0, 0], [0, cosP, sinP], [0, -sinP, cosP]])
    cosR, sinR = math.cos(R * math.pi / 180.0), math.sin(R * math.pi / 180.0)
    Rz = np.matrix([[cosR, sinR, 0], [-sinR, cosR, 0], [0, 0, 1]])

    cbw = Rz.dot(Rx).dot(Ry).dot(cb)
    cbc = cbw.copy()
    cbc[2, :] += dz

    TL = (imgx / 2 + cbc[0, 0] / cbc[2, 0] * f, imgy / 2 + cbc[1, 0] / cbc[2, 0] * f)
    TR = (imgx / 2 + cbc[0, 1] / cbc[2, 1] * f, imgy / 2 + cbc[1, 1] / cbc[2, 1] * f)
    BR = (imgx / 2 + cbc[0, 2] / cbc[2, 2] * f, imgy / 2 + cbc[1, 2] / cbc[2, 2] * f)
    BL = (imgx / 2 + cbc[0, 3] / cbc[2, 3] * f, imgy / 2 + cbc[1, 3] / cbc[2, 3] * f)
    return [TL, TR, BR, BL]


def _build_target_positions():
    positions = []

    # Flat (no rotation), 3x3 grid
    positions.append(generate_checkerboard_positions(dx=-0.09, dy=-0.07))
    positions.append(generate_checkerboard_positions(dx=0.0, dy=-0.07))
    positions.append(generate_checkerboard_positions(dx=0.09, dy=-0.07))
    positions.append(generate_checkerboard_positions(dx=-0.09, dy=0.0))
    positions.append(generate_checkerboard_positions(dx=0.0, dy=0.0))
    positions.append(generate_checkerboard_positions(dx=0.09, dy=0.0))
    positions.append(generate_checkerboard_positions(dx=-0.09, dy=0.07))
    positions.append(generate_checkerboard_positions(dx=0.0, dy=0.07))
    positions.append(generate_checkerboard_positions(dx=0.09, dy=0.07))

    # Turned + rolled (heading and roll varied, pitch held at 0)
    positions.append(generate_checkerboard_positions(dx=-0.04, dy=-0.06, H=10, R=20))
    positions.append(generate_checkerboard_positions(dx=0.05, dy=-0.05, H=20, R=30))
    positions.append(generate_checkerboard_positions(dx=0.16, dy=-0.06, H=30, R=-10))
    positions.append(generate_checkerboard_positions(dx=-0.10, dy=0.0, H=-20, R=20))
    positions.append(generate_checkerboard_positions(dx=0.02, dy=0.0, H=-10, R=-30))
    positions.append(generate_checkerboard_positions(dx=0.06, dy=0.0, H=-30, R=10))
    positions.append(generate_checkerboard_positions(dx=-0.04, dy=0.06, H=20, R=-20))
    positions.append(generate_checkerboard_positions(dx=0.0, dy=0.06, H=-20, R=0))
    positions.append(generate_checkerboard_positions(dx=0.07, dy=0.01, H=-10, R=-10))

    # Turned + pitched (heading and pitch varied, roll held at 0)
    positions.append(generate_checkerboard_positions(dx=-0.06, dy=-0.07, H=10, P=20))
    positions.append(generate_checkerboard_positions(dx=0.0, dy=-0.06, H=20, P=-20))
    positions.append(generate_checkerboard_positions(dx=0.18, dy=-0.08, H=30, P=10))
    positions.append(generate_checkerboard_positions(dx=-0.10, dy=0.0, H=-20, P=30))
    positions.append(generate_checkerboard_positions(dx=0.0, dy=0.0, H=-10, P=-20))
    positions.append(generate_checkerboard_positions(dx=0.06, dy=0.0, H=-30, P=10))
    positions.append(generate_checkerboard_positions(dx=-0.05, dy=0.06, H=20, P=20))
    positions.append(generate_checkerboard_positions(dx=0.0, dy=0.06, H=-20, P=30))
    positions.append(generate_checkerboard_positions(dx=0.08, dy=0.08, H=-10, P=-30))

    # Distance variation — see camera-prd.md "Addition over the existing
    # position set: distance variation". Everything above is at the same
    # dz=0.5; these add closer and farther depths so the calibration
    # isn't fit against a single fixed scale.
    #
    # dz=0.3 (originally tried) puts the board at ~1050px wide in a
    # 1280px-wide frame — only ~115px margin each side, so even a modest
    # dx offset pushes it off-frame entirely (confirmed by testing: see
    # camera-prd.md). dz=0.4 leaves a safer ~245px margin.
    positions.append(generate_checkerboard_positions(dx=-0.03, dy=0.0, dz=0.4))
    positions.append(generate_checkerboard_positions(dx=0.0, dy=0.0, dz=0.4))
    positions.append(generate_checkerboard_positions(dx=0.03, dy=0.0, dz=0.4))
    positions.append(generate_checkerboard_positions(dx=-0.15, dy=0.0, dz=1.3))
    positions.append(generate_checkerboard_positions(dx=0.0, dy=0.0, dz=1.3))
    positions.append(generate_checkerboard_positions(dx=0.15, dy=0.0, dz=1.3))

    return positions


TARGET_POSITIONS = _build_target_positions()


# cv2.findChessboardCorners only ever finds the INTERNAL corners, which
# are inherently inset from the board's printed outer edge by one square
# on each side — so their convex hull can never reach the full area of a
# polygon representing the board's outer edge, even for a mathematically
# perfect placement. For a (cols, rows) internal-corner board:
#   internal-corner extent = (cols-1) x (rows-1) squares
#   full printed board     = (cols+1) x (rows+1) squares
# MAX_FIT_RATIO is a naive estimate of that ceiling (~0.429 for this 4x6
# board), assuming a purely affine (fronto-parallel) mapping — the
# ORIGINAL ratio check compared against a flat 0.7 threshold, which is
# *above* even this naive ceiling and could never be satisfied by any
# real placement. Found via testing (both a geometric proof and a real
# checkerboard that wouldn't trigger "Ok" no matter how well-placed).
#
# This estimate isn't a hard ceiling in practice, though — tilted poses
# involve genuine perspective (non-affine) distortion that doesn't
# preserve this square-unit ratio exactly, and sub-pixel corner
# refinement adds its own small effect; testing found real "good" fits
# scoring well above 1.0 (e.g. 1.5) rather than approaching it from
# below. So `fit` below is "higher is better, 0.8 needed to pass" —
# useful as a live, monotonic diagnostic — not a literal percentage of
# a precisely-known maximum.
MAX_FIT_RATIO = ((CHECKERBOARD[0] - 1) * (CHECKERBOARD[1] - 1)) / ((CHECKERBOARD[0] + 1) * (CHECKERBOARD[1] + 1))
FIT_PASS_THRESHOLD = 0.8

# Position/orientation strictness — added after testing showed containment
# + size alone isn't enough: adjacent target positions can be close enough
# in image-space that a barely-moved board satisfies both in a row, and —
# more surprisingly — a completely FLAT board at the right position/size
# can satisfy a TILTED target's containment+size checks almost as well as
# an actually-tilted one, since tilting shifts a target polygon's centroid
# only slightly (confirmed by testing: ~20px out of ~600px board width for
# a 25°/20°/15° H/P/R tilt) even though the polygon's SHAPE changes a lot.
# So two independent checks, each catching a different kind of mismatch a
# plain containment+area check can't:
#   position_error — detected-corners centroid vs polygon centroid,
#                     normalized by polygon scale. Catches "right shape,
#                     wrong place" (e.g. an adjacent grid position).
#   shape_error     — normalized (centred, unit-scale) 4-corner shape
#                      compared to the target polygon's shape, trying
#                      every relabelling of the 4 extracted corners
#                      needed to cover two DIFFERENT sources of ambiguity
#                      (conflated in an earlier version of this fix — see
#                      camera-prd.md for how a real test run exposed it):
#                      (a) the checkerboard's own coloring symmetry
#                      (identity/180°/either mirror — this board's 5x7
#                      squares are both odd, so a single-axis mirror also
#                      preserves its colouring, not just 180°), and
#                      (b) `extreme_corners` below is extracted from
#                      cv2's row-major corner order, which can start at
#                      ANY of the 4 physical corners and trace around in
#                      EITHER direction depending on fine detection
#                      details — a full 8-element (4 rotations x 2
#                      directions) dihedral group, not the 4-element
#                      subgroup (a) alone. Catches "right place, wrong
#                      tilt" (e.g. a flat board held where a tilted
#                      target was asked for).
# Both are 0 for an exact match, growing for a mismatch — combined into
# one ALIGNMENT_ERROR for the live display, since showing 3+ separate
# numbers on the page would be more clutter than help (the full breakdown
# is written to each session's capture_log.csv instead — see
# CalibrationSession._run). First-attempt threshold, deliberately picked
# on the loose side and loosened further after a real test run (see
# below): this can only be properly validated against a real camera (the
# synthetic test images used during development can't faithfully
# reproduce a true tilted perspective — see camera-prd.md — so a
# stricter value risked being un-satisfiable in practice the same way the
# old "too small" threshold was). Loosen/tighten based on real numbers
# from capture_log.csv, rather than guessing further.
ALIGNMENT_ERROR_THRESHOLD = 0.5
_SYMMETRIC_RELABELLINGS = (
    [0, 1, 2, 3], [1, 2, 3, 0], [2, 3, 0, 1], [3, 0, 1, 2],  # 4 rotations
    [3, 2, 1, 0], [0, 3, 2, 1], [1, 0, 3, 2], [2, 1, 0, 3],  # + each reversed
)


def _normalize_quad(points):
    points = np.asarray(points, dtype=float)
    centred = points - points.mean(axis=0)
    scale = np.sqrt((centred ** 2).sum() / len(points))
    return centred / scale if scale > 0 else centred


def _expected_inner_quad(polygon):
    """The 4 extreme internal-corner positions a perfectly-placed board
    would project to — NOT the same as `polygon` itself, which is the
    full printed board's outer edge. Approximated via bilinear
    interpolation of the outer edge's 4 corners (exact for a flat,
    fronto-parallel target; an approximation for tilted ones, since true
    perspective isn't bilinear — but much closer than comparing against
    the outer edge directly).

    Found by testing: comparing detected inner corners' shape against the
    outer polygon's shape directly is comparing two rectangles of
    genuinely different aspect ratio (inner-corner extent is
    (cols-1)x(rows-1) squares, the full board is (cols+1)x(rows+1)) — a
    real, systematic bias present even at perfect alignment, not just
    detection noise (confirmed geometrically: ~0.08 from this mismatch
    alone, before this fix)."""
    tl, tr, br, bl = (np.asarray(p, dtype=float) for p in polygon)
    cols, rows = CHECKERBOARD
    sw = 1.0 / (cols + 1)  # Inset fraction along the TL->TR ("width") edge
    sh = 1.0 / (rows + 1)  # Inset fraction along the TL->BL ("height") edge

    def bilerp(s, t):
        return (1 - s) * (1 - t) * tl + s * (1 - t) * tr + s * t * br + (1 - s) * t * bl

    return [bilerp(sw, sh), bilerp(1 - sw, sh), bilerp(1 - sw, 1 - sh), bilerp(sw, 1 - sh)]


def _shape_error(detected_extreme_corners, expected_inner_quad):
    a = _normalize_quad(detected_extreme_corners)
    b = _normalize_quad(expected_inner_quad)
    return min(np.sqrt(((a - b[order]) ** 2).sum(axis=1)).mean() for order in _SYMMETRIC_RELABELLINGS)


def check_checkerboard(frame_bgr, polygon):
    """Looks for the checkerboard in a frame and checks it against the
    target polygon. Returns (message, corners_or_None, fit_or_None,
    alignment_error_or_None, components_or_None) where message is one of
    "No checkerboard" / "Align checkerboard" / "Too small (...)" / "Ok".
    fit is a live size score (higher is better, 0.8 to pass — see the
    note above on why it isn't a strict 0-1 percentage); alignment_error
    is a combined position+orientation mismatch score (lower is better,
    must be under ALIGNMENT_ERROR_THRESHOLD to pass); components is a
    dict with the underlying position_error/shape_error split out, for
    logging (not shown live — see note above)."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if not found:
        return "No checkerboard", None, None, None, None

    corner_points = corners.reshape(-1, 2)
    grid = corners.reshape(CHECKERBOARD[1], CHECKERBOARD[0], 2)
    extreme_corners = [grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]]
    expected_inner_quad = _expected_inner_quad(polygon)

    polygon_hull = ConvexHull(polygon)
    corners_hull = ConvexHull(corner_points)
    fit = (corners_hull.area / polygon_hull.area) / MAX_FIT_RATIO

    expected_centroid = np.mean(expected_inner_quad, axis=0)
    corner_centroid = np.mean(corner_points, axis=0)
    position_error = np.linalg.norm(corner_centroid - expected_centroid) / np.sqrt(polygon_hull.area)
    shape_error = _shape_error(extreme_corners, expected_inner_quad)
    alignment_error = position_error + shape_error
    components = {"position_error": position_error, "shape_error": shape_error}

    polygon_path = mpltPath.Path(polygon)
    contained = all(polygon_path.contains_points(corner_points))

    if not contained or alignment_error > ALIGNMENT_ERROR_THRESHOLD:
        return "Align checkerboard", corners, fit, alignment_error, components

    if fit < FIT_PASS_THRESHOLD:
        return f"Too small ({fit:.2f})", corners, fit, alignment_error, components

    return "Ok", corners, fit, alignment_error, components


def draw_overlay(frame_bgr, polygon, corners=None):
    """Draws the target polygon (and detected corners, if any) on a copy
    of the frame, mirrored — mirroring makes positioning the checkerboard
    by hand much easier, carried over from the original code."""
    img = frame_bgr.copy()
    cv2.polylines(img, [np.array(polygon, dtype=np.int32)], True, (0, 255, 255), 3)
    if corners is not None:
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners, True)
    return np.fliplr(img)


class CalibrationSession:
    """Server-side singleton owning the current (or just-finished)
    calibration session — not tied to any browser connection. See
    camera-prd.md "Calibration page (Calibrate): session lifecycle"."""

    def __init__(self, cam):
        self.cam = cam
        self.lock = threading.Lock()
        self.state = "idle"  # idle | capturing | computing | done
        self.position = 0
        self.message = ""
        self.session_dir = None
        self.result = None
        self.fit = None
        self.alignment_error = None
        self._last_polygon = None
        self._last_corners = None

    def snapshot(self):
        with self.lock:
            return {
                "state": self.state,
                "position": self.position,
                "count": len(TARGET_POSITIONS),
                "message": self.message,
                "fit": self.fit,
                "alignment_error": self.alignment_error,
                "result": self.result,
            }

    def overlay_frame(self):
        with self.lock:
            active = self.state == "capturing"
            polygon = self._last_polygon
            corners = self._last_corners
        frame, *_ = self.cam.latest()
        if active and polygon is not None:
            return draw_overlay(frame, polygon, corners)
        return np.fliplr(frame)

    def start(self):
        with self.lock:
            if self.state in ("capturing", "computing"):
                return  # Already running — Start is disabled client-side for this reason
            DATA_DIR.mkdir(exist_ok=True)
            self.session_dir = DATA_DIR / f"cal_{datetime.now():%y%m%d_%H%M%S}"
            self.session_dir.mkdir()
            with open(self.session_dir / CAPTURE_LOG_NAME, "w", newline="") as f:
                csv.writer(f).writerow(CAPTURE_LOG_FIELDS)
            self.position = 0
            self.message = "Starting…"
            self.result = None
            self.fit = None
            self.alignment_error = None
            self._last_polygon = None
            self._last_corners = None
            self.state = "capturing"
        threading.Thread(target=self._run, daemon=True).start()

    def abort(self):
        # Deliberately does NOT delete session_dir. Originally it did —
        # "no calibration.yaml, so nothing worth keeping" — but that
        # stopped being true once capture_log.csv existed: aborting is
        # often *exactly* when there's something worth looking at (a
        # position that wouldn't trigger), and deleting the folder
        # destroyed the one diagnostic that could explain why. Consistent
        # with camera-prd.md's "let camera/data/ grow" decision — an
        # aborted session's leftovers cost disk space, nothing else.
        with self.lock:
            if self.state not in ("capturing", "computing"):
                return
            self.session_dir = None
            self.state = "idle"
            self.message = ""

    def promote(self):
        with self.lock:
            if self.state != "done" or self.session_dir is None:
                return False
            src = self.session_dir / "calibration.yaml"
        shutil.copyfile(src, ACTIVE_CAL_PATH)
        return True

    @staticmethod
    def _log_frame(session_dir, position, message, corners, polygon, fit, components, alignment_error):
        # Full geometry (not just the scalar scores) for every frame that
        # got far enough to be checked at all — added after two rounds of
        # scalar-only numbers not being enough to explain a real failure.
        # With the actual corner/polygon points, position/shape/fit can
        # all be independently recomputed and plotted after the fact,
        # rather than needing another guess-and-retest cycle.
        corners_json = json.dumps(corners.reshape(-1, 2).tolist()) if corners is not None else ""
        polygon_json = json.dumps([list(p) for p in polygon])
        with open(session_dir / CAPTURE_LOG_NAME, "a", newline="") as f:
            csv.writer(f).writerow([
                time.time(), position, message,
                fit if fit is not None else "",
                components["position_error"] if components else "",
                components["shape_error"] if components else "",
                alignment_error if alignment_error is not None else "",
                corners_json,
                polygon_json,
            ])

    def _run(self):
        while True:
            with self.lock:
                if self.state != "capturing":
                    return  # Aborted
                position = self.position
                session_dir = self.session_dir
            if position >= len(TARGET_POSITIONS):
                break

            frame, *_ = self.cam.latest()
            polygon = TARGET_POSITIONS[position]
            message, corners, fit, alignment_error, components = check_checkerboard(frame, polygon)

            # Aborted while we were busy checking this frame — session_dir
            # may already be gone; nothing more to do.
            with self.lock:
                if self.state != "capturing":
                    return
                self.message = message
                self.fit = fit
                self.alignment_error = alignment_error
                self._last_polygon = polygon
                self._last_corners = corners

            is_ok = message == "Ok"

            # abort() can still race in right around here (it runs on a
            # different thread, outside this lock, and can rmtree
            # session_dir at any time) — treat that as "session ended",
            # not a crash: an unhandled exception here would silently kill
            # this daemon thread and leave the session stuck in
            # "capturing" forever, worse than just stopping cleanly.
            try:
                self._log_frame(session_dir, position, message, corners, polygon, fit, components, alignment_error)
                if is_ok:
                    cv2.imwrite(str(session_dir / f"image{position}.jpg"), frame)
            except OSError:
                return

            if is_ok:
                with self.lock:
                    if self.state != "capturing":
                        return
                    self.position += 1

        with self.lock:
            if self.state != "capturing":
                return
            self.state = "computing"
            self.message = "Computing calibration…"
            session_dir = self.session_dir

        self._compute(session_dir)

    def _compute(self, session_dir):
        objpoints, imgpoints = [], []
        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * CHECKERBOARD_SQUARE

        gray_shape = None
        for path in sorted(session_dir.glob("image*.jpg"), key=lambda p: int(p.stem[len("image"):])):
            img = cv2.imread(str(path))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray_shape = gray.shape[::-1]
            found, corners = cv2.findChessboardCorners(
                gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            if found:
                objpoints.append(objp)
                imgpoints.append(cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CORNER_CRITERIA))

        rms, mtx, dist, _rvecs, _tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)

        cv_file = cv2.FileStorage(str(session_dir / "calibration.yaml"), cv2.FILE_STORAGE_WRITE)
        cv_file.write("camera_matrix", mtx)
        cv_file.write("distortion_coefficients", dist)
        cv_file.write("resolution", np.array(RESOLUTION))
        cv_file.write("rms_reprojection_error", rms)
        cv_file.release()

        logging.info("[calibration] Done: RMS reprojection error %.4f px over %d images", rms, len(imgpoints))

        with self.lock:
            self.result = {
                "rms": rms,
                "image_count": len(imgpoints),
                "resolution": f"{RESOLUTION[0]}x{RESOLUTION[1]}",
            }
            self.state = "done"
            self.message = "Done"
