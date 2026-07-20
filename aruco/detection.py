"""ArUco marker detection: find markers in a frame and estimate each
one's pose relative to the camera. Deliberately has no knowledge of nav
data, the marker map, or GAD — see gad.py and marker_map.py for what's
built on top of this. See aruco-prd.md ("Detection").
"""

from pathlib import Path

import cv2
import numpy as np

DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
DEFAULT_MARKER_SIZE = 0.097  # metres — fallback only, when a detected id isn't in the marker map yet

_params = cv2.aruco.DetectorParameters()
_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
_detector = cv2.aruco.ArucoDetector(DICTIONARY, _params)


def load_calibration(path):
    """Read camera_matrix/distortion_coefficients as written by
    camera/calibration.py's cv2.FileStorage — same file format,
    consumed directly (see aruco-prd.md, no new calibration convention)."""
    path = Path(path)
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    try:
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("distortion_coefficients").mat()
    finally:
        fs.release()
    return camera_matrix, dist_coeffs


def detect_markers(frame_bgr, camera_matrix, dist_coeffs, size_for_id=None):
    """Detect every marker in frame_bgr and estimate its pose.

    size_for_id: optional callable(marker_id) -> size in metres, e.g. a
    marker_map.py lookup. Falls back to DEFAULT_MARKER_SIZE for an id it
    doesn't know (an unmapped marker) — same "have to assume a size"
    situation v1 never resolved either, see aruco-prd.md Out of Scope.

    estimatePoseSingleMarkers takes one marker length per call, so a
    marker is processed on its own rather than all-at-once, to let each
    one use its own map-configured size.

    Returns a list of dicts: {id, corners (4x2 pixel array), rvec, tvec}.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _rejected = _detector.detectMarkers(gray)
    detections = []
    if ids is None:
        return detections

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)
        size = size_for_id(marker_id) if size_for_id else None
        if size is None:
            size = DEFAULT_MARKER_SIZE
        rvecs, tvecs, _obj_points = cv2.aruco.estimatePoseSingleMarkers(
            [marker_corners], size, camera_matrix, dist_coeffs
        )
        detections.append(
            {
                "id": marker_id,
                "corners": marker_corners.reshape(-1, 2),
                "rvec": rvecs[0][0],
                "tvec": tvecs[0][0],
                "size": size,
            }
        )
    return detections


def draw_overlay(frame_bgr, detections, camera_matrix, dist_coeffs):
    """Return a copy of frame_bgr with marker outlines, ids, and pose
    axes drawn on — for the live MJPEG feed and the frozen Add Marker
    "grab" preview."""
    overlay = frame_bgr.copy()
    if not detections:
        return overlay

    corners = [np.expand_dims(d["corners"].astype(np.float32), axis=0) for d in detections]
    ids = np.array([[d["id"]] for d in detections])
    cv2.aruco.drawDetectedMarkers(overlay, corners, ids)

    for d in detections:
        cv2.drawFrameAxes(overlay, camera_matrix, dist_coeffs, d["rvec"], d["tvec"], d["size"] * 0.75)

    return overlay
