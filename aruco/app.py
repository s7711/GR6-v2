"""GR6-v2 aruco: detects ArUco markers in the camera service's live
frames, sends known-marker detections to the xNAV650 as GAD position/
heading updates, and provides pages for live viewing, a plan-view map,
and surveying new markers.

See aruco-prd.md for the requirements this implements.
"""

import io
import json
import logging
import sys
import threading
import time
from pathlib import Path

from flask import Flask, Response, jsonify, request
from flask_sock import Sock
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.feed_client import FeedClient  # noqa: E402
from shared.frame_ipc import FrameReader  # noqa: E402
from shared.web import register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "oxts-nav"))
from ncomrx import machine_time_to_gps  # noqa: E402

import detection  # noqa: E402
import gad  # noqa: E402
import marker_map  # noqa: E402
import survey  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
CAMERA_CAL_FILE = Path(__file__).resolve().parent.parent / "shared" / "camera-cal.yaml"
STATUS_HZ = 5  # /ws/aruco update rate — independent of camera_fps

cfg = load_config()
service_cfg = cfg["services"]["aruco"]
oxtsnav_cfg = cfg["services"]["oxts-nav"]
xnav_ip = cfg["xnav_ip"]

hpr_cb = tuple(service_cfg["camera_extrinsics"]["hpr_cb"])
dxc_b = tuple(service_cfg["camera_extrinsics"]["d_xc_b"])
hpr_ib = tuple(oxtsnav_cfg["hpr_ib"])
marker_map_path = Path(__file__).resolve().parent.parent / service_cfg["marker_map_file"]

_frame_reader = None
_frame_reader_lock = threading.Lock()


def _get_frame_reader(wait=False):
    """FrameReader() attaches to a shared-memory segment the camera
    service creates — it raises FileNotFoundError if camera isn't
    running yet, which used to crash aruco/app.py at import time with a
    cryptic multiprocessing traceback if camera happened not to be up.
    Lazy + retryable instead: `wait=True` (the detection loop) blocks
    and logs once; `wait=False` (an HTTP route) just returns None so the
    caller can respond 503 rather than hang the request."""
    global _frame_reader
    logged = False
    while True:
        with _frame_reader_lock:
            if _frame_reader is not None:
                return _frame_reader
            try:
                _frame_reader = FrameReader(cfg["services"]["camera"]["camera_shm_name"])
                if logged:
                    logging.info("[aruco] Connected to the camera frame feed")
                return _frame_reader
            except FileNotFoundError:
                if not wait:
                    return None
                if not logged:
                    logging.warning("[aruco] Waiting for the camera service to be running")
                    logged = True
        time.sleep(2.0)


nav_client = FeedClient(
    oxtsnav_cfg["nav_feed_socket"], default={"nav": {}, "status": {}, "connection": {}}
)
gad_sender = gad.GadSender(xnav_ip)

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)


@app.context_processor
def inject_urls():
    browser_host = request.host.split(":")[0]
    return {
        "manager_url": service_url(browser_host, "manager") + "/",
        "oxtsnav_ws_url": service_url(browser_host, "oxts-nav", scheme="ws") + "/ws/nav",
    }


class _SharedState:
    """Latest detection + GAD status, written by _detection_loop, read by
    the MJPEG generator and /ws/aruco. One writer, several readers — a
    plain lock is fine at this update rate (a handful of Hz), no need
    for the seqlock camera frames use at full frame rate."""

    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None  # BGR ndarray, with overlay already drawn
        self.visible_ids = []
        self.gad_status = None  # {"id": int, "at": float} of the last accepted GAD send
        self.unmapped = {}  # {id: {lat, lon}} rough single-shot estimate, for the Map page
        self.debug = {}  # {id: {...survey_marker's debug dict...}} for hand-verifying a real measurement
        # False until the first real frame has been through detection —
        # otherwise "visible_ids: []" (nothing seen yet) is indistinguishable
        # from "detection loop hasn't connected to the camera feed at all",
        # which is exactly the ambiguity that made this state confusing to
        # read from the Home page.
        self.running = False

    def update(self, frame, visible_ids, gad_status=None, unmapped=None, debug=None):
        with self.lock:
            self.frame = frame
            self.visible_ids = visible_ids
            if gad_status is not None:
                self.gad_status = gad_status
            self.unmapped = unmapped or {}
            self.debug = debug or {}
            self.running = True

    def snapshot(self):
        with self.lock:
            return {
                "running": self.running,
                "visible_ids": list(self.visible_ids),
                "gad_status": self.gad_status,
                "unmapped": dict(self.unmapped),
                "debug": dict(self.debug),
            }

    def latest_frame(self):
        with self.lock:
            return self.frame


state = _SharedState()


def _marker_size_lookup(marker_id):
    record = marker_map.find_marker(marker_map_path, marker_id)
    return record["size"] if record else None


def _wait_for_calibration():
    """cv2.FileStorage doesn't raise on a missing file — it logs its own
    warning and getNode(...).mat() just comes back None — so a one-shot
    load at startup would silently run detection with camera_matrix=None
    until a marker actually appeared, then fail confusingly deep inside
    cv2. shared/camera-cal.yaml doesn't exist until a calibration has been
    run on the camera service and promoted ("make active"), which won't
    have happened yet on a fresh checkout — so this is a real startup-
    ordering case to wait out, not just a defensive check."""
    logged = False
    while True:
        camera_matrix, dist_coeffs = detection.load_calibration(CAMERA_CAL_FILE)
        if camera_matrix is not None:
            return camera_matrix, dist_coeffs
        if not logged:
            logging.warning(
                "[aruco] Waiting for %s — run a calibration on the camera service and make it active",
                CAMERA_CAL_FILE,
            )
            logged = True
        time.sleep(2.0)


def _detection_loop():
    camera_matrix, dist_coeffs = _wait_for_calibration()
    frame_reader = _get_frame_reader(wait=True)
    while True:
        read = frame_reader.read()
        if read is None:
            time.sleep(0.05)
            continue

        frame = read["frame"]  # BGR byte order — see shared/frame_ipc.py
        detections = detection.detect_markers(frame, camera_matrix, dist_coeffs, size_for_id=_marker_size_lookup)

        gad_status = None
        unmapped = {}
        debug = {}
        nav_payload = nav_client.latest()
        connection = nav_payload.get("connection", {})
        nav = nav_payload.get("nav", {})
        gps_time = machine_time_to_gps(read["timestamp"], connection.get("timeOffset"))

        for d in detections:
            marker = marker_map.find_marker(marker_map_path, d["id"])
            if marker is not None:
                if gps_time is not None:
                    gad_sender.send(d, marker, gps_time[0], gps_time[1], hpr_cb, dxc_b, hpr_ib)
                    gad_status = {"id": d["id"], "at": time.time()}
            elif nav:
                # Not in the map yet — a rough, single-shot position
                # estimate for the Map page to show greyed out (see
                # aruco-prd.md's "unmapped marker" display). Not a
                # survey, just a "roughly here, go measure it properly"
                # hint — same limitation v1 had.
                try:
                    marker_debug = {}
                    est = survey.survey_marker(nav, d, hpr_cb, dxc_b, hpr_ib, debug=marker_debug)
                    unmapped[d["id"]] = {"lat": est["lat"], "lon": est["lon"]}
                    debug[d["id"]] = marker_debug
                except (KeyError, ZeroDivisionError, ValueError):
                    pass

        overlay = detection.draw_overlay(frame, detections, camera_matrix, dist_coeffs)
        state.update(overlay, [d["id"] for d in detections], gad_status, unmapped, debug)


def _mjpeg_generator(get_frame):
    while True:
        frame = get_frame()
        if frame is None:
            time.sleep(0.1)
            continue
        buf = io.BytesIO()
        # BGR -> RGB for display, same picamera2-quirk reversal camera/app.py uses.
        Image.fromarray(frame[:, :, ::-1]).save(buf, format="JPEG")
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.getvalue() + b"\r\n"


@app.route("/aruco.mjpg")
def aruco_mjpg():
    return Response(
        _mjpeg_generator(state.latest_frame),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        direct_passthrough=True,
    )


@sock.route("/ws/aruco")
def ws_aruco(ws):
    period = 1.0 / STATUS_HZ
    while True:
        ws.send(json.dumps(state.snapshot()))
        time.sleep(period)


@app.route("/marker-map")
def marker_map_route():
    # Re-read from disk on every request, deliberately no caching layer
    # — see aruco-prd.md ("marker map... re-read from disk"). The Map
    # and Markers pages fetch this once per page load, not polled.
    return jsonify(marker_map.load_markers(marker_map_path))


@app.route("/marker-map/<int:marker_id>", methods=["DELETE"])
def marker_map_delete(marker_id):
    return ("", 204) if marker_map.delete_marker(marker_map_path, marker_id) else ("", 404)


# --- Add Marker workflow: grab (freeze + candidate poses) -> cancel/save ---
# See aruco-prd.md ("Add Marker workflow: single-shot Grab, then Cancel
# or Save"). One global session — only one operator surveys at a time.

add_marker_lock = threading.Lock()
add_marker_state = {"grabbed": False, "frame": None, "candidates": {}}


@app.route("/add-marker/grab", methods=["POST"])
def add_marker_grab():
    frame_reader = _get_frame_reader(wait=False)
    if frame_reader is None:
        return "", 503  # camera service isn't up yet
    read = frame_reader.read()
    if read is None:
        return "", 503
    frame = read["frame"]
    camera_matrix, dist_coeffs = detection.load_calibration(CAMERA_CAL_FILE)
    if camera_matrix is None:
        return "", 503  # shared/camera-cal.yaml doesn't exist / isn't active yet
    detections = detection.detect_markers(frame, camera_matrix, dist_coeffs, size_for_id=_marker_size_lookup)
    nav = nav_client.latest().get("nav", {})

    candidates = {}
    if nav:
        for d in detections:
            try:
                candidates[d["id"]] = survey.survey_marker(nav, d, hpr_cb, dxc_b, hpr_ib)
            except (KeyError, ZeroDivisionError, ValueError):
                pass  # Missing/invalid nav fix this instant — that id just won't be offered to save

    overlay = detection.draw_overlay(frame, detections, camera_matrix, dist_coeffs)
    with add_marker_lock:
        add_marker_state["grabbed"] = True
        add_marker_state["frame"] = overlay
        add_marker_state["candidates"] = candidates
    return "", 204


@app.route("/add-marker/cancel", methods=["POST"])
def add_marker_cancel():
    with add_marker_lock:
        add_marker_state["grabbed"] = False
        add_marker_state["frame"] = None
        add_marker_state["candidates"] = {}
    return "", 204


@app.route("/add-marker/save", methods=["POST"])
def add_marker_save():
    body = request.get_json(force=True)
    marker_id = int(body["id"])
    size = float(body["size"])
    with add_marker_lock:
        candidate = add_marker_state["candidates"].get(marker_id)
        if candidate is None:
            return "", 404
        record = {**candidate, "id": marker_id, "size": size}
        add_marker_state["grabbed"] = False
        add_marker_state["frame"] = None
        add_marker_state["candidates"] = {}
    marker_map.upsert_marker(marker_map_path, record)
    return "", 204


@app.route("/add-marker.mjpg")
def add_marker_mjpg():
    def get_frame():
        with add_marker_lock:
            if add_marker_state["grabbed"]:
                return add_marker_state["frame"]
        return state.latest_frame()

    return Response(
        _mjpeg_generator(get_frame),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        direct_passthrough=True,
    )


@sock.route("/ws/add-marker")
def ws_add_marker(ws):
    period = 1.0 / STATUS_HZ
    while True:
        with add_marker_lock:
            payload = {
                "grabbed": add_marker_state["grabbed"],
                "candidates": add_marker_state["candidates"],
            }
        ws.send(json.dumps(payload))
        time.sleep(period)


register_pages(app, PAGES_DIR, index_slug="home")


if __name__ == "__main__":
    nav_client.start()
    threading.Thread(target=_detection_loop, daemon=True).start()
    try:
        app.run(host=service_cfg["host"], port=service_cfg["port"])
    except KeyboardInterrupt:
        pass
    finally:
        # Same reason as camera/app.py's cleanup: without this, Ctrl+C
        # leaves the attached shared-memory segment registered with this
        # process's resource_tracker, which prints a "leaked
        # shared_memory" warning on exit. Harmless, but silenced anyway.
        if _frame_reader is not None:
            _frame_reader.close()
