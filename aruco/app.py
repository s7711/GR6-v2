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

import numpy as np
from flask import Flask, Response, abort, jsonify, request
from flask_sock import Sock
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.feed_client import FeedClient  # noqa: E402
from shared.frame_ipc import FrameReader  # noqa: E402
from shared.web import register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "oxts-nav"))
from ncomrx import machine_time_to_gps  # noqa: E402

import coords  # noqa: E402
import detection  # noqa: E402
import gad  # noqa: E402
import marker_map  # noqa: E402
import survey  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent / "boresight"))
from boresight import layouts as boresight_layouts  # noqa: E402
from boresight.service import BoresightService  # noqa: E402

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
        # For the shared header's Aruco status badge — see
        # shared/web/static/sysstatus.js. Loops back to this same
        # service, same as any other service's — harmless.
        "aruco_ws_url": service_url(browser_host, "aruco", scheme="ws") + "/ws/aruco",
        "drive_ws_url": service_url(browser_host, "drive", scheme="ws") + "/ws/drive",  # battery badge - see sysstatus.js
        "wheelspeed_ws_url": service_url(browser_host, "wheelspeed", scheme="ws") + "/ws/wheelspeed",  # "W" badge - see sysstatus.js
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
    # Not hard-real-time the way drive/oxts-nav's own critical threads
    # are (a late GAD send just means a slightly stale marker-position
    # aid, not a stuck motor command), but checked 2026-09-09 anyway
    # alongside them: no disk I/O here, and none should ever be added -
    # this loop's whole job is to notice a marker and call
    # gad_sender.send() promptly.
    camera_matrix, dist_coeffs = _wait_for_calibration()
    frame_reader = _get_frame_reader(wait=True)
    # Detection (and the GAD sends it triggers) is deliberately capped
    # below the camera's own frame rate — not for CPU (see Nice=5 in
    # robot-aruco.service.example for that), but because the Kalman
    # filter in the xNAV650 wants roughly-independent measurement
    # errors; sampling faster than this correlates consecutive errors
    # instead. See aruco-prd.md's "Detection rate limit" and Ben's note
    # (2026-07-27): 1-2Hz is already faster than needed. frame_reader
    # is a single-slot shared-memory latest-frame read, not a queue, so
    # skipping a tick here just means "read the same/next latest frame
    # next time" - nothing backs up or gets missed as a batch.
    min_period_s = 1.0 / service_cfg["max_detection_hz"]
    last_detection_time = None
    while True:
        if last_detection_time is not None:
            remaining = min_period_s - (time.monotonic() - last_detection_time)
            if remaining > 0:
                time.sleep(remaining)

        read = frame_reader.read()
        # frame_reader.read() only guards against the shared-memory
        # segment not existing yet (see _get_frame_reader) - it can
        # still return a technically-valid seqlock read whose width/
        # height are still zero, if the camera service has attached but
        # hasn't written its first real frame yet. That's a genuine
        # startup race, not a hypothetical: it crashed this exact loop
        # live 2026-08-09/10, on cv2.cvtColor's "!_src.empty()"
        # assertion, 6 seconds after this service started - killing
        # detection silently for the rest of the night, since an
        # uncaught exception in a background thread only takes down
        # that thread, leaving the page/websocket looking perfectly
        # alive with no new data. Treated the same as read is None:
        # skip this tick, try again next time.
        if read is None or read["frame"].size == 0:
            time.sleep(0.05)
            continue

        last_detection_time = time.monotonic()

        try:
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
                # Body-frame (X forward, Y right, Z down - vehicle
                # convention, not the raw camera frame) displacement to
                # every detected marker, mapped or not - independent of
                # nav/GNSS entirely (only needs the tvec plus the
                # camera's own static mounting calibration), for
                # waterbutt's GNSS-independent "distance from ideal" QC
                # check - see waterbutt-prd.md.
                debug[d["id"]] = {
                    "tvec_camera_frame": list(d["tvec"]),
                    "rvec_camera_frame": list(d["rvec"]),
                    "displacement_body_frame": (
                        np.array(dxc_b) + coords.displacement_camera_to_body(d["tvec"], hpr_cb)
                    ).tolist(),
                }

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
        except Exception:
            # Safety net, not the empty-frame fix's substitute - any
            # other unexpected error in a single frame's processing must
            # not be allowed to silently kill detection for hours the
            # way the empty-frame case did.
            logging.exception("[aruco] Detection loop hit an unexpected error, skipping this frame")


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


@app.route("/marker-map/<int:marker_id>/nudge", methods=["POST"])
def marker_map_nudge(marker_id):
    """Small live position corrections from the Markers page - see
    marker_map.nudge_marker. Applied and persisted immediately (no
    caching layer to invalidate - see marker_map.py's own docstring),
    so the very next detection picks up the new position."""
    body = request.get_json(force=True)
    updated = marker_map.nudge_marker(
        marker_map_path, marker_id,
        north_m=float(body.get("north_m", 0.0)),
        east_m=float(body.get("east_m", 0.0)),
        alt_m=float(body.get("alt_m", 0.0)),
    )
    if updated is None:
        abort(404)
    return jsonify(updated)


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


# --- Bore-sight calibration (see boresight/boresight-prd.md) ---
# Everything behind these routes lives in boresight/service.py, so the
# routes stay thin and the logic stays testable without a Flask app.

boresight = BoresightService(
    cfg=cfg,
    nav_client=nav_client,
    frame_reader_fn=lambda: _get_frame_reader(wait=False),
    detect_fn=lambda frame, k, d: detection.detect_markers(frame, k, d, size_for_id=_marker_size_lookup),
    marker_size=detection.DEFAULT_MARKER_SIZE,
    hpr_cb=hpr_cb,
    dxc_b=dxc_b,
    camera_cal_fn=lambda: detection.load_calibration(CAMERA_CAL_FILE),
    paths_dir=Path(__file__).resolve().parent.parent / cfg["services"]["navigate"]["paths_dir"],
    # Server-to-server on localhost, same convention jobs/app.py uses to
    # reach waterbutt — the bore-sight run drives the robot itself rather
    # than asking the operator to.
    navigate_base_url=f"http://localhost:{cfg['services']['navigate']['port']}",
    jobs_base_url=f"http://localhost:{cfg['services']['jobs']['port']}",
    max_detection_hz=service_cfg["max_detection_hz"],
)


@app.route("/boresight/build-path", methods=["POST"])
def boresight_build_path():
    body = request.get_json(force=True)
    return jsonify(boresight.build_capture_job(
        body.get("path", "Boresight limits"),
        float(body.get("height_m", boresight_layouts.RECOMMENDED_HEIGHT_M)),
        boresight_layouts.INS_HEIGHT_M,
    ))


@app.route("/boresight/run/start", methods=["POST"])
def boresight_run_start():
    body = request.get_json(force=True)
    return jsonify(boresight.start_run(
        body.get("path", "Boresight limits"),
        float(body.get("height_m", boresight_layouts.RECOMMENDED_HEIGHT_M)),
        boresight_layouts.INS_HEIGHT_M,
        passes=int(body.get("passes", 4)),
    ))


@app.route("/boresight/run/stop", methods=["POST"])
def boresight_run_stop():
    return jsonify(boresight.stop_run())


@app.route("/boresight/plan")
def boresight_plan():
    return jsonify(boresight.placement_plan(
        request.args.get("path", "Boresight limits"),
        float(request.args.get("height_m", boresight_layouts.RECOMMENDED_HEIGHT_M)),
        boresight_layouts.INS_HEIGHT_M,
    ))


@app.route("/boresight/guidance", methods=["POST"])
def boresight_guidance():
    return jsonify(boresight.placement_guidance(request.get_json(force=True)["targets"]))


@app.route("/boresight/capture/start", methods=["POST"])
def boresight_capture_start():
    return jsonify(boresight.start_capture())


@app.route("/boresight/capture/stop", methods=["POST"])
def boresight_capture_stop():
    return jsonify(boresight.stop_capture())


@app.route("/boresight/capture/status")
def boresight_capture_status():
    return jsonify(boresight.capture_status())


@app.route("/boresight/sessions")
def boresight_sessions():
    return jsonify(boresight.list_sessions())


@app.route("/boresight/solve", methods=["POST"])
def boresight_solve():
    body = request.get_json(force=True)
    return jsonify(boresight.solve_session(
        boresight.data_dir / body["session"],
        estimate_size_scale=bool(body.get("estimate_size_scale", True)),
    ))


@app.route("/boresight/apply", methods=["POST"])
def boresight_apply():
    from shared.config import CONFIG_PATH
    return jsonify(boresight.apply_to_config(CONFIG_PATH, request.get_json(force=True)["hpr_cb"]))


register_pages(app, PAGES_DIR, index_slug="home")


if __name__ == "__main__":
    nav_client.start()
    threading.Thread(target=_detection_loop, daemon=True).start()
    try:
        # threaded=True — see oxts-nav/app.py's app.run() comment: the
        # shared header's "A" badge now needs its own /ws/aruco
        # connection on top of whatever a page already opens for itself.
        app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        # Same reason as camera/app.py's cleanup: without this, Ctrl+C
        # leaves the attached shared-memory segment registered with this
        # process's resource_tracker, which prints a "leaked
        # shared_memory" warning on exit. Harmless, but silenced anyway.
        if _frame_reader is not None:
            _frame_reader.close()
