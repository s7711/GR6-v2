"""GR6-v2 navigate: records and drives paths — pure-pursuit path
following against `drive`'s /command/auto, live position from
`oxts-nav`'s feed, per-segment clearance tolerance (not one fixed
global). See navigate-prd.md for the requirements this implements.
"""

import json
import logging
import math
import sys
import threading
import time
from pathlib import Path

import requests
from flask import Flask, abort, jsonify, request
from flask_sock import Sock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.feed_client import FeedClient  # noqa: E402
from shared.web import register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

import geometry  # noqa: E402
import paths  # noqa: E402
from control import PathRunner  # noqa: E402
from feed import NavigateFeedServer  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
NAVIGATE_STATUS_HZ = 5
DRIVE_TIMEOUT_S = 0.5
MOVE_FORWARD_CLEARANCE_M = 0.5  # generous fixed clearance for this one internal segment — not authored into a saved path, so no need to prompt for it
MOVE_FORWARD_TIMEOUT_S = 20.0  # safety cap — should never actually take this long for a 2m nudge
DEBUG_LOG_INTERVAL_S = 1.0  # quiet background record of the most recent run, for debugging aborts after the fact — see navigate-prd.md

# The run/create-path pages poll /control/entry-check while the operator
# is lining the robot up — same journal-flooding concern as drive's jog
# page (see drive/app.py). Drop routine 200s to WARNING.
logging.getLogger("werkzeug").setLevel(logging.WARNING)

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)

cfg = load_config()
service_cfg = cfg["services"]["navigate"]
drive_cfg = cfg["services"]["drive"]
oxtsnav_cfg = cfg["services"]["oxts-nav"]

PATHS_DIR = Path(__file__).resolve().parent.parent / service_cfg["paths_dir"]
DRIVE_BASE_URL = f"http://localhost:{drive_cfg['port']}"  # server-to-server, same machine — not a browser-facing URL, see shared/web.py's service_url for that case
DEBUG_LOG_PATH = PATHS_DIR / "last_run_debug.jsonl"  # overwritten fresh at the start of each run — the *last* run only, not a growing history

CONTROL_CONFIG = {
    "entry_max_distance_m": service_cfg["entry_max_distance_m"],
    "entry_max_heading_deg": service_cfg["entry_max_heading_deg"],
    "lookahead_distance_m": service_cfg["lookahead_distance_m"],
    "heading_gain": service_cfg["heading_gain"],
    "cte_gain": service_cfg["cte_gain"],
    "localisation_accuracy_limit_m": service_cfg["localisation_accuracy_limit_m"],
    "max_heading_correction_deg": service_cfg["max_heading_correction_deg"],
    "wheel_base_m": drive_cfg["wheel_base_m"],
}


def send_velocity(left_mps, right_mps):
    try:
        requests.post(
            f"{DRIVE_BASE_URL}/command/auto",
            json={"left_mps": left_mps, "right_mps": right_mps},
            timeout=DRIVE_TIMEOUT_S,
        )
    except requests.exceptions.RequestException:
        logging.warning("[navigate] Couldn't reach drive to send a velocity command")


def send_pump(on):
    try:
        requests.post(f"{DRIVE_BASE_URL}/pump", json={"on": on}, timeout=DRIVE_TIMEOUT_S)
    except requests.exceptions.RequestException:
        logging.warning("[navigate] Couldn't reach drive to send a pump command")


nav_client = FeedClient(oxtsnav_cfg["nav_feed_socket"], default={"nav": {}, "status": {}, "connection": {}})
runner = PathRunner(CONTROL_CONFIG, send_velocity, send_pump)

_recording_lock = threading.Lock()
_recording_points = []


def _current_position():
    """{"lat":, "lon":, "heading_deg":, "horizontal_accuracy_m":} from
    oxts-nav's live feed (Lat/Lon there are radians; converted to
    degrees here since that's what geometry.py/paths.py both expect),
    or None if no fix has been received yet."""
    payload = nav_client.latest()
    nav = payload.get("nav", {})
    status = payload.get("status", {})
    if "Lat" not in nav or "Lon" not in nav or "Heading" not in nav:
        return None
    north_acc = status.get("NorthAcc")
    east_acc = status.get("EastAcc")
    horizontal_accuracy_m = (
        math.hypot(north_acc, east_acc) if north_acc is not None and east_acc is not None else None
    )
    return {
        "lat": math.degrees(nav["Lat"]),
        "lon": math.degrees(nav["Lon"]),
        "heading_deg": nav["Heading"],
        "horizontal_accuracy_m": horizontal_accuracy_m,
    }


def _reset_debug_log():
    """Called when a run starts — the log covers only the most recent
    run, not an ever-growing history (see navigate-prd.md)."""
    DEBUG_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    DEBUG_LOG_PATH.write_text("")


def _append_debug_log(position):
    entry = {"t": time.time(), **position, **runner.status()}
    with open(DEBUG_LOG_PATH, "a") as f:
        f.write(json.dumps(entry, default=str) + "\n")


def _control_loop():
    period = 1.0 / service_cfg["control_hz"]
    last_logged_state = None
    last_log_time = 0.0
    while True:
        position = _current_position()
        if position is not None:
            runner.step(
                position["lat"], position["lon"], position["heading_deg"], position["horizontal_accuracy_m"]
            )
            state = runner.status()["state"]
            now = time.monotonic()
            # Quiet ~1Hz snapshot while running, for debugging aborts
            # after the fact — plus always log the exact instant the
            # state changes (e.g. the step that triggered an abort),
            # regardless of the 1Hz gate, since that's the one line that
            # actually matters.
            if state != "idle" and (
                state != last_logged_state or now - last_log_time >= DEBUG_LOG_INTERVAL_S
            ):
                _append_debug_log(position)
                last_log_time = now
            last_logged_state = state
        time.sleep(period)


@app.context_processor
def inject_urls():
    browser_host = request.host.split(":")[0]
    return {
        "manager_url": service_url(browser_host, "manager") + "/",
        "oxtsnav_ws_url": service_url(browser_host, "oxts-nav", scheme="ws") + "/ws/nav",
    }


# --- Jog proxy (create-path page) ---
#
# The browser posts here, not directly to drive — a cross-origin fetch()
# (unlike the WebSockets everything else uses) is subject to CORS, and
# drive doesn't send CORS headers. Proxying server-to-server through
# navigate's own backend (same requests-based pattern already used for
# the control loop's send_velocity/send_pump) avoids needing to add CORS
# as a second cross-service mechanism — see feedback_prefer_existing_
# mechanisms in project memory.


@app.route("/jog/manual", methods=["POST"])
def jog_manual():
    payload = request.get_json(force=True)
    try:
        resp = requests.post(
            f"{DRIVE_BASE_URL}/command/manual",
            json={"left_mps": payload["left_mps"], "right_mps": payload["right_mps"]},
            timeout=DRIVE_TIMEOUT_S,
        )
    except requests.exceptions.RequestException:
        abort(502)
    return resp.content, resp.status_code, {"Content-Type": "application/json"}


# --- Path storage API ---


@app.route("/api/paths")
def api_list_paths():
    return jsonify(paths.list_paths(PATHS_DIR))


@app.route("/api/paths/<name>")
def api_get_path(name):
    try:
        return jsonify(paths.load_path(PATHS_DIR, name))
    except (FileNotFoundError, paths.InvalidPathName):
        abort(404)


@app.route("/api/paths/<name>", methods=["DELETE"])
def api_delete_path(name):
    try:
        paths.delete_path(PATHS_DIR, name)
    except (FileNotFoundError, paths.InvalidPathName):
        abort(404)
    return "", 204


# --- Recording (create-path page) ---


@app.route("/record/new", methods=["POST"])
def record_new():
    with _recording_lock:
        _recording_points.clear()
    return "", 204


@app.route("/record/drop", methods=["POST"])
def record_drop():
    position = _current_position()
    if position is None:
        abort(409)  # no position fix yet — nothing to drop
    payload = request.get_json(force=True)
    point = {
        "lat": position["lat"],
        "lon": position["lon"],
        "speed_mps": float(payload["speed_mps"]),
        "pump": bool(payload["pump"]),
        "clearance_m": float(payload["clearance_m"]),
    }
    with _recording_lock:
        _recording_points.append(point)
        count = len(_recording_points)
    return jsonify({"point_count": count, "point": point})


@app.route("/record/current")
def record_current():
    with _recording_lock:
        return jsonify(list(_recording_points))


@app.route("/record/save", methods=["POST"])
def record_save():
    payload = request.get_json(force=True)
    name = payload["name"]
    with _recording_lock:
        points = list(_recording_points)
    if len(points) < 2:
        abort(400)
    try:
        paths.save_path(PATHS_DIR, name, points)
    except paths.InvalidPathName:
        abort(400)
    return "", 204


@app.route("/record/forward", methods=["POST"])
def record_forward():
    """Drives forward `distance_m` in a straight line from the robot's
    current position/heading, under the same pure-pursuit control as a
    real path — much smoother than manual jogging for a precise nudge
    while recording. The synthetic path's actual endpoint is placed
    `distance_m + lookahead_distance_m` ahead, not just `distance_m`, so
    the controller has proper lookahead room throughout (a bare
    distance_m-long path would leave zero room for the shortest distance
    option, which is less than the configured lookahead distance) — see
    navigate-prd.md. The robot may therefore travel a little further
    than requested; that's fine, this doesn't need to be exact. Blocks
    until the maneuver finishes (aborts, completes, or times out) —
    the caller disables its button and shows a "moving" state meanwhile."""
    payload = request.get_json(force=True)
    distance_m = float(payload["distance_m"])
    speed_mps = float(payload["speed_mps"])

    position = _current_position()
    if position is None:
        return jsonify({"ok": False, "reason": "no position fix yet"})

    target_lat, target_lon = geometry.project_forward(
        position["lat"], position["lon"], position["heading_deg"],
        distance_m + service_cfg["lookahead_distance_m"],
    )
    synthetic_path = [
        {"lat": position["lat"], "lon": position["lon"],
         "speed_mps": speed_mps, "pump": False, "clearance_m": MOVE_FORWARD_CLEARANCE_M},
        {"lat": target_lat, "lon": target_lon,
         "speed_mps": speed_mps, "pump": False, "clearance_m": MOVE_FORWARD_CLEARANCE_M},
    ]

    nudge_runner = PathRunner(CONTROL_CONFIG, send_velocity, send_pump)
    nudge_runner.load_path(synthetic_path)
    start_result = nudge_runner.start(position["lat"], position["lon"], position["heading_deg"])
    if not start_result["ok"]:
        return jsonify(start_result)

    period = 1.0 / service_cfg["control_hz"]
    deadline = time.monotonic() + MOVE_FORWARD_TIMEOUT_S
    while nudge_runner.status()["state"] == "running" and time.monotonic() < deadline:
        time.sleep(period)
        current = _current_position()
        if current is not None:
            nudge_runner.step(current["lat"], current["lon"], current["heading_deg"], current["horizontal_accuracy_m"])

    result = nudge_runner.status()
    if result["state"] == "running":
        # Timed out — force stop. stop() resets state to "idle" and clears
        # abort_reason, so this must only run in the timeout case: calling
        # it unconditionally would overwrite a real "stopped_ok"/"aborted"
        # result the moment the loop above exits.
        nudge_runner.stop()
        result = nudge_runner.status()
    return jsonify(result)


# --- Path-following control ---


@app.route("/control/load/<name>", methods=["POST"])
def control_load(name):
    try:
        points = paths.load_path(PATHS_DIR, name)
    except (FileNotFoundError, paths.InvalidPathName):
        abort(404)
    runner.load_path(points)
    return "", 204


@app.route("/control/entry-check")
def control_entry_check():
    position = _current_position()
    if position is None:
        return jsonify({"ok": False, "reason": "no position fix yet"})
    return jsonify(runner.entry_check(position["lat"], position["lon"], position["heading_deg"]))


@app.route("/control/start", methods=["POST"])
def control_start():
    position = _current_position()
    if position is None:
        return jsonify({"ok": False, "reason": "no position fix yet"})
    result = runner.start(position["lat"], position["lon"], position["heading_deg"])
    if result["ok"]:
        _reset_debug_log()
    return jsonify(result)


@app.route("/control/stop", methods=["POST"])
def control_stop():
    runner.stop()
    return "", 204


def _snapshot():
    return runner.status()


@sock.route("/ws/navigate")
def ws_navigate(ws):
    period = 1.0 / NAVIGATE_STATUS_HZ
    while True:
        ws.send(json.dumps(_snapshot()))
        time.sleep(period)


# --- Pages ---


def run_context():
    return {"paths": paths.list_paths(PATHS_DIR)}


def paths_context():
    return {"paths": paths.list_paths(PATHS_DIR)}


def config_context():
    return {
        "control_hz": service_cfg["control_hz"],
        "entry_max_distance_m": service_cfg["entry_max_distance_m"],
        "entry_max_heading_deg": service_cfg["entry_max_heading_deg"],
        "lookahead_distance_m": service_cfg["lookahead_distance_m"],
        "heading_gain": service_cfg["heading_gain"],
        "cte_gain": service_cfg["cte_gain"],
        "localisation_accuracy_limit_m": service_cfg["localisation_accuracy_limit_m"],
        "max_heading_correction_deg": service_cfg["max_heading_correction_deg"],
        "wheel_base_m": drive_cfg["wheel_base_m"],
        "paths_dir": service_cfg["paths_dir"],
    }


register_pages(
    app,
    PAGES_DIR,
    index_slug="run",
    context_providers={
        "run": run_context,
        "paths": paths_context,
        "config": config_context,
    },
)


if __name__ == "__main__":
    nav_client.start()
    threading.Thread(target=_control_loop, daemon=True).start()

    feed = NavigateFeedServer(service_cfg["navigate_feed_socket"], _snapshot, service_cfg["navigate_feed_hz"])
    feed.start()

    app.run(host=service_cfg["host"], port=service_cfg["port"])
