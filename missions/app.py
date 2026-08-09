"""GR6-v2 missions: sequences saved navigate paths into a mission —
drives navigate's existing HTTP API (load/start/stop) and watches its
navigate_feed for real status, same "one process per responsibility"
boundary every other service follows: missions never talks to drive or
navigate's control loop directly. See missions-prd.md for the
requirements this implements.
"""

import datetime
import json
import logging
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

sys.path.append(str(Path(__file__).resolve().parent.parent / "navigate"))  # append, not insert(0) - see continuity.py's comment
import paths as navigate_paths  # noqa: E402

import continuity  # noqa: E402
import missions as missions_module  # noqa: E402
from control import MissionRunner  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
MISSIONS_STATUS_HZ = 2
NAVIGATE_TIMEOUT_S = 2.0

cfg = load_config()
service_cfg = cfg["services"]["missions"]
navigate_cfg = cfg["services"]["navigate"]
waterbutt_cfg = cfg["services"]["waterbutt"]

MISSIONS_DIR = Path(__file__).resolve().parent.parent / service_cfg["missions_dir"]
LOGS_DIR = MISSIONS_DIR / "logs"
NAVIGATE_PATHS_DIR = Path(__file__).resolve().parent.parent / navigate_cfg["paths_dir"]
NAVIGATE_BASE_URL = f"http://localhost:{navigate_cfg['port']}"  # server-to-server, see navigate/app.py's own DRIVE_BASE_URL comment
WATERBUTT_BASE_URL = f"http://localhost:{waterbutt_cfg['port']}"  # server-to-server - "fill" steps talk to waterbutt directly, it's a peer service like navigate, not owned by navigate the way drive is

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)

navigate_feed = FeedClient(navigate_cfg["navigate_feed_socket"], default={"state": "idle", "abort_reason": None})


def load_path(name):
    try:
        resp = requests.post(f"{NAVIGATE_BASE_URL}/control/load/{name}", timeout=NAVIGATE_TIMEOUT_S)
        return resp.json()
    except requests.exceptions.RequestException:
        return {"ok": False, "reason": "couldn't reach navigate"}


def start_path():
    try:
        resp = requests.post(f"{NAVIGATE_BASE_URL}/control/start", timeout=NAVIGATE_TIMEOUT_S)
        return resp.json()
    except requests.exceptions.RequestException:
        return {"ok": False, "reason": "couldn't reach navigate"}


def stop_path():
    try:
        requests.post(f"{NAVIGATE_BASE_URL}/control/stop", timeout=NAVIGATE_TIMEOUT_S)
    except requests.exceptions.RequestException:
        logging.warning("[missions] Couldn't reach navigate to stop")


def navigate_status():
    return navigate_feed.latest()


def pump_on(on):
    """For `water` steps - navigate owns the pump (see its /pump/manual),
    missions never talks to drive directly, same boundary as run_path
    steps."""
    try:
        resp = requests.post(f"{NAVIGATE_BASE_URL}/pump/manual", json={"on": on}, timeout=NAVIGATE_TIMEOUT_S)
        return resp.json()
    except requests.exceptions.RequestException:
        return {"ok": False, "reason": "couldn't reach navigate"}


def waterbutt_go(duration_s):
    """For `fill` steps - waterbutt is a peer service (like navigate),
    not something owned by another service, so missions can call it
    directly."""
    try:
        resp = requests.post(f"{WATERBUTT_BASE_URL}/go", json={"duration_s": duration_s}, timeout=NAVIGATE_TIMEOUT_S)
        if resp.status_code != 200:
            return {"ok": False, "reason": f"waterbutt refused duration_s={duration_s}"}
        return {"ok": True}
    except requests.exceptions.RequestException:
        return {"ok": False, "reason": "couldn't reach waterbutt"}


def waterbutt_stop():
    try:
        requests.post(f"{WATERBUTT_BASE_URL}/stop", timeout=NAVIGATE_TIMEOUT_S)
    except requests.exceptions.RequestException:
        logging.warning("[missions] Couldn't reach waterbutt to stop")


runner = MissionRunner(load_path, start_path, stop_path, navigate_status, pump_on, waterbutt_go, waterbutt_stop)

_log_lock = threading.Lock()
_log_path = None
_logged_step_count = 0


def _start_new_log(mission_name):
    """Fresh log per mission run, unlike navigate's own debug log
    (overwritten each run) — a mission run isn't watched live the way a
    single path run is, so there's nothing to compare a stale file
    against; see missions-prd.md's "Logging"."""
    global _log_path, _logged_step_count
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
    _log_path = LOGS_DIR / f"{mission_name}_{timestamp}.jsonl"
    _logged_step_count = 0
    _append_log({"event": "mission_start", "mission": mission_name})


def _append_log(entry: dict):
    if _log_path is None:
        return
    with _log_lock:
        with open(_log_path, "a") as f:
            f.write(json.dumps({"t": time.time(), **entry}, default=str) + "\n")


def _sweep_old_logs():
    """Run once at startup — logs accumulate (unlike navigate's single
    overwritten debug log), so old ones need actual cleanup."""
    if not LOGS_DIR.exists():
        return
    cutoff = time.time() - service_cfg["log_retention_days"] * 86400
    for file in LOGS_DIR.glob("*.jsonl"):
        if file.stat().st_mtime < cutoff:
            file.unlink()


def _tick_loop():
    global _logged_step_count
    period = 1.0 / service_cfg["mission_status_hz"]
    while True:
        runner.tick()
        status = runner.status()
        log = status["step_log"]
        for entry in log[_logged_step_count:]:
            _append_log({"event": "step_result", **entry})
        _logged_step_count = len(log)
        time.sleep(period)


@app.context_processor
def inject_urls():
    browser_host = request.host.split(":")[0]
    return {"manager_url": service_url(browser_host, "manager") + "/"}


# --- Navigate paths proxy (Create/Edit mission page's path dropdowns) ---
#
# The browser fetches this, not navigate directly — same reasoning as
# navigate's own /jog/manual proxy: a cross-origin fetch() is subject to
# CORS, and navigate doesn't send CORS headers. Proxying server-to-
# server avoids adding CORS as a second cross-service mechanism.


@app.route("/api/navigate-paths")
def api_navigate_paths():
    try:
        resp = requests.get(f"{NAVIGATE_BASE_URL}/api/paths", timeout=NAVIGATE_TIMEOUT_S)
    except requests.exceptions.RequestException:
        abort(502)
    return resp.content, resp.status_code, {"Content-Type": "application/json"}


@app.route("/api/navigate-paths/<name>")
def api_navigate_path_points(name):
    """A step's actual lat/lon points, for the Create/Edit mission map
    preview — same proxy reasoning as the list endpoint above."""
    try:
        resp = requests.get(f"{NAVIGATE_BASE_URL}/api/paths/{name}", timeout=NAVIGATE_TIMEOUT_S)
    except requests.exceptions.RequestException:
        abort(502)
    return resp.content, resp.status_code, {"Content-Type": "application/json"}


# --- Mission storage API ---


@app.route("/api/missions")
def api_list_missions():
    return jsonify(missions_module.list_missions(MISSIONS_DIR))


@app.route("/api/missions/<name>")
def api_get_mission(name):
    try:
        return jsonify(missions_module.load_mission(MISSIONS_DIR, name))
    except (FileNotFoundError, missions_module.InvalidMissionName):
        abort(404)


@app.route("/api/missions/<name>", methods=["POST"])
def api_save_mission(name):
    """Saves the mission, and reports (but doesn't block on) any step
    pair that navigate's own entry logic wouldn't accept back-to-back —
    see missions-prd.md's "Path continuity"."""
    payload = request.get_json(force=True)
    steps = payload["steps"]

    warnings = []
    for i in range(len(steps) - 1):
        if steps[i]["type"] != "run_path" or steps[i + 1]["type"] != "run_path":
            continue
        from_path, to_path = steps[i]["path"], steps[i + 1]["path"]
        try:
            points_a = navigate_paths.load_path(NAVIGATE_PATHS_DIR, from_path)
            points_b = navigate_paths.load_path(NAVIGATE_PATHS_DIR, to_path)
        except (FileNotFoundError, navigate_paths.InvalidPathName):
            continue
        result = continuity.check(
            points_a, points_b, navigate_cfg["entry_max_distance_m"], navigate_cfg["entry_max_heading_deg"]
        )
        if not result["ok"]:
            warnings.append({"after_step": i, "from_path": from_path, "to_path": to_path, **result})

    try:
        missions_module.save_mission(MISSIONS_DIR, name, steps)
    except missions_module.InvalidMissionName:
        abort(400)
    return jsonify({"warnings": warnings})


@app.route("/api/missions/<name>", methods=["DELETE"])
def api_delete_mission(name):
    try:
        missions_module.delete_mission(MISSIONS_DIR, name)
    except (FileNotFoundError, missions_module.InvalidMissionName):
        abort(404)
    return "", 204


# --- Mission control ---


@app.route("/control/start", methods=["POST"])
def control_start():
    payload = request.get_json(force=True)
    name = payload["name"]
    start_index = int(payload.get("start_index", 0))
    try:
        mission = missions_module.load_mission(MISSIONS_DIR, name)
    except (FileNotFoundError, missions_module.InvalidMissionName):
        abort(404)
    if not mission["steps"]:
        return jsonify({"ok": False, "reason": "mission has no steps"})
    if start_index < 0 or start_index >= len(mission["steps"]):
        return jsonify({"ok": False, "reason": "invalid start_index"})
    _start_new_log(name)
    runner.go(name, mission["steps"], start_index)
    return jsonify(runner.status())


@app.route("/control/stop", methods=["POST"])
def control_stop():
    _append_log({"event": "operator_stop"})
    runner.stop()
    return "", 204


@sock.route("/ws/missions")
def ws_missions(ws):
    period = 1.0 / MISSIONS_STATUS_HZ
    while True:
        ws.send(json.dumps(runner.status()))
        time.sleep(period)


# --- Pages ---


def run_context():
    return {"missions": missions_module.list_missions(MISSIONS_DIR)}


def missions_context():
    return {"missions": missions_module.list_missions(MISSIONS_DIR)}


register_pages(
    app,
    PAGES_DIR,
    index_slug="run",
    context_providers={
        "run": run_context,
        "missions": missions_context,
    },
)


if __name__ == "__main__":
    _sweep_old_logs()
    navigate_feed.start()
    threading.Thread(target=_tick_loop, daemon=True).start()

    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
