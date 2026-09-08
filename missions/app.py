"""GR6-v2 missions: sequences saved jobs into a mission — drives
jobs' existing HTTP API (start/stop/status), same "one process per
responsibility" boundary every other service follows: missions never
talks to navigate or drive directly, only to jobs. See
missions-prd.md for the requirements this implements.
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
from shared.web import register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

import missions as missions_module  # noqa: E402
from control import MissionRunner  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
MISSIONS_STATUS_HZ = 2
JOBS_TIMEOUT_S = 2.0

cfg = load_config()
service_cfg = cfg["services"]["missions"]
jobs_cfg = cfg["services"]["jobs"]

MISSIONS_DIR = Path(__file__).resolve().parent.parent / service_cfg["missions_dir"]
LOGS_DIR = MISSIONS_DIR / "logs"
JOBS_BASE_URL = f"http://localhost:{jobs_cfg['port']}"  # server-to-server, see navigate/app.py's own DRIVE_BASE_URL comment

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)


def start_job(name):
    # jobs' own /control/start returns either {"ok": False, "reason":
    # ...} (refused before even trying - bad start_index, no steps) or
    # the full status dict with no "ok" key at all (go() was called,
    # whatever its outcome). Normalised here to the same {"ok": bool,
    # "reason": ...} shape every other injected start_* callable in
    # this project returns, so MissionRunner doesn't need to know about
    # jobs' particular response shape.
    try:
        resp = requests.post(f"{JOBS_BASE_URL}/control/start", json={"name": name}, timeout=JOBS_TIMEOUT_S)
        data = resp.json()
    except requests.exceptions.RequestException:
        return {"ok": False, "reason": "couldn't reach jobs"}
    if data.get("ok") is False:
        return data
    if data.get("state") == "stopped_ok":
        # The job finished synchronously, inside this same call - e.g. a
        # single-step job whose only step was legitimately skipped (a
        # "fill" refused by waterbutt - see jobs-prd.md's "Water butt
        # fill refusal") rather than run. Not a failure to start: let
        # MissionRunner's own tick() pick this up as a normal completed
        # step, same as if it had watched the job run and finish. Found
        # live 2026-09-03 - "Fill with water" is exactly this shape, and
        # every refusal was aborting the whole mission over it.
        return {"ok": True}
    if data.get("state") != "running":
        # The job's own first step (or its own continuity/entry check)
        # already failed synchronously inside jobs' go() - surface that
        # as this step's own failure reason, same as a job step surfaces
        # navigate's own abort_reason.
        return {"ok": False, "reason": data.get("abort_reason") or f"jobs is unexpectedly {data.get('state')!r}"}
    return {"ok": True}


def stop_job():
    try:
        requests.post(f"{JOBS_BASE_URL}/control/stop", timeout=JOBS_TIMEOUT_S)
    except requests.exceptions.RequestException:
        logging.warning("[missions] Couldn't reach jobs to stop")


def job_status():
    try:
        resp = requests.get(f"{JOBS_BASE_URL}/control/status", timeout=JOBS_TIMEOUT_S)
        return resp.json()
    except requests.exceptions.RequestException:
        return {"state": "idle", "abort_reason": None}


runner = MissionRunner(start_job, stop_job, job_status)

_log_lock = threading.Lock()
_log_path = None
_logged_step_count = 0


def _start_new_log(mission_name):
    """Fresh log per mission run, same reasoning as jobs' own per-run
    log (see jobs-prd.md's "Logging") — an unattended mission-length
    run needs to be diagnosable after the fact, not just while someone
    happens to be watching."""
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
    """Run once at startup — logs accumulate, so old ones need actual
    cleanup (see jobs/app.py's own version of this)."""
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
    return {
        "manager_url": service_url(browser_host, "manager") + "/",
        # For the shared header's GNSS/Aruco/logging status badges — see
        # shared/web/static/sysstatus.js.
        "oxtsnav_ws_url": service_url(browser_host, "oxts-nav", scheme="ws") + "/ws/nav",
        "aruco_ws_url": service_url(browser_host, "aruco", scheme="ws") + "/ws/aruco",
        "map_manager_ws_url": service_url(browser_host, "map-manager", scheme="ws") + "/ws/map-manager",
        "drive_ws_url": service_url(browser_host, "drive", scheme="ws") + "/ws/drive",  # battery badge - see sysstatus.js
        "wheelspeed_ws_url": service_url(browser_host, "wheelspeed", scheme="ws") + "/ws/wheelspeed",  # "W" badge - see sysstatus.js
    }


# --- Jog proxy (Run page) ---
#
# Proxies through jobs (which itself proxies to navigate) rather than
# talking to navigate directly - missions never skips a layer, same
# "one process per responsibility" boundary as everywhere else in this
# project (see jobs/app.py's own jog_manual for the next hop down).
# Needed for the same reason jobs grew one: lining the robot up at
# wherever the next job's own first path actually starts, which a
# mission spanning several jobs won't always leave the robot sitting
# at already.


@app.route("/jog/manual", methods=["POST"])
def jog_manual():
    payload = request.get_json(force=True)
    try:
        resp = requests.post(
            f"{JOBS_BASE_URL}/jog/manual",
            json={"left_mps": payload["left_mps"], "right_mps": payload["right_mps"]},
            timeout=JOBS_TIMEOUT_S,
        )
    except requests.exceptions.RequestException:
        abort(502)
    return resp.content, resp.status_code, {"Content-Type": "application/json"}


# --- Jobs proxy (Create/Edit mission page's job dropdown, and the Run
# page's "next job" path preview) ---
#
# The browser fetches this, not jobs directly — same reasoning as
# jobs' own /api/navigate-paths proxy: a cross-origin fetch() is
# subject to CORS, and jobs doesn't send CORS headers.


@app.route("/api/available-jobs")
def api_available_jobs():
    try:
        resp = requests.get(f"{JOBS_BASE_URL}/api/jobs", timeout=JOBS_TIMEOUT_S)
    except requests.exceptions.RequestException:
        abort(502)
    return resp.content, resp.status_code, {"Content-Type": "application/json"}


@app.route("/api/jobs/<name>")
def api_job(name):
    """A job's own steps, so the Run page can find its first run_path
    step and preview that path — the thing the robot actually needs to
    be lined up against before starting from this job."""
    try:
        resp = requests.get(f"{JOBS_BASE_URL}/api/jobs/{name}", timeout=JOBS_TIMEOUT_S)
    except requests.exceptions.RequestException:
        abort(502)
    return resp.content, resp.status_code, {"Content-Type": "application/json"}


@app.route("/api/navigate-paths/<name>")
def api_navigate_path_points(name):
    """A path's actual lat/lon points, proxied one hop further through
    jobs' own already-existing proxy to navigate — same reasoning as
    api_job above."""
    try:
        resp = requests.get(f"{JOBS_BASE_URL}/api/navigate-paths/{name}", timeout=JOBS_TIMEOUT_S)
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
    payload = request.get_json(force=True)
    steps = payload["steps"]
    try:
        missions_module.save_mission(MISSIONS_DIR, name, steps)
    except missions_module.InvalidMissionName:
        abort(400)
    return jsonify({"warnings": []})


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


@app.route("/control/status")
def control_status():
    return jsonify(runner.status())


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
    threading.Thread(target=_tick_loop, daemon=True).start()

    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
