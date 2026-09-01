"""GR6-v2 navigate: records and drives paths — pure-pursuit path
following against `drive`'s /command/auto, live position from
`oxts-nav`'s feed, per-segment clearance tolerance (not one fixed
global). See navigate-prd.md for the requirements this implements.
"""

import datetime
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
from turn_control import TurnRunner  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
NAVIGATE_STATUS_HZ = 5
DRIVE_TIMEOUT_S = 0.5
OXTSNAV_TIMEOUT_S = 0.5
MOVE_FORWARD_CLEARANCE_M = 0.5  # generous fixed clearance for this one internal segment — not authored into a saved path, so no need to prompt for it
MOVE_FORWARD_TIMEOUT_S = 20.0  # safety cap — should never actually take this long for a 2m nudge

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
OXTSNAV_BASE_URL = f"http://localhost:{oxtsnav_cfg['port']}"  # same — see "Aruco priority" in navigate-prd.md
LOGS_DIR = PATHS_DIR / "logs"  # one retained file per run — see jobs'/missions' identical convention; viewed via the "viewer" service now, see viewer-prd.md

CONTROL_CONFIG = {
    "entry_max_distance_m": service_cfg["entry_max_distance_m"],
    "entry_max_heading_deg": service_cfg["entry_max_heading_deg"],
    "lookahead_distance_m": service_cfg["lookahead_distance_m"],
    "heading_gain": service_cfg["heading_gain"],
    "cte_gain": service_cfg["cte_gain"],
    "localisation_accuracy_limit_m": service_cfg["localisation_accuracy_limit_m"],
    "max_heading_correction_deg": service_cfg["max_heading_correction_deg"],
    "wheel_base_m": drive_cfg["wheel_base_m"],
    "stall_check_window_s": service_cfg["stall_check_window_s"],
    "stall_min_distance_m": service_cfg["stall_min_distance_m"],
}

TURN_CONFIG = {
    "wheel_base_m": drive_cfg["wheel_base_m"],
    "turn_gain": service_cfg["turn_gain"],
    "turn_max_rate_rad_s": service_cfg["turn_max_rate_rad_s"],
    "turn_max_mps": service_cfg["turn_max_mps"],
    "stall_check_window_s": service_cfg["stall_check_window_s"],
    "turn_stall_min_deg": service_cfg["turn_stall_min_deg"],
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


# --- Aruco priority (added 2026-08-14) — see navigate-prd.md ---
#
# oxts-nav owns the actual GNSS toggle and the 3s no-marker fallback
# (see its gnss_mode.py) - navigate's job is only to ask for it at the
# right moments: entering when a flagged path's run actually starts,
# leaving whenever that run ends, by whatever means (finished, aborted,
# or an operator Stop). _aruco_priority_active_for_run tracks whether
# *this* run is the one that asked, so an ordinary run never sends a
# spurious "back to normal" - though that call is harmless/idempotent
# either way (see gnss_mode.py's exit_aruco_priority).
_aruco_priority_active_for_run = False


def _maybe_enter_aruco_priority():
    global _aruco_priority_active_for_run
    if _current_path_name is None:
        return
    if not paths.load_flags(PATHS_DIR, _current_path_name).get("aruco_priority"):
        return
    _aruco_priority_active_for_run = True
    try:
        requests.post(f"{OXTSNAV_BASE_URL}/gnss/aruco-priority", timeout=OXTSNAV_TIMEOUT_S)
    except requests.exceptions.RequestException:
        logging.warning("[navigate] Couldn't reach oxts-nav to enter aruco-priority mode")


def _end_aruco_priority_if_active():
    global _aruco_priority_active_for_run
    if not _aruco_priority_active_for_run:
        return
    _aruco_priority_active_for_run = False
    try:
        requests.post(f"{OXTSNAV_BASE_URL}/gnss/normal", timeout=OXTSNAV_TIMEOUT_S)
    except requests.exceptions.RequestException:
        logging.warning("[navigate] Couldn't reach oxts-nav to restore normal GNSS")


nav_client = FeedClient(oxtsnav_cfg["nav_feed_socket"], default={"nav": {}, "status": {}, "connection": {}})
drive_client = FeedClient(drive_cfg["drive_feed_socket"], default={})
runner = PathRunner(CONTROL_CONFIG, send_velocity, send_pump)
turn_runner = TurnRunner(TURN_CONFIG, send_velocity)

_recording_lock = threading.Lock()
_recording_points = []

# Which saved path is currently loaded - PathRunner itself only ever
# sees raw points, not a name (see control.py), so this is tracked
# here instead, purely for display: the Run page's own dropdown/map
# only updated in response to *that page's own* Load button being
# clicked, so a path loaded a different way (the Paths page's own Run
# button, or a job driving navigate directly) left the page showing
# nothing useful despite navigate actually running something real.
# Published in the feed below so any page watching it can stay in sync
# regardless of how the path got loaded.
_current_path_name = None

# Which of runner/turn_runner is the one whose status/debug-log actually
# matters right now (added 2026-08-18 alongside turn_runner) - only one
# of the two is ever "running" at a time (both /control/start and
# /control/turn refuse to start while the other is), but once a run
# finishes its terminal state (stopped_ok/aborted) needs to keep being
# reported until something else starts, exactly like _current_path_name
# already does for path runs - this is that same idea, generalised to
# cover which *kind* of run it was.
_active_kind = "path"  # "path" | "turn"


def _current_position():
    """{"lat":, "lon":, "heading_deg":, "horizontal_accuracy_m":,
    "horizontal_speed_mps":, "wheel_left_mps":, "wheel_right_mps":}
    from oxts-nav's and drive's live feeds (Lat/Lon there are radians;
    converted to degrees here since that's what geometry.py/paths.py
    both expect), or None if no fix has been received yet.
    horizontal_speed_mps is the xNAV's own INS-derived ground speed
    (hypot of its Vn/Ve velocity components); wheel_left_mps/
    wheel_right_mps are drive's own real per-wheel measured speed
    (LM_vel_filt_mps/RM_vel_filt_mps, straight from the firmware's FV
    telemetry) - both logged purely for comparison against each other
    and against control.py's *commanded* left_mps/right_mps (see
    drive-prd.md/wheelspeed's counts_per_metre calibration question;
    note PathRunner.status()'s left_mps/right_mps below are the
    differential-drive *command*, not a measurement - don't confuse
    the two when reading a log). Not used for control."""
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
    vn = nav.get("Vn")
    ve = nav.get("Ve")
    horizontal_speed_mps = math.hypot(vn, ve) if vn is not None and ve is not None else None
    drive_state = drive_client.latest()
    return {
        "lat": math.degrees(nav["Lat"]),
        "lon": math.degrees(nav["Lon"]),
        "heading_deg": nav["Heading"],
        "horizontal_accuracy_m": horizontal_accuracy_m,
        "horizontal_speed_mps": horizontal_speed_mps,
        "wheel_left_mps": drive_state.get("LM_vel_filt_mps"),
        "wheel_right_mps": drive_state.get("RM_vel_filt_mps"),
    }


_debug_log_path = None


def _start_new_debug_log():
    """Called when a run starts — a fresh, retained file per run (see
    jobs'/missions' identical convention), not a single overwritten
    file, so past runs can be compared afterward. Filename is a
    yymmdd_hhmmss timestamp plus the path name (path names are already
    validated slash/dot-free on save — see paths.py's _validate_name —
    so no extra sanitising is needed here), so a plain `ls` already
    says what each file was without opening it. The path name is also
    written on every line below — that's the one an actual viewer
    should key off, since a filename could in principle collide or get
    truncated; the name in the filename is just a convenience mirror
    of it."""
    global _debug_log_path
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
    # A turn has no path name of its own - label its log with the target
    # heading instead, same reasoning (a plain `ls` already says what it
    # was).
    if _active_kind == "turn" and turn_runner.target_heading_deg is not None:
        label = f"turn_{turn_runner.target_heading_deg:.0f}deg"
    else:
        label = _current_path_name
    stem = f"{timestamp}_{label}" if label else timestamp
    candidate = LOGS_DIR / f"{stem}.jsonl"
    suffix = 1
    while candidate.exists():
        # Two starts within the same second (e.g. a quick real Start,
        # Stop, Start again, or two tests running fast) — never
        # silently overwrite an existing run's log.
        candidate = LOGS_DIR / f"{stem}_{suffix}.jsonl"
        suffix += 1
    _debug_log_path = candidate
    _debug_log_path.write_text("")


def _append_debug_log(position):
    if _debug_log_path is None:
        return
    if _active_kind == "turn":
        entry = {"t": time.time(), "path_name": None, **position, **turn_runner.status()}
    else:
        entry = {"t": time.time(), "path_name": _current_path_name, **position, **runner.status()}
    with open(_debug_log_path, "a") as f:
        f.write(json.dumps(entry, default=str) + "\n")


def _sweep_old_debug_logs():
    """Run once at startup, same as jobs'/missions' own sweep — logs
    accumulate now (one retained file per run), so old ones need
    actual cleanup."""
    if not LOGS_DIR.exists():
        return
    cutoff = time.time() - service_cfg["log_retention_days"] * 86400
    for file in LOGS_DIR.glob("*.jsonl"):
        if file.stat().st_mtime < cutoff:
            file.unlink()


_control_loop_last_state = "idle"
# Set the moment state leaves "running" (naturally finishing -> "stopped_ok",
# or aborting -> "aborted" - see control.py/turn_control.py). Used to bound
# the debug log's tail below - found live 2026-08-31 that neither of those
# states is "idle", and nothing else ever moves state back to idle on its
# own, so without this bound the log kept appending at control_hz forever
# (one file grew to 689k lines/~19 hours) until the next path was loaded.
# Only an operator-pressed Stop (PathRunner.stop()/TurnRunner.stop(), which
# set state straight to "idle") ever ended it before this fix.
_terminal_state_entered_at = None
LOG_TAIL_AFTER_STOP_S = 2.0  # deliberately short - "let me see what happens right after it stops", not an unbounded idle recording


def _control_tick():
    """One control-loop iteration, factored out of _control_loop so it
    can be called directly in tests without the thread/sleep."""
    global _control_loop_last_state, _terminal_state_entered_at
    position = _current_position()
    if position is None:
        # oxts-nav's feed has gone stale (xNAV disconnected/rebooting) -
        # a run in progress used to just silently stop being stepped,
        # leaving it in "running" forever with drive still holding
        # whatever velocity was last commanded (drive has no watchdog of
        # its own for "auto" commands, unlike a manual jog - see
        # control.py's ControlArbiter). Abort it properly instead, same
        # as any other abort reason, so the robot actually stops.
        if _active_kind == "turn":
            turn_runner.abort_if_running("no position - oxts-nav feed is stale/lost")
            state = turn_runner.status()["state"]
        else:
            runner.abort_if_running("no position - oxts-nav feed is stale/lost")
            state = runner.status()["state"]
    elif _active_kind == "turn":
        turn_runner.step(position["heading_deg"])
        state = turn_runner.status()["state"]
    else:
        runner.step(position["lat"], position["lon"], position["heading_deg"], position["horizontal_accuracy_m"])
        runner.preview(position["lat"], position["lon"], position["heading_deg"])
        state = runner.status()["state"]
    # A path/turn can end by finishing or aborting entirely inside
    # step() above, with no HTTP call marking the moment - /control/stop
    # covers the operator-Stop case, this edge detection covers the
    # other two, so aruco priority always gets switched off and the log
    # tail's clock starts however the run actually ended.
    if _control_loop_last_state == "running" and state != "running":
        _terminal_state_entered_at = time.time()
        _end_aruco_priority_if_active()
    in_tail = (
        state not in ("idle", "running")
        and _terminal_state_entered_at is not None
        and time.time() - _terminal_state_entered_at <= LOG_TAIL_AFTER_STOP_S
    )
    # Every control tick while running (or briefly after), not a
    # throttled ~1Hz snapshot - this is now the data an analysis tool
    # compares runs with, so it needs the same resolution the control
    # loop itself acts at (see navigate-prd.md's "Debug log").
    if position is not None and (state == "running" or in_tail):
        _append_debug_log(position)
    _control_loop_last_state = state


def _control_loop():
    period = 1.0 / service_cfg["control_hz"]
    while True:
        _control_tick()
        time.sleep(period)


@app.context_processor
def inject_urls():
    browser_host = request.host.split(":")[0]
    return {
        "manager_url": service_url(browser_host, "manager") + "/",
        "oxtsnav_ws_url": service_url(browser_host, "oxts-nav", scheme="ws") + "/ws/nav",
        # For the shared header's Aruco status badge — see
        # shared/web/static/sysstatus.js.
        "aruco_ws_url": service_url(browser_host, "aruco", scheme="ws") + "/ws/aruco",
        "map_manager_ws_url": service_url(browser_host, "map-manager", scheme="ws") + "/ws/map-manager",
        "drive_ws_url": service_url(browser_host, "drive", scheme="ws") + "/ws/drive",  # battery badge - see sysstatus.js
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


@app.route("/api/paths/<name>/flags")
def api_get_path_flags(name):
    try:
        return jsonify(paths.load_flags(PATHS_DIR, name))
    except paths.InvalidPathName:
        abort(404)


@app.route("/api/paths/<name>/flags", methods=["POST"])
def api_save_path_flags(name):
    payload = request.get_json(force=True)
    try:
        paths.save_flags(PATHS_DIR, name, payload)
    except paths.InvalidPathName:
        abort(400)
    return "", 204


@app.route("/api/paths/<name>", methods=["DELETE"])
def api_delete_path(name):
    try:
        paths.delete_path(PATHS_DIR, name)
    except (FileNotFoundError, paths.InvalidPathName):
        abort(404)
    return "", 204


@app.route("/api/paths/<name>", methods=["POST"])
def api_save_path(name):
    """Save (or overwrite) a path from the Edit map page's in-browser
    point array — the write-side counterpart to the GET/DELETE above.
    Unlike /record/save (which saves the server-side recording buffer),
    the edited points come entirely from the request body: the editor
    never uses the recording buffer at all."""
    payload = request.get_json(force=True)
    points = payload["points"]
    if len(points) < 2:
        abort(400)
    try:
        paths.save_path(PATHS_DIR, name, points)
    except paths.InvalidPathName:
        abort(400)
    return "", 204


@app.route("/api/project", methods=["POST"])
def api_project():
    """{lat, lon} -> {lat, lon}, projected bearing_deg/distance_m
    forward — thin wrapper over geometry.project_forward so the Edit
    map page's N/S/E/W buttons use exactly the same maths as every
    other manual path edit, rather than a second, JS-side
    approximation of it (see navigate-prd.md's "Path editing")."""
    payload = request.get_json(force=True)
    new_lat, new_lon = geometry.project_forward(
        float(payload["lat"]), float(payload["lon"]),
        float(payload["bearing_deg"]), float(payload["distance_m"]),
    )
    return jsonify({"lat": new_lat, "lon": new_lon})


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


@app.route("/record/undo", methods=["POST"])
def record_undo():
    with _recording_lock:
        if _recording_points:
            _recording_points.pop()
        count = len(_recording_points)
    return jsonify({"point_count": count})


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
    global _current_path_name
    try:
        points = paths.load_path(PATHS_DIR, name)
    except (FileNotFoundError, paths.InvalidPathName):
        abort(404)
    result = runner.load_path(points)
    if result["ok"]:
        _current_path_name = name
    return jsonify(result)


@app.route("/pump/manual", methods=["POST"])
def pump_manual():
    """Direct on/off outside of any path-following - for jobs'
    stationary "water" step (see jobs/control.py). Refuses while a
    path is actually running: that path's own step() is already
    resending pump commands every tick, and an out-of-band manual
    command here would just race it."""
    if runner.status()["state"] == "running":
        return jsonify({"ok": False, "reason": "a path is already running"})
    payload = request.get_json(force=True)
    send_pump(bool(payload["on"]))
    return jsonify({"ok": True})


@app.route("/control/entry-check")
def control_entry_check():
    position = _current_position()
    if position is None:
        return jsonify({"ok": False, "reason": "no position fix yet"})
    return jsonify(runner.entry_check(position["lat"], position["lon"], position["heading_deg"]))


@app.route("/control/start", methods=["POST"])
def control_start():
    global _active_kind
    if turn_runner.status()["state"] == "running":
        return jsonify({"ok": False, "reason": "a turn is already running - stop it first"})
    position = _current_position()
    if position is None:
        return jsonify({"ok": False, "reason": "no position fix yet"})
    result = runner.start(position["lat"], position["lon"], position["heading_deg"])
    if result["ok"]:
        _active_kind = "path"
        _start_new_debug_log()
        _maybe_enter_aruco_priority()
    return jsonify(result)


@app.route("/control/turn", methods=["POST"])
def control_turn():
    """Turn-in-place to a target heading - see turn_control.py. For
    jobs' `turn_to_heading` step, but also directly usable on its own
    (see the Run page) - independent testability was the whole point,
    see navigate-prd.md's "Turn in place"."""
    global _active_kind
    if runner.status()["state"] == "running":
        return jsonify({"ok": False, "reason": "a path is already running - stop it first"})
    position = _current_position()
    if position is None:
        return jsonify({"ok": False, "reason": "no position fix yet"})
    payload = request.get_json(force=True)
    heading_deg = float(payload["heading_deg"])
    tolerance_deg = float(payload.get("tolerance_deg", service_cfg["turn_tolerance_deg"]))
    result = turn_runner.start(heading_deg, tolerance_deg, position["heading_deg"])
    if result["ok"]:
        _active_kind = "turn"
        _start_new_debug_log()
    return jsonify(result)


@app.route("/control/stop", methods=["POST"])
def control_stop():
    # Both, unconditionally - whichever wasn't running is a harmless
    # no-op (see PathRunner.stop()/TurnRunner.stop()), and the caller
    # shouldn't have to know which kind of run it's stopping.
    runner.stop()
    turn_runner.stop()
    _end_aruco_priority_if_active()
    return "", 204


def _snapshot():
    if _active_kind == "turn":
        return {**turn_runner.status(), "path_name": None}
    return {**runner.status(), "path_name": _current_path_name}


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
    _sweep_old_debug_logs()
    nav_client.start()
    drive_client.start()
    threading.Thread(target=_control_loop, daemon=True).start()

    feed = NavigateFeedServer(service_cfg["navigate_feed_socket"], _snapshot, service_cfg["navigate_feed_hz"])
    feed.start()

    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
