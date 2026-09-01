"""GR6-v2 map-manager: builds and persists a single garden-wide
occupancy grid from the ultrasonic sensors + live GNSS/INS pose, gated
on both a manual on/off switch and oxts-nav's reported accuracy. See
map-manager-prd.md for the requirements this implements — in
particular "Reference frame" (why the origin is fixed, not per-path
like navigate) and "Accuracy/logging gating".

Image capture and the human override-editing UI are deliberately not
built yet — see map-manager-prd.md's "Not yet built".
"""

import json
import math
import sys
import threading
import time
from pathlib import Path

from flask import Flask, jsonify, request
from flask_sock import Sock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.feed_client import FeedClient  # noqa: E402
from shared.web import register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

from grid import LogOddsParams, MapGrid, effective_range_m, sensor_world_pose  # noqa: E402
import reprocess  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
STATUS_HZ = 2
FEED_STALE_S = 3.0  # if the nav/drive feeds haven't produced a fresh-looking payload in this long, treat as "not eligible" rather than trusting stale numbers

cfg = load_config()
service_cfg = cfg["services"]["map-manager"]
oxtsnav_cfg = cfg["services"]["oxts-nav"]
drive_cfg = cfg["services"]["drive"]

MAP_DIR = Path(__file__).resolve().parent.parent / service_cfg["map_dir"]
GRID_PATH = MAP_DIR / "grid.json"
RAW_LOGS_DIR = MAP_DIR / "raw"
GRIDS_DIR = MAP_DIR / "grids"  # post-processor snapshots - see reprocess.py
SENSORS = service_cfg["sensors"]
ACCURACY_H_MAX_M = service_cfg["accuracy_h_max_m"]
ACCURACY_V_MAX_M = service_cfg["accuracy_v_max_m"]
MAP_ORIGIN_LAT = service_cfg["map_origin_lat"]
MAP_ORIGIN_LON = service_cfg["map_origin_lon"]

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)

nav_client = FeedClient(oxtsnav_cfg["nav_feed_socket"], default={"nav": {}, "status": {}, "connection": {}})
drive_client = FeedClient(drive_cfg["drive_feed_socket"], default={})

LOG_ODDS_PARAMS = LogOddsParams(service_cfg["p_hit"], service_cfg["p_miss"], service_cfg["p_min"], service_cfg["p_max"])

grid = MapGrid(
    cell_size_m=service_cfg["cell_size_m"],
    max_range_m=service_cfg["max_range_m"],
    beam_half_angle_deg=service_cfg["beam_half_angle_deg"],
    forget_after_s=service_cfg["forget_after_days"] * 86400,
    params=LOG_ODDS_PARAMS,
)

# Manual on/off — deliberately in-memory only (always starts "off"), so
# a service restart never silently resumes logging while the robot's
# being picked up/carried about. See map-manager-prd.md
# "Accuracy/logging gating". Two independent switches (added 2026-08-18)
# - the live grid and the separate raw-event capture (see
# map-manager-prd.md's "Raw event capture") - each gated by the same
# accuracy check, but switchable independently: you might want the live
# grid running day-to-day without ever capturing raw data, or capture a
# one-off raw session without wanting it folded into the live grid at
# all.
_logging_enabled = False
_raw_capture_enabled = False
_logging_lock = threading.Lock()
_raw_log_path = None


def set_logging_enabled(enabled: bool):
    global _logging_enabled
    with _logging_lock:
        _logging_enabled = enabled


def logging_enabled() -> bool:
    with _logging_lock:
        return _logging_enabled


def set_raw_capture_enabled(enabled: bool):
    global _raw_capture_enabled
    with _logging_lock:
        was_enabled = _raw_capture_enabled
        _raw_capture_enabled = enabled
    if enabled and not was_enabled:
        _start_new_raw_log()


def raw_capture_enabled() -> bool:
    with _logging_lock:
        return _raw_capture_enabled


def _current_pose():
    """{"north", "east", "heading_deg", "horizontal_accuracy_m",
    "vertical_accuracy_m"} in map-manager's own fixed frame (see
    map-manager-prd.md "Reference frame"), or None if oxts-nav has no
    fix — same accuracy formula as navigate/app.py's
    _current_position, reused deliberately for consistency."""
    payload = nav_client.latest()
    nav = payload.get("nav", {})
    status = payload.get("status", {})
    if "Lat" not in nav or "Lon" not in nav or "Heading" not in nav:
        return None
    north, east = geodesy_to_local(math.degrees(nav["Lat"]), math.degrees(nav["Lon"]))
    north_acc = status.get("NorthAcc")
    east_acc = status.get("EastAcc")
    horizontal_accuracy_m = math.hypot(north_acc, east_acc) if north_acc is not None and east_acc is not None else None
    return {
        "north": north,
        "east": east,
        "heading_deg": nav["Heading"],
        "horizontal_accuracy_m": horizontal_accuracy_m,
        "vertical_accuracy_m": status.get("AltAcc"),
    }


from shared.geodesy import lla_to_ned  # noqa: E402


def geodesy_to_local(lat, lon):
    north, east, _down = lla_to_ned(lat, lon, 0.0, MAP_ORIGIN_LAT, MAP_ORIGIN_LON, 0.0)
    return north, east


def _accuracy_reason(pose):
    """Why (if at all) the current fix isn't good enough to trust for
    either the live grid or raw capture - independent of either
    manual switch, see _eligibility/_raw_capture_reason below."""
    if pose is None:
        return "no GNSS fix"
    if pose["horizontal_accuracy_m"] is None or pose["horizontal_accuracy_m"] > ACCURACY_H_MAX_M:
        return f"horizontal accuracy too poor (limit {ACCURACY_H_MAX_M}m)"
    if pose["vertical_accuracy_m"] is None or pose["vertical_accuracy_m"] > ACCURACY_V_MAX_M:
        return f"vertical accuracy too poor (limit {ACCURACY_V_MAX_M}m)"
    return None


def _eligibility(pose):
    """Why (if at all) this tick isn't allowed to update the live grid —
    used both to gate the tick loop and to explain the current state on
    the home page."""
    if not logging_enabled():
        return "logging is off"
    return _accuracy_reason(pose)


def _raw_capture_reason(pose):
    """Same idea as _eligibility, for the separate raw-capture switch."""
    if not raw_capture_enabled():
        return "raw capture is off"
    return _accuracy_reason(pose)


def _tick():
    pose = _current_pose()
    grid_reason = _eligibility(pose)
    raw_reason = _raw_capture_reason(pose)
    if grid_reason is not None and raw_reason is not None:
        return
    ranges = drive_client.latest()
    for sensor in SENSORS:
        range_mm = ranges.get(f"ultrasonic_{sensor['tag']}_mm")
        if range_mm is None:
            continue
        if raw_reason is None:
            _append_raw_log({"t": time.time(), "north": pose["north"], "east": pose["east"],
                              "heading_deg": pose["heading_deg"], "tag": sensor["tag"], "range_mm": range_mm})
        if grid_reason is None:
            range_m = effective_range_m(range_mm, grid.max_range_m)
            sensor_north, sensor_east, beam_heading_deg = sensor_world_pose(
                pose["north"], pose["east"], pose["heading_deg"], sensor["x"], sensor["y"], sensor["heading_deg"]
            )
            grid.record_detection(sensor_north, sensor_east, beam_heading_deg, range_m)


def _tick_loop():
    period = 1.0 / service_cfg["control_hz"]
    last_save = time.monotonic()
    while True:
        _tick()
        if grid.dirty and time.monotonic() - last_save > service_cfg["save_interval_s"]:
            _save_grid()
            last_save = time.monotonic()
        time.sleep(period)


def _save_grid():
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    tmp_path = GRID_PATH.with_suffix(".json.tmp")
    with open(tmp_path, "w") as f:
        json.dump(grid.dump_state(), f)
    tmp_path.replace(GRID_PATH)  # atomic — never leaves grid.json half-written if the process dies mid-save
    grid.dirty = False


def _load_grid():
    if not GRID_PATH.exists():
        return
    with open(GRID_PATH) as f:
        grid.load_state(json.load(f))


# --- Raw event capture (added 2026-08-18) — see map-manager-prd.md's
# "Raw event capture". One file per capture session (started when the
# switch flips off->on, see set_raw_capture_enabled above), same
# yymmdd_hhmmss-plus-collision-suffix naming as waterbutt's own QC log.


def _start_new_raw_log():
    global _raw_log_path
    RAW_LOGS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%y%m%d_%H%M%S")
    candidate = RAW_LOGS_DIR / f"{timestamp}.jsonl"
    suffix = 1
    while candidate.exists():
        candidate = RAW_LOGS_DIR / f"{timestamp}_{suffix}.jsonl"
        suffix += 1
    _raw_log_path = candidate


def _append_raw_log(record):
    if _raw_log_path is None:
        return
    with open(_raw_log_path, "a") as f:
        f.write(json.dumps(record) + "\n")


def _sweep_old_raw_logs():
    """Run once at startup, same pattern as navigate/jobs/missions'
    debug logs - but a much longer retention (raw_log_retention_days,
    default 4) than those, since the whole point of this capture is
    still being useful days later for a tuning session, not just
    immediate debugging."""
    if not RAW_LOGS_DIR.exists():
        return
    cutoff = time.time() - service_cfg["raw_log_retention_days"] * 86400
    for file in RAW_LOGS_DIR.glob("*.jsonl"):
        if file.stat().st_mtime < cutoff:
            file.unlink()


@app.context_processor
def inject_urls():
    browser_host = request.host.split(":")[0]
    return {
        "manager_url": service_url(browser_host, "manager") + "/",
        # For the shared header's GNSS/Aruco status badges — see
        # shared/web/static/sysstatus.js.
        "oxtsnav_ws_url": service_url(browser_host, "oxts-nav", scheme="ws") + "/ws/nav",
        "aruco_ws_url": service_url(browser_host, "aruco", scheme="ws") + "/ws/aruco",
        "map_manager_ws_url": service_url(browser_host, "map-manager", scheme="ws") + "/ws/map-manager",
        "drive_ws_url": service_url(browser_host, "drive", scheme="ws") + "/ws/drive",  # battery badge - see sysstatus.js
    }


@app.route("/logging", methods=["POST"])
def set_logging():
    payload = request.get_json(force=True)
    set_logging_enabled(bool(payload["enabled"]))
    return jsonify({"enabled": logging_enabled()})


@app.route("/raw-capture", methods=["POST"])
def set_raw_capture():
    payload = request.get_json(force=True)
    set_raw_capture_enabled(bool(payload["enabled"]))
    return jsonify({"enabled": raw_capture_enabled()})


@app.route("/api/raw-logs")
def api_raw_logs():
    return jsonify(reprocess.list_raw_logs(RAW_LOGS_DIR))


@app.route("/reprocess", methods=["POST"])
def do_reprocess():
    payload = request.get_json(force=True)
    filenames = payload["files"]
    if not filenames:
        return jsonify({"ok": False, "reason": "no input files selected"}), 400
    params = LogOddsParams(payload["p_hit"], payload["p_miss"], payload["p_min"], payload["p_max"])
    new_grid = reprocess.build_grid_from_raw_logs(
        RAW_LOGS_DIR, filenames, SENSORS,
        cell_size_m=payload["cell_size_m"], max_range_m=payload["max_range_m"],
        beam_half_angle_deg=payload["beam_half_angle_deg"],
        forget_after_s=service_cfg["forget_after_days"] * 86400,
        params=params,
    )
    result = reprocess.write_snapshot(GRIDS_DIR, new_grid, filenames, SENSORS)
    return jsonify({"ok": True, **result})


def _status():
    pose = _current_pose()
    status = {
        "logging_enabled": logging_enabled(),
        "eligibility_reason": _eligibility(pose),
        "raw_capture_enabled": raw_capture_enabled(),
        "raw_capture_reason": _raw_capture_reason(pose),
        **grid.stats(),
    }
    # Only included when actually known - see ws-utils.js's fillFields,
    # which blanks a *missing* key back to "—" but would otherwise print
    # a literal "null" for a key present with value None.
    if pose and pose["horizontal_accuracy_m"] is not None:
        status["horizontal_accuracy_m"] = pose["horizontal_accuracy_m"]
    if pose and pose["vertical_accuracy_m"] is not None:
        status["vertical_accuracy_m"] = pose["vertical_accuracy_m"]
    return status


@sock.route("/ws/map-manager")
def ws_map_manager(ws):
    period = 1.0 / STATUS_HZ
    while True:
        ws.send(json.dumps(_status()))
        time.sleep(period)


def reprocess_context():
    """Current config's values, as the reprocess page's default form
    values - a starting point to tweak before recomputing, not a
    fixed set (see map-manager-prd.md's "Post-processor")."""
    return {
        "default_params": {"p_hit": service_cfg["p_hit"], "p_miss": service_cfg["p_miss"],
                            "p_min": service_cfg["p_min"], "p_max": service_cfg["p_max"]},
        "default_cell_size_m": service_cfg["cell_size_m"],
        "default_max_range_m": service_cfg["max_range_m"],
        "default_beam_half_angle_deg": service_cfg["beam_half_angle_deg"],
    }


register_pages(app, PAGES_DIR, index_slug="home", context_providers={"reprocess": reprocess_context})


if __name__ == "__main__":
    _load_grid()
    _sweep_old_raw_logs()
    nav_client.start()
    drive_client.start()
    threading.Thread(target=_tick_loop, daemon=True).start()
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
