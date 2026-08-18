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

from grid import MapGrid, sensor_world_pose  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
STATUS_HZ = 2
FEED_STALE_S = 3.0  # if the nav/drive feeds haven't produced a fresh-looking payload in this long, treat as "not eligible" rather than trusting stale numbers

cfg = load_config()
service_cfg = cfg["services"]["map-manager"]
oxtsnav_cfg = cfg["services"]["oxts-nav"]
drive_cfg = cfg["services"]["drive"]

MAP_DIR = Path(__file__).resolve().parent.parent / service_cfg["map_dir"]
GRID_PATH = MAP_DIR / "grid.json"
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

grid = MapGrid(
    cell_size_m=service_cfg["cell_size_m"],
    max_range_m=service_cfg["max_range_m"],
    beam_half_angle_deg=service_cfg["beam_half_angle_deg"],
    forget_after_s=service_cfg["forget_after_days"] * 86400,
    min_observations=service_cfg["min_observations"],
)

# Manual on/off — deliberately in-memory only (always starts "off"), so
# a service restart never silently resumes logging while the robot's
# being picked up/carried about. See map-manager-prd.md
# "Accuracy/logging gating".
_logging_enabled = False
_logging_lock = threading.Lock()


def set_logging_enabled(enabled: bool):
    global _logging_enabled
    with _logging_lock:
        _logging_enabled = enabled


def logging_enabled() -> bool:
    with _logging_lock:
        return _logging_enabled


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


def _eligibility(pose):
    """Why (if at all) this tick isn't allowed to update the grid — used
    both to gate the tick loop and to explain the current state on the
    home page."""
    if not logging_enabled():
        return "logging is off"
    if pose is None:
        return "no GNSS fix"
    if pose["horizontal_accuracy_m"] is None or pose["horizontal_accuracy_m"] > ACCURACY_H_MAX_M:
        return f"horizontal accuracy too poor (limit {ACCURACY_H_MAX_M}m)"
    if pose["vertical_accuracy_m"] is None or pose["vertical_accuracy_m"] > ACCURACY_V_MAX_M:
        return f"vertical accuracy too poor (limit {ACCURACY_V_MAX_M}m)"
    return None


def _tick():
    pose = _current_pose()
    reason = _eligibility(pose)
    if reason is not None:
        return
    ranges = drive_client.latest()
    for sensor in SENSORS:
        range_mm = ranges.get(f"ultrasonic_{sensor['tag']}_mm")
        if range_mm is None:
            continue
        # Firmware sentinel for "nothing in range" isn't documented here
        # (see drive/protocol.py) - clamping at our own configured
        # max_range_m gets the same "no detection" behaviour regardless
        # of what the firmware actually reports on a timeout. Revisit if
        # that assumption turns out wrong once real ultrasonic data is
        # captured - see map-manager-prd.md.
        range_m = range_mm / 1000.0
        range_m = None if range_m >= grid.max_range_m else range_m
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
    }


@app.route("/logging", methods=["POST"])
def set_logging():
    payload = request.get_json(force=True)
    set_logging_enabled(bool(payload["enabled"]))
    return jsonify({"enabled": logging_enabled()})


def _status():
    pose = _current_pose()
    status = {"logging_enabled": logging_enabled(), "eligibility_reason": _eligibility(pose), **grid.stats()}
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


register_pages(app, PAGES_DIR, index_slug="home")


if __name__ == "__main__":
    _load_grid()
    nav_client.start()
    drive_client.start()
    threading.Thread(target=_tick_loop, daemon=True).start()
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
