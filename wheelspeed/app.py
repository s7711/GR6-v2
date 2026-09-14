"""GR6-v2 wheelspeed: sends per-wheel GAD speed/velocity aiding updates
to the xNAV650 from drive's filtered wheel-velocity telemetry, and
shows a live chart/numeric page comparing each wheel's speed against
the INS's own forward body-frame velocity (display only, never sent as
aiding — see velocity.py). See wheelspeed-prd.md for the requirements
this implements, in particular the still-open "Update rate" question
and the placeholder (unmeasured) lever arms.
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
from shared.geodesy import lla_to_ned, ned_to_lla  # noqa: E402
from shared.rotating_jsonl_log import RotatingJsonlLog  # noqa: E402
from shared.web import manager_url, register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "oxts-nav"))
from ncomrx import machine_time_to_gps  # noqa: E402

from gad_switch import GadSwitch  # noqa: E402
from gad_wheelspeed import GadWheelspeed  # noqa: E402
from scale_factor import ScaleFactorTracker  # noqa: E402
from scale_factor_gate import gnss_velocity_trustworthy  # noqa: E402
from scale_factor_map import ScaleFactorMap  # noqa: E402
from timing import midpoint_time  # noqa: E402
from velocity import forward_velocity, wheel_forward_velocity  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
DATA_DIR = Path(__file__).resolve().parent / "data"
WHEELSPEED_STATUS_HZ = 5

cfg = load_config()
service_cfg = cfg["services"]["wheelspeed"]
drive_cfg = cfg["services"]["drive"]
oxtsnav_cfg = cfg["services"]["oxts-nav"]
xnav_ip = cfg["xnav_ip"]

# Same fixed reference frame as map-manager's own persistent grid (see
# map-manager/app.py's "Reference frame") - not this service's own
# setting, so both grids stay directly comparable/overlayable.
map_manager_cfg = cfg["services"]["map-manager"]
MAP_ORIGIN_LAT = map_manager_cfg["map_origin_lat"]
MAP_ORIGIN_LON = map_manager_cfg["map_origin_lon"]
SCALE_FACTOR_MAP_PATH = DATA_DIR / "scale_factor_map.json"

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)

drive_client = FeedClient(drive_cfg["drive_feed_socket"], default={})
nav_client = FeedClient(oxtsnav_cfg["nav_feed_socket"], default={"nav": {}, "status": {}, "connection": {}})

gad_wheelspeed = GadWheelspeed(
    xnav_ip, oxtsnav_cfg["hpr_ib"], service_cfg["gad_type"],
    service_cfg["left_wheel"]["lever_arm_i"], service_cfg["left_wheel"]["scale"],
    service_cfg["right_wheel"]["lever_arm_i"], service_cfg["right_wheel"]["scale"],
)

# Live, persisted on/off switch for whether GAD is actually sent — see
# gad_switch.py for why this is a side-file, not config.yaml.
gad_switch = GadSwitch(DATA_DIR / "gad_enabled.json")

# Always-on log (independent of the GAD switch above — see app.py's own
# 2026-09-08 history: stopping the service to stop GAD used to stop
# logging too, which is exactly the problem this fixes).
log = RotatingJsonlLog(DATA_DIR / "logs", service_cfg["log_rotate_s"], service_cfg["log_retention_days"])


scale_factor_map = ScaleFactorMap(service_cfg["scale_factor_distance_m"], service_cfg["scale_factor_map_max_n"])

# Set by _update_loop just before it calls scale_factor_tracker.update()
# each tick, so _on_scale_factor_result (called synchronously from
# inside that same update() call, whenever a sample completes) can see
# the GNSS status current *at* that sample - see scale_factor_gate.py
# for why this gates the map (not the always-on log below, which never
# gates anything).
_last_gnss_status = {}


def _on_scale_factor_result(record):
    log.append({"event": "scale_factor", **record})

    lat, lon = record.get("lat"), record.get("lon")
    left_wheel, right_wheel = record.get("left_wheel_distance_m"), record.get("right_wheel_distance_m")
    left_ins, right_ins = record.get("left_ins_distance_m"), record.get("right_ins_distance_m")
    if lat is None or lon is None:
        return
    if not gnss_velocity_trustworthy(_last_gnss_status, service_cfg["scale_factor_map_max_vel_innovation"]):
        return
    # Combined scale factor from both wheels' totals, not an average of
    # the two sides' own separate ratios - see wheelspeed-prd.md's
    # "Scale-factor map" (boresight/differential effects are deliberately
    # out of scope for this first pass, so one scalar per cell is enough).
    scale_factor = (left_ins + right_ins) / (left_wheel + right_wheel)
    north, east, _down = lla_to_ned(lat, lon, 0.0, MAP_ORIGIN_LAT, MAP_ORIGIN_LON, 0.0)
    scale_factor_map.update(north, east, scale_factor, t=time.time())


scale_factor_tracker = ScaleFactorTracker(
    service_cfg["scale_factor_distance_m"],
    service_cfg["scale_factor_min_wheel_ratio"],
    service_cfg["scale_factor_min_wheel_ins_ratio"],
    _on_scale_factor_result,
)

_state_lock = threading.Lock()
_state = {
    "left_mps": None, "right_mps": None, "forward_mps": None, "packets_sent": 0,
    "left_ins_mps": None, "right_ins_mps": None,
    "left_counts_s": None, "right_counts_s": None,
    "gad_enabled": False,
}


def _forward_mps(nav):
    if "Vn" not in nav or "Ve" not in nav or "Heading" not in nav:
        return None
    return forward_velocity(nav["Vn"], nav["Ve"], nav["Heading"])


def _wheel_ins_mps(nav):
    """Each wheel's own predicted forward speed, per velocity.py's
    wheel_forward_velocity() — needs Wz (yaw rate) in addition to what
    _forward_mps() needs, since the two wheels only differ from the
    centreline speed while turning."""
    if "Vn" not in nav or "Ve" not in nav or "Heading" not in nav or "Wz" not in nav:
        return None, None
    wheel_base_m = drive_cfg["wheel_base_m"]
    left = wheel_forward_velocity(nav["Vn"], nav["Ve"], nav["Heading"], nav["Wz"], wheel_base_m, "left")
    right = wheel_forward_velocity(nav["Vn"], nav["Ve"], nav["Heading"], nav["Wz"], wheel_base_m, "right")
    return left, right


def _update_loop():
    # CRITICAL THREAD: this is what sends GAD wheelspeed aiding to the
    # xNAV (see gad_wheelspeed.update() below) - already clean (checked
    # 2026-09-09, alongside the oxts-nav/navigate logging-stall fixes),
    # and must stay that way: no disk I/O in this loop. Persistent
    # logging runs on the separate _log_loop thread below instead.
    # Event-driven off drive's own FV_timestamp (real arrival time of
    # each filtered-velocity telemetry line — see serial_link.py), not a
    # fixed poll rate: this loop just needs to notice a new timestamp
    # promptly, it's never the source of the GAD packet's own time. See
    # wheelspeed-prd.md's "Update rate / timing" — an update fires
    # exactly once per real new FV reading, at whatever rate that
    # actually arrives (GR6-v1 did this at ~20Hz and it worked fine).
    global _last_gnss_status
    poll_period = 0.02
    last_fv_timestamp = None
    last_tick = time.monotonic()
    while True:
        now = time.monotonic()
        dt = now - last_tick
        last_tick = now

        drive_state = drive_client.latest()
        nav_payload = nav_client.latest()
        nav = nav_payload.get("nav", {})
        connection = nav_payload.get("connection", {})

        fv_timestamp = drive_state.get("FV_timestamp")
        left_mps = drive_state.get("LM_vel_filt_mps")
        right_mps = drive_state.get("RM_vel_filt_mps")
        forward_mps = _forward_mps(nav)
        left_ins_mps, right_ins_mps = _wheel_ins_mps(nav)

        gad_enabled = gad_switch.enabled()
        is_new_reading = fv_timestamp is not None and fv_timestamp != last_fv_timestamp
        if (
            gad_enabled and is_new_reading and last_fv_timestamp is not None
            and left_mps is not None and right_mps is not None
        ):
            # The velocity just read is an average over
            # [last_fv_timestamp, fv_timestamp] — report it at the
            # midpoint of that interval, not "now", so the GAD time
            # actually matches what the velocity represents.
            machine_time = midpoint_time(last_fv_timestamp, fv_timestamp)
            gps_time = machine_time_to_gps(machine_time, connection.get("timeOffset"))
            if gps_time is not None:
                gad_wheelspeed.update(gps_time[0], gps_time[1], left_mps, right_mps)
        if is_new_reading:
            last_fv_timestamp = fv_timestamp

        # Scale-factor tracking runs regardless of the GAD switch — it's
        # a pure observer of drive's odometer + the INS feed, not
        # something that touches the xNAV. See scale_factor.py.
        left_pos_m = drive_state.get("LM_position_m")
        right_pos_m = drive_state.get("RM_position_m")
        if None not in (left_pos_m, right_pos_m, left_ins_mps, right_ins_mps):
            lat = nav.get("Lat")
            lon = nav.get("Lon")
            _last_gnss_status = nav_payload.get("status", {})
            scale_factor_tracker.update(
                now, left_pos_m, right_pos_m, left_ins_mps, right_ins_mps, dt,
                lat=math.degrees(lat) if lat is not None else None,
                lon=math.degrees(lon) if lon is not None else None,
            )

        with _state_lock:
            _state.update({
                "left_mps": left_mps,
                "right_mps": right_mps,
                "forward_mps": forward_mps,
                "left_ins_mps": left_ins_mps,
                "right_ins_mps": right_ins_mps,
                "left_counts_s": drive_state.get("LM_vel_filt"),
                "right_counts_s": drive_state.get("RM_vel_filt"),
                "packets_sent": gad_wheelspeed.packets_sent,
                "gad_enabled": gad_enabled,
            })
        time.sleep(poll_period)


def _snapshot():
    with _state_lock:
        return dict(_state)


def _log_loop():
    """Always-on, independent of the GAD switch and at a much lower rate
    than _update_loop's own 50Hz notice-a-new-reading polling — see
    log_hz's own config comment."""
    period = 1.0 / service_cfg["log_hz"]
    while True:
        snap = _snapshot()
        log.append({
            "left_counts_s": snap["left_counts_s"],
            "right_counts_s": snap["right_counts_s"],
            "left_mps": snap["left_mps"],
            "right_mps": snap["right_mps"],
            "left_ins_mps": snap["left_ins_mps"],
            "right_ins_mps": snap["right_ins_mps"],
            "gad_enabled": int(snap["gad_enabled"]),
        })
        time.sleep(period)


@app.context_processor
def inject_urls():
    browser_host = request.host.split(":")[0]
    return {
        "manager_url": manager_url(browser_host),
        # For the shared header's GNSS/Aruco status badges — see
        # shared/web/static/sysstatus.js.
        "oxtsnav_ws_url": service_url(browser_host, "oxts-nav", scheme="ws") + "/ws/nav",
        "aruco_ws_url": service_url(browser_host, "aruco", scheme="ws") + "/ws/aruco",
        "drive_ws_url": service_url(browser_host, "drive", scheme="ws") + "/ws/drive",  # battery badge - see sysstatus.js
        "wheelspeed_ws_url": service_url(browser_host, "wheelspeed", scheme="ws") + "/ws/wheelspeed",  # "W" badge - see sysstatus.js
    }


@sock.route("/ws/wheelspeed")
def ws_wheelspeed(ws):
    period = 1.0 / WHEELSPEED_STATUS_HZ
    while True:
        ws.send(json.dumps(_snapshot()))
        time.sleep(period)


def _save_scale_factor_map():
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    tmp_path = SCALE_FACTOR_MAP_PATH.with_suffix(".json.tmp")
    with open(tmp_path, "w") as f:
        json.dump(scale_factor_map.dump_state(), f)
    tmp_path.replace(SCALE_FACTOR_MAP_PATH)  # atomic — same reasoning as map-manager's _save_grid
    scale_factor_map.dirty = False


def _load_scale_factor_map():
    if not SCALE_FACTOR_MAP_PATH.exists():
        return
    with open(SCALE_FACTOR_MAP_PATH) as f:
        scale_factor_map.load_state(json.load(f))


def _scale_factor_map_save_loop():
    period = 5.0
    last_save = time.monotonic()
    while True:
        if scale_factor_map.dirty and time.monotonic() - last_save > service_cfg["scale_factor_map_save_interval_s"]:
            _save_scale_factor_map()
            last_save = time.monotonic()
        time.sleep(period)


@app.route("/api/scale-factor-map")
def api_scale_factor_map():
    """{lat, lon, n, mean, stdev, t} per populated cell - lat/lon (not
    local north/east) since this is consumed directly by geomap.js
    layers (viewer's map, see wheelspeed-prd.md's "Scale-factor map"),
    which only knows lat/lon."""
    cells = []
    for c in scale_factor_map.all_cells():
        lat, lon, _alt = ned_to_lla(c["north"], c["east"], 0.0, MAP_ORIGIN_LAT, MAP_ORIGIN_LON, 0.0)
        cells.append({"lat": lat, "lon": lon, "n": c["n"], "mean": c["mean"], "stdev": c["stdev"], "t": c["t"]})
    return jsonify(cells)


@app.route("/gad-switch", methods=["POST"])
def gad_switch_route():
    payload = request.get_json(force=True)
    gad_switch.set_enabled(bool(payload["enabled"]))
    return jsonify({"enabled": gad_switch.enabled()})


def config_context():
    return {
        "gad_type": service_cfg["gad_type"],
        "left_lever_arm_i": service_cfg["left_wheel"]["lever_arm_i"],
        "left_scale": service_cfg["left_wheel"]["scale"],
        "right_lever_arm_i": service_cfg["right_wheel"]["lever_arm_i"],
        "right_scale": service_cfg["right_wheel"]["scale"],
        "gad_enabled": gad_switch.enabled(),
    }


register_pages(
    app,
    PAGES_DIR,
    index_slug="home",
    context_providers={"config": config_context},
)


if __name__ == "__main__":
    drive_client.start()
    nav_client.start()
    log.start()
    _load_scale_factor_map()
    threading.Thread(target=_update_loop, daemon=True).start()
    threading.Thread(target=_log_loop, daemon=True).start()
    threading.Thread(target=_scale_factor_map_save_loop, daemon=True).start()
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
