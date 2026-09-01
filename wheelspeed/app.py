"""GR6-v2 wheelspeed: sends per-wheel GAD speed/velocity aiding updates
to the xNAV650 from drive's filtered wheel-velocity telemetry, and
shows a live chart/numeric page comparing each wheel's speed against
the INS's own forward body-frame velocity (display only, never sent as
aiding — see velocity.py). See wheelspeed-prd.md for the requirements
this implements, in particular the still-open "Update rate" question
and the placeholder (unmeasured) lever arms.
"""

import json
import sys
import threading
import time
from pathlib import Path

from flask import Flask, request
from flask_sock import Sock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.feed_client import FeedClient  # noqa: E402
from shared.web import manager_url, register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "oxts-nav"))
from ncomrx import machine_time_to_gps  # noqa: E402

from gad_wheelspeed import GadWheelspeed  # noqa: E402
from timing import midpoint_time  # noqa: E402
from velocity import forward_velocity  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
WHEELSPEED_STATUS_HZ = 5

cfg = load_config()
service_cfg = cfg["services"]["wheelspeed"]
drive_cfg = cfg["services"]["drive"]
oxtsnav_cfg = cfg["services"]["oxts-nav"]
xnav_ip = cfg["xnav_ip"]

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

_state_lock = threading.Lock()
_state = {"left_mps": None, "right_mps": None, "forward_mps": None, "packets_sent": 0}


def _forward_mps(nav):
    if "Vn" not in nav or "Ve" not in nav or "Heading" not in nav:
        return None
    return forward_velocity(nav["Vn"], nav["Ve"], nav["Heading"])


def _update_loop():
    # Event-driven off drive's own FV_timestamp (real arrival time of
    # each filtered-velocity telemetry line — see serial_link.py), not a
    # fixed poll rate: this loop just needs to notice a new timestamp
    # promptly, it's never the source of the GAD packet's own time. See
    # wheelspeed-prd.md's "Update rate / timing" — an update fires
    # exactly once per real new FV reading, at whatever rate that
    # actually arrives (GR6-v1 did this at ~20Hz and it worked fine).
    poll_period = 0.02
    last_fv_timestamp = None
    while True:
        drive_state = drive_client.latest()
        nav_payload = nav_client.latest()
        nav = nav_payload.get("nav", {})
        connection = nav_payload.get("connection", {})

        fv_timestamp = drive_state.get("FV_timestamp")
        left_mps = drive_state.get("LM_vel_filt_mps")
        right_mps = drive_state.get("RM_vel_filt_mps")
        forward_mps = _forward_mps(nav)

        is_new_reading = fv_timestamp is not None and fv_timestamp != last_fv_timestamp
        if is_new_reading and last_fv_timestamp is not None and left_mps is not None and right_mps is not None:
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

        with _state_lock:
            _state.update({
                "left_mps": left_mps,
                "right_mps": right_mps,
                "forward_mps": forward_mps,
                "packets_sent": gad_wheelspeed.packets_sent,
            })
        time.sleep(poll_period)


def _snapshot():
    with _state_lock:
        return dict(_state)


@app.context_processor
def inject_urls():
    browser_host = request.host.split(":")[0]
    return {
        "manager_url": manager_url(browser_host),
        # For the shared header's GNSS/Aruco status badges — see
        # shared/web/static/sysstatus.js.
        "oxtsnav_ws_url": service_url(browser_host, "oxts-nav", scheme="ws") + "/ws/nav",
        "aruco_ws_url": service_url(browser_host, "aruco", scheme="ws") + "/ws/aruco",
        "map_manager_ws_url": service_url(browser_host, "map-manager", scheme="ws") + "/ws/map-manager",
        "drive_ws_url": service_url(browser_host, "drive", scheme="ws") + "/ws/drive",  # battery badge - see sysstatus.js
    }


@sock.route("/ws/wheelspeed")
def ws_wheelspeed(ws):
    period = 1.0 / WHEELSPEED_STATUS_HZ
    while True:
        ws.send(json.dumps(_snapshot()))
        time.sleep(period)


def config_context():
    return {
        "gad_type": service_cfg["gad_type"],
        "left_lever_arm_i": service_cfg["left_wheel"]["lever_arm_i"],
        "left_scale": service_cfg["left_wheel"]["scale"],
        "right_lever_arm_i": service_cfg["right_wheel"]["lever_arm_i"],
        "right_scale": service_cfg["right_wheel"]["scale"],
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
    threading.Thread(target=_update_loop, daemon=True).start()
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
