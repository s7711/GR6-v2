"""GR6-v2 waterbutt: a small web UI + timed-duration control over the
water butt's ESP8266 pinch-valve controller (documentation/
Waterbutt_261112-3.ino) — the firmware itself only exposes "open"/
"close" with its own 5-second fail-safe auto-close, so this service's
job is entirely the duration timing (see control.py's ValveController)
plus the page. See waterbutt-prd.md for the requirements this
implements.
"""

import json
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

from control import ValveController  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
WATERBUTT_STATUS_HZ = 2
TICK_HZ = 2
VALVE_TIMEOUT_S = 3.0  # the ESP8266's own /open,/close handler blocks for ~0.5s moving the servo, plus wifi round-trip
# Slider steps — non-linear, matching how short a watering burst is
# actually useful for vs. a longer soak; also the server-side allow-list
# for /go's duration_s, so a stray/malicious client can't request an
# arbitrary duration.
DURATIONS_S = [1, 2, 5, 10, 20, 50, 120]

cfg = load_config()
service_cfg = cfg["services"]["waterbutt"]

VALVE_BASE_URL = f"http://{service_cfg['hostname']}"

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)


def send_open():
    try:
        requests.get(f"{VALVE_BASE_URL}/open", timeout=VALVE_TIMEOUT_S, allow_redirects=False)
    except requests.exceptions.RequestException:
        app.logger.warning("[waterbutt] Couldn't reach the valve to open it")


def send_close():
    try:
        requests.get(f"{VALVE_BASE_URL}/close", timeout=VALVE_TIMEOUT_S, allow_redirects=False)
    except requests.exceptions.RequestException:
        app.logger.warning("[waterbutt] Couldn't reach the valve to close it")


valve = ValveController(send_open, send_close)


def _tick_loop():
    period = 1.0 / TICK_HZ
    while True:
        valve.tick()
        time.sleep(period)


@app.context_processor
def inject_urls():
    browser_host = request.host.split(":")[0]
    return {
        "manager_url": service_url(browser_host, "manager") + "/",
    }


@app.route("/go", methods=["POST"])
def go():
    payload = request.get_json(force=True)
    duration_s = payload["duration_s"]
    if duration_s not in DURATIONS_S:
        abort(400)
    valve.go(duration_s)
    return jsonify(valve.status())


@app.route("/stop", methods=["POST"])
def stop():
    valve.stop()
    return jsonify(valve.status())


@sock.route("/ws/waterbutt")
def ws_waterbutt(ws):
    period = 1.0 / WATERBUTT_STATUS_HZ
    while True:
        ws.send(json.dumps(valve.status()))
        time.sleep(period)


def run_context():
    return {"durations_s": DURATIONS_S}


register_pages(app, PAGES_DIR, index_slug="run", context_providers={"run": run_context})


if __name__ == "__main__":
    threading.Thread(target=_tick_loop, daemon=True).start()
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
