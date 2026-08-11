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

import numpy as np
import requests
from flask import Flask, abort, jsonify, request
from flask_sock import Sock
from simple_websocket import Client as WsClient

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.web import register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

sys.path.append(str(Path(__file__).resolve().parent.parent / "aruco"))  # append, not insert(0) - see jobs/continuity.py's comment: insert(0) here risked shadowing this directory's own same-named modules in a same-process test run
import coords  # noqa: E402

import qc_check  # noqa: E402
import qc_marker  # noqa: E402
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
# QC threshold choices, and the server-side allow-list for /go's
# qc_threshold_m - same "operator picks from a fixed set, server
# re-validates" reasoning as DURATIONS_S above.
QC_THRESHOLD_OPTIONS_M = [0.05, 0.08, 0.10]

cfg = load_config()
service_cfg = cfg["services"]["waterbutt"]
aruco_cfg = cfg["services"]["aruco"]

# Used for a /go call that doesn't pick a tolerance at all (e.g. jobs'
# `fill` step, which has no threshold selector of its own yet) - a
# config value, not a hardcoded constant, since this is exactly the
# kind of thing worth tuning without a code change. See
# waterbutt-prd.md's "QC gating on fill".
QC_DEFAULT_THRESHOLD_M = service_cfg["qc_default_threshold_m"]

VALVE_BASE_URL = f"http://{service_cfg['hostname']}"
QC_MARKER_PATH = Path(__file__).resolve().parent.parent / service_cfg["qc_marker_file"]
QC_HPR_CB = tuple(aruco_cfg["camera_extrinsics"]["hpr_cb"])
QC_DXC_B = tuple(aruco_cfg["camera_extrinsics"]["d_xc_b"])
QC_FUNNEL_OFFSET_C = tuple(cfg["waterbutt_funnel_offset_c"])  # camera -> funnel, in c - see config.yaml's comment
ARUCO_WS_URL = f"ws://127.0.0.1:{aruco_cfg['port']}/ws/aruco"  # server-to-server, same machine — 127.0.0.1 not localhost: simple_websocket's raw client, unlike requests/curl, doesn't fall back from IPv6 ::1 to IPv4 on refusal
QC_RECONNECT_DELAY_S = 2.0

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

_qc_lock = threading.Lock()
_qc_reading = {"state": "not_configured"}


def _set_qc_reading(reading):
    global _qc_reading
    with _qc_lock:
        _qc_reading = reading


def get_qc_reading():
    with _qc_lock:
        return dict(_qc_reading)


def _qc_loop():
    """Connects to aruco's own /ws/aruco as a client (server-to-server,
    like every other cross-service feed in this project, just over a
    websocket rather than FeedClient's Unix socket) and keeps
    get_qc_reading() current - the rotation-aware comparison itself
    (qc_check.compare) needs real matrix math (see coords.py's
    qc_marker_delta_body_frame), so it stays in Python rather than being
    reimplemented in the page's own JS the way most live comparisons in
    this project are."""
    while True:
        try:
            ws = WsClient.connect(ARUCO_WS_URL)
            while True:
                msg = json.loads(ws.receive())
                saved = qc_marker.load(QC_MARKER_PATH)
                _set_qc_reading(qc_check.compare(saved, msg.get("debug"), QC_HPR_CB, QC_FUNNEL_OFFSET_C))
        except Exception as e:
            app.logger.warning("[waterbutt] QC marker: lost/couldn't reach aruco's feed (%s), retrying", e)
            _set_qc_reading({"state": "aruco_unreachable"})
            time.sleep(QC_RECONNECT_DELAY_S)


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
        "aruco_ws_url": service_url(browser_host, "aruco", scheme="ws") + "/ws/aruco",
    }


# --- QC marker (independent-of-GNSS pre-fill sanity check) ---
#
# Deliberately just one saved record, not a list - one waterbutt, one
# marker allowed for this check (see waterbutt-prd.md's "QC marker").


@app.route("/api/qc-marker")
def api_get_qc_marker():
    saved = qc_marker.load(QC_MARKER_PATH)
    if saved is not None:
        # Display-only convenience for the QC Marker page - a single
        # reading's own forward/right/down, same formula aruco itself
        # uses (not the ideal-vs-live comparison, which needs both
        # readings' rvec - see qc_check.compare).
        displacement = np.array(QC_DXC_B) + coords.displacement_camera_to_body(saved["tvec_camera_frame"], QC_HPR_CB)
        saved = {**saved, "displacement_body_frame": displacement.tolist()}
    return jsonify(saved)


@app.route("/api/qc-marker", methods=["POST"])
def api_save_qc_marker():
    payload = request.get_json(force=True)
    qc_marker.save(QC_MARKER_PATH, int(payload["marker_id"]), payload["tvec_camera_frame"], payload["rvec_camera_frame"])
    return jsonify(qc_marker.load(QC_MARKER_PATH))


def _qc_refusal_reason(reading: dict, threshold_m: float) -> str:
    state = reading.get("state")
    if state == "not_configured":
        return "no QC marker configured - see the QC Marker page"
    if state == "not_visible":
        return f"QC marker {reading.get('marker_id')} not visible"
    if state == "aruco_unreachable":
        return "can't reach aruco to check the QC marker"
    if state == "ok":
        return (
            f"{reading['distance_m'] * 100:.1f}cm from ideal, "
            f"exceeds the {threshold_m * 100:.0f}cm limit"
        )
    return f"QC marker check failed (unexpected state {state!r})"


@app.route("/go", methods=["POST"])
def go():
    payload = request.get_json(force=True)
    duration_s = payload["duration_s"]
    if duration_s not in DURATIONS_S:
        abort(400)
    threshold_m = payload.get("qc_threshold_m", QC_DEFAULT_THRESHOLD_M)
    if threshold_m not in QC_THRESHOLD_OPTIONS_M:
        abort(400)

    # No QC marker means no fill - a missing/unreachable/out-of-range
    # check refuses the same as a too-far-away one, never silently
    # skipped. See waterbutt-prd.md's "QC gating on fill".
    reading = get_qc_reading()
    if not qc_check.passes(reading, threshold_m):
        return jsonify({"ok": False, "reason": _qc_refusal_reason(reading, threshold_m)}), 409

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
        ws.send(json.dumps({**valve.status(), "qc_marker": get_qc_reading()}))
        time.sleep(period)


def run_context():
    return {"durations_s": DURATIONS_S, "qc_threshold_options_m": QC_THRESHOLD_OPTIONS_M, "qc_default_threshold_m": QC_DEFAULT_THRESHOLD_M}


register_pages(app, PAGES_DIR, index_slug="run", context_providers={"run": run_context})


if __name__ == "__main__":
    threading.Thread(target=_tick_loop, daemon=True).start()
    threading.Thread(target=_qc_loop, daemon=True).start()
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
