"""GR6-v2 waterbutt: a small web UI + timed-duration control over the
water butt's ESP8266 pinch-valve controller (documentation/
Waterbutt_261112-3.ino) — the firmware itself only exposes "open"/
"close" with its own 5-second fail-safe auto-close, so this service's
job is entirely the duration timing (see control.py's ValveController)
plus the page. See waterbutt-prd.md for the requirements this
implements.
"""

import datetime
import json
import socket
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
from shared.feed_client import FeedClient  # noqa: E402
from shared.web import register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

sys.path.append(str(Path(__file__).resolve().parent.parent / "aruco"))  # append, not insert(0) - see jobs/continuity.py's comment: insert(0) here risked shadowing this directory's own same-named modules in a same-process test run
import coords  # noqa: E402

import qc_check  # noqa: E402
import qc_marker  # noqa: E402
from control import ValveController  # noqa: E402
from level import LevelEstimate  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
WATERBUTT_STATUS_HZ = 2
TICK_HZ = 1  # see control.py's REOPEN_INTERVAL_S comment - this needs to keep up with a sub-1s reopen interval
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
drive_cfg = cfg["services"]["drive"]

# Used for a /go call that doesn't pick a tolerance at all (e.g. jobs'
# `fill` step, which has no threshold selector of its own yet) - a
# config value, not a hardcoded constant, since this is exactly the
# kind of thing worth tuning without a code change. See
# waterbutt-prd.md's "QC gating on fill".
QC_DEFAULT_THRESHOLD_M = service_cfg["qc_default_threshold_m"]

VALVE_HOSTNAME = service_cfg["hostname"]
QC_MARKER_PATH = Path(__file__).resolve().parent.parent / service_cfg["qc_marker_file"]
LOGS_DIR = Path(__file__).resolve().parent / "data" / "logs"
QC_HPR_CB = tuple(aruco_cfg["camera_extrinsics"]["hpr_cb"])
QC_DXC_B = tuple(aruco_cfg["camera_extrinsics"]["d_xc_b"])
QC_FUNNEL_OFFSET_C = tuple(cfg["waterbutt_funnel_offset_c"])  # camera -> funnel, in c - see config.yaml's comment
ARUCO_WS_URL = f"ws://127.0.0.1:{aruco_cfg['port']}/ws/aruco"  # server-to-server, same machine — 127.0.0.1 not localhost: simple_websocket's raw client, unlike requests/curl, doesn't fall back from IPv6 ::1 to IPv4 on refusal
QC_RECONNECT_DELAY_S = 2.0

# See level.py/waterbutt-prd.md's "Tank level estimate" - drive's own
# `pump` telemetry (not just whether *this* service commanded it), so a
# manually-jogged pump run from drive's own page counts too.
drive_client = FeedClient(drive_cfg["drive_feed_socket"], default={})
level = LevelEstimate(service_cfg["drain_confirm_s"])

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)


# `VALVE_HOSTNAME` is mDNS (avahi/nss-mdns) - resolving it fresh on
# every single /open call (2026-08-15: measured live, ~1 in 20-30
# calls) occasionally stalls for 2.5-2.8s, eating almost all of
# VALVE_TIMEOUT_S before the actual HTTP request even starts. A
# phone/browser hitting the same hostname doesn't show this because it
# caches the resolved address far longer than a bare requests.get()
# call does. Resolved once and cached as an IP instead; re-resolved
# only after a request actually fails (the ESP8266's DHCP address can
# shift too, same as amundsen's own wifi IP did) rather than on every
# call.
_valve_ip_lock = threading.Lock()
_valve_ip = None


def _resolve_valve_ip():
    global _valve_ip
    try:
        ip = socket.gethostbyname(VALVE_HOSTNAME)
    except OSError as e:
        app.logger.warning("[waterbutt] Couldn't resolve valve hostname %s: %s", VALVE_HOSTNAME, e)
        return None
    with _valve_ip_lock:
        _valve_ip = ip
    return ip


def _valve_base_url():
    with _valve_ip_lock:
        ip = _valve_ip
    if ip is None:
        ip = _resolve_valve_ip()
    return f"http://{ip}" if ip else None


def send_open():
    base = _valve_base_url()
    if base is None:
        app.logger.warning("[waterbutt] Couldn't reach the valve to open it (no address)")
        return
    try:
        requests.get(f"{base}/open", timeout=VALVE_TIMEOUT_S, allow_redirects=False)
    except requests.exceptions.RequestException:
        app.logger.warning("[waterbutt] Couldn't reach the valve to open it")
        _resolve_valve_ip()  # address may have moved - pick that up for the next attempt, not this one


def send_close():
    base = _valve_base_url()
    if base is None:
        app.logger.warning("[waterbutt] Couldn't reach the valve to close it (no address)")
        return
    try:
        requests.get(f"{base}/close", timeout=VALVE_TIMEOUT_S, allow_redirects=False)
    except requests.exceptions.RequestException:
        _resolve_valve_ip()
        app.logger.warning("[waterbutt] Couldn't reach the valve to close it")


valve = ValveController(send_open, send_close)

_qc_lock = threading.Lock()
_qc_reading = {"state": "not_configured"}

_qc_log_lock = threading.Lock()
_qc_log_path = None
_qc_log_opened_at = None  # time.monotonic() - drives the periodic rotation below


def _set_qc_reading(reading):
    global _qc_reading
    with _qc_lock:
        _qc_reading = reading
    _append_qc_log(reading)


def get_qc_reading():
    with _qc_lock:
        return dict(_qc_reading)


def _start_new_qc_log():
    """One file per service run (this service has no discrete "run"
    the way navigate/jobs/missions do — the QC reading just streams
    continuously whenever aruco is reachable), named the same
    yymmdd_hhmmss way as navigate's per-run debug log, so the two can
    be lined up afterward by timestamp. Also rotated periodically while
    the service keeps running (see _append_qc_log) - a continuous
    stream has no other natural end, so without this a long-
    uninterrupted service run would grow one file forever and
    log_retention_days' startup-only sweep would never get a chance to
    reclaim any of it. Found live 2026-08-12 - noticed growing with no
    fill attempt in progress."""
    global _qc_log_path, _qc_log_opened_at
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
    candidate = LOGS_DIR / f"{timestamp}.jsonl"
    suffix = 1
    while candidate.exists():
        # A rotation landing in the same second as the file it's
        # replacing (e.g. two tests running fast) - never silently
        # overwrite an existing file.
        candidate = LOGS_DIR / f"{timestamp}_{suffix}.jsonl"
        suffix += 1
    _qc_log_path = candidate
    _qc_log_opened_at = time.monotonic()


def _append_qc_log(reading):
    if _qc_log_path is None:
        return
    if time.monotonic() - _qc_log_opened_at > service_cfg["qc_log_rotate_s"]:
        # Piggyback the rotation check on the natural write cadence
        # (every reading, up to a few Hz whenever aruco is reachable)
        # rather than a separate timer thread - see _start_new_qc_log's
        # docstring for why this exists at all.
        _sweep_old_qc_logs()
        _start_new_qc_log()
    with _qc_log_lock:
        with open(_qc_log_path, "a") as f:
            f.write(json.dumps({"t": time.time(), **reading}, default=str) + "\n")


def _sweep_old_qc_logs():
    """Run once at startup, same pattern as navigate/jobs/missions."""
    if not LOGS_DIR.exists():
        return
    cutoff = time.time() - service_cfg["log_retention_days"] * 86400
    for file in LOGS_DIR.glob("*.jsonl"):
        if file.stat().st_mtime < cutoff:
            file.unlink()


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
        level.set_pump_on(bool(drive_client.latest().get("pump")))
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

    # Not confident the butt is actually empty - refuse rather than
    # risk overflowing. See waterbutt-prd.md's "Tank level estimate".
    if not level.believed_empty():
        remaining_s = service_cfg["drain_confirm_s"] - level.status()["pump_seconds_since_full"]
        return jsonify({"ok": False, "reason": f"not confident the butt is empty yet ({remaining_s:.0f}s more pump time needed, or Mark empty)"}), 409

    valve.go(duration_s)
    level.mark_full()  # a fill starting means it's (about to be) full again - see level.py's "reset"
    return jsonify(valve.status())


@app.route("/level/mark-full", methods=["POST"])
def level_mark_full():
    level.mark_full()
    return jsonify(level.status())


@app.route("/level/mark-empty", methods=["POST"])
def level_mark_empty():
    level.mark_empty()
    return jsonify(level.status())


@app.route("/stop", methods=["POST"])
def stop():
    valve.stop()
    return jsonify(valve.status())


@sock.route("/ws/waterbutt")
def ws_waterbutt(ws):
    period = 1.0 / WATERBUTT_STATUS_HZ
    while True:
        ws.send(json.dumps({**valve.status(), "qc_marker": get_qc_reading(), "level": level.status()}))
        time.sleep(period)


def run_context():
    return {"durations_s": DURATIONS_S, "qc_threshold_options_m": QC_THRESHOLD_OPTIONS_M, "qc_default_threshold_m": QC_DEFAULT_THRESHOLD_M}


register_pages(app, PAGES_DIR, index_slug="run", context_providers={"run": run_context})


if __name__ == "__main__":
    _sweep_old_qc_logs()
    _start_new_qc_log()
    drive_client.start()
    threading.Thread(target=_tick_loop, daemon=True).start()
    threading.Thread(target=_qc_loop, daemon=True).start()
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
