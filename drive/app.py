"""GR6-v2 drive: talks to the motor-controller microcontroller over USB
serial — motor velocity, water pump, ultrasonic ranges, encoder/PID
telemetry — plus a small web UI (manual jog, tuning, config) and a
Unix-socket feed for other services to consume.

See drive-prd.md for the requirements this implements.
"""

import json
import logging
import sys
import threading
import time
from pathlib import Path

from flask import Flask, abort, jsonify, request
from flask_sock import Sock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.web import manager_url, register_pages, use_shared_static, use_shared_templates  # noqa: E402

import protocol  # noqa: E402
from control import AUTO, ControlArbiter, MANUAL  # noqa: E402
from feed import DriveFeedServer  # noqa: E402
from serial_link import SerialLink  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
DRIVE_STATUS_HZ = 5
FIRMWARE_LOG_TIMEOUT = 5.0  # seconds to wait, in the background, before logging "no version seen"

# The jog page posts a command roughly every 300ms while held (see
# home.html) — still enough to flood the systemd journal via Flask/
# Werkzeug's default per-request access log ("POST /command/manual
# HTTP/1.1 200"). Drop those to WARNING; our own logging.warning() calls
# (e.g. a firmware version mismatch) still come through.
logging.getLogger("werkzeug").setLevel(logging.WARNING)

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)

cfg = load_config()
service_cfg = cfg["services"]["drive"]

COUNTS_PER_METRE = service_cfg["counts_per_metre"]
MAX_MOTOR_COUNTS_S = 200  # firmware's own clamp — see drive-prd.md
MAX_SPEED_MPS = MAX_MOTOR_COUNTS_S / COUNTS_PER_METRE

# Not constructed here — opening the serial port is a real, physical side
# effect (on this project's actual machine, a real Arduino with real motors
# attached to real hardware). Built in the `__main__` guard below for a real
# run; tests construct their own `SerialLink` with a fake serial object and
# assign it here before exercising routes, so importing this module never
# touches real hardware. See serial_link.py/test_app.py.
link: SerialLink = None
arbiter = ControlArbiter(hold_seconds=service_cfg["human_control_hold_ms"] / 1000.0)


def mps_to_counts_s(mps: float) -> float:
    return mps * COUNTS_PER_METRE


def counts_s_to_mps(counts_s: float) -> float:
    return counts_s / COUNTS_PER_METRE if COUNTS_PER_METRE else 0.0


def push_configured_tuning():
    for name, values in service_cfg.get("tuning", {}).items():
        if name not in protocol.ALL_TUNING_PARAMS:
            logging.warning("[drive] Ignoring unknown tuning parameter in config: %s", name)
            continue
        left, right = values
        link.send(protocol.encode_tuning(name, left, right))


def log_firmware_version_once():
    """Runs once in a background thread at startup, purely to log a
    warning early if something's off — doesn't gate startup and doesn't
    freeze its result. The firmware only reports its Version line once
    every ~1.8s (it's one of several telemetry tags cycled round-robin),
    so this can't be a quick blocking check without risking a false
    "not received" on nothing worse than bad luck in the cycle's timing
    — that was the bug that made drive's own Home page banner get stuck
    on "no version received" even after the real version had since
    arrived. The banner itself is computed fresh from live state every
    time (see `_snapshot`), so this function only needs to log, not
    store anything."""
    deadline = time.monotonic() + FIRMWARE_LOG_TIMEOUT
    while time.monotonic() < deadline:
        actual = link.snapshot().get("firmware_version")
        if actual is not None:
            expected = service_cfg.get("expected_firmware_version")
            if expected is not None and actual != expected:
                logging.warning("[drive] Firmware version mismatch: expected %s, got %s", expected, actual)
            return
        time.sleep(0.2)
    logging.warning(
        "[drive] No firmware Version telemetry received within %.1fs of startup", FIRMWARE_LOG_TIMEOUT
    )


@app.context_processor
def inject_manager_url():
    return {"manager_url": manager_url(request.host.split(":")[0])}


# Fields that are firmware-native counts/counts-per-second, alongside the
# physical-unit (m or m/s) name added to the published snapshot. Kept
# alongside the raw fields, not replacing them — raw counts remain useful
# for hardware debugging, but every consumer outside `drive` should read
# the *_m/*_mps fields (see drive-prd.md, "Units").
_PHYSICAL_UNIT_FIELDS = {
    "LM_position": "LM_position_m",
    "RM_position": "RM_position_m",
    "LM_setvel": "LM_setvel_mps",
    "RM_setvel": "RM_setvel_mps",
    "LM_vel_filt": "LM_vel_filt_mps",
    "RM_vel_filt": "RM_vel_filt_mps",
    "LM_err": "LM_err_mps",
    "RM_err": "RM_err_mps",
}


def _firmware_status(state: dict) -> dict:
    actual = state.get("firmware_version")
    expected = service_cfg.get("expected_firmware_version")
    mismatch = actual is not None and expected is not None and actual != expected
    return {"expected": expected, "actual": actual, "mismatch": mismatch}


def _snapshot():
    state = link.snapshot()
    combined = dict(state)
    combined["control"] = arbiter.status()
    combined["firmware"] = _firmware_status(state)
    for raw_field, physical_field in _PHYSICAL_UNIT_FIELDS.items():
        if raw_field in state:
            combined[physical_field] = counts_s_to_mps(state[raw_field])
    return combined


def _command(source: str):
    payload = request.get_json(force=True)
    left_mps = float(payload["left_mps"])
    right_mps = float(payload["right_mps"])
    accepted = arbiter.try_command(source)
    if accepted:
        link.send(protocol.encode_set_velocity(mps_to_counts_s(left_mps), mps_to_counts_s(right_mps)))
    return jsonify({"accepted": accepted})


@app.route("/command/manual", methods=["POST"])
def command_manual():
    return _command(MANUAL)


@app.route("/command/auto", methods=["POST"])
def command_auto():
    return _command(AUTO)


@app.route("/pump", methods=["POST"])
def pump():
    payload = request.get_json(force=True)
    link.send(protocol.encode_pump(bool(payload["on"])))
    return "", 204


@app.route("/tuning", methods=["POST"])
def set_tuning():
    payload = request.get_json(force=True)
    name = payload["name"]
    if name not in protocol.ALL_TUNING_PARAMS:
        abort(400)
    link.send(protocol.encode_tuning(name, float(payload["left"]), float(payload["right"])))
    return "", 204


@sock.route("/ws/drive")
def ws_drive(ws):
    period = 1.0 / DRIVE_STATUS_HZ
    while True:
        ws.send(json.dumps(_snapshot()))
        time.sleep(period)


def home_context():
    return {"max_speed_mps": MAX_SPEED_MPS}


def tuning_context():
    return {"tuning_params": sorted(protocol.ALL_TUNING_PARAMS)}


def config_context():
    return {
        "serial_port": service_cfg["serial_port"],
        "baud": service_cfg["baud"],
        "counts_per_metre": COUNTS_PER_METRE,
        "human_control_hold_ms": service_cfg["human_control_hold_ms"],
        "expected_firmware_version": service_cfg.get("expected_firmware_version"),
        "tuning": service_cfg.get("tuning", {}),
    }


register_pages(
    app,
    PAGES_DIR,
    index_slug="home",
    context_providers={
        "home": home_context,
        "tuning": tuning_context,
        "config": config_context,
    },
)


if __name__ == "__main__":
    link = SerialLink(service_cfg["serial_port"], service_cfg["baud"])
    link.start()
    push_configured_tuning()
    threading.Thread(target=log_firmware_version_once, daemon=True).start()

    feed = DriveFeedServer(service_cfg["drive_feed_socket"], _snapshot, service_cfg["drive_feed_hz"])
    feed.start()

    app.run(host=service_cfg["host"], port=service_cfg["port"])
