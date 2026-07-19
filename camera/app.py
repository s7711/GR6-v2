"""GR6-v2 camera: captures frames from the Pi camera and publishes them
for any other service to consume (shared memory), plus a small web UI
for a live preview and basic status.

See camera-prd.md for the requirements this implements.
"""

import io
import json
import sys
import threading
import time
from pathlib import Path

from flask import Flask, Response, request
from flask_sock import Sock
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.frame_ipc import FrameWriter  # noqa: E402
from shared.web import manager_url, register_pages, use_shared_static, use_shared_templates  # noqa: E402

from bg_camera import RESOLUTION, BgCamera  # noqa: E402
from calibration import CalibrationSession  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
CAMERA_STATUS_HZ = 2  # Stat readouts refresh rate — independent of capture rate
CALIBRATE_STATUS_HZ = 2

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)

cfg = load_config()
service_cfg = cfg["services"]["camera"]
camera_fps = service_cfg["camera_fps"]

cam = BgCamera(fps=camera_fps)
writer = FrameWriter(service_cfg["camera_shm_name"], *RESOLUTION)
calibration_session = CalibrationSession(cam)


def publish_loop():
    while True:
        frame, timestamp, exposure_us, gain, _sequence = cam.latest()
        writer.publish(frame, timestamp, exposure_us or 0, gain or 0.0)


@app.context_processor
def inject_manager_url():
    return {"manager_url": manager_url(request.host.split(":")[0])}


def mjpeg_generator():
    while True:
        frame, _timestamp, _exposure_us, _gain, _sequence = cam.latest()
        # picamera2's "RGB888" format is actually BGR byte order (a known
        # picamera2 quirk, kept for cv2 compatibility elsewhere) — reverse
        # the channel axis so colours display correctly here.
        buf = io.BytesIO()
        Image.fromarray(frame[:, :, ::-1]).save(buf, format="JPEG")
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.getvalue() + b"\r\n"


@app.route("/camera.mjpg")
def camera_mjpg():
    # direct_passthrough=True is required for an infinite generator response
    # — without it, Werkzeug tries to buffer/measure the whole body first
    # (to set Content-Length), which never returns for a stream like this.
    return Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        direct_passthrough=True,
    )


def calibration_mjpeg_generator():
    while True:
        frame = calibration_session.overlay_frame()
        buf = io.BytesIO()
        Image.fromarray(frame[:, :, ::-1]).save(buf, format="JPEG")
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.getvalue() + b"\r\n"


@app.route("/calibration.mjpg")
def calibration_mjpg():
    return Response(
        calibration_mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        direct_passthrough=True,
    )


@app.route("/calibrate/start", methods=["POST"])
def calibrate_start():
    calibration_session.start()
    return "", 204


@app.route("/calibrate/abort", methods=["POST"])
def calibrate_abort():
    calibration_session.abort()
    return "", 204


@app.route("/calibrate/promote", methods=["POST"])
def calibrate_promote():
    ok = calibration_session.promote()
    return ("", 204) if ok else ("", 409)


@sock.route("/ws/calibrate")
def ws_calibrate(ws):
    period = 1.0 / CALIBRATE_STATUS_HZ
    while True:
        ws.send(json.dumps(calibration_session.snapshot()))
        time.sleep(period)


def home_context():
    return {"camera_fps": camera_fps}


def config_context():
    return {"resolution": f"{RESOLUTION[0]}x{RESOLUTION[1]}", "camera_fps": camera_fps}


register_pages(
    app,
    PAGES_DIR,
    index_slug="home",
    context_providers={
        "home": home_context,
        "config": config_context,
    },
)


@sock.route("/ws/camera")
def ws_camera(ws):
    period = 1.0 / CAMERA_STATUS_HZ
    while True:
        ws.send(json.dumps(cam.snapshot()))
        time.sleep(period)


if __name__ == "__main__":
    threading.Thread(target=publish_loop, daemon=True).start()
    try:
        app.run(host=service_cfg["host"], port=service_cfg["port"])
    except KeyboardInterrupt:
        pass
    finally:
        # Without this, Ctrl+C leaves the shared memory segment unlinked
        # by us — harmless (resource_tracker unlinks it anyway, and the
        # next startup's unlink-if-stale handles it regardless), but it
        # prints a "leaked shared_memory" warning every time. Clean up
        # explicitly so that warning stops showing up at all.
        cam.stop()
        writer.close()
