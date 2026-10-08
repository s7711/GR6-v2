"""GR6-v2 oxts-nav: decodes the xNAV650's NCOM or UCOM stream and publishes it.

See oxts-nav-prd.md for the requirements this implements. `ncomrx.py` and
`ncomrx_thread.py` are ported from GR6-v1 close to unchanged (see PRD);
everything else here is new — a thin publisher in place of GR6-v1's
xnav.py, built around this project's IPC/web conventions instead.

Which protocol is used (`ncom` or `ucom`) is set by config.yaml's
oxts-nav.protocol, read once at startup — see ncom-to-ucom-mapping.md.
Restart the service to switch; this isn't a live/dynamic toggle, same
as every other config value in this project (see top-prd.md's "Shared
configuration" decision).
"""

import ftplib
import io
import json
import logging
import socket
import sys
import threading
import time
from pathlib import Path

from flask import Flask, Response, abort, jsonify, request, send_from_directory
from flask_sock import Sock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.web import manager_url, register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

import ncomrx_thread  # noqa: E402
import ucomrx_thread  # noqa: E402
import nav_feed  # noqa: E402
import gnss_mode  # noqa: E402
import data_log  # noqa: E402
import xnav_ftp  # noqa: E402

XNAV_COMMAND_PORT = 3001
# mobile.rd is the xNAV's raw data recording, not a config file — it's
# renamed to a timestamped .rd file once time is available, so it should
# never be synced/edited here even though it matches "mobile.*".
XNAV_CONFIG_EXCLUDE = {"mobile.rd"}
XNAV_CONFIG_DIR = Path(__file__).resolve().parent / "xnav-config"
PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)

cfg = load_config()
xnav_ip = cfg["xnav_ip"]
service_cfg = cfg["services"]["oxts-nav"]
nav_update_hz = service_cfg["nav_update_hz"]

protocol = service_cfg["protocol"]
if protocol == "ncom":
    nrxs = ncomrx_thread.NcomRxThread()
elif protocol == "ucom":
    nrxs = ucomrx_thread.UcomRxThread()
else:
    raise ValueError(f"oxts-nav.protocol must be 'ncom' or 'ucom', got {protocol!r}")

nav_feed_server = nav_feed.NavFeedServer(
    socket_path=service_cfg["nav_feed_socket"],
    nrxs=nrxs,
    xnav_ip=xnav_ip,
    hz=service_cfg["nav_feed_hz"],
    stale_after_s=service_cfg["stale_after_s"],
)

DATA_DIR = Path(__file__).resolve().parent / "data"
data_logger = data_log.DataLogger(
    nrxs=nrxs,
    xnav_ip=xnav_ip,
    protocol=protocol,
    data_dir=DATA_DIR,
    decoded_fields=service_cfg["decoded_log_fields"],
    rotate_s=service_cfg["log_rotate_s"],
    retention_days=service_cfg["log_retention_days"],
    decoded_hz=service_cfg["decoded_log_hz"],
    stale_after_s=service_cfg["stale_after_s"],
)


def send_xnav_command(message: str) -> None:
    # No automatic sequence — this is only ever called for a manual,
    # occasional command typed by the operator, or (see gnss_mode.py)
    # navigate asking for aruco-priority mode. See oxts-nav-prd.md
    # ("xNAV650 commands") for why there's no dispatch logic here.
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.sendto((message + "\n").encode("utf-8"), (xnav_ip, XNAV_COMMAND_PORT))


aruco_cfg = cfg["services"]["aruco"]
# 127.0.0.1, not localhost: simple_websocket's raw client, unlike
# requests/curl, doesn't fall back from IPv6 ::1 to IPv4 on refusal -
# see waterbutt/app.py's identical comment on its own aruco feed client.
ARUCO_WS_URL = f"ws://127.0.0.1:{aruco_cfg['port']}/ws/aruco"
gnss_controller = gnss_mode.GnssModeController(
    send_xnav_command, ARUCO_WS_URL, service_cfg["aruco_priority_timeout_s"]
)


def download_xnav_config() -> None:
    XNAV_CONFIG_DIR.mkdir(exist_ok=True)
    try:
        with ftplib.FTP(xnav_ip, timeout=10) as ftp:
            ftp.login()
            try:
                names = list(xnav_ftp.list_files(ftp))
            except ftplib.all_errors as e:
                logging.info("Cannot list xNAV650 FTP directory: %s", e)
                return
            filenames = sorted(
                n for n in names if n.startswith("mobile.") and n not in XNAV_CONFIG_EXCLUDE
            )
            others = sorted(n for n in names if not n.startswith("mobile.") and not xnav_ftp.RD_NAME.match(n))
            if others:
                # e.g. a stray .ptp file — seen but not managed here.
                logging.info("xNAV650 FTP also has non-config files, left alone: %s", others)
            for filename in filenames:
                dest = XNAV_CONFIG_DIR / f"{filename}.txt"
                try:
                    with open(dest, "wb") as f:
                        ftp.retrbinary(f"RETR {filename}", f.write)
                except ftplib.all_errors as e:
                    logging.info("Cannot download %s: %s", filename, e)
    except OSError as e:
        logging.info("Cannot connect to xNAV650 FTP: %s", e)


def upload_xnav_config_file(filename: str, content: bytes) -> tuple[bool, str | None]:
    ftp_name = filename[: -len(".txt")]
    try:
        with ftplib.FTP(xnav_ip, timeout=10) as ftp:
            ftp.login()
            ftp.storbinary(f"STOR {ftp_name}", io.BytesIO(content))
    except ftplib.all_errors as e:
        return False, str(e)
    (XNAV_CONFIG_DIR / filename).write_bytes(content)
    return True, None


@app.context_processor
def inject_manager_url():
    browser_host = request.host.split(":")[0]
    return {
        "manager_url": manager_url(browser_host),
        # For the shared header's GNSS/Aruco status badges — see
        # shared/web/static/sysstatus.js. oxts-nav's own pages loop back
        # to themselves here, same as any other service's — harmless.
        "oxtsnav_ws_url": service_url(browser_host, "oxts-nav", scheme="ws") + "/ws/nav",
        "aruco_ws_url": service_url(browser_host, "aruco", scheme="ws") + "/ws/aruco",
        "drive_ws_url": service_url(browser_host, "drive", scheme="ws") + "/ws/drive",  # battery badge - see sysstatus.js
        "wheelspeed_ws_url": service_url(browser_host, "wheelspeed", scheme="ws") + "/ws/wheelspeed",  # "W" badge - see sysstatus.js
    }


@app.route("/command", methods=["POST"])
def command():
    message = request.form["message"]
    send_xnav_command(message)
    return "", 204


def xnav_config_context():
    files = sorted(p.name for p in XNAV_CONFIG_DIR.glob("*.txt")) if XNAV_CONFIG_DIR.exists() else []
    return {"files": files}


register_pages(
    app,
    PAGES_DIR,
    index_slug="home",
    context_providers={
        "home": lambda: {"nav_update_hz": nav_update_hz},
        "emi-monitor": lambda: {"nav_update_hz": nav_update_hz},  # temporary interference-testing page, 2026-10-02
        "xnav-config": xnav_config_context,
    },
)


@app.route("/xnav-config/<filename>", methods=["GET", "POST"])
def xnav_config_file(filename):
    if request.method == "GET":
        return send_from_directory(XNAV_CONFIG_DIR, filename)

    if not filename.endswith(".txt") or filename not in {p.name for p in XNAV_CONFIG_DIR.glob("*.txt")}:
        abort(404)
    ok, reason = upload_xnav_config_file(filename, request.get_data())
    if not ok:
        return jsonify(ok=False, reason=reason), 502
    return jsonify(ok=True)


@app.route("/xnav-rd/list")
def xnav_rd_list():
    """The xNAV's raw log (.rd) files, newest first - for the xNAV Config
    page's download list (e.g. to send OxTS). Names are the xNAV's own
    start time, YYMMDD_HHMMSS, so name order is time order."""
    try:
        with ftplib.FTP(xnav_ip, timeout=10) as ftp:
            ftp.login()
            files = xnav_ftp.list_files(ftp)
    except (OSError, ftplib.all_errors) as e:
        return jsonify(ok=False, reason=str(e)), 502
    rd = sorted(((n, size) for n, size in files.items() if xnav_ftp.RD_NAME.match(n)), reverse=True)
    return jsonify(ok=True, files=[{"name": n, "size": size} for n, size in rd])


@app.route("/xnav-rd/<name>")
def xnav_rd_download(name):
    """Streams one .rd file from the xNAV's FTP straight to the browser
    (some are over 1GB, so nothing is staged on the Pi's SD card)."""
    if not xnav_ftp.RD_NAME.match(name):
        abort(404)
    try:
        ftp = ftplib.FTP(xnav_ip, timeout=30)
        ftp.login()
        ftp.voidcmd("TYPE I")
        size = ftp.size(name)
        conn = ftp.transfercmd(f"RETR {name}")
    except (OSError, ftplib.all_errors):
        abort(404)

    def stream():
        try:
            while True:
                chunk = conn.recv(256 * 1024)
                if not chunk:
                    break
                yield chunk
        finally:
            conn.close()
            try:
                ftp.voidresp()
                ftp.quit()
            except (OSError, ftplib.all_errors):
                ftp.close()

    headers = {"Content-Disposition": f'attachment; filename="{name}"'}
    if size is not None:
        headers["Content-Length"] = str(size)
    return Response(stream(), mimetype="application/octet-stream", headers=headers)


@app.route("/xnav-config/reset", methods=["POST"])
def xnav_config_reset():
    # Config file changes only take effect on power-on/reset — see
    # oxts-nav-prd.md ("xNAV650 commands"). This is the button an
    # operator hits after editing/uploading files.
    send_xnav_command("!reset")
    return "", 204


@app.route("/gnss/aruco-priority", methods=["POST"])
def gnss_aruco_priority():
    gnss_controller.enter_aruco_priority()
    return "", 204


@app.route("/gnss/normal", methods=["POST"])
def gnss_normal():
    gnss_controller.exit_aruco_priority()
    return "", 204


@app.route("/gnss/status")
def gnss_status():
    return jsonify(gnss_controller.status())


@sock.route("/ws/nav")
def ws_nav(ws):
    period = 1.0 / nav_update_hz
    while True:
        ws.send(json.dumps(nav_feed.snapshot(nrxs, xnav_ip, service_cfg["stale_after_s"]), default=str))
        time.sleep(period)


if __name__ == "__main__":
    threading.Thread(target=download_xnav_config, daemon=True).start()
    nav_feed_server.start()
    gnss_controller.start()
    data_logger.start()
    # threaded=True: without it, Flask's dev server handles one connection
    # at a time, and /ws/nav's handler never returns (infinite loop) — so
    # a second simultaneous connection just hangs forever. Every page now
    # needs its own /ws/nav (for the shared header's "G" badge) *in
    # addition to* whatever a page like status.html already opens for
    # itself — on oxts-nav's own pages that's two concurrent connections
    # to this same process, which a single-threaded server can't serve at
    # once. Found live (2026-07-25) as "page load got much slower" right
    # after the header badges shipped.
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
