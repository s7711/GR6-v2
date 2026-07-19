"""GR6-v2 oxts-nav: decodes the xNAV650's NCOM stream and publishes it.

See oxts-nav-prd.md for the requirements this implements. `ncomrx.py` and
`ncomrx_thread.py` are ported from GR6-v1 close to unchanged (see PRD);
everything else here is new — a thin publisher in place of GR6-v1's
xnav.py, built around this project's IPC/web conventions instead.
"""

import ftplib
import json
import logging
import socket
import sys
import threading
import time
from pathlib import Path

from flask import Flask, request, send_from_directory
from flask_sock import Sock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.web import manager_url, register_pages, use_shared_static, use_shared_templates  # noqa: E402

import ncomrx_thread  # noqa: E402
import nav_feed  # noqa: E402

XNAV_COMMAND_PORT = 3001
XNAV_CONFIG_FILES = [
    "mobile.cfg",  # main configuration file
    "mobile.gap",  # GNSS antenna position
    "mobile.gpa",  # GNSS antenna position accuracy
    "mobile.vat",  # Vehicle attitude
    "mobile.vaa",  # Vehicle attitude accuracy
    "mobile.att",  # (GNSS) antenna attitude
    "mobile.ata",  # (GNSS) antenna attitude accuracy
]
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

nrxs = ncomrx_thread.NcomRxThread()

nav_feed_server = nav_feed.NavFeedServer(
    socket_path=service_cfg["nav_feed_socket"],
    nrxs=nrxs,
    xnav_ip=xnav_ip,
    hz=service_cfg["nav_feed_hz"],
)


def send_xnav_command(message: str) -> None:
    # No automatic sequence — this is only ever called for a manual,
    # occasional command typed by the operator. See oxts-nav-prd.md
    # ("xNAV650 commands") for why.
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.sendto((message + "\n").encode("utf-8"), (xnav_ip, XNAV_COMMAND_PORT))


def download_xnav_config() -> None:
    XNAV_CONFIG_DIR.mkdir(exist_ok=True)
    try:
        with ftplib.FTP(xnav_ip, timeout=10) as ftp:
            ftp.login()
            for filename in XNAV_CONFIG_FILES:
                dest = XNAV_CONFIG_DIR / f"{filename}.txt"
                try:
                    with open(dest, "wb") as f:
                        ftp.retrbinary(f"RETR {filename}", f.write)
                except ftplib.all_errors as e:
                    logging.info("Cannot download %s: %s", filename, e)
    except OSError as e:
        logging.info("Cannot connect to xNAV650 FTP: %s", e)


@app.context_processor
def inject_manager_url():
    return {"manager_url": manager_url(request.host.split(":")[0])}


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
        "xnav-config": xnav_config_context,
    },
)


@app.route("/xnav-config/<filename>")
def xnav_config_file(filename):
    return send_from_directory(XNAV_CONFIG_DIR, filename)


@sock.route("/ws/nav")
def ws_nav(ws):
    period = 1.0 / nav_update_hz
    while True:
        with nrxs.lock:
            decoder = nrxs.nrx.get(xnav_ip, {}).get("decoder")
            nav = dict(decoder.nav) if decoder else {}
            status = dict(decoder.status) if decoder else {}
            connection = dict(decoder.connection) if decoder else {}
        ws.send(json.dumps({"nav": nav, "status": status, "connection": connection}, default=str))
        time.sleep(period)


if __name__ == "__main__":
    threading.Thread(target=download_xnav_config, daemon=True).start()
    nav_feed_server.start()
    app.run(host=service_cfg["host"], port=service_cfg["port"])
