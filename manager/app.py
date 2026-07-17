"""GR6-v2 manager: home icon-grid + services status/control page.

See manager-prd.md for the requirements this implements. A few details
were left "TBD" in the PRD and are pinned down here as first-draft
assumptions — flagged in the accompanying message, not repeated as
comments throughout the code.
"""

import datetime
import json
import subprocess
import sys
import time
from pathlib import Path

import yaml
from flask import Flask, abort, render_template, request, send_from_directory
from flask_sock import Sock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import CONFIG_PATH, load_config  # noqa: E402
from shared.web import manager_url, use_shared_static, use_shared_templates  # noqa: E402

ALLOWED_ACTIONS = {"start", "stop", "restart"}
STATUS_POLL_SECONDS = 2
MANAGER_SERVICE_NAME = "manager"
CONFIG_BACKUP_DIR = Path(__file__).resolve().parent / "config-backup"

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)


@app.context_processor
def inject_manager_url():
    return {"manager_url": manager_url(request.host.split(":")[0])}


def services() -> dict:
    return load_config()["services"]


def icon_path(name: str) -> str | None:
    service_dir = Path(__file__).resolve().parent.parent / name
    for ext in ("svg", "png"):
        candidate = service_dir / f"icon.{ext}"
        if candidate.exists():
            return f"/icons/{name}/icon.{ext}"
    return None


def unit_status(unit: str) -> str:
    result = subprocess.run(
        ["systemctl", "is-active", unit],
        capture_output=True,
        text=True,
    )
    state = result.stdout.strip()
    if state == "active":
        return "running"
    if state == "failed":
        return "failed"
    return "stopped"


def control_unit(unit: str, action: str) -> None:
    subprocess.run(["sudo", "systemctl", action, unit], check=True)


@app.route("/")
def home():
    browser_host = request.host.split(":")[0]
    tiles = []
    for name, cfg in services().items():
        if name == MANAGER_SERVICE_NAME:
            continue
        tiles.append(
            {
                "name": name,
                "icon": icon_path(name),
                "url": f"http://{browser_host}:{cfg['port']}/" if cfg.get("web_ui") else None,
            }
        )
    return render_template("home.html", tiles=tiles)


@app.route("/services")
def services_page():
    rows = []
    for name, cfg in services().items():
        rows.append({"name": name, "unit": cfg["unit"], "status": unit_status(cfg["unit"])})
    return render_template("services.html", rows=rows)


@app.route("/icons/<name>/<filename>")
def icon(name, filename):
    if name not in services():
        abort(404)
    service_dir = Path(__file__).resolve().parent.parent / name
    return send_from_directory(service_dir, filename)


@app.route("/services/<name>/<action>", methods=["POST"])
def services_action(name, action):
    cfg = services().get(name)
    if cfg is None:
        abort(404)
    if action not in ALLOWED_ACTIONS:
        abort(400)
    control_unit(cfg["unit"], action)
    return "", 204


@app.route("/config", methods=["GET", "POST"])
def config_page():
    error = None
    config_text = CONFIG_PATH.read_text()
    if request.method == "POST":
        text = request.form["config_text"]
        try:
            yaml.safe_load(text)
        except yaml.YAMLError as e:
            error = f"Not saved — invalid YAML: {e}"
            config_text = text
        else:
            CONFIG_BACKUP_DIR.mkdir(exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
            backup_path = CONFIG_BACKUP_DIR / f"config_{timestamp}.yaml"
            backup_path.write_text(config_text)
            CONFIG_PATH.write_text(text)
            config_text = text
    return render_template("config.html", config_text=config_text, error=error)


@sock.route("/ws/status")
def ws_status(ws):
    while True:
        statuses = {name: unit_status(cfg["unit"]) for name, cfg in services().items()}
        ws.send(json.dumps(statuses))
        time.sleep(STATUS_POLL_SECONDS)


if __name__ == "__main__":
    cfg = load_config()["services"]["manager"]
    app.run(host=cfg["host"], port=cfg["port"])
