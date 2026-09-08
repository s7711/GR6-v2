"""GR6-v2 viewer: a generic cross-app log browser/plotter. Auto-discovers
any service directory with a `data/logs/` folder (navigate, waterbutt,
jobs, missions, drive, ...) — no per-app registration needed, so a new
service gains a viewer entry just by writing its logs in the same
`yymmdd_hhmmss*.jsonl` convention everyone else already uses. Read-only:
this service never writes to another app's data. See viewer-prd.md.
"""

import sys
from pathlib import Path

from flask import Flask, abort, jsonify, request, send_from_directory

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.logs import log_summaries  # noqa: E402
from shared.web import register_pages, service_url, use_shared_static, use_shared_templates  # noqa: E402

PAGES_DIR = Path(__file__).resolve().parent / "templates" / "pages"
REPO_ROOT = Path(__file__).resolve().parent.parent
# Directories that are never a service's own log source, even though they
# sit alongside the real service directories at the repo root.
_EXCLUDED_DIRS = {"shared", "venv", "viewer"}

app = Flask(__name__)
use_shared_templates(app)
use_shared_static(app)

cfg = load_config()
service_cfg = cfg["services"]["viewer"]


def discover_sources() -> dict:
    """{source_name: logs_dir}, recomputed on every call rather than
    cached - a source's logs/ folder can appear after this service has
    already started (e.g. drive's first log, or a future computer-vision
    app's first run), and the discovery itself is cheap (a handful of
    `is_dir()` checks)."""
    sources = {}
    for entry in sorted(REPO_ROOT.iterdir()):
        if not entry.is_dir() or entry.name.startswith(".") or entry.name in _EXCLUDED_DIRS:
            continue
        logs_dir = entry / "data" / "logs"
        if logs_dir.is_dir():
            sources[entry.name] = logs_dir
    return sources


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


@app.route("/api/logs")
def api_logs():
    return jsonify({source: log_summaries(logs_dir) for source, logs_dir in discover_sources().items()})


@app.route("/api/logs/<source>/<filename>")
def api_log_file(source, filename):
    logs_dir = discover_sources().get(source)
    if logs_dir is None or filename not in {p.name for p in logs_dir.glob("*.jsonl")}:
        abort(404)
    return send_from_directory(logs_dir, filename)


register_pages(app, PAGES_DIR, index_slug="home")


if __name__ == "__main__":
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
