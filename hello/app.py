"""Minimal example child service — proves out the shared-config pattern
(reads its own host/port from ../config.yaml rather than hardcoding it),
otherwise just a Flask 'hello world'.
"""

import sys
from pathlib import Path

from flask import Flask

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import service_config  # noqa: E402

app = Flask(__name__)


@app.route("/")
def index():
    return "<h1>Hello, world!</h1>"


if __name__ == "__main__":
    cfg = service_config("hello")
    app.run(host=cfg["host"], port=cfg["port"])
