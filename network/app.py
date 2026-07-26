"""GR6-v2 network: a thin web UI over NetworkManager (`nmcli`) for
amundsen's network interfaces — DHCP/static, wifi client/hotspot, and a
per-interface "recovery" safety net for changes that might lock the
operator out. See network-prd.md for the design/reasoning.

This service owns no network state itself — `nm.py` reads/writes
NetworkManager directly, and `recovery.py` is the only state genuinely
local to this service (which connection each interface falls back to).
"""

import sys
from pathlib import Path

from flask import Flask, flash, redirect, render_template, request, url_for

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.web import manager_url, use_shared_static, use_shared_templates  # noqa: E402

import nm  # noqa: E402
import recovery  # noqa: E402

cfg = load_config()
service_cfg = cfg["services"]["network"]

app = Flask(__name__)
app.secret_key = "gr6-network"  # only used for flash() error messages — trusted LAN, no auth, see manager-prd.md
use_shared_templates(app)
use_shared_static(app)


@app.context_processor
def inject_manager_url():
    return {"manager_url": manager_url(request.host.split(":")[0])}


def _connection_name_for(device: str) -> str:
    """The connection profile this service creates/manages for a given
    device — kept separate from whatever profile predates this app
    (e.g. wlan0's existing "preconfigured"), so applying a change here
    never silently edits a connection the operator didn't create
    through this page."""
    return f"gr6-{device}"


@app.route("/")
def index():
    interfaces = nm.list_interfaces()
    recovery_map = recovery.load_recovery()
    all_connections = nm.list_connection_names()
    for iface in interfaces:
        iface.update(nm.interface_detail(iface["device"], iface["connection"]))
        iface["recovery_connection"] = recovery_map.get(iface["device"], "")
    return render_template(
        "index.html",
        interfaces=interfaces,
        all_connections=all_connections,
        revert_pending=recovery.revert_pending(),
        revert_timeout_s=service_cfg["revert_timeout_s"],
    )


@app.route("/interface/<device>/apply", methods=["POST"])
def apply_interface(device):
    interfaces = {i["device"]: i for i in nm.list_interfaces()}
    if device not in interfaces:
        return "Unknown device", 404

    form = request.form
    connection_name = _connection_name_for(device)
    ipv4_method = "manual" if form.get("ipv4_mode") == "static" else "auto"
    address = form.get("address") or None
    gateway = form.get("gateway") or None

    try:
        if interfaces[device]["type"] == "wifi":
            if form.get("mode") == "hotspot":
                nm.apply_hotspot(
                    device, connection_name, form["ssid"], form["password"],
                    address=address or "192.168.4.1/24",
                )
            else:
                nm.apply_wifi_client(
                    device, connection_name, form["ssid"], form["password"],
                    ipv4_method=ipv4_method, address=address, gateway=gateway,
                )
        else:
            nm.apply_ethernet(device, connection_name, ipv4_method=ipv4_method, address=address, gateway=gateway)
    except nm.NmError as e:
        # A failed apply (bad password, unreachable SSID, ...) is an
        # expected, recoverable outcome here, not a server error — show
        # it on the page rather than a stack trace. Note this can still
        # leave a half-broken connection profile behind (created but
        # never activated) — harmless, just re-apply to fix it.
        flash(f"{device}: {e}", "danger")
        return redirect(url_for("index"))

    # Any change might have just broken the operator's own access to this
    # page — see network-prd.md's "Recovery / safe-apply". Always arm the
    # countdown, even if *this* device has no recovery profile defined,
    # since another device's defined profile may still need to fire.
    recovery.schedule_revert(service_cfg["revert_timeout_s"])
    return redirect(url_for("index"))


@app.route("/interface/<device>/recovery", methods=["POST"])
def set_recovery(device):
    connection_name = request.form.get("recovery_connection") or None
    recovery.set_recovery(device, connection_name)
    return redirect(url_for("index"))


@app.route("/confirm", methods=["POST"])
def confirm():
    recovery.cancel_pending()
    return redirect(url_for("index"))


if __name__ == "__main__":
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
