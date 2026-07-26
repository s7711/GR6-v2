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


# Form value for "None (disconnected)" in the "Set to" dropdown —
# distinct from "" (which set_recovery treats as "don't care"); see
# recovery.NONE for the equivalent Recovery-dropdown sentinel.
NONE_VALUE = "__none__"


@app.route("/")
def index():
    interfaces = nm.list_interfaces()
    recovery_map = recovery.load_recovery()
    all_connections = nm.list_connections()
    connection_descriptions = {c["name"]: nm.describe_connection(c["name"]) for c in all_connections}
    active = nm.active_connections()
    for iface in interfaces:
        iface.update(nm.interface_detail(iface["device"], iface["connection"]))
        iface["recovery_connection"] = recovery_map.get(iface["device"], "")
        # Only offer profiles of the same type (wifi/ethernet) — a wifi
        # profile can't be activated on an ethernet device or vice
        # versa, nmcli just refuses it (see nm.list_connections()).
        iface["matching_connections"] = [
            c["name"] for c in all_connections if c["device_type"] == iface["type"]
        ]
        # Flag profiles currently active on a *different* device so the
        # page can show it before the operator even tries to Apply —
        # the actual block still happens server-side in apply_interface.
        iface["connections_active_elsewhere"] = {
            name: dev for name, dev in active.items()
            if dev != iface["device"] and name in iface["matching_connections"]
        }
    return render_template(
        "index.html",
        interfaces=interfaces,
        connection_descriptions=connection_descriptions,
        revert_pending=recovery.revert_pending(),
        revert_timeout_s=service_cfg["revert_timeout_s"],
        none_value=NONE_VALUE,
        recovery_none_value=recovery.NONE,
    )


@app.route("/interface/<device>/apply", methods=["POST"])
def apply_interface(device):
    """Switch a device to an already-existing connection profile, or to
    "None (disconnected)" — the only way to change what's live on an
    interface in v2 (see network-prd.md's "Page shape (v2)"); creating
    a new profile happens on the separate new_wifi/new_ethernet pages."""
    interfaces = {i["device"]: i for i in nm.list_interfaces()}
    if device not in interfaces:
        return "Unknown device", 404

    connection_name = request.form.get("connection_name") or NONE_VALUE
    try:
        if connection_name == NONE_VALUE:
            nm.disconnect_device(device)
        else:
            active_elsewhere = nm.active_connections().get(connection_name)
            if active_elsewhere and active_elsewhere != device:
                flash(
                    f"'{connection_name}' is already active on {active_elsewhere} — "
                    "use a separate profile for each device.", "danger",
                )
                return redirect(url_for("index"))
            nm.activate_connection(connection_name, device=device)
    except nm.NmError as e:
        # A failed apply (bad password, unreachable SSID, ...) is an
        # expected, recoverable outcome here, not a server error — show
        # it on the page rather than a stack trace.
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
    try:
        recovery.set_recovery(device, connection_name)
    except recovery.RecoveryConflict as e:
        flash(str(e), "danger")
    return redirect(url_for("index"))


@app.route("/new/wifi", methods=["GET", "POST"])
def new_wifi():
    if request.method == "POST":
        form = request.form
        try:
            if form.get("mode") == "hotspot":
                nm.create_hotspot(
                    form["name"], form["ssid"], form["password"],
                    address=form.get("address") or "192.168.4.1/24",
                )
            else:
                nm.create_wifi_client(
                    form["name"], form["ssid"], form["password"],
                    ipv4_method="manual" if form.get("ipv4_mode") == "static" else "auto",
                    address=form.get("address") or None, gateway=form.get("gateway") or None,
                )
        except nm.NmError as e:
            flash(str(e), "danger")
            return redirect(url_for("new_wifi"))
        flash(f"Created '{form['name']}' — select it on the relevant interface's 'Set to' to use it.", "success")
        return redirect(url_for("index"))
    return render_template("new_wifi.html")


@app.route("/new/ethernet", methods=["GET", "POST"])
def new_ethernet():
    if request.method == "POST":
        form = request.form
        try:
            nm.create_ethernet(
                form["name"],
                ipv4_method="manual" if form.get("ipv4_mode") == "static" else "auto",
                address=form.get("address") or None, gateway=form.get("gateway") or None,
                share=form.get("share") == "on",
            )
        except nm.NmError as e:
            flash(str(e), "danger")
            return redirect(url_for("new_ethernet"))
        flash(f"Created '{form['name']}' — select it on the relevant interface's 'Set to' to use it.", "success")
        return redirect(url_for("index"))
    return render_template("new_ethernet.html")


@app.route("/confirm", methods=["POST"])
def confirm():
    recovery.cancel_pending()
    return redirect(url_for("index"))


if __name__ == "__main__":
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
