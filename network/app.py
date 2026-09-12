"""GR6-v2 network: a thin web UI over NetworkManager (`nmcli`) for
amundsen's network interfaces — DHCP/static, wifi client/hotspot.
See network-prd.md for the design/reasoning, especially the "v2 → v3"
section: there is deliberately no per-interface recovery/confirm timer
any more (removed 2026-07-30) — wifi profile choice is priority-based
instead, leaning on NetworkManager's own retry/fallback behaviour
rather than fighting it with a bespoke revert.

This service owns no network state itself — `nm.py` reads/writes
NetworkManager directly.
"""

import json
import logging
import sys
import threading
import time
from pathlib import Path

from flask import Flask, flash, redirect, render_template, request, url_for
from flask_sock import Sock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.config import load_config  # noqa: E402
from shared.feed_client import FeedClient  # noqa: E402
from shared.rotating_jsonl_log import RotatingJsonlLog  # noqa: E402
from shared.web import manager_url, use_shared_static, use_shared_templates  # noqa: E402

import nm  # noqa: E402
import scanner  # noqa: E402
from scanner_state import ScannerState  # noqa: E402

cfg = load_config()
service_cfg = cfg["services"]["network"]
oxtsnav_cfg = cfg["services"]["oxts-nav"]

BSSID_NAMES = service_cfg.get("bssid_names", {})  # {bssid: friendly name} - an unlisted BSSID just shows/logs as itself

DATA_DIR = Path(__file__).resolve().parent / "data"

app = Flask(__name__)
app.secret_key = "gr6-network"  # only used for flash() error messages — trusted LAN, no auth, see manager-prd.md
use_shared_templates(app)
use_shared_static(app)
sock = Sock(app)

STATUS_HZ = 1

# Form values for the "Set to" dropdown's non-profile options.
NONE_VALUE = "__none__"   # ethernet: nmcli device disconnect
OFF_VALUE = "__off__"     # wifi: nmcli device set managed no — see network-prd.md's "Turn off this interface"
# Same underlying nmcli action as OFF_VALUE (managed no) — scanner_state.py
# is the only thing distinguishing "just off" from "deliberately the wifi
# signal-mapping scanner" — see its own docstring and scanner.py.
SCANNER_VALUE = "__scanner__"

scanner_state = ScannerState(DATA_DIR / "scanner.json")
nav_client = FeedClient(oxtsnav_cfg["nav_feed_socket"], default={"nav": {}, "status": {}, "connection": {}})
scan_log = RotatingJsonlLog(DATA_DIR / "logs", service_cfg["scan_log_rotate_s"], service_cfg["scan_log_retention_days"])

# Autoconnect-priority scheme — see network-prd.md's "Wifi profile
# choice becomes priority-based, not exclusive-activate".
PRIORITY_SELECTED = 10
PRIORITY_NORMAL = 0
PRIORITY_HOTSPOT = -10


@app.context_processor
def inject_manager_url():
    return {"manager_url": manager_url(request.host.split(":")[0])}


def _interfaces_with_detail() -> list[dict]:
    interfaces = nm.list_interfaces()
    all_connections = nm.list_connections()
    connection_descriptions = {c["name"]: nm.describe_connection(c["name"]) for c in all_connections}
    active = nm.connections_in_use()
    for iface in interfaces:
        iface.update(nm.interface_detail(iface["device"], iface["connection"]))
        iface["matching_connections"] = [
            c["name"] for c in all_connections if c["device_type"] == iface["type"]
        ]
        # Flag profiles currently active on a *different* device so the
        # page can show it before the operator even tries to Apply —
        # the actual block still happens server-side in apply().
        iface["connections_active_elsewhere"] = {
            name: dev for name, dev in active.items()
            if dev != iface["device"] and name in iface["matching_connections"]
        }
        iface["link_stats"] = _labelled_link_stats(iface["device"], iface["type"])
    return interfaces, connection_descriptions


def _labelled_link_stats(device: str, device_type: str) -> dict:
    """nm.link_stats(), with a wifi BSSID translated to its configured
    friendly name (BSSID_NAMES) where there is one — see config.yaml's
    network.bssid_names comment."""
    stats = nm.link_stats(device, device_type)
    if stats.get("bssid"):
        stats["bssid"] = BSSID_NAMES.get(stats["bssid"], stats["bssid"])
    return stats


@app.route("/")
def index():
    interfaces, connection_descriptions = _interfaces_with_detail()
    return render_template(
        "index.html",
        interfaces=interfaces,
        connection_descriptions=connection_descriptions,
        none_value=NONE_VALUE,
        off_value=OFF_VALUE,
        scanner_value=SCANNER_VALUE,
        scanner_device=scanner_state.get_device(),
    )


def _collect_desired(interfaces: list[dict]) -> dict[str, str]:
    """{device: chosen "Set to" value} for every interface with a
    submitted (non-empty) choice — every card's <select> always
    submits something, so in practice this is every device."""
    return {
        iface["device"]: request.form.get(f"set_{iface['device']}")
        for iface in interfaces
        if request.form.get(f"set_{iface['device']}")
    }


def _validate_batch(desired: dict[str, str]) -> list[str]:
    """Whole-batch conflict check — see network-prd.md's "atomic wifi
    change" (2026-07-30): swapping two devices' profiles (each already
    active on the *other* one) is a legitimate, intentional change, not
    a conflict — a profile already in use elsewhere is only a real
    problem if that other device *isn't also* moving away from it in
    this same batch. Checking one device at a time against
    NetworkManager's live state (the previous design) made every
    swap look like a conflict, since of course the target was "in use"
    — by the very device it was about to be freed from."""
    errors = []
    real_choices = {d: c for d, c in desired.items() if c not in (NONE_VALUE, OFF_VALUE, SCANNER_VALUE)}

    seen = {}
    for device, chosen in real_choices.items():
        if chosen in seen:
            errors.append(f"'{chosen}' selected for both {seen[chosen]} and {device} in the same Apply — pick one.")
        else:
            seen[chosen] = device

    in_use = nm.connections_in_use()
    for device, chosen in real_choices.items():
        holder = in_use.get(chosen)
        if not holder or holder == device:
            continue
        holders_new_choice = desired.get(holder)
        relinquishing = holders_new_choice is not None and holders_new_choice != chosen
        if not relinquishing:
            errors.append(
                f"{device}: '{chosen}' is in use on {holder} — "
                f"change {holder} away from it in the same Apply to swap them."
            )
    return errors


def _apply_batch(iface_by_device: dict[str, dict], desired: dict[str, str]) -> None:
    """Actually apply an already-validated batch. Priorities are set in
    one pass across every wifi profile touched by *any* device in this
    batch first — not per-device — so two devices each choosing a
    different profile in the same Apply (a swap) don't clobber each
    other's "this is the selected one" priority: whatever's chosen by
    any device this time becomes that profile's priority, regardless of
    whether it's normally a hotspot; only a profile nobody chose this
    time keeps the hotspot-is-lowest-priority default."""
    wifi_profiles = set()
    for iface in iface_by_device.values():
        if iface["type"] == "wifi":
            wifi_profiles.update(iface["matching_connections"])
    chosen_profiles = {c for c in desired.values() if c not in (NONE_VALUE, OFF_VALUE, SCANNER_VALUE)}
    for name in wifi_profiles:
        if name in chosen_profiles:
            priority = PRIORITY_SELECTED
        elif nm.is_hotspot(name):
            priority = PRIORITY_HOTSPOT
        else:
            priority = PRIORITY_NORMAL
        nm.set_autoconnect_priority(name, priority)

    current_scanner = scanner_state.get_device()
    for device, chosen in desired.items():
        iface = iface_by_device[device]

        # Whatever this device is becoming, it isn't the scanner any more
        # unless it's explicitly chosen again this batch.
        if current_scanner == device and chosen != SCANNER_VALUE:
            scanner_state.clear()
            current_scanner = None

        if iface["type"] == "wifi" and chosen in (OFF_VALUE, SCANNER_VALUE):
            if iface["state"] != "unmanaged":
                nm.set_managed(device, False)
            if chosen == SCANNER_VALUE:
                scanner_state.set_device(device)
            continue

        if chosen == NONE_VALUE:
            if iface["connection"]:
                nm.disconnect_device(device)
            continue

        if chosen == iface["connection"] and iface["state"] != "unmanaged":
            continue  # already set to this — nothing to do

        if iface["state"] == "unmanaged":
            nm.set_managed(device, True)

        nm.activate_connection(chosen, device=device)


@app.route("/apply", methods=["POST"])
def apply():
    """One combined Apply for every interface at once, applied
    atomically — see network-prd.md's "v2 → v3" and "atomic wifi
    change": either the whole batch is valid and all of it happens, or
    none of it does (a single clear error, nothing partially applied)."""
    interfaces, _ = _interfaces_with_detail()
    iface_by_device = {i["device"]: i for i in interfaces}
    desired = _collect_desired(interfaces)

    errors = _validate_batch(desired)
    if errors:
        for msg in errors:
            flash(msg, "danger")
        return redirect(url_for("index"))

    try:
        _apply_batch(iface_by_device, desired)
    except nm.NmError as e:
        flash(str(e), "danger")
        return redirect(url_for("index"))

    flash("Applied.", "success")
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
        speed = form.get("speed") or ""
        try:
            nm.create_ethernet(
                form["name"],
                ipv4_method="manual" if form.get("ipv4_mode") == "static" else "auto",
                address=form.get("address") or None, gateway=form.get("gateway") or None,
                share=form.get("share") == "on",
                speed_mbps=int(speed) if speed.isdigit() else None,
            )
        except nm.NmError as e:
            flash(str(e), "danger")
            return redirect(url_for("new_ethernet"))
        flash(f"Created '{form['name']}' — select it on the relevant interface's 'Set to' to use it.", "success")
        return redirect(url_for("index"))
    return render_template("new_ethernet.html")


@sock.route("/ws/status")
def ws_status(ws):
    """Live per-interface signal strength / lost-packets, at STATUS_HZ
    — see network-prd.md's "Live link-quality display"."""
    period = 1.0 / STATUS_HZ
    while True:
        snapshot = {iface["device"]: _labelled_link_stats(iface["device"], iface["type"]) for iface in nm.list_interfaces()}
        ws.send(json.dumps(snapshot))
        time.sleep(period)


def _restore_scanner_state():
    """Re-applies `managed no` for whichever device scanner_state.json
    says is in Scanner mode, if any — found live 2026-09-10: `nmcli
    device set <dev> managed no` is a NetworkManager runtime setting,
    not something it persists across a reboot (unlike scanner_state.json
    itself, a plain file). So after a Pi power-cycle, NetworkManager
    forgets and quietly re-manages/reconnects the device on its own
    (autoconnect priority, same as any other device) before this service
    even starts - the device just looks like an ordinary connected
    interface again, Scanner mode silently lost, with nothing in the log
    to say so. A `network` service *restart* alone never hits this
    (nmcli's own runtime state survives that fine) - only an actual
    reboot does."""
    device = scanner_state.get_device()
    if device is None:
        return
    try:
        nm.set_managed(device, False)
    except nm.NmError as e:
        logging.warning("[network] Couldn't restore Scanner mode on %s after startup: %s", device, e)


if __name__ == "__main__":
    _restore_scanner_state()
    nav_client.start()
    scan_log.start()
    threading.Thread(
        target=scanner.run_scan_loop,
        args=(scanner_state, nav_client, service_cfg["scan_ssid_filter"], scan_log, BSSID_NAMES),
        daemon=True,
    ).start()
    app.run(host=service_cfg["host"], port=service_cfg["port"], threaded=True)
