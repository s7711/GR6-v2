"""Thin wrapper around NetworkManager's `nmcli` — this service's only job
is presenting/editing what NetworkManager already manages, not owning
any network state of its own (see network-prd.md's "Why NetworkManager,
not hand-rolled tools"). Every write goes through `nmcli`, run via
`sudo` (passwordless sudo for the `pi` user is already relied on
elsewhere on this Pi — see network-prd.md).

`-t` (terse) output is used for anything parsed programmatically —
human-readable `nmcli` output is never parsed.
"""

import subprocess

# Pseudo-devices nmcli reports that aren't real, configurable interfaces.
_IGNORED_TYPES = {"loopback", "wifi-p2p"}


class NmError(Exception):
    """An `nmcli` command failed — message is nmcli's own stderr."""


def _run(args: list[str], sudo: bool = False) -> str:
    cmd = (["sudo"] if sudo else []) + ["nmcli"] + args
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise NmError(result.stderr.strip() or f"nmcli exited {result.returncode}")
    return result.stdout


def _parse_terse(output: str) -> list[dict]:
    """`nmcli -t` separates fields with `:` and rows with newlines — good
    enough here since none of the fields we read contain a literal `:`
    (nmcli itself escapes `:` within a field as `\\:`, not handled here
    because nothing we query does that)."""
    rows = []
    for line in output.splitlines():
        if line:
            rows.append(line.split(":"))
    return rows


def list_interfaces() -> list[dict]:
    """{"device", "type", "state", "connection"} for every real
    interface (wifi/ethernet) — excludes loopback and wifi's own
    p2p-dev-* pseudo-devices, which aren't configurable interfaces."""
    out = _run(["-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"])
    interfaces = []
    for device, dev_type, state, connection in _parse_terse(out):
        if dev_type in _IGNORED_TYPES:
            continue
        interfaces.append({
            "device": device,
            "type": dev_type,
            "state": state,
            "connection": connection or None,
        })
    return interfaces


def connection_settings(name: str) -> dict:
    """Flat {field: value} for a connection profile, straight from
    `nmcli connection show <name>` — every field nmcli reports, not
    filtered down, since different callers care about different subsets
    (wifi vs ethernet, client vs hotspot)."""
    out = _run(["-t", "-f", "all", "connection", "show", name])
    settings = {}
    for line in out.splitlines():
        if ":" in line:
            key, _, value = line.partition(":")
            settings[key] = value
    return settings


def _summarize(settings: dict) -> dict:
    is_wifi = settings.get("connection.type") == "802-11-wireless"
    return {
        "mode": (
            "hotspot" if settings.get("802-11-wireless.mode") == "ap" else "client"
        ) if is_wifi else None,
        "ssid": settings.get("802-11-wireless.ssid") or None if is_wifi else None,
        "ipv4_method": settings.get("ipv4.method") or None,
        "ipv4_address": (settings.get("ipv4.addresses") or None),
        "ipv4_gateway": (settings.get("ipv4.gateway") or None),
    }


def interface_detail(device: str, connection: str | None) -> dict:
    """One interface's full picture for the page: its own device-level
    info plus (if connected) its connection profile's settings. Returns
    {"device", "type", "state", "mode", "ssid", "ipv4_method",
    "ipv4_address", "ipv4_gateway"} — mode/ssid are None for ethernet."""
    settings = connection_settings(connection) if connection else {}
    return _summarize(settings)


def describe_connection(name: str) -> str:
    """A short, human-readable summary of a connection profile — for the
    Recovery dropdown, where the bare NetworkManager connection name
    (e.g. "preconfigured") means nothing on its own (see
    network-prd.md's "Recovery" discussion — Ben couldn't tell what the
    options meant from the name alone)."""
    summary = _summarize(connection_settings(name))
    if summary["mode"] == "hotspot":
        return f"wifi hotspot — SSID {summary['ssid']}, broadcasting at {summary['ipv4_address'] or '?'}"
    if summary["mode"] == "client":
        return f"wifi client — SSID {summary['ssid']}, {_ipv4_desc(summary)}"
    if summary["ipv4_method"] == "shared":
        return f"ethernet — sharing another interface's internet, at {summary['ipv4_address'] or '?'}"
    return f"ethernet — {_ipv4_desc(summary)}"


def _ipv4_desc(summary: dict) -> str:
    return "DHCP" if summary["ipv4_method"] != "manual" else f"static {summary['ipv4_address'] or '?'}"


def apply_wifi_client(
    device: str, connection_name: str, ssid: str, password: str,
    ipv4_method: str = "auto", address: str | None = None, gateway: str | None = None,
) -> None:
    """Create (or replace) `connection_name`, bound to `device`, as a
    wifi client of `ssid`/`password`. `ipv4_method` "auto" (DHCP) or
    "manual" (static, needs address/gateway too)."""
    _run(["connection", "delete", connection_name], sudo=True) if _connection_exists(connection_name) else None
    args = [
        "connection", "add", "type", "wifi", "con-name", connection_name,
        "ifname", device, "ssid", ssid,
        "wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", password,
        "ipv4.method", ipv4_method,
    ]
    if ipv4_method == "manual":
        args += ["ipv4.addresses", address, "ipv4.gateway", gateway]
    _run(args, sudo=True)
    _run(["connection", "up", connection_name], sudo=True)


def apply_hotspot(
    device: str, connection_name: str, ssid: str, password: str, address: str = "192.168.4.1/24",
) -> None:
    """Create (or replace) `connection_name`, bound to `device`, as a
    wifi hotspot (access point + its own DHCP server for clients that
    join it) — NetworkManager's `ipv4.method: shared` runs a dnsmasq
    instance automatically, no separate hostapd/dnsmasq config needed."""
    _run(["connection", "delete", connection_name], sudo=True) if _connection_exists(connection_name) else None
    _run([
        "connection", "add", "type", "wifi", "con-name", connection_name,
        "ifname", device, "ssid", ssid, "802-11-wireless.mode", "ap",
        "wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", password,
        "ipv4.method", "shared", "ipv4.addresses", address,
    ], sudo=True)
    _run(["connection", "up", connection_name], sudo=True)


def apply_ethernet(
    device: str, connection_name: str, ipv4_method: str = "auto",
    address: str | None = None, gateway: str | None = None, share: bool = False,
) -> None:
    """Create (or replace) `connection_name`, bound to `device`, as a
    plain wired connection — DHCP, static, or `share=True` to NAT/route
    another interface's internet connection through this one (the
    xNAV650's NTRIP corrections use-case, see network-prd.md — this is
    the same NetworkManager `ipv4.method: shared` mechanism
    apply_hotspot() uses, just on a wired interface instead of wifi, and
    with no DHCP-vs-static choice for the same reason a hotspot has
    none: a shared interface always both owns `address` itself *and*
    runs its own DHCP server for whatever's plugged into it)."""
    _run(["connection", "delete", connection_name], sudo=True) if _connection_exists(connection_name) else None
    if share:
        method, extra = "shared", ["ipv4.addresses", address]
    elif ipv4_method == "manual":
        method, extra = "manual", ["ipv4.addresses", address, "ipv4.gateway", gateway]
    else:
        method, extra = "auto", []
    args = [
        "connection", "add", "type", "ethernet", "con-name", connection_name,
        "ifname", device, "ipv4.method", method,
    ] + extra
    _run(args, sudo=True)
    _run(["connection", "up", connection_name], sudo=True)


def activate_connection(connection_name: str) -> None:
    """Bring up an already-existing connection profile by name — this is
    what a recovery revert calls (re-applying a stored known-good
    profile), as opposed to *creating* one via the apply_* functions
    above."""
    _run(["connection", "up", connection_name], sudo=True)


def list_connection_names() -> list[str]:
    """Every connection profile NetworkManager knows about, excluding
    the loopback pseudo-connection — not a real interface a device can
    fall back to, but nmcli's connection list includes it alongside
    real ones (unlike `device status`, which is already filtered by
    _IGNORED_TYPES in list_interfaces)."""
    out = _run(["-t", "-f", "NAME,TYPE", "connection", "show"])
    names = []
    for line in out.splitlines():
        if not line:
            continue
        name, _, conn_type = line.rpartition(":")
        if conn_type not in _IGNORED_TYPES:
            names.append(name)
    return names


def _connection_exists(name: str) -> bool:
    return name in list_connection_names()
