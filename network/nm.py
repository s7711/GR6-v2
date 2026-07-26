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


def interface_detail(device: str, connection: str | None) -> dict:
    """One interface's full picture for the page: its own device-level
    info plus (if connected) its connection profile's settings. Returns
    {"device", "type", "state", "mode", "ssid", "ipv4_method",
    "ipv4_address", "ipv4_gateway"} — mode/ssid are None for ethernet."""
    settings = connection_settings(connection) if connection else {}
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
    address: str | None = None, gateway: str | None = None,
) -> None:
    """Create (or replace) `connection_name`, bound to `device`, as a
    plain wired connection — DHCP or static, no SSID/password concept."""
    _run(["connection", "delete", connection_name], sudo=True) if _connection_exists(connection_name) else None
    args = [
        "connection", "add", "type", "ethernet", "con-name", connection_name,
        "ifname", device, "ipv4.method", ipv4_method,
    ]
    if ipv4_method == "manual":
        args += ["ipv4.addresses", address, "ipv4.gateway", gateway]
    _run(args, sudo=True)
    _run(["connection", "up", connection_name], sudo=True)


def activate_connection(connection_name: str) -> None:
    """Bring up an already-existing connection profile by name — this is
    what a recovery revert calls (re-applying a stored known-good
    profile), as opposed to *creating* one via the apply_* functions
    above."""
    _run(["connection", "up", connection_name], sudo=True)


def list_connection_names() -> list[str]:
    out = _run(["-t", "-f", "NAME", "connection", "show"])
    return [line for line in out.splitlines() if line]


def _connection_exists(name: str) -> bool:
    return name in list_connection_names()
