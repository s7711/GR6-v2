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

# nmcli's connection.type strings, translated to list_interfaces()'s
# device-type vocabulary ("wifi"/"ethernet") so a profile can be matched
# against the device it's offered for (see list_connections()).
_CONN_TYPE_TO_DEVICE_TYPE = {"802-11-wireless": "wifi", "802-3-ethernet": "ethernet"}


class NmError(Exception):
    """An `nmcli` command failed — message is nmcli's own stderr."""


# A wifi activation can legitimately take a long time to fail (measured
# live 2026-07-30: 58s for NetworkManager to declare "association took
# too long" on a real device) — this timeout is a safety net against a
# genuinely stuck nmcli call hanging forever (found live the same day,
# needed a physical unplug/replug to clear), set well above that
# observed worst case rather than tightly around it.
_NMCLI_TIMEOUT_S = 90


def _run(args: list[str], sudo: bool = False) -> str:
    cmd = (["sudo"] if sudo else []) + ["nmcli"] + args
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=_NMCLI_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        raise NmError(f"nmcli timed out after {_NMCLI_TIMEOUT_S}s — it may still be working in the background")
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
    "Set to" dropdown, where the bare NetworkManager connection name
    (e.g. "preconfigured") means nothing on its own (Ben couldn't tell
    what the options meant from the name alone)."""
    summary = _summarize(connection_settings(name))
    if summary["mode"] == "hotspot":
        return f"wifi hotspot — SSID {summary['ssid']}, broadcasting at {summary['ipv4_address'] or '?'}"
    if summary["mode"] == "client":
        return f"wifi client — SSID {summary['ssid']}, {_ipv4_desc(summary)}"
    if summary["ipv4_method"] == "shared":
        return f"ethernet — sharing another interface's internet, at {summary['ipv4_address'] or '?'}"
    return f"ethernet — {_ipv4_desc(summary)}"


def is_hotspot(name: str) -> bool:
    """Whether a wifi profile is a hotspot (AP mode) rather than a
    client — see network-prd.md's priority scheme: a hotspot profile
    always gets a low autoconnect-priority, never the same treatment as
    a client profile."""
    return _summarize(connection_settings(name)).get("mode") == "hotspot"


def _ipv4_desc(summary: dict) -> str:
    return "DHCP" if summary["ipv4_method"] != "manual" else f"static {summary['ipv4_address'] or '?'}"


def _require_new_name(name: str) -> None:
    if _connection_exists(name):
        raise NmError(f"A connection named '{name}' already exists")


def create_wifi_client(
    name: str, ssid: str, password: str,
    ipv4_method: str = "auto", address: str | None = None, gateway: str | None = None,
) -> None:
    """Create a new, device-unbound wifi client profile (no `ifname` —
    see network-prd.md's "Connection profiles are shared, not
    per-device": profiles here are deliberately activatable on any
    matching device, never pinned). Does not activate it — the
    operator does that separately via the "Set to" dropdown."""
    _require_new_name(name)
    args = [
        "connection", "add", "type", "wifi", "con-name", name,
        "ssid", ssid,
        "wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", password,
        "ipv4.method", ipv4_method,
    ]
    if ipv4_method == "manual":
        args += ["ipv4.addresses", address, "ipv4.gateway", gateway]
    _run(args, sudo=True)


def create_hotspot(name: str, ssid: str, password: str, address: str = "192.168.4.1/24") -> None:
    """Create a new, device-unbound wifi hotspot profile (access point +
    its own DHCP server for clients that join it) — NetworkManager's
    `ipv4.method: shared` runs a dnsmasq instance automatically, no
    separate hostapd/dnsmasq config needed. Not activated on creation."""
    _require_new_name(name)
    _run([
        "connection", "add", "type", "wifi", "con-name", name,
        "ssid", ssid, "802-11-wireless.mode", "ap",
        "wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", password,
        "ipv4.method", "shared", "ipv4.addresses", address,
    ], sudo=True)


def create_ethernet(
    name: str, ipv4_method: str = "auto",
    address: str | None = None, gateway: str | None = None, share: bool = False,
    speed_mbps: int | None = None,
) -> None:
    """Create a new, device-unbound wired profile — DHCP, static, or
    `share=True` to NAT/route another interface's internet connection
    through this one (the xNAV650's NTRIP corrections use-case, see
    network-prd.md — the same `ipv4.method: shared` mechanism
    create_hotspot() uses, just on a wired interface, and with no
    DHCP-vs-static choice for the same reason a hotspot has none: a
    shared interface always both owns `address` itself *and* runs its
    own DHCP server for whatever's plugged into it). Not activated on
    creation.

    `speed_mbps` forces the link speed (e.g. 100) instead of
    auto-negotiating — see network-prd.md's "Force eth0 link speed to
    100M": NetworkManager requires `auto-negotiate` off and an explicit
    `duplex` whenever a fixed `speed` is set, can't force speed alone."""
    _require_new_name(name)
    if share:
        method, extra = "shared", ["ipv4.addresses", address]
    elif ipv4_method == "manual":
        method, extra = "manual", ["ipv4.addresses", address, "ipv4.gateway", gateway]
    else:
        method, extra = "auto", []
    args = [
        "connection", "add", "type", "ethernet", "con-name", name, "ipv4.method", method,
    ] + extra
    if speed_mbps:
        args += [
            "802-3-ethernet.auto-negotiate", "no",
            "802-3-ethernet.speed", str(speed_mbps),
            "802-3-ethernet.duplex", "full",
        ]
    _run(args, sudo=True)


def activate_connection(connection_name: str, device: str | None = None) -> None:
    """Bring up an already-existing connection profile by name — this is
    what the "Set to" control calls. `device` pins it to a specific
    interface — needed whenever more than one device could take the
    connection (e.g. two wifi devices), otherwise nmcli picks one
    itself."""
    args = ["connection", "up", connection_name]
    if device:
        args += ["ifname", device]
    _run(args, sudo=True)


def disconnect_device(device: str) -> None:
    """Take a device down with nothing active — a normal NetworkManager
    state (`nmcli device disconnect`), not a synthetic one. This is
    what the "Set to: None (disconnected)" option calls (ethernet
    only — wifi uses `set_managed(device, False)` instead, since
    disconnect alone doesn't stop NetworkManager's autoconnect from
    immediately reactivating something else)."""
    _run(["device", "disconnect", device], sudo=True)


def set_autoconnect_priority(connection_name: str, priority: int) -> None:
    """Set a profile's `connection.autoconnect-priority` — see
    network-prd.md's "Wifi profile choice becomes priority-based, not
    exclusive-activate": the chosen profile gets a high priority,
    other client profiles for that device get reset to 0, and any
    hotspot profile gets a distinctly low priority so it's only ever
    NetworkManager's last resort, never a peer choice."""
    _run(["connection", "modify", connection_name, "connection.autoconnect-priority", str(priority)], sudo=True)


def set_managed(device: str, managed: bool) -> None:
    """Take a device fully out of (or back into) NetworkManager's
    control — see network-prd.md's "Turn off this interface": unlike
    `disconnect_device`, a `managed no` device is not eligible for
    NetworkManager's autoconnect at all, so it actually stays off."""
    _run(["device", "set", device, "managed", "yes" if managed else "no"], sudo=True)


def link_stats(device: str, device_type: str) -> dict:
    """Live link-quality figures for one interface — signal strength
    (wifi only) and a lost-packets count, both None if unavailable
    (e.g. device not connected). See network-prd.md's "Live
    link-quality display": wifi's `tx failed` (from `iw ... station
    dump`) is genuine lost packets, not `tx retries` (a retry that
    eventually succeeds isn't lost) — this is the same figure that
    surfaced the AP-not-hearing-the-Pi asymmetry investigated
    2026-07-30. Ethernet has no signal strength; its lost-packets
    figure is the sum of `ip -s link`'s RX/TX errors and drops."""
    if device_type == "wifi":
        return _wifi_link_stats(device)
    return {"signal_dbm": None, "lost_packets": _ethernet_lost_packets(device)}


# `iw`/`ip` are local, normally-instant queries (unlike nmcli, which
# can genuinely wait on a real wifi association) — a short timeout here
# is just a safety net against a wedged command blocking /ws/status's
# snapshot loop, not something expected to legitimately need it.
_LOCAL_CMD_TIMEOUT_S = 5


def _wifi_link_stats(device: str) -> dict:
    signal_dbm = None
    link_out = subprocess.run(
        ["iw", "dev", device, "link"], capture_output=True, text=True, timeout=_LOCAL_CMD_TIMEOUT_S
    ).stdout
    for line in link_out.splitlines():
        line = line.strip()
        if line.startswith("signal:"):
            # e.g. "signal: -58 dBm"
            signal_dbm = int(line.split(":", 1)[1].strip().split()[0])

    tx_failed = None
    station_out = subprocess.run(
        ["iw", "dev", device, "station", "dump"], capture_output=True, text=True, timeout=_LOCAL_CMD_TIMEOUT_S
    ).stdout
    for line in station_out.splitlines():
        line = line.strip()
        if line.startswith("tx failed:"):
            tx_failed = int(line.split(":", 1)[1].strip())

    return {"signal_dbm": signal_dbm, "lost_packets": tx_failed}


def _ethernet_lost_packets(device: str) -> int | None:
    out = subprocess.run(
        ["ip", "-s", "link", "show", device], capture_output=True, text=True, timeout=_LOCAL_CMD_TIMEOUT_S
    ).stdout
    lines = out.splitlines()
    total = 0
    found = False
    for i, line in enumerate(lines):
        if (line.strip().startswith("RX:") or line.strip().startswith("TX:")) and i + 1 < len(lines):
            found = True
            values = lines[i + 1].split()
            # ip -s link's fixed column order: RX bytes packets errors dropped missed mcast
            #                                   TX bytes packets errors dropped carrier collsns
            if len(values) >= 4:
                total += int(values[2]) + int(values[3])  # errors + dropped
    return total if found else None


def active_connections() -> dict[str, str]:
    """{connection_name: device} for every currently-active connection.
    Not used for the same-profile-two-devices conflict check any more
    (see connections_in_use()) — found live 2026-07-30 that this
    misses a profile still *activating* (not yet fully active) on
    another device, a real race window given how long a wifi
    activation can legitimately take to resolve."""
    out = _run(["-t", "-f", "NAME,DEVICE", "connection", "show", "--active"])
    return {name: device for name, device in _parse_terse(out) if device}


def connections_in_use() -> dict[str, str]:
    """{connection_name: device} for every device currently *using* a
    connection — including one still activating, not yet fully active.
    This is `nmcli device status`'s CONNECTION column (same source as
    list_interfaces()'s "connection" field, already known to populate
    as soon as activation starts), deliberately broader than
    active_connections()'s `--active` filter: a same-profile-on-two-
    devices conflict check needs to catch a profile mid-activation too,
    or a second device can start activating it before the first has
    finished, evicting one from the other (the exact bug found live
    2026-07-30 — see network-prd.md)."""
    out = _run(["-t", "-f", "DEVICE,CONNECTION", "device", "status"])
    return {connection: device for device, connection in _parse_terse(out) if connection}


def list_connections() -> list[dict]:
    """Every connection profile NetworkManager knows about, as
    {"name", "device_type"} — device_type is "wifi"/"ethernet" (matching
    list_interfaces()'s vocabulary), so callers can offer/activate a
    profile only on a device of the same type (a wifi profile can't
    bind to an ethernet device, and vice versa — nmcli itself refuses
    with "no suitable device found" if you try). Excludes loopback,
    which nmcli's connection list includes alongside real profiles
    (unlike `device status`, already filtered in list_interfaces)."""
    out = _run(["-t", "-f", "NAME,TYPE", "connection", "show"])
    connections = []
    for line in out.splitlines():
        if not line:
            continue
        name, _, conn_type = line.rpartition(":")
        if conn_type in _IGNORED_TYPES:
            continue
        connections.append({"name": name, "device_type": _CONN_TYPE_TO_DEVICE_TYPE.get(conn_type, conn_type)})
    return connections


def list_connection_names() -> list[str]:
    """Every connection profile's name, regardless of type — see
    list_connections() when the type is also needed."""
    return [c["name"] for c in list_connections()]


def _connection_exists(name: str) -> bool:
    return name in list_connection_names()
