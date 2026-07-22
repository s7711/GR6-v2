"""Host-level system health (this Raspberry Pi itself, not any GR6-v2
service): brown-out/under-voltage, wifi signal, CPU load. Read once by the
manager (see manager-prd.md's header status badges) rather than by every
service separately — there's only one Pi, so one reader is enough, and
every service's shared header just watches the manager's `/ws/system`.
"""

import subprocess
import time
from pathlib import Path

_WIRELESS_PROC = Path("/proc/net/wireless")

_prev_cpu_total = None
_prev_cpu_idle = None

# vcgencmd's own "has happened since boot" bit is sticky forever once set —
# not useful for a status badge on its own, since it would stay "amber"
# indefinitely after a single brief dip. We track our own recency instead
# (age_seconds since the last observed event) and let the header decide
# red/amber/green from that age.
_last_brownout_monotonic = None
_prev_since_boot = False


def read_brownout() -> dict:
    """{"active": ..., "age_seconds": ...} — age_seconds is how long ago
    the last under-voltage event was observed (0 if active right now),
    or None if none has been seen since this process started. The
    since_boot bit is used only to catch a dip shorter than the poll
    interval (already cleared again by the time we poll) — detected as
    a false->true *transition*, not repeatedly re-triggered while it
    stays set."""
    global _last_brownout_monotonic, _prev_since_boot
    try:
        out = subprocess.run(
            ["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2
        ).stdout.strip()
        value = int(out.split("=")[1], 16)
        active = bool(value & 0x1)
        since_boot = bool(value & 0x10000)
    except (FileNotFoundError, IndexError, ValueError, subprocess.SubprocessError):
        active, since_boot = False, False

    now = time.monotonic()
    if active or (since_boot and not _prev_since_boot):
        _last_brownout_monotonic = now
    _prev_since_boot = since_boot

    age = None if _last_brownout_monotonic is None else now - _last_brownout_monotonic
    return {"active": active, "age_seconds": age}


def read_wifi_bars() -> int | None:
    """Signal strength as 1-5 bars (not dBm), from /proc/net/wireless's
    link-quality column. None if there's no wireless interface up (e.g.
    wired connection, or the wifi is down)."""
    try:
        lines = _WIRELESS_PROC.read_text().splitlines()[2:]
    except FileNotFoundError:
        return None
    for line in lines:
        parts = line.split()
        if len(parts) < 3:
            continue
        quality = float(parts[2].rstrip("."))
        bars = round(quality / 70 * 5)
        return max(1, min(5, bars))
    return None


def read_cpu_percent() -> float | None:
    """CPU load %, averaged since the previous call. None on the very
    first call (no prior sample yet to diff against) — the manager's
    poll loop calls this every few seconds, so that's the averaging
    window, not a fixed instant."""
    global _prev_cpu_total, _prev_cpu_idle
    try:
        with open("/proc/stat") as f:
            fields = [int(x) for x in f.readline().split()[1:]]
    except (FileNotFoundError, ValueError):
        return None
    idle = fields[3]
    total = sum(fields)
    percent = None
    if _prev_cpu_total is not None and total > _prev_cpu_total:
        d_total = total - _prev_cpu_total
        d_idle = idle - _prev_cpu_idle
        percent = round((1 - d_idle / d_total) * 100, 1)
    _prev_cpu_total, _prev_cpu_idle = total, idle
    return percent


def snapshot() -> dict:
    return {
        "brownout": read_brownout(),
        "wifi_bars": read_wifi_bars(),
        "cpu_percent": read_cpu_percent(),
    }
