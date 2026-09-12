"""Wifi signal-strength scanning/logging for whichever device (if any) is
set to Scanner mode — see network-prd.md and scanner_state.py.

Logs one line per scan, wide by BSSID (Ben's own design, 2026-09-08):
`{"t":..., "lat":..., "lon":..., "<bssid>": <signal_dbm>, ...}` — a flat
numeric field per access point means viewer's existing quantity-picker
(shared/logs.py's numericKeysOf()) already lists each AP as its own
selectable series with signal strength as the y-axis, no viewer changes
needed. SSID is deliberately not logged — the whole point of
`scan_ssid_filter` is to only keep the APs worth keeping, so by the time
a line is written its SSID is already known/filtered, not worth storing
again per line. Each BSSID field name is itself translated through
config.yaml's network.bssid_names where possible (added 2026-09-11) - an
unlisted BSSID still logs/displays as its raw address.

Also logs `connected_bssid` (added 2026-09-10) — whichever BSSID this
Pi's *actual* wifi client connection is associated to right now
(nm.connected_wifi_bssid()), translated to its configured friendly name
(config.yaml's network.bssid_names) same as the home page's live
display, or left as the raw BSSID if it isn't in that list. Deliberately
NOT the scanned device's own link state - the scanned device is usually
the one dedicated to Scanner mode, which is left unmanaged/disconnected
on purpose (see scanner_state.py), so it's never itself "connected" to
anything (found live 2026-09-10: the first version asked the scanned
device, so this field silently never appeared at all). A string, not a
number, on purpose - the whole point is a human-readable name in
viewer's legend, not a chart value; see
viewer/templates/pages/home.html's categoricalKeysOf/categoryLevelsOf
for how a string field still ends up plottable.

Uses `iw dev <device> scan` directly (not nmcli) — confirmed live
2026-09-08 that this works even while the device is actively connected
(tested against a device running a hotspot), taking ~3.5-4s for a full
sweep; that scan duration is the natural pacing for this loop, not a
fixed sleep.
"""

import logging
import math
import queue
import subprocess
import threading
import time

import nm

_SCAN_TIMEOUT_S = 15  # generous — a real scan took ~3.7s in testing, this is just a safety net
_IDLE_POLL_S = 2.0  # how often to check whether a device has been set to Scanner mode


# 2.4GHz tops out around 2495MHz, 5GHz starts at 5150MHz — anywhere
# between is a clean split. Found live 2026-09-08: a real mesh AP
# broadcasts both bands on separate BSSIDs, so an unfiltered scan
# reported "10+ BSSIDs" for what's really 2-3 physical units — 2.4GHz
# only, both because that halves the noise and because (per Ben) it's
# the band that actually reaches further outdoors, i.e. the one whose
# coverage map is actually useful here.
_MAX_2GHZ_FREQ_MHZ = 3000


def _parse_scan(output: str, ssid_filter: list[str]) -> dict[str, int]:
    """{bssid: signal_dbm} for every 2.4GHz AP in `iw dev <device>
    scan`'s output whose SSID is in ssid_filter (or every AP, if
    ssid_filter is empty)."""
    results = {}
    bssid = ssid = None
    signal = freq = None

    def flush():
        if (
            bssid is not None and signal is not None and freq is not None
            and freq < _MAX_2GHZ_FREQ_MHZ and (not ssid_filter or ssid in ssid_filter)
        ):
            results[bssid] = signal

    for line in output.splitlines():
        line = line.strip()
        if line.startswith("BSS "):
            flush()
            # e.g. "BSS b4:fb:e4:c7:f1:96(on wlan0)" or "...(on wlan0) -- associated"
            bssid = line.split()[1].split("(")[0].lower()
            ssid = None
            signal = freq = None
        elif line.startswith("signal:"):
            signal = round(float(line.split(":", 1)[1].strip().split()[0]))
        elif line.startswith("SSID:"):
            ssid = line.split(":", 1)[1].strip()
        elif line.startswith("freq:"):
            freq = float(line.split(":", 1)[1].strip())
    flush()
    return results


def scan_device(device: str, ssid_filter: list[str]) -> dict[str, int]:
    result = subprocess.run(
        ["sudo", "iw", "dev", device, "scan"],
        capture_output=True, text=True, timeout=_SCAN_TIMEOUT_S,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f"iw scan exited {result.returncode}")
    return _parse_scan(result.stdout, ssid_filter)


def run_scan_loop(scanner_state, nav_client, ssid_filter: list[str], log, bssid_names: dict[str, str]) -> None:
    """Runs forever — scans whichever device scanner_state currently
    names, logs one row per scan, and just waits (polling scanner_state
    at _IDLE_POLL_S) whenever nothing's set. `log` is a
    shared.rotating_jsonl_log.RotatingJsonlLog, already start()ed.

    The actual `log.append()` disk write happens on a separate thread
    (see _scan_log_writer_loop) - not because a scan loop is timing-
    critical the way a control/receive loop is, but per Ben's 2026-09-09
    call: once one thread's inline logging was found to be able to stall
    a control loop (see oxts-nav's ncomrx_thread.py/navigate's
    _control_loop), the rule became "all logging is a separate thread",
    everywhere, rather than judging each spot as safe or not on its own."""
    record_queue: "queue.Queue" = queue.Queue()
    threading.Thread(target=_scan_log_writer_loop, args=(record_queue, log), daemon=True).start()

    while True:
        device = scanner_state.get_device()
        if device is None:
            time.sleep(_IDLE_POLL_S)
            continue
        if not _scan_tick(device, ssid_filter, nav_client, bssid_names, record_queue):
            time.sleep(_IDLE_POLL_S)
        # Otherwise no sleep — the scan call itself (~3.5-4s observed) is
        # the loop's own pacing, same reasoning as wheelspeed's
        # event-driven update loop not needing a fixed rate of its own.


def _scan_tick(device: str, ssid_filter: list[str], nav_client, bssid_names: dict[str, str], record_queue) -> bool:
    """One scan-and-log pass, factored out of run_scan_loop so it can be
    tested directly without threads/an infinite loop - same reasoning as
    gnss_mode.py's _watchdog_tick. Returns False on a failed scan (the
    loop's cue to back off with _IDLE_POLL_S instead of retrying
    immediately), True otherwise."""
    try:
        readings = scan_device(device, ssid_filter)
    except (subprocess.TimeoutExpired, RuntimeError) as e:
        logging.warning("[network] Wifi scan on %s failed: %s", device, e)
        return False

    nav = nav_client.latest().get("nav", {})
    # Per-AP field names translated too (added 2026-09-11, Ben's ask) -
    # same bssid_names lookup as connected_bssid below. If two different
    # BSSIDs were ever given the same friendly name, one would overwrite
    # the other here - a config mistake to fix in bssid_names, not
    # something this loop tries to detect.
    record = {bssid_names.get(bssid, bssid): signal for bssid, signal in readings.items()}
    if "Lat" in nav and "Lon" in nav:
        record["lat"] = math.degrees(nav["Lat"])
        record["lon"] = math.degrees(nav["Lon"])
    connected_bssid = nm.connected_wifi_bssid()
    if connected_bssid:
        record["connected_bssid"] = bssid_names.get(connected_bssid, connected_bssid)
    if record:
        record_queue.put(record)
    return True


def _scan_log_writer_loop(record_queue: "queue.Queue", log) -> None:
    while True:
        log.append(record_queue.get())
