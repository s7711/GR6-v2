"""Persistent oxts-nav logging — added 2026-09-08. Until now oxts-nav kept
no history of its own at all; every GNSS-loss investigation only had data
because navigate's per-run debug log happened to carry a few nav fields,
and only while a path/turn was actually running (see project memory's
2026-09-07 session).

Two logs, both rotated hourly and swept for retention at startup (same
pattern as missions/waterbutt/etc's own logs - see their _sweep_old_*
functions):

- Raw NCOM/UCOM bytes, straight off the wire, for full-fidelity replay
  later if a decoded field turns out to be the wrong one to have kept.
  Written by ncomrx_thread.py/ucomrx_thread.py's own run() loops (they're
  the only place the raw bytes exist) into whichever file handle is
  currently sitting in nrxs.nrx[xnav_ip]['logfile'] - this module just
  manages opening/rotating/closing that handle. Not viewer-visible (not
  jsonl) - `oxts-nav/data/raw-logs/`, separate from the jsonl directory
  the viewer scans.
- A decoded-fields jsonl, viewer-visible (`oxts-nav/data/logs/`, matching
  every other service's convention), sampled from nav_feed.snapshot() -
  reusing its existing staleness blanking rather than re-deriving it, so
  a lost NCOM feed shows as gaps here exactly like it does everywhere
  else that snapshot() feeds. Which fields is config-driven
  (oxts-nav.decoded_log_fields) rather than "everything" - see Ben's
  request 2026-09-08 for why: raw NCOM already covers full fidelity if
  ever needed, this is for the fields actually worth glancing at.
"""

import math
import sys
import threading
import time
from pathlib import Path

import nav_feed

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.rotating_jsonl_log import RotatingJsonlLog  # noqa: E402

RAW_LOG_SUFFIX = {"ncom": ".ncom", "ucom": ".ucom"}


class DataLogger:
    def __init__(
        self, nrxs, xnav_ip, protocol, data_dir: Path, decoded_fields,
        rotate_s, retention_days, decoded_hz, stale_after_s,
    ):
        self.nrxs = nrxs
        self.xnav_ip = xnav_ip
        self.raw_suffix = RAW_LOG_SUFFIX[protocol]
        self.raw_dir = data_dir / "raw-logs"
        self.decoded_dir = data_dir / "logs"
        self.decoded_fields = decoded_fields
        self.rotate_s = rotate_s
        self.retention_days = retention_days
        self.decoded_period = 1.0 / decoded_hz
        self.stale_after_s = stale_after_s
        self.decoded_log = RotatingJsonlLog(self.decoded_dir, rotate_s, retention_days)

    def start(self):
        self.raw_dir.mkdir(parents=True, exist_ok=True)
        self._sweep(self.raw_dir, f"*{self.raw_suffix}")
        self.decoded_log.start()
        threading.Thread(target=self._raw_rotate_loop, daemon=True).start()
        threading.Thread(target=self._decoded_loop, daemon=True).start()

    def _sweep(self, directory, pattern):
        cutoff = time.time() - self.retention_days * 86400
        for file in directory.glob(pattern):
            if file.stat().st_mtime < cutoff:
                file.unlink()

    def _timestamp(self):
        return time.strftime("%y%m%d_%H%M%S")

    def _wait_for_entry(self):
        # nrxs.nrx[xnav_ip] is only created lazily, on the first UDP
        # packet actually received from that address - at process start
        # (or while the xNAV is rebooting/reconfiguring) it may not exist
        # yet.
        while True:
            with self.nrxs.lock:
                entry = self.nrxs.nrx.get(self.xnav_ip)
                if entry is not None:
                    return entry
            time.sleep(1.0)

    def _raw_rotate_loop(self):
        # Aligned to the wall-clock hour, same reasoning/math as
        # shared/rotating_jsonl_log.py's _next_boundary() — this loop
        # can't reuse that class directly (it manages a raw file *handle*
        # that ncomrx_thread.py/ucomrx_thread.py write into directly,
        # not line-by-line append() calls), so the alignment is
        # duplicated here rather than shared.
        entry = self._wait_for_entry()
        while True:
            now = time.time()
            next_boundary = (int(now) // self.rotate_s + 1) * self.rotate_s
            path = self.raw_dir / f"{self._timestamp()}{self.raw_suffix}"
            fp = open(path, "wb")
            with self.nrxs.lock:
                entry["logfile"] = fp
            time.sleep(max(0.0, next_boundary - time.time()))
            with self.nrxs.lock:
                entry["logfile"] = None
            fp.close()
            self._sweep(self.raw_dir, f"*{self.raw_suffix}")

    def _decoded_loop(self):
        while True:
            tick_start = time.monotonic()
            snap = nav_feed.snapshot(self.nrxs, self.xnav_ip, self.stale_after_s)
            merged = {**snap["nav"], **snap["status"]}
            self._add_derived_fields(merged)
            record = {k: merged[k] for k in self.decoded_fields if k in merged}
            if record:
                self.decoded_log.append(record)
            time.sleep(max(0.0, self.decoded_period - (time.monotonic() - tick_start)))

    def _add_derived_fields(self, merged: dict) -> None:
        """Values worth logging that NCOM doesn't hand over directly —
        added here (rather than only computed downstream, e.g. in
        navigate) so oxts-nav's own log is self-sufficient. Only ever
        adds keys, never overwrites a raw NCOM field's own name.

        - lat/lon: NCOM's own Lat/Lon are radians (see ncomrx.py) and
          the raw field names don't match what viewer's map layer looks
          for (lowercase lat/lon, degrees — see
          shared/web/static/... home.html's fileHasPosition()) — found
          live 2026-09-08, "the map isn't picking up lat/lon". Every
          other service's own log already uses this exact lowercase/
          degrees convention (see navigate/app.py's _current_position()
          for the same math.degrees() conversion) — matching it here
          rather than inventing a second convention.
        - HorizontalSpeed: NCOM has no speed field, only the Vn/Ve
          velocity components — same hypot() navigate's own
          _current_position() already uses for its own (now-removed,
          see 2026-09-08 session) horizontal_speed_mps, so a run's
          actual ground speed can be read straight from oxts-nav's log
          without cross-referencing a navigate run's file, which might
          not even exist for a given time period (oxts-nav logs
          continuously, navigate only logs while a path/turn is
          running)."""
        if "Lat" in merged and "Lon" in merged:
            merged["lat"] = math.degrees(merged["Lat"])
            merged["lon"] = math.degrees(merged["Lon"])
        if "Vn" in merged and "Ve" in merged:
            merged["HorizontalSpeed"] = math.hypot(merged["Vn"], merged["Ve"])
