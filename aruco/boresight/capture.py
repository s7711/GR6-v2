"""Bore-sight capture: record marker detections paired with an accurately
time-matched nav solution, and write them to a session file the solver
can read back.

The whole accuracy of the exercise rests on the time matching, so that's
what most of this module is. Pairing a frame with whatever nav sample
happens to be latest gives a 0-52ms sawtooth (the nav feed's update
interval), averaging ~25ms — and a capture-timestamp lag is *systematic*,
so it biases hpr_cb rather than averaging out. Measured in the simulator:
about 5.6 deg of hpr_cb error per second of lag, so 25ms is 0.14 deg,
which on its own blows the 0.1 deg target.

Both sides are ALREADY on Python's clock, and nothing here re-derives
that (Ben, 2026-09-16 — an earlier version did, via a GPS-epoch datetime
conversion, and did it badly):

  * camera/bg_camera.py stamps each frame with `SensorTimestamp` — the
    sensor's own exposure clock, CLOCK_MONOTONIC, exactly what
    time.monotonic() reads. No pipeline lag, no conversion needed.
  * ncomrx sets `connection['timeOffset']` as
    `GpsSeconds + GpsMinutes*60 - machineTime` (ncomrx.py:317), so
    inverting it gives each nav sample the machine time it was measured
    at — see nav_machine_time().

So frames and nav are compared directly in machine time, and nav is
interpolated to the frame's own exposure instant rather than taken
as-is. NavBuffer below is that interpolation.
"""

import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
# Nav samples arrive ~20Hz; a couple of seconds of history is far more
# than interpolation needs and keeps the buffer trivially small.
BUFFER_SECONDS = 4.0
# Refuse to interpolate across a gap longer than this — better to drop
# the frame than to invent a pose across a feed dropout.
MAX_INTERP_GAP_S = 0.5

REQUIRED_NAV_FIELDS = ("Lat", "Lon", "Alt", "Heading", "Pitch", "Roll", "GpsSeconds")
# Quality fields live in the feed's `status` dict, NOT in `nav` — reading
# them from nav silently yields None and the gate then rejects every
# frame ("not RTK fixed (GnssPosMode None)"), which is how this was
# found. The decoded LOG merges nav and status into one flat record, so
# the field layout there is no guide to the live feed's.
STATUS_QUALITY_FIELDS = ("NorthAcc", "EastAcc", "AltAcc", "HeadingAcc",
                         "GnssPosMode", "GnssAttMode", "InsNavMode")


def horizontal_speed(nav):
    """NCOM has no speed field of its own; oxts-nav derives it as
    hypot(Vn, Ve) when writing the decoded log (data_log.py's
    _add_derived_fields) and the live feed doesn't carry it at all."""
    vn, ve = nav.get("Vn"), nav.get("Ve")
    return None if vn is None or ve is None else math.hypot(vn, ve)


def nav_machine_time(nav, status, time_offset):
    """The machine time (time.monotonic scale) a nav sample was measured
    at — directly comparable with a camera frame's SensorTimestamp.

    Straight inversion of how ncomrx builds the offset in the first place
    (ncomrx.py:317):

        timeOffset = GpsSeconds + GpsMinutes*60 - machineTime

    Note `GpsSeconds` alone is seconds within the current MINUTE (13.29,
    not 311840), which is why `GpsMinutes` from `status` is needed with
    it. Keying a buffer on GpsSeconds by itself matches nothing and wraps
    every 60s — a 20-second live dry run rejected all 100 frames as "no
    time-matched nav" before this was understood.

    Working in machine time rather than converting both sides to GPS time
    also keeps the arithmetic at ~3e5 rather than ~1.5e9, where float64
    resolves nanoseconds instead of a quarter of a microsecond.
    """
    if time_offset is None or nav.get("GpsSeconds") is None or status.get("GpsMinutes") is None:
        return None
    return nav["GpsSeconds"] + status["GpsMinutes"] * 60.0 - time_offset


def _wrap180(degrees):
    return (degrees + 180.0) % 360.0 - 180.0


class NavBuffer:
    """Ring buffer of recent nav samples, with interpolation to an
    arbitrary GPS time.

    Heading is interpolated through the short way round the circle, not
    linearly: a vehicle crossing north goes 359 -> 1, and averaging those
    gives 180 — pointing exactly backwards. Pitch and roll never wrap in
    practice but are treated the same way for consistency rather than
    relying on that.
    """

    def __init__(self, max_seconds=BUFFER_SECONDS):
        self.max_seconds = max_seconds
        self._samples = []  # [(gps_seconds, nav_dict), ...] ascending
        self._lock = threading.Lock()

    def add(self, nav, status, time_offset):
        if not all(nav.get(f) is not None for f in REQUIRED_NAV_FIELDS):
            return False
        gps_s = nav_machine_time(nav, status, time_offset)
        if gps_s is None:
            return False
        with self._lock:
            if self._samples and gps_s <= self._samples[-1][0]:
                return False  # same sample again, or time went backwards
            self._samples.append((gps_s, dict(nav), dict(status)))
            cutoff = gps_s - self.max_seconds
            while len(self._samples) > 2 and self._samples[0][0] < cutoff:
                self._samples.pop(0)
        return True

    def __len__(self):
        with self._lock:
            return len(self._samples)

    def newest(self):
        """Time of the most recent sample, or -inf if empty — lets a
        caller tell "this frame is too new to bracket yet" (wait) from
        "this frame is older than anything held" (give up)."""
        with self._lock:
            return self._samples[-1][0] if self._samples else float("-inf")

    def span_s(self):
        with self._lock:
            if len(self._samples) < 2:
                return 0.0
            return self._samples[-1][0] - self._samples[0][0]

    def at(self, gps_seconds):
        """Nav interpolated to `gps_seconds`, or None if that time isn't
        bracketed by two samples close enough together to trust."""
        with self._lock:
            samples = list(self._samples)
        if len(samples) < 2:
            return None
        if gps_seconds < samples[0][0] or gps_seconds > samples[-1][0]:
            return None  # outside the buffer — never extrapolate

        times = [s[0] for s in samples]
        i = int(np.searchsorted(times, gps_seconds)) - 1
        i = max(0, min(i, len(samples) - 2))
        t0, a, status_a = samples[i]
        t1, b, status_b = samples[i + 1]
        if t1 - t0 > MAX_INTERP_GAP_S:
            return None
        f = 0.0 if t1 == t0 else (gps_seconds - t0) / (t1 - t0)

        out = {"machine_time": gps_seconds}
        for key in ("Lat", "Lon", "Alt"):
            out[key] = a[key] + f * (b[key] - a[key])
        for key in ("Heading", "Pitch", "Roll"):
            out[key] = _wrap180(a[key] + f * _wrap180(b[key] - a[key]))
        # Carry the quality fields from the nearer sample rather than
        # blending them — an accuracy figure or a mode number isn't
        # meaningful halfway between two values.
        nearer_status = status_a if f < 0.5 else status_b
        for key in STATUS_QUALITY_FIELDS:
            if key in nearer_status:
                out[key] = nearer_status[key]
        speed = horizontal_speed(a if f < 0.5 else b)
        if speed is not None:
            out["HorizontalSpeed"] = speed
        return out


class Gate:
    """Should this frame's detections be recorded?

    Deliberately permissive about motion — the whole point of
    interpolating nav is that moving capture is fine, and moving capture
    yields several times the data per minute that stopping does. What's
    rejected is data that can't be trusted: no RTK fix, a heading the INS
    itself doesn't believe, or a nav solution that couldn't be
    interpolated to the frame's time.
    """

    def __init__(self, max_heading_acc_deg=0.5, max_pos_acc_m=0.05,
                 require_rtk=True, max_speed_mps=0.6):
        self.max_heading_acc_deg = max_heading_acc_deg
        self.max_pos_acc_m = max_pos_acc_m
        self.require_rtk = require_rtk
        self.max_speed_mps = max_speed_mps

    def check(self, nav):
        """Returns None if acceptable, else a short reason string."""
        if nav is None:
            return "no time-matched nav"
        if self.require_rtk and nav.get("GnssPosMode") != 6:
            return f"not RTK fixed (GnssPosMode {nav.get('GnssPosMode')})"
        acc = max(nav.get("NorthAcc", 99.0), nav.get("EastAcc", 99.0))
        if acc > self.max_pos_acc_m:
            return f"position accuracy {acc:.3f} m"
        if nav.get("HeadingAcc", 99.0) > self.max_heading_acc_deg:
            return f"heading accuracy {nav.get('HeadingAcc'):.2f} deg"
        if nav.get("HorizontalSpeed", 0.0) > self.max_speed_mps:
            return f"speed {nav.get('HorizontalSpeed'):.2f} m/s"
        return None


class Session:
    """One capture run, appended to a .jsonl file as it goes.

    Written incrementally rather than held in memory and saved at the
    end: a 20-minute run is thousands of detections, and losing the lot
    to a crash or a stopped service at minute 19 would mean re-driving
    the whole pattern.
    """

    def __init__(self, path, marker_ids, marker_size, hpr_cb, dxc_b):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.marker_ids = list(marker_ids)
        self.started_at = time.time()
        self.counts = {int(i): 0 for i in self.marker_ids}
        self.rejected = {}
        self.frames = 0
        self._lock = threading.Lock()
        self._file = self.path.open("a")
        self._write({
            "type": "header", "started_at": self.started_at,
            "marker_ids": self.marker_ids, "marker_size": marker_size,
            "hpr_cb_at_capture": list(hpr_cb), "dxc_b": list(dxc_b),
        })

    def _write(self, record):
        self._file.write(json.dumps(record) + "\n")
        self._file.flush()

    def reject(self, reason):
        with self._lock:
            self.rejected[reason] = self.rejected.get(reason, 0) + 1

    def add(self, gps_seconds, nav, detections):
        """detections: [{id, corners (4x2), size}, ...] already filtered
        to the bore-sight marker ids."""
        with self._lock:
            self.frames += 1
            for d in detections:
                self.counts[int(d["id"])] = self.counts.get(int(d["id"]), 0) + 1
                self._write({
                    "type": "obs",
                    "t": gps_seconds,
                    "id": int(d["id"]),
                    "size": float(d["size"]),
                    "corners": np.asarray(d["corners"], dtype=float).reshape(4, 2).tolist(),
                    # DEGREES on disk. NCOM's nav dict carries Lat/Lon in
                    # radians (see oxts-nav/ncomrx.py) while the marker
                    # map, geodesy and everything else use degrees, so the
                    # conversion happens once, here, at the boundary.
                    #
                    # Do NOT be tempted to infer the unit downstream from
                    # the magnitude: this site's longitude is -1.46, which
                    # is an entirely plausible number of degrees AND of
                    # radians. An earlier version guessed, and silently
                    # turned -1.46 deg into -83.7 deg.
                    "lat": math.degrees(nav["Lat"]), "lon": math.degrees(nav["Lon"]),
                    "alt": nav["Alt"],
                    "heading": nav["Heading"], "pitch": nav["Pitch"], "roll": nav["Roll"],
                    "heading_acc": nav.get("HeadingAcc"),
                    "north_acc": nav.get("NorthAcc"),
                    "speed": nav.get("HorizontalSpeed"),
                })

    def status(self):
        with self._lock:
            return {
                "path": str(self.path),
                "started_at": self.started_at,
                "elapsed_s": time.time() - self.started_at,
                "frames": self.frames,
                "counts": dict(self.counts),
                "total": sum(self.counts.values()),
                "rejected": dict(self.rejected),
            }

    def close(self):
        with self._lock:
            if not self._file.closed:
                self._file.close()


def load_session(path):
    """Read a session file back as (header, Observations-ready columns).

    Returns (header, rows) where rows is a list of dicts. Converting to
    the solver's local-NED frame needs a reference origin, which is the
    caller's choice, so that's done in solve_session() rather than here.
    """
    header, rows = None, []
    for line in Path(path).read_text().splitlines():
        if not line.strip():
            continue
        rec = json.loads(line)
        if rec.get("type") == "header":
            header = rec
        elif rec.get("type") == "obs":
            rows.append(rec)
    return header, rows


def session_to_observations(rows, ref_lat, ref_lon, ref_alt, marker_ids=None):
    """Session rows -> a solve.Observations, in local NED about
    (ref_lat, ref_lon, ref_alt).

    Row lat/lon are DEGREES — Session.add converts once when writing, and
    nothing here tries to work the unit out for itself. See the note
    there for why that matters at this particular longitude.
    """
    import solve as solve_mod  # local import: capture.py is usable without scipy
    from shared.geodesy import lla_to_ned

    ids = sorted({r["id"] for r in rows}) if marker_ids is None else list(marker_ids)
    index = {mid: i for i, mid in enumerate(ids)}
    rows = [r for r in rows if r["id"] in index]
    if not rows:
        return None

    t = np.array([r["t"] for r in rows])
    pos = np.array([lla_to_ned(r["lat"], r["lon"], r["alt"], ref_lat, ref_lon, ref_alt)
                    for r in rows])
    hpr = np.array([[r["heading"], r["pitch"], r["roll"]] for r in rows])
    idx = np.array([index[r["id"]] for r in rows])
    corners = np.array([r["corners"] for r in rows])
    sizes = np.array([r["size"] for r in rows])
    return solve_mod.Observations(t, pos, hpr, idx, corners, sizes, ids)
