"""Persistent occupancy grid: pure logic, no Flask/IO — same
control.py-style separation as navigate/waterbutt/jobs use, so this is
unit-testable without a running service. See map-manager-prd.md
("Occupancy grid") for the design this implements.

Conventions: local (north, east) metres and compass heading degrees
(0=North, 90=East, clockwise) — the same as navigate/geometry.py, so a
detection's (north, east, heading_deg) is directly interoperable with
this project's other local-frame math. Unlike navigate's paths (which
reference their own first point), map-manager's frame is one fixed
origin for the whole robot's life — see map-manager-prd.md ("Reference
frame") for why.
"""

import math
import time


def sensor_world_pose(robot_north, robot_east, robot_heading_deg, sensor_x_m, sensor_y_m, sensor_heading_deg):
    """Where a sensor actually is and which way it's pointing in the
    local frame, given the robot's own pose and the sensor's fixed
    mounting offset in the robot's body frame (x forward, y right — same
    body-frame convention as aruco's camera_extrinsics/waterbutt's
    funnel offset). Pitch/roll of the robot are deliberately ignored —
    see map-manager-prd.md ("Reference frame") for why, and when that
    stops being a safe assumption."""
    heading_rad = math.radians(robot_heading_deg)
    world_dn = sensor_x_m * math.cos(heading_rad) - sensor_y_m * math.sin(heading_rad)
    world_de = sensor_x_m * math.sin(heading_rad) + sensor_y_m * math.cos(heading_rad)
    beam_heading_deg = (robot_heading_deg + sensor_heading_deg) % 360
    return robot_north + world_dn, robot_east + world_de, beam_heading_deg


def effective_range_m(range_mm, max_range_m):
    """Raw millimetre reading (or None) -> the range_m record_detection
    expects, or None for "nothing in range". Firmware's own "no echo"
    sentinel isn't documented (see drive/protocol.py) - clamping at
    max_range_m gets the same behaviour regardless of what the firmware
    actually reports on a timeout. A shared helper (not duplicated
    between app.py's live tick and reprocess.py's replay) specifically
    so max_range_m can differ between the two - raw captures store the
    untouched range_mm, letting a later replay reinterpret it against a
    different max_range_m than was live at capture time."""
    if range_mm is None:
        return None
    range_m = range_mm / 1000.0
    return None if range_m >= max_range_m else range_m


def _angle_diff_deg(a, b):
    """Signed a-b, wrapped to [-180, 180) — same idea as
    navigate/geometry.py's angle_diff, kept local since it's a
    one-liner and grid.py has no other reason to depend on navigate's
    package."""
    return (a - b + 180) % 360 - 180


def logit(p: float) -> float:
    """Probability (0-1, exclusive) -> log-odds. Inverse of sigmoid."""
    return math.log(p / (1.0 - p))


def sigmoid(l: float) -> float:
    """Log-odds -> probability (0-1). Inverse of logit."""
    return 1.0 / (1.0 + math.exp(-l))


class LogOddsParams:
    """The sensor-confidence/clamp numbers a grid update needs, as
    human-meaningful probabilities rather than raw log-odds - see
    map-manager-prd.md ("Occupancy grid: clamped log-odds") for what
    each one means and why clamping is what fixes the "15,000
    observations make the belief immovable" problem a plain hit/miss
    ratio has. Bundled into one object because the post-processor needs
    to pass a whole alternate set through at once (see reprocess.py)."""

    def __init__(self, p_hit, p_miss, p_min, p_max):
        self.p_hit = p_hit
        self.p_miss = p_miss
        self.p_min = p_min
        self.p_max = p_max
        self.l_hit = logit(p_hit)
        self.l_miss = logit(p_miss)
        self.l_min = logit(p_min)
        self.l_max = logit(p_max)

    def to_dict(self):
        return {"p_hit": self.p_hit, "p_miss": self.p_miss, "p_min": self.p_min, "p_max": self.p_max}


class MapGrid:
    """Sparse dict of (i, j) grid-cell-index -> {"log_odds", "t"}, plus
    a separate sparse dict of human overrides that always wins over
    sensor data and never decays. No raw event log in here - this is
    the *derived* grid; see raw_log.py for the separately-toggled raw
    capture a post-processor can rebuild one of these from."""

    def __init__(self, cell_size_m, max_range_m, beam_half_angle_deg, forget_after_s, params: LogOddsParams, now=time.time):
        self.cell_size_m = cell_size_m
        self.max_range_m = max_range_m
        self.beam_half_angle_deg = beam_half_angle_deg
        self.forget_after_s = forget_after_s
        self.params = params
        self._now = now
        self._cells = {}  # (i, j) -> {"log_odds": float, "t": float}
        self._overrides = {}  # (i, j) -> "blocked" | "clear"
        self.dirty = False

    def _index(self, north, east):
        return (round(north / self.cell_size_m), round(east / self.cell_size_m))

    def _cell_center(self, i, j):
        return i * self.cell_size_m, j * self.cell_size_m

    def _bump(self, index, hit, t):
        cell = self._cells.setdefault(index, {"log_odds": 0.0, "t": t})
        cell["log_odds"] += self.params.l_hit if hit else self.params.l_miss
        cell["log_odds"] = max(self.params.l_min, min(self.params.l_max, cell["log_odds"]))
        cell["t"] = t
        self.dirty = True

    def record_detection(self, sensor_north, sensor_east, beam_heading_deg, range_m):
        """range_m is the measured distance to an obstruction, or None if
        the sensor saw nothing within max_range_m. Marks a wedge of cells
        within the beam's half-angle: cells nearer than the
        obstruction (or the whole wedge, if range_m is None) as "miss",
        and cells right at the obstruction's range as "hit"."""
        sweep_range = min(range_m, self.max_range_m) if range_m is not None else self.max_range_m
        t = self._now()
        cell = self.cell_size_m
        min_i = math.floor((sensor_north - sweep_range) / cell)
        max_i = math.ceil((sensor_north + sweep_range) / cell)
        min_j = math.floor((sensor_east - sweep_range) / cell)
        max_j = math.ceil((sensor_east + sweep_range) / cell)
        for i in range(min_i, max_i + 1):
            for j in range(min_j, max_j + 1):
                center_north, center_east = self._cell_center(i, j)
                dn = center_north - sensor_north
                de = center_east - sensor_east
                dist = math.hypot(dn, de)
                if dist > sweep_range:
                    continue
                bearing_deg = math.degrees(math.atan2(de, dn)) % 360
                if abs(_angle_diff_deg(bearing_deg, beam_heading_deg)) > self.beam_half_angle_deg:
                    continue
                is_hit = range_m is not None and abs(dist - range_m) <= cell / 2
                self._bump((i, j), is_hit, t)

    def cell_value(self, north, east):
        """{"source": "override", "value": ...} | {"source": "unknown"} |
        {"source": "sensor", "p_occupied":, "age_s":} - an override
        always wins; otherwise a cell that's never been touched, or
        nothing recent enough (see forget_after_s), reads as unknown
        rather than guessing. p_occupied comes from the clamped
        log-odds value via sigmoid()."""
        index = self._index(north, east)
        override = self._overrides.get(index)
        if override is not None:
            return {"source": "override", "value": override}
        entry = self._cells.get(index)
        if entry is None:
            return {"source": "unknown"}
        age_s = self._now() - entry["t"]
        if age_s > self.forget_after_s:
            return {"source": "unknown"}
        return {"source": "sensor", "p_occupied": sigmoid(entry["log_odds"]), "age_s": age_s}

    def set_override(self, north, east, value):
        if value not in ("blocked", "clear"):
            raise ValueError(f"bad override value {value!r}")
        self._overrides[self._index(north, east)] = value
        self.dirty = True

    def clear_override(self, north, east):
        self._overrides.pop(self._index(north, east), None)
        self.dirty = True

    def stats(self):
        return {"cells": len(self._cells), "overrides": len(self._overrides)}

    def dump_state(self):
        return {
            "cells": {f"{i},{j}": v for (i, j), v in self._cells.items()},
            "overrides": {f"{i},{j}": v for (i, j), v in self._overrides.items()},
            "params": self.params.to_dict(),
            "cell_size_m": self.cell_size_m,
            "max_range_m": self.max_range_m,
            "beam_half_angle_deg": self.beam_half_angle_deg,
        }

    def load_state(self, data):
        self._cells = {}
        for key, val in data.get("cells", {}).items():
            i, j = (int(x) for x in key.split(","))
            self._cells[(i, j)] = val
        self._overrides = {}
        for key, val in data.get("overrides", {}).items():
            i, j = (int(x) for x in key.split(","))
            self._overrides[(i, j)] = val
        self.dirty = False
