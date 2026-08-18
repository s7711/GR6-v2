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


def _angle_diff_deg(a, b):
    """Signed a-b, wrapped to [-180, 180) — same idea as
    navigate/geometry.py's angle_diff, kept local since it's a
    one-liner and grid.py has no other reason to depend on navigate's
    package."""
    return (a - b + 180) % 360 - 180


class MapGrid:
    """Sparse dict of (i, j) grid-cell-index -> {"hit", "miss", "t"}, plus
    a separate sparse dict of human overrides that always wins over
    sensor data and never decays. No raw event log — see
    map-manager-prd.md ("Why not a raw event list") for why that was
    deliberately dropped in favour of updating this grid live."""

    def __init__(self, cell_size_m, max_range_m, beam_half_angle_deg, forget_after_s, min_observations=2, now=time.time):
        self.cell_size_m = cell_size_m
        self.max_range_m = max_range_m
        self.beam_half_angle_deg = beam_half_angle_deg
        self.forget_after_s = forget_after_s
        self.min_observations = min_observations
        self._now = now
        self._cells = {}  # (i, j) -> {"hit": int, "miss": int, "t": float}
        self._overrides = {}  # (i, j) -> "blocked" | "clear"
        self.dirty = False

    def _index(self, north, east):
        return (round(north / self.cell_size_m), round(east / self.cell_size_m))

    def _cell_center(self, i, j):
        return i * self.cell_size_m, j * self.cell_size_m

    def _bump(self, index, hit, t):
        cell = self._cells.setdefault(index, {"hit": 0, "miss": 0, "t": t})
        cell["hit" if hit else "miss"] += 1
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
        {"source": "sensor", "p_occupied":, "hit":, "miss":, "age_s":} —
        an override always wins; otherwise a cell with too few
        observations, or nothing recent enough (see forget_after_s),
        reads as unknown rather than guessing. p_occupied is a first
        cut (see map-manager-prd.md) - expect to revisit once a path
        planner actually consumes this."""
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
        total = entry["hit"] + entry["miss"]
        if total < self.min_observations:
            return {"source": "unknown"}
        return {"source": "sensor", "p_occupied": entry["hit"] / total, "hit": entry["hit"], "miss": entry["miss"], "age_s": age_s}

    def set_override(self, north, east, value):
        if value not in ("blocked", "clear"):
            raise ValueError(f"bad override value {value!r}")
        self._overrides[self._index(north, east)] = value
        self.dirty = True

    def clear_override(self, north, east):
        self._overrides.pop(self._index(north, east), None)
        self.dirty = True

    def stats(self):
        total_hit = sum(c["hit"] for c in self._cells.values())
        total_miss = sum(c["miss"] for c in self._cells.values())
        return {"cells": len(self._cells), "overrides": len(self._overrides), "total_hit": total_hit, "total_miss": total_miss}

    def dump_state(self):
        return {
            "cells": {f"{i},{j}": v for (i, j), v in self._cells.items()},
            "overrides": {f"{i},{j}": v for (i, j), v in self._overrides.items()},
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
