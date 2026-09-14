"""Spatial map of the wheelspeed scale factor - added 2026-09-14, see
wheelspeed-prd.md ("Scale-factor map"). Ben found the scale factor
scale_factor.py already logs isn't one constant: it's noticeably
different on grass vs gravel, and may vary further with slope/ground
condition even within one groundcover. This turns the single running
value into a sparse 2D grid of per-cell running statistics, one cell
per ScaleFactorTracker sample's own footprint (scale_factor_distance_m,
usually 1m) - same sparse-dict-of-(i,j) shape as map-manager/grid.py's
MapGrid, since that already solved "grow a grid over an open-ended area
without pre-allocating anything", just with a different payload per
cell (running mean/stdev instead of log-odds occupancy).

Deliberately NOT wired into gad_wheelspeed.py yet - same "observe
first" reasoning as scale_factor.py's own docstring. Building the map
is independent of the GAD switch (see app.py) so it keeps learning (or
not - see the gating note below) regardless of whether GAD is being
sent at all.

Welford's online algorithm gives an exact running mean/variance from
one sample at a time, in O(1) storage per cell - no need to keep the
raw sample list the way scale_factor.py briefly does per-interval.
n is capped at max_n: below the cap this is an ordinary sample mean
(shrinking uncertainty as it fills in, same as an uncapped mean), but
capping it turns the update into a bounded-memory exponential-ish
average once n reaches max_n, so a cell can keep tracking a slow real
change (tyre wear, wet vs dry ground) instead of freezing solid once
it's seen "enough" samples - without needing a separate "trusted vs
re-estimating" state machine or a residual-based retrigger, which
would risk not being independent of the very estimate it is checking
(see the design discussion in wheelspeed-prd.md this was born from).
"""

import math


class ScaleFactorMap:
    def __init__(self, cell_size_m, max_n, now=None):
        self.cell_size_m = cell_size_m
        self.max_n = max_n
        self._now = now
        self._cells = {}  # (i, j) -> {"n", "mean", "m2", "t"}
        self.dirty = False

    def _index(self, north, east):
        return (round(north / self.cell_size_m), round(east / self.cell_size_m))

    def update(self, north, east, scale_factor, t=None):
        index = self._index(north, east)
        cell = self._cells.setdefault(index, {"n": 0, "mean": 0.0, "m2": 0.0, "t": t})
        n = min(cell["n"] + 1, self.max_n)  # capped n - see module docstring
        delta = scale_factor - cell["mean"]
        cell["mean"] += delta / n
        cell["m2"] += delta * (scale_factor - cell["mean"])
        cell["n"] = n
        cell["t"] = t if t is not None else (self._now() if self._now else None)
        self.dirty = True

    def cell(self, north, east):
        """{"n", "mean", "stdev", "t"} or None if this cell has never
        had a sample. stdev is None for a single-sample cell (there's no
        spread yet to report)."""
        entry = self._cells.get(self._index(north, east))
        if entry is None:
            return None
        stdev = math.sqrt(entry["m2"] / entry["n"]) if entry["n"] > 1 else None
        return {"n": entry["n"], "mean": entry["mean"], "stdev": stdev, "t": entry["t"]}

    def stats(self):
        return {"cells": len(self._cells)}

    def all_cells(self):
        """One entry per populated cell - {"north", "east", "n", "mean",
        "stdev", "t"} - "north"/"east" are the cell's centre, for a
        caller (e.g. wheelspeed/app.py's /api/scale-factor-map) to turn
        into lat/lon however it likes; this class stays local-frame-only,
        same as map-manager/grid.py's MapGrid."""
        cells = []
        for (i, j), entry in self._cells.items():
            north, east = i * self.cell_size_m, j * self.cell_size_m
            stdev = math.sqrt(entry["m2"] / entry["n"]) if entry["n"] > 1 else None
            cells.append({"north": north, "east": east, "n": entry["n"], "mean": entry["mean"], "stdev": stdev, "t": entry["t"]})
        return cells

    def dump_state(self):
        return {
            "cells": {f"{i},{j}": v for (i, j), v in self._cells.items()},
            "cell_size_m": self.cell_size_m,
            "max_n": self.max_n,
        }

    def load_state(self, data):
        self._cells = {}
        for key, val in data.get("cells", {}).items():
            i, j = (int(x) for x in key.split(","))
            self._cells[(i, j)] = val
        self.dirty = False
