"""Rebuilds a MapGrid from previously-captured raw detection records —
the offline counterpart to app.py's live tick, sharing grid.py's exact
same record_detection/effective_range_m so a replay is a faithful
reproduction of what the live grid would have done with the same
parameters. See map-manager-prd.md's "Post-processor" for the design
this implements: parameters (log-odds probabilities, cell size, max
range, beam angle) can differ from what was live at capture time, since
each raw line only ever recorded the robot pose, which sensor, and the
raw millimetre reading - not anything already derived from a fixed set
of parameters.
"""

import json
import time
from pathlib import Path

from grid import LogOddsParams, MapGrid, effective_range_m, sensor_world_pose


def list_raw_logs(raw_logs_dir: Path) -> list:
    """[{"filename", "start_t", "end_t", "line_count"}, ...] - same
    shape as navigate's own /api/logs entries, for the reprocess page's
    file-selection list to reuse that convention directly."""
    entries = []
    if not raw_logs_dir.exists():
        return entries
    for path in sorted(raw_logs_dir.glob("*.jsonl")):
        start_t = None
        end_t = None
        line_count = 0
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except ValueError:
                    continue
                line_count += 1
                t = record.get("t")
                if t is None:
                    continue
                if start_t is None or t < start_t:
                    start_t = t
                if end_t is None or t > end_t:
                    end_t = t
        entries.append({"filename": path.name, "start_t": start_t, "end_t": end_t, "line_count": line_count})
    return entries


def build_grid_from_raw_logs(raw_logs_dir: Path, filenames: list, sensors: list, cell_size_m: float,
                              max_range_m: float, beam_half_angle_deg: float, forget_after_s: float,
                              params: LogOddsParams) -> MapGrid:
    """Replays the chosen raw-log files, in filename order (which sorts
    chronologically - see app.py's timestamp-based naming), through a
    fresh MapGrid. `sensors` is the calibration table to project each
    record's robot pose with - not necessarily the same one live at
    capture time, since a raw record only stores robot pose/tag/range."""
    sensors_by_tag = {s["tag"]: s for s in sensors}
    grid = MapGrid(cell_size_m=cell_size_m, max_range_m=max_range_m, beam_half_angle_deg=beam_half_angle_deg,
                    forget_after_s=forget_after_s, params=params)
    for filename in sorted(filenames):
        path = raw_logs_dir / filename
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except ValueError:
                    continue
                sensor = sensors_by_tag.get(record["tag"])
                if sensor is None:
                    continue  # a tag no longer present in the calibration table
                range_m = effective_range_m(record.get("range_mm"), max_range_m)
                sensor_north, sensor_east, beam_heading_deg = sensor_world_pose(
                    record["north"], record["east"], record["heading_deg"], sensor["x"], sensor["y"], sensor["heading_deg"]
                )
                grid.record_detection(sensor_north, sensor_east, beam_heading_deg, range_m)
    return grid


def _unique_path(directory: Path, stem: str, suffix: str) -> Path:
    """Same "never silently overwrite" collision handling as waterbutt's
    _start_new_qc_log - two reprocess runs landing in the same second
    (or a re-run against the same second's data) get their own file."""
    candidate = directory / f"{stem}{suffix}"
    n = 1
    while candidate.exists():
        candidate = directory / f"{stem}_{n}{suffix}"
        n += 1
    return candidate


def write_snapshot(grids_dir: Path, grid: MapGrid, filenames: list, sensors: list) -> dict:
    """Writes grid_yymmdd_hhmmss.json + a paired settings_yymmdd_hhmmss.
    json recording everything that went into building it - see
    map-manager-prd.md's "Post-processor". Returns {"grid_filename",
    "settings_filename", "stats"}."""
    grids_dir.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%y%m%d_%H%M%S")
    grid_path = _unique_path(grids_dir, f"grid_{timestamp}", ".json")
    stem = grid_path.stem.replace("grid_", "settings_", 1)
    settings_path = grids_dir / f"{stem}.json"

    with open(grid_path, "w") as f:
        json.dump(grid.dump_state(), f)
    with open(settings_path, "w") as f:
        json.dump({"input_files": sorted(filenames), "sensors": sensors, "created_at": time.time()}, f, indent=2)

    return {"grid_filename": grid_path.name, "settings_filename": settings_path.name, "stats": grid.stats()}
