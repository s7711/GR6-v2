"""Path file storage: list/load/save/delete recorded paths as YAML files
under navigate's configured paths_dir. See navigate-prd.md's "Path
storage" for the per-point schema (lat/lon/speed_mps/pump/clearance_m)
and why lat/lon (not local XY) is what's persisted.
"""

import json
import math
import re
from pathlib import Path

import yaml

import geometry

DEFAULT_FLAGS = {"aruco_priority": False}

# Only real path-traversal characters are excluded (no slashes, no
# leading dot) — anything else, including spaces, is a normal filename
# character and shouldn't be rejected. Found live 2026-07-30: the
# original [A-Za-z0-9_-]-only version rejected a perfectly reasonable
# name ("Water stable bed") for no security reason at all.
_SAFE_NAME = re.compile(r"^[^./\\][^/\\]*$")


class InvalidPathName(ValueError):
    pass


def _validate_name(name: str) -> str:
    """Reject anything that isn't a plain filename component — no
    slashes, no leading dot (blocks "." and ".." too) — since `name`
    ultimately comes from an HTTP request and must never be used to
    escape paths_dir. Everything else (spaces, punctuation, ...) is
    fine; leading/trailing whitespace is trimmed first so a stray space
    from a text box doesn't itself cause a rejection or a confusing
    filename."""
    name = name.strip()
    if not _SAFE_NAME.match(name):
        raise InvalidPathName(f"Invalid path name: {name!r}")
    return name


def _file_for(paths_dir, name: str) -> Path:
    return Path(paths_dir) / f"{_validate_name(name)}.yaml"


def _flags_file_for(paths_dir, name: str) -> Path:
    # .flags.json, not .yaml - list_paths()'s glob("*.yaml") must never
    # pick this up as if it were a path in its own right.
    return Path(paths_dir) / f"{_validate_name(name)}.flags.json"


def path_length_m(points: list) -> float:
    """Total path length in metres, via the same local-frame conversion
    used for path-following itself (see geometry.py) — not a separate
    great-circle calculation."""
    if len(points) < 2:
        return 0.0
    ref_lat, ref_lon = geometry.path_reference(points)
    local = [geometry.to_local(p["lat"], p["lon"], ref_lat, ref_lon) for p in points]
    total = 0.0
    for (n1, e1), (n2, e2) in zip(local, local[1:]):
        total += math.hypot(n2 - n1, e2 - e1)
    return total


def list_paths(paths_dir) -> list:
    """[{"name":, "point_count":, "length_m":}, ...] for every saved
    path, sorted by name."""
    paths_dir = Path(paths_dir)
    if not paths_dir.exists():
        return []
    result = []
    for file in sorted(paths_dir.glob("*.yaml")):
        points = yaml.safe_load(file.read_text()) or []
        result.append({
            "name": file.stem,
            "point_count": len(points),
            "length_m": path_length_m(points),
        })
    return result


def load_path(paths_dir, name: str) -> list:
    """Raises FileNotFoundError if the path doesn't exist."""
    return yaml.safe_load(_file_for(paths_dir, name).read_text()) or []


def save_path(paths_dir, name: str, points: list) -> None:
    paths_dir = Path(paths_dir)
    paths_dir.mkdir(parents=True, exist_ok=True)
    _file_for(paths_dir, name).write_text(yaml.safe_dump(points, sort_keys=False))


def delete_path(paths_dir, name: str) -> None:
    """Raises FileNotFoundError if the path doesn't exist."""
    _file_for(paths_dir, name).unlink()
    _flags_file_for(paths_dir, name).unlink(missing_ok=True)


def load_flags(paths_dir, name: str) -> dict:
    """Per-path behavioural flags (currently just aruco_priority - see
    navigate-prd.md's "Aruco priority") - a small sidecar file, kept
    deliberately separate from the points file rather than folded into
    it, since the points file's bare-list-of-points shape is read
    directly in a dozen places (jobs' continuity check, the reference-
    path overlay, run.html's map, ...) that have no reason to know
    about path-level flags at all. Missing file (every path saved
    before this existed) or missing keys default to DEFAULT_FLAGS, so
    no path needs migrating."""
    file = _flags_file_for(paths_dir, name)
    if not file.exists():
        return dict(DEFAULT_FLAGS)
    saved = json.loads(file.read_text())
    return {**DEFAULT_FLAGS, **saved}


def save_flags(paths_dir, name: str, flags: dict) -> None:
    paths_dir = Path(paths_dir)
    paths_dir.mkdir(parents=True, exist_ok=True)
    merged = {**DEFAULT_FLAGS, **flags}
    _flags_file_for(paths_dir, name).write_text(json.dumps(merged))
