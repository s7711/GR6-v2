"""Path file storage: list/load/save/delete recorded paths as YAML files
under navigate's configured paths_dir. See navigate-prd.md's "Path
storage" for the per-point schema (lat/lon/speed_mps/pump/clearance_m)
and why lat/lon (not local XY) is what's persisted.
"""

import math
import re
from pathlib import Path

import yaml

import geometry

_SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]+$")


class InvalidPathName(ValueError):
    pass


def _validate_name(name: str) -> str:
    """Reject anything that isn't a plain filename component — no
    slashes, no "..", no leading dot — since `name` ultimately comes from
    an HTTP request and must never be used to escape paths_dir."""
    if not _SAFE_NAME.match(name):
        raise InvalidPathName(f"Invalid path name: {name!r}")
    return name


def _file_for(paths_dir, name: str) -> Path:
    return Path(paths_dir) / f"{_validate_name(name)}.yaml"


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
