"""Marker map: the YAML file of surveyed marker id -> global pose.

See aruco-prd.md ("Marker map file: YAML, per-marker lat/lon, no shared
origin"). Read fresh off disk on every call — deliberately no in-memory
caching layer, so edits made via the Add Marker page are visible
immediately, with no cache-invalidation logic and no service restart
needed.
"""

from pathlib import Path

import yaml

# See aruco-prd.md's precision note: 9 decimal places gives comfortable
# sub-mm ground precision at any latitude (6 places =~ 11cm, 7 =~ 1.1cm,
# 8 =~ 1.1mm, 9 =~ 0.1mm) against a 1mm-or-better target. This is purely
# about how many digits get written to the file — float64 (what this is
# stored as regardless) already carries this precision natively; the
# risk is only ever writing too few digits, not the numeric type.
LATLON_DECIMALS = 9
ALT_DECIMALS = 3  # millimetres
ANGLE_DECIMALS = 3  # far beyond what a survey can actually achieve anyway


def load_markers(path) -> list:
    path = Path(path)
    if not path.exists():
        return []
    with open(path) as f:
        return yaml.safe_load(f) or []


def save_markers(path, markers: list) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(markers, f, sort_keys=False)


def _rounded(marker: dict) -> dict:
    return {
        "id": int(marker["id"]),
        "size": float(marker["size"]),
        "lat": round(float(marker["lat"]), LATLON_DECIMALS),
        "lon": round(float(marker["lon"]), LATLON_DECIMALS),
        "alt": round(float(marker["alt"]), ALT_DECIMALS),
        "heading": round(float(marker["heading"]), ANGLE_DECIMALS),
        "pitch": round(float(marker["pitch"]), ANGLE_DECIMALS),
        "roll": round(float(marker["roll"]), ANGLE_DECIMALS),
    }


def upsert_marker(path, marker: dict) -> None:
    """Add or overwrite (by id) a marker record — aruco-prd.md's
    "duplicate id: overwrite, no warning, for now" decision."""
    markers = load_markers(path)
    record = _rounded(marker)
    markers = [m for m in markers if int(m["id"]) != record["id"]]
    markers.append(record)
    markers.sort(key=lambda m: m["id"])
    save_markers(path, markers)


def find_marker(path, marker_id: int):
    for m in load_markers(path):
        if int(m["id"]) == int(marker_id):
            return m
    return None


def delete_marker(path, marker_id: int) -> bool:
    """Removes a marker by id. Returns False (no-op) if it wasn't there —
    for the marker management page's delete button."""
    markers = load_markers(path)
    remaining = [m for m in markers if int(m["id"]) != int(marker_id)]
    if len(remaining) == len(markers):
        return False
    save_markers(path, remaining)
    return True
