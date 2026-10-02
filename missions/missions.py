"""Mission file storage: list/load/save/delete saved missions as YAML
files under missions_dir. Mirrors jobs/jobs.py's shape (which itself
mirrors navigate/paths.py) rather than inventing a third storage
convention — see missions-prd.md's "Mission file format".
"""

import re
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.file_cache import FileCache, load_yaml  # noqa: E402

# Same reasoning as navigate/paths.py's _SAFE_NAME - only real
# path-traversal characters are excluded, everything else (spaces,
# punctuation) is a normal filename character.
_SAFE_NAME = re.compile(r"^[^./\\][^/\\]*$")


class InvalidMissionName(ValueError):
    pass


def _validate_name(name: str) -> str:
    name = name.strip()
    if not _SAFE_NAME.match(name):
        raise InvalidMissionName(f"Invalid mission name: {name!r}")
    return name


def _file_for(missions_dir, name: str) -> Path:
    return Path(missions_dir) / f"{_validate_name(name)}.yaml"


# Per-file step count (shared/file_cache.py), so listing re-parses only
# the missions that have actually been re-saved - same as navigate's
# list_paths (2026-10-01).
_step_counts = FileCache()


def _load_step_count(file: Path) -> int:
    return len((load_yaml(file) or {}).get("steps", []))


def list_missions(missions_dir) -> list:
    """[{"name":, "step_count":}, ...] for every saved mission, sorted
    by name."""
    missions_dir = Path(missions_dir)
    if not missions_dir.exists():
        return []
    result = []
    for file in sorted(missions_dir.glob("*.yaml")):
        try:
            step_count = _step_counts.get(file, _load_step_count)
        except FileNotFoundError:
            continue  # deleted between glob() and stat()
        result.append({"name": file.stem, "step_count": step_count})
    return result


def load_mission(missions_dir, name: str) -> dict:
    """Raises FileNotFoundError if the mission doesn't exist. Returns
    {"name":, "steps": [...]}."""
    data = yaml.safe_load(_file_for(missions_dir, name).read_text()) or {}
    data.setdefault("name", name)
    data.setdefault("steps", [])
    return data


def save_mission(missions_dir, name: str, steps: list) -> None:
    missions_dir = Path(missions_dir)
    missions_dir.mkdir(parents=True, exist_ok=True)
    _file_for(missions_dir, name).write_text(
        yaml.safe_dump({"name": name, "steps": steps}, sort_keys=False)
    )


def delete_mission(missions_dir, name: str) -> None:
    """Raises FileNotFoundError if the mission doesn't exist."""
    _file_for(missions_dir, name).unlink()
