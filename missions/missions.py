"""Mission file storage: list/load/save/delete saved missions as YAML
files under missions_dir. Mirrors navigate/paths.py's shape (same
filename-validation reasoning) rather than inventing a second storage
convention — see missions-prd.md's "Mission file format".
"""

import re
from pathlib import Path

import yaml

# Same reasoning as navigate/paths.py's _SAFE_NAME — only real
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


def list_missions(missions_dir) -> list:
    """[{"name":, "step_count":}, ...] for every saved mission, sorted
    by name."""
    missions_dir = Path(missions_dir)
    if not missions_dir.exists():
        return []
    result = []
    for file in sorted(missions_dir.glob("*.yaml")):
        data = yaml.safe_load(file.read_text()) or {}
        result.append({"name": file.stem, "step_count": len(data.get("steps", []))})
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
