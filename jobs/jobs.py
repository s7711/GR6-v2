"""Job file storage: list/load/save/delete saved jobs as YAML
files under jobs_dir. Mirrors navigate/paths.py's shape (same
filename-validation reasoning) rather than inventing a second storage
convention — see jobs-prd.md's "Job file format".
"""

import re
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.file_cache import FileCache, load_yaml  # noqa: E402

# Same reasoning as navigate/paths.py's _SAFE_NAME — only real
# path-traversal characters are excluded, everything else (spaces,
# punctuation) is a normal filename character.
_SAFE_NAME = re.compile(r"^[^./\\][^/\\]*$")


class InvalidJobName(ValueError):
    pass


def _validate_name(name: str) -> str:
    name = name.strip()
    if not _SAFE_NAME.match(name):
        raise InvalidJobName(f"Invalid job name: {name!r}")
    return name


def _file_for(jobs_dir, name: str) -> Path:
    return Path(jobs_dir) / f"{_validate_name(name)}.yaml"


# Per-file step count (shared/file_cache.py), so listing re-parses only
# the jobs that have actually been re-saved - same as navigate's
# list_paths (2026-10-01).
_step_counts = FileCache()


def _load_step_count(file: Path) -> int:
    return len((load_yaml(file) or {}).get("steps", []))


def list_jobs(jobs_dir) -> list:
    """[{"name":, "step_count":}, ...] for every saved job, sorted
    by name."""
    jobs_dir = Path(jobs_dir)
    if not jobs_dir.exists():
        return []
    result = []
    for file in sorted(jobs_dir.glob("*.yaml")):
        try:
            step_count = _step_counts.get(file, _load_step_count)
        except FileNotFoundError:
            continue  # deleted between glob() and stat()
        result.append({"name": file.stem, "step_count": step_count})
    return result


def load_job(jobs_dir, name: str) -> dict:
    """Raises FileNotFoundError if the job doesn't exist. Returns
    {"name":, "steps": [...]}."""
    data = yaml.safe_load(_file_for(jobs_dir, name).read_text()) or {}
    data.setdefault("name", name)
    data.setdefault("steps", [])
    return data


def save_job(jobs_dir, name: str, steps: list) -> None:
    jobs_dir = Path(jobs_dir)
    jobs_dir.mkdir(parents=True, exist_ok=True)
    _file_for(jobs_dir, name).write_text(
        yaml.safe_dump({"name": name, "steps": steps}, sort_keys=False)
    )


def delete_job(jobs_dir, name: str) -> None:
    """Raises FileNotFoundError if the job doesn't exist."""
    _file_for(jobs_dir, name).unlink()
