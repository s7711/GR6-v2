"""Saved snapshots of the xNAV650's config files - the xNAV Config
History page. See oxts-nav-prd.md's "xNAV config history".

Each snapshot is a folder under data/xnav-config-history/, named by the
operator (default YYMMDD_HHMMSS), holding the config files under their
real xNAV names (mobile.cfg, comment.txt, ...) - no ".txt" added, unlike
the xnav-config/ mirror - so a folder can go straight into NAVconfig.
Plain files, no index: a folder copied in by hand is a snapshot too.
"""

import difflib
import re
import shutil
from pathlib import Path

# Same rule as navigate's path names: a plain filename component - no
# slashes, no leading dot (which also hides the ".tmp-" folders below).
_SAFE_NAME = re.compile(r"^[^./\\][^/\\]*$")

# A diff of mobile.dbu (one 64 kB line) is no use to anyone.
_DIFF_MAX_LINE = 2000


class InvalidName(ValueError):
    pass


def _validate(name):
    name = name.strip()
    if not _SAFE_NAME.match(name):
        raise InvalidName(f"Invalid name: {name!r}")
    return name


def _dir_for(history_dir, name):
    return Path(history_dir) / _validate(name)


def _last_line(data):
    lines = [ln.strip() for ln in data.decode(errors="replace").splitlines() if ln.strip()]
    return lines[-1] if lines else ""


def list_snapshots(history_dir):
    """[{"name":, "files": [...], "comment":}, ...], name descending, so
    date-named snapshots come newest first. "comment" is the last line
    of the snapshot's comment.txt - the note written before saving."""
    history_dir = Path(history_dir)
    if not history_dir.exists():
        return []
    result = []
    for d in sorted(history_dir.iterdir(), key=lambda p: p.name, reverse=True):
        if not d.is_dir() or not _SAFE_NAME.match(d.name):
            continue
        comment = d / "comment.txt"
        result.append(
            {
                "name": d.name,
                "files": sorted(f.name for f in d.iterdir() if f.is_file()),
                "comment": _last_line(comment.read_bytes()) if comment.is_file() else "",
            }
        )
    return result


def save_snapshot(history_dir, name, files):
    """Write {filename: bytes} as a new snapshot. Never overwrites: an
    existing name raises FileExistsError. Written to a hidden temporary
    folder first, so a failure part-way leaves no half snapshot."""
    target = _dir_for(history_dir, name)
    if target.exists():
        raise FileExistsError(f"A snapshot called {target.name!r} already exists")
    tmp = Path(history_dir) / f".tmp-{target.name}"
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    for filename, data in files.items():
        (tmp / _validate(filename)).write_bytes(data)
    tmp.rename(target)
    return target.name


def load_snapshot(history_dir, name):
    d = _dir_for(history_dir, name)
    if not d.is_dir():
        raise FileNotFoundError(name)
    return {f.name: f.read_bytes() for f in sorted(d.iterdir()) if f.is_file()}


def read_file(history_dir, name, filename):
    f = _dir_for(history_dir, name) / _validate(filename)
    if not f.is_file():
        raise FileNotFoundError(filename)
    return f.read_bytes()


def delete_snapshot(history_dir, name):
    d = _dir_for(history_dir, name)
    if not d.is_dir():
        raise FileNotFoundError(name)
    shutil.rmtree(d)


def _diff(current, snapshot, filename, name):
    try:
        a = current.decode().splitlines()
        b = snapshot.decode().splitlines()
    except UnicodeDecodeError:
        return "(not text - the files differ)"
    if a == b:
        return "(same text - only line endings or a final newline differ)"
    if any(len(ln) > _DIFF_MAX_LINE for ln in a + b):
        return "(very long lines - the files differ; view the whole file)"
    return "\n".join(
        difflib.unified_diff(a, b, f"xNAV now: {filename}", f"{name}: {filename}", lineterm="", n=2)
    )


def compare(name, snapshot, current):
    """How the snapshot differs from the xNAV's files now, file by file:
    [{"file":, "status": "same"|"changed"|"snapshot_only"|"xnav_only",
    "diff":}] - the diff reads as what uploading the snapshot would do
    ("-" lines are the xNAV now, "+" lines the snapshot)."""
    result = []
    for f in sorted(set(snapshot) | set(current)):
        if f not in current:
            result.append({"file": f, "status": "snapshot_only", "diff": ""})
        elif f not in snapshot:
            result.append({"file": f, "status": "xnav_only", "diff": ""})
        elif snapshot[f] == current[f]:
            result.append({"file": f, "status": "same", "diff": ""})
        else:
            result.append({"file": f, "status": "changed", "diff": _diff(current[f], snapshot[f], f, name)})
    return result


def plan_upload(snapshot_names, current_names):
    """What uploading a snapshot does: every snapshot file is written,
    and every mobile.* file on the xNAV that the snapshot doesn't have is
    deleted, so the xNAV ends up matching it. comment.txt is never
    deleted - older snapshots (GR6-v1's) don't have one, and losing the
    notes would be worse than keeping a stale one."""
    return {
        "upload": sorted(snapshot_names),
        "delete": sorted(n for n in current_names if n not in snapshot_names and n.startswith("mobile.")),
    }
