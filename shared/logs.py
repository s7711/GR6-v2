"""Generic per-run jsonl log summarizing, shared by any service that
writes `data/logs/yymmdd_hhmmss*.jsonl` files (navigate, waterbutt,
jobs, missions, drive, ...) and by viewer, which reads all of them.
Ported from navigate/app.py's original `_log_file_summary`/
`_log_summaries` (which only knew about navigate's own and waterbutt's
logs) - see viewer-prd.md.
"""

import json
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.file_cache import FileCache  # noqa: E402

# Cached per file, invalidated by mtime+size rather than a TTL (see
# shared/file_cache.py, promoted from here 2026-10-01). Added 2026-09-14:
# viewer's own refreshSources() polls GET /api/logs every 5s, which used to
# mean log_file_summary() re-read-and-JSON-parsed *every* line of *every*
# log file of *every* service on *every single poll*, forever, for as
# long as the page stayed open - found live pegging a whole CPU core
# once enough log history had accumulated (most of it long-finished
# runs that will never change again). Process-wide, not per-request -
# there's only one viewer process, and this is exactly the kind of
# read to share across concurrent requests (multiple browser tabs)
# rather than repeat per-request.
_cache = FileCache()


def _read_log_file_summary(path: Path) -> dict:
    lines = path.read_text().splitlines()
    if not lines:
        return {"filename": path.name, "start_t": None, "end_t": None, "line_count": 0, "path_name": None}
    first = json.loads(lines[0])
    # The file for a run that's still going gets read mid-write here
    # sometimes - its last line can be a torn/incomplete write caught
    # between the writer's open() and close() (see navigate-prd.md's log
    # viewer notes - a bad file here used to take the whole listing
    # down). Fall back through trailing lines until one actually parses.
    last = None
    for line in reversed(lines):
        try:
            last = json.loads(line)
            break
        except json.JSONDecodeError:
            continue
    return {
        "filename": path.name,
        "start_t": first.get("t"),
        "end_t": last.get("t") if last else None,
        "line_count": len(lines),
        "path_name": first.get("path_name"),  # None for sources that don't have one (e.g. waterbutt, drive)
    }


def log_file_summary(path: Path) -> dict:
    return _cache.get(path, _read_log_file_summary)


def log_summaries(logs_dir: Path) -> list:
    if not logs_dir.exists():
        return []
    summaries = []
    for p in logs_dir.glob("*.jsonl"):
        try:
            summaries.append(log_file_summary(p))
        except json.JSONDecodeError:
            # First line itself unparseable - genuinely corrupt, not just
            # a live-write race. Skip it rather than take the whole
            # listing down for every other file.
            logging.warning("Skipping unreadable log file %s", p)
        except FileNotFoundError:
            # Swept for retention (or, for a run's own log, simply never
            # written) between glob() and stat()/read - another poll a
            # few seconds later will just see it gone from the listing.
            continue
    return sorted(summaries, key=lambda s: s["start_t"] or 0, reverse=True)
