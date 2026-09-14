"""Generic per-run jsonl log summarizing, shared by any service that
writes `data/logs/yymmdd_hhmmss*.jsonl` files (navigate, waterbutt,
jobs, missions, drive, ...) and by viewer, which reads all of them.
Ported from navigate/app.py's original `_log_file_summary`/
`_log_summaries` (which only knew about navigate's own and waterbutt's
logs) - see viewer-prd.md.
"""

import json
import logging
import threading
from pathlib import Path

# Cached per file (keyed by absolute path string, so the same cache
# serves every source directory viewer discovers), invalidated by
# mtime+size rather than a TTL - a file that hasn't changed doesn't
# need rescanning at all, one that has (still being actively written
# to) always gets a fresh read. Added 2026-09-14: viewer's own
# refreshSources() polls GET /api/logs every 5s, which used to mean
# log_file_summary() re-read-and-JSON-parsed *every* line of *every*
# log file of *every* service on *every single poll*, forever, for as
# long as the page stayed open - found live pegging a whole CPU core
# once enough log history had accumulated (most of it long-finished
# runs that will never change again). Process-wide, not per-request -
# there's only one viewer process, and this is exactly the kind of
# read to share across concurrent requests (multiple browser tabs)
# rather than repeat per-request.
_cache_lock = threading.Lock()
_cache = {}  # path (str) -> {"mtime_ns", "size", "summary"}


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
    stat = path.stat()
    key = str(path)
    with _cache_lock:
        cached = _cache.get(key)
        if cached is not None and cached["mtime_ns"] == stat.st_mtime_ns and cached["size"] == stat.st_size:
            return cached["summary"]

    # Deliberately outside the lock - the (potentially large) read/parse
    # itself shouldn't block every other file's cache lookup, only the
    # dict access around it needs to be serialised. Two requests racing
    # to (re-)compute the exact same freshly-changed file just do the
    # same work twice occasionally, rather than one blocking on the
    # other - a much smaller cost than serialising every file read.
    summary = _read_log_file_summary(path)

    with _cache_lock:
        _cache[key] = {"mtime_ns": stat.st_mtime_ns, "size": stat.st_size, "summary": summary}
    return summary


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
