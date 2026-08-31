"""Generic per-run jsonl log summarizing, shared by any service that
writes `data/logs/yymmdd_hhmmss*.jsonl` files (navigate, waterbutt,
jobs, missions, drive, ...) and by viewer, which reads all of them.
Ported from navigate/app.py's original `_log_file_summary`/
`_log_summaries` (which only knew about navigate's own and waterbutt's
logs) - see viewer-prd.md.
"""

import json
import logging
from pathlib import Path


def log_file_summary(path: Path) -> dict:
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
    return sorted(summaries, key=lambda s: s["start_t"] or 0, reverse=True)
