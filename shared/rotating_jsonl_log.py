"""Generic hourly-rotating, retention-swept jsonl logger. Every entry gets
a "t" (unix epoch seconds) field, matching what shared/logs.py's
log_file_summary() already expects from every service's data/logs/*.jsonl
file - so anything logged through this class is automatically
viewer-visible with no extra registration.

Pulled out of oxts-nav's data_log.py (2026-09-08) once wheelspeed needed
a near-identical logger the same day - see
[[feedback-prefer-existing-mechanisms]].
"""

import json
import logging
import time
from pathlib import Path


class RotatingJsonlLog:
    def __init__(self, directory: Path, rotate_s, retention_days):
        self.directory = directory
        self.rotate_s = rotate_s
        self.retention_days = retention_days
        self._path = None
        self._next_rotate_at = None

    def start(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        self._sweep()

    def _sweep(self):
        cutoff = time.time() - self.retention_days * 86400
        for file in self.directory.glob("*.jsonl"):
            if file.stat().st_mtime < cutoff:
                file.unlink()

    def _next_boundary(self, now: float) -> float:
        """The next multiple of rotate_s since the epoch — for the usual
        rotate_s=3600 this lands exactly on the wall-clock hour (:00),
        since a whole-hour UTC offset (as the UK always has, BST
        included) doesn't shift which second a UTC hour boundary falls
        on locally. Found live 2026-09-08: without this, rotation was
        "3600s after whenever the service last started/restarted" —
        e.g. files starting 13:34:49, 14:34:49 - which made it hard to
        tell by eye which file(s) a given time range actually needs
        (and see viewer's own Suggested list, which depends on exactly
        this to line files up sensibly)."""
        return (int(now) // self.rotate_s + 1) * self.rotate_s

    def append(self, record: dict):
        now = time.time()
        if self._path is None or now >= self._next_rotate_at:
            timestamp = time.strftime("%y%m%d_%H%M%S")
            self._path = self.directory / f"{timestamp}.jsonl"
            self._next_rotate_at = self._next_boundary(now)
            self._sweep()
        try:
            with open(self._path, "a") as f:
                f.write(json.dumps({"t": now, **record}, default=str) + "\n")
        except OSError as e:
            logging.warning("[rotating_jsonl_log] Couldn't write to %s: %s", self.directory, e)
