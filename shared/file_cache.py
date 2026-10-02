"""Per-file result cache, invalidated by mtime+size rather than a TTL - a
file that hasn't changed is never re-read, one that has is always re-read
on the next call. Promoted from shared/logs.py's log-summary cache
(2026-10-01, before the first endurance test) once navigate's path list,
jobs'/missions' lists and waterbutt's QC marker all turned out to re-read
and re-parse every file on every call too: navigate's /api/paths was
measured at ~750ms of pure-Python YAML parsing for 59 small paths, growing
with every path saved, all of it competing with the control loop for the
CPU and the GIL.

Callers get back the exact object compute() returned, shared with every
later caller - treat it as read-only (copy it before changing it).

Blind spot: file timestamps only tick every few ms on Linux, so a rewrite
to exactly the same size within that window looks unchanged. Fine for
files saved by a person pressing a button; don't use this for a file
something rewrites in a tight loop.
"""

import threading
from pathlib import Path

import yaml

# libyaml's C parser when it's installed (it is on the robot) - ~10x faster
# than the pure-Python one for the same safe subset of YAML.
_YAML_LOADER = getattr(yaml, "CSafeLoader", yaml.SafeLoader)


def load_yaml(path: Path):
    return yaml.load(Path(path).read_text(), Loader=_YAML_LOADER)


class FileCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._entries = {}  # path (str) -> {"mtime_ns", "size", "value"}

    def get(self, path: Path, compute):
        """compute(path)'s result, re-running compute only when the file's
        mtime or size has changed since the last call. Raises whatever
        stat()/compute raise (FileNotFoundError, a parse error, ...) and
        caches nothing in that case."""
        path = Path(path)
        stat = path.stat()
        key = str(path)
        with self._lock:
            cached = self._entries.get(key)
            if cached is not None and cached["mtime_ns"] == stat.st_mtime_ns and cached["size"] == stat.st_size:
                return cached["value"]

        # Deliberately outside the lock - a (potentially large) read/parse
        # shouldn't block every other file's lookup. Two callers racing on
        # the same freshly-changed file just do the same work twice.
        value = compute(path)

        with self._lock:
            self._entries[key] = {"mtime_ns": stat.st_mtime_ns, "size": stat.st_size, "value": value}
        return value

    def __contains__(self, path) -> bool:
        with self._lock:
            return str(path) in self._entries
