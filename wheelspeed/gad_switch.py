"""Persisted, live-togglable on/off switch for whether wheelspeed actually
sends GAD - added 2026-09-08 so the operator can stop/start aiding without
restarting the service (which would also interrupt logging - see
data_log.py/app.py's own logging, added the same day for the same reason
oxts-nav's was: turning things off shouldn't blind us to what happened).

Deliberately a small side-file, not a config.yaml key - every other
config value in this project is read once at startup, no live reload (see
top-prd.md's "Shared configuration"), and wheelspeed-prd.md's own "Out of
Scope" list explicitly ruled out editing config from this service's own
page for v1. A flag that must flip without a restart doesn't fit that
model anyway; this follows the same "service's own small persisted-choice
file" shape as network/recovery.json instead.

Defaults to OFF the very first time (no file yet) - Ben's call
2026-09-08: a fresh install/rebuild should never silently start aiding
before it's been deliberately turned on once.
"""

import json
import logging
import threading
from pathlib import Path


class GadSwitch:
    def __init__(self, path: Path):
        self.path = path
        self.lock = threading.Lock()
        self._enabled = self._load()

    def _load(self):
        try:
            return bool(json.loads(self.path.read_text())["enabled"])
        except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError):
            return False

    def enabled(self) -> bool:
        with self.lock:
            return self._enabled

    def set_enabled(self, enabled: bool):
        with self.lock:
            self._enabled = enabled
            self.path.parent.mkdir(parents=True, exist_ok=True)
            try:
                self.path.write_text(json.dumps({"enabled": enabled}))
            except OSError as e:
                logging.warning("[wheelspeed] Couldn't persist GAD switch: %s", e)
