"""Entry point for the auto-revert timer (see recovery.py's
schedule_revert) — run standalone via systemd-run, not imported."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import recovery  # noqa: E402

if __name__ == "__main__":
    recovery.revert_now()
