"""Control arbitration between manual (web jog page) and automatic
(future `navigate`/`missions`) callers of `drive`.

No lock that can *block* a manual command — the moment a human needs
to take over is often exactly when something automatic is going wrong,
so an override that has to wait or be granted is the wrong shape for a
safety mechanism. Instead: last-command-wins, except a manual command
holds control for a fixed window afterwards, so a slightly-delayed
automatic message can't slip in and fight the human mid-manoeuvre. See
drive-prd.md ("Control arbitration") for the full reasoning.
"""

import time

MANUAL = "manual"
AUTO = "auto"


class ControlArbiter:
    def __init__(self, hold_seconds: float = 0.5, clock=time.monotonic):
        self._hold_seconds = hold_seconds
        self._clock = clock
        self._manual_until = 0.0
        self._last_source = None

    def try_command(self, source: str) -> bool:
        """Returns True if a command from `source` should be accepted and
        forwarded to the microcontroller now."""
        now = self._clock()
        if source == MANUAL:
            self._manual_until = now + self._hold_seconds
            self._last_source = MANUAL
            return True

        if now < self._manual_until:
            return False  # A manual hold is active — automatic caller is rejected
        self._last_source = AUTO
        return True

    def status(self) -> dict:
        now = self._clock()
        locked = now < self._manual_until
        return {
            "controller": MANUAL if locked else (self._last_source or "none"),
            "manual_lock_until": self._manual_until if locked else None,
        }
