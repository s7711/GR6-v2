"""Tank-level estimate: this robot's current electronics have no actual
level sensor, so this infers "probably empty enough to refill" purely
from how long the pump has been observed running, via drive's own
`pump` telemetry (not just whether *this* service asked for it to run -
a manually-jogged pump run from drive's own page counts too). See
waterbutt-prd.md's "Tank level estimate" for the design this
implements.
"""

import time


class LevelEstimate:
    """Tracks accumulated pump-on seconds since the tank was last
    considered full (a completed fill, or an explicit mark_full()).
    believed_empty() is true once that reaches drain_confirm_s. No
    persistence across a restart — deliberately always starts "full"
    (accumulator at zero) on every process start, per waterbutt-prd.md
    ("safer to assume full than empty, even though it usually won't
    be" - a wrong "full" costs one skipped fill; a wrong "empty" risks
    overflowing)."""

    def __init__(self, drain_confirm_s, now=time.monotonic):
        self.drain_confirm_s = drain_confirm_s
        self.now = now
        self._pump_on_since = None  # monotonic timestamp, or None while the pump's observed off
        self._accumulated_s = 0.0

    def set_pump_on(self, is_on: bool):
        """Call whenever fresh pump telemetry arrives - not just when
        this service itself turned the pump on/off."""
        now = self.now()
        if is_on and self._pump_on_since is None:
            self._pump_on_since = now
        elif not is_on and self._pump_on_since is not None:
            self._accumulated_s += now - self._pump_on_since
            self._pump_on_since = None

    def _current_accumulated_s(self) -> float:
        if self._pump_on_since is None:
            return self._accumulated_s
        return self._accumulated_s + (self.now() - self._pump_on_since)

    def believed_empty(self) -> bool:
        return self._current_accumulated_s() >= self.drain_confirm_s

    def mark_full(self):
        """Manual correction, or called automatically once a fill
        actually starts (see app.py's /go) - resets the drain
        accumulator, leaving any pump-on period already in progress
        counting from where it actually started."""
        self._accumulated_s = 0.0

    def mark_empty(self):
        """Manual correction ("I've looked, it's empty") - jumps
        straight to the confirmed-empty threshold rather than waiting
        out drain_confirm_s again."""
        self._accumulated_s = self.drain_confirm_s

    def status(self) -> dict:
        return {
            "believed_empty": self.believed_empty(),
            "pump_seconds_since_full": round(self._current_accumulated_s(), 1),
            "drain_confirm_s": self.drain_confirm_s,
        }
