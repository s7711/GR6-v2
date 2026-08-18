"""Shared "hasn't made progress" watchdog for PathRunner and TurnRunner
(see control.py/turn_control.py) — added 2026-08-18 because an
unattended robot that gets physically stuck (wedged, wheel slip, an
obstacle) would otherwise keep trying forever with nobody watching.
Same abort mechanism those runners already have for other limits
(localisation accuracy, heading-correction), just watching a different
quantity — distance moved for path-following, heading turned for
turning in place.

A resetting checkpoint, not a sliding buffer: every window_s, compare
against wherever things were at the start of that window. Precise
sub-window stall detection isn't needed for a coarse safety net like
this, and it avoids keeping a growing history around.
"""


class StallGuard:
    def __init__(self, window_s: float, min_progress: float):
        self.window_s = window_s
        self.min_progress = min_progress
        self._checkpoint = None  # (value, time)

    def reset(self, value, now: float):
        self._checkpoint = (value, now)

    def stalled(self, value, now: float, progress_fn):
        """Call every tick with the current progress-tracking value
        (e.g. an (north, east) position tuple, or a heading in degrees)
        and the current clock reading. progress_fn(checkpoint_value,
        value) -> a non-negative scalar measuring how far things have
        moved since the checkpoint, in whatever units min_progress is
        in. Returns that progress value if it's under min_progress once
        window_s has elapsed (caller should abort), else None — and
        slides the checkpoint forward every window_s regardless of
        outcome, so it's always checked against a fresh baseline rather
        than an ever-growing one."""
        if self._checkpoint is None:
            self.reset(value, now)
            return None
        checkpoint_value, checkpoint_time = self._checkpoint
        if now - checkpoint_time < self.window_s:
            return None
        progress = progress_fn(checkpoint_value, value)
        self.reset(value, now)
        return progress if progress < self.min_progress else None
