"""GNSS-rejection hold (added 2026-10-02, Ben's design): stop a path run
while the xNAV is refusing GNSS, and only carry on once things have
settled.

Why: with the DC motors running, the u-blox sometimes keeps an RTK
"integer" fix that is actually wrong (an undetected cycle slip or two).
The xNAV sees the GNSS position disagree with its inertial solution and
rejects it (its GnssPosReject counter climbs); after a few seconds its
recovery logic gives in and follows GNSS, stepping its own output
position by ~0.2-0.5m - and the u-blox may then refix, which is another
step back. Seen live 2026-10-02 09:13:49 and 09:18:08: three such steps in
~10s each time, the second time 2s into a path with 0.5m clearance, which
aborted it on cross-track. Ben has spent a long time on the root cause
(DC motors interfering with GNSS) without a full fix, so this side-steps
it: navigate stops chasing a position the xNAV itself says is doubtful.

The rule:
- GnssPosReject reaching `threshold` (12 = ~3s of rejected updates at the
  u-blox's 4Hz) starts an event.
- The settle timer starts only once GnssPosReject has dropped back to 0 -
  the xNAV's recovery step (and any refix after it) happens around then,
  not while it's still rejecting.
- `settle_s` after that reset with no new excursion to `threshold`, the
  event ends. A new excursion during settling sends it back to waiting
  for the next reset (counted as a relapse) - the same event, not a new
  one.

This class only tracks the state; app.py decides what to do with it (hold
a path run, log the event, abort if it never settles). Tracked all the
time, not only during a run, so the event log also counts events that
happened while the robot was turning or idle.
"""

CLEAR = "clear"
REJECTING = "rejecting"  # waiting for GnssPosReject to drop back to 0
SETTLING = "settling"    # back at 0, waiting out settle_s


class GnssHold:
    def __init__(self, threshold: int, settle_s: float):
        self.threshold = threshold
        self.settle_s = settle_s
        self.state = CLEAR
        self.started_at = None  # monotonic time the current event began
        self.peak_reject = 0
        self.relapses = 0
        self._settle_started_at = None

    @property
    def active(self) -> bool:
        return self.state != CLEAR

    def duration_s(self, now: float):
        return None if self.started_at is None else now - self.started_at

    def settle_remaining_s(self, now: float):
        if self.state != SETTLING:
            return None
        return max(0.0, self.settle_s - (now - self._settle_started_at))

    def update(self, reject, now: float):
        """Call every control tick with the latest GnssPosReject (None if
        the xNAV isn't reporting it - treated as no news, the state is
        left as it is). Returns "started" or "ended" on the tick an event
        starts or ends, else None."""
        if reject is None:
            return None

        if self.state == CLEAR:
            if reject >= self.threshold:
                self.state = REJECTING
                self.started_at = now
                self.peak_reject = reject
                self.relapses = 0
                return "started"
            return None

        self.peak_reject = max(self.peak_reject, reject)

        if self.state == REJECTING:
            if reject == 0:
                self.state = SETTLING
                self._settle_started_at = now
            return None

        # SETTLING
        if reject >= self.threshold:
            self.state = REJECTING
            self.relapses += 1
            return None
        if now - self._settle_started_at >= self.settle_s:
            self.state = CLEAR
            return "ended"
        return None
