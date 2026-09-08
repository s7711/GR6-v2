"""Wheelspeed scale-factor estimator - added 2026-09-08, log-only for now
(see project memory's discussion of *why*: feeding a biased wheelspeed
into the Kalman filter once already caused a real, unexpected instability
via the accelerometer-bias estimate absorbing the error, so this is
deliberately observe-first, not fed back into gad_wheelspeed.py).

Compares each wheel's own real odometer distance (drive's LM_position_m/
RM_position_m) against that wheel's *predicted* distance if the INS
solution is right and there's no slip (velocity.wheel_forward_velocity(),
integrated over time) - once one side reaches `distance_m` (config,
Ben's suggested default 1m), a result is computed and both sides reset,
so this naturally re-triggers every ~1m of real travel rather than on a
fixed clock or a grid boundary.

Two rejection traps, both suggested live by Ben 2026-09-08, both "skip
this interval and start again" rather than trying to salvage a partial
reading:
  - left/right wheel distances disagreeing by more than
    `min_wheel_ratio` allows (default 90%) - a tight turn/spin-in-place
    can have one wheel barely move while the other covers the whole
    interval; there's no meaningful single "scale factor" for that shape
    of motion, and we don't want it polluting the log.
  - wheel-vs-INS distance disagreeing by more than `min_wheel_ins_ratio`
    allows (default 10%, i.e. reject anything below a 1:10 ratio either
    way) - catches the robot being picked up and carried (INS keeps
    integrating real motion, wheels don't turn at all) or, symmetrically,
    a wheel spinning free with no ground contact.
Both checks use the same min/max ratio shape, just different thresholds -
the first cares about *precision* (reject anything that isn't a fairly
clean straight-ish run), the second only cares about ruling out the
pathological "wheels and INS aren't even measuring the same event" case
and tolerates a real, large scale-factor bias (which is the whole point of
running this at all - see the 32%-on-grass finding that started this).

Triggering off *either* side (wheel or INS) reaching distance_m, not just
the wheel side, matters for the carried-robot case too - otherwise a long
carry would leave the wheel side stuck near zero forever while the INS
side ran away unbounded, corrupting whatever real driving happens once
it's set back down.
"""


def _ratio(a, b):
    if a <= 0 or b <= 0:
        return 0.0
    return min(a, b) / max(a, b)


class ScaleFactorTracker:
    def __init__(self, distance_m, min_wheel_ratio, min_wheel_ins_ratio, on_result):
        self.distance_m = distance_m
        self.min_wheel_ratio = min_wheel_ratio
        self.min_wheel_ins_ratio = min_wheel_ins_ratio
        self.on_result = on_result
        self._reset()

    def _reset(self):
        self.left_wheel_dist = 0.0
        self.right_wheel_dist = 0.0
        self.left_ins_dist = 0.0
        self.right_ins_dist = 0.0
        self._samples = []  # (t, lat, lon), for the midpoint lat/lon lookup
        self._last_left_pos = None
        self._last_right_pos = None

    def update(self, t, left_pos_m, right_pos_m, left_ins_mps, right_ins_mps, dt, lat=None, lon=None):
        """t: time.monotonic() at this sample. left_pos_m/right_pos_m:
        drive's cumulative wheel-odometer distance (LM_position_m/
        RM_position_m). left_ins_mps/right_ins_mps:
        velocity.wheel_forward_velocity() for each wheel. dt: real
        elapsed seconds since the previous call (not assumed fixed - see
        app.py's own tick-timing comment)."""
        if self._last_left_pos is not None:
            self.left_wheel_dist += abs(left_pos_m - self._last_left_pos)
            self.right_wheel_dist += abs(right_pos_m - self._last_right_pos)
            self.left_ins_dist += abs(left_ins_mps) * dt
            self.right_ins_dist += abs(right_ins_mps) * dt
        self._last_left_pos = left_pos_m
        self._last_right_pos = right_pos_m
        if lat is not None and lon is not None:
            self._samples.append((t, lat, lon))

        trigger_dist = max(self.left_wheel_dist, self.right_wheel_dist, self.left_ins_dist, self.right_ins_dist)
        if trigger_dist >= self.distance_m:
            self._finish()

    def _finish(self):
        wheel_ratio = _ratio(self.left_wheel_dist, self.right_wheel_dist)
        wheel_avg = (self.left_wheel_dist + self.right_wheel_dist) / 2
        ins_avg = (self.left_ins_dist + self.right_ins_dist) / 2
        wheel_ins_ratio = _ratio(wheel_avg, ins_avg)

        if wheel_ratio >= self.min_wheel_ratio and wheel_ins_ratio >= self.min_wheel_ins_ratio:
            lat = lon = None
            if self._samples:
                mid_t = (self._samples[0][0] + self._samples[-1][0]) / 2
                _, lat, lon = min(self._samples, key=lambda s: abs(s[0] - mid_t))
            self.on_result({
                "left_wheel_distance_m": self.left_wheel_dist,
                "right_wheel_distance_m": self.right_wheel_dist,
                "left_ins_distance_m": self.left_ins_dist,
                "right_ins_distance_m": self.right_ins_dist,
                "left_scale_estimate": self.left_ins_dist / self.left_wheel_dist if self.left_wheel_dist else None,
                "right_scale_estimate": self.right_ins_dist / self.right_wheel_dist if self.right_wheel_dist else None,
                "wheel_ratio": wheel_ratio,
                "wheel_ins_ratio": wheel_ins_ratio,
                "lat": lat,
                "lon": lon,
            })
        self._reset()
