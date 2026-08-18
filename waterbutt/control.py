"""Valve timing state machine for the waterbutt's ESP8266 pinch-valve
controller (see documentation/Waterbutt_261112-3.ino).

The firmware only knows "open" and "close" — there's no duration
parameter over there, and every /open call resets a 5-second fail-safe
timer that auto-closes the valve if nothing calls /open again in time
(deliberate on the firmware's side: if it ever loses contact with us,
the valve defaults to shut, not stuck open). So a "water for N seconds"
request from our side means re-issuing /open every REOPEN_INTERVAL_S
for as long as the requested duration lasts, then /close at the end —
this class owns exactly that timing, with send_open/send_close injected
so it can be tested without the real ESP8266 (see test_control.py),
same shape as navigate's PathRunner.
"""

import threading
import time

# 0.9s, not the much-more-conservative-looking 3s this was before
# 2026-08-15: with app.py's tick loop itself only running at 1Hz,
# anything above ~1s here would mean tick() doesn't necessarily catch
# every reopen on time. This trades a much smaller margin under the
# firmware's 5s FAILSAFE_TIMEOUT for a request going out roughly once a
# second instead of every 3s - deliberately, to paper over what looks
# like occasional dropped/delayed /open requests over wifi (accepted
# risk: a genuinely bad link could now trip the fail-safe *faster*, not
# slower, since there's less slack per missed send - see waterbutt-prd.md).
REOPEN_INTERVAL_S = 0.9


class ValveController:
    def __init__(self, send_open, send_close):
        self.send_open = send_open
        self.send_close = send_close
        self.lock = threading.RLock()
        self._reset()

    def _reset(self):
        self.state = "idle"  # idle | watering
        self.duration_s = 0.0
        self.remaining_s = 0.0
        self._deadline = None
        self._last_open_at = None

    def go(self, duration_s):
        """Starts (or restarts) a watering run of duration_s seconds,
        opening the valve immediately."""
        with self.lock:
            now = time.monotonic()
            self.state = "watering"
            self.duration_s = duration_s
            self.remaining_s = duration_s
            self._deadline = now + duration_s
            self._last_open_at = now
        self.send_open()

    def stop(self):
        """Operator-requested stop — closes the valve regardless of
        current state, since it's the "just in case" button and must
        always be trusted to actually shut the valve."""
        with self.lock:
            self._reset()
        self.send_close()

    def tick(self):
        """Call periodically (see app.py) while idle or watering — the
        thing that actually keeps the valve open past the firmware's
        5s window, and closes it the instant the duration runs out."""
        with self.lock:
            if self.state != "watering":
                return
            now = time.monotonic()
            self.remaining_s = max(0.0, self._deadline - now)
            if now >= self._deadline:
                do_open, do_close = False, True
                self._reset()
            elif now - self._last_open_at >= REOPEN_INTERVAL_S:
                do_open, do_close = True, False
                self._last_open_at = now
            else:
                do_open, do_close = False, False

        if do_open:
            self.send_open()
        if do_close:
            self.send_close()

    def status(self) -> dict:
        with self.lock:
            return {
                "state": self.state,
                "duration_s": self.duration_s,
                "remaining_s": round(self.remaining_s, 1),
            }
