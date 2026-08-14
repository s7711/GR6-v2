"""Aruco-priority GNSS mode — see navigate-prd.md's "Aruco priority".

`navigate` tells us (via app.py's /gnss/aruco-priority, /gnss/normal)
when it's about to run a path that wants GNSS disabled in favour of
the aruco marker alone, and when that's over. We own the actual xNAV
command and the safety fallback: if no marker has been detected for
`timeout_s` (config: oxts-nav.aruco_priority_timeout_s), we re-enable
GNSS automatically regardless of what
navigate does or doesn't tell us next — navigate asking for aruco
priority is a request, this timeout is the guarantee that a lost
marker (camera fault, marker knocked over, aruco itself down) can
never leave the robot running on dead reckoning alone indefinitely.

Watches aruco's own /ws/aruco feed as a client the whole time (not
just while in aruco-priority mode) — same always-connected,
reconnect-on-failure pattern waterbutt's own QC loop already uses for
the same feed. Simpler than starting/stopping that connection on every
mode change, and the idle cost of one open websocket is negligible.
"""

import json
import logging
import threading
import time

from simple_websocket import Client as WsClient

ARUCO_RECONNECT_DELAY_S = 2.0
WATCHDOG_PERIOD_S = 0.5


class GnssModeController:
    def __init__(self, send_xnav_command, aruco_ws_url, timeout_s):
        self._send_xnav_command = send_xnav_command
        self._aruco_ws_url = aruco_ws_url
        self._timeout_s = timeout_s
        self._lock = threading.Lock()
        self._mode = "normal"
        self._last_marker_seen_at = None  # time.monotonic(); None until a marker's actually been seen

    def status(self):
        with self._lock:
            return {"mode": self._mode, "last_marker_seen_at": self._last_marker_seen_at}

    def enter_aruco_priority(self):
        with self._lock:
            self._mode = "aruco_priority"
            # Grace period starts from the moment we're asked, not from
            # whenever a marker last happened to be seen (which could
            # already be stale if aruco-priority wasn't running before).
            self._last_marker_seen_at = time.monotonic()
        self._send_xnav_command("!disable gnss")
        logging.info("[oxts-nav] Aruco priority: GNSS disabled")

    def exit_aruco_priority(self):
        with self._lock:
            if self._mode == "normal":
                return
            self._mode = "normal"
        self._send_xnav_command("!enable gnss")
        logging.info("[oxts-nav] Aruco priority: GNSS re-enabled")

    def _note_marker_seen(self):
        with self._lock:
            self._last_marker_seen_at = time.monotonic()

    def _watchdog_tick(self):
        """One check, factored out of _watchdog_loop so it can be
        tested directly without threads/sleep."""
        with self._lock:
            mode = self._mode
            last_seen = self._last_marker_seen_at
        if mode == "aruco_priority" and last_seen is not None and time.monotonic() - last_seen > self._timeout_s:
            logging.warning(
                "[oxts-nav] Aruco priority: no marker seen for over %.1fs, falling back to GNSS",
                self._timeout_s,
            )
            self.exit_aruco_priority()

    def _watchdog_loop(self):
        while True:
            self._watchdog_tick()
            time.sleep(WATCHDOG_PERIOD_S)

    def _aruco_feed_loop(self):
        while True:
            try:
                ws = WsClient.connect(self._aruco_ws_url)
                while True:
                    msg = json.loads(ws.receive())
                    if msg.get("debug"):
                        self._note_marker_seen()
            except Exception as e:
                logging.warning("[oxts-nav] Aruco priority: lost/couldn't reach aruco's feed (%s), retrying", e)
                time.sleep(ARUCO_RECONNECT_DELAY_S)

    def start(self):
        threading.Thread(target=self._aruco_feed_loop, daemon=True).start()
        threading.Thread(target=self._watchdog_loop, daemon=True).start()
