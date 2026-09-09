"""Owns the USB-serial connection to the motor-controller microcontroller:
a background thread reads and parses telemetry lines into shared state,
and `send()` writes a raw command line. See drive-prd.md ("Firmware
interface", "Solution: architecture / data flow").

`serial_factory` is injectable so tests can pass a fake serial-shaped
object instead of opening a real port (mirrors how `oxts-nav`/`aruco`
keep hardware access behind a swappable seam).
"""

import logging
import threading
import time

import serial as pyserial

import protocol


class SerialLink:
    def __init__(self, port, baud, serial_factory=pyserial.Serial):
        self._serial = serial_factory(port, baudrate=baud, timeout=0.5)
        self._lock = threading.Lock()
        self._state = {}

    def start(self):
        threading.Thread(target=self._read_loop, daemon=True).start()

    def _read_loop(self):
        # CRITICAL THREAD: the only place motor/battery telemetry is
        # ever received, and drive's control-arbitration timing depends
        # on it staying prompt. Already clean (checked 2026-09-09,
        # alongside the oxts-nav/navigate logging-stall fixes) - this
        # loop does no disk I/O and never will: app.py's own logging
        # runs on a separate _log_loop thread, appending a periodic
        # snapshot rather than anything from in here. Keep it that way.
        while True:
            try:
                raw = self._serial.readline()
            except OSError:
                logging.exception("[drive] Serial read failed")
                continue
            if not raw:
                continue  # Timeout with nothing received — normal, keep polling
            try:
                line = raw.decode("utf-8", errors="ignore")
            except UnicodeDecodeError:
                continue
            updates = protocol.parse_line(line)
            if updates:
                if "LM_vel_filt" in updates:
                    # Real arrival time of this specific telemetry line —
                    # not "whenever a consumer happens to poll" — so a GAD
                    # aiding consumer (wheelspeed) can compute the true
                    # midpoint of the averaging interval its velocity
                    # covers, rather than timestamping it "now". See
                    # wheelspeed-prd.md's "Update rate / timing".
                    updates = {**updates, "FV_timestamp": time.monotonic()}
                with self._lock:
                    self._state.update(updates)

    def send(self, command: str):
        # CRITICAL: called directly from whichever thread issues a drive
        # command (navigate's control loop, a manual jog request, ...) -
        # must stay a pure serial write, never anything that could block
        # on disk/network.
        self._serial.write(command.encode("utf-8"))

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._state)
