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
                with self._lock:
                    self._state.update(updates)

    def send(self, command: str):
        self._serial.write(command.encode("utf-8"))

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._state)
