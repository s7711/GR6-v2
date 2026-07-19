"""Captures frames from the Pi camera in a background thread.

Ported from GR6-v1's bgCamera2.py (see camera-prd.md "Prior art"), with
two changes: gain is captured alongside exposure (not just exposure),
and the capture loop schedules against a running target time instead of
a fixed sleep, so a single slow frame doesn't compound into permanent
drift over a long session.

Resolution is fixed, not a constructor parameter — see camera-prd.md
("Resolution: fixed, not configurable"): camera calibration is tied to
this exact resolution.
"""

import threading
import time

import picamera2

RESOLUTION = (1280, 960)  # (width, height) — fixed, see camera-prd.md


class BgCamera:
    def __init__(self, fps=5):
        self.period = 1.0 / fps
        self.picam = picamera2.Picamera2()
        config = self.picam.create_still_configuration(main={"format": "RGB888", "size": RESOLUTION})
        self.picam.configure(config)
        self.picam.start()
        self.picam.set_controls({"ExposureValue": 0.7})

        self._cond = threading.Condition()
        self._frame = None
        self._timestamp = None  # time.monotonic() seconds
        self._exposure_us = None
        self._gain = None
        self._sequence = 0
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        next_tick = time.monotonic()
        while self._running:
            request = self.picam.capture_request()
            frame = request.make_array("main")
            metadata = request.get_metadata()
            request.release()

            # SensorTimestamp is nanoseconds since boot (CLOCK_MONOTONIC),
            # same clock time.monotonic() uses — directly comparable to
            # oxts-nav's nav feed timestamps with no extra conversion.
            sensor_ns = metadata.get("SensorTimestamp")
            with self._cond:
                self._frame = frame
                self._timestamp = sensor_ns / 1e9 if sensor_ns is not None else time.monotonic()
                self._exposure_us = metadata.get("ExposureTime")
                self._gain = metadata.get("AnalogueGain")
                self._sequence += 1
                self._cond.notify_all()

            next_tick += self.period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()  # Fell behind — resync rather than spin to catch up

    def latest(self):
        """Waits for the next frame. Returns (frame, timestamp,
        exposure_us, gain, sequence)."""
        with self._cond:
            last_sequence = self._sequence
            while self._sequence == last_sequence:
                self._cond.wait()
            return self._frame, self._timestamp, self._exposure_us, self._gain, self._sequence

    def snapshot(self):
        """Non-blocking: the most recent frame's metadata, no pixels —
        for status display at a lower rate than the capture loop."""
        with self._cond:
            return {
                "timestamp": self._timestamp,
                "exposure_us": self._exposure_us,
                "gain": self._gain,
                "sequence": self._sequence,
            }

    def stop(self):
        self._running = False
        self._thread.join()
        self.picam.stop()
