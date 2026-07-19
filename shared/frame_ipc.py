"""Shared-memory publish/subscribe for camera frames.

See camera/camera-prd.md ("IPC: shared memory + seqlock"). One writer
(the camera service), any number of readers (aruco, future vision
services) — writes never wait for a reader, reads retry instead of
locking against the writer.

Layout: a small fixed-size header, then raw RGB888 pixel bytes.
    seq          Q (8 bytes) - seqlock counter. Odd while a write is in
                                progress, even when stable; frame number
                                is seq // 2.
    timestamp    d (8 bytes) - time.monotonic() seconds at capture (see
                                oxts-nav-prd.md "Nav data feed" for why
                                this clock, not perf_counter()).
    exposure_us  I (4 bytes) - exposure time, microseconds.
    gain         f (4 bytes) - analogue gain.
    width        H (2 bytes)
    height       H (2 bytes)

Reader protocol: read seq; if odd, retry (write in progress). Read
header + frame. Read seq again; if it changed, the read was torn —
retry. Otherwise the data is consistent.
"""

import struct
from multiprocessing import shared_memory

import numpy as np

_SEQ_FORMAT = "<Q"
_META_FORMAT = "<dIfHH"
SEQ_SIZE = struct.calcsize(_SEQ_FORMAT)
META_SIZE = struct.calcsize(_META_FORMAT)
HEADER_SIZE = SEQ_SIZE + META_SIZE
CHANNELS = 3  # RGB888


def frame_buffer_size(width, height):
    return HEADER_SIZE + width * height * CHANNELS


class FrameWriter:
    """Publishes frames into a named shared memory segment. Only the
    camera service should create one of these for a given name."""

    def __init__(self, name, width, height):
        self.name = name
        self.width = width
        self.height = height
        self._seq = 0
        size = frame_buffer_size(width, height)
        try:
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        except FileExistsError:
            # Stale segment from a crashed/killed previous run — unlink
            # and recreate rather than attach and risk a size mismatch.
            # See top-prd.md's "Debugging" section on unlink-if-stale.
            stale = shared_memory.SharedMemory(name=name, create=False)
            stale.close()
            stale.unlink()
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)

    def publish(self, frame: np.ndarray, timestamp: float, exposure_us: int, gain: float):
        height, width = frame.shape[0], frame.shape[1]
        odd_seq = self._seq + 1
        struct.pack_into(_SEQ_FORMAT, self.shm.buf, 0, odd_seq)  # Write starting
        struct.pack_into(_META_FORMAT, self.shm.buf, SEQ_SIZE, timestamp, int(exposure_us), float(gain), width, height)
        self.shm.buf[HEADER_SIZE:HEADER_SIZE + frame.nbytes] = frame.tobytes()
        self._seq = odd_seq + 1
        struct.pack_into(_SEQ_FORMAT, self.shm.buf, 0, self._seq)  # Write complete

    def close(self):
        self.shm.close()
        self.shm.unlink()


class FrameReader:
    """Attaches to an existing frame shared-memory segment (created by
    the camera service) and reads the latest frame."""

    def __init__(self, name):
        self.shm = shared_memory.SharedMemory(name=name, create=False)

    def read(self, max_retries=10):
        """Returns the latest frame + metadata dict, or None if
        max_retries torn/in-progress reads were hit in a row (the
        writer is either unusually busy or dead — a consumer should
        treat None as "try again next tick", not as an error)."""
        for _ in range(max_retries):
            seq1 = struct.unpack_from(_SEQ_FORMAT, self.shm.buf, 0)[0]
            if seq1 % 2 == 1:
                continue  # Write in progress
            timestamp, exposure_us, gain, width, height = struct.unpack_from(_META_FORMAT, self.shm.buf, SEQ_SIZE)
            frame_bytes = bytes(self.shm.buf[HEADER_SIZE:HEADER_SIZE + width * height * CHANNELS])
            seq2 = struct.unpack_from(_SEQ_FORMAT, self.shm.buf, 0)[0]
            if seq1 != seq2:
                continue  # Torn read
            frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((height, width, CHANNELS))
            return {
                "frame": frame,
                "timestamp": timestamp,
                "exposure_us": exposure_us,
                "gain": gain,
                "width": width,
                "height": height,
                "sequence": seq1 // 2,
            }
        return None

    def close(self):
        self.shm.close()
