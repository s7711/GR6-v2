"""JPEG encoding for the browser video streams (camera's /camera.mjpg,
aruco's /aruco.mjpg) - added 2026-10-02, shrinking what goes over wifi
without touching the frames anything measures from.

Why: the wifi dongle sits ~20-40cm from the GNSS antennas, and RTK went
wrong (xNAV rejecting GNSS) with the motors unplugged and the robot pulled
by hand - with fewer events once the dongle's power was cut to 10 dBm. A
full 1280x960 JPEG at the default quality is ~190KB, ~7.6 Mbit/s per open
tab at 5fps: more than the link carries, so the transmitter keys almost
non-stop, and more so on a weak link. 640x480 at quality 50 is ~40KB, about
a fifth.

Detection, calibration and the bore-sight capture all read full-resolution
frames from shared memory (shared/frame_ipc.py), not these streams, so
their accuracy is unaffected.
"""

import io

from PIL import Image


def encode_stream_jpeg(frame_bgr, width: int, quality: int) -> bytes:
    """frame_bgr: an HxWx3 BGR array (picamera2's "RGB888" is BGR byte
    order - reversed here for display). Scaled down to `width` pixels wide
    (aspect kept) if it's wider; never scaled up."""
    img = Image.fromarray(frame_bgr[:, :, ::-1])
    if img.width > width:
        img = img.resize((width, round(img.height * width / img.width)), Image.BILINEAR)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()
