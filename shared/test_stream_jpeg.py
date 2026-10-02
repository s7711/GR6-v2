import io
import unittest

import numpy as np
from PIL import Image

from stream_jpeg import encode_stream_jpeg


def decode(data):
    return Image.open(io.BytesIO(data))


class TestEncodeStreamJpeg(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(0)
        self.frame = rng.integers(0, 255, (960, 1280, 3), dtype=np.uint8)

    def test_scales_down_keeping_aspect(self):
        img = decode(encode_stream_jpeg(self.frame, 640, 50))
        self.assertEqual(img.size, (640, 480))
        self.assertEqual(img.format, "JPEG")

    def test_never_scales_up(self):
        img = decode(encode_stream_jpeg(self.frame, 2000, 50))
        self.assertEqual(img.size, (1280, 960))

    def test_lower_quality_is_smaller(self):
        self.assertLess(len(encode_stream_jpeg(self.frame, 640, 30)), len(encode_stream_jpeg(self.frame, 640, 90)))

    def test_bgr_is_reversed_for_display(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:, :, 0] = 255  # blue, in BGR order
        r, g, b = decode(encode_stream_jpeg(frame, 640, 95)).getpixel((320, 240))
        self.assertGreater(b, 200)
        self.assertLess(r, 50)


if __name__ == "__main__":
    unittest.main()
