import tempfile
import unittest
from pathlib import Path

import qc_marker

TVEC = [0.72, -0.12, 0.64]
RVEC = [0.1, 0.2, 0.05]


class TestQcMarker(unittest.TestCase):
    def setUp(self):
        self.path = Path(tempfile.mkdtemp()) / "qc-marker.yaml"

    def test_load_missing_file_returns_none(self):
        self.assertIsNone(qc_marker.load(self.path))

    def test_round_trip(self):
        qc_marker.save(self.path, 12, TVEC, RVEC)
        loaded = qc_marker.load(self.path)
        self.assertEqual(loaded, {"marker_id": 12, "tvec_camera_frame": TVEC, "rvec_camera_frame": RVEC})

    def test_save_overwrites_the_previous_record(self):
        # Only one saved record allowed - one waterbutt, one marker.
        qc_marker.save(self.path, 12, TVEC, RVEC)
        qc_marker.save(self.path, 7, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        loaded = qc_marker.load(self.path)
        self.assertEqual(loaded["marker_id"], 7)

    def test_creates_parent_directory(self):
        nested = self.path.parent / "nested" / "qc-marker.yaml"
        qc_marker.save(nested, 12, TVEC, RVEC)
        self.assertTrue(nested.exists())


if __name__ == "__main__":
    unittest.main()
