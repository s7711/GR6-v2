import shutil
import tempfile
import unittest
from pathlib import Path

import marker_map


class TestNudgeMarker(unittest.TestCase):
    def setUp(self):
        self.tmpdir = Path(tempfile.mkdtemp())
        self.path = self.tmpdir / "marker-map.yaml"
        marker_map.upsert_marker(self.path, {
            "id": 5, "size": 0.2, "lat": 52.0, "lon": -1.0, "alt": 10.0,
            "heading": 90.0, "pitch": 0.0, "roll": 0.0,
        })

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_nudge_north_increases_latitude(self):
        updated = marker_map.nudge_marker(self.path, 5, north_m=1.0)
        self.assertGreater(updated["lat"], 52.0)
        self.assertAlmostEqual(updated["lon"], -1.0, places=6)

    def test_nudge_east_increases_longitude_in_northern_hemisphere(self):
        updated = marker_map.nudge_marker(self.path, 5, east_m=1.0)
        self.assertGreater(updated["lon"], -1.0)
        self.assertAlmostEqual(updated["lat"], 52.0, places=6)

    def test_nudge_alt_adds_to_existing_altitude(self):
        updated = marker_map.nudge_marker(self.path, 5, alt_m=0.5)
        self.assertAlmostEqual(updated["alt"], 10.5)

    def test_nudge_persists_immediately(self):
        marker_map.nudge_marker(self.path, 5, north_m=1.0)
        reloaded = marker_map.find_marker(self.path, 5)
        self.assertNotEqual(reloaded["lat"], 52.0)

    def test_nudge_preserves_other_fields(self):
        updated = marker_map.nudge_marker(self.path, 5, north_m=1.0)
        self.assertEqual(updated["heading"], 90.0)
        self.assertEqual(updated["size"], 0.2)

    def test_nudge_unknown_marker_returns_none(self):
        self.assertIsNone(marker_map.nudge_marker(self.path, 999, north_m=1.0))


if __name__ == "__main__":
    unittest.main()
