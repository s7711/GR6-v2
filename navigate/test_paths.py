import tempfile
import unittest
from pathlib import Path

import paths

SAMPLE_POINTS = [
    {"lat": 52.2, "lon": -1.5, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
    {"lat": 52.2005, "lon": -1.5, "speed_mps": 0.6, "pump": True, "clearance_m": 1.0},
]


class TestPathStorage(unittest.TestCase):
    def setUp(self):
        self.paths_dir = Path(tempfile.mkdtemp())

    def test_round_trip(self):
        paths.save_path(self.paths_dir, "garden-loop", SAMPLE_POINTS)
        loaded = paths.load_path(self.paths_dir, "garden-loop")
        self.assertEqual(loaded, SAMPLE_POINTS)

    def test_saved_as_yaml_file_with_name(self):
        paths.save_path(self.paths_dir, "garden-loop", SAMPLE_POINTS)
        self.assertTrue((self.paths_dir / "garden-loop.yaml").exists())

    def test_load_missing_path_raises(self):
        with self.assertRaises(FileNotFoundError):
            paths.load_path(self.paths_dir, "does-not-exist")

    def test_delete_removes_file(self):
        paths.save_path(self.paths_dir, "garden-loop", SAMPLE_POINTS)
        paths.delete_path(self.paths_dir, "garden-loop")
        with self.assertRaises(FileNotFoundError):
            paths.load_path(self.paths_dir, "garden-loop")

    def test_delete_missing_path_raises(self):
        with self.assertRaises(FileNotFoundError):
            paths.delete_path(self.paths_dir, "does-not-exist")

    def test_list_paths_empty_directory(self):
        empty_dir = self.paths_dir / "does-not-exist-yet"
        self.assertEqual(paths.list_paths(empty_dir), [])

    def test_list_paths_metadata(self):
        paths.save_path(self.paths_dir, "b-path", SAMPLE_POINTS)
        paths.save_path(self.paths_dir, "a-path", SAMPLE_POINTS[:1])
        rows = paths.list_paths(self.paths_dir)
        self.assertEqual([r["name"] for r in rows], ["a-path", "b-path"])  # sorted
        self.assertEqual(rows[0]["point_count"], 1)
        self.assertEqual(rows[0]["length_m"], 0.0)  # single point, no length
        self.assertEqual(rows[1]["point_count"], 2)
        self.assertGreater(rows[1]["length_m"], 0.0)

    def test_invalid_names_rejected(self):
        for bad_name in ["../escape", "a/b", ".hidden", "", "   "]:
            with self.assertRaises(paths.InvalidPathName):
                paths.save_path(self.paths_dir, bad_name, SAMPLE_POINTS)
            with self.assertRaises(paths.InvalidPathName):
                paths.load_path(self.paths_dir, bad_name)
            with self.assertRaises(paths.InvalidPathName):
                paths.delete_path(self.paths_dir, bad_name)

    def test_spaces_and_punctuation_are_fine(self):
        # Found live 2026-07-30: "Water stable bed" was rejected by the
        # old digits/letters/underscore/hyphen-only rule for no security
        # reason at all — spaces aren't a path-traversal risk.
        for name in ["Water stable bed", "Bed #2 (north)", "Ben's patch"]:
            paths.save_path(self.paths_dir, name, SAMPLE_POINTS)
            self.assertEqual(paths.load_path(self.paths_dir, name), SAMPLE_POINTS)
            paths.delete_path(self.paths_dir, name)

    def test_leading_and_trailing_whitespace_is_trimmed(self):
        paths.save_path(self.paths_dir, "  Waterstablebed  ", SAMPLE_POINTS)
        self.assertEqual(paths.load_path(self.paths_dir, "Waterstablebed"), SAMPLE_POINTS)

    def test_load_flags_defaults_when_never_saved(self):
        paths.save_path(self.paths_dir, "garden-loop", SAMPLE_POINTS)
        self.assertEqual(paths.load_flags(self.paths_dir, "garden-loop"), {"aruco_priority": False})

    def test_save_and_load_flags_round_trip(self):
        paths.save_flags(self.paths_dir, "garden-loop", {"aruco_priority": True})
        self.assertEqual(paths.load_flags(self.paths_dir, "garden-loop"), {"aruco_priority": True})

    def test_flags_file_is_not_picked_up_by_list_paths(self):
        paths.save_path(self.paths_dir, "garden-loop", SAMPLE_POINTS)
        paths.save_flags(self.paths_dir, "garden-loop", {"aruco_priority": True})
        rows = paths.list_paths(self.paths_dir)
        self.assertEqual([r["name"] for r in rows], ["garden-loop"])

    def test_delete_path_also_removes_its_flags_file(self):
        paths.save_path(self.paths_dir, "garden-loop", SAMPLE_POINTS)
        paths.save_flags(self.paths_dir, "garden-loop", {"aruco_priority": True})
        paths.delete_path(self.paths_dir, "garden-loop")
        self.assertEqual(paths.load_flags(self.paths_dir, "garden-loop"), {"aruco_priority": False})

    def test_delete_path_is_fine_if_no_flags_were_ever_saved(self):
        paths.save_path(self.paths_dir, "garden-loop", SAMPLE_POINTS)
        paths.delete_path(self.paths_dir, "garden-loop")  # must not raise


if __name__ == "__main__":
    unittest.main()
