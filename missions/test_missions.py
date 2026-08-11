import tempfile
import unittest
from pathlib import Path

import missions

SAMPLE_STEPS = [
    {"type": "run_job", "job": "Water kitchen bed"},
    {"type": "run_job", "job": "Water stable bed"},
]


class TestMissionStorage(unittest.TestCase):
    def setUp(self):
        self.missions_dir = Path(tempfile.mkdtemp())

    def test_round_trip(self):
        missions.save_mission(self.missions_dir, "flowerbeds", SAMPLE_STEPS)
        loaded = missions.load_mission(self.missions_dir, "flowerbeds")
        self.assertEqual(loaded["name"], "flowerbeds")
        self.assertEqual(loaded["steps"], SAMPLE_STEPS)

    def test_saved_as_yaml_file_with_name(self):
        missions.save_mission(self.missions_dir, "flowerbeds", SAMPLE_STEPS)
        self.assertTrue((self.missions_dir / "flowerbeds.yaml").exists())

    def test_load_missing_mission_raises(self):
        with self.assertRaises(FileNotFoundError):
            missions.load_mission(self.missions_dir, "does-not-exist")

    def test_delete_removes_file(self):
        missions.save_mission(self.missions_dir, "flowerbeds", SAMPLE_STEPS)
        missions.delete_mission(self.missions_dir, "flowerbeds")
        with self.assertRaises(FileNotFoundError):
            missions.load_mission(self.missions_dir, "flowerbeds")

    def test_delete_missing_mission_raises(self):
        with self.assertRaises(FileNotFoundError):
            missions.delete_mission(self.missions_dir, "does-not-exist")

    def test_list_missions_empty_directory(self):
        empty_dir = self.missions_dir / "does-not-exist-yet"
        self.assertEqual(missions.list_missions(empty_dir), [])

    def test_list_missions_metadata(self):
        missions.save_mission(self.missions_dir, "b-mission", SAMPLE_STEPS)
        missions.save_mission(self.missions_dir, "a-mission", SAMPLE_STEPS[:1])
        rows = missions.list_missions(self.missions_dir)
        self.assertEqual([r["name"] for r in rows], ["a-mission", "b-mission"])  # sorted
        self.assertEqual(rows[0]["step_count"], 1)
        self.assertEqual(rows[1]["step_count"], 2)

    def test_invalid_names_rejected(self):
        for bad_name in ["../escape", "a/b", ".hidden", "", "   "]:
            with self.assertRaises(missions.InvalidMissionName):
                missions.save_mission(self.missions_dir, bad_name, SAMPLE_STEPS)
            with self.assertRaises(missions.InvalidMissionName):
                missions.load_mission(self.missions_dir, bad_name)
            with self.assertRaises(missions.InvalidMissionName):
                missions.delete_mission(self.missions_dir, bad_name)

    def test_spaces_and_punctuation_are_fine(self):
        for name in ["Front beds", "Beds #2 (north)"]:
            missions.save_mission(self.missions_dir, name, SAMPLE_STEPS)
            self.assertEqual(missions.load_mission(self.missions_dir, name)["steps"], SAMPLE_STEPS)
            missions.delete_mission(self.missions_dir, name)

    def test_leading_and_trailing_whitespace_is_trimmed(self):
        missions.save_mission(self.missions_dir, "  Front beds  ", SAMPLE_STEPS)
        self.assertEqual(missions.load_mission(self.missions_dir, "Front beds")["steps"], SAMPLE_STEPS)


if __name__ == "__main__":
    unittest.main()
