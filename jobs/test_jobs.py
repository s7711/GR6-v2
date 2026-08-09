import tempfile
import unittest
from pathlib import Path

import jobs

SAMPLE_STEPS = [
    {"type": "run_path", "path": "HouseFrontEight"},
    {"type": "run_path", "path": "HouseFrontSquiggle"},
]


class TestJobStorage(unittest.TestCase):
    def setUp(self):
        self.jobs_dir = Path(tempfile.mkdtemp())

    def test_round_trip(self):
        jobs.save_job(self.jobs_dir, "front-beds", SAMPLE_STEPS)
        loaded = jobs.load_job(self.jobs_dir, "front-beds")
        self.assertEqual(loaded["name"], "front-beds")
        self.assertEqual(loaded["steps"], SAMPLE_STEPS)

    def test_saved_as_yaml_file_with_name(self):
        jobs.save_job(self.jobs_dir, "front-beds", SAMPLE_STEPS)
        self.assertTrue((self.jobs_dir / "front-beds.yaml").exists())

    def test_load_missing_job_raises(self):
        with self.assertRaises(FileNotFoundError):
            jobs.load_job(self.jobs_dir, "does-not-exist")

    def test_delete_removes_file(self):
        jobs.save_job(self.jobs_dir, "front-beds", SAMPLE_STEPS)
        jobs.delete_job(self.jobs_dir, "front-beds")
        with self.assertRaises(FileNotFoundError):
            jobs.load_job(self.jobs_dir, "front-beds")

    def test_delete_missing_job_raises(self):
        with self.assertRaises(FileNotFoundError):
            jobs.delete_job(self.jobs_dir, "does-not-exist")

    def test_list_jobs_empty_directory(self):
        empty_dir = self.jobs_dir / "does-not-exist-yet"
        self.assertEqual(jobs.list_jobs(empty_dir), [])

    def test_list_jobs_metadata(self):
        jobs.save_job(self.jobs_dir, "b-job", SAMPLE_STEPS)
        jobs.save_job(self.jobs_dir, "a-job", SAMPLE_STEPS[:1])
        rows = jobs.list_jobs(self.jobs_dir)
        self.assertEqual([r["name"] for r in rows], ["a-job", "b-job"])  # sorted
        self.assertEqual(rows[0]["step_count"], 1)
        self.assertEqual(rows[1]["step_count"], 2)

    def test_invalid_names_rejected(self):
        for bad_name in ["../escape", "a/b", ".hidden", "", "   "]:
            with self.assertRaises(jobs.InvalidJobName):
                jobs.save_job(self.jobs_dir, bad_name, SAMPLE_STEPS)
            with self.assertRaises(jobs.InvalidJobName):
                jobs.load_job(self.jobs_dir, bad_name)
            with self.assertRaises(jobs.InvalidJobName):
                jobs.delete_job(self.jobs_dir, bad_name)

    def test_spaces_and_punctuation_are_fine(self):
        for name in ["Front beds", "Beds #2 (north)"]:
            jobs.save_job(self.jobs_dir, name, SAMPLE_STEPS)
            self.assertEqual(jobs.load_job(self.jobs_dir, name)["steps"], SAMPLE_STEPS)
            jobs.delete_job(self.jobs_dir, name)

    def test_leading_and_trailing_whitespace_is_trimmed(self):
        jobs.save_job(self.jobs_dir, "  Front beds  ", SAMPLE_STEPS)
        self.assertEqual(jobs.load_job(self.jobs_dir, "Front beds")["steps"], SAMPLE_STEPS)


if __name__ == "__main__":
    unittest.main()
