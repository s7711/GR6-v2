import json
import os
import shutil
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from rotating_jsonl_log import RotatingJsonlLog


class TestRotatingJsonlLog(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.dir = self.tmp / "logs"

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_append_creates_directory_and_writes_a_line(self):
        log = RotatingJsonlLog(self.dir, rotate_s=3600, retention_days=2)
        log.start()
        log.append({"x": 1})
        files = list(self.dir.glob("*.jsonl"))
        self.assertEqual(len(files), 1)
        entry = json.loads(files[0].read_text().splitlines()[0])
        self.assertEqual(entry["x"], 1)
        self.assertIn("t", entry)

    def test_second_append_reuses_the_same_file_within_rotate_period(self):
        log = RotatingJsonlLog(self.dir, rotate_s=3600, retention_days=2)
        log.start()
        log.append({"x": 1})
        log.append({"x": 2})
        files = list(self.dir.glob("*.jsonl"))
        self.assertEqual(len(files), 1)
        self.assertEqual(len(files[0].read_text().splitlines()), 2)

    def test_rotates_to_a_new_file_once_the_next_boundary_passes(self):
        # time.strftime (the filename source) is patched alongside
        # time.time here - unlike production, where both always agree on
        # "now", left unpatched it would use the real wall clock and
        # both writes would collide on the same real-world filename.
        log = RotatingJsonlLog(self.dir, rotate_s=3600, retention_days=2)
        log.start()
        with patch("time.time", return_value=1000.0), patch("time.strftime", return_value="file_a"):
            log.append({"x": 1})
        with patch("time.time", return_value=5000.0), patch("time.strftime", return_value="file_b"):
            log.append({"x": 2})  # well past the next 3600 boundary
        files = list(self.dir.glob("*.jsonl"))
        self.assertEqual(len(files), 2)

    def test_rotation_aligns_to_a_wall_clock_hour_not_the_first_write_time(self):
        # First write at 12:47:00 - the next rotation must land on 13:00:00
        # exactly (a multiple of rotate_s since the epoch), not 13:47:00.
        log = RotatingJsonlLog(self.dir, rotate_s=3600, retention_days=2)
        log.start()
        first_write = 12 * 3600 + 47 * 60  # 12:47:00 on the epoch's own day
        log.append({"x": 1})
        with patch("time.time", return_value=float(first_write)):
            log._path = None  # force a fresh boundary computation from this "now"
            log.append({"x": 1})
        self.assertEqual(log._next_rotate_at, 13 * 3600)

    def test_sweep_deletes_only_files_past_retention(self):
        log = RotatingJsonlLog(self.dir, rotate_s=3600, retention_days=2)
        log.start()
        old_file = self.dir / "old.jsonl"
        new_file = self.dir / "new.jsonl"
        old_file.write_text("{}\n")
        new_file.write_text("{}\n")
        old_time = time.time() - 3 * 86400
        os.utime(old_file, (old_time, old_time))

        log._sweep()

        self.assertFalse(old_file.exists())
        self.assertTrue(new_file.exists())

    def test_a_write_error_is_logged_not_raised(self):
        log = RotatingJsonlLog(self.dir, rotate_s=3600, retention_days=2)
        log.start()
        with patch("builtins.open", side_effect=OSError("disk full")):
            log.append({"x": 1})  # must not raise


if __name__ == "__main__":
    unittest.main()
