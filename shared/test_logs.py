import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import logs


class TestLogFileSummaryCaching(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        logs._cache = {}

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _write(self, name, records):
        path = self.tmp / name
        content = "".join(json.dumps(r) + "\n" for r in records)
        path.write_text(content)
        return path

    def test_first_call_reads_the_file(self):
        path = self._write("a.jsonl", [{"t": 1.0}, {"t": 2.0}])
        summary = logs.log_file_summary(path)
        self.assertEqual(summary, {"filename": "a.jsonl", "start_t": 1.0, "end_t": 2.0, "line_count": 2, "path_name": None})

    def test_unchanged_file_is_served_from_cache_not_reread(self):
        path = self._write("a.jsonl", [{"t": 1.0}, {"t": 2.0}])
        logs.log_file_summary(path)
        with patch.object(Path, "read_text", side_effect=AssertionError("should not re-read an unchanged file")):
            summary = logs.log_file_summary(path)
        self.assertEqual(summary["line_count"], 2)

    def test_appended_file_is_rescanned(self):
        path = self._write("a.jsonl", [{"t": 1.0}])
        first = logs.log_file_summary(path)
        self.assertEqual(first["line_count"], 1)
        with open(path, "a") as f:
            f.write(json.dumps({"t": 2.0}) + "\n")
        second = logs.log_file_summary(path)
        self.assertEqual(second["line_count"], 2)
        self.assertEqual(second["end_t"], 2.0)

    def test_rewritten_file_of_the_same_size_is_still_rescanned(self):
        # Content can change without the line/byte count changing (e.g.
        # a torn write settling to its final content) - mtime alone must
        # still catch this even when size doesn't. Bump mtime explicitly
        # rather than relying on real wall-clock time passing between
        # the two writes, so this test doesn't depend on filesystem
        # timestamp resolution.
        path = self._write("a.jsonl", [{"t": 1.0}])
        logs.log_file_summary(path)
        path.write_text(json.dumps({"t": 9.0}) + "\n")
        st = path.stat()
        os.utime(path, ns=(st.st_atime_ns, st.st_mtime_ns + 1))
        summary = logs.log_file_summary(path)
        self.assertEqual(summary["start_t"], 9.0)

    def test_log_summaries_uses_the_cache_across_calls(self):
        self._write("a.jsonl", [{"t": 1.0}])
        self._write("b.jsonl", [{"t": 2.0}])
        first = logs.log_summaries(self.tmp)
        self.assertEqual(len(first), 2)
        with patch.object(Path, "read_text", side_effect=AssertionError("should not re-read unchanged files")):
            second = logs.log_summaries(self.tmp)
        self.assertEqual(second, first)

    def test_log_summaries_skips_a_corrupt_file_without_caching_it(self):
        good = self._write("a.jsonl", [{"t": 1.0}])
        bad = self.tmp / "b.jsonl"
        bad.write_text("not json\n")
        with self.assertLogs(level="WARNING"):
            result = logs.log_summaries(self.tmp)
        self.assertEqual([s["filename"] for s in result], ["a.jsonl"])
        self.assertNotIn(str(bad), logs._cache)
        self.assertIn(str(good), logs._cache)

    def test_log_summaries_tolerates_a_file_disappearing_mid_scan(self):
        self._write("a.jsonl", [{"t": 1.0}])
        gone = self._write("b.jsonl", [{"t": 2.0}])
        real_stat = Path.stat

        def flaky_stat(self, *args, **kwargs):
            if self == gone:
                raise FileNotFoundError()
            return real_stat(self, *args, **kwargs)

        with patch.object(Path, "stat", flaky_stat):
            result = logs.log_summaries(self.tmp)
        self.assertEqual([s["filename"] for s in result], ["a.jsonl"])

    def test_empty_file_summary(self):
        path = self._write("a.jsonl", [])
        summary = logs.log_file_summary(path)
        self.assertEqual(summary, {"filename": "a.jsonl", "start_t": None, "end_t": None, "line_count": 0, "path_name": None})

    def test_no_logs_dir_returns_empty_list(self):
        self.assertEqual(logs.log_summaries(self.tmp / "missing"), [])


if __name__ == "__main__":
    unittest.main()
