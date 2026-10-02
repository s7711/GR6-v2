import shutil
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from queued_writer import QueuedWriter


class TestQueuedWriter(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_before_start_writes_synchronously(self):
        writer = QueuedWriter("test")
        path = self.tmp / "a.jsonl"
        writer.put(path, "one\n")
        self.assertEqual(path.read_text(), "one\n")

    def test_creates_the_directory_on_first_line(self):
        writer = QueuedWriter("test")
        path = self.tmp / "logs" / "deeper" / "a.jsonl"
        writer.put(path, "one\n")
        self.assertEqual(path.read_text(), "one\n")

    def test_empty_line_still_creates_the_file(self):
        writer = QueuedWriter("test")
        path = self.tmp / "a.jsonl"
        writer.put(path, "")
        self.assertTrue(path.exists())

    def test_after_start_put_never_touches_the_disk_itself(self):
        writer = QueuedWriter("test")
        writer._started = True  # as start() would, without the thread
        path = self.tmp / "a.jsonl"
        with patch("builtins.open", side_effect=AssertionError("put() must not open files")):
            writer.put(path, "one\n")
            writer.put(path, "two\n")
        self.assertFalse(path.exists())
        writer.drain()
        self.assertEqual(path.read_text(), "one\ntwo\n")

    def test_a_slow_disk_does_not_block_put(self):
        writer = QueuedWriter("test")
        release = threading.Event()
        real_write = writer._write

        def slow_write(path, line):
            release.wait(5)
            real_write(path, line)

        writer._write = slow_write
        writer.start()
        path = self.tmp / "a.jsonl"
        done = threading.Event()
        threading.Thread(target=lambda: (writer.put(path, "one\n"), writer.put(path, "two\n"), done.set())).start()
        self.assertTrue(done.wait(1), "put() blocked behind a stalled write")
        release.set()
        for _ in range(100):
            if path.exists() and path.read_text() == "one\ntwo\n":
                break
            threading.Event().wait(0.02)
        self.assertEqual(path.read_text(), "one\ntwo\n")

    def test_write_failure_is_logged_not_raised(self):
        writer = QueuedWriter("test")
        (self.tmp / "a_dir").mkdir()
        with self.assertLogs(level="WARNING"):
            writer.put(self.tmp / "a_dir", "one\n")  # a directory can't be opened for append


if __name__ == "__main__":
    unittest.main()
