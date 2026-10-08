import tempfile
import unittest
from pathlib import Path

import xnav_config_history as h


class TestSnapshots(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name) / "history"

    def tearDown(self):
        self._tmp.cleanup()

    def test_save_list_load_delete(self):
        h.save_snapshot(self.dir, "261008_120000", {"mobile.cfg": b"a", "comment.txt": b"x\n261008 - first\n\n"})
        h.save_snapshot(self.dir, "261009_120000", {"mobile.cfg": b"b"})
        self.assertEqual(
            h.list_snapshots(self.dir),
            [
                {"name": "261009_120000", "files": ["mobile.cfg"], "comment": ""},
                {"name": "261008_120000", "files": ["comment.txt", "mobile.cfg"], "comment": "261008 - first"},
            ],
        )
        self.assertEqual(h.load_snapshot(self.dir, "261009_120000"), {"mobile.cfg": b"b"})
        self.assertEqual(h.read_file(self.dir, "261008_120000", "mobile.cfg"), b"a")
        h.delete_snapshot(self.dir, "261009_120000")
        self.assertEqual([s["name"] for s in h.list_snapshots(self.dir)], ["261008_120000"])

    def test_never_overwrites(self):
        h.save_snapshot(self.dir, "one", {"mobile.cfg": b"a"})
        with self.assertRaises(FileExistsError):
            h.save_snapshot(self.dir, "one", {"mobile.cfg": b"b"})
        self.assertEqual(h.load_snapshot(self.dir, "one"), {"mobile.cfg": b"a"})

    def test_rejects_unsafe_names(self):
        for bad in ["", "../x", "a/b", ".hidden"]:
            with self.assertRaises(h.InvalidName):
                h.save_snapshot(self.dir, bad, {})
        with self.assertRaises(h.InvalidName):
            h.read_file(self.dir, "x", "../../etc/passwd")

    def test_failed_save_leaves_nothing(self):
        with self.assertRaises(h.InvalidName):
            h.save_snapshot(self.dir, "partial", {"mobile.cfg": b"a", "../evil": b"b"})
        self.assertEqual(h.list_snapshots(self.dir), [])
        h.save_snapshot(self.dir, "partial", {"mobile.cfg": b"a"})  # a retry isn't blocked by leftovers


class TestCompare(unittest.TestCase):
    def test_statuses_and_diff(self):
        snap = {"mobile.cfg": b"a\nb\n", "mobile.gap": b"1", "mobile.nsp": b"x"}
        now = {"mobile.cfg": b"a\nc\n", "mobile.gap": b"1", "mobile.dbu": b"y"}
        result = {f["file"]: f for f in h.compare("old", snap, now)}
        self.assertEqual(
            {f: r["status"] for f, r in result.items()},
            {"mobile.cfg": "changed", "mobile.gap": "same", "mobile.nsp": "snapshot_only", "mobile.dbu": "xnav_only"},
        )
        self.assertIn("-c", result["mobile.cfg"]["diff"])
        self.assertIn("+b", result["mobile.cfg"]["diff"])

    def test_line_endings_only(self):
        result = h.compare("old", {"mobile.gap": b"1\r\n2\r\n"}, {"mobile.gap": b"1\n2\n"})
        self.assertEqual(result[0]["status"], "changed")
        self.assertIn("line endings", result[0]["diff"])

    def test_long_lines_not_diffed(self):
        result = h.compare("old", {"mobile.dbu": b"a" * 5000}, {"mobile.dbu": b"b" * 5000})
        self.assertLess(len(result[0]["diff"]), 200)


class TestPlanUpload(unittest.TestCase):
    def test_deletes_only_extra_mobile_files(self):
        plan = h.plan_upload({"mobile.cfg": b"", "mobile.nsp": b""}, {"mobile.cfg": b"", "mobile.dbu": b"", "comment.txt": b""})
        self.assertEqual(plan, {"upload": ["mobile.cfg", "mobile.nsp"], "delete": ["mobile.dbu"]})


if __name__ == "__main__":
    unittest.main()
