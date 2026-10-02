import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

from file_cache import FileCache, load_yaml


class TestFileCache(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.path = self.tmp / "a.yaml"
        self.path.write_text("- 1\n- 2\n")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_unchanged_file_is_computed_once(self):
        cache = FileCache()
        compute = Mock(return_value="v")
        self.assertEqual(cache.get(self.path, compute), "v")
        self.assertEqual(cache.get(self.path, compute), "v")
        compute.assert_called_once()

    def test_changed_size_recomputes(self):
        cache = FileCache()
        self.assertEqual(cache.get(self.path, load_yaml), [1, 2])
        self.path.write_text("- 1\n- 2\n- 3\n")
        self.assertEqual(cache.get(self.path, load_yaml), [1, 2, 3])

    def test_changed_mtime_same_size_recomputes(self):
        cache = FileCache()
        self.assertEqual(cache.get(self.path, load_yaml), [1, 2])
        self.path.write_text("- 7\n- 8\n")
        st = self.path.stat()
        os.utime(self.path, ns=(st.st_atime_ns, st.st_mtime_ns + 1_000_000_000))
        self.assertEqual(cache.get(self.path, load_yaml), [7, 8])

    def test_missing_file_raises_and_caches_nothing(self):
        cache = FileCache()
        with self.assertRaises(FileNotFoundError):
            cache.get(self.tmp / "nope.yaml", load_yaml)
        self.assertNotIn(self.tmp / "nope.yaml", cache)

    def test_failed_compute_caches_nothing(self):
        cache = FileCache()
        with self.assertRaises(ValueError):
            cache.get(self.path, Mock(side_effect=ValueError))
        self.assertNotIn(self.path, cache)

    def test_load_yaml_matches_safe_load(self):
        import yaml
        self.path.write_text("name: x\nsteps:\n  - {type: run_path, path: 'A b'}\n  - 1.5\n")
        self.assertEqual(load_yaml(self.path), yaml.safe_load(self.path.read_text()))


if __name__ == "__main__":
    unittest.main()
