"""Tests for viewer's auto-discovery and generic log API — no real
service directories are touched, `app.REPO_ROOT` is redirected to a
throwaway tree of fake `<service>/data/logs/*.jsonl` files (same
"redirect the module-level dir constant" pattern as waterbutt's own
test_app.py)."""

import json
import shutil
import tempfile
import unittest
from pathlib import Path

import app


class ViewerAppTestCase(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        app.REPO_ROOT = self.tmp
        self.client = app.app.test_client()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _write_log(self, service, filename, lines):
        logs_dir = self.tmp / service / "data" / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)
        (logs_dir / filename).write_text("\n".join(json.dumps(line) for line in lines) + "\n")

    def test_discover_sources_finds_only_dirs_with_data_logs(self):
        self._write_log("navigate", "run.jsonl", [{"t": 1.0}])
        (self.tmp / "shared").mkdir()  # excluded even if it somehow had data/logs
        (self.tmp / "camera").mkdir()  # a real service dir, but no data/logs - not a source
        sources = app.discover_sources()
        self.assertEqual(set(sources.keys()), {"navigate"})

    def test_excluded_dirs_never_become_sources(self):
        for name in ("shared", "venv", "viewer"):
            self._write_log(name, "x.jsonl", [{"t": 1.0}])
        self.assertEqual(app.discover_sources(), {})

    def test_api_logs_lists_every_discovered_source(self):
        self._write_log("navigate", "260830_1.jsonl", [{"t": 10.0, "lat": 1.0}, {"t": 20.0, "lat": 1.1}])
        self._write_log("drive", "260830_2.jsonl", [{"t": 15.0}])
        resp = self.client.get("/api/logs")
        data = resp.get_json()
        self.assertEqual(set(data.keys()), {"navigate", "drive"})
        self.assertEqual(data["navigate"][0]["line_count"], 2)
        self.assertEqual(data["navigate"][0]["start_t"], 10.0)
        self.assertEqual(data["navigate"][0]["end_t"], 20.0)

    def test_api_log_file_serves_raw_contents(self):
        self._write_log("navigate", "260830_1.jsonl", [{"t": 10.0}])
        resp = self.client.get("/api/logs/navigate/260830_1.jsonl")
        self.assertEqual(resp.status_code, 200)
        self.assertIn(b'"t": 10.0', resp.data)

    def test_api_log_file_404s_for_unknown_source_or_file(self):
        self._write_log("navigate", "260830_1.jsonl", [{"t": 10.0}])
        self.assertEqual(self.client.get("/api/logs/no-such-service/x.jsonl").status_code, 404)
        self.assertEqual(self.client.get("/api/logs/navigate/no-such-file.jsonl").status_code, 404)

    def test_api_log_file_blocks_path_traversal(self):
        self._write_log("navigate", "260830_1.jsonl", [{"t": 10.0}])
        resp = self.client.get("/api/logs/navigate/..%2F..%2Fapp.py")
        self.assertEqual(resp.status_code, 404)

    def test_home_page_renders(self):
        resp = self.client.get("/")
        self.assertEqual(resp.status_code, 200)


if __name__ == "__main__":
    unittest.main()
