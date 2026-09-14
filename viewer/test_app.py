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
from unittest.mock import patch

import requests

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


class ScaleFactorMapPseudoFileTestCase(unittest.TestCase):
    """wheelspeed's scale-factor map (see app.py's own comment) shows up
    as a pseudo-file "map" inside wheelspeed's own folder, proxied live
    from wheelspeed rather than read off disk."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        app.REPO_ROOT = self.tmp
        self.client = app.app.test_client()
        (self.tmp / "wheelspeed" / "data" / "logs").mkdir(parents=True)
        (self.tmp / "wheelspeed" / "data" / "logs" / "260914_1.jsonl").write_text('{"t": 1.0}\n')

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_api_logs_lists_the_map_pseudo_entry_first(self):
        with patch.object(app, "_fetch_scale_factor_cells", return_value=[{"lat": 1.0, "lon": 2.0, "mean": 1.1}] * 3):
            data = self.client.get("/api/logs").get_json()
        entry = data["wheelspeed"][0]
        self.assertEqual(entry["filename"], "map")
        self.assertTrue(entry["is_map"])
        self.assertEqual(entry["line_count"], 3)
        self.assertIsNone(entry["start_t"])

    def test_api_logs_lists_the_map_entry_with_zero_count_when_wheelspeed_unreachable(self):
        with patch.object(app, "_fetch_scale_factor_cells", side_effect=requests.exceptions.ConnectionError()):
            data = self.client.get("/api/logs").get_json()
        entry = data["wheelspeed"][0]
        self.assertEqual(entry["filename"], "map")
        self.assertEqual(entry["line_count"], 0)

    def test_api_log_file_proxies_the_map_as_jsonl(self):
        cells = [{"lat": 1.0, "lon": 2.0, "mean": 1.1}, {"lat": 3.0, "lon": 4.0, "mean": 0.9}]
        with patch.object(app, "_fetch_scale_factor_cells", return_value=cells):
            resp = self.client.get("/api/logs/wheelspeed/map")
        self.assertEqual(resp.status_code, 200)
        lines = [json.loads(l) for l in resp.get_data(as_text=True).splitlines()]
        self.assertEqual(lines, cells)

    def test_api_log_file_502s_for_the_map_when_wheelspeed_unreachable(self):
        with patch.object(app, "_fetch_scale_factor_cells", side_effect=requests.exceptions.ConnectionError()):
            resp = self.client.get("/api/logs/wheelspeed/map")
        self.assertEqual(resp.status_code, 502)


if __name__ == "__main__":
    unittest.main()
