import json
import tempfile
import unittest
from pathlib import Path

from grid import LogOddsParams
from reprocess import build_grid_from_raw_logs, list_raw_logs, write_snapshot

SENSORS = [{"tag": 0, "x": 0.0, "y": 0.0, "z": 0.0, "heading_deg": 0.0}]


def write_jsonl(path, records):
    with open(path, "w") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")


class TestListRawLogs(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def test_empty_directory_returns_empty_list(self):
        self.assertEqual(list_raw_logs(self.tmp / "does-not-exist"), [])

    def test_reports_time_range_and_line_count(self):
        write_jsonl(self.tmp / "a.jsonl", [
            {"t": 100.0, "north": 0, "east": 0, "heading_deg": 0, "tag": 0, "range_mm": 500},
            {"t": 110.0, "north": 0, "east": 0, "heading_deg": 0, "tag": 0, "range_mm": None},
        ])
        entries = list_raw_logs(self.tmp)
        self.assertEqual(len(entries), 1)
        self.assertEqual(entries[0]["filename"], "a.jsonl")
        self.assertEqual(entries[0]["start_t"], 100.0)
        self.assertEqual(entries[0]["end_t"], 110.0)
        self.assertEqual(entries[0]["line_count"], 2)

    def test_ignores_blank_and_malformed_lines(self):
        path = self.tmp / "a.jsonl"
        path.write_text('{"t": 1.0, "north": 0, "east": 0, "heading_deg": 0, "tag": 0, "range_mm": 500}\n\nnot json\n')
        entries = list_raw_logs(self.tmp)
        self.assertEqual(entries[0]["line_count"], 1)


class TestBuildGridFromRawLogs(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.params = LogOddsParams(p_hit=0.7, p_miss=0.3, p_min=0.02, p_max=0.98)

    def test_replays_a_hit_at_the_measured_range(self):
        write_jsonl(self.tmp / "a.jsonl", [
            {"t": 1.0, "north": 0.0, "east": 0.0, "heading_deg": 0.0, "tag": 0, "range_mm": 500},
        ])
        grid = build_grid_from_raw_logs(self.tmp, ["a.jsonl"], SENSORS, cell_size_m=0.1, max_range_m=1.2,
                                         beam_half_angle_deg=7.5, forget_after_s=1e9, params=self.params)
        self.assertAlmostEqual(grid.cell_value(0.5, 0.0)["p_occupied"], 0.7)

    def test_null_range_mm_replays_as_a_clear_wedge(self):
        write_jsonl(self.tmp / "a.jsonl", [
            {"t": 1.0, "north": 0.0, "east": 0.0, "heading_deg": 0.0, "tag": 0, "range_mm": None},
        ])
        grid = build_grid_from_raw_logs(self.tmp, ["a.jsonl"], SENSORS, cell_size_m=0.1, max_range_m=1.2,
                                         beam_half_angle_deg=7.5, forget_after_s=1e9, params=self.params)
        self.assertLess(grid.cell_value(1.0, 0.0)["p_occupied"], 0.5)

    def test_different_max_range_reinterprets_a_stored_raw_reading(self):
        write_jsonl(self.tmp / "a.jsonl", [
            {"t": 1.0, "north": 0.0, "east": 0.0, "heading_deg": 0.0, "tag": 0, "range_mm": 1000},
        ])
        narrow = build_grid_from_raw_logs(self.tmp, ["a.jsonl"], SENSORS, cell_size_m=0.1, max_range_m=0.8,
                                           beam_half_angle_deg=7.5, forget_after_s=1e9, params=self.params)
        self.assertEqual(narrow.cell_value(1.0, 0.0)["source"], "unknown")  # 1000mm treated as "no detection" at 0.8m max range

        wide = build_grid_from_raw_logs(self.tmp, ["a.jsonl"], SENSORS, cell_size_m=0.1, max_range_m=1.5,
                                         beam_half_angle_deg=7.5, forget_after_s=1e9, params=self.params)
        self.assertAlmostEqual(wide.cell_value(1.0, 0.0)["p_occupied"], 0.7)  # same reading now a real hit at 1.5m max range

    def test_multiple_files_all_get_replayed(self):
        write_jsonl(self.tmp / "a.jsonl", [{"t": 1.0, "north": 0.0, "east": 0.0, "heading_deg": 0.0, "tag": 0, "range_mm": 500}])
        write_jsonl(self.tmp / "b.jsonl", [{"t": 2.0, "north": 0.0, "east": 0.0, "heading_deg": 0.0, "tag": 0, "range_mm": 500}])
        grid = build_grid_from_raw_logs(self.tmp, ["a.jsonl", "b.jsonl"], SENSORS, cell_size_m=0.1, max_range_m=1.2,
                                         beam_half_angle_deg=7.5, forget_after_s=1e9, params=self.params)
        # Two hits should have moved the belief further than one alone would.
        single = build_grid_from_raw_logs(self.tmp, ["a.jsonl"], SENSORS, cell_size_m=0.1, max_range_m=1.2,
                                           beam_half_angle_deg=7.5, forget_after_s=1e9, params=self.params)
        self.assertGreater(grid.cell_value(0.5, 0.0)["p_occupied"], single.cell_value(0.5, 0.0)["p_occupied"])

    def test_unknown_sensor_tag_is_skipped_not_an_error(self):
        write_jsonl(self.tmp / "a.jsonl", [
            {"t": 1.0, "north": 0.0, "east": 0.0, "heading_deg": 0.0, "tag": 99, "range_mm": 500},
        ])
        grid = build_grid_from_raw_logs(self.tmp, ["a.jsonl"], SENSORS, cell_size_m=0.1, max_range_m=1.2,
                                         beam_half_angle_deg=7.5, forget_after_s=1e9, params=self.params)
        self.assertEqual(grid.stats()["cells"], 0)


class TestWriteSnapshot(unittest.TestCase):
    def test_writes_a_paired_grid_and_settings_file(self):
        tmp = Path(tempfile.mkdtemp())
        params = LogOddsParams(0.7, 0.3, 0.02, 0.98)
        from grid import MapGrid
        grid = MapGrid(cell_size_m=0.1, max_range_m=1.2, beam_half_angle_deg=7.5, forget_after_s=1e9, params=params)
        grid.record_detection(0.0, 0.0, 0.0, range_m=0.5)

        result = write_snapshot(tmp, grid, ["a.jsonl"], SENSORS)

        grid_path = tmp / result["grid_filename"]
        settings_path = tmp / result["settings_filename"]
        self.assertTrue(grid_path.exists())
        self.assertTrue(settings_path.exists())
        self.assertEqual(json.loads(settings_path.read_text())["input_files"], ["a.jsonl"])
        self.assertEqual(result["stats"]["cells"], grid.stats()["cells"])

    def test_a_second_write_in_the_same_second_does_not_clobber_the_first(self):
        tmp = Path(tempfile.mkdtemp())
        params = LogOddsParams(0.7, 0.3, 0.02, 0.98)
        from grid import MapGrid
        grid = MapGrid(cell_size_m=0.1, max_range_m=1.2, beam_half_angle_deg=7.5, forget_after_s=1e9, params=params)

        first = write_snapshot(tmp, grid, [], SENSORS)
        second = write_snapshot(tmp, grid, [], SENSORS)

        self.assertNotEqual(first["grid_filename"], second["grid_filename"])
        self.assertTrue((tmp / first["grid_filename"]).exists())
        self.assertTrue((tmp / second["grid_filename"]).exists())


if __name__ == "__main__":
    unittest.main()
