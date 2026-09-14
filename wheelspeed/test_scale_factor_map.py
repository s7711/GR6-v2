import unittest

from scale_factor_map import ScaleFactorMap


class TestScaleFactorMap(unittest.TestCase):
    def test_unvisited_cell_returns_none(self):
        m = ScaleFactorMap(cell_size_m=1.0, max_n=30)
        self.assertIsNone(m.cell(5.0, 5.0))

    def test_first_sample_sets_the_mean_with_no_stdev_yet(self):
        m = ScaleFactorMap(cell_size_m=1.0, max_n=30)
        m.update(5.0, 5.0, 1.2, t=100.0)
        cell = m.cell(5.0, 5.0)
        self.assertEqual(cell["n"], 1)
        self.assertEqual(cell["mean"], 1.2)
        self.assertIsNone(cell["stdev"])
        self.assertEqual(cell["t"], 100.0)

    def test_stdev_matches_population_stdev(self):
        m = ScaleFactorMap(cell_size_m=1.0, max_n=30)
        samples = [1.0, 1.2, 1.4, 0.9]
        for x in samples:
            m.update(5.0, 5.0, x, t=100.0)
        mean = sum(samples) / len(samples)
        expected_stdev = (sum((x - mean) ** 2 for x in samples) / len(samples)) ** 0.5
        cell = m.cell(5.0, 5.0)
        self.assertAlmostEqual(cell["mean"], mean)
        self.assertAlmostEqual(cell["stdev"], expected_stdev)

    def test_different_locations_land_in_different_cells(self):
        m = ScaleFactorMap(cell_size_m=1.0, max_n=30)
        m.update(0.0, 0.0, 1.0, t=100.0)
        m.update(10.0, 0.0, 1.5, t=100.0)
        self.assertEqual(m.cell(0.0, 0.0)["mean"], 1.0)
        self.assertEqual(m.cell(10.0, 0.0)["mean"], 1.5)

    def test_nearby_locations_within_half_a_cell_share_a_cell(self):
        m = ScaleFactorMap(cell_size_m=1.0, max_n=30)
        m.update(0.2, -0.3, 1.0, t=100.0)
        m.update(-0.1, 0.4, 2.0, t=100.0)
        cell = m.cell(0.0, 0.0)
        self.assertEqual(cell["n"], 2)

    def test_n_never_exceeds_max_n_so_old_samples_eventually_lose_influence(self):
        # Once n is capped, a long run of identical new samples should be
        # able to pull the mean all the way to that new value - it
        # couldn't if every sample counted forever (see module docstring).
        m = ScaleFactorMap(cell_size_m=1.0, max_n=5)
        for _ in range(5):
            m.update(0.0, 0.0, 1.0, t=100.0)
        for _ in range(200):
            m.update(0.0, 0.0, 2.0, t=100.0)
        cell = m.cell(0.0, 0.0)
        self.assertEqual(cell["n"], 5)
        self.assertAlmostEqual(cell["mean"], 2.0)

    def test_dirty_flag_set_on_update_and_cleared_on_load(self):
        m = ScaleFactorMap(cell_size_m=1.0, max_n=30)
        self.assertFalse(m.dirty)
        m.update(0.0, 0.0, 1.0, t=100.0)
        self.assertTrue(m.dirty)

    def test_state_round_trips(self):
        m = ScaleFactorMap(cell_size_m=1.0, max_n=30)
        m.update(0.0, 0.0, 1.0, t=100.0)
        m.update(10.0, -3.0, 1.5, t=101.0)
        state = m.dump_state()

        reloaded = ScaleFactorMap(cell_size_m=1.0, max_n=30)
        reloaded.load_state(state)

        self.assertEqual(reloaded.cell(0.0, 0.0), m.cell(0.0, 0.0))
        self.assertEqual(reloaded.cell(10.0, -3.0), m.cell(10.0, -3.0))
        self.assertFalse(reloaded.dirty)

    def test_all_cells_reports_centres_and_stats(self):
        m = ScaleFactorMap(cell_size_m=2.0, max_n=30)
        m.update(5.0, -3.0, 1.2, t=100.0)  # -> cell (2, -2) 4m,-4m ... 6m,-2m grid depending on rounding
        cells = m.all_cells()
        self.assertEqual(len(cells), 1)
        c = cells[0]
        self.assertEqual(c["n"], 1)
        self.assertEqual(c["mean"], 1.2)
        self.assertIsNone(c["stdev"])
        self.assertEqual(c["t"], 100.0)
        # Cell centre should be within half a cell of the sample itself.
        self.assertLessEqual(abs(c["north"] - 5.0), 1.0)
        self.assertLessEqual(abs(c["east"] - -3.0), 1.0)

    def test_stats_reports_cell_count(self):
        m = ScaleFactorMap(cell_size_m=1.0, max_n=30)
        m.update(0.0, 0.0, 1.0, t=100.0)
        m.update(10.0, 0.0, 1.0, t=100.0)
        self.assertEqual(m.stats(), {"cells": 2})


if __name__ == "__main__":
    unittest.main()
