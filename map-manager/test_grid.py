import math
import unittest

from grid import LogOddsParams, MapGrid, effective_range_m, logit, sensor_world_pose, sigmoid


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_params(p_hit=0.7, p_miss=0.3, p_min=0.02, p_max=0.98):
    return LogOddsParams(p_hit, p_miss, p_min, p_max)


def make_grid(clock=None, params=None, **overrides):
    kwargs = dict(cell_size_m=0.1, max_range_m=1.2, beam_half_angle_deg=7.5, forget_after_s=2592000)
    kwargs.update(overrides)
    return MapGrid(now=clock or FakeClock(), params=params or make_params(), **kwargs)


class TestLogitSigmoid(unittest.TestCase):
    def test_round_trips(self):
        for p in (0.02, 0.3, 0.5, 0.7, 0.98):
            self.assertAlmostEqual(sigmoid(logit(p)), p)

    def test_logit_of_half_is_zero(self):
        self.assertAlmostEqual(logit(0.5), 0.0)


class TestEffectiveRangeM(unittest.TestCase):
    def test_none_reading_stays_none(self):
        self.assertIsNone(effective_range_m(None, max_range_m=1.2))

    def test_within_range_converts_mm_to_m(self):
        self.assertAlmostEqual(effective_range_m(500, max_range_m=1.2), 0.5)

    def test_at_or_beyond_max_range_becomes_none(self):
        self.assertIsNone(effective_range_m(1200, max_range_m=1.2))
        self.assertIsNone(effective_range_m(5000, max_range_m=1.2))

    def test_different_max_range_reinterprets_the_same_raw_reading(self):
        # The whole point of storing raw mm - the same 1000mm reading is
        # "in range" against a larger max_range_m than it was captured
        # with.
        self.assertIsNone(effective_range_m(1000, max_range_m=0.8))
        self.assertAlmostEqual(effective_range_m(1000, max_range_m=1.5), 1.0)


class TestSensorWorldPose(unittest.TestCase):
    def test_facing_north_forward_offset_adds_to_north(self):
        n, e, heading = sensor_world_pose(0.0, 0.0, 0.0, 0.2, 0.0, 0.0)
        self.assertAlmostEqual(n, 0.2)
        self.assertAlmostEqual(e, 0.0)
        self.assertAlmostEqual(heading, 0.0)

    def test_facing_east_forward_offset_adds_to_east(self):
        n, e, heading = sensor_world_pose(0.0, 0.0, 90.0, 0.2, 0.0, 0.0)
        self.assertAlmostEqual(n, 0.0, places=6)
        self.assertAlmostEqual(e, 0.2)
        self.assertAlmostEqual(heading, 90.0)

    def test_sensor_heading_offset_combines_with_robot_heading(self):
        _, _, heading = sensor_world_pose(0.0, 0.0, 30.0, 0.0, 0.0, 90.0)
        self.assertAlmostEqual(heading, 120.0)

    def test_right_offset_while_facing_north_adds_to_east(self):
        n, e, _ = sensor_world_pose(0.0, 0.0, 0.0, 0.0, 0.2, 0.0)
        self.assertAlmostEqual(n, 0.0, places=6)
        self.assertAlmostEqual(e, 0.2)


class TestRecordDetection(unittest.TestCase):
    def test_a_single_hit_moves_probability_by_exactly_p_hit(self):
        g = make_grid(params=make_params(p_hit=0.7, p_miss=0.3, p_min=0.02, p_max=0.98))
        g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        value = g.cell_value(0.5, 0.0)
        self.assertEqual(value["source"], "sensor")
        self.assertAlmostEqual(value["p_occupied"], 0.7)

    def test_a_single_miss_moves_probability_by_exactly_p_miss(self):
        g = make_grid(params=make_params(p_hit=0.7, p_miss=0.3, p_min=0.02, p_max=0.98))
        g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        near = g.cell_value(0.1, 0.0)
        self.assertEqual(near["source"], "sensor")
        self.assertAlmostEqual(near["p_occupied"], 0.3)

    def test_repeated_hits_saturate_at_p_max_not_beyond(self):
        g = make_grid(params=make_params(p_hit=0.9, p_miss=0.1, p_min=0.02, p_max=0.9))
        for _ in range(50):
            g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        value = g.cell_value(0.5, 0.0)
        self.assertAlmostEqual(value["p_occupied"], 0.9)

    def test_a_clamped_belief_responds_quickly_to_new_contradicting_evidence(self):
        # This is the actual point of clamping - a cell saturated as
        # "very clear" from many misses should still flip to "occupied"
        # within a handful of real hits, not need thousands of them.
        params = make_params(p_hit=0.7, p_miss=0.3, p_min=0.02, p_max=0.98)
        g = make_grid(params=params)
        for _ in range(1000):
            g.record_detection(0.0, 0.0, 0.0, range_m=None)  # saturates at p_min
        self.assertLess(g.cell_value(1.0, 0.0)["p_occupied"], 0.05)
        for _ in range(5):
            g.record_detection(0.0, 0.0, 0.0, range_m=1.0)
        self.assertGreater(g.cell_value(1.0, 0.0)["p_occupied"], 0.5)

    def test_no_detection_marks_whole_wedge_clear_up_to_max_range(self):
        g = make_grid()
        g.record_detection(0.0, 0.0, 0.0, range_m=None)
        far = g.cell_value(1.1, 0.0)
        self.assertEqual(far["source"], "sensor")
        self.assertLess(far["p_occupied"], 0.5)

    def test_cells_outside_the_beam_angle_are_untouched(self):
        g = make_grid()
        g.record_detection(0.0, 0.0, 0.0, range_m=None)
        # 90 degrees off-axis, well outside the +/-7.5 degree beam
        side = g.cell_value(0.0, 0.5)
        self.assertEqual(side["source"], "unknown")

    def test_cells_beyond_the_hit_range_are_not_touched(self):
        g = make_grid()
        g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        beyond = g.cell_value(0.9, 0.0)
        self.assertEqual(beyond["source"], "unknown")


class TestForgetting(unittest.TestCase):
    def test_cell_reverts_to_unknown_after_forget_after_s(self):
        clock = FakeClock()
        g = make_grid(clock=clock, forget_after_s=100)
        g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        self.assertEqual(g.cell_value(0.5, 0.0)["source"], "sensor")
        clock.advance(101)
        self.assertEqual(g.cell_value(0.5, 0.0)["source"], "unknown")


class TestOverrides(unittest.TestCase):
    def test_override_wins_over_sensor_data(self):
        g = make_grid()
        for _ in range(5):
            g.record_detection(0.0, 0.0, 0.0, range_m=None)
        g.set_override(0.5, 0.0, "blocked")
        self.assertEqual(g.cell_value(0.5, 0.0), {"source": "override", "value": "blocked"})

    def test_override_never_forgotten(self):
        clock = FakeClock()
        g = make_grid(clock=clock, forget_after_s=10)
        g.set_override(0.5, 0.0, "blocked")
        clock.advance(1000)
        self.assertEqual(g.cell_value(0.5, 0.0), {"source": "override", "value": "blocked"})

    def test_clear_override_falls_back_to_sensor_data(self):
        g = make_grid()
        g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        g.set_override(0.5, 0.0, "clear")
        g.clear_override(0.5, 0.0)
        self.assertEqual(g.cell_value(0.5, 0.0)["source"], "sensor")


class TestStatsAndPersistence(unittest.TestCase):
    def test_stats_counts_cells_and_overrides(self):
        g = make_grid()
        g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        g.set_override(2.0, 2.0, "blocked")
        stats = g.stats()
        self.assertEqual(stats["overrides"], 1)
        self.assertGreater(stats["cells"], 0)

    def test_dump_and_load_state_round_trips(self):
        g = make_grid()
        g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        g.set_override(2.0, 2.0, "blocked")
        state = g.dump_state()

        g2 = make_grid()
        g2.load_state(state)
        self.assertEqual(g2.cell_value(0.5, 0.0), g.cell_value(0.5, 0.0))
        self.assertEqual(g2.cell_value(2.0, 2.0), {"source": "override", "value": "blocked"})
        self.assertFalse(g2.dirty)

    def test_dump_state_records_the_params_used(self):
        params = make_params(p_hit=0.8, p_miss=0.2, p_min=0.05, p_max=0.95)
        g = make_grid(params=params)
        dumped = g.dump_state()
        self.assertEqual(dumped["params"], {"p_hit": 0.8, "p_miss": 0.2, "p_min": 0.05, "p_max": 0.95})


if __name__ == "__main__":
    unittest.main()
