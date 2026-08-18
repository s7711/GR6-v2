import unittest

from grid import MapGrid, sensor_world_pose


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_grid(clock=None, **overrides):
    kwargs = dict(cell_size_m=0.1, max_range_m=1.2, beam_half_angle_deg=7.5, forget_after_s=2592000, min_observations=2)
    kwargs.update(overrides)
    return MapGrid(now=clock or FakeClock(), **kwargs)


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
    def test_hit_marks_cell_at_the_measured_range(self):
        g = make_grid()
        g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        value = g.cell_value(0.5, 0.0)
        self.assertEqual(value["source"], "unknown")  # only 1 observation - below min_observations
        g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        value = g.cell_value(0.5, 0.0)
        self.assertEqual(value["source"], "sensor")
        self.assertEqual(value["hit"], 2)
        self.assertEqual(value["p_occupied"], 1.0)

    def test_cells_before_the_hit_are_marked_miss_not_hit(self):
        g = make_grid()
        for _ in range(2):
            g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        near = g.cell_value(0.1, 0.0)
        self.assertEqual(near["source"], "sensor")
        self.assertEqual(near["miss"], 2)
        self.assertEqual(near["hit"], 0)

    def test_no_detection_marks_whole_wedge_clear_up_to_max_range(self):
        g = make_grid()
        for _ in range(2):
            g.record_detection(0.0, 0.0, 0.0, range_m=None)
        far = g.cell_value(1.1, 0.0)
        self.assertEqual(far["source"], "sensor")
        self.assertEqual(far["miss"], 2)
        self.assertEqual(far["hit"], 0)

    def test_cells_outside_the_beam_angle_are_untouched(self):
        g = make_grid()
        for _ in range(2):
            g.record_detection(0.0, 0.0, 0.0, range_m=None)
        # 90 degrees off-axis, well outside the +/-7.5 degree beam
        side = g.cell_value(0.0, 0.5)
        self.assertEqual(side["source"], "unknown")

    def test_cells_beyond_the_hit_range_are_not_touched(self):
        g = make_grid()
        for _ in range(2):
            g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        beyond = g.cell_value(0.9, 0.0)
        self.assertEqual(beyond["source"], "unknown")


class TestForgetting(unittest.TestCase):
    def test_cell_reverts_to_unknown_after_forget_after_s(self):
        clock = FakeClock()
        g = make_grid(clock=clock, forget_after_s=100)
        for _ in range(2):
            g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        self.assertEqual(g.cell_value(0.5, 0.0)["source"], "sensor")
        clock.advance(101)
        self.assertEqual(g.cell_value(0.5, 0.0)["source"], "unknown")


class TestOverrides(unittest.TestCase):
    def test_override_wins_over_sensor_data(self):
        g = make_grid()
        for _ in range(5):
            g.record_detection(0.0, 0.0, 0.0, range_m=None)  # cell would otherwise read "clear"-ish
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
        for _ in range(2):
            g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        g.set_override(0.5, 0.0, "clear")
        g.clear_override(0.5, 0.0)
        self.assertEqual(g.cell_value(0.5, 0.0)["source"], "sensor")


class TestStatsAndPersistence(unittest.TestCase):
    def test_stats_counts_cells_and_totals(self):
        g = make_grid()
        g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        g.set_override(2.0, 2.0, "blocked")
        stats = g.stats()
        self.assertEqual(stats["overrides"], 1)
        self.assertGreater(stats["cells"], 0)
        self.assertGreater(stats["total_hit"] + stats["total_miss"], 0)

    def test_dump_and_load_state_round_trips(self):
        g = make_grid()
        for _ in range(2):
            g.record_detection(0.0, 0.0, 0.0, range_m=0.5)
        g.set_override(2.0, 2.0, "blocked")
        state = g.dump_state()

        g2 = make_grid()
        g2.load_state(state)
        self.assertEqual(g2.cell_value(0.5, 0.0), g.cell_value(0.5, 0.0))
        self.assertEqual(g2.cell_value(2.0, 2.0), {"source": "override", "value": "blocked"})
        self.assertFalse(g2.dirty)


if __name__ == "__main__":
    unittest.main()
