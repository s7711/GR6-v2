import math
import unittest

import geometry
from geometry import PathPoint


class TestLocalFrameConversion(unittest.TestCase):
    def test_reference_point_maps_to_origin(self):
        north, east = geometry.to_local(52.2, -1.5, 52.2, -1.5)
        self.assertAlmostEqual(north, 0.0, places=6)
        self.assertAlmostEqual(east, 0.0, places=6)

    def test_one_arcsecond_north_is_about_thirty_metres(self):
        # ~1 arcsecond of latitude is ~30.9m at this latitude — a coarse
        # sanity check that lla_to_ned is being driven correctly, not a
        # precise geodesy test (that's shared/geodesy.py's own concern).
        north, east = geometry.to_local(52.2 + 1 / 3600, -1.5, 52.2, -1.5)
        self.assertAlmostEqual(east, 0.0, places=3)
        self.assertGreater(north, 25)
        self.assertLess(north, 35)


class TestProjectForward(unittest.TestCase):
    def test_due_north_matches_to_local_round_trip(self):
        lat, lon = geometry.project_forward(52.2, -1.5, heading_deg=0, distance_m=2.0)
        north, east = geometry.to_local(lat, lon, 52.2, -1.5)
        self.assertAlmostEqual(north, 2.0, places=3)
        self.assertAlmostEqual(east, 0.0, places=3)

    def test_due_east(self):
        lat, lon = geometry.project_forward(52.2, -1.5, heading_deg=90, distance_m=2.0)
        north, east = geometry.to_local(lat, lon, 52.2, -1.5)
        self.assertAlmostEqual(north, 0.0, places=3)
        self.assertAlmostEqual(east, 2.0, places=3)

    def test_zero_distance_is_the_same_point(self):
        lat, lon = geometry.project_forward(52.2, -1.5, heading_deg=45, distance_m=0.0)
        self.assertAlmostEqual(lat, 52.2, places=9)
        self.assertAlmostEqual(lon, -1.5, places=9)

    def test_path_reference_is_first_point(self):
        points = [{"lat": 1.0, "lon": 2.0}, {"lat": 3.0, "lon": 4.0}]
        self.assertEqual(geometry.path_reference(points), (1.0, 2.0))

    def test_path_to_local_first_point_is_origin(self):
        points = [
            {"lat": 52.2, "lon": -1.5, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
            {"lat": 52.2001, "lon": -1.5, "speed_mps": 0.5, "pump": True, "clearance_m": 0.5},
        ]
        ref_lat, ref_lon = geometry.path_reference(points)
        local = geometry.path_to_local(points, ref_lat, ref_lon)
        self.assertAlmostEqual(local[0].north, 0.0, places=6)
        self.assertAlmostEqual(local[0].east, 0.0, places=6)
        self.assertGreater(local[1].north, 0)  # second point is further north
        self.assertEqual(local[1].pump, True)


class TestBearingAndAngleDiff(unittest.TestCase):
    def test_bearing_cardinal_directions(self):
        self.assertAlmostEqual(geometry.bearing(0, 0, 1, 0), 0)  # due north
        self.assertAlmostEqual(geometry.bearing(0, 0, 0, 1), 90)  # due east
        self.assertAlmostEqual(geometry.bearing(0, 0, -1, 0), 180)  # due south
        self.assertAlmostEqual(geometry.bearing(0, 0, 0, -1), 270)  # due west

    def test_angle_diff_shortest_path_across_zero(self):
        self.assertAlmostEqual(geometry.angle_diff(10, 350), 20)
        self.assertAlmostEqual(geometry.angle_diff(350, 10), -20)

    def test_angle_diff_no_wraparound_needed(self):
        self.assertAlmostEqual(geometry.angle_diff(100, 80), 20)


class TestHeadingErrorDeg(unittest.TestCase):
    def test_facing_directly_at_target_is_zero_error(self):
        err = geometry.heading_error_deg(robot_heading_deg=0, from_north=0, from_east=0, to_north=10, to_east=0)
        self.assertAlmostEqual(err, 0)

    def test_facing_away_from_target(self):
        err = geometry.heading_error_deg(robot_heading_deg=180, from_north=0, from_east=0, to_north=10, to_east=0)
        self.assertAlmostEqual(abs(err), 180)


class TestProjectOntoSegment(unittest.TestCase):
    def test_point_on_segment_midpoint(self):
        px, py, t, dist = geometry.project_onto_segment(5, 0, 0, 0, 10, 0)
        self.assertAlmostEqual(t, 0.5)
        self.assertAlmostEqual(dist, 0.0)
        self.assertAlmostEqual(px, 5)
        self.assertAlmostEqual(py, 0)

    def test_point_off_to_the_side(self):
        px, py, t, dist = geometry.project_onto_segment(5, 3, 0, 0, 10, 0)
        self.assertAlmostEqual(t, 0.5)
        self.assertAlmostEqual(dist, 3.0)

    def test_projection_clamped_before_segment_start(self):
        px, py, t, dist = geometry.project_onto_segment(-5, 0, 0, 0, 10, 0)
        self.assertEqual(t, 0.0)
        self.assertAlmostEqual(dist, 5.0)

    def test_projection_clamped_after_segment_end(self):
        px, py, t, dist = geometry.project_onto_segment(15, 0, 0, 0, 10, 0)
        self.assertEqual(t, 1.0)
        self.assertAlmostEqual(dist, 5.0)


def _straight_north_path(length=20, step=10):
    """A path running due north from the origin, points every `step`m."""
    return [
        PathPoint(north=n, east=0, speed_mps=0.5, pump=False, clearance_m=0.5)
        for n in range(0, length + 1, step)
    ]


class TestFindEntrySegment(unittest.TestCase):
    def test_close_and_aligned_at_start_returns_first_segment(self):
        path = _straight_north_path()
        index = geometry.find_entry_segment(
            path, robot_north=0.1, robot_east=0.1, robot_heading_deg=1,
            max_distance_m=1.0, max_heading_deg=45,
        )
        self.assertEqual(index, 0)

    def test_too_far_from_every_segment_returns_none(self):
        path = _straight_north_path()
        index = geometry.find_entry_segment(
            path, robot_north=5, robot_east=100, robot_heading_deg=0,
            max_distance_m=1.0, max_heading_deg=45,
        )
        self.assertIsNone(index)

    def test_wrong_heading_at_every_segment_returns_none(self):
        path = _straight_north_path()
        index = geometry.find_entry_segment(
            path, robot_north=5, robot_east=0, robot_heading_deg=180,  # facing backwards
            max_distance_m=1.0, max_heading_deg=45,
        )
        self.assertIsNone(index)

    def test_close_to_first_but_only_aligned_at_second_segment(self):
        # Two path segments with different headings: first heads east,
        # second heads north. Robot sits right at the junction, facing
        # north — should be rejected on the first (heading mismatch) and
        # accepted on the second.
        path = [
            PathPoint(north=0, east=0, speed_mps=0.5, pump=False, clearance_m=0.5),
            PathPoint(north=0, east=10, speed_mps=0.5, pump=False, clearance_m=0.5),
            PathPoint(north=10, east=10, speed_mps=0.5, pump=False, clearance_m=0.5),
        ]
        index = geometry.find_entry_segment(
            path, robot_north=0, robot_east=10, robot_heading_deg=0,
            max_distance_m=1.0, max_heading_deg=45,
        )
        self.assertEqual(index, 1)


class TestFindLookaheadPoint(unittest.TestCase):
    def test_on_path_lookahead_is_ahead_by_lookahead_distance(self):
        path = _straight_north_path(length=20, step=10)
        result = geometry.find_lookahead_point(
            path, start_index=0, robot_north=5, robot_east=0, lookahead_distance_m=2,
        )
        self.assertAlmostEqual(result.north, 7)
        self.assertAlmostEqual(result.east, 0)
        self.assertAlmostEqual(result.cross_track_error_m, 0.0)
        self.assertEqual(result.tracked_index, 0)

    def test_off_path_to_the_east_has_positive_cross_track_error(self):
        path = _straight_north_path(length=20, step=10)
        result = geometry.find_lookahead_point(
            path, start_index=0, robot_north=5, robot_east=2, lookahead_distance_m=2,
        )
        self.assertGreater(result.cross_track_error_m, 0)

    def test_off_path_to_the_west_has_negative_cross_track_error(self):
        path = _straight_north_path(length=20, step=10)
        result = geometry.find_lookahead_point(
            path, start_index=0, robot_north=5, robot_east=-2, lookahead_distance_m=2,
        )
        self.assertLess(result.cross_track_error_m, 0)

    def test_speed_and_pump_come_from_tracked_point_not_lookahead(self):
        path = [
            PathPoint(north=0, east=0, speed_mps=0.3, pump=True, clearance_m=0.5),
            PathPoint(north=10, east=0, speed_mps=0.7, pump=False, clearance_m=0.5),
        ]
        result = geometry.find_lookahead_point(
            path, start_index=0, robot_north=1, robot_east=0, lookahead_distance_m=2,
        )
        self.assertEqual(result.speed_mps, 0.3)
        self.assertEqual(result.pump, True)

    def test_lookahead_crossing_into_next_segment(self):
        path = _straight_north_path(length=20, step=10)
        result = geometry.find_lookahead_point(
            path, start_index=0, robot_north=9, robot_east=0, lookahead_distance_m=5,
        )
        self.assertAlmostEqual(result.north, 14)

    def test_path_complete_at_last_point_returns_none(self):
        path = _straight_north_path(length=20, step=10)
        result = geometry.find_lookahead_point(
            path, start_index=len(path) - 1, robot_north=20, robot_east=0, lookahead_distance_m=2,
        )
        self.assertIsNone(result)

    def test_path_complete_flag_set_when_robot_reaches_final_point(self):
        path = _straight_north_path(length=20, step=10)
        result = geometry.find_lookahead_point(
            path, start_index=0, robot_north=20, robot_east=0, lookahead_distance_m=2,
        )
        self.assertTrue(result.path_complete)

    def test_path_complete_flag_false_mid_path(self):
        path = _straight_north_path(length=20, step=10)
        result = geometry.find_lookahead_point(
            path, start_index=0, robot_north=5, robot_east=0, lookahead_distance_m=2,
        )
        self.assertFalse(result.path_complete)

    def test_tracking_index_never_moves_backward(self):
        path = _straight_north_path(length=20, step=10)
        result = geometry.find_lookahead_point(
            path, start_index=1, robot_north=5, robot_east=0, lookahead_distance_m=2,
        )
        # Even though north=5 is geometrically closer to segment 0, the
        # scan starts at index 1 and must not move backward.
        self.assertEqual(result.tracked_index, 1)


class TestDifferentialDrive(unittest.TestCase):
    def test_straight_forward_no_turn(self):
        left, right = geometry.differential_drive(0.5, 0, wheel_base_m=0.42)
        self.assertAlmostEqual(left, 0.5)
        self.assertAlmostEqual(right, 0.5)

    def test_positive_turn_speeds_up_left_wheel_slows_right(self):
        # Positive turn = "need to turn right" (matches heading_error_deg's
        # sign convention) — turning right means the right/inside wheel
        # slows down, the left/outside wheel speeds up.
        left, right = geometry.differential_drive(0.5, 1.0, wheel_base_m=0.42)
        self.assertAlmostEqual(left, 0.5 + 0.21)
        self.assertAlmostEqual(right, 0.5 - 0.21)

    def test_clamped_symmetrically_preserving_ratio(self):
        left, right = geometry.differential_drive(0.5, 10.0, wheel_base_m=0.42, max_mps=0.8)
        self.assertAlmostEqual(max(abs(left), abs(right)), 0.8)
        # ratio between wheels preserved after scaling
        left_unclamped, right_unclamped = geometry.differential_drive(0.5, 10.0, wheel_base_m=0.42)
        self.assertAlmostEqual(left / right, left_unclamped / right_unclamped)

    def test_under_limit_is_unchanged(self):
        left, right = geometry.differential_drive(0.3, 0.1, wheel_base_m=0.42, max_mps=0.8)
        left2, right2 = geometry.differential_drive(0.3, 0.1, wheel_base_m=0.42)
        self.assertAlmostEqual(left, left2)
        self.assertAlmostEqual(right, right2)


class TestTurnCommand(unittest.TestCase):
    def test_heading_error_contribution_matches_gr6v1_magnitude(self):
        result = geometry.turn_command(heading_error_deg=10, cross_track_error_m=0.0,
                                        heading_gain=2.0, cte_gain=0.6)
        self.assertAlmostEqual(result, 2.0 * math.radians(10))

    def test_being_east_of_the_path_turns_left_not_right(self):
        # Positive cross-track error = robot is east of a path heading
        # north (see find_lookahead_point's own tests) — correcting back
        # onto the path means turning left, i.e. a NEGATIVE turn command
        # (positive = turn right, per differential_drive's convention).
        # GR6-v1's own equivalent formula added this term instead of
        # subtracting it, and its own code comment admitted the sign was
        # never verified — this test locks in the corrected, verified
        # direction.
        result = geometry.turn_command(heading_error_deg=0, cross_track_error_m=0.5,
                                        heading_gain=2.0, cte_gain=0.6)
        self.assertLess(result, 0)
        self.assertAlmostEqual(result, -0.6 * 0.5)

    def test_zero_error_is_zero_turn(self):
        self.assertAlmostEqual(geometry.turn_command(0, 0, 2.0, 0.6), 0.0)


if __name__ == "__main__":
    unittest.main()
