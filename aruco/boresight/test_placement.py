"""Tests for marker placement: the box check and the target positions.

The box check is the one piece of this project an operator can most
easily feed rubbish to (any saved path at all), so most of these are
about rejecting things that aren't boxes, and saying why.
"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import layouts  # noqa: E402
import placement  # noqa: E402
from shared.geodesy import ned_to_lla  # noqa: E402

REAL_BOX = Path(__file__).resolve().parent.parent.parent / "navigate" / "data" / "Boresight limits.yaml"
LAT0, LON0 = 52.2354, -1.4605


def _points(local_ne):
    """Local north/east metres -> the lat/lon point dicts a path holds."""
    out = []
    for north, east in local_ne:
        lat, lon, _ = ned_to_lla(north, east, 0.0, LAT0, LON0, 0.0)
        out.append({"lat": lat, "lon": lon})
    return out


def _rect(length=12.0, width=7.0, rotation_deg=0.0):
    half_l, half_w = length / 2.0, width / 2.0
    corners = [(+half_l, +half_w), (+half_l, -half_w), (-half_l, -half_w), (-half_l, +half_w)]
    r = math.radians(rotation_deg)
    rot = [(n * math.cos(r) - e * math.sin(r), n * math.sin(r) + e * math.cos(r)) for n, e in corners]
    return _points(rot)


class BoxCheckTestCase(unittest.TestCase):
    def test_accepts_a_clean_rectangle(self):
        out = placement.check_box(_rect())
        self.assertTrue(out["is_box"], out["problems"])
        self.assertEqual(out["problems"], [])
        for ang in out["corners_deg"]:
            self.assertAlmostEqual(ang, 90.0, delta=0.5)

    def test_accepts_the_real_recorded_boundary(self):
        """The actual path Ben drove — 86.8-95.3 deg corners, opposite
        sides differing by up to 11%. If the tolerances ever tighten past
        real hand-driving, this is what catches it."""
        points = yaml.safe_load(REAL_BOX.read_text())
        out = placement.check_box(points)
        self.assertTrue(out["is_box"], out["problems"])
        self.assertEqual(len(out["side_lengths_m"]), 4)
        self.assertAlmostEqual(max(out["side_lengths_m"]), 12.0, delta=0.5)
        self.assertAlmostEqual(min(out["side_lengths_m"]), 6.3, delta=0.5)

    def test_rejects_the_wrong_number_of_corners(self):
        out = placement.check_box(_rect()[:3])
        self.assertFalse(out["is_box"])
        self.assertIn("4 corners", out["problems"][0])

    def test_rejects_a_parallelogram(self):
        skewed = _points([(6.0, 3.5), (6.0, -3.5), (-6.0, -7.0), (-6.0, 0.0)])
        out = placement.check_box(skewed)
        self.assertFalse(out["is_box"])
        self.assertTrue(any("right angle" in p for p in out["problems"]))

    def test_rejects_a_trapezium_with_mismatched_opposite_sides(self):
        trap = _points([(6.0, 6.0), (6.0, -6.0), (-6.0, -1.5), (-6.0, 1.5)])
        out = placement.check_box(trap)
        self.assertFalse(out["is_box"])

    def test_rejects_a_box_too_small_to_drive_in(self):
        out = placement.check_box(_rect(length=4.0, width=2.0))
        self.assertFalse(out["is_box"])
        self.assertTrue(any("too small" in p for p in out["problems"]))

    def test_reports_geometry_even_when_it_rejects(self):
        """A bare 'not a box' is useless in the field — the operator needs
        to see which corner or side is wrong."""
        out = placement.check_box(_rect(length=4.0, width=2.0))
        self.assertFalse(out["is_box"])
        self.assertEqual(len(out["corners_deg"]), 4)
        self.assertEqual(len(out["side_lengths_m"]), 4)

    def test_long_axis_follows_the_rectangle_however_it_is_rotated(self):
        for rotation in (0.0, 30.0, 73.0, -125.0):
            out = placement.check_box(_rect(rotation_deg=rotation))
            self.assertTrue(out["is_box"])
            # The long axis is a direction, so either end of it is correct.
            diff = (out["long_axis_bearing_deg"] - rotation) % 180.0
            self.assertLess(min(diff, 180.0 - diff), 1.0)


class MarkerTargetsTestCase(unittest.TestCase):
    IDS = (20, 21, 22)

    def _targets(self, points=None):
        return placement.marker_targets(
            points or _rect(), self.IDS, height_m=layouts.RECOMMENDED_HEIGHT_M,
            spacing_m=0.5, ins_height_m=layouts.INS_HEIGHT_M,
        )

    def test_returns_nothing_for_a_path_that_is_not_a_box(self):
        info, targets = self._targets(_rect(length=4.0, width=2.0))
        self.assertFalse(info["is_box"])
        self.assertEqual(targets, [])

    def test_places_the_row_centred_on_the_panel_with_the_right_spacing(self):
        info, targets = self._targets()
        self.assertEqual([t["id"] for t in targets], list(self.IDS))
        ne = np.array([[t["north"], t["east"]] for t in targets])
        np.testing.assert_allclose(ne.mean(axis=0), info["panel_centre_ne"], atol=1e-9)
        gaps = np.linalg.norm(np.diff(ne, axis=0), axis=1)
        np.testing.assert_allclose(gaps, 0.5, atol=1e-6)

    def test_the_panel_is_backed_up_to_the_rear_edge_not_the_middle(self):
        """You can't see a marker from behind it, so a panel at the
        centroid wastes half the box. Backing it up converts that into
        usable ground — and into turning room (Ben, 2026-09-16)."""
        info, _targets = self._targets()
        self.assertLess(info["clearance_behind_m"], 1.5)
        self.assertGreater(info["clearance_m"], 3.0 * info["clearance_behind_m"])
        # ... and it is still inside the box.
        self.assertGreater(info["clearance_behind_m"], 0.0)

    def test_the_row_runs_perpendicular_to_the_facing_direction(self):
        info, targets = self._targets()
        ne = np.array([[t["north"], t["east"]] for t in targets])
        row = ne[-1] - ne[0]
        face = math.radians(info["face_bearing_deg"])
        face_vec = np.array([math.cos(face), math.sin(face)])
        self.assertAlmostEqual(float(np.dot(row / np.linalg.norm(row), face_vec)), 0.0, places=6)

    def test_faces_along_the_long_axis(self):
        info, _targets = self._targets(_rect(rotation_deg=73.0))
        diff = (info["face_bearing_deg"] - 73.0) % 180.0
        self.assertLess(min(diff, 180.0 - diff), 1.0)

    def test_faces_whichever_way_has_more_room(self):
        """The centroid of a hand-driven box isn't central, and the
        approach fan wants the longer run in front of it."""
        # A rectangle whose corners put the centroid off to one end.
        lopsided = _points([(10.0, 3.5), (10.0, -3.5), (-2.0, -3.5), (-2.0, 3.5)])
        info, _targets = placement.marker_targets(
            lopsided, self.IDS, height_m=0.25, spacing_m=0.5, ins_height_m=0.063)
        self.assertTrue(info["is_box"], info["problems"])
        # Centroid sits at local (0,0); the long run is towards +north.
        self.assertLess(abs(info["face_bearing_deg"]), 10.0)
        self.assertGreater(info["clearance_m"], 5.0)

    def test_marker_heading_is_180_from_the_facing_bearing(self):
        """The m convention has X out the marker's BACK — getting this
        backwards points every marker away from the robot."""
        info, targets = self._targets()
        for t in targets:
            diff = (t["heading_deg"] - info["face_bearing_deg"]) % 360.0
            self.assertAlmostEqual(diff, 180.0, places=6)

    def test_the_facing_direction_is_stable_across_repeated_calls(self):
        """On a symmetric box the two facing directions are a near-tie
        (5.87m vs 5.86m of clearance on the real boundary). A bare max()
        flips between them on rounding — and a flip after the markers are
        already planted would send the capture path round the wrong end
        of the box, looking at their backs."""
        points = yaml.safe_load(REAL_BOX.read_text())
        bearings = {placement.marker_targets(
            points, self.IDS, height_m=0.25, spacing_m=0.5,
            ins_height_m=0.063)[0]["face_bearing_deg"] for _ in range(5)}
        self.assertEqual(len(bearings), 1)
        # ... and reversing the order the corners were recorded in, which
        # changes nothing physical, must not change it either.
        reversed_info, _ = placement.marker_targets(
            list(reversed(points)), self.IDS, height_m=0.25, spacing_m=0.5, ins_height_m=0.063)
        diff = (reversed_info["face_bearing_deg"] - bearings.pop()) % 360.0
        self.assertLess(min(diff, 360.0 - diff), 1.0)

    def test_heights_are_above_ground_not_above_the_ins_plane(self):
        _info, targets = self._targets()
        for t in targets:
            self.assertAlmostEqual(t["height_m"], layouts.RECOMMENDED_HEIGHT_M)

    def test_matches_the_layout_the_simulator_studied(self):
        """The whole design study assumed a 1m-wide row at the box centre;
        if placement drifts from that, the predicted accuracy no longer
        applies."""
        points = yaml.safe_load(REAL_BOX.read_text())
        info, targets = placement.marker_targets(
            points, self.IDS, height_m=layouts.RECOMMENDED_HEIGHT_M, spacing_m=0.5,
            ins_height_m=layouts.INS_HEIGHT_M)
        self.assertTrue(info["is_box"], info["problems"])
        ne = np.array([[t["north"], t["east"]] for t in targets])
        span = float(np.linalg.norm(ne[-1] - ne[0]))
        self.assertAlmostEqual(span, 1.0, places=6)
        # Modulo 180: the study hard-codes one end of the long axis,
        # placement picks an end for itself, and on a symmetric box the
        # two ends are physically equivalent. What must agree is the
        # *axis* — facing across the short dimension would wreck the
        # approach fan.
        diff = (info["face_bearing_deg"] - layouts.PANEL_FACE_BEARING_DEG) % 180.0
        self.assertLess(min(diff, 180.0 - diff), 5.0)


class MoveGuidanceTestCase(unittest.TestCase):
    """Once a marker exists, "range/bearing from the robot" can't be
    acted on — you can't stand the robot where the marker must go. These
    are the nudge fields, resolved in the robot's own frame."""

    def _target(self, north=0.0, east=0.0, heading=0.0):
        lat, lon, _ = ned_to_lla(north, east, 0.0, LAT0, LON0, 0.0)
        return {"id": 20, "lat": lat, "lon": lon, "heading_deg": heading}

    def _actual(self, north=0.0, east=0.0, heading=0.0):
        lat, lon, _ = ned_to_lla(north, east, 0.0, LAT0, LON0, 0.0)
        return {"lat": lat, "lon": lon, "heading": heading}

    def test_nothing_to_do_when_it_is_already_right(self):
        out = placement.move_guidance(0.0, self._actual(), self._target())
        self.assertAlmostEqual(out["towards_robot_m"], 0.0, places=6)
        self.assertAlmostEqual(out["to_robot_right_m"], 0.0, places=6)
        self.assertAlmostEqual(out["rotate_deg"], 0.0, places=6)

    def test_marker_too_far_from_the_robot_must_come_towards_it(self):
        """Robot heads north, so the marker is north of it. A target 0.5m
        SOUTH of where the marker is means moving it 0.5m towards the
        robot."""
        out = placement.move_guidance(0.0, self._actual(north=2.0), self._target(north=1.5))
        self.assertAlmostEqual(out["towards_robot_m"], 0.5, places=3)
        self.assertAlmostEqual(out["to_robot_right_m"], 0.0, places=3)

    def test_marker_too_close_gives_a_negative_towards(self):
        out = placement.move_guidance(0.0, self._actual(north=2.0), self._target(north=2.5))
        self.assertAlmostEqual(out["towards_robot_m"], -0.5, places=3)

    def test_sideways_is_in_the_robots_right_hand_sense(self):
        """Robot heading north: its right is east."""
        out = placement.move_guidance(0.0, self._actual(north=2.0), self._target(north=2.0, east=0.4))
        self.assertAlmostEqual(out["to_robot_right_m"], 0.4, places=3)
        self.assertAlmostEqual(out["towards_robot_m"], 0.0, places=3)

    def test_the_frame_follows_the_robot_not_north(self):
        """Same physical correction, robot turned 90 degrees: what was
        'right' becomes 'towards'."""
        actual, target = self._actual(north=2.0), self._target(north=2.0, east=0.4)
        facing_east = placement.move_guidance(90.0, actual, target)
        self.assertAlmostEqual(facing_east["towards_robot_m"], -0.4, places=3)
        self.assertAlmostEqual(facing_east["to_robot_right_m"], 0.0, places=3)

    def test_the_two_distances_are_orthogonal(self):
        """They must compose without interacting, or applying one then
        the other doesn't land on the target."""
        out = placement.move_guidance(37.0, self._actual(north=2.0, east=1.0),
                                      self._target(north=1.6, east=1.3))
        combined = math.hypot(out["towards_robot_m"], out["to_robot_right_m"])
        self.assertAlmostEqual(combined, out["move_m"], places=6)

    def test_rotation_is_the_short_way_round(self):
        out = placement.move_guidance(0.0, self._actual(heading=170.0), self._target(heading=-170.0))
        self.assertAlmostEqual(out["rotate_deg"], 20.0, places=3)
        out = placement.move_guidance(0.0, self._actual(heading=-170.0), self._target(heading=170.0))
        self.assertAlmostEqual(out["rotate_deg"], -20.0, places=3)


class GuidanceTestCase(unittest.TestCase):
    def test_range_and_bearing_to_a_target_due_north(self):
        lat, lon, _ = ned_to_lla(5.0, 0.0, 0.0, LAT0, LON0, 0.0)
        out = placement.guidance_from(LAT0, LON0, {"id": 20, "lat": lat, "lon": lon})
        self.assertAlmostEqual(out["range_m"], 5.0, places=2)
        self.assertAlmostEqual(out["bearing_deg"], 0.0, places=2)

    def test_range_and_bearing_to_a_target_due_east(self):
        lat, lon, _ = ned_to_lla(0.0, 3.0, 0.0, LAT0, LON0, 0.0)
        out = placement.guidance_from(LAT0, LON0, {"id": 21, "lat": lat, "lon": lon})
        self.assertAlmostEqual(out["range_m"], 3.0, places=2)
        self.assertAlmostEqual(out["bearing_deg"], 90.0, places=2)

    def test_range_goes_to_zero_on_the_target(self):
        out = placement.guidance_from(LAT0, LON0, {"id": 22, "lat": LAT0, "lon": LON0})
        self.assertAlmostEqual(out["range_m"], 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
