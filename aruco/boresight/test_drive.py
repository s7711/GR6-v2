"""Tests for capture-path generation and driving."""

import math
import sys
import unittest
from pathlib import Path
import unittest.mock
from unittest.mock import MagicMock

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import drive  # noqa: E402
import layouts  # noqa: E402
import placement  # noqa: E402

REAL_BOX = Path(__file__).resolve().parent.parent.parent / "navigate" / "data" / "Boresight limits.yaml"


def _box_points():
    return yaml.safe_load(REAL_BOX.read_text())


def _as_points(offsets_ne):
    """Local north/east offsets -> path point dicts, for turn_stats."""
    return [{"lat": 52.235464 + n / 111320.0, "lon": -1.460508 + e / 68000.0}
            for n, e in offsets_ne]


class EndsInwardOrderTestCase(unittest.TestCase):
    def test_runs_from_both_ends_inwards(self):
        self.assertEqual(drive.ends_inward_order(7), [0, 6, 1, 5, 2, 4, 3])
        self.assertEqual(drive.ends_inward_order(6), [0, 5, 1, 4, 2, 3])

    def test_covers_every_index_exactly_once(self):
        for n in range(1, 12):
            self.assertEqual(sorted(drive.ends_inward_order(n)), list(range(n)))

    def test_successive_steps_alternate_direction(self):
        """A monotonic sweep turns the same way at every transition, which
        the xNAV650 doesn't like (Ben, 2026-09-16)."""
        order = drive.ends_inward_order(7)
        steps = [b - a for a, b in zip(order, order[1:])]
        for s0, s1 in zip(steps, steps[1:]):
            self.assertLess(s0 * s1, 0, f"consecutive steps {s0},{s1} go the same way")


class TurnStepsTestCase(unittest.TestCase):
    """navigate's turn controller is shortest-way (turn_control.py), so
    asking for a heading does not let you choose which way it goes round.
    Forcing the other direction needs intermediate headings."""

    def test_single_step_when_the_shortest_way_is_already_right(self):
        steps = drive.turn_steps(0.0, 40.0, +1)
        self.assertEqual(len(steps), 1)
        self.assertAlmostEqual(steps[0]["heading_deg"], 40.0)

    def test_splits_into_pieces_to_force_the_long_way_round(self):
        steps = drive.turn_steps(0.0, 40.0, -1)
        self.assertEqual(len(steps), drive.FORCED_TURN_PIECES)
        self.assertAlmostEqual(steps[-1]["heading_deg"], 40.0)

    def test_every_forced_piece_turns_the_wanted_way(self):
        """Each intermediate hop must itself be shortest-way in the
        direction wanted, or the controller undoes the whole point."""
        for want in (+1, -1):
            for target in (30.0, 90.0, 150.0, -60.0, -170.0):
                steps = drive.turn_steps(0.0, target, want)
                heading = 0.0
                for step in steps:
                    delta = (step["heading_deg"] - heading + 180.0) % 360.0 - 180.0
                    self.assertGreater(delta * want, 0, f"piece turned the wrong way: {steps}")
                    self.assertLess(abs(delta), 180.0)
                    heading = step["heading_deg"]
                self.assertAlmostEqual((heading - target + 180) % 360 - 180, 0.0, places=6)

    def test_zero_sign_means_do_not_care(self):
        self.assertEqual(len(drive.turn_steps(0.0, 40.0, 0)), 1)


class BuildJobTestCase(unittest.TestCase):
    def _job(self):
        box = _box_points()
        info, targets = placement.marker_targets(
            box, (20, 21, 22), layouts.RECOMMENDED_HEIGHT_M, 0.5, layouts.INS_HEIGHT_M)
        return drive.build_job(box, info["face_bearing_deg"], targets[1]["lat"], targets[1]["lon"])

    def test_produces_legs_and_alternating_steps(self):
        legs, steps, info = self._job()
        self.assertGreater(len(legs), 8)
        self.assertEqual(steps[0]["type"], "run_path")
        self.assertEqual(len([s for s in steps if s["type"] == "run_path"]), len(legs))
        self.assertGreater(info["turn_count"], 0)

    def test_turns_are_balanced_left_and_right(self):
        """Ben's point: the xNAV650 does better when turns aren't all one
        way — and turn_to_heading won't do that for us."""
        _legs, _steps, info = self._job()
        self.assertEqual(info["turns_left"], info["turns_right"])

    def test_legs_form_a_connected_chain(self):
        """Each leg must end where the next begins, or navigate's entry
        check (1.0m) rejects the next run_path step."""
        legs, _steps, _info = self._job()
        for (_n0, a), (_n1, b) in zip(legs, legs[1:]):
            self.assertAlmostEqual(a[-1]["lat"], b[0]["lat"], places=9)
            self.assertAlmostEqual(a[-1]["lon"], b[0]["lon"], places=9)

    def test_every_leg_point_is_inside_the_recorded_boundary(self):
        legs, _steps, info = self._job()
        from shared.geodesy import lla_to_ned
        local = info["local"]
        for _name, pts in legs:
            for p in pts:
                ne = np.array(lla_to_ned(p["lat"], p["lon"], 0.0,
                                         info["centroid"][0], info["centroid"][1], 0.0)[:2])
                for i in range(len(local)):
                    a, b = local[i], local[(i + 1) % len(local)]
                    edge = b - a
                    cross = edge[0] * (ne[1] - a[1]) - edge[1] * (ne[0] - a[0])
                    self.assertLessEqual(cross, 1e-6)

    def test_rejects_a_boundary_that_is_not_a_box(self):
        legs, steps, info = drive.build_job([{"lat": 52.2, "lon": -1.4}] * 3, 0.0, 52.2, -1.4)
        self.assertIsNone(legs)
        self.assertIsNone(steps)
        self.assertFalse(info["is_box"])

    def test_approach_legs_are_slower_than_return_legs(self):
        """Ben's point: slow down driving IN (where a marker is about to
        drop off the image edge) and speed up driving back OUT."""
        legs, _steps, _info = self._job()
        for _name, points in legs:
            self.assertEqual(points[0]["speed_mps"], points[1]["speed_mps"])
            self.assertIn(points[0]["speed_mps"], (drive.APPROACH_SPEED_MPS, drive.RETURN_SPEED_MPS))

    def test_length_and_seconds_account_for_mixed_speeds(self):
        _legs, _steps, info = self._job()
        self.assertGreater(info["seconds"], 0)
        # A single-speed estimate using the faster speed would understate
        # the time; using the slower speed would overstate it.
        naive_fast = info["length_m"] / drive.RETURN_SPEED_MPS
        naive_slow = info["length_m"] / drive.APPROACH_SPEED_MPS
        self.assertGreater(info["seconds"], naive_fast)
        self.assertLess(info["seconds"], naive_slow)

    def test_no_lead_in_without_a_robot_position(self):
        legs, steps, _info = self._job()
        self.assertEqual(steps[0]["type"], "run_path")
        self.assertEqual(steps[0]["path"], legs[0][0])

    def test_lead_in_drives_to_leg_zero_then_turns_onto_it(self):
        """Ben's point: he had to drive the robot to the start by hand.
        A parked-elsewhere robot should get turn -> run_path(approach) ->
        turn -> run_path(leg 0), not start leg 0's run_path cold."""
        box = _box_points()
        info, targets = placement.marker_targets(
            box, (20, 21, 22), layouts.RECOMMENDED_HEIGHT_M, 0.5, layouts.INS_HEIGHT_M)
        legs, steps, _info2 = drive.build_job(
            box, info["face_bearing_deg"], targets[1]["lat"], targets[1]["lon"],
            robot_lat=box[0]["lat"], robot_lon=box[0]["lon"], robot_heading_deg=0.0)
        self.assertEqual(legs[0][0], f"{drive.LEG_PATH_PREFIX} approach")
        self.assertEqual(legs[0][1][0]["lat"], box[0]["lat"])
        self.assertEqual(legs[0][1][0]["lon"], box[0]["lon"])
        self.assertEqual(legs[0][1][1]["lat"], legs[1][1][0]["lat"])
        self.assertEqual(steps[0]["type"], "turn_to_heading")
        run_path_types = [s["type"] for s in steps[:4]]
        self.assertIn("run_path", run_path_types)
        approach_run_index = next(i for i, s in enumerate(steps)
                                  if s["type"] == "run_path" and s["path"] == legs[0][0])
        leg0_run_index = next(i for i, s in enumerate(steps)
                              if s["type"] == "run_path" and s["path"] == legs[1][0])
        self.assertLess(approach_run_index, leg0_run_index)
        # A turn_to_heading separates the approach arrival from leg 0.
        self.assertEqual(steps[leg0_run_index - 1]["type"], "turn_to_heading")

    def test_no_approach_leg_when_already_at_the_start(self):
        """Below MIN_APPROACH_LEG_M, only the alignment turn is needed —
        a near-zero-length run_path would just fail navigate's entry
        check."""
        box = _box_points()
        info, targets = placement.marker_targets(
            box, (20, 21, 22), layouts.RECOMMENDED_HEIGHT_M, 0.5, layouts.INS_HEIGHT_M)
        legs, steps, _ = drive.build_job(box, info["face_bearing_deg"],
                                         targets[1]["lat"], targets[1]["lon"])
        first_start = legs[0][1][0]
        legs2, steps2, _ = drive.build_job(
            box, info["face_bearing_deg"], targets[1]["lat"], targets[1]["lon"],
            robot_lat=first_start["lat"], robot_lon=first_start["lon"], robot_heading_deg=200.0)
        self.assertEqual(len(legs2), len(legs))
        self.assertEqual(steps2[0]["type"], "turn_to_heading")
        self.assertEqual(steps2[1]["type"], "run_path")
        self.assertEqual(steps2[1]["path"], legs2[0][0])


class JobDriverTestCase(unittest.TestCase):
    def setUp(self):
        self.driver = drive.JobDriver("http://nav", "http://jobs")

    def test_save_posts_each_leg_then_the_job(self):
        """navigate's api_save_path reads payload['points'] and jobs'
        api_save_job reads payload['steps'] — a bare list 400s either."""
        with unittest.mock.patch.object(drive.requests, "post") as post:
            post.return_value = MagicMock(content=b"", status_code=204)
            self.driver.save([("Leg 00", [{"lat": 1.0, "lon": 2.0}])],
                             [{"type": "run_path", "path": "Leg 00"}])
            urls = [c.args[0] for c in post.call_args_list]
            self.assertEqual(urls, ["http://nav/api/paths/Leg 00", "http://jobs/api/jobs/Boresight capture"])
            self.assertEqual(post.call_args_list[0].kwargs["json"],
                             {"points": [{"lat": 1.0, "lon": 2.0}]})
            self.assertEqual(post.call_args_list[1].kwargs["json"],
                             {"steps": [{"type": "run_path", "path": "Leg 00"}]})

    def test_wait_until_idle_returns_when_not_running(self):
        self.driver.status = MagicMock(return_value={"state": "idle"})
        self.assertEqual(self.driver.wait_until_idle(poll_s=0)["state"], "idle")

    def test_wait_until_idle_stops_the_robot_when_asked_to(self):
        """A capture stopped from the page must halt the drive too —
        otherwise the robot carries on with nothing recording."""
        self.driver.status = MagicMock(return_value={"state": "running"})
        self.driver.stop = MagicMock()
        self.driver.wait_until_idle(poll_s=0, should_stop=lambda: True)
        self.driver.stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
