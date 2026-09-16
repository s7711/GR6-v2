"""Tests for the capture simulator — mostly the visibility gates and the
noise model, since those are what make a simulated result mean anything.
A gate that's wrong doesn't produce an error, it produces a confident
recommendation to plant markers somewhere they can't be seen.
"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import layouts  # noqa: E402
import sim  # noqa: E402

DXC_B = np.array([0.0775, 0.002, -0.07])
HPR_CB = np.array([0.0, 0.0, 0.0])


def _pose_facing(north, east, target=(0.0, 0.0)):
    """One pose at (north, east), pointing at `target`."""
    heading = np.degrees(np.arctan2(target[1] - east, target[0] - north))
    return np.array([[north, east, 0.0]]), np.array([[heading, 0.0, 0.0]])


class OuProcessTestCase(unittest.TestCase):
    def test_has_the_requested_standard_deviation(self):
        rng = np.random.default_rng(0)
        x = sim.ou_process(200000, 0.2, sigma=0.5, tau=5.0, rng=rng)
        self.assertAlmostEqual(float(np.std(x)), 0.5, delta=0.03)

    def test_is_correlated_at_tau_and_uncorrelated_when_tau_is_zero(self):
        rng = np.random.default_rng(1)
        dt, tau = 0.2, 5.0
        x = sim.ou_process(200000, dt, sigma=1.0, tau=tau, rng=rng)
        lag = int(tau / dt)
        rho = float(np.corrcoef(x[:-lag], x[lag:])[0, 1])
        self.assertAlmostEqual(rho, np.exp(-1.0), delta=0.05)  # e^-1 at one correlation time

        white = sim.ou_process(20000, dt, sigma=1.0, tau=0.0, rng=rng)
        self.assertLess(abs(float(np.corrcoef(white[:-1], white[1:])[0, 1])), 0.05)


class VisibilityGateTestCase(unittest.TestCase):
    def setUp(self):
        # One marker at the origin, 0.45m up, face looking due south.
        self.layout = sim.Layout([20], [[0.0, 0.0, -0.45]], [[0.0, 0.0, 0.0]])

    def _visible_from(self, north, east):
        pos, hpr = _pose_facing(north, east)
        ok, _corners = sim.visible(pos, hpr, self.layout, 0, HPR_CB, DXC_B)
        return bool(ok[0])

    def test_sees_a_marker_head_on_at_a_sensible_range(self):
        self.assertTrue(self._visible_from(-2.5, 0.0))  # 2.5m south, looking north at the face

    def test_rejects_beyond_the_aruco_range_limit(self):
        self.assertFalse(self._visible_from(-(sim.MAX_RANGE_M + 0.5), 0.0))

    def test_rejects_the_back_of_a_marker(self):
        self.assertFalse(self._visible_from(2.5, 0.0))  # north side — looking at the back

    def test_rejects_too_oblique_a_view(self):
        # Almost edge-on: 2.5m east of the marker, which faces south.
        self.assertFalse(self._visible_from(-0.3, 2.5))

    def test_rejects_a_high_marker_seen_from_close_up(self):
        """Ben's point: a marker 1m up leaves the 45-degree vertical FOV
        at short range, so 'drive close to survey it' doesn't work."""
        high = sim.Layout([20], [[0.0, 0.0, -1.0]], [[0.0, 0.0, 0.0]])
        pos, hpr = _pose_facing(-1.0, 0.0)
        close, _ = sim.visible(pos, hpr, high, 0, HPR_CB, DXC_B)
        pos, hpr = _pose_facing(-2.5, 0.0)
        far, _ = sim.visible(pos, hpr, high, 0, HPR_CB, DXC_B)
        self.assertFalse(bool(close[0]))
        self.assertTrue(bool(far[0]))

    def test_rejects_a_marker_behind_the_camera(self):
        """projectPoints will happily project a point behind the camera
        into a plausible-looking pixel; without the depth gate that
        becomes a ghost detection."""
        pos = np.array([[-2.5, 0.0, 0.0]])
        hpr = np.array([[180.0, 0.0, 0.0]])  # standing south of it but facing away
        ok, _ = sim.visible(pos, hpr, self.layout, 0, HPR_CB, DXC_B)
        self.assertFalse(bool(ok[0]))


class PoseGenerationTestCase(unittest.TestCase):
    def test_leg_poses_run_from_start_to_end_with_the_leg_heading(self):
        legs = [((-3.0, 0.0), (-1.0, 0.0))]  # 2m due north
        t, pos, hpr = sim.legs_to_poses(legs, speed_mps=0.2, fps=5.0, bump_deg=0.0,
                                        rng=np.random.default_rng(0))
        self.assertEqual(len(t), 50)  # 2m at 0.2m/s, 5fps
        self.assertAlmostEqual(pos[0, 0], -3.0, places=6)
        self.assertLess(pos[-1, 0], -1.0)
        self.assertAlmostEqual(float(np.median(hpr[:, 0])), 0.0, delta=3.0)  # north, plus wobble

    def test_bumps_appear_as_real_pitch_and_roll(self):
        legs = [((-3.5, 0.0), (-1.0, 0.0))]
        _t, _pos, flat = sim.legs_to_poses(legs, bump_deg=0.0, rng=np.random.default_rng(0))
        _t, _pos, bumpy = sim.legs_to_poses(legs, bump_deg=2.0, rng=np.random.default_rng(0))
        self.assertAlmostEqual(float(np.std(flat[:, 1])), 0.0, places=9)
        self.assertGreater(float(np.std(bumpy[:, 1])), 0.5)

    def test_stations_give_one_fixed_pose_each(self):
        t, pos, hpr = sim.stations_to_poses([((-2.0, 0.0), 0.0), ((-2.0, 1.0), 20.0)],
                                            dwell_s=2.0, fps=5.0,
                                            rng=np.random.default_rng(0))
        self.assertEqual(len(t), 20)
        self.assertEqual(len(np.unique(pos, axis=0)), 2)


class TimestampLatencyTestCase(unittest.TestCase):
    """The lag between the shutter opening and the nav sample paired with
    it. Systematic, so it biases rather than averaging out — the reason
    stationary capture is even a candidate."""

    def _straight_leg(self, speed=0.2, fps=5.0):
        legs = [((-4.0, 0.0), (-1.0, 0.0))]  # 3m due north at constant speed
        return sim.legs_to_poses(legs, speed_mps=speed, fps=fps, bump_deg=0.0,
                                 rng=np.random.default_rng(0))

    def test_zero_latency_changes_nothing(self):
        t, pos, hpr = self._straight_leg()
        out_pos, out_hpr = sim.apply_timestamp_latency(t, pos, hpr, 0.0)
        self.assertIs(out_pos, pos)
        self.assertIs(out_hpr, hpr)

    def _actual_speed(self, t, pos):
        """Measured, not requested: legs_to_poses divides a leg into a
        whole number of frames, so the achieved speed can differ from the
        nominal one by up to a frame's worth (0.395 vs 0.400 m/s in one
        of the cases below). Asserting against the nominal figure tests
        that discretisation, not the latency model."""
        return float(np.linalg.norm(np.diff(pos, axis=0), axis=1).mean() / np.median(np.diff(t)))

    def test_lagged_position_is_behind_by_speed_times_latency(self):
        latency = 0.05
        t, pos, hpr = self._straight_leg(speed=0.2)
        lagged, _ = sim.apply_timestamp_latency(t, pos, hpr, latency)
        # Travelling due north, so the lag shows as being that much south.
        shift = (pos - lagged)[5:-5, 0]  # trim the gradient's end effects
        np.testing.assert_allclose(shift, self._actual_speed(t, pos) * latency, atol=1e-6)

    def test_the_shift_scales_with_speed(self):
        latency = 0.05
        ratios = []
        for speed in (0.1, 0.4):
            t, pos, hpr = self._straight_leg(speed=speed)
            lagged, _ = sim.apply_timestamp_latency(t, pos, hpr, latency)
            shift = float((pos - lagged)[5:-5, 0].mean())
            ratios.append(shift / (self._actual_speed(t, pos) * latency))
        # Both must match their own actual speed, so the ratio of shifts
        # is the ratio of speeds whatever the discretisation did.
        np.testing.assert_allclose(ratios, 1.0, atol=1e-6)

    def test_heading_discontinuities_between_legs_do_not_blow_up(self):
        """Consecutive legs meet at a heading jump; differentiating across
        it would manufacture an enormous fake latency error at those
        frames if the rate weren't clipped."""
        legs = [((-3.0, 0.0), (-1.0, 0.0)), ((0.0, -3.0), (0.0, -1.0))]  # north, then east
        t, pos, hpr = sim.legs_to_poses(legs, bump_deg=0.0, rng=np.random.default_rng(0))
        _, lagged_hpr = sim.apply_timestamp_latency(t, pos, hpr, 0.05)
        self.assertLess(np.max(np.abs(hpr - lagged_hpr)), 5.0)

    def test_stationary_capture_is_immune_by_construction(self):
        t, pos, hpr = sim.stations_to_poses([((-2.5, 0.0), 0.0), ((-2.0, 1.0), 20.0)],
                                            dwell_s=3.0, fps=5.0, rng=np.random.default_rng(0))
        lagged_pos, _ = sim.apply_timestamp_latency(t, pos, hpr, 0.05)
        within_station = np.abs(pos - lagged_pos).max(axis=1)[2:13]  # inside the first dwell
        np.testing.assert_allclose(within_station, 0.0, atol=1e-9)


class BoxTestCase(unittest.TestCase):
    def test_centroid_is_inside_and_far_outside_is_not(self):
        self.assertTrue(layouts.inside_box(np.array([[0.0, 0.0]]))[0])
        self.assertFalse(layouts.inside_box(np.array([[20.0, 20.0]]))[0])

    def test_recorded_corners_are_outside_the_shrunk_boundary(self):
        """The margin has to actually bite, or a 'driveable' path can run
        along the very edge of what Ben measured."""
        self.assertFalse(layouts.inside_box(layouts.BOX_CORNERS).any())

    def test_generated_legs_stay_inside_the_box(self):
        for legs in (layouts.fan_legs(), layouts.past_legs(), layouts.both_sides_legs()):
            self.assertTrue(legs)
            for leg in legs:
                self.assertTrue(layouts.inside_box(np.array(leg)).all())


class HeightReferenceTestCase(unittest.TestCase):
    """The simulator's down=0 is the INS reference plane, not the ground.
    Conflating the two is not hypothetical — it happened, and the
    recommended marker heights were reported 63mm too low as a result."""

    def test_ground_height_round_trips_through_the_down_coordinate(self):
        for h in (0.0, 0.133, 0.35, 1.0):
            self.assertAlmostEqual(float(layouts.down_to_height(layouts.height_to_down(h))), h)

    def test_the_ins_plane_sits_below_the_camera_by_the_lever_arm(self):
        self.assertAlmostEqual(layouts.INS_HEIGHT_M,
                               layouts.CAMERA_HEIGHT_M + layouts.DXC_B_DOWN_M)
        self.assertLess(layouts.INS_HEIGHT_M, layouts.CAMERA_HEIGHT_M)  # camera is above the INS

    def test_a_marker_at_camera_height_lands_on_the_image_centreline(self):
        """Ties the height convention to something physical: a marker
        level with the camera must project to the principal point's row,
        whatever the numbers in between are doing."""
        down = float(layouts.height_to_down(layouts.CAMERA_HEIGHT_M))
        lay = sim.Layout([20], [[0.0, 0.0, down]], [[0.0, 0.0, 0.0]])
        pos, hpr = _pose_facing(-2.5, 0.0)
        _ok, corners = sim.visible(pos, hpr, lay, 0, HPR_CB, DXC_B)
        centre_row = float(corners[0].mean(axis=0)[1])
        self.assertAlmostEqual(centre_row, sim.CAMERA_MATRIX[1, 2], delta=2.0)


class LayoutTestCase(unittest.TestCase):
    def test_staggered_row_spreads_both_across_and_in_height(self):
        lay = layouts.row3h()
        self.assertEqual(len(lay), 3)
        heights = layouts.down_to_height(lay.positions[:, 2])
        self.assertGreater(np.ptp(heights), 0.3)  # height spread
        self.assertGreater(heights.min(), 0.05)   # not buried in the grass
        across = np.hypot(np.ptp(lay.positions[:, 0]), np.ptp(lay.positions[:, 1]))
        self.assertGreater(across, 0.9)  # width spread

    def test_back_to_back_layout_has_two_opposed_faces(self):
        lay = layouts.row3h_back_to_back()
        self.assertEqual(len(lay), 6)
        normals = sim.model.marker_normal_n(lay.hprs)
        self.assertLess(float(np.dot(normals[0], normals[3])), -0.9)  # facing opposite ways

    def test_marker_ids_are_unique_and_clear_of_the_deployed_map(self):
        """The bore-sight markers must not collide with the surveyed ones
        (10/11/12/17/19) — a mapped id would make aruco send GAD from it,
        steering the nav solution with the very hpr_cb being calibrated."""
        deployed = {10, 11, 12, 17, 19}
        for lay in (layouts.single(), layouts.stack3(), layouts.row3h(),
                    layouts.row3h_back_to_back()):
            self.assertEqual(len(set(lay.ids)), len(lay.ids))
            self.assertFalse(set(lay.ids) & deployed)


if __name__ == "__main__":
    unittest.main()
