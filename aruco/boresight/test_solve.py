"""Tests for the bore-sight solve.

Deliberately small (few markers, few poses): the point is to check the
estimator's behaviour, not to re-run the design study — evaluate.py does
that, and takes minutes.
"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import layouts  # noqa: E402
import sim  # noqa: E402
import solve  # noqa: E402

DXC_B = np.array([0.0775, 0.002, -0.07])
TRUE_HPR_CB = np.array([2.0, -1.2, 1.5])

CLEAN = sim.NoiseModel(sigma_px=1e-9, pos_sigma_m=0.0, heading_sigma_deg=0.0,
                       heading_bias_deg=0.0, tilt_sigma_deg=0.0, tilt_bias_deg=0.0)


def _observations(noise=CLEAN, seed=1, legs=None, bump_deg=1.5):
    rng = np.random.default_rng(seed)
    legs = legs or (layouts.fan_legs(count=4) + layouts.past_legs(count=4))
    # 1fps — plenty of geometry, few enough rows to keep the tests quick
    t, pos, hpr = sim.legs_to_poses(legs, fps=1.0, bump_deg=bump_deg, rng=rng)
    return sim.simulate(layouts.row3h(), t, pos, hpr, TRUE_HPR_CB, DXC_B, noise, rng)


class SolveTestCase(unittest.TestCase):
    def test_recovers_hpr_cb_from_a_wrong_start_with_clean_data(self):
        obs, _bias = _observations()
        self.assertGreater(len(obs), 50)
        out = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS,
                          sigma_px=1e-9)
        self.assertTrue(out["success"])
        np.testing.assert_allclose(out["hpr_cb"], TRUE_HPR_CB, atol=1e-3)

    def test_also_recovers_the_marker_poses(self):
        """The markers are surveyed as a by-product — that's what makes it
        safe for Ben to plant them only approximately."""
        obs, _bias = _observations()
        out = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS,
                          sigma_px=1e-9)
        layout = layouts.row3h()
        np.testing.assert_allclose(out["marker_pos"], layout.positions, atol=2e-3)
        np.testing.assert_allclose(out["marker_hpr"], layout.hprs, atol=0.05)

    def _heading_bias_absorption(self, bump_deg=1.5, hpr_cb=TRUE_HPR_CB, dxc_b=DXC_B):
        """How much of a 0.5 deg INS heading bias ends up in hpr_cb."""
        rng = np.random.default_rng(1)
        legs = layouts.fan_legs(count=4) + layouts.past_legs(count=4)
        t, pos, hpr = sim.legs_to_poses(legs, fps=1.0, bump_deg=bump_deg, rng=rng)
        obs, _b = sim.simulate(layouts.row3h(), t, pos, hpr, np.asarray(hpr_cb),
                               np.asarray(dxc_b), CLEAN, rng)
        kw = dict(camera_matrix=sim.CAMERA_MATRIX, dist_coeffs=sim.DIST_COEFFS, sigma_px=1e-9)
        baseline = solve.solve(obs, [0.0, 0.0, 0.0], dxc_b, **kw)["hpr_cb"]
        shifted = solve.Observations(
            obs.t, obs.nav_pos_n, obs.nav_hpr + np.array([0.5, 0.0, 0.0]), obs.marker_idx,
            obs.corners, obs.sizes, obs.marker_ids,
        )
        moved = solve.solve(shifted, [0.0, 0.0, 0.0], dxc_b, **kw)["hpr_cb"]
        return moved - baseline

    def test_ins_heading_bias_is_absorbed_exactly_with_no_lever_arm_or_mount_angle(self):
        """The clean case: camera at the INS origin, zero mount angle,
        level ground. C_nb(h+b,p,r) = Rz(b).C_nb, and that nav-frame
        rotation passes straight through to the camera, so hpr_cb's
        heading takes up the entire bias and nothing else moves."""
        delta = self._heading_bias_absorption(bump_deg=0.0, hpr_cb=[0, 0, 0], dxc_b=[0, 0, 0])
        self.assertAlmostEqual(delta[0], 0.5, places=4)
        np.testing.assert_allclose(delta[1:], 0.0, atol=1e-4)

    def test_the_lever_arm_is_what_stops_the_absorption_being_exact(self):
        """A heading bias swings the camera *position* by |d_xc_b|.b, and
        no camera rotation reproduces a translation — so ~1.3% of the bias
        survives. This is the term that makes the real numbers come out a
        little over the bias, and it is the lever arm that does it, not
        the mount angle and not vehicle tilt."""
        with_arm = self._heading_bias_absorption(bump_deg=0.0, hpr_cb=[0, 0, 0], dxc_b=DXC_B)
        self.assertGreater(with_arm[0], 0.502)
        self.assertLess(with_arm[0], 0.52)

    def test_a_mount_angle_leaks_into_pitch_and_roll_instead_of_heading(self):
        """An hpr_cb heading change rotates about the c-frame vertical
        while the INS bias rotates about the body vertical; a non-zero
        mount angle separates the two, and the leak shows up in pitch and
        roll rather than in the heading figure."""
        delta = self._heading_bias_absorption(bump_deg=0.0, hpr_cb=TRUE_HPR_CB, dxc_b=[0, 0, 0])
        self.assertAlmostEqual(delta[0], 0.5, places=3)
        self.assertGreater(np.max(np.abs(delta[1:])), 5e-3)

    def test_vehicle_tilt_is_not_what_breaks_the_degeneracy(self):
        """Worth pinning because it's the intuitive guess and it's wrong:
        bumpy ground changes the absorbed fraction by well under a tenth
        of what the lever arm does."""
        level = self._heading_bias_absorption(bump_deg=0.0)
        bumpy = self._heading_bias_absorption(bump_deg=1.5)
        self.assertLess(abs(bumpy[0] - level[0]), 0.001)

    def test_residual_rms_reflects_real_ins_error_not_pixel_noise(self):
        """With a realistic INS the fit cannot reach sub-pixel: the solver
        treats the reported pose as exact, so INS attitude error shows up
        as reprojection residual. ~0.2 deg at ~3m is several pixels, and
        that is why the formal covariance reads optimistically."""
        obs, _bias = _observations(noise=sim.NoiseModel())
        out = solve.solve(obs, TRUE_HPR_CB, DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS)
        self.assertGreater(out["rms_px"], 1.0)
        self.assertLess(out["rms_px"], 25.0)
        self.assertGreater(out["chi2_reduced"], 1.0)

    def test_starting_marker_poses_do_not_change_the_answer(self):
        obs, _bias = _observations()
        auto = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX,
                           sim.DIST_COEFFS, sigma_px=1e-9)["hpr_cb"]
        layout = layouts.row3h()
        seeded = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS,
                             sigma_px=1e-9, marker_pos0=layout.positions + 0.1,
                             marker_hpr0=layout.hprs + 3.0)["hpr_cb"]
        np.testing.assert_allclose(auto, seeded, atol=1e-3)


class SizeScaleTestCase(unittest.TestCase):
    """A wrong marker size is a systematic error that a healthy-looking
    solve absorbs silently. The real case, measured 2026-09-16: markers
    are 100.2mm, config says 97mm — a 3.3% range error, and the thing
    that pulls navigation towards the marker at the waterbutt."""

    TRUE_SIZE = 0.1002
    CONFIGURED_SIZE = 0.097

    def _mismatched(self, noise=CLEAN):
        rng = np.random.default_rng(4)
        legs = layouts.fan_legs(count=4) + layouts.past_legs(count=4)
        t, pos, hpr = sim.legs_to_poses(legs, fps=1.0, rng=rng)
        layout = layouts.row3h(size=self.TRUE_SIZE)
        return sim.simulate(layout, t, pos, hpr, TRUE_HPR_CB, DXC_B, noise, rng,
                            assumed_size=self.CONFIGURED_SIZE)

    def test_recovers_the_size_scale(self):
        obs, _b = self._mismatched()
        out = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS,
                          sigma_px=1e-9, estimate_size_scale=True)
        expected = self.TRUE_SIZE / self.CONFIGURED_SIZE
        self.assertAlmostEqual(out["size_scale"], expected, places=3)
        self.assertAlmostEqual(out["size_scale"] * self.CONFIGURED_SIZE, self.TRUE_SIZE, places=4)

    def test_estimating_the_scale_protects_hpr_cb(self):
        obs, _b = self._mismatched()
        out = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS,
                          sigma_px=1e-9, estimate_size_scale=True)
        np.testing.assert_allclose(out["hpr_cb"], TRUE_HPR_CB, atol=5e-3)

    def test_a_wrong_size_does_NOT_meaningfully_bias_hpr_cb(self):
        """Pins the surprise, because the intuitive expectation is wrong
        and was written into this module's docstring before being
        measured: a 3.3% size error left unestimated moves hpr_cb by
        ~0.0035 deg and the marker positions by ~0mm.

        hpr_cb is fixed by the *bearings* to the corners; size only
        affects *range*. Orthogonal. With many views the bearings pin the
        positions hard, and the size error ends up in the residual
        instead of in any parameter.

        The live GAD path has no such protection — it uses a single-shot
        tvec whose magnitude scales directly with the assumed size — which
        is why the waterbutt gets pulled while this solve does not care."""
        obs, _b = self._mismatched()
        fixed = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS,
                            sigma_px=1e-9, estimate_size_scale=False)
        self.assertTrue(fixed["success"])
        self.assertLess(np.max(np.abs(fixed["hpr_cb"] - TRUE_HPR_CB)), 0.01)
        layout = layouts.row3h(size=self.TRUE_SIZE)
        np.testing.assert_allclose(fixed["marker_pos"], layout.positions, atol=1e-3)

    def test_the_size_error_shows_up_in_the_residual_instead(self):
        """Where it does go. Note this is only legible on clean data: 0.8px
        in quadrature with the ~6px an INS-realistic run produces is
        invisible, so elevated residuals are NOT a practical detector for
        a wrong marker size. Estimating the scale is."""
        obs, _b = self._mismatched()
        fixed = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS,
                            sigma_px=1e-9, estimate_size_scale=False)
        freed = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS,
                            sigma_px=1e-9, estimate_size_scale=True)
        self.assertGreater(fixed["rms_px"], 0.5)
        self.assertLess(freed["rms_px"], fixed["rms_px"] / 10)

    def test_the_scale_is_harmless_when_the_size_is_already_right(self):
        """Carrying the extra unknown must not cost anything when there is
        nothing for it to find."""
        obs, _b = _observations()
        out = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS,
                          sigma_px=1e-9, estimate_size_scale=True)
        self.assertAlmostEqual(out["size_scale"], 1.0, places=3)
        np.testing.assert_allclose(out["hpr_cb"], TRUE_HPR_CB, atol=5e-3)

    def test_size_scale_is_reported_as_one_when_not_estimated(self):
        obs, _b = _observations()
        out = solve.solve(obs, [0.0, 0.0, 0.0], DXC_B, sim.CAMERA_MATRIX, sim.DIST_COEFFS,
                          sigma_px=1e-9)
        self.assertEqual(out["size_scale"], 1.0)


class InitialMarkerPosesTestCase(unittest.TestCase):
    def test_bootstrap_lands_close_enough_to_be_a_starting_point(self):
        obs, _bias = _observations()
        pos, hpr = solve.initial_marker_poses(obs, TRUE_HPR_CB, DXC_B,
                                              sim.CAMERA_MATRIX, sim.DIST_COEFFS)
        layout = layouts.row3h()
        np.testing.assert_allclose(pos, layout.positions, atol=0.02)
        np.testing.assert_allclose(hpr, layout.hprs, atol=1.0)

    def test_subsampling_caps_the_work_without_changing_the_bootstrap_much(self):
        obs, _bias = _observations()
        few = solve.initial_marker_poses(obs, TRUE_HPR_CB, DXC_B, sim.CAMERA_MATRIX,
                                         sim.DIST_COEFFS, max_per_marker=5)[0]
        many = solve.initial_marker_poses(obs, TRUE_HPR_CB, DXC_B, sim.CAMERA_MATRIX,
                                          sim.DIST_COEFFS, max_per_marker=10000)[0]
        np.testing.assert_allclose(few, many, atol=0.02)


class GeometryCheckTestCase(unittest.TestCase):
    """A degenerate solve does not look degenerate. A real stationary
    capture (318 detections, residual 0.42px, success=True) solved to a
    marker size of 11.25 METRES with markers 130m away. Nothing in the
    fit gave it away, so the data has to be checked before solving."""

    def test_accepts_a_properly_driven_pattern(self):
        obs, _bias = _observations()
        out = solve.geometry_check(obs)
        self.assertTrue(out["ok"], out["problems"])
        self.assertGreater(out["position_spread_m"], solve.MIN_POSITION_SPREAD_M)
        self.assertGreater(out["heading_spread_deg"], solve.MIN_HEADING_SPREAD_DEG)

    def test_rejects_a_stationary_capture(self):
        """The exact shape of the real failure: plenty of observations,
        no viewpoint diversity at all."""
        obs, _bias = _observations()
        frozen = solve.Observations(
            obs.t, np.tile(obs.nav_pos_n[0], (len(obs), 1)),
            np.tile(obs.nav_hpr[0], (len(obs), 1)),
            obs.marker_idx, obs.corners, obs.sizes, obs.marker_ids)
        out = solve.geometry_check(frozen)
        self.assertFalse(out["ok"])
        self.assertIn("position_spread_m", out["problems"])
        self.assertIn("heading_spread_deg", out["problems"])
        self.assertGreater(out["n_obs"], 100)  # not a shortage of data

    def test_rejects_a_marker_never_seen(self):
        obs, _bias = _observations()
        out = solve.geometry_check(obs.subset(obs.marker_idx != 2))
        self.assertFalse(out["ok"])
        self.assertIn("marker_22", out["problems"])

    def _with_headings(self, obs, headings):
        return solve.Observations(
            obs.t, obs.nav_pos_n,
            np.column_stack([headings, obs.nav_hpr[:, 1], obs.nav_hpr[:, 2]]),
            obs.marker_idx, obs.corners, obs.sizes, obs.marker_ids)

    def test_heading_spread_is_circular_not_a_plain_range(self):
        """+179 and -179 are 2 degrees apart, not 358. A plain max-minus-min
        would call this the widest possible spread and wave through a robot
        that never turned."""
        obs, _bias = _observations()
        alternating = np.where(np.arange(len(obs)) % 2, 179.0, -179.0)
        out = solve.geometry_check(self._with_headings(obs, alternating))
        self.assertLess(out["heading_spread_deg"], 5.0)
        self.assertIn("heading_spread_deg", out["problems"])  # correctly rejected

    def test_genuinely_opposed_headings_read_as_wide(self):
        obs, _bias = _observations()
        opposed = np.where(np.arange(len(obs)) % 2, 0.0, 180.0)
        out = solve.geometry_check(self._with_headings(obs, opposed))
        self.assertGreater(out["heading_spread_deg"], 150.0)


class SplitCheckTestCase(unittest.TestCase):
    def test_reports_one_estimate_per_group_and_a_scatter(self):
        obs, _bias = _observations(noise=sim.NoiseModel())
        out = solve.split_check(obs, TRUE_HPR_CB, DXC_B, sim.CAMERA_MATRIX,
                                sim.DIST_COEFFS, n_groups=3)
        self.assertGreaterEqual(len(out["estimates"]), 2)
        self.assertEqual(out["estimates"].shape[1], 3)
        self.assertTrue(np.all(np.isfinite(out["scatter"])))

    def test_skips_groups_that_do_not_see_every_marker(self):
        """A time slice missing a marker leaves 6 parameters unconstrained;
        it must be dropped, not solved into nonsense."""
        obs, _bias = _observations()
        # Let marker 2 appear only in the first fifth of the session, then
        # ask for two groups. The later group then cannot contain it.
        #
        # Note the group edges are time QUANTILES, so they crowd towards
        # wherever the rows are: blinding merely the second half isn't
        # enough, because the edges simply move with it and every group
        # still sees all three markers.
        cut = np.quantile(obs.t, 0.2)
        keep = (obs.marker_idx != 2) | (obs.t <= cut)
        starved = obs.subset(keep)
        self.assertGreater(np.quantile(starved.t, 0.5), cut)  # the split really does strand marker 2
        out = solve.split_check(starved, TRUE_HPR_CB, DXC_B, sim.CAMERA_MATRIX,
                                sim.DIST_COEFFS, n_groups=2, sigma_px=1e-9)
        self.assertEqual(len(out["estimates"]), 1)

    def test_groups_by_capture_time_not_by_row_order(self):
        """Rows arrive grouped by marker, not by time. Splitting on row
        order would hand each group a single marker and quietly drop every
        one of them — which is exactly what happened before obs.t existed."""
        obs, _bias = _observations(noise=sim.NoiseModel())
        self.assertGreater(len(np.unique(obs.marker_idx[:len(obs) // 3])), 0)
        out = solve.split_check(obs, TRUE_HPR_CB, DXC_B, sim.CAMERA_MATRIX,
                                sim.DIST_COEFFS, n_groups=3)
        self.assertEqual(len(out["estimates"]), 3)


class ObservationsTestCase(unittest.TestCase):
    def test_subset_keeps_the_marker_id_list_intact(self):
        obs, _bias = _observations()
        sub = obs.subset(obs.marker_idx == 0)
        self.assertEqual(sub.marker_ids, obs.marker_ids)  # indices must stay meaningful
        self.assertEqual(sub.n_markers, obs.n_markers)
        self.assertLess(len(sub), len(obs))


if __name__ == "__main__":
    unittest.main()
