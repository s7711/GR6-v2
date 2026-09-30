"""Tests for the bore-sight service glue.

The config writer gets the most attention: it edits the live config.yaml,
so it must never mangle a file it doesn't fully understand, and it must
preserve the comments that carry this project's tuning history.
"""

import csv
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import drive as drive_mod  # noqa: E402
import layouts  # noqa: E402
import service  # noqa: E402
import sim  # noqa: E402

from shared.geodesy import ned_to_lla  # noqa: E402

TRUE_HPR_CB = (2.0, -1.2, 1.5)
DXC_B = (0.0775, 0.002, -0.07)

REAL_BOX = Path(__file__).resolve().parent.parent.parent / "navigate" / "data" / "Boresight limits.yaml"

CONFIG_SAMPLE = """\
services:
  aruco:
    port: 8004
    camera_extrinsics:
      hpr_cb: [0, 0, 0]  # camera mount HPR relative to body; unmeasured/assumed
      d_xc_b: [0.0775, 0.002, -0.07]  # camera displacement in body frame, metres
  drive:
    counts_per_metre: 261            # 260901: adjusted down 2.8% from 269
"""


class _FakeNavClient:
    def __init__(self, payload):
        self._payload = payload

    def latest(self):
        return self._payload


class _FakeDriver:
    """Records what would have been sent to navigate/jobs, without any
    real HTTP - so build_capture_job/_run_loop's SEQUENCING can be
    checked (which job_name, in what order) without a live robot."""

    def __init__(self, idle_results=None):
        self.saved = []          # [{"job_name":, "legs":, "steps":}, ...]
        self.started = []        # [job_name, ...] in call order
        self.stopped = 0
        self._idle_results = list(idle_results or [])

    def save(self, leg_paths, steps, job_name=drive_mod.JOB_NAME):
        self.saved.append({"job_name": job_name, "legs": leg_paths, "steps": steps})
        return {}

    def start(self, job_name=drive_mod.JOB_NAME):
        self.started.append(job_name)
        return {"state": "running"}

    def stop(self):
        self.stopped += 1

    def wait_until_idle(self, should_stop=None):
        return self._idle_results.pop(0) if self._idle_results else {"state": "idle"}


def _service(tmp, nav_payload=None, paths_dir=None):
    return service.BoresightService(
        cfg={}, nav_client=_FakeNavClient(nav_payload or {"nav": {}, "connection": {}}),
        frame_reader_fn=lambda: None, detect_fn=lambda *a: [],
        marker_size=0.097, hpr_cb=(0.0, 0.0, 0.0), dxc_b=(0.0775, 0.002, -0.07),
        camera_cal_fn=lambda: (None, None),
        paths_dir=paths_dir or (tmp / "paths"), data_dir=tmp / "data",
    )


def _write_session(path, lat0=52.235464, lon0=-1.460508, seed=1):
    """A real-shaped session file — geometry good enough to pass
    geometry_check, same recipe test_solve.py uses for its own fixtures
    — so log_run can be exercised without a live robot."""
    rng = np.random.default_rng(seed)
    legs = layouts.fan_legs(count=4) + layouts.past_legs(count=4)
    t, pos, hpr = sim.legs_to_poses(legs, fps=1.0, bump_deg=1.5, rng=rng)
    noise = sim.NoiseModel(sigma_px=1e-9, pos_sigma_m=0.0, heading_sigma_deg=0.0,
                           heading_bias_deg=0.0, tilt_sigma_deg=0.0, tilt_bias_deg=0.0)
    obs, _bias = sim.simulate(layouts.row3h(), t, pos, hpr, TRUE_HPR_CB, DXC_B, noise, rng)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write(json.dumps({"type": "header", "marker_ids": obs.marker_ids,
                            "marker_size": 0.097}) + "\n")
        for i in range(len(obs)):
            lat, lon, alt = ned_to_lla(*obs.nav_pos_n[i], lat0, lon0, 0.0)
            f.write(json.dumps({
                "type": "obs", "t": float(obs.t[i]),
                "id": int(obs.marker_ids[obs.marker_idx[i]]), "size": float(obs.sizes[i]),
                "corners": obs.corners[i].tolist(),
                "lat": float(lat), "lon": float(lon), "alt": float(alt),
                "heading": float(obs.nav_hpr[i, 0]), "pitch": float(obs.nav_hpr[i, 1]),
                "roll": float(obs.nav_hpr[i, 2]),
            }) + "\n")


class PlacementPlanTestCase(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        (self.tmp / "paths").mkdir()
        shutil.copy2(REAL_BOX, self.tmp / "paths" / "Boresight limits.yaml")
        self.svc = _service(self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_plans_from_the_real_recorded_boundary(self):
        out = self.svc.placement_plan("Boresight limits", layouts.RECOMMENDED_HEIGHT_M,
                                      layouts.INS_HEIGHT_M)
        self.assertTrue(out["is_box"], out["problems"])
        self.assertEqual([t["id"] for t in out["targets"]], [20, 21, 22])
        self.assertEqual(len(out["box"]), 4)
        self.assertIn("clearance_m", out)

    def test_reports_a_missing_path_rather_than_raising(self):
        out = self.svc.placement_plan("no such path", 0.25, 0.063)
        self.assertIn("error", out)

    def test_says_that_size_means_the_black_square(self):
        """A 5cm white border makes the printed sheet ~197mm; using that
        as markerLength would double every range, and the solve would
        still converge. The page has to say so."""
        out = self.svc.placement_plan("Boresight limits", 0.25, 0.063)
        self.assertIn("BLACK SQUARE", out["marker_size_note"])

    def test_guidance_needs_a_nav_fix(self):
        self.assertIn("error", self.svc.placement_guidance([]))

    def test_guidance_gives_range_and_bearing_to_each_target(self):
        import math
        svc = _service(self.tmp, nav_payload={
            "nav": {"Lat": math.radians(52.2354), "Lon": math.radians(-1.4605)},
            "connection": {}})
        targets = [{"id": 20, "lat": 52.2354, "lon": -1.4605}]
        out = svc.placement_guidance(targets)
        self.assertAlmostEqual(out["targets"][0]["range_m"], 0.0, places=3)
        self.assertEqual(out["targets"][0]["id"], 20)


class ApplyToConfigTestCase(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.config = self.tmp / "config.yaml"
        self.config.write_text(CONFIG_SAMPLE)
        self.backups = self.tmp / "backups"
        self.svc = _service(self.tmp)
        self._patch = patch.object(service, "CONFIG_BACKUP_DIR", self.backups)
        self._patch.start()

    def tearDown(self):
        self._patch.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_writes_the_new_value(self):
        out = self.svc.apply_to_config(self.config, [1.234, -0.567, 2.0])
        self.assertEqual(out["written"], [1.234, -0.567, 2.0])
        loaded = yaml.safe_load(self.config.read_text())
        self.assertEqual(loaded["services"]["aruco"]["camera_extrinsics"]["hpr_cb"],
                         [1.234, -0.567, 2.0])

    def test_preserves_every_other_comment_in_the_file(self):
        """A yaml round-trip would strip these. config.yaml's comments
        carry tuning dates and the reasons values are what they are."""
        self.svc.apply_to_config(self.config, [1.0, 2.0, 3.0])
        text = self.config.read_text()
        self.assertIn("# camera displacement in body frame, metres", text)
        self.assertIn("# 260901: adjusted down 2.8% from 269", text)

    def test_leaves_the_rest_of_the_config_untouched(self):
        self.svc.apply_to_config(self.config, [1.0, 2.0, 3.0])
        loaded = yaml.safe_load(self.config.read_text())
        self.assertEqual(loaded["services"]["aruco"]["port"], 8004)
        self.assertEqual(loaded["services"]["drive"]["counts_per_metre"], 261)
        self.assertEqual(loaded["services"]["aruco"]["camera_extrinsics"]["d_xc_b"],
                         [0.0775, 0.002, -0.07])

    def test_backs_up_before_writing(self):
        out = self.svc.apply_to_config(self.config, [1.0, 2.0, 3.0])
        backup = Path(out["backup"])
        self.assertTrue(backup.exists())
        self.assertEqual(backup.read_text(), CONFIG_SAMPLE)

    def test_refuses_when_hpr_cb_is_ambiguous(self):
        """Two hpr_cb lines means this isn't the file shape we understand
        — refuse rather than edit the wrong one."""
        self.config.write_text(CONFIG_SAMPLE + "\n  other:\n    hpr_cb: [9, 9, 9]\n")
        before = self.config.read_text()
        out = self.svc.apply_to_config(self.config, [1.0, 2.0, 3.0])
        self.assertIn("error", out)
        self.assertEqual(self.config.read_text(), before)  # nothing written

    def test_refuses_when_there_is_no_hpr_cb_at_all(self):
        self.config.write_text("services:\n  aruco:\n    port: 8004\n")
        out = self.svc.apply_to_config(self.config, [1.0, 2.0, 3.0])
        self.assertIn("error", out)

    def test_records_that_the_value_was_calibrated(self):
        self.svc.apply_to_config(self.config, [1.0, 2.0, 3.0])
        self.assertIn("bore-sight calibrated", self.config.read_text())

    def test_reminds_that_aruco_needs_restarting(self):
        out = self.svc.apply_to_config(self.config, [1.0, 2.0, 3.0])
        self.assertIn("restart", out["note"])


class CaptureStatusTestCase(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.svc = _service(self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_status_before_any_capture(self):
        status = self.svc.capture_status()
        self.assertFalse(status["running"])
        self.assertIsNone(status["session"])

    def test_listing_sessions_when_none_exist(self):
        self.assertEqual(self.svc.list_sessions(), [])

    def test_solving_a_session_with_no_observations(self):
        path = self.tmp / "empty.jsonl"
        path.write_text('{"type": "header", "marker_ids": [20, 21, 22]}\n')
        self.assertIn("error", self.svc.solve_session(path))


class RunLogTestCase(unittest.TestCase):
    """The run log is a plain CSV next to the session files — a handful
    of full-length runs a season, meant to be compared by eye (Ben,
    2026-09-23)."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.svc = _service(self.tmp)
        self.svc.camera_cal_fn = lambda: (sim.CAMERA_MATRIX, sim.DIST_COEFFS)
        self.session_name = "260923_120000.jsonl"
        _write_session(self.svc.data_dir / self.session_name)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_empty_before_anything_is_logged(self):
        self.assertEqual(self.svc.list_runs(), [])

    def test_logging_a_real_session_appends_a_row(self):
        row = self.svc.log_run(self.session_name, notes="clean run")
        self.assertNotIn("error", row)
        self.assertEqual(row["session"], self.session_name)
        self.assertEqual(row["notes"], "clean run")
        self.assertGreater(row["n_obs"], 100)
        self.assertGreater(row["rotations"], 0)

        rows = self.svc.list_runs()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["session"], self.session_name)

    def test_appends_rather_than_overwrites(self):
        self.svc.log_run(self.session_name)
        self.svc.log_run(self.session_name)
        self.assertEqual(len(self.svc.list_runs()), 2)

    def test_timestamp_is_the_logging_time_not_the_session_time(self):
        """Two different things: when the data was captured (the session
        filename) vs when it was judged worth recording (the log row) —
        keeping both distinguishes a same-day re-solve from a new run."""
        row = self.svc.log_run(self.session_name)
        self.assertNotEqual(row["timestamp"], "260923_120000")
        self.assertRegex(row["timestamp"], r"^\d{6}_\d{6}$")

    def test_refuses_a_session_that_does_not_exist(self):
        out = self.svc.log_run("no-such-session.jsonl")
        self.assertIn("error", out)
        self.assertEqual(self.svc.list_runs(), [])

    def test_csv_survives_a_round_trip_on_disk(self):
        """Not just self.svc's in-memory view — a fresh read of the file
        itself, the way Ben opening it later actually would."""
        self.svc.log_run(self.session_name)
        with (self.svc.data_dir / "runs.csv").open(newline="") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 1)
        self.assertEqual(set(service.BoresightService.RUN_LOG_FIELDS), set(rows[0].keys()))


class BuildCaptureJobLeadInTestCase(unittest.TestCase):
    """build_capture_job must save the lead-in as its own separate job,
    never folded into the job that repeats every pass. Folding it in
    worked for pass 1 and then failed navigate's entry check on every
    pass after, because the saved approach leg still targeted the
    position the robot was in before pass 1 (found live 2026-09-23)."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        (self.tmp / "paths").mkdir()
        shutil.copy2(REAL_BOX, self.tmp / "paths" / "Boresight limits.yaml")
        box = yaml.safe_load(REAL_BOX.read_text())
        # A corner of the recorded boundary is metres from any fan leg
        # (all within R_FAR_M=3.5m of the panel), so a lead-in is
        # guaranteed to be needed from here.
        self.far_point = box[0]

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _svc_at(self, lat, lon, heading_deg=0.0):
        import math
        svc = _service(self.tmp, nav_payload={
            "nav": {"Lat": math.radians(lat), "Lon": math.radians(lon), "Heading": heading_deg},
            "connection": {}})
        svc.driver = _FakeDriver()
        return svc

    def test_saves_core_and_approach_as_two_separate_jobs(self):
        svc = self._svc_at(self.far_point["lat"], self.far_point["lon"])
        built = svc.build_capture_job("Boresight limits", layouts.RECOMMENDED_HEIGHT_M, layouts.INS_HEIGHT_M)
        self.assertTrue(built["has_lead_in"])
        self.assertEqual([s["job_name"] for s in svc.driver.saved],
                         [drive_mod.JOB_NAME, drive_mod.APPROACH_JOB_NAME])

    def test_core_job_does_not_contain_the_approach_leg(self):
        svc = self._svc_at(self.far_point["lat"], self.far_point["lon"])
        svc.build_capture_job("Boresight limits", layouts.RECOMMENDED_HEIGHT_M, layouts.INS_HEIGHT_M)
        core = svc.driver.saved[0]
        self.assertEqual(core["job_name"], drive_mod.JOB_NAME)
        self.assertNotIn(f"{drive_mod.LEG_PATH_PREFIX} approach", [name for name, _ in core["legs"]])

    def test_approach_job_has_no_numbered_fan_leg(self):
        svc = self._svc_at(self.far_point["lat"], self.far_point["lon"])
        svc.build_capture_job("Boresight limits", layouts.RECOMMENDED_HEIGHT_M, layouts.INS_HEIGHT_M)
        approach = svc.driver.saved[1]
        self.assertEqual(approach["job_name"], drive_mod.APPROACH_JOB_NAME)
        run_paths = [s["path"] for s in approach["steps"] if s["type"] == "run_path"]
        self.assertTrue(run_paths)
        for p in run_paths:
            self.assertEqual(p, f"{drive_mod.LEG_PATH_PREFIX} approach")

    def test_no_lead_in_saved_without_a_position_fix(self):
        svc = _service(self.tmp, nav_payload={"nav": {}, "connection": {}})
        svc.driver = _FakeDriver()
        built = svc.build_capture_job("Boresight limits", layouts.RECOMMENDED_HEIGHT_M, layouts.INS_HEIGHT_M)
        self.assertFalse(built["has_lead_in"])
        self.assertEqual(len(svc.driver.saved), 1)
        self.assertEqual(svc.driver.saved[0]["job_name"], drive_mod.JOB_NAME)


class RunLoopSequencingTestCase(unittest.TestCase):
    """The approach must be driven exactly once, before any pass — not
    repeated, and not skipped when it's actually needed."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.svc = _service(self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_approach_runs_once_then_every_pass_uses_the_core_job(self):
        self.svc.driver = _FakeDriver()
        self.svc._run_loop(passes=3, has_lead_in=True)
        self.assertEqual(self.svc.driver.started,
                         [drive_mod.APPROACH_JOB_NAME, drive_mod.JOB_NAME,
                          drive_mod.JOB_NAME, drive_mod.JOB_NAME])
        self.assertEqual(self.svc.run_state["state"], "done")

    def test_no_approach_step_when_none_was_built(self):
        self.svc.driver = _FakeDriver()
        self.svc._run_loop(passes=2, has_lead_in=False)
        self.assertEqual(self.svc.driver.started, [drive_mod.JOB_NAME, drive_mod.JOB_NAME])

    def test_a_failed_approach_blocks_before_any_pass_runs(self):
        self.svc.driver = _FakeDriver(idle_results=[{"state": "aborted", "abort_reason": "no segment within entry tolerance"}])
        self.svc._run_loop(passes=4, has_lead_in=True)
        self.assertEqual(self.svc.driver.started, [drive_mod.APPROACH_JOB_NAME])  # no pass ever started
        self.assertEqual(self.svc.run_state["state"], "blocked")
        self.assertIn("approach failed", self.svc.run_state["message"])
        self.assertEqual(self.svc.run_state["passes_done"], 0)


class DetectionRateTestCase(unittest.TestCase):
    """Capture must respect the same rate cap aruco's own detection loop
    uses. Uncapped it ran at the camera's 5Hz *alongside* that loop — ~7Hz
    of ArUco detection on a Pi — and bought nothing, since INS error is
    correlated over tens of seconds."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_period_comes_from_the_configured_rate(self):
        svc = service.BoresightService(
            cfg={}, nav_client=_FakeNavClient({"nav": {}, "connection": {}}),
            frame_reader_fn=lambda: None, detect_fn=lambda *a: [], marker_size=0.097,
            hpr_cb=(0, 0, 0), dxc_b=(0, 0, 0), camera_cal_fn=lambda: (None, None),
            paths_dir=self.tmp, data_dir=self.tmp, max_detection_hz=2.0)
        self.assertAlmostEqual(svc.min_frame_period_s, 0.5)

    def test_zero_means_uncapped(self):
        svc = service.BoresightService(
            cfg={}, nav_client=_FakeNavClient({"nav": {}, "connection": {}}),
            frame_reader_fn=lambda: None, detect_fn=lambda *a: [], marker_size=0.097,
            hpr_cb=(0, 0, 0), dxc_b=(0, 0, 0), camera_cal_fn=lambda: (None, None),
            paths_dir=self.tmp, data_dir=self.tmp, max_detection_hz=0)
        self.assertEqual(svc.min_frame_period_s, 0.0)


class ContinuityWarningFilterTestCase(unittest.TestCase):
    """jobs checks continuity between each run_path and the NEXT one,
    skipping whatever lies between. That's right for pause/water steps
    but wrong for turn_to_heading, whose whole purpose is to change the
    heading — so every leg of this pattern raised a ~180 degree warning
    and buried anything real."""

    STEPS = [
        {"type": "run_path", "path": "A"},
        {"type": "turn_to_heading", "heading_deg": 10.0},
        {"type": "run_path", "path": "B"},
        {"type": "run_path", "path": "C"},
    ]

    def test_drops_a_heading_warning_a_turn_step_already_answers(self):
        warnings = [{"after_step": 0, "before_step": 2, "distance_m": 0.0,
                     "heading_error_deg": 178.0, "ok": False}]
        self.assertEqual(service._continuity_warnings_worth_showing(warnings, self.STEPS), [])

    def test_keeps_a_real_positional_gap_even_across_a_turn(self):
        """A turn fixes heading, not distance — a metres-apart join is
        still a genuine problem."""
        warnings = [{"after_step": 0, "before_step": 2, "distance_m": 4.4,
                     "heading_error_deg": 178.0, "ok": False}]
        self.assertEqual(len(service._continuity_warnings_worth_showing(warnings, self.STEPS)), 1)

    def test_keeps_a_warning_with_no_turn_between(self):
        warnings = [{"after_step": 2, "before_step": 3, "distance_m": 0.0,
                     "heading_error_deg": 120.0, "ok": False}]
        self.assertEqual(len(service._continuity_warnings_worth_showing(warnings, self.STEPS)), 1)


if __name__ == "__main__":
    unittest.main()
