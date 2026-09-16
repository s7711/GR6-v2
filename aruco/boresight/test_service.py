"""Tests for the bore-sight service glue.

The config writer gets the most attention: it edits the live config.yaml,
so it must never mangle a file it doesn't fully understand, and it must
preserve the comments that carry this project's tuning history.
"""

import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import layouts  # noqa: E402
import service  # noqa: E402

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


def _service(tmp, nav_payload=None, paths_dir=None):
    return service.BoresightService(
        cfg={}, nav_client=_FakeNavClient(nav_payload or {"nav": {}, "connection": {}}),
        frame_reader_fn=lambda: None, detect_fn=lambda *a: [],
        marker_size=0.097, hpr_cb=(0.0, 0.0, 0.0), dxc_b=(0.0775, 0.002, -0.07),
        camera_cal_fn=lambda: (None, None),
        paths_dir=paths_dir or (tmp / "paths"), data_dir=tmp / "data",
    )


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
