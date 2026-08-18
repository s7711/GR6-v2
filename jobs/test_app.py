"""End-to-end tests against the Flask app — routes, job control
wiring, save-time continuity warnings. Never makes a real HTTP call to
navigate: app.runner is replaced with a fresh JobRunner built over
recording stubs (see test_control.py for JobRunner's own
behaviour). app.JOBS_DIR/app.NAVIGATE_PATHS_DIR point at temp
directories so tests never touch this robot's real saved jobs/paths.
"""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import requests

import app
import jobs as jobs_module
from control import JobRunner

import paths as navigate_paths  # noqa: E402 - navigate/ already on sys.path, added by importing app above

SAMPLE_STEPS = [
    {"type": "run_path", "path": "loop-a"},
    {"type": "run_path", "path": "loop-b"},
]

SAMPLE_PATH_POINTS = [
    {"lat": 52.2, "lon": -1.5, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
    {"lat": 52.2005, "lon": -1.5, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
]


class NavigateStub:
    def __init__(self):
        self.loaded = []
        self.start_calls = 0
        self.stop_calls = 0
        self.load_result = {"ok": True}
        self.start_result = {"ok": True}
        self._status = {"state": "idle", "abort_reason": None}

    def load_path(self, name):
        self.loaded.append(name)
        return self.load_result

    def start_path(self):
        self.start_calls += 1
        return self.start_result

    def start_turn(self, heading_deg, tolerance_deg):
        return {"ok": True}

    def stop_path(self):
        self.stop_calls += 1

    def navigate_status(self):
        return self._status

    def set_status(self, state, abort_reason=None):
        self._status = {"state": state, "abort_reason": abort_reason}

    def pump_on(self, on):
        return {"ok": True}

    def waterbutt_go(self, duration_s):
        return {"ok": True}

    def waterbutt_stop(self):
        pass


class JobsAppTestCase(unittest.TestCase):
    def setUp(self):
        app.JOBS_DIR = Path(tempfile.mkdtemp())
        app.LOGS_DIR = app.JOBS_DIR / "logs"
        app.NAVIGATE_PATHS_DIR = Path(tempfile.mkdtemp())
        self.stub = NavigateStub()
        app.runner = JobRunner(
            self.stub.load_path, self.stub.start_path, self.stub.stop_path, self.stub.start_turn,
            self.stub.navigate_status, self.stub.pump_on, self.stub.waterbutt_go, self.stub.waterbutt_stop,
        )
        self.client = app.app.test_client()

    def test_list_jobs_empty(self):
        self.assertEqual(self.client.get("/api/jobs").get_json(), [])

    def test_save_get_delete_job(self):
        resp = self.client.post("/api/jobs/front-beds", json={"steps": SAMPLE_STEPS})
        self.assertEqual(resp.get_json(), {"warnings": []})

        loaded = self.client.get("/api/jobs/front-beds").get_json()
        self.assertEqual(loaded["steps"], SAMPLE_STEPS)

        self.assertEqual(self.client.delete("/api/jobs/front-beds").status_code, 204)
        self.assertEqual(self.client.get("/api/jobs").get_json(), [])

    def test_get_missing_job_404(self):
        self.assertEqual(self.client.get("/api/jobs/does-not-exist").status_code, 404)

    def test_save_warns_on_a_discontinuous_pair(self):
        navigate_paths.save_path(app.NAVIGATE_PATHS_DIR, "loop-a", SAMPLE_PATH_POINTS)
        # loop-b starts miles away from loop-a's end — a genuine gap.
        far_points = [
            {"lat": 10.0, "lon": 10.0, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
            {"lat": 10.001, "lon": 10.0, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
        ]
        navigate_paths.save_path(app.NAVIGATE_PATHS_DIR, "loop-b", far_points)

        resp = self.client.post("/api/jobs/front-beds", json={"steps": SAMPLE_STEPS})
        warnings = resp.get_json()["warnings"]
        self.assertEqual(len(warnings), 1)
        self.assertEqual(warnings[0]["from_path"], "loop-a")
        self.assertEqual(warnings[0]["to_path"], "loop-b")

    def test_save_warns_on_a_discontinuous_pair_separated_by_a_pause(self):
        # A pause (or any non-run_path step) between two run_path steps
        # doesn't move the robot, so continuity should still be checked
        # across it - found live 2026-08-10 on "Water kitchen bed",
        # where exactly this shape (run_path, pause, run_path) hid a
        # real ~4.4m/143deg discontinuity until the job aborted live.
        navigate_paths.save_path(app.NAVIGATE_PATHS_DIR, "loop-a", SAMPLE_PATH_POINTS)
        far_points = [
            {"lat": 10.0, "lon": 10.0, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
            {"lat": 10.001, "lon": 10.0, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5},
        ]
        navigate_paths.save_path(app.NAVIGATE_PATHS_DIR, "loop-b", far_points)
        steps = [
            {"type": "run_path", "path": "loop-a"},
            {"type": "pause", "duration_s": 20},
            {"type": "run_path", "path": "loop-b"},
        ]

        resp = self.client.post("/api/jobs/front-beds", json={"steps": steps})
        warnings = resp.get_json()["warnings"]
        self.assertEqual(len(warnings), 1)
        self.assertEqual(warnings[0]["after_step"], 0)
        self.assertEqual(warnings[0]["before_step"], 2)
        self.assertEqual(warnings[0]["from_path"], "loop-a")
        self.assertEqual(warnings[0]["to_path"], "loop-b")

    def test_save_does_not_warn_when_the_last_run_path_has_nothing_after_it(self):
        # A trailing pause/water step after the final run_path step has
        # no "next run_path" to check against - should be skipped, not
        # crash on `j is None`.
        navigate_paths.save_path(app.NAVIGATE_PATHS_DIR, "loop-a", SAMPLE_PATH_POINTS)
        steps = [
            {"type": "run_path", "path": "loop-a"},
            {"type": "pause", "duration_s": 10},
        ]
        resp = self.client.post("/api/jobs/front-beds", json={"steps": steps})
        self.assertEqual(resp.get_json(), {"warnings": []})

    def test_control_start_loads_and_starts_first_step(self):
        jobs_module.save_job(app.JOBS_DIR, "front-beds", SAMPLE_STEPS)
        resp = self.client.post("/control/start", json={"name": "front-beds"})
        self.assertEqual(resp.get_json()["state"], "running")
        self.assertEqual(self.stub.loaded, ["loop-a"])

    def test_control_start_can_resume_from_a_step(self):
        jobs_module.save_job(app.JOBS_DIR, "front-beds", SAMPLE_STEPS)
        resp = self.client.post("/control/start", json={"name": "front-beds", "start_index": 1})
        self.assertEqual(resp.get_json()["current_step_index"], 1)
        self.assertEqual(self.stub.loaded, ["loop-b"])

    def test_control_start_missing_job_404(self):
        resp = self.client.post("/control/start", json={"name": "does-not-exist"})
        self.assertEqual(resp.status_code, 404)

    def test_control_status(self):
        # Plain synchronous status - see app.py's own comment: missions
        # polls this directly rather than needing a push feed.
        jobs_module.save_job(app.JOBS_DIR, "front-beds", SAMPLE_STEPS)
        self.client.post("/control/start", json={"name": "front-beds"})
        resp = self.client.get("/control/status")
        self.assertEqual(resp.get_json()["state"], "running")

    def test_control_stop(self):
        jobs_module.save_job(app.JOBS_DIR, "front-beds", SAMPLE_STEPS)
        self.client.post("/control/start", json={"name": "front-beds"})
        resp = self.client.post("/control/stop")
        self.assertEqual(resp.status_code, 204)
        self.assertEqual(app.runner.status()["state"], "idle")
        self.assertEqual(self.stub.stop_calls, 1)

    def test_pages_render(self):
        for path in ["/", "/pages/jobs", "/pages/create-job"]:
            resp = self.client.get(path)
            self.assertEqual(resp.status_code, 200, path)

    def test_navigate_paths_proxies_to_navigate(self):
        fake_response = requests.Response()
        fake_response.status_code = 200
        fake_response._content = b'[{"name": "loop-a", "point_count": 2, "length_m": 5.0}]'
        with patch.object(app.requests, "get", return_value=fake_response) as mock_get:
            resp = self.client.get("/api/navigate-paths")
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.get_json(), [{"name": "loop-a", "point_count": 2, "length_m": 5.0}])
        mock_get.assert_called_once_with(f"{app.NAVIGATE_BASE_URL}/api/paths", timeout=app.NAVIGATE_TIMEOUT_S)

    def test_navigate_paths_returns_502_if_navigate_unreachable(self):
        with patch.object(app.requests, "get", side_effect=requests.exceptions.ConnectionError):
            resp = self.client.get("/api/navigate-paths")
        self.assertEqual(resp.status_code, 502)


if __name__ == "__main__":
    unittest.main()
