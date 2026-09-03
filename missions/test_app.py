"""End-to-end tests against the Flask app — routes, mission control
wiring. Never makes a real HTTP call to jobs: app.runner is replaced
with a fresh MissionRunner built over stub callables (see
test_control.py for MissionRunner's own behaviour). app.MISSIONS_DIR
points at a temp directory so tests never touch this robot's real
saved missions.
"""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import requests

import app
import missions as missions_module
from control import MissionRunner

SAMPLE_STEPS = [
    {"type": "run_job", "job": "Water kitchen bed"},
    {"type": "run_job", "job": "Water stable bed"},
]


class JobsStub:
    def __init__(self):
        self.started = []
        self.stop_calls = 0
        self.start_result = {"ok": True}
        self._status = {"state": "idle", "abort_reason": None}

    def start_job(self, name):
        self.started.append(name)
        return self.start_result

    def stop_job(self):
        self.stop_calls += 1

    def job_status(self):
        return self._status

    def set_status(self, state, abort_reason=None):
        self._status = {"state": state, "abort_reason": abort_reason}


class MissionsAppTestCase(unittest.TestCase):
    def setUp(self):
        app.MISSIONS_DIR = Path(tempfile.mkdtemp())
        app.LOGS_DIR = app.MISSIONS_DIR / "logs"
        self.stub = JobsStub()
        app.runner = MissionRunner(self.stub.start_job, self.stub.stop_job, self.stub.job_status)
        self.client = app.app.test_client()

    def test_list_missions_empty(self):
        self.assertEqual(self.client.get("/api/missions").get_json(), [])

    def test_save_get_delete_mission(self):
        resp = self.client.post("/api/missions/flowerbeds", json={"steps": SAMPLE_STEPS})
        self.assertEqual(resp.get_json(), {"warnings": []})

        loaded = self.client.get("/api/missions/flowerbeds").get_json()
        self.assertEqual(loaded["steps"], SAMPLE_STEPS)

        self.assertEqual(self.client.delete("/api/missions/flowerbeds").status_code, 204)
        self.assertEqual(self.client.get("/api/missions").get_json(), [])

    def test_get_missing_mission_404(self):
        self.assertEqual(self.client.get("/api/missions/does-not-exist").status_code, 404)

    def test_control_start_starts_first_step(self):
        missions_module.save_mission(app.MISSIONS_DIR, "flowerbeds", SAMPLE_STEPS)
        resp = self.client.post("/control/start", json={"name": "flowerbeds"})
        self.assertEqual(resp.get_json()["state"], "running")
        self.assertEqual(self.stub.started, ["Water kitchen bed"])

    def test_control_start_missing_mission_404(self):
        resp = self.client.post("/control/start", json={"name": "does-not-exist"})
        self.assertEqual(resp.status_code, 404)

    def test_control_start_with_no_steps_refuses(self):
        missions_module.save_mission(app.MISSIONS_DIR, "empty", [])
        resp = self.client.post("/control/start", json={"name": "empty"})
        data = resp.get_json()
        self.assertFalse(data["ok"])
        self.assertIn("no steps", data["reason"])

    def test_control_start_can_resume_from_a_later_step(self):
        missions_module.save_mission(app.MISSIONS_DIR, "flowerbeds", SAMPLE_STEPS)
        resp = self.client.post("/control/start", json={"name": "flowerbeds", "start_index": 1})
        self.assertEqual(resp.get_json()["current_step_index"], 1)
        self.assertEqual(self.stub.started, ["Water stable bed"])

    def test_control_start_invalid_start_index_refuses(self):
        missions_module.save_mission(app.MISSIONS_DIR, "flowerbeds", SAMPLE_STEPS)
        resp = self.client.post("/control/start", json={"name": "flowerbeds", "start_index": 5})
        data = resp.get_json()
        self.assertFalse(data["ok"])
        self.assertIn("invalid start_index", data["reason"])

    def test_control_stop(self):
        missions_module.save_mission(app.MISSIONS_DIR, "flowerbeds", SAMPLE_STEPS)
        self.client.post("/control/start", json={"name": "flowerbeds"})
        resp = self.client.post("/control/stop")
        self.assertEqual(resp.status_code, 204)
        self.assertEqual(self.stub.stop_calls, 1)
        self.assertEqual(app.runner.status()["state"], "idle")

    def test_control_status(self):
        missions_module.save_mission(app.MISSIONS_DIR, "flowerbeds", SAMPLE_STEPS)
        self.client.post("/control/start", json={"name": "flowerbeds"})
        resp = self.client.get("/control/status")
        self.assertEqual(resp.get_json()["state"], "running")


class FakeJobsResponse:
    """Stand-in for requests.post's return value - just enough for
    start_job()'s resp.json() call."""

    def __init__(self, data):
        self._data = data

    def json(self):
        return self._data


class StartJobTestCase(unittest.TestCase):
    """Tests app.start_job() itself - the thin wrapper around jobs'
    /control/start that MissionRunner is built over in production (the
    tests above replace it with JobsStub, so never exercise the real
    thing). Found live 2026-09-03: a single-step job whose only step is
    legitimately skipped (e.g. a "fill" refused by waterbutt) finishes
    synchronously inside jobs' own go() call, so jobs' /control/start
    response already carries state "stopped_ok" rather than "running" -
    this used to be treated as a failed start, aborting the whole
    mission over what jobs itself considers a normal, non-fatal outcome.
    """

    def test_running_is_a_successful_start(self):
        with patch.object(app.requests, "post", return_value=FakeJobsResponse({"state": "running"})):
            self.assertEqual(app.start_job("Water kitchen bed"), {"ok": True})

    def test_stopped_ok_is_a_successful_start_not_a_failure(self):
        with patch.object(app.requests, "post", return_value=FakeJobsResponse({"state": "stopped_ok"})):
            self.assertEqual(app.start_job("Fill with water"), {"ok": True})

    def test_aborted_surfaces_the_jobs_abort_reason_as_a_failure(self):
        response = FakeJobsResponse({"state": "aborted", "abort_reason": "couldn't reach navigate"})
        with patch.object(app.requests, "post", return_value=response):
            self.assertEqual(
                app.start_job("Water kitchen bed"),
                {"ok": False, "reason": "couldn't reach navigate"},
            )

    def test_refused_before_starting_is_passed_through(self):
        response = FakeJobsResponse({"ok": False, "reason": "no such job"})
        with patch.object(app.requests, "post", return_value=response):
            self.assertEqual(app.start_job("nonexistent"), {"ok": False, "reason": "no such job"})

    def test_unreachable_jobs_is_a_failure(self):
        with patch.object(app.requests, "post", side_effect=requests.exceptions.ConnectionError):
            self.assertEqual(app.start_job("Water kitchen bed"), {"ok": False, "reason": "couldn't reach jobs"})


if __name__ == "__main__":
    unittest.main()
