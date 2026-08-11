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


if __name__ == "__main__":
    unittest.main()
