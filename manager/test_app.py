"""Tests for manager/app.py — currently just /power-off, the one route
with real side effects worth pinning down (arming drive's power cutoff,
then shelling out to `sudo shutdown`). Everything else in this app is
either a thin systemctl wrapper or page rendering, untested so far.
"""

import unittest
from unittest.mock import patch

import requests

import app


class RecordingPost:
    """Stand-in for requests.post - records every call, returns a fake
    204 response. Same shape as navigate/test_app.py's own version."""

    def __init__(self):
        self.calls = []

    def __call__(self, url, **kwargs):
        self.calls.append((url, kwargs))
        resp = requests.Response()
        resp.status_code = 204
        return resp


class PowerOffTestCase(unittest.TestCase):
    def setUp(self):
        self.client = app.app.test_client()

    def test_arms_drive_cutoff_then_shells_out_to_shutdown(self):
        post = RecordingPost()
        with patch.object(app, "services", return_value={"drive": {"port": 8005}}), \
             patch.object(app.requests, "post", post), \
             patch.object(app.subprocess, "run") as run:
            run.return_value.returncode = 0
            run.return_value.stderr = ""
            resp = self.client.post("/power-off")

        self.assertEqual(resp.status_code, 204)
        self.assertEqual(len(post.calls), 1)
        url, kwargs = post.calls[0]
        self.assertEqual(url, "http://localhost:8005/power-off")
        self.assertEqual(kwargs["json"], {"delay_s": app.POWER_OFF_DELAY_S})
        run.assert_called_once_with(["sudo", "shutdown", "-h", "now"], capture_output=True, text=True)

    def test_still_shuts_down_if_drive_is_unreachable(self):
        with patch.object(app, "services", return_value={"drive": {"port": 8005}}), \
             patch.object(app.requests, "post", side_effect=requests.exceptions.ConnectionError), \
             patch.object(app.subprocess, "run") as run:
            run.return_value.returncode = 0
            run.return_value.stderr = ""
            resp = self.client.post("/power-off")

        self.assertEqual(resp.status_code, 204)
        run.assert_called_once()


class SysstatsSamplerTestCase(unittest.TestCase):
    # Regression coverage for the 2026-09-10 fix: sysstats_snapshot()
    # (specifically its CPU-percent diff) used to be called directly by
    # every open /ws/system connection - with several browser tabs open,
    # each tab's own polling thread raced the others over
    # shared/sysstats.py's module-level "previous sample" globals, so
    # tabs disagreed and jumped around. Now there's exactly one caller
    # (_sample_sysstats_once, from manager's own background thread) and
    # every ws connection just reads the cached result.
    def test_caches_the_snapshot_for_ws_system_to_read(self):
        app._latest_sysstats = None
        with patch.object(app, "sysstats_snapshot", return_value={"cpu_percent": 12.3}):
            app._sample_sysstats_once()
        self.assertEqual(app._latest_sysstats, {"cpu_percent": 12.3})

    def test_only_calls_sysstats_snapshot_once_per_sample(self):
        with patch.object(app, "sysstats_snapshot", return_value={"cpu_percent": 1.0}) as snap:
            app._sample_sysstats_once()
        snap.assert_called_once()


if __name__ == "__main__":
    unittest.main()
