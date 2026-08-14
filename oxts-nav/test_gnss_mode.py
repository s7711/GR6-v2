"""Tests for gnss_mode.GnssModeController's pure logic — the actual
aruco websocket connection isn't exercised here (same testing
philosophy as waterbutt's own _qc_loop: the network loop itself isn't
unit tested, just what it drives). _watchdog_tick() is called directly
rather than through _watchdog_loop's infinite sleep loop.
"""

import time
import unittest

import gnss_mode


class FakeCommandSender:
    def __init__(self):
        self.commands = []

    def __call__(self, message):
        self.commands.append(message)


class GnssModeControllerTestCase(unittest.TestCase):
    def setUp(self):
        self.sender = FakeCommandSender()
        self.timeout_s = 3.0
        self.controller = gnss_mode.GnssModeController(self.sender, "ws://unused", self.timeout_s)

    def test_starts_in_normal_mode(self):
        self.assertEqual(self.controller.status()["mode"], "normal")
        self.assertEqual(self.sender.commands, [])

    def test_enter_aruco_priority_sends_disable_and_sets_mode(self):
        self.controller.enter_aruco_priority()
        self.assertEqual(self.sender.commands, ["!disable gnss"])
        self.assertEqual(self.controller.status()["mode"], "aruco_priority")

    def test_exit_aruco_priority_sends_enable_and_restores_normal(self):
        self.controller.enter_aruco_priority()
        self.controller.exit_aruco_priority()
        self.assertEqual(self.sender.commands, ["!disable gnss", "!enable gnss"])
        self.assertEqual(self.controller.status()["mode"], "normal")

    def test_exit_when_already_normal_is_a_no_op(self):
        self.controller.exit_aruco_priority()
        self.assertEqual(self.sender.commands, [])

    def test_watchdog_does_nothing_in_normal_mode(self):
        self.controller._watchdog_tick()
        self.assertEqual(self.sender.commands, [])
        self.assertEqual(self.controller.status()["mode"], "normal")

    def test_watchdog_does_nothing_while_marker_recently_seen(self):
        self.controller.enter_aruco_priority()
        self.controller._note_marker_seen()
        self.controller._watchdog_tick()
        self.assertEqual(self.controller.status()["mode"], "aruco_priority")

    def test_watchdog_falls_back_to_gnss_after_timeout(self):
        self.controller.enter_aruco_priority()
        # Simulate the grace/last-seen period having expired.
        self.controller._last_marker_seen_at = time.monotonic() - (self.timeout_s + 1)
        self.controller._watchdog_tick()
        self.assertEqual(self.controller.status()["mode"], "normal")
        self.assertEqual(self.sender.commands, ["!disable gnss", "!enable gnss"])

    def test_note_marker_seen_resets_the_watchdog_clock(self):
        self.controller.enter_aruco_priority()
        self.controller._last_marker_seen_at = time.monotonic() - (self.timeout_s + 1)
        self.controller._note_marker_seen()
        self.controller._watchdog_tick()
        self.assertEqual(self.controller.status()["mode"], "aruco_priority")


if __name__ == "__main__":
    unittest.main()
