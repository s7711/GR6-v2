import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import recovery


def _completed(returncode=0, stdout=""):
    return subprocess.CompletedProcess(args=[], returncode=returncode, stdout=stdout, stderr="")


class _IsolatedRecoveryFile(unittest.TestCase):
    """Every test here patches recovery.RECOVERY_FILE to a throwaway
    temp path — never the real one. Found the hard way: an earlier
    version of these tests deleted the operator's actual recovery.json
    (a real, currently-in-use safety net) via an unlink-in-addCleanup
    that assumed it only ever pointed at test data."""

    def setUp(self):
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        patcher = patch.object(recovery, "RECOVERY_FILE", Path(tmpdir.name) / "recovery.json")
        self.addCleanup(patcher.stop)
        patcher.start()


class TestLoadSetRecovery(_IsolatedRecoveryFile):
    def test_no_file_returns_empty(self):
        self.assertEqual(recovery.load_recovery(), {})

    def test_set_then_load_round_trips(self):
        recovery.set_recovery("wlan0", "preconfigured")
        self.assertEqual(recovery.load_recovery(), {"wlan0": "preconfigured"})

    def test_set_none_removes_entry(self):
        recovery.set_recovery("wlan0", "preconfigured")
        recovery.set_recovery("wlan0", None)
        self.assertEqual(recovery.load_recovery(), {})

    def test_multiple_devices_independent(self):
        recovery.set_recovery("wlan0", "preconfigured")
        recovery.set_recovery("eth0", "Wired connection 1")
        self.assertEqual(
            recovery.load_recovery(), {"wlan0": "preconfigured", "eth0": "Wired connection 1"}
        )

    def test_none_sentinel_stored_as_is(self):
        recovery.set_recovery("wlan0", recovery.NONE)
        self.assertEqual(recovery.load_recovery(), {"wlan0": recovery.NONE})

    def test_refuses_same_profile_as_another_devices_recovery(self):
        recovery.set_recovery("wlan0", "CoffeebeanWifi")
        with self.assertRaises(recovery.RecoveryConflict):
            recovery.set_recovery("wlan1", "CoffeebeanWifi")
        # the conflicting set must not have partially applied
        self.assertEqual(recovery.load_recovery(), {"wlan0": "CoffeebeanWifi"})

    def test_re_setting_the_same_device_to_the_same_profile_is_not_a_conflict(self):
        recovery.set_recovery("wlan0", "CoffeebeanWifi")
        recovery.set_recovery("wlan0", "CoffeebeanWifi")  # must not raise
        self.assertEqual(recovery.load_recovery(), {"wlan0": "CoffeebeanWifi"})


class TestRevertNow(_IsolatedRecoveryFile):
    def test_reactivates_every_stored_connection(self):
        recovery.set_recovery("wlan0", "preconfigured")
        recovery.set_recovery("eth0", "Wired connection 1")
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            recovery.revert_now()
        calls = [call.args[0] for call in mock_run.call_args_list]
        self.assertTrue(any("preconfigured" in c and "wlan0" in c for c in calls))
        self.assertTrue(any("Wired connection 1" in c and "eth0" in c for c in calls))

    def test_devices_marked_dont_care_are_skipped(self):
        recovery.set_recovery("wlan0", "preconfigured")
        # eth0 never set — "don't care", per network-prd.md
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            recovery.revert_now()
        self.assertEqual(mock_run.call_count, 1)

    def test_one_failing_device_does_not_stop_the_others(self):
        recovery.set_recovery("wlan0", "preconfigured")
        recovery.set_recovery("eth0", "Wired connection 1")
        with patch("subprocess.run", return_value=_completed(returncode=1)):
            recovery.revert_now()  # must not raise

    def test_none_sentinel_disconnects_instead_of_activating(self):
        recovery.set_recovery("wlan0", recovery.NONE)
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            recovery.revert_now()
        calls = [call.args[0] for call in mock_run.call_args_list]
        self.assertTrue(any("disconnect" in c and "wlan0" in c for c in calls))


class TestScheduleCancel(unittest.TestCase):
    def test_schedule_cancels_any_existing_timer_first(self):
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            recovery.schedule_revert(30)
        called_units = [call.args[0] for call in mock_run.call_args_list]
        self.assertTrue(any("systemctl" in c and "stop" in c for c in called_units))
        self.assertTrue(any("systemd-run" in c for c in called_units))

    def test_revert_pending_reads_timer_state(self):
        with patch("subprocess.run", return_value=_completed(stdout="active\n")):
            self.assertTrue(recovery.revert_pending())
        with patch("subprocess.run", return_value=_completed(stdout="inactive\n")):
            self.assertFalse(recovery.revert_pending())


if __name__ == "__main__":
    unittest.main()
