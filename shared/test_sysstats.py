import subprocess
import unittest
import unittest.mock
from unittest.mock import patch

import sysstats


class TestBrownout(unittest.TestCase):
    def setUp(self):
        sysstats._last_brownout_monotonic = None
        sysstats._prev_since_boot = False

    def test_never_happened(self):
        with patch.object(subprocess, "run", return_value=_completed("throttled=0x0")), \
             patch.object(sysstats.time, "monotonic", return_value=100.0):
            self.assertEqual(sysstats.read_brownout(), {"active": False, "age_seconds": None})

    def test_currently_active_has_zero_age(self):
        with patch.object(subprocess, "run", return_value=_completed("throttled=0x1")), \
             patch.object(sysstats.time, "monotonic", return_value=100.0):
            result = sysstats.read_brownout()
        self.assertEqual(result, {"active": True, "age_seconds": 0.0})

    def test_age_grows_once_event_clears(self):
        with patch.object(subprocess, "run", return_value=_completed("throttled=0x1")), \
             patch.object(sysstats.time, "monotonic", return_value=100.0):
            sysstats.read_brownout()
        with patch.object(subprocess, "run", return_value=_completed("throttled=0x0")), \
             patch.object(sysstats.time, "monotonic", return_value=145.0):
            result = sysstats.read_brownout()
        self.assertEqual(result, {"active": False, "age_seconds": 45.0})

    def test_since_boot_transition_counts_as_a_missed_short_dip(self):
        # A dip shorter than the poll interval: the active bit is already
        # clear again by the time we poll, but the sticky since_boot bit
        # newly appeared — that should still register as a recent event.
        with patch.object(subprocess, "run", return_value=_completed("throttled=0x0")), \
             patch.object(sysstats.time, "monotonic", return_value=100.0):
            sysstats.read_brownout()
        with patch.object(subprocess, "run", return_value=_completed("throttled=0x10000")), \
             patch.object(sysstats.time, "monotonic", return_value=110.0):
            result = sysstats.read_brownout()
        self.assertEqual(result, {"active": False, "age_seconds": 0.0})

    def test_since_boot_already_set_does_not_keep_resetting_age(self):
        with patch.object(subprocess, "run", return_value=_completed("throttled=0x10000")), \
             patch.object(sysstats.time, "monotonic", return_value=100.0):
            sysstats.read_brownout()
        with patch.object(subprocess, "run", return_value=_completed("throttled=0x10000")), \
             patch.object(sysstats.time, "monotonic", return_value=130.0):
            result = sysstats.read_brownout()
        self.assertEqual(result, {"active": False, "age_seconds": 30.0})

    def test_vcgencmd_missing(self):
        with patch.object(subprocess, "run", side_effect=FileNotFoundError), \
             patch.object(sysstats.time, "monotonic", return_value=100.0):
            self.assertEqual(sysstats.read_brownout(), {"active": False, "age_seconds": None})


def _completed(stdout):
    return subprocess.CompletedProcess(args=[], returncode=0, stdout=stdout)


class TestWifiBars(unittest.TestCase):
    def test_no_wireless_interface(self):
        fake_path = unittest.mock.MagicMock()
        fake_path.read_text.side_effect = FileNotFoundError
        with patch.object(sysstats, "_WIRELESS_PROC", fake_path):
            self.assertIsNone(sysstats.read_wifi_bars())

    def test_full_quality(self):
        text = (
            "Inter-| sta-|   Quality        |   Discarded packets\n"
            " face | tus | link level noise |  nwid  crypt   frag  retry   misc\n"
            "wlan0: 0000   70.  -40.  -256        0      0      0      0      0\n"
        )
        fake_path = unittest.mock.MagicMock()
        fake_path.read_text.return_value = text
        with patch.object(sysstats, "_WIRELESS_PROC", fake_path):
            self.assertEqual(sysstats.read_wifi_bars(), 5)

    def test_low_quality_clamped_to_one(self):
        text = (
            "Inter-| sta-|   Quality        |\n"
            " face | tus | link level noise |\n"
            "wlan0: 0000    2.  -90.  -256\n"
        )
        fake_path = unittest.mock.MagicMock()
        fake_path.read_text.return_value = text
        with patch.object(sysstats, "_WIRELESS_PROC", fake_path):
            self.assertEqual(sysstats.read_wifi_bars(), 1)


class TestCpuPercent(unittest.TestCase):
    def setUp(self):
        sysstats._prev_cpu_total = None
        sysstats._prev_cpu_idle = None

    def test_first_call_returns_none(self):
        with patch("builtins.open", _fake_proc_stat("cpu 100 0 0 900 0 0 0 0 0 0")):
            self.assertIsNone(sysstats.read_cpu_percent())

    def test_second_call_computes_delta(self):
        with patch("builtins.open", _fake_proc_stat("cpu 100 0 0 900 0 0 0 0 0 0")):
            sysstats.read_cpu_percent()
        with patch("builtins.open", _fake_proc_stat("cpu 150 0 0 950 0 0 0 0 0 0")):
            # +50 busy, +50 idle over a +100 total delta -> 50% busy
            self.assertEqual(sysstats.read_cpu_percent(), 50.0)


def _fake_proc_stat(line):
    from unittest.mock import mock_open

    return mock_open(read_data=line + "\n")


if __name__ == "__main__":
    unittest.main()
