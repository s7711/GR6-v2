import queue
import unittest
from unittest.mock import patch

from scanner import _parse_scan, _scan_tick

# A trimmed real `iw dev wlan0 scan` excerpt (see the live 2026-09-08 test
# that this feature is based on) — four BSS blocks (one is the same
# Coffeebean mesh unit's 5GHz radio, same SSID, different BSSID/freq —
# see the real 10+-BSSID complaint that prompted the 2.4GHz-only filter),
# only fields _parse_scan actually reads.
_SAMPLE_OUTPUT = """\
BSS b4:fb:e4:c7:f1:96(on wlan0)
\tfreq: 2422
\tsignal: -64.00 dBm
\tSSID: Coffeebean
BSS 44:e9:dd:fe:3e:5b(on wlan0)
\tfreq: 2462
\tsignal: -59.00 dBm
\tSSID: BTHub5-Q35C
BSS b6:fb:e4:c8:f1:0d(on wlan0)
\tfreq: 2462
\tsignal: -85.00 dBm
\tSSID: Coffeebean
BSS b6:fb:e4:c8:f1:96(on wlan0)
\tfreq: 5200
\tsignal: -62.00 dBm
\tSSID: Coffeebean
"""


class TestParseScan(unittest.TestCase):
    def test_filters_to_given_ssids(self):
        result = _parse_scan(_SAMPLE_OUTPUT, ["Coffeebean"])
        self.assertEqual(result, {"b4:fb:e4:c7:f1:96": -64, "b6:fb:e4:c8:f1:0d": -85})

    def test_excludes_5ghz_even_when_ssid_matches(self):
        result = _parse_scan(_SAMPLE_OUTPUT, ["Coffeebean"])
        self.assertNotIn("b6:fb:e4:c8:f1:96", result)  # the 5GHz radio of an included AP

    def test_empty_filter_keeps_everything_on_2_4ghz_only(self):
        result = _parse_scan(_SAMPLE_OUTPUT, [])
        self.assertEqual(len(result), 3)
        self.assertIn("44:e9:dd:fe:3e:5b", result)
        self.assertNotIn("b6:fb:e4:c8:f1:96", result)

    def test_no_matching_ssid_gives_empty_dict(self):
        result = _parse_scan(_SAMPLE_OUTPUT, ["SomeOtherNetwork"])
        self.assertEqual(result, {})

    def test_signal_rounded_to_int(self):
        result = _parse_scan(_SAMPLE_OUTPUT, ["Coffeebean"])
        self.assertIsInstance(result["b4:fb:e4:c7:f1:96"], int)

    def test_empty_output(self):
        self.assertEqual(_parse_scan("", ["Coffeebean"]), {})


class NavClientStub:
    def __init__(self, nav=None):
        self._nav = nav or {}

    def latest(self):
        return {"nav": self._nav}


class TestScanTick(unittest.TestCase):
    def test_translates_per_ap_field_names_to_their_friendly_names(self):
        record_queue = queue.Queue()
        with patch("scanner.scan_device", return_value={"b4:fb:e4:c7:f1:96": -64, "aa:aa:aa:aa:aa:aa": -80}), \
             patch("scanner.nm.connected_wifi_bssid", return_value=None):
            _scan_tick("wlan0", ["Coffeebean"], NavClientStub(), {"b4:fb:e4:c7:f1:96": "Front hall"}, record_queue)
        record = record_queue.get()
        self.assertEqual(record["Front hall"], -64)
        self.assertEqual(record["aa:aa:aa:aa:aa:aa"], -80)  # unlisted - stays as its raw BSSID
        self.assertNotIn("b4:fb:e4:c7:f1:96", record)

    def test_records_connected_bssid_translated_to_its_friendly_name(self):
        record_queue = queue.Queue()
        with patch("scanner.scan_device", return_value={"b4:fb:e4:c7:f1:96": -64}), \
             patch("scanner.nm.connected_wifi_bssid", return_value="b4:fb:e4:c7:f1:96"):
            ok = _scan_tick("wlan0", ["Coffeebean"], NavClientStub(), {"b4:fb:e4:c7:f1:96": "Stables"}, record_queue)
        self.assertTrue(ok)
        self.assertEqual(record_queue.get()["connected_bssid"], "Stables")

    def test_records_raw_bssid_when_not_in_the_name_list(self):
        record_queue = queue.Queue()
        with patch("scanner.scan_device", return_value={"b4:fb:e4:c7:f1:96": -64}), \
             patch("scanner.nm.connected_wifi_bssid", return_value="b4:fb:e4:c7:f1:96"):
            _scan_tick("wlan0", ["Coffeebean"], NavClientStub(), {}, record_queue)
        self.assertEqual(record_queue.get()["connected_bssid"], "b4:fb:e4:c7:f1:96")

    def test_omits_connected_bssid_when_not_associated(self):
        record_queue = queue.Queue()
        with patch("scanner.scan_device", return_value={"b4:fb:e4:c7:f1:96": -64}), \
             patch("scanner.nm.connected_wifi_bssid", return_value=None):
            _scan_tick("wlan0", ["Coffeebean"], NavClientStub(), {}, record_queue)
        self.assertNotIn("connected_bssid", record_queue.get())

    def test_returns_false_and_queues_nothing_on_a_failed_scan(self):
        record_queue = queue.Queue()
        with patch("scanner.scan_device", side_effect=RuntimeError("boom")), \
             patch("scanner.nm.connected_wifi_bssid", return_value=None):
            ok = _scan_tick("wlan0", ["Coffeebean"], NavClientStub(), {}, record_queue)
        self.assertFalse(ok)
        self.assertTrue(record_queue.empty())


if __name__ == "__main__":
    unittest.main()
