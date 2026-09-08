import unittest

from scanner import _parse_scan

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


if __name__ == "__main__":
    unittest.main()
