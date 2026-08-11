"""Tests for nav_feed.snapshot()'s staleness detection - see
nav_feed.py's own "Staleness" docstring section for why this exists:
ncomrx_thread.py/ucomrx_thread.py just block on their UDP socket, so if
the xNAV stops sending entirely, decoder.nav/status/connection sit
frozen at their last real values forever with nothing to say so.
"""

import threading
import time
import unittest

import nav_feed

XNAV_IP = "192.168.196.147"


class FakeDecoder:
    def __init__(self, nav=None, status=None, connection=None):
        self.nav = nav or {"Lat": 0.1, "Lon": -0.2, "Heading": 90.0}
        self.status = status or {"GnssPosMode": 4}
        self.connection = connection or {"ip": XNAV_IP}


class FakeNrxs:
    """Minimal stand-in for ncomrx_thread.NcomRxThread/
    ucomrx_thread.UcomRxThread - just the .lock/.nrx shape snapshot()
    actually reads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.nrx = {}

    def set_last_packet(self, ip, decoder, last_packet_at):
        self.nrx[ip] = {"decoder": decoder, "last_packet_at": last_packet_at}


class TestSnapshot(unittest.TestCase):
    def test_fresh_data_passes_through_unchanged(self):
        nrxs = FakeNrxs()
        decoder = FakeDecoder()
        nrxs.set_last_packet(XNAV_IP, decoder, time.monotonic())

        result = nav_feed.snapshot(nrxs, XNAV_IP, stale_after_s=2.0)
        self.assertEqual(result["nav"], decoder.nav)
        self.assertEqual(result["status"], decoder.status)
        self.assertEqual(result["connection"], decoder.connection)

    def test_stale_data_is_blanked_not_frozen(self):
        nrxs = FakeNrxs()
        decoder = FakeDecoder()
        nrxs.set_last_packet(XNAV_IP, decoder, time.monotonic() - 5.0)  # 5s since the last real packet

        result = nav_feed.snapshot(nrxs, XNAV_IP, stale_after_s=2.0)
        self.assertEqual(result, {"nav": {}, "status": {}, "connection": {}})

    def test_exactly_at_the_threshold_is_not_yet_stale(self):
        nrxs = FakeNrxs()
        decoder = FakeDecoder()
        nrxs.set_last_packet(XNAV_IP, decoder, time.monotonic() - 1.0)

        result = nav_feed.snapshot(nrxs, XNAV_IP, stale_after_s=2.0)
        self.assertEqual(result["nav"], decoder.nav)

    def test_unknown_ip_is_blanked_not_a_crash(self):
        nrxs = FakeNrxs()
        result = nav_feed.snapshot(nrxs, XNAV_IP, stale_after_s=2.0)
        self.assertEqual(result, {"nav": {}, "status": {}, "connection": {}})

    def test_ip_present_but_never_received_a_packet_yet_is_blanked(self):
        # e.g. right at startup, before the first UDP packet has arrived.
        nrxs = FakeNrxs()
        nrxs.nrx[XNAV_IP] = {"decoder": FakeDecoder()}  # no 'last_packet_at' key yet
        result = nav_feed.snapshot(nrxs, XNAV_IP, stale_after_s=2.0)
        self.assertEqual(result, {"nav": {}, "status": {}, "connection": {}})

    def test_blanking_returns_empty_dicts_not_none_valued_keys(self):
        # Every consumer (navigate's _current_position, wheelspeed's
        # _forward_mps, aruco's "if nav:" guards) treats a missing key
        # or an empty dict as "no fix" - a dict with None-valued keys
        # would sail straight through an "if nav:" truthiness check and
        # then crash on the first real field access. See nav_feed.py's
        # own docstring for the aruco/survey.py case this would break.
        nrxs = FakeNrxs()
        decoder = FakeDecoder()
        nrxs.set_last_packet(XNAV_IP, decoder, time.monotonic() - 5.0)
        result = nav_feed.snapshot(nrxs, XNAV_IP, stale_after_s=2.0)
        self.assertFalse(result["nav"])  # falsy - not a dict full of Nones
        self.assertNotIn("Lat", result["nav"])


if __name__ == "__main__":
    unittest.main()
