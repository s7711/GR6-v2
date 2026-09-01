"""Tests for ncomrx.py's NCOM decode - currently just the InsNavMode==11
("Structure-B packet") handling, see ncomrx.py's own comment on that
branch and oxts-nav-prd.md. The rest of decode() is ported close to
unchanged from GR6-v1's original and has no other test coverage yet.
"""

import struct
import unittest

from ncomrx import ANG2RAD, RAD2DEG, NcomRx

SYNC = 0xE7


def _ncom_packet(nav_status, lat=0.0, lon=0.0, heading_deg=0.0):
    """A minimal 72-byte NCOM packet with a valid checksum1, decodable by
    ncomrx.py's Structure-A path. Only fills what decode() actually reads
    for the fields this test cares about - Batch A's Ax/Ay/Az/Wx/Wy/Wz
    are left zero."""
    b = bytearray(72)
    b[0] = SYNC
    b[21] = nav_status
    b[23:31] = struct.pack("<d", lat)
    b[31:39] = struct.pack("<d", lon)
    heading_raw = round(heading_deg / (ANG2RAD * RAD2DEG))
    b[52:55] = heading_raw.to_bytes(3, byteorder="little", signed=True)
    b[22] = sum(b[1:22]) % 256  # checksum1 - the only one this test relies on
    return bytes(b)


class TestStructureBHandling(unittest.TestCase):
    def test_locked_packet_decodes_lat_lon_heading(self):
        decoder = NcomRx()
        decoder.decode(_ncom_packet(4, lat=0.9, lon=-0.02, heading_deg=90.0))
        self.assertAlmostEqual(decoder.nav["Lat"], 0.9)
        self.assertAlmostEqual(decoder.nav["Lon"], -0.02)
        self.assertAlmostEqual(decoder.nav["Heading"], 90.0, places=3)
        self.assertEqual(decoder.nav["InsNavMode"], 4)

    def test_nav_status_11_is_ignored_not_treated_as_reacquiring(self):
        # Regression test (found live 2026-09-01): a genuine, brief
        # navigation status 11 arriving between otherwise-healthy mode-4
        # packets used to wipe Lat/Lon/Heading (treated the same as
        # modes 1/2 "reacquiring"), causing navigate to abort an
        # in-progress run despite the xNAV's real Structure-A solution
        # never actually losing lock - see ncomrx.py's own comment on
        # this branch.
        decoder = NcomRx()
        decoder.decode(_ncom_packet(4, lat=0.9, lon=-0.02, heading_deg=90.0))

        decoder.decode(_ncom_packet(11))

        self.assertEqual(decoder.nav["InsNavMode"], 4)  # untouched by the mode-11 packet
        self.assertAlmostEqual(decoder.nav["Lat"], 0.9)
        self.assertAlmostEqual(decoder.nav["Lon"], -0.02)
        self.assertAlmostEqual(decoder.nav["Heading"], 90.0, places=3)

    def test_reacquiring_modes_other_than_11_still_clear_position(self):
        # The fix is specific to 11 (a different packet structure
        # entirely, per the NCOM manual) - genuine reacquiring modes
        # (e.g. 2) must still clear stale Lat/Lon/Heading, same as
        # before.
        decoder = NcomRx()
        decoder.decode(_ncom_packet(4, lat=0.9, lon=-0.02, heading_deg=90.0))

        decoder.decode(_ncom_packet(2))

        self.assertEqual(decoder.nav["InsNavMode"], 2)
        self.assertNotIn("Lat", decoder.nav)
        self.assertNotIn("Heading", decoder.nav)


if __name__ == "__main__":
    unittest.main()
