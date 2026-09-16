"""Tests for bore-sight capture — overwhelmingly about time matching,
because that's what the accuracy rests on. A 25ms pairing error is
0.14 deg of hpr_cb bias, which alone exceeds the whole budget."""

import json
import math
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import capture  # noqa: E402


# Realistic-ish values: ncomrx defines
#   timeOffset = GpsSeconds + GpsMinutes*60 - machineTime
TIME_OFFSET = 1473262891.43
GPS_MINUTES = 24560080
# Quality fields live in `status`, not `nav` — see STATUS_QUALITY_FIELDS.
STATUS = {"GpsMinutes": GPS_MINUTES, "NorthAcc": 0.009, "EastAcc": 0.009,
          "HeadingAcc": 0.18, "GnssPosMode": 6, "InsNavMode": 4}


def _status(**extra):
    return {**STATUS, **extra}


def _t(gps_s):
    """The machine time _nav(gps_s) corresponds to."""
    return GPS_MINUTES * 60.0 + gps_s - TIME_OFFSET


def _nav(gps_s, lat=0.9, lon=-0.025, alt=100.0, heading=10.0, pitch=0.0, roll=0.0, **extra):
    out = {"GpsSeconds": gps_s, "Lat": lat, "Lon": lon, "Alt": alt,
           "Heading": heading, "Pitch": pitch, "Roll": roll,
           # NCOM carries velocity, not speed — the feed has no
           # HorizontalSpeed field at all.
           "Vn": 0.0, "Ve": 0.2}
    out.update(extra)
    return out


class NavBufferTestCase(unittest.TestCase):
    def setUp(self):
        self.buf = capture.NavBuffer()

    def test_rejects_samples_missing_required_fields(self):
        self.assertFalse(self.buf.add({"Lat": 1.0}, STATUS, TIME_OFFSET))
        self.assertEqual(len(self.buf), 0)

    def test_nav_sample_time_inverts_ncomrx_own_offset(self):
        """ncomrx defines timeOffset = GpsSeconds + GpsMinutes*60 -
        machineTime, so inverting it must return the machine time the
        sample was measured at — the same clock the camera stamps frames
        with. Nothing here re-derives a mapping that already exists."""
        machine_time = 341922.5378
        nav = _nav(13.29)
        status = {"GpsMinutes": 24560080}
        offset = nav["GpsSeconds"] + status["GpsMinutes"] * 60.0 - machine_time
        self.assertAlmostEqual(capture.nav_machine_time(nav, status, offset),
                               machine_time, places=6)

    def test_gps_seconds_alone_is_not_a_timescale(self):
        """It is seconds within the current MINUTE (13.29, not 311840) —
        it wraps every 60s and shares no origin with the frame clock.
        GpsMinutes is what makes it absolute. Keying on GpsSeconds by
        itself matched nothing: a live 20-second dry run rejected all
        100 frames as 'no time-matched nav'."""
        nav = _nav(13.29)
        self.assertLess(nav["GpsSeconds"], 60.0)
        self.assertNotAlmostEqual(
            capture.nav_machine_time(nav, STATUS, TIME_OFFSET), nav["GpsSeconds"], places=0)

    def test_needs_the_offset_and_the_minutes(self):
        self.assertIsNone(capture.nav_machine_time(_nav(100.0), STATUS, None))
        self.assertIsNone(capture.nav_machine_time(_nav(100.0), {}, TIME_OFFSET))

    def test_rejects_a_repeated_or_backwards_sample(self):
        self.assertTrue(self.buf.add(_nav(100.0), STATUS, TIME_OFFSET))
        self.assertFalse(self.buf.add(_nav(100.0), STATUS, TIME_OFFSET))
        self.assertFalse(self.buf.add(_nav(99.5), STATUS, TIME_OFFSET))
        self.assertEqual(len(self.buf), 1)

    def test_interpolates_position_linearly_between_samples(self):
        self.buf.add(_nav(100.0, alt=10.0), STATUS, TIME_OFFSET)
        self.buf.add(_nav(100.05, alt=20.0), STATUS, TIME_OFFSET)
        out = self.buf.at(_t(100.025))
        # places=4, not 9: GPS-epoch seconds are ~1.47e9, where a float64
        # resolves ~2.4e-7s. This sample climbs 10m in 0.05s (200 m/s),
        # so that becomes ~5e-5m here. At the robot's actual 0.2 m/s the
        # same time resolution is 5e-8 m, which is why this is a test
        # artefact and not a reason to re-base the timescale.
        self.assertAlmostEqual(out["Alt"], 15.0, places=4)

    def test_interpolates_heading_the_short_way_around_north(self):
        """359 -> 1 must average to 0, not 180. Linear interpolation here
        would point the vehicle exactly backwards at the moment it
        crosses north."""
        self.buf.add(_nav(100.0, heading=179.0), STATUS, TIME_OFFSET)
        self.buf.add(_nav(100.05, heading=-179.0), STATUS, TIME_OFFSET)
        out = self.buf.at(_t(100.025))
        # The property that matters is which WAY it went: the short way
        # lands on +/-180, linear interpolation would land on 0. The
        # tolerance is loose because timeOffset arithmetic passes through
        # ~1.5e9 (GpsMinutes*60), so the machine time carries ~2.4e-7s of
        # float error — which at this test's absurd 7160 deg/s shows up
        # in the 5th decimal, and at real turn rates is ~1e-5 deg.
        self.assertAlmostEqual(abs(out["Heading"]), 180.0, places=3)
        self.assertGreater(abs(out["Heading"]), 90.0)  # emphatically not 0

    def test_heading_interpolation_stays_in_range(self):
        self.buf.add(_nav(100.0, heading=170.0), STATUS, TIME_OFFSET)
        self.buf.add(_nav(100.05, heading=-170.0), STATUS, TIME_OFFSET)
        out = self.buf.at(_t(100.025))
        self.assertGreaterEqual(out["Heading"], -180.0)
        self.assertLess(out["Heading"], 180.0)

    def test_never_extrapolates_outside_the_buffer(self):
        self.buf.add(_nav(100.0), STATUS, TIME_OFFSET)
        self.buf.add(_nav(100.05), STATUS, TIME_OFFSET)
        self.assertIsNone(self.buf.at(_t(99.9)))
        self.assertIsNone(self.buf.at(_t(100.2)))

    def test_refuses_to_interpolate_across_a_feed_dropout(self):
        """Better to drop the frame than invent a pose across a gap."""
        self.buf.add(_nav(100.0), STATUS, TIME_OFFSET)
        self.buf.add(_nav(101.0), STATUS, TIME_OFFSET)  # 1s gap, over MAX_INTERP_GAP_S
        self.assertIsNone(self.buf.at(_t(100.5)))

    def test_quality_fields_come_from_the_nearer_sample_not_blended(self):
        self.buf.add(_nav(100.0), _status(GnssPosMode=6, HeadingAcc=0.1), TIME_OFFSET)
        self.buf.add(_nav(100.05), _status(GnssPosMode=4, HeadingAcc=0.9), TIME_OFFSET)
        self.assertEqual(self.buf.at(_t(100.01))["GnssPosMode"], 6)
        self.assertEqual(self.buf.at(_t(100.04))["GnssPosMode"], 4)

    def test_newest_distinguishes_too_new_from_too_old(self):
        """A frame newer than every nav sample can still be matched once
        nav catches up; one older than the buffer never will. Without
        that distinction the capture discarded 42% of its frames."""
        self.assertEqual(self.buf.newest(), float("-inf"))
        self.buf.add(_nav(100.0), STATUS, TIME_OFFSET)
        self.buf.add(_nav(100.05), STATUS, TIME_OFFSET)
        self.assertAlmostEqual(self.buf.newest(), _t(100.05), places=6)
        self.assertGreater(_t(100.2), self.buf.newest())   # too new — wait
        self.assertLess(_t(99.0), self.buf.newest())       # too old — give up

    def test_old_samples_are_dropped(self):
        for i in range(200):
            self.buf.add(_nav(100.0 + i * 0.05), STATUS, TIME_OFFSET)
        self.assertLessEqual(self.buf.span_s(), capture.BUFFER_SECONDS + 0.05)
        self.assertGreater(len(self.buf), 2)

    def test_interpolation_beats_taking_the_latest_sample(self):
        """The reason this class exists. Nav at 20Hz, a frame landing
        mid-interval: 'latest' is up to 52ms stale, interpolation is
        exact."""
        speed_deg_per_s = 0.001
        for i in range(5):
            self.buf.add(_nav(100.0 + i * 0.05, lat=0.9 + i * 0.05 * speed_deg_per_s), STATUS, TIME_OFFSET)
        frame_t = 100.0 + 2 * 0.05 + 0.025  # squarely between two samples
        exact = 0.9 + (frame_t - 100.0) * speed_deg_per_s
        interpolated = self.buf.at(_t(frame_t))["Lat"]
        latest = 0.9 + (100.0 + 4 * 0.05 - 100.0) * speed_deg_per_s
        self.assertLess(abs(interpolated - exact), 1e-12)
        self.assertGreater(abs(latest - exact), 1e-6)


class GateTestCase(unittest.TestCase):
    """The gate sees an INTERPOLATED sample (NavBuffer.at's output), which
    merges nav's pose with status's quality fields — not a raw nav dict."""

    def setUp(self):
        self.gate = capture.Gate()

    def _interpolated(self, **extra):
        buf = capture.NavBuffer()
        status = _status(**{k: v for k, v in extra.items() if k in capture.STATUS_QUALITY_FIELDS})
        nav_extra = {k: v for k, v in extra.items() if k not in capture.STATUS_QUALITY_FIELDS}
        buf.add(_nav(100.0, **nav_extra), status, TIME_OFFSET)
        buf.add(_nav(100.05, **nav_extra), status, TIME_OFFSET)
        return buf.at(_t(100.02))

    def test_accepts_a_healthy_moving_sample(self):
        self.assertIsNone(self.gate.check(self._interpolated()))

    def test_derives_speed_from_velocity(self):
        """There is no HorizontalSpeed in the live feed; it is hypot(Vn, Ve)."""
        sample = self._interpolated(Vn=0.3, Ve=0.4)
        self.assertAlmostEqual(sample["HorizontalSpeed"], 0.5, places=6)

    def test_moving_is_fine(self):
        """Explicitly: the point of interpolating nav is that we don't
        have to stop, and moving yields far more data per minute."""
        self.assertIsNone(self.gate.check(self._interpolated(Vn=0.0, Ve=0.3)))

    def test_rejects_no_time_matched_nav(self):
        self.assertEqual(self.gate.check(None), "no time-matched nav")

    def test_rejects_without_rtk(self):
        self.assertIn("RTK", self.gate.check(self._interpolated(GnssPosMode=4)))

    def test_rejects_poor_heading_accuracy(self):
        self.assertIn("heading", self.gate.check(self._interpolated(HeadingAcc=1.5)))

    def test_rejects_poor_position_accuracy(self):
        self.assertIn("position", self.gate.check(self._interpolated(NorthAcc=0.3)))

    def test_rejects_driving_too_fast(self):
        self.assertIn("speed", self.gate.check(self._interpolated(Vn=0.0, Ve=1.5)))

    def test_quality_fields_must_come_from_status_not_nav(self):
        """Reading them from nav yields None and the gate then rejects
        every single frame — found live, 91 frames lost to
        'not RTK fixed (GnssPosMode None)'."""
        for field in ("GnssPosMode", "NorthAcc", "HeadingAcc"):
            self.assertNotIn(field, _nav(100.0))
            self.assertIn(field, STATUS)


class SessionTestCase(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.session = capture.Session(self.tmp / "s.jsonl", [20, 21, 22], 0.097,
                                       (0.0, 0.0, 0.0), (0.0775, 0.002, -0.07))

    def tearDown(self):
        self.session.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _det(self, marker_id):
        return {"id": marker_id, "size": 0.097,
                "corners": [[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]]}

    def test_writes_a_header_first(self):
        first = json.loads((self.tmp / "s.jsonl").read_text().splitlines()[0])
        self.assertEqual(first["type"], "header")
        self.assertEqual(first["marker_ids"], [20, 21, 22])

    def test_records_one_row_per_detection_and_counts_them(self):
        self.session.add(100.0, _nav(100.0), [self._det(20), self._det(21)])
        self.session.add(100.2, _nav(100.2), [self._det(20)])
        status = self.session.status()
        self.assertEqual(status["counts"][20], 2)
        self.assertEqual(status["counts"][21], 1)
        self.assertEqual(status["total"], 3)
        self.assertEqual(status["frames"], 2)

    def test_rows_are_flushed_as_they_go_not_held_to_the_end(self):
        """A 20-minute run is thousands of detections; losing them at
        minute 19 would mean re-driving the whole pattern."""
        self.session.add(100.0, _nav(100.0), [self._det(20)])
        lines = (self.tmp / "s.jsonl").read_text().splitlines()
        self.assertEqual(len(lines), 2)  # header + one obs, already on disk

    def test_counts_rejections_by_reason(self):
        self.session.reject("not RTK fixed")
        self.session.reject("not RTK fixed")
        self.session.reject("speed")
        self.assertEqual(self.session.status()["rejected"],
                         {"not RTK fixed": 2, "speed": 1})

    def test_round_trips_through_load_session(self):
        self.session.add(100.0, _nav(100.0), [self._det(20)])
        self.session.close()
        header, rows = capture.load_session(self.tmp / "s.jsonl")
        self.assertEqual(header["marker_size"], 0.097)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["id"], 20)
        self.assertEqual(np.array(rows[0]["corners"]).shape, (4, 2))

    def test_writes_lat_lon_in_degrees_converting_from_ncom_radians(self):
        """The nav dict is radians; the file is degrees. Converting once
        at this boundary is what lets everything downstream stop guessing
        — see the note in Session.add."""
        self.session.add(100.0, _nav(100.0, lat=0.9, lon=-0.025), [self._det(20)])
        self.session.close()
        _header, rows = capture.load_session(self.tmp / "s.jsonl")
        self.assertAlmostEqual(rows[0]["lat"], math.degrees(0.9), places=9)
        self.assertAlmostEqual(rows[0]["lon"], math.degrees(-0.025), places=9)


class SessionToObservationsTestCase(unittest.TestCase):
    def test_builds_observations_in_local_ned(self):
        rows = []
        for i in range(6):
            rows.append({
                "t": 100.0 + i * 0.2, "id": 20 + (i % 3), "size": 0.097,
                "corners": [[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]],
                "lat": 52.2354, "lon": -1.4605, "alt": 100.0,
                "heading": 10.0, "pitch": 0.0, "roll": 0.0,
            })
        obs = capture.session_to_observations(rows, 52.2354, -1.4605, 100.0)
        self.assertEqual(len(obs), 6)
        self.assertEqual(obs.marker_ids, [20, 21, 22])
        self.assertEqual(obs.corners.shape, (6, 4, 2))
        np.testing.assert_allclose(obs.nav_pos_n, 0.0, atol=1e-6)  # all at the reference

    def test_row_lat_lon_are_degrees_with_no_unit_guessing(self):
        """Rows are degrees, full stop. An earlier version tried to infer
        the unit from the magnitude, which cannot work here: this site's
        longitude is -1.46, a perfectly plausible number of degrees AND
        of radians. It silently became -83.7 deg."""
        base = {"t": 100.0, "id": 20, "size": 0.097,
                "corners": [[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]],
                "alt": 100.0, "heading": 0.0, "pitch": 0.0, "roll": 0.0}
        obs = capture.session_to_observations(
            [{**base, "lat": 52.2354, "lon": -1.4605}], 52.2354, -1.4605, 100.0)
        np.testing.assert_allclose(obs.nav_pos_n, 0.0, atol=1e-6)

    def test_returns_none_when_no_rows_match_the_wanted_ids(self):
        rows = [{"t": 1.0, "id": 99, "size": 0.097, "corners": [[0, 0]] * 4,
                 "lat": 52.2, "lon": -1.4, "alt": 1.0,
                 "heading": 0.0, "pitch": 0.0, "roll": 0.0}]
        self.assertIsNone(capture.session_to_observations(rows, 52.2, -1.4, 1.0,
                                                          marker_ids=[20, 21, 22]))


if __name__ == "__main__":
    unittest.main()
