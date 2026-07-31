import unittest

import continuity
import geometry  # noqa: E402 - navigate/ already on sys.path, appended by importing continuity above

ENTRY_MAX_DISTANCE_M = 1.0
ENTRY_MAX_HEADING_DEG = 45


def straight_path(start_lat, start_lon, heading_deg, count=3, step_m=5.0):
    points = []
    lat, lon = start_lat, start_lon
    for _ in range(count):
        points.append({"lat": lat, "lon": lon, "speed_mps": 0.5, "pump": False, "clearance_m": 0.5})
        lat, lon = geometry.project_forward(lat, lon, heading_deg, step_m)
    return points


class TestCheck(unittest.TestCase):
    def test_ok_when_path_b_continues_straight_on_from_path_a(self):
        path_a = straight_path(52.2, -1.5, heading_deg=0)
        path_b = straight_path(path_a[-1]["lat"], path_a[-1]["lon"], heading_deg=0)
        result = continuity.check(path_a, path_b, ENTRY_MAX_DISTANCE_M, ENTRY_MAX_HEADING_DEG)
        self.assertEqual(result, {"ok": True})

    def test_fails_when_path_b_starts_too_far_away(self):
        path_a = straight_path(52.2, -1.5, heading_deg=0)
        far_lat, far_lon = geometry.project_forward(path_a[-1]["lat"], path_a[-1]["lon"], 90, 5.0)
        path_b = straight_path(far_lat, far_lon, heading_deg=0)
        result = continuity.check(path_a, path_b, ENTRY_MAX_DISTANCE_M, ENTRY_MAX_HEADING_DEG)
        self.assertFalse(result["ok"])
        self.assertGreater(result["distance_m"], ENTRY_MAX_DISTANCE_M)

    def test_fails_when_path_b_starts_at_the_wrong_heading(self):
        path_a = straight_path(52.2, -1.5, heading_deg=0)
        path_b = straight_path(path_a[-1]["lat"], path_a[-1]["lon"], heading_deg=90)
        result = continuity.check(path_a, path_b, ENTRY_MAX_DISTANCE_M, ENTRY_MAX_HEADING_DEG)
        self.assertFalse(result["ok"])
        self.assertGreater(abs(result["heading_error_deg"]), ENTRY_MAX_HEADING_DEG)

    def test_ok_trivially_for_a_single_point_path(self):
        path_a = straight_path(52.2, -1.5, heading_deg=0, count=1)
        path_b = straight_path(52.2, -1.5, heading_deg=0)
        result = continuity.check(path_a, path_b, ENTRY_MAX_DISTANCE_M, ENTRY_MAX_HEADING_DEG)
        self.assertEqual(result, {"ok": True})


if __name__ == "__main__":
    unittest.main()
