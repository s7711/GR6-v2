import unittest

import qc_check
import coords

SAVED = {
    "marker_id": 12,
    "tvec_camera_frame": [0.72, -0.12, 0.64],
    "rvec_camera_frame": [0.1, 0.2, 0.05],
}
HPR_CB = (0.0, 0.0, 0.0)


class TestCompare(unittest.TestCase):
    def test_not_configured_when_nothing_saved(self):
        result = qc_check.compare(None, {"12": SAVED}, HPR_CB)
        self.assertEqual(result, {"state": "not_configured"})

    def test_not_visible_when_saved_marker_is_not_in_the_live_debug_dict(self):
        result = qc_check.compare(SAVED, {"7": {}}, HPR_CB)
        self.assertEqual(result["state"], "not_visible")
        self.assertEqual(result["marker_id"], 12)

    def test_not_visible_when_live_debug_is_empty(self):
        result = qc_check.compare(SAVED, {}, HPR_CB)
        self.assertEqual(result["state"], "not_visible")

    def test_zero_distance_for_an_identical_live_reading(self):
        live_debug = {"12": {"tvec_camera_frame": SAVED["tvec_camera_frame"], "rvec_camera_frame": SAVED["rvec_camera_frame"]}}
        result = qc_check.compare(SAVED, live_debug, HPR_CB)
        self.assertEqual(result["state"], "ok")
        self.assertAlmostEqual(result["distance_m"], 0.0, places=9)

    def test_zero_distance_for_a_pure_rotation_at_the_same_position(self):
        # Same underlying scenario as aruco/test_coords.py's rotation-
        # invariance test - this is the actual bug report: a heading
        # difference alone must not read as a positional error.
        import numpy as np

        true_position_m = np.array([0.5, -0.2, 1.0])
        rvec_live = [0.4, -0.3, 0.2]
        tvec_live = (-coords.rvec_to_C_CM(np.array(rvec_live)) @ true_position_m).tolist()
        tvec_ideal = (-coords.rvec_to_C_CM(np.array(SAVED["rvec_camera_frame"])) @ true_position_m).tolist()
        saved = {**SAVED, "tvec_camera_frame": tvec_ideal}

        live_debug = {"12": {"tvec_camera_frame": tvec_live, "rvec_camera_frame": rvec_live}}
        result = qc_check.compare(saved, live_debug, HPR_CB)
        self.assertEqual(result["state"], "ok")
        self.assertAlmostEqual(result["distance_m"], 0.0, places=9)

    def test_nonzero_distance_for_a_real_positional_difference(self):
        live_debug = {"12": {"tvec_camera_frame": [1.0, -0.12, 0.64], "rvec_camera_frame": SAVED["rvec_camera_frame"]}}
        result = qc_check.compare(SAVED, live_debug, HPR_CB)
        self.assertEqual(result["state"], "ok")
        self.assertGreater(result["distance_m"], 0.1)

    def test_zero_distance_for_a_pure_rotation_with_the_funnel_offset(self):
        # The actual ask: the funnel (~23cm behind the camera) must be
        # the point tracked, and it must still be rotation-invariant.
        import numpy as np

        funnel_offset_c = (-0.228, 0.0, 0.0)
        true_funnel_position_m = np.array([0.5, -0.2, 1.0])
        rvec_ideal = SAVED["rvec_camera_frame"]
        rvec_live = [0.4, -0.3, 0.2]

        def tvec_for(rvec):
            c_cm = coords.rvec_to_C_CM(np.array(rvec))
            offset_M = c_cm.T @ (coords.C_Cc @ np.array(funnel_offset_c))
            camera_position_m = true_funnel_position_m - offset_M
            return (-c_cm @ camera_position_m).tolist()

        saved = {**SAVED, "tvec_camera_frame": tvec_for(rvec_ideal)}
        live_debug = {"12": {"tvec_camera_frame": tvec_for(rvec_live), "rvec_camera_frame": rvec_live}}

        result = qc_check.compare(saved, live_debug, HPR_CB, funnel_offset_c)
        self.assertEqual(result["state"], "ok")
        self.assertAlmostEqual(result["distance_m"], 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
