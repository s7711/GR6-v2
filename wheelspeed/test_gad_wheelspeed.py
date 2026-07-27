import unittest
from unittest.mock import MagicMock, patch

import gad_wheelspeed
from gad_wheelspeed import LEFT_STREAM_ID, RIGHT_STREAM_ID, GadWheelspeed


class _MockedSdk(unittest.TestCase):
    """Every test here mocks gad_wheelspeed.oxts_sdk out for the whole
    test (construction AND update()) — never opens a real UDP socket or
    touches real hardware."""

    def setUp(self):
        patcher = patch.object(gad_wheelspeed, "oxts_sdk")
        self.mock_sdk = patcher.start()
        self.addCleanup(patcher.stop)
        self.mock_sdk.GadSpeed.side_effect = lambda stream_id: MagicMock(stream_id=stream_id)
        self.mock_sdk.GadVelocity.side_effect = lambda stream_id: MagicMock(stream_id=stream_id)

    def make(self, gad_type="GadVelocity", hpr_ib_deg=(0.0, 0.0, 0.0)):
        return GadWheelspeed(
            xnav_ip="192.168.1.1", hpr_ib_deg=hpr_ib_deg, gad_type=gad_type,
            left_lever_arm_i=[0.1, 0.2, 0.3], left_scale=1.0,
            right_lever_arm_i=[0.4, 0.5, 0.6], right_scale=1.0,
        )


class TestGadVelocity(_MockedSdk):
    def test_sends_one_packet_per_wheel_with_correct_stream_ids(self):
        gw = self.make(gad_type="GadVelocity")
        gw.update(gps_week=2300, gps_seconds=123456.0, left_mps=1.0, right_mps=-1.0)
        self.assertEqual(gw._handler.send_packet.call_count, 2)
        sent_left, sent_right = (call.args[0] for call in gw._handler.send_packet.call_args_list)
        self.assertEqual(sent_left.stream_id, LEFT_STREAM_ID)
        self.assertEqual(sent_right.stream_id, RIGHT_STREAM_ID)
        self.assertEqual(gw.packets_sent, 2)

    def test_identity_rotation_puts_forward_speed_on_x(self):
        # hpr_ib_deg=(0,0,0) -> C_ib is the identity, so a purely-forward
        # wheel velocity should appear unrotated as vel_odom[0].
        gw = self.make(gad_type="GadVelocity", hpr_ib_deg=(0.0, 0.0, 0.0))
        gw.update(gps_week=1, gps_seconds=2.0, left_mps=1.5, right_mps=2.5)
        sent_left, sent_right = (call.args[0] for call in gw._handler.send_packet.call_args_list)
        self.assertAlmostEqual(sent_left.vel_odom[0], 1.5)
        self.assertAlmostEqual(sent_right.vel_odom[0], 2.5)

    def test_90_degree_heading_rotation_moves_forward_speed_to_y(self):
        # A 90deg heading-only rotation should move a purely-forward (x)
        # wheel velocity onto the y axis in the IMU frame — a real check
        # that hpr_to_dcm is actually being applied, not just passed
        # through as identity by coincidence.
        gw = self.make(gad_type="GadVelocity", hpr_ib_deg=(90.0, 0.0, 0.0))
        gw.update(gps_week=1, gps_seconds=1.0, left_mps=1.0, right_mps=1.0)
        sent_left, _ = (call.args[0] for call in gw._handler.send_packet.call_args_list)
        self.assertAlmostEqual(sent_left.vel_odom[0], 0.0, places=9)
        self.assertAlmostEqual(sent_left.vel_odom[1], 1.0)

    def test_lever_arms_and_time_are_passed_through(self):
        gw = self.make(gad_type="GadVelocity")
        gw.update(gps_week=2300, gps_seconds=999.0, left_mps=1.0, right_mps=1.0)
        sent_left, sent_right = (call.args[0] for call in gw._handler.send_packet.call_args_list)
        self.assertEqual(sent_left.aiding_lever_arm_fixed, [0.1, 0.2, 0.3])
        self.assertEqual(sent_right.aiding_lever_arm_fixed, [0.4, 0.5, 0.6])
        self.assertEqual(sent_left.time_gps, [2300, 999.0])

    def test_scale_multiplies_velocity(self):
        gw = self.make(gad_type="GadVelocity")
        gw.left_scale = 2.0
        gw.update(gps_week=1, gps_seconds=1.0, left_mps=1.0, right_mps=1.0)
        sent_left, _ = (call.args[0] for call in gw._handler.send_packet.call_args_list)
        self.assertAlmostEqual(sent_left.vel_odom[0], 2.0)


class TestGadSpeed(_MockedSdk):
    def test_positive_velocity_sets_forward_speed(self):
        gw = self.make(gad_type="GadSpeed")
        gw.update(gps_week=1, gps_seconds=1.0, left_mps=1.5, right_mps=1.5)
        sent_left, _ = (call.args[0] for call in gw._handler.send_packet.call_args_list)
        self.assertEqual(sent_left.speed_fw_ms, 1.5)

    def test_negative_velocity_sets_backward_speed_as_positive_magnitude(self):
        gw = self.make(gad_type="GadSpeed")
        gw.update(gps_week=1, gps_seconds=1.0, left_mps=-2.0, right_mps=1.0)
        sent_left, _ = (call.args[0] for call in gw._handler.send_packet.call_args_list)
        self.assertEqual(sent_left.speed_bw_ms, 2.0)


class TestUpdateFailureIsSwallowed(_MockedSdk):
    def test_exception_during_send_does_not_propagate(self):
        gw = self.make(gad_type="GadVelocity")
        gw._handler.send_packet.side_effect = RuntimeError("boom")
        gw.update(gps_week=1, gps_seconds=1.0, left_mps=1.0, right_mps=1.0)  # must not raise


if __name__ == "__main__":
    unittest.main()
