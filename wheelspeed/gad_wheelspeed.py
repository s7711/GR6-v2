"""Sends per-wheel speed/velocity aiding data to the xNAV650 as GAD
(Generic Aiding Data) updates. Ported from GR6-v1's gad_wheelspeed.py —
see wheelspeed-prd.md for what changed and why (mainly: config-driven
rather than a runtime command, and reads already-in-m/s wheel velocity
from drive's feed rather than raw encoder counts).
"""

import logging
import sys
from pathlib import Path

import numpy as np
import oxts_sdk

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "aruco"))
from coords import hpr_to_dcm  # noqa: E402

# Stream IDs, unchanged from GR6-v1 — arbitrary OxTS aiding-stream identifiers.
LEFT_STREAM_ID = 133
RIGHT_STREAM_ID = 134

# Fixed parameters, unchanged from GR6-v1 — not exposed in config, see
# wheelspeed-prd.md's "Config" section.
SPEED_VAR = 0.01  # m^2/s^2 — GadSpeed's speed_ms_var
# GadVelocity's variance, in the IMU frame before rotation — deliberately
# NOT the "correct" ~1.0 for the lateral/vertical axes (unmeasured), since
# GR6-v1 found that stops the update being useful; 0.01 isotropic is what
# was empirically found to work. Carried forward as-is, not re-derived.
VELOCITY_VAR_IMU = np.diag([0.01, 0.01, 0.01])
LEVER_ARM_VAR = [0.001, 0.001, 0.001]  # ~3cm, both message types


class GadWheelspeed:
    def __init__(
        self, xnav_ip, hpr_ib_deg, gad_type,
        left_lever_arm_i, left_scale, right_lever_arm_i, right_scale,
    ):
        self._handler = oxts_sdk.GadHandler()
        self._handler.set_encoder_to_bin()
        self._handler.set_output_mode_to_udp(xnav_ip)
        self._c_ib = hpr_to_dcm(*hpr_ib_deg)
        self.gad_type = gad_type  # "GadVelocity" or "GadSpeed" — see wheelspeed-prd.md
        self.left_lever_arm_i = left_lever_arm_i
        self.left_scale = left_scale
        self.right_lever_arm_i = right_lever_arm_i
        self.right_scale = right_scale
        self.packets_sent = 0

    def update(self, gps_week, gps_seconds, left_mps, right_mps):
        """Sends one GAD packet per wheel — left_mps/right_mps already in
        m/s (drive's own filtered wheel velocity), signed +forward/-back."""
        try:
            left_velocity = left_mps * self.left_scale
            right_velocity = right_mps * self.right_scale
            send = self._send_speed if self.gad_type == "GadSpeed" else self._send_velocity
            send(LEFT_STREAM_ID, left_velocity, self.left_lever_arm_i, gps_week, gps_seconds)
            send(RIGHT_STREAM_ID, right_velocity, self.right_lever_arm_i, gps_week, gps_seconds)
        except Exception:
            logging.exception("[wheelspeed] Failed to send GAD update")

    def _send_speed(self, stream_id, velocity, lever_arm_i, gps_week, gps_seconds):
        # GR6-v1's own note, carried forward: GadSpeed's effect on the INS
        # solution was never confirmed — "I have been unable to get
        # GadSpeed to do anything... I cannot detect that it is
        # functional using this code." Included anyway since both
        # message types were asked for — see wheelspeed-prd.md.
        g = oxts_sdk.GadSpeed(stream_id)
        if velocity >= 0:
            g.speed_fw_ms = velocity
        else:
            g.speed_bw_ms = -velocity
        g.speed_ms_var = SPEED_VAR
        g.time_gps = [gps_week, gps_seconds]
        g.aiding_lever_arm_fixed = lever_arm_i
        g.aiding_lever_arm_var = LEVER_ARM_VAR
        self._handler.send_packet(g)
        self.packets_sent += 1

    def _send_velocity(self, stream_id, velocity, lever_arm_i, gps_week, gps_seconds):
        g = oxts_sdk.GadVelocity(stream_id)
        v_i = self._c_ib @ np.array([velocity, 0.0, 0.0])
        g.vel_odom = v_i.tolist()
        var_i = self._c_ib @ VELOCITY_VAR_IMU @ self._c_ib.T
        g.vel_odom_var = [var_i[0][0], var_i[1][1], var_i[2][2], var_i[0][1], var_i[0][2], var_i[1][2]]
        g.time_gps = [gps_week, gps_seconds]
        g.aiding_lever_arm_fixed = lever_arm_i
        g.aiding_lever_arm_var = LEVER_ARM_VAR
        self._handler.send_packet(g)
        self.packets_sent += 1
