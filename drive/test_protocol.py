import unittest

import protocol


class TestParseLine(unittest.TestCase):
    def test_blank_and_malformed(self):
        self.assertEqual(protocol.parse_line(""), {})
        self.assertEqual(protocol.parse_line("\n"), {})
        self.assertEqual(protocol.parse_line("EN 1"), {})  # missing a field
        self.assertEqual(protocol.parse_line("EN abc def"), {})  # not ints
        self.assertEqual(protocol.parse_line("NOPE 1 2"), {})  # unknown tag

    def test_encoder_position_raw_ints(self):
        self.assertEqual(protocol.parse_line("EN 1234 -5678"), {"LM_position": 1234, "RM_position": -5678})

    def test_scaled_pair_fields(self):
        self.assertEqual(protocol.parse_line("SV 150 -50"), {"LM_setvel": 1.5, "RM_setvel": -0.5})
        self.assertEqual(protocol.parse_line("FV 100 100"), {"LM_vel_filt": 1.0, "RM_vel_filt": 1.0})

    def test_motor_output_raw_ints(self):
        self.assertEqual(protocol.parse_line("MO 200 -200"), {"LM_out": 200, "RM_out": -200})

    def test_pump(self):
        self.assertEqual(protocol.parse_line("WP 1"), {"pump": True})
        self.assertEqual(protocol.parse_line("WP 0"), {"pump": False})

    def test_control_enabled(self):
        self.assertEqual(protocol.parse_line("GO 4"), {"ctrl_enabled": 4})

    def test_ultrasonic(self):
        self.assertEqual(protocol.parse_line("U0 450"), {"ultrasonic_0_mm": 450})
        self.assertEqual(protocol.parse_line("U4 -1"), {"ultrasonic_4_mm": -1})

    def test_tuning_scaled(self):
        self.assertEqual(protocol.parse_line("Kp 100 100"), {"tuning_Kp_L": 1.0, "tuning_Kp_R": 1.0})

    def test_tuning_raw(self):
        self.assertEqual(protocol.parse_line("Db 5 5"), {"tuning_Db_L": 5, "tuning_Db_R": 5})

    def test_version(self):
        self.assertEqual(protocol.parse_line("Version 250819#5.GR6"), {"firmware_version": "250819#5.GR6"})


class TestEncodeSetVelocity(unittest.TestCase):
    def test_rounds_to_int(self):
        self.assertEqual(protocol.encode_set_velocity(100.4, -50.6), "SV 100 -51\n")

    def test_zero_zero(self):
        self.assertEqual(protocol.encode_set_velocity(0, 0), "SV 0 0\n")


class TestEncodePump(unittest.TestCase):
    def test_on_off(self):
        self.assertEqual(protocol.encode_pump(True), "WP 1\n")
        self.assertEqual(protocol.encode_pump(False), "WP 0\n")


class TestEncodeTuning(unittest.TestCase):
    def test_scaled_param(self):
        self.assertEqual(protocol.encode_tuning("Kp", 1.5, 2.0), "Kp 150 200\n")

    def test_raw_param(self):
        self.assertEqual(protocol.encode_tuning("Db", 5, 5), "Db 5 5\n")

    def test_unknown_param_raises(self):
        with self.assertRaises(ValueError):
            protocol.encode_tuning("Nope", 1, 1)

    def test_round_trip(self):
        line = protocol.encode_tuning("Ki", 3.0, 3.0)
        parsed = protocol.parse_line("Ki " + line.split(" ", 1)[1])
        self.assertEqual(parsed, {"tuning_Ki_L": 3.0, "tuning_Ki_R": 3.0})


if __name__ == "__main__":
    unittest.main()
