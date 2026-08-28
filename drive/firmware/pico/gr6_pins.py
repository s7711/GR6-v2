"""
GR6-Robot-Power -- Raspberry Pi Pico (U3) GPIO map.

Derived from schematics/main + circuit/circuit.lp (board_2).
Regenerate/check against the netlist if the schematic changes.
"""

# ---- User signal breakout (J_USR_SIG, 3V3-referenced) ----
USR_1 = 0
USR_2 = 1
USR_3 = 2
USR_4 = 3
USR_5 = 4
USR_6 = 5

# ---- 12V rail soft-latch enable ----
# Drive HIGH as early as possible in firmware to latch Q2 on (via Q1),
# keeping 12V_PWR alive after the external wake button is released.
# Until this pin goes high, only the physical J_WAKE button holds power up.
PWR_12V_EN = 6

# ---- Pump enables (direct low-side MOSFET gate drive, Q3/Q4) ----
PUMP1_EN = 7
PUMP2_EN = 8

# ---- Right motor (TB6612FNG channel B) ----
RM_PWM = 9
RM_IN2 = 10
RM_IN1 = 11

# ---- Motor driver standby (shared by both channels) ----
MOTOR_STBY = 12

# ---- Left motor (TB6612FNG channel A) ----
LM_IN1 = 13
LM_IN2 = 14
LM_PWM = 15

# ---- Tank level sensors (3144 Hall effect, active-high) ----
TANK_FULL = 16
TANK_EMPTY = 17

# ---- Wheel encoders (3144 Hall effect, active-high) ----
RM_ENC_B = 18
RM_ENC_A = 19
LM_ENC_B = 20
LM_ENC_A = 21

# ---- Spare GPIOs, broken out to J_SPARE_SIG (3V3-referenced) ----
SPARE1 = 22
SPARE2 = 26
SPARE3 = 27

# ---- Battery voltage sense ----
# Resistive divider R4 (47k, to V_BATT) / R5 (10k, to GND).
# Vbatt = Vadc * (R4+R5)/R5 = Vadc * VBATT_DIVIDER_RATIO
VBATT_SENSE_PIN = 28
VBATT_SENSE_ADC_CHANNEL = 2  # GPIO28 = ADC channel 2
VBATT_DIVIDER_RATIO = 5.7

# Not used on this board (reserved on the Pico module itself):
#   GPIO23 - SMPS power-save mode control
#   GPIO24 - VBUS sense
#   GPIO25 - onboard LED
#   GPIO29 - VSYS sense / ADC3


def read_vbatt(adc):
    """Convert a raw ADC.read_u16() reading (0-65535) to battery volts.

    Usage:
        from machine import ADC
        import gr6_pins as pins
        vbatt_adc = ADC(pins.VBATT_SENSE_PIN)
        voltage = read_vbatt(vbatt_adc)
    """
    v_adc = (adc.read_u16() / 65535) * 3.3
    return v_adc * VBATT_DIVIDER_RATIO
