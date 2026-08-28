/*
 * GR6-Robot-Power — Raspberry Pi Pico (U3) GPIO map
 *
 * Derived from schematics/main + circuit/circuit.lp (board_2).
 * Regenerate/check against the netlist if the schematic changes.
 */
#ifndef GR6_PINS_H
#define GR6_PINS_H

/* ---- User signal breakout (J_USR_SIG, 3V3-referenced, J_USR_3V/J_USR_GND for power/ground) ---- */
#define PIN_USR_1        0
#define PIN_USR_2        1
#define PIN_USR_3        2
#define PIN_USR_4        3
#define PIN_USR_5        4
#define PIN_USR_6        5

/* ---- 12V rail soft-latch enable ----
 * Drive HIGH as early as possible in firmware to latch Q2 on (via Q1),
 * keeping 12V_PWR alive after the external wake button is released.
 * Until this pin goes high, only the physical J_WAKE button holds power up.
 */
#define PIN_12V_PWR_EN   6

/* ---- Pump enables (direct low-side MOSFET gate drive, Q3/Q4) ---- */
#define PIN_PUMP1_EN     7
#define PIN_PUMP2_EN     8

/* ---- Right motor (TB6612FNG channel B) ---- */
#define PIN_RM_PWM       9
#define PIN_RM_IN2       10
#define PIN_RM_IN1       11

/* ---- Motor driver standby (shared by both channels) ---- */
#define PIN_MOTOR_STBY   12

/* ---- Left motor (TB6612FNG channel A) ---- */
#define PIN_LM_IN1       13
#define PIN_LM_IN2       14
#define PIN_LM_PWM       15

/* ---- Tank level sensors (3144 Hall effect, active-high) ---- */
#define PIN_TANK_FULL    16
#define PIN_TANK_EMPTY   17

/* ---- Wheel encoders (3144 Hall effect, active-high) ---- */
#define PIN_RM_ENC_B     18
#define PIN_RM_ENC_A     19
#define PIN_LM_ENC_B     20
#define PIN_LM_ENC_A     21

/* ---- Spare GPIOs, broken out to J_SPARE_SIG (3V3-referenced) ---- */
#define PIN_SPARE1       22
#define PIN_SPARE2       26
#define PIN_SPARE3       27

/* ---- Battery voltage sense ----
 * Resistive divider R4 (47k, to V_BATT) / R5 (10k, to GND).
 * Vbatt = Vadc * (R4+R5)/R5 = Vadc * 5.7
 */
#define PIN_VBATT_SENSE     28
#define ADC_VBATT_INPUT     2      /* GPIO28 = ADC channel 2 */
#define VBATT_DIVIDER_RATIO 5.7f

/* ---- Not used on this board (reserved on the Pico module itself) ----
 * GPIO23 - SMPS power-save mode control
 * GPIO24 - VBUS sense
 * GPIO25 - onboard LED
 * GPIO29 - VSYS sense / ADC3
 */

#endif /* GR6_PINS_H */
