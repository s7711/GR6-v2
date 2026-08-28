# Pico firmware notes

## Status as of 2026-08-28

`main.py` is a line-for-line MicroPython port of
`drive/firmware/arduino/GR6_motor.ino` (v260720), sharing the same wire
protocol (`drive/protocol.py`) so the Pi side needs no changes beyond
`serial_port` in config once the Pico replaces the Arduino. See the
module docstring in `main.py` for the two copy/paste bugs found and
fixed during the port (RM deadband/D-term using the wrong L/R constant).

Every GPIO on the board has been individually bench-tested and
confirmed working: `PWR_12V_EN` (and the Q1/Q2 power latch it drives),
`MOTOR_STBY`, all four motor direction pins, both PWM pins, both pump
enables, all six `USR` ultrasonic pins (single-pin trigger+echo, real
SR04 hardware), `VBATT_SENSE`, all four Hall-encoder inputs, and both
tank-level inputs.

## Known issue: encoder direction decode does not work with the A3144s

The Arduino's decode (and this port's direct copy of it) is a classic
X2 quadrature trick: an interrupt fires on channel A's edges only, and
reads channel B's *current level* at that instant to decide direction
(`if A == B: position--; else: position++`). That trick requires the
two channels' active windows to *overlap*, so that A's two edges
(rising and falling) land on genuinely different B levels depending on
rotation direction.

Our A3144s are unipolar (only the South-pole magnets trigger them; see
the sensor-polarity discussion this session) and mounted 45 degrees
apart to get proper quadrature phase against the 4-magnet ring
(N/S/N/S at 90 degree spacing, confirmed on the bench: LM_ENC_A/B show
clean, chatter-free single transitions per magnet pass). But because
each channel's pulse is narrow and the two channels' pulses are fully
sequential rather than overlapping - the real edge order measured on
the bench (2026-08-28, motor spinning, 60% duty) was:

```
A-fall, A-rise, B-fall, B-rise, (long gap), A-fall, A-rise, B-fall, B-rise, ...
```

i.e. triggers at roughly 0 deg (A), 45 deg (B), then a long coast to
180 deg (A), 225 deg (B) - matching the physical layout (S magnets
180 degrees apart, sensors offset 45 degrees). Because A's whole pulse
completes before B ever moves, sampling B's level at *both* of A's
edges reads the *same* B value each time, so every A pulse contributes
+1 then -1 (or vice versa) - a guaranteed cancel to ~0 regardless of
actual rotation direction. Confirmed on the bench: 200 real A edges
during a 2-second spin produced a net decoded position of 0-1, not the
expected ~200.

This is not a wiring or sensor fault - the 3144s, mounting, and 45
degree spacing are all confirmed correct and clean. It's a mismatch
between the decode algorithm (suited to overlapping ~50% duty
channels, which is what the old board's linear SS49E Hall sensors
would have produced through a comparator) and the actual non-
overlapping pulse shape unipolar digital switches like the A3144
produce.

## Plan (agreed 2026-08-28, staged)

1. Replace the level-sampling X2 decode with an alternation-required
   order-based decode: track which channel's pulse last completed:
   only count a step (and only then decide direction) when the
   *other* channel's pulse completes next. If the same channel fires
   twice in a row (wheel rocking/backlash without a full step,
   plausible at low speed or right at motor start/stop), treat it as
   a no-op rather than guessing - safer to undercount briefly than to
   misattribute direction.
2. Separately (once (1) is bench-proven), add a time-based velocity
   estimate using the precise per-edge microsecond timestamps
   (already proven out in the bench diagnostic scripts) rather than
   the current fixed-100ms-window position-delta approach, which is
   noisy at low speed (few counts per window) and needed the `Kb`
   filter to compensate at the cost of added lag. Needs calibrated
   angular widths for the two distinct segment types (the ~45 degree
   A-to-B gap and the ~135 degree B-to-next-A gap) since they are not
   symmetric. Direction still comes from (1) - timing only gives
   speed magnitude, not sign.

Both steps get bench-tested independently (motor spin + logged
edges/position, same technique used to find this bug) before being
combined into the real PID loop, so a fault in one is not confused
with a fault in the other.
