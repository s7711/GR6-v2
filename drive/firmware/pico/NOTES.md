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

## Fixed encoder direction decode (2026-08-28)

First attempt was an "alternation-required" decode: only count a step
when the *other* channel's pulse completes next, treating a repeated
same-channel pulse (backlash/jitter) as a no-op. Bench-tested on the
real spinning LM motor and it still failed (`position` stuck near 0
despite ~130 real edges/channel over 2s, confirmed genuinely happening
by eye via the pump-style LEDs wired onto the encoder lines) - because
alternation (A,B,A,B,...) happens on *every* cycle regardless of
rotation direction; individual transition order alone carries no
direction information for two phase-offset unipolar channels like
these.

The actual usable signal is the timing asymmetry already documented
above: the genuine ~45 degree gap (short, 3-6ms) versus the ~135
degree wrap-around gap (long, 14-21ms). Only the short transition's
channel order reveals which sensor is really leading; the long one is
skipped for counting to avoid double-counting/flip-flop. Bench-
confirmed on LM: forward spin gave `position = -132` against 132 real
short-transition events (clean, consistent sign throughout, no more
cancelling), and a matched-duration reverse spin brought it back to
`-2` - correctly flipping sign on reversal. Implemented in `main.py`
as `_lm_encoder_edge`/`_rm_encoder_edge`, replacing the old ISRs.

Caveats:
- ~~`ENC_SHORT_THRESHOLD_US` is a fixed cutoff validated at only one
  bench speed~~ - this bit navigate for real on 2026-08-30 and was
  replaced with a ratio-based decode; see "Adaptive (ratio-based)
  decode" below.
- **Resolution is coarse**: 2 counts per revolution per motor (only
  the short transition counts), same order as the original x2 decode,
  just with correct sign now. This is exactly where the time-based
  velocity estimate (below) is expected to help.
- ~~RM's sign convention is unconfirmed~~ - confirmed on real hardware
  2026-08-29, see below. Originally guessed to mirror the Arduino's
  RM-vs-LM convention; turned out to need the *opposite* pairing once
  everything was actually wired up.

## Sign convention confirmed on real hardware (2026-08-29)

Fitting the new board into amundsen turned up two independent wiring
mix-ups, found and fixed one at a time using an open-loop (no SV/PID)
spin test on each motor output in turn, watching which physical wheel
moved:

1. The motor+encoder connectors for left/right were plugged into the
   wrong PCB headers (a physical mix-up, not a PCB fault) - driving
   the "LM" output moved the physical right wheel, and its encoder
   counted on `RM_position`, not `LM_position` (fully consistent both
   ways, i.e. the motor and its own encoder were crossed together as
   a pair). Fixed by swapping which connector plugs into which header
   - no code change needed for this part.
2. With the connectors corrected, both wheels turned *backwards* on
   what should have been each channel's forward command. Fixed by
   swapping motor lead polarity on both motors (not the encoders).

After both fixes, confirmed by direct observation (wheels turning the
robot's real forward direction) that `_lm_encoder_edge` needs the
*mirrored* convention and `_rm_encoder_edge` needs the *plain* one -
the opposite pairing from the original guess (which had assumed LM
plain / RM mirrored, based on the Arduino's RM-vs-LM convention).
`main.py` updated accordingly. If either motor's connector or lead
polarity is ever changed again, this sign pairing will need re-
checking with the same open-loop spin test.

## Adaptive (ratio-based) decode (2026-08-30)

The fixed `ENC_SHORT_THRESHOLD_US` (10000) caused a real outdoor
`navigate` abort: `heading error -72deg exceeds limit 70deg`, with
`left_mps`/`right_mps` diverging wildly (`0.089` vs `0.512` against a
`0.3` target) over about 9 real seconds before the abort - gradual,
not sudden, and asymmetric between wheels.

Bench diagnostic (comparing raw rising-edge count against the
threshold-decoded position, on the bench indoors, open-loop, no PID):
found a hard cliff, not a gentle miscalibration - `position` reported
exactly zero below ~56 rises/s on both motors, despite the raw edge
count climbing normally the whole time (i.e. real motion, completely
undetected):

```
duty=0.15  raw_rises=139  threshold_pos=0    <-- zero despite real motion
duty=0.20  raw_rises=208  threshold_pos=0    <-- zero despite real motion
duty=0.30  raw_rises=340  threshold_pos=-168  ratio=0.988  (fine)
```

The math lines up: at ~56 rises/s the real ~45 degree short gap works
out to almost exactly 10ms; below that speed the short gap takes
*longer* than the fixed cutoff and gets thrown away along with the
genuine long gap, so nothing ever counts. This is almost certainly
what actually caused the field abort - if a wheel's real speed dipped
under this cliff, the PID would see zero feedback despite commanding
real power, wind up its integral term, and drive it far harder than
intended for real, while still reporting near-zero - matching the
gradual, asymmetric divergence seen in the log.

Fix: classify each transition against the duration of the *previous*
transition (short = under `ENC_RATIO` (0.5) of it) instead of a fixed
absolute cutoff. The short:long ratio (~1:3) holds regardless of
speed, only the absolute durations scale - so this removes the cliff
entirely rather than just relocating it. Bench-confirmed accurate
(97.7-99.8% of the raw-edge-count reference) at every tested speed
on both motors, including the two speeds that reported exact zero
before, all the way down to each motor's own physical stall point (no
decode-side floor found - LM stalls around duty 0.08, RM around 0.10;
above that both are correctly decoded immediately, no gradual
degradation zone). Implemented in `main.py`, replacing
`ENC_SHORT_THRESHOLD_US`.

Known limit, inherent to any 2-channel incremental encoder, not
specific to this implementation: right after a stop (or a direction
reversal), there's no previous transition to compare against, so the
very first transition is genuinely ambiguous. Costs at most one
transition per start/reversal, versus the old cliff's entire wide
speed band of silence.

## Stop now coasts instead of braking (2026-08-30)

Reported after the first real outdoor run: stopping used to glide on
the old board, now it's an abrupt jam-on-the-brakes. `SV 0 0`'s
"coast to zero speed" bypass of the `Am` ramp (see the Arduino
original's own comment saying exactly that) is unchanged and still
correct - the actual bug was in `set_motor_speed()`'s direction-pin
logic, ported byte-for-byte from the Arduino: `if motor_speed > 0: ...
else: ...` has no genuine third case for zero, so a stop fell into the
"reverse" branch, setting IN2 high. Checked against the real TB6612FNG
datasheet: `IN1=L, IN2=H, PWM=L` is defined as **short brake**, not
stop - true stop (outputs off, genuine coast) needs `IN1=L, IN2=L`.

This exact same two-way branch was in the original Arduino code too,
but never mattered there: the old L298 board's separate Enable pin
cuts the outputs entirely whenever PWM is 0, regardless of what IN1/
IN2 say, masking the missing zero-case by accident. The TB6612 doesn't
work that way - PWM-low with a direction still selected is explicitly
a braking state on this chip, not a simple disable. Identical
software, genuinely different real-world behaviour, purely from the
driver-chip swap.

Fixed with a real three-way branch in `set_motor_speed()` (stop/
forward/reverse), using the already-deadbanded `abs_speed` to decide
"stop" so it also covers small values inside the deadband, not just
exactly zero.

## Time-based velocity estimate - not yet built

Still the planned next step: use the precise per-edge microsecond
timestamps (already proven out in the bench diagnostic scripts) for a
velocity estimate, rather than the current fixed-100ms-window
position-delta approach, which is noisy at low speed (few counts per
window) and needed the `Kb` filter to compensate at the cost of added
lag. Needs calibrated angular widths for the two distinct segment
types (the ~45 degree and ~135 degree gaps), since they are not
symmetric. Direction still comes from the fix above - timing only
gives speed magnitude, not sign. Bench-test independently (motor spin
+ logged edges/position) before combining into the real PID loop, same
approach used throughout this work, so a fault in one piece isn't
confused with a fault in another.

## Test rig status (2026-08-28)

Amundsen's two original motors (both the same design) remain on the
robot. Two spare motors were bought hoping to match them for bench
testing, but turned out to have a different gearbox internally, with
no way to fit encoder magnets - so only one motor+encoder bench rig
exists right now (used for all the LM testing above), built from a
third, different spare motor. Options discussed for the right-motor
channel: test the electronics alone against the spare bench motor;
prove PID on LM first then physically swap the same bench motor onto
the RM channel for its own test; or the higher-risk step of pulling
the working electronics out of Amundsen entirely and fitting the new
board. No decision made yet on which.
