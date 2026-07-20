# Drive Service — User Manual

This is a how-to-use guide for the `drive` service's web pages. For the
design/engineering background (why things work the way they do — the
firmware protocol, why there's no server-side command resend, the
manual/automatic control arbitration, real bugs found along the way),
see `drive-prd.md` instead — this document is deliberately just "how do
I use it."

Open the drive service from the manager's home page, or go directly to
its address (port 8005 by default) — e.g. `http://amundsen:8005`.

This service owns the USB-serial connection to the motor-controller
microcontroller (currently an Arduino, moving to a Raspberry Pi Pico
later) directly — it doesn't depend on any other GR6-v2 service to
function.

## Home page

Manual "jog" control, plus live telemetry.

- **Jog square** — drag inside it with a mouse or finger; releasing
  stops the motors immediately. While held, a command re-sends roughly
  every 300ms — that's deliberate (see "If jogging feels unresponsive"
  below), not a bug.
- **Controller** — shows whether `drive` is currently obeying manual
  (jog) or automatic (a future `navigate`/`missions`) commands. Manual
  input always wins immediately and holds control for half a second
  afterwards, so an automatic caller can't sneak in between two jog
  updates.
- **Pump** — a plain on/off toggle. Not connected to anything yet on
  the real robot at time of writing — safe to click.
- **Position / velocities / ramp state** — live readouts, useful for
  hand-verifying against a tape measure/stopwatch if something looks
  wrong (see below).
- A firmware-version-mismatch banner appears here automatically if the
  microcontroller's reported version doesn't match what's expected —
  see "If the firmware banner appears."

## Tuning page

Live PID/filter constants for the motor control loop, plus scrolling
graphs of how the control loop is actually behaving.

### What each constant does (in plain terms — see
`drive/firmware/GR6_motor.ino` for the actual formulas)

- **Kp (Proportional gain)** — how hard the motor pushes back *right
  now*, in proportion to today's speed error. Higher = snappier
  response, too high = oscillation/overshoot.
- **Ki (Integral gain)** — how hard the motor corrects for an error
  that's persisted for a while. This is what eventually erases a
  steady-state error that Kp alone can't (e.g. one wheel needing
  slightly more power than the other to hold the same speed).
- **Kd (Differential gain)** — reacts to how *fast* the error is
  changing, not just its size — damps overshoot/oscillation. Currently
  `0` on both wheels (not in use).
- **Kf (Feed-forward gain)** — adds a contribution straight from the
  *target* speed itself, before any error has even had a chance to
  appear — a head start rather than a correction. Currently `0`.
- **Ka (Differential-error filter)** — smooths the error's rate-of-
  change before Kd sees it. `0` = no smoothing (raw); closer to `1` =
  heavier smoothing/slower to react.
- **Kb (Velocity filter)** — smooths the *measured* speed before it's
  compared against the target. Same scale as Ka: `0` = raw, closer to
  `1` = heavier smoothing.
- **Db (Deadband)** — motor commands below this are treated as zero,
  so the motor doesn't buzz/twitch trying to hold a barely-there speed.
- **Mi / Mj (Maximum / minimum integral error)** — caps on how far the
  integral term (see Ki) can wind up in either direction, so a
  long-stuck error can't eventually demand a huge, sudden correction.
- **Id (Integral decay)** — how much of the integral term's memory
  carries over each control cycle. Close to `1` = long memory; lower =
  forgets old error faster.
- **Am (Maximum acceleration)** — limits how quickly the *commanded*
  speed itself is allowed to ramp, in counts/second per second — smooths
  out a sudden jump in requested speed, independent of the PID loop
  itself.

Each of these is a **pair** — left wheel, right wheel — since the two
motors don't behave identically.

### Setting a value live

Type a new left/right pair into a row and click **Set** — takes effect
immediately, but only until the microcontroller resets (e.g. `drive`
restarts, or the board is power-cycled). There's no range-checking
here (neither of us knows a universally safe bound for every one of
these) — a bad value can make the robot lurch, so change one at a time
and watch the graphs.

### Making a value stick across restarts

Add it to the shared config file, under `drive:` → `tuning:` — for
example, to make the current `Kp`/`Ki` values durable:

```yaml
drive:
  # ...existing drive config...
  tuning:
    Kp: [1.0, 1.0]
    Ki: [3.0, 3.0]
```

Each entry is `ParamName: [left, right]` — only list the ones you
actually want to override; anything not listed keeps the firmware's own
built-in default (shown as its starting value on this page before you
change anything). Edit this via the manager's Config page, then restart
`drive` (from the manager's Services page) — the new values are pushed
to the microcontroller automatically, right after `drive` reconnects to
the serial port.

### The graphs

Two scrolling charts (left motor, right motor), each with all six of:
set velocity, filtered (measured) velocity, error, integrated error,
differential error, and motor output — in the firmware's own native
units (counts/second, raw PWM), not m/s, since that's what these gains
actually act on. Reproduces GR6-v1's equivalent tuning graph. Watch
these while adjusting a constant above — that's the actual point of
this page.

## Ultrasonics page

A top-down diagram of the five ultrasonic sensors' current readings —
green (clear, &ge;600mm), amber (getting close, 200&ndash;599mm), red
(near, &lt;200mm), or grey ("no echo" — this can mean a genuinely clear
path *or* a sensor angle/surface that just doesn't reflect well; the
two aren't distinguishable from this reading alone).

Sensor ids (`U0`&ndash;`U4`) match the firmware's own numbering, not
renumbered for this page:

- **U0** — front-right, facing forward
- **U1** — back-right, facing right
- **U4** — front-left, facing forward
- **U3** — back-left, facing left
- **U2** — back-centre, facing backward

Not currently used for any automatic obstacle-avoidance — that's
future work, once these are trusted (see drive-prd.md). This page
exists purely so you can judge how well they're actually working.

## Config page

A read-only summary of `drive`'s configuration — serial port, wheel
calibration constant, manual-control hold time, expected firmware
version, and any configured tuning overrides. Edit the underlying
values from the manager's Config page, not here.

## If jogging feels unresponsive

The jog page only actually sends a command roughly every 300ms while
held (dragging the joystick around doesn't send any faster than that)
— this is deliberate, to avoid flooding the connection and the systemd
journal with requests (see drive-prd.md's "Real bugs found via
testing"). It shouldn't feel laggy in practice, but it's not
instantaneous on every pixel of movement either.

## If the firmware banner appears

Means the microcontroller reported a different version string than
`drive` expected (`expected_firmware_version` in config). Usually means
new firmware was uploaded but the config wasn't updated to match, or
vice versa — check `drive/firmware/GR6_motor.ino`'s own version-history
comment against what the Home page banner reports, and update whichever
one is stale.

## If a reported speed/position looks wrong

Compare the Home page's live position/velocity readouts against a tape
measure and stopwatch. If they're self-consistent (a doubled real-world
speed shows as roughly double the reported one) but the absolute number
is off, that's almost certainly `counts_per_metre` needing
re-measurement (see drive-prd.md) — not a code bug.
