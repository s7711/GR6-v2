# PRD: Motor / Drive Service (drive)

**Status: draft, for discussion — nothing built yet.**

## Problem Statement

The robot needs a service that talks to the motor-controller microcontroller
(currently an Arduino, moving to a Raspberry Pi Pico once the new PCB is
fitted) over serial: sending motor speed and water-pump commands, and
reading back encoder counts, ultrasonic ranges, and control-loop
diagnostics. Nothing in this repo does that yet — GR6-v1 had this logic
mixed into one process alongside path-following and everything else (see
Prior art). This service is the low-level "make the wheels/pump do what
I say, tell me what the sensors see" layer everything else (future
`navigate`, `missions`, `safety`, and a future wheelspeed-GAD sender)
will be built on top of.

Explicitly not in scope here: path-following, waypoint navigation,
mission scripting, obstacle-avoidance decisions, or sending wheelspeed
GAD updates to the xNAV650. Those are separate, later services that will
*consume* what `drive` publishes — see "Out of Scope" below and the
"Suggested migration order" in `top-prd.md`.

## Prior art (GR6-v1)

`/home/pi/share/python/GR6-v1/motors.py` — a `MotorController` class:
opens the serial port directly (`pyserial`, `/dev/ttyUSB0`, 115200 baud),
runs a background read thread parsing tagged telemetry lines into a
`motor_state` dict, and a `send(lm, rm, w)` method that writes an `SV`
and a `WP` command every call. Also supported raw passthrough of
tuning commands prefixed with `&` from the web UI straight to the
Arduino. Reused: the overall shape (background reader thread, a
`send()` that's called repeatedly rather than a fire-and-forget
command) and the wheel-geometry knowledge (see below). Not reused
as-is: it's tightly coupled into one big process with no clean
consumer interface for other code — this service replaces it with
something other services can actually subscribe to.

`/home/pi/share/python/GR6-v1/gad_wheelspeed.py` — a working prototype
of wheelspeed GAD aiding (per-wheel velocity, `oxts_sdk.GadVelocity`,
lever arm from IMU to each wheel via `HPR_ib`). Confirmed `GadVelocity`
worked; `GadSpeed` reportedly never did. This is valuable prior art for
the wheelspeed-GAD service logged in `top-prd.md`, but it isn't part of
`drive` — it's a future consumer of `drive`'s telemetry, same
relationship `aruco` has to `oxts-nav`'s nav feed.

`/home/pi/share/python/GR6-v1/static/drive.html` — a manual-drive page:
camera feed with a joystick overlay, plus a table of live telemetry.
Good prior art for `drive`'s own live/jog page.

Wheel geometry, from `motors.py`'s own comments (empirical, not
precisely measured): roughly 250 encoder counts/metre, and a commanded
speed of 100 (firmware units) gives roughly 160 counts/s ≈ 0.6 m/s. Both
numbers are "about" — worth re-measuring once the real robot is
running, not trusting blindly.

## Firmware interface (from `drive/firmware/GR6_motor.ino`, v260720#1)

This is the direct contract with the microcontroller — documented here
because everything else in this service is built on top of it, same
way `oxts-nav-prd.md` documents the xNAV650's own command set.

**Commands (Pi → microcontroller), ASCII, newline-terminated, two
space-separated integers unless noted:**

| Command | Meaning |
|---|---|
| `SV <left> <right>` | Target velocity per wheel, encoder counts/s, clamped to ±200 by firmware. `SV 0 0` specifically disables the control loop (coasts to a stop) rather than commanding zero via the PID loop. |
| `WP <0\|1>` | Water pump on/off. |
| `Kp/Ki/Kd/Kf/Ka/Kb/Db/Mi/Mj/Id/Am <left> <right>` | PID/filter tuning constants — see firmware comments for meaning of each. **No validation in firmware** ("No checks! Take good care") — sending garbage here can make the robot lurch. |

**Built-in safety already in the firmware, not something `drive` needs
to (re)implement:** if no `SV` command arrives within 2000ms, the
firmware zeroes both target velocities and disables the control loop.
Independently, if no `WP` command arrives within 2000ms, the pump is
turned off. This is the one dead-man's switch in the system — `drive`
forwards commands as they arrive and does **not** add a second timeout
layer of its own (see "Control arbitration" below): if a caller wants a
speed held for longer than 2000ms, that caller (not `drive`) is
responsible for re-sending it. `navigate` will run its own control loop
at 10–20Hz, so this is a non-issue there; the web jog page handles it
client-side (see below).

**Telemetry (microcontroller → Pi), one line per update, cycled
round-robin roughly every 10ms:**

| Tag | Meaning |
|---|---|
| `EN <left> <right>` | Raw encoder position (counts), monotonically increasing/decreasing. |
| `SV <left> <right>` | Current (ramped) set velocity, ×100 int-encoded. |
| `FV <left> <right>` | Filtered measured velocity, ×100 int-encoded. |
| `ER <left> <right>` | Velocity error (set − filtered). |
| `EI <left> <right>` | PID integral term. |
| `ED <left> <right>` | PID derivative term. |
| `MO <left> <right>` | Raw motor PWM output. |
| `WP <0\|1>` | Current pump state. |
| `GO <n>` | Control-loop ramp state (0 = disabled). |
| `U0`…`U4 <mm>` | Ultrasonic range per sensor, mm; `-1` means no echo/out of range. |
| `Kp`/`Ki`/.../`Version` | Echo of current tuning constants and firmware version string, for confirming what's actually running. |

Note: GR6-v1's `motors.py` expected a tag `EA` for the filtered
derivative-error term; the current firmware actually sends `ED`. Not
carrying that mismatch forward — just flagging it as a real discrepancy
between the old Python and the firmware version it was apparently
written against.

## Solution: architecture / data flow

- `drive` owns the serial connection to the microcontroller over
  **USB** (`/dev/ttyUSB0` or similar) — one process, one port, same
  pattern as every other service owning its one piece of hardware. Note
  this is USB, not the Pi's own UART header — that header exists but
  isn't used by anything in this project.
- A background thread continuously reads and parses telemetry lines
  into current state (mirrors `oxts-nav`'s `ncomrx_thread` / `aruco`'s
  detection loop shape: a reader thread feeding shared state, a Flask
  app serving it).
- Commands (`SV`/`WP`) are forwarded to the serial link as soon as
  `drive` receives them — no server-side resend/keep-alive thread (see
  "Control arbitration" below for why, and for what happens instead).
- Web UI, four pages: a live/jog page (joystick or arrow buttons, live
  telemetry table — echoing GR6-v1's `drive.html`), a Tuning page for
  the PID constants plus live scrolling graphs of the control loop
  (Set velocity / Filtered velocity / Error / Integrated error /
  Differential error / Motor output, per wheel — reproduces GR6-v1's
  `motors.html` chart, ported to uPlot per `ui-style.md` rather than
  Chart.js), an Ultrasonics page (the robot-outline sensor diagram
  `ui-style.md` already specifies — colour via success/warning/danger
  thresholds, built here first since `drive` is the only current
  consumer; promote to `shared/` if a second service ever needs it,
  same pattern as `ncom-strings.js`), and a read-only Config page
  (mirrors `camera`'s Config page).
- Cross-service consumption: future services (`navigate`, a
  wheelspeed-GAD sender, `safety`) will need this telemetry without
  re-parsing serial themselves — a Unix domain socket feed,
  `drive_feed`, same shape as `oxts-nav`'s `nav_feed.py` (a small
  pub-style socket publishing the current state dict at a fixed rate).
  See "Feed naming" below for why it's one feed, not split by sensor
  vs. actuator.

## Implementation Decisions

### Units: `drive`'s external interface is m/s, not raw encoder counts
Firmware speaks in encoder counts/s (an implementation detail of *this*
motor/encoder pair). Everything above `drive` — the jog UI, and later
`navigate`/wheelspeed-GAD — should think in real-world units (m/s).
Proposed: `drive` holds a `counts_per_metre` config constant (start from
GR6-v1's ~250, re-measure on the real robot) and converts at the
boundary — commands accepted in m/s, telemetry published in m/s (and
metres, for encoder position), raw firmware units are `drive`'s private
concern. This also means the ~250 constant only has to be measured once
and lives in one place, not copy-pasted into every future consumer.
This is most critical for the future wheelspeed-GAD sender — a bad
`counts_per_metre` there produces a confidently-wrong aiding update, so
when that goes wrong, `drive`'s config is exactly where to look first
(see "Config additions" below).

### Control arbitration: last-command-wins, with a human hold timer
No lock that can *block* a human from taking over — the moment someone
needs manual control is often exactly when something (e.g. `navigate`)
is going wrong, so an override that has to wait or be granted is the
wrong shape for what's ultimately a safety mechanism. Instead:
- Two distinct command endpoints/paths — one for manual/human input
  (the jog page), one for automatic callers (`navigate`, `missions`
  later). `drive` trusts *which endpoint* was called to know the
  source, rather than trusting a caller-supplied "I'm human" claim.
- Any command via the manual endpoint immediately takes control **and
  holds it for a fixed window (proposed 500ms)** — commands arriving
  via the automatic endpoint during that window are rejected/ignored,
  not merely raced against. 500ms is chosen against two known cadences:
  the jog page's own client-side repeat (see below, ~300ms while
  held — comfortably inside one 500ms window even if a single repeat
  is delayed) and `navigate`'s expected 10Hz control loop (~100ms —
  so up to 5 of its attempts get quietly rejected during one human hold,
  which is fine, it just tries again next cycle). Once 500ms passes
  with no further manual command, control reverts to plain
  last-command-wins and whichever caller sends next is obeyed
  immediately — no explicit "release" action needed, releasing the
  joystick is enough.
- `drive` publishes who's currently "in control" (and until when) in
  `drive_feed`, so `navigate`/`missions` can notice they've been
  overridden and back off gracefully (e.g. pause and show "overridden
  by manual control" in their own UI) rather than silently spamming
  rejected commands — a nice-to-have for those future services, not a
  correctness requirement of `drive` itself.
- The web jog page runs its own client-side repeat timer (~300ms,
  while the mouse/finger is down) sending the current stick position —
  this lives entirely in the page's JS, not in `drive` itself, keeping
  with "callers implement their own keep-alive, `drive` doesn't guess
  at policy for them."

### Feed naming: one `drive_feed`, not split by sensor vs. actuator
Motors, encoders, pump, and ultrasonics are all wired to the same
microcontroller and read over the same serial link — whichever service
owns that port owns all of it, splitting ultrasonics into a separate
process would just mean two processes fighting over one port, or one
relaying to the other for no real gain today. The service stays named
`drive` and the feed is just called `drive_feed`, same as `oxts-nav`'s
feed is called `nav_feed` even though it carries far more than one
narrow "nav" concept — the name identifies *whose* feed it is, not an
exhaustive contents list. Revisit only if a genuinely separate sensor
gets its own dedicated hardware later.

### `drive` does not reimplement velocity control
The firmware already has a tuned PID loop per wheel (see the version
history in the `.ino` file — this has clearly been tuned iteratively).
`drive` forwards target velocities and reads back what actually
happened; it doesn't second-guess or replace that loop. Tuning-constant
changes are exposed (see below) but that's adjusting the existing loop,
not building a new one.

### Tuning constants: config-driven, pushed at `drive`'s own startup
No server-side range validation — neither of us currently knows what a
"sane" bound for e.g. `Kp` actually is, so validation here would be
false confidence, not real safety. Instead: `config.yaml` may optionally
hold the settled-on tuning constants for `drive`; right after `drive`
opens the serial port at its own startup, it pushes any configured
values to the microcontroller. This conveniently covers both restart
cases in one code path — classic Arduino boards reset on serial-port-
open (DTR toggle), so opening the port from a fresh `drive` process
tends to coincide with the microcontroller resetting to its own
hard-coded firmware defaults, which `drive` then immediately overwrites
with the configured values. The live tuning page (raw passthrough,
same as GR6-v1's `&`-prefixed commands) stays for active tuning work;
the config file is what makes a good set of values durable across
restarts/reflashes rather than lost the moment someone else fiddles
with the live page.

### Ultrasonics: telemetry only, not acted on, for now
Per your plan (map first, then assess ultrasonic consistency against
it, then build `safety`), `drive` just publishes the five raw ranges
(and the `-1`/no-echo case as a distinct value, not silently dropped) —
it does not stop or slow the motors based on them. That decision
belongs to the future `safety` service once the sensors are actually
trusted.

### Pump control: simple on/off, exposed alongside drive commands
No timed/volume-based dosing logic here — that's a `missions`-layer
concern later. `drive` just exposes on/off and reports current state,
same as the firmware does.

### Firmware version check: log + UI banner, no new alerting system
`drive` reads the `Version` telemetry line at startup and compares
against `expected_firmware_version` in config. On mismatch: log a
warning (via normal Python logging, which lands in the systemd journal
— exactly what the manager's Journal viewer, `manager/manual.md`, is
for) and show a small banner on `drive`'s own Home page ("firmware
version mismatch: expected X, got Y"). Does not refuse to start — a
silent mismatch after re-flashing (or once the Pico swap happens) would
be a nasty thing to debug blind, but an unusable service over a version
string mismatch would be worse. No new alerting infrastructure needed;
this reuses what's already there.

### Pico transition
Keep the serial-protocol parsing isolated from the rest of the service
(one module, not spread through `app.py`) so that when the Pico
firmware arrives — with its different feature set (battery voltage,
on/off control) — swapping it in doesn't mean restructuring the whole
service, just extending/replacing the protocol module and adding new
telemetry fields.

## Config additions (shared config file) — proposed, not yet written

```yaml
drive:
  unit: robot-drive.service
  host: 0.0.0.0
  port: 8005
  web_ui: true
  serial_port: /dev/ttyUSB0       # re-check once Pico is fitted — likely /dev/ttyACM0 or similar
  baud: 115200
  counts_per_metre: 250            # from GR6-v1, needs re-measuring on the real robot — the wheelspeed-GAD sender will depend on this being right
  human_control_hold_ms: 500       # how long a manual jog command locks out automatic callers (navigate/missions)
  expected_firmware_version: "260720#1.GR6"
  tuning:                          # optional — pushed to the microcontroller right after drive opens the serial port
    Kp: [1.0, 1.0]                 # [left, right], matches firmware's per-wheel constants
    Ki: [3.0, 3.0]
    # ... Kd/Kf/Ka/Kb/Db/Mi/Mj/Id/Am as needed — omit any not being overridden from firmware's own defaults
```

## Real bugs found via testing

- **Encoder position wraparound (firmware bug)** — `EN` telemetry went
  through `sendFloat()`, which multiplies by 100 and truncates to a
  16-bit `int` for transmission. Fine for velocities (small values,
  never overflow), but silently wraps encoder position after only ~327
  counts of travel (about 1.3m at ~250 counts/metre) — even though
  firmware's own internal `long LM_position`/`RM_position` never
  wrapped, only the transmitted value did. Found via real hardware
  testing: the robot visibly travelled several metres while the
  reported position told a much smaller/wrapping story. Fixed by adding
  a dedicated `sendLong()` (no scaling, no truncation) for `EN`
  specifically — see the firmware's own version history. Velocity
  (`FV`) was never affected — it's computed firmware-side from the real
  unwrapped position, independently transmitted.
- **Reported speed not matching real-world travel** — a *different*
  issue from the above, still open: `counts_per_metre` (currently
  `250`) is carried over from GR6-v1's own rough, never-precisely-
  measured guess (see Prior art). Once the position-wraparound fix
  above is uploaded, re-measure this properly on the real robot (drive
  a known distance, compare against `LM_position_m`/`RM_position_m`)
  rather than trusting the inherited value.
- **Home page id mismatches** — `home.html` originally referenced the
  firmware's raw wire tags (`GO`, `U0`–`U4`) for element ids, but
  `protocol.py` translates those into friendlier field names
  (`ctrl_enabled`, `ultrasonic_0_mm`, etc.) before the page ever sees
  them, so the lookups silently never matched and those rows stayed
  blank. Fixed by correcting the ids to match the actual published
  field names.
- **Firmware-version banner got stuck** — the original startup check
  waited a fixed 2s for a `Version` telemetry line and froze whatever
  it saw (or didn't) as the permanent answer. But `Version` is only one
  of ~12 tags cycled round-robin, taking ~1.8s to come back round —
  close enough to the 2s window that it sometimes lost the race,
  leaving the Home page banner stuck reporting "no version received"
  forever even after a real version had since arrived. Fixed by
  computing the banner live from current state on every read instead
  (see `app.py`'s `_firmware_status`), with a separate non-blocking
  background log-only check purely for a startup warning.
- **Jog page flooding the systemd journal** — the joystick was POSTing
  a command on every raw `pointermove` event (which can fire dozens of
  times a second while dragging), not just the intended 300ms repeat —
  found when real usage made the journal grow alarmingly fast. Fixed by
  having `pointermove` only update the on-screen thumb (cheap, client-
  only); the 300ms repeat timer is now the sole thing that actually
  sends a command. Also dropped Flask/Werkzeug's per-request access log
  to `WARNING` as a second layer of defence, so routine 200s never hit
  the journal regardless of request rate — our own `logging.warning()`
  calls still come through.

## Testing Decisions

- Protocol parsing (telemetry lines → state dict, state → command
  strings) unit-tested against captured/synthetic serial lines — no
  real hardware needed, same approach as `aruco`'s coordinate-math
  tests.
- Control-arbitration logic (manual hold timer, rejection of automatic
  commands during the hold window, reversion to last-command-wins once
  it lapses) tested against a fake serial object (a `Serial`-shaped
  stub) with a fake clock, rather than real `time.sleep()` — the 500ms
  window shouldn't make the test suite slow.
- Real hardware needed to validate: actual motor response, encoder
  counts-per-metre, ultrasonic sensor behaviour, and the joystick UI
  end-to-end — can't be faked meaningfully.

## Out of Scope (v1 of this service)

- Path-following / waypoint navigation — future `navigate` service.
- Mission sequencing / scripting — future `missions` service.
- Obstacle-avoidance decisions from ultrasonic (or any future vision)
  data — future `safety` service; `drive` only publishes raw sensor
  values.
- Sending wheelspeed GAD updates to the xNAV650 — logged as its own
  future item in `top-prd.md`; needs `drive`'s encoder telemetry to
  exist first, but the GAD-sending itself is a separate consumer
  service, not part of `drive`.
- Pico-specific features (battery voltage, on/off control) — added
  once the new PCB is fitted and its firmware exists; today's scope is
  the current Arduino firmware only.
- Any change to the firmware's own PID tuning algorithm.
