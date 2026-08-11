# PRD: Waterbutt Valve Service

## Problem Statement

The robot fills from a water butt via a spout controlled by a pinch
valve, driven by its own standalone ESP8266 (`documentation/
Waterbutt_261112-3.ino`) — not part of the GR6-v2 Pi at all, a separate
microcontroller on the same wifi network. Its firmware only exposes two
plain HTTP endpoints, `/open` and `/close` (plus a `/` status page),
with no concept of duration: leaving the valve open for a controlled
amount of time is left entirely to whatever calls it. It does have its
own 5-second fail-safe — if nothing calls `/open` again within 5
seconds of the last one, the valve auto-closes — so a caller that goes
away (crash, wifi drop) fails safe by default, but a caller that
*wants* the valve open for longer than 5 seconds has to keep re-calling
`/open` to hold it there.

Nothing in this repo talks to it yet. This service is that caller: a
small, standalone web UI to run the valve for an operator-chosen
duration, independent of `navigate`/`jobs` (which don't yet do
anything with the water butt — see top-prd.md's future "job" idea
of stopping near it, turning, and watering automatically; this service
doesn't presume that exists yet, and would likely be reused *by* it
rather than being replaced).

## Solution

A new service, `waterbutt`, following this project's usual shape
(Flask + shared header/template, `config.yaml`-driven, systemd unit,
port 8009). Like `wheelspeed`, it has no path-following/motor
responsibilities and never touches `drive` — it only talks to the
ESP8266 valve controller over plain HTTP on the local network.

### Duration timing (owning what the firmware doesn't)

The firmware's fail-safe (5s) is the one and only thing standing
between "valve stuck open" and safety, so this service leans on it
rather than replacing it — see `control.py`'s `ValveController`:

- **Go**: calls `/open` immediately, and again every
  `REOPEN_INTERVAL_S` (3s — comfortably inside the firmware's 5s
  window) for as long as the requested duration lasts, via a
  background tick loop (`app.py`'s `_tick_loop`, 2Hz).
- **Duration elapses**: the next tick calls `/close` and returns to
  idle — no separate timer thread per run, one shared tick handles it.
- **Stop**: always calls `/close` immediately regardless of current
  state (safe to press even if nothing is running) and cancels any
  pending re-open.
- If the network/ESP8266 is unreachable, `/open`/`/close` calls just
  log a warning and give up for that tick (`requests` timeout, see
  `VALVE_TIMEOUT_S`) — no retry loop here, because the firmware's own
  fail-safe already guarantees the valve shuts itself within 5s of the
  last successful `/open`, which is the actual safety property that
  matters. This service doesn't need to reimplement that.

`ValveController` takes `send_open`/`send_close` as injected
functions (same pattern as `navigate`'s `PathRunner` taking
`send_velocity`/`send_pump`), so the timing logic is unit-testable
without a real ESP8266 — see `test_control.py`.

### Page

One page ("Run"), matching the ask:

- A **duration slider** (`form-range`, per `ui-style.md`'s component
  mapping) over a fixed, non-linear set of steps — 1s, 2s, 5s, 10s,
  20s, 50s, 2 minutes (`DURATIONS_S` in `app.py`) — with the selected
  duration shown as text next to it, since a plain slider alone
  doesn't communicate "50s" vs "2 min" clearly at a glance.
- **Go** and **Stop** buttons. Stop is only enabled while a run is in
  progress (mirrors `navigate`'s Run page's Start/Stop pattern); Go
  disables the slider for the duration of a run so the operator can't
  submit a second, overlapping request.
- Live state (`idle`/`watering`) and remaining seconds via a websocket
  (`/ws/waterbutt`, 2Hz) — same `connectWs`/`fillFields` pattern every
  other service's live page already uses.
- `/go`'s duration is validated server-side against `DURATIONS_S` too
  (not just the slider's own range) — a stray or malicious client
  can't request an arbitrary open-ended duration.

### QC marker

Added 2026-08-09: a pre-fill sanity check for once an automatic
"drive to the water butt and fill" job exists (see "Out of Scope"
below) — the robot doesn't always stop in exactly the right spot, and
a miss risks pouring water over the electronics (uncovered by design,
for now, to let heat escape from a prototype build). Deliberately
**independent of GNSS** — GNSS is exactly what's unreliable at close
range near a marker, so a check built on it would share the same
blind spot it's meant to catch.

Instead this reads `aruco`'s live per-marker `tvec` (camera-to-marker,
in the raw camera frame) and re-expresses it in body-frame terms —
X forward, Y right, Z down, the same convention as the rest of this
project's vehicle-frame math — using the existing, calibrated
`coords.displacement_camera_to_body()` (the same transform
`survey.py` already uses; see `aruco-prd.md`). No nav/GNSS fix is
needed for this at all, only the tvec plus the camera's own static
mounting calibration (`hpr_cb`) — genuinely independent, not just
downstream of the same fix.

One saved record only (`waterbutt/qc_marker.py`, `waterbutt/data/
qc-marker.yaml`) — one waterbutt, one marker allowed for this check,
always overwritten on save, no list of named records the way
`navigate`/`jobs` save paths/jobs. A new **QC Marker** page lists
whichever markers are currently visible (live, via `aruco`'s own
`/ws/aruco`) with their live body-frame vectors, and a per-row "Save
as ideal" button. The Run page then shows a **Distance from ideal**
line with three states:

- nothing saved yet → "No QC marker set — see the QC Marker page"
- saved, but that marker isn't currently visible → "No marker
  visible (expecting marker `<id>`)"
- saved and visible → the live vector compared against the saved one,
  per-axis (forward/right/down) plus the overall distance — a sideways
  miss and a too-far-back miss mean different things for whether
  water lands where it should, so both are shown, not just one
  number.

For now this is just a measurement, to see whether it actually tracks
real positioning drift usefully before anything gates on it — becomes
a real safeguard (refusing an automatic fill past some threshold)
once there's an automatic approach job to gate.

**Rotation-independence fix (2026-08-09)**: the first version compared
`displacement_body_frame` directly (marker position expressed in the
robot's *current* body axes) — but since that frame rotates with the
robot, a pure heading difference between the ideal reading and a live
one showed up as a false positional error, even at the same physical
spot (found live: Ben moved much closer to the ideal position, but the
reading didn't reflect it). Fixed by also saving/comparing the raw
`rvec`, using a new `coords.qc_marker_delta_body_frame()`: it computes
each reading's camera position in the *marker's own* fixed frame
(`camera_position_in_marker_frame()` — the marker doesn't move, so this
frame doesn't rotate with the robot the way body-frame does), differences
those, then rotates the result back into body-frame terms anchored to
the *ideal* reading's orientation — so it stays interpretable as
forward/right/down, just "as the robot was facing when the ideal was
saved." Verified numerically (`aruco/test_coords.py`'s
`TestCameraPositionInMarkerFrame`/`TestQcMarkerDeltaBodyFrame`) before
trusting it, given `coords.py`'s own documented history of a real
sign/transpose bug in a similar rotation composition.

The comparison itself now runs server-side in `waterbutt` (`qc_check.py`,
tested in `test_qc_check.py`), not the page's own JS — unlike every
other live comparison in this project, it needs real rotation-matrix
math, which stays far more trustworthy done once in Python (reusing
`aruco/coords.py`, cross-imported the same way `jobs` imports
`navigate`'s `paths.py`) than hand-derived a second time in JavaScript
with no test coverage. `app.py` runs a small background thread
(`_qc_loop`) that connects to `aruco`'s `/ws/aruco` as a client (a
websocket, not `FeedClient`'s Unix socket, since `aruco` is a separate
service) and republishes the result over `waterbutt`'s own
`/ws/waterbutt` — the Run page just renders whichever state arrives.
One real gotcha hit along the way: that client connection must use
`127.0.0.1`, not `localhost` — unlike `requests`/curl, the
`simple_websocket` client doesn't fall back from IPv6 `::1` to IPv4 on
a refused connection, and Flask's dev server here only listens on IPv4.

**Funnel offset (2026-08-09)**: the check was still tracking the
*camera's* position, not the water spout's target — the funnel is
~23cm behind the camera on the same rigid mount, not co-located with
it. Added `waterbutt_funnel_offset_c` to `config.yaml`'s top level
(a robot-wide physical measurement, like `xnav_ip`, not scoped under
one service's own section) — `[-0.228, 0, 0]` in `c` (X forward out of
the camera, Y right, Z down; funnel and camera are both centred, so Y
is zero, and Z genuinely doesn't matter for this check). `coords.py`
gained `target_position_in_marker_frame()`: since the offset is fixed
*relative to the camera*, it has to be rotated by that reading's own
orientation before being added — the same reasoning as the camera
position itself needing de-rotation, just one step further removed.
`qc_marker_delta_body_frame()`'s old (camera-only) behaviour is just
this function's zero-offset case, not a separate code path. Verified
numerically again before trusting it: a point offset behind the
camera stays rotation-invariant too, not just the camera itself (see
`test_coords.py`'s `test_zero_for_a_pure_rotation_with_a_real_offset_
behind_the_camera`).

### QC gating on fill

Added 2026-08-11: everything above only ever *measured* the QC
distance — nothing yet stopped a fill from actually happening when it
was bad, or missing entirely. "No QC marker means no fill", not "no
QC marker means skip the check": `POST /go` now refuses (`409`, with
a specific `reason`) unless `qc_check.passes()` is true, which
requires state `"ok"` *and* the live distance within a threshold —
`"not_configured"`, `"not_visible"`, `"aruco_unreachable"`, or any
other non-`"ok"` state all refuse the same as being too far away, none
of them silently pass. This runs in `app.py`'s `/go` route (not
`ValveController`, which stays timing-only and has no QC knowledge at
all) — the same place `duration_s` is already validated against
`DURATIONS_S`.

The threshold is selectable, not fixed — `qc_threshold_m`, an
allow-listed choice (`QC_THRESHOLD_OPTIONS_M`: 5/8/10cm, same
"operator picks from a fixed set, server re-validates" reasoning as
`DURATIONS_S`) on the Run page next to the duration slider. A `/go`
call that doesn't specify one at all (`jobs`' own `fill` step, which
has no threshold selector of its own yet — see jobs-prd.md) gets
`QC_DEFAULT_THRESHOLD_M` (8cm). `jobs`' `waterbutt_go()` surfaces the
refusal's actual `reason` (not a generic "waterbutt refused") so a
`fill` step's abort reason says *why* — too far, not visible, or
never configured — not just that it failed.

### Icon

Per the ask: a bucket-with-water-drop glyph, white-on-black-circle,
matching every other service's `icon.<ext>` convention (see
`ui-style.md`'s component mapping table and e.g. `oxts-nav/icon.svg`).
Hand-drawn for this project (not a traced/embedded copy of the
referenced `svgviewer.dev` bucket, just the same general shape) to
keep licensing simple, consistent with how other services' icons note
their own source/license in a comment.

### Config

```yaml
waterbutt:
  unit: robot-waterbutt.service
  host: 0.0.0.0
  port: 8009
  web_ui: true
  hostname: waterbutt.local   # mDNS name the ESP8266 advertises (WiFi.hostname("waterbutt") + MDNS.begin("waterbutt")) — per the ask, this is the one thing about "which network device to talk to" that's config, not hardcoded
  qc_marker_file: waterbutt/data/qc-marker.yaml  # the single saved "ideal position" record — see "QC marker" above
```

`hostname` (not the wifi SSID/password — those live only on the
ESP8266 itself, and are already redacted out of the committed
firmware source) is the "network name" asked for as a config
parameter: it's how *this* service finds the valve controller on the
LAN, resolved via mDNS/avahi (the Pi and the ESP8266 need to be on the
same wifi network — normally amundsen's own hotspot or whatever
network `network`'s wifi client profile is on).

## Out of Scope

- Any automatic "drive to the water butt, turn, wait, water" job
  logic — that's the future `jobs` service's job (see top-prd.md
  and navigate-prd.md's "turn on the spot" note), which would call
  this service's `/go`/`/stop` the same way an operator's browser does,
  not duplicate the timing logic.
- Flow/level sensing, leak detection, or any feedback that the valve
  actually opened (the firmware has no sensor for this) — purely
  open-loop, matching what the firmware itself supports.
- Firmware changes to the ESP8266 sketch itself — out of scope for
  this Pi-side service; the `.ino` is kept here only as reference
  documentation for how the fail-safe/endpoints behave.
