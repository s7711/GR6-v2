# PRD: Wheelspeed Service

## Problem Statement

`top-prd.md` item 6 ("Wheelspeed GAD aiding") has been blocked since it
was first identified while surveying ArUco markers with `aruco`:
position-only GAD from a stationary/known marker doesn't help the INS
solution *between* good fixes, whereas wheelspeed aiding keeps drift
much lower while driving through a GNSS-poor patch. It was blocked on
`drive` actually publishing wheel velocity — `drive` is now done and
publishes filtered per-wheel velocity in m/s (`LM_vel_filt_mps`,
`RM_vel_filt_mps`) over its feed socket, so this is unblocked.

GR6-v1 already had a working implementation (`gad_wheelspeed.py`,
`share/python/GR6-v1`) — this service ports it into GR6-v2's
architecture (its own process, config-driven, a small status page),
rather than reinventing the GAD-sending approach.

## Solution

A new service, `wheelspeed`, following this project's usual shape
(Flask + shared header/template, `config.yaml`-driven, systemd unit,
port 8008). It has no control-loop responsibilities and doesn't touch
`drive`'s motor commands at all — strictly a one-way consumer of
`drive`'s and `oxts-nav`'s existing feeds, sending GAD aiding packets
to the xNAV650, same spirit as `aruco`'s `gad.py`.

Named `wheelspeed`, not `gad-wheelspeed` — GR6-v1's ArUco GAD sender
was informally called "gad-aruco"; this project didn't carry that
naming into `aruco`'s own name, so it isn't carried in here either
(consistent, and shorter).

### Data flow

- **`drive`'s feed** (`drive_feed_socket`, already published at
  `drive_feed_hz`, currently 20Hz) — `LM_vel_filt_mps`/`RM_vel_filt_mps`,
  drive's own filtered per-wheel velocity, already in m/s (drive's
  external interface is m/s throughout, not raw encoder counts — see
  drive-prd.md's "Units"). This is the actual aiding measurement.
- **`oxts-nav`'s feed** (`nav_feed_socket`) — `Vn`/`Ve`/`Heading` and
  `connection.timeOffset`. `timeOffset` converts our own
  `time.monotonic()` reading into GPS time for the outgoing GAD packet
  (via `oxts-nav/ncomrx.py`'s `machine_time_to_gps`, the same
  standalone helper `aruco` already uses for this). `Vn`/`Ve`/`Heading`
  are used only to compute a **display-only** forward body-frame
  velocity for comparison on this service's own page (see below) — this
  is never sent back to the xNAV650 as aiding, which would be circular
  (feeding the INS its own resolved velocity back as if it were an
  independent measurement).
- **Sent to the xNAV650**: one `GadVelocity` or `GadSpeed` packet per
  wheel (config-selectable, see "GadVelocity vs GadSpeed" below), each
  with its own lever arm (IMU → wheel, IMU frame) and scale factor.
  Stream IDs unchanged from GR6-v1: **133** (left), **134** (right).

### GadVelocity vs GadSpeed

GR6-v1 implemented both, selectable via a runtime command
(`self.gad_type`), defaulting to `GadVelocity` — its own comment notes
`GadSpeed` was never confirmed to actually affect the INS solution
("I have been unable to get GadSpeed to do anything... I cannot detect
that it is functional using this code"). Both are ported here unchanged
in substance, but selection is **config-only** (`wheelspeed.gad_type`,
see below) rather than a runtime command — matches this project's
existing convention (`network`, `navigate`, etc. — settings that
persist across restarts live in `config.yaml`, edited via the manager's
Config page, not a live in-page control). Default: `GadVelocity`,
matching GR6-v1's own working default.

Both message types' fixed parameters (variance, lever-arm variance) are
carried forward from GR6-v1 **unchanged**, including a known oddity:
`GadVelocity`'s lateral/vertical variance is set to the same small value
as the forward-axis variance (0.01, isotropic in the IMU frame before
rotation) rather than a physically-motivated loose value (~1.0) for the
axes that aren't actually measured — GR6-v1's own comment says using the
"correct" loose value stops the update being useful, and 0.01 isotropic
is what was empirically found to work. Not re-derived or "fixed" here —
carried forward as-is, since the ask was "whatever the GR6-v1 code
does, that's what we want," not a from-scratch re-design.

### Config

```yaml
wheelspeed:
  unit: robot-wheelspeed.service
  host: 0.0.0.0
  port: 8008
  web_ui: true
  gad_type: GadVelocity   # or GadSpeed - see "GadVelocity vs GadSpeed" above
  left_wheel:
    lever_arm_i: [0.0, 0.0, 0.0]   # IMU -> left wheel, IMU frame, metres - MEASURE AND SET
    scale: 1.0                      # fudge factor on top of drive's own m/s velocity; normally 1.0
  right_wheel:
    lever_arm_i: [0.0, 0.0, 0.0]   # IMU -> right wheel, IMU frame, metres - MEASURE AND SET
    scale: 1.0
```

`xnav_ip` (top-level) and `oxts-nav.hpr_ib` (IMU-to-body mounting) are
already available from existing config entries — reused, not
duplicated, same pattern `aruco` already established for both. Lever
arms default to `[0,0,0]` (unmeasured placeholder) — **these need
measuring on the real robot before this is trustworthy aiding data**,
same caveat `aruco`'s `camera_extrinsics` has for its own unmeasured
bore-sight.

Scale is per-wheel (not one shared value) — Ben's call, even though in
practice both wheels are likely close to the same value, since
`drive`'s velocity is already correctly scaled to m/s (via
`counts_per_metre`), this is a pure calibration fudge-factor on top of
an already-correct unit conversion, and there's no reason the two
wheels' small residual errors would be identical.

### Update rate / timing — resolved with Ben, 2026-07-27

GR6-v1 had **no separate update rate**: `GadWheelspeed.update()` was
called directly, once, every time a new `EN` (encoder position) line
arrived from the Arduino over serial, at whatever rate the firmware
reports telemetry (~20Hz) — and that rate is known to have worked fine.
Ben's call: keep that rate, don't try to slow it down further.

The real issue turned out to be **timing accuracy, not rate**: the
first cut of this service polled `drive_client.latest()` (whatever
FeedClient happens to have cached) and stamped the outgoing GAD packet
with "now" — with no idea how stale that reading actually was, and no
correction for the fact that a filtered wheel velocity is an *average
over the preceding interval*, not an instantaneous reading at the
moment it's read. Reporting an interval-average at the interval's *end*
systematically biases the aiding time by half the update period — small
at 20Hz, but real, and Ben specifically wants this accurate (GAD timing
errors turn into apparent extra velocity/position error the Kalman
filter has to explain away).

Fixed by making `drive` itself timestamp reality, not by polling faster:

- `drive/serial_link.py`'s read loop now stamps `FV_timestamp =
  time.monotonic()` at the exact moment each `FV` (filtered velocity)
  telemetry line is actually read from the Arduino — a real event
  timestamp, not "whenever some consumer happened to check."
- `wheelspeed`'s update loop polls frequently (`poll_period = 0.02s`,
  well under the ~20Hz real interval) purely to *notice* a new
  `FV_timestamp` promptly — the poll rate itself has no bearing on
  accuracy any more, since the GAD packet's time comes from `drive`'s
  own recorded timestamps, not from when `wheelspeed` happened to look.
- The GAD packet's machine-time is the **midpoint** of
  `[previous_FV_timestamp, current_FV_timestamp]` (`timing.py`'s
  `midpoint_time`) — correctly representing "when, on average, this
  velocity reading was true" — converted to GPS/xNAV time via
  `oxts-nav/ncomrx.py`'s `machine_time_to_gps` (the same helper `aruco`
  already uses; Ben specifically flagged this is the right function —
  GPS time, not UTC).
- The very first reading after startup is skipped (no previous
  timestamp yet to form an interval from) — one measurement's worth of
  startup latency, not worth complicating the logic to avoid.

## Page

One page (`home`), no top banner dropdown needed beyond the usual
shared header:

1. **One scrolling chart** (not two, per Ben's explicit steer) — three
   series: left wheel speed, right wheel speed (both `drive`'s own
   `*_vel_filt_mps`), and the INS's own forward body-frame velocity
   (see below) — all in m/s, for a quick "do the wheels roughly agree
   with what the INS thinks it's doing" visual check.
2. **Numeric boxes** for the same three values (plus a GAD-packets-sent
   counter), same plain-table convention as `oxts-nav`'s home page
   status boxes.

Forward body-frame velocity is computed here, not read from NCOM/UCOM
directly — Ben's instruction: NCOM doesn't have an equivalent field, so
whatever's used has to work identically regardless of which protocol
`oxts-nav` is configured for. Simple trigonometry from the nav feed's
own `Vn`/`Ve`/`Heading`:

```
forward_mps = Vn * cos(Heading) + Ve * sin(Heading)
```

(`Heading` measured clockwise from North, standard OxTS convention —
same convention already used throughout `aruco`/`navigate`.)

A separate **Config** page shows the current configuration read-only (
`gad_type`, both wheels' lever arms and scales) — same convention as
every other service's Config page: a plain table plus a note that
values are changed via the manager's Config page, not here.

## Scale-factor map (added 2026-09-14)

`scale_factor.py`'s tracker already logs one running scale-factor value,
but Ben found it isn't one constant: it's noticeably different on grass
(~1.2) vs the gravel path it was tuned on (~1.0), and may vary further
with slope/ground condition even within one groundcover. `scale_factor_map.py`
turns this into a sparse 2D grid (`ScaleFactorMap`, same
sparse-dict-of-(i,j)-cell shape as `map-manager/grid.py`'s `MapGrid`,
same fixed reference frame - `map-manager`'s own `map_origin_lat/lon` -
so the two grids stay directly comparable), one cell per
`scale_factor_distance_m` (1m) of travel, holding a running mean/stdev
via Welford's algorithm with `n` capped at `scale_factor_map_max_n`
(config) - below the cap it's an honest shrinking-uncertainty sample
mean, above it the update behaves like a bounded-memory average so a
real change (tyre wear, wet vs dry ground) keeps being tracked, without
a separate "learning vs trusted" state machine or a residual-based
retrigger (which risks not being independent of the very estimate it's
checking).

Deliberately not wired into `gad_wheelspeed.py` - same "observe first"
reasoning as `scale_factor.py`'s own history. The map is built
unconditionally, independent of the GAD switch.

**GNSS-quality gate** (`scale_factor_gate.py`): once the map is ever
fed back into GAD, the wheel-vs-INS comparison stops being independent
- the INS solution would itself be partly shaped by the wheelspeed
being checked. Gates on GNSS *velocity* quality, not position accuracy
- some spots worth mapping (e.g. under a tree) have poor GNSS position
from multipath while GNSS velocity (Doppler-derived) stays comparatively
good. Uses `InnVelXFilt`/`InnVelYFilt` (not the raw instantaneous
`InnVelX`/`InnVelY`) and `GnssVelReject`, deliberately: a bad GNSS
velocity update disturbs the filter's state/covariance for a while
after the event, not just for that one epoch (Ben), and the Filt
fields' peak-hold-with-slow-decay behaviour (`ncomrx.py`'s
`_updateInnovation`) is what captures that lingering effect. The
threshold (`scale_factor_map_max_vel_innovation`) can't be assumed to
be "1 sigma" - checked live against the 2026-09-14 mission,
`InnVelXFilt`/`InnVelYFilt` sit around 1.5-2 (median) even during
wholly ordinary driving with no GNSS problem at all (p95 ~3.2); a naive
1.0 threshold rejected 96% of an otherwise-fine mission. Set to 3.5
(roughly that mission's own p95).

**Backfill**: seeded from the 2026-09-14 10:51:37 "Water garden (big)"
mission (the first to complete the whole garden) via a one-off script
(not committed - see session notes), reusing wheelspeed's own already-
logged `scale_factor` samples cross-referenced against the raw NCOM
archive (decoded fresh, since `InnVelXFilt`/`InnVelYFilt` weren't in
`decoded_log_fields` yet at the time) for the gate. Result: 128 cells,
mostly 5-20 samples each, spatially coherent - cells near the waterbutt
approach/return path (gravel) came out 0.93-1.05, cells further out in
the beds (grass) 1.3-1.8.

**Visualisation**: `GET /api/scale-factor-map` returns `[{lat, lon, n,
mean, stdev, t}, ...]` (lat/lon, not local north/east - it's consumed
directly by `geomap.js` layers, which only know lat/lon). Shown in
`viewer`'s map as a toggleable background layer (checkbox + opacity
slider, since `viewer`'s per-run file list doesn't fit a perpetual,
always-growing map) coloured by each cell's mean scale factor relative
to 1.0 (blue = slower than expected, red = grass/slip, both scaled
against the loaded data's own spread). Needed two small `geomap.js`
additions: a `style.background` flag (excluded from "auto" zoom's
extent calculation, and always drawn first, so a persistent whole-
garden layer never dominates the view or hides a file's own trail) and
`style.colorFn(point)` for per-point colour (previously one fixed
colour per layer).

## Implementation Decisions

- `GadWheelspeed` (in `gad_wheelspeed.py`) owns only the GAD-sending
  logic — construction, per-wheel packet building/sending, packet
  count. It does not read any feed itself; `app.py`'s update loop reads
  `drive`'s and `oxts-nav`'s feeds and calls `update(gps_week,
  gps_seconds, left_mps, right_mps)`, same separation of concerns as
  `aruco`'s `GadSender.send(...)` (aruco's detection loop, not `gad.py`
  itself, reads the nav feed).
- The IMU-frame rotation (`hpr_to_dcm`) is reused directly from
  `aruco/coords.py` via the same cross-service import pattern `aruco`
  already uses for `oxts-nav/ncomrx.py`'s `machine_time_to_gps` — one
  well-defined pure function, not a business-logic dependency between
  services, and not worth promoting to `shared/` for a second user
  alone (see `ui-style.md`'s "build where first needed, promote once a
  second consumer needs it" — a third consumer might justify moving it,
  two doesn't yet).
- No `wheelspeed`-owned feed socket (`FeedServer`) — nothing else
  consumes wheelspeed's own output yet (same situation `aruco` is in;
  it has no outbound feed either, just its own `/ws/aruco`). Add one if
  a real consumer shows up.
- Sending is entirely one-way and best-effort: a failed GAD send (e.g.
  `drive` or `oxts-nav` not running yet, no `timeOffset` available) is
  skipped for that tick, not retried or queued — matches `aruco`'s GAD
  sending, and there's no reason to catch up on a stale wheel-velocity
  reading later.

## Testing Decisions

- `GadWheelspeed`'s packet-building logic is unit-tested against a
  fake/stub GAD handler (same pattern `aruco`'s `gad.py` could use, had
  it needed one) — no real UDP packet ever sent by a test.
- The forward-velocity trigonometry (`Vn`/`Ve`/`Heading` → forward
  m/s) is unit-tested directly against known angle/vector cases.
- Real hardware needed to validate: actual wheel lever arms (currently
  unmeasured placeholders), whether `GadSpeed` does anything detectable
  in practice (GR6-v1 never confirmed it did), and the update-rate
  question above.

## Out of Scope (v1 of this service)

- GAD-aruco-style stream-ID/output rethinking — Ben: "we didn't use
  GAD-aruco, maybe we should, but we can drop that for now." Stream IDs
  133/134 kept exactly as GR6-v1 had them.
- Editing `gad_type`/lever-arms/scale from within this service's own
  page — config-only, same convention as every other service.
- Any control-loop or safety interaction with `drive` — this service
  only reads `drive`'s feed, never sends it commands.
- Resolving the update-rate question above — implemented with a
  reasonable default, flagged for a real conversation.

## Further Notes

Ported from GR6-v1's `gad_wheelspeed.py`/`motors.py` (see
`/home/pi/share/python/GR6-v1`) the same day Ben asked for it. Built,
unit tested, and running live (2026-07-27) with real measured lever
arms and the timing fix described above — confirmed correct while
stationary (both wheels reporting 0 m/s, GAD packets sending at the
real measured rate, ~8Hz per wheel, not the previously-assumed 20Hz).

**Not yet field-tested with the wheels actually turning** — everything
above is a static/stationary confirmation only. Watch the INS solution
during real driving before trusting this as genuine aiding rather than
just "it runs without crashing."
