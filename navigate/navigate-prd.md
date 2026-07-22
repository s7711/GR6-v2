# PRD: Path-Following Service (navigate)

See `top-prd.md` for where this fits in the overall migration order: `drive`
(done) → **navigate** (this document) → `missions` (sequencing multiple
stops + pump actions, future) → `safety` (obstacle-avoidance, future). This
is the "dumb path-following" layer — it drives a pre-recorded path exactly
as authored and gives up cleanly when it can't safely continue. Deciding
*what to do* about a failure (replan, try an alternative route, call for
help) is explicitly `missions`' future job, not this one's — see
"Ownership of tolerance/accuracy enforcement" below.

## Problem Statement

The robot needs to be able to record a path by driving it once, then
re-drive that same path autonomously later, at an operator-chosen speed
per segment, with the water pump on/off at the right points. It also
needs to refuse to drive itself into trouble: if the GPS position
estimate degrades (RTK dropout under trees/near buildings) or the robot
drifts too far off the path, it should stop rather than guess.

Unlike GR6-v1, the tolerance for "how far off the path is too far" needs
to vary by location — GR6-v1 had one fixed number for the whole garden,
which meant places with plenty of open space around them were just as
strict as tight spots, and became a source of frustration once the
robot needed to run somewhere the fixed tolerance couldn't tolerate.

## Prior art (GR6-v1)

`path_follow.py` (see `drive-prd.md`'s Prior art section for the
sibling `motors.py`/`gad_wheelspeed.py` history) already solved most of
this well — the goal here is to keep what worked, fix what didn't, and
add per-segment tolerance where v1 only had one global number.

**What transfers directly:**
- Paths were plain files, one point per line: X/Y (local metres), speed,
  pump on/off. No heading or per-point tolerance stored.
- Path creation was manual and button-driven: drive with the joystick,
  press "Add Waypoint," it captures the robot's *current* live position
  + a typed speed. No automatic distance-based sampling. This matches
  what's wanted here too (see Implementation Decisions).
- Control was a pure-pursuit controller: project the robot's position
  onto the nearest path segment ahead of where it currently is, walk
  forward by a fixed lookahead distance (0.4m in v1), blend cross-track
  error and heading error into a turn command, convert to differential
  wheel speeds.
- Path entry used a forward scan from the start of the path: the first
  segment within a distance threshold (1.0m) and heading threshold
  (45°) of the robot's current pose became the starting point for
  tracking. Reused here (see "Path entry" below).

**What's being deliberately changed:**
- **Tolerance was one global constant** (`MAX_LINE_DEPARTURE = 0.4m` for
  cross-track, `MAX_LOCALISATION_ERROR = 1.0m` for GPS accuracy RMS) —
  this PRD replaces it with a per-segment "clearance" value (see below).
- **v1's entry scan didn't check its own result** — if no segment
  qualified, `start()` proceeded anyway, silently tracking a meaningless
  index. Fixed here: a failed entry scan is reported to the operator,
  not silently ignored (see "Path entry").
- **v1's live gain-tuning commands didn't actually work** — they
  reassigned a local variable instead of the real tunable, so
  `>LOOKAHEAD_DISTANCE ...`-style commands had no effect. Fixed here by
  reusing `drive`'s own tuning-page pattern (config-driven at startup,
  live-settable, no silent no-ops).
- **Abort was a hard stop with no graceful slow-down** — kept as-is for
  v1 of *this* service (not inventing new robot behaviour yet), but see
  "Ownership of tolerance/accuracy enforcement" for why this is
  deliberately not the final word.

## Architecture / data flow

`navigate` is its own systemd-managed service, following the same shape
as `drive`/`oxts-nav`:

- Reads path files from disk (see "Path storage" below).
- Reads live position/heading/accuracy from `oxts-nav`'s `/ws/nav`
  directly, cross-service (the same pattern `aruco`'s Map page already
  uses via `service_url(..., "oxts-nav", scheme="ws")` +
  `connectWsUrl` — no new mechanism needed).
- Sends velocity commands to `drive`'s `/command/auto` endpoint at
  ~10Hz while a path is running, in the same `{"left_mps": ...,
  "right_mps": ...}` shape the manual jog page already uses on
  `/command/manual`. `navigate` is the first real "auto" caller
  `drive-prd.md` was written anticipating — it owns its own repeat
  cadence (`drive`'s firmware dead-man's switch is still the only
  hard timeout; `navigate` re-sends the same way the jog page does).
- Reads `drive`'s own `drive_feed` for pump status / to confirm motor
  commands are being obeyed (not required for the control loop itself,
  useful for the run-path page's readouts).
- Publishes its own `navigate_feed` (state, tracked position on path,
  cross-track error, distance travelled, clearance headroom) for its
  own web pages, following the same Unix-socket feed pattern as
  `drive_feed`/`nav_feed`.

`navigate` does not talk to the Arduino/motor controller directly, and
does not re-implement GPS decoding — both already exist and are owned
elsewhere.

## Path storage

- YAML, not GR6-v1's G-code-like text format — matches every other
  config/data file in this project, and is trivial to hand-edit if
  ever needed (GR6-v1's ad-hoc format wasn't).
- One file per path, in a `navigate/data/` directory (parallel to
  `aruco/data/` for survey data) — kept out of git via `.gitignore`,
  since paths are this robot's real-world data, not source.
- Per point: **`lat`/`lon`** (absolute, degrees) — not local XY.
  Matches `aruco/data/marker-map.yaml`'s existing precedent (markers
  are stored absolute, never as a local offset from some origin), for
  the same reason: a stored local XY would silently go stale if
  whatever local reference it was measured against ever changed,
  whereas lat/lon has no such failure mode. `shared/geodesy.py`'s
  `lla_to_ned`/`ned_to_lla` already support this — both take an
  arbitrary reference lat/lon as an argument each call (as
  `aruco/survey.py` already does, passing in the robot's current live
  position), so there's no persisted "origin" to invalidate.
- **In-memory conversion, once per run**: when a path is loaded to run,
  convert every point to local north/east metres in one pass, using
  the path's own first point as the reference (`lla_to_ned` with that
  point's lat/lon as `ref_lat`/`ref_lon`). The robot's own live position
  is converted the same way, every control cycle, against that same
  fixed reference for the duration of the run. This keeps the actual
  pure-pursuit math working in plain local metres almost unchanged from
  v1 (which operated in local metres throughout), rather than needing
  every line of that logic rewritten to work in lat/lon directly. Flat-
  earth error from this approximation is on the order of 1cm over a few
  km (per `shared/geodesy.py`'s own docstring) — negligible at this
  project's garden scale, so there's nothing to guard against here.
- Also per point: `speed_mps`, `pump`, `clearance_m` (see below). No
  heading stored — implicit from consecutive points, as in v1.

## Path creation ("create path" page)

Matches the "drive it once, drop points by hand" approach explicitly
wanted over automatic distance-sampling (driving quality varies, and a
20cm auto-sample would capture every wobble).

- Motor jog control (reused from `drive`'s own jog widget/pattern —
  not reimplemented) to drive the robot while recording.
- "Drop point" button: captures the robot's *current* live position
  from `oxts-nav`, plus operator-chosen speed and pump state for the
  segment about to start, and appends a point to the in-progress path.
- Speed selector: one of a fixed set (0.2 / 0.3 / 0.4 / 0.5 / 0.6 / 0.7
  / 0.8 m/s) rather than free entry — matches the discrete set of
  speeds actually worth tuning/trusting on this robot.
- Clearance selector per point: "how much space is actually around
  this segment" — see "Variable tolerance" below for what this drives.
- Pump on/off toggle, capturing pump state into the point being
  dropped (fixes v1's quirk where pump state was read from the
  *tracked* point rather than the point actually being authored).
- Position-accuracy readout (horizontal accuracy only — `GpsPosMode` is
  not needed here, only the accuracy number matters for deciding
  whether a dropped point's position is trustworthy) so the operator
  can see when recording is happening at a good moment.
- No camera view — GR6-v1 had one so the operator could drive from
  indoors; GR6-v2's target usage is standing with the robot, so this is
  dropped as unnecessary scope.
- Save under a name; naming is manual entry, not auto-generated.
- **Live map** (added after real-world use surfaced the need): shows
  the points dropped so far plus live position, using the same shared
  canvas-map module the Run page uses (`shared/web/static/geomap.js`,
  promoted once this became its third use). Motivation: driving a
  smooth line by hand — especially on gravel — is hard, and a jagged
  recorded path causes sudden heading changes between segments that
  trip path-following's heading-error abort later. Deliberately no
  automatic smoothing (explicitly rejected) — the map exists so the
  operator can see and compensate for wobble themselves, not to paper
  over it afterwards.
- **Jog/accuracy overlaid directly on the map**, not in a separate
  card — matches GR6-v1's own pattern of overlaying the joystick on the
  live camera feed; here the map is the equivalent "live view" since
  there's no camera.
- **"Move forward" button**: drives a precise short distance (0.2 /
  0.5 / 1.0 / 2.0 m) in a straight line from the current position/
  heading, using `navigate`'s own pure-pursuit control rather than
  manual jogging — much steadier, especially on gravel. Implemented as
  a synthetic 2-point mini-path run through a throwaway `PathRunner`
  instance (never the shared `runner` used for real saved-path runs,
  to avoid state clashes with an actual run in progress elsewhere).
  The synthetic path's actual endpoint is placed `distance_m +
  lookahead_distance_m` ahead, not just `distance_m` — a bare
  `distance_m`-long path would leave pure-pursuit with no lookahead
  room at all for the shortest option (0.2m, less than the default
  0.4m lookahead distance), degenerating to simple point-chasing.
  The maneuver is then just left to finish naturally
  (`path_complete`) rather than stopped at an exact tracked distance —
  the requested distance doesn't need to be exact, so the robot may
  travel a little further than asked; simpler to build and matches
  "whatever's easiest" from the design discussion. A generous fixed
  clearance (0.5m) is used for this one internal segment, since it's
  never authored into the saved path itself.
- **Duplicate-point protection** (added after real wifi flakiness
  produced two near-identical points): if a "Drop point" response is
  lost over a flaky wifi connection, the operator has no confirmation
  and may reasonably press it again — a genuinely fresh, non-stale
  second request the server has no way to distinguish from "wanted two
  points here." A timestamp/staleness check (as used for the wifi-
  reconnect problem discussed below) doesn't catch this case, since
  the request itself isn't stale. Instead: a client-side check against
  the last dropped point's position (already held in the browser) —
  within 30cm, a confirm/cancel prompt showing the distance, rather
  than silently adding it. No server change needed.

## Deferred: wifi-reconnect command staleness

A jog/drive command that queues up during a wifi outage and then
arrives late, in a burst, once connectivity returns, can make the
robot behave unexpectedly for a moment — worse under Flask's request
handling than GR6-v1's own hand-rolled websocket code, though every
channel here is affected to some degree. A timestamp-and-reject-if-
stale mechanism on `drive`'s browser-facing command endpoints
(`/command/manual`, `/pump` — the ones actually exposed to the flaky
operator↔Pi wifi link, not `navigate`'s own server-to-server calls to
`drive` over localhost) was discussed as the fix. Explicitly parked,
not built: the planned fix is hardware/network first (a dual-band USB
wifi adapter, and driving from a tablet hotspotting directly to the
Pi at close range) rather than a software workaround that adds
complexity/latency for the driver — see `top-prd.md`'s migration-order
item 7. Revisit only if the hardware/hotspot change doesn't actually
fix it.

## Run-path page

- Path selector (load one of the saved paths from `navigate/data/`).
- Start / Stop controls.
- Live readouts: speed, lateral (cross-track) error, distance
  travelled, `GpsPosMode`, horizontal accuracy, differential age (if
  available — see open question below), current clearance headroom
  (how much of the segment's allowed clearance is currently used up).
- A map: the loaded path plus the robot's actual driven trail, same
  canvas-based approach as `aruco`'s existing Map page (no charting
  library needed, hand-rolled is already proven to work well here).
- If the path can't be entered (see "Path entry" below), show the
  distance and heading-angle to the nearest valid entry point, so the
  operator can drive the robot into range by hand before retrying —
  directly fixes v1's silent-failure gap.

## Path management page

- List saved paths, with basic metadata (length, point count).
- Delete.
- Run directly from this list — loads the path server-side and jumps
  to the Run page (Start still needs pressing there; loading isn't the
  same as starting). Automatically swapping back to this list once a
  run finishes was considered but not built — see Out of Scope.
- Text/YAML editing by hand is out of scope for a UI — YAML is fine to
  hand-edit with a normal text editor if ever truly needed, no in-app
  editor is worth building for this.

## Variable tolerance ("clearance")

Each segment carries its own `clearance_m` value (set at recording
time, editable later if paths ever get a hand-edit pass) instead of
GR6-v1's single global `MAX_LINE_DEPARTURE`. While running:

- Cross-track error exceeding the *current segment's* clearance stops
  the robot — same hard-stop behaviour as v1 (zero motors + pump
  immediately via `drive`'s manual-equivalent path — actually via
  simply stopping the auto command stream, since `drive`'s firmware
  dead-man's switch already zeroes everything if `navigate` just stops
  sending), not a new graceful-stop mechanism.
- Localisation accuracy (horizontal accuracy from `oxts-nav`) exceeding
  a threshold also stops the robot — same idea as v1's
  `MAX_LOCALISATION_ERROR`, but expect this to fire rarely in practice
  once wheelspeed GAD aiding is in place (a known near-term addition,
  see `top-prd.md`), since that should keep the INS's own accuracy
  estimate good through most RTK dropouts. Not gold-plating this for a
  problem that may already be mostly solved elsewhere.
- Both thresholds are read from the segment's own `clearance_m` — one
  authored number per place, not two separate constants to reason
  about, since in practice "how much room is here" is the single
  question both checks are really asking.

## Ownership of tolerance/accuracy enforcement

`navigate` owns this fully for now — it is the only layer that exists.
When `missions` is eventually built, it can read the same
`navigate_feed` numbers (cross-track error, clearance headroom,
accuracy) `navigate` is already publishing, and decide to do something
smarter than a hard stop (replan, try an alternative route, wait and
retry). Nothing about `navigate`'s design needs to anticipate that
architecturally beyond "publish the numbers clearly" — no new
plumbing is added now on the assumption `missions` will need it later.

## Path entry

Reused from GR6-v1 almost as-is, with the one real fix:

- On Start, scan forward through the path from its beginning; for each
  segment, project the robot's current position onto it and check (a)
  perpendicular distance within a threshold and (b) heading alignment
  within a threshold. Take the first (lowest-index) segment satisfying
  both.
- Both thresholds start at v1's values (1.0m distance, 45° heading) —
  not re-derived from first principles, since v1's choices worked in
  practice and there's no evidence they need to change.
- **Fixed from v1**: if no segment qualifies, this is surfaced to the
  operator (distance + angle to the nearest candidate segment shown on
  the run-path page), not silently ignored. The operator drives closer
  by hand and retries, rather than the robot starting to track a
  meaningless index.

## Config additions (shared config file) — proposed

```yaml
navigate:
  unit: robot-navigate.service
  port: 8006
  web_ui: true
  paths_dir: navigate/data
  control_hz: 10
  entry_max_distance_m: 1.0
  entry_max_heading_deg: 45
  lookahead_distance_m: 0.4
  heading_gain: 2.0
  cte_gain: 0.6
  localisation_accuracy_limit_m: 1.0
  max_heading_correction_deg: 70    # GR6-v1's third abort threshold — kept as a
                                    # single global constant, not per-segment
                                    # (a wildly wrong heading is a controller-
                                    # sanity problem, not a "how much room is
                                    # here" one, so it doesn't belong on
                                    # clearance_m)
  navigate_feed_socket: /tmp/gr6-navigate-feed.sock
  navigate_feed_hz: 10  # matches control_hz — no reason for navigate's
                        # own status feed to lag behind (or race ahead
                        # of) the loop that's actually generating the
                        # data; unrelated to oxts-nav's own feed rate,
                        # which already runs faster (nav_feed_hz: 20)
                        # than this control loop needs

drive:
  # ...existing drive config...
  wheel_base_m: 0.42  # NEW: distance between the two wheels' contact
                      # points — needed to turn navigate's single
                      # forward+turn command into two wheel speeds.
                      # Lives here, not under navigate:, because it's a
                      # physical drivetrain constant alongside
                      # counts_per_metre, which already lives here —
                      # one home for "facts about this robot's wheels,"
                      # not two copies that could drift apart. drive
                      # itself doesn't need the value for anything;
                      # navigate reads it from drive's config block.
```

`heading_gain`/`cte_gain`/`lookahead_distance_m` are pushed at startup
and live-settable via a tuning page, following `drive`'s own tuning
pattern exactly (config-driven defaults, `Set` button, no server-side
range validation) — this also fixes v1's broken live-tuning commands,
since the mechanism being reused here is already proven to work.

## Abort reason logged to the journal

Every abort (real path-following runs and `/record/forward`'s mini-path
alike — both are `PathRunner` instances) logs a `logging.warning` from
`PathRunner._abort()` itself, one shared code path covering both
callers. Visible via `journalctl -u robot-navigate` — much quicker to
check than opening the debug log file below, which stays useful for the
surrounding detail (exact position/error trend) once the journal has
told you *that* something aborted and roughly why.

## Debug log (last run only)

A real-hardware run aborted on a heading-error breach that looked
suspicious, but with no record of the run afterward there was nothing
to actually check it against. Added a quiet background log rather than
guessing further:

- `navigate/data/last_run_debug.jsonl` (gitignored alongside saved
  paths — this robot's own runtime data, not source) — one JSON line
  per snapshot: timestamp, position/heading/accuracy, and the full
  `PathRunner.status()` (state, tracked index, cross-track/heading
  error, clearance headroom, distance travelled).
- Logged at ~1Hz while a run is `running` — frequent enough to
  reconstruct what happened, far too infrequent to matter for disk/
  performance. Also logged immediately on any state *transition*
  (e.g. the exact step that triggered an abort), regardless of the
  1Hz gate, since that's the one line that actually matters most.
- Reset (truncated) at the start of each successful `Start` — this is
  "the last run," not a growing history. A failed start (no path
  loaded, no position fix, too far to enter) leaves the previous run's
  log untouched, since nothing new actually happened.
- Deliberately not surfaced in the UI (no page, no download button) —
  it's a debugging aid, read by hand (or by a future Claude session)
  when something looks wrong, not a feature end users need.

## Open question: heading-error aborts on jagged paths

A real run aborted on a heading-error breach, suspected by the
operator to be a computation bug rather than a genuinely large error.
Investigated (2026-07-21) by independently re-deriving `bearing`/
`heading_error_deg`'s sign convention from scratch and cross-checking
against GR6-v1's own (differently-derived, math-convention) formula
via its actual decode code — both are mathematically consistent with
each other and with the already-hardware-confirmed `differential_drive`
turn direction. No sign/formula bug found on this pass.

One real, plausible contributing factor was identified but not yet
confirmed: `find_lookahead_point`'s nearest-segment search (inherited
from GR6-v1 unchanged) stops as soon as the projected distance starts
increasing again ("getting worse — matches GR6-v1's early-stop," see
the code comment) — a greedy heuristic that can pick a suboptimal
segment on a path with more than one local distance minimum, i.e.
exactly the kind of wiggly, hand-driven-on-gravel path this session's
map-while-recording feature exists to reduce. This would show up as a
seemingly-oversized heading error relative to the "obviously correct"
segment a human would pick by eye. Not fixed speculatively — the new
debug log (above) should make it possible to confirm or rule this out
directly from a real abort, rather than continuing to reason about it
without data.

## Testing Decisions

- Pure-pursuit math (lookahead point, cross-track error, heading
  error, differential-drive conversion) is unit-testable directly —
  same approach as `drive/test_protocol.py`/`test_control.py`, no
  hardware or even a running `drive` service needed.
- Path entry scan (distance/heading threshold search, first-match
  logic, and the "no segment qualifies" case) is pure logic, testable
  the same way.
- The auto-command sending loop and feed-reading are exercised with a
  fake/stub `drive` and `oxts-nav` endpoint, mirroring `drive/test_app.py`'s
  `StubArbiter` pattern — real hardware is never touched by tests.

## Real bugs found via testing

- **Jog steering was inverted, on both `drive`'s Home page and
  `create-path`'s jog widget** — confirmed on real hardware: pushing the
  stick right turned the robot left. Root cause was the joystick-to-
  wheel-speed formula (`left = forward - turn; right = forward + turn`)
  — turning right requires the right (inside) wheel to slow and the
  left (outside) wheel to speed up, which is the opposite mapping.
  Fixed in both places (`drive/templates/pages/home.html`,
  `navigate/templates/pages/create-path.html`) by swapping which term
  gets added/subtracted. `drive`'s forward-only motion and its
  telemetry (`LM_position`/`RM_position` incrementing correctly for
  their respective physical wheels) were confirmed correct first — this
  ruled out a wiring swap, isolating the bug to the turn-direction
  arithmetic.
- **The same sign convention, copied from GR6-v1, was in
  `differential_drive`'s `turn` parameter** — not yet hardware-tested at
  the time this was caught, but the identical bug pattern, so fixed the
  same way (swapped which term is added/subtracted).
- **`turn_command`'s cross-track-error term had an unverified (and, on
  inspection, wrong) sign** — GR6-v1's own code comment admitted "not…
  even verified the sign of cte." Worked through from first principles:
  a positive cross-track error means the robot is east of a
  north-heading path, which needs a *left* turn to correct, but the
  original formula added the cte term (steering further right, away
  from the path). Fixed by subtracting it instead. This was caught by
  reasoning through the sign conventions after the jog bug was found,
  not by a hardware test of path-following itself (that hasn't
  happened yet) — worth double-checking against a real low-speed test
  run once one's possible, per "If a run aborts" in manual.md.

## Resolved design questions

- **Differential age**: already available with no upstream change
  needed. `ncomrx.py`'s `decodeStatus20` decodes `GpsDiffAge`
  (`ncomrx.py:669-671`) into the `status` dict, and both `nav_feed.py`
  and `oxts-nav/app.py`'s `/ws/nav` pass that entire `status` dict
  through unfiltered — so the run-path page reads it exactly the same
  way it reads `HeadingAcc`/`GpsPosMode` today, as `status.GpsDiffAge`.
- **Wheel-base/scale constants**: `navigate` reads `wheel_base_m` from
  `drive`'s config block (see "Config additions" above) rather than
  keeping its own copy, and consumes `drive`'s already-converted m/s
  values for everything else — it does not re-derive counts↔m/s
  scaling itself.

## Out of Scope (v1 of this service)

- Automatic distance-based waypoint sampling — explicitly rejected in
  favour of manual "drop point," per the Problem Statement.
- Camera view during path recording — dropped, see "Path creation."
- In-app YAML editing of saved paths — a text editor is sufficient;
  not worth building a UI for.
- Graceful (non-hard-stop) recovery from a tolerance/accuracy breach —
  `missions`' future job, not this service's, per "Ownership of
  tolerance/accuracy enforcement."
- Obstacle avoidance / ultrasonics-based decisions — `safety`'s future
  job, per `top-prd.md`'s migration order.
- Auto-navigating back to the Paths list once a run finishes — the Run
  page just sits on its result (state/abort reason visible) instead;
  a small nicety, not built.
