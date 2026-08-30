# map-manager

A small service that builds and persists a single, garden-wide
occupancy grid from the robot's ultrasonic sensors and live GNSS/INS
pose. Built 2026-08-18, prompted by a recurring real pain point: there
was no way to plan a route back from a watering point to the water
butt other than retracing the outbound path. This is the first slice
of that — the grid itself, gated on manual on/off and on GNSS/INS
accuracy. A path planner that actually reads the grid, a human
override-editing UI, and geolocated image capture are all deliberately
out of scope for this slice — see "Not yet built".

## Reference frame

Unlike navigate's paths (each of which uses its own first point as a
throwaway local-frame reference — see navigate-prd.md's "Path
storage"), this grid is meant to persist and accumulate across
sessions/days/months, potentially with the operator hand-editing
"never go here" cells into it once an editor exists. That only makes
sense against one fixed origin for the robot's whole life, not a
per-session one. `map_origin_lat`/`map_origin_lon` in config.yaml are
that origin — set once (2026-08-18, to aruco marker id 12's surveyed
location, the waterbutt's own QC marker — already accurately surveyed,
and a physically meaningful point), never derived from a path or a run.
`shared/geodesy.py`'s `lla_to_ned` converts oxts-nav's live lat/lon
into (north, east) metres relative to it, same primitive
navigate/geometry.py's own `to_local` wraps.

This is a one-time copy of that marker's lat/lon into config.yaml, not
a live reference to the marker map — if marker 12 is ever physically
moved (there's talk of shifting it east/down for camera visibility) or
resurveyed, `map_origin_lat`/`map_origin_lon` must NOT be updated to
follow it. Doing so would silently reinterpret the position of every
cell already recorded in the grid, without moving the grid data itself
— effectively corrupting it. The origin is just a fixed number now,
deliberately decoupled from the marker's own future life. (Note:
config.yaml.example keeps this as a `0.0` placeholder rather than the
real value — see its own comment for why.)

Robot pitch/roll are ignored when projecting a sensor's position/
beam-heading into this frame — flat-enough-garden assumption. The one
case this could bite: the robot pitched enough that a horizontally-
mounted sensor picks up ground reflections instead of a real obstacle.
Not handled - revisit if/when it's actually seen, per "start simple"
throughout this feature's design discussion.

## Occupancy grid: clamped log-odds (revised 2026-08-18)

`grid.py`'s `MapGrid`: a sparse dict keyed by `(i, j)` cell index
(`cell_size_m` metres per cell, default 10cm), each holding
`{log_odds, t}` — a running clamped log-odds belief that the cell is
occupied, and the wall-clock time of the last touch. A second, separate
sparse dict holds human overrides (`"blocked"` / `"clear"`), which
always wins over sensor data for a cell and never decays — this is the
"human says don't go here, don't update the map" mechanism, though
nothing writes to it yet (see "Not yet built").

**First cut was a plain hit/miss ratio - replaced after real field
data exposed a real bug in it.** The very first version of this grid
stored `{hit, miss, t}` counts and computed `p_occupied = hit /
(hit+miss)`. A few hours of real driving showed the actual problem
with that: a robot parked near the origin for a long stretch
accumulated 15,000+ observations on a handful of cells, while normally-
driven cells had a median of 3. The *volume* difference alone wasn't
the issue - it's that a plain ratio becomes nearly immovable once a
cell has thousands of observations: if something real later appears in
a cell that's accumulated 15,000 "clear" readings, it would take
thousands of contradicting "occupied" readings to cross 50% again. Not
a display problem, an estimator problem - see below for the fix, and
the "why not a raw event list" reasoning below for the other half of
what this incident changed.

**The fix: clamped log-odds, the standard occupancy-grid-mapping
approach** (Moravec/Elfes; see Thrun/Burgard/Fox's *Probabilistic
Robotics* for the reference treatment). `grid.py`'s `LogOddsParams`
holds four config values, all human-meaningful probabilities, not raw
log-odds: `p_hit` = P(occupied | sensor reported a hit), `p_miss` =
P(occupied | sensor reported a miss) - how much a single reading is
trusted - and `p_min`/`p_max`, which clamp the resulting probability
after every single update. Each observation converts its percentage to
a log-odds nudge via `logit(p) = log(p/(1-p))`, adds it to the cell's
running value, then clamps to `[logit(p_min), logit(p_max)]`.
Converting back to a probability for display/querying is `sigmoid(l) =
1/(1+e^-l)`. The clamp is what actually fixes the immovability problem:
confidence saturates at "very sure" but is bounded so a handful of
real, later, contradicting readings can still move it - see
`test_grid.py`'s `test_a_clamped_belief_responds_quickly_to_new_
contradicting_evidence` for this behaviour under test. A side effect
worth noting: the old `min_observations` gate (a cell needed a few
combined readings before being trusted at all) is gone - log-odds
starts every cell at exactly 0 (p=0.5, "unknown") and a single
observation only ever nudges it by the bounded amount `p_hit`/`p_miss`
implies, so a single lucky reading can no longer look like certainty
the way a 1-observation ratio could.

**Each detection updates a wedge, not a single cell.** An ultrasonic
reading is a range along a ~15 degree cone, not a point. On each
reading, every cell within `beam_half_angle_deg` of the sensor's
pointing direction gets touched: cells nearer than the measured range
are marked "miss" (the sound passed through, or nothing was in range
at all if the sensor reported no detection within `max_range_m`);
cells right at the measured range are marked "hit". Cells further away
than the measured range are left untouched — the ping never reached
them.

**"Forgetting".** A cell only reads back as trustworthy while its last
touch is more recent than `forget_after_days` (config, default 30) —
past that it's `"unknown"`, same as never-observed. Unaffected by the
log-odds change above; still a simple age cutoff, not a continuous
decay curve, for the same reason as before - no path planner yet to
say whether the extra sophistication would matter.

**Overrides never expire**, deliberately - a human-set restriction
shouldn't quietly erode back to unknown just because nobody's driven
past it in a month.

**Persistence.** `grid.dump_state()`/`load_state()` (de)serialise to
JSON (`map-manager/data/grid.json`, gitignored — this robot's actual
garden data, not source). The tick loop autosaves whenever the grid is
dirty and `save_interval_s` has elapsed, via a write-to-temp-then-
atomic-rename so a mid-save crash never leaves a half-written file.
Loaded once at startup. `dump_state()` also records the params/
cell_size_m/max_range_m/beam_half_angle_deg that produced it, purely
for a human reading the file directly - `load_state()` ignores them,
never lets a saved file silently override the running config.

**What a cell's value is actually good for.** `p_occupied` is
deliberately not yet turned into a path-planning cost function (no
"prefer not to be here" blur, no inflation) — that's the path
planner's job, not this service's, once it exists. This service's
scope ends at "here is what the grid currently believes."

**Longer-term "permanence" layer - designed, not built.** Ben's own
follow-on idea, worth recording even though nothing here implements it
yet: distinguish permanent structure (walls, bed edges) from temporary
clutter (a hosepipe, a ball) by running a *second* clamped log-odds
filter at a much slower tick rate - once per day rather than once per
reading. Each day's rollup asks one question per cell ("did today's
live grid say occupied here?") and feeds that single yes/no into the
same log-odds math as the live grid, just accumulating day-over-day.
A wall says "yes" every day and saturates permanent quickly; a moved
hosepipe says "yes" once then "no" for several days and decays back
down. Same building block, two timescales - this maps onto the
established multi-timescale approach to long-term mobile robot mapping
(Biber & Duckett) rather than being a new idea from scratch. Deferred
until the live grid (this section) has been seen behaving sensibly for
a few real days - there's nothing to design for the daily layer until
its input (the live grid's own "occupied today" verdict) actually
exists and looks trustworthy.

## Sensor and pose integration

Consumes two existing feeds via `shared/feed_client.py`'s `FeedClient`
(no new plumbing): oxts-nav's `nav_feed_socket` for
lat/lon/heading/accuracy (`NorthAcc`/`EastAcc`/`AltAcc` from NCOM
status channel 3, same fields/formula navigate/app.py's
`_current_position` already uses for `horizontal_accuracy_m`), and
drive's `drive_feed_socket` for the five `ultrasonic_<tag>_mm`
readings. Per-sensor mounting offset (body frame: x forward, y right,
z down — metres, same lever-arm convention as wheelspeed's
`lever_arm_i`/aruco's `camera_extrinsics`/waterbutt's funnel offset) and
mounting heading come from `config.yaml`'s `sensors` list, keyed by
`tag` (0-indexed, matching drive's own `ultrasonic_<tag>_mm` feed field
and firmware's `ultrasonicPins` array — not 1-indexed, caught and fixed
2026-08-18 before any real measurements were taken against it). Heading
is filled in already, straight from drive/templates/pages/
ultrasonics.html's documented position/facing diagram (U0/U1 front
corners facing forward, U2 back-right facing right, U3 back-centre
facing backward, U4 back-left facing left) — the x/y/z position offsets
are still unmeasured placeholders (`MEASURE AND SET`), pending CAD
measurements. The wedge geometry means being off by a few degrees is
fine (matches the sensors' own ~15 degree cone / ~5 degree practical
boresight tolerance, per field discussion), but the position offsets
still need real values before the grid means anything.

**Undocumented firmware "no echo" behaviour.** `drive/protocol.py`
doesn't document what an ultrasonic sensor reports when nothing's in
range. This service works around that by clamping: any reading
`>= max_range_m` (1.2m default) is treated as "no detection", same as
whatever sentinel the firmware actually uses. Flagged here as an
assumption to check once real data is being captured, not verified
against the firmware source.

## Accuracy/logging gating

A tick (`control_hz`, default 5Hz) computes one accuracy verdict
(`_accuracy_reason`) shared by two independent manual switches - the
live grid and raw event capture (see below) can be on or off
separately, but both still require the same fix quality:

- **oxts-nav has a fix at all** (Lat/Lon/Heading present in the nav
  feed — absent whenever oxts-nav's own staleness logic has blanked
  it, see oxts-nav-prd.md's "Staleness").
- **Horizontal accuracy** (`hypot(NorthAcc, EastAcc)`) is at or better
  than `accuracy_h_max_m` (default 3cm).
- **Vertical accuracy** (`AltAcc`) is at or better than
  `accuracy_v_max_m` (default 5cm).

Each switch is in-memory only, always starts `False` on service start/
restart — deliberately not persisted, so a restart never silently
resumes either one while the robot is being carried around or worked
on. Live grid: `POST /logging {"enabled": bool}`. Raw capture: `POST
/raw-capture {"enabled": bool}`. The home page's status feed reports
each switch's own current reason (if blocked), so it's obvious from the
page alone why either one isn't currently doing anything, without
needing to check logs.

## Raw event capture (added 2026-08-18)

A second, independent capture path alongside the live grid, specifically
so parameters (the log-odds percentages, cell size, max range, beam
angle, even sensor calibration) can be tried against real data without
re-driving - see "Why not a raw event list" below for why this wasn't
built the first time round, and what changed.

**Why not a raw event list (original reasoning, since revisited).**
Early design discussion considered logging every detection as a
standalone record and periodically regenerating a grid from it - the
appeal being reprocessability if the aggregation logic or calibration
improves later. Dropped at the time in favour of updating the live grid
directly: a raw list only earns its cost if reprocessing history is
actually needed, and nothing about the return-route-planning goal
needed it yet. That predicted cost showed up almost immediately: fixing
the hit/miss-ratio bug above meant the day's already-captured data
literally couldn't be reused - only the aggregated totals were kept,
and a clamped log-odds replay is order-dependent (it clamps after every
single reading), so there was no way to reconstruct what really
happened from totals alone. Rebuilt properly this time, as its own
opt-in capture rather than the default.

**What gets stored.** One JSONL line per sensor per tick while capture
is on and accuracy passes: `{t, north, east, heading_deg, tag,
range_mm}` - the robot's own pose and heading, which sensor, and the
*raw, unclamped* millimetre reading, not anything already derived from
a particular set of parameters. Deliberately minimal: storing the raw
`range_mm` (not a pre-computed `range_m`/None) means a later replay can
choose a different `max_range_m` than was live at capture time and
reinterpret the same reading (see `grid.py`'s `effective_range_m`,
shared by both the live tick and the post-processor so they interpret
a reading identically); storing robot pose rather than the pre-computed
sensor world-pose means a replay can also try different sensor
calibration offsets, not just different occupancy math.

**File handling.** One file per capture session (started fresh each
time the switch flips off→on), named the same yymmdd_hhmmss(+collision
suffix) way as every other per-run log in this project
(`map-manager/data/raw/`). Swept on startup past
`raw_log_retention_days` (default 4) - deliberately much shorter than
the grid's own `forget_after_days`: Ben's own call, expecting to tune
against a capture for a few days at most and never need it again,
unlike the derived grid itself.

## Post-processor (added 2026-08-18)

A dedicated page (`/pages/reprocess`) - select one or more raw capture
sessions (same file-picker convention as navigate's own Log Viewer),
adjust any of the four log-odds percentages plus cell size/max range/
beam angle (pre-filled from current config, not fixed to it), and
rebuild a grid from scratch via `reprocess.py`'s
`build_grid_from_raw_logs` - the exact same `record_detection`/
`effective_range_m` the live tick uses, so a replay is a faithful
reproduction, not an approximation. Sensor calibration itself isn't
exposed as an editable field yet (uses current config) - a deliberate
scope cut to keep the page to a reasonable size, not an oversight;
worth revisiting if recalibration-after-the-fact ever actually comes
up. Never touches the live grid - each run writes a freshly timestamped
`grid_yymmdd_hhmmss.json` (`map-manager/data/grids/`) plus a paired
`settings_yymmdd_hhmmss.json` recording the input files, sensor table,
and creation time, so a few attempts can sit side by side and be
compared once a viewer exists to actually look at them (see "Not yet
built" - by Ben's own admission, reprocessing "won't make any sense"
without one, so this was built and left at "produces a file" rather
than chasing a UI for results nothing can render yet).

## Home page

Shows: the live-grid on/off state plus Enable/Disable buttons and its
current eligibility reason; live horizontal/vertical accuracy; running
cell/override counts; and, separately, the raw-capture on/off state
plus its own Enable/Disable buttons and reason - with a link through to
the Reprocess page. Deliberately no map visualisation yet.

## Not yet built

Left out of this slice on purpose, not overlooked:

- **Map visualisation.** How to actually *see* the grid (rendered
  cells, overlaid on the same canvas/trail navigate and jobs already
  use) is still an open design question, not just an unimplemented
  feature - almost certainly a raster/bitmap approach (canvas
  `putImageData`) rather than drawing individual cells as shapes, given
  the cell counts involved.
- **Human override editor.** The data model (`set_override`/
  `clear_override`, "blocked"/"clear" sentinels that never decay) is
  built and unit-tested, but there's no UI to actually set one yet —
  explicitly deferred per field discussion ("I think we ignore this
  for now").
- **Daily permanence layer.** Designed (see "Occupancy grid" above) but
  not built - waiting on the live grid itself being seen to behave
  sensibly first.
- **Path planner.** Nothing here reads the grid to plan a route yet —
  this service only builds and exposes it.
- **Geolocated image capture.** A related, later idea (X/Y/heading-
  bucket grid of capture sites, quality-based replacement) that this
  service is expected to eventually host alongside the occupancy grid,
  but using its own cell scheme, not a probability grid. Not started.
