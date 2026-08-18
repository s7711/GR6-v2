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

## Occupancy grid

`grid.py`'s `MapGrid`: a sparse dict keyed by `(i, j)` cell index
(`cell_size_m` metres per cell, default 10cm), each holding
`{hit, miss, t}` — a running count of sensor readings that did/didn't
find an obstruction there, and the wall-clock time of the last touch.
A second, separate sparse dict holds human overrides (`"blocked"` /
`"clear"`), which always wins over sensor data for a cell and never
decays — this is the "human says don't go here, don't update the map"
mechanism, though nothing writes to it yet (see "Not yet built").

**Why not a raw event list.** Early design discussion considered
logging every detection as a standalone record ("obstruction at
lat/lon at time yymmdd hhmmss") and periodically regenerating a grid
from that list overnight — the appeal being that it stays
reprocessable if the aggregation logic or sensor calibration improves
later. Dropped in favour of updating the live grid directly: a raw
list only earns its cost if reprocessing history is actually needed,
and nothing about the return-route-planning goal needs it. If
recalibration-after-the-fact turns out to matter later, add a raw log
then, targeted at that problem, rather than carrying the extra
complexity now on spec.

**Each detection updates a wedge, not a single cell.** An ultrasonic
reading is a range along a ~15 degree cone, not a point. On each
reading, every cell within `beam_half_angle_deg` of the sensor's
pointing direction gets touched: cells nearer than the measured range
are marked "miss" (the sound passed through, or nothing was in range
at all if the sensor reported no detection within `max_range_m`);
cells right at the measured range are marked "hit". Cells further away
than the measured range are left untouched — the ping never reached
them.

**"Forgetting".** A cell's `p_occupied = hit / (hit + miss)` is only
trusted once it has at least `min_observations` combined readings, and
only while its last touch is more recent than `forget_after_days`
(config, default 30) — past that it reads back as `"unknown"`, the
same as a cell that's never been observed. This is a first cut, not a
continuous decay curve: simple age cutoff on the whole cell, chosen
over a weighted-decay formula because there's no path planner yet to
tell us whether the extra sophistication would even matter. Expect to
revisit once one exists.

**Overrides never expire**, deliberately - a human-set restriction
shouldn't quietly erode back to unknown just because nobody's driven
past it in a month.

**Persistence.** `grid.dump_state()`/`load_state()` (de)serialise to
JSON (`map-manager/data/grid.json`, gitignored — this robot's actual
garden data, not source). The tick loop autosaves whenever the grid is
dirty and `save_interval_s` has elapsed, via a write-to-temp-then-
atomic-rename so a mid-save crash never leaves a half-written file.
Loaded once at startup.

**What a cell's value is actually good for.** `p_occupied` is
deliberately not yet turned into a path-planning cost function (no
"prefer not to be here" blur, no inflation) — that's the path
planner's job, not this service's, once it exists. This service's
scope ends at "here is what the grid currently believes."

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
ultrasonics.html's documented position/facing diagram (U0/U4 front
corners facing forward, U1 back-right facing right, U2 back-centre
facing backward, U3 back-left facing left) — the x/y/z position offsets
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

A tick (`control_hz`, default 5Hz) only updates the grid when *all* of:

- **Manual switch is on.** In-memory only, always starts `False` on
  service start/restart — deliberately not persisted, so a restart
  never silently resumes logging while the robot is being carried
  around or worked on. Toggled via `POST /logging {"enabled": bool}`
  from the home page's Enable/Disable buttons.
- **oxts-nav has a fix at all** (Lat/Lon/Heading present in the nav
  feed — absent whenever oxts-nav's own staleness logic has blanked
  it, see oxts-nav-prd.md's "Staleness").
- **Horizontal accuracy** (`hypot(NorthAcc, EastAcc)`) is at or better
  than `accuracy_h_max_m` (default 3cm).
- **Vertical accuracy** (`AltAcc`) is at or better than
  `accuracy_v_max_m` (default 5cm).

The home page's status feed reports which of these (if any) is
currently failing, so it's obvious from the page alone why the grid
isn't growing right now, without needing to check logs.

## Home page

Shows: the logging on/off state plus Enable/Disable buttons; the
current eligibility reason (or "Updating the map." when nothing's
blocking); live horizontal/vertical accuracy; and running totals
(cell count, override count, total hit/miss observations). Deliberately
no map visualisation yet.

## Not yet built

Left out of this slice on purpose, not overlooked:

- **Map visualisation.** How to actually *see* the grid (rendered
  cells, overlaid on the same canvas/trail navigate and jobs already
  use) is still an open design question, not just an unimplemented
  feature.
- **Human override editor.** The data model (`set_override`/
  `clear_override`, "blocked"/"clear" sentinels that never decay) is
  built and unit-tested, but there's no UI to actually set one yet —
  explicitly deferred per field discussion ("I think we ignore this
  for now").
- **Path planner.** Nothing here reads the grid to plan a route yet —
  this service only builds and exposes it.
- **Geolocated image capture.** A related, later idea (X/Y/heading-
  bucket grid of capture sites, quality-based replacement) that this
  service is expected to eventually host alongside the occupancy grid,
  but using its own cell scheme, not hit/miss counts. Not started.
