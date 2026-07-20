# ArUco Service — User Manual

This is a how-to-use guide for the `aruco` service's web pages. For the
design/engineering background (why things work the way they do — the
coordinate frame conventions, why GAD position/heading but not roll/
pitch, why there's no fixed map origin, real bugs found along the way),
see `aruco-prd.md` instead — this document is deliberately just "how do I
use it."

Open the aruco service from the manager's home page, or go directly to
its address (port 8004 by default) — e.g. `http://amundsen:8004`.

This service depends on two other services already running and set up:
the **camera** service (for frames) and **oxts-nav** (for the xNAV650's
live position — and the xNAV650 itself needs a good GNSS/RTK fix, ideally
with internet access for NTRIP corrections, for anything here to be
accurate). If a page looks stuck or empty, check those first.

## Home page

The live camera preview, with detected markers outlined and their pose
axes drawn on, plus two readouts:

- **Visible markers** — the id(s) currently detected in frame, or "none".
  Shows "waiting for camera feed…" instead if the detection loop hasn't
  actually connected to the camera service yet — that's a different
  situation from "connected but nothing in view," worth knowing which one
  you're looking at.
- **GAD** — whether a known marker's position/heading is currently being
  sent to the xNAV650 as an aiding update, and when the last one went
  out.

If a **known** marker (already in the marker map) is in view, a
diagnostics table appears below showing the numbers behind that
detection: the vehicle heading/pitch/roll used, the raw camera-frame
detection (`tvec`), the resulting body-frame and NED displacement, and
the range. Useful for hand-checking a measurement against a tape measure
if a position ever looks wrong — see "If a marker's position looks wrong"
below.

## Map page

A plan view, always centred on the robot's current position (it recentres
live as the robot moves — there's no fixed point on this map, by design).

- **Blue dot and line** — current position and its recent trail.
- **Green dots** — markers already in the marker map, labelled by id.
- **Grey dots** — markers detected but **not yet** in the marker map — a
  rough, single-shot estimate of where they are, just enough to help you
  find them again with the Add Marker page. Not a real survey.
- **Gridlines and the scale bar** (bottom-left) always agree with each
  other — if a gridline square doesn't look like the size the scale bar
  says, something's genuinely wrong, worth reporting.
- **Zoom buttons** (top-right, floating over the map): **Auto** fits
  whatever's currently on screen with a bit of margin; the fixed buttons
  (1/5/10/20/50/100/200 m) pin the view to that many metres across,
  useful for comparing two markers' separation at a scale you can count
  gridlines against (e.g. the 1 m setting gives 10cm gridlines — good for
  checking a small separation like two markers mounted back-to-back).

Refresh the page to pick up a marker added or deleted since it loaded —
it isn't polled automatically, on purpose (see `aruco-prd.md`).

## Add Marker page

This is how you tell the system where a physical marker actually is, so
it can start aiding the xNAV650 with it.

### Before you start

Check the **GPS Position Mode** and **3D position accuracy** readouts on
this page first (same numbers oxts-nav's own pages show). A poor fix
(e.g. `SPS` instead of `RTK integer`, or a large accuracy number) means
whatever you're about to survey will be wrong by roughly that much —
better to wait for a good fix than survey now and find out later. There's
no way to fix a bad survey after the fact except doing it again.

### Doing a survey

1. Point the camera steadily at the marker you want to add (or replace —
   see below).
2. Click **Grab**. This freezes the view and computes a candidate
   position for every marker currently in view — it does **not** average
   over time, so hold the camera still rather than repeating the click.
3. Look at the frozen image's drawn axes. ArUco's own pose estimate
   occasionally gets the angle wrong (a known quirk, not unique to this
   project) — if the axes look physically implausible, click **Cancel**
   and try again rather than trusting it.
4. If it looks right: pick the marker's **id** from the dropdown (there
   may be more than one candidate if several markers were in view),
   enter its **size** in metres (remembers the last value you used,
   since it's usually the same marker type), and click **Save**.

Saving a marker id that's already in the map **overwrites it silently** —
useful for re-surveying a marker whose true position or heading has
drifted (e.g. after a year, or after it was bumped), but there's no undo,
so make sure you meant to.

## Markers page

A plain table of every marker currently in the map — id, size, lat, lon,
alt, heading, pitch, roll — with a **Delete** button per row (asks for
confirmation first; this can't be undone). Useful for cleaning up a
marker that was added by mistake, or one that's been physically removed
from the robot's environment.

## If a marker's position looks wrong

- Check the Home page's diagnostics table (only shown while that marker
  is in view) — compare the range and displacement numbers against a
  tape measure, and the vehicle heading against what you'd expect. This
  is exactly what caught a real bug during initial field testing (see
  `aruco-prd.md`'s "Real bugs found via testing").
- Check the GPS Position Mode / accuracy at the time it was likely
  surveyed — a marker saved during a poor fix will be off by roughly
  that fix's error, with no way to tell afterward except re-surveying it
  during a good fix and comparing.
- If it's simply wrong (moved, bumped, badly surveyed originally), just
  survey it again via Add Marker — saving over an existing id is
  designed for exactly this.

## Where your data goes

The marker map lives at `aruco/data/marker-map.yaml` — a plain YAML file,
one entry per marker, machine-written by the Add Marker and Markers
pages (not meant to be hand-edited, though there's nothing stopping you
if you know what you're doing — see `aruco-prd.md` for the exact field
meanings and precision requirements). It's read fresh from disk on every
use, so any edit — by the app or by hand — takes effect immediately, no
restart needed.
