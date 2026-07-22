# Navigate Service — User Manual

This is a how-to-use guide for the `navigate` service's web pages. For
the design/engineering background (why things work the way they do —
the pure-pursuit controller, why lat/lon rather than local XY is what's
stored, per-segment clearance vs. GR6-v1's one fixed tolerance, the
control-loop/drive-command relationship), see `navigate-prd.md` instead
— this document is deliberately just "how do I use it."

Open the navigate service from the manager's home page, or go directly
to its address (port 8006 by default) — e.g. `http://amundsen:8006`.

`navigate` doesn't talk to the motor controller or the xNAV650 directly
— it drives `drive` and reads `oxts-nav`'s live position, so both of
those need to be running for anything here to work.

## Run page (home)

The main page — select a saved path and run it.

- **Path** — choose a saved path from the dropdown, then **Load**. The
  path's shape appears on the map (grey line) once loaded.
- **Start** — begins path-following from wherever the robot currently
  is, *if* it's close enough to the path and pointed roughly the right
  way (within the configured entry tolerance — see Config page). If
  not, a message shows how far away and how many degrees off heading
  you are, so you can drive closer by hand and try again. Nothing
  starts silently on a bad entry point.
- **Stop** — immediately stops the robot and returns to idle. Safe to
  press any time.
- **State** — `idle`, `running`, `stopped_ok` (reached the end of the
  path cleanly), or `aborted` (see "If a run aborts" below).
- **Lateral error / clearance headroom** — how far off the path the
  robot currently is, and how much of that segment's allowed clearance
  is left before an abort. Watch this on a new or tight path.
- **Distance travelled**, **GPS position mode**, **horizontal
  accuracy**, **differential age** — live readouts for judging whether
  the run is going well.
- **Map** — the loaded path (grey) and the robot's actual driven trail
  (blue), with the same zoom buttons/gridlines/scale-bar style as
  aruco's Map page.

## Create Path page

Record a new path by driving it once and dropping points along the way.

- **Map** — shows the points dropped so far (green dots, numbered in
  order) and the robot's live position, so you can actually see the
  shape you're creating while you drive — useful since it's hard to
  drive a smooth line by hand (especially on gravel), and a jagged path
  causes sudden heading changes that can trip path-following's abort
  logic later. There's deliberately no automatic smoothing — this map
  is here so you can compensate yourself, not to paper over a wobbly
  path afterwards.
- **Jog square** (bottom-left, overlaid on the map) — drag to drive
  manually, same joystick behaviour as `drive`'s own Home page, proxied
  through `navigate` so it works without needing a direct cross-service
  connection. There's deliberately no manual pump toggle on this page —
  no reason to actually water anything while recording a path, only to
  record *where* the pump should be on later (see the checkbox below).
- **Move forward** (dropdown + button, just above the jog square) —
  drives forward a precise distance (0.2/0.5/1.0/2.0m) in a straight
  line from wherever the robot currently is, using the same speed as
  the dropdown above and the same pure-pursuit control path-following
  itself uses — much steadier than jogging by hand, especially on
  gravel. The robot may end up travelling a little further than the
  number picked (it's not trying to be exact); if something goes wrong
  partway (e.g. it can't hold a straight line), you'll see an alert with
  the reason and the robot stops immediately, same as a real run
  aborting.
- **Horizontal accuracy** (top-left, overlaid on the map) — watch this
  before dropping a point; a point recorded during a poor fix will be
  wrong later.
- **Speed for the segment starting at the next point** — one of
  0.2–0.8 m/s. This is the speed the robot will drive *from this point
  onward*, once the path is run.
- **Clearance around this segment** — how far the robot may drift off
  this segment before a run aborts here. A wide-open area can have a
  generous clearance; a tight spot needs a small one. This is exactly
  what replaces GR6-v1's one-size-fits-all tolerance — see
  navigate-prd.md if you want the reasoning.
- **Pump on for this point** — whether the pump should be on starting
  from this point.
- **Drop point** — captures the robot's *current* live position plus
  the speed/clearance/pump settings above, and appends it to the
  path being built. Drive a little, drop a point, repeat. If a wifi
  hiccup means you don't see a response and press it again, you'll get
  a warning (with add/discard options) if the new point would land
  within 30cm of the last one — a likely accidental duplicate, not a
  deliberate one.
- **New (clear)** — discards whatever's been recorded so far, to start
  over.
- **Save** — type a name and save. A path needs at least 2 points to
  save (a single point isn't a path).

## Paths page

- Lists every saved path with its point count and length.
- **Run** — loads that path and jumps to the Run page (still needs
  Start pressed there — loading isn't the same as starting).
- **Delete** — asks for confirmation first; can't be undone.

## Config page

A read-only summary of `navigate`'s configuration — control-loop rate,
path-entry tolerance, pure-pursuit lookahead/gains, the localisation-
accuracy and max-heading-correction abort limits, the wheel base (read
from `drive`'s own config, not a separate copy), and where paths are
stored. Edit the underlying values from the manager's Config page, not
here.

## If Start won't work

The message under the Start button tells you the distance and heading
angle to the nearest point on the path that would be enterable — drive
towards it (jog from the Run page isn't available; use another page's
jog, or drive by hand) and try Start again.

## If a run aborts

The State field shows the reason. Three things can trigger an abort:

- **Cross-track error exceeds this segment's clearance** — the robot
  has drifted further off the path than that segment allows. Either
  the clearance was set too tight when the path was recorded, or
  something (GPS glitch, an obstacle nudging the robot) pushed it
  further than expected.
- **Heading error exceeds the configured limit** — the robot's heading
  relative to where it needs to go is badly wrong; usually a sign
  something upstream (GPS heading, wheel encoders) is misbehaving.
- **Localisation accuracy exceeds the configured limit** — `oxts-nav`'s
  horizontal accuracy estimate has gotten too poor to trust (e.g. an
  RTK dropout under cover). Should get rarer once wheelspeed GAD aiding
  is in place (see top-prd.md).

In every case the robot stops immediately (not a graceful slow-down) —
this matches GR6-v1's behaviour. Press Stop, then Start again once
you're happy the underlying cause has been addressed.
