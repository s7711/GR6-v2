# PRD: Camera Service (camera)

See `../top-prd.md` for overall architecture decisions (process model, IPC,
shared config, systemd, version control) and `../ui-style.md` for web UI
conventions — this document only covers the camera capture service itself.

## Problem Statement

The robot has a Pi camera used today (in GR6-v1) for one thing — ArUco
marker detection — but the point of splitting vision into its own
service is that more than one consumer will eventually want the same
camera frames at once (ArUco now, other vision experiments later — see
`top-prd.md`'s note on wanting the Pi's other cores put to use). Nothing
should own "point the camera and capture frames" except one service, and
every consumer of frames should be independent of, and unaware of, each
other.

This service is that one thing: it captures frames, timestamps them
precisely enough to correlate with the xNAV650's nav data (see
`oxts-nav-prd.md`'s "Nav data feed"), and publishes them for any number
of consumers — plus a small web UI of its own for a live preview and
basic status.

## Prior art

GR6-v1's `bgCamera2.py` already does the camera-handling part well:
picamera2, a background capture thread, a condition-variable-based
`img()`/`img2()` API giving the latest frame plus its `SensorTimestamp`
(nanoseconds, `CLOCK_MONOTONIC`-based) and `ExposureTime`. Reused
close to as-is for the capture loop itself; what's new for GR6-v2 is
everything around it — cross-process publishing (GR6-v1 has one process,
so `aruco.py` just called `cam.img2()` directly), a web UI, and shared
config.

## Solution

An independent systemd service (`robot-camera.service`) that:

1. Opens the Pi camera via picamera2, captures frames in a background
   thread at a configurable rate.
2. Publishes each frame — pixels plus timing/exposure/gain metadata — via
   `multiprocessing.shared_memory`, guarded by a seqlock, per
   `top-prd.md`'s IPC choice for "large/high-rate data". Nothing consumes
   this yet (aruco is next), but it's in scope for v1, unlike the nav
   feed's build-ahead-of-need situation — a camera service that can't get
   frames out to another process isn't doing its job.
3. Serves a small web UI: a live preview page (MJPEG stream, same idea as
   GR6-v1's `/camera.mjpg`) and a read-only config page.
4. Does **not** do camera calibration in v1 — see Out of Scope. Deferred
   until straight after this, not dropped.

## Implementation Decisions

### Folder / naming
`camera/`. Considered `camera-manager/` — rejected as needlessly long;
every other service folder here is a short, direct noun (`oxts-nav`,
`manager`, `hello`), and "camera" is unambiguous in this repo.

### Resolution: fixed, not configurable
1280×960, hardcoded (a constant in the module, not shared config).
Camera calibration (`.gad_camcal.yaml` — see Out of Scope) is tied to a
specific resolution; letting resolution be changed via config would
silently invalidate calibration with no code change to flag it. The
camera can do higher resolutions, but there's no felt need to go beyond
1280×960 for ArUco range/accuracy. If a second, faster mode is ever
wanted (640×480 is known to work, faster but shorter marker-detection
range), that should be a deliberate code change (arguably a second,
explicitly-calibrated mode), not a config toggle.

### Frame rate: configurable
`camera_fps` in shared config (default 5, matching GR6-v1's current
`time.sleep(0.2)` rate). Real reason to expose this: ArUco processing at
1280×960 can't keep up with 5Hz on the Pi's CPU today (settles around
2Hz) — a future, lighter-weight vision consumer might genuinely want the
full rate, and there's no reason to hardcode a number that's really a
tuning knob.

One implementation improvement over GR6-v1 while porting: schedule
against a running target time (`next_tick += period; sleep(next_tick -
now)`) rather than a fixed `time.sleep(period)` after each capture — not
because this needs video-grade timing, but so a single slow frame
doesn't compound into permanent drift over a long session.

### Frame metadata published alongside pixels
- `timestamp` — seconds, `time.monotonic()`-based (from picamera2's
  `SensorTimestamp`, same conversion GR6-v1's `img2()` already does:
  nanoseconds / 1e9). This is the field that matters most: it's what
  lets a consumer correlate a frame with xNAV650 nav data via
  `ncomrx.machine_time_to_gps()` (see `oxts-nav-prd.md`'s "Nav data
  feed") — both clocks are `CLOCK_MONOTONIC`-based, so no separate
  time-sync mechanism is needed, just carrying the right number through.
- `exposure_us` — from picamera2 metadata (`ExposureTime`), already
  captured in GR6-v1.
- `gain` — from picamera2 metadata (`AnalogueGain`) — not currently
  captured in GR6-v1, worth adding now since it's cheap and a future
  consumer (or just the web UI) may want to know if the image is
  underexposed/noisy.
- `width`, `height` — included even though fixed for this service's
  lifetime, so a consumer never has to hardcode a resolution assumption
  of its own; it just reads what's actually in the buffer.
- `sequence` — incrementing frame counter, so a consumer can tell whether
  it's seen this exact frame before without comparing pixel data.

### IPC: shared memory + seqlock
- One named `multiprocessing.shared_memory.SharedMemory` segment
  (`camera_shm_name` in shared config), sized for the fixed resolution's
  RGB888 buffer plus a small fixed-layout metadata header (sequence
  counter, timestamp, exposure, gain, width, height).
- Seqlock protocol (per `top-prd.md`): writer sets the sequence counter
  to an odd value, writes metadata + frame, then sets it to the next
  even value. A reader reads the counter, reads metadata + frame, then
  re-reads the counter — if it changed (or was odd at the start), the
  reader retries. This gives lock-free reads for any number of
  consumers without blocking the capture thread.
- **Unlink-if-stale at startup:** if a previous run crashed or was
  killed without cleanup, the named segment may still exist. On
  startup, try to attach; if it's the wrong size or clearly stale,
  unlink and recreate it — per `top-prd.md`'s "Debugging" section,
  this is expected normal behaviour, not an edge case to skip.

### Web UI
- Standard shared header/nav (`shared/web`'s `use_shared_templates`,
  `use_shared_static`, page registry) — same conventions as `oxts-nav`
  and `manager`.
- **Home page:** live MJPEG preview (`/camera.mjpg`, same path
  convention as GR6-v1), served directly from the capture thread's
  latest in-process frame — no need to round-trip through the shared
  memory segment for the service's own page, only external consumers
  need that. Also shows current exposure/gain/fps as simple stat
  readouts, consistent with `ui-style.md`.
- **Config page:** read-only for v1. Shows the current resolution
  (fixed) and frame rate, with a note that resolution isn't
  configurable (calibration-tied, see above) and frame rate is set via
  the shared config file, not from this page (consistent with the
  manager's config editor being the one place config changes happen —
  no reason to duplicate that here for one field).
- **Calibration page:** not built in v1 — see Out of Scope for why, and
  "Calibration page (Calibrate): session lifecycle" below for the full
  design, decided now even though building it comes later. The
  page-registry pattern (`shared/web.py`) makes adding it a template
  (+ session-state module) change, same as every other page so far.

### Calibration file location (decided now; calibration itself still built later)

Worth deciding now even though the calibration procedure is out of scope
for v1 (see Out of Scope), because where the file lives shapes the IPC/
folder layout other services will assume.

- **Active/current calibration:** `shared/camera-cal.yaml` — dropped
  GR6-v1's `gad` prefix (`~/.gad_camcal.yaml`); this is camera-specific,
  not GAD-specific. Lives in `shared/`, not `camera/`, because it's a
  fact any consumer computing real-world geometry from pixels needs
  (`aruco` today, any future vision service) — exactly the same
  reasoning `top-prd.md` already gives for `config.yaml` being a single
  shared source of truth rather than one service's private data. Putting
  it under `camera/` would make every consumer reach into another
  service's folder for a cross-service fact.
  Gitignored, like `config.yaml`'s real IP — it's device/lens-specific.
  No meaningful `.example` placeholder exists for a camera matrix, so the
  expected format (camera matrix, distortion coefficients, the
  resolution it was measured at) is documented here instead of via an
  example file.
- **Rejected: push calibration through the frame IPC itself.** It's
  static data that only changes when a full recalibration is run — which
  requires restarting the camera service anyway (consistent with
  `config.yaml`'s no-live-reload convention used elsewhere). Repeating it
  on every frame in the high-rate shared-memory channel would add
  complexity there for no benefit; a consumer just reads the file once
  at startup.
- **Historical calibrations:** `camera/data/cal_yymmdd_hhmmss/` — one
  folder per calibration session, matching the convention already used
  for GR6-v1's real past calibrations (e.g. `cal_221110_vespucci_
  1280x960/`, containing that session's checkerboard images plus its own
  `calibration.yaml`), minus the resolution/location tag in the folder
  name — no longer useful now that resolution is fixed for this service,
  and it's recorded inside the yaml anyway (see below), so keeping it in
  the folder name too would just be redundant. Each folder holds the
  full record of one session: every accepted checkerboard image
  (`image0.jpg`, `image1.jpg`, ...) plus the `calibration.yaml` computed
  from them — self-contained, so a session can be reviewed or
  recomputed later without needing anything outside its own folder.
  `camera/data/` as a whole is gitignored — session output, not source,
  same reasoning as `manager/config-backup/`/`oxts-nav/xnav-config/`.
- **Promotion to active:** running a calibration session produces
  `camera/data/cal_.../calibration.yaml`; making it the one actually
  used means copying it to `shared/camera-cal.yaml`. Decided now (see
  "Calibration page (Calibrate): session lifecycle" below): a manual
  "Make this the active calibration" button on the results page, not
  automatic — the two files are deliberately distinct (history vs.
  active), not the same file referenced two ways.
- The file records the resolution it was measured at (GR6-v1's existing
  calibration loader already checks this against the live frame's shape
  before trusting it — worth keeping that check) — since resolution is
  fixed at 1280×960 for this service, one active file is enough for now;
  the resolution field is a sanity check, not a selector between
  multiple concurrent calibrations.

### Calibration procedure (from `temp-cam-cal`, ported after v1)

`temp-cam-cal/` (a temporary, will-be-deleted copy of existing, admittedly
"quick and dirty" working code) shows the procedure to port once v1 of
this service is running. Recorded here now so the design is decided even
though building it is still out of scope for v1:

- A generated list of ~27 target checkerboard poses (position + heading/
  pitch/roll spread), each expressed as a 4-point polygon in image
  pixels (`generate_checkerboard_positions()` — pure geometry, no camera
  dependency, ports unchanged).
- A calibration page shows the live image with the current target
  polygon overlaid (mirrored, to make positioning the checkerboard by
  hand easier) — this needs its own MJPEG-style stream with the overlay
  burned in server-side (`cv2.polylines` + `cv2.findChessboardCorners`
  drawing), separate from the plain `/camera.mjpg` preview, and only
  active during a calibration session — the extra per-frame OpenCV work
  shouldn't run during normal operation.
- Each frame is tested (`cv2.findChessboardCorners`, then checked against
  the target polygon: all corners inside it, and a convex-hull-area
  ratio against the polygon for "big enough") — "No checkerboard" /
  "Align checkerboard" / "Too small" / "Ok" — and on "Ok" the frame is
  saved and the procedure advances to the next target pose.
- **Real bug found by testing (both a geometric proof and a real
  checkerboard that wouldn't trigger "Ok" at all):**
  `cv2.findChessboardCorners` only ever finds a board's *internal*
  corners — inherently inset from the full printed board's outer edge —
  so their convex hull can never reach the full area of a polygon
  representing the *outer* edge, even for a mathematically perfect
  placement. For this 4×6-corner board, the best a perfect fit can ever
  achieve is a ~0.429 ratio; the existing code's own "too small"
  threshold was 0.7 — *above* that ceiling, so it could never pass, no
  matter how well the board was positioned. Fixed by normalizing the
  ratio against this ceiling (`fit = raw_ratio / MAX_FIT_RATIO`, pass
  bar 0.8) instead of comparing the raw ratio to an arbitrary constant.
  This normalization isn't a strict 0–1 percentage in practice, though —
  tilted poses involve genuine perspective distortion that doesn't
  preserve the flat square-unit ratio exactly, and testing found good
  real fits scoring well above 1.0 (e.g. ~1.5–2.0) rather than
  approaching it from below. Treated as "higher is better, 0.8 to pass"
  — a live, monotonic diagnostic, not a precise percentage — and
  surfaced on the Calibrate page as a running "fit score" for exactly
  this reason: it's the fastest way to notice a threshold problem like
  this (or simple misalignment) instead of just "nothing is happening."
- **Second real gap found after the fit-threshold fix, from an actual
  test run:** containment + size alone don't discriminate *position* or
  *orientation* well enough — a real calibration run completed, but many
  of its captured images were "almost identical," meaning nearby target
  positions (or a flat board substituting for a tilted target) were
  satisfying "Ok" without the board genuinely moving/tilting between
  shots. Confirmed geometrically: tilting a target by 25°/20°/15°
  (H/P/R) only shifts its polygon's *centroid* by ~20px out of a ~600px
  frame width — containment alone can't tell a flat board from a tilted
  one held at the same spot. Fixed with two additional, independent
  checks, combined into one `alignment_error` (0 = exact match, lower is
  better):
  - **position_error** — detected-corner centroid vs. target polygon
    centroid, normalized by polygon scale. Catches "right shape, wrong
    place" (e.g. an adjacent grid position).
  - **shape_error** — the detected corners' 4 extreme points, normalized
    (centred, unit-scale) and compared to the target polygon's shape,
    trying all 4 symmetric relabellings a rectangular checkerboard can
    be detected under (this board's 5×7 squares are both odd, so a
    single-axis mirror also preserves its colouring, not just a 180°
    rotation — both must be tried, or a correctly-oriented board could
    register as "wrong shape" purely from an arbitrary detection
    ordering). Catches "right place, wrong tilt."
  - Threshold (`ALIGNMENT_ERROR_THRESHOLD`) picked deliberately loose to
    start (now 0.5, loosened once more after a real test run — see
    below): the same synthetic-image limitation that made the fit ratio
    hard to validate applies here too (development test images aren't a
    true tilted perspective render — see the earlier note — so a
    stricter value risked being un-satisfiable in practice the same way
    the original "too small" threshold was). Meant to be tuned from real
    numbers once tried on an actual camera, not guessed further from
    synthetic tests alone — which is exactly what happened next.
- **Third real bug, found from an actual (non-tilted!) test attempt:** a
  real board on a flat, unrotated target (position 0 — no tilt at all)
  couldn't trigger "Ok" even when the operator was confident it was
  well-placed (`fit=1.47`, comfortably over the 0.8 bar, but
  `alignment_error=1.27`, far over the 0.3 bar at the time). Since
  position 0 has zero rotation, this ruled out "genuinely hard tilt" as
  the explanation. Root cause, proven geometrically: `shape_error` was
  comparing the detected *internal* corners' shape directly against the
  *outer* board polygon's shape — two rectangles of genuinely different
  aspect ratio ((cols-1)×(rows-1) vs (cols+1)×(rows+1) squares — 3×5 vs
  5×7 here), so there was a real, systematic bias baked in even at
  perfect alignment (confirmed: ~0.08 from the aspect mismatch alone,
  before this fix — not the whole 1.27, but a real, provable component of
  it). Fixed by comparing against `_expected_inner_quad(polygon)` instead
  of the raw outer polygon — the internal corners' expected position,
  approximated via bilinear interpolation of the polygon's 4 corners
  (exact for a flat target, an approximation for tilted ones, since true
  perspective isn't bilinear). Confirmed by testing: `shape_error` for an
  exact synthetic match dropped from ~0.08 to ~0.003 once fixed. Also
  loosened `ALIGNMENT_ERROR_THRESHOLD` further (0.3 → 0.5) as a safety
  margin on top of the fix, since the fix alone wasn't confirmed to
  explain the entire originally-reported 1.27.
- **Diagnosability addition, prompted by the above:** the live page can
  only show so many numbers before it's more clutter than help (already
  true with just `fit`/`alignment_error`), and turned out to be hard to
  read anyway while actually holding a checkerboard with both hands.
  Every frame `_run` evaluates is now also appended to
  `<session_dir>/capture_log.csv` (`timestamp, position, message, fit,
  position_error, shape_error, alignment_error, corners, polygon`) — the
  *decomposed* components, not just the combined score, **and now the
  actual detected corner points and target polygon points** (JSON-encoded
  lists), added after a real test run still couldn't get below
  `alignment_error≈1.2` even after the aspect-ratio fix above, which
  wasn't confirmed to explain a number that large. Rather than guess at a
  fourth fix blind, the point-level data lets position/shape/fit all be
  independently recomputed and plotted from what the camera actually saw,
  which is a strictly stronger diagnostic than any more scalar summaries
  would be. Kept permanently (not deleted after a successful run, and not
  temporary debug code to be later removed) — it's small (text, not
  images) and self-contained within its own session folder, consistent
  with every other per-session artifact.
- **Fourth real bug, found from the point-level log data above, on a
  real (still non-tilted) position that couldn't trigger "Ok":** the
  `shape_error` fix two rounds ago tried 4 relabellings of the detected
  corners, reasoning about the checkerboard's own coloring symmetry
  (identity/180°/either mirror). That conflated two different things.
  The real logged corner data showed `extreme_corners[0]` (assumed "top
  left") actually landing on the physical top-right corner —
  `extreme_corners` is extracted from cv2's row-major corner order,
  which can start at any of the 4 physical corners and trace around in
  either direction depending on fine detection details, independent of
  the checkerboard's own symmetry. That's a full 8-element (4 rotations
  x 2 directions) group, not the 4-element one alone — the other 4
  relabellings were never tried, so a genuinely good match could still
  score high. Recomputing the real logged session with all 8 confirmed
  it: the best real frame's `shape_error` dropped from ~1.07 (a
  near-constant, clearly-wrong floor) to ~0.17, and 54 of 103 real
  detected frames would have passed at the existing 0.5 threshold —
  strong evidence this, not the threshold value, was the dominant real
  blocker all along.
- **`abort()` used to also delete the session folder**, which directly
  fought the point above — aborting is often exactly when there's
  something worth reviewing, and deleting destroyed the log needed to
  find this exact bug. Fixed: `abort()` no longer deletes anything (see
  "Calibration page (Calibrate): session lifecycle" below).
- **Known remaining crash, fixed:** `abort()` deletes its session's
  folder from a different thread than `_run()`, which could still be
  mid-check (blocked on `cam.latest()` or inside `check_checkerboard()`,
  both slow relative to a lock acquisition) when that happens — writing
  the log/image afterward hit a bare `FileNotFoundError`, an unhandled
  exception that silently killed the daemon thread and left the session
  stuck in "capturing" forever (crashed a real session; reproduced
  deliberately with a slowed-down fake camera to confirm the fix).
  Fixed by re-checking `self.state` (under the lock) immediately before
  any file I/O, and wrapping the I/O itself in `except OSError: return`
  as a second layer — a race losing to `abort()` is expected and
  harmless, not a bug to crash loudly about.
- **`camera/data/` growth:** completed sessions are never automatically
  deleted (only `abort()` deletes anything, and only its own incomplete
  session — see above). Decided to leave it that way for now rather than
  add pruning (e.g. keep the last N) — calibration is infrequent and
  `data/` is gitignored, so unbounded local growth costs disk space, not
  much else. Trivial to add a keep-last-N policy later if it ever
  actually becomes a nuisance.
- Status (message, position index, resolution) is pushed to the browser
  over a websocket — the existing code hand-rolls this; porting it means
  just using `ws-utils.js`'s `connectWs`/`fillFields`, consistent with
  every other page in this project.
- At the end, `cv2.calibrateCamera()` runs over all saved images and
  writes `camera_matrix`/`distortion_coefficients`/`resolution` via
  `cv2.FileStorage` — this is the part that becomes
  `camera/data/cal_.../calibration.yaml` (see above).
- **Addition over the existing code:** `cv2.calibrateCamera()`'s return
  value is the RMS reprojection error, which the existing code discards.
  Report it (and ideally the per-image error too) alongside the result —
  standard practice, and the cheapest possible way to catch a bad
  session immediately instead of only noticing later as unexplained
  marker-pose error.
- **Known gap, explicitly not fixed initially:** there's no check that
  the checkerboard is stationary before capturing — motion blur or a
  mid-move capture can degrade a calibration silently. Flagged in the
  existing code as a TODO, and agreed to be the biggest real weakness of
  the procedure as it stands. Worth fixing once the procedure is ported,
  not a blocker to porting it as-is first.
- Extra dependencies this needed that aren't used elsewhere in this
  project: `opencv-contrib-python-headless` (`findChessboardCorners`,
  `calibrateCamera`), `scipy` (`ConvexHull`), `matplotlib` (`Path`,
  point-in-polygon test).

### Dependencies: numpy pin, not apt

Installing `opencv-contrib-python-headless`/`scipy` via pip on the same
venv as `picamera2` (see "IPC: shared memory + seqlock" / the
`--system-site-packages` note in `requirements.txt`) hits a real
conflict, not just a "whichever shadows the other" nuisance: NumPy 2.0
was a deliberate ABI break, and picamera2's `simplejpeg` extension is
built against Debian's system numpy 1.24.x. Left to its own resolution,
pip picks the latest opencv/scipy, which pull in numpy 2.x and break
picamera2's import (`ValueError: numpy.dtype size changed...`).

Fix: pin `numpy<2` in `requirements.txt`. That forces pip's resolver to
pick numpy-1.x-compatible releases of everything else too — confirmed
(by testing) this lands on `opencv-contrib-python-headless` 4.11.0 (same
version already independently working via a `pip install --user` on
amundsen, and comfortably past the 4.7 cutoff where `cv2.aruco.
ArucoDetector` — the API `aruco.py` already uses — was introduced),
`scipy` 1.17.1, `matplotlib` 3.11.0, all mutually consistent.

Considered apt instead (`python3-opencv` etc., same treatment as
`picamera2` — apt packages already match the system's own numpy, so
nothing to pin). Not used: apt's opencv isn't currently installed on
amundsen at all, and its candidate version wasn't verified — risking a
downgrade below the ArucoDetector cutoff for no benefit once the pip+pin
route was confirmed working.
- Checkerboard-specific constants (4×6 internal corners, 30mm squares,
  ~0.21×0.15m board, ~0.5m working distance) are specific to the
  physical board used for past calibrations — carried over as-is unless
  a different board is used, in which case these need updating together
  (they're derived from the same physical object).
- **Addition over the existing position set: distance variation.** All
  27 existing target poses use the same `dz=0.5` (never overridden) —
  good coverage across the frame and across heading/pitch/roll, but zero
  scale/distance diversity. Worth adding a handful more positions at a
  closer (~0.3m) and one or two farther (~1–1.5m) distances, reusing
  `generate_checkerboard_positions()` unchanged (it's a true perspective
  projection, so the target polygon already scales correctly with `dz`
  with no other code change needed). Motivated by ArUco markers actually
  being seen well beyond 0.5m in practice (`gad_aruco.py`'s lever-arm
  variance comments imply a working range out to 3–4m).

### Calibration page (Calibrate): session lifecycle

Decided in full even though building it is still out of scope for v1
(see Out of Scope) — this is the design to build against once it's next.

- **Server-side singleton session, not per-connection state.** The
  in-progress (or just-finished) calibration session is one object
  owned by the camera service itself — same pattern as `cam`/`nrxs`
  elsewhere in this project — not something tied to a particular
  websocket/browser connection. This one choice is what makes several
  of the trickier questions below simple rather than needing their own
  special-case handling:
  - **Closing the browser tab** does nothing to the session — it isn't
    "attached" to any connection, so there's nothing to abort or clean
    up on disconnect. Reopening the Calibrate page just reflects
    whatever the current state already is.
  - **No "resume" feature needed** — there's never anything paused to
    resume. The session either is running, finished, or doesn't exist;
    a page load always shows the true current state, not a snapshot
    that can go stale.
  - **Two browser tabs/windows open at once** are safe by construction:
    both are just views of the same one session, not two independent
    calibrations racing each other. No locking, no "someone else is
    calibrating" error needed.
- **Page states:** idle (no session) → capturing (target polygon
  overlay + "Place in the polygon" / "Image N of <count>", same status
  line style as the existing code) → computing (brief — a few seconds
  for ~27+ images of a 4×6 board on a Pi 4, not worth a progress bar;
  just a "Computing…" message in the same status line) → done (results)
  or aborted (back to idle). Whichever state is current is what any page
  load / any open tab shows.
- **Start and Abort are two separate, always-visible buttons**, not one
  button that toggles — greyed out per state (Start disabled once a
  session is running; Abort disabled when idle) rather than swapped in
  place, specifically so a double-click can't hit "Start" and
  accidentally register as "Abort" (or vice versa) a moment later.
  Considered a delay-before-abort-is-clickable pattern instead — more
  complex for no real benefit here, since a mis-click on a *disabled*
  button simply does nothing, which is enough protection on its own.
- **Abort** stops the capture loop. Originally also deleted that
  session's partial `camera/data/cal_.../` folder ("no `calibration.yaml`,
  nothing worth keeping") — **reversed** once `capture_log.csv` existed
  (see "Diagnosability addition" below): aborting is often *exactly*
  when there's something worth reviewing (a position that wouldn't
  trigger), so deleting on abort was destroying the one diagnostic that
  could explain why. Now leaves the folder in place, consistent with the
  "let `camera/data/` grow" decision — leftover disk space, not lost data.
- **Results page:** RMS reprojection error (the addition agreed earlier)
  shown prominently, colour-coded with the same `success`/`warning`/
  `danger` semantics used everywhere else in this project — rule of
  thumb, not a hard law: roughly under ~0.5px good, up to ~1.0px
  acceptable, above that suspect. Also shows resolution and image count.
  A **"Make this the active calibration"** button copies this session's
  `calibration.yaml` to `shared/camera-cal.yaml` — this is the answer to
  the "promotion to active" question left open above: deliberately a
  manual action, not automatic, so a bad session can't silently become
  the live one.

## Config additions (shared config file)

Under this service's own entry: `camera_fps` (capture rate),
`camera_shm_name` (shared memory segment name for the frame IPC), plus
the usual `unit`/`host`/`port`/`web_ui` fields every service has.
Resolution is deliberately **not** here — see "Resolution: fixed, not
configurable" above.

## Testing Decisions

- The capture loop needs real hardware (the Pi camera) — no practical
  way to unit-test frame capture itself without it.
- The shared-memory publish/seqlock logic can be tested independently of
  the camera: write synthetic frames + metadata into the segment from a
  small script, and confirm a separate reader process gets consistent,
  non-torn data even under rapid concurrent writes.
- The web UI's MJPEG stream and config page can be exercised without the
  real camera by substituting a synthetic frame source in the capture
  thread for local dev/testing (mirrors how `oxts-nav` was tested against
  synthetic NCOM packets rather than a real xNAV650).

## Out of Scope (v1)

- **Camera calibration.** There's existing calibration code (not yet
  shared into this repo) that automates checkerboard-image capture —
  it's not especially sophisticated, mainly there because pressing a key
  while holding a checkerboard still is awkward. Planned as the very
  next piece of work after v1 of this service, not dropped — deliberately
  scoped out only so v1 stays small enough to get running and tested
  first. Where its output file lives is, however, decided now — see
  "Calibration file location" above.
- **Configurable resolution.** See "Resolution: fixed, not configurable"
  above — a deliberate, not a deferred, decision.
- **Consumers of the shared-memory frame feed** (ArUco detection, any
  other vision processing) — that's the next service, not this one.
- **Editing config from the camera's own Config page.** Read-only for
  now, consistent with keeping the manager as the one place config gets
  edited (see `manager-prd.md`).
