# PRD: ArUco Marker Detection Service (aruco)

See `top-prd.md` for how this fits the overall migration (vision was split
into `camera`, done, and `aruco`, this document).

## Problem Statement

GR6-v1 ran ArUco marker detection as threads inside one monolithic process
(`aruco.py`, `gad_aruco.py`, `main.py`), sharing memory directly with the
camera-capture and nav-decode code because it was all one process. GR6-v2
has since split those into independent services: `camera` (frame capture +
calibration, publishing over shared memory) and `oxts-nav` (xNAV650 decode,
publishing nav data over a Unix socket feed and its own `/ws/nav`
websocket). `aruco` is the next service: detect ArUco markers in the
camera's live frames, compute each marker's pose, and turn known markers
into position/heading aiding updates ("GAD" — Generic Aiding Data) sent to
the xNAV650, to keep the nav solution accurate in GNSS-poor areas (this was
the whole point in GR6-v1 — the garden has spots with poor sky view).

## Prior art (GR6-v1)

Reviewed the old repo (`aruco.py`, `gad_aruco.py`, `bgCamera2.py`,
`config.py`, `map.csv`, `static/gad.html`, `main.py`). What carried forward,
and what didn't:

- **Detection**: `cv2.aruco.DICT_6X6_250`, `ArucoDetector` with
  `CORNER_REFINE_SUBPIX`, `cv2.aruco.estimatePoseSingleMarkers()`. Marker
  size was hardcoded (0.10m) despite `map.csv` having an (unused) per-marker
  size column — fixed in this version, see below.
- **Coordinate frames**: a documented rotation chain relating nav, body,
  camera and marker frames — carried forward deliberately close to
  verbatim (see below), not re-derived, since it's dense and easy to get
  subtly wrong rewriting from scratch.
- **Marker locations**: `map.csv`, hand-measured, hand-entered, nothing
  wrote back to it. A partial "unmapped marker" feature gave a
  single-observation position estimate for markers not in the map
  (shown greyed-out on the web page) but wasn't a real survey tool — no
  averaging, no persistence, no promotion to the map. No automatic
  survey/SLAM existed, and this PRD doesn't build one either (see Out of
  Scope) — deliberately too complex for what this project needs.
- **Web page**: one page (`static/gad.html`) bundled a live MJPEG feed,
  a plan-view scatter map, a measurement table, and manual GAD debug
  controls. Split into four pages this time (see below).
- **Calibration**: loaded from an external, hand-produced yaml file.
  GR6-v2's `camera` service already produces a compatible one
  (`shared/camera-cal.yaml`) — consumed directly, no new convention needed.
- **Known unresolved issue carried forward as a risk, not a solved
  problem**: image-capture-time vs. NCOM-time alignment was never fully
  solved to better than ~1ms in v1. GR6-v2's frame timestamps (picamera2
  `SensorTimestamp`, monotonic ns since boot) and `oxts-nav`'s
  `machine_time_to_gps()` share the same `time.monotonic()` base across
  processes, which should make this easier — but it still needs care, not
  an assumption that it's already fixed.
- **Known limitation, not fixable by us**: the NCOM output has a field
  meant to report GAD health/status, but it's broken in the current xNAV650
  firmware (OxTS haven't fixed it yet). The newer UCOM output format fixes
  this, but adopting it requires a firmware update, planned for later, not
  this service. Until then, we can't fully verify from telemetry alone
  that heading-GAD is being accepted/working correctly — worth being
  conservative with heading aiding until UCOM is available.

## Solution: architecture / data flow

`aruco` is its own service/process, following the `camera`/`oxts-nav`
pattern:

- **Input: camera frames** — read from `camera`'s shared-memory feed
  (`shared/frame_ipc.py`'s `FrameReader`), not a new capture of its own.
- **Input: camera calibration** — read `shared/camera-cal.yaml` directly.
- **Input: nav data (Python side)** — read `oxts-nav`'s `nav_feed` Unix
  socket for the current position/attitude, needed to convert a detected
  marker's camera-relative pose into a geodetic (lat/lon/alt) position for
  the marker map, and for GAD timestamp alignment.
- **Output: GAD updates to the xNAV650** — position + heading, sent
  directly over the network to the xNAV650 via `oxts_sdk`. This is a
  separate connection to the device from `oxts-nav`'s own NCOM decode
  socket (GAD aiding and NCOM decode are independent connections to the
  same device, as in v1).
- **Browser-side data**: pages get their live data from websockets — but
  not all from `aruco` itself. `aruco` only publishes what it uniquely
  knows (detections, GAD status) over its own websocket, at close to frame
  rate. Nav/position/trail data is fetched by the browser **directly from
  `oxts-nav`'s existing `/ws/nav`** (a second, independent websocket
  connection from the same page) rather than being re-published through
  `aruco` — avoids `aruco`'s Python side needing to know or duplicate
  anything about nav internals. Needs a small `service_url(name)` helper
  added to `shared/web.py` (today only `manager_url()` exists) so a page
  can be told another service's host:port from `config.yaml`.
- **Marker map**: read straight off disk on every request, no in-memory
  caching layer — it's tiny, so this is cheap, and it means edits (from the
  Add Marker page) are visible immediately with no cache-invalidation
  logic and no service restart. The Map page fetches it once per page
  load (not polled) — new markers appear on refresh, not automatically;
  deliberately kept this simple (see Out of Scope).

## Coordinate frame convention (carried forward from v1, verbatim)

This is the part most worth getting exactly right, and the part general
LLMs (including other assistants consulted on this) have been observed to
get wrong — particularly confusing the capital-`C`/lowercase-`c` camera
frames. Comments in the actual code should spell out each frame in words,
not rely on case alone.

Six frames:

- **`n`** — nav/NED frame: X north, Y east, Z down. In v1 this was anchored
  at one shared config origin (`AmBaseLLA`); in this version there is no
  shared origin — each marker's own lat/lon defines its own local tangent
  plane, computed at the point of use (position comparison / GAD update),
  which also avoids flat-earth approximation error growing with distance
  from some single shared point.
- **`b`** — vehicle body frame: X forward, Y right, Z down. Related to `n`
  by `V_n = C_nb(HPR) · V_b`, `C_nb = C_Heading · C_Pitch · C_Roll` — the
  xNAV650's own HPR convention.
- **`C`** (capital) — raw OpenCV camera frame: X right (viewed from behind
  the camera), Y down, Z along the camera's boresight into the scene. Fixed
  by OpenCV's own conventions, not a choice we make.
- **`c`** (lowercase) — the same physical camera, re-expressed in an
  X-forward/Y-right/Z-down convention purely so its mount angle onto the
  body can be written as an ordinary HPR triple.
- **`M`** (capital) — the marker frame exactly as `cv2.aruco`'s pose
  estimation returns it: X right (facing the marker from the front), Y up,
  Z towards the camera (out of the marker's printed face).
- **`m`** (lowercase) — a second convention for describing the same
  physical marker, chosen so its pose in the map file can be written as an
  ordinary HPR triple: X out the back of the marker, Y to the marker's
  right, Z down. At `Hm=Pm=Rm=0`, `X_M` points east while `X_m` points
  north — different physical directions for "the marker's X axis" under
  the two conventions; not a typo, just two label choices for one object.

Chain (read right-to-left: "transform a vector in the rightmost frame into
the leftmost frame"):

```
C_nb = C_nm · C_mM · C_MC · C_Cc · C_cb

V_b →(C_cb)→ V_c →(C_Cc)→ V_C →(C_MC)→ V_M →(C_mM)→ V_m →(C_nm)→ V_n
```

- `C_cb` — fixed camera-mount HPR (config: `HPR_cb`). **Currently `[0,0,0]`
  in the live config — an assumption, not a measured bore-sight.** The
  camera has never been properly bore-sighted; this is likely the
  dominant source of error in marker measurements and GAD updates.
  Bore-sighting is real, complex work, explicitly deferred (see Out of
  Scope) — but a config/UI page for entering these values is in scope,
  since operators need a place to update them once bore-sighting does
  happen.
- `C_Cc` — fixed, definitional, relates the two camera-frame conventions.
  Not measured, not configured.
- `C_MC` — from ArUco's own pose output (`(C_CM).T`, `C_CM` being what
  `estimatePoseSingleMarkers`'s Rodrigues vector gives). The one live
  measurement in the chain.
- `C_mM` — fixed, definitional, relates the two marker-frame conventions.
- `C_nm` — built from the map file's marker heading/pitch/roll the same
  way `C_nb` is built from vehicle HPR.

A note on naming `HPR_cb`: technically, heading/pitch/roll describe the
nav→body relationship specifically; a camera mount offset is really a
plain ZYX Euler triple, not "heading/pitch/roll" in the compass-and-level
sense. Keeping the HPR name anyway, deliberately — renaming to ZYX doesn't
remove the need to explain the convention to a reader, and HPR still
communicates more (rotation order, general intuition) than ZYX would, so
long as one clear code comment says explicitly that "heading" here has no
compass meaning — it's just the Z-axis term in the same rotation-
composition convention, reused for the fixed camera-mount offset instead
of the vehicle's live attitude.

## Implementation Decisions

### Marker map file: YAML, per-marker lat/lon, no shared origin

Format: YAML, one record per marker:

```yaml
- id: 11
  size: 0.097
  lat: 52.234722222
  lon: -1.460833333
  alt: 175.000
  heading: 170.3
  pitch: -2.57
  roll: -3.93
```

**Precision**: lat/lon must be written with **9 decimal places** (not the
6 in a careless copy-paste) — each decimal place of latitude is worth
roughly a factor of 10 in ground precision (6 places ≈ 11cm, 7 ≈ 1.1cm,
8 ≈ 1.1mm, 9 ≈ 0.1mm), and longitude is at least as precise as latitude at
the same decimal count since its degree-length shrinks with
`cos(latitude)`. 9 places gives comfortable sub-mm margin everywhere,
against a target of 1mm or better. This is purely a
"how many digits get formatted into the file" concern, not a numeric-type
one — float64 (what Python/YAML/numpy all use) carries this precision
natively; the risk is only ever a careless `f"{lat:.6f}"`-style format
string when writing the file, which the Add Marker save code must avoid.

Decided lat/lon over local NED-from-an-origin because a marker's true
position shouldn't move just because someone redefines or re-measures the
origin — with lat/lon per marker there's no shared origin to configure or
get wrong at all (removes v1's `AmBaseLLA`/`MapBaseLLA` config entirely).
It also matches how the marker would actually be surveyed in practice: walk
up to it with the RTK-corrected xNAV650 itself and read off what it
reports, which is naturally lat/lon.

YAML over CSV: this file becomes a machine-owned artifact once the Add
Marker page exists (see below) — it's no longer hand-authored in a
spreadsheet, so CSV's main advantage (Excel-editability) stops applying,
while YAML extends more cleanly for future fields (e.g. accuracy, see Out
of Scope) since records are read by key, not column position. A CSV export
function is an easy, cheap future addition if hand-editing/inspection in
Excel is ever wanted again — not needed now.

Marker size (`AmSize` in v1) is read and used properly this time — no
longer hardcoded — since the map already has to carry a size value.

### GAD scope: position + heading, not roll/pitch

Position and heading aiding are in scope (v1 confirmed heading GAD does
work). Roll/pitch GAD is explicitly excluded, not deferred — ArUco
measurements are known to give non-white innovations, and feeding that into
the Kalman filter corrupts the accelerometer bias estimate. Heading is
worth the same risk because IMU heading drift causes enough downstream
problems that fixing it periodically via markers outweighs the innovation-
whiteness concern; roll/pitch don't have an equivalent payoff, so the
same trade isn't worth making for them.

### Pages: four, each simple, each websocket/JSON-driven from HTML+JS

Following the same principle as `oxts-nav`: pages should be changeable
without touching Python, by having the server side just publish JSON over
a websocket (or serve the map file as-is) and letting the page's own JS
decide what to render.

- **Home** — live view: MJPEG feed with detection overlay (marker
  outlines/axes), plus at-a-glance status (frame rate; GAD status — is a
  position/heading update currently going out, last accepted marker id),
  and a diagnostics table (visible only while an unmapped marker is in
  view) showing the intermediate numbers behind a survey estimate —
  vehicle H/P/R used, raw camera-frame `tvec`, body-frame and NED
  displacement, range — added after a real field bug (see "Real bugs
  found via testing" below) needed exactly this to diagnose; kept
  permanently since "hand-verify a measurement against a tape measure" is
  a recurring, not one-off, need. Data: `aruco`'s own websocket.
- **Map** — plan view, vehicle-centred (recentred live on the current
  position every redraw — no fixed origin, see the origin discussion
  below), with gridlines and a scale bar at a matching "nice" spacing,
  Auto/1/5/10/20/50/100/200m zoom buttons floating over the canvas
  (rather than a separate control row, so the canvas stays full-size),
  current position + nav trail (from `oxts-nav`'s `/ws/nav`, fetched
  directly by the page, not proxied), known markers (fetched from the map
  file on page load), and unmapped-marker rough estimates shown greyed
  out (ported from v1 — genuinely useful precisely *because* the map is
  hand-surveyed, giving a starting point to go and measure properly,
  rather than a real survey result).
- **Add marker** — the deliberate measure-and-save workflow (see below).
- **Markers** — a plain table of every surveyed marker (id, size, lat,
  lon, alt, heading, pitch, roll) with a Delete button per row. Added
  once markers actually needed correcting/removing in the field (e.g. a
  marker resurveyed after its true heading turned out to have drifted
  from a year-old value) — not part of the original three-page plan, but
  a small, obvious enough addition not to need its own design discussion.

### Add Marker workflow: single-shot Grab, then Cancel or Save

No averaging over multiple stationary frames — INS position error changes
slowly enough that the current live estimate is already effectively
averaged, and a *stationary* robot actually has worse observable heading
(heading needs accelerations to be observable), so averaging while
stationary could make things worse, not better. Multi-position averaging
(moving the robot to several spots and combining estimates) is a
plausible future enhancement, not attempted here.

Flow:
1. **Grab** — freeze the current frame + detection + the nav fix at that
   instant; compute the resulting marker lat/lon/heading; draw the pose
   axes on the frozen image so the operator can eyeball whether it looks
   physically sane (ArUco's pose estimate is known to sometimes get the
   angle wrong).
2. **Cancel** — discard the grabbed frame, go back to live, try again.
3. **Save** — commit the grabbed pose to the map file. The operator
   supplies/confirms the marker id (shown from the detection, though there
   is no independent way for a human to verify it's correct — it's trusted
   as-is) and the size (a text field, pre-filled with whatever was entered
   last time, since it's usually the same).
   - **Duplicate id**: overwrite, no warning, for now — there's currently
     only one set of markers in use, so this is a non-issue in practice.
     A future version might allow multiple markers sharing one id so long
     as they're spatially separated (e.g. 5m+) — not built now.

Also shows **GPS Position Mode** and **3D position accuracy**
(`sqrt(NorthAcc² + EastAcc² + AltAcc²)`), read live and directly from
`oxts-nav`'s own websocket (same borrow-it-directly pattern as the Map
page) — added after a real field case where a marker was suspected to
have been surveyed during a poor GNSS fix, with no way at the time to
tell after the fact. These let the operator judge *before* saving whether
the fix quality was good enough to trust, rather than finding out later.

## Config additions (shared config file)

Other fields seen in the old `.GR6_configuration.json` — `MapBaseLLA`/
`MapResolution`/`MapSizeMeters`/`Imu2*Wheel_i`/`WheelSf` — belong to a
future occupancy-map or path-following service, not here.

`HPR_ib` (IMU-to-body mounting) is a partial exception: it conceptually
belongs to `oxts-nav` (added to *its* config section, not `aruco`'s), but
`aruco` reads it cross-service — the xNAV650's GAD API specifically wants
lever-arm/heading-alignment values in IMU-frame terms, regardless of what
the device's own `mobile.vat` alignment does to its primary NCOM output
(see `gad.py`, and the coordinate frame section above). It is **not**
used to reinterpret `nav.Heading/Pitch/Roll` themselves — those are
already vehicle/body-frame once `mobile.vat` is loaded onto the device,
and treating them as IMU-frame needing a further `HPR_ib` rotation was a
real bug, found via a field test — see "Real bugs found via testing"
below.

```yaml
aruco:
  unit: robot-aruco.service
  host: 0.0.0.0
  port: 8004
  web_ui: true
  marker_map_file: aruco/data/marker-map.yaml
  camera_extrinsics:
    hpr_cb: [0, 0, 0]        # camera mount HPR relative to body; unmeasured/assumed, see bore-sight note above
    d_xc_b: [0.0775, 0.002, -0.07]   # camera displacement in body frame, metres
```

(`xnav_ip` for sending GAD is already available from `oxts-nav`'s existing
config entry — reused, not duplicated.)

## Real bugs found via testing

Matching `camera-prd.md`'s convention of keeping a permanent log of real
bugs found once actual testing (not just review) happened — these were
all found via genuine field/hardware use, not caught by code review or
the unit tests in `test_coords.py` (which check the rotation-chain
algebra is internally consistent, not that it models the real world
correctly):

1. **Wrong attitude frame for surveying a new marker** (found 2026-07-19,
   marker 10 real-world test: marker known to be ~61cm dead ahead showed
   up ~7cm away in a clearly wrong direction). `survey.py` assumed
   `nav.Heading/Pitch/Roll` were IMU-referenced and rotated them through
   `HPR_ib` to reach body frame — but `oxts-nav` loads `mobile.vat`
   ("Vehicle attitude") onto the xNAV650, so its primary NCOM output is
   *already* vehicle/body-frame. The extra rotation, through a large and
   exactly gimbal-locked (`pitch=90°`) `HPR_ib`, badly corrupted the
   result. Fixed by using `nav.Heading/Pitch/Roll` directly as `C_nb`
   with no further rotation; `gad.py`'s own use of `HPR_ib` for the GAD
   lever-arm/alignment is a genuinely different, IMU-frame-specific API
   and was unaffected. Confirmed fixed both by re-deriving the expected
   NED offset by hand from the same real numbers, and by the marker
   showing in the correct place afterward on the Map page.
2. **Map page scale bar didn't match its own gridlines** (found
   2026-07-19, field use: a known ~50cm marker separation didn't look
   like 50cm on-screen). The gridlines and the scale bar each called the
   "nice spacing" helper with a *different* target line count, so at a
   5m-wide view they showed 0.5m gridlines next to a scale bar labelled
   "1 m" — internally inconsistent, not just imprecise. Fixed by having
   the scale bar reuse the exact spacing value the gridlines were drawn
   with, rather than computing its own.
3. **A crashed/killed `aruco` process could delete `camera`'s live shared
   memory** (found 2026-07-19, after an abrupt `aruco` exit — `FATAL:
   exception not rethrown / Aborted` — left `camera` needing a restart
   with no obvious cause). `multiprocessing.shared_memory`'s
   `resource_tracker` tracks *any* `SharedMemory()` instantiation for
   cleanup-on-exit, even a read-only attach (`create=False`) — so a
   reader that exits abnormally can unlink a segment it never created,
   destroying a completely unrelated process's shared memory (a known
   upstream gotcha, see https://bugs.python.org/issue38119). Fixed in
   `shared/frame_ipc.py`'s `FrameReader` by calling
   `resource_tracker.unregister()` immediately after attaching, so a
   reader can never be responsible for unlinking something it doesn't
   own. Confirmed fixed by simulating the exact failure (attach, then
   `os._exit()` with no clean `close()`) against a throwaway
   shared-memory segment and checking it survived.

## Testing Decisions

- Unit-test the coordinate frame chain against hand-worked synthetic cases
  (marker dead ahead / zero mount angles / zero marker angles → known
  expected nav-frame vector; then repeated with non-zero marker heading, a
  non-zero mount roll, etc.) — this is the real safety net against a
  silent sign-flip or transpose error, independent of how carefully the
  code was first written.
- Synthetic ArUco markers rendered into fake frames (similar to the fake
  picamera2/checkerboard approach used for `camera`) for testing detection
  + pose end-to-end without real hardware.
- A fake/mock nav feed (matching `oxts-nav`'s `nav_feed` JSON shape) for
  testing GAD-trigger logic and marker-to-geodetic conversion without a
  live xNAV650.

## Out of Scope (v1 of this service)

- Automatic marker location surveying/triangulation (SLAM or otherwise) —
  markers remain hand-measured and hand-entered (via the Add Marker page),
  deliberately, given the complexity a real automatic system would need.
- Auto-promoting an "unmapped marker" estimate into the permanent map —
  even though the estimate is displayed, promotion stays a manual,
  deliberate Add Marker action.
- Camera-to-body extrinsic auto-calibration (hand-eye/bore-sight
  calibration) — extrinsics remain manually measured/configured constants;
  the camera is currently *not* properly bore-sighted at all (`HPR_cb =
  [0,0,0]`), which is flagged as the likely dominant error source, but
  fixing it is a separate, later piece of work.
- Roll/pitch GAD updates — deliberately excluded (see above), not
  deferred.
- Multi-position averaged marker surveying — plausible future enhancement,
  not built now.
- Multiple markers sharing one id, distinguished by separation distance —
  future idea, not needed while only one marker set is in use.
- Per-marker accuracy estimates (position/orientation uncertainty feeding
  into the GAD accuracy figure, instead of today's flat empirical formula)
  — future enhancement; the YAML map format is chosen partly so this can
  be added later as extra fields without breaking anything.
- CSV export of the marker map — easy to add later if hand-inspection in
  Excel is ever wanted again; not needed while the map file is written
  by the Add Marker page rather than hand-authored.
- Push-notify the Map page when the marker map file changes — refreshing
  the page is enough for how rarely markers are added; not worth the
  complexity yet.
- Verifying heading-GAD is actually being accepted/working via telemetry —
  blocked on OxTS fixing NCOM's GAD-status field, or on migrating to UCOM
  (a separate, later firmware-update project) — until then, be
  conservative about how much heading aiding is trusted.
- Real first-field-test result (2026-07-19, marker 10 surveyed via Add
  Marker): `GpsPosMode` changed to `GAD (34)` confirming position
  aiding is being accepted (note: it showing under *position* mode at
  all is itself an OxTS NCOM quirk, not something to fix here). No
  equivalent change was observed anywhere for heading — unclear whether
  there's a separate status field that should reflect accepted heading
  aiding (e.g. `GpsAttMode`) that we're not reading/showing yet, or
  whether this genuinely isn't visible via NCOM regardless (consistent
  with the point above). Investigate later, not blocking — noted here
  so it isn't lost.
