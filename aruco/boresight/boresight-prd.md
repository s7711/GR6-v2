# Camera bore-sight calibration (`hpr_cb`)

Estimate the camera's mounting rotation relative to the vehicle body —
`services.aruco.camera_extrinsics.hpr_cb` — from marker observations
taken while driving a known area, and report it with an honest
uncertainty.

`hpr_cb` has never been measured. It is `[0, 0, 0]` in the live config,
an assumption, and aruco-prd.md has long flagged it as the likely
dominant error source in every marker measurement and GAD update. It
also gates the visual-mapping work that comes next.

Target accuracy: **0.1 degrees** (Ben, 2026-09-14) — about 5cm of
cross-range error at 3m.

## Status (2026-09-16)

All built and tested — 154 tests. Forward model (`model.py`), solver
(`solve.py`), simulator (`sim.py`, `layouts.py`, `evaluate.py`),
placement (`placement.py`), capture (`capture.py`), driving (`drive.py`),
service glue (`service.py`) and the page
(`aruco/templates/pages/boresight.html`).

Exercised end to end against the live robot. One real run happened and
was aborted for GNSS interference; see "First real run" below. Not yet
committed to git.

## How it works

A bundle adjustment on **corner reprojection error**. Unknowns are
`hpr_cb` (3) plus a free 6-DOF pose per marker; each vehicle pose comes
from the INS and is treated as the reference.

Two decisions worth stating plainly:

**Pixels, not rvec/tvec.** The bearing to a detected corner is good to
~0.01 deg, whereas a small planar marker's out-of-plane rotation is the
classic ill-conditioned case and can be degrees out. Reprojection error
weights those correctly and automatically; hand-picked residual weights
on rvec/tvec would not.

**Marker poses free, not surveyed.** Ben can only plant a marker
approximately and can't measure inter-marker spacing to better than a
centimetre or two. Estimating the poses costs 6 parameters each and buys
total independence from planting accuracy. The markers get surveyed as a
by-product, far better than by hand — and their solved attitudes are then
an independent cross-check against a spirit-level reading.

## Observability

- **Heading** is exactly degenerate with the markers' own headings if you
  use rotations alone. It is broken by *position* consistency: a camera
  yaw error `psi` puts the estimated marker ~`r.psi` sideways in the body
  frame, pointing a different way in the nav frame for every vehicle
  heading. Azimuth spread per marker is what buys it.
- **Roll** is near-degenerate with a marker's own roll while that marker
  sits at the image centre. Camera roll displaces an image point at
  radius `rho` by `rho.phi`, so it needs observations well off-centre —
  which the path provides by driving *past* a marker, not only at it.
- **Pitch** trades against marker height as `r.pitch`, so range diversity
  separates it.

A constant INS heading bias is ~98.7% absorbed into `hpr_cb` heading
rather than measured. That is the right outcome, not a flaw: aruco's GAD
updates want the camera consistent with the frame the xNAV650 reports,
not with true north. The 1.3% that isn't absorbed comes from the 0.105m
**lever arm** — a heading bias swings the camera *position*, and no
rotation reproduces a translation. Vehicle tilt, the intuitive candidate,
turns out not to matter measurably. All three claims are pinned by tests
in `test_solve.py` rather than asserted here.

## Simulator results (2026-09-14)

Monte Carlo over the recorded "Boresight limits" box, with the INS noise
model calibrated from the 2026-09-14 mission logs (RTK fixed, NorthAcc
median 0.009m, HeadingAcc 0.18 deg moving). Errors are RMS degrees
relative to truth-plus-INS-bias, i.e. the part the setup controls.

**Sampling noise is ~14% at 25 trials — differences smaller than that
are not real.** Two rows below violate quadrature slightly for exactly
this reason; they are noise, not physics.

Where the error comes from (3 staggered markers, fan+past path):

| source | H | P | R |
|---|---|---|---|
| camera corner noise only (0.2px) | 0.000 | 0.001 | 0.001 |
| INS position only (0.01m RTK) | 0.035 | 0.042 | 0.043 |
| INS heading only (0.18 deg, tau=30s) | 0.080 | 0.019 | 0.029 |
| ... the same, were it white | 0.021 | 0.005 | 0.007 |
| INS pitch/roll only (0.04 deg) | 0.015 | 0.024 | 0.033 |
| everything together | 0.075 | 0.056 | 0.065 |

The camera contributes **nothing**. Accuracy is set entirely by the INS,
and specifically by how *correlated* its heading error is — the white-noise
row is 4x better than the correlated one. That makes session length, not
hardware, the lever that matters.

Marker arrangement, path, and session length:

| variation | H | P | R |
|---|---|---|---|
| 1 marker | 0.105 | 0.059 | 0.107 |
| 3 stacked vertically | 0.109 | 0.071 | 0.135 |
| 3 in a horizontal row, one height | 0.092 | 0.058 | 0.054 |
| 3 in a row, staggered heights | 0.075 | 0.056 | 0.065 |
| fan only (drive straight at it) | 0.130 | 0.063 | **0.383** |
| past only (drive past it) | 0.330 | 0.167 | 0.176 |
| fan + past | 0.075 | 0.056 | 0.065 |
| 5 min driving | 0.075 | 0.056 | 0.065 |
| 20 min driving | 0.051 | 0.027 | 0.025 |
| 41 min driving | 0.051 | 0.027 | 0.015 |
| 150mm / 200mm markers | no improvement (within noise) | | |
| 6 markers, two panels back-to-back | no improvement (within noise) | | |

Conclusions: **the path matters more than the markers** (6x on roll);
**97mm markers are fine** and bigger ones buy nothing; **three markers
are enough** and a second panel is wasted; **~20 minutes** of driving is
the sweet spot, with little gained after that.

Note what is NOT a conclusion: the horizontal row beats one marker and
beats a vertical stack (clearly, on roll), but staggered-versus-flat
heights is *inside* the sampling noise. It was briefly written up here as
a win; it isn't one.

### Marker height, with the real camera height (0.133m, CAD)

| heights above ground | obs | range seen | H | P | R |
|---|---|---|---|---|---|
| flat 0.15 (level with camera) | 1211 | 1.0-3.8m | 0.088 | 0.049 | 0.055 |
| flat 0.35 | 1211 | 1.0-3.8m | 0.100 | 0.044 | 0.055 |
| stagger 0.15/0.25/0.35 | 1211 | 1.0-3.8m | 0.137 | 0.036 | 0.046 |
| stagger 0.15/0.35/0.55 | 1203 | 1.0-3.9m | 0.082 | 0.055 | 0.059 |
| stagger 0.21/0.51/0.81 | 1130 | 1.0-3.9m | 0.075 | 0.056 | 0.065 |

**Height does not materially matter** anywhere in 0.15-0.8m: the whole
spread is sampling noise (the 0.137 row is ~2 sigma and has no physical
story — a *tighter* stagger cannot plausibly be worse than flat, which
scores 0.088). Observation counts barely move either, so the vertical-FOV
worry turns out not to bind at these heights.

### Line vs triangle (50 trials, ~10% sampling noise)

Ben's question: a triangle is "more 2D" than a line, so oughtn't it beat
a row?

| layout | obs | H | P | R |
|---|---|---|---|---|
| flat row, all 0.20 | 1201 | 0.094 | 0.057 | 0.049 |
| row, stagger 0.20/0.40/0.60 | 1188 | 0.112 | 0.053 | 0.049 |
| triangle, base 1.0m apex 0.60 | 1202 | 0.109 | 0.061 | 0.045 |
| triangle, base 1.0m apex 0.85 | 1124 | 0.089 | 0.050 | 0.044 |

**No.** The entire spread is inside the sampling noise; roll — what a
triangle would most plausibly help — is 0.044-0.049 across all four, a
10% spread against 10% noise. Ben's own reasoning for why is the right
one: the robot's motion already sweeps the markers across the image in
two dimensions (horizontally while driving past, vertically as elevation
changes with range), so instantaneous 2D spread adds nothing the path has
not already provided. This is the same reason the path is worth 6x on
roll while every marker-geometry variation keeps coming out as noise.

**Build a flat row.** It is much the easiest, and nothing measurable is
given up.

## Recommended physical setup

One panel of **three 97mm markers in a flat horizontal row, all at
0.25m**, at the centre of the box, facing along its long axis.

Local NED metres about the box centroid (52.235464288, -1.460508331):

| id | north | east | height above ground | lat | lon |
|---|---|---|---|---|---|
| 20 | -0.478 | +0.146 | 0.25m | 52.235459991 | -1.460506191 |
| 21 | 0.000 | 0.000 | 0.25m | 52.235464288 | -1.460508331 |
| 22 | +0.478 | -0.146 | 0.25m | 52.235468585 | -1.460510471 |

Heights are to the **marker centre**. Ben's markers carry a 5cm border
all round, so the printed sheet is ~197mm: a centre at 0.25m puts the
black square's bottom edge at 0.20m and the white quiet zone's at 0.15m,
leaving real clearance for grass. (0.20m centres would put the quiet zone
at 0.10m, which is tighter than is comfortable in a lawn.) At 0.5m
spacing the 197mm sheets leave ~300mm gaps, so the row is buildable as
one rail.

**`size` is the black square, not the sheet**: 0.097, not 0.197. That is
what `estimatePoseSingleMarkers`' markerLength means, and it's the number
in marker-map.yaml's `size:` field. Getting it wrong scales every range
and the solve still converges — quietly, onto wrong marker positions and
a biased hpr_cb. The placement UI must say this explicitly. Print
scaling is handled separately, by carrying a global marker-size scale
factor as a free parameter in the solve (one extra unknown, well
observed given RTK-known vehicle positions, and a useful diagnostic in
its own right — a solved scale of 1.03 means the print is 3% large).

The faces look along bearing **253 deg**; the row runs along bearing
163/343 deg, roughly 0.5m between markers. On one pass of the pattern
each marker is seen in ~400 of 1526 frames, over a 1.0-3.9m range spread.

**Heights are above ground, and that distinction has already bitten
once.** This module's local NED has down=0 at the INS reference point's
plane, which is only 0.063m above the ground (0.133m camera, CAD, minus
the 0.07m d_xc_b lever arm). An unused `CAMERA_HEIGHT_M = 0.30` constant
in sim.py — read by nothing, guessed before Ben supplied the CAD figure —
made an earlier version of this table report heights 63mm low. The
conversion now lives in exactly one place (layouts.py's `height_to_down`)
and is pinned by a test that requires a marker level with the camera to
project onto the principal point's row.

Ids 20/21/22 are deliberately clear of the deployed markers
(10/11/12/17/19): a mapped id would make aruco send GAD from it,
steering the nav solution with the very `hpr_cb` being calibrated. The
bore-sight markers must therefore **never** be added to
`aruco/data/marker-map.yaml`. They are temporary and get removed after
the session — worth noting since the waterbutt return path runs through
this box.

The capture pattern is ~26 legs, half driven straight at the panel (the
"fan", for azimuth spread) and half swinging past it (for image-edge
content, and so roll). The past-legs are generated in mirrored pairs on
purpose: distortion-model error at the image edge also looks like roll,
and only cancels if the edge is visited symmetrically.

## Capture: move, but interpolate (decided 2026-09-16)

Capture-timestamp latency — the lag between the shutter opening and the
nav sample paired with the frame — is **systematic**, so it biases
`hpr_cb` rather than averaging out. Measured bias, 40 trials:

| lag | H error | H **bias** |
|---|---|---|
| 0 ms | 0.111 | +0.003 |
| 25 ms | 0.179 | +0.139 |
| 50 ms | 0.306 | +0.278 |
| 100 ms | 0.606 | +0.577 |

Linear at roughly **5.6 deg per second of lag**, and the physics agrees:
0.2 m/s x 50ms = 10mm along-track, over a ~2.5m range, is 0.23 deg.

Two things make this manageable rather than fatal:

- **camera/bg_camera.py already stamps frames with `SensorTimestamp`**
  (CLOCK_MONOTONIC nanoseconds, the sensor's own exposure clock), so the
  frame side carries no pipeline lag at all.
- **The nav payload carries `GpsSeconds`/`GpsTime` per sample** and
  updates every ~52ms, and `connection.timeOffset` +
  `ncomrx.machine_time_to_gps` puts the frame's SensorTimestamp on that
  same GPS timescale.

So the lag is entirely an artefact of pairing a frame with
`nav_client.latest()`, which gives a 0-52ms sawtooth averaging ~25ms —
0.139 deg of bias, over budget on its own. **The capture must buffer nav
samples and interpolate to each frame's own GPS time.** Residual is then
only the exposure-centre convention (a few ms, <0.03 deg), and a free
latency parameter in the solve can absorb even that.

Stationary capture is genuinely unbiased (bias +0.035 deg) but too noisy
alone — 405 observations and much less attitude diversity give H 0.339,
P 0.375. It earns a place as an **anchor**: a handful of deliberate stops
inside a mostly-moving session, to cross-check the moving solution.

### A claim that did not survive testing

`past_legs` generates mirrored pairs, with a comment that image-edge
distortion error "only cancels if the edge is visited symmetrically".
That was written about *distortion*, and was then cited in conversation
as if it also handled latency. It does not: mirroring the fan made the
latency bias worse, not better (-0.170 deg one-directional vs +0.326 deg
mirrored, at 50ms). The distortion claim itself remains **untested** — a
reason to keep the symmetry, not a result to rely on.

## Marker size as a free parameter (2026-09-16)

The solve carries an optional global multiplier on the assumed marker
size (`estimate_size_scale`). Recovery under the full noise model:

| session | solved size | true |
|---|---|---|
| 1 pass (~5 min) | 100.3 +/- 0.2 mm | 100.2 |
| 4 passes (~20 min) | 100.2 +/- 0.1 mm | 100.2 |

`hpr_cb` comes out identical with and without it, so the extra unknown
costs nothing. This agrees with a completely independent single-image
measurement against marker 12 (100.2mm, see below), so two routes concur.

**A wrong marker size does NOT bias this solve** — 3.3% error, held
fixed, moves hpr_cb by 0.0035 deg and the solved marker positions by 0mm.
An earlier version of this document and of solve.py's docstring claimed
the opposite. The reason it doesn't: hpr_cb is fixed by the *bearings* to
the corners while size only affects *range*, and the two are orthogonal;
with many views the bearings pin the positions hard enough that the size
error goes into the residual instead of into any parameter. It shows as
~0.8px of extra residual on clean data, which is invisible against the
~6px an INS-realistic run produces — so elevated residuals are not a
practical detector. Estimating the scale is.

Consequence: the size fix and the bore-sight are **independent**, and can
be done in either order.

### The live GAD path is not immune

aruco's GAD updates use a single-shot `tvec` whose magnitude scales
directly with the assumed size — no multi-view averaging to protect it.
Measured live 2026-09-16 against marker 12, RTK fixed:

    true range (INS + surveyed position):  1.0883 m
    range from ArUco assuming 0.097:       1.0534 m   => 100.2 mm

Camera-to-marker *distance* is independent of hpr_cb (a rotation cannot
change a distance), so this measurement is valid despite the bore-sight
being uncalibrated. config.yaml/marker-map.yaml say 0.097; the markers
are 0.100. Every GAD range is therefore 3.3% short, placing the robot
nearer the marker than it is — which is the "pulls the navigation
towards the marker" behaviour seen at the waterbutt with GNSS removed.

Not yet changed: it shifts every GAD range on a working system, so it is
Ben's call. Markers 10/11/12/17/19 were surveyed under the wrong size and
are themselves ~3% close, so they want re-surveying afterwards.

## Built (2026-09-16)

Page at `/pages/boresight`, three sections; everything behind the routes
is in `service.py`.

1. **Plan** — validates the boundary path is a box (and says WHICH corner
   or side is wrong if not), then shows each marker target with live
   range/bearing from the robot, plus, while a marker is visible, how to
   NUDGE it (`placement.move_guidance`).
2. **Drive & capture** — one button: builds the job, drives it, records.
3. **Solve** — hpr_cb with two uncertainties, implied marker size, and a
   config writer.

### Placement: panel at one end, not the centre

You cannot see a marker from behind it, so a panel at the box centroid
wastes half the area. Backed up to 90% of the way to the rear edge
(Ben, 2026-09-16): 11.15m of usable ground ahead instead of 5.87, 0.59m
wasted behind. The gain isn't range — ArUco's ~4m limit binds long before
the boundary does — it's turning room.

Facing comes from `placement.marker_targets()`, NEVER from layouts.py's
study constant: they pick opposite ends of the same axis on this box.
The two facing directions are also a near-tie (5.87m vs 5.86m clearance),
so the choice is deliberately deterministic — a flip between runs, after
the markers are planted, would send the robot round the wrong end.

### Nudge guidance

"Range and bearing from the robot" finds the spot, but can't be acted on
once a marker exists — you can't park the robot where the marker must go
(Ben). So while a marker is visible, the page also shows the correction
in the ROBOT's frame: towards/away from the robot, to its left/right, and
a rotation. Those two distances are the robot's own forward and right
axes, hence orthogonal, so they can be applied one after the other
without interacting. Poses come from surveying each detection with the
current, uncalibrated hpr_cb — irrelevant at tens-of-centimetres
tolerance, and every marker pose is a free parameter in the solve anyway.

### Driving: a JOB of straight legs and turns on the spot

The pattern is driven as `run_path` / `turn_to_heading` steps, not as one
path. This is the only structure that works, and the failures are worth
recording:

- Alternating far-ring/near-ring waypoints reverses radial direction at
  every vertex, so the turn is ~180 minus the change in bearing. With a
  110-degree sector the best possible is 70 degrees, and a constraint
  search found NO ordering covering all seven bearings under even 110.
- Rounding didn't rescue it: a 0.5m fillet on a 154-degree turn needs
  2.2m of tangent per leg; the legs are shorter, so arcs collapsed and
  ate 45% of every leg (path fell 40m -> 12m).
- A figure-8 is smooth and balanced but scored 12-39 marker observations
  against the weave's 780 — a forward-facing camera on a circle spends
  most of its time looking away from the middle.
- Turning on the spot between straight legs removes the radius
  constraint entirely (Ben, 2026-09-16) and lets the geometry the
  simulator wants be driven as-is.

**Turns must be forced left and right.** The xNAV650 does better when
turns aren't all one way (it's why the warm-up paths are figure-8s), and
`turn_to_heading` is explicitly shortest-way (turn_control.py) — so it
will NOT alternate for you. A turn wanting the long way round is split
into three intermediate headings, each itself shortest-way in the
direction wanted. Current pattern: 13 legs, 40.4m, 3.4 min/pass, turns
forced 6 left / 6 right.

jobs' continuity check warns on all twelve joins, because it skips over
intervening steps when comparing consecutive run_paths — right for
pause/water, wrong for turn_to_heading. Those warnings are filtered; a
genuine positional gap still shows, since a turn fixes heading not
distance.

### Capture: what the live tests found

Three bugs that would only have surfaced after a full session, all found
by a dry run against live data before the markers went in:

1. **`GpsSeconds` is seconds within the current MINUTE** (13.29, not
   311840). Keying the nav buffer on it rejected 100 frames of 100.
   ncomrx defines `timeOffset = GpsSeconds + GpsMinutes*60 - machineTime`
   (ncomrx.py:317), so inverting it gives each sample its machine time —
   the same clock the camera's SensorTimestamp uses. Both sides were
   already on Python's clock; nothing needed re-deriving (Ben).
2. **Quality fields are in `status`, not `nav`.** Reading them from nav
   gave None and the gate rejected all 91 time-matched frames as "not RTK
   fixed (GnssPosMode None)". The decoded LOG flattens nav and status
   together, which is what misleads.
3. **`HorizontalSpeed` doesn't exist in the live feed** — it is derived
   as hypot(Vn, Ve) only when writing the log.

And one throughput fix: nav samples arrive ~0.35s behind real time while
a frame's SensorTimestamp is ~0.05-0.2s old, so fresh frames were NEWER
than any nav sample and couldn't be bracketed — 47 of 113 frames lost.
Detections now wait in a small queue until nav catches up: 99% retention.

Capture is rate-limited to config's `max_detection_hz` (2Hz), the same
cap aruco's own loop uses. Uncapped it ran at the camera's 5Hz alongside
that loop (~7Hz of ArUco detection on a Pi) and bought nothing, since
INS error is correlated over tens of seconds.

### The solve refuses degenerate data

A stationary capture — 318 detections of all three markers, residual
**0.42px**, `success: True` — solved to a marker size of **11.25 metres**
with markers 130m away and hpr_cb at [76, 0.8, 75]. Nothing in the fit
gave it away: from one viewpoint the model trades marker distance against
marker size almost freely, fitting the pixels beautifully while meaning
nothing. `solve.geometry_check()` now refuses unless there is real
viewpoint diversity (1.5m position spread, 25 deg heading spread,
every marker seen). Heading spread is measured circularly — +179 and -179
are 2 degrees apart, not 358.

## First real run (2026-09-16, aborted for interference)

118 observations over 9 minutes, markers still lying at ground level.
Not applied, but informative:

    hpr_cb  [0.84, 1.33, -0.93] deg   split scatter [0.30, 0.016, 0.15]
    rms 17.0 px (expected ~6), max 81 px
    implied marker size 94.1 mm

- Solved marker row spacing came out **0.495m and 0.510m** against the
  0.5m set by hand — nothing told the solve that, so the geometry is
  being recovered correctly.
- Dropping the worst 30% of residuals barely moves it
  ([0.859, 1.235, -0.934]), so it isn't chasing outliers.
- One 60s window has median residual 49px against 5-16px elsewhere — a
  localised nav failure, matching the interference Ben saw.
- **The size scale disagrees with the single-image measurement** (94-95mm
  vs 100.2mm). Which is right is unknown: the single-image figure leans
  on marker 12's surveyed position, itself surveyed with the wrong size
  AND an uncalibrated hpr_cb. The free size parameter is doing exactly
  its job — failing loudly when something systematic is off.

Lesson worth keeping: **reported accuracy stays optimistic during
interference** (every surviving sample claimed 0.16-0.19 deg heading
accuracy and 7-19mm position). The quality gates did not catch the bad
window; the residual did.

## Still to do

- **After calibration**: markers 10/11/12/17/19 were surveyed under
  `hpr_cb = [0,0,0]` and become stale. 12 especially — waterbutt's QC
  depends on it.
- **The 0.097 -> 0.100 marker size fix** in marker-map.yaml and
  detection.py — Ben's call, and the two size estimates should agree
  first.
- Nothing in `aruco/boresight/` is committed yet.

## Out of scope

- Estimating `d_xc_b`. It's CAD-derived and good to ~5mm (Ben). Holding
  it fixed and reporting what a free solve would say is a diagnostic, not
  a correction.
- Estimating the camera intrinsics. That's camera/calibration.py's job
  and a separate, already-solved problem.
- Making the markers plumb or measuring their angles as an *input*. The
  solve beats a 0.5 deg spirit-level reading comfortably; the measurement
  is useful only as an independent check.
