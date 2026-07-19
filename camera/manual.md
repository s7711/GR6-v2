# Camera Service — User Manual

This is a how-to-use guide for the `camera` service's web pages. For the
design/engineering background (why things work the way they do), see
`camera-prd.md` instead — this document is deliberately just "how do I
use it."

Open the camera service from the manager's home page, or go directly to
its address (port 8003 by default) — e.g. `http://amundsen:8003`.

## Home page

The live camera preview, plus four readouts:

- **Frame rate (target)** — how fast the camera is capturing, set in the
  shared config file (not from this page).
- **Exposure** and **Gain** — update live. Useful as a rough lighting
  check: if Gain is very high, the scene is dark and the image will look
  noisy/grainy; if Exposure is very high, fast motion may blur.
- **Frame** — a running frame counter, mostly useful to confirm the
  camera is actually capturing (it should keep climbing).

## Config page

Read-only. Shows the current resolution and frame rate, and explains
why resolution can't be changed here (it's fixed, because the
calibration is tied to a specific resolution — recalibrating is needed
if that ever changes).

## Calibrate page

This is where you (re-)calibrate the camera. You'll want to do this
once when the camera is first set up, and again any time the camera or
lens is physically moved, refocused, or replaced.

### What you need

A printed checkerboard: **5×7 squares, 30mm per square**, mounted flat
and rigid (a wobbly printout won't hold still enough). Print it as flat
as you can and check the square size with a ruler — if your printer
scaled the page, the calibration will be subtly wrong.

**Good, even light.** Check the **Exposure** reading on the Home page
before you start — it tells you how much light the camera thinks it
needs, and lower is better here. As a rule of thumb, aim for **under
10000µs**; well above that means the room's too dark and the images are
more likely to be blurry or noisy, which makes detection harder and the
calibration less accurate.

### Running a calibration

1. Click **Start**. The preview switches to showing a yellow target box
   overlaid on the live image — this is where the checkerboard needs to
   go next.
2. Hold the checkerboard so it fills the yellow box as closely as you
   can, facing the camera squarely (or tilted, if the box itself looks
   tilted — some targets deliberately ask for an angled board). Try to
   hold it still; a moving board is harder to detect well.
3. Watch the **Message** line and the **Fit score** / **Alignment
   error** readouts just below it:
   - **Fit score** — how close to the right *size/distance* you are.
     Higher is better; needs to reach about 0.80 to count.
   - **Alignment error** — how close to the right *position and tilt*
     you are. Lower is better; needs to drop to about 0.50 or under.
   - The message tells you what's currently wrong: **"No checkerboard"**
     (not detected at all — check lighting/focus/framing), **"Align
     checkerboard"** (detected, but position/tilt isn't close enough
     yet), **"Too small (...)"** (detected and positioned, but not
     filling enough of the frame — move closer), or **"Ok"** (accepted
     — it'll automatically move on to the next of the ~33 target poses).
4. Repeat for each target pose shown in "Image N of 33". This takes a
   few minutes — there's no need to rush between poses, the software
   just waits until each one is satisfied.
5. Once all poses are captured, the calibration is computed
   automatically (a few seconds) and the **Result** panel appears on its
   own — you don't need to click anything to see it. It shows:
   - The **RMS reprojection error**, in pixels — lower is better, and
     **under 0.5px counts as a good result**. The badge colour gives a
     rough at-a-glance read too: green is good, yellow is borderline,
     red is suspect.
   - The resolution, and how many images were actually used.
6. If you're happy with the result, click **"Make this the active
   calibration"**. This is what actually puts it to use: it becomes the
   **one shared calibration for the whole camera service** (and anything
   else that reads it later, like the future ArUco marker detection),
   not just something kept with this one session. Computing a
   calibration doesn't do this automatically — it's a separate,
   deliberate step, so a bad session can't silently replace a good one
   that's already active.

### Abort

If you need to stop partway through (wrong checkerboard, need a break,
anything), click **Abort**. Nothing is lost — even an aborted attempt
keeps its captured data around (see "Where your data goes" below), in
case it's useful to look back on.

### If it won't trigger "Ok"

- Check the **Fit score** and **Alignment error** numbers rather than
  guessing — they tell you which is off (too far/too small vs.
  wrong position or tilt), which is more useful than just seeing the
  message repeat.
- More light generally helps detection — check the Exposure reading on
  the Home page; if it's well above 10000µs, try brightening the room.
- A slightly bigger/closer board (raising Fit score) is usually easier
  to fix than getting the exact tilt right — try that first.
- If a specific pose consistently won't cooperate even though it looks
  right to you, that's worth reporting — every evaluated frame is
  logged (see below) with enough detail to diagnose it properly rather
  than guessing again.

### Where your data goes

Every calibration attempt (finished, or aborted) gets its own folder
under `camera/data/`, named by date and time. Inside:

- `imageN.jpg` — each accepted capture.
- `calibration.yaml` — the computed result (only present if the session
  finished).
- `capture_log.csv` — a row for every frame evaluated during capture
  (not just accepted ones), including the fit/alignment numbers and the
  raw detected checkerboard points. Kept permanently, even for aborted
  sessions — it's small, and useful for diagnosing anything that didn't
  go as expected.

These folders are never automatically deleted, so they'll accumulate
over time — that's fine, just something to be aware of if disk space
ever becomes a concern.
