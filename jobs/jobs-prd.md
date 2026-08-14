# PRD: Job Sequencing Service (jobs)

See `top-prd.md` for where this fits: `drive` → `navigate` → **jobs**
(this document) → `missions` (sequences several jobs — see its own
`missions-prd.md`) → `safety` (obstacle-avoidance, future, blocked on
relocating the ultrasonics). Unlike the earlier services, `jobs` has
no equivalent at all in GR6-v1 — it's the first genuinely new capability
this project has needed, not a port of prior art.

## Problem Statement

`navigate` (done, field-proven) drives one pre-recorded path and stops.
In practice, watering the garden means several paths — one per bed/row
— run one after another, plus (eventually) actions in between. Standing
and watching the robot repeat a single path 10+ times per bed isn't
practical; the first real motivating case is exactly this: water one
line of plants, then the next, then the next, unattended.

Two problems specific to sequencing (not present when running one path
by hand):

1. **Path continuity.** Path B's start needs to be within `navigate`'s
   own entry tolerance (`entry_max_distance_m`/`entry_max_heading_deg`)
   of wherever path A actually left the robot — otherwise `navigate`
   will simply refuse to start B (correctly — see navigate-prd.md's
   "Path entry"). Nothing today checks this across a *sequence* of
   paths ahead of time.
2. **Observability over an unattended run.** A single path run is short
   enough to watch directly; a job (several paths back to back) is
   not. Debugging so far (see navigate-prd.md's "Real bugs found via
   testing") has relied on live telemetry captured while someone was
   watching — that doesn't work once nobody's watching.

## Solution

A new service, `jobs`, following this project's usual shape (Flask
+ shared header/template, `config.yaml`-driven, systemd unit, port
8010). It owns *sequencing and monitoring* only — it has no path-
following logic of its own and never talks to `drive` directly; it
drives `navigate` exactly the way a browser does today
(`/control/load/<name>`, `/control/start`, `/control/stop`,
`/control/entry-check`, `/ws/navigate`), the same "one process per
responsibility" boundary every other service already follows. `drive`
stays the only thing that talks to the motor controller; `navigate`
stays the only thing that owns the control loop.

### Why not a scripting language

Considered and rejected in favour of a small, fixed step vocabulary
interpreted by a plain Python state machine:

- **Arbitrary Python** (load and `exec` a user script): maximum
  flexibility, but no natural way to report "which step are we on" from
  arbitrary imperative code without building a state machine around it
  anyway — so you end up here regardless, with a much harder-to-debug
  and harder-to-visualise path to get there. Not worth it for one
  operator authoring jobs occasionally.
- **A Scratch-style visual language**: real UI effort to build from
  scratch with no existing tool to reuse — not worth it for this
  project's scale.
- **A custom text-based DSL/interpreter**: more expressive than a fixed
  step list, but that expressiveness isn't needed yet, and parsing/
  debugging a bespoke language is its own maintenance burden for no
  current benefit.

A job file is a YAML list of typed step dicts — structurally the
same idea as a path (a YAML list of typed point dicts interpreted by
`PathRunner`), reusing an established pattern rather than inventing a
new one. Adding a new capability later (waiting, pump-only steps,
replanning) means adding a new step *type* to the interpreter — a
deliberate, reviewable Python change — not something an open-ended
script could already attempt unpredictably.

### Job file format

```yaml
name: Front beds
steps:
  - type: run_path
    path: HouseFrontEight
  - type: pause
    duration_s: 30
  - type: water
    duration_s: 60
  - type: fill
    duration_s: 20
  - type: run_path
    path: HouseFrontSquiggle
```

Stored as `jobs/data/<name>.yaml`, same `paths.py`-style storage
(list/load/save/delete, same `_SAFE_NAME` filename validation) — a
`jobs/jobs.py` module mirroring `navigate/paths.py` rather than
a new storage convention.

v1 shipped with exactly one step type (`run_path`), matching the stated
first need ("Run path xxx" twice). Three more were added 2026-08-08 for
single-plant watering (go to a plant, water it stationary, come back for
more) - `pause` (wait, nothing else), `water` (the robot's own pump on
for `duration_s`, stationary - see "Timed steps" below), `fill`
(`waterbutt`'s valve open for `duration_s`). All three are deliberately
just a fixed duration, no conditions - see "Deferred: conditional
steps" below for why that's a bigger step than adding a step type.
Everything else discussed (waiting for a GNSS quality condition,
waiting for an aruco marker to come into view) is still deliberately
not built yet - paths already carry their own per-point pump state for
watering while moving, so only *stationary* watering needed a new step
to get the first watering sequence working.

### Path continuity

Checked two ways, both against `navigate`'s existing
`/control/entry-check` (never reimplemented — that endpoint already
does exactly this check):

- **At save time**: for every `run_path` step, entry-check *would* be
  run against wherever the *nearest preceding* `run_path` step's own
  last point/heading leaves the robot (computed the same way
  `navigate`'s own entry logic does, from the two paths' point data
  alone — no live robot needed) — skipping over any `pause`/`water`
  steps in between, since those don't move the robot, so continuity is
  really between the two nearest `run_path` steps, not literally
  adjacent ones. (Found live 2026-08-10 on "Water kitchen bed": a
  `pause` between two `run_path` steps let a genuine ~4.4m/143deg
  discontinuity through unwarned at save time, only discovered when the
  job aborted mid-run.) A failing pair is flagged to the operator as a
  warning when saving the job (not blocked outright — a route the robot
  doesn't naturally end facing correctly might still be intentionally
  fixed by the operator jogging it between paths, see below).
- **At run time**: `/control/load/<path>` then `/control/start` —
  `start` already runs the real entry-check internally against the
  robot's live position and returns `{"ok": false, "reason": ...}` on
  failure, so there's no need for a separate check-then-start
  round-trip (and no race between the two). A rejected start fails the
  step immediately with that same reason surfaced on the job page
  — **no automatic "bridge" path is generated to cover the gap**.
  That's deliberately left to a future path planner
  (`top-prd.md` item 4), not invented here as a stand-in; for now the
  fix is either editing the paths (the just-built Edit map page) so
  they line up, or the operator jogging the robot into position by hand
  and retrying the step (see "Resume" below) — the jog widget just
  added to `navigate`'s Run/Edit map pages exists for exactly this.

### Job execution (`JobRunner`, mirrors `navigate`'s `PathRunner`)

States: `idle` → `running` → `stopped_ok` (all steps completed) /
`aborted` (a step failed). One `current_step_index`, advanced only on a
step's success — same "forward-only tracking" shape `PathRunner` uses
for its own `tracked_index`.

Per `run_path` step: `/control/load/<path>` → `/control/start` → watch
`navigate`'s own `navigate_feed` Unix socket (a `FeedClient`, same
mechanism `wheelspeed` and `navigate` itself already use to read
`oxts-nav`'s/`drive`'s feeds — not a second outgoing websocket
connection from a Flask backend) until state leaves `running` →
`stopped_ok` advances to the next step, `aborted` stops the job and
records the reason against that step. A rejected load or start (e.g.
`navigate` already running something else — see its own "refuse while
running" guard, added 2026-07-31) fails the step the same way, logged
as `failed_to_load`/`failed_to_start` rather than `aborted`, since
nothing ever actually started running for that step.

**Stop** (operator-requested): calls `/control/stop` immediately and
marks the job `idle`, same distinction `PathRunner.stop()` already
draws between an operator stop and a real abort.

**Resume**: Start can be given a step index to begin from (not just
0) — after fixing whatever caused a step to fail (repositioning the
robot, editing a path), the operator can retry just that step rather
than rerunning the whole job from scratch. No automatic retry
policy in v1 (`on_fail` is effectively always "stop") — the operator
decides what "fixed" means before pressing Start again.

**`GET /control/status`** (added for `missions`, see its own PRD):
plain synchronous `runner.status()`, no push feed involved — `missions`
polls this directly while a `run_job` step is in progress.

**Timed steps** (`pause`/`water`/`fill`, added 2026-08-08): no external
state to poll like `run_path` has, so completion is tracked against an
injected clock (`now`, defaulting to `time.monotonic`, overridable in
tests so they don't sleep in real wall-clock time) instead — `tick()`
just checks whether `duration_s` has elapsed since the step started.
`water` calls navigate's `/pump/manual` (new — direct pump on/off
outside of any path-following, refused if a path is actually running,
since that path's own `step()` is already resending pump commands every
tick) at step start, **resends it on every tick** while waiting (not
just once — `drive`'s firmware watchdog turns the pump off after 2000ms
of silence, the same reason `navigate/control.py`'s own `step()`
resends every tick rather than only on change), then turns it off once
the duration elapses. `fill` calls `waterbutt`'s own `/go` directly (a
peer service, not something owned by another service the way `drive`
is) — its valve controller self-terminates at the same duration on its
own, so no explicit stop call is needed on the success path. An
operator **Stop** mid-`water`/mid-`fill` explicitly turns the pump off
/ calls `waterbutt`'s `/stop` — leaving hardware running unattended on
an operator-requested stop would defeat the whole point of a stop
button. `fill`'s duration options are drawn from `waterbutt`'s own
`DURATIONS_S` allow-list (`[1, 2, 5, 10, 20, 50, 120]`) rather than the
same fixed set offered for `pause`/`water` — a duration `waterbutt`
would just reject isn't offered as a choice in the first place.

### Deferred: conditional steps

The harder half of "water one plant" is the return trip: get to open
ground, wait for a GNSS quality condition (ideally `gxInteger`, rare
enough that getting it is "game over" — good enough to finish the
approach on GNSS alone), then switch to marker-guided approach for the
final stretch where GNSS is least reliable anyway (multipath near the
waterbutt/marker structure). That's real conditional logic — wait for a
live condition, then behave differently depending on the outcome — which
none of the four step types above have: each just runs and either
succeeds or fails the job, matching "no branching, no `on_fail:
retry`" from this doc's original Out of Scope list. Solving that isn't
"add another step type", it's the first step type whose *outcome*
branches, and deserves its own design pass (a `wait_for_condition` step?
per-step `on_timeout`?) rather than being squeezed in alongside the
three purely time-based ones above. Not built yet.

### Logging

Every job run writes a fresh JSONL log — `jobs/data/logs/
<job_name>_<yymmdd_hhmmss>.jsonl` — one line per step transition
(step index, path name, start/end time, outcome, abort reason if any),
plus periodic position snapshots while a step is running (same shape as
`navigate`'s own per-run debug log under `navigate/data/logs/`, reused
rather than reinvented — that log was itself later changed from a
single overwritten file to one retained file per run, 2026-08-12, same
convention this already used). Retained for a configurable number of days
(`log_retention_days`, default a couple of days per disk space being
cheap), swept on service startup. A log *viewer* page is out of scope
for this first version — raised in planning as a real future need
("develop a viewer for logged files") but not blocking jobs v1;
the raw JSONL is readable as-is in the meantime.

### Job page

One page, same shape as `navigate`'s Run page: a dropdown of saved
jobs, Start (optionally from a chosen step)/Stop, live state +
current step number/name via a websocket (`/ws/jobs`), and the
abort reason if stopped that way. A separate page lists saved jobs
(name, step count) — mirrors `navigate`'s Paths page.

Added 2026-08-09, so lining a job up no longer needs switching to
`navigate`: a jog joystick (identical `shared/web/static/jog.js`
widget as `navigate`'s own Run page), proxied through a new
`POST /jog/manual` here that just forwards to `navigate`'s own
`/jog/manual` rather than talking to `drive` directly (same boundary
as `pump_on()` above) — so `drive`'s control-arbiter manual/auto
lockout is enforced in exactly one place regardless of which page's
joystick sent the command. Disabled (dimmed, `pointer-events: none`)
whenever a step is actually running, same reasoning as `navigate`'s
own page. The preview map also plots a live trail of the robot's
actual driven position (not just the planned step paths) by
connecting the browser directly to `oxts-nav`'s feed
(`oxtsnav_ws_url`, same pattern as `navigate`'s own trail) — no proxy
needed since a WebSocket isn't subject to CORS the way a `fetch()`
is.

### Create/Edit job page

One page for both, not two — unlike `navigate`'s Create Path (live
recording by driving) vs. Edit map (hand-editing an existing file),
composing a new job and editing an existing one are the same
operation (an ordered step list), just starting empty vs. loaded.
Reached either via "Create Job" in the nav dropdown (no `name`
in the URL — empty step list) or via the Jobs page's "Edit"
button (`?name=<job>` — loads that job's steps). The nav
dropdown's own label is necessarily static ("Create Job" — same
as `create-path.html`'s dropdown entry staying "Create Path"
regardless of progress), so the page's own on-page heading is what
actually reflects "New job" vs. "Editing: `<name>`", not the
dropdown text.

Each step is one row: a dropdown of `navigate`'s saved paths (fetched
via a same-origin proxy, `GET /api/navigate-paths` → navigate's own
`/api/paths` — same CORS-avoidance reasoning as `navigate`'s own
`/jog/manual` proxy), up/down to reorder (no drag-and-drop, same
"skip the fancier interaction" call as the path editor's no-click-to-
select-on-map), and Remove. **Add path** appends a new `run_path` step
directly (same one-click append as before, just renamed once a second
kind of step existed to disambiguate from). **Add step** (2026-08-08)
opens a small modal instead — type (Pause/Water/Fill) plus a duration
dropdown scoped to that type (see "Job execution"'s "Timed steps")
— since unlike picking a path from a dropdown, a timed step needs two
choices made before it means anything, so appending one blank and
editing in place (the run_path pattern) wouldn't leave a step in any
sensible default state.

A step's colour swatch and the preview map below only ever draw
`run_path` steps (a `pause`/`water`/`fill` step has no points to plot) —
the map layer computation already skips any step without a `.path`, so
this fell out for free rather than needing special-casing.

Save uses the same dialog shape as the path editor (filename,
live overwrite detection, Save-and-continue/Save-and-exit) and
surfaces any continuity warnings the save endpoint returns. No undo —
unlike the path editor's N/S/E/W nudges (easy to mis-click, expensive
to redo by eye), a wrong dropdown pick or reorder here is a single
obvious click to fix, so the added complexity wasn't judged worth it
for v1.

### Config additions

```yaml
jobs:
  unit: robot-jobs.service
  host: 0.0.0.0
  port: 8010
  web_ui: true
  jobs_dir: jobs/data
  job_status_hz: 2        # poll rate against navigate's /ws/navigate while a step runs
  log_retention_days: 2
```

### Run page staying in sync with the job actually running (added 2026-08-14)

Same gap `navigate`'s own Run page had, for the same reason: a mission
driving this job's `/control/start` directly loads it without this
page's own dropdown ever being touched, so the dropdown/map preview
could show something stale next to a status table correctly reporting
what's actually running. `JobRunner.status()` already published
`job_name`; the fix is entirely client-side — `run.html`'s
`/ws/jobs` handler now tracks whichever job name it last displayed and,
when the feed reports a different one, updates the dropdown and
re-fetches the step/map preview to match, same pattern as `navigate`'s
own fix (see `navigate-prd.md`'s "Path entry").

## Out of Scope (v1 of this service)

- Any step type beyond `run_path`/`pause`/`water`/`fill` (turn-in-place,
  a `wait_for_condition` step) — see "Deferred: conditional steps" above
  for the marker-approach case that actually needs one.
- Configurable durations for `pause`/`water`/`fill` beyond a fixed
  dropdown of choices — free-text would be easy to add later if the
  fixed set (5/10/20/60s, or `waterbutt`'s own allow-list for `fill`)
  turns out not to be enough.
- Automatic "bridge path" generation to cover a continuity gap — that's
  the future path planner's job (`top-prd.md` item 4), not a stand-in
  built here.
- Retry/replan policy beyond "stop and let the operator decide" — no
  `on_fail: retry`, no conditionals, no branching.
- Reading `safety`'s output to decide whether to stop — `safety`
  doesn't exist yet; `jobs` currently only reacts to `navigate`'s
  own abort reasons (cross-track/heading/accuracy), same as an operator
  watching the Run page would.
- A log viewer page — logs are written in a form a future viewer can
  read, but no viewer is built now.
- Job *authoring* UI (a drag-and-reorder step list, etc.) — v1
  jobs are hand-written YAML, same starting point paths themselves
  had before the Edit map page existed.
