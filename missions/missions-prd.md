# PRD: Mission Sequencing Service (missions)

See `top-prd.md` for where this fits: `drive` → `navigate` → **missions**
(this document) → `safety` (obstacle-avoidance, future, blocked on
relocating the ultrasonics). Unlike the earlier services, `missions` has
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
   enough to watch directly; a mission (several paths back to back) is
   not. Debugging so far (see navigate-prd.md's "Real bugs found via
   testing") has relied on live telemetry captured while someone was
   watching — that doesn't work once nobody's watching.

## Solution

A new service, `missions`, following this project's usual shape (Flask
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
  operator authoring missions occasionally.
- **A Scratch-style visual language**: real UI effort to build from
  scratch with no existing tool to reuse — not worth it for this
  project's scale.
- **A custom text-based DSL/interpreter**: more expressive than a fixed
  step list, but that expressiveness isn't needed yet, and parsing/
  debugging a bespoke language is its own maintenance burden for no
  current benefit.

A mission file is a YAML list of typed step dicts — structurally the
same idea as a path (a YAML list of typed point dicts interpreted by
`PathRunner`), reusing an established pattern rather than inventing a
new one. Adding a new capability later (waiting, pump-only steps,
replanning) means adding a new step *type* to the interpreter — a
deliberate, reviewable Python change — not something an open-ended
script could already attempt unpredictably.

### Mission file format

```yaml
name: Front beds
steps:
  - type: run_path
    path: HouseFrontEight
  - type: run_path
    path: HouseFrontSquiggle
```

Stored as `missions/data/<name>.yaml`, same `paths.py`-style storage
(list/load/save/delete, same `_SAFE_NAME` filename validation) — a
`missions/missions.py` module mirroring `navigate/paths.py` rather than
a new storage convention.

v1 has exactly one step type, matching the stated first need ("Run path
xxx" twice). Everything else discussed (waiting near a marker, pump-
only steps) is deliberately not built yet — paths already carry their
own per-point pump state, so a mission doesn't need its own pump step
to get the first watering sequence working.

### Path continuity

Checked two ways, both against `navigate`'s existing
`/control/entry-check` (never reimplemented — that endpoint already
does exactly this check):

- **At save time**: for every step after the first, entry-check *would*
  be run against wherever the previous path's own last point/heading
  leaves the robot (computed the same way `navigate`'s own entry logic
  does, from the two paths' point data alone — no live robot needed).
  A failing pair is flagged to the operator as a warning when saving the
  mission (not blocked outright — a route the robot doesn't naturally
  end facing correctly might still be intentionally fixed by the
  operator jogging it between paths, see below).
- **At run time**: `/control/load/<path>` then `/control/start` —
  `start` already runs the real entry-check internally against the
  robot's live position and returns `{"ok": false, "reason": ...}` on
  failure, so there's no need for a separate check-then-start
  round-trip (and no race between the two). A rejected start fails the
  step immediately with that same reason surfaced on the mission page
  — **no automatic "bridge" path is generated to cover the gap**.
  That's deliberately left to a future path planner
  (`top-prd.md` item 4), not invented here as a stand-in; for now the
  fix is either editing the paths (the just-built Edit map page) so
  they line up, or the operator jogging the robot into position by hand
  and retrying the step (see "Resume" below) — the jog widget just
  added to `navigate`'s Run/Edit map pages exists for exactly this.

### Mission execution (`MissionRunner`, mirrors `navigate`'s `PathRunner`)

States: `idle` → `running` → `stopped_ok` (all steps completed) /
`aborted` (a step failed). One `current_step_index`, advanced only on a
step's success — same "forward-only tracking" shape `PathRunner` uses
for its own `tracked_index`.

Per `run_path` step: `/control/load/<path>` → `/control/start` → watch
`navigate`'s own `navigate_feed` Unix socket (a `FeedClient`, same
mechanism `wheelspeed` and `navigate` itself already use to read
`oxts-nav`'s/`drive`'s feeds — not a second outgoing websocket
connection from a Flask backend) until state leaves `running` →
`stopped_ok` advances to the next step, `aborted` (or a failed start)
stops the mission and records the reason against that step.

**Stop** (operator-requested): calls `/control/stop` immediately and
marks the mission `idle`, same distinction `PathRunner.stop()` already
draws between an operator stop and a real abort.

**Resume**: Start can be given a step index to begin from (not just
0) — after fixing whatever caused a step to fail (repositioning the
robot, editing a path), the operator can retry just that step rather
than rerunning the whole mission from scratch. No automatic retry
policy in v1 (`on_fail` is effectively always "stop") — the operator
decides what "fixed" means before pressing Start again.

### Logging

Every mission run writes a fresh JSONL log — `missions/data/logs/
<mission_name>_<yymmdd_hhmmss>.jsonl` — one line per step transition
(step index, path name, start/end time, outcome, abort reason if any),
plus periodic position snapshots while a step is running (same shape as
`navigate`'s own `last_run_debug.jsonl`, reused rather than
reinvented). Unlike that file (overwritten each run, "last run only"),
mission logs accumulate — a mission run isn't watched live the way a
single path run is, so there's nothing to compare a fresh log against
after the fact. Retained for a configurable number of days
(`log_retention_days`, default a couple of days per disk space being
cheap), swept on service startup. A log *viewer* page is out of scope
for this first version — raised in planning as a real future need
("develop a viewer for logged files") but not blocking missions v1;
the raw JSONL is readable as-is in the meantime.

### Mission page

One page, same shape as `navigate`'s Run page: a dropdown of saved
missions, Start (optionally from a chosen step)/Stop, live state +
current step number/name via a websocket (`/ws/missions`), and the
abort reason if stopped that way. A separate page lists saved missions
(name, step count) — mirrors `navigate`'s Paths page.

### Create/Edit mission page

One page for both, not two — unlike `navigate`'s Create Path (live
recording by driving) vs. Edit map (hand-editing an existing file),
composing a new mission and editing an existing one are the same
operation (an ordered step list), just starting empty vs. loaded.
Reached either via "Create Mission" in the nav dropdown (no `name`
in the URL — empty step list) or via the Missions page's "Edit"
button (`?name=<mission>` — loads that mission's steps). The nav
dropdown's own label is necessarily static ("Create Mission" — same
as `create-path.html`'s dropdown entry staying "Create Path"
regardless of progress), so the page's own on-page heading is what
actually reflects "New mission" vs. "Editing: `<name>`", not the
dropdown text.

Each step is one row: a dropdown of `navigate`'s saved paths (fetched
via a same-origin proxy, `GET /api/navigate-paths` → navigate's own
`/api/paths` — same CORS-avoidance reasoning as `navigate`'s own
`/jog/manual` proxy), up/down to reorder (no drag-and-drop, same
"skip the fancier interaction" call as the path editor's no-click-to-
select-on-map), and Remove. Add step appends a new `run_path` step.
Save uses the same dialog shape as the path editor (filename,
live overwrite detection, Save-and-continue/Save-and-exit) and
surfaces any continuity warnings the save endpoint returns. No undo —
unlike the path editor's N/S/E/W nudges (easy to mis-click, expensive
to redo by eye), a wrong dropdown pick or reorder here is a single
obvious click to fix, so the added complexity wasn't judged worth it
for v1.

### Config additions

```yaml
missions:
  unit: robot-missions.service
  host: 0.0.0.0
  port: 8010
  web_ui: true
  missions_dir: missions/data
  mission_status_hz: 2        # poll rate against navigate's /ws/navigate while a step runs
  log_retention_days: 2
```

## Out of Scope (v1 of this service)

- Any step type beyond `run_path` (waiting, turn-in-place, pump-only
  steps) — natural, small future additions to the step interpreter, not
  needed for the first real mission.
- Automatic "bridge path" generation to cover a continuity gap — that's
  the future path planner's job (`top-prd.md` item 4), not a stand-in
  built here.
- Retry/replan policy beyond "stop and let the operator decide" — no
  `on_fail: retry`, no conditionals, no branching.
- Reading `safety`'s output to decide whether to stop — `safety`
  doesn't exist yet; `missions` currently only reacts to `navigate`'s
  own abort reasons (cross-track/heading/accuracy), same as an operator
  watching the Run page would.
- A log viewer page — logs are written in a form a future viewer can
  read, but no viewer is built now.
- Mission *authoring* UI (a drag-and-reorder step list, etc.) — v1
  missions are hand-written YAML, same starting point paths themselves
  had before the Edit map page existed.
