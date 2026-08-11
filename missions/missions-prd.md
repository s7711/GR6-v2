# PRD: Mission Sequencing Service (missions)

See `top-prd.md` for where this fits: `drive` → `navigate` → `jobs` →
**missions** (this document). One more layer in the same "one process
per responsibility" chain jobs already established over navigate.

## Problem Statement

`jobs` (done, field-proven) sequences saved `navigate` paths (plus
pause/water/fill) into one job — one bed, one visit. In practice, a
day's watering round is several jobs back to back ("go water the
kitchen bed, then the stable bed, then the front beds") — originally
discussed as "mission" during the missions→jobs rename (see
`jobs-prd.md`'s own history): a mission was always meant to be the
*outer* wrapper, composed of several trips, not the single-visit unit
`jobs` ended up naming itself. This is that outer layer, finally built.

Same two problems jobs solved one level down, one level up:

1. **Sequencing multiple jobs unattended** — starting the next job
   once the previous one finishes, stopping the whole mission if one
   job fails partway rather than silently carrying on to the next.
2. **Observability over a mission-length unattended run** — longer
   than any single job, so the same "log it, don't rely on someone
   watching" reasoning applies again.

## Solution

A new service, `missions`, following this project's usual shape
(Flask + shared header/template, `config.yaml`-driven, systemd unit,
port 8011). It owns *sequencing and monitoring* only — no path-
following or job-authoring logic of its own, and it never talks to
`navigate` or `drive` directly; it drives `jobs` exactly the way a
browser does today (`/control/start`, `/control/stop`,
`/control/status`, `/ws/jobs`). `jobs` stays the only thing that talks
to `navigate`; `navigate` stays the only thing that owns the control
loop.

One step type so far: `run_job` (runs a saved job to completion,
watching jobs' own status). No `pause` at the mission level yet — each
job can already carry its own trailing `pause` step if a wait is
wanted between two jobs, so there's nothing a mission-level pause
would do that isn't already possible one layer down. Extending the
step vocabulary later (see "Deferred" below) follows the same
reasoning `jobs-prd.md` gives for its own step types: a new step type
is a deliberate, reviewable Python change, not something an
open-ended script could attempt unpredictably.

### Mission file format

```yaml
name: Flowerbeds
steps:
  - type: run_job
    job: Water kitchen bed
  - type: run_job
    job: Water stable bed
```

Stored as `missions/data/<name>.yaml`, same storage shape as
`jobs/jobs.py` (itself mirroring `navigate/paths.py`) —
`missions/missions.py` is the third copy of the same
list/load/save/delete-with-`_SAFE_NAME`-validation pattern rather than
a fourth new convention.

### Polling jobs' status: no push feed needed

`JobRunner` watches `navigate` via `navigate`'s own push feed
(`navigate_feed_socket`, a `FeedClient`/`FeedServer` Unix socket) —
built because something else (the live map trail) already needed
`navigate` publishing state continuously. `jobs` has no equivalent
need yet, so it never built a push feed of its own. Rather than adding
one purely so `missions` had something to consume, `missions` polls a
new plain synchronous endpoint, `GET /control/status` on `jobs`
(returns `runner.status()` directly) — simpler than standing up a
feed, and it always reflects jobs' true current state, not a cached
snapshot, since there's no publish-on-a-timer layer in between to go
stale.

That said, a *different* race applies for the same underlying reason
one always does when two threads touch the same state machine
concurrently: `go()` sets `self.state = "running"` *before* the
blocking `start_job()` HTTP call to `jobs` returns, so the background
tick loop's own thread can call `tick()` → `job_status()` in that
window and see whatever `jobs` was reporting *before* it had actually
processed the start call — e.g. the previous mission run's terminal
state. Guarded the same way, and for the same class of bug, as
`JobRunner`'s own `RUN_PATH_FEED_GRACE_S` fix (found live 2026-08-09,
see `jobs-prd.md`): stamp a start time before the blocking call, and
don't trust a "finished" status until either it's actually been seen
`"running"` at least once, or a short grace period
(`RUN_JOB_STATUS_GRACE_S`) has passed. See `control.py`'s module
docstring for the full reasoning, and `test_control.py`'s
`TestRunJobRace` for a reproduction of the exact crash class this
guards against (mirroring `jobs/test_control.py`'s own regression
test).

### Mission execution (`MissionRunner`, mirrors `JobRunner`)

States: `idle` → `running` → `stopped_ok` (all steps completed) /
`aborted` (a step failed) — identical shape to `JobRunner`/
`PathRunner`. One `current_step_index`, advanced only on a step's
success.

Per `run_job` step: `/control/start` on `jobs` → poll `/control/status`
until state leaves `running` → `stopped_ok` advances to the next step,
`aborted` stops the mission and records the reason against that step
— so a job aborting (for whatever reason: a bad entry, an
accuracy/heading limit, an operator stop elsewhere) aborts the whole
mission, not just that one job. `jobs`' own response shape from
`/control/start` is normalised in `app.py`'s `start_job()` before
reaching `MissionRunner`: it can return either `{"ok": False,
"reason": ...}` (refused before even trying) or the full status dict
with no `"ok"` key at all (a real attempt, whatever its outcome) —
`MissionRunner` only ever sees the one consistent `{"ok": bool,
"reason": ...}` shape every other injected `start_*` callable in this
project already returns.

**Stop**/**Resume**: identical semantics to `JobRunner` — Stop leaves
state `idle` (not `aborted`) and always stops `jobs` regardless of
what it thinks it's doing; Start can be given a `start_index` to
resume from a chosen job rather than always from the mission's start.

### Logging

Same shape as `jobs`' own per-run JSONL log
(`missions/data/logs/<mission_name>_<yymmdd_hhmmss>.jsonl`), for the
same reason: a mission-length run isn't watched live, so there's
nothing to compare a fresh log against after the fact without one.
Retained for `log_retention_days`, swept on startup.

### Missions page

Same shape as `jobs`' own Run/Jobs pages: a dropdown of saved
missions, Start (optionally from a chosen job)/Stop, live state +
current job number/name via a websocket (`/ws/missions`). A separate
page lists saved missions (name, job count) — mirrors `jobs`' own
Jobs page.

**Jog + upcoming-job preview** (added at the same time as v1, not
deferred): lining a mission up needs the same jog control `jobs`
already has on its own Run page, proxied one hop further —
`missions`' `POST /jog/manual` forwards to `jobs`' own `/jog/manual`,
which forwards again to `navigate`'s — never skipping a layer, same
boundary as everywhere else. The preview map shows whichever job will
actually run next (the chosen start index, not necessarily the
mission's first job) — fetched via two thin proxies through `jobs`
(`GET /api/jobs/<name>` for that job's own steps, `GET
/api/navigate-paths/<name>` for each referenced path's points, both
already existing on `jobs`, just forwarded one hop further) — since
that's the job the robot actually needs to be lined up against, not
the mission's own start. The live trail connects the browser straight
to `oxts-nav`'s feed (not proxied — a WebSocket isn't subject to CORS
the way a `fetch()` is), same pattern `jobs`' own Run page uses.

### Create/Edit mission page

Same shape as `jobs`' own Create/Edit Job page, simplified by having
only one step type: each step is a row with a dropdown of saved jobs
(fetched via `GET /api/available-jobs`, proxying `jobs`' own `/api/
jobs`), up/down to reorder, Remove. **Add job** appends a new
`run_job` step. No map on this page (unlike Create/Edit Job) — a
mission's own steps don't carry any geometry directly, only job
names; previewing a job's actual route is the Run page's job (see
above), not this one's.

### Config additions

```yaml
missions:
  unit: robot-missions.service
  host: 0.0.0.0
  port: 8011
  web_ui: true
  missions_dir: missions/data
  mission_status_hz: 2   # poll rate against jobs' /control/status while a run_job step runs
  log_retention_days: 2
```

## Deferred

- **Cross-job continuity checking.** `jobs`' own save-time continuity
  check (`entry-check`-equivalent between two `run_path` steps) has no
  analogue here yet — saving a mission doesn't check whether the last
  `run_path` step of one job lines up with the first `run_path` step
  of the next. The same "skip over non-`run_path` steps to find the
  real nearest pair" fix just applied within `jobs` (see its own PRD)
  would need to reach *across* a job boundary to cover this — a
  natural follow-up, not built yet.
- **A generalised recovery/retry mechanism.** Discussed at length
  2026-08-11 (heading-drift-while-stationary recovery, a waterbutt
  return-accuracy calibration loop or fine-adjustment tool, GNSS-jump
  handling, bump/obstacle recovery) — all of them want the same shape
  ("detect not-good-enough, try a bounded number of recoveries, then
  give up and abort upward"), which `missions`' own abort propagation
  (job aborts → mission aborts) is one necessary piece of, but the
  rest (a per-step retry policy in `jobs`, and navigate-level assist
  maneuvers below the level of a saved path) isn't designed yet. Not
  blocked on anything here — `missions` degrades gracefully to today's
  "just abort" when no recovery is configured.
- **A mission-level `pause` step** — not needed yet; a job can already
  carry its own trailing pause.
- A log viewer page — same reasoning as `jobs-prd.md`'s own deferral.
- Mission *authoring* UI beyond a plain step list (drag-and-reorder,
  etc.) — same starting point `jobs`' own Create/Edit Job page had.
