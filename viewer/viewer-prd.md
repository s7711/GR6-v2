# viewer

A generic cross-app log browser/plotter. Built 2026-08-31, prompted by
two things happening at once: (1) navigate's own `/pages/logs` had
grown a real limitation — its `/api/logs` hardcoded a two-entry dict
(`{"navigate": ..., "waterbutt": ...}`, see navigate-prd.md's "Log
Viewer") that map-manager's own raw-capture logs were already outside
of, and drive had no logging at all; (2) computer-vision work is coming
that will need the same kind of "load a run, plot it, compare it
against another service's run at the same time" debugging that
navigate's page already did for its own runs. Rather than keep bolting
more hardcoded source directories onto navigate, this pulls that
concern out into its own service that auto-discovers any app's logs,
with no per-app registration step.

This is the first slice — the generic browse/select/plot mechanism.
navigate's own `/pages/logs` was initially left as-is (not touched)
while this was being built, to avoid risking an already-working page;
once this service had seen some real use it was retired outright (see
navigate-prd.md's "Log Viewer — superseded by viewer") rather than kept
running alongside this one.

## Auto-discovery, not registration

`discover_sources()` (app.py) scans every top-level directory in the
repo (excluding `shared`, `venv`, `viewer` itself, and dot-directories)
for a `data/logs/` subfolder, on every request — not once at startup,
not cached — since a source's logs directory can appear after this
service has already started (drive's very first log, or some future
service's first run), and the scan itself is a handful of cheap
`is_dir()` checks. No config entry, no code change, and no service
restart is needed for a new source to show up — it just needs to write
`data/logs/*.jsonl` files, same convention as everyone else (navigate,
waterbutt, jobs, missions, drive).

This deliberately keeps every logging service independent of viewer:
none of them import anything from here, or know this service exists.
viewer only ever reads; it never writes into another service's
directory. See top-prd.md's "Shared configuration" for the same
independence principle applied to config.yaml.

`shared/logs.py`'s `log_file_summary`/`log_summaries` (ported out of
navigate/app.py's original, near-identical private functions) do the
actual per-file parsing — first/last line's `t`, line count, and
`path_name` if the file has one (navigate's own logs do; waterbutt's
and drive's don't) — and both tolerate a file that's still being
written (the last line can be torn mid-write; only the *first* line
failing to parse is treated as genuine corruption and skipped with a
warning, not a 500 — this was a real bug in navigate's original version,
see its own commit history).

## Selecting files across sources: overlap, not exact start-time match

The whole point of a cross-app viewer is comparing files from different
sources that happened at the same real time (e.g. a navigate run against
the drive log it triggered) — but different services' logs for "the same
event" don't share a start time or a duration: a drive log doesn't start
until the wheels actually move (a beat after navigate issues the first
command), and navigate's own logs run on a good deal past their real
end (see navigate-prd.md — deliberate, not a bug: "I like to see what
happens afterward"). So home.html's "Suggested" list is a real
time-*range* overlap test — `[selection.min_start - slack, selection.
max_end + slack]` intersecting a candidate file's own `[start_t, end_t]`
— not a closeness check on start times alone. `OVERLAP_SLACK_S = 3`
(home.html) is a first guess, not measured against anything yet.

Quantities-to-plot are tabbed per loaded file (not one flat
checkbox list across every file, which stopped scaling once a real
mix of navigate/drive/waterbutt files could be loaded together) but
ticking a quantity in any file's tab still overlays it on the one
shared chart — the tabs are purely a findability aid over the checkbox
list, they don't partition what can be plotted together. Same
reasoning, same uPlot/`combinedX`/`timebaseT0` mechanics as navigate's
original page (a single x-axis, seconds since the *earliest* loaded
file's own start, fixed once per Load — not recomputed per file, which
was itself a fix for a real live misalignment bug, see navigate-prd.md).

**Cross-file interpolation (found live 2026-08-31)**: different files
write on their own independent clock/cadence — drive at its own `LOG_HZ`
vs. navigate's `control_hz`, or just two files that don't happen to tick
at the same offset — so their real sample times almost never coincide.
Plotting each series only at its own exact sample times against the
shared `combinedX` (comparing e.g. drive's `LM_setvel_mps` against
navigate's `horizontal_speed_mps` this way) meant nearly every point of
one series landed on an x where the other had no data, breaking each
line into isolated dots ("the graph goes wrong, lots of gaps"). Fixed
by `interpolateAtX` — each series is linearly interpolated between its
own consecutive real points at every `combinedX` tick; a genuine gap
(before this file's own first point, after its last, or a key it simply
doesn't have) still renders as an actual break, only the "no sample
landed exactly here" case is filled in now. Legend labels are just the
quantity name (`key`), not `file.label — key` — dropping the
filename/line-count kept them readable once several files' worth of
series were on one chart together; which file's tab a quantity was
ticked from, plus its colour, is enough context in practice.

The map only plots files that actually carry `lat`/`lon` numeric fields
in their lines (checked per loaded file, not hardcoded to a particular
source name) — waterbutt's and drive's logs have none, so they simply
never appear on it, same behaviour as navigate's original page had
hardcoded to "source === navigate".

### Config

```yaml
viewer:
  unit: robot-viewer.service
  host: 0.0.0.0
  port: 8013
  web_ui: true
```

No config beyond the standard four keys — there is nothing else to
configure; see "Auto-discovery" above.

## Timescale clipping and load performance (2026-09-14)

Picking one selected file as the "Timescale" (a radio choice above the
Load button) clips **both** the chart's x-axis and the map's plotted
points to that file's own `[start, end]` span — added specifically for
loading a short per-run file (e.g. a single navigate path) alongside a
continuously-logged one (oxts-nav, wheelspeed, network) that otherwise
covers far more time than the run being looked at. Found live: the map
side of this was missing entirely (only the chart clipped), so picking
a short timescale still showed a continuously-logged file's entire
untrimmed trail on the map — both the wrong picture, and slower to
render/auto-zoom than the much smaller clipped set actually needed.

Loading a large file was also independently slow for its own reason:
`numericKeys`/`categoricalKeys`/`categoryLevelsByKey` (which fields a
file has, and a categorical field's distinct values) used to be
recomputed by scanning every field of every line from scratch on every
redraw, every quantity-checkbox toggle, and (for the categorical scan)
once per *selected quantity* — several redundant full-file passes per
Load for no reason, since none of that changes once a file is fetched.
Now computed once per file (`indexFile()`, right after fetching) and
cached on the file object. Files are also fetched in parallel
(`Promise.all`) rather than one at a time, and the Load button shows
"Loading…" (disabled) for the whole operation - fetch, indexing, and
redraw - so a slow load is visibly still working rather than looking
stuck with no feedback either way.

## Not yet built

- **Global communication bus.** Ben's own longer-term idea: services
  "shout" what they're doing (e.g. navigate shouting "I'm following a
  path") and other services can choose to listen/act/ignore — a real
  cross-app coordination mechanism, of which "start logging together"
  would be one use among several future ones. Deliberately deferred
  (2026-08-31) in favour of the simpler thing that was actually needed
  right now: every logging service already decides for itself when it
  has something worth recording (navigate on run start, drive on
  idle→moving — see drive-prd.md), and viewer joins files after the
  fact by time-range overlap instead of needing a shared "session"
  concept at record time. Revisit if/when a real second use case for the
  bus shows up.
- **Time-range scrollbar/filter** for whittling down a folder's file
  list once it's got a lot of small entries (drive's jog/turn bursts
  especially) — a VS-Code-search-style minimap the user can click into.
  Explicitly asked to be left for later.
- Any kind of write access, editing, or deleting of another service's
  logs from this UI — viewer is read-only by design (see "Auto-discovery").
