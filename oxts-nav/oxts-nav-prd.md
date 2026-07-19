# PRD: OXTS Nav Decode Service (oxts-nav)

See `../top-prd.md` for overall architecture decisions (process model, IPC,
shared config, systemd, version control) and `../ui-style.md` for web UI
conventions — this document only covers the nav-decode service itself.

## Problem Statement

The robot needs real-time position/velocity/attitude data from the
xNAV650, decoded from its NCOM UDP stream, made available to any other
service that needs it (path-following, a future manager header status
badge, a future NCOM viewer) — without re-implementing NCOM decoding in
each consumer, and without the tight coupling the GR6-v1 monolith had
(decoder push logic hardcoded to a specific bespoke websocket object,
requiring server-side code changes whenever a webpage's needs changed).

## Prior art

GR6-v1 (`https://github.com/s7711/GR6-v1`, and the working copy at
`/home/pi/share/python/GR6-v1/`) already has a mature NCOM decoder:

- `ncomrx.py` — the actual protocol decoder (Batch A/B/S, ~40 status
  channels, GPS time handling). MIT-licensed, already correct and
  hard-won; not worth recoding from scratch.
- `ncomrx_thread.py` — background thread reading UDP port 3000, one
  `NcomRx` decoder instance per source IP (with CRC dedup for a real
  Linux UDP-repeat issue). Also solid, kept close to unchanged.
- `xnav.py` — the part that's actually obsolete for this project: pushes
  nav/status/connection dicts to a bespoke `ws` pub/sub object every
  0.5s, dispatches prefixed user commands, does FTP config download/
  upload. This is the piece being replaced, not reused, because GR6-v2's
  IPC/web conventions are different by design (see below).

Decision: **reuse `ncomrx.py` and `ncomrx_thread.py` largely as-is**;
**write a new, thin publishing layer** for this service in place of
`xnav.py`.

## Solution

An independent systemd service (`robot-oxts-nav.service`) that:

1. Runs the (reused) `NcomRxThread` to receive and decode NCOM UDP
   packets on port 3000.
2. Only pays attention to the configured xNAV650 IP address (`xnav_ip`
   in the shared config file — already anticipated by `top-prd.md`).
   Packets from any other source are ignored by this service. The
   underlying `ncomrx_thread.py` class is deliberately left capable of
   handling multiple source IPs, unmodified — GR6-v1 needed that for a
   multi-INS setup that no longer applies here, but keeping it means a
   possible future NCOM viewer (see Out of scope) can reuse the same
   class to show/select from multiple streams without this service's
   code changing.
3. Publishes the decoder's `nav`, `status`, and `connection` dicts, in
   full, as JSON over a plain WebSocket (`flask-sock`, consistent with
   the manager) at a configurable rate (`nav_update_hz` in shared
   config). The service never picks out individual fields for a
   particular consumer — it broadcasts everything it has, and each
   consuming webpage or service decides what it needs from that. This
   is the core fix for the GR6-v1 problem: **what the server sends must
   not change because a webpage changed** — only the browser side
   should need to change when a page's requirements change.
4. Serves a minimal web page (nav data table/readouts, consistent with
   `ui-style.md`) for manual viewing/debugging, connecting to the same
   websocket feed any other consumer would use — no separate/duplicated
   feed for "the UI" vs. "other services."
5. Provides a free-text command box on the web page: whatever the
   operator types is sent, verbatim, straight to the xNAV650 over UDP
   (port 3001). This is genuinely needed — most of the time the xNAV650
   self-initialises fine outdoors, but indoors it sometimes needs a
   manual command before it starts outputting full nav data. No
   automatic startup sequence, no config-driven command list — this is
   an occasional manual action, not something to script.
6. Downloads the xNAV650's own configuration files over FTP at startup
   (reusing `xnav.py`'s approach) and serves them as plain static files
   for viewing in a browser — view-only for now.

## xNAV650 commands

There is no automatic initialisation sequence — that idea (and the
"send time" example carried over from GR6-v1's `xnav.py`) was an
abandoned experiment in GR6-v1 that never got cleaned up, not a real
requirement. What's actually needed is simple: a text box on the web
page, and whatever the operator types gets sent straight to the xNAV650
over UDP. This is normal, occasional, manual behaviour (mostly needed
indoors), not something to automate at startup.

**Future note:** OXTS are adding cyber-security features to the xNAV650,
which will apparently mean commands need to be encrypted somehow.
Details aren't available yet. When they are, this command-sending path
is where that change lands — worth remembering when it comes up rather
than being surprised commands stop working plainly one day.

## Implementation Decisions

- **Folder:** `oxts-nav/`.
- **Reused from GR6-v1 near-unchanged:** `ncomrx.py`, `ncomrx_thread.py`.
  One cleanup while porting: `NcomRxThread.__init__` currently calls
  `ncomrx.NcomRx.__init__(self)` on itself, but every actual decoder
  instance lives in `self.nrx[ip]['decoder']` instead — this looks like
  dead code left over from an earlier design and should be dropped
  during the port (confirm nothing relies on it first).
- **Not reused:** `xnav.py` — replaced by a new, small publisher module
  written for GR6-v2's IPC/web conventions. (Also note: the copy of
  `xnav.py` in GR6-v1 has its entire contents accidentally duplicated in
  the file — a paste error, not a design worth preserving.)
- **Single xNAV650 assumption:** the service reads `xnav_ip` from shared
  config and only surfaces `nrxs.nrx[xnav_ip]`. No UI or IPC surface for
  selecting between multiple INS units in this service — that's
  explicitly a future NCOM-viewer concern, not this one.
- **Websocket / no-buffering requirement:** `flask-sock`
  (`simple-websocket` underneath), not Flask-SocketIO. Rationale:
  Flask-SocketIO's Engine.IO layer keeps a per-client outgoing queue and
  does ack/reconnect bookkeeping — exactly the buffer-and-replay
  behaviour that caused problems over flaky wifi previously. `flask-sock`
  does a direct synchronous socket write per `ws.send()`; a broken or
  badly congested connection raises rather than silently queuing. The
  publish loop itself must also not buffer: each tick reads the *current*
  `nav`/`status`/`connection` dicts and sends them; if a send stalls or
  fails, that tick's data is simply dropped, and the next tick sends
  whatever is current then. Never queue up stale data to send later.
- **Update rate:** `nav_update_hz` lives in the shared config file (not
  hardcoded), so it can be tuned without a code change. Default
  suggestion: 2Hz, consistent with `ui-style.md`'s graph update-rate
  default — no reason to publish faster than any consumer plots, and
  bandwidth is a real concern on wifi.
- **Cross-thread access to nav/status/connection:** the decoder thread
  writes these dicts continuously while the publisher thread reads them.
  GR6-v1's own code has a `# todo: protect nav, status with a lock`
  comment that was never actioned. For GR6-v2, add a simple `Lock`
  around read/write of these dicts now rather than deferring again —
  it's cheap (single process, low contention) and avoids a snapshot
  being read half-old/half-new across keys.
- **Command sending:** reuse `xnav.py`'s pattern of a UDP socket to
  `(xnav_ip, 3001)`. Whatever text the operator enters in the web page's
  command box is sent verbatim — no prefix parsing/dispatch logic needed
  (GR6-v1's `!`/`#`/`&`/... prefix routing existed because one monolith
  handled many subsystems from one input box; this service only ever
  talks to the xNAV650, so there's nothing to route between). No
  automatic sequence at startup — see "xNAV650 commands" above.
- **xNAV config file download:** reuse `xnav.py`'s FTP download logic
  (list of `mobile.*` files, pulled to a local folder) at startup, and
  serve that folder as static files from this service's web UI —
  essentially the same "just view it in a browser" approach GR6-v1 used
  via its `static/xnav-config/` folder. No editing capability yet —
  see Out of Scope.
- **GAD aiding data:** still deferred — carried over conceptually from
  `xnav.py`/`gad_aruco.py` but not designed here. Revisit once
  path-following or vision needs to send aiding data back to the
  xNAV650. (The nav data feed below is what those consumers will read
  *from* the xNAV — this bullet is the separate, still-undesigned,
  send-*to*-the-xNAV direction.)

## Nav data feed (cross-process)

Built ahead of need — nothing consumes this yet, but every future
consumer (aruco/GAD send, path-following) needs the same thing, so it's
built once now rather than three times later.

- **What:** `nav_feed.py` runs a small Unix domain socket server
  alongside the web app, publishing the same full `nav`/`status`/
  `connection` dicts the websocket sends — same "don't tailor what's
  sent to one consumer" principle as the websocket (see above).
- **Path/rate:** socket path is `nav_feed_socket`, rate is `nav_feed_hz`,
  both in shared config, deliberately separate from the web UI's
  `nav_update_hz` — other services will likely want a higher rate than a
  browser chart does.
- **Protocol:** any number of clients may connect. Each gets its own send
  loop; a 4-byte big-endian length prefix followed by that many bytes of
  a pickled dict (needs framing since it's a stream socket, unlike the
  browser's message-based websocket). A stalled/dead client's loop exits
  independently — it never blocks or backs up delivery to other clients.
- **Timing:** the one thing that has to survive the move to a separate
  process is the xNAV's machine-time-to-GPS-time mapping
  (`connection['timeOffset']`, filtered in `ncomrx.py` from
  `time.monotonic()` timestamps stamped by `ncomrx_thread.py` on receipt).
  A consumer with its own `time.monotonic()` timestamp for something else
  (e.g. a future camera frame's capture time) can correlate it with GPS
  time via the new standalone `ncomrx.machine_time_to_gps(machine_time,
  time_offset)` function — a consumer doesn't need a live `NcomRx`
  decoder instance of its own, just the `timeOffset` this feed already
  publishes in `connection`. `time.monotonic()` (not `time.perf_counter()`)
  is the clock in use throughout, since that's what `ncomrx_thread.py`
  already stamps packets with; both track `CLOCK_MONOTONIC` on Linux, so
  timestamps are directly comparable across processes on the same
  machine — no separate time-sync mechanism needed.

## Config additions (shared config file)

- `xnav_ip` — the xNAV650's IP address (top-level, since other future
  services may also need it, not nested under this service alone).
- Under this service's own entry: `nav_update_hz` (websocket publish
  rate), `nav_feed_socket`/`nav_feed_hz` (the cross-process nav feed, see
  above), plus the usual `unit`/`host`/`port`/`web_ui` fields every
  service has. No command-related config — see "xNAV650 commands" above.

## Testing Decisions

- Decode logic (`ncomrx.py`) can be tested by replaying a captured
  `.ncom` file's raw bytes into the decoder directly — no hardware or
  network needed, and GR6-v1's `ncomrx_thread.py` already supports
  logging raw NCOM to a file, so a recording exists or is easy to make.
- The publishing layer can be tested by pointing a plain WebSocket
  client at the service and confirming it receives the full nav dict at
  roughly `nav_update_hz`, without needing a real xNAV650 (feed the
  decoder synthetic/replayed packets instead).
- The nav feed can be tested the same way: connect a plain
  `socket.AF_UNIX` client to `nav_feed_socket`, read the 4-byte length
  prefix then that many bytes, `pickle.loads()` it, and confirm the
  dicts match what the websocket reports for the same instant.

## Out of Scope

- Multi-INS NCOM viewer with stream selection — a future idea; not
  designed here, but not precluded either, since `ncomrx_thread.py`
  stays multi-IP-capable underneath.
- Editing xNAV650 config files (upload/write-back over FTP) — download
  and view only for now; an editor is a real future want (OxTS's own
  NAVconfig tool being Windows-only and painful is the motivation) but
  explicitly not now.
- GAD aiding data sent to the xNAV650 — carried over conceptually from
  `xnav.py`/`gad_aruco.py`, not designed here.
- Command encryption — OXTS are adding cyber-security features that will
  apparently require commands to be encrypted somehow; no details yet,
  not designed here, but noted so it isn't a surprise later.
- Any consumer-specific display logic (graphs, readouts, tables) — that
  lives in whichever webpage/service consumes this feed, per `ui-style.md`
  and the "server doesn't change for the webpage's sake" principle above.
