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
- **xNAV config file download:** FTP `NLST` the xNAV's root directory at
  startup and pull down every `mobile.*` file except `mobile.rd` (the
  raw-data recording, not a config file — see "xNAV config editing and
  reset" below), rather than a hardcoded filename list, so a config file
  OXTS adds later is picked up automatically. Any non-`mobile.*` file
  found (e.g. a stray `.ptp` file) is logged but left alone — not
  understood well enough to manage here.
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
- **Headroom above the current `nav_feed_hz: 20`**: the xNAV650 itself
  can supply NCOM at up to 100Hz (GR6-v1's `path_follow.py` consumed it
  at full rate), and `top` shows the ncom-decode-and-publish work costs
  only about 7.3% of one core — plenty of room on this 4-core Pi to run
  at 100Hz if a future consumer (`navigate` being the obvious one)
  actually needs finer-grained position updates than 20Hz gives. Not
  raised now since nothing needs it yet (`navigate`'s ~10Hz control
  loop is already comfortably served by 20Hz) — noted here as an easy,
  low-risk dial to turn later rather than something requiring redesign.
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

### Staleness (added 2026-08-11)

`ncomrx_thread.py`/`ucomrx_thread.py` just block on their UDP socket —
if the xNAV stops sending entirely (a deliberate reset, e.g. changing
the NTRIP mountpoint, or a real fault), nothing ever calls `decode()`
again, and `decoder.nav`/`status`/`connection` sit frozen at their last
real values forever. Both the websocket and this feed were publishing
that frozen snapshot on their own fixed timer regardless — every page
showing live nav data looked perfectly healthy, just frozen on
old numbers, with nothing to say otherwise.

Fixed by stamping `nrx[addr]['last_packet_at']` (`time.monotonic()`)
each time a genuinely new packet is decoded — independent of anything
inside the decoder itself, so it works identically for NCOM and UCOM
and needs no GPS lock to have ever been achieved. `nav_feed.snapshot()`
is now the one place both the websocket route and this feed read
through: past `stale_after_s` (config, default 2.0s) since the last
real packet, it publishes empty `nav`/`status`/`connection` dicts
rather than the frozen ones.

**Empty dicts, not `None`-valued keys** — every consumer (`navigate`'s
`_current_position()`, `wheelspeed`'s `_forward_mps()`, `aruco`'s own
`if nav:` guards) already treats a missing key or an empty dict as "no
fix", so this is exactly the shape every consumer already expects.
`nav["Lat"] = None` would have been a real, findable crash instead:
`aruco/survey.py`'s `if nav:` guard is a truthiness check, which a
non-empty dict full of `None`s sails straight through, then blows up
on `math.degrees(None)`.

**The web UI's own `fillFields()`/`translateNcomCodes()` helpers**
(`shared/web/static/ws-utils.js`/`ncom-strings.js`) only ever filled in
keys they were given — they had the exact same "frozen forever"
problem one layer up, on the display side: a field's element just kept
showing its last value once the key stopped arriving. `fillFields()`
now remembers every element id it has ever set for a given prefix, and
blanks any of them back to "—" once a subsequent call no longer
includes that key — `translateNcomCodes()` needed no equivalent fix,
since every caller already calls `fillFields()` on the same prefix/
data immediately before it, which now blanks the shared element first.

**A second, narrower version of the same bug (found 2026-08-18):**
`stale_after_s` only covers packets stopping *entirely*. Resetting the
xNAV stops packets for a while (triggering the blanking above), but
once it reboots and starts sending again — before it's re-acquired a
real position/heading — packets are flowing, so nothing's stale by the
above definition, yet `ncomrx.py`'s `decoder.nav`/`status` are single
long-lived dicts for the whole process's life, and several fields
(`Lat`/`Lon`/`Heading`/etc.) were only ever *set* when `InsNavMode`
indicated they were valid, never *cleared* when it didn't. A value from
before the reset just sat there, now looking current since real packets
were arriving again — e.g. Heading showing a stale reading with no
actual fix yet, only fixed by restarting oxts-nav itself (which starts
both dicts fresh). Fixed in `ncomrx.py`'s `decode()`: the
`InsNavMode in [0,5,6,7]` ("all quantities invalid") branch now clears
`self.nav`/`self.status` instead of only `self.status`, and the
position/velocity/attitude fields are now explicitly popped whenever
`InsNavMode` isn't one of the modes that decodes them (e.g. an
IMU-only reacquisition mode right after a reset — valid enough to skip
the branch above, but still with no real position/heading yet). No
dedicated test added — `ncomrx.py`'s `decode()` has no unit tests at
all today (it works on raw byte-level NCOM packets with sync/checksum
framing), consistent with the rest of this file, not a gap introduced
by this fix.

## xNAV config editing and reset (added 2026-08-11)

The "download and view only" config page now supports editing and
uploading `mobile.*` files, and resetting the xNAV — the operator flow
this exists for is: edit one or more files, then hit reset once so
they all take effect together (config files are only read at power-on/
reset, never live — see "xNAV650 commands" above).

- **File discovery is now dynamic**, not a hardcoded list (see the
  "xNAV config file download" bullet above) — `mobile.rd` (raw data
  recording) is explicitly excluded, and any non-`mobile.*` file found
  is logged but not touched.
- **Edit/upload** (`GET`/`POST /xnav-config/<filename>.txt`): the page's
  "Edit" button opens a modal with the file's current text in a
  `<textarea>` (fetched from the same route the "view" link already
  used); "Upload" `POST`s the edited text straight back to this route,
  which FTP-`STOR`s it to the xNAV *and* overwrites the local mirror on
  success, so the page immediately reflects what's now on the device.
  "Cancel" just closes the modal — no request sent. No parsing/
  validation of the content — an operator who uploads something the
  xNAV can't parse will find out when it doesn't come up correctly
  after reset, same as if they'd edited it by hand over FTP.
- **Reset** (`POST /xnav-config/reset`): sends `!reset` over the same
  UDP command path the manual command box already uses (see "xNAV650
  commands" above) — not a new mechanism. Gated behind a confirmation
  modal warning that anything currently driving will lose its position
  feed, since a reset takes the xNAV offline for a while to reboot.

## Aruco-priority GNSS mode (added 2026-08-14)

`navigate` can ask this service (via `POST /gnss/aruco-priority` /
`POST /gnss/normal`) to disable GNSS in favour of the aruco marker
alone for specific paths — see `navigate-prd.md`'s "Aruco priority"
for the full reasoning (live testing found GNSS+aruco together
producing a worse, more variable water-butt approach than aruco alone
with GNSS disabled). `navigate` only asks; this service owns the
actual xNAV command and the safety net:

- `gnss_mode.GnssModeController` sends `!disable gnss`/`!enable gnss`
  (the exact xNAV650 command strings, confirmed live — not documented
  anywhere else in this codebase before now) and becomes a client of
  `aruco`'s own `/ws/aruco` feed the whole time it runs (not just
  while in aruco-priority mode) — same always-connected,
  reconnect-on-failure pattern `waterbutt`'s QC loop already uses for
  that identical feed.
- If no marker has been detected for `aruco_priority_timeout_s`
  (config, default 3.0s) while in aruco-priority mode, GNSS is
  re-enabled automatically — regardless of whether `navigate` ever
  calls `/gnss/normal` itself. `navigate` asking for aruco priority is
  a request; this timeout is the guarantee that a lost marker (camera
  fault, marker knocked over, `aruco` itself down) can't leave the
  robot on dead reckoning alone indefinitely.
- Watches for *any* marker detection, not specifically a named one —
  keeps this service from needing to know "the water-butt marker" is
  special; the actual question this timeout answers is just "do we
  currently have a trustworthy vision fix at all."
- `GET /gnss/status` for visibility (`{"mode":, "last_marker_seen_at":}`).
- The watchdog check itself is factored out as `_watchdog_tick()`
  (called directly in tests) rather than only living inside its
  `while True` loop — same reason `navigate`'s `_control_tick()` was
  split out the same way.

## Config additions (shared config file)

- `xnav_ip` — the xNAV650's IP address (top-level, since other future
  services may also need it, not nested under this service alone).
- Under this service's own entry: `protocol` (`ncom` or `ucom` — which
  decoder/port `app.py` uses, read once at startup; see "NCOM/UCOM
  protocol switch" below), `nav_update_hz` (websocket publish rate),
  `nav_feed_socket`/`nav_feed_hz` (the cross-process nav feed, see
  above), `aruco_priority_timeout_s` (see "Aruco-priority GNSS mode"
  above), plus the usual `unit`/`host`/`port`/`web_ui` fields every
  service has. No other command-related config — see "xNAV650
  commands" above.

## NCOM/UCOM protocol switch

OXTS is deprecating NCOM in favour of UCOM (a newer, self-describing
"sources and signals" protocol — see `UCOM_Manual_260707.pdf` in this
folder). Full field-by-field comparison, design reasoning, and a running
log of what's been built/found/fixed lives in `ncom-to-ucom-mapping.md`
rather than here, since it's a large, evolving research document — this
section is just the summary.

- `ucomrx.py`/`ucomrx_thread.py` mirror `ncomrx.py`/`ncomrx_thread.py`'s
  structure exactly (own socket on UCOM's fixed port 50487, one explicit
  `decodeMessageN` per message rather than a generic schema-interpreting
  decoder, same `nav`/`status`/`connection` dict shape) — deliberately
  duplicated rather than sharing a base class with the NCOM versions.
- `oxts-nav.protocol` picks which one `app.py` constructs at startup —
  restart-only, no dynamic switch, same as every other config value in
  this project. NCOM remains the default; UCOM is opt-in while it's
  still being field-validated.
- `oxts-nav/documentation/` holds the xNAV650's reference `oxts.dbu`/
  `oxts.dbs` (every message/signal OXTS defines), alongside both manual
  PDFs and `ncom-to-ucom-mapping.md`. The actual deployed config,
  `mobile.dbu` (what we've built and uploaded to Amundsen, enabling only
  what's actually consumed today — nav PVA, accuracies, GNSS status, GAD
  statuses/innovations, INS status, SDN time offset, plus one custom
  message — see below), lives at `xnav-config/mobile.dbu.txt` instead —
  auto-downloaded alongside the other `mobile.*` config files (see
  `app.py`'s `download_xnav_config`), not documentation, since it's the
  live deployed artifact rather than reference material.
- Custom UCOM messages (IDs 64512–65535) are confirmed to work on this
  firmware even though OXTS's own NAVconfig tool has no way to create
  one — several fields exist in `oxts.dbs` but aren't packaged into any
  of OXTS's 93 predefined messages (`BaseStationID`, GNSS reject
  counters, IMU bias/scale-factor, etc.). `mobile.dbu`'s message 64513
  pulls these in by hand.
- All of this was built and field-tested against Amundsen's real
  xNAV650, not just unit-tested — see `ncom-to-ucom-mapping.md` for the
  real bugs found and fixed along the way (a CRC32 linearity bug causing
  false duplicate-packet detection, a missing time-correlation
  mechanism, message-routing gaps).

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
- ~~Editing xNAV650 config files (upload/write-back over FTP)~~ — **done,
  see "xNAV config editing and reset (added 2026-08-11)" below.**
- GAD aiding data sent to the xNAV650 — carried over conceptually from
  `xnav.py`/`gad_aruco.py`, not designed here.
- Command encryption — OXTS are adding cyber-security features that will
  apparently require commands to be encrypted somehow; no details yet,
  not designed here, but noted so it isn't a surprise later.
- Any consumer-specific display logic (graphs, readouts, tables) — that
  lives in whichever webpage/service consumes this feed, per `ui-style.md`
  and the "server doesn't change for the webpage's sake" principle above.
- ~~NCOM → UCOM decoding (future idea, not designed here)~~ — **done, see
  "NCOM/UCOM protocol switch" above and `ncom-to-ucom-mapping.md`.**
  Still out of scope: actually flipping `oxts-nav.protocol` to `ucom` by
  default (NCOM stays the default until UCOM's had more field time), and
  a couple of `STR`-type fields (`BaseStationID`, `DevID`) deliberately
  left out of the custom message for now.
