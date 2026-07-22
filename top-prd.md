# Top-Level PRD: GR6-v2 Multi-Program Architecture

The GR6 robot is a two wheeled (plus casters) robot designed for watering
plants. It has a Raspberry Pi 4, an OXTS xNAV650, a pi camera, an Arduino
(soon to be changed to a Raspberry Pi Pico), and battery. The Arduino has
a motor controller, wheel encoders, water pump, SH-04 unltra sonic sensors.
The Raspberry Pi is running Raspberry Pi OS.

This is the umbrella document for the overall architecture. Individual
services (vision, nav decode, path following, manager, etc.) each get their
own PRD in their own subfolder.

## Repository structure

One repo, one folder tree (monorepo) — not one repo per service. Reasons:
services are tightly coupled (shared IPC protocol, shared config schema,
versions that must move together), so ten repos would mean ten places to
keep the protocol in sync and a real risk of version-mismatch bugs (service
A updated, service B not).

Suggested top-level layout (adjust as needed):

```
gr6-v2/
  top-prd.md          <- this file
  ui-style.md         <- appearance/UI conventions shared by every service's web UI
  shared/             <- shared code: config loader, IPC helpers, seqlock class
                         (also shared/web/ — see ui-style.md)
  camera/
    camera-prd.md
    ...
  aruco/
    aruco-prd.md
    ...
  oxts-nav/
    oxts-nav-prd.md
    ...
  navigate/
    navigate-prd.md
    ...
  manager/
    prd.md
    ...
  config.yaml (or similar)  <- shared runtime config, see below
```

Only split a piece into its own repo if it becomes genuinely independent
(reusable elsewhere, different release cadence, different maintainer) —
nothing currently in scope meets that bar.

## Problem Statement

The robot control software currently runs as a single-threaded Python
program on a Raspberry Pi ("amundsen"), handling camera/ArUco processing,
nav data decoding (OXTS xNAV650), and path following in one process. This
leaves the Pi's other CPU cores unused, couples unrelated tasks together,
and makes debugging harder (a breakpoint anywhere pauses everything).

The plan is to split these responsibilities into independent programs
("services"), each with its own lifecycle, running concurrently on separate
cores, communicating over well-defined, deliberately simple interfaces.

## Solution

Each robot responsibility becomes its own independent Python program
(own PID, not a thread, not a `multiprocessing.Process` child), managed as
a systemd service, communicating over IPC interfaces chosen by data
size/rate. A shared config file is the single source of truth for
cross-service settings. A small custom manager app gives a single place to
see status and control services, without taking on Docker or a
general-purpose sysadmin tool.

## Architectural Decisions (apply to every service)

### Process model
- Each service is a separate OS process, started/stopped/restarted via a
  **systemd unit** (`robot-<service>.service`), not spawned as a child of
  any other program (including the manager — see manager PRD).
- Rejected alternatives: Docker (unneeded isolation/portability overhead for
  one Pi; shared-memory access across containers is awkward), `supervisord`
  (duplicates systemd on a systemd-based OS), `multiprocessing.Process`
  children of one parent (defeats the goal of independent lifecycle/
  debuggability).

### IPC, chosen per data shape
- **Large / high-rate data (camera frames):** `multiprocessing.shared_memory`
  — named POSIX shared segment, attached independently by each process.
  Near-zero-copy. Requires explicit synchronisation (lock or seqlock) since
  `shared_memory` gives no tearing protection itself.
- **High-rate structured data (nav data, ~200 fields, up to 100Hz):**
  pickled dict over a Unix domain socket is sufficient (~50–150µs overhead
  vs a 10ms budget at 100Hz). Optional future refinement: fixed-layout
  struct/msgpack in shared memory with a seqlock, for lower/more
  deterministic latency — not required to hit current targets.
- **Low-rate structured data (Arduino telemetry, motor commands, ~10Hz):**
  pickled dict over Unix domain socket — overhead is negligible against a
  100ms budget.
- **Commands / mode changes:** Unix domain socket (ZeroMQ if pub/sub
  semantics become useful later).
- Any shared-memory structure needs synchronisation against torn reads:
  a `Lock`, or a seqlock (writer bumps a sequence counter before/after
  writing; reader retries on odd/changed sequence) for lock-free low-latency
  reads on the control-loop side.

### Shared configuration
- A single config file (format TBD — YAML/`.env`/similar) is the source of
  truth for: xNAV650 IP, each service's host/port (if it has a UI), shared
  memory segment names, socket paths. Every service reads this at startup;
  nothing is hardcoded locally. Schema itself is not fixed by this document —
  left for the implementing service (or the manager) to propose.

### Debugging
- Any service can be stopped (`systemctl stop robot-<service>`) and run
  directly under the VS Code debugger instead, while the rest keep running
  normally — this is the expected, normal debug workflow, not a workaround.
- Debugging two connected services at once: run two independent debug
  sessions (e.g. via Remote-SSH, two windows or two launch configs into the
  same remote host) rather than needing lock-step cross-process debugging.
- Stale shared-memory segments left behind by a crashed or forcibly-stopped
  debug session should be handled by an unlink-if-stale check at each
  service's startup.

### Version control
- One git repo for the whole tree. `.gitignore` should exclude
  `__pycache__/`, local config containing real IPs/secrets, and logs.

## Suggested migration order

1. Nav decode (xNAV650 data handling) — done (`oxts-nav`).
2. Manager — done (`manager`), giving services status/control visibility
   as they came online.
3. Vision, split into two services: `camera` — done (capture, timing/
   exposure/gain metadata, shared-memory frame publishing, calibration) —
   then `aruco` (marker detection + GAD updates to the xNAV650, consuming
   the camera's frames) — in progress, see `aruco/aruco-prd.md`.
4. Motor control, split into `drive` (low-level: motor velocity, water
   pump, ultrasonics, encoder/PID telemetry, over USB serial to the
   motor-controller microcontroller) — done, see `drive/drive-prd.md` —
   then `navigate` (waypoint/path-following), `missions` (sequencing
   multiple stops + pump actions), and `safety` (obstacle-avoidance
   decisions, once the ultrasonics are trusted) built on top of it, in
   that order. These later three are the safety-critical pieces — most
   confidence wanted before touching them, hence saved for last.
5. Network sharing (wifi -> ethernet internet sharing) — not yet built,
   identified as needed while testing `aruco`/`oxts-nav` together: the
   xNAV650 sits on `eth0`, and needs internet access (for NTRIP
   corrections) shared from the Pi's `wlan0`. A working iptables
   MASQUERADE + FORWARD script already exists (not yet in this repo) —
   it needs turning into a proper managed service, since both the
   iptables rules and `net.ipv4.ip_forward` reset on every reboot as-is.
   Also needs a static IP configured on `eth0` (and the xNAV650's own
   gateway/DNS pointed at it) as a prerequisite, not something the
   service itself can do. No PRD yet — do not build until picked up
   properly.
6. Wheelspeed GAD aiding — not yet built, identified while surveying
   ArUco markers with `aruco`: position-only GAD from a stationary/known
   marker doesn't help the INS solution *between* good fixes, whereas
   wheelspeed aiding keeps drift much lower while driving through a
   GNSS-poor patch, making the fix at the next marker (or on return to
   good sky view) far more accurate. Needs wheel encoder data from the
   Arduino/Pico first — blocked on the `drive` service (see "motor
   control", item 4) actually publishing encoder ticks; can't be
   built before that exists. No PRD yet.
7. Wifi improvements — not yet built, deliberately deferred until
   `navigate`'s path-following surfaced real pain from it (a jog/drive
   command arriving late after a wifi reconnect could make the robot
   behave unexpectedly for a moment — worse under Flask's own request
   handling than the hand-rolled websocket code GR6-v1 used, though
   every channel is affected to some degree). Planned approach is
   hardware/network first, not a software workaround: (a) a dual-band
   USB wifi adapter already bought, and (b) driving from a tablet
   hotspotting directly to the Pi (short range, operator right next to
   the robot) rather than through the house's own wifi.
   A software mitigation was considered (timestamp browser-originated
   commands, reject anything too stale by the time `drive` processes
   it) but explicitly parked — it adds real complexity/latency for the
   driver, and the hardware/network fix is expected to remove the
   problem at its source instead. Revisit only if the hardware/hotspot
   change doesn't actually fix it. No PRD yet.

## Out of Scope (at this level)

- Exact config file schema (per-service or shared) — left to implementation.
- Public/remote exposure of any service UI (assume LAN/Tailscale-only, no
  Cloudflare Tunnel implied).
- Migrating the separate Coolify-hosted apps (photo server, bargain
  scraper, energy dashboard) into this architecture — kept deliberately
  separate; different lifecycle and exposure needs.
- Future idea (not in scope yet): the manager fetching/installing/updating
  service code via git as part of its own duties.
- Future idea (not in scope yet): per-service config validation. The
  manager's config editor only checks that `config.yaml` is syntactically
  valid YAML (see `manager-prd.md`) — it has no idea whether, say, a
  `drive` PID constant or a `camera` frame rate is a *sane* value for
  that service, and centralising that knowledge in the manager feels
  wrong (it would need to know every other service's internals). Raised
  while designing `drive`'s tuning-constants config (see
  `drive/drive-prd.md`). A plausible future shape: each service exposes
  its own config-validation (a callback, or a `--check-config` CLI flag),
  called by whoever cares — the manager before saving, a service at its
  own startup, or a standalone script — rather than one central
  validator trying to know everything. Not needed until a service
  actually has config values worth getting wrong in a way syntax
  checking can't catch.
- Future idea (not in scope yet, not needed for this project, but worth
  trying out): an AI page-configurator — the user describes what they
  want on a page (in a chat-style UI) and an LLM (e.g. via the Claude
  SDK) generates the HTML/CSS/JS for it, saved to disk as a normal
  static page thereafter. Motivation: most vendor visualisation tools
  (OXTS's included) are rigid and rarely show exactly what a user wants;
  a describe-it-and-get-it page builder sidesteps that, and the
  read-only/telemetry-display pages in this project are a low-risk place
  to prove it out. Requires internet access to call the API at
  generation time — not a new constraint in practice, since the robot
  already needs internet for NTRIP corrections, which tolerates flaky
  connections. Applies generically across any service's web UI, not just
  nav/telemetry data — could plausibly grow into a shared library usable
  outside this project too, if it works out. All pages for now are built
  by hand (well: by Claude
  Code, at dev time, following `ui-style.md`), with zero runtime AI
  dependency — this future idea is a separate, later feature, not a
  prerequisite for anything currently planned.

## Further Notes

Existing project context: GR6-v1 robot (Raspberry Pi 4 "amundsen" +
Arduino), Python throughout, OXTS xNAV650 over Ethernet for navigation,
ArUco markers for GNSS-poor areas. Existing repo (pre-restructure):
https://github.com/s7711/GR6-v1
