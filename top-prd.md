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
  vision/
    prd.md
    ...
  oxts-nav/
    oxts-nav-prd.md
    ...
  path-follow/
    prd.md
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

1. Nav decode (xNAV650 data handling) — sorted out first; no PRD for this
   service yet, not being planned in detail at this stage.
2. Manager — needed early so services gain status/control visibility as
   they come online, rather than bolting it on last.
3. Vision/ArUco.
4. Path-following/motor control last (safety-critical — most confidence
   wanted before touching it).

## Out of Scope (at this level)

- Exact config file schema (per-service or shared) — left to implementation.
- Public/remote exposure of any service UI (assume LAN/Tailscale-only, no
  Cloudflare Tunnel implied).
- Migrating the separate Coolify-hosted apps (photo server, bargain
  scraper, energy dashboard) into this architecture — kept deliberately
  separate; different lifecycle and exposure needs.
- Future idea (not in scope yet): the manager fetching/installing/updating
  service code via git as part of its own duties.
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
