# PRD: Manager Service

See `../top-prd.md` for overall architecture decisions (process model, IPC,
shared config, systemd, version control) — this document only covers the
manager app itself and assumes those decisions as given.

## Problem Statement

With each robot responsibility split into an independent systemd-managed
service, the user needs a simple way to see which services are running and
to start/stop/restart them and reach each one's own web UI — without
remembering hostnames/ports/systemctl commands, and without adopting a
general-purpose tool (e.g. Cockpit) that manages everything on the box
rather than just these services.

## Solution

A small Flask web app with two pages:

- **Home page** — an Android-homescreen-style icon grid, one icon-link per
  `robot-*` service that has a web UI, taking the user straight to that
  service's own page. Lightweight status (e.g. a colour/badge on the icon)
  may be layered on, but the primary job of this page is fast navigation,
  not detail.
- **Services page** — a detailed table: one row per `robot-*` service, with
  status (running/stopped/failed), start/stop/restart controls, and
  resource usage (CPU/memory). This is where the operator goes to actually
  manage services, as opposed to just jumping to one.
- **Config page** — a plain text editor for the shared `config.yaml`.
  One file, one source of truth (per `top-prd.md`), so one editor here
  beats every service growing its own config-editing UI.

The manager controls services via `systemctl` only — it never spawns or
owns service processes directly. Runs on a normal high port (not 80); no
requirement to bind a privileged port.

## User Stories

1. As the robot operator, I want a home page of icon-links to each service's web UI, so that I can jump straight to any service without remembering hostnames/ports.
2. As the robot operator, I want a services page listing all robot services with whether each is running, so that I can tell at a glance if something has crashed or hasn't started.
3. As the robot operator, I want to start, stop, and restart any individual service from the services page, so that I don't need to SSH in and remember `systemctl` commands.
4. As the robot operator, I want the manager to keep working even if a service is crashed or misbehaving, so that one broken service doesn't take down my ability to see/control the others.
5. As the robot operator, I want the manager itself to be able to crash, restart, or be redeployed without affecting whether the robot's actual control services are running, so that the manager is never a single point of failure.
6. As a developer, I want status checks to be lightweight and near-real-time (e.g. via websocket push), so that the manager reflects reality within a second or two without hammering `systemctl`.
7. As the robot operator, I want the manager scoped only to robot-related services (`robot-*`), so that it doesn't expose or risk affecting unrelated system services.
8. As a developer, I want the manager to run with minimum privilege (not root, not blanket sudo), so that a bug in the manager can't affect the wider system.

## Implementation Decisions

- **UI model:** two pages, see Solution above — a home icon-grid for
  navigation, and a services table for status/control/detail. Both are
  driven by the same underlying service list; the table additionally needs
  status (running / stopped / failed — systemd already distinguishes
  stopped from failed, worth surfacing that distinction rather than
  collapsing to a boolean) and CPU/memory usage per service.
- **Home page icons:** each service's folder may provide an icon named
  `icon.<ext>` (e.g. `icon.svg` or `icon.png`) at a conventional path
  within its folder. Lookup is format-agnostic — the manager looks for
  any supported extension rather than assuming one, so a hand-coded SVG
  placeholder today can be swapped for a nicer bitmap icon later (e.g.
  AI-generated) just by replacing the file, no code change needed. A
  service without any icon file gets a blank box/circle placeholder rather than
  failing to render — icon presence is expected, not required, to avoid
  the whole home page breaking because one service forgot an icon.
- **Resource usage:** CPU/memory per service for the services-page table —
  source TBD (e.g. `systemctl show <unit> -p CPUUsageNSec,MemoryCurrent`,
  or reading `/proc` for the unit's main PID). Needs a decision on
  polling cost vs freshness, same as status retrieval below.
- **Status retrieval:** `systemctl is-active <unit>` (or `systemctl
  list-units` parsed once for all `robot-*` units), polled server-side and
  pushed to the browser over a websocket for near-live updates — reuses the
  websocket approach already used for robot telemetry.
- **Start/stop/restart:** shells out to `systemctl start/stop/restart
  <unit>`. Must not use `subprocess.Popen` or similar to run services
  directly — see top-prd.md for why (lifecycle coupling, blast radius).
- **Privilege:** scoped `sudoers` rule permitting the manager's user to run
  `systemctl {start,stop,restart} robot-*` without a password; nothing
  broader. Do not run the Flask app as root. Runs on a normal (non-
  privileged) port — no port-80 requirement, so no need for `setcap`,
  a reverse proxy, or socket activation to get there.
- **Service list & web-UI ports source:** built from the shared config file
  (service name → host/port), not a separately maintained hardcoded list,
  and not passed to child services at process-start time by the manager —
  the manager never starts child processes (systemd does, independently;
  see Process model in top-prd.md), so there is no such hand-off moment.
  Each service reads its own host/port for its web UI from the same shared
  config file at its own startup (per top-prd.md's "Shared configuration"
  decision); the manager reads the same file to know where to link/poll.
  This keeps the mapping in one place instead of two.
- **Config editor:** plain textarea showing the raw `config.yaml`
  contents, a Save button, nothing fancier (no per-field form, no schema
  UI) — this is a first cut, not a config management product.
  - **Validate before writing:** parse the submitted text as YAML before
    saving; reject (with an error shown to the user) rather than writing
    something that breaks every service's config load on next restart.
  - **Back up before overwrite:** copy the existing `config.yaml` to a
    timestamped backup (same pattern as GR6-v1's xNAV config backups)
    before writing the new version, so a bad edit is recoverable.
  - **No live reload:** services read config at startup only (per
    top-prd.md's "Shared configuration" decision) — saving here does not
    affect already-running services. The page should say so explicitly,
    so the operator knows to restart affected services from the
    Services page afterwards.
- Explicitly out of scope for the manager: general system administration
  (networking, users, storage, non-robot services).

## Testing Decisions

- Test the Flask routes for start/stop/restart against a couple of
  dummy/test systemd units (not the real robot services), to confirm
  correct `systemctl` invocation and status parsing without needing real
  hardware.
- Prefer testing observable behaviour (does the tile show "running" after
  starting the unit) over internal implementation details. No existing test
  prior art in this codebase yet — this establishes the pattern.

## Out of Scope

- Authentication/authorization on the manager UI (assume trusted LAN,
  single user, for now).
- Fetching/installing/updating service repos via git (future idea, not
  this PRD).
- Home-page "widgets" beyond service icons — e.g. wifi strength, battery %,
  other at-a-glance robot status (future idea, not this PRD).
- Public/remote exposure (LAN/Tailscale-only, consistent with other
  self-hosted services; no Cloudflare Tunnel implied here).

## Further Notes

During development, individual services will often be stopped via
`systemctl stop robot-<service>` and run directly under the VS Code
debugger instead. The manager's tile for that service is expected to show
"stopped" in this situation — that's normal, not a bug.
