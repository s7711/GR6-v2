# GR6-v2

**GR6** is a two-wheeled (plus casters) robot for watering plants: a
Raspberry Pi 4, an OXTS xNAV650 for navigation, a Pi camera with ArUco
markers for GNSS-poor areas, and an Arduino (soon a Raspberry Pi Pico)
handling the motor controller, wheel encoders, water pump, and ultrasonic
sensors.

**GR6-v2** is a from-scratch restructure of the previous single-process
robot control program
([GR6-v1](https://github.com/s7711/GR6-v1)) into several independent
services (vision, nav decode, path following, a manager UI, ...), each
its own systemd-managed process, communicating over well-defined IPC —
see `top-prd.md` for why and how.

> ⚠️ This project is designed for a one-off robot and may not run as-is
> on other hardware.

## Status

Early days. Nine of the planned services exist so far:

- **`oxts-nav`** — decodes the xNAV650's NCOM stream and serves live
  nav/status/connection data over a websocket, with web pages for a
  live dashboard, connection diagnostics, full status detail, sending
  ad-hoc commands to the xNAV650, and viewing its downloaded config
  files.
- **`camera`** — captures frames from the Pi camera and publishes them
  (shared memory) for other services to consume, with a live preview
  page and a full checkerboard-based calibration procedure (guided
  capture across ~33 poses, `cv2.calibrateCamera()`, promote-to-active).
- **`aruco`** — detects ArUco markers in the camera's frames and sends
  known markers' positions/heading to the xNAV650 as GAD aiding updates,
  for GNSS-poor spots. Pages for a live view, a vehicle-centred plan-view
  map, surveying a new marker from a live detection, and managing the
  marker list. Field-validated outdoors with real RTK/NTRIP corrections.
- **`drive`** — talks to the motor-controller microcontroller (Arduino,
  moving to a Raspberry Pi Pico) over USB serial: motor velocity, water
  pump, ultrasonic ranges, encoder/PID telemetry. Pages for manual jog
  control, live PID tuning (with scrolling graphs), an ultrasonic
  sensor diagram, and config. Publishes a `drive_feed` Unix
  socket for future services (`navigate`, `jobs`, a wheelspeed-GAD
  sender) to consume. See `drive/drive-prd.md`.
- **`navigate`** — records a path by driving it once ("drop point"
  button, per-point speed/pump/clearance), then drives it back
  autonomously with a pure-pursuit controller against `drive`'s
  `/command/auto`, aborting cleanly on a per-segment clearance breach or
  poor GPS accuracy rather than one fixed global tolerance. Pages for
  running a saved path (with a live map), recording a new one, and
  managing saved paths. See `navigate/navigate-prd.md`. **Field-proven
  (2026-07-30)**: a real autonomous run (`Waterstablebed`, 29 points,
  30.2m) completed end-to-end with no abort, watering a real bed.
- **`manager`** — a home-screen-style launcher: icon-grid to jump to
  each service's own web UI, a services table (status/start/stop/
  restart/journal), and a plain-text editor for the shared config file.
- **`hello`** — a minimal example child service, mostly there to prove
  out the shared config/IPC/web conventions for whatever service comes
  next.
- **`network`** — a thin web UI over NetworkManager (`nmcli`) for
  amundsen's network interfaces: DHCP/static, wifi client/hotspot, and
  a per-interface "recovery" safety net (an unconfirmed change
  auto-reverts) so a bad wifi/IP edit doesn't require physical access
  to undo. **Built but only partly field-tested (2026-07-26)**: the
  external wifi dongle's client-mode config and the recovery/
  auto-revert mechanism are confirmed working live; the onboard wifi
  chip's hotspot mode and `eth0`'s static/DHCP switching are built but
  deliberately untested so far, per the safety-first order in
  `network/network-prd.md`.
- **`wheelspeed`** — sends per-wheel GAD speed/velocity aiding updates to
  the xNAV650 from `drive`'s filtered wheel-velocity telemetry (ported
  from GR6-v1's `gad_wheelspeed.py`), with both `GadSpeed` and
  `GadVelocity` message types implemented, config-selectable. A page
  shows a live chart/numeric comparison of each wheel's speed against
  the INS's own forward body-frame velocity. GAD timing uses the real
  arrival time of `drive`'s telemetry (not poll time) plus the midpoint
  of the averaging interval — see `wheelspeed/wheelspeed-prd.md`.
  **Running live with real measured lever arms, confirmed correct while
  stationary — not yet field-tested with the wheels actually turning.**
- **`waterbutt`** — a standalone duration-based "Go"/"Stop" page for the
  water butt's own ESP8266 pinch-valve controller (a separate
  microcontroller on the same wifi network, not part of the Pi). The
  firmware only exposes plain `/open`/`/close` with a 5-second
  fail-safe auto-close; this service holds the valve open for an
  operator-chosen duration by re-issuing `/open` before that fail-safe
  fires, then `/close`s at the end — see `waterbutt/waterbutt-prd.md`.
- **`jobs`** — sequences saved `navigate` paths (a YAML step list,
  interpreted by a small state machine rather than a scripting
  language), driving `navigate`'s own HTTP API exactly as a browser
  would. The first capability this project has needed with no GR6-v1
  equivalent to port from. Checks at save time whether each pair of
  consecutive paths lines up within `navigate`'s own entry tolerance,
  flagging (not blocking) any gap — see `jobs/jobs-prd.md`.

Nav decode, then the manager, were deliberately tackled first, then
camera, then aruco, then drive, then navigate, then network, then
wheelspeed, then waterbutt, then jobs — see "Suggested migration
order" in `top-prd.md` for the reasoning and what's still to come
(`safety` built on top of `navigate`/`jobs`).

## Architecture

Start with `top-prd.md` — the umbrella document for process model, IPC
choices, shared config, and why this is one monorepo rather than one
repo per service. `ui-style.md` covers the shared look/feel (Bootstrap,
colour semantics, the page registry pattern, real-time graphs) that
every service's web UI follows. Each service then has its own PRD
alongside its code (e.g. `manager/manager-prd.md`,
`oxts-nav/oxts-nav-prd.md`).

## Getting started

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

cp config.yaml.example config.yaml
# edit config.yaml — at minimum, set xnav_ip to the real xNAV650 address
```

Each service normally runs as a systemd unit, managed from the
manager's Services page (see `manager/manual.md` for the one-time
`.service.example` → `/etc/systemd/system/` install step). For
development/debugging, stop a service's unit and run it directly in a
terminal instead — every service is a standalone Flask app underneath:

```bash
python manager/app.py
python oxts-nav/app.py
python camera/app.py
python aruco/app.py
python drive/app.py
python navigate/app.py
python network/app.py
python wheelspeed/app.py
python waterbutt/app.py
python jobs/app.py
```

Then visit the manager's home page (port 8000 by default) to reach
everything else.

## Prior art

The xNAV650 NCOM decoder (`oxts-nav/ncomrx.py`, `ncomrx_thread.py`) is
carried over from GR6-v1 largely unchanged — mature, already correct,
not worth recoding. See `oxts-nav-prd.md` for what was reused vs.
rewritten and why.

## License

MIT — see `LICENSE`. Third-party assets (CDN libraries, icons) are
listed in `THIRD_PARTY.md`.
