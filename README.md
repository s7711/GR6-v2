# GR6-v2

**GR6** ("amundsen") is a two-wheeled (plus casters) robot for watering
plants: a Raspberry Pi 4, an OXTS xNAV650 for navigation, a Pi camera
with ArUco markers for GNSS-poor areas, a Raspberry Pi Pico (MicroPython + TB6612FNG driver) handling motor control, wheel encoders, and the
water pump, plus ultrasonic sensors and a water butt with its own
ESP8266 pinch-valve controller.

It's built as several independent services (vision, nav decode, path
following, mission sequencing, a manager UI, ...), each its own
systemd-managed process, communicating over well-defined IPC — see
`top-prd.md` for why and how.

> ⚠️ This project is designed for a one-off robot and may not run as-is
> on other hardware.

## Status

In active development, field-tested on the real robot. Fourteen
services exist so far:

- **`manager`** — a home-screen-style launcher: icon-grid to jump to
  each service's own web UI, a services table (status/start/stop/
  restart/journal), and a plain-text editor for the shared config file.
- **`hello`** — a minimal example child service, mostly there to prove
  out the shared config/IPC/web conventions for whatever service comes
  next.
- **`oxts-nav`** — decodes the xNAV650's NCOM stream and serves live
  nav/status/connection data over a websocket, with web pages for a
  live dashboard, connection diagnostics, full status detail, sending
  ad-hoc commands to the xNAV650, and viewing its downloaded config
  files. Persistently logs raw NCOM and decoded fields.
- **`camera`** — captures frames from the Pi camera and publishes them
  (shared memory) for other services to consume, with a live preview
  page and a full checkerboard-based calibration procedure.
- **`aruco`** — detects ArUco markers in the camera's frames and sends
  known markers' positions/heading to the xNAV650 as GAD aiding updates,
  for GNSS-poor spots. Pages for a live view, a vehicle-centred plan-view
  map, surveying a new marker from a live detection, and managing the
  marker list. Field-validated outdoors with real RTK/NTRIP corrections.
- **`drive`** — talks to the Pico motor controller over USB serial:
  motor velocity, water pump, ultrasonic ranges, encoder/PID telemetry.
  Pages for manual jog control, live PID tuning (with scrolling graphs),
  an ultrasonic sensor diagram, and config. Publishes a `drive_feed`
  Unix socket for other services (`navigate`, `jobs`, `wheelspeed`) to
  consume. See `drive/drive-prd.md`.
- **`navigate`** — records a path by driving it once ("drop point"
  button, per-point speed/pump/clearance), then drives it back
  autonomously with a pure-pursuit controller against `drive`'s
  `/command/auto`, aborting cleanly on a per-segment clearance breach,
  poor GPS accuracy, a stalled/stuck robot, or a lost nav feed rather
  than one fixed global tolerance. Pages for running a saved path (with
  a live map), recording a new one, and managing saved paths. See
  `navigate/navigate-prd.md`. Field-proven repeatedly on real watering
  runs.
- **`network`** — a thin web UI over NetworkManager (`nmcli`) for
  amundsen's network interfaces: DHCP/static, wifi client/hotspot/
  scanner, per-BSSID signal-strength logging, and a per-interface
  "recovery" safety net (an unconfirmed change auto-reverts) so a bad
  wifi/IP edit doesn't require physical access to undo.
- **`wheelspeed`** — sends per-wheel GAD speed/velocity aiding updates to
  the xNAV650 from `drive`'s filtered wheel-velocity telemetry, with
  both `GadSpeed` and `GadVelocity` message types implemented,
  config-selectable, plus a live-togglable on/off switch and a
  log-only scale-factor estimator comparing wheel odometry against the
  INS. A page shows a live chart/numeric comparison of each wheel's
  speed against the INS's own forward body-frame velocity. See
  `wheelspeed/wheelspeed-prd.md`.
- **`waterbutt`** — a duration-based "Go"/"Stop" page for the water
  butt's own ESP8266 pinch-valve controller, plus a GNSS-independent
  QC check (using a surveyed ArUco marker) that the robot is actually
  lined up with the funnel before allowing a fill, and a tank-level
  estimate (inferred from cumulative pump-on time, no level sensor
  fitted) that refuses a fill it doesn't believe will do anything
  useful. See `waterbutt/waterbutt-prd.md`.
- **`jobs`** — sequences saved `navigate` paths (plus pause/water/fill/
  turn-to-heading steps) into one job — one bed, one visit — driving
  `navigate`'s own HTTP API exactly as a browser would. Checks at save
  time whether each pair of consecutive paths lines up within
  `navigate`'s own entry tolerance. See `jobs/jobs-prd.md`.
- **`missions`** — sequences saved `jobs` into a full watering round
  (e.g. "water the whole garden"), the outer wrapper `jobs` doesn't
  itself provide. Field-proven completing a full multi-job round
  end-to-end. See `missions/missions-prd.md`.
- **`map-manager`** — builds and persists a single, garden-wide
  occupancy grid from the ultrasonic sensors and live GNSS/INS pose,
  gated on accuracy thresholds — a first step towards route planning
  beyond simply retracing a recorded path. See
  `map-manager/map-manager-prd.md`.
- **`viewer`** — a generic cross-app log browser/plotter: auto-discovers
  any service's `data/logs/` folder with no per-app registration,
  overlays quantities from multiple files/services on one chart. The
  primary tool for diagnosing a run after the fact. See
  `viewer/viewer-prd.md`.

See "Suggested migration order" in `top-prd.md` for the order these
were built in and what's still to come (`safety`, built on top of
`navigate`/`jobs`; camera/ArUco bore-sight calibration; a route planner
that reads `map-manager`'s grid).

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
`.service.example` → `/etc/systemd/system/` install step); only
`jobs`, `missions`, and `waterbutt` are enabled to start automatically
on boot, everything else is started by hand. For development/debugging,
stop a service's unit and run it directly in a terminal instead — every
service is a standalone Flask app underneath:

```bash
python manager/app.py
python hello/app.py
python oxts-nav/app.py
python camera/app.py
python aruco/app.py
python drive/app.py
python navigate/app.py
python network/app.py
python wheelspeed/app.py
python waterbutt/app.py
python jobs/app.py
python missions/app.py
python map-manager/app.py
python viewer/app.py
```

Then visit the manager's home page (port 8000 by default) to reach
everything else.

## Prior art

This project began as a from-scratch restructure of an earlier
single-process robot control program
([GR6-v1](https://github.com/s7711/GR6-v1)) into independent services
communicating over well-defined IPC. The xNAV650 NCOM decoder
(`oxts-nav/ncomrx.py`, `ncomrx_thread.py`) is carried over from GR6-v1
largely unchanged — mature, already correct, not worth recoding. See
`oxts-nav-prd.md` for what was reused vs. rewritten and why.

## License

MIT — see `LICENSE`. Third-party assets (CDN libraries, icons) are
listed in `THIRD_PARTY.md`.
