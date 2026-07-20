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

Early days. Six of the planned services exist so far:

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
  socket for future services (`navigate`, `missions`, a wheelspeed-GAD
  sender) to consume. See `drive/drive-prd.md`.
- **`manager`** — a home-screen-style launcher: icon-grid to jump to
  each service's own web UI, a services table (status/start/stop/
  restart/journal), and a plain-text editor for the shared config file.
- **`hello`** — a minimal example child service, mostly there to prove
  out the shared config/IPC/web conventions for whatever service comes
  next.

Nav decode, then the manager, were deliberately tackled first, then
camera, then aruco, then drive — see "Suggested migration order" in
`top-prd.md` for the reasoning and what's still to come (`navigate`/
`missions`/`safety` built on top of `drive`, wheelspeed GAD aiding, and
a network-sharing service to give the xNAV650 internet access for
NTRIP).

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

Each service is a standalone Flask app for now (systemd units come
later — see the `.service.example` files in each service's folder).
Run whichever you need in its own terminal:

```bash
python manager/app.py
python oxts-nav/app.py
python camera/app.py
python aruco/app.py
python drive/app.py
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
