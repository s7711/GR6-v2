# Manager — Setup & User Manual

## What this is

The `manager` service is the home screen for the robot: an icon grid to
jump to each service's web UI, a Services table to start/stop/restart
each one and view its log, and a plain-text editor for the shared
`config.yaml`. See `manager-prd.md` for the design background — this
document is just "how do I get it running and use it."

## One-time setup (per service, per machine)

Systemd needs to be told a service exists before the manager (or you)
can start/stop/restart it. This is a one-off step per service, done
once from a terminal with `sudo` — after this, day-to-day start/stop/
restart/journal-viewing all happen through the manager's web UI with no
further terminal use needed for that service.

All five services have a `.service.example` file now (`manager`,
`hello`, `oxts-nav`, `camera`, `aruco`). For each one:

```bash
sudo cp manager/robot-manager.service.example /etc/systemd/system/robot-manager.service
sudo cp hello/robot-hello.service.example /etc/systemd/system/robot-hello.service
sudo cp oxts-nav/robot-oxts-nav.service.example /etc/systemd/system/robot-oxts-nav.service
sudo cp camera/robot-camera.service.example /etc/systemd/system/robot-camera.service
sudo cp aruco/robot-aruco.service.example /etc/systemd/system/robot-aruco.service
sudo systemctl daemon-reload
```

`daemon-reload` tells systemd to (re-)read unit files — needed once
after copying, and again any time a `.service` file's *contents* change
later (editing code inside a service doesn't need this, only editing
the unit file itself does).

Also install the sudoers rule that lets the manager's own web UI issue
passwordless `start`/`stop`/`restart` — scoped to `robot-*` units only,
nothing broader:

```bash
sudo visudo -f /etc/sudoers.d/robot-manager
# paste in the contents of manager/sudoers-robot-manager.example, save, exit
```

**We deliberately do not `systemctl enable` any of these units** — that
would make them start automatically at boot. For now, everything starts
manually (see below), so a boot doesn't silently bring the robot to
life unattended while it's still under active development. Revisit this
once the project is further along and unattended startup is actually
wanted.

## Getting going (every time, after a reboot or fresh terminal)

Since nothing is enabled at boot, start the manager itself once, by
hand:

```bash
sudo systemctl start robot-manager
```

Then open the manager's web UI (port 8000 — e.g. `http://amundsen:8000`)
and use the Services page to start each of `oxts-nav`, `camera`, `aruco`,
`hello` in turn — no more terminal needed from here. This is a manual
step each session for now, by choice; automating that startup sequence
is a "later" improvement, not done yet.

To stop the manager itself (it can't restart/stop itself cleanly from
its own UI — see "Debugging" below):

```bash
sudo systemctl stop robot-manager
```

## Services page

- **Start / Restart / Stop** — calls `systemctl <action>` on that
  service's unit.
- **Status badge** — green (running), red (failed — i.e. it crashed and
  didn't come back, since these units run with `Restart=no`; you'll
  need to hit Restart yourself once you've looked at why), grey
  (stopped).
- **Journal** — opens a page showing the last 200 lines of that
  service's systemd journal (`journalctl -u <unit>`). It's a live
  system log, not something this app manages storage for — systemd
  handles its own retention/rotation in the background, so there's no
  "clear" button here on purpose.

## Debugging a service

If you're actively chasing a bug in one service, running it directly in
a terminal (`python <service>/app.py`) is still often nicer than the
systemd loop — you get breakpoints, instant Ctrl+C, and prints as they
happen, rather than edit → restart-via-button → refresh journal page →
repeat. Stop that service's systemd unit first (`sudo systemctl stop
robot-<name>`, or the Stop button) so the two don't fight over the same
port, then run it by hand; restart the unit when you're done.

## Config page

A plain-text editor for the shared `config.yaml` — saves a timestamped
backup to `manager/config-backup/` before overwriting, and refuses to
save if the YAML doesn't parse.
