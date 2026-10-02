# GNSS EMI problems — working notes

Status: **open**, as of 2026-10-02 (Fri) evening. Temporary file: delete it,
or fold the conclusion into `oxts-nav-prd.md`'s "Interference diagnostics",
once the cause is found.

## The problem

While moving, the u-blox keeps reporting RTK **Integer** (mode 6, all 30–32
satellites) but its position is wrong — most likely an undetected cycle slip
or two. The xNAV sees GNSS disagree with the INS and rejects it, then its
recovery logic follows GNSS and the position steps 0.2–0.5 m, sometimes two
or three times in ~10 s (the u-blox re-fixing). On a path with 0.5 m
clearance that is an abort.

Signature in `oxts-nav/data/logs/*.jsonl` (2 Hz):

- `InnPosXFilt`/`InnPosYFilt` jump from ~0.3 to the 6.4 ceiling;
- `GnssPosReject` climbs to ~20, resets to 0 (the xNAV accepts GNSS — the
  position step), sometimes climbs again;
- `GnssPosMode` stays 6, `GnssDiffAge` stays 0–2 s, satellite count unchanged.

Events need **slow motion** (0.1–0.4 m/s). Never seen with the robot parked.

## Coordinates used below

N/E metres from the end of path "Patio start" (lat 52.23543, lon −1.46056).
"Boresight limits" in that frame: point 0 (+8.3, +8.2), point 1 (+5.7, −3.0),
point 2 (−1.2, −1.3), point 3 (+2.3, +10.2).

The **Segway Navimow GNSS base station** is very near point 1, ~(+5.7, −3.0).
Its radio band is unknown (EU 868 MHz suspected).

## Where events happen

Two clusters, both within ~12 m of the base station:

- **North strip**: (+4..+6, −4..+2), mostly heading west (~245–260°), i.e. the
  leg into point 1. Events 13:12, 13:14, 13:44, 16:43, 18:11.
- **Near point 2 / south-west**: (−3..0, −10..+1). Events 13:10, 14:47,
  14:49, 15:16, 15:19, 18:22, 18:26.

Rate against distance from the base station (2026-10-02, moving with GNSS in
use, excluding the INS-bias period and my own tests):

| Distance | Events | Moving time | Per 10 min |
|---|---|---|---|
| 0–6 m | 14 | 18 min | 7.7 |
| 6–12 m | 22 | 38 min | 5.8 |
| 12–20 m | 8 | 16 min | 4.9 |
| 20–40 m | 2 | 12 min | 1.7 |

Suggestive, not proof: the near and far time come mostly from different
activities (afternoon slow loops near, morning driving far).

## Ruled out (2026-10-02)

| Suspect | Test | Result |
|---|---|---|
| DC motors (needed?) | Robot pulled on cardboard, motors unplugged | Events still happen, so the motors aren't needed for them |
| Motors under load | Driven loops; core noise driving vs parked | Noise unchanged (39.4/43.1 vs 39.2/45.7); events at the same places as pulled |
| Motors free-running | On bricks: left, right, both, 0.2 and 0.35 m/s | No change in noise or innovations |
| Wifi transmission (RF) | Parked, 30 s UDP flood at 10 dBm (17 Mbit/s) and 20 dBm (29–42 Mbit/s) | No effect on innovations, sats, mode or core noise |
| Wifi reconnects | 5 forced reconnects parked, 4 while pulled (`wpa_cli` disconnect, 3 s, reconnect) | No effect |
| Corrections stall / burst | 10 s NTRIP drop at the Pi (iptables), then release | Integer held; a mild innovation bump (3.8, no rejects) while the corrections were ~10 s old; no backlog burst on release |
| Correction gaps before events | All 40 events | No more often than chance (12% vs 16% base rate) |
| Weak wifi | Event at 18:11 with a steady, strong link; clean pulled loops at 17:41 with weak wifi | Coincidental: the problem areas just happen to be weak-wifi areas |
| Antenna cables | Wiggled at the antenna and xNAV ends, parked | Nothing |
| Pump, camera, CPU load, dongle unplugged | Parked, 30–60 s each | No change |
| xNAV switching GNSS source | Mode around every event | 23 of 34 events were Integer throughout (no switch) |
| GNSS "no data" blips | 136 blips vs event times | Not associated |
| RTK engine | u-blox and the xNAV's internal (Gx) engine | Both fail: one receiver's raw data, two RTK algorithms |
| Dongle distance | Moved from ~20 cm to ~40 cm from the antennas | Not enough on its own |

Not yet tested: the ultrasonic sensors (hard to remove).

## Leading hypothesis

The **Navimow base station's radio** (or a spurious emission from it, or an
oscillating active GNSS antenna on it) disturbs the u-blox's tracking near
it, so carrier phase slips while moving slowly. Patchy rather than smooth
with distance, which ground and wall reflections, lobed spurious emissions,
or the robot's own antenna pattern (direction-dependence on the north strip)
could all explain. The Navimow itself is designed around that radio, so it
coping is not evidence against it.

Earlier idea, weakened: satellite geometry or time of day (clean pulled loops
at 17:41, but driven events at the same spot at 18:11).

## Next tests, in order

1. **Base station off/on**, driving "Boresight limits drive loop": a few loops
   with it on (done once: 18:26–18:31, clean; earlier loops 18:09–18:26 had
   events at 18:11, 18:22, 18:26), then a few with it **off** (wait 1–2 min
   after switching), then on again. Compare events near point 1 and point 2.
2. If confirmed, map events as distance and bearing from the base station, and
   find its radio band (FCC/CE ID on the label).
3. Different GNSS antennas (cables and holder made; needs fitting and the xNAV
   antenna position reconfigured). Before fitting them, do a few loops with
   the current antennas at the equivalent time of day, so a time effect
   can't be mistaken for an antenna effect.
4. Per-satellite data: the xNAV's own logging may hold the u-blox raw data
   (per-satellite C/N0, slip flags). That would show the exact satellites and
   moments.
5. Let ordinary missions collect evidence: `navigate/data/gnss_hold_log.jsonl`
   records every event (time, position, what was running).

## Tools and changes made for this (all committed 2026-10-02)

- **navigate GNSS-rejection hold**: `GnssPosReject >= 12` holds a path run
  (stopped, still "running"), and it resumes 30 s after the count returns to
  0. Config `gnss_hold_*`. The Run page shows "GNSS recovery". It catches
  events before the xNAV's first jump.
- **oxts-nav "EMI monitor" page**: speed, mode, filtered innovations, diff
  age, rejects and core noise, live.
- **Core noise logged**: `GpsPrimaryCoreNoise` / `GpsSecondaryCoreNoise` in
  oxts-nav's decoded log. Meaning on the u-blox is undocumented (an OXTS figure
  from the Novatel days). Parked ~36–41 / ~40–50. It drifts down slowly over
  hours, and a person near the robot raises it by 4–5. Nothing tested moved it.
- **Browser video** scaled to 640×480, quality 50 (`stream_width`,
  `stream_jpeg_quality` in the camera config).
- `network/wlan1-txpower.dispatcher.example`: a 10 dBm wifi cap, tried and
  **not installed**.

## Gotchas found along the way

- After restarting **camera**, restart **aruco** too: aruco stays attached to
  the old shared-memory frame buffer and serves frozen frames.
- `iw dev wlan1 disconnect` is silently ignored while wpa_supplicant owns the
  link; use `sudo wpa_cli -i wlan1 disconnect` / `reconnect`.
- `iw dev wlan1 set txpower` is ignored on rt2800usb; `iw phy phy1 set
  txpower fixed <mBm>` works.
- A bad accelerometer bias (AxBias ~0.2 m/s²) after repeated slips made the INS
  run away (17 m/s on a walking robot, 12:40). Reset the INS. The xNAV doesn't
  save biases across power cycles.
- The scanner was off all day because a network test wrote the live
  `scanner.json` (fixed); turned back on at ~15:30.
