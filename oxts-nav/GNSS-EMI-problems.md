# GNSS EMI problems — working notes

Status: **parked until the chimney base station is installed**, as of
2026-10-08 (Thu). Current conclusion (see "Summary, 2026-10-08" at the
end): the 38 km baseline is the main factor, and a poor local GNSS
environment (point 2, evenings) is the trigger. Motors, wifi, the
Navimow radio and direction of travel are ruled out. Temporary file:
delete it, or fold the conclusion into `oxts-nav-prd.md`'s "Interference
diagnostics", once the base station has settled it. The sections below
are in date order; the early ones reflect what was believed then.

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

## Leading hypothesis (2026-10-02; superseded, see the summary at the end)

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

## 2026-10-05: geomagnetic storm, reset tests

The base station off/on test wasn't possible (the Navimow was mowing, so
its base station stayed on). The day turned into a different problem.

### What happened

- The robot had sat indoors in a window since Saturday. Outside, the INS
  ran away on its first path ("Patio pot 1", 15:28): INS speed up to 5x
  the wheel speed, position swinging ~2.8 m east-west in 12 s, GNSS
  (Integer, 32 sats) rejected. Same signature as Friday's 12:40 bias
  runaway. **After a long spell indoors, reset the xNAV before driving.**
- Before that, at the waterbutt (14:50–15:12), every GNSS re-enable after
  the aruco-priority approach was rejected, then accepted with a
  0.13–0.17 m step, always north-ish. Saturday's identical mission had no
  rejects at all.
- After the reset, the u-blox held an RTK Integer fix that was **1.6 m low
  and ~0.5 m east**: everything looked healthy, paths tracked, but marker
  10 at the waterbutt was out of view and altitude was 1.63 m below
  Saturday's in the same 20 cm cells. A genuinely wrong Integer fix, not
  a reference change.
- Stationary at Patio start (15:22:38), the u-blox's Integer altitude
  stepped −15 cm with no rejection (vertical innovation 4.8).
- Later, with the u-blox in Integer, the xNAV stopped GNSS updates
  altogether for ~10 min (raw position/velocity/heading innovations all
  flagged invalid, accuracy growing, `GnssPosReject` 0). Cause unknown.

### Geomagnetic storm

NOAA planetary Kp (3-hourly max): Mon 28 Sep – Fri 2 Oct 1.3–2.0 (quiet),
Sat 3 Oct 2.7, **Sun 4 Oct 5.7, Mon 5 Oct 5.3** (03:00 UTC), then 2.3–3.3
in the afternoon. Kp 5 = G1 storm. Disturbed ionosphere + the 38 km
baseline to OxTS's base (PolaRx5 near Bicester, mountpoint OXTS1) fits
wrong/wandering fixes better than anything local, and fits "fine on
Saturday, bad on Monday". Friday's EMI events were at Kp ~1, so they're a
separate problem. Kp: `services.swpc.noaa.gov/products/noaa-planetary-k-index.json`.
European TEC / ROTI maps: Royal Observatory of Belgium.

### Reset tests (robot stationary, markers out of view)

Repeated `!reset`, then record a 10 s median position once the xNAV is
settled on the u-blox (source mode 6, no rejects, filtered innovations
< 1.0 for 5 s).

| Run | Site | Integer cycles | Alt SD | Alt range | Outliers > 4 cm | Worst |
|---|---|---|---|---|---|---|
| Dual antenna, xNAV picks its own source | open sky, by marker 10 | 6 of 7 | 6.5 cm | 18 cm | — | — |
| Single antenna, INS initialised from the u-blox | open sky, by marker 10 | 20 of 20 | 2.4 cm | 11 cm | 4 | 7.5 cm up |
| Same | problem site, ~3.5 m from the Navimow base (+5.5, +0.3) | 20 of 20 | 5.7 cm | 26 cm | 6 | **21 cm up** |

- The good fixes are equally good at both sites: ±1–1.5 cm horizontal and
  vertical. Receiver, antenna and corrections work when the ambiguities
  are right.
- Near the Navimow base, fresh fixes are wrong more often and by more
  (two vertical outliers of 11 and 21 cm). Small sample, but the same
  direction as Friday's moving events. Not recorded whether the Navimow
  base was transmitting.
- With dual antenna the xNAV often initialises on its own Gx Float (mode
  23), overtrusts it, then fights the u-blox's Integer for up to ~1.5 min
  (innovations 6.3–6.4). Its recorded altitude then depends on how far
  it has come round: that, not the u-blox, made the first run scatter.
  A float solution absorbs ionospheric error into its ambiguities, so it
  probably suffers most in a storm.
- Single-antenna method: unplug the secondary antenna (no config change),
  wait for the xNAV's chosen source to be u-blox Integer, then send
  `!set init hea <deg>`: the INS starts on the u-blox's position.
- This spot's reference altitude is ~168.84 m (the 168.91 m read from the
  Gx engine with marker 10 in view was off). Marker 10's surveyed alt is
  168.926, but that's the marker centre, not the INS height.

### Things that don't work (yet)

- **UCOM message 33** ("Primary GNSS card measurements",
  `PGCApproxLat/Lon/Alt` = the u-blox antenna's own position) is in the
  stream once enabled in `mobile.dbu` (done: v1, `MessageEnabled: true`;
  original kept in the session scratchpad), but every value is NaN: per the
  UCOM manual that's a feature-code-locked signal. Needs a feature code
  from OxTS. UCOM itself flows on UDP 50487 with the existing
  `-udp5_ucom`.
- NCOM has no raw GNSS position: channels 12/57 are the antenna lever arm
  (GAP), not a measurement.
- The xNAV's `.rd` files on its FTP hold the raw u-blox data (one packet
  type per receiver; primary vs secondary undetermined; RD framing
  undocumented). Possible future reverse engineering.

### Decisions and next steps

- **Ordered:** ArduSimple simpleRTK2B Budget (ZED-F9P) + Budget Survey
  Tripleband antenna + 20 m RG58 SMA extender, for a chimney base station
  (<1 km baseline). Arrives while Ben is away; install after 20 Oct. F9P
  over UM980/X20P because it matches the rover's signals exactly (X20P
  lacks E5b/B2I; UM980 base risks GLONASS float on an F9P rover); the
  tripleband antenna is the future-proof part. Survey the chimney antenna
  against OxTS's corrections (on a quiet-Kp day) so recorded paths stay
  valid. Free interim option: RTK2go **MCHM01** (Rugby, 22 km), but its
  coordinates are owner-entered, so for event counting only.
- Repeat the reset test with a metal ground plane under the (currently
  bodged-on) antennas.
- The base station off/on test (above) is still to do.

## 2026-10-07: antennas and baseline (rain, Kp ~1)

Robot under a waterproof cover; wifi dongle back ~20 cm from the GNSS
antennas. Same single-antenna reset method (10 cycles each).

- **Wifi again, stationary:** 5 × (60 s quiet / 30 s flood at ~40 Mbit/s,
  20 dBm). No rejects, mode changes or satellite loss; core noise
  identical (36.4/41.6 quiet, 36.3/41.9 blasting). Small innovation
  blips (to ~0.4) a little more frequent during floods, but the biggest
  (1.0, 0.6) were in quiet windows and all activity faded over the run.
  Wifi position is not the problem.

| Run (Wed, rain) | Integer | Alt SD | Alt range | N / E SD | > 4 cm off |
|---|---|---|---|---|---|
| G5ANT, problem site, OXTS1 (38 km) | 10/10 | 10.2 cm | 28 cm | 6.5 / 1.3 cm | 8 |
| G5ANT, open sky, OXTS1 | **3/5** | **179 cm** | **3.45 m** | 41 / 106 cm | 2 (−2.5 m, +0.9 m) |
| Helix (primary only), open sky, OXTS1 | 10/10 | 8.5 cm | 31 cm | 6.3 / 4.1 cm | 8 |
| **Helix, open sky, RTK2go MCHM01 (22 km)** | 10/10 | **3.7 cm** | 13 cm | **2.6 / 0.7 cm** | **2** |

(Monday, dry, G5ANT, open sky, OXTS1: 2.4 cm SD, 4/20 > 4 cm.)

- The **AntCom G5ANT-1A198MNS1** (back of robot, fitted with no ground
  plane) failed badly in the rain: metre-level wrong Integer fixes,
  accepted by the xNAV. A patch antenna needs its ground plane.
- The **helix** removed the gross failures, but on OXTS1 was still
  ±8 cm.
- **Halving the baseline** (MCHM01, Rugby, 22 km), same antenna, same
  rain, 25 minutes later: 8 of 10 fixes within 4 cm, SD ~2–4 cm. Points
  at the long baseline as a major factor. Not A-B-A (sequential), so
  some time effect can't be excluded. MCHM01 sits ~20 cm lower than
  OXTS1 in absolute terms (its own coordinates), so not for missions.
- Even on a quiet ionosphere (Kp ~1), OXTS1 gave poor fixes today: the
  storm wasn't the whole story.

Config (2026-10-07): front helix geometry (`-blength0.368_0.02`,
`mobile.gap` 0.186 / 0.170 / −0.026), OXTS1 corrections. G5ANT geometry
for going back: blength 0.196 (commented out in `mobile.cfg`), gap
0.100 / 0.117 / 0.366.

Next: chimney base (<1 km), then compare antennas on a dry day (helix vs
G5ANT with a ground plane), then the Navimow radio test with driving
loops.

### Evening (2026-10-07)

- **RTK2go banned our IP** (13:06 UTC → Thu 8 Oct 13:06 UTC). Their
  report: the xNAV's OxTS NTRIP client makes an anonymous source-table
  request on every connect (22 "Table Only" connections, all counted as
  failed), and after a reset it sent a garbage GGA (2 sats, a position
  ~25 km away). Our part: the OxTS `mobile.cfg` was restored after the
  MCHM01 run without a reset, so the xNAV stayed on MCHM01 and later
  reconnected without a fix. If using RTK2go again: few connections,
  connect only once the xNAV has a fix, never reset indoors while on it,
  and restore + reset + check the mountpoint when done. The
  reset-per-cycle method can't be used with RTK2go.
- **Config changed by Ben (NAVconfig):** `-stat_delay5` / `-stat_speed1`
  commented out (stationary detection below 1 m/s, on a robot that
  drives at 0.1–0.5 m/s; a candidate for "worse while moving slowly");
  no-slip off (`-no_slip-1`, car-speed only); triggers / IO /
  `-top_speed` removed; GNSS time sources added. Kept: `-headlock`
  (useful), `-motion_speed1` (single-antenna initialisation only).
  Second helix fitted at the front: dual antenna, blength 0.368.
- **Stuck filter:** after a reset, u-blox Integer + dual-antenna heading
  fixed, but raw position innovations steady at −4.5 / −1.5 / −0.1 with
  `GnssPosReject` 0 and accuracy steady at ~0.24 m: the xNAV neither
  rejected nor used GNSS (~1.1 m disagreement). Cleared by
  `!disable gnss` then `!enable gnss`. Unexplained; a question for OxTS.

### Navimow radio on/off: not the cause (2026-10-07 evening)

"Boresight limits drive loop", driven repeatedly 18:08–19:50 (dusk into
dark), helix antennas, new config, OXTS1. Every loop `stopped_ok`, XTE
≤ 0.12 m; every event caught by the GNSS hold.

| Navimow base | Time | Loops | Holds | Per loop |
|---|---|---|---|---|
| On | 18:08–18:21 | 4 (1 partial) | 0 | 0 |
| **Off** | 18:21–19:29 | 18 | 12 | 0.67 |
| On | 19:31–19:46 | 4 | 3 | 0.75 |

- Events continue at the same rate with the base station off, including
  one 1.6 m from it (19:15:45, base off). **The radio is ruled out.**
- Events are tied to **place**: of 16 holds, 11 within ~2.6 m of point
  2 (−1.2, −1.3), 4 on the point 2 → point 3 leg around (+0.3, +3.5), 1
  by point 1. Same spots loop after loop.
- Next: what's around point 2 (sky view, walls, fences, metal)? Heading
  at each event; drive the loop in reverse to separate place from
  heading (antenna pattern / robot body).

### Gotchas

- The NCOM "Filt" innovations shown by oxts-nav are its own display
  filter (holds a big value, decays slowly). Check the raw `InnPosX/Y/Z`:
  absent means the xNAV didn't do that update at all. Since 2026-10-08 the
  filtered value is dropped after 2 s with no valid update (before that it
  froze at its last value, e.g. across a reset).
- `!set init hea` (not `ini`).
- The EMI monitor page left open over a weekend grew the browser tab to
  500 MB+ (the page's own data is bounded; ~53 MB fresh). Close it when
  done.

## 2026-10-08: direction vs time of day; summary

Dry, Kp 1.0–1.7. Helix antennas, Ben's new `mobile.cfg`, OXTS1.

- **Stationary relock, dual antenna, ~4.4 m from the Navimow base**
  (normal operation, 7 cycles): 4 of 7 had no u-blox Integer within 60 s
  of initialising; INS initialisation took 80–241 s; cycle 3 fought for
  **228 s** (innovation 6.4); cycle 5 settled "cleanly" on a fix
  **79 cm low**. RD files for OxTS: `261008_080405.rd` (the fight),
  `261008_081121.rd` (the wrong fix).
- **Driving, both directions, morning:**

| Block | Direction | Loops | Holds | Loops with any rejection |
|---|---|---|---|---|
| Wed 18:08–19:50 | anticlockwise | 31 | 16 | 17 |
| Thu 09:32–10:02 | clockwise (new path "Boresight limits drive loop clockwise") | 9 | 0 | 0 |
| Thu 10:08–10:37 | anticlockwise | 9 | 0 | 1 (count 4, no hold) |

  Direction isn't it; the same loop is clean in the morning and bad in
  the evening. Most likely satellite geometry: in the evening, low
  satellites reflect off something near point 2. To confirm: repeat the
  evening loops ~4 min earlier per day (sidereal repeat).

### Summary, 2026-10-08

| Suspect | Verdict |
|---|---|
| Motors / EMI | Not the cause (events pulled with motors unplugged; noise unchanged under load) |
| Wifi (power, position, reconnects) | Not the cause (repeated 2026-10-07 with the dongle 20 cm away) |
| Navimow base radio | Not the cause (same rate on/off; an event 1.6 m from it while off) |
| Direction of travel | Not the cause |
| Ionosphere | Makes it worse (Monday's storm), but problems at Kp 1 too |
| G5ANT without ground plane | Made it much worse in rain; helix fixed that |
| `-stat_speed1` (stationary below 1 m/s) | Probably unhelpful; removed |
| **38 km baseline** | **Main factor**: 22 km cut the scatter ~2/3 |
| **Site + time of day** | **Trigger**: point 2 in the evening |

Waiting for the chimney base (F9P + tripleband, ordered 2026-10-05).
Meanwhile the GNSS hold catches every event (no path aborts this week).
Ben is reporting to OxTS the xNAV holding a constant INS position while
it neither uses nor rejects GNSS (raw innovations steady, e.g. −4.5σ, with
`GnssPosReject` 0; possibly GNSS velocity holding it), with the RD
files. `mobile.cfg` may be put back to a standard version.

**u-blox firmware ruled out (2026-10-08 afternoon).** The 22 July
xNAV update had moved the receivers from HPG 1.13 to HPG 1.50 (OxTS's
setup `.cfg` files for both versions have the same key settings). Flashed
back to HPG 1.13 (confirmed in the RD files) and repeated both tests:

| Test | HPG 1.50 | HPG 1.13 |
|---|---|---|
| Static dual-antenna resets near the Navimow base (settled / no Integer in 60 s / fights) | am: 4 / 3 / 1 of 7 | pm: 4 / 5 / 4 of 10, plus 1 never accepted |
| Boresight loop, holds per loop | Wed eve 0.67; Thu am 0 | Thu pm 0.33 (4 in 12, all on the point 2 → 3 leg, both directions) |

1.13 fixed more slowly (fewer usable signals) and fought more; wrong fixes
on both. Back on HPG 1.50 (flashed 13:53, `261008_125513.rd` confirms).

Tools added 2026-10-08: `oxts-nav/tools/reset_cycle.py` (the reset
test); the oxts-nav **xNAV Logs** page (lists the xNAV's `.rd` files,
newest first, and streams one to the browser).
