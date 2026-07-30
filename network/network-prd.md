# PRD: Network Service

## Problem Statement

Wifi is the last real blocker identified in `top-prd.md` (item 7) —
`navigate`'s path-following made the pain concrete, and Ben's own
testing shows outdoor signal is often marginal (usually 2-3/5, briefly
touching 4/5, never 5/5), which will only get worse further from the
house once real watering runs start. Separately, `top-prd.md` item 5
(network sharing: wifi → ethernet internet access for the xNAV650's
NTRIP corrections) needs a static IP on `eth0` and a persistent
MASQUERADE/FORWARD setup — today that's a hand-written iptables script
that resets on every reboot, not a managed service.

Both of these are instances of the same underlying gap: there's no way
to configure or reason about amundsen's network interfaces except by
hand, over SSH, with `nmcli`. This service is that missing piece — one
place to configure every interface (wifi client, wifi hotspot, wired),
see which one currently has internet access, and set up routing/sharing
between them.

Ben already has a new external USB wifi adapter (chosen for better
roaming between access points as the robot moves around outside) ready
to fit, and wants to improve outdoor wifi before creating paths and
watering in earnest — this PRD exists to unblock building that today.

## Solution

A new service, `network`, following this project's usual shape (Flask
+ shared header/template, `config.yaml`-driven, systemd unit) — but
unlike every other service so far, its whole job is orchestrating
**NetworkManager** (`nmcli`) rather than owning any state of its own.
NetworkManager is already what's actually managing every interface on
amundsen today (confirmed live: `wlan0` and `eth0` are both
NetworkManager-managed connections already, `eth0`'s existing static IP
for the xNAV650 link is an `ipv4.method: manual` NM connection profile)
— so this service is a UI on top of an already-authoritative mechanism,
not a second source of truth to keep in sync with it.

### Why NetworkManager, not hand-rolled tools

`nmcli` already does everything this PRD needs, natively, and
persists it across reboots (directly fixing item 5's "iptables resets
on every reboot" problem):

- Per-interface DHCP vs static (`ipv4.method: auto` vs `manual` +
  `ipv4.addresses`/`gateway`/`dns`).
- Wifi client config (SSID/PSK) — `802-11-wireless-security.psk`.
  Confirmed live: amundsen's onboard chip (`wlan0`) already supports AP
  mode alongside its normal client mode (`iw list`'s "Supported
  interface modes" includes `AP`, not just `managed`), so the hotspot
  plan below is viable on the existing hardware, no new chip needed for
  that half.
- Wifi **hotspot**, including its own built-in DHCP server for
  connecting clients — an NM connection with `802-11-wireless.mode: ap`
  plus `ipv4.method: shared` runs its own dnsmasq instance automatically.
- **Internet sharing/routing** between interfaces — the same
  `ipv4.method: shared` mechanism NATs and forwards from whichever
  interface needs the connection to whichever has it, without a
  hand-written iptables script at all. This should let item 5's
  wifi→`eth0` NTRIP sharing be expressed as ordinary NM config on
  `eth0` (sharing wlan0/wlan1's internet connection), rather than a
  separate service.

So `network`'s job is: read `nmcli`'s current state and present it
nicely, and write new connection profiles / bring them up when the
operator changes something — a thin, honest wrapper, same spirit as
`manager` wrapping `systemctl` rather than reimplementing service
supervision.

### Hardware layout (as planned, not yet fitted at PRD-write-time)

- **Onboard wifi chip (`wlan0`)** — dedicated to **hotspot** duty
  ("Coffeebean" network unreachable → fall back to being an access
  point of its own, so a phone/tablet can connect directly), *and*
  doubles as the trusted **recovery** path (see below), since it's
  soldered onto the board and can't be physically disconnected the way
  a USB dongle can.
- **External USB wifi dongle (new hardware, better at roaming between
  access points, mounted outside the buried Pi/near-antenna
  electronics)** — the normal day-to-day wifi **client**, connecting to
  whichever real access point is in range.
- **`eth0`** — unchanged, static IP, xNAV650 link. `network` can still
  edit it like any other interface (see "Recovery" below on why this
  isn't specially protected), but nothing about its normal operation
  changes because of this service.

A single wifi radio can't be both an AP and a client at the same time —
this is *why* the hotspot and roaming-client roles are split across two
different radios, not a UI restriction `network` invents.

### Page shape (v2 — see "v1 → v2" below for why this changed)

One page, one card per interface (`wlan0`, `wlan1`, `eth0`, ...). No
inline "edit this interface's settings" form anymore — a card only
ever *selects* an already-existing NetworkManager connection profile,
never edits one in place:

1. **Set to** — a dropdown of every existing connection profile of the
   matching type (wifi profiles for `wlan0`/`wlan1`, ethernet profiles
   for `eth0`), plus a **"None (disconnected)"** option (`nmcli device
   disconnect <device>` — a normal NetworkManager state, not a fake
   one), plus a single **Apply** button.
2. **Recovers to** — the same style of dropdown (same profile list plus
   "None (disconnected)"), for the known-good fallback this interface
   reverts to if a change isn't confirmed in time, or "don't care,
   leave alone" (distinct from "None" — "don't care" means recovery
   skips this interface entirely, "None" means recovery actively
   disconnects it).
3. A single **Apply** button per card, not one per section — applying
   activates whichever profile is selected in "Set to"; changing
   "Recovers to" saves immediately on its own (no separate profile is
   created for either action).

Creating or editing a profile's actual settings (SSID/password,
DHCP/static, hotspot broadcast address, etc.) happens somewhere else
entirely: **"New wifi configuration"** / **"New ethernet
configuration"** buttons at the bottom of the page open a small
form (name + the relevant settings) that creates a brand-new named
profile and adds it to every matching dropdown on the page. There is
no "adjust an existing profile" — if a profile's settings need to
change, retire it and create a new one with a new name instead.

A page also shows, for the whole machine: which interface(s) currently
have a real internet route (not just a link), and which interfaces
are set to share/route through which — the "highlight which has
internet, which need to route through it" piece from the original
discussion.

### Connection profiles are shared, not per-device

A NetworkManager connection profile with no device pinned (the normal
case here, e.g. `CoffeebeanWifi`) can be activated on *any* matching
device, but only on **one device at a time** — activating it on a
second device silently deactivates it on the first (confirmed the hard
way, see "v1 → v2" below). `network` deliberately does not "fix" this
by pinning profiles to a specific device (e.g. via `interface-name`) —
Ben's call: that would need the app to silently disambiguate
same-named profiles per device (e.g. hidden `wlan0-`/`wlan1-`
prefixing), which is exactly the kind of hidden-from-the-operator
magic this project avoids. Instead:

- Two wifi devices that both need to be "on Coffeebean" get **two
  separate, identically-configured profiles** — `CoffeebeanWifi` and
  `CoffeebeanWifiSpare` — one per device, both visible, both plain
  `nmcli` profiles an operator could activate by hand.
- Before activating (or setting as Recovery) a profile that's already
  active on a *different* device, `network` checks first and refuses
  with a plain message, rather than letting NetworkManager silently
  evict it from wherever it currently is.
- Only one hotspot profile (`AmundsenHotspot`) is needed for now — it's
  not something two devices would ever want active simultaneously.

### Recovery / safe-apply

Applying a network change over the network you're changing risks
locking yourself out (wrong wifi password, wrong static IP — no way
back in except physically at amundsen). Rather than trying to "undo
the one edit" (fragile — the previous state might not be well-defined,
or might itself be the thing under test), each interface has its own
independently-defined **recovery profile**: apply the intended change,
start a short countdown (a `systemd-run --on-active=<N>s` one-shot
timer is the natural mechanism, given this project already reaches for
systemd rather than hand-rolled process supervision elsewhere), and if
the operator doesn't explicitly confirm the change from the page before
it fires, every interface with a defined recovery profile gets that
profile re-applied. Confirming just cancels the pending timer.

Ben's actual answer for what recovery means today:
- `wlan0` (onboard chip): recovery = go back to being a wifi **client**
  of "Coffeebean" (hotspot mode off). Rationale: the dongle is a cable
  that can physically fall out; the onboard chip can't, so it's the
  more trustworthy fallback, even though the dongle roams better
  day-to-day. In practice: `wlan0` recovers to `CoffeebeanWifi`, `wlan1`
  recovers to `CoffeebeanWifiSpare` — never the same profile as each
  other's recovery target (see "Connection profiles are shared, not
  per-device" above — the same conflict check applies to *setting*
  recovery, not just to Apply, otherwise two interfaces could be
  configured to fight over one profile the moment a revert fires).
- `eth0`: recovery = **don't care, leave alone**. Explicitly agreed:
  during a network-config session nothing is navigating (the xNAV650's
  state genuinely doesn't matter then), and if the operator is
  *deliberately* changing `eth0` (e.g. fixing routing for NTRIP
  sharing), an auto-revert would fight them for no reason. "A user who
  changes the network while the xNAV650 is navigating gets what they
  deserve" — not a case this service needs to protect against.

## User Stories

- As the operator, I can see every network interface on amundsen, its
  current mode (DHCP/static/hotspot) and settings, on one page per
  interface.
- As the operator, I can switch an interface to any already-existing
  connection profile (or disconnect it entirely), and the change
  applies immediately via NetworkManager — with a plain refusal if the
  profile I picked is already active on a different interface, instead
  of it being silently pulled off that interface.
- As the operator, I can create a brand-new named profile (wifi
  client/hotspot SSID+password, or ethernet DHCP/static/share) when I
  need one that doesn't already exist, separately from switching an
  interface to it.
- As the operator, after changing a risky setting, I get a countdown to
  confirm the change is good — if I don't (e.g. because it broke my
  access), every interface with a defined recovery profile reverts to
  it automatically, without needing physical access to amundsen.
- As the operator, I can see at a glance which interface currently has
  real internet access, and configure another interface to share/route
  through it (for the xNAV650's NTRIP corrections over `eth0`, sharing
  whichever wifi interface has internet).

## Implementation Decisions

- All reads/writes go through `nmcli` (via `subprocess`), never
  `/etc/dhcpcd.conf`/`wpa_supplicant.conf`/hostapd/dnsmasq directly —
  NetworkManager already owns this on amundsen (Raspberry Pi OS
  Bookworm's default), and mixing a second mechanism in would fight it.
- `nmcli -t` (terse, stable machine-readable output) for anything
  parsed programmatically; human-readable `nmcli` output is not parsed.
- The auto-revert timer is a `systemd-run --on-active=<N>s` transient
  unit invoking a small helper script/CLI entry point in this service
  that re-applies every interface's stored recovery profile; confirming
  a change calls `systemctl stop` on that transient unit before it
  fires.
- Recovery profiles are stored in this service's own state (not as
  literal duplicate NM connection profiles) — just enough to know
  which NM connection (or "don't care") applies to each interface on
  revert. Exact storage format (flat file vs `config.yaml` entry) TBD
  at implementation time.
- Following `manager`'s "no live reload" convention for `config.yaml`
  itself (this service's *own* host/port), but NOT for the network
  settings it manages — those are inherently live, that's the point.
- Before activating a profile (Apply or setting Recovery), check
  whether it's currently active on a *different* device
  (`nmcli -t -f NAME,DEVICE connection show --active`, or equivalent)
  and refuse with a plain message if so — this is the only guard
  against the profile-sharing conflict described above; there is no
  device-pinning at the NetworkManager level.
- "None (disconnected)" is `nmcli device disconnect <device>` — a
  normal state, not a fake/synthetic option.

## Config additions (proposed)

```yaml
network:
  unit: robot-network.service
  host: 0.0.0.0
  port: 8007
  web_ui: true
```

(`revert_timeout_s` removed under v3 — see "v2 → v3" below; recovery/
confirm goes away entirely.)

## Testing Decisions

- `nmcli` calls are wrapped behind a thin module so unit tests can stub
  subprocess calls out entirely (same pattern as `manager`'s
  `systemctl`/journal wrapper) — no test should actually reconfigure a
  real interface.
- The countdown/auto-revert timer logic (start, cancel-on-confirm,
  fire-and-reapply) is unit-testable with a fake clock/stubbed
  `systemd-run`, same reasoning as `drive`'s human-control-hold-timeout
  tests.
- Real hardware needed to validate: the external dongle actually being
  recognised and roaming as expected, the onboard chip's hotspot mode
  actually being connectable-to from a phone, and an actual end-to-end
  "apply a bad wifi password, watch it revert" drill — none of this is
  meaningfully fakeable.

## Out of Scope (v1 of this service)

- Editing an existing connection profile's settings in place (SSID,
  password, address, ...) — v2's page can only select an
  already-existing profile or create a brand-new one; if a profile's
  settings need to change, retire it and create a new one with a new
  name via "New wifi/ethernet configuration" instead.
- Encryption/security beyond ordinary WPA2-PSK — "not too worried about
  encryption, unless it's easy" (Ben's words); no enterprise auth, no
  captive portal, no per-client access control on the hotspot.
- Per-client bandwidth/usage visibility on the hotspot.
- Multiple simultaneous hotspots, or mesh/roaming between amundsen's
  own hotspot and the house wifi (the recovery mechanism handles
  "switch between them," not "be on both at once").
- Editing `network`'s own `config.yaml` entry from within its own page
  (same convention as every other service — edited via the manager's
  Config page).

## v1 → v2: why the page shape changed (2026-07-26)

v1 (below, "Implementation Status") shipped a per-interface
Configuration form that always created/replaced a `gr6-<device>`
profile, plus a separate "use an existing connection" control added
afterwards, plus an inline create-or-select Recovery dropdown — three
different mechanisms doing overlapping jobs, and it showed: using the
real page surfaced a live incident. Ben renamed `wlan0`'s existing
client profile to `CoffeebeanWifi`, then used the new "use an existing
connection" control to activate that same profile on `wlan1`. It
looked like nothing happened — until NetworkManager's own autoconnect
policy noticed `wlan0` had gone idle (activating a profile on `wlan1`
had *silently deactivated it on `wlan0`*, since one connection profile
can only be active on one device at a time) and auto-started
`AmundsenHotspot` on `wlan0` instead, unasked. Nobody's access was
actually lost (the operator's session happened to already be reaching
amundsen via `wlan1`), but it was a real "did the tool just do
something I didn't ask for" moment.

Root cause: a connection profile with no device pinned is genuinely
shared across every matching device, not "available to any one of
them" — this project's earlier "two wifi interfaces can join the same
SSID at once" finding (see v1 status below) was tested with two
*separate* profiles, not one profile reused across two devices, and
that distinction got lost when `network`'s UI let you point one
profile at either device without warning.

The fix is a design simplification, not just a bug patch — see "Page
shape (v2)" and "Connection profiles are shared, not per-device"
above: no more inline create-or-edit, only select-existing-or-refuse,
plus explicit per-role profiles (`CoffeebeanWifi` / `CoffeebeanWifiSpare`)
so the conflict can't arise for the one case (both wifi devices on
Coffeebean) where it would actually come up day-to-day.

## v2 → v3: removing recovery/confirm, priority-based wifi fallback, live link-quality display (2026-07-30, built — not yet live-tested)

### Why (the incident that prompted this)

`wlan1` (the external dongle) dropped its `CoffeebeanWifiSpare`
connection overnight with `reason 'ssid-not-found'` at 08:46, retried
three more times (~25-35s each, ~2 minutes total), then fell back to
auto-activating `AmundsenHotspot` — unprompted, and on the interface we
specifically wanted to be the *more* reliable one, not `wlan0`. The
robot hadn't moved; there was no reason for the AP it had been talking
to, to vanish. It was only survivable because Ben's own desktop
happened to be reaching amundsen via `wlan0` at the time — pure luck,
not something the design guaranteed.

This exposed the actual flaw in v2's recovery/confirm design, not just
a bug: the confirm-within-`revert_timeout_s` flow assumes the operator
can still reach the confirm button after the change — but the thing
changing is often the very link the operator is using to reach
amundsen at all. Ben's insight: NetworkManager already has its own
retry/blacklist/fallback logic (that's exactly what produced the
"3 retries then fall back to hotspot" sequence above) — the 30s
confirm-or-revert scheme doesn't add safety so much as fight NM's own,
already-reasonable, recovery behaviour, while adding a real risk of its
own (a change that breaks the very path needed to confirm it).

### New design

- **One combined Apply button for all three interface cards**
  (`eth0`, `wlan0`, `wlan1`) instead of a per-card Apply. Recovery
  dropdowns and the auto-revert timer are removed entirely from the
  wifi and ethernet cards — this supersedes the "Recovers to" part of
  "Page shape (v2)" and the whole "Recovery / safe-apply" section
  above, for these two interface types.
- `eth0` stays the deliberate out-of-band recovery path — if a wifi
  change goes wrong, physical/wired access via `eth0` is the fallback,
  same as it's always effectively been, just now the *only* one, not
  one of several recovery mechanisms. Ben's own words: "I'll have to be
  careful, or I will have to connect on eth0 to recover things" —
  treated as the accepted risk, not something the UI tries to soften
  further.
- **Wifi profile choice becomes priority-based, not exclusive-activate.**
  "Set to X" sets X's `connection.autoconnect-priority` higher (e.g.
  `10`) than every other client profile eligible for that device
  (reset to `0`), and any hotspot profile for that device sits at a
  distinctly *lower* priority (e.g. `-10`) so it's only ever reached
  once every client profile has been tried and NM has blacklisted them
  after repeated failures — never a peer choice the way it was this
  morning. NM's own retry/fallback cascade is then deliberately left to
  do its job if the chosen profile stops working, instead of a
  bespoke timer intervening — this is the mechanism now, not a failure
  case to guard against.
- **"None" is replaced by a real "Turn off this interface" action**:
  `nmcli device set <dev> managed no` (reversible with `managed yes`),
  not `nmcli device disconnect`. Disconnect-only is what today's page
  actually does for "None", and it doesn't stick — the device stays
  eligible for NM's autoconnect, which immediately reactivates
  something (exactly Ben's "'None' doesn't work, because network
  manager knows better" report). `managed no` genuinely stops NM from
  touching that device at all. Flagged clearly in the UI as dangerous
  — same accepted-risk treatment as above.

### Live link-quality display (new)

Two more figures on each interface card, next to the existing
connected/disconnected text:

- **Signal strength (wifi only), in dBm** — from `iw dev <device>
  link`'s `signal:` line, not `nmcli`'s 0-100% figure.
- **Lost packets (all interfaces)** — wifi: `iw dev <device> station
  dump`'s `tx failed` / `tx retries` (this is exactly what surfaced the
  AP-not-hearing-the-Pi asymmetry investigated 2026-07-30); ethernet:
  `ip -s link show <device>`'s RX/TX `errors`/`dropped`.

Updated live, ideally at 1Hz, over a new `/ws/status` websocket —
reusing `shared/web/static/ws-utils.js`'s existing `connectWs`/
`fillFields` helpers (same pattern every other service's live page
already uses), backed by a small periodic snapshot loop in
`network/app.py`, structurally the same shape as `shared/sysstats.py`'s
snapshot. If that's more effort than it's worth, a plain page
auto-refresh (`setInterval(location.reload, 1000)` or a meta-refresh)
is an explicitly acceptable fallback — Ben's own preference stated up
front, not a compromise forced on him.

### Force `eth0` link speed to 100M (new config option)

Ben's suspicion, from direct observation: with a switch in the path,
`eth0` auto-negotiates to 1G; plugged straight into the xNAV650, it's
100M — and 100M seems to behave better. Plausible even before testing
further: Gigabit (1000BASE-T) uses all 4 cable pairs with more complex
encoding/echo-cancellation and a higher clock rate than Fast Ethernet
(100BASE-TX, 2 pairs, simpler encoding) — both a pickier cable/
connector requirement and a more likely RF noise source, which matters
here given the xNAV650's GNSS antenna is already a known-sensitive
neighbour (same class of concern as the earlier USB3-interference
finding, just a different culprit).

Add an optional **"Force link speed"** field to the "New ethernet
configuration" page (Auto / 100 Mbps / 1000 Mbps), via NetworkManager's
`802-3-ethernet.speed`/`.duplex`/`.auto-negotiate` properties — NM
requires `auto-negotiate` off and an explicit `duplex` whenever a fixed
`speed` is set (can't force speed alone); exact property interaction
to confirm against real `nmcli` behaviour at implementation time, same
"don't assume, verify live" approach as the rest of this service.

### Open questions — resolved during build (2026-07-30)

- Autoconnect-priority values: built as `10` (selected) / `0` (normal,
  other client profiles) / `-10` (hotspot profiles) — as suggested
  above, not otherwise tuned against real roaming behaviour yet.
- "Turn off this interface" got its own plain client-side `confirm()`
  on the combined Apply button, only triggered if a wifi card's select
  is set to "Off" — no network dependency, so it doesn't reintroduce
  the original confirm-timer problem.
- `eth0`-alone-as-fallback: left as an accepted risk per the discussion
  above, not mitigated further — genuinely not addressed by this build,
  worth keeping in mind.

### "Atomic wifi change" — the whole batch is one intent, not one device at a time (2026-07-30)

Ben's second live test tried to swap `AmundsenHotspot` and `CoffeebeanWifi`
between `wlan0`/`wlan1` in one combined Apply — and it reported both as
"already in use", refusing the swap entirely. Root cause: `apply()`
checked each device's chosen profile against NetworkManager's *current*
live state, one device at a time — so of course the target looked "in
use", by the very device it was about to be freed from a moment later
in the same submission. The whole point of a combined Apply is that
it's one coordinated intent, not three independent ones.

Fixed: `_validate_batch()` now checks the *whole submitted batch*
together — a profile already in use elsewhere is only a real conflict
if that other device *isn't also* moving away from it in this same
Apply (checked by comparing what that other device itself chose in the
same form submission, not by re-querying live NM state per device).
Validation runs fully before anything is applied — either the whole
batch is valid and all of it happens (`_apply_batch()`), or none of it
does, one clear error, no partial swaps left half-done.

This also fixed a related priority bug the old per-device design had:
priorities were previously set separately as each device was processed
in turn, so a swap's *second* device silently overwrote the *first*
device's priority assignment (both wifi profiles are unbound/shared,
so `autoconnect-priority` is a property of the *profile*, not the
device). `_apply_batch()` now computes priorities once, across every
wifi profile touched by *any* device in the batch, before activating
anything — whatever's chosen by any device this Apply gets
`PRIORITY_SELECTED`, regardless of whether it's normally a hotspot;
only a profile nobody chose this time keeps the
hotspot-is-lowest-priority default.

Known remaining limitation, not fixed (inherent to shared/unbound
profiles + a single scalar priority per profile, not per device — see
"Connection profiles are shared, not per-device" above): if `wlan0`
and `wlan1` each want a *different* profile as their own long-term top
choice, both profiles end up at the same high priority system-wide.
If one later drops and NetworkManager has to autonomously decide a
fallback, it can't tell "wlan0's preferred profile" from "wlan1's
preferred profile" — only "everything explicitly chosen recently vs.
not". A real fix would need per-device pinning, which this project has
deliberately avoided (see "Connection profiles are shared, not
per-device"). Worth knowing about, not urgent enough to solve now.

45 unit tests now (8 new, in a new `test_app.py` covering the batch
validate/apply logic directly — pure enough to test without a Flask
request context). One of the new tests caught a real bug in the first
draft of this fix before it ever reached Ben (an inverted
"is-it-being-relinquished" check when the other device wasn't part of
the submitted batch at all).

### First live test (2026-07-30) — two real bugs found and fixed

Ben's first real "test in anger" surfaced two genuine bugs, not just
rough edges:

1. **`nmcli` calls had no timeout.** Applying `wlan1` → `CoffeebeanWifiSpare`
   took NetworkManager **58 seconds** to declare `ssid-not-found` and
   fall back — which it did correctly, per the priority design — but
   `nm._run()`'s `subprocess.run()` had no timeout, so the Flask
   request (and the page) just froze for that whole time with zero
   feedback. Fixed: `_run()` now has a 90s timeout (comfortably above
   the observed 58s worst case), raising a clean `NmError` instead of
   hanging forever; `iw`/`ip` calls (used for the link-quality display)
   got a short 5s safety-net timeout too, since they're local/normally-
   instant and shouldn't ever need long. The Apply button now also
   carries a plain-text warning that a wifi change can take up to a
   minute to return.
2. **The conflict check had a real race window.** Right after the
   `wlan1` fallback above, a second Apply set `wlan0` → `AmundsenHotspot`
   — but `AmundsenHotspot` was, at that exact moment, still
   *activating* (not yet fully active) on `wlan1` from the fallback.
   The conflict check (`active_connections()`, built from `nmcli
   connection show --active`) only sees fully-active connections, so
   it missed this and let `wlan0`'s activation through — which evicted
   `AmundsenHotspot` straight back off `wlan1`, the exact
   same-profile-two-devices conflict v1→v2 was built to prevent, just
   re-opened by a timing gap the check didn't cover. This combination
   (a frozen page + a profile yanked mid-activation) most likely left
   NetworkManager's `wlan1` state confused enough that only a physical
   unplug/replug cleared it. Fixed: added `connections_in_use()`,
   built from `nmcli device status`'s CONNECTION column instead —
   that field is populated the instant a device *starts* using a
   profile, not just once fully active, closing the race. (A much
   smaller theoretical TOCTOU window remains between checking and
   actually activating — not addressed, judged disproportionate for a
   single-operator admin tool.)

37 unit tests now (4 new: `connections_in_use()`, the timeout
behaviour). Re-verified read-only after the fix (page renders,
`/ws/status` streams live figures) — the actual apply/off/priority
paths still need Ben's own live re-test, now wired directly via
`eth0` so a stuck wifi change can't cost access to the page itself.

## Implementation Status (v2, as of 2026-07-26)

Built and installed (`robot-network.service` restarted with the v2
code, port 8007). Confirmed live, safely, without ever touching
`wlan0`/`wlan1`/`eth0`'s working connections:

- The page now shows the "Set to" / "Recovers to" dropdown pair per
  interface described above, with an inline note on any option that's
  "currently active on <other device>".
- The conflict check works end-to-end: applying `CoffeebeanWifiSpare`
  (active on `wlan1`) to `wlan0` was correctly refused with a plain
  message, and neither device's real connection changed.
- The same check works for Recovery: setting `wlan1`'s recovery to
  `CoffeebeanWifi` (already `wlan0`'s recovery target) was correctly
  refused, `recovery.json` unchanged.
- A blocked Apply/Recovery attempt does not arm the revert timer (only
  a successful change should start the countdown) — confirmed the
  timer stayed inactive through both blocked attempts above.
- "New wifi configuration" creates a profile without activating it —
  confirmed a throwaway profile appeared in `nmcli connection show`
  but no device's active connection changed, then deleted it.
- `wlan0` recovers to `CoffeebeanWifi`, `wlan1` recovers to
  `CoffeebeanWifiSpare` (fixed from a stale pre-v2 `recovery.json` that
  had both pointing at `CoffeebeanWifi` — exactly the conflict v2
  exists to prevent).
- 36 unit tests (`test_nm.py`, `test_recovery.py`), all `nmcli`/
  `systemd` calls stubbed, no real device ever touched by a test.

**Not yet tested live** (deliberately left alone — same safety-first
order as v1):

- Actually applying "None (disconnected)" to a real wifi device.
- `wlan0` switched into hotspot mode via the v2 page itself (confirmed
  working previously via direct `nmcli`, not yet via "Set to").
- `eth0`'s "Share internet" via a "New ethernet configuration" profile.

## Implementation Status (v1, as of 2026-07-26 — superseded by v2 above, kept as a historical record of what was tested)

Built and installed (`robot-network.service`, port 8007, shown on the
manager's home page). Confirmed live, safely, without ever touching
`wlan0`'s or `eth0`'s working connections:

- The external USB dongle needed no driver install — recognised
  instantly as `wlan1` (MediaTek `mt76x2u`, kernel driver auto-bound).
- Two wifi interfaces (the onboard chip and the dongle) can both
  connect to the same SSID (Coffeebean) at once without conflict —
  NetworkManager picks one as the default route by metric, the other
  stays up and reachable; nothing broke. This was a real open question
  going in, not assumed.
- The write path (`nm.apply_wifi_client` → `connection add` + `up`)
  works end-to-end through the real app — tested with a deliberately
  nonexistent SSID, which correctly failed with a friendly on-page
  error rather than a 500 (a bug this test caught and fixed — errors
  weren't handled at all in the first cut).
- The recovery/auto-revert mechanism — `schedule_revert`,
  `cancel_pending`, and a real timer firing and re-activating a stored
  connection via `systemd-run` — all confirmed working, independent of
  whether the Flask process itself is still alive.
- 18 unit tests (`test_nm.py`, `test_recovery.py`), all `nmcli`/
  `systemd` calls stubbed, no real device ever touched by a test.

**Not yet tested** (built per the design above, deliberately left
alone today per the safety-first order — `wlan1` proven working before
touching anything the operator's own access might depend on):

- `wlan0` (onboard chip) switched into hotspot mode.
- `eth0` switched between DHCP/static through the app (its existing
  static profile, set up directly via `nmcli` before this app existed,
  has never been touched by it).
- Internet sharing — `apply_ethernet(..., share=True)` now exists (a
  "Share internet" checkbox on `eth0`'s Configuration form, forcing
  `ipv4.method: shared` the same way `apply_hotspot` already does),
  confirmed the xNAV650's own gateway is already pointed at the Pi's
  `eth0` address (`mobile.cfg.txt`'s `-gateway_address192.168.196.22`)
  so this prerequisite is already satisfied — but the checkbox itself
  hasn't been ticked live yet. `eth0`'s Address field is pre-filled
  with its current real address (`192.168.196.22/24`) specifically so
  ticking "Share" and applying without touching that field preserves
  xNAV650 compatibility rather than needing it typed in.
- The real Coffeebean SSID/password through `wlan1`'s Configuration
  form (only a throwaway nonexistent SSID has been tried) — recovery
  is armed (`wlan1` → `preconfigured`) ready for this test.

Also fixed three real UX bugs found by Ben using the page rather than
just reading the code: a hotspot's default address (`192.168.4.1/24`)
was shown as placeholder text too long for its box, invisible/truncated
— now the actual proposed value, editable, as the field's real value
(and every address/gateway field now pre-fills from the interface's
*current* state, not just the hotspot default); the DHCP/Static/Gateway
fields were shown-but-silently-ignored whenever Hotspot mode was
selected (a hotspot is always `ipv4.method: shared`, no such choice
exists) — now hidden, same treatment given to `eth0`'s new "Share"
checkbox; and the Recovery dropdown's options were bare NetworkManager
connection names ("preconfigured") with no way to tell what any of them
actually configure — now shows a plain-text description of whichever
is selected (deliberately not a tooltip — this page is as likely to be
used from a phone/tablet, which has no hover state).

## Further Notes

This PRD was written the same day the external dongle arrived on Ben's
desk, specifically to unblock building and testing it immediately
(motivation: outdoor wifi needs improving before creating paths and
watering in earnest) — see the conversation log for the fuller design
discussion (NetworkManager-vs-hand-rolled tradeoff, the self-lockout
risk, and why recovery ended up per-interface rather than its own
page). Update this document as real hardware testing surfaces anything
the design got wrong — expected, not a sign the plan was bad.
