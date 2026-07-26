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

### Page shape

One page per interface (`wlan0`, `wlan1`/dongle once fitted, `eth0`,
...), each with two sections:

1. **Configuration** — the interface's live/normal settings: DHCP vs
   static (+ address/gateway/DNS if static), and for a wifi interface,
   SSID/password, or hotspot mode (SSID/password/DHCP range for
   *its* clients).
2. **Recovery** — *not* a separate top-level page (an earlier sketch of
   this PRD had it as one; per-interface is simpler and matches how
   Ben actually thinks about it: "what should this adaptor do if
   something's gone wrong?"). Lets the operator define a known-good
   fallback profile for *this* interface, or explicitly mark it
   "don't care, leave alone" (this is what `eth0` gets — see below).

A page also shows, for the whole machine: which interface(s) currently
have a real internet route (not just a link), and which interfaces
are set to share/route through which — the "highlight which has
internet, which need to route through it" piece from the original
discussion.

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
  day-to-day. (Confirmed live: this is *already* almost exactly
  `wlan0`'s current NM profile — `preconfigured`, SSID `Coffeebean`,
  DHCP — so the recovery profile for `wlan0` may end up being "the
  profile that's already there today," not a new one to invent.)
- `eth0`: recovery = **don't care, leave alone**. Explicitly agreed:
  during a network-config session nothing is navigating (the xNAV650's
  state genuinely doesn't matter then), and if the operator is
  *deliberately* changing `eth0` (e.g. fixing routing for NTRIP
  sharing), an auto-revert would fight them for no reason. "A user who
  changes the network while the xNAV650 is navigating gets what they
  deserve" — not a case this service needs to protect against.
- The dongle's own recovery: presumably also "don't care" (it's not
  the trusted path), but Ben should confirm once it's fitted and has
  an interface name to configure against.

## User Stories

- As the operator, I can see every network interface on amundsen, its
  current mode (DHCP/static/hotspot) and settings, on one page per
  interface.
- As the operator, I can change an interface's mode/settings (wifi
  client SSID+password, static IP, or turn it into a hotspot with its
  own SSID/password), and the change applies immediately via
  NetworkManager.
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

## Config additions (proposed)

```yaml
network:
  unit: robot-network.service
  host: 0.0.0.0
  port: 8007
  web_ui: true
  revert_timeout_s: 30   # how long an unconfirmed change has before recovery profiles are re-applied
```

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

## Implementation Status (as of 2026-07-26)

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
