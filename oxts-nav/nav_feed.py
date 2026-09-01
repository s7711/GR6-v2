# nav_feed.py
"""
Publishes live nav/status/connection data over a Unix domain socket, for
other services on this machine to consume — navigate, the aruco/GAD
service, etc. See oxts-nav-prd.md ("Nav data feed") for the design
rationale.

Protocol: any number of clients may connect to the socket. Each gets its
own send loop at `hz`. Every message is a 4-byte big-endian length prefix
followed by that many bytes of a pickled dict:
    {"nav": {...}, "status": {...}, "connection": {...}}
— the same full dicts the web UI's websocket sends, for the same reason:
what's published must not change because one consumer's needs changed.

Built on shared/feed_server.py's generic FeedServer — this module just
supplies the nrxs/xnav_ip-specific snapshot function.

Note for consumers correlating timestamps (e.g. a camera frame's
time.monotonic() capture time) with GPS time: use
ncomrx.machine_time_to_gps(machine_time, connection['timeOffset']) rather
than re-deriving it — see that function's docstring.

## Staleness

Both `ncomrx_thread.py` and `ucomrx_thread.py` just block on their UDP
socket - if the xNAV stops sending entirely (reset, rebooting, e.g. to
pick up an NTRIP mountpoint change), nothing ever calls decode() again,
so decoder.nav/status/connection just sit frozen at their last real
values forever. Every consumer (this feed, and the web UI's own /ws/nav)
was publishing that frozen snapshot on its own fixed timer regardless -
same "push timer independent of real state" shape as a couple of other
feed bugs found elsewhere in this project, just one layer further down:
here it means every page showing live nav data looks perfectly healthy,
frozen on old numbers, with nothing to say otherwise.

`ncomrx_thread`/`ucomrx_thread` now stamp `nrx[addr]['last_packet_at']`
(a plain `time.monotonic()`) each time they actually decode a new
packet - independent of anything inside the decoder itself, so this
works identically for both protocols and needs no GPS lock to have ever
been achieved. `snapshot()` below is the one place that checks it:
past `stale_after_s` with nothing new, it publishes empty dicts rather
than the frozen ones. Empty, not `None`-valued keys - every consumer
(`navigate`'s `_current_position()`, `wheelspeed`'s `_forward_mps()`,
aruco's own `if nav:` guards) already treats a missing key or an empty
dict as "no fix", so this is exactly the shape every one of them
already expects, not a new failure mode to guard against.
"""

import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.feed_server import FeedServer  # noqa: E402

# Edge-triggered staleness logging (added 2026-09-01 while debugging
# navigate's stale-feed aborts): kept as an attribute on nrxs itself
# (below), keyed by xnav_ip, rather than module-global - each nrxs
# (real NcomRxThread/UcomRxThread, or a test's fake) gets its own
# independent transition history, not shared process-wide state. snapshot()
# itself is called concurrently by every connected consumer (navigate's
# nav_feed_socket, oxts-nav's own /ws/nav for possibly several open
# browser tabs) - all of them serialise through nrxs.lock already, so
# reading/updating this dict inside that same lock is race-free:
# whichever call is first to observe the transition logs it once, every
# later concurrent call already sees the flipped state and stays quiet.
# Without this, there was no direct way to tell whether a real NCOM gap
# happened (only reconstructable after the fact from a downstream
# consumer's own debug log, or by catching a brief "-" on a page that
# only redraws twice a second) - now it's a plain, timestamped line in
# journalctl -u robot-oxts-nav.


def snapshot(nrxs, xnav_ip, stale_after_s=2.0):
    with nrxs.lock:
        if not hasattr(nrxs, "_stale_seen"):
            nrxs._stale_seen = {}
        entry = nrxs.nrx.get(xnav_ip)
        decoder = entry.get("decoder") if entry else None
        last_packet_at = entry.get("last_packet_at") if entry else None
        now = time.monotonic()
        is_stale = decoder is None or last_packet_at is None or now - last_packet_at > stale_after_s
        was_stale = nrxs._stale_seen.get(xnav_ip, False)

        if is_stale and not was_stale:
            gap_s = now - last_packet_at if last_packet_at is not None else None
            jitter_ms = decoder.connection.get("timeJitterMax_ms") if decoder is not None else None
            logging.warning(
                "[nav_feed] %s: NCOM feed went stale (no packet for %s; last-known timeJitterMax_ms=%s)",
                xnav_ip,
                f"{gap_s:.2f}s" if gap_s is not None else "none ever received",
                f"{jitter_ms:.2f}" if jitter_ms is not None else "?",
            )
        elif was_stale and not is_stale:
            logging.warning("[nav_feed] %s: NCOM feed recovered", xnav_ip)

        nrxs._stale_seen[xnav_ip] = is_stale

        if is_stale:
            return {"nav": {}, "status": {}, "connection": {}}
        return {
            "nav": dict(decoder.nav),
            "status": dict(decoder.status),
            "connection": dict(decoder.connection),
        }


class NavFeedServer(FeedServer):
    def __init__(self, socket_path, nrxs, xnav_ip, hz, stale_after_s=2.0):
        super().__init__(socket_path, lambda: snapshot(nrxs, xnav_ip, stale_after_s), hz)
