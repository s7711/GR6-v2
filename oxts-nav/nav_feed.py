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
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.feed_server import FeedServer  # noqa: E402


class NavFeedServer(FeedServer):
    def __init__(self, socket_path, nrxs, xnav_ip, hz):
        def snapshot():
            with nrxs.lock:
                decoder = nrxs.nrx.get(xnav_ip, {}).get("decoder")
                return {
                    "nav": dict(decoder.nav) if decoder else {},
                    "status": dict(decoder.status) if decoder else {},
                    "connection": dict(decoder.connection) if decoder else {},
                }

        super().__init__(socket_path, snapshot, hz)
