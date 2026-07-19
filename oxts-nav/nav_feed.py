# nav_feed.py
"""
Publishes live nav/status/connection data over a Unix domain socket, for
other services on this machine to consume — path-following, the future
aruco/GAD service, etc. Nothing consumes this yet; it's built now, ahead
of need, per top-prd.md's IPC pattern for high-rate structured data
("pickled dict over a Unix domain socket"). See oxts-nav-prd.md ("Nav
data feed") for the design rationale.

Protocol: any number of clients may connect to the socket. Each gets its
own send loop at `hz`. Every message is a 4-byte big-endian length prefix
followed by that many bytes of a pickled dict:
    {"nav": {...}, "status": {...}, "connection": {...}}
— the same full dicts the web UI's websocket sends, for the same reason:
what's published must not change because one consumer's needs changed.

Note for consumers correlating timestamps (e.g. a camera frame's
time.monotonic() capture time) with GPS time: use
ncomrx.machine_time_to_gps(machine_time, connection['timeOffset']) rather
than re-deriving it — see that function's docstring.
"""

import logging
import os
import pickle
import socket
import struct
import threading
import time


class NavFeedServer:
    def __init__(self, socket_path, nrxs, xnav_ip, hz):
        self.socket_path = socket_path
        self.nrxs = nrxs
        self.xnav_ip = xnav_ip
        self.period = 1.0 / hz

    def start(self):
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        if os.path.exists(self.socket_path):
            os.remove(self.socket_path)  # Stale socket from a crashed/killed previous run
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(self.socket_path)
        server.listen(5)
        logging.info("[nav_feed] Listening on %s", self.socket_path)
        while True:
            conn, _ = server.accept()
            threading.Thread(target=self._client_loop, args=(conn,), daemon=True).start()

    def _client_loop(self, conn):
        try:
            while True:
                with self.nrxs.lock:
                    decoder = self.nrxs.nrx.get(self.xnav_ip, {}).get("decoder")
                    payload = {
                        "nav": dict(decoder.nav) if decoder else {},
                        "status": dict(decoder.status) if decoder else {},
                        "connection": dict(decoder.connection) if decoder else {},
                    }
                data = pickle.dumps(payload)
                conn.sendall(struct.pack(">I", len(data)) + data)
                time.sleep(self.period)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # Client gone — that client's loop simply ends, others unaffected
        finally:
            conn.close()
