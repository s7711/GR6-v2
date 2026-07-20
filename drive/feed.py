"""Publishes drive's live state over a Unix domain socket, for other
services on this machine to consume — the future `navigate`, `missions`,
a wheelspeed-GAD sender, `safety`. Same shape as `oxts-nav`'s
`nav_feed.py` — see drive-prd.md ("Feed naming") for why this is one
feed (`drive_feed`) covering motors/encoders/pump/ultrasonics together,
not split by concern.

Protocol: any number of clients may connect. Each gets its own send
loop at `hz`. Every message is a 4-byte big-endian length prefix
followed by that many bytes of a pickled dict — the same snapshot the
web UI's own websocket sends.
"""

import logging
import os
import pickle
import socket
import struct
import threading
import time


class DriveFeedServer:
    def __init__(self, socket_path, snapshot_fn, hz):
        self.socket_path = socket_path
        self.snapshot_fn = snapshot_fn
        self.period = 1.0 / hz

    def start(self):
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        if os.path.exists(self.socket_path):
            os.remove(self.socket_path)  # Stale socket from a crashed/killed previous run
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(self.socket_path)
        server.listen(5)
        logging.info("[drive_feed] Listening on %s", self.socket_path)
        while True:
            conn, _ = server.accept()
            threading.Thread(target=self._client_loop, args=(conn,), daemon=True).start()

    def _client_loop(self, conn):
        try:
            while True:
                data = pickle.dumps(self.snapshot_fn())
                conn.sendall(struct.pack(">I", len(data)) + data)
                time.sleep(self.period)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # Client gone — that client's loop simply ends, others unaffected
        finally:
            conn.close()
