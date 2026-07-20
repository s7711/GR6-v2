"""Client for oxts-nav's nav_feed Unix domain socket — the first real
consumer of it (oxts-nav/nav_feed.py's docstring: "Nothing consumes this
yet; it's built now, ahead of need"). Reconnects automatically if
oxts-nav hasn't started yet, or restarts.
"""

import logging
import pickle
import socket
import struct
import threading
import time


class NavFeedClient:
    def __init__(self, socket_path):
        self.socket_path = socket_path
        self.lock = threading.Lock()
        self._latest = {"nav": {}, "status": {}, "connection": {}}

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def latest(self) -> dict:
        with self.lock:
            return self._latest

    def _run(self):
        while True:
            try:
                conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                conn.connect(self.socket_path)
                self._read_loop(conn)
            except OSError:
                time.sleep(1.0)  # oxts-nav not up yet, or restarting — keep retrying

    def _read_loop(self, conn):
        try:
            while True:
                (length,) = struct.unpack(">I", self._recv_exact(conn, 4))
                payload = pickle.loads(self._recv_exact(conn, length))
                with self.lock:
                    self._latest = payload
        except (OSError, EOFError):
            logging.info("[nav_client] Lost connection to nav_feed, reconnecting")
        finally:
            conn.close()

    @staticmethod
    def _recv_exact(conn, n):
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                raise EOFError("nav_feed connection closed")
            buf += chunk
        return buf
