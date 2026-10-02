"""Appends lines to files from one background thread, so the thread that
produced a line never waits on the disk. Promoted from navigate's debug-log
queue (2026-10-01, before the first endurance test) once jobs, missions,
waterbutt and map-manager all turned out to still write their logs inline,
from their tick loops and request handlers.

Why it matters on this robot: the SD card was measured stalling a file
create for up to 4.2s and an append for hundreds of ms (2026-10-01), while
reads never slowed. jobs gives navigate 5s and missions gives jobs only a
few seconds to reply - an inline write in a handler or tick can turn a
slow card into a step that "couldn't reach" a service that was fine.

Lines are written strictly in the order they're put(). The file (and its
directory) is created by the writer thread on its first line, so callers
never touch the disk at all. A put() before start() writes synchronously
instead - that's what every unit test relies on (they import the app and
call its functions directly, never running __main__), and it means a
missing start() degrades to the old inline behaviour rather than losing
lines.
"""

import logging
import queue
import threading
from pathlib import Path


class QueuedWriter:
    def __init__(self, name: str):
        self.name = name  # for the warning only
        self._queue = queue.Queue()
        self._started = False

    def start(self):
        self._started = True
        threading.Thread(target=self._loop, daemon=True, name=f"{self.name}-log-writer").start()

    def put(self, path: Path, line: str):
        """line should end with its own newline. An empty line still
        creates the file - see navigate's _start_new_debug_log."""
        if self._started:
            self._queue.put((Path(path), line))
        else:
            self._write(Path(path), line)

    def drain(self):
        """Synchronously writes anything still queued - for tests."""
        while not self._queue.empty():
            self._write(*self._queue.get_nowait())

    def _loop(self):
        while True:
            self._write(*self._queue.get())

    def _write(self, path: Path, line: str):
        try:
            try:
                f = open(path, "a")
            except FileNotFoundError:
                path.parent.mkdir(parents=True, exist_ok=True)
                f = open(path, "a")
            with f:
                f.write(line)
        except OSError as e:
            logging.warning("[%s] Couldn't write log %s: %s", self.name, path, e)
