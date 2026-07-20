import queue
import time
import unittest

from serial_link import SerialLink


class FakeSerial:
    """Serial-shaped stub: readline() blocks up to `timeout` for a queued
    line (mirrors pyserial's own timeout behaviour), returning b"" if
    nothing arrives in time — same as a real idle serial port."""

    def __init__(self, port, baudrate, timeout):
        self.queue = queue.Queue()
        self.written = []
        self.timeout = timeout

    def readline(self):
        try:
            return self.queue.get(timeout=self.timeout)
        except queue.Empty:
            return b""

    def write(self, data):
        self.written.append(data)


def wait_until(predicate, timeout=1.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class TestSerialLink(unittest.TestCase):
    def _make_link(self):
        holder = {}

        def factory(port, baudrate, timeout):
            fake = FakeSerial(port, baudrate, timeout)
            holder["fake"] = fake
            return fake

        link = SerialLink("ignored-port", 115200, serial_factory=factory)
        return link, holder

    def test_read_loop_updates_state_from_telemetry(self):
        link, holder = self._make_link()
        link.start()
        holder["fake"].queue.put(b"EN 10 20\n")
        self.assertTrue(wait_until(lambda: link.snapshot().get("LM_position") == 10))
        self.assertEqual(link.snapshot(), {"LM_position": 10, "RM_position": 20})

    def test_state_accumulates_across_multiple_lines(self):
        link, holder = self._make_link()
        link.start()
        holder["fake"].queue.put(b"EN 1 2\n")
        holder["fake"].queue.put(b"WP 1\n")
        self.assertTrue(wait_until(lambda: link.snapshot().get("pump") is True))
        self.assertEqual(link.snapshot(), {"LM_position": 1, "RM_position": 2, "pump": True})

    def test_malformed_line_ignored_without_crashing(self):
        link, holder = self._make_link()
        link.start()
        holder["fake"].queue.put(b"garbage not a telemetry line\n")
        holder["fake"].queue.put(b"EN 5 6\n")
        self.assertTrue(wait_until(lambda: link.snapshot().get("LM_position") == 5))

    def test_send_writes_raw_command(self):
        link, holder = self._make_link()
        link.send("SV 100 100\n")
        self.assertEqual(holder["fake"].written, [b"SV 100 100\n"])


if __name__ == "__main__":
    unittest.main()
