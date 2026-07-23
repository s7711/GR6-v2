# ucomrx_thread.py
# Licensed under the MIT License – see LICENSE file for details.


"""
ucomrx_thread.py

Sets up a background thread, which receives data from OxTS INSs
(on port 50487, the fixed UCOM port - see UCOM_Manual_260707.pdf).
Each IP address is sent to a separate UcomRx decoder.

Mirrors ncomrx_thread.py's structure deliberately (own socket, own
per-IP decoder dict, own lock) rather than sharing a generic base class
with it - see ucomrx.py's docstring for why this codebase prefers
explicit duplication here over a shared abstraction.

Use by:

nrxs = ucomrx_thread.UcomRxThread()

nrxs.nrx['<ip>']['decoder'] will be a UcomRx class that can be used to
access the decoded data. For example:

  nrxs.nrx['192.168.2.62']['decoder'].nav['Lat']

Call nrxs.stop() to end, but note that the thread will be blocked on
data from the socket so it will only stop after data is received.
"""

import time
import socket
import ucomrx
import collections
import threading

UCOM_PORT = 50487


class UcomRxThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon_threads = True
        self.keepGoing = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', UCOM_PORT))
        self.nrx = {}
        # Guards each decoder's nav/status/connection dicts against being
        # read (e.g. by a publisher thread) mid-write. See oxts-nav-prd.md.
        self.lock = threading.Lock()
        self.start()

    def run(self):
        while(self.keepGoing):
            # Get data from socket
            nb, addrport = self.sock.recvfrom(1500) # New bytes - UCOM caps a message at 1452 bytes to avoid IPv4 fragmentation, see the manual
            myTime = time.monotonic() # Grab time asap - used to keep connection['timeOffset'] fresh, see UcomRx.decode()

            addr = addrport[0] # Just grab the IP address, not port

            # Is this a new IP address
            if addr not in self.nrx:
                # Then create a new recent-packets list and decoder in nrx
                self.nrx[addr] = {
                    'recentPackets': collections.deque(maxlen=200),
                    'decoder': ucomrx.UcomRx(),
                    }
                # Add IP address to connection, useful for user
                self.nrx[addr]['decoder'].connection['ip'] = addr
                self.nrx[addr]['decoder'].connection['repeatedUdp'] = 0

            # Under linux, UDP packets can be repeated, which messes up
            # the decoding - same issue ncomrx_thread.py guards against.
            # ncomrx_thread.py fingerprints each packet with binascii.crc32()
            # instead of comparing raw bytes, which works fine for NCOM (real
            # sensor noise in Batch A gives every packet enough entropy) but
            # is NOT safe here: several UCOM messages (e.g. Heartbeat) are
            # almost entirely constant except one steadily-incrementing
            # timer field, and CRC32 is linear over GF(2) - a steady
            # increment can land back on the exact same 32-bit CRC at a
            # predictable interval, causing real, reproducible false
            # "duplicate" hits (found by testing against Amundsen's real
            # UCOM stream - nearly every Heartbeat was being flagged
            # repeated despite genuinely differing). Comparing the raw
            # bytes directly has no such failure mode, and packets here are
            # small enough (well under 100 bytes) that keeping the last 200
            # of them costs nothing worth worrying about.
            if nb not in self.nrx[addr]['recentPackets']:
                self.nrx[addr]['recentPackets'].append(nb)
                with self.lock:
                    self.nrx[addr]['decoder'].decode(nb, machineTime=myTime)
                    # And process all possible data
                    while self.nrx[addr]['decoder'].decode(b'', machineTime=myTime):
                        pass
            else:
                self.nrx[addr]['decoder'].connection['repeatedUdp'] += 1

    def stop(self):
        self.keepGoing = False
