# ncomrx_thread.py
# Licensed under the MIT License – see LICENSE file for details.


"""
ncomrx_thread.py

Sets up a background thread, which receives data from OxTS INSs
(on port 3000). Each IP address is send to a separate NComRx decoder.

Use by:

nrxs = ncomrx_thread.NcomRxThread()

nrxs.nrx['<ip>']['decoder'] will be an NcomRx class that can be used to
access the decoded data. For example:

  nrxs.nrx['192.168.2.62']['decoder'].nav['GpsTime']

Call nrxs.stop() to end, but note that the thread will be blocked on
data from the socket so it will only stop after data is received.
"""

import time
import socket
import ncomrx
import collections
import binascii
import threading
import queue
import logging


class NcomRxThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon_threads = True
        self.keepGoing = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 3000))
        self.nrx = {}
        # Guards each decoder's nav/status/connection dicts against being
        # read (e.g. by a publisher thread) mid-write. See oxts-nav-prd.md.
        self.lock = threading.Lock()
        # Guards each entry's 'logfile' handle only - deliberately a
        # *separate* lock from self.lock above (found live 2026-09-09: a
        # real ~2.5s NCOM "feed stale" abort during "Patio return" turned
        # out to be a disk write stalling this thread, not a real
        # dropped/delayed xNAV packet - see run()'s CRITICAL THREAD note
        # and project memory). If this used self.lock, a slow write in
        # _log_writer_loop would block run() from decoding the next
        # packet just as badly as writing inline used to.
        self.log_lock = threading.Lock()
        self._log_queue = queue.Queue()
        self.moreCalcs = [] # List of calculation functions to expand nrx, see ncomrx
        self.start()
        threading.Thread(target=self._log_writer_loop, daemon=True).start()

    def run(self):
        # CRITICAL THREAD: this loop is the only place NCOM packets are
        # ever received (a UDP socket recv buffer is finite) and its
        # myTime/last_packet_at timestamps are what nav_feed.py's
        # staleness check - and therefore navigate's "abort, no
        # position" logic - relies on. Nothing in this loop may block on
        # anything slower than memory (no disk I/O, no HTTP, no logging)
        # - see _log_writer_loop below for where raw-byte logging
        # actually happens instead, and project memory's 2026-09-09
        # finding for why this rule exists.
        while(self.keepGoing):
            # Get data from socket
            nb, addrport = self.sock.recvfrom(256) # New bytes
            myTime = time.monotonic() # Grab time asap

            addr = addrport[0] # Just grab the IP address, not port

            # Is this a new IP address
            if addr not in self.nrx:
                # Then create a new crclist and decoder in nrx
                self.nrx[addr] = {
                    'crcList': collections.deque(maxlen=200),
                    'decoder': ncomrx.NcomRx(),
                    'logfile': None
                    }
                self.nrx[addr]['decoder'].moreCalcs = self.moreCalcs
                # Add IP address to connection, useful for user
                self.nrx[addr]['decoder'].connection['ip'] = addr
                self.nrx[addr]['decoder'].connection['repeatedUdp'] = 0

            # Under linux, UDP packets can be repeated, which messes up
            # the ncom decoding. Compute CRC and use it to identify
            # repeated packets
            crc = binascii.crc32(nb)
            if crc not in self.nrx[addr]['crcList']:
                self.nrx[addr]['crcList'].append(crc)
                with self.lock:
                    self.nrx[addr]['last_packet_at'] = myTime  # for staleness detection - see nav_feed.py's snapshot()
                    self.nrx[addr]['decoder'].decode(nb, machineTime=myTime)
                    # And process all possible data
                    while self.nrx[addr]['decoder'].decode(b'', machineTime=myTime):
                        pass
                # Hand the raw bytes to the log writer thread instead of
                # writing them here - this is just an in-memory enqueue
                # (no I/O), so it can never be the thing that delays the
                # next recvfrom() above. See _log_writer_loop.
                self._log_queue.put((addr, nb))
            else:
                self.nrx[addr]['decoder'].connection['repeatedUdp'] += 1

    def _log_writer_loop(self):
        # Not critical - this thread exists specifically so a slow/
        # stalled disk write happens *here*, off run()'s critical path.
        # See run()'s CRITICAL THREAD note.
        while True:
            addr, nb = self._log_queue.get()
            with self.log_lock:
                entry = self.nrx.get(addr)
                logfile = entry.get('logfile') if entry else None
                if logfile is None:
                    continue
                try:
                    logfile.write(nb)
                except (OSError, ValueError):
                    # Handle was closed by data_log.py's rotator between
                    # the check above and this write (hourly boundary,
                    # extremely rare) - drop this one line rather than
                    # crash the writer thread.
                    continue
                connection = entry['decoder'].connection
                connection['loggedBytes'] = connection.get('loggedBytes', 0) + len(nb)

    def stop(self):
        self.keepGoing = False

    def user_command(self, message):
        # Commands:
        #  :logging on [ip]
        #  :logging off [ip]
        if message.startswith(":logging"):
            args = message.split()

            # Check that the command has enough arguments
            if len(args) < 3:
                logging.warning("[NcomRxThread]: Invalid logging command format.")
                return

            # Check that the ip address is being received
            ip_address = args[2]
            if ip_address not in self.nrx:
                logging.warning(f"[NcomRxThread]: IP address {ip_address} not being received.")
                return

            command = args[1]
            logfile_info = self.nrx[ip_address]

            # Guarded by log_lock, same as _log_writer_loop/data_log.py's
            # rotator - this is unused in the current app (no caller in
            # the codebase), kept only for interactive/manual use, but
            # it touches the same 'logfile' handle so it must use the
            # same lock to stay safe if ever called from another thread.
            if command == "off":
                with self.log_lock:
                    fp = logfile_info.get('logfile')
                    if fp is not None:
                        logging.info(f"Closing log file for {ip_address}.")
                        logfile_info['logfile'] = None
                if fp is not None:
                    fp.close()
                else:
                    logging.warning(f"No log file open for {ip_address}.")

            elif command == "on":
                with self.log_lock:
                    # Check if a file is already open for this IP
                    if logfile_info.get('logfile') is not None:
                        logging.warning(f"Log file already open for {ip_address}. Ignoring 'on' command.")
                        return

                    try:
                        filename = f"{ip_address}.ncom"
                        # Open the file in binary write mode
                        fp = open(filename, "wb")
                        logfile_info['decoder'].connection['loggedBytes'] = 0
                        logfile_info['logfile'] = fp
                        logging.info(f"Opened log file {filename} for {ip_address}.")
                    except IOError as e:
                        logging.error(f"Failed to open log file for {ip_address}: {e}")