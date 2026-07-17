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
        self.moreCalcs = [] # List of calculation functions to expand nrx, see ncomrx
        self.start()
            
    def run(self):
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
                    self.nrx[addr]['decoder'].decode(nb, machineTime=myTime)
                    # And process all possible data
                    while self.nrx[addr]['decoder'].decode(b'', machineTime=myTime):
                        pass
                # Check if a log file is currently open for this IP address
                if self.nrx[addr]['logfile'] is not None:
                    # Write the raw UDP packet's binary data to the file
                    self.nrx[addr]['logfile'].write(nb)
                    if 'loggedBytes' in self.nrx[addr]['decoder'].connection:
                        self.nrx[addr]['decoder'].connection['loggedBytes'] += len(nb)
                    else:
                        self.nrx[addr]['decoder'].connection['loggedBytes'] = len(nb)

            else:
                self.nrx[addr]['decoder'].connection['repeatedUdp'] += 1
                                        
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

            if command == "off":
                # Check if a file is actually open before trying to close it
                fp = logfile_info.get('logfile')
                if fp is not None:
                    logging.info(f"Closing log file for {ip_address}.")
                    logfile_info['logfile'] = None
                    fp.close()
                else:
                    logging.warning(f"No log file open for {ip_address}.")

            elif command == "on":
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