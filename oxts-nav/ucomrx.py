# ucomrx.py
# Licensed under the MIT License - see LICENSE file for details.

"""
ucomrx.py
Decoder for OxTS UCOM data stream (the successor to NCOM - see ncomrx.py).
Decodes measurements into dictionaries, same shape as ncomrx.py:
  nav - navigation measurements
  status - status and configuration
  connection - information about the decoding (characters, skipped, etc)

Unlike NCOM, UCOM's own byte layout isn't fixed by the protocol itself -
it's whatever *we* configure in mobile.dbu (kept at
oxts-nav/xnav-config/mobile.dbu.txt - auto-downloaded from the xNAV650
alongside its other config files, see app.py's XNAV_CONFIG_FILES; edit
that file directly and upload it back to change what's configured. See
documentation/ncom-to-ucom-mapping.md, "Configuring UCOM output by
hand", for how that's built). What's fixed, and what this file relies
on, is that every UCOM header states its own MessageID and MessageVersion
explicitly, so we always know unambiguously which layout arrived -
there's no risk of misinterpreting an old-format packet as a new one.
Every message OXTS currently defines (per
oxts-nav/documentation/oxts.dbu, downloaded from Amundsen's xNAV650) has
its own explicit decodeMessageN method below, the same way ncomrx.py has
one decodeStatusN per NCOM status channel - not a generic schema-
interpreting decoder, on purpose (easier to see exactly what's decoded,
easier to trust each one has actually been checked against the manual).
A message/version combo we haven't written a method for yet is simply
counted as a miss (see connection['decodeMessageErrors']), not a crash -
same as ncomrx.py already does for unrecognised NCOM status channels.
When OXTS ships a new message version, the fix is one more explicit
method, not a rewrite.

Two important things NOT obvious from the manual alone, checked directly
against OXTS's own reference decoder instead
(github.com/OxfordTechnicalSolutions/ucom-decoder - the C++ core in
ucom_decoder/src/, not the Python bindings, which Ben doesn't trust,
having been written by a summer student):
  1. Signal values are used as-is - the DataType/ScaleFactor/Offset
     fields in a message's SignalsInMessage entry are NOT applied
     during decoding (confirmed: ucom_data.cpp never reads them).
     Native units already match what we want (e.g. Ax in m/s^2,
     Lat in degrees) - see oxts.dbs for each signal's native unit.
  2. The CRC is NOT the same as zlib.crc32(). OXTS's variant starts
     the running value at 0 (not 0xFFFFFFFF) and only complements the
     result once, at the very end - see ucom_crc32() below.

Reference information:
 OxTS UCOM manual: documentation/UCOM_Manual_260707.pdf
 OxTS UCOM decoder (C++ core, trusted): https://github.com/OxfordTechnicalSolutions/ucom-decoder
 The exact oxts.dbu/oxts.dbs this was checked against: oxts-nav/documentation/
   (downloaded from Amundsen's xNAV650 via FTP, 2026-07-23 - the master
   catalog of every possible message OXTS defines; not to be confused
   with mobile.dbu, which is Amundsen's own configured *subset* of
   these - see xnav-config/mobile.dbu.txt)
"""

import struct
import math
import re
import datetime

########################################################################
# Definitions: from UCOM_Manual_260707.pdf and the reference C++ decoder

UCOM_SYNC = b'UM'          # 0x55 0x4D - the first two bytes of every UCOM packet
UCOM_HEADER_LENGTH = 16    # bytes, before the payload
UCOM_CRC_LENGTH = 4        # bytes, after the payload

# Same epoch ncomrx.py uses - GNSS/GPS time has no leap seconds, unlike UTC
GPS_STARTTIME = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)

# ncomrx.py gets this live from the device (NCOM status channel 7's
# TimeUtcOffset), because NCOM actually transmits it. UCOM doesn't - checked
# oxts.dbu/oxts.dbs directly, no UTC or leap-second signal exists anywhere
# in the protocol (see ncom-to-ucom-mapping.md's UTC section). So this is a
# fixed constant instead: GPS time has no leap seconds and has been exactly
# 18s ahead of UTC since the last leap second (2016-12-31). Update by hand
# on the rare occasion IERS announces a new one - see https://www.ietf.org/timezones/data/leap-seconds.list
GPS_UTC_OFFSET_S = -18

# CRC-32 lookup table, polynomial 0x04C11DB7 (reflected: 0xEDB88320) -
# copied directly from OXTS's reference decoder (crc.cpp) rather than
# generated from the polynomial by hand, to avoid a transcription bug
# in something that fails silently (a wrong table just rejects every
# real packet's CRC, which looks identical to "no valid data yet")
CRC32_TABLE = (
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D,
)


def ucom_crc32(data):
    """CRC-32 (poly 0x04C11DB7) as used by UCOM.

    NOT the same as zlib.crc32(): OXTS's version starts the running
    value at 0 (not 0xFFFFFFFF) and only complements the result once,
    at the end. The manual doesn't spell this out - checked directly
    against OXTS's own reference decoder (crc.cpp) instead.
    """
    crc = 0
    for byte in data:
        crc = CRC32_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
    return (~crc) & 0xFFFFFFFF


########################################################################
# UCOM class
class UcomRx:
    def __init__(self):
        # todo: protect nav, status with a lock when multi-threaded
        self.nav = {}  # Dictionary for navigation measurements
        self.status = {} # Dictionary for status/configuration
        self.connection = {} # Dictionary for decoding status variables
        self.ucomBytes = b'' # Holds bytes waiting to be decoded

        # Find all 'decodeMessageN' or 'decodeMessageNvV' functions and
        # build a dictionary keyed by (MessageID | MessageVersion << 16)
        # - the same UID scheme OXTS's own reference decoder uses. This
        # mirrors ncomrx.py's decodeStatus dict, just with an optional
        # version suffix since UCOM messages (unlike NCOM status
        # channels) can have more than one schema version.
        decodeList = []
        for name in dir(self):
            m = re.match(r'^decodeMessage(\d+)(?:v(\d+))?$', name)
            if m:
                messageId = int(m.group(1))
                messageVersion = int(m.group(2)) if m.group(2) else 0
                uid = messageId | (messageVersion << 16)
                decodeList.append((uid, getattr(self, name)))
        self.decodeMessage = dict(decodeList)
        self.connection['decodeMessageErrors'] = {} # Useful for debugging or identifying new message/version combos

        # Information about decoding the stream
        self.connection['numChars'] = 0
        self.connection['skippedChars'] = 0
        self.connection['numPackets'] = 0

        # Filter for converting machineTime to GpsTime - same purpose and
        # semantics as ncomrx.py's connection['timeOffset'] (so
        # machine_time_to_gps() works unchanged regardless of which decoder
        # is running), computed differently since UCOM has no GpsMinutes/
        # GpsSeconds of its own. Every UCOM header carries an "arbitrary"
        # SDN-clock timestamp (nanoseconds since power-up); message 100
        # ("The SDN time offsets") gives the offset from that SDN clock to
        # GNSS time. GpsSecondsSinceEpoch = arbitraryTimeNs/1e9 +
        # sdnToGpsOffsetS - confirmed empirically against Amundsen's real
        # xNAV650 (computed time matched the system clock to the second),
        # not from anything documented in the manual.
        self._sdnToGpsOffsetS = None  # Set by decodeMessage100 once a GNSS-sourced SDNOFF arrives
        self.connection['timeOffset'] = None   # GpsTime = machineTime + timeOffset
        # Same filter/jitter-tracking constants and algorithm as ncomrx.py's
        # decode() (asymmetric EWMA - slower to decrease than increase, plus
        # decayed mean/variance/max for the jitter stats), copied rather
        # than reinvented so timeOffset behaves the same way regardless of
        # which decoder is running.
        self.f1 = 0.1            # Factor to decrease timeOffset
        self.f2 = 0.001          # Factor to increase timeOffset
        self.timeJitterMean = 0.0
        self.timeJitterVariance = 0.0
        self.timeJitterMax = 0.0
        self.timeJitterDecayStdev = 0.99  # 1 - dt/Tdecay
        self.timeJitterDecayMax = 0.999   # 1 - dt/Tdecay

    ####################################################################
    # decode() is the normal function to call when new data is available
    # It will update the nav, status and connection dictionaries with
    # new measurements.
    # Only one packet will be decoded so either ensure that rxBytes is
    # a single packet or call multiple times until the return value is 0
    def decode(self, rxBytes, machineTime=None):
        # ucomBytes should be a bytes object
        # Returns 1 if a packet is decoded
        # Returns 0 if a packet cannot be decoded (yet)
        # machineTime (a time.monotonic() value, captured by the caller as
        # close to socket receipt as possible) is used to maintain
        # connection['timeOffset'] - see __init__'s comment. Pass it every
        # call if you want that kept fresh; harmless to omit (or call with
        # rxBytes=b'' to drain buffered packets) if you don't care about it.

        self.ucomBytes += rxBytes # Add received bytes to ucomBytes

        # Find the first valid packet
        while True:
            # Find the UCOM_SYNC bytes
            syncOffset = self.ucomBytes.find(UCOM_SYNC)

            if syncOffset < 0:
                # No sync found. Keep the very last byte in case it's
                # the first sync character, split from its pair by the
                # read boundary (UCOM_SYNC is two bytes, unlike NCOM's
                # single sync byte, so this case is possible here)
                if len(self.ucomBytes) > 0 and self.ucomBytes[-1] == UCOM_SYNC[0]:
                    skipped = len(self.ucomBytes) - 1
                    self.ucomBytes = self.ucomBytes[-1:]
                else:
                    skipped = len(self.ucomBytes)
                    self.ucomBytes = b''
                self.connection['numChars'] += skipped
                self.connection['skippedChars'] += skipped
                return 0

            # Realign to the sync bytes
            self.connection['numChars'] += syncOffset
            self.connection['skippedChars'] += syncOffset
            self.ucomBytes = self.ucomBytes[syncOffset:]

            # Need at least the header to find the payload length
            if len(self.ucomBytes) < UCOM_HEADER_LENGTH:
                return 0

            payloadLength = int.from_bytes(self.ucomBytes[14:16], byteorder='little', signed=False)
            packetLength = UCOM_HEADER_LENGTH + payloadLength + UCOM_CRC_LENGTH

            # Is there enough data for a full packet?
            if len(self.ucomBytes) < packetLength:
                return 0

            # Test the packet integrity
            crcReceived = int.from_bytes(self.ucomBytes[packetLength - UCOM_CRC_LENGTH:packetLength], byteorder='little', signed=False)
            crcCalculated = ucom_crc32(self.ucomBytes[0:packetLength - UCOM_CRC_LENGTH])
            if crcReceived == crcCalculated:
                self.connection['numChars'] += packetLength
                self.connection['numPackets'] += 1
                break # Valid packet

            # This sync is not a valid packet so skip over it and keep looking
            self.ucomBytes = self.ucomBytes[1:]
            self.connection['numChars'] += 1
            self.connection['skippedChars'] += 1

        # ... Must have a valid packet or we would have returned
        messageId = int.from_bytes(self.ucomBytes[2:4], byteorder='little', signed=False)
        messageVersion = int(self.ucomBytes[4])
        # Byte 5: low nibble is the time frame, high nibble is the trigger
        # type - not currently used for anything, so not decoded further
        arbitraryTimeNs = int.from_bytes(self.ucomBytes[6:14], byteorder='little', signed=True)
        payload = self.ucomBytes[UCOM_HEADER_LENGTH:UCOM_HEADER_LENGTH + payloadLength]

        # Keep the machineTime<->GpsTime correlation fresh on every packet
        # (the header's arbitrary time applies to all of them, not just
        # nav) - see __init__'s comment for the maths and where it's from.
        # Filtering/jitter tracking below is copied from ncomrx.py's
        # decode() verbatim (same constants, same asymmetric EWMA) rather
        # than reinvented.
        if self._sdnToGpsOffsetS is not None:
            gpsSecondsSinceEpoch = arbitraryTimeNs / 1e9 + self._sdnToGpsOffsetS
            if messageId == 1:
                # The nav message - mirrors ncomrx.py putting GpsTime in self.nav
                self.nav['GpsTime'] = GPS_STARTTIME + datetime.timedelta(seconds=gpsSecondsSinceEpoch)
                self.nav['UtcTime'] = self.nav['GpsTime'] + datetime.timedelta(seconds=GPS_UTC_OFFSET_S)

            try:
                if machineTime is None:
                    self.connection['timeOffset'] = None
                elif self.connection['timeOffset'] is None:
                    self.connection['timeOffset'] = gpsSecondsSinceEpoch - machineTime
                else:
                    to = gpsSecondsSinceEpoch - machineTime
                    dto = to - self.connection['timeOffset']  # Unfiltered adjustment for this epoch
                    self.connection['timeOffset'] += dto * self.f2 if dto < 0.0 else dto * self.f1
                    self.connection['timeJitter_ms'] = dto * 1000.0
                    self.timeJitterMean = self.timeJitterDecayStdev * self.timeJitterMean + (1.0 - self.timeJitterDecayStdev) * dto
                    self.timeJitterVariance = self.timeJitterDecayStdev * self.timeJitterVariance + (1.0 - self.timeJitterDecayStdev) * (dto - self.timeJitterMean) ** 2
                    self.connection['timeJitterStdev_ms'] = math.sqrt(self.timeJitterVariance) * 1000.0
                    to_abs = abs(dto)
                    self.timeJitterMax = to_abs if to_abs > self.timeJitterMax else self.timeJitterMax * self.timeJitterDecayMax
                    self.connection['timeJitterMax_ms'] = self.timeJitterMax * 1000.0
            except Exception:
                self.connection['timeOffset'] = None

        uid = messageId | (messageVersion << 16)
        try:
            self.decodeMessage[uid](payload)
        except KeyError:
            # A message/version combo we don't have a decodeMessage
            # method for yet (e.g. new firmware, new schema version) -
            # count it rather than fail, same as ncomrx.py does for
            # unrecognised NCOM status channels
            try:
                self.connection['decodeMessageErrors'][uid] += 1
            except KeyError:
                self.connection['decodeMessageErrors'][uid] = 1 # Start new key

        # Remove this packet
        self.ucomBytes = self.ucomBytes[packetLength:]
        self.connection['unprocessedBytes'] = len(self.ucomBytes)

        return 1

    def _updateInnovationFilt(self, key, rawValue):
        # Same fast-attack/slow-decay envelope as ncomrx.py's
        # _updateInnovation (same 0.9/0.1 constants) - shows how bad an
        # innovation has been recently rather than its instantaneous
        # value, so a brief good moment doesn't hide an occasional problem.
        # UCOM's innovation signals are already real-valued (unlike NCOM's
        # packed single-byte ones), so there's no unpacking to do here,
        # just the same filtering NCOM applies client-side after decoding.
        if math.isnan(rawValue):
            return
        inn = abs(rawValue)
        key = key + 'Filt'
        if key not in self.status:
            self.status[key] = inn
        elif inn > self.status[key]:
            self.status[key] = inn
        else:
            self.status[key] = 0.9 * self.status[key] + 0.1 * inn

    ####################################################################
    # Messages 64510/64511 carry a variable-length string, so they
    # aren't decoded via the generic struct-based approach the rest of
    # this file uses (see the decodeMessageN methods below)
    def decodeMessage64510(self, payload):
        # Message 64510 v0: "Warnings" - WarMsg is a variable-length
        # string, not a fixed-size numeric type
        warNo = payload[0]
        warMsg = payload[1:].split(b'\x00')[0].decode('utf-8', errors='replace')
        self.status['WarNo'] = warNo
        self.status['WarMsg'] = warMsg

    def decodeMessage64511(self, payload):
        # Message 64511 v0: "Invalid mobile DBU" - see decodeMessage64510,
        # same reasoning (ErrMsg is a variable-length string). This is
        # what the xNAV650 sends instead of our configured messages if
        # our own mobile.dbu has a mistake in it
        errNo = payload[0]
        errMsg = payload[1:].split(b'\x00')[0].decode('utf-8', errors='replace')
        self.status['ErrNo'] = errNo
        self.status['ErrMsg'] = errMsg

    ####################################################################
    # One decodeMessageN (or decodeMessageNvV for a non-zero
    # MessageVersion) per message OXTS defines in oxts.dbu - mechanically
    # generated from oxts-nav/documentation/oxts.dbu and checked against
    # the manual for a sample of messages, not hand-transcribed (93
    # messages, many with several signals each, is too much to
    # transcribe by hand without errors). Signals are decoded in the
    # order oxts.dbu lists them - UCOM has no per-signal offset marker
    # of its own, so that order fully determines the byte layout for a
    # given MessageID/MessageVersion.
    def decodeMessage0(self, payload):
        # Message 0 v0: "Heartbeat"
        (SerialNumber,) = struct.unpack("<I", payload)
        self.status['SerialNumber'] = SerialNumber
        if SerialNumber == 0xFFFFFFFF: del self.status['SerialNumber']

    def decodeMessage100(self, payload):
        # Message 100 v0: "The SDN time offsets"
        (SDNOFF_source, SDNOFF) = struct.unpack("<Bq", payload)
        self.status['SDNOFF'] = SDNOFF
        self.status['SDNOFFSource'] = SDNOFF_source
        # Source 1 = GNSS (per the reference decoder's TimeSources enum -
        # not documented as numeric values in the manual itself). This is
        # what lets decode() maintain connection['timeOffset'] - see
        # __init__'s comment.
        if SDNOFF_source == 1:
            self._sdnToGpsOffsetS = SDNOFF / 1e9

    def decodeMessage54(self, payload):
        # Message 54 v0: "INS Status"
        (InsNavMode,) = struct.unpack("<B", payload)
        self.status['InsNavMode'] = InsNavMode
        if InsNavMode == 0xFF: del self.status['InsNavMode']
        # Also mirror into self.nav - matches ncomrx.py's own NavStatus,
        # which status.html's "Navigation status" row binds to (nav-InsNavMode)
        self.nav['InsNavMode'] = InsNavMode
        if InsNavMode == 0xFF: del self.nav['InsNavMode']

    def decodeMessage1(self, payload):
        # Message 1 v0: "OXTS navigation frame PVA"
        (Nano, Lat, Lon, Alt, Vn, Ve, Vd, Heading, Pitch, Roll) = struct.unpack("<qddddddddd", payload)
        self.nav['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.nav['Nano']
        self.nav['Lat'] = Lat
        if math.isnan(Lat): del self.nav['Lat']
        self.nav['Lon'] = Lon
        if math.isnan(Lon): del self.nav['Lon']
        self.nav['Alt'] = Alt
        if math.isnan(Alt): del self.nav['Alt']
        self.nav['Vn'] = Vn
        if math.isnan(Vn): del self.nav['Vn']
        self.nav['Ve'] = Ve
        if math.isnan(Ve): del self.nav['Ve']
        self.nav['Vd'] = Vd
        if math.isnan(Vd): del self.nav['Vd']
        self.nav['Heading'] = Heading
        if math.isnan(Heading): del self.nav['Heading']
        self.nav['Pitch'] = Pitch
        if math.isnan(Pitch): del self.nav['Pitch']
        self.nav['Roll'] = Roll
        if math.isnan(Roll): del self.nav['Roll']

    def decodeMessage1v1(self, payload):
        # Message 1 v1: "OXTS navigation frame PVA"
        (Lat, Lon, Alt, Vn, Ve, Vd, Heading, Pitch, Roll) = struct.unpack("<ddddddddd", payload)
        self.nav['Lat'] = Lat
        if math.isnan(Lat): del self.nav['Lat']
        self.nav['Lon'] = Lon
        if math.isnan(Lon): del self.nav['Lon']
        self.nav['Alt'] = Alt
        if math.isnan(Alt): del self.nav['Alt']
        self.nav['Vn'] = Vn
        if math.isnan(Vn): del self.nav['Vn']
        self.nav['Ve'] = Ve
        if math.isnan(Ve): del self.nav['Ve']
        self.nav['Vd'] = Vd
        if math.isnan(Vd): del self.nav['Vd']
        self.nav['Heading'] = Heading
        if math.isnan(Heading): del self.nav['Heading']
        self.nav['Pitch'] = Pitch
        if math.isnan(Pitch): del self.nav['Pitch']
        self.nav['Roll'] = Roll
        if math.isnan(Roll): del self.nav['Roll']

    def decodeMessage2(self, payload):
        # Message 2 v0: "ISO-8855 Earth fixed frame VA"
        (Nano, IsoVnX, IsoVnY, IsoVnZ, IsoYaw, IsoPitch, IsoRoll) = struct.unpack("<qdddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['IsoVnX'] = IsoVnX
        if math.isnan(IsoVnX): del self.status['IsoVnX']
        self.status['IsoVnY'] = IsoVnY
        if math.isnan(IsoVnY): del self.status['IsoVnY']
        self.status['IsoVnZ'] = IsoVnZ
        if math.isnan(IsoVnZ): del self.status['IsoVnZ']
        self.status['IsoYaw'] = IsoYaw
        if math.isnan(IsoYaw): del self.status['IsoYaw']
        self.status['IsoPitch'] = IsoPitch
        if math.isnan(IsoPitch): del self.status['IsoPitch']
        self.status['IsoRoll'] = IsoRoll
        if math.isnan(IsoRoll): del self.status['IsoRoll']

    def decodeMessage2v1(self, payload):
        # Message 2 v1: "ISO-8855 Earth fixed frame VA"
        (IsoVnX, IsoVnY, IsoVnZ, IsoYaw, IsoPitch, IsoRoll) = struct.unpack("<dddddd", payload)
        self.status['IsoVnX'] = IsoVnX
        if math.isnan(IsoVnX): del self.status['IsoVnX']
        self.status['IsoVnY'] = IsoVnY
        if math.isnan(IsoVnY): del self.status['IsoVnY']
        self.status['IsoVnZ'] = IsoVnZ
        if math.isnan(IsoVnZ): del self.status['IsoVnZ']
        self.status['IsoYaw'] = IsoYaw
        if math.isnan(IsoYaw): del self.status['IsoYaw']
        self.status['IsoPitch'] = IsoPitch
        if math.isnan(IsoPitch): del self.status['IsoPitch']
        self.status['IsoRoll'] = IsoRoll
        if math.isnan(IsoRoll): del self.status['IsoRoll']

    def decodeMessage3(self, payload):
        # Message 3 v0: "OXTS navigation frame PVA accuracies"
        (Nano, NorthAcc, EastAcc, AltAcc, VnAcc, VeAcc, VdAcc, VuAcc, HeadingAcc, PitchAcc, RollAcc) = struct.unpack("<qdddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['NorthAcc'] = NorthAcc
        if math.isnan(NorthAcc): del self.status['NorthAcc']
        self.status['EastAcc'] = EastAcc
        if math.isnan(EastAcc): del self.status['EastAcc']
        self.status['AltAcc'] = AltAcc
        if math.isnan(AltAcc): del self.status['AltAcc']
        self.status['VnAcc'] = VnAcc
        if math.isnan(VnAcc): del self.status['VnAcc']
        self.status['VeAcc'] = VeAcc
        if math.isnan(VeAcc): del self.status['VeAcc']
        self.status['VdAcc'] = VdAcc
        if math.isnan(VdAcc): del self.status['VdAcc']
        self.status['VuAcc'] = VuAcc
        if math.isnan(VuAcc): del self.status['VuAcc']
        self.status['HeadingAcc'] = HeadingAcc
        if math.isnan(HeadingAcc): del self.status['HeadingAcc']
        self.status['PitchAcc'] = PitchAcc
        if math.isnan(PitchAcc): del self.status['PitchAcc']
        self.status['RollAcc'] = RollAcc
        if math.isnan(RollAcc): del self.status['RollAcc']

    def decodeMessage3v1(self, payload):
        # Message 3 v1: "OXTS navigation frame PVA accuracies"
        (NorthAcc, EastAcc, AltAcc, VnAcc, VeAcc, VdAcc, VuAcc, HeadingAcc, PitchAcc, RollAcc) = struct.unpack("<dddddddddd", payload)
        self.status['NorthAcc'] = NorthAcc
        if math.isnan(NorthAcc): del self.status['NorthAcc']
        self.status['EastAcc'] = EastAcc
        if math.isnan(EastAcc): del self.status['EastAcc']
        self.status['AltAcc'] = AltAcc
        if math.isnan(AltAcc): del self.status['AltAcc']
        self.status['VnAcc'] = VnAcc
        if math.isnan(VnAcc): del self.status['VnAcc']
        self.status['VeAcc'] = VeAcc
        if math.isnan(VeAcc): del self.status['VeAcc']
        self.status['VdAcc'] = VdAcc
        if math.isnan(VdAcc): del self.status['VdAcc']
        self.status['VuAcc'] = VuAcc
        if math.isnan(VuAcc): del self.status['VuAcc']
        self.status['HeadingAcc'] = HeadingAcc
        if math.isnan(HeadingAcc): del self.status['HeadingAcc']
        self.status['PitchAcc'] = PitchAcc
        if math.isnan(PitchAcc): del self.status['PitchAcc']
        self.status['RollAcc'] = RollAcc
        if math.isnan(RollAcc): del self.status['RollAcc']

    def decodeMessage4(self, payload):
        # Message 4 v0: "ISO-8855 Earth fixed VA accuracies"
        (Nano, IsoVnXAcc, IsoVnYAcc, IsoVnZAcc, IsoYawAcc, IsoPitchAcc, IsoRollAcc) = struct.unpack("<qdddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['IsoVnXAcc'] = IsoVnXAcc
        if math.isnan(IsoVnXAcc): del self.status['IsoVnXAcc']
        self.status['IsoVnYAcc'] = IsoVnYAcc
        if math.isnan(IsoVnYAcc): del self.status['IsoVnYAcc']
        self.status['IsoVnZAcc'] = IsoVnZAcc
        if math.isnan(IsoVnZAcc): del self.status['IsoVnZAcc']
        self.status['IsoYawAcc'] = IsoYawAcc
        if math.isnan(IsoYawAcc): del self.status['IsoYawAcc']
        self.status['IsoPitchAcc'] = IsoPitchAcc
        if math.isnan(IsoPitchAcc): del self.status['IsoPitchAcc']
        self.status['IsoRollAcc'] = IsoRollAcc
        if math.isnan(IsoRollAcc): del self.status['IsoRollAcc']

    def decodeMessage4v1(self, payload):
        # Message 4 v1: "ISO-8855 Earth fixed VA accuracies"
        (IsoVnXAcc, IsoVnYAcc, IsoVnZAcc, IsoYawAcc, IsoPitchAcc, IsoRollAcc) = struct.unpack("<dddddd", payload)
        self.status['IsoVnXAcc'] = IsoVnXAcc
        if math.isnan(IsoVnXAcc): del self.status['IsoVnXAcc']
        self.status['IsoVnYAcc'] = IsoVnYAcc
        if math.isnan(IsoVnYAcc): del self.status['IsoVnYAcc']
        self.status['IsoVnZAcc'] = IsoVnZAcc
        if math.isnan(IsoVnZAcc): del self.status['IsoVnZAcc']
        self.status['IsoYawAcc'] = IsoYawAcc
        if math.isnan(IsoYawAcc): del self.status['IsoYawAcc']
        self.status['IsoPitchAcc'] = IsoPitchAcc
        if math.isnan(IsoPitchAcc): del self.status['IsoPitchAcc']
        self.status['IsoRollAcc'] = IsoRollAcc
        if math.isnan(IsoRollAcc): del self.status['IsoRollAcc']

    def decodeMessage5(self, payload):
        # Message 5 v0: "GNSS Status"
        (Nano, InsNavMode, GnssPosMode, GnssVelMode, GnssAttMode, GnssDiffAge, PriSatTrac, SecSatTrac, GnssPosNumSats, HeaSatTrac, GnssAttNumSats) = struct.unpack("<qBBBBdBBBBB", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['InsNavMode'] = InsNavMode
        if InsNavMode == 0xFF: del self.status['InsNavMode']
        self.status['GnssPosMode'] = GnssPosMode
        if GnssPosMode == 0xFF: del self.status['GnssPosMode']
        self.status['GnssVelMode'] = GnssVelMode
        if GnssVelMode == 0xFF: del self.status['GnssVelMode']
        self.status['GnssAttMode'] = GnssAttMode
        if GnssAttMode == 0xFF: del self.status['GnssAttMode']
        self.status['GnssDiffAge'] = GnssDiffAge
        if math.isnan(GnssDiffAge): del self.status['GnssDiffAge']
        self.status['PriSatTrac'] = PriSatTrac
        if PriSatTrac == 0xFF: del self.status['PriSatTrac']
        self.status['SecSatTrac'] = SecSatTrac
        if SecSatTrac == 0xFF: del self.status['SecSatTrac']
        self.status['GnssPosNumSats'] = GnssPosNumSats
        if GnssPosNumSats == 0xFF: del self.status['GnssPosNumSats']
        self.status['HeaSatTrac'] = HeaSatTrac
        if HeaSatTrac == 0xFF: del self.status['HeaSatTrac']
        self.status['GnssAttNumSats'] = GnssAttNumSats
        if GnssAttNumSats == 0xFF: del self.status['GnssAttNumSats']

    def decodeMessage5v1(self, payload):
        # Message 5 v1: "GNSS Status"
        (GnssPosMode, GnssVelMode, GnssAttMode, GnssDiffAge, PriSatTrac, SecSatTrac, GnssPosNumSats, HeaSatTrac, GnssAttNumSats) = struct.unpack("<BBBdBBBBB", payload)
        self.status['GnssPosMode'] = GnssPosMode
        if GnssPosMode == 0xFF: del self.status['GnssPosMode']
        self.status['GnssVelMode'] = GnssVelMode
        if GnssVelMode == 0xFF: del self.status['GnssVelMode']
        self.status['GnssAttMode'] = GnssAttMode
        if GnssAttMode == 0xFF: del self.status['GnssAttMode']
        self.status['GnssDiffAge'] = GnssDiffAge
        if math.isnan(GnssDiffAge): del self.status['GnssDiffAge']
        self.status['PriSatTrac'] = PriSatTrac
        if PriSatTrac == 0xFF: del self.status['PriSatTrac']
        self.status['SecSatTrac'] = SecSatTrac
        if SecSatTrac == 0xFF: del self.status['SecSatTrac']
        self.status['GnssPosNumSats'] = GnssPosNumSats
        if GnssPosNumSats == 0xFF: del self.status['GnssPosNumSats']
        self.status['HeaSatTrac'] = HeaSatTrac
        if HeaSatTrac == 0xFF: del self.status['HeaSatTrac']
        self.status['GnssAttNumSats'] = GnssAttNumSats
        if GnssAttNumSats == 0xFF: del self.status['GnssAttNumSats']

    def decodeMessage6(self, payload):
        # Message 6 v0: "OXTS horizontal frame velocity"
        (Nano, Vf, Vl, Vd) = struct.unpack("<qddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['Vf'] = Vf
        if math.isnan(Vf): del self.status['Vf']
        self.status['Vl'] = Vl
        if math.isnan(Vl): del self.status['Vl']
        self.status['Vd'] = Vd
        if math.isnan(Vd): del self.status['Vd']

    def decodeMessage6v1(self, payload):
        # Message 6 v1: "OXTS horizontal frame velocity"
        (Vf, Vl, Vd) = struct.unpack("<ddd", payload)
        self.status['Vf'] = Vf
        if math.isnan(Vf): del self.status['Vf']
        self.status['Vl'] = Vl
        if math.isnan(Vl): del self.status['Vl']
        self.status['Vd'] = Vd
        if math.isnan(Vd): del self.status['Vd']

    def decodeMessage7(self, payload):
        # Message 7 v0: "OXTS vehicle frame velocity"
        (Nano, Vx, Vy, Vz) = struct.unpack("<qddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['Vx'] = Vx
        if math.isnan(Vx): del self.status['Vx']
        self.status['Vy'] = Vy
        if math.isnan(Vy): del self.status['Vy']
        self.status['Vz'] = Vz
        if math.isnan(Vz): del self.status['Vz']

    def decodeMessage7v1(self, payload):
        # Message 7 v1: "OXTS vehicle frame velocity"
        (Vx, Vy, Vz) = struct.unpack("<ddd", payload)
        self.status['Vx'] = Vx
        if math.isnan(Vx): del self.status['Vx']
        self.status['Vy'] = Vy
        if math.isnan(Vy): del self.status['Vy']
        self.status['Vz'] = Vz
        if math.isnan(Vz): del self.status['Vz']

    def decodeMessage8(self, payload):
        # Message 8 v0: "ISO-8855 intermediate frame velocity"
        (Nano, IsoVhX, IsoVhY, IsoVhZ) = struct.unpack("<qddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['IsoVhX'] = IsoVhX
        if math.isnan(IsoVhX): del self.status['IsoVhX']
        self.status['IsoVhY'] = IsoVhY
        if math.isnan(IsoVhY): del self.status['IsoVhY']
        self.status['IsoVhZ'] = IsoVhZ
        if math.isnan(IsoVhZ): del self.status['IsoVhZ']

    def decodeMessage8v1(self, payload):
        # Message 8 v1: "ISO-8855 intermediate frame velocity"
        (IsoVhX, IsoVhY, IsoVhZ) = struct.unpack("<ddd", payload)
        self.status['IsoVhX'] = IsoVhX
        if math.isnan(IsoVhX): del self.status['IsoVhX']
        self.status['IsoVhY'] = IsoVhY
        if math.isnan(IsoVhY): del self.status['IsoVhY']
        self.status['IsoVhZ'] = IsoVhZ
        if math.isnan(IsoVhZ): del self.status['IsoVhZ']

    def decodeMessage9(self, payload):
        # Message 9 v0: "ISO-8855 vehicle system velocity"
        (Nano, IsoVoX, IsoVoY, IsoVoZ) = struct.unpack("<qddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['IsoVoX'] = IsoVoX
        if math.isnan(IsoVoX): del self.status['IsoVoX']
        self.status['IsoVoY'] = IsoVoY
        if math.isnan(IsoVoY): del self.status['IsoVoY']
        self.status['IsoVoZ'] = IsoVoZ
        if math.isnan(IsoVoZ): del self.status['IsoVoZ']

    def decodeMessage9v1(self, payload):
        # Message 9 v1: "ISO-8855 vehicle system velocity"
        (IsoVoX, IsoVoY, IsoVoZ) = struct.unpack("<ddd", payload)
        self.status['IsoVoX'] = IsoVoX
        if math.isnan(IsoVoX): del self.status['IsoVoX']
        self.status['IsoVoY'] = IsoVoY
        if math.isnan(IsoVoY): del self.status['IsoVoY']
        self.status['IsoVoZ'] = IsoVoZ
        if math.isnan(IsoVoZ): del self.status['IsoVoZ']

    def decodeMessage10(self, payload):
        # Message 10 v0: "OXTS vehicle frame inertial measurements"
        (Nano, Ax, Ay, Az, Jx, Jy, Jz, Wx, Wy, Wz, Yx, Yy, Yz) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['Ax'] = Ax
        if math.isnan(Ax): del self.status['Ax']
        self.status['Ay'] = Ay
        if math.isnan(Ay): del self.status['Ay']
        self.status['Az'] = Az
        if math.isnan(Az): del self.status['Az']
        self.status['Jx'] = Jx
        if math.isnan(Jx): del self.status['Jx']
        self.status['Jy'] = Jy
        if math.isnan(Jy): del self.status['Jy']
        self.status['Jz'] = Jz
        if math.isnan(Jz): del self.status['Jz']
        self.status['Wx'] = Wx
        if math.isnan(Wx): del self.status['Wx']
        self.status['Wy'] = Wy
        if math.isnan(Wy): del self.status['Wy']
        self.status['Wz'] = Wz
        if math.isnan(Wz): del self.status['Wz']
        self.status['Yx'] = Yx
        if math.isnan(Yx): del self.status['Yx']
        self.status['Yy'] = Yy
        if math.isnan(Yy): del self.status['Yy']
        self.status['Yz'] = Yz
        if math.isnan(Yz): del self.status['Yz']

    def decodeMessage10v1(self, payload):
        # Message 10 v1: "OXTS vehicle frame linear accelerations and angular rates"
        (Ax, Ay, Az, Wx, Wy, Wz) = struct.unpack("<dddddd", payload)
        self.status['Ax'] = Ax
        if math.isnan(Ax): del self.status['Ax']
        self.status['Ay'] = Ay
        if math.isnan(Ay): del self.status['Ay']
        self.status['Az'] = Az
        if math.isnan(Az): del self.status['Az']
        self.status['Wx'] = Wx
        if math.isnan(Wx): del self.status['Wx']
        self.status['Wy'] = Wy
        if math.isnan(Wy): del self.status['Wy']
        self.status['Wz'] = Wz
        if math.isnan(Wz): del self.status['Wz']

    def decodeMessage11(self, payload):
        # Message 11 v0: "OXTS navigation frame inertial measurements"
        (Nano, An, Ae, Ad, Jn, Je, Jd, Wn, We, Wd, Yn, Ye, Yd) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['An'] = An
        if math.isnan(An): del self.status['An']
        self.status['Ae'] = Ae
        if math.isnan(Ae): del self.status['Ae']
        self.status['Ad'] = Ad
        if math.isnan(Ad): del self.status['Ad']
        self.status['Jn'] = Jn
        if math.isnan(Jn): del self.status['Jn']
        self.status['Je'] = Je
        if math.isnan(Je): del self.status['Je']
        self.status['Jd'] = Jd
        if math.isnan(Jd): del self.status['Jd']
        self.status['Wn'] = Wn
        if math.isnan(Wn): del self.status['Wn']
        self.status['We'] = We
        if math.isnan(We): del self.status['We']
        self.status['Wd'] = Wd
        if math.isnan(Wd): del self.status['Wd']
        self.status['Yn'] = Yn
        if math.isnan(Yn): del self.status['Yn']
        self.status['Ye'] = Ye
        if math.isnan(Ye): del self.status['Ye']
        self.status['Yd'] = Yd
        if math.isnan(Yd): del self.status['Yd']

    def decodeMessage11v1(self, payload):
        # Message 11 v1: "OXTS navigation frame linear accelerations and angular rates"
        (An, Ae, Ad, Wn, We, Wd) = struct.unpack("<dddddd", payload)
        self.status['An'] = An
        if math.isnan(An): del self.status['An']
        self.status['Ae'] = Ae
        if math.isnan(Ae): del self.status['Ae']
        self.status['Ad'] = Ad
        if math.isnan(Ad): del self.status['Ad']
        self.status['Wn'] = Wn
        if math.isnan(Wn): del self.status['Wn']
        self.status['We'] = We
        if math.isnan(We): del self.status['We']
        self.status['Wd'] = Wd
        if math.isnan(Wd): del self.status['Wd']

    def decodeMessage12(self, payload):
        # Message 12 v0: "OXTS intermediate frame inertial measurements"
        (Nano, Af, Al, Ad, Jf, Jl, Jd, Wf, Wl, Wd, Yf, Yl, Yd) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['Af'] = Af
        if math.isnan(Af): del self.status['Af']
        self.status['Al'] = Al
        if math.isnan(Al): del self.status['Al']
        self.status['Ad'] = Ad
        if math.isnan(Ad): del self.status['Ad']
        self.status['Jf'] = Jf
        if math.isnan(Jf): del self.status['Jf']
        self.status['Jl'] = Jl
        if math.isnan(Jl): del self.status['Jl']
        self.status['Jd'] = Jd
        if math.isnan(Jd): del self.status['Jd']
        self.status['Wf'] = Wf
        if math.isnan(Wf): del self.status['Wf']
        self.status['Wl'] = Wl
        if math.isnan(Wl): del self.status['Wl']
        self.status['Wd'] = Wd
        if math.isnan(Wd): del self.status['Wd']
        self.status['Yf'] = Yf
        if math.isnan(Yf): del self.status['Yf']
        self.status['Yl'] = Yl
        if math.isnan(Yl): del self.status['Yl']
        self.status['Yd'] = Yd
        if math.isnan(Yd): del self.status['Yd']

    def decodeMessage12v1(self, payload):
        # Message 12 v1: "OXTS intermediate frame linear accelerations and angular rates"
        (Af, Al, Ad, Wf, Wl, Wd) = struct.unpack("<dddddd", payload)
        self.status['Af'] = Af
        if math.isnan(Af): del self.status['Af']
        self.status['Al'] = Al
        if math.isnan(Al): del self.status['Al']
        self.status['Ad'] = Ad
        if math.isnan(Ad): del self.status['Ad']
        self.status['Wf'] = Wf
        if math.isnan(Wf): del self.status['Wf']
        self.status['Wl'] = Wl
        if math.isnan(Wl): del self.status['Wl']
        self.status['Wd'] = Wd
        if math.isnan(Wd): del self.status['Wd']

    def decodeMessage13(self, payload):
        # Message 13 v0: "ISO-8855 Earth fixed inertial measurements"
        (Nano, IsoAnX, IsoAnY, IsoAnZ, IsoJnX, IsoJnY, IsoJnZ, IsoWnX, IsoWnY, IsoWnZ, IsoYnX, IsoYnY, IsoYnZ) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['IsoAnX'] = IsoAnX
        if math.isnan(IsoAnX): del self.status['IsoAnX']
        self.status['IsoAnY'] = IsoAnY
        if math.isnan(IsoAnY): del self.status['IsoAnY']
        self.status['IsoAnZ'] = IsoAnZ
        if math.isnan(IsoAnZ): del self.status['IsoAnZ']
        self.status['IsoJnX'] = IsoJnX
        if math.isnan(IsoJnX): del self.status['IsoJnX']
        self.status['IsoJnY'] = IsoJnY
        if math.isnan(IsoJnY): del self.status['IsoJnY']
        self.status['IsoJnZ'] = IsoJnZ
        if math.isnan(IsoJnZ): del self.status['IsoJnZ']
        self.status['IsoWnX'] = IsoWnX
        if math.isnan(IsoWnX): del self.status['IsoWnX']
        self.status['IsoWnY'] = IsoWnY
        if math.isnan(IsoWnY): del self.status['IsoWnY']
        self.status['IsoWnZ'] = IsoWnZ
        if math.isnan(IsoWnZ): del self.status['IsoWnZ']
        self.status['IsoYnX'] = IsoYnX
        if math.isnan(IsoYnX): del self.status['IsoYnX']
        self.status['IsoYnY'] = IsoYnY
        if math.isnan(IsoYnY): del self.status['IsoYnY']
        self.status['IsoYnZ'] = IsoYnZ
        if math.isnan(IsoYnZ): del self.status['IsoYnZ']

    def decodeMessage13v1(self, payload):
        # Message 13 v1: "ISO-8855 Earth fixed linear accelerations and angular rates"
        (IsoAnX, IsoAnY, IsoAnZ, IsoWnX, IsoWnY, IsoWnZ) = struct.unpack("<dddddd", payload)
        self.status['IsoAnX'] = IsoAnX
        if math.isnan(IsoAnX): del self.status['IsoAnX']
        self.status['IsoAnY'] = IsoAnY
        if math.isnan(IsoAnY): del self.status['IsoAnY']
        self.status['IsoAnZ'] = IsoAnZ
        if math.isnan(IsoAnZ): del self.status['IsoAnZ']
        self.status['IsoWnX'] = IsoWnX
        if math.isnan(IsoWnX): del self.status['IsoWnX']
        self.status['IsoWnY'] = IsoWnY
        if math.isnan(IsoWnY): del self.status['IsoWnY']
        self.status['IsoWnZ'] = IsoWnZ
        if math.isnan(IsoWnZ): del self.status['IsoWnZ']

    def decodeMessage14(self, payload):
        # Message 14 v0: "ISO-8855 intermediate frame inertial measurements"
        (Nano, IsoAhX, IsoAhY, IsoAhZ, IsoJhX, IsoJhY, IsoJhZ, IsoWhX, IsoWhY, IsoWhZ, IsoYhX, IsoYhY, IsoYhZ) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['IsoAhX'] = IsoAhX
        if math.isnan(IsoAhX): del self.status['IsoAhX']
        self.status['IsoAhY'] = IsoAhY
        if math.isnan(IsoAhY): del self.status['IsoAhY']
        self.status['IsoAhZ'] = IsoAhZ
        if math.isnan(IsoAhZ): del self.status['IsoAhZ']
        self.status['IsoJhX'] = IsoJhX
        if math.isnan(IsoJhX): del self.status['IsoJhX']
        self.status['IsoJhY'] = IsoJhY
        if math.isnan(IsoJhY): del self.status['IsoJhY']
        self.status['IsoJhZ'] = IsoJhZ
        if math.isnan(IsoJhZ): del self.status['IsoJhZ']
        self.status['IsoWhX'] = IsoWhX
        if math.isnan(IsoWhX): del self.status['IsoWhX']
        self.status['IsoWhY'] = IsoWhY
        if math.isnan(IsoWhY): del self.status['IsoWhY']
        self.status['IsoWhZ'] = IsoWhZ
        if math.isnan(IsoWhZ): del self.status['IsoWhZ']
        self.status['IsoYhX'] = IsoYhX
        if math.isnan(IsoYhX): del self.status['IsoYhX']
        self.status['IsoYhY'] = IsoYhY
        if math.isnan(IsoYhY): del self.status['IsoYhY']
        self.status['IsoYhZ'] = IsoYhZ
        if math.isnan(IsoYhZ): del self.status['IsoYhZ']

    def decodeMessage14v1(self, payload):
        # Message 14 v1: "ISO-8855 intermediate frame linear accelerations and angular rates"
        (IsoAhX, IsoAhY, IsoAhZ, IsoWhX, IsoWhY, IsoWhZ) = struct.unpack("<dddddd", payload)
        self.status['IsoAhX'] = IsoAhX
        if math.isnan(IsoAhX): del self.status['IsoAhX']
        self.status['IsoAhY'] = IsoAhY
        if math.isnan(IsoAhY): del self.status['IsoAhY']
        self.status['IsoAhZ'] = IsoAhZ
        if math.isnan(IsoAhZ): del self.status['IsoAhZ']
        self.status['IsoWhX'] = IsoWhX
        if math.isnan(IsoWhX): del self.status['IsoWhX']
        self.status['IsoWhY'] = IsoWhY
        if math.isnan(IsoWhY): del self.status['IsoWhY']
        self.status['IsoWhZ'] = IsoWhZ
        if math.isnan(IsoWhZ): del self.status['IsoWhZ']

    def decodeMessage15(self, payload):
        # Message 15 v0: "ISO-8855 vehicle frame inertial measurements"
        (Nano, IsoAoX, IsoAoY, IsoAoZ, IsoJoX, IsoJoY, IsoJoZ, IsoWoX, IsoWoY, IsoWoZ, IsoYoX, IsoYoY, IsoYoZ) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['IsoAoX'] = IsoAoX
        if math.isnan(IsoAoX): del self.status['IsoAoX']
        self.status['IsoAoY'] = IsoAoY
        if math.isnan(IsoAoY): del self.status['IsoAoY']
        self.status['IsoAoZ'] = IsoAoZ
        if math.isnan(IsoAoZ): del self.status['IsoAoZ']
        self.status['IsoJoX'] = IsoJoX
        if math.isnan(IsoJoX): del self.status['IsoJoX']
        self.status['IsoJoY'] = IsoJoY
        if math.isnan(IsoJoY): del self.status['IsoJoY']
        self.status['IsoJoZ'] = IsoJoZ
        if math.isnan(IsoJoZ): del self.status['IsoJoZ']
        self.status['IsoWoX'] = IsoWoX
        if math.isnan(IsoWoX): del self.status['IsoWoX']
        self.status['IsoWoY'] = IsoWoY
        if math.isnan(IsoWoY): del self.status['IsoWoY']
        self.status['IsoWoZ'] = IsoWoZ
        if math.isnan(IsoWoZ): del self.status['IsoWoZ']
        self.status['IsoYoX'] = IsoYoX
        if math.isnan(IsoYoX): del self.status['IsoYoX']
        self.status['IsoYoY'] = IsoYoY
        if math.isnan(IsoYoY): del self.status['IsoYoY']
        self.status['IsoYoZ'] = IsoYoZ
        if math.isnan(IsoYoZ): del self.status['IsoYoZ']

    def decodeMessage15v1(self, payload):
        # Message 15 v1: "ISO-8855 vehicle frame linear accelerations and angular rates"
        (IsoAoX, IsoAoY, IsoAoZ, IsoWoX, IsoWoY, IsoWoZ) = struct.unpack("<dddddd", payload)
        self.status['IsoAoX'] = IsoAoX
        if math.isnan(IsoAoX): del self.status['IsoAoX']
        self.status['IsoAoY'] = IsoAoY
        if math.isnan(IsoAoY): del self.status['IsoAoY']
        self.status['IsoAoZ'] = IsoAoZ
        if math.isnan(IsoAoZ): del self.status['IsoAoZ']
        self.status['IsoWoX'] = IsoWoX
        if math.isnan(IsoWoX): del self.status['IsoWoX']
        self.status['IsoWoY'] = IsoWoY
        if math.isnan(IsoWoY): del self.status['IsoWoY']
        self.status['IsoWoZ'] = IsoWoZ
        if math.isnan(IsoWoZ): del self.status['IsoWoZ']

    def decodeMessage16(self, payload):
        # Message 16 v0: "OXTS vehicle frame filtered inertial measurements"
        (Nano, FiltAx, FiltAy, FiltAz, FiltJx, FiltJy, FiltJz, FiltWx, FiltWy, FiltWz, FiltYx, FiltYy, FiltYz) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['FiltAx'] = FiltAx
        if math.isnan(FiltAx): del self.status['FiltAx']
        self.status['FiltAy'] = FiltAy
        if math.isnan(FiltAy): del self.status['FiltAy']
        self.status['FiltAz'] = FiltAz
        if math.isnan(FiltAz): del self.status['FiltAz']
        self.status['FiltJx'] = FiltJx
        if math.isnan(FiltJx): del self.status['FiltJx']
        self.status['FiltJy'] = FiltJy
        if math.isnan(FiltJy): del self.status['FiltJy']
        self.status['FiltJz'] = FiltJz
        if math.isnan(FiltJz): del self.status['FiltJz']
        self.status['FiltWx'] = FiltWx
        if math.isnan(FiltWx): del self.status['FiltWx']
        self.status['FiltWy'] = FiltWy
        if math.isnan(FiltWy): del self.status['FiltWy']
        self.status['FiltWz'] = FiltWz
        if math.isnan(FiltWz): del self.status['FiltWz']
        self.status['FiltYx'] = FiltYx
        if math.isnan(FiltYx): del self.status['FiltYx']
        self.status['FiltYy'] = FiltYy
        if math.isnan(FiltYy): del self.status['FiltYy']
        self.status['FiltYz'] = FiltYz
        if math.isnan(FiltYz): del self.status['FiltYz']

    def decodeMessage16v1(self, payload):
        # Message 16 v1: "OXTS vehicle frame filtered inertial measurements"
        (FiltAx, FiltAy, FiltAz, FiltJx, FiltJy, FiltJz, FiltWx, FiltWy, FiltWz, FiltYx, FiltYy, FiltYz) = struct.unpack("<dddddddddddd", payload)
        self.status['FiltAx'] = FiltAx
        if math.isnan(FiltAx): del self.status['FiltAx']
        self.status['FiltAy'] = FiltAy
        if math.isnan(FiltAy): del self.status['FiltAy']
        self.status['FiltAz'] = FiltAz
        if math.isnan(FiltAz): del self.status['FiltAz']
        self.status['FiltJx'] = FiltJx
        if math.isnan(FiltJx): del self.status['FiltJx']
        self.status['FiltJy'] = FiltJy
        if math.isnan(FiltJy): del self.status['FiltJy']
        self.status['FiltJz'] = FiltJz
        if math.isnan(FiltJz): del self.status['FiltJz']
        self.status['FiltWx'] = FiltWx
        if math.isnan(FiltWx): del self.status['FiltWx']
        self.status['FiltWy'] = FiltWy
        if math.isnan(FiltWy): del self.status['FiltWy']
        self.status['FiltWz'] = FiltWz
        if math.isnan(FiltWz): del self.status['FiltWz']
        self.status['FiltYx'] = FiltYx
        if math.isnan(FiltYx): del self.status['FiltYx']
        self.status['FiltYy'] = FiltYy
        if math.isnan(FiltYy): del self.status['FiltYy']
        self.status['FiltYz'] = FiltYz
        if math.isnan(FiltYz): del self.status['FiltYz']

    def decodeMessage17(self, payload):
        # Message 17 v0: "OXTS navigation frame filtered inertial measurements"
        (Nano, FiltAn, FiltAe, FiltAd, FiltJn, FiltJe, FiltJd, FiltWn, FiltWe, FiltWd, FiltYn, FiltYe, FiltYd) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['FiltAn'] = FiltAn
        if math.isnan(FiltAn): del self.status['FiltAn']
        self.status['FiltAe'] = FiltAe
        if math.isnan(FiltAe): del self.status['FiltAe']
        self.status['FiltAd'] = FiltAd
        if math.isnan(FiltAd): del self.status['FiltAd']
        self.status['FiltJn'] = FiltJn
        if math.isnan(FiltJn): del self.status['FiltJn']
        self.status['FiltJe'] = FiltJe
        if math.isnan(FiltJe): del self.status['FiltJe']
        self.status['FiltJd'] = FiltJd
        if math.isnan(FiltJd): del self.status['FiltJd']
        self.status['FiltWn'] = FiltWn
        if math.isnan(FiltWn): del self.status['FiltWn']
        self.status['FiltWe'] = FiltWe
        if math.isnan(FiltWe): del self.status['FiltWe']
        self.status['FiltWd'] = FiltWd
        if math.isnan(FiltWd): del self.status['FiltWd']
        self.status['FiltYn'] = FiltYn
        if math.isnan(FiltYn): del self.status['FiltYn']
        self.status['FiltYe'] = FiltYe
        if math.isnan(FiltYe): del self.status['FiltYe']
        self.status['FiltYd'] = FiltYd
        if math.isnan(FiltYd): del self.status['FiltYd']

    def decodeMessage17v1(self, payload):
        # Message 17 v1: "OXTS navigation frame filtered inertial measurements"
        (FiltAn, FiltAe, FiltAd, FiltJn, FiltJe, FiltJd, FiltWn, FiltWe, FiltWd, FiltYn, FiltYe, FiltYd) = struct.unpack("<dddddddddddd", payload)
        self.status['FiltAn'] = FiltAn
        if math.isnan(FiltAn): del self.status['FiltAn']
        self.status['FiltAe'] = FiltAe
        if math.isnan(FiltAe): del self.status['FiltAe']
        self.status['FiltAd'] = FiltAd
        if math.isnan(FiltAd): del self.status['FiltAd']
        self.status['FiltJn'] = FiltJn
        if math.isnan(FiltJn): del self.status['FiltJn']
        self.status['FiltJe'] = FiltJe
        if math.isnan(FiltJe): del self.status['FiltJe']
        self.status['FiltJd'] = FiltJd
        if math.isnan(FiltJd): del self.status['FiltJd']
        self.status['FiltWn'] = FiltWn
        if math.isnan(FiltWn): del self.status['FiltWn']
        self.status['FiltWe'] = FiltWe
        if math.isnan(FiltWe): del self.status['FiltWe']
        self.status['FiltWd'] = FiltWd
        if math.isnan(FiltWd): del self.status['FiltWd']
        self.status['FiltYn'] = FiltYn
        if math.isnan(FiltYn): del self.status['FiltYn']
        self.status['FiltYe'] = FiltYe
        if math.isnan(FiltYe): del self.status['FiltYe']
        self.status['FiltYd'] = FiltYd
        if math.isnan(FiltYd): del self.status['FiltYd']

    def decodeMessage18(self, payload):
        # Message 18 v0: "OXTS intermediate frame filtered inertial measurements"
        (Nano, FiltAf, FiltAl, FiltAd, FiltJf, FiltJl, FiltJd, FiltWf, FiltWl, FiltWd, FiltYf, FiltYl, FiltYd) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['FiltAf'] = FiltAf
        if math.isnan(FiltAf): del self.status['FiltAf']
        self.status['FiltAl'] = FiltAl
        if math.isnan(FiltAl): del self.status['FiltAl']
        self.status['FiltAd'] = FiltAd
        if math.isnan(FiltAd): del self.status['FiltAd']
        self.status['FiltJf'] = FiltJf
        if math.isnan(FiltJf): del self.status['FiltJf']
        self.status['FiltJl'] = FiltJl
        if math.isnan(FiltJl): del self.status['FiltJl']
        self.status['FiltJd'] = FiltJd
        if math.isnan(FiltJd): del self.status['FiltJd']
        self.status['FiltWf'] = FiltWf
        if math.isnan(FiltWf): del self.status['FiltWf']
        self.status['FiltWl'] = FiltWl
        if math.isnan(FiltWl): del self.status['FiltWl']
        self.status['FiltWd'] = FiltWd
        if math.isnan(FiltWd): del self.status['FiltWd']
        self.status['FiltYf'] = FiltYf
        if math.isnan(FiltYf): del self.status['FiltYf']
        self.status['FiltYl'] = FiltYl
        if math.isnan(FiltYl): del self.status['FiltYl']
        self.status['FiltYd'] = FiltYd
        if math.isnan(FiltYd): del self.status['FiltYd']

    def decodeMessage18v1(self, payload):
        # Message 18 v1: "OXTS intermediate frame filtered inertial measurements"
        (FiltAf, FiltAl, FiltAd, FiltJf, FiltJl, FiltJd, FiltWf, FiltWl, FiltWd, FiltYf, FiltYl, FiltYd) = struct.unpack("<dddddddddddd", payload)
        self.status['FiltAf'] = FiltAf
        if math.isnan(FiltAf): del self.status['FiltAf']
        self.status['FiltAl'] = FiltAl
        if math.isnan(FiltAl): del self.status['FiltAl']
        self.status['FiltAd'] = FiltAd
        if math.isnan(FiltAd): del self.status['FiltAd']
        self.status['FiltJf'] = FiltJf
        if math.isnan(FiltJf): del self.status['FiltJf']
        self.status['FiltJl'] = FiltJl
        if math.isnan(FiltJl): del self.status['FiltJl']
        self.status['FiltJd'] = FiltJd
        if math.isnan(FiltJd): del self.status['FiltJd']
        self.status['FiltWf'] = FiltWf
        if math.isnan(FiltWf): del self.status['FiltWf']
        self.status['FiltWl'] = FiltWl
        if math.isnan(FiltWl): del self.status['FiltWl']
        self.status['FiltWd'] = FiltWd
        if math.isnan(FiltWd): del self.status['FiltWd']
        self.status['FiltYf'] = FiltYf
        if math.isnan(FiltYf): del self.status['FiltYf']
        self.status['FiltYl'] = FiltYl
        if math.isnan(FiltYl): del self.status['FiltYl']
        self.status['FiltYd'] = FiltYd
        if math.isnan(FiltYd): del self.status['FiltYd']

    def decodeMessage19(self, payload):
        # Message 19 v0: "ISO-8855 Earth fixed filtered inertial measurements"
        (Nano, FiltIsoAnX, FiltIsoAnY, FiltIsoAnZ, FiltIsoJnX, FiltIsoJnY, FiltIsoJnZ, FiltWnX, FiltWnY, FiltWnZ, FiltIsoYnX, FiltIsoYnY, FiltIsoYnZ) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['FiltIsoAnX'] = FiltIsoAnX
        if math.isnan(FiltIsoAnX): del self.status['FiltIsoAnX']
        self.status['FiltIsoAnY'] = FiltIsoAnY
        if math.isnan(FiltIsoAnY): del self.status['FiltIsoAnY']
        self.status['FiltIsoAnZ'] = FiltIsoAnZ
        if math.isnan(FiltIsoAnZ): del self.status['FiltIsoAnZ']
        self.status['FiltIsoJnX'] = FiltIsoJnX
        if math.isnan(FiltIsoJnX): del self.status['FiltIsoJnX']
        self.status['FiltIsoJnY'] = FiltIsoJnY
        if math.isnan(FiltIsoJnY): del self.status['FiltIsoJnY']
        self.status['FiltIsoJnZ'] = FiltIsoJnZ
        if math.isnan(FiltIsoJnZ): del self.status['FiltIsoJnZ']
        self.status['FiltWnX'] = FiltWnX
        if math.isnan(FiltWnX): del self.status['FiltWnX']
        self.status['FiltWnY'] = FiltWnY
        if math.isnan(FiltWnY): del self.status['FiltWnY']
        self.status['FiltWnZ'] = FiltWnZ
        if math.isnan(FiltWnZ): del self.status['FiltWnZ']
        self.status['FiltIsoYnX'] = FiltIsoYnX
        if math.isnan(FiltIsoYnX): del self.status['FiltIsoYnX']
        self.status['FiltIsoYnY'] = FiltIsoYnY
        if math.isnan(FiltIsoYnY): del self.status['FiltIsoYnY']
        self.status['FiltIsoYnZ'] = FiltIsoYnZ
        if math.isnan(FiltIsoYnZ): del self.status['FiltIsoYnZ']

    def decodeMessage19v1(self, payload):
        # Message 19 v1: "ISO-8855 Earth fixed filtered inertial measurements"
        (FiltIsoAnX, FiltIsoAnY, FiltIsoAnZ, FiltIsoJnX, FiltIsoJnY, FiltIsoJnZ, FiltWnX, FiltWnY, FiltWnZ, FiltIsoYnX, FiltIsoYnY, FiltIsoYnZ) = struct.unpack("<dddddddddddd", payload)
        self.status['FiltIsoAnX'] = FiltIsoAnX
        if math.isnan(FiltIsoAnX): del self.status['FiltIsoAnX']
        self.status['FiltIsoAnY'] = FiltIsoAnY
        if math.isnan(FiltIsoAnY): del self.status['FiltIsoAnY']
        self.status['FiltIsoAnZ'] = FiltIsoAnZ
        if math.isnan(FiltIsoAnZ): del self.status['FiltIsoAnZ']
        self.status['FiltIsoJnX'] = FiltIsoJnX
        if math.isnan(FiltIsoJnX): del self.status['FiltIsoJnX']
        self.status['FiltIsoJnY'] = FiltIsoJnY
        if math.isnan(FiltIsoJnY): del self.status['FiltIsoJnY']
        self.status['FiltIsoJnZ'] = FiltIsoJnZ
        if math.isnan(FiltIsoJnZ): del self.status['FiltIsoJnZ']
        self.status['FiltWnX'] = FiltWnX
        if math.isnan(FiltWnX): del self.status['FiltWnX']
        self.status['FiltWnY'] = FiltWnY
        if math.isnan(FiltWnY): del self.status['FiltWnY']
        self.status['FiltWnZ'] = FiltWnZ
        if math.isnan(FiltWnZ): del self.status['FiltWnZ']
        self.status['FiltIsoYnX'] = FiltIsoYnX
        if math.isnan(FiltIsoYnX): del self.status['FiltIsoYnX']
        self.status['FiltIsoYnY'] = FiltIsoYnY
        if math.isnan(FiltIsoYnY): del self.status['FiltIsoYnY']
        self.status['FiltIsoYnZ'] = FiltIsoYnZ
        if math.isnan(FiltIsoYnZ): del self.status['FiltIsoYnZ']

    def decodeMessage20(self, payload):
        # Message 20 v0: "ISO-8855 intermediate frame filtered inertial measurements"
        (Nano, FiltIsoAhX, FiltIsoAhY, FiltIsoAhZ, FiltIsoJhX, FiltIsoJhY, FiltIsoJhZ, FiltWhX, FiltWhY, FiltWhZ, FiltIsoYhX, FiltIsoYhY, FiltIsoYhZ) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['FiltIsoAhX'] = FiltIsoAhX
        if math.isnan(FiltIsoAhX): del self.status['FiltIsoAhX']
        self.status['FiltIsoAhY'] = FiltIsoAhY
        if math.isnan(FiltIsoAhY): del self.status['FiltIsoAhY']
        self.status['FiltIsoAhZ'] = FiltIsoAhZ
        if math.isnan(FiltIsoAhZ): del self.status['FiltIsoAhZ']
        self.status['FiltIsoJhX'] = FiltIsoJhX
        if math.isnan(FiltIsoJhX): del self.status['FiltIsoJhX']
        self.status['FiltIsoJhY'] = FiltIsoJhY
        if math.isnan(FiltIsoJhY): del self.status['FiltIsoJhY']
        self.status['FiltIsoJhZ'] = FiltIsoJhZ
        if math.isnan(FiltIsoJhZ): del self.status['FiltIsoJhZ']
        self.status['FiltWhX'] = FiltWhX
        if math.isnan(FiltWhX): del self.status['FiltWhX']
        self.status['FiltWhY'] = FiltWhY
        if math.isnan(FiltWhY): del self.status['FiltWhY']
        self.status['FiltWhZ'] = FiltWhZ
        if math.isnan(FiltWhZ): del self.status['FiltWhZ']
        self.status['FiltIsoYhX'] = FiltIsoYhX
        if math.isnan(FiltIsoYhX): del self.status['FiltIsoYhX']
        self.status['FiltIsoYhY'] = FiltIsoYhY
        if math.isnan(FiltIsoYhY): del self.status['FiltIsoYhY']
        self.status['FiltIsoYhZ'] = FiltIsoYhZ
        if math.isnan(FiltIsoYhZ): del self.status['FiltIsoYhZ']

    def decodeMessage20v1(self, payload):
        # Message 20 v1: "ISO-8855 intermediate frame filtered inertial measurements"
        (FiltIsoAhX, FiltIsoAhY, FiltIsoAhZ, FiltIsoJhX, FiltIsoJhY, FiltIsoJhZ, FiltWhX, FiltWhY, FiltWhZ, FiltIsoYhX, FiltIsoYhY, FiltIsoYhZ) = struct.unpack("<dddddddddddd", payload)
        self.status['FiltIsoAhX'] = FiltIsoAhX
        if math.isnan(FiltIsoAhX): del self.status['FiltIsoAhX']
        self.status['FiltIsoAhY'] = FiltIsoAhY
        if math.isnan(FiltIsoAhY): del self.status['FiltIsoAhY']
        self.status['FiltIsoAhZ'] = FiltIsoAhZ
        if math.isnan(FiltIsoAhZ): del self.status['FiltIsoAhZ']
        self.status['FiltIsoJhX'] = FiltIsoJhX
        if math.isnan(FiltIsoJhX): del self.status['FiltIsoJhX']
        self.status['FiltIsoJhY'] = FiltIsoJhY
        if math.isnan(FiltIsoJhY): del self.status['FiltIsoJhY']
        self.status['FiltIsoJhZ'] = FiltIsoJhZ
        if math.isnan(FiltIsoJhZ): del self.status['FiltIsoJhZ']
        self.status['FiltWhX'] = FiltWhX
        if math.isnan(FiltWhX): del self.status['FiltWhX']
        self.status['FiltWhY'] = FiltWhY
        if math.isnan(FiltWhY): del self.status['FiltWhY']
        self.status['FiltWhZ'] = FiltWhZ
        if math.isnan(FiltWhZ): del self.status['FiltWhZ']
        self.status['FiltIsoYhX'] = FiltIsoYhX
        if math.isnan(FiltIsoYhX): del self.status['FiltIsoYhX']
        self.status['FiltIsoYhY'] = FiltIsoYhY
        if math.isnan(FiltIsoYhY): del self.status['FiltIsoYhY']
        self.status['FiltIsoYhZ'] = FiltIsoYhZ
        if math.isnan(FiltIsoYhZ): del self.status['FiltIsoYhZ']

    def decodeMessage21(self, payload):
        # Message 21 v0: "ISO-8855 vehicle frame filtered inertial measurements"
        (Nano, FiltIsoAoX, FiltIsoAoY, FiltIsoAoZ, FiltIsoJoX, FiltIsoJoY, FiltIsoJoZ, FiltWoX, FiltWoY, FiltWoZ, FiltIsoYoX, FiltIsoYoY, FiltIsoYoZ) = struct.unpack("<qdddddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['FiltIsoAoX'] = FiltIsoAoX
        if math.isnan(FiltIsoAoX): del self.status['FiltIsoAoX']
        self.status['FiltIsoAoY'] = FiltIsoAoY
        if math.isnan(FiltIsoAoY): del self.status['FiltIsoAoY']
        self.status['FiltIsoAoZ'] = FiltIsoAoZ
        if math.isnan(FiltIsoAoZ): del self.status['FiltIsoAoZ']
        self.status['FiltIsoJoX'] = FiltIsoJoX
        if math.isnan(FiltIsoJoX): del self.status['FiltIsoJoX']
        self.status['FiltIsoJoY'] = FiltIsoJoY
        if math.isnan(FiltIsoJoY): del self.status['FiltIsoJoY']
        self.status['FiltIsoJoZ'] = FiltIsoJoZ
        if math.isnan(FiltIsoJoZ): del self.status['FiltIsoJoZ']
        self.status['FiltWoX'] = FiltWoX
        if math.isnan(FiltWoX): del self.status['FiltWoX']
        self.status['FiltWoY'] = FiltWoY
        if math.isnan(FiltWoY): del self.status['FiltWoY']
        self.status['FiltWoZ'] = FiltWoZ
        if math.isnan(FiltWoZ): del self.status['FiltWoZ']
        self.status['FiltIsoYoX'] = FiltIsoYoX
        if math.isnan(FiltIsoYoX): del self.status['FiltIsoYoX']
        self.status['FiltIsoYoY'] = FiltIsoYoY
        if math.isnan(FiltIsoYoY): del self.status['FiltIsoYoY']
        self.status['FiltIsoYoZ'] = FiltIsoYoZ
        if math.isnan(FiltIsoYoZ): del self.status['FiltIsoYoZ']

    def decodeMessage21v1(self, payload):
        # Message 21 v1: "ISO-8855 vehicle frame filtered inertial measurements"
        (FiltIsoAoX, FiltIsoAoY, FiltIsoAoZ, FiltIsoJoX, FiltIsoJoY, FiltIsoJoZ, FiltWoX, FiltWoY, FiltWoZ, FiltIsoYoX, FiltIsoYoY, FiltIsoYoZ) = struct.unpack("<dddddddddddd", payload)
        self.status['FiltIsoAoX'] = FiltIsoAoX
        if math.isnan(FiltIsoAoX): del self.status['FiltIsoAoX']
        self.status['FiltIsoAoY'] = FiltIsoAoY
        if math.isnan(FiltIsoAoY): del self.status['FiltIsoAoY']
        self.status['FiltIsoAoZ'] = FiltIsoAoZ
        if math.isnan(FiltIsoAoZ): del self.status['FiltIsoAoZ']
        self.status['FiltIsoJoX'] = FiltIsoJoX
        if math.isnan(FiltIsoJoX): del self.status['FiltIsoJoX']
        self.status['FiltIsoJoY'] = FiltIsoJoY
        if math.isnan(FiltIsoJoY): del self.status['FiltIsoJoY']
        self.status['FiltIsoJoZ'] = FiltIsoJoZ
        if math.isnan(FiltIsoJoZ): del self.status['FiltIsoJoZ']
        self.status['FiltWoX'] = FiltWoX
        if math.isnan(FiltWoX): del self.status['FiltWoX']
        self.status['FiltWoY'] = FiltWoY
        if math.isnan(FiltWoY): del self.status['FiltWoY']
        self.status['FiltWoZ'] = FiltWoZ
        if math.isnan(FiltWoZ): del self.status['FiltWoZ']
        self.status['FiltIsoYoX'] = FiltIsoYoX
        if math.isnan(FiltIsoYoX): del self.status['FiltIsoYoX']
        self.status['FiltIsoYoY'] = FiltIsoYoY
        if math.isnan(FiltIsoYoY): del self.status['FiltIsoYoY']
        self.status['FiltIsoYoZ'] = FiltIsoYoZ
        if math.isnan(FiltIsoYoZ): del self.status['FiltIsoYoZ']

    def decodeMessage22(self, payload):
        # Message 22 v0: "Point of interest 1"
        (Nano, MeasPt1_Vf, MeasPt1_Vl, MeasPt1_Vd, MeasPt1_Speed2d, MeasPt1_Af, MeasPt1_Al, MeasPt1_Ad, MeasPt1_Track, MeasPt1_Slip, MeasPt1_Curvature) = struct.unpack("<qdddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['MeasPt1_Vf'] = MeasPt1_Vf
        if math.isnan(MeasPt1_Vf): del self.status['MeasPt1_Vf']
        self.status['MeasPt1_Vl'] = MeasPt1_Vl
        if math.isnan(MeasPt1_Vl): del self.status['MeasPt1_Vl']
        self.status['MeasPt1_Vd'] = MeasPt1_Vd
        if math.isnan(MeasPt1_Vd): del self.status['MeasPt1_Vd']
        self.status['MeasPt1_Speed2d'] = MeasPt1_Speed2d
        if math.isnan(MeasPt1_Speed2d): del self.status['MeasPt1_Speed2d']
        self.status['MeasPt1_Af'] = MeasPt1_Af
        if math.isnan(MeasPt1_Af): del self.status['MeasPt1_Af']
        self.status['MeasPt1_Al'] = MeasPt1_Al
        if math.isnan(MeasPt1_Al): del self.status['MeasPt1_Al']
        self.status['MeasPt1_Ad'] = MeasPt1_Ad
        if math.isnan(MeasPt1_Ad): del self.status['MeasPt1_Ad']
        self.status['MeasPt1_Track'] = MeasPt1_Track
        if math.isnan(MeasPt1_Track): del self.status['MeasPt1_Track']
        self.status['MeasPt1_Slip'] = MeasPt1_Slip
        if math.isnan(MeasPt1_Slip): del self.status['MeasPt1_Slip']
        self.status['MeasPt1_Curvature'] = MeasPt1_Curvature
        if math.isnan(MeasPt1_Curvature): del self.status['MeasPt1_Curvature']

    def decodeMessage22v1(self, payload):
        # Message 22 v1: "Point of interest 1"
        (MeasPt1_Vf, MeasPt1_Vl, MeasPt1_Vd, MeasPt1_Speed2d, MeasPt1_Af, MeasPt1_Al, MeasPt1_Ad, MeasPt1_An, MeasPt1_Ae, MeasPt1_Track, MeasPt1_Slip, MeasPt1_Curvature) = struct.unpack("<dddddddddddd", payload)
        self.status['MeasPt1_Vf'] = MeasPt1_Vf
        if math.isnan(MeasPt1_Vf): del self.status['MeasPt1_Vf']
        self.status['MeasPt1_Vl'] = MeasPt1_Vl
        if math.isnan(MeasPt1_Vl): del self.status['MeasPt1_Vl']
        self.status['MeasPt1_Vd'] = MeasPt1_Vd
        if math.isnan(MeasPt1_Vd): del self.status['MeasPt1_Vd']
        self.status['MeasPt1_Speed2d'] = MeasPt1_Speed2d
        if math.isnan(MeasPt1_Speed2d): del self.status['MeasPt1_Speed2d']
        self.status['MeasPt1_Af'] = MeasPt1_Af
        if math.isnan(MeasPt1_Af): del self.status['MeasPt1_Af']
        self.status['MeasPt1_Al'] = MeasPt1_Al
        if math.isnan(MeasPt1_Al): del self.status['MeasPt1_Al']
        self.status['MeasPt1_Ad'] = MeasPt1_Ad
        if math.isnan(MeasPt1_Ad): del self.status['MeasPt1_Ad']
        self.status['MeasPt1_An'] = MeasPt1_An
        if math.isnan(MeasPt1_An): del self.status['MeasPt1_An']
        self.status['MeasPt1_Ae'] = MeasPt1_Ae
        if math.isnan(MeasPt1_Ae): del self.status['MeasPt1_Ae']
        self.status['MeasPt1_Track'] = MeasPt1_Track
        if math.isnan(MeasPt1_Track): del self.status['MeasPt1_Track']
        self.status['MeasPt1_Slip'] = MeasPt1_Slip
        if math.isnan(MeasPt1_Slip): del self.status['MeasPt1_Slip']
        self.status['MeasPt1_Curvature'] = MeasPt1_Curvature
        if math.isnan(MeasPt1_Curvature): del self.status['MeasPt1_Curvature']

    def decodeMessage23(self, payload):
        # Message 23 v0: "Point of interest 2"
        (Nano, MeasPt2_Vf, MeasPt2_Vl, MeasPt2_Vd, MeasPt2_Speed2d, MeasPt2_Af, MeasPt2_Al, MeasPt2_Ad, MeasPt2_Track, MeasPt2_Slip, MeasPt2_Curvature) = struct.unpack("<qdddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['MeasPt2_Vf'] = MeasPt2_Vf
        if math.isnan(MeasPt2_Vf): del self.status['MeasPt2_Vf']
        self.status['MeasPt2_Vl'] = MeasPt2_Vl
        if math.isnan(MeasPt2_Vl): del self.status['MeasPt2_Vl']
        self.status['MeasPt2_Vd'] = MeasPt2_Vd
        if math.isnan(MeasPt2_Vd): del self.status['MeasPt2_Vd']
        self.status['MeasPt2_Speed2d'] = MeasPt2_Speed2d
        if math.isnan(MeasPt2_Speed2d): del self.status['MeasPt2_Speed2d']
        self.status['MeasPt2_Af'] = MeasPt2_Af
        if math.isnan(MeasPt2_Af): del self.status['MeasPt2_Af']
        self.status['MeasPt2_Al'] = MeasPt2_Al
        if math.isnan(MeasPt2_Al): del self.status['MeasPt2_Al']
        self.status['MeasPt2_Ad'] = MeasPt2_Ad
        if math.isnan(MeasPt2_Ad): del self.status['MeasPt2_Ad']
        self.status['MeasPt2_Track'] = MeasPt2_Track
        if math.isnan(MeasPt2_Track): del self.status['MeasPt2_Track']
        self.status['MeasPt2_Slip'] = MeasPt2_Slip
        if math.isnan(MeasPt2_Slip): del self.status['MeasPt2_Slip']
        self.status['MeasPt2_Curvature'] = MeasPt2_Curvature
        if math.isnan(MeasPt2_Curvature): del self.status['MeasPt2_Curvature']

    def decodeMessage23v1(self, payload):
        # Message 23 v1: "Point of interest 2"
        (MeasPt2_Vf, MeasPt2_Vl, MeasPt2_Vd, MeasPt2_Speed2d, MeasPt2_Af, MeasPt2_Al, MeasPt2_Ad, MeasPt2_An, MeasPt2_Ae, MeasPt2_Track, MeasPt2_Slip, MeasPt2_Curvature) = struct.unpack("<dddddddddddd", payload)
        self.status['MeasPt2_Vf'] = MeasPt2_Vf
        if math.isnan(MeasPt2_Vf): del self.status['MeasPt2_Vf']
        self.status['MeasPt2_Vl'] = MeasPt2_Vl
        if math.isnan(MeasPt2_Vl): del self.status['MeasPt2_Vl']
        self.status['MeasPt2_Vd'] = MeasPt2_Vd
        if math.isnan(MeasPt2_Vd): del self.status['MeasPt2_Vd']
        self.status['MeasPt2_Speed2d'] = MeasPt2_Speed2d
        if math.isnan(MeasPt2_Speed2d): del self.status['MeasPt2_Speed2d']
        self.status['MeasPt2_Af'] = MeasPt2_Af
        if math.isnan(MeasPt2_Af): del self.status['MeasPt2_Af']
        self.status['MeasPt2_Al'] = MeasPt2_Al
        if math.isnan(MeasPt2_Al): del self.status['MeasPt2_Al']
        self.status['MeasPt2_Ad'] = MeasPt2_Ad
        if math.isnan(MeasPt2_Ad): del self.status['MeasPt2_Ad']
        self.status['MeasPt2_An'] = MeasPt2_An
        if math.isnan(MeasPt2_An): del self.status['MeasPt2_An']
        self.status['MeasPt2_Ae'] = MeasPt2_Ae
        if math.isnan(MeasPt2_Ae): del self.status['MeasPt2_Ae']
        self.status['MeasPt2_Track'] = MeasPt2_Track
        if math.isnan(MeasPt2_Track): del self.status['MeasPt2_Track']
        self.status['MeasPt2_Slip'] = MeasPt2_Slip
        if math.isnan(MeasPt2_Slip): del self.status['MeasPt2_Slip']
        self.status['MeasPt2_Curvature'] = MeasPt2_Curvature
        if math.isnan(MeasPt2_Curvature): del self.status['MeasPt2_Curvature']

    def decodeMessage24(self, payload):
        # Message 24 v0: "Point of interest 3"
        (Nano, MeasPt3_Vf, MeasPt3_Vl, MeasPt3_Vd, MeasPt3_Speed2d, MeasPt3_Af, MeasPt3_Al, MeasPt3_Ad, MeasPt3_Track, MeasPt3_Slip, MeasPt3_Curvature) = struct.unpack("<qdddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['MeasPt3_Vf'] = MeasPt3_Vf
        if math.isnan(MeasPt3_Vf): del self.status['MeasPt3_Vf']
        self.status['MeasPt3_Vl'] = MeasPt3_Vl
        if math.isnan(MeasPt3_Vl): del self.status['MeasPt3_Vl']
        self.status['MeasPt3_Vd'] = MeasPt3_Vd
        if math.isnan(MeasPt3_Vd): del self.status['MeasPt3_Vd']
        self.status['MeasPt3_Speed2d'] = MeasPt3_Speed2d
        if math.isnan(MeasPt3_Speed2d): del self.status['MeasPt3_Speed2d']
        self.status['MeasPt3_Af'] = MeasPt3_Af
        if math.isnan(MeasPt3_Af): del self.status['MeasPt3_Af']
        self.status['MeasPt3_Al'] = MeasPt3_Al
        if math.isnan(MeasPt3_Al): del self.status['MeasPt3_Al']
        self.status['MeasPt3_Ad'] = MeasPt3_Ad
        if math.isnan(MeasPt3_Ad): del self.status['MeasPt3_Ad']
        self.status['MeasPt3_Track'] = MeasPt3_Track
        if math.isnan(MeasPt3_Track): del self.status['MeasPt3_Track']
        self.status['MeasPt3_Slip'] = MeasPt3_Slip
        if math.isnan(MeasPt3_Slip): del self.status['MeasPt3_Slip']
        self.status['MeasPt3_Curvature'] = MeasPt3_Curvature
        if math.isnan(MeasPt3_Curvature): del self.status['MeasPt3_Curvature']

    def decodeMessage24v1(self, payload):
        # Message 24 v1: "Point of interest 3"
        (MeasPt3_Vf, MeasPt3_Vl, MeasPt3_Vd, MeasPt3_Speed2d, MeasPt3_Af, MeasPt3_Al, MeasPt3_Ad, MeasPt3_An, MeasPt3_Ae, MeasPt3_Track, MeasPt3_Slip, MeasPt3_Curvature) = struct.unpack("<dddddddddddd", payload)
        self.status['MeasPt3_Vf'] = MeasPt3_Vf
        if math.isnan(MeasPt3_Vf): del self.status['MeasPt3_Vf']
        self.status['MeasPt3_Vl'] = MeasPt3_Vl
        if math.isnan(MeasPt3_Vl): del self.status['MeasPt3_Vl']
        self.status['MeasPt3_Vd'] = MeasPt3_Vd
        if math.isnan(MeasPt3_Vd): del self.status['MeasPt3_Vd']
        self.status['MeasPt3_Speed2d'] = MeasPt3_Speed2d
        if math.isnan(MeasPt3_Speed2d): del self.status['MeasPt3_Speed2d']
        self.status['MeasPt3_Af'] = MeasPt3_Af
        if math.isnan(MeasPt3_Af): del self.status['MeasPt3_Af']
        self.status['MeasPt3_Al'] = MeasPt3_Al
        if math.isnan(MeasPt3_Al): del self.status['MeasPt3_Al']
        self.status['MeasPt3_Ad'] = MeasPt3_Ad
        if math.isnan(MeasPt3_Ad): del self.status['MeasPt3_Ad']
        self.status['MeasPt3_An'] = MeasPt3_An
        if math.isnan(MeasPt3_An): del self.status['MeasPt3_An']
        self.status['MeasPt3_Ae'] = MeasPt3_Ae
        if math.isnan(MeasPt3_Ae): del self.status['MeasPt3_Ae']
        self.status['MeasPt3_Track'] = MeasPt3_Track
        if math.isnan(MeasPt3_Track): del self.status['MeasPt3_Track']
        self.status['MeasPt3_Slip'] = MeasPt3_Slip
        if math.isnan(MeasPt3_Slip): del self.status['MeasPt3_Slip']
        self.status['MeasPt3_Curvature'] = MeasPt3_Curvature
        if math.isnan(MeasPt3_Curvature): del self.status['MeasPt3_Curvature']

    def decodeMessage25(self, payload):
        # Message 25 v0: "Point of interest 4"
        (Nano, MeasPt4_Vf, MeasPt4_Vl, MeasPt4_Vd, MeasPt4_Speed2d, MeasPt4_Af, MeasPt4_Al, MeasPt4_Ad, MeasPt4_Track, MeasPt4_Slip, MeasPt4_Curvature) = struct.unpack("<qdddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['MeasPt4_Vf'] = MeasPt4_Vf
        if math.isnan(MeasPt4_Vf): del self.status['MeasPt4_Vf']
        self.status['MeasPt4_Vl'] = MeasPt4_Vl
        if math.isnan(MeasPt4_Vl): del self.status['MeasPt4_Vl']
        self.status['MeasPt4_Vd'] = MeasPt4_Vd
        if math.isnan(MeasPt4_Vd): del self.status['MeasPt4_Vd']
        self.status['MeasPt4_Speed2d'] = MeasPt4_Speed2d
        if math.isnan(MeasPt4_Speed2d): del self.status['MeasPt4_Speed2d']
        self.status['MeasPt4_Af'] = MeasPt4_Af
        if math.isnan(MeasPt4_Af): del self.status['MeasPt4_Af']
        self.status['MeasPt4_Al'] = MeasPt4_Al
        if math.isnan(MeasPt4_Al): del self.status['MeasPt4_Al']
        self.status['MeasPt4_Ad'] = MeasPt4_Ad
        if math.isnan(MeasPt4_Ad): del self.status['MeasPt4_Ad']
        self.status['MeasPt4_Track'] = MeasPt4_Track
        if math.isnan(MeasPt4_Track): del self.status['MeasPt4_Track']
        self.status['MeasPt4_Slip'] = MeasPt4_Slip
        if math.isnan(MeasPt4_Slip): del self.status['MeasPt4_Slip']
        self.status['MeasPt4_Curvature'] = MeasPt4_Curvature
        if math.isnan(MeasPt4_Curvature): del self.status['MeasPt4_Curvature']

    def decodeMessage25v1(self, payload):
        # Message 25 v1: "Point of interest 4"
        (MeasPt4_Vf, MeasPt4_Vl, MeasPt4_Vd, MeasPt4_Speed2d, MeasPt4_Af, MeasPt4_Al, MeasPt4_Ad, MeasPt4_An, MeasPt4_Ae, MeasPt4_Track, MeasPt4_Slip, MeasPt4_Curvature) = struct.unpack("<dddddddddddd", payload)
        self.status['MeasPt4_Vf'] = MeasPt4_Vf
        if math.isnan(MeasPt4_Vf): del self.status['MeasPt4_Vf']
        self.status['MeasPt4_Vl'] = MeasPt4_Vl
        if math.isnan(MeasPt4_Vl): del self.status['MeasPt4_Vl']
        self.status['MeasPt4_Vd'] = MeasPt4_Vd
        if math.isnan(MeasPt4_Vd): del self.status['MeasPt4_Vd']
        self.status['MeasPt4_Speed2d'] = MeasPt4_Speed2d
        if math.isnan(MeasPt4_Speed2d): del self.status['MeasPt4_Speed2d']
        self.status['MeasPt4_Af'] = MeasPt4_Af
        if math.isnan(MeasPt4_Af): del self.status['MeasPt4_Af']
        self.status['MeasPt4_Al'] = MeasPt4_Al
        if math.isnan(MeasPt4_Al): del self.status['MeasPt4_Al']
        self.status['MeasPt4_Ad'] = MeasPt4_Ad
        if math.isnan(MeasPt4_Ad): del self.status['MeasPt4_Ad']
        self.status['MeasPt4_An'] = MeasPt4_An
        if math.isnan(MeasPt4_An): del self.status['MeasPt4_An']
        self.status['MeasPt4_Ae'] = MeasPt4_Ae
        if math.isnan(MeasPt4_Ae): del self.status['MeasPt4_Ae']
        self.status['MeasPt4_Track'] = MeasPt4_Track
        if math.isnan(MeasPt4_Track): del self.status['MeasPt4_Track']
        self.status['MeasPt4_Slip'] = MeasPt4_Slip
        if math.isnan(MeasPt4_Slip): del self.status['MeasPt4_Slip']
        self.status['MeasPt4_Curvature'] = MeasPt4_Curvature
        if math.isnan(MeasPt4_Curvature): del self.status['MeasPt4_Curvature']

    def decodeMessage26(self, payload):
        # Message 26 v0: "Point of interest 5"
        (Nano, MeasPt5_Vf, MeasPt5_Vl, MeasPt5_Vd, MeasPt5_Speed2d, MeasPt5_Af, MeasPt5_Al, MeasPt5_Ad, MeasPt5_Track, MeasPt5_Slip, MeasPt5_Curvature) = struct.unpack("<qdddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['MeasPt5_Vf'] = MeasPt5_Vf
        if math.isnan(MeasPt5_Vf): del self.status['MeasPt5_Vf']
        self.status['MeasPt5_Vl'] = MeasPt5_Vl
        if math.isnan(MeasPt5_Vl): del self.status['MeasPt5_Vl']
        self.status['MeasPt5_Vd'] = MeasPt5_Vd
        if math.isnan(MeasPt5_Vd): del self.status['MeasPt5_Vd']
        self.status['MeasPt5_Speed2d'] = MeasPt5_Speed2d
        if math.isnan(MeasPt5_Speed2d): del self.status['MeasPt5_Speed2d']
        self.status['MeasPt5_Af'] = MeasPt5_Af
        if math.isnan(MeasPt5_Af): del self.status['MeasPt5_Af']
        self.status['MeasPt5_Al'] = MeasPt5_Al
        if math.isnan(MeasPt5_Al): del self.status['MeasPt5_Al']
        self.status['MeasPt5_Ad'] = MeasPt5_Ad
        if math.isnan(MeasPt5_Ad): del self.status['MeasPt5_Ad']
        self.status['MeasPt5_Track'] = MeasPt5_Track
        if math.isnan(MeasPt5_Track): del self.status['MeasPt5_Track']
        self.status['MeasPt5_Slip'] = MeasPt5_Slip
        if math.isnan(MeasPt5_Slip): del self.status['MeasPt5_Slip']
        self.status['MeasPt5_Curvature'] = MeasPt5_Curvature
        if math.isnan(MeasPt5_Curvature): del self.status['MeasPt5_Curvature']

    def decodeMessage26v1(self, payload):
        # Message 26 v1: "Point of interest 5"
        (MeasPt5_Vf, MeasPt5_Vl, MeasPt5_Vd, MeasPt5_Speed2d, MeasPt5_Af, MeasPt5_Al, MeasPt5_Ad, MeasPt5_An, MeasPt5_Ae, MeasPt5_Track, MeasPt5_Slip, MeasPt5_Curvature) = struct.unpack("<dddddddddddd", payload)
        self.status['MeasPt5_Vf'] = MeasPt5_Vf
        if math.isnan(MeasPt5_Vf): del self.status['MeasPt5_Vf']
        self.status['MeasPt5_Vl'] = MeasPt5_Vl
        if math.isnan(MeasPt5_Vl): del self.status['MeasPt5_Vl']
        self.status['MeasPt5_Vd'] = MeasPt5_Vd
        if math.isnan(MeasPt5_Vd): del self.status['MeasPt5_Vd']
        self.status['MeasPt5_Speed2d'] = MeasPt5_Speed2d
        if math.isnan(MeasPt5_Speed2d): del self.status['MeasPt5_Speed2d']
        self.status['MeasPt5_Af'] = MeasPt5_Af
        if math.isnan(MeasPt5_Af): del self.status['MeasPt5_Af']
        self.status['MeasPt5_Al'] = MeasPt5_Al
        if math.isnan(MeasPt5_Al): del self.status['MeasPt5_Al']
        self.status['MeasPt5_Ad'] = MeasPt5_Ad
        if math.isnan(MeasPt5_Ad): del self.status['MeasPt5_Ad']
        self.status['MeasPt5_An'] = MeasPt5_An
        if math.isnan(MeasPt5_An): del self.status['MeasPt5_An']
        self.status['MeasPt5_Ae'] = MeasPt5_Ae
        if math.isnan(MeasPt5_Ae): del self.status['MeasPt5_Ae']
        self.status['MeasPt5_Track'] = MeasPt5_Track
        if math.isnan(MeasPt5_Track): del self.status['MeasPt5_Track']
        self.status['MeasPt5_Slip'] = MeasPt5_Slip
        if math.isnan(MeasPt5_Slip): del self.status['MeasPt5_Slip']
        self.status['MeasPt5_Curvature'] = MeasPt5_Curvature
        if math.isnan(MeasPt5_Curvature): del self.status['MeasPt5_Curvature']

    def decodeMessage27(self, payload):
        # Message 27 v0: "Point of interest 6"
        (Nano, MeasPt6_Vf, MeasPt6_Vl, MeasPt6_Vd, MeasPt6_Speed2d, MeasPt6_Af, MeasPt6_Al, MeasPt6_Ad, MeasPt6_Track, MeasPt6_Slip, MeasPt6_Curvature) = struct.unpack("<qdddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['MeasPt6_Vf'] = MeasPt6_Vf
        if math.isnan(MeasPt6_Vf): del self.status['MeasPt6_Vf']
        self.status['MeasPt6_Vl'] = MeasPt6_Vl
        if math.isnan(MeasPt6_Vl): del self.status['MeasPt6_Vl']
        self.status['MeasPt6_Vd'] = MeasPt6_Vd
        if math.isnan(MeasPt6_Vd): del self.status['MeasPt6_Vd']
        self.status['MeasPt6_Speed2d'] = MeasPt6_Speed2d
        if math.isnan(MeasPt6_Speed2d): del self.status['MeasPt6_Speed2d']
        self.status['MeasPt6_Af'] = MeasPt6_Af
        if math.isnan(MeasPt6_Af): del self.status['MeasPt6_Af']
        self.status['MeasPt6_Al'] = MeasPt6_Al
        if math.isnan(MeasPt6_Al): del self.status['MeasPt6_Al']
        self.status['MeasPt6_Ad'] = MeasPt6_Ad
        if math.isnan(MeasPt6_Ad): del self.status['MeasPt6_Ad']
        self.status['MeasPt6_Track'] = MeasPt6_Track
        if math.isnan(MeasPt6_Track): del self.status['MeasPt6_Track']
        self.status['MeasPt6_Slip'] = MeasPt6_Slip
        if math.isnan(MeasPt6_Slip): del self.status['MeasPt6_Slip']
        self.status['MeasPt6_Curvature'] = MeasPt6_Curvature
        if math.isnan(MeasPt6_Curvature): del self.status['MeasPt6_Curvature']

    def decodeMessage27v1(self, payload):
        # Message 27 v1: "Point of interest 6"
        (MeasPt6_Vf, MeasPt6_Vl, MeasPt6_Vd, MeasPt6_Speed2d, MeasPt6_Af, MeasPt6_Al, MeasPt6_Ad, MeasPt6_An, MeasPt6_Ae, MeasPt6_Track, MeasPt6_Slip, MeasPt6_Curvature) = struct.unpack("<dddddddddddd", payload)
        self.status['MeasPt6_Vf'] = MeasPt6_Vf
        if math.isnan(MeasPt6_Vf): del self.status['MeasPt6_Vf']
        self.status['MeasPt6_Vl'] = MeasPt6_Vl
        if math.isnan(MeasPt6_Vl): del self.status['MeasPt6_Vl']
        self.status['MeasPt6_Vd'] = MeasPt6_Vd
        if math.isnan(MeasPt6_Vd): del self.status['MeasPt6_Vd']
        self.status['MeasPt6_Speed2d'] = MeasPt6_Speed2d
        if math.isnan(MeasPt6_Speed2d): del self.status['MeasPt6_Speed2d']
        self.status['MeasPt6_Af'] = MeasPt6_Af
        if math.isnan(MeasPt6_Af): del self.status['MeasPt6_Af']
        self.status['MeasPt6_Al'] = MeasPt6_Al
        if math.isnan(MeasPt6_Al): del self.status['MeasPt6_Al']
        self.status['MeasPt6_Ad'] = MeasPt6_Ad
        if math.isnan(MeasPt6_Ad): del self.status['MeasPt6_Ad']
        self.status['MeasPt6_An'] = MeasPt6_An
        if math.isnan(MeasPt6_An): del self.status['MeasPt6_An']
        self.status['MeasPt6_Ae'] = MeasPt6_Ae
        if math.isnan(MeasPt6_Ae): del self.status['MeasPt6_Ae']
        self.status['MeasPt6_Track'] = MeasPt6_Track
        if math.isnan(MeasPt6_Track): del self.status['MeasPt6_Track']
        self.status['MeasPt6_Slip'] = MeasPt6_Slip
        if math.isnan(MeasPt6_Slip): del self.status['MeasPt6_Slip']
        self.status['MeasPt6_Curvature'] = MeasPt6_Curvature
        if math.isnan(MeasPt6_Curvature): del self.status['MeasPt6_Curvature']

    def decodeMessage28(self, payload):
        # Message 28 v0: "Point of interest 7"
        (Nano, MeasPt7_Vf, MeasPt7_Vl, MeasPt7_Vd, MeasPt7_Speed2d, MeasPt7_Af, MeasPt7_Al, MeasPt7_Ad, MeasPt7_Track, MeasPt7_Slip, MeasPt7_Curvature) = struct.unpack("<qdddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['MeasPt7_Vf'] = MeasPt7_Vf
        if math.isnan(MeasPt7_Vf): del self.status['MeasPt7_Vf']
        self.status['MeasPt7_Vl'] = MeasPt7_Vl
        if math.isnan(MeasPt7_Vl): del self.status['MeasPt7_Vl']
        self.status['MeasPt7_Vd'] = MeasPt7_Vd
        if math.isnan(MeasPt7_Vd): del self.status['MeasPt7_Vd']
        self.status['MeasPt7_Speed2d'] = MeasPt7_Speed2d
        if math.isnan(MeasPt7_Speed2d): del self.status['MeasPt7_Speed2d']
        self.status['MeasPt7_Af'] = MeasPt7_Af
        if math.isnan(MeasPt7_Af): del self.status['MeasPt7_Af']
        self.status['MeasPt7_Al'] = MeasPt7_Al
        if math.isnan(MeasPt7_Al): del self.status['MeasPt7_Al']
        self.status['MeasPt7_Ad'] = MeasPt7_Ad
        if math.isnan(MeasPt7_Ad): del self.status['MeasPt7_Ad']
        self.status['MeasPt7_Track'] = MeasPt7_Track
        if math.isnan(MeasPt7_Track): del self.status['MeasPt7_Track']
        self.status['MeasPt7_Slip'] = MeasPt7_Slip
        if math.isnan(MeasPt7_Slip): del self.status['MeasPt7_Slip']
        self.status['MeasPt7_Curvature'] = MeasPt7_Curvature
        if math.isnan(MeasPt7_Curvature): del self.status['MeasPt7_Curvature']

    def decodeMessage28v1(self, payload):
        # Message 28 v1: "Point of interest 7"
        (MeasPt7_Vf, MeasPt7_Vl, MeasPt7_Vd, MeasPt7_Speed2d, MeasPt7_Af, MeasPt7_Al, MeasPt7_Ad, MeasPt7_An, MeasPt7_Ae, MeasPt7_Track, MeasPt7_Slip, MeasPt7_Curvature) = struct.unpack("<dddddddddddd", payload)
        self.status['MeasPt7_Vf'] = MeasPt7_Vf
        if math.isnan(MeasPt7_Vf): del self.status['MeasPt7_Vf']
        self.status['MeasPt7_Vl'] = MeasPt7_Vl
        if math.isnan(MeasPt7_Vl): del self.status['MeasPt7_Vl']
        self.status['MeasPt7_Vd'] = MeasPt7_Vd
        if math.isnan(MeasPt7_Vd): del self.status['MeasPt7_Vd']
        self.status['MeasPt7_Speed2d'] = MeasPt7_Speed2d
        if math.isnan(MeasPt7_Speed2d): del self.status['MeasPt7_Speed2d']
        self.status['MeasPt7_Af'] = MeasPt7_Af
        if math.isnan(MeasPt7_Af): del self.status['MeasPt7_Af']
        self.status['MeasPt7_Al'] = MeasPt7_Al
        if math.isnan(MeasPt7_Al): del self.status['MeasPt7_Al']
        self.status['MeasPt7_Ad'] = MeasPt7_Ad
        if math.isnan(MeasPt7_Ad): del self.status['MeasPt7_Ad']
        self.status['MeasPt7_An'] = MeasPt7_An
        if math.isnan(MeasPt7_An): del self.status['MeasPt7_An']
        self.status['MeasPt7_Ae'] = MeasPt7_Ae
        if math.isnan(MeasPt7_Ae): del self.status['MeasPt7_Ae']
        self.status['MeasPt7_Track'] = MeasPt7_Track
        if math.isnan(MeasPt7_Track): del self.status['MeasPt7_Track']
        self.status['MeasPt7_Slip'] = MeasPt7_Slip
        if math.isnan(MeasPt7_Slip): del self.status['MeasPt7_Slip']
        self.status['MeasPt7_Curvature'] = MeasPt7_Curvature
        if math.isnan(MeasPt7_Curvature): del self.status['MeasPt7_Curvature']

    def decodeMessage29(self, payload):
        # Message 29 v0: "Point of interest 8"
        (Nano, MeasPt8_Vf, MeasPt8_Vl, MeasPt8_Vd, MeasPt8_Speed2d, MeasPt8_Af, MeasPt8_Al, MeasPt8_Ad, MeasPt8_Track, MeasPt8_Slip, MeasPt8_Curvature) = struct.unpack("<qdddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['MeasPt8_Vf'] = MeasPt8_Vf
        if math.isnan(MeasPt8_Vf): del self.status['MeasPt8_Vf']
        self.status['MeasPt8_Vl'] = MeasPt8_Vl
        if math.isnan(MeasPt8_Vl): del self.status['MeasPt8_Vl']
        self.status['MeasPt8_Vd'] = MeasPt8_Vd
        if math.isnan(MeasPt8_Vd): del self.status['MeasPt8_Vd']
        self.status['MeasPt8_Speed2d'] = MeasPt8_Speed2d
        if math.isnan(MeasPt8_Speed2d): del self.status['MeasPt8_Speed2d']
        self.status['MeasPt8_Af'] = MeasPt8_Af
        if math.isnan(MeasPt8_Af): del self.status['MeasPt8_Af']
        self.status['MeasPt8_Al'] = MeasPt8_Al
        if math.isnan(MeasPt8_Al): del self.status['MeasPt8_Al']
        self.status['MeasPt8_Ad'] = MeasPt8_Ad
        if math.isnan(MeasPt8_Ad): del self.status['MeasPt8_Ad']
        self.status['MeasPt8_Track'] = MeasPt8_Track
        if math.isnan(MeasPt8_Track): del self.status['MeasPt8_Track']
        self.status['MeasPt8_Slip'] = MeasPt8_Slip
        if math.isnan(MeasPt8_Slip): del self.status['MeasPt8_Slip']
        self.status['MeasPt8_Curvature'] = MeasPt8_Curvature
        if math.isnan(MeasPt8_Curvature): del self.status['MeasPt8_Curvature']

    def decodeMessage29v1(self, payload):
        # Message 29 v1: "Point of interest 8"
        (MeasPt8_Vf, MeasPt8_Vl, MeasPt8_Vd, MeasPt8_Speed2d, MeasPt8_Af, MeasPt8_Al, MeasPt8_Ad, MeasPt8_An, MeasPt8_Ae, MeasPt8_Track, MeasPt8_Slip, MeasPt8_Curvature) = struct.unpack("<dddddddddddd", payload)
        self.status['MeasPt8_Vf'] = MeasPt8_Vf
        if math.isnan(MeasPt8_Vf): del self.status['MeasPt8_Vf']
        self.status['MeasPt8_Vl'] = MeasPt8_Vl
        if math.isnan(MeasPt8_Vl): del self.status['MeasPt8_Vl']
        self.status['MeasPt8_Vd'] = MeasPt8_Vd
        if math.isnan(MeasPt8_Vd): del self.status['MeasPt8_Vd']
        self.status['MeasPt8_Speed2d'] = MeasPt8_Speed2d
        if math.isnan(MeasPt8_Speed2d): del self.status['MeasPt8_Speed2d']
        self.status['MeasPt8_Af'] = MeasPt8_Af
        if math.isnan(MeasPt8_Af): del self.status['MeasPt8_Af']
        self.status['MeasPt8_Al'] = MeasPt8_Al
        if math.isnan(MeasPt8_Al): del self.status['MeasPt8_Al']
        self.status['MeasPt8_Ad'] = MeasPt8_Ad
        if math.isnan(MeasPt8_Ad): del self.status['MeasPt8_Ad']
        self.status['MeasPt8_An'] = MeasPt8_An
        if math.isnan(MeasPt8_An): del self.status['MeasPt8_An']
        self.status['MeasPt8_Ae'] = MeasPt8_Ae
        if math.isnan(MeasPt8_Ae): del self.status['MeasPt8_Ae']
        self.status['MeasPt8_Track'] = MeasPt8_Track
        if math.isnan(MeasPt8_Track): del self.status['MeasPt8_Track']
        self.status['MeasPt8_Slip'] = MeasPt8_Slip
        if math.isnan(MeasPt8_Slip): del self.status['MeasPt8_Slip']
        self.status['MeasPt8_Curvature'] = MeasPt8_Curvature
        if math.isnan(MeasPt8_Curvature): del self.status['MeasPt8_Curvature']

    def decodeMessage30(self, payload):
        # Message 30 v0: "Distance and speed"
        (Nano, Dist2d, Dist2dHold, Dist3d, Dist3dHold, Speed2d, Speed3d) = struct.unpack("<qdddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['Dist2d'] = Dist2d
        if math.isnan(Dist2d): del self.status['Dist2d']
        self.status['Dist2dHold'] = Dist2dHold
        if math.isnan(Dist2dHold): del self.status['Dist2dHold']
        self.status['Dist3d'] = Dist3d
        if math.isnan(Dist3d): del self.status['Dist3d']
        self.status['Dist3dHold'] = Dist3dHold
        if math.isnan(Dist3dHold): del self.status['Dist3dHold']
        self.status['Speed2d'] = Speed2d
        if math.isnan(Speed2d): del self.status['Speed2d']
        self.status['Speed3d'] = Speed3d
        if math.isnan(Speed3d): del self.status['Speed3d']

    def decodeMessage30v1(self, payload):
        # Message 30 v1: "Distance and speed"
        (Dist2d, Dist2dHold, Dist3d, Dist3dHold, Speed2d, Speed3d) = struct.unpack("<dddddd", payload)
        self.status['Dist2d'] = Dist2d
        if math.isnan(Dist2d): del self.status['Dist2d']
        self.status['Dist2dHold'] = Dist2dHold
        if math.isnan(Dist2dHold): del self.status['Dist2dHold']
        self.status['Dist3d'] = Dist3d
        if math.isnan(Dist3d): del self.status['Dist3d']
        self.status['Dist3dHold'] = Dist3dHold
        if math.isnan(Dist3dHold): del self.status['Dist3dHold']
        self.status['Speed2d'] = Speed2d
        if math.isnan(Speed2d): del self.status['Speed2d']
        self.status['Speed3d'] = Speed3d
        if math.isnan(Speed3d): del self.status['Speed3d']

    def decodeMessage31(self, payload):
        # Message 31 v0: "Track, slip, curvature, angle gradient"
        (Nano, Curvature, Slip, Track, AngleGradient) = struct.unpack("<qdddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['Curvature'] = Curvature
        if math.isnan(Curvature): del self.status['Curvature']
        self.status['Slip'] = Slip
        if math.isnan(Slip): del self.status['Slip']
        self.status['Track'] = Track
        if math.isnan(Track): del self.status['Track']
        self.status['AngleGradient'] = AngleGradient
        if math.isnan(AngleGradient): del self.status['AngleGradient']

    def decodeMessage31v1(self, payload):
        # Message 31 v1: "Track, slip, curvature, angle gradient"
        (Curvature, Slip, Track, AngleGradient) = struct.unpack("<dddd", payload)
        self.status['Curvature'] = Curvature
        if math.isnan(Curvature): del self.status['Curvature']
        self.status['Slip'] = Slip
        if math.isnan(Slip): del self.status['Slip']
        self.status['Track'] = Track
        if math.isnan(Track): del self.status['Track']
        self.status['AngleGradient'] = AngleGradient
        if math.isnan(AngleGradient): del self.status['AngleGradient']

    def decodeMessage32(self, payload):
        # Message 32 v0: "Heave"
        (Nano, Heave, LPHeave) = struct.unpack("<qdd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['Heave'] = Heave
        if math.isnan(Heave): del self.status['Heave']
        self.status['LPHeave'] = LPHeave
        if math.isnan(LPHeave): del self.status['LPHeave']

    def decodeMessage32v1(self, payload):
        # Message 32 v1: "Heave"
        (Heave,) = struct.unpack("<d", payload)
        self.status['Heave'] = Heave
        if math.isnan(Heave): del self.status['Heave']

    def decodeMessage33(self, payload):
        # Message 33 v0: "Primary GNSS card measurements"
        (Nano, PGCApproxLat, PGCApproxLon, PGCApproxAlt, PGCVn, PGCVe, PGCVd, PGCHeading, PGCPitch, PGCRoll) = struct.unpack("<qddddddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['PGCApproxLat'] = PGCApproxLat
        if math.isnan(PGCApproxLat): del self.status['PGCApproxLat']
        self.status['PGCApproxLon'] = PGCApproxLon
        if math.isnan(PGCApproxLon): del self.status['PGCApproxLon']
        self.status['PGCApproxAlt'] = PGCApproxAlt
        if math.isnan(PGCApproxAlt): del self.status['PGCApproxAlt']
        self.status['PGCVn'] = PGCVn
        if math.isnan(PGCVn): del self.status['PGCVn']
        self.status['PGCVe'] = PGCVe
        if math.isnan(PGCVe): del self.status['PGCVe']
        self.status['PGCVd'] = PGCVd
        if math.isnan(PGCVd): del self.status['PGCVd']
        self.status['PGCHeading'] = PGCHeading
        if math.isnan(PGCHeading): del self.status['PGCHeading']
        self.status['PGCPitch'] = PGCPitch
        if math.isnan(PGCPitch): del self.status['PGCPitch']
        self.status['PGCRoll'] = PGCRoll
        if math.isnan(PGCRoll): del self.status['PGCRoll']

    def decodeMessage33v1(self, payload):
        # Message 33 v1: "Primary GNSS card measurements"
        (PGCApproxLat, PGCApproxLon, PGCApproxAlt, PGCVn, PGCVe, PGCVd) = struct.unpack("<dddddd", payload)
        self.status['PGCApproxLat'] = PGCApproxLat
        if math.isnan(PGCApproxLat): del self.status['PGCApproxLat']
        self.status['PGCApproxLon'] = PGCApproxLon
        if math.isnan(PGCApproxLon): del self.status['PGCApproxLon']
        self.status['PGCApproxAlt'] = PGCApproxAlt
        if math.isnan(PGCApproxAlt): del self.status['PGCApproxAlt']
        self.status['PGCVn'] = PGCVn
        if math.isnan(PGCVn): del self.status['PGCVn']
        self.status['PGCVe'] = PGCVe
        if math.isnan(PGCVe): del self.status['PGCVe']
        self.status['PGCVd'] = PGCVd
        if math.isnan(PGCVd): del self.status['PGCVd']

    def decodeMessage34(self, payload):
        # Message 34 v0: "Secondary GNSS card measurements"
        (Nano, SGCApproxLat, SGCApproxLon, SGCApproxAlt) = struct.unpack("<qddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['SGCApproxLat'] = SGCApproxLat
        if math.isnan(SGCApproxLat): del self.status['SGCApproxLat']
        self.status['SGCApproxLon'] = SGCApproxLon
        if math.isnan(SGCApproxLon): del self.status['SGCApproxLon']
        self.status['SGCApproxAlt'] = SGCApproxAlt
        if math.isnan(SGCApproxAlt): del self.status['SGCApproxAlt']

    def decodeMessage34v1(self, payload):
        # Message 34 v1: "Secondary GNSS card measurements"
        (SGCApproxLat, SGCApproxLon, SGCApproxAlt) = struct.unpack("<ddd", payload)
        self.status['SGCApproxLat'] = SGCApproxLat
        if math.isnan(SGCApproxLat): del self.status['SGCApproxLat']
        self.status['SGCApproxLon'] = SGCApproxLon
        if math.isnan(SGCApproxLon): del self.status['SGCApproxLon']
        self.status['SGCApproxAlt'] = SGCApproxAlt
        if math.isnan(SGCApproxAlt): del self.status['SGCApproxAlt']

    def decodeMessage35(self, payload):
        # Message 35 v0: "Local coordinate PVA"
        (Nano, RefFrameX, RefFrameY, RefFrameVelX, RefFrameVelY, RefFrameYaw, RefFrameTrack) = struct.unpack("<qdddddd", payload)
        self.status['Nano'] = Nano
        if Nano == 0x7FFFFFFFFFFFFFFF: del self.status['Nano']
        self.status['RefFrameX'] = RefFrameX
        if math.isnan(RefFrameX): del self.status['RefFrameX']
        self.status['RefFrameY'] = RefFrameY
        if math.isnan(RefFrameY): del self.status['RefFrameY']
        self.status['RefFrameVelX'] = RefFrameVelX
        if math.isnan(RefFrameVelX): del self.status['RefFrameVelX']
        self.status['RefFrameVelY'] = RefFrameVelY
        if math.isnan(RefFrameVelY): del self.status['RefFrameVelY']
        self.status['RefFrameYaw'] = RefFrameYaw
        if math.isnan(RefFrameYaw): del self.status['RefFrameYaw']
        self.status['RefFrameTrack'] = RefFrameTrack
        if math.isnan(RefFrameTrack): del self.status['RefFrameTrack']

    def decodeMessage35v1(self, payload):
        # Message 35 v1: "Local coordinate PVA"
        (RefFrameX, RefFrameY, RefFrameVelX, RefFrameVelY, RefFrameYaw, RefFrameTrack) = struct.unpack("<dddddd", payload)
        self.status['RefFrameX'] = RefFrameX
        if math.isnan(RefFrameX): del self.status['RefFrameX']
        self.status['RefFrameY'] = RefFrameY
        if math.isnan(RefFrameY): del self.status['RefFrameY']
        self.status['RefFrameVelX'] = RefFrameVelX
        if math.isnan(RefFrameVelX): del self.status['RefFrameVelX']
        self.status['RefFrameVelY'] = RefFrameVelY
        if math.isnan(RefFrameVelY): del self.status['RefFrameVelY']
        self.status['RefFrameYaw'] = RefFrameYaw
        if math.isnan(RefFrameYaw): del self.status['RefFrameYaw']
        self.status['RefFrameTrack'] = RefFrameTrack
        if math.isnan(RefFrameTrack): del self.status['RefFrameTrack']

    def decodeMessage36(self, payload):
        # Message 36 v0: "GAD decoder information"
        (GADChars, GADPkts) = struct.unpack("<II", payload)
        self.status['GADChars'] = GADChars
        if GADChars == 0xFFFFFFFF: del self.status['GADChars']
        self.status['GADPkts'] = GADPkts
        if GADPkts == 0xFFFFFFFF: del self.status['GADPkts']

    def decodeMessage37(self, payload):
        # Message 37 v0: "GAD statuses"
        (GADPosStatus, GADVelStatus, GADAttStatus, GADAngRateStatus) = struct.unpack("<BBBB", payload)
        self.status['GADPosStatus'] = GADPosStatus
        if GADPosStatus == 0xFF: del self.status['GADPosStatus']
        self.status['GADVelStatus'] = GADVelStatus
        if GADVelStatus == 0xFF: del self.status['GADVelStatus']
        self.status['GADAttStatus'] = GADAttStatus
        if GADAttStatus == 0xFF: del self.status['GADAttStatus']
        self.status['GADAngRateStatus'] = GADAngRateStatus
        if GADAngRateStatus == 0xFF: del self.status['GADAngRateStatus']

    def decodeMessage38(self, payload):
        # Message 38 v0: "Position aiding innovations"
        (PosAidingStreamID, InnPosX, InnPosY, InnPosZ) = struct.unpack("<iddd", payload)
        self.status['PosAidingStreamID'] = PosAidingStreamID
        if PosAidingStreamID == 0x7FFFFFFF: del self.status['PosAidingStreamID']
        self.status['InnPosX'] = InnPosX
        if math.isnan(InnPosX): del self.status['InnPosX']
        self.status['InnPosY'] = InnPosY
        if math.isnan(InnPosY): del self.status['InnPosY']
        self.status['InnPosZ'] = InnPosZ
        if math.isnan(InnPosZ): del self.status['InnPosZ']
        self._updateInnovationFilt('InnPosX', InnPosX)
        self._updateInnovationFilt('InnPosY', InnPosY)
        self._updateInnovationFilt('InnPosZ', InnPosZ)

    def decodeMessage39(self, payload):
        # Message 39 v0: "Velocity aiding innovations"
        (VelAidingStreamID, InnVelX, InnVelY, InnVelZ) = struct.unpack("<iddd", payload)
        self.status['VelAidingStreamID'] = VelAidingStreamID
        if VelAidingStreamID == 0x7FFFFFFF: del self.status['VelAidingStreamID']
        self.status['InnVelX'] = InnVelX
        if math.isnan(InnVelX): del self.status['InnVelX']
        self.status['InnVelY'] = InnVelY
        if math.isnan(InnVelY): del self.status['InnVelY']
        self.status['InnVelZ'] = InnVelZ
        if math.isnan(InnVelZ): del self.status['InnVelZ']

    def decodeMessage40(self, payload):
        # Message 40 v0: "Attitude aiding innovations"
        (AttAidingStreamID, InnPitch, InnHeading) = struct.unpack("<idd", payload)
        self.status['AttAidingStreamID'] = AttAidingStreamID
        if AttAidingStreamID == 0x7FFFFFFF: del self.status['AttAidingStreamID']
        self.status['InnPitch'] = InnPitch
        if math.isnan(InnPitch): del self.status['InnPitch']
        self.status['InnHeading'] = InnHeading
        if math.isnan(InnHeading): del self.status['InnHeading']
        self._updateInnovationFilt('InnPitch', InnPitch)
        self._updateInnovationFilt('InnHeading', InnHeading)

    def decodeMessage41(self, payload):
        # Message 41 v0: "GAD packet statistics"
        (GADLatestStreamID, GADNumEarly, GADNumLate, GADNumScheduled, GADLatestStatus) = struct.unpack("<BIIIB", payload)
        self.status['GADLatestStreamID'] = GADLatestStreamID
        if GADLatestStreamID == 0xFF: del self.status['GADLatestStreamID']
        self.status['GADNumEarly'] = GADNumEarly
        if GADNumEarly == 0xFFFFFFFF: del self.status['GADNumEarly']
        self.status['GADNumLate'] = GADNumLate
        if GADNumLate == 0xFFFFFFFF: del self.status['GADNumLate']
        self.status['GADNumScheduled'] = GADNumScheduled
        if GADNumScheduled == 0xFFFFFFFF: del self.status['GADNumScheduled']
        self.status['GADLatestStatus'] = GADLatestStatus
        if GADLatestStatus == 0xFF: del self.status['GADLatestStatus']

    def decodeMessage42(self, payload):
        # Message 42 v0: "Position aiding in the INS frame"
        (PosAidingStreamID, PosUpdateLat, PosUpdateLon, PosUpdateAlt) = struct.unpack("<iddd", payload)
        self.status['PosAidingStreamID'] = PosAidingStreamID
        if PosAidingStreamID == 0x7FFFFFFF: del self.status['PosAidingStreamID']
        self.status['PosUpdateLat'] = PosUpdateLat
        if math.isnan(PosUpdateLat): del self.status['PosUpdateLat']
        self.status['PosUpdateLon'] = PosUpdateLon
        if math.isnan(PosUpdateLon): del self.status['PosUpdateLon']
        self.status['PosUpdateAlt'] = PosUpdateAlt
        if math.isnan(PosUpdateAlt): del self.status['PosUpdateAlt']

    def decodeMessage43(self, payload):
        # Message 43 v0: "Velocity aiding of the INS in navigation frame"
        (VelAidingStreamID, OdomStyleAiding, VelUpdateNorth, VelUpdateEast, VelUpdateDown) = struct.unpack("<iBddd", payload)
        self.status['VelAidingStreamID'] = VelAidingStreamID
        if VelAidingStreamID == 0x7FFFFFFF: del self.status['VelAidingStreamID']
        self.status['OdomStyleAiding'] = OdomStyleAiding
        if OdomStyleAiding == 0xFF: del self.status['OdomStyleAiding']
        self.status['VelUpdateNorth'] = VelUpdateNorth
        if math.isnan(VelUpdateNorth): del self.status['VelUpdateNorth']
        self.status['VelUpdateEast'] = VelUpdateEast
        if math.isnan(VelUpdateEast): del self.status['VelUpdateEast']
        self.status['VelUpdateDown'] = VelUpdateDown
        if math.isnan(VelUpdateDown): del self.status['VelUpdateDown']

    def decodeMessage44(self, payload):
        # Message 44 v0: "Orientation aiding of the INS in dual-antenna frame"
        (AttAidingStreamID, AttPitchDiff, AttHeadingDiff) = struct.unpack("<idd", payload)
        self.status['AttAidingStreamID'] = AttAidingStreamID
        if AttAidingStreamID == 0x7FFFFFFF: del self.status['AttAidingStreamID']
        self.status['AttPitchDiff'] = AttPitchDiff
        if math.isnan(AttPitchDiff): del self.status['AttPitchDiff']
        self.status['AttHeadingDiff'] = AttHeadingDiff
        if math.isnan(AttHeadingDiff): del self.status['AttHeadingDiff']

    def decodeMessage45(self, payload):
        # Message 45 v0: "Covariances of the latest position aiding update in navigation frame"
        (PosAidingStreamID, PosUpdateVarNorth, PosUpdateVarEast, PosUpdateVarDown, PosUpdateCovarEN, PosUpdateCovarND, PosUpdateCovarED) = struct.unpack("<idddddd", payload)
        self.status['PosAidingStreamID'] = PosAidingStreamID
        if PosAidingStreamID == 0x7FFFFFFF: del self.status['PosAidingStreamID']
        self.status['PosUpdateVarNorth'] = PosUpdateVarNorth
        if math.isnan(PosUpdateVarNorth): del self.status['PosUpdateVarNorth']
        self.status['PosUpdateVarEast'] = PosUpdateVarEast
        if math.isnan(PosUpdateVarEast): del self.status['PosUpdateVarEast']
        self.status['PosUpdateVarDown'] = PosUpdateVarDown
        if math.isnan(PosUpdateVarDown): del self.status['PosUpdateVarDown']
        self.status['PosUpdateCovarEN'] = PosUpdateCovarEN
        if math.isnan(PosUpdateCovarEN): del self.status['PosUpdateCovarEN']
        self.status['PosUpdateCovarND'] = PosUpdateCovarND
        if math.isnan(PosUpdateCovarND): del self.status['PosUpdateCovarND']
        self.status['PosUpdateCovarED'] = PosUpdateCovarED
        if math.isnan(PosUpdateCovarED): del self.status['PosUpdateCovarED']

    def decodeMessage46(self, payload):
        # Message 46 v0: "Covariances of the latest velocity aiding update in navigation frame"
        (VelAidingStreamID, OdomStyleAiding, VelUpdateVarNorth, VelUpdateVarEast, VelUpdateVarDown, VelUpdateCovarEN, VelUpdateCovarND, VelUpdateCovarED) = struct.unpack("<iBdddddd", payload)
        self.status['VelAidingStreamID'] = VelAidingStreamID
        if VelAidingStreamID == 0x7FFFFFFF: del self.status['VelAidingStreamID']
        self.status['OdomStyleAiding'] = OdomStyleAiding
        if OdomStyleAiding == 0xFF: del self.status['OdomStyleAiding']
        self.status['VelUpdateVarNorth'] = VelUpdateVarNorth
        if math.isnan(VelUpdateVarNorth): del self.status['VelUpdateVarNorth']
        self.status['VelUpdateVarEast'] = VelUpdateVarEast
        if math.isnan(VelUpdateVarEast): del self.status['VelUpdateVarEast']
        self.status['VelUpdateVarDown'] = VelUpdateVarDown
        if math.isnan(VelUpdateVarDown): del self.status['VelUpdateVarDown']
        self.status['VelUpdateCovarEN'] = VelUpdateCovarEN
        if math.isnan(VelUpdateCovarEN): del self.status['VelUpdateCovarEN']
        self.status['VelUpdateCovarND'] = VelUpdateCovarND
        if math.isnan(VelUpdateCovarND): del self.status['VelUpdateCovarND']
        self.status['VelUpdateCovarED'] = VelUpdateCovarED
        if math.isnan(VelUpdateCovarED): del self.status['VelUpdateCovarED']

    def decodeMessage47(self, payload):
        # Message 47 v0: "Raw IMU measurements"
        (RawAx, RawAy, RawAz, RawWx, RawWy, RawWz) = struct.unpack("<dddddd", payload)
        self.status['RawAx'] = RawAx
        if math.isnan(RawAx): del self.status['RawAx']
        self.status['RawAy'] = RawAy
        if math.isnan(RawAy): del self.status['RawAy']
        self.status['RawAz'] = RawAz
        if math.isnan(RawAz): del self.status['RawAz']
        self.status['RawWx'] = RawWx
        if math.isnan(RawWx): del self.status['RawWx']
        self.status['RawWy'] = RawWy
        if math.isnan(RawWy): del self.status['RawWy']
        self.status['RawWz'] = RawWz
        if math.isnan(RawWz): del self.status['RawWz']

    def decodeMessage48(self, payload):
        # Message 48 v0: "OXTS vehicle frame linear jerks and angular accelerations"
        (Jx, Jy, Jz, Yx, Yy, Yz) = struct.unpack("<dddddd", payload)
        self.status['Jx'] = Jx
        if math.isnan(Jx): del self.status['Jx']
        self.status['Jy'] = Jy
        if math.isnan(Jy): del self.status['Jy']
        self.status['Jz'] = Jz
        if math.isnan(Jz): del self.status['Jz']
        self.status['Yx'] = Yx
        if math.isnan(Yx): del self.status['Yx']
        self.status['Yy'] = Yy
        if math.isnan(Yy): del self.status['Yy']
        self.status['Yz'] = Yz
        if math.isnan(Yz): del self.status['Yz']

    def decodeMessage49(self, payload):
        # Message 49 v0: "OXTS navigation frame linear jerks and angular accelerations"
        (Jn, Je, Jd, Yn, Ye, Yd) = struct.unpack("<dddddd", payload)
        self.status['Jn'] = Jn
        if math.isnan(Jn): del self.status['Jn']
        self.status['Je'] = Je
        if math.isnan(Je): del self.status['Je']
        self.status['Jd'] = Jd
        if math.isnan(Jd): del self.status['Jd']
        self.status['Yn'] = Yn
        if math.isnan(Yn): del self.status['Yn']
        self.status['Ye'] = Ye
        if math.isnan(Ye): del self.status['Ye']
        self.status['Yd'] = Yd
        if math.isnan(Yd): del self.status['Yd']

    def decodeMessage50(self, payload):
        # Message 50 v0: "OXTS intermediate frame linear jerks and angular accelerations"
        (Jf, Jl, Jd, Yf, Yl, Yd) = struct.unpack("<dddddd", payload)
        self.status['Jf'] = Jf
        if math.isnan(Jf): del self.status['Jf']
        self.status['Jl'] = Jl
        if math.isnan(Jl): del self.status['Jl']
        self.status['Jd'] = Jd
        if math.isnan(Jd): del self.status['Jd']
        self.status['Yf'] = Yf
        if math.isnan(Yf): del self.status['Yf']
        self.status['Yl'] = Yl
        if math.isnan(Yl): del self.status['Yl']
        self.status['Yd'] = Yd
        if math.isnan(Yd): del self.status['Yd']

    def decodeMessage51(self, payload):
        # Message 51 v0: "ISO-8855 Earth fixed linear jerks and angular accelerations"
        (IsoJnX, IsoJnY, IsoJnZ, IsoYnX, IsoYnY, IsoYnZ) = struct.unpack("<dddddd", payload)
        self.status['IsoJnX'] = IsoJnX
        if math.isnan(IsoJnX): del self.status['IsoJnX']
        self.status['IsoJnY'] = IsoJnY
        if math.isnan(IsoJnY): del self.status['IsoJnY']
        self.status['IsoJnZ'] = IsoJnZ
        if math.isnan(IsoJnZ): del self.status['IsoJnZ']
        self.status['IsoYnX'] = IsoYnX
        if math.isnan(IsoYnX): del self.status['IsoYnX']
        self.status['IsoYnY'] = IsoYnY
        if math.isnan(IsoYnY): del self.status['IsoYnY']
        self.status['IsoYnZ'] = IsoYnZ
        if math.isnan(IsoYnZ): del self.status['IsoYnZ']

    def decodeMessage52(self, payload):
        # Message 52 v0: "ISO-8855 intermediate frame linear jerks and angular accelerations"
        (IsoJhX, IsoJhY, IsoJhZ, IsoYhX, IsoYhY, IsoYhZ) = struct.unpack("<dddddd", payload)
        self.status['IsoJhX'] = IsoJhX
        if math.isnan(IsoJhX): del self.status['IsoJhX']
        self.status['IsoJhY'] = IsoJhY
        if math.isnan(IsoJhY): del self.status['IsoJhY']
        self.status['IsoJhZ'] = IsoJhZ
        if math.isnan(IsoJhZ): del self.status['IsoJhZ']
        self.status['IsoYhX'] = IsoYhX
        if math.isnan(IsoYhX): del self.status['IsoYhX']
        self.status['IsoYhY'] = IsoYhY
        if math.isnan(IsoYhY): del self.status['IsoYhY']
        self.status['IsoYhZ'] = IsoYhZ
        if math.isnan(IsoYhZ): del self.status['IsoYhZ']

    def decodeMessage53(self, payload):
        # Message 53 v0: "ISO-8855 vehicle frame linear jerks and angular accelerations"
        (IsoJoX, IsoJoY, IsoJoZ, IsoYoX, IsoYoY, IsoYoZ) = struct.unpack("<dddddd", payload)
        self.status['IsoJoX'] = IsoJoX
        if math.isnan(IsoJoX): del self.status['IsoJoX']
        self.status['IsoJoY'] = IsoJoY
        if math.isnan(IsoJoY): del self.status['IsoJoY']
        self.status['IsoJoZ'] = IsoJoZ
        if math.isnan(IsoJoZ): del self.status['IsoJoZ']
        self.status['IsoYoX'] = IsoYoX
        if math.isnan(IsoYoX): del self.status['IsoYoX']
        self.status['IsoYoY'] = IsoYoY
        if math.isnan(IsoYoY): del self.status['IsoYoY']
        self.status['IsoYoZ'] = IsoYoZ
        if math.isnan(IsoYoZ): del self.status['IsoYoZ']


    ####################################################################
    # Custom message (not one of OXTS's own 93) - confirmed the firmware
    # does support user-defined messages (64512-65535) via a one-off test
    # (UpTime alone, message 64512, since retired). This one pulls in
    # every status-page field that exists in oxts.dbs but wasn't packaged
    # into any of OXTS's predefined messages - see ncom-to-ucom-mapping.md.
    def decodeMessage64513(self, payload):
        # Message 64513 v0: custom - not one of OXTS's own 93. Fields that
        # exist in oxts.dbs but weren't packaged into any predefined message -
        # see mobile.dbu's MessageDescription for 64513.
        (UpTime, GnssPosReject, GnssVelReject, GnssAttReject, BaseLineLength, HeadingMisAlign, GnssPosNumSatsUsed, GnssVelNumSatsUsed, TimeMismatch, GnssInt1_Chars, GnssInt1_Pkts, GnssInt1_CharsSkipped, GnssInt1_OldPkts, GnssInt2_Chars, GnssInt2_Pkts, GnssInt2_CharsSkipped, GnssInt2_OldPkts, ImuChars, ImuPkts, WxBias, WyBias, WzBias, WxSf, WySf, WzSf, AxBias, AyBias, AzBias, DiskSpace, FileSize) = struct.unpack("<iiiiddBBiIIIIIIIIIIdddddddddii", payload)
        self.status['UpTime'] = UpTime
        if UpTime == 0x7FFFFFFF: del self.status['UpTime']
        self.status['GnssPosReject'] = GnssPosReject
        if GnssPosReject == 0x7FFFFFFF: del self.status['GnssPosReject']
        self.status['GnssVelReject'] = GnssVelReject
        if GnssVelReject == 0x7FFFFFFF: del self.status['GnssVelReject']
        self.status['GnssAttReject'] = GnssAttReject
        if GnssAttReject == 0x7FFFFFFF: del self.status['GnssAttReject']
        self.status['BaseLineLength'] = BaseLineLength
        if math.isnan(BaseLineLength): del self.status['BaseLineLength']
        self.status['HeadingMisAlign'] = HeadingMisAlign
        if math.isnan(HeadingMisAlign): del self.status['HeadingMisAlign']
        self.status['GnssPosNumSatsUsed'] = GnssPosNumSatsUsed
        if GnssPosNumSatsUsed == 0xFF: del self.status['GnssPosNumSatsUsed']
        self.status['GnssVelNumSatsUsed'] = GnssVelNumSatsUsed
        if GnssVelNumSatsUsed == 0xFF: del self.status['GnssVelNumSatsUsed']
        self.status['TimeMismatch'] = TimeMismatch
        if TimeMismatch == 0x7FFFFFFF: del self.status['TimeMismatch']
        self.status['GnssInt1_Chars'] = GnssInt1_Chars
        if GnssInt1_Chars == 0xFFFFFFFF: del self.status['GnssInt1_Chars']
        self.status['GnssInt1_Pkts'] = GnssInt1_Pkts
        if GnssInt1_Pkts == 0xFFFFFFFF: del self.status['GnssInt1_Pkts']
        self.status['GnssInt1_CharsSkipped'] = GnssInt1_CharsSkipped
        if GnssInt1_CharsSkipped == 0xFFFFFFFF: del self.status['GnssInt1_CharsSkipped']
        self.status['GnssInt1_OldPkts'] = GnssInt1_OldPkts
        if GnssInt1_OldPkts == 0xFFFFFFFF: del self.status['GnssInt1_OldPkts']
        self.status['GnssInt2_Chars'] = GnssInt2_Chars
        if GnssInt2_Chars == 0xFFFFFFFF: del self.status['GnssInt2_Chars']
        self.status['GnssInt2_Pkts'] = GnssInt2_Pkts
        if GnssInt2_Pkts == 0xFFFFFFFF: del self.status['GnssInt2_Pkts']
        self.status['GnssInt2_CharsSkipped'] = GnssInt2_CharsSkipped
        if GnssInt2_CharsSkipped == 0xFFFFFFFF: del self.status['GnssInt2_CharsSkipped']
        self.status['GnssInt2_OldPkts'] = GnssInt2_OldPkts
        if GnssInt2_OldPkts == 0xFFFFFFFF: del self.status['GnssInt2_OldPkts']
        self.status['ImuChars'] = ImuChars
        if ImuChars == 0xFFFFFFFF: del self.status['ImuChars']
        self.status['ImuPkts'] = ImuPkts
        if ImuPkts == 0xFFFFFFFF: del self.status['ImuPkts']
        self.status['WxBias'] = WxBias
        if math.isnan(WxBias): del self.status['WxBias']
        self.status['WyBias'] = WyBias
        if math.isnan(WyBias): del self.status['WyBias']
        self.status['WzBias'] = WzBias
        if math.isnan(WzBias): del self.status['WzBias']
        self.status['WxSf'] = WxSf
        if math.isnan(WxSf): del self.status['WxSf']
        self.status['WySf'] = WySf
        if math.isnan(WySf): del self.status['WySf']
        self.status['WzSf'] = WzSf
        if math.isnan(WzSf): del self.status['WzSf']
        self.status['AxBias'] = AxBias
        if math.isnan(AxBias): del self.status['AxBias']
        self.status['AyBias'] = AyBias
        if math.isnan(AyBias): del self.status['AyBias']
        self.status['AzBias'] = AzBias
        if math.isnan(AzBias): del self.status['AzBias']
        self.status['DiskSpace'] = DiskSpace
        if DiskSpace == 0x7FFFFFFF: del self.status['DiskSpace']
        self.status['FileSize'] = FileSize
        if FileSize == 0x7FFFFFFF: del self.status['FileSize']
