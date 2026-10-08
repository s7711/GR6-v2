"""Listing the xNAV650's FTP root.

The xNAV's FTP server answers NLST with full `ls -l`-style lines (plus a
couple of MLSD-style "Type=cdir;..." entries for `/` and `..`), not bare
names - so `name.startswith("mobile.")` on NLST output matches nothing.
LIST (`ftp.dir`) gives the same `ls -l` lines without the MLSD noise;
parse the name and size from those. Every file's date is "Oct 9 1999"
(no real clock behind it), so dates aren't parsed. Found 2026-10-08.
"""

import re

# -rw-r--r-- 1 owner group 530167296 Oct 9 1999 260928_235913.rd
_LS_LINE = re.compile(r"^-\S*\s+\d+\s+\S+\s+\S+\s+(\d+)\s+\S+\s+\d+\s+\S+\s+(\S+)$")

# Raw log files: named by their start time, YYMMDD_HHMMSS.rd (UTC). The
# xNAV has no clock battery, so each log is "mobile.rd" until GNSS time
# arrives (~1 min after power-on/reset), then renamed - hence the gap
# between a reset and the next file's name.
RD_NAME = re.compile(r"^\d{6}_\d{6}\.rd$")


def parse_listing(lines):
    """Regular files from LIST output, as {name: size_bytes}."""
    files = {}
    for line in lines:
        m = _LS_LINE.match(line.strip())
        if m:
            files[m.group(2)] = int(m.group(1))
    return files


def list_files(ftp):
    lines = []
    ftp.dir(lines.append)
    return parse_listing(lines)


# The u-blox receivers report their firmware (UBX MON-VER) when the xNAV
# starts them, so it's in the first few kB of every .rd file (seen at
# ~5kB). Reading this much is plenty and avoids pulling whole files.
RD_HEAD_BYTES = 64 * 1024
_FWVER = re.compile(rb"FWVER=([A-Z]+ [0-9]+\.[0-9]+)")


def firmware_versions(data):
    """Distinct u-blox firmware versions mentioned in raw log bytes, e.g.
    ["HPG 1.13"]. One per receiver type, not per receiver: both
    receivers on the same firmware give a single entry."""
    return sorted({m.group(1).decode() for m in _FWVER.finditer(data)})


def read_head(ftp, name, nbytes=RD_HEAD_BYTES):
    """The first nbytes of a file (less if it's shorter). Abandons the
    transfer part-way, so the caller should discard this ftp session."""
    ftp.voidcmd("TYPE I")
    conn = ftp.transfercmd(f"RETR {name}")
    data = bytearray()
    try:
        while len(data) < nbytes:
            chunk = conn.recv(nbytes - len(data))
            if not chunk:
                break
            data.extend(chunk)
    finally:
        conn.close()
    return bytes(data)


# The config files: every "mobile.*" except mobile.rd (the raw log being
# recorded, renamed once GNSS time arrives - see RD_NAME), plus
# comment.txt, the free-text setup log NAVconfig keeps with them.
# Anything else (ptpd.conf, oxts.dbs/.dbu, info.txt) is left alone.
CONFIG_EXCLUDE = {"mobile.rd"}
CONFIG_EXTRA = {"comment.txt"}


def is_config_file(name):
    return (name.startswith("mobile.") and name not in CONFIG_EXCLUDE) or name in CONFIG_EXTRA


def fetch_config(ftp):
    """Every config file on the xNAV now, as {name: bytes}."""
    files = {}
    for name in sorted(n for n in list_files(ftp) if is_config_file(n)):
        buf = bytearray()
        ftp.retrbinary(f"RETR {name}", buf.extend)
        files[name] = bytes(buf)
    return files
