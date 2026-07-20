"""Lat/lon/alt <-> local NED (north, east, down) tangent-plane conversion.

Ported near-unchanged from GR6-v1's ncomrx.py (LLA2NED/NED2LLA). Generic —
not specific to ncomrx/xNAV650 decoding — so it lives in shared/, for any
service that needs to relate a geodetic position to a local tangent plane
(currently: aruco, converting a marker's stored lat/lon into a local frame
around the vehicle's current position, and vice versa for a newly surveyed
marker).

Flat-earth local-tangent-plane approximation, not exact ECEF/geodetic
maths — fine at the tens-of-metres scale this project operates at (error
grows with distance from RefFrame and isn't validated near the poles).
"""

import math

EARTH_EQUAT_RADIUS = 6378137.0  # m, WGS84 semi-major axis
EARTH_ECCENTRICITY = 0.0818191908426  # WGS84 first eccentricity


def _radii(ref_lat_rad, ref_alt):
    tmp = 1.0 - (EARTH_ECCENTRICITY * math.sin(ref_lat_rad)) ** 2
    sqt = math.sqrt(tmp)
    rho_e = EARTH_EQUAT_RADIUS * (1.0 - EARTH_ECCENTRICITY**2) / (sqt * tmp)
    rho_n = EARTH_EQUAT_RADIUS / sqt
    return rho_e + ref_alt, rho_n + ref_alt  # rad_lat, rad_lon


def lla_to_ned(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """Local NED position of (lat, lon, alt) relative to the tangent plane
    at (ref_lat, ref_lon, ref_alt). All angles in degrees, alt/NED in
    metres. Returns (north, east, down)."""
    ref_lat_rad = math.radians(ref_lat)
    rad_lat, rad_lon = _radii(ref_lat_rad, ref_alt)
    north = math.radians(lat - ref_lat) * rad_lat
    east = math.radians(lon - ref_lon) * rad_lon * math.cos(ref_lat_rad)
    down = ref_alt - alt
    return north, east, down


def ned_to_lla(north, east, down, ref_lat, ref_lon, ref_alt):
    """Inverse of lla_to_ned: (lat, lon, alt) in degrees/metres from a
    local NED offset relative to the tangent plane at (ref_lat, ref_lon,
    ref_alt)."""
    ref_lat_rad = math.radians(ref_lat)
    rad_lat, rad_lon = _radii(ref_lat_rad, ref_alt)
    lat = ref_lat + math.degrees(north / rad_lat)
    lon = ref_lon + math.degrees(east / rad_lon) / math.cos(ref_lat_rad)
    alt = ref_alt - down
    return lat, lon, alt
