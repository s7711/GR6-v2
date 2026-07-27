"""Timing helper for wheelspeed GAD updates — see wheelspeed-prd.md's
"Update rate / timing" for why the midpoint of the averaging interval
is used rather than the arrival time of the newer reading.
"""


def midpoint_time(prev_timestamp, current_timestamp):
    """drive's filtered wheel velocity is an average over
    [prev_timestamp, current_timestamp] (both real FV-line arrival
    times, see drive/serial_link.py's FV_timestamp) — reporting it at
    current_timestamp would systematically bias the GAD aiding time by
    half the update interval, so the midpoint is used instead."""
    return (prev_timestamp + current_timestamp) / 2.0
