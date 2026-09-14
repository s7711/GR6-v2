"""Whether a scale-factor sample's GNSS backing was good enough to trust
for scale_factor_map.py - added 2026-09-14, see wheelspeed-prd.md
("Scale-factor map"/"GNSS-quality gate").

Once the map's per-cell scale factor is actually fed back into GAD, the
wheel-vs-INS comparison it's built from stops being independent: the
INS solution it's checked against would itself be partly shaped by the
very wheelspeed being checked. Position accuracy isn't a safe gate for
this on its own either - some spots worth mapping (e.g. under a tree)
have poor GNSS position from multipath while still having good GNSS
*velocity* (Doppler-derived, far less affected by multipath than a
pseudorange position fix). So this gates on velocity innovations/reject
count instead of horizontal_accuracy_m - during a genuinely degraded-
GNSS spell (not just a shadowed sky), velocity is unreliable too and a
cell simply doesn't get a sample that tick, rather than risk learning
from a self-reinforcing loop.

status is oxts-nav's nav_feed "status" dict (ncomrx.py's decoder.status,
published live regardless of decoded_log_fields - that config only
gates what gets written to the historical log, not what's in the live
feed).

Uses the *Filt* fields, not the raw instantaneous InnVelX/InnVelY,
despite the raw ones looking a lot cleaner in isolation (median ~0) -
deliberately: a bad GNSS velocity update disturbs the filter's state
and covariance for a while after the event itself, not just for the
one epoch it occurred on (Ben, 2026-09-14), so checking only the
current instant would happily accept a sample taken moments after a
real upset had already passed. The Filt fields' peak-hold-with-slow-
decay behaviour (see ncomrx.py's _updateInnovation) is exactly what
makes them track that lingering effect rather than just the instant.

max_innovation needs to be picked empirically against this, not
assumed to be "1 sigma" - checked live against the 2026-09-14 mission:
InnVelXFilt/InnVelYFilt sit around 1.5-2 (median) even during wholly
ordinary driving with no GNSS problem at all, with a 3.0-3.2 tail
(p95) - see wheelspeed-prd.md's "Scale-factor map" for the numbers this
was tuned against.

status is oxts-nav's nav_feed "status" dict (ncomrx.py's decoder.status,
published live regardless of decoded_log_fields - that config only
gates what gets written to the historical log, not what's in the live
feed)."""


def gnss_velocity_trustworthy(status: dict, max_innovation: float) -> bool:
    if status.get("GnssVelReject", 0):
        return False
    for key in ("InnVelXFilt", "InnVelYFilt"):
        value = status.get(key)
        if value is None:
            return False  # not (yet) available - don't guess
        if abs(value) > max_innovation:
            return False
    return True
