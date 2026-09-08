"""Forward body-frame velocity, computed from the nav feed's own
Vn/Ve/Heading — display/comparison only (see wheelspeed-prd.md's
"Page" section), never sent back to the xNAV650 as aiding. Deliberately
computed here rather than read from a UCOM/NCOM field directly, since
NCOM has no equivalent field and this must work the same regardless of
which protocol oxts-nav is configured for.
"""

import math


def forward_velocity(vn, ve, heading_deg):
    """Vn/Ve in m/s (NED), heading_deg clockwise from North (OxTS
    convention, same as used throughout aruco/navigate)."""
    heading_rad = math.radians(heading_deg)
    return vn * math.cos(heading_rad) + ve * math.sin(heading_rad)


def wheel_forward_velocity(vn, ve, heading_deg, wz_deg_s, wheel_base_m, side):
    """Predicted forward speed at one wheel if the INS solution and
    wheel_base_m are both correct and there's no slip - added 2026-09-08
    for wheelspeed's own scale-factor estimator (see scale_factor.py).
    Same rigid-body correction navigate/geometry.py's differential_drive()
    applies in the other direction (turn -> per-wheel speed): here it's
    the INS's own centreline forward_velocity() plus a wz*wheel_base_m/2
    correction, instead of a commanded speed + turn. Only the horizontal
    plane is used (no pitch/roll) - deliberately, see Ben's call
    2026-09-08 that a full 3D lever-arm rotation isn't worth it here.

    wz_deg_s is nav's own Wz (yaw rate, deg/s - see ncomrx.py). Positive
    is assumed to mean turning right/clockwise, matching turn_command's
    own convention in navigate/geometry.py (so the outside/left wheel
    speeds up, inside/right wheel slows down) - not yet independently
    verified against a real turn's logged data, worth a first sanity
    check against which way the robot actually turned."""
    centre_mps = forward_velocity(vn, ve, heading_deg)
    wz_rad_s = math.radians(wz_deg_s)
    half_track_mps = wz_rad_s * wheel_base_m / 2
    return centre_mps + half_track_mps if side == "left" else centre_mps - half_track_mps
