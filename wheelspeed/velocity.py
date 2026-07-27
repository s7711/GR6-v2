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
