"""Generate the capture path and drive it.

This is the robot-control half of the bore-sight job, and it belongs
here rather than with the operator: Ben plants the markers and puts the
robot outside, this drives it round the rectangle and produces the data.

The pattern is the one the simulator settled on (see boresight-prd.md):

  * "fan" legs, driven straight at the panel from a spread of bearings —
    azimuth spread is what separates hpr_cb heading from the markers'
    own headings.
  * "past" legs, swinging off to one side so the markers sweep out
    towards the image edge — camera roll displaces an image point by
    rho.phi, so edge content is where roll is actually measured. Without
    these, roll comes out at 0.38 deg instead of 0.05.

Facing comes from placement.marker_targets(), never from layouts.py's
hard-coded study constant: the two pick opposite ends of the same axis
on this box, and fanning round the wrong end would photograph the backs
of the markers.
"""

import math
import sys
import time
from pathlib import Path

import numpy as np
import requests

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from shared.geodesy import lla_to_ned, ned_to_lla  # noqa: E402

import placement  # noqa: E402

SPEED_MPS = 0.2
CLEARANCE_M = 0.5

# Sector and radii, as studied. r_near is kept off the panel so the robot
# never drives into the rail; r_far stays inside ArUco's ~4m limit.
HALF_SECTOR_DEG = 55.0
FAN_COUNT = 7
R_FAR_M = 3.5
R_NEAR_M = 1.3

LEG_PATH_PREFIX = "Boresight leg"
JOB_NAME = "Boresight capture"
# Forcing a turn the long way round needs the arc split into pieces each
# comfortably under 180 degrees, or the controller's own shortest-way
# logic just takes the short side of each piece instead. Three is plenty
# (a 359-degree forced turn splits into 120s).
FORCED_TURN_PIECES = 3


def _ne_to_lla(ne, lat0, lon0):
    lat, lon, _ = ned_to_lla(float(ne[0]), float(ne[1]), 0.0, lat0, lon0, 0.0)
    return lat, lon


def ends_inward_order(n):
    """Bearing indices ordered from both ends of the sector inwards:
    0, n-1, 1, n-2, ... — so consecutive legs come from opposite sides.

    This is what makes the turns alternate. Walking the sector
    monotonically turns the same way at every transition, and the
    xNAV650 does better when turns aren't all one direction (Ben,
    2026-09-16 — it's why the warm-up paths are figure-8s).
    """
    out, lo, hi = [], 0, n - 1
    while lo <= hi:
        out.append(lo)
        if hi != lo:
            out.append(hi)
        lo, hi = lo + 1, hi - 1
    return out


def build_chain(face_bearing_deg):
    """The capture pattern as a chain of straight legs.

    Each leg ends where the next begins, so the robot can run a leg, turn
    on the spot to the next leg's bearing, and run again — which is how
    this is executed (see build_job). That structure is what makes the
    geometry possible at all.

    The history matters, because the obvious approaches all fail:
    alternating between a far ring and a near ring means every vertex
    reverses radial direction, so the turn is about 180 minus the change
    in bearing. With only a 110 degree sector the best possible turn is
    70 degrees, and a constraint search over the whole sequence space
    found NO ordering that covered all seven bearings while staying under
    even 110 degrees. Rounding the corners didn't rescue it either: a
    0.5m fillet on a 154 degree turn needs 2.2m of tangent on each leg,
    and the legs are shorter than that, so the arcs collapsed and ate 45%
    of every leg (the path fell from 40m to 12m).

    A figure-8 is smooth and perfectly balanced, but scored 12-39 marker
    observations against the weave's 780: a forward-facing camera on a
    circle spends most of its time looking away from the middle.

    Turning on the spot between straight legs removes the radius
    constraint entirely (Ben, 2026-09-16), so the geometry the simulator
    actually wants becomes driveable as-is.
    """
    bearings = np.linspace(face_bearing_deg - HALF_SECTOR_DEG,
                           face_bearing_deg + HALF_SECTOR_DEG, FAN_COUNT)
    order = ends_inward_order(FAN_COUNT)
    sequence = order + list(reversed([FAN_COUNT - 1 - i for i in order]))
    points = []
    for k, i in enumerate(sequence):
        radius = R_FAR_M if k % 2 == 0 else R_NEAR_M
        a = math.radians(bearings[i])
        points.append(np.array([radius * math.cos(a), radius * math.sin(a)]))
    return [(points[i], points[i + 1]) for i in range(len(points) - 1)]


def turn_stats(points):
    """Signed heading changes along a path: {net_deg, left, right}.

    `net_deg` near zero and left/right roughly balanced is what keeps the
    xNAV650 happy — a path that only ever turns one way is the thing to
    avoid.
    """
    if len(points) < 3:
        return {"net_deg": 0.0, "left": 0, "right": 0}
    lat0, lon0 = points[0]["lat"], points[0]["lon"]
    local = [lla_to_ned(p["lat"], p["lon"], 0.0, lat0, lon0, 0.0)[:2] for p in points]
    headings = [math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
                for a, b in zip(local, local[1:])]
    net, left, right = 0.0, 0, 0
    for h0, h1 in zip(headings, headings[1:]):
        delta = (h1 - h0 + 180.0) % 360.0 - 180.0
        net += delta
        if delta > 5.0:
            right += 1
        elif delta < -5.0:
            left += 1
    return {"net_deg": net, "left": left, "right": right}


def turn_steps(from_heading_deg, to_heading_deg, want_sign, tolerance_deg=8.0):
    """turn_to_heading steps taking the robot from one heading to another,
    turning in the direction `want_sign` (+1 right, -1 left, 0 don't care).

    navigate's turn controller is explicitly "shortest-way proportional
    heading control" (turn_control.py), so asking for a heading does NOT
    let you choose which way it goes round. If the shortest way is the
    wrong direction, the only way to force the other is to break the turn
    into intermediate headings, each of which is itself a shortest-way
    turn in the direction wanted. (Ben, 2026-09-16 — without this, the
    intended left/right alternation is silently whatever the controller
    felt like, which for the xNAV650 is the thing being avoided.)
    """
    delta = (to_heading_deg - from_heading_deg + 180.0) % 360.0 - 180.0
    if want_sign == 0 or delta == 0 or (delta > 0) == (want_sign > 0):
        return [{"type": "turn_to_heading", "heading_deg": round(to_heading_deg, 1),
                 "tolerance_deg": tolerance_deg}]
    # Go the long way: same total rotation, opposite sign.
    long_way = delta - 360.0 * (1 if delta > 0 else -1)
    steps = []
    for piece in range(1, FORCED_TURN_PIECES + 1):
        heading = from_heading_deg + long_way * piece / FORCED_TURN_PIECES
        steps.append({"type": "turn_to_heading",
                      "heading_deg": round((heading + 180.0) % 360.0 - 180.0, 1),
                      "tolerance_deg": tolerance_deg})
    return steps


def build_job(box_points, face_bearing_deg, panel_lat, panel_lon, inside_margin_m=0.4,
              turn_tolerance_deg=8.0):
    """Turn the chain into (leg_paths, job_steps).

    leg_paths: [(path_name, [point, point]), ...] to save into navigate.
    job_steps: run_path / turn_to_heading, alternating.

    Legs whose ends fall outside the recorded boundary are shortened
    along their own direction rather than dropped, so the chain stays
    connected — a gap would leave the robot too far from the next leg's
    start for navigate's entry check.
    """
    info = placement.check_box(box_points)
    if not info["is_box"]:
        return None, None, info

    lat0, lon0 = info["centroid"]
    panel_ne = np.array(lla_to_ned(panel_lat, panel_lon, 0.0, lat0, lon0, 0.0)[:2])
    local = info["local"]
    centre = local.mean(axis=0)
    shrunk = centre + (local - centre) * (1.0 - inside_margin_m / 3.0)

    def inside(point):
        for i in range(len(shrunk)):
            a, b = shrunk[i], shrunk[(i + 1) % len(shrunk)]
            edge = b - a
            if edge[0] * (point[1] - a[1]) - edge[1] * (point[0] - a[0]) > 0:
                return False
        return True

    def pull_inside(point):
        if inside(point):
            return point
        direction = point - panel_ne
        radius = float(np.linalg.norm(direction))
        if radius < 1e-6:
            return point
        direction /= radius
        while radius > 0.5:
            radius -= 0.1
            candidate = panel_ne + direction * radius
            if inside(candidate):
                return candidate
        return None

    leg_paths, steps, pulled_in = [], [], 0
    turn_signs = []
    previous_end, previous_heading = None, None
    for index, (start, end) in enumerate(build_chain(face_bearing_deg)):
        s = pull_inside(panel_ne + start)
        e = pull_inside(panel_ne + end)
        if s is None or e is None:
            continue
        if not np.allclose(panel_ne + start, s) or not np.allclose(panel_ne + end, e):
            pulled_in += 1
        if previous_end is not None:
            s = previous_end  # keep the chain connected after any shortening
        if float(np.linalg.norm(e - s)) < 0.8:
            continue  # too short to be worth a leg of its own
        name = f"{LEG_PATH_PREFIX} {index:02d}"
        points = []
        for ne in (s, e):
            lat, lon = _ne_to_lla(ne, lat0, lon0)
            points.append({"lat": lat, "lon": lon, "speed_mps": SPEED_MPS,
                           "pump": False, "clearance_m": CLEARANCE_M})
        leg_paths.append((name, points))
        heading = math.degrees(math.atan2(e[1] - s[1], e[0] - s[0]))
        if steps:
            # Alternate which way each turn goes round. turn_to_heading
            # always takes the shortest way, so this has to be forced.
            want = 1 if (len(leg_paths) % 2 == 0) else -1
            steps.extend(turn_steps(previous_heading, heading, want, turn_tolerance_deg))
            turn_signs.append(want)
        steps.append({"type": "run_path", "path": name})
        previous_end, previous_heading = e, heading

    info = dict(info)
    info["waypoints_pulled_in"] = pulled_in
    info["leg_count"] = len(leg_paths)
    info["length_m"] = sum(
        float(np.linalg.norm(np.array(lla_to_ned(p[1]["lat"], p[1]["lon"], 0.0, lat0, lon0, 0.0)[:2])
                             - np.array(lla_to_ned(p[0]["lat"], p[0]["lon"], 0.0, lat0, lon0, 0.0)[:2])))
        for _n, p in leg_paths)
    info["turn_count"] = sum(1 for s in steps if s["type"] == "turn_to_heading")
    info["turns_left"] = sum(1 for x in turn_signs if x < 0)
    info["turns_right"] = sum(1 for x in turn_signs if x > 0)
    return leg_paths, steps, info


class JobDriver:
    """Saves the legs into navigate and the job into jobs, then runs it.

    Server-to-server on localhost, the same convention jobs/app.py uses
    to reach waterbutt. The run itself goes through jobs rather than
    navigate directly, because jobs is what knows how to sequence
    run_path and turn_to_heading steps — and turning on the spot between
    straight legs is the whole reason this pattern is driveable.
    """

    def __init__(self, navigate_base_url, jobs_base_url, timeout_s=10.0):
        self.navigate = navigate_base_url
        self.jobs = jobs_base_url
        self.timeout_s = timeout_s

    def save(self, leg_paths, steps):
        for name, points in leg_paths:
            # navigate's api_save_path wants {"points": [...]}, not a bare list.
            r = requests.post(f"{self.navigate}/api/paths/{name}",
                              json={"points": points}, timeout=self.timeout_s)
            r.raise_for_status()
        r = requests.post(f"{self.jobs}/api/jobs/{JOB_NAME}",
                          json={"steps": steps}, timeout=self.timeout_s)
        r.raise_for_status()
        return r.json() if r.content else {}

    def start(self):
        return requests.post(f"{self.jobs}/control/start", json={"name": JOB_NAME},
                             timeout=self.timeout_s).json()

    def stop(self):
        requests.post(f"{self.jobs}/control/stop", timeout=self.timeout_s)

    def status(self):
        return requests.get(f"{self.jobs}/control/status", timeout=self.timeout_s).json()

    def wait_until_idle(self, poll_s=1.0, should_stop=None):
        """Block until the job finishes, returning its final status.

        `should_stop` is checked each poll so stopping the capture from
        the page halts the robot too — otherwise it would carry on round
        the pattern with nothing recording.
        """
        while True:
            status = self.status()
            if status.get("state") != "running":
                return status
            if should_stop is not None and should_stop():
                self.stop()
                return self.status()
            time.sleep(poll_s)
