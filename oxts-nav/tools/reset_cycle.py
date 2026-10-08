"""Repeated xNAV reset test (2026-10-05): does the u-blox come back to the
same RTK Integer position after each reset? See GNSS-EMI-problems.md
("Reset tests") for the method and results.

Each cycle:
  1. send "!reset" via oxts-nav's /command;
  2. wait for the INS to drop out of locked, then to re-initialise
     (InsNavMode 4 = position and heading);
  3. wait up to 60 s for the u-blox (GpsPrimaryPosMode) to reach
     RTK Integer (6);
  4. wait for the xNAV to be using it (see below), then record HOLD_S
     seconds of position/altitude, so each cycle gets a median.

Refuses to reset while navigate is running a path or a turn. Robot must
be stationary with the aruco marker out of view (aruco GAD would pull the
INS towards the marker and hide what GNSS says).

Usage (from the repo root, robot stationary):
  venv/bin/python3 oxts-nav/tools/reset_cycle.py [cycles] [out.jsonl] [init_heading_deg] [cfg1,cfg2,...]

- init_heading_deg: single-antenna mode (secondary unplugged): wait for
  the xNAV's chosen source to be u-blox Integer, then `!set init hea`.
- cfgN: mobile.cfg variants uploaded in rotation before each reset (e.g.
  two NTRIP sources). Not with RTK2go: a reconnect per cycle gets the IP
  banned (see the notes).
"""

import json
import math
import statistics
import sys
import time

from pathlib import Path

import requests
from simple_websocket import Client

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from shared.config import load_config  # noqa: E402

_services = load_config()["services"]
OXTS = f"http://127.0.0.1:{_services['oxts-nav']['port']}"
NAV_WS = f"ws://127.0.0.1:{_services['oxts-nav']['port']}/ws/nav"
NAVIGATE_WS = f"ws://127.0.0.1:{_services['navigate']['port']}/ws/navigate"
DROP_TIMEOUT_S = 20
INIT_TIMEOUT_S = 300
FIX_TIMEOUT_S = 60
HOLD_S = 10
SETTLE_TIMEOUT_S = 600  # after the u-blox's Integer, for the xNAV to accept it
SETTLE_N = 10            # consecutive samples (5 s at 2 Hz)
SETTLE_INN = 1.0         # filtered innovation, all axes

cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 20
out_path = sys.argv[2] if len(sys.argv) > 2 else "reset_cycles.jsonl"
# Optional: dual antenna disabled, so the INS can't initialise by itself.
# Wait for the xNAV's chosen position source to be u-blox Integer, then
# give it this heading - it initialises from the u-blox's position.
init_heading = float(sys.argv[3]) if len(sys.argv) > 3 else None
PRE_INIT_FIX_TIMEOUT_S = 180
# Optional: comma-separated mobile.cfg variants, uploaded in rotation
# before each cycle's reset (which is what loads them) - interleaves
# e.g. two correction sources cycle by cycle, so slow drifts in
# conditions hit both equally.
cfg_variants = sys.argv[4].split(",") if len(sys.argv) > 4 else []

ws = None


def sample():
    """Latest /ws/nav message; reconnects if the feed drops (it will,
    briefly, while the xNAV resets)."""
    global ws
    while True:
        try:
            if ws is None:
                ws = Client.connect(NAV_WS)
            return json.loads(ws.receive(timeout=5))
        except Exception:
            try:
                ws.close()
            except Exception:
                pass
            ws = None
            time.sleep(1)


def wait_for(cond, timeout_s):
    end = time.time() + timeout_s
    while time.time() < end:
        m = sample()
        if cond(m):
            return m
    return None


def navigate_busy():
    try:
        c = Client.connect(NAVIGATE_WS)
        st = json.loads(c.receive(timeout=5))
        c.close()
        return st.get("state") == "running"
    except Exception:
        return False  # navigate down is not a reason to stop


ins = lambda m: (m.get("nav") or {}).get("InsNavMode")
pri = lambda m: (m.get("status") or {}).get("GpsPrimaryPosMode")
src = lambda m: (m.get("status") or {}).get("GnssPosMode")

with open(out_path, "a") as out:
    for n in range(1, cycles + 1):
        if navigate_busy():
            print("navigate is running something - stopping")
            break
        rec = {"cycle": n, "t_reset": time.time()}
        if cfg_variants:
            cfg = cfg_variants[(n - 1) % len(cfg_variants)]
            with open(cfg, "rb") as f:
                r = requests.post(f"{OXTS}/xnav-config/mobile.cfg.txt", data=f.read(), timeout=20)
            if not r.ok:
                print(n, "config upload failed:", r.text, "- stopping")
                break
            rec["cfg"] = cfg
        requests.post(f"{OXTS}/command", data={"message": "!reset"}, timeout=5)
        dropped = wait_for(lambda m: ins(m) != 4, DROP_TIMEOUT_S)
        rec["reset_seen"] = dropped is not None
        if init_heading is not None:
            # Don't trust pre-reset status: wait for the u-blox to drop
            # out (the reset actually happening) before waiting for its
            # fresh Integer.
            wait_for(lambda m: pri(m) != 6, DROP_TIMEOUT_S)
            m = wait_for(lambda m: src(m) == 6 and pri(m) == 6, PRE_INIT_FIX_TIMEOUT_S)
            if m is None:
                rec["result"] = "no_integer_before_init"
                out.write(json.dumps(rec) + "\n"); out.flush()
                print(f"{n:2d} {time.strftime('%H:%M:%S')} no u-blox Integer in {PRE_INIT_FIX_TIMEOUT_S}s", flush=True)
                continue
            rec["t_preinit_integer"] = time.time()
            # Resend every 10 s until the INS locks, in case one is lost.
            m = None
            end = time.time() + INIT_TIMEOUT_S
            while m is None and time.time() < end:
                requests.post(f"{OXTS}/command", data={"message": f"!set init hea {init_heading:g}"}, timeout=5)
                m = wait_for(lambda m: ins(m) == 4, 10)
        else:
            m = wait_for(lambda m: ins(m) == 4, INIT_TIMEOUT_S)
        if m is None:
            rec["result"] = "no_init"
            out.write(json.dumps(rec) + "\n"); out.flush()
            print(n, "no init in", INIT_TIMEOUT_S, "s - stopping")
            break
        rec["t_init"] = time.time()
        rec["pri_at_init"] = pri(m)
        m = wait_for(lambda m: pri(m) == 6, FIX_TIMEOUT_S)
        if m is None:
            rec["result"] = "no_integer"
        else:
            # The INS altitude is only the u-blox's once the xNAV is
            # actually using it: it may still be on its own Gx solution
            # and rejecting the u-blox. So wait until it's settled on the
            # u-blox (source Integer, no rejects, small innovations for
            # SETTLE_N samples), then record HOLD_S seconds.
            rec["t_integer"] = time.time()
            rows = []
            settled_at = None
            end = time.time() + SETTLE_TIMEOUT_S
            while time.time() < end:
                m = sample()
                nav, st = m.get("nav") or {}, m.get("status") or {}
                if "Lat" not in nav:
                    continue
                inn = max(abs(st.get(k) or 0) for k in ("InnPosXFilt", "InnPosYFilt", "InnPosZFilt"))
                rows.append({"t": time.time(), "lat": math.degrees(nav["Lat"]), "lon": math.degrees(nav["Lon"]),
                             "alt": nav["Alt"], "pri": st.get("GpsPrimaryPosMode"),
                             "pos": st.get("GnssPosMode"), "sats": st.get("GnssPosNumSats"),
                             "rej": st.get("GnssPosReject"), "inn": inn})
                if settled_at is None:
                    tail = rows[-SETTLE_N:]
                    if len(tail) == SETTLE_N and all(
                        r["pri"] == 6 and r["pos"] == 6 and not r["rej"] and r["inn"] < SETTLE_INN for r in tail
                    ):
                        settled_at = time.time()
                        end = settled_at + HOLD_S
            rec["samples"] = rows
            rec["max_inn"] = max((r["inn"] for r in rows), default=None)
            rec["max_rej"] = max((r["rej"] or 0 for r in rows), default=None)
            if settled_at is not None:
                rec["result"] = "integer"
                rec["t_settled"] = settled_at
                held = [r for r in rows if r["t"] >= settled_at]
                for k in ("lat", "lon", "alt"):
                    rec[k] = statistics.median(r[k] for r in held)
            else:
                rec["result"] = "not_accepted"
        out.write(json.dumps(rec) + "\n"); out.flush()
        summary = (f"alt {rec['alt']:.3f} " if "alt" in rec else "") + f"maxInn {rec.get('max_inn')} maxRej {rec.get('max_rej')}"
        print(f"{n:2d} {time.strftime('%H:%M:%S')} {rec.get('cfg', '')} {rec['result']:12s} "
              f"init {rec['t_init'] - rec['t_reset']:.0f}s "
              f"{'fix %.0fs ' % (rec['t_integer'] - rec['t_init']) if 't_integer' in rec else ''}"
              f"{'settle %.0fs ' % (rec['t_settled'] - rec['t_integer']) if 't_settled' in rec else ''}{summary}",
              flush=True)
