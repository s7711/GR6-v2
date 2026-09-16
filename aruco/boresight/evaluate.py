"""Compare candidate bore-sight marker layouts and capture paths by
Monte Carlo, and print the table that decides the physical setup.

    ../../venv/bin/python evaluate.py

Read the "absorbed" columns: those are the RMS hpr_cb error relative to
truth-plus-INS-bias, i.e. the part the layout and path actually control.
The "total" column additionally carries the constant INS attitude bias,
which no geometry can separate from hpr_cb and which is harmless here
anyway (see solve.py).
"""

import sys

import numpy as np

import layouts
import sim

DXC_B = np.array([0.0775, 0.002, -0.07])
TRUE_HPR_CB = np.array([2.0, -1.2, 1.5])  # a plausible real mount error; the solve starts from [0,0,0]
TRIALS = 25


def run(name, layout, legs, trials=TRIALS, noise=None, seed=0):
    if not legs:
        return {"name": name, "note": "no driveable legs"}
    out = sim.monte_carlo(layout, layouts.path_fn(legs), TRUE_HPR_CB, DXC_B,
                          noise=noise, trials=trials, seed=seed)
    out["name"] = name
    return out


def image_radius_stats(layout, legs, seed=3):
    """Median distance of a detected marker from the principal point, in
    pixels — the single number that predicts roll observability."""
    rng = np.random.default_rng(seed)
    t, pos, hpr = sim.legs_to_poses(legs, rng=rng)
    cx, cy = sim.CAMERA_MATRIX[0, 2], sim.CAMERA_MATRIX[1, 2]
    radii = []
    for k in range(len(layout)):
        ok, corners = sim.visible(pos, hpr, layout, k, TRUE_HPR_CB, DXC_B)
        if ok.any():
            centres = corners[ok].mean(axis=1)
            radii.append(np.hypot(centres[:, 0] - cx, centres[:, 1] - cy))
    return float(np.median(np.concatenate(radii))) if radii else float("nan")


def print_table(rows):
    print(f"{'layout / path':<34} {'obs':>6} {'rms_px':>7} "
          f"{'H':>7} {'P':>7} {'R':>7}   {'H_tot':>7}")
    print("-" * 82)
    for r in rows:
        if "note" in r:
            print(f"{r['name']:<34} {r['note']}")
            continue
        a = r["absorbed_rms_deg"]
        print(f"{r['name']:<34} {r['obs_median']:>6.0f} {r['rms_px']:>7.3f} "
              f"{a[0]:>7.3f} {a[1]:>7.3f} {a[2]:>7.3f}   {r['total_rms_deg'][0]:>7.3f}")


def error_budget(layout, legs, trials=TRIALS):
    """Turn one error source on at a time. This is the most useful table
    here: it says where the accuracy actually goes, and so what is worth
    spending money or driving time on."""
    off = dict(sigma_px=1e-9, pos_sigma_m=0.0, heading_sigma_deg=0.0,
               heading_bias_deg=0.0, tilt_sigma_deg=0.0, tilt_bias_deg=0.0)
    cases = [
        ("camera corner noise only (0.2px)", {**off, "sigma_px": 0.2}),
        ("INS position only (0.01m RTK)", {**off, "pos_sigma_m": 0.01}),
        ("INS heading only (0.18d, tau=30s)", {**off, "heading_sigma_deg": 0.18, "heading_tau_s": 30.0}),
        ("  ... same, if it were WHITE", {**off, "heading_sigma_deg": 0.18, "heading_tau_s": 0.0}),
        ("INS pitch/roll only (0.04d)", {**off, "tilt_sigma_deg": 0.04, "tilt_tau_s": 30.0}),
        ("everything together", {}),
    ]
    return [run(name, layout, legs, trials=trials, noise=sim.NoiseModel(**kw))
            for name, kw in cases]


def main():
    fan = layouts.fan_legs()
    past = layouts.past_legs()
    both = fan + past

    print("Driveable legs inside the recorded box: "
          f"fan={len(fan)} past={len(past)}\n")

    print("=== 0. Where does the error actually come from? (3 staggered, fan+past) ===")
    print_table(error_budget(layouts.row3h(), both))

    print("\n=== 1. How much does the marker arrangement matter? (fan+past path) ===")
    rows = []
    for name, lay in (("1 marker", layouts.single()),
                      ("3 stacked vertically", layouts.stack3()),
                      ("3 in a flat row (recommended)", layouts.recommended()),
                      ("3 in a row, staggered heights", layouts.row3h()),
                      ("3 in a triangle", layouts.triangle3()),
                      ("3 in a triangle, taller apex", layouts.triangle3(apex_height=0.85))):
        rows.append(run(name, lay, both))
    print_table(rows)
    print("Only the first two are distinguishable from the rest — a horizontal row")
    print("beats one marker and beats a vertical stack, on roll. Flat vs staggered vs")
    print("triangle is inside the sampling noise even at 50 trials, so build flat.")

    print("\n=== 2. How much does the path matter? (3 staggered) ===")
    rows = []
    for name, legs in (("fan only (drive straight at it)", fan),
                       ("past only (drive past it)", past),
                       ("fan + past", both)):
        lay = layouts.row3h()
        r = run(name, lay, legs)
        r["name"] = f"{name}  [rho={image_radius_stats(lay, legs):.0f}px]"
        rows.append(r)
    print_table(rows)

    print("\n=== 3. How long does the session need to be? (3 staggered, fan+past) ===")
    rows = []
    for repeats in (1, 2, 4, 8):
        legs = both * repeats
        minutes = sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in legs) / 0.2 / 60.0
        rows.append(run(f"{repeats}x the pattern (~{minutes:.0f} min driving)",
                        layouts.row3h(), legs))
    print_table(rows)

    print("\n=== 4. Is 97mm enough? (3 staggered, fan+past) ===")
    rows = []
    for size in (0.097, 0.150, 0.200):
        rows.append(run(f"{size*1000:.0f} mm markers", layouts.row3h(size=size), both))
    print_table(rows)

    print("\n=== 5. Is a second panel (6 markers, both sides) worth it? ===")
    rows = [run("3 staggered, one side, fan+past", layouts.row3h(), both),
            run("6 staggered, back-to-back, both sides", layouts.row3h_back_to_back(),
                layouts.both_sides_legs())]
    print_table(rows)

    print("\nColumns: obs = median detections per session; H/P/R = RMS hpr_cb error")
    print("in degrees relative to truth+INS-bias; H_tot includes the absorbed bias.")
    print(f"Target: 0.1 deg.  Simulated truth hpr_cb = {TRUE_HPR_CB}, solve starts from [0,0,0].")


if __name__ == "__main__":
    sys.exit(main())
