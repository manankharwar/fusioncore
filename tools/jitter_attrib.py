#!/usr/bin/env python3
"""Where does the fused position's output-rate jitter come from?

    python3 tools/jitter_attrib.py <attrib.csv> <exported.txt> [--gap-s 10]

Reads the per-event split written by `hardware/replay.cpp attrib=...`: for every IMU
and ENC event, how far the PREDICT step moved the position and how far the UPDATE
moved it afterwards. Replay with attrib on is byte-identical to replay without it,
because the split works by running the predict explicitly (FusionCore::predict_only)
before an update whose own predict then has nothing left to do.

Jitter is path that does not become displacement. For each source this reports the
path it contributes (sum of |step|), its net displacement (|sum of steps|) and their
ratio. A source that only ever moves the robot forward has ratio near 1. A pair of
sources pulling against each other show large paths, small nets, and updates that
point backwards against the predict just before them.

Windows come from the RAW fix stream in the exported log: "gap" is any stretch with
no fix for >= --gap-s seconds, everything else is "gnss".
"""
import argparse
import math
import sys

import numpy as np


def fix_times(path):
    out = []
    for ln in open(path):
        if ln.startswith("GNSS"):
            out.append(float(ln.split()[1]))
    return np.array(sorted(out))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("attrib")
    ap.add_argument("exported")
    ap.add_argument("--gap-s", type=float, default=10.0)
    a = ap.parse_args(argv)

    d = np.genfromtxt(a.attrib, delimiter=",", names=True, dtype=None, encoding=None)
    if d.size == 0:
        sys.exit("attrib file is empty")
    t = d["t"].astype(float)
    kind = d["kind"].astype(str)
    pdx, pdy = d["pred_dx"].astype(float), d["pred_dy"].astype(float)
    udx, udy = d["upd_dx"].astype(float), d["upd_dy"].astype(float)

    g = fix_times(a.exported)
    in_gap = np.zeros(len(t), dtype=bool)
    dg = np.diff(g)
    for i in np.where(dg >= a.gap_s)[0]:
        in_gap |= (t > g[i]) & (t < g[i + 1])

    def report(mask, label):
        n = int(mask.sum())
        if n == 0:
            return
        dur = t[mask].max() - t[mask].min()
        print(f"\n  {label}: {n} events over {dur:.1f} s")
        print(f"    {'source':18s} {'path m':>10s} {'net m':>9s} {'path/net':>9s} "
              f"{'mean |step| mm':>15s}")
        tot_x = np.zeros(n); tot_y = np.zeros(n)
        rows = [("predict (all)", pdx[mask], pdy[mask])]
        for k in ("IMU", "ENC"):
            m = mask & (kind == k)
            rows.append((f"{k} update", udx[m], udy[m]))
        for name, x, y in rows:
            step = np.hypot(x, y)
            path = step.sum()
            net = math.hypot(x.sum(), y.sum())
            ratio = path / net if net > 0 else float("inf")
            print(f"    {name:18s} {path:10.1f} {net:9.1f} {ratio:9.2f} "
                  f"{1000 * step.mean():15.3f}")
        # Output-rate path of the combined position, event by event.
        cx = pdx[mask] + udx[mask]; cy = pdy[mask] + udy[mask]
        comb = np.hypot(cx, cy).sum()
        print(f"    {'combined output':18s} {comb:10.1f} "
              f"{math.hypot(cx.sum(), cy.sum()):9.1f}")
        # Does each update point back against the predict that just ran?
        for k in ("IMU", "ENC"):
            m = mask & (kind == k)
            px, py, ux, uy = pdx[m], pdy[m], udx[m], udy[m]
            pn, un = np.hypot(px, py), np.hypot(ux, uy)
            ok = (pn > 1e-9) & (un > 1e-9)
            cos = (px[ok] * ux[ok] + py[ok] * uy[ok]) / (pn[ok] * un[ok])
            undo = (-(px[ok] * ux[ok] + py[ok] * uy[ok]) / pn[ok]).clip(min=0).sum()
            print(f"    {k} update vs its own predict: cos median {np.median(cos):+.3f}, "
                  f"opposed (cos<-0.5) {100 * np.mean(cos < -0.5):.1f}%, "
                  f"predict motion undone {undo:.1f} m")

    report(~in_gap, "GNSS present")
    report(in_gap, "inside gaps")


if __name__ == "__main__":
    main()
