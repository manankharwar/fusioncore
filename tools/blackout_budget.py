#!/usr/bin/env python3
"""Where does the error actually live: in the GNSS gaps, in the re-convergence after
them, or while coverage is steady?

    python3 tools/blackout_budget.py <exported.txt> <fused.tum> <ground_truth.tum>

Windows are defined from the RAW fix stream, not from what the filter accepted, so the
classification does not depend on the thing being measured:

    gap            no fix available for >= GAP_S seconds
    reconvergence  from the first fix after a gap until the fused error has stayed
                   within GNSS-level error for HOLD_S seconds
    steady         everything else

Each window's share of TOTAL SQUARED ERROR is reported, not its RMSE. RMSE is driven by
squared error, so the shares sum to 100% and answer "what fraction of this sequence's
error lives in the gaps" directly.

Drift inside a gap is reported as a percentage of distance travelled during that gap, so
a 20 s gap and a 400 s gap can be compared.

Every comparison this prints carries a provenance tag: pipeline, config, scoring points,
coverage. See tools/provenance.py for why.
"""
import argparse
import bisect
import math
import sys

GAP_S = 10.0
HOLD_S = 10.0


def load_tum(path):
    rows = []
    for ln in open(path):
        p = ln.split()
        if len(p) < 4 or ln.startswith("#"):
            continue
        try:
            rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3])))
        except ValueError:
            continue
    rows.sort()
    return rows


def fix_times(exported):
    ts = []
    for ln in open(exported):
        if ln.startswith("GNSS "):
            try:
                ts.append(float(ln.split()[1]))
            except (ValueError, IndexError):
                pass
    ts.sort()
    return ts


def align(traj, gt):
    """SE3 is not re-estimated here: the caller passes trajectories already in the same
    frame. Associate by nearest timestamp within 50 ms."""
    gts = [g[0] for g in gt]
    out = []
    for t, x, y, z in traj:
        i = bisect.bisect_left(gts, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(gts):
                if best is None or abs(gts[j] - t) < abs(gts[best] - t):
                    best = j
        if best is not None and abs(gts[best] - t) < 0.05:
            g = gt[best]
            out.append((t, math.dist((x, y, z), (g[1], g[2], g[3]))))
    return out


def umeyama_xy(traj, gt):
    """Least-squares rigid alignment in the horizontal plane, so the budget is not
    dominated by a constant frame offset. Returns the aligned trajectory."""
    gts = [g[0] for g in gt]
    pairs = []
    for t, x, y, z in traj:
        i = bisect.bisect_left(gts, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(gts):
                if best is None or abs(gts[j] - t) < abs(gts[best] - t):
                    best = j
        if best is not None and abs(gts[best] - t) < 0.05:
            pairs.append(((x, y, z), gt[best][1:]))
    if len(pairs) < 10:
        return traj
    n = len(pairs)
    mx = [sum(p[0][k] for p in pairs) / n for k in range(3)]
    my = [sum(p[1][k] for p in pairs) / n for k in range(3)]
    sxx = sxy = syx = syy = 0.0
    for a, b in pairs:
        ax, ay = a[0] - mx[0], a[1] - mx[1]
        bx, by = b[0] - my[0], b[1] - my[1]
        sxx += ax * bx; sxy += ax * by; syx += ay * bx; syy += ay * by
    theta = math.atan2(sxy - syx, sxx + syy)
    c, s = math.cos(theta), math.sin(theta)
    out = []
    for t, x, y, z in traj:
        dx, dy, dz = x - mx[0], y - mx[1], z - mx[2]
        out.append((t, c * dx - s * dy + my[0], s * dx + c * dy + my[1], dz + my[2]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("exported"); ap.add_argument("fused"); ap.add_argument("gt")
    ap.add_argument("--label", default="")
    ap.add_argument("--pipeline", default="replay")
    ap.add_argument("--config", default="nclt_shipped")
    ap.add_argument("--ref", type=float, default=None,
                    help="GNSS-level error in metres. Pass the RAW fix median "
                         "against ground truth. Without it the fused steady "
                         "median is used, which is the thing under test and is "
                         "far too loose, so re-convergence is under-detected.")
    a = ap.parse_args(argv)

    ft = fix_times(a.exported)
    gt = load_tum(a.gt)
    fused = umeyama_xy(load_tum(a.fused), gt)
    if not ft or not gt or not fused:
        sys.exit("missing data")

    # gaps from the RAW fix stream
    gaps = [(x, y) for x, y in zip(ft, ft[1:]) if y - x >= GAP_S]
    t0, t1 = ft[0], ft[-1]
    span = t1 - t0
    cov = 1.0

    err = align(fused, gt)
    if not err:
        sys.exit("no associated pairs")

    # GNSS-level error: median error of the fused solution during steady coverage is
    # not usable as the reference (it is the thing under test), so use the median of
    # the raw fixes against ground truth, computed by the caller and passed through
    # the steady window's own median as a fallback.
    steady_errs = []
    for t, e in err:
        if not any(g0 <= t <= g1 for g0, g1 in gaps):
            steady_errs.append(e)
    steady_errs.sort()
    ref = a.ref if a.ref is not None else (
        steady_errs[len(steady_errs) // 2] if steady_errs else 0.0)
    ref_src = "raw GNSS median (supplied)" if a.ref is not None else \
              "fused steady median (FALLBACK, too loose)"

    # re-convergence: after each gap, until error holds under ref for HOLD_S
    reconv = []
    for g0, g1 in gaps:
        good_since = None
        end = g1
        for t, e in err:
            if t <= g1:
                continue
            if e <= ref:
                if good_since is None:
                    good_since = t
                elif t - good_since >= HOLD_S:
                    end = t; break
            else:
                good_since = None
            if t - g1 > 600:
                end = t; break
        if end > g1:
            reconv.append((g1, end))

    def cls(t):
        if any(g0 <= t <= g1 for g0, g1 in gaps): return "gap"
        if any(r0 < t <= r1 for r0, r1 in reconv): return "reconv"
        return "steady"

    tot = {"gap": 0.0, "reconv": 0.0, "steady": 0.0}
    cnt = {"gap": 0, "reconv": 0, "steady": 0}
    for t, e in err:
        k = cls(t); tot[k] += e * e; cnt[k] += 1
    grand = sum(tot.values()) or 1.0

    tag = f"{a.pipeline}/{a.config}/windows/{cov:.0%}"
    print(f"\n  {a.label}   provenance: {tag}")
    print(f"  fix stream: {len(ft)} fixes over {span:.0f} s, "
          f"{len(gaps)} gaps of >= {GAP_S:.0f} s")
    print(f"  GNSS-level reference {ref:.3f} m, from {ref_src}")
    print(f"\n  {'window':>8} {'samples':>9} {'%time':>7} {'RMSE':>9} "
          f"{'share of squared error':>24}")
    for k in ("gap", "reconv", "steady"):
        rm = math.sqrt(tot[k] / cnt[k]) if cnt[k] else 0.0
        print(f"  {k:>8} {cnt[k]:>9} {100*cnt[k]/len(err):>6.1f}% {rm:>9.3f} "
              f"{100*tot[k]/grand:>23.1f}%")
    print(f"\n  gaps: " + ", ".join(f"{g1-g0:.0f}s" for g0, g1 in gaps[:12]) +
          ("" if len(gaps) <= 12 else f" ... ({len(gaps)} total)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
