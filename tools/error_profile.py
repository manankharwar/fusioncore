#!/usr/bin/env python3
"""Where does a run's error actually come from: steady drift, or a few excursions?

ATE RMSE is one number for a whole run and it hides which of those it is. Squaring
means a 400 second excursion in a 90 minute run can dominate a figure that otherwise
describes a filter doing fine. Those are completely different bugs and they have
completely different fixes, so it is worth knowing which one a number is describing
before trying to improve it.

This was written on 2026-10-10 after three days of benchmarking had established that
FusionCore scored 155 m on NCLT 2012-08-20 and 14 m on 2013-04-05 without anyone
asking when, inside those runs, the error appeared. The answer took ten minutes from
trajectories already on disk: 2012-08-20 sits near 32 m for the entire run and then
jumps 778 m in a single step at t=3973, which is 14 s after a 171 s GPS blackout ends.
Two raw fixes 0.07 s apart differ by 825 m there. See issue #117.

Usage:
    python3 tools/error_profile.py --gt GT.tum --est EST.tum [--est2 RL.tum]
    python3 tools/error_profile.py --run <dir>      # expects fc.tum and rl.tum inside

Reports, per trajectory:
  * RMSE, median and max, because RMSE minus median is the excursion tax
  * the largest single-step error jumps, with timestamps, which is where to look
  * RMSE with the worst contiguous window removed, which says how much of the
    headline number lives in a small part of the run

A caution learned by making the mistake: a large excursion tax does NOT mean fixing
the excursion changes the verdict. On the five NCLT rows measured here, scoring by
median instead of RMSE flipped the FusionCore-versus-control verdict on zero of five.
Check that before concluding a filter is "really" fine.
"""
import argparse
import os
import sys

import numpy as np

os.environ.setdefault("MPLBACKEND", "Agg")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def profile(gt, est, label, window_frac, n_jumps):
    import evaluate
    a = evaluate.compute_ate(gt, est)
    e = a["errors"]
    t = a["gt_s"].timestamps
    rel = t - t[0]
    n = len(e)

    w = max(1, int(window_frac * n))
    csum = np.cumsum(np.concatenate([[0.0], e ** 2]))
    worst = int(np.argmax(csum[w:] - csum[:-w]))
    keep = np.ones(n, dtype=bool)
    keep[worst:worst + w] = False

    rmse = float(np.sqrt(np.mean(e ** 2)))
    trimmed = float(np.sqrt(np.mean(e[keep] ** 2)))
    print(f"\n  {label}")
    print(f"    samples            {n}")
    print(f"    RMSE               {rmse:10.2f} m")
    print(f"    median             {float(np.median(e)):10.2f} m")
    print(f"    max                {float(e.max()):10.2f} m   at t={rel[int(np.argmax(e))]:.0f} s")
    print(f"    RMSE without the worst {window_frac*100:.0f}% window "
          f"({rel[worst]:.0f}-{rel[min(worst+w, n-1)]:.0f} s): {trimmed:10.2f} m")
    if rmse > 0:
        print(f"    so {100.0*(1.0 - (trimmed/rmse)**2):.0f}% of the squared error "
              f"lives in {window_frac*100:.0f}% of the run")
    if n > 1:
        d = np.diff(e)
        idx = np.argsort(d)[-n_jumps:][::-1]
        print(f"    largest single-step jumps:")
        for i in idx:
            print(f"      t={rel[i+1]:8.1f} s   +{d[i]:8.2f} m   ({e[i]:.2f} -> {e[i+1]:.2f})")
    return rmse, float(np.median(e))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt", help="ground truth TUM")
    ap.add_argument("--est", help="estimate TUM")
    ap.add_argument("--est2", default=None, help="second estimate, e.g. the control")
    ap.add_argument("--run", default=None,
                    help="run directory containing fc.tum and rl.tum")
    ap.add_argument("--gt-dir", default=os.path.expanduser("~/nclt"),
                    help="where <sequence>/ground_truth.tum lives, for --run")
    ap.add_argument("--window", type=float, default=0.05,
                    help="contiguous fraction of the run to trim (default 0.05)")
    ap.add_argument("--jumps", type=int, default=5)
    args = ap.parse_args(argv)

    import evaluate

    if args.run:
        run = args.run.rstrip("/")
        seq = os.path.basename(run).split("_", 1)[-1]
        gt_path = os.path.join(args.gt_dir, seq, "ground_truth.tum")
        pairs = [("FusionCore", os.path.join(run, "fc.tum")),
                 ("control", os.path.join(run, "rl.tum"))]
        pairs = [(n, p) for n, p in pairs if os.path.exists(p)]
        if not pairs:
            print(f"no fc.tum or rl.tum in {run}", file=sys.stderr)
            return 2
    else:
        if not (args.gt and args.est):
            ap.error("need --gt and --est, or --run")
        gt_path = args.gt
        pairs = [("estimate", args.est)]
        if args.est2:
            pairs.append(("second", args.est2))

    if not os.path.exists(gt_path):
        print(f"ground truth not found: {gt_path}", file=sys.stderr)
        return 2

    gt = evaluate.load_tum(gt_path)
    print(f"ground truth: {gt_path}")
    out = {}
    for name, path in pairs:
        out[name] = profile(gt, evaluate.load_tum(path), f"{name}: {path}",
                            args.window, args.jumps)

    if len(out) == 2:
        (n1, (r1, m1)), (n2, (r2, m2)) = list(out.items())
        print("\n  verdict by each metric, because they can disagree:")
        print(f"    by RMSE   {n1 if r1 < r2 else n2} is better "
              f"({r1:.2f} vs {r2:.2f})")
        print(f"    by median {n1 if m1 < m2 else n2} is better "
              f"({m1:.2f} vs {m2:.2f})")
        if (r1 < r2) != (m1 < m2):
            print("    THEY DISAGREE: the RMSE verdict is being set by excursions, "
                  "not by typical accuracy.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
