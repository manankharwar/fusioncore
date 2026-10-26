#!/usr/bin/env python3
"""Score several trajectories of one NCLT sequence, every metric, on identical points.

    python3 tools/score_nclt_suite.py <exported.txt> <ground_truth.tum> \
        NAME=path[@pipeline] [NAME=path[@pipeline] ...] --seq SEQ

For each named trajectory, two scorings, both through hardware/ate_tum.py (Umeyama SE3,
0.1 s association, which matches evo at tools/evaluate.py's tolerance):

  full_traj   the whole trajectory: ATE RMSE 3D, median, truth-rate path ratio and the
              output-rate path ratio (jitter)
  fix_times   one row per GPS fix (the single nearest pose within 50 ms, exactly as
              tools/subsample_to_fix_times.py), every column AND raw GPS restamped to the
              fix time and restricted to fixes all columns have, so every column is scored
              on literally the same timestamps: ATE RMSE 3D and median

Coverage is reported per trajectory as the share of the sequence's GPS-fix span its
timestamps cover; anything under 99% is flagged LOW COVERAGE. The tag on every row names
this script's content hash, the pipeline, the scoring points and the coverage, so a number
can always be traced to the code and run that produced it. It prints no aggregate and no
"winner": tools/report_rule_20261025.md forbids one.
"""
import argparse
import hashlib
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ATE = os.path.join(HERE, "..", "hardware", "ate_tum.py")


def version():
    with open(os.path.abspath(__file__), "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:12]


def tum_rows(path):
    rows = [l for l in open(path) if len(l.split()) >= 8 and not l.startswith("#")]
    return sorted(rows, key=lambda l: float(l.split()[0]))


def ate(gt, rows):
    with tempfile.NamedTemporaryFile("w", suffix=".tum", delete=False) as f:
        f.writelines(rows)
        p = f.name
    try:
        out = subprocess.run([sys.executable, ATE, gt, p], capture_output=True, text=True).stdout
    finally:
        os.unlink(p)
    get = lambda pat: float(re.search(pat, out).group(1)) if re.search(pat, out) else float("nan")
    return dict(n=int(get(r"associated pairs\s+(\d+)")), rmse=get(r"ATE RMSE 3D\s+([\d.]+)"),
                median=get(r"ATE median / max\s+([\d.]+)"), path=get(r"path length ratio\s+([\d.]+)"),
                outrate=get(r"output-rate path ratio\s*([\d.]+)"))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("exported"); ap.add_argument("gt")
    ap.add_argument("arms", nargs="+", help="NAME=path[@pipeline]")
    ap.add_argument("--seq", default="")
    a = ap.parse_args(argv)

    g = {}
    for l in open(a.exported):
        if l.startswith("GNSS "):
            p = l.split()
            g[p[1]] = f"{float(p[2]):.6f} {float(p[3]):.6f} {float(p[4]):.6f}"
    fts = sorted(g, key=float)
    ft = np.array([float(s) for s in fts])
    span = ft[-1] - ft[0]

    arms = []
    for spec in a.arms:
        name, rest = spec.split("=", 1)
        path, pipe = (rest.split("@", 1) + ["?"])[:2]
        arms.append((name, path, pipe))

    picks, cover = {}, {}
    for name, path, _ in arms:
        rows = tum_rows(path)
        rt = np.array([float(l.split()[0]) for l in rows])
        lo, hi = max(rt[0], ft[0]), min(rt[-1], ft[-1])
        cover[name] = max(0.0, hi - lo) / span if span > 0 else 0.0
        idx = np.clip(np.searchsorted(rt, ft), 1, len(rt) - 1)
        best = np.where(abs(rt[idx - 1] - ft) <= abs(rt[idx] - ft), idx - 1, idx)
        ok = abs(rt[best] - ft) < 0.05
        picks[name] = {fts[i]: rows[best[i]].split()[1:8] for i in range(len(fts)) if ok[i]}
        picks[name + "__full"] = rows
    common = [s for s in fts if all(s in picks[n] for n, _, _ in arms)]

    tag = lambda pipe, pts, cov: f"[{version()} | {pipe} | {pts} | cov {100 * cov:.1f}%]"
    print(f"# {a.seq}  score_nclt_suite {version()}  fixes in common {len(common)} of {len(fts)}")
    gn = ate(a.gt, [f"{s} {g[s]} 0 0 0 1\n" for s in common])
    print(f"{a.seq} raw_gps  fix_times  n {gn['n']:6d}  rmse {gn['rmse']:9.3f}  median {gn['median']:8.3f}  "
          f"{tag('raw', 'fix_times', len(common) / len(fts))}")
    for name, path, pipe in arms:
        fx = ate(a.gt, [s + " " + " ".join(picks[name][s]) + "\n" for s in common])
        if fx["n"] != gn["n"]:
            print(f"{a.seq} {name} REFUSED at fix_times: {fx['n']} pairs against raw GPS {gn['n']}")
        else:
            print(f"{a.seq} {name:8s} fix_times  n {fx['n']:6d}  rmse {fx['rmse']:9.3f}  median {fx['median']:8.3f}  "
                  f"{tag(pipe, 'fix_times', cover[name])}")
        fu = ate(a.gt, picks[name + "__full"])
        flag = "  LOW COVERAGE" if cover[name] < 0.99 else ""
        print(f"{a.seq} {name:8s} full_traj  n {fu['n']:6d}  rmse {fu['rmse']:9.3f}  median {fu['median']:8.3f}  "
              f"path {fu['path']:.3f}  outrate {fu['outrate']:.3f}  {tag(pipe, 'full_traj', cover[name])}{flag}")


if __name__ == "__main__":
    main()
