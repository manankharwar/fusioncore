#!/usr/bin/env python3
"""Raw GNSS against FusionCore, scored at identical GPS fix times. The README table's source.

    python3 tools/score_raw_vs_fc.py <exported.txt> <ground_truth.tum> \
        <fc_before.tum> <fc_after.tum> [--label SEQ]

Exists because the README's 2026-10-18 table could not be reproduced: its files still
scored, but the scoring step that produced the published numbers was not on disk. This
script IS that step now, and it prints its own content hash in the provenance tag so a
table row can always be traced to the exact code that produced it.

Steps, all on disk:
  1. raw GNSS fixes as a trajectory (tools/gnss_to_tum.py logic)
  2. each FusionCore trajectory subsampled to within 50 ms of a fix
     (tools/subsample_to_fix_times.py logic)
  3. every column stamped with the FIX time and restricted to fixes all three have, so
     they are scored on literally identical timestamps
  4. each scored through hardware/ate_tum.py (Umeyama SE3, 0.1 s association), the scorer
     that matches evo at tools/evaluate.py's tolerance

Refuses to print a row whose three columns were not scored on the same number of pairs.
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


def script_version():
    with open(os.path.abspath(__file__), "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:12]


def tum_rows(path):
    return [l for l in open(path) if len(l.split()) >= 8 and not l.startswith("#")]


def times(rows):
    return np.array([float(l.split()[0]) for l in rows])


def score(gt, rows):
    with tempfile.NamedTemporaryFile("w", suffix=".tum", delete=False) as f:
        f.writelines(rows)
        p = f.name
    try:
        out = subprocess.run([sys.executable, ATE, gt, p], capture_output=True, text=True).stdout
    finally:
        os.unlink(p)
    n = int(re.search(r"associated pairs\s+(\d+)", out).group(1))
    r = float(re.search(r"ATE RMSE 3D\s+([\d.]+)", out).group(1))
    return n, r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("exported"); ap.add_argument("gt")
    ap.add_argument("before"); ap.add_argument("after")
    ap.add_argument("--label", default="")
    a = ap.parse_args(argv)

    # One row per fix in every column, all stamped with the FIX time, so the three columns
    # are scored on literally the same timestamps. A fused row is the single nearest row to
    # the fix (exactly subsample_to_fix_times.py) and is kept only within 50 ms; restamping
    # it moves it at most that far in time, a few centimetres at NCLT speeds.
    g = {}
    for l in open(a.exported):
        if l.startswith("GNSS "):
            p = l.split()
            g[p[1]] = f"{float(p[2]):.6f} {float(p[3]):.6f} {float(p[4]):.6f}"
    fts = sorted(g, key=float)
    ft = np.array([float(s) for s in fts])
    picks = {}
    for k, path in (("before", a.before), ("after", a.after)):
        rows = sorted(tum_rows(path), key=lambda l: float(l.split()[0]))
        rt = times(rows)
        idx = np.clip(np.searchsorted(rt, ft), 1, len(rt) - 1)
        best = np.where(abs(rt[idx - 1] - ft) <= abs(rt[idx] - ft), idx - 1, idx)
        ok = abs(rt[best] - ft) < 0.05
        picks[k] = {fts[i]: rows[best[i]].split()[1:8] for i in range(len(fts)) if ok[i]}
    common = [s for s in fts if s in picks["before"] and s in picks["after"]]
    cols = {"gnss": [f"{s} {g[s]} 0 0 0 1\n" for s in common]}
    for k in ("before", "after"):
        cols[k] = [s + " " + " ".join(picks[k][s]) + "\n" for s in common]

    res = {k: score(a.gt, v) for k, v in cols.items()}
    pairs = {v[0] for v in res.values()}
    if len(pairs) != 1:
        sys.exit(f"REFUSED: columns scored on different pair counts {res}")
    g, b, f = res["gnss"][1], res["before"][1], res["after"][1]
    print(f"{a.label:11s} {pairs.pop():6d} {g:10.3f} {b:12.3f} {f:12.3f} {b / g:9.3f}x {f / g:9.3f}x  "
          f"[score_raw_vs_fc {script_version()} / fix_times / ate_tum 0.1s]")


if __name__ == "__main__":
    main()
