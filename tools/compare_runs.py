#!/usr/bin/env python3
"""Compare two NCLT runs where ATE RMS is the wrong question.

    python3 tools/compare_runs.py <runA> <runB> [--seq 2013-04-05]

FusionCore's error on a sequence with GPS outages is heavy-tailed: the median is
around 5 m while the RMS is 130-185 m, because a handful of blackout excursions
dominate. Comparing RMS therefore compares the worst excursion, not the filter.
This reports the distribution, the fraction of the run spent lost, and the GPS
rejection cascades that keep it lost.
"""
import math, os, re, sys

def load(p, stride=1):
    out = []
    for i, l in enumerate(open(p)):
        if i % stride: continue
        f = l.split()
        if len(f) >= 4: out.append((float(f[0]), float(f[1]), float(f[2])))
    return out

def at(tr, t):
    lo, hi = 0, len(tr) - 1
    if t <= tr[0][0]: return tr[0]
    if t >= tr[-1][0]: return tr[-1]
    while hi - lo > 1:
        m = (lo + hi) // 2
        if tr[m][0] < t: lo = m
        else: hi = m
    return tr[lo]

def pct(v, q):
    v = sorted(v); return v[min(len(v) - 1, int(q * len(v)))]

def rejections(run):
    """Longest run of consecutive chi2 rejections: the gate-lockout measure."""
    log = os.path.join(run, "launch.log")
    if not os.path.isfile(log): return None
    ts = []
    for l in open(log, errors="ignore"):
        if "GNSS fix rejected" in l:
            m = re.search(r"\[(\d{10}\.\d+)\]", l)
            if m: ts.append(float(m.group(1)))
    if not ts: return (0, 0, 0.0)
    best = cur = 1; span = 0.0; start = ts[0]
    for a, b in zip(ts, ts[1:]):
        if b - a < 12:
            cur += 1
        else:
            if cur > best: best, span = cur, a - start
            cur = 1; start = b
    if cur > best: best, span = cur, ts[-1] - start
    return (len(ts), best, span)

def report(run, gt, label):
    fc = os.path.join(run, "fc.tum")
    if not os.path.isfile(fc):
        print("  %-22s no fc.tum yet" % label); return
    tr = load(fc, 10)
    e = [math.hypot(at(tr, g[0])[1] - g[1], at(tr, g[0])[2] - g[2]) for g in gt]
    rms = math.sqrt(sum(x * x for x in e) / len(e))
    lost = sum(1 for x in e if x > 50) / len(e) * 100
    r = rejections(run)
    print("  %-22s %7.2f %7.2f %8.2f %8.2f %8.2f %6.1f%%" % (
        label, pct(e, .5), pct(e, .75), pct(e, .90), pct(e, .95), rms, lost))
    if r:
        print("  %-22s   GPS rejections %d, longest cascade %d fixes over %.0f s" % ("", r[0], r[1], r[2]))

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    seq = "2013-04-05"
    if "--seq" in sys.argv: seq = sys.argv[sys.argv.index("--seq") + 1]
    gt = load("/home/manankharwar/nclt/%s/ground_truth.tum" % seq)
    print("\n  %s: error vs ground truth (metres)\n" % seq)
    print("  %-22s %7s %7s %8s %8s %8s %7s" % ("run", "median", "p75", "p90", "p95", "RMS", ">50m"))
    for a in args:
        report(a, gt, os.path.basename(a.rstrip("/")))
    print("\n  Median is the filter's real accuracy. >50m is the share of the run spent lost.")
    print("  A long rejection cascade means GPS was present and being refused.\n")

main()
