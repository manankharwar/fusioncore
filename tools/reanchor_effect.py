"""Did each re-anchor help or hurt?

    python3 tools/reanchor_effect.py <replay.log> <fused.tum> <ground_truth.tum>

For each REANCHOR line the replay emitted, the mean fused error in the 10 s before
the event against the 2-12 s after, in the aligned frame blackout_budget uses. A
re-anchor that raises the error is harm.

Events with no ground truth in either window are reported as "n/a" in their own
column, NOT folded into either side. An earlier version counted them as "better"
because `nan > nan` is false, which is the same class of labelling error as the
seven logged in benchmark_regression.md on 2026-10-17.
"""
import sys, os, importlib.util, bisect

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "bb", os.path.join(HERE, "blackout_budget.py"))
bb = importlib.util.module_from_spec(spec); spec.loader.exec_module(bb)

if len(sys.argv) < 4:
    sys.exit(__doc__)
log, tum, gtp = sys.argv[1], sys.argv[2], sys.argv[3]
ev = []
for ln in open(log):
    if ln.startswith("REANCHOR "):
        p = ln.split(); ev.append((float(p[1]), float(p[2])))
gt = bb.load_tum(gtp)
fused = bb.umeyama_xy(bb.load_tum(tum), gt)
err = bb.align(fused, gt)
ts = [t for t, _ in err]; es = [e for _, e in err]
if not ev:
    print("  no events"); raise SystemExit

def near(t, lo, hi):
    i = bisect.bisect_left(ts, t + lo); j = bisect.bisect_right(ts, t + hi)
    w = es[i:j]
    return sum(w) / len(w) if w else float("nan")

worse = 0; na = 0; ok = 0
print("    %12s %9s %9s %9s %7s" % ("t_rel", "jump_m", "err_before", "err_after", "verdict"))
for t, j in ev:
    b = near(t, -10.0, -0.5); a = near(t, 2.0, 12.0)
    if b != b or a != a:
        v = "n/a"; na += 1          # no ground truth within the window: unknown,
    elif a > b:                      # NOT "better"
        v = "WORSE"; worse += 1
    else:
        v = "better"; ok += 1
    print("    %12.1f %9.2f %9.2f %9.2f %7s" % (t - ts[0], j, b, a, v))
print("  %d better, %d WORSE, %d unscorable (no truth nearby), %d total"
      % (ok, worse, na, len(ev)))
