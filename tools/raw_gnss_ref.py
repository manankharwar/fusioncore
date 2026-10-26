"""Raw GNSS median error against ground truth.

    python3 tools/raw_gnss_ref.py <exported.txt> <ground_truth.tum>

Computed with blackout_budget's OWN alignment and error metric, so the number it
prints is a valid --ref for that tool. Without a pinned --ref the budget tool falls
back to the fused steady median, which is the thing under test and changes between
arms, so window shares stop being comparable across arms.

Scored at FIX TIMES. Not comparable to a full-trajectory fused RMSE.
"""
import sys, os, importlib.util

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "bb", os.path.join(HERE, "blackout_budget.py"))
bb = importlib.util.module_from_spec(spec); spec.loader.exec_module(bb)

if len(sys.argv) < 3:
    sys.exit(__doc__)
exported, gtp = sys.argv[1], sys.argv[2]
fixes = []
zeros = 0
for ln in open(exported):
    if not ln.startswith("GNSS "):
        continue
    p = ln.split()
    t, x, y, z = float(p[1]), float(p[2]), float(p[3]), float(p[4])
    if x == 0.0 and y == 0.0 and z == 0.0:
        zeros += 1
        continue
    fixes.append((t, x, y, z))
gt = bb.load_tum(gtp)
al = bb.umeyama_xy(fixes, gt)
err = bb.align(al, gt)
e = sorted(v for _, v in err)
med = e[len(e)//2]
rms = (sum(v*v for v in e)/len(e))**0.5
print("%d fixes (%d all-zero dropped)  raw GNSS median %.3f  RMSE %.3f" % (len(fixes), zeros, med, rms))
print("REF=%.6f" % med)
