"""Heading error of a fused trajectory against heading truth derived from positions.

    python3 tools/heading_error.py <ground_truth.tum> <fused.tum> [--from T] [--label L]

NCLT ground-truth ORIENTATION is identity and unusable, but its POSITIONS give course
over ground, which on a Segway is heading. tools/truth_heading.py builds that series
and cross-checks it against the IMU's own attitude; on 2012-08-20 and 2012-06-15 it
fits one constant offset to about 5 deg median, so it is good to a few degrees.

The fused trajectory lives in its own ENU frame, so its yaw is rotated by the SAME
Umeyama angle that aligns its positions to ground truth, and the residual constant
offset between the two heading conventions is removed as a median. What is left is the
part that varies, which is the only part a filter can be right or wrong about.

--from T scores only from timestamp T onward, for asking whether a change that took
effect at a known moment moved heading toward truth or away from it.
"""
import sys, math, bisect, argparse, importlib.util, os

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, name + ".py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("gt"); ap.add_argument("fused")
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    th_mod = _load("truth_heading")
    bb = _load("blackout_budget")
    gd = _load("gap_deadreckon")

    gt_xy = th_mod.load_xy(a.gt)
    truth = th_mod.truth_heading(gt_xy)
    if not truth:
        sys.exit("no heading truth")

    gt = bb.load_tum(a.gt)
    raw = bb.load_tum(a.fused)
    theta = gd.align_theta(raw, gt)
    fyaw = gd.load_fused_yaw(a.fused, theta)
    fts = [t for t, _ in fyaw]

    pairs = []
    for t, yaw, _ in truth:
        if a.t_from is not None and t < a.t_from:
            continue
        i = bisect.bisect_left(fts, t)
        if i == 0 or i >= len(fyaw):
            continue
        i = i if abs(fts[i] - t) < abs(fts[i - 1] - t) else i - 1
        if abs(fts[i] - t) > 0.2:
            continue
        pairs.append(wrap(fyaw[i][1] - yaw))
    if not pairs:
        sys.exit("no overlap")

    # One constant convention offset, removed as a median. Only the varying part is
    # the filter's own heading error.
    srt = sorted(pairs)
    off = srt[len(srt) // 2]
    err = sorted(abs(math.degrees(wrap(p - off))) for p in pairs)
    n = len(err)
    rms = math.sqrt(sum(e * e for e in err) / n)
    print("  %-28s n=%5d  median %6.2f deg  p90 %7.2f deg  RMS %7.2f deg"
          % (a.label or os.path.basename(a.fused), n,
             err[n // 2], err[int(0.9 * n)], rms))


if __name__ == "__main__":
    main()
