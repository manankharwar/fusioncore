"""Inside a GNSS blackout, does the filter's coast beat plain dead reckoning?

    python3 tools/gap_deadreckon.py <exported.txt> <fused.tum> <ground_truth.tum>
                                    [--gap-s 60] [--min-dist 20]

84.6% of 2012-06-15's error lives in a single 462 s blackout, and re-anchoring
cannot reach it because it acts only after a gap ends. So the question is whether
the filter is even earning its keep in there, against baselines a rover could run
with no filter at all.

Every variant starts from the GROUND TRUTH pose at the gap's first sample, so this
measures drift accumulated DURING the gap and not error inherited from before it.
Error is reported as a percentage of distance travelled, so a 20 s gap and a 400 s
gap are comparable.

Every variant is anchored at the FILTER's heading at window entry. There is no
truth-heading variant, and that is not an omission: NCLT's ground_truth.tum carries
an EXACTLY IDENTITY quaternion on every row, so it has no heading to anchor to. An
earlier version of this tool read yaw out of those quaternions and therefore anchored
dead reckoning at 0 rad rather than at the truth heading. Those rows were published
on 2026-10-18 and are withdrawn; the tool now refuses a ground truth whose
orientation is identity rather than quietly scoring against a constant.

Variants:
  filter      the fused trajectory's own displacement over the gap
  gyro        encoder body speed, heading integrated from gyro Z
  enc         encoder body speed, heading integrated from encoder WZ
  enc_neg     the same with WZ negated, because the NCLT wheel yaw rate was never
              converted to ENU and has fought the gyro on every published run

Scored at gap exit. Provenance: pipeline replay, points `gaps`.
"""
import sys, os, math, bisect, argparse, importlib.util

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "bb", os.path.join(HERE, "blackout_budget.py"))
bb = importlib.util.module_from_spec(spec); spec.loader.exec_module(bb)


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def load_truth_pose(path):
    """t, x, y, yaw from a TUM file."""
    out = []
    for ln in open(path):
        if ln.startswith("#"):
            continue
        p = ln.split()
        if len(p) < 8:
            continue
        out.append((float(p[0]), float(p[1]), float(p[2]),
                    yaw_of(float(p[4]), float(p[5]), float(p[6]), float(p[7]))))
    out.sort()
    return out


def read_streams(path):
    enc, gyro, imuq, fixes = [], [], [], []
    for ln in open(path):
        p = ln.split()
        if not p:
            continue
        k = p[0]
        if k == "ENC":
            enc.append((float(p[1]), float(p[2]), float(p[3]), float(p[4])))
        elif k == "IMU":
            gyro.append((float(p[1]), float(p[4])))
        elif k == "IMUQ":
            imuq.append((float(p[1]), yaw_of(float(p[2]), float(p[3]),
                                             float(p[4]), float(p[5]))))
        elif k == "GNSS":
            fixes.append(float(p[1]))
    return enc, gyro, imuq, sorted(fixes)


def at(series, t):
    """Nearest sample in a sorted (t, value) series."""
    ts = [s[0] for s in series]
    i = bisect.bisect_left(ts, t)
    if i == 0:
        return series[0]
    if i >= len(series):
        return series[-1]
    return series[i] if abs(ts[i] - t) < abs(ts[i - 1] - t) else series[i - 1]


def dr(enc, t0, t1, yaw0, yaw_src, wz_sign=0.0, pin=0.0):
    """Integrate encoder body velocity over [t0, t1].

    yaw_src is a sorted (t, yaw) series to read absolute heading from, or None to
    integrate yaw from wz_sign * encoder WZ.
    """
    x = y = 0.0
    yaw = yaw0
    dist = 0.0
    prev_t = None
    yts = [s[0] for s in yaw_src] if yaw_src else None
    for t, vx, vy, wz in enc:
        if t < t0 or t > t1:
            continue
        if prev_t is None:
            prev_t = t
            continue
        dt = t - prev_t
        prev_t = t
        if dt <= 0 or dt > 1.0:
            continue
        if yaw_src:
            i = bisect.bisect_left(yts, t)
            i = min(max(i, 0), len(yaw_src) - 1)
            yaw = yaw_src[i][1] + pin
        else:
            yaw += wz_sign * wz * dt
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        dist += math.hypot(vx, vy) * dt
    return x, y, dist


def align_theta(traj, gt):
    """The rotation blackout_budget's umeyama_xy applies, so fused YAW can be put
    into the same frame as the fused positions."""
    gts = [p[0] for p in gt]
    pairs = []
    for t, x, y, z in traj:
        i = bisect.bisect_left(gts, t)
        if i == 0 or i >= len(gt):
            continue
        j = i if abs(gts[i] - t) < abs(gts[i - 1] - t) else i - 1
        if abs(gts[j] - t) > 0.05:
            continue
        pairs.append(((x, y, z), (gt[j][1], gt[j][2], gt[j][3])))
    if len(pairs) < 2:
        return 0.0
    n = len(pairs)
    mx = [sum(p[0][k] for p in pairs) / n for k in range(3)]
    my = [sum(p[1][k] for p in pairs) / n for k in range(3)]
    sxx = sxy = syx = syy = 0.0
    for aa, bb_ in pairs:
        ax, ay = aa[0] - mx[0], aa[1] - mx[1]
        bx, by = bb_[0] - my[0], bb_[1] - my[1]
        sxx += ax * bx; sxy += ax * by; syx += ay * bx; syy += ay * by
    return math.atan2(sxy - syx, sxx + syy)


def load_fused_yaw(path, theta):
    """(t, yaw) from the fused TUM, rotated into the truth frame by theta."""
    out = []
    for ln in open(path):
        if ln.startswith("#"):
            continue
        p = ln.split()
        if len(p) < 8:
            continue
        out.append((float(p[0]),
                    yaw_of(float(p[4]), float(p[5]), float(p[6]), float(p[7]))
                    + theta))
    out.sort()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("exported"); ap.add_argument("fused"); ap.add_argument("gt")
    ap.add_argument("--gap-s", type=float, default=60.0)
    ap.add_argument("--min-dist", type=float, default=20.0)
    a = ap.parse_args()

    enc, gyro, imuq, fixes = read_streams(a.exported)

    truth = load_truth_pose(a.gt)
    gt = bb.load_tum(a.gt)
    fused_raw = bb.load_tum(a.fused)
    fused = bb.umeyama_xy(fused_raw, gt)
    theta = align_theta(fused_raw, gt)
    fyaw = load_fused_yaw(a.fused, theta)
    tts = [p[0] for p in truth]
    fts = [p[0] for p in fused]

    gaps = [(x, y) for x, y in zip(fixes, fixes[1:]) if y - x >= a.gap_s]
    if not gaps:
        print("no gaps >= %.0f s" % a.gap_s)
        return

    print("  gaps >= %.0fs: %d" % (a.gap_s, len(gaps)))
    skipped = []
    scored = 0
    for t0, t1 in gaps:
        # NCLT ground truth is itself gappy and its holes tend to coincide with the
        # GNSS outages, so the scoring window is CLIPPED to the truth-covered span
        # inside the gap. Without this the 2012-06-15 "462 s gap" integrated dead
        # reckoning over 462 s against a truth displacement covering only its last
        # 276 s, because truth has a 185 s hole at the gap's start.
        i0 = bisect.bisect_left(tts, t0)
        i1 = bisect.bisect_right(tts, t1) - 1
        if i1 <= i0:
            skipped.append((t1 - t0, "no truth inside"))
            continue
        ta, tb = tts[i0], tts[i1]
        hole = max((tts[k + 1] - tts[k] for k in range(i0, i1)), default=0.0)
        p0, p1 = truth[i0], truth[i1]
        tdx, tdy = p1[1] - p0[1], p1[2] - p0[2]
        tdisp = math.hypot(tdx, tdy)
        if tb - ta < 20.0:
            skipped.append((t1 - t0, "truth span only %.0fs" % (tb - ta)))
            continue
        if hole > 30.0:
            skipped.append((t1 - t0, "%.0fs hole inside truth" % hole))
            continue
        if tdisp < 1.0:
            skipped.append((t1 - t0, "truth displacement %.1fm" % tdisp))
            continue

        fy0 = at(fyaw, ta)[1]
        filt_anchor = {
            "gyro": dr(enc, ta, tb, fy0, _integrate(gyro, ta, tb, fy0)),
            "enc": dr(enc, ta, tb, fy0, None, 1.0),
            "enc_neg": dr(enc, ta, tb, fy0, None, -1.0),
        }

        j0 = bisect.bisect_left(fts, ta); j1 = bisect.bisect_left(fts, tb)
        if j0 >= len(fused) or j1 >= len(fused):
            skipped.append((t1 - t0, "no fused samples"))
            continue
        f0, f1 = fused[j0], fused[j1]
        fdx, fdy = f1[1] - f0[1], f1[2] - f0[2]

        dist = filt_anchor["enc"][2]
        if dist < a.min_dist:
            skipped.append((t1 - t0, "only %.1fm travelled" % dist))
            continue

        ferr = math.hypot(fdx - tdx, fdy - tdy)
        scored += 1
        print("")
        print("    gap %.0fs, truth covers %.0fs of it (%.0f%%), %.1f m travelled,"
              % (t1 - t0, tb - ta, 100.0 * (tb - ta) / (t1 - t0), dist))
        print("    truth displacement %.1f m, largest truth hole inside %.1f s"
              % (tdisp, hole))
        print("      %-18s %9s %9s" % ("", "err_m", "%dist"))
        print("      %-18s %9.1f %9.1f" % ("filter coast", ferr, 100.0 * ferr / dist))
        for k in ("gyro", "enc", "enc_neg"):
            x, y, _ = filt_anchor[k]
            e = math.hypot(x - tdx, y - tdy)
            print("      %-18s %9.1f %9.1f" % (k + " @filter yaw", e, 100.0 * e / dist))

    print("")
    print("  %d gap(s) scored. Error is at the clipped window's exit." % scored)
    print("  dead reckoning is anchored at the filter's own heading at window entry,")
    print("  so it gets no information the filter does not already have.")
    if skipped:
        print("  skipped %d: %s"
              % (len(skipped), ", ".join("%.0fs (%s)" % x for x in skipped)))


def _integrate(gyro, t0, t1, yaw0):
    """Yaw series integrated from gyro Z, anchored at yaw0."""
    out = []
    yaw = yaw0
    prev = None
    for t, gz in gyro:
        if t < t0 or t > t1:
            continue
        if prev is None:
            prev = t
            out.append((t, yaw))
            continue
        dt = t - prev
        prev = t
        if 0 < dt <= 1.0:
            yaw += gz * dt
        out.append((t, yaw))
    return out


if __name__ == "__main__":
    main()
