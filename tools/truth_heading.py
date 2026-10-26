"""Heading truth for NCLT, derived from ground-truth POSITIONS.

    python3 tools/truth_heading.py <ground_truth.tum> [exported.txt]

NCLT's ground_truth.tum carries an exactly identity quaternion on every row, so the
dataset appears to have no heading. It does: the positions are real, and a Segway does
not slide sideways, so course over ground IS heading whenever the robot is moving and
not turning hard.

Guards, because a naive atan2 between consecutive samples is mostly noise:

  baseline   the chord is taken over at least MIN_BASELINE_M of travel, not between
             adjacent samples, so receiver noise divides by the baseline
  speed      at least MIN_SPEED_M_S over that chord, or heading is undefined
  turn       the heading must change by less than MAX_TURN_DEG across the chord, or
             the chord is not the tangent
  sanity     a chord implying more than MAX_SPEED_M_S is a ground-truth glitch, and
             NCLT has those: ~12% of its samples imply over 5 m/s on a 1.5 m/s robot

With the exported log given as a second argument, the series is cross-checked against
the IMU's own absolute attitude (IMUQ), which is an INDEPENDENT source. They should
agree up to one constant offset. The spread of that offset is the validation: if
course-over-ground were noise, no constant would fit it.
"""
import sys, math, bisect

MIN_BASELINE_M = 3.0
MIN_SPEED_M_S  = 0.5
MAX_TURN_DEG   = 10.0
MAX_SPEED_M_S  = 3.0


def load_xy(path):
    out = []
    for ln in open(path):
        if ln.startswith("#"):
            continue
        p = ln.split()
        if len(p) < 4:
            continue
        out.append((float(p[0]), float(p[1]), float(p[2])))
    out.sort()
    return out


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def truth_heading(gt):
    """[(t, yaw, speed)] at the chord midpoint, for chords that pass every guard."""
    out = []
    j = 0
    for i in range(len(gt)):
        # grow the chord from i until it spans MIN_BASELINE_M
        j = max(j, i + 1)
        while j < len(gt):
            d = math.hypot(gt[j][1] - gt[i][1], gt[j][2] - gt[i][2])
            if d >= MIN_BASELINE_M:
                break
            j += 1
        if j >= len(gt):
            break
        t0, x0, y0 = gt[i]
        t1, x1, y1 = gt[j]
        dt = t1 - t0
        if dt <= 0:
            continue
        d = math.hypot(x1 - x0, y1 - y0)
        speed = d / dt
        if speed < MIN_SPEED_M_S or speed > MAX_SPEED_M_S:
            continue
        yaw = math.atan2(y1 - y0, x1 - x0)
        # turn across the chord: compare its two halves
        m = (i + j) // 2
        if m <= i or m >= j:
            continue
        ya = math.atan2(gt[m][2] - y0, gt[m][1] - x0)
        yb = math.atan2(y1 - gt[m][2], x1 - gt[m][1])
        if abs(math.degrees(wrap(yb - ya))) > MAX_TURN_DEG:
            continue
        out.append((0.5 * (t0 + t1), yaw, speed))
    return out


def imuq_yaw(path):
    out = []
    for ln in open(path):
        if not ln.startswith("IMUQ "):
            continue
        p = ln.split()
        qx, qy, qz, qw = (float(v) for v in p[2:6])
        out.append((float(p[1]),
                    math.atan2(2.0 * (qw * qz + qx * qy),
                               1.0 - 2.0 * (qy * qy + qz * qz))))
    out.sort()
    return out


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    gt = load_xy(sys.argv[1])
    th = truth_heading(gt)
    if not th:
        sys.exit("no usable chords")
    span = gt[-1][0] - gt[0][0]
    cov = (th[-1][0] - th[0][0]) / span if span else 0.0
    print("  ground truth rows %d over %.0f s" % (len(gt), span))
    print("  heading samples   %d, spanning %.0f%% of the run, %.1f%% of rows"
          % (len(th), 100.0 * cov, 100.0 * len(th) / len(gt)))
    print("  speed over the usable chords: %.2f to %.2f m/s"
          % (min(s for _, _, s in th), max(s for _, _, s in th)))

    if len(sys.argv) > 2:
        iq = imuq_yaw(sys.argv[2])
        if iq:
            its = [t for t, _ in iq]
            offs = []
            for t, yaw, _ in th:
                k = bisect.bisect_left(its, t)
                if k == 0 or k >= len(iq):
                    continue
                k = k if abs(its[k] - t) < abs(its[k - 1] - t) else k - 1
                if abs(its[k] - t) > 0.5:
                    continue
                offs.append(math.degrees(wrap(yaw - iq[k][1])))
            if offs:
                offs.sort()
                med = offs[len(offs) // 2]
                dev = sorted(abs(wrap(math.radians(o - med))) for o in offs)
                p50 = math.degrees(dev[len(dev) // 2])
                p90 = math.degrees(dev[int(0.9 * len(dev))])
                print("  cross-check against the IMU's own attitude, %d pairs:" % len(offs))
                print("    constant offset  %.2f deg (median)" % med)
                print("    scatter about it %.2f deg median, %.2f deg 90th pct"
                      % (p50, p90))
                print("    a course-over-ground series that was noise would not fit "
                      "one constant")
    for t, yaw, _ in th[:0]:
        pass


if __name__ == "__main__":
    main()
