#!/usr/bin/env python3
"""What do the yaw-rate and speed sensors do inside each GNSS gap?

    python3 tools/gap_sensors.py <exported.txt> <ground_truth.tum> [--gap-s 10]

Written for 2013-04-05, where gaps hold 1.8% of the time and 60.3% of the squared error.
For every gap of >= --gap-s seconds (from the RAW fix stream), it separates three
explanations that look the same in the error budget:

  gyro bias drift   gyro and encoder yaw rate agree in shape (high correlation) but not
                    in mean: a constant offset between them, growing heading disagreement
  slip              wheel distance exceeds truth distance, or the encoder yaw rate
                    disagrees with the gyro in shape (low correlation) during turns
  something else    the sensors agree with each other AND with truth, so the error is
                    in what the filter does with them

Heading truth comes from ground-truth POSITIONS via truth_heading.py (NCLT's truth
quaternions are identity), so the heading columns exist only where truth does. The
convention offset between truth heading and sensor heading cancels because only
CHANGES of heading are compared.
"""
import argparse
import math
import os
import importlib.util

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("th", os.path.join(HERE, "truth_heading.py"))
th = importlib.util.module_from_spec(spec); spec.loader.exec_module(th)


def load(exported):
    fixes, imu, enc = [], [], []
    for ln in open(exported):
        p = ln.split()
        if not p:
            continue
        if p[0] == "GNSS":
            fixes.append(float(p[1]))
        elif p[0] == "IMU":
            imu.append((float(p[1]), float(p[4])))
        elif p[0] == "ENC":
            enc.append((float(p[1]), float(p[2]), float(p[4])))
    return np.array(sorted(fixes)), np.array(imu), np.array(enc)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("exported"); ap.add_argument("gt")
    ap.add_argument("--gap-s", type=float, default=10.0)
    a = ap.parse_args(argv)

    fixes, imu, enc = load(a.exported)
    gt = th.load_xy(a.gt)
    gt_t = np.array([r[0] for r in gt]); gt_xy = np.array([[r[1], r[2]] for r in gt])
    hd = th.truth_heading(gt)                      # [(t, yaw), ...]
    hd_t = np.array([h[0] for h in hd]); hd_y = np.array([h[1] for h in hd])

    dg = np.diff(fixes)
    gaps = [(fixes[i], fixes[i + 1]) for i in np.where(dg >= a.gap_s)[0]]
    print(f"  {len(gaps)} gaps >= {a.gap_s:.0f} s\n")
    hdr = (f"  {'start':>8s} {'len s':>6s} {'wheel m':>8s} {'truth m':>8s} {'gt cov':>6s}"
           f" {'gyro dpsi':>9s} {'enc dpsi':>9s} {'true dpsi':>9s} {'corr':>6s}"
           f" {'mean(g-e)':>10s} {'turn s':>6s}")
    print(hdr)
    t_first = fixes[0]
    for g0, g1 in gaps:
        mi = (imu[:, 0] > g0) & (imu[:, 0] < g1)
        me = (enc[:, 0] > g0) & (enc[:, 0] < g1)
        gi, ge = imu[mi], enc[me]
        if len(gi) < 10 or len(ge) < 10:
            continue
        gyro_dpsi = math.degrees(np.trapz(gi[:, 1], gi[:, 0]))
        enc_dpsi = math.degrees(np.trapz(ge[:, 2], ge[:, 0]))
        wheel = float(np.sum(np.abs(ge[1:, 1]) * np.diff(ge[:, 0])))
        e_on_g = np.interp(gi[:, 0], ge[:, 0], ge[:, 2])
        corr = float(np.corrcoef(gi[:, 1], e_on_g)[0, 1]) if np.std(e_on_g) > 0 else float("nan")
        bias = float(np.mean(gi[:, 1] - e_on_g))
        turning = float(np.sum(np.abs(gi[:, 1]) > 0.2) * np.median(np.diff(gi[:, 0])))
        mg = (gt_t > g0) & (gt_t < g1)
        cov = 0.0
        tdist = float("nan")
        if mg.sum() > 1:
            seg = gt_xy[mg]; segt = gt_t[mg]
            steps = np.linalg.norm(np.diff(seg, axis=0), axis=1)
            dt = np.diff(segt)
            ok = (dt > 0) & (steps / np.maximum(dt, 1e-9) < th.MAX_SPEED_M_S) & (dt < 5.0)
            tdist = float(steps[ok].sum())
            cov = float(dt[ok].sum() / (g1 - g0))
        mh = (hd_t > g0) & (hd_t < g1)
        true_dpsi = float("nan")
        if mh.sum() > 1:
            y = np.unwrap(hd_y[mh])
            true_dpsi = math.degrees(y[-1] - y[0])
            # The sensors over the SAME truth-covered sub-span, for a like-for-like change.
            s0, s1 = hd_t[mh][0], hd_t[mh][-1]
            sg = (gi[:, 0] >= s0) & (gi[:, 0] <= s1)
            se = (ge[:, 0] >= s0) & (ge[:, 0] <= s1)
            sub_g = math.degrees(np.trapz(gi[sg, 1], gi[sg, 0])) if sg.sum() > 1 else float("nan")
            sub_e = math.degrees(np.trapz(ge[se, 2], ge[se, 0])) if se.sum() > 1 else float("nan")
        print(f"  {g0 - t_first:8.0f} {g1 - g0:6.0f} {wheel:8.1f} {tdist:8.1f} {100*cov:5.0f}%"
              f" {gyro_dpsi:9.1f} {enc_dpsi:9.1f} {true_dpsi:9.1f} {corr:6.3f}"
              f" {bias:10.5f} {turning:6.1f}")
        if mh.sum() > 1:
            print(f"  {'':8s} over the {s1 - s0:.0f} s with heading truth: gyro {sub_g:+.1f}, "
                  f"encoder {sub_e:+.1f}, truth {true_dpsi:+.1f} deg "
                  f"({mh.sum()} truth heading samples)")
    print("\n  dpsi = integrated heading change over the gap, degrees. corr = gyro vs encoder"
          " yaw rate.\n  mean(g-e) = mean gyro minus encoder yaw rate, rad/s (a bias"
          " signature). turn s = seconds above 0.2 rad/s.\n  truth m sums ground-truth"
          " steps that pass truth_heading's speed sanity check; gt cov is the share of the"
          " gap\n  those steps span.")


if __name__ == "__main__":
    main()
