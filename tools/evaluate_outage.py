#!/usr/bin/env python3
"""
GPS outage / dead reckoning drift evaluator.

Measures how far each filter drifts from ground truth at the moment GPS
resumes after a simulated outage. This tests dead-reckoning quality:
FusionCore benefits from IMU bias estimation; RL filters do not.

Usage:
  python3 tools/evaluate_outage.py \
    --gt          benchmarks/nclt/2012-01-08/ground_truth.tum \
    --fusioncore  benchmarks/nclt/2012-01-08/fusioncore_outage.tum \
    --rl_ekf      benchmarks/nclt/2012-01-08/rl_ekf_outage.tum \
    --rl_ukf      benchmarks/nclt/2012-01-08/rl_ukf_outage.tum \
    --outage_start 120.0 \
    --outage_duration 45.0 \
    --out_dir     benchmarks/nclt/2012-01-08/results
"""

import argparse
import os
import sys
from pathlib import Path


def load_tum(path: str) -> list:
    """Load TUM file → sorted list of (timestamp, x, y, z)."""
    poses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            poses.append((ts, x, y, z))
    poses.sort(key=lambda p: p[0])
    return poses


def interpolate_at(poses: list, t: float):
    """Linear interpolation of (x,y,z) at time t."""
    if not poses:
        return None
    if t <= poses[0][0]:
        return poses[0][1:]
    if t >= poses[-1][0]:
        return poses[-1][1:]
    for i in range(1, len(poses)):
        if poses[i][0] >= t:
            t0, x0, y0, z0 = poses[i-1]
            t1, x1, y1, z1 = poses[i]
            alpha = (t - t0) / (t1 - t0)
            return (x0 + alpha*(x1-x0),
                    y0 + alpha*(y1-y0),
                    z0 + alpha*(z1-z0))
    return None


def dist3d(a, b) -> float:
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2) ** 0.5


def error_at(poses: list, gt: list, t: float) -> float:
    """3D error at time t."""
    est = interpolate_at(poses, t)
    gnd = interpolate_at(gt, t)
    if est is None or gnd is None:
        return float('nan')
    return dist3d(est, gnd)


def mean_error_in_window(poses: list, gt: list, t_start: float, t_end: float) -> float:
    errors = []
    for ts, x, y, z in poses:
        if ts < t_start or ts > t_end:
            continue
        gnd = interpolate_at(gt, ts)
        if gnd is None:
            continue
        errors.append(dist3d((x, y, z), gnd))
    return sum(errors) / len(errors) if errors else float('nan')


def max_error_in_window(poses: list, gt: list, t_start: float, t_end: float) -> float:
    max_err = 0.0
    for ts, x, y, z in poses:
        if ts < t_start or ts > t_end:
            continue
        gnd = interpolate_at(gt, ts)
        if gnd is None:
            continue
        max_err = max(max_err, dist3d((x, y, z), gnd))
    return max_err


def write_markdown(results: dict, outage_start: float, outage_dur: float, out_dir: str):
    md_path = os.path.join(out_dir, 'OUTAGE_TEST.md')
    with open(md_path, 'w') as f:
        f.write('# GPS Outage / Dead Reckoning Test\n\n')
        f.write(f'GPS cut from t={outage_start:.1f}s to t={outage_start+outage_dur:.1f}s '
                f'({outage_dur:.0f}s outage).\n\n')
        f.write('## Results\n\n')
        f.write('| Filter | Error at outage start (m) | Drift at GPS return (m) | Max drift during outage (m) |\n')
        f.write('|--------|--------------------------|-------------------------|-----------------------------|\n')
        for name, r in results.items():
            f.write(f'| {name} | {r["err_at_start"]:.2f} | {r["err_at_return"]:.2f} | {r["max_during"]:.2f} |\n')
        f.write('\n## What This Means\n\n')
        f.write('- **Drift at GPS return** = how far the filter is from ground truth at the moment GPS comes back.\n')
        f.write('  This is the dead-reckoning error accumulated over the outage.\n')
        f.write('- FusionCore\'s IMU bias estimation keeps gyro/accel bias in the state vector,\n')
        f.write('  so heading drift is corrected continuously — reducing dead-reckoning error.\n')
        f.write('- RL filters have no bias estimation: heading error grows as `bias × time`,\n')
        f.write('  which compounds into position error via `sin(heading_error) × distance_travelled`.\n\n')
        f.write('## Methodology\n\n')
        f.write(f'- Outage duration: {outage_dur:.0f}s\n')
        f.write('- All filters run on IMU + wheel odometry only during outage\n')
        f.write('- Comparison: 3D Euclidean error vs RTK GPS ground truth\n')
    print(f'  Results written to {md_path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--gt',               required=True)
    parser.add_argument('--fusioncore',       required=True)
    parser.add_argument('--rl_ekf',           required=True)
    parser.add_argument('--rl_ukf',           required=True)
    parser.add_argument('--outage_start',     type=float, required=True,
                        help='Sim-time seconds when GPS outage started')
    parser.add_argument('--outage_duration',  type=float, default=45.0,
                        help='Outage duration in seconds')
    parser.add_argument('--out_dir',          default='./benchmarks/nclt/2012-01-08/results')
    args = parser.parse_args()

    for path, name in [(args.gt, 'ground truth'), (args.fusioncore, 'FusionCore'),
                       (args.rl_ekf, 'RL-EKF'), (args.rl_ukf, 'RL-UKF')]:
        if not os.path.exists(path):
            print(f'Error: {name} file not found: {path}', file=sys.stderr)
            sys.exit(1)

    Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    gt = load_tum(args.gt)
    t0 = gt[0][0] if gt else 0.0

    outage_abs_start  = t0 + args.outage_start
    outage_abs_end    = outage_abs_start + args.outage_duration

    print(f'\n{"="*60}')
    print(f'  GPS Outage / Dead Reckoning Test')
    print(f'  Outage: {args.outage_duration:.0f}s  '
          f'[t={args.outage_start:.1f}s → t={args.outage_start+args.outage_duration:.1f}s]')
    print(f'{"="*60}\n')

    traj = {
        'FusionCore': load_tum(args.fusioncore),
        'RL-EKF':     load_tum(args.rl_ekf),
        'RL-UKF':     load_tum(args.rl_ukf),
    }

    results = {}
    for name, poses in traj.items():
        err_start  = error_at(poses, gt, outage_abs_start)
        err_return = error_at(poses, gt, outage_abs_end)
        max_during = max_error_in_window(poses, gt, outage_abs_start, outage_abs_end)
        drift      = err_return - err_start   # net new error accumulated

        results[name] = {
            'err_at_start':  err_start,
            'err_at_return': err_return,
            'max_during':    max_during,
            'drift':         drift,
        }
        print(f'  {name:15s}  '
              f'at_start={err_start:.2f}m  '
              f'at_return={err_return:.2f}m  '
              f'max_during={max_during:.2f}m  '
              f'net_drift={drift:+.2f}m')

    write_markdown(results, args.outage_start, args.outage_duration, args.out_dir)

    print(f'\n── Summary ──────────────────────────────────────────────')
    fc_drift  = results['FusionCore']['drift']
    ekf_drift = results['RL-EKF']['drift']
    ukf_drift = results['RL-UKF']['drift']
    best_rl   = min(ekf_drift, ukf_drift)
    if fc_drift < best_rl:
        improvement = (best_rl - fc_drift) / best_rl * 100 if best_rl > 0 else 0
        print(f'  FusionCore dead-reckoning drift {fc_drift:.2f}m vs '
              f'best-RL {best_rl:.2f}m  ({improvement:.1f}% less drift)')
    else:
        print(f'  RL wins dead-reckoning: FC {fc_drift:.2f}m vs best-RL {best_rl:.2f}m')
    print(f'\nResults in {args.out_dir}/OUTAGE_TEST.md')


if __name__ == '__main__':
    main()
