#!/usr/bin/env python3
"""
GPS spike rejection test evaluator.

Measures the maximum position deviation each filter experiences after a
GPS spike is injected. FusionCore should reject the spike via Mahalanobis
gating and stay put; RL filters will jump toward the fake position.

Usage:
  python3 tools/evaluate_spike.py \
    --gt          benchmarks/nclt/2012-01-08/ground_truth.tum \
    --fusioncore  benchmarks/nclt/2012-01-08/fusioncore_spike.tum \
    --rl_ekf      benchmarks/nclt/2012-01-08/rl_ekf_spike.tum \
    --rl_ukf      benchmarks/nclt/2012-01-08/rl_ukf_spike.tum \
    --spike_time  120.0 \
    --window      30.0 \
    --out_dir     benchmarks/nclt/2012-01-08/results
"""

import argparse
import os
import sys
from pathlib import Path


def load_tum(path: str) -> list:
    """Load TUM file → list of (timestamp, x, y, z)."""
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


def max_deviation_in_window(est_poses: list, gt_poses: list,
                             t_start: float, t_end: float) -> tuple:
    """
    Max 3D error between estimated and ground truth within [t_start, t_end].
    Returns (max_error_m, time_of_max).
    """
    max_err = 0.0
    max_t   = t_start
    for ts, x, y, z in est_poses:
        if ts < t_start or ts > t_end:
            continue
        gt_xyz = interpolate_at(gt_poses, ts)
        if gt_xyz is None:
            continue
        err = dist3d((x, y, z), gt_xyz)
        if err > max_err:
            max_err = err
            max_t   = ts
    return max_err, max_t


def steady_state_error(est_poses: list, gt_poses: list,
                        t_start: float, t_end: float) -> float:
    """Mean 3D error in window (pre-spike baseline)."""
    errors = []
    for ts, x, y, z in est_poses:
        if ts < t_start or ts > t_end:
            continue
        gt_xyz = interpolate_at(gt_poses, ts)
        if gt_xyz is None:
            continue
        errors.append(dist3d((x, y, z), gt_xyz))
    return sum(errors) / len(errors) if errors else float('nan')


def write_markdown(results: dict, spike_time: float, magnitude: float,
                   window: float, out_dir: str):
    md_path = os.path.join(out_dir, 'SPIKE_TEST.md')
    with open(md_path, 'w') as f:
        f.write('# GPS Spike Rejection Test\n\n')
        f.write(f'Spike injected at t={spike_time:.1f}s, magnitude={magnitude:.0f}m.\n')
        f.write(f'Measurement window: [{spike_time:.1f}s, {spike_time+window:.1f}s].\n\n')
        f.write('## Results\n\n')
        f.write('| Filter | Baseline error (m) | Max deviation after spike (m) | Rejected? |\n')
        f.write('|--------|--------------------|-------------------------------|-----------|\n')
        for name, r in results.items():
            rejected = 'YES' if r['max_dev'] < r['baseline'] * 3 else 'NO: jumped'
            f.write(f'| {name} | {r["baseline"]:.2f} | {r["max_dev"]:.2f} | {rejected} |\n')
        f.write('\n## Methodology\n\n')
        f.write('- One GPS fix is corrupted to lat+500m/lon+500m (~707m NE).\n')
        f.write('- Baseline: mean position error in 30s before spike.\n')
        f.write('- Max deviation: peak error in 30s after spike.\n')
        f.write('- Rejection criterion: max_dev < 3× baseline.\n')
    print(f'  Results written to {md_path}')
    return md_path


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--gt',          required=True)
    parser.add_argument('--fusioncore',  required=True)
    parser.add_argument('--rl_ekf',      required=True)
    parser.add_argument('--rl_ukf',      required=True)
    parser.add_argument('--spike_time',  type=float, required=True,
                        help='Sim-time seconds when spike was injected')
    parser.add_argument('--spike_magnitude_m', type=float, default=500.0)
    parser.add_argument('--window',      type=float, default=30.0,
                        help='Seconds after spike to measure max deviation')
    parser.add_argument('--out_dir',     default='./benchmarks/nclt/2012-01-08/results')
    args = parser.parse_args()

    for path, name in [(args.gt, 'ground truth'), (args.fusioncore, 'FusionCore'),
                       (args.rl_ekf, 'RL-EKF'), (args.rl_ukf, 'RL-UKF')]:
        if not os.path.exists(path):
            print(f'Error: {name} file not found: {path}', file=sys.stderr)
            sys.exit(1)

    Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    print(f'\n{"="*60}')
    print(f'  GPS Spike Rejection Test')
    print(f'  Spike: {args.spike_magnitude_m:.0f}m at t={args.spike_time:.1f}s')
    print(f'{"="*60}\n')

    gt   = load_tum(args.gt)
    traj = {
        'FusionCore': load_tum(args.fusioncore),
        'RL-EKF':     load_tum(args.rl_ekf),
        'RL-UKF':     load_tum(args.rl_ukf),
    }

    # Determine time origin from ground truth (first timestamp)
    t0_gt = gt[0][0] if gt else 0.0

    # Convert spike_time (sim elapsed seconds) to absolute TUM timestamp
    spike_abs = t0_gt + args.spike_time
    baseline_start = spike_abs - args.window
    baseline_end   = spike_abs
    measure_start  = spike_abs
    measure_end    = spike_abs + args.window

    results = {}
    for name, poses in traj.items():
        baseline = steady_state_error(poses, gt, baseline_start, baseline_end)
        max_dev, max_t = max_deviation_in_window(poses, gt, measure_start, measure_end)
        results[name] = {'baseline': baseline, 'max_dev': max_dev, 'max_t': max_t}
        rejected = max_dev < baseline * 3
        status = 'REJECTED spike' if rejected else f'JUMPED {max_dev:.1f}m'
        print(f'  {name:15s}  baseline={baseline:.2f}m  '
              f'max_dev={max_dev:.2f}m  → {status}')

    write_markdown(results, args.spike_time, args.spike_magnitude_m,
                   args.window, args.out_dir)
    print(f'\nDone. Results in {args.out_dir}/SPIKE_TEST.md')


if __name__ == '__main__':
    main()
