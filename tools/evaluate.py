#!/usr/bin/env python3
"""
NCLT benchmark evaluator: FusionCore vs robot_localization.

Converts bags to TUM format, runs evo_ape + evo_rpe, and prints a
comparison table. Generates trajectory plots saved as PNG files.

Prerequisites:
  pip install evo

Usage:
  # Step 1: Convert NCLT ground truth to TUM
  python3 tools/nclt_rtk_to_tum.py \
    --rtk /path/to/nclt/2012-01-08/gps_rtk.csv \
    --out ground_truth.tum

  # Step 2: Extract filter outputs from results bag
  python3 tools/odom_to_tum.py --bag ./nclt_results --topic /fusion/odom  --out fusioncore.tum
  python3 tools/odom_to_tum.py --bag ./nclt_results --topic /rl/odometry  --out rl_ekf.tum

  # Step 3: Evaluate
  python3 tools/evaluate.py \
    --gt           ground_truth.tum \
    --fusioncore   fusioncore.tum \
    --rl           rl_ekf.tum \
    --sequence     2012-01-08 \
    --out_dir      ./results
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path


def run_evo(cmd: list, label: str) -> dict:
    """Run an evo CLI command and parse RMSE from its stdout."""
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f'[WARN] evo failed for {label}:')
        print(result.stderr[-500:])
        return {'rmse': float('nan'), 'mean': float('nan'), 'max': float('nan')}

    metrics = {}
    for line in result.stdout.splitlines():
        line = line.strip()
        for key in ('rmse', 'mean', 'max', 'min', 'median', 'std'):
            if line.lower().startswith(key):
                try:
                    metrics[key] = float(line.split()[-1])
                except ValueError:
                    pass
    return metrics


def evo_ape(gt: str, est: str, label: str, out_dir: str) -> dict:
    plot_path = os.path.join(out_dir, f'ate_{label}.png')
    cmd = [
        'evo_ape', 'tum', gt, est,
        '--align',
        '--t_max_diff', '0.1',
    ]
    m = run_evo(cmd, f'ATE {label}')
    print(f'  ATE {label:20s}  RMSE={m.get("rmse", float("nan")):.3f}m  '
          f'max={m.get("max", float("nan")):.3f}m')
    return m


def evo_rpe(gt: str, est: str, label: str, out_dir: str, delta: float = 10.0) -> dict:
    plot_path = os.path.join(out_dir, f'rpe_{label}.png')
    cmd = [
        'evo_rpe', 'tum', gt, est,
        '--align',
        '--t_max_diff', '0.1',
        '--delta', str(delta),
        '--delta_unit', 'm',
    ]
    m = run_evo(cmd, f'RPE {label}')
    print(f'  RPE {label:20s}  RMSE={m.get("rmse", float("nan")):.3f}m  '
          f'(per {delta}m segment)')
    return m


def traj_plot(gt: str, estimates: dict, out_dir: str, title: str):
    """Overlay all trajectories in one plot."""
    plot_path = os.path.join(out_dir, 'trajectories.png')
    cmd = ['evo_traj', 'tum', '--ref', gt] + list(estimates.values()) + \
          ['-p', '--save_plot', plot_path, '--plot_mode', 'xy']
    subprocess.run(cmd, capture_output=True)
    print(f'  Trajectory overlay → {plot_path}')


def write_markdown(results: dict, sequence: str, out_dir: str):
    """Write BENCHMARK.md with the results table."""
    md_path = os.path.join(out_dir, 'BENCHMARK.md')
    with open(md_path, 'w') as f:
        f.write(f'# Benchmark Results: NCLT Sequence {sequence}\n\n')
        f.write('## Absolute Trajectory Error (ATE)\n\n')
        f.write('| Filter | RMSE (m) | Max error (m) |\n')
        f.write('|--------|----------|---------------|\n')
        for label, m in results['ate'].items():
            f.write(f'| {label} | {m.get("rmse", float("nan")):.3f} | '
                    f'{m.get("max", float("nan")):.3f} |\n')

        f.write('\n## Relative Pose Error (RPE, per 10m segment)\n\n')
        f.write('| Filter | RMSE (m) |\n')
        f.write('|--------|----------|\n')
        for label, m in results['rpe'].items():
            f.write(f'| {label} | {m.get("rmse", float("nan")):.3f} |\n')

        f.write('\n## Methodology\n\n')
        f.write('- Dataset: NCLT (University of Michigan)\n')
        f.write(f'- Sequence: {sequence}\n')
        f.write('- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU\n')
        f.write('- Evaluation tool: [evo](https://github.com/MichaelGrupp/evo)\n')
        f.write('- Alignment: SE(3) alignment\n')
        f.write('- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)\n\n')
        f.write('### Reproducing\n\n')
        f.write('```bash\n')
        f.write('# 1. Download NCLT sequence\n')
        f.write('# 2. Run benchmark\n')
        f.write(f'ros2 launch fusioncore_datasets nclt_benchmark.launch.py \\\n')
        f.write(f'  data_dir:=/path/to/nclt/{sequence} output_bag:=./nclt_results\n')
        f.write('# 3. Convert ground truth\n')
        f.write('python3 tools/nclt_rtk_to_tum.py --rtk gps_rtk.csv --out gt.tum\n')
        f.write('# 4. Extract trajectories\n')
        f.write('python3 tools/odom_to_tum.py --bag ./nclt_results --topic /fusion/odom --out fc.tum\n')
        f.write('python3 tools/odom_to_tum.py --bag ./nclt_results --topic /rl/odometry --out rl.tum\n')
        f.write('# 5. Evaluate\n')
        f.write(f'python3 tools/evaluate.py --gt gt.tum --fusioncore fc.tum --rl rl.tum --sequence {sequence}\n')
        f.write('```\n')

    print(f'  Results written to {md_path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--gt',          required=True, help='Ground truth TUM file')
    parser.add_argument('--fusioncore',  required=True, help='FusionCore TUM file')
    parser.add_argument('--rl',          required=True, help='robot_localization TUM file')
    parser.add_argument('--sequence',    default='unknown', help='NCLT sequence name')
    parser.add_argument('--out_dir',     default='./benchmarks/nclt/2012-01-08/results',
                        help='Output directory for plots and report')
    parser.add_argument('--rpe_delta',   type=float, default=10.0,
                        help='RPE segment length in meters (default: 10)')
    args = parser.parse_args()

    for path, name in [(args.gt, 'ground truth'), (args.fusioncore, 'FusionCore'),
                       (args.rl, 'robot_localization')]:
        if not os.path.exists(path):
            print(f'Error: {name} file not found: {path}', file=sys.stderr)
            sys.exit(1)

    Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    print(f'\n{"="*60}')
    print(f'  NCLT Benchmark: {args.sequence}')
    print(f'{"="*60}\n')

    results = {'ate': {}, 'rpe': {}}

    print('── Absolute Trajectory Error (ATE) ──────────────────────')
    results['ate']['FusionCore'] = evo_ape(args.gt, args.fusioncore, 'fusioncore', args.out_dir)
    results['ate']['RL-EKF']     = evo_ape(args.gt, args.rl,          'rl_ekf',     args.out_dir)

    print('\n── Relative Pose Error (RPE) ─────────────────────────────')
    results['rpe']['FusionCore'] = evo_rpe(args.gt, args.fusioncore, 'fusioncore', args.out_dir, args.rpe_delta)
    results['rpe']['RL-EKF']     = evo_rpe(args.gt, args.rl,          'rl_ekf',     args.out_dir, args.rpe_delta)

    print('\n── Trajectory overlay ────────────────────────────────────')
    traj_plot(args.gt, {'FusionCore': args.fusioncore, 'RL-EKF': args.rl}, args.out_dir, args.sequence)

    print('\n── Summary ───────────────────────────────────────────────')
    fc_ate = results['ate']['FusionCore'].get('rmse', float('nan'))
    rl_ate = results['ate']['RL-EKF'].get('rmse', float('nan'))
    if fc_ate < rl_ate:
        diff = (rl_ate - fc_ate) / rl_ate * 100
        print(f'  FusionCore wins ATE by {diff:.1f}%  ({fc_ate:.3f}m vs {rl_ate:.3f}m)')
    elif rl_ate < fc_ate:
        diff = (fc_ate - rl_ate) / fc_ate * 100
        print(f'  RL-EKF wins ATE by {diff:.1f}%  ({rl_ate:.3f}m vs {fc_ate:.3f}m)')
    else:
        print(f'  Tie  ({fc_ate:.3f}m)')

    write_markdown(results, args.sequence, args.out_dir)
    print(f'\nPlots saved to {args.out_dir}/')


if __name__ == '__main__':
    main()
