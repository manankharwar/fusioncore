#!/usr/bin/env python3
"""
NCLT benchmark evaluator: FusionCore vs robot_localization EKF vs robot_localization UKF.

Metrics computed per filter (SE3-aligned to ground truth):
  - ATE translational RMSE, mean, max
  - ATE rotational RMSE (degrees)
  - % of poses within 5 m and 10 m of ground truth
  - Path length ratio (estimated / ground truth)
  - Drift rate (ATE RMSE per km traveled)
  - RPE at 10 m segment length

Prerequisites:
  pip install evo matplotlib

Usage:
  python3 tools/evaluate.py \
    --gt           ground_truth.tum \
    --fusioncore   fusioncore.tum \
    --rl           rl_ekf.tum \
    --rl_ukf       rl_ukf.tum \
    --sequence     2012-01-08 \
    --out_dir      ./benchmarks/nclt/2012-01-08/results
"""

import argparse
import copy
import json
import os
import sys
from pathlib import Path

import numpy as np

try:
    from evo.core import metrics, sync
    from evo.core.trajectory import PoseTrajectory3D
    from evo.tools import file_interface
except ImportError:
    print("Error: evo not installed. Run: pip install evo", file=sys.stderr)
    sys.exit(1)


# ── loaders ────────────────────────────────────────────────────────────────────

def load_tum(path: str) -> PoseTrajectory3D:
    return file_interface.read_tum_trajectory_file(path)


def align(est: PoseTrajectory3D, gt: PoseTrajectory3D) -> PoseTrajectory3D:
    """Return SE3-aligned copy of est matched to gt timestamps."""
    est_copy = copy.deepcopy(est)
    est_copy.align(gt, correct_scale=False)
    return est_copy


# ── metric helpers ─────────────────────────────────────────────────────────────

def compute_ate(gt: PoseTrajectory3D, est: PoseTrajectory3D) -> dict:
    gt_s, est_s = sync.associate_trajectories(gt, est, max_diff=0.1)
    est_aligned = align(est_s, gt_s)
    metric = metrics.APE(metrics.PoseRelation.translation_part)
    metric.process_data((gt_s, est_aligned))
    errors = metric.error

    # XY-only ATE: horizontal error independent of Z. This is the metric that
    # actually matters for ground robot navigation (nav2, obstacle avoidance).
    # 3D ATE is distorted by GPS altitude noise in FC vs forced-zero Z in RL-EKF.
    gt_xy  = gt_s.positions_xyz[:, :2]
    est_xy = est_aligned.positions_xyz[:, :2]
    xy_errors = np.linalg.norm(gt_xy - est_xy, axis=1)

    return {
        'rmse':        float(np.sqrt(np.mean(errors**2))),
        'mean':        float(np.mean(errors)),
        'max':         float(np.max(errors)),
        'pct5':        float(np.mean(errors <= 5.0) * 100),
        'pct10':       float(np.mean(errors <= 10.0) * 100),
        'xy_rmse':     float(np.sqrt(np.mean(xy_errors**2))),
        'xy_pct5':     float(np.mean(xy_errors <= 5.0) * 100),
        'xy_pct10':    float(np.mean(xy_errors <= 10.0) * 100),
        'errors':      errors,
        'gt_s':        gt_s,
        'est_aligned': est_aligned,
    }


def compute_ate_rot(gt: PoseTrajectory3D, est: PoseTrajectory3D) -> float:
    gt_s, est_s = sync.associate_trajectories(gt, est, max_diff=0.1)
    est_aligned = align(est_s, gt_s)
    metric = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
    metric.process_data((gt_s, est_aligned))
    return float(np.sqrt(np.mean(metric.error**2)))


def compute_rpe(gt: PoseTrajectory3D, est: PoseTrajectory3D, delta_m: float) -> dict:
    gt_s, est_s = sync.associate_trajectories(gt, est, max_diff=0.1)
    metric = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=delta_m,
        delta_unit=metrics.Unit.meters,
        all_pairs=False,
    )
    try:
        metric.process_data((gt_s, est_s))
        errors = metric.error
        return {'rmse': float(np.sqrt(np.mean(errors**2))),
                'mean': float(np.mean(errors)),
                'max':  float(np.max(errors))}
    except Exception:
        return {'rmse': float('nan'), 'mean': float('nan'), 'max': float('nan')}


def path_length(traj: PoseTrajectory3D) -> float:
    diffs = np.diff(traj.positions_xyz, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def path_length_ratio(gt: PoseTrajectory3D, est: PoseTrajectory3D) -> float:
    gt_s, est_s = sync.associate_trajectories(gt, est, max_diff=0.1)
    gt_len  = path_length(gt_s)
    est_len = path_length(est_s)
    return est_len / gt_len if gt_len > 0 else float('nan')


def drift_rate(ate_rmse: float, gt: PoseTrajectory3D) -> float:
    km = path_length(gt) / 1000.0
    return ate_rmse / km if km > 0 else float('nan')


# ── printing ───────────────────────────────────────────────────────────────────

def fmt(val, unit='m', decimals=3):
    if val != val:
        return 'N/A'
    return f'{val:.{decimals}f}{unit}'


def print_section(title: str):
    pad = max(0, 54 - len(title))
    print(f'\n-- {title} {"-" * pad}')


def evaluate_filter(label: str, gt: PoseTrajectory3D, est: PoseTrajectory3D) -> dict:
    ate   = compute_ate(gt, est)
    rpe10 = compute_rpe(gt, est, 10.0)
    plr   = path_length_ratio(gt, est)
    dr    = drift_rate(ate['rmse'], ate['gt_s'])

    print(f'\n  [{label}]')
    print(f'    ATE RMSE (3D)       {fmt(ate["rmse"])}')
    print(f'    ATE RMSE (XY only)  {fmt(ate["xy_rmse"])}')
    print(f'    Within 5 m          {ate["pct5"]:.1f}%  |  Within 10 m  {ate["pct10"]:.1f}%')
    print(f'    XY within 5 m       {ate["xy_pct5"]:.1f}%  |  XY within 10 m  {ate["xy_pct10"]:.1f}%')
    print(f'    Path length ratio   {fmt(plr, "", 4)}  (1.0 = perfect scale)')
    print(f'    Drift rate          {fmt(dr, " m/km", 2)}')
    print(f'    RPE@10m             RMSE={fmt(rpe10["rmse"])}')

    return {'ate': ate, 'rpe10': rpe10, 'plr': plr, 'drift': dr}


# ── plots ──────────────────────────────────────────────────────────────────────

def save_trajectory_plot(gt, filters, out_dir):
    """filters: list of (label, result_dict, color)"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(10, 8))

        gt_xy = gt.positions_xyz[:, :2]
        ax.plot(gt_xy[:, 0], gt_xy[:, 1], '--', color='#2ca02c',
                label='Ground Truth', alpha=0.7, lw=1.5)

        for label, res, color in filters:
            xy = res['ate']['est_aligned'].positions_xyz[:, :2]
            ax.plot(xy[:, 0], xy[:, 1], '-', color=color,
                    label=label, alpha=0.85, lw=1.2)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Trajectory Overlay (XY, SE3-aligned)')
        ax.set_aspect('equal')
        ax.legend()
        ax.grid(True, alpha=0.3)

        path = os.path.join(out_dir, 'trajectories.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'  Trajectory overlay  -> {path}')
    except Exception as e:
        print(f'  [WARN] trajectory plot failed: {e}')


def save_ate_plot(filters, out_dir):
    """filters: list of (label, result_dict, color)"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(12, 4))

        for label, res, color in filters:
            t = res['ate']['gt_s'].timestamps
            t = t - t[0]
            rmse = res['ate']['rmse']
            ax.plot(t, res['ate']['errors'],
                    label=f'{label} (RMSE={rmse:.1f}m)',
                    color=color, alpha=0.8, lw=0.8)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('ATE (m)')
        ax.set_title('Per-pose Translational Error over Time')
        ax.legend()
        ax.grid(True, alpha=0.3)

        path = os.path.join(out_dir, 'ate_over_time.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'  ATE over time       -> {path}')
    except Exception as e:
        print(f'  [WARN] ATE over time plot failed: {e}')


def save_error_distribution(filters, out_dir):
    """filters: list of (label, result_dict, color)"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 5))
        thresholds = np.linspace(0, 50, 200)

        for label, res, color in filters:
            errors = res['ate']['errors']
            cdf = [np.mean(errors <= t) * 100 for t in thresholds]
            ax.plot(thresholds, cdf, label=label, color=color, lw=2)

        ax.axvline(5,  color='gray', linestyle=':',  alpha=0.7, label='5 m')
        ax.axvline(10, color='gray', linestyle='--', alpha=0.7, label='10 m')
        ax.set_xlabel('ATE threshold (m)')
        ax.set_ylabel('% poses within threshold')
        ax.set_title('Error Distribution (CDF)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, 50)
        ax.set_ylim(0, 100)

        path = os.path.join(out_dir, 'error_distribution.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'  Error distribution  -> {path}')
    except Exception as e:
        print(f'  [WARN] error distribution plot failed: {e}')


# ── markdown ───────────────────────────────────────────────────────────────────

def write_markdown(filters, sequence: str, out_dir: str):
    """filters: list of (label, result_dict, color)"""
    md_path = os.path.join(out_dir, 'BENCHMARK.md')
    with open(md_path, 'w') as f:
        f.write(f'# Benchmark Results: NCLT Sequence {sequence}\n\n')

        f.write('## Metrics (SE3-aligned to RTK ground truth)\n\n')
        f.write('| Filter | ATE RMSE 3D (m) | ATE RMSE XY (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |\n')
        f.write('|--------|----------------|----------------|------------|-------------|-------------------|--------------|------------------|\n')
        for label, r, _ in filters:
            ate = r['ate']
            f.write(f"| {label} | {ate['rmse']:.3f} | {ate['xy_rmse']:.3f} | {ate['pct5']:.1f}% | {ate['pct10']:.1f}% | "
                    f"{r['plr']:.4f} | {r['drift']:.2f} | {r['rpe10']['rmse']:.3f} |\n")

        f.write('\n## Methodology\n\n')
        f.write('- Dataset: NCLT (University of Michigan)\n')
        f.write(f'- Sequence: {sequence}\n')
        f.write('- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU\n')
        f.write('- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment\n')
        f.write('- All filters consume identical sensor streams: same IMU, wheel odometry, and GPS topics\n')
        f.write('- FusionCore: full 3D UKF, adaptive noise, ZUPT, IMU bias estimation\n')
        f.write('- RL-EKF: two_d_mode=true (flat-terrain Segway RMP), GPS via navsat_transform\n')
        f.write('- RL-UKF excluded: robot_localization UKF diverges under high-rate sim time playback\n')
        f.write('  (rapid timer catchup causes near-zero dt between predictions, Cholesky failure, immediate NaN)\n')

    print(f'  Results written to {md_path}')


# ── machine-readable output (for regression tracking) ───────────────────────────

def write_json(filters, sequence: str, poses: dict, out_dir: str):
    """Dump per-filter metrics to metrics.json so a regression harness can diff
    scores across commits instead of anyone eyeballing a markdown table."""
    data = {'sequence': sequence, 'poses': poses, 'filters': {}}
    for label, res, _ in filters:
        ate = res['ate']
        data['filters'][label] = {
            'ate_rmse_3d': round(ate['rmse'], 3),
            'ate_rmse_xy': round(ate['xy_rmse'], 3),
            'within_5m':   round(ate['pct5'], 1),
            'within_10m':  round(ate['pct10'], 1),
            'path_length_ratio': round(res['plr'], 4),
            'drift_m_per_km':    round(res['drift'], 2),
            'rpe10_rmse':        round(res['rpe10']['rmse'], 3),
        }
    path = os.path.join(out_dir, 'metrics.json')
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f'  Metrics JSON        -> {path}')


# ── main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--gt',         required=True)
    parser.add_argument('--fusioncore', required=True)
    parser.add_argument('--rl',         required=True)
    parser.add_argument('--rl_ukf',     default=None, help='RL-UKF trajectory (optional)')
    parser.add_argument('--sequence',   default='unknown')
    parser.add_argument('--out_dir',    default='./results')
    args = parser.parse_args()

    for path, name in [(args.gt, 'ground truth'), (args.fusioncore, 'FusionCore'),
                       (args.rl, 'robot_localization EKF')]:
        if not os.path.exists(path):
            print(f'Error: {name} file not found: {path}', file=sys.stderr)
            sys.exit(1)

    if args.rl_ukf and not os.path.exists(args.rl_ukf):
        print(f'Error: RL-UKF file not found: {args.rl_ukf}', file=sys.stderr)
        sys.exit(1)

    Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    gt = load_tum(args.gt)
    fc = load_tum(args.fusioncore)
    rl = load_tum(args.rl)

    print(f'\n{"="*60}')
    print(f'  NCLT Benchmark: {args.sequence}')
    print(f'{"="*60}')

    pose_line = f'  Poses: FC={len(fc.timestamps)}  RL-EKF={len(rl.timestamps)}  GT={len(gt.timestamps)}'
    if args.rl_ukf:
        rl_ukf_traj = load_tum(args.rl_ukf)
        pose_line += f'  RL-UKF={len(rl_ukf_traj.timestamps)}'
    print(pose_line)

    print_section('Metrics (SE3-aligned to RTK ground truth)')
    fc_res = evaluate_filter('FusionCore', gt, fc)
    rl_res = evaluate_filter('RL-EKF',     gt, rl)

    filters = [
        ('FusionCore', fc_res, '#1f77b4'),
        ('RL-EKF',     rl_res, '#ff7f0e'),
    ]

    if args.rl_ukf:
        rl_ukf_res = evaluate_filter('RL-UKF', gt, rl_ukf_traj)
        filters.append(('RL-UKF', rl_ukf_res, '#9467bd'))

    print_section('Summary')
    fc_ate    = fc_res['ate']['rmse']
    fc_ate_xy = fc_res['ate']['xy_rmse']
    for label, res, _ in filters[1:]:
        other_ate    = res['ate']['rmse']
        other_ate_xy = res['ate']['xy_rmse']

        # 3D summary
        if fc_ate < other_ate:
            diff = (other_ate - fc_ate) / other_ate * 100
            print(f'  3D: FusionCore beats {label} by {diff:.1f}%  ({fc_ate:.3f}m vs {other_ate:.3f}m)')
        elif other_ate < fc_ate:
            diff = (fc_ate - other_ate) / fc_ate * 100
            print(f'  3D: {label} beats FusionCore by {diff:.1f}%  ({other_ate:.3f}m vs {fc_ate:.3f}m)')
        else:
            print(f'  3D: Tie vs {label}  ({fc_ate:.3f}m)')

        # XY summary (what actually matters for ground robot navigation)
        if fc_ate_xy < other_ate_xy:
            diff = (other_ate_xy - fc_ate_xy) / other_ate_xy * 100
            print(f'  XY: FusionCore beats {label} by {diff:.1f}%  ({fc_ate_xy:.3f}m vs {other_ate_xy:.3f}m)')
        elif other_ate_xy < fc_ate_xy:
            diff = (fc_ate_xy - other_ate_xy) / fc_ate_xy * 100
            print(f'  XY: {label} beats FusionCore by {diff:.1f}%  ({other_ate_xy:.3f}m vs {fc_ate_xy:.3f}m)')
        else:
            print(f'  XY: Tie vs {label}  ({fc_ate_xy:.3f}m)')

    print_section('Plots')
    save_trajectory_plot(gt, filters, args.out_dir)
    save_ate_plot(filters, args.out_dir)
    save_error_distribution(filters, args.out_dir)

    write_markdown(filters, args.sequence, args.out_dir)
    poses = {'FusionCore': len(fc.timestamps), 'RL-EKF': len(rl.timestamps),
             'GT': len(gt.timestamps)}
    write_json(filters, args.sequence, poses, args.out_dir)
    print(f'\nDone. Results in {args.out_dir}/\n')


if __name__ == '__main__':
    main()
