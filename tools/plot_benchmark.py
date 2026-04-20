#!/usr/bin/env python3
"""
FusionCore NCLT benchmark visualizer.

Produces a single PNG with four panels:
  1. Top-down trajectory comparison (GT vs FC vs RL-EKF)
  2. ATE bar chart (FC vs RL-EKF)
  3. GPS spike response — position error over time around t=120s
  4. RL-UKF divergence — position magnitude over time (shows death at t≈31s)

Usage:
  python3 tools/plot_benchmark.py \
    --seq_dir  benchmarks/nclt/2012-01-08 \
    --out      benchmarks/nclt/2012-01-08/results/benchmark_plot.png
"""

import argparse
import math
import os
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


# ── palette ──────────────────────────────────────────────────────────────────
C_GT  = '#888888'
C_FC  = '#2196F3'   # blue
C_EKF = '#FF5722'   # red-orange
C_UKF = '#9C27B0'   # purple


def load_tum(path: str):
    """Returns (timestamps, x, y, z) as numpy arrays. Skips NaN rows."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            vals = [float(v) for v in p[:4]]
            if any(math.isnan(v) or math.isinf(v) for v in vals):
                continue
            rows.append(vals)
    if not rows:
        return np.array([]), np.array([]), np.array([]), np.array([])
    arr = np.array(rows)
    return arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]


def align_se2(src_xy, ref_xy):
    """Least-squares SE(2) alignment of src onto ref (translation + rotation)."""
    # Downsample to avoid SVD slowdown on large arrays
    step = max(1, len(src_xy) // 2000)
    s = src_xy[::step]
    r = ref_xy[::step]
    n = min(len(s), len(r))
    s, r = s[:n], r[:n]
    mu_s = s.mean(axis=0)
    mu_r = r.mean(axis=0)
    H = (s - mu_s).T @ (r - mu_r)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = mu_r - R @ mu_s
    return (R @ src_xy.T).T + t


def error_over_time(est_ts, est_x, est_y, gt_ts, gt_x, gt_y):
    """Interpolate GT onto est timestamps, return per-pose 2D error."""
    errs = np.full(len(est_ts), np.nan)
    for i, t in enumerate(est_ts):
        idx = np.searchsorted(gt_ts, t)
        if idx == 0 or idx >= len(gt_ts):
            continue
        t0, t1 = gt_ts[idx-1], gt_ts[idx]
        if t1 == t0:
            continue
        alpha = (t - t0) / (t1 - t0)
        gx = gt_x[idx-1] + alpha * (gt_x[idx] - gt_x[idx-1])
        gy = gt_y[idx-1] + alpha * (gt_y[idx] - gt_y[idx-1])
        errs[i] = math.hypot(est_x[i] - gx, est_y[i] - gy)
    return errs


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seq_dir', default='benchmarks/nclt/2012-01-08',
                        help='Path to the NCLT sequence directory')
    parser.add_argument('--out', default=None,
                        help='Output PNG path (default: <seq_dir>/results/benchmark_plot.png)')
    args = parser.parse_args()

    seq = Path(args.seq_dir)
    out = Path(args.out) if args.out else seq / 'results' / 'benchmark_plot.png'
    out.parent.mkdir(parents=True, exist_ok=True)

    # ── load trajectories ────────────────────────────────────────────────────
    def load(name):
        p = seq / name
        if not p.exists():
            return None, None, None, None
        return load_tum(str(p))

    gt_ts,  gt_x,  gt_y,  _    = load('ground_truth.tum')
    fc_ts,  fc_x,  fc_y,  _    = load('fusioncore.tum')
    ek_ts,  ek_x,  ek_y,  _    = load('rl_ekf.tum')
    uk_ts,  uk_x,  uk_y,  _    = load('rl_ukf.tum')

    fcs_ts, fcs_x, fcs_y, _    = load('fusioncore_spike.tum')
    eks_ts, eks_x, eks_y, _    = load('rl_ekf_spike.tum')

    # ── figure layout ────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 12), facecolor='#0d1117')
    fig.suptitle('FusionCore — NCLT 2012-01-08 Benchmark', fontsize=16,
                 color='white', fontweight='bold', y=0.98)

    gs = fig.add_gridspec(2, 2, hspace=0.38, wspace=0.32,
                          left=0.07, right=0.97, top=0.93, bottom=0.07)
    ax_traj  = fig.add_subplot(gs[0, 0])
    ax_ate   = fig.add_subplot(gs[0, 1])
    ax_spike = fig.add_subplot(gs[1, 0])
    ax_ukf   = fig.add_subplot(gs[1, 1])

    for ax in (ax_traj, ax_ate, ax_spike, ax_ukf):
        ax.set_facecolor('#161b22')
        ax.tick_params(colors='#8b949e', labelsize=9)
        for spine in ax.spines.values():
            spine.set_edgecolor('#30363d')
        ax.title.set_color('white')
        ax.xaxis.label.set_color('#8b949e')
        ax.yaxis.label.set_color('#8b949e')

    # ── Panel 1: top-down trajectory ─────────────────────────────────────────
    ax = ax_traj
    ax.set_title('Top-Down Trajectory', fontsize=11, pad=8)

    if gt_x is not None and len(gt_x):
        gt_xy = np.stack([gt_x, gt_y], axis=1)
        # Center on GT mean
        center = gt_xy.mean(axis=0)

        ax.plot(gt_x - center[0], gt_y - center[1],
                color=C_GT, lw=1.2, alpha=0.7, label='Ground Truth (RTK GPS)', zorder=1)

        if fc_x is not None and len(fc_x):
            fc_xy = np.stack([fc_x, fc_y], axis=1)
            fc_al = align_se2(fc_xy, gt_xy)
            ax.plot(fc_al[:, 0] - center[0], fc_al[:, 1] - center[1],
                    color=C_FC, lw=1.4, alpha=0.9, label='FusionCore (ATE 5.5m)', zorder=3)

        if ek_x is not None and len(ek_x):
            ek_xy = np.stack([ek_x, ek_y], axis=1)
            ek_al = align_se2(ek_xy, gt_xy)
            ax.plot(ek_al[:, 0] - center[0], ek_al[:, 1] - center[1],
                    color=C_EKF, lw=1.0, alpha=0.7, label='RL-EKF (ATE 23.4m)', zorder=2)

        # start marker
        ax.plot(gt_x[0] - center[0], gt_y[0] - center[1],
                'o', color='white', ms=5, zorder=5, label='Start')

    ax.set_aspect('equal')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    leg = ax.legend(fontsize=8, loc='upper left',
                    facecolor='#21262d', edgecolor='#30363d', labelcolor='white')

    # ── Panel 2: ATE bar chart ────────────────────────────────────────────────
    ax = ax_ate
    ax.set_title('ATE RMSE — Lower is Better', fontsize=11, pad=8)

    filters = ['FusionCore', 'RL-EKF']
    rmses   = [5.517, 23.434]
    colors  = [C_FC, C_EKF]
    bars = ax.bar(filters, rmses, color=colors, width=0.45, zorder=3)

    for bar, val in zip(bars, rmses):
        ax.text(bar.get_x() + bar.get_width() / 2, val + 0.3,
                f'{val:.1f} m', ha='center', va='bottom',
                color='white', fontsize=11, fontweight='bold')

    # improvement annotation
    pct = (rmses[1] - rmses[0]) / rmses[1] * 100
    ax.annotate('', xy=(1, rmses[0]), xytext=(1, rmses[1]),
                arrowprops=dict(arrowstyle='<->', color='#58a6ff', lw=1.5))
    ax.text(1.28, (rmses[0] + rmses[1]) / 2,
            f'−{pct:.0f}%', color='#58a6ff', fontsize=11, va='center', fontweight='bold')

    ax.set_ylabel('ATE RMSE (m)')
    ax.set_ylim(0, rmses[1] * 1.25)
    ax.grid(axis='y', color='#30363d', lw=0.6, zorder=0)
    ax.tick_params(axis='x', labelsize=10, colors='white')

    # ── Panel 3: GPS spike response ───────────────────────────────────────────
    ax = ax_spike
    ax.set_title('GPS Spike Rejection — 707 m Corrupted Fix at t=120 s', fontsize=11, pad=8)

    SPIKE_T = 120.0

    def plot_spike_error(ts, x, y, color, label):
        if ts is None or len(ts) == 0:
            return
        t0 = gt_ts[0]
        rel_ts = ts - t0
        errs = error_over_time(ts, x, y, gt_ts, gt_x, gt_y)
        # center on spike
        mask = (rel_ts >= SPIKE_T - 40) & (rel_ts <= SPIKE_T + 50)
        ax.plot(rel_ts[mask] - SPIKE_T, errs[mask], color=color, lw=1.6, label=label)

    plot_spike_error(fcs_ts, fcs_x, fcs_y, C_FC,  'FusionCore — held position (Mahalanobis gate)')
    plot_spike_error(eks_ts, eks_x, eks_y, C_EKF, 'RL-EKF — jumped 93 m then recovered')

    ax.axvline(0, color='#f85149', lw=1.4, ls='--', label='Spike injected')
    ax.set_xlabel('Time relative to spike (s)')
    ax.set_ylabel('2D position error vs GT (m)')
    ax.grid(color='#30363d', lw=0.5, zorder=0)
    leg = ax.legend(fontsize=8, facecolor='#21262d', edgecolor='#30363d', labelcolor='white')

    ax.annotate('707 m fake fix', xy=(0, 5), xytext=(8, 20),
                color='#f85149', fontsize=8,
                arrowprops=dict(arrowstyle='->', color='#f85149', lw=1.0))

    # ── Panel 4: RL-UKF divergence ────────────────────────────────────────────
    ax = ax_ukf
    ax.set_title('RL-UKF Numerical Divergence', fontsize=11, pad=8)

    if uk_ts is not None and len(uk_ts):
        t0 = uk_ts[0]
        rel = uk_ts - t0
        mag = np.hypot(uk_x, uk_y)
        ax.plot(rel, mag, color=C_UKF, lw=1.4, label='RL-UKF position magnitude')

    # show FC on same scale for reference
    if fc_ts is not None and len(fc_ts):
        t0_fc = fc_ts[0]
        fc_rel = fc_ts - t0_fc
        fc_mag = np.hypot(fc_x, fc_y)
        mask = fc_rel <= 60
        ax.plot(fc_rel[mask], fc_mag[mask], color=C_FC, lw=1.2, alpha=0.7,
                label='FusionCore (stable reference)')

    ax.axvline(31, color='#f85149', lw=1.4, ls='--')
    ax.text(32, ax.get_ylim()[1] * 0.9 if ax.get_ylim()[1] > 1 else 50,
            'Diverges\nt ≈ 31 s', color='#f85149', fontsize=8)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position magnitude (m)')
    ax.set_xlim(0, 60)
    ax.grid(color='#30363d', lw=0.5, zorder=0)
    leg = ax.legend(fontsize=8, facecolor='#21262d', edgecolor='#30363d', labelcolor='white')

    # Add a note about NaN → truncated trajectory
    ax.text(0.97, 0.05,
            'RL-UKF trajectory truncated at divergence\n(covariance NaN explosion)',
            transform=ax.transAxes, ha='right', va='bottom',
            color='#8b949e', fontsize=7.5)

    # ── save ─────────────────────────────────────────────────────────────────
    fig.savefig(str(out), dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f'Saved: {out}')


if __name__ == '__main__':
    main()
