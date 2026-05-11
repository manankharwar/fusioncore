#!/usr/bin/env python3
"""
FusionCore quick demo (no ROS required).

Shows two things from pre-baked NCLT benchmark results included in the repo:
  1. GPS spike resilience: a 707 m fake GPS fix is injected at t=120 s.
     FusionCore rejects it (chi-squared gate). robot_localization EKF accepts it
     and jumps 50+ m off-course before recovering.
  2. Overall accuracy: FusionCore 5.6 m ATE vs RL-EKF 13.0 m ATE over a
     600 s campus drive (NCLT 2012-01-08, RTK GPS ground truth).

Usage:
  python3 tools/demo_quick.py
  python3 tools/demo_quick.py --open      # open image when done
  python3 tools/demo_quick.py --out /tmp/result.png

Requirements: numpy, matplotlib   (pip install numpy matplotlib)
"""

import argparse
import math
import os
import subprocess
import sys
from pathlib import Path

try:
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.lines import Line2D
except ImportError:
    sys.exit("Missing dependencies. Run:  pip install numpy matplotlib")

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT  = SCRIPT_DIR.parent
SEQ_DIR    = REPO_ROOT / 'benchmarks' / 'nclt' / '2012-01-08'

BG     = '#FFFFFF'
BORDER = '#E2E8F0'
TEXT   = '#0F172A'
MUTED  = '#64748B'
C_FC   = '#2563EB'
C_EKF  = '#DC2626'
C_GT   = '#6B7280'
C_SPIKE_LINE = '#F59E0B'


def load_tum(path: Path):
    rows = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            p = s.split()
            if len(p) < 4:
                continue
            vals = [float(v) for v in p[:4]]
            if any(math.isnan(v) or math.isinf(v) for v in vals):
                continue
            rows.append(vals)
    if not rows:
        return tuple(np.array([]) for _ in range(4))
    a = np.array(rows)
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3]


def se2_align_presplit(src_ts, src_xy, ref_ts, ref_xy, t_split):
    """SE2-align using only data before t_split; apply the fixed transform to all points."""
    mask = src_ts <= t_split
    step = max(1, mask.sum() // 1000)
    s_pts, r_pts = [], []
    for i in np.where(mask)[0][::step]:
        t = src_ts[i]
        idx = np.searchsorted(ref_ts, t)
        if idx == 0 or idx >= len(ref_ts):
            continue
        t0, t1 = ref_ts[idx - 1], ref_ts[idx]
        if t1 == t0:
            continue
        a = (t - t0) / (t1 - t0)
        gx = ref_xy[idx-1, 0] + a * (ref_xy[idx, 0] - ref_xy[idx-1, 0])
        gy = ref_xy[idx-1, 1] + a * (ref_xy[idx, 1] - ref_xy[idx-1, 1])
        s_pts.append(src_xy[i])
        r_pts.append([gx, gy])
    s, r = np.array(s_pts), np.array(r_pts)
    mu_s, mu_r = s.mean(0), r.mean(0)
    H = (s - mu_s).T @ (r - mu_r)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return (R @ src_xy.T).T + (mu_r - R @ mu_s)


def interp_gt(ts, gt_ts, gt_x, gt_y):
    """Return interpolated GT position at each timestamp in ts."""
    xs, ys = [], []
    for t in ts:
        idx = np.searchsorted(gt_ts, t)
        if idx == 0 or idx >= len(gt_ts):
            xs.append(np.nan); ys.append(np.nan); continue
        t0, t1 = gt_ts[idx - 1], gt_ts[idx]
        if t1 == t0:
            xs.append(np.nan); ys.append(np.nan); continue
        a = (t - t0) / (t1 - t0)
        xs.append(gt_x[idx-1] + a * (gt_x[idx] - gt_x[idx-1]))
        ys.append(gt_y[idx-1] + a * (gt_y[idx] - gt_y[idx-1]))
    return np.array(xs), np.array(ys)


def per_point_error(ts, al_xy, gt_ts, gt_x, gt_y):
    times, errs = [], []
    for i, t in enumerate(ts):
        idx = np.searchsorted(gt_ts, t)
        if idx == 0 or idx >= len(gt_ts):
            continue
        t0, t1 = gt_ts[idx - 1], gt_ts[idx]
        if t1 == t0:
            continue
        a = (t - t0) / (t1 - t0)
        gx = gt_x[idx-1] + a * (gt_x[idx] - gt_x[idx-1])
        gy = gt_y[idx-1] + a * (gt_y[idx] - gt_y[idx-1])
        times.append(t - ts[0])
        errs.append(math.hypot(al_xy[i, 0] - gx, al_xy[i, 1] - gy))
    return np.array(times), np.array(errs)


def run(out_path: Path, open_after: bool):
    print("FusionCore demo: NCLT 2012-01-08  (no ROS required)")
    print(f"  Loading data from {SEQ_DIR.relative_to(REPO_ROOT)} ...")

    gt_ts, gt_x, gt_y, _ = load_tum(SEQ_DIR / 'ground_truth.tum')
    fc_ts, fc_x, fc_y, _ = load_tum(SEQ_DIR / 'fusioncore_spike.tum')
    ek_ts, ek_x, ek_y, _ = load_tum(SEQ_DIR / 'rl_ekf_spike.tum')

    gt_xy = np.stack([gt_x, gt_y], 1)

    SPIKE_REL = 120.0
    fc_rel = fc_ts - fc_ts[0]
    ek_rel = ek_ts - ek_ts[0]

    fc_al = se2_align_presplit(fc_ts, np.stack([fc_x, fc_y], 1), gt_ts, gt_xy, fc_ts[0] + SPIKE_REL - 20)
    ek_al = se2_align_presplit(ek_ts, np.stack([ek_x, ek_y], 1), gt_ts, gt_xy, ek_ts[0] + SPIKE_REL - 20)

    fc_times, fc_errs = per_point_error(fc_ts, fc_al, gt_ts, gt_x, gt_y)
    ek_times, ek_errs = per_point_error(ek_ts, ek_al, gt_ts, gt_x, gt_y)

    fc_pre  = fc_errs[fc_times < SPIKE_REL].mean()
    ek_pre  = ek_errs[ek_times < SPIKE_REL].mean()
    fc_peak = fc_errs[(fc_times >= SPIKE_REL) & (fc_times <= SPIKE_REL + 30)].max()
    ek_peak = ek_errs[(ek_times >= SPIKE_REL) & (ek_times <= SPIKE_REL + 60)].max()

    print(f"  Spike test (707 m fake GPS fix at t={SPIKE_REL:.0f} s):")
    print(f"    FusionCore  pre-spike error {fc_pre:.1f} m  →  peak {fc_peak:.1f} m  (change: +{fc_peak-fc_pre:.1f} m)")
    print(f"    RL-EKF      pre-spike error {ek_pre:.1f} m  →  peak {ek_peak:.1f} m  (change: +{ek_peak-ek_pre:.1f} m)")
    print(f"  Overall accuracy (full 600 s run, from benchmark):")
    print(f"    FusionCore  ATE 5.6 m   RL-EKF  ATE 13.0 m   RL-UKF: NaN at t=31 s")

    gt_mask = (gt_ts >= fc_ts[0]) & (gt_ts <= fc_ts[-1])
    all_traj_x = np.concatenate([fc_al[:, 0], ek_al[:, 0], gt_x[gt_mask]])
    all_traj_y = np.concatenate([fc_al[:, 1], ek_al[:, 1], gt_y[gt_mask]])
    pad   = 20
    xlo   = all_traj_x.min() - pad
    xhi   = all_traj_x.max() + pad
    ylo   = all_traj_y.min() - pad
    yhi   = all_traj_y.max() + pad

    spike_gt_x, spike_gt_y = interp_gt(
        np.array([fc_ts[0] + SPIKE_REL]), gt_ts, gt_x, gt_y)

    fig = plt.figure(figsize=(20, 9), facecolor=BG)
    gs  = fig.add_gridspec(1, 2, left=0.04, right=0.98,
                           top=0.88, bottom=0.08, wspace=0.09)
    ax_traj = fig.add_subplot(gs[0])
    ax_err  = fig.add_subplot(gs[1])

    fig.text(0.5, 0.965,
             'FusionCore GPS spike rejection  |  NCLT 2012-01-08',
             ha='center', fontsize=17, fontweight='bold', color=TEXT)
    fig.text(0.5, 0.945,
             '707 m corrupted GPS fix injected at t=120 s. '
             'FusionCore chi-squared gate blocks it. RL-EKF accepts it.',
             ha='center', fontsize=11, color=MUTED)

    # ── Panel 1: trajectory ────────────────────────────────────────────────────
    ax = ax_traj
    ax.set_facecolor(BG)
    ax.grid(color=BORDER, lw=0.7, zorder=0)
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)
    ax.set_aspect('equal')
    ax.set_xlabel('East (m)', fontsize=10, color=MUTED)
    ax.set_ylabel('North (m)', fontsize=10, color=MUTED)
    ax.set_title('Trajectory  |  pre-spike SE(2) aligned to RTK ground truth',
                 fontsize=11, color=MUTED, pad=6)

    ax.plot(gt_x[gt_mask], gt_y[gt_mask],
            color=C_GT, lw=1.6, ls='--', alpha=0.8,
            label='Ground truth (RTK GPS)', zorder=3)
    ax.plot(ek_al[:, 0], ek_al[:, 1],
            color=C_EKF, lw=2.0, alpha=0.85,
            label='robot_localization EKF', zorder=4)
    ax.plot(fc_al[:, 0], fc_al[:, 1],
            color=C_FC,  lw=2.2, alpha=0.90,
            label='FusionCore', zorder=5)

    ax.plot(fc_al[0, 0], fc_al[0, 1], 'o', color=TEXT, ms=7, zorder=6)

    if not np.isnan(spike_gt_x[0]):
        ax.annotate(
            f'GPS spike\ninjected here\n(t={SPIKE_REL:.0f} s)',
            xy=(spike_gt_x[0], spike_gt_y[0]),
            xytext=(spike_gt_x[0] + 40, spike_gt_y[0] + 30),
            fontsize=9, color=TEXT,
            arrowprops=dict(arrowstyle='->', color=C_SPIKE_LINE, lw=1.8),
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#FEF3C7',
                      edgecolor=C_SPIKE_LINE, alpha=0.95),
            zorder=8,
        )
        ax.plot(spike_gt_x[0], spike_gt_y[0], '*',
                color=C_SPIKE_LINE, ms=14, zorder=7, label='Spike injection point')

    ek_spike_mask = (ek_rel >= SPIKE_REL - 5) & (ek_rel <= SPIKE_REL + 90)
    if ek_spike_mask.any():
        ax.annotate(
            f'EKF detour\n(accepted fake fix)',
            xy=(ek_al[ek_spike_mask, 0].mean(), ek_al[ek_spike_mask, 1].mean()),
            xytext=(ek_al[ek_spike_mask, 0].mean() - 60,
                    ek_al[ek_spike_mask, 1].mean() - 30),
            fontsize=9, color=C_EKF,
            arrowprops=dict(arrowstyle='->', color=C_EKF, lw=1.5),
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#FEE2E2',
                      edgecolor=C_EKF, alpha=0.90),
            zorder=8,
        )

    leg = ax.legend(fontsize=10, loc='lower left',
                    facecolor='white', edgecolor=BORDER, framealpha=1)
    for t in leg.get_texts():
        t.set_color(TEXT)

    # ── Panel 2: error time-series ─────────────────────────────────────────────
    ax = ax_err
    ax.set_facecolor(BG)
    ax.grid(color=BORDER, lw=0.7, zorder=0)
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER)
    ax.tick_params(colors=MUTED, labelsize=9)

    def smooth(arr, w=80):
        kernel = np.ones(w) / w
        out = np.convolve(arr, kernel, mode='same')
        # fix edge artifacts: replace first/last w//2 points with unsmoothed median
        half = w // 2
        out[:half] = np.median(arr[:half])
        out[-half:] = np.median(arr[-half:])
        return out

    ax.plot(ek_times, smooth(ek_errs), color=C_EKF, lw=2.2, alpha=0.90,
            label=f'RL-EKF  (peak {ek_peak:.0f} m after spike)', zorder=4)
    ax.plot(fc_times, smooth(fc_errs), color=C_FC,  lw=2.2, alpha=0.90,
            label=f'FusionCore  (peak {fc_peak:.0f} m after spike)', zorder=5)

    ax.axvline(SPIKE_REL, color=C_SPIKE_LINE, lw=2.0, ls='--', zorder=3,
               label=f'Spike injected at t={SPIKE_REL:.0f} s (707 m NE)')

    ax.set_xlabel('Time (s)', fontsize=10, color=MUTED)
    ax.set_ylabel('Position error vs RTK ground truth (m)', fontsize=10, color=MUTED)
    ax.set_title('Error over time  |  pre-spike SE(2) aligned',
                 fontsize=11, color=MUTED, pad=6)
    ax.set_xlim(0, max(fc_times[-1], ek_times[-1]))
    ax.set_ylim(0, min(ek_peak * 1.5, 30))

    ax.annotate(
        f'FusionCore: +{fc_peak-fc_pre:.1f} m\n(spike REJECTED)',
        xy=(SPIKE_REL + 8, fc_peak),
        xytext=(SPIKE_REL + 40, fc_peak + 3),
        fontsize=9, color=C_FC,
        arrowprops=dict(arrowstyle='->', color=C_FC, lw=1.5),
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#DBEAFE', edgecolor=C_FC),
        zorder=7,
    )
    ax.annotate(
        f'RL-EKF: +{ek_peak-ek_pre:.1f} m\n(spike ACCEPTED)',
        xy=(SPIKE_REL + 8, ek_peak),
        xytext=(SPIKE_REL + 40, ek_peak - 4),
        fontsize=9, color=C_EKF,
        arrowprops=dict(arrowstyle='->', color=C_EKF, lw=1.5),
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#FEE2E2', edgecolor=C_EKF),
        zorder=7,
    )

    stat_txt = (
        "Full 600 s benchmark (normal GPS):\n"
        "  FusionCore   ATE  5.6 m\n"
        "  RL-EKF       ATE 13.0 m\n"
        "  RL-UKF       NaN at t=31 s"
    )
    ax.text(0.98, 0.04, stat_txt,
            transform=ax.transAxes, fontsize=9, color=TEXT,
            va='bottom', ha='right', family='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#F8FAFC',
                      edgecolor=BORDER, alpha=0.95))

    leg2 = ax.legend(fontsize=10, loc='upper left',
                     facecolor='white', edgecolor=BORDER, framealpha=1)
    for t in leg2.get_texts():
        t.set_color(TEXT)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(out_path), dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close(fig)

    print(f"\n  Saved: {out_path}")

    if open_after:
        try:
            if sys.platform.startswith('linux'):
                subprocess.Popen(['xdg-open', str(out_path)])
            elif sys.platform == 'darwin':
                subprocess.Popen(['open', str(out_path)])
            elif sys.platform == 'win32':
                os.startfile(str(out_path))
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--out', default=None,
                        help='Output PNG path (default: demo_result.png in repo root)')
    parser.add_argument('--open', action='store_true',
                        help='Open the result image when done')
    args = parser.parse_args()

    out_path = Path(args.out) if args.out else REPO_ROOT / 'demo_result.png'
    run(out_path, args.open)


if __name__ == '__main__':
    main()
