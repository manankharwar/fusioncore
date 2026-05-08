#!/usr/bin/env python3
"""
FusionCore quick demo -- no ROS required.

Generates a comparison plot from pre-baked NCLT benchmark results included
in the repository. Shows FusionCore vs robot_localization EKF vs RTK GPS
ground truth on the 2012-01-08 sequence (600 second campus drive).

Usage:
  python3 tools/demo_quick.py
  python3 tools/demo_quick.py --open      # open image when done
  python3 tools/demo_quick.py --out /tmp/result.png

Requirements: numpy, matplotlib (pip install numpy matplotlib)
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
except ImportError:
    sys.exit("Missing dependencies. Run:  pip install numpy matplotlib")

# ── locate seq dir relative to this script ────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT  = SCRIPT_DIR.parent
SEQ_DIR    = REPO_ROOT / 'benchmarks' / 'nclt' / '2012-01-08'

# ── colour palette (matches the full benchmark plots) ─────────────────────────
BG     = '#FFFFFF'
BORDER = '#E2E8F0'
TEXT   = '#0F172A'
MUTED  = '#64748B'
C_FC   = '#2563EB'   # blue
C_EKF  = '#DC2626'   # red
C_GT   = '#94A3B8'   # grey


# ── helpers ───────────────────────────────────────────────────────────────────

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


def se2_align(src_ts, src_xy, ref_ts, ref_xy):
    step = max(1, len(src_ts) // 2000)
    s_pts, r_pts = [], []
    for i in range(0, len(src_ts), step):
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


def ate_rmse(est_ts, est_x, est_y, gt_ts, gt_x, gt_y):
    errs = []
    for i, t in enumerate(est_ts):
        idx = np.searchsorted(gt_ts, t)
        if idx == 0 or idx >= len(gt_ts):
            continue
        t0, t1 = gt_ts[idx - 1], gt_ts[idx]
        if t1 == t0:
            continue
        a = (t - t0) / (t1 - t0)
        gx = gt_x[idx-1] + a * (gt_x[idx] - gt_x[idx-1])
        gy = gt_y[idx-1] + a * (gt_y[idx] - gt_y[idx-1])
        errs.append(math.hypot(est_x[i] - gx, est_y[i] - gy))
    return math.sqrt(sum(e**2 for e in errs) / len(errs)) if errs else float('nan')


# ── main plot ─────────────────────────────────────────────────────────────────

def run(out_path: Path, open_after: bool, live_tum: Path = None):
    using_live = live_tum is not None and Path(live_tum).exists()
    mode_str   = "live run" if using_live else "pre-baked results"
    print(f"FusionCore demo -- NCLT 2012-01-08 ({mode_str})")
    print(f"  Loading trajectories from {SEQ_DIR.relative_to(REPO_ROOT)} ...")

    gt_ts, gt_x, gt_y, _ = load_tum(SEQ_DIR / 'ground_truth.tum')
    fc_tum_path = Path(live_tum) if using_live else SEQ_DIR / 'fusioncore.tum'
    fc_ts, fc_x, fc_y, _ = load_tum(fc_tum_path)
    ek_ts, ek_x, ek_y, _ = load_tum(SEQ_DIR / 'rl_ekf.tum')

    if using_live:
        print(f"  Using live FusionCore output: {live_tum}")

    gt_xy = np.stack([gt_x, gt_y], 1)
    fc_al = se2_align(fc_ts, np.stack([fc_x, fc_y], 1), gt_ts, gt_xy)
    ek_al = se2_align(ek_ts, np.stack([ek_x, ek_y], 1), gt_ts, gt_xy)

    fc_ate = ate_rmse(fc_ts, fc_al[:, 0], fc_al[:, 1], gt_ts, gt_x, gt_y)
    ek_ate = ate_rmse(ek_ts, ek_al[:, 0], ek_al[:, 1], gt_ts, gt_x, gt_y)

    print(f"  FusionCore  ATE RMSE: {fc_ate:.2f} m")
    print(f"  RL-EKF      ATE RMSE: {ek_ate:.2f} m")
    print(f"  FusionCore is {ek_ate/fc_ate:.1f}x more accurate")

    # ── figure ────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(18, 9), facecolor=BG)
    fig.subplots_adjust(left=0.05, right=0.97, top=0.91, bottom=0.07, wspace=0.06)

    t_end    = max(fc_ts[-1], ek_ts[-1])
    gt_mask  = gt_ts <= t_end
    pad      = 60
    all_x    = np.concatenate([fc_al[:, 0], ek_al[:, 0], gt_x[gt_mask]])
    all_y    = np.concatenate([fc_al[:, 1], ek_al[:, 1], gt_y[gt_mask]])
    xlo, xhi = all_x.min() - pad, all_x.max() + pad
    ylo, yhi = all_y.min() - pad, all_y.max() + pad

    fig.text(0.5, 0.97, 'FusionCore vs robot_localization -- NCLT 2012-01-08',
             ha='center', fontsize=18, fontweight='bold', color=TEXT)
    fig.text(0.5, 0.955, '600 s campus drive  |  RTK GPS ground truth  |  SE(2) aligned',
             ha='center', fontsize=11, color=MUTED)

    for ax, (traj_al, label, color, ate) in zip(axes, [
        (ek_al, f'robot_localization EKF  --  ATE {ek_ate:.1f} m', C_EKF, ek_ate),
        (fc_al, f'FusionCore  --  ATE {fc_ate:.1f} m',             C_FC,  fc_ate),
    ]):
        ax.set_facecolor(BG)
        ax.tick_params(colors=MUTED, labelsize=9)
        for sp in ax.spines.values():
            sp.set_edgecolor(BORDER)
        ax.set_xlim(xlo, xhi)
        ax.set_ylim(ylo, yhi)
        ax.set_aspect('equal')
        ax.grid(color=BORDER, lw=0.7, zorder=0)

        ax.plot(gt_x[gt_mask], gt_y[gt_mask],
                color='#111827', lw=1.8, ls='--', alpha=0.7,
                label='Ground Truth (RTK GPS)', zorder=3)
        ax.plot(traj_al[:, 0], traj_al[:, 1],
                color=color, lw=2.2, alpha=0.9,
                label=label, zorder=4)
        ax.plot(traj_al[0, 0], traj_al[0, 1], 'o', color=TEXT, ms=6, zorder=5)

        is_winner = (ate == min(fc_ate, ek_ate))
        badge_bg  = '#DCFCE7' if is_winner else '#FEE2E2'
        badge_tc  = '#15803D' if is_winner else '#B91C1C'
        badge_txt = f'{"WINNER" if is_winner else "LOSER"}  --  ATE {ate:.1f} m'
        ax.text(0.03, 0.97, badge_txt,
                transform=ax.transAxes, fontsize=10, color=badge_tc,
                fontweight='bold', va='top',
                bbox=dict(boxstyle='round,pad=0.4', facecolor=badge_bg, edgecolor='none'))

        leg = ax.legend(fontsize=10, loc='upper right',
                        facecolor='white', edgecolor=BORDER, framealpha=1)
        for t in leg.get_texts():
            t.set_color(TEXT)

        ax.set_xlabel('East (m)', fontsize=10, color=MUTED)
        if ax is axes[0]:
            ax.set_ylabel('North (m)', fontsize=10, color=MUTED)

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
    parser.add_argument('--live_tum', default=None,
                        help='Path to a live FusionCore .tum file (from run_demo.sh)')
    args = parser.parse_args()

    out_path = Path(args.out) if args.out else REPO_ROOT / 'demo_result.png'
    run(out_path, args.open, live_tum=args.live_tum)


if __name__ == '__main__':
    main()
