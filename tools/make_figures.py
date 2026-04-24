#!/usr/bin/env python3
"""
Generate publication-quality benchmark figures for FusionCore post.

Outputs (in figures/):
  fig1_bar_chart.png  : FC vs RL ATE RMSE across all 6 sequences
  fig2_traj_grid.png  : 2×3 trajectory overlay grid, all sequences
  fig3_coast_mode.png : coast mode before/after for 2012-11-04

Usage:
  python3 tools/make_figures.py
"""

import copy
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
import numpy as np

from evo.tools import file_interface
from evo.core import sync

# ── Config ────────────────────────────────────────────────────────────────────

BASE = Path('benchmarks/nclt')
OUT  = Path('figures')
OUT.mkdir(exist_ok=True)

SEQUENCES = ['2012-01-08', '2012-02-04', '2012-03-31',
             '2012-08-20', '2012-11-04', '2013-02-23']

LABELS  = ['Jan 2012\n(Winter)', 'Feb 2012\n(Winter)', 'Mar 2012\n(Spring)',
           'Aug 2012\n(Summer)', 'Nov 2012\n(Fall)',   'Feb 2013\n(Winter)']

FC_ATE  = [5.633, 9.651, 4.209, 7.451, 28.687, 4.124]
RL_ATE  = [23.487, 20.530, 10.786, 9.448, 10.901, 5.832]

# Coast mode story data for 2012-11-04
COAST_BEFORE = 61.4
COAST_AFTER  = 28.687
RL_NOV       = 10.901

# ── Palette (colorblind-safe) ─────────────────────────────────────────────────

FC_C  = '#0072B2'   # blue
RL_C  = '#D55E00'   # orange
GT_C  = '#888888'   # grey
WIN_C = '#16A34A'   # green
LOS_C = '#DC2626'   # red

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 11,
    'axes.spines.top': False,
    'axes.spines.right': False,
})


# ── Figure 1: Bar chart ───────────────────────────────────────────────────────

def make_bar_chart():
    fig, ax = plt.subplots(figsize=(12, 5.5))

    x = np.arange(len(SEQUENCES))
    w = 0.33

    bars_fc = ax.bar(x - w/2, FC_ATE, w, label='FusionCore',
                     color=FC_C, zorder=3, linewidth=0)
    bars_rl = ax.bar(x + w/2, RL_ATE, w, label='robot_localization EKF',
                     color=RL_C, zorder=3, linewidth=0)

    # Value labels
    for bar, val in zip(bars_fc, FC_ATE):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.4,
                f'{val:.1f}', ha='center', va='bottom',
                fontsize=8.5, color=FC_C, fontweight='bold')
    for bar, val in zip(bars_rl, RL_ATE):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.4,
                f'{val:.1f}', ha='center', va='bottom',
                fontsize=8.5, color=RL_C, fontweight='bold')

    # Column background + win/loss badge
    for i, (fc, rl) in enumerate(zip(FC_ATE, RL_ATE)):
        fc_wins = fc < rl
        bg  = '#F0FDF4' if fc_wins else '#FFF5F5'
        clr = WIN_C     if fc_wins else LOS_C
        txt = '↑ FC wins' if fc_wins else '↑ RL wins'
        ax.axvspan(x[i] - 0.5, x[i] + 0.5, color=bg, alpha=0.5, zorder=0)
        ax.text(x[i], -3.2, txt, ha='center', va='top',
                fontsize=8, color=clr, fontweight='bold')

    ymax = max(max(FC_ATE), max(RL_ATE))
    ax.set_ylim(-4.5, ymax * 1.22)
    ax.set_xticks(x)
    ax.set_xticklabels(LABELS, fontsize=10)
    ax.set_ylabel('ATE RMSE (m)', fontsize=11)
    ax.set_title(
        'FusionCore vs robot_localization EKF: 6 NCLT sequences, identical config, same pipeline',
        fontsize=12, fontweight='bold', pad=14)
    ax.legend(loc='upper right', fontsize=10, framealpha=0.92)
    ax.yaxis.grid(True, linestyle='--', alpha=0.35, zorder=0)
    ax.set_axisbelow(True)

    plt.tight_layout()
    out = OUT / 'fig1_bar_chart.png'
    fig.savefig(out, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  ✓ {out}')


# ── Figure 2: Trajectory grid ─────────────────────────────────────────────────

def align_to_ref(traj_est_full, traj_ref_full, max_diff=1.0, max_pts=2500):
    """
    Compute SE3 alignment from synced timestamp pairs, apply to the FULL
    est trajectory, return downsampled aligned XY (N,2).
    """
    ref_s, est_s = sync.associate_trajectories(
        traj_ref_full, traj_est_full, max_diff=max_diff)

    if len(ref_s.timestamps) < 10:
        # Too few pairs: fall back: center at origin
        xy = traj_est_full.positions_xyz[:, :2]
        xy = xy - xy[0]
        return xy[::max(1, len(xy) // max_pts)]

    # Estimate R, t from synced pairs (modifies est_s copy in-place)
    est_s_copy = copy.deepcopy(est_s)
    R, t, _ = est_s_copy.align(ref_s, correct_scale=False)

    # Apply the transform to ALL positions in the full trajectory
    pos = traj_est_full.positions_xyz          # (N, 3)
    pos_aligned = (R @ pos.T + t.reshape(3, 1)).T  # (N, 3)
    xy = pos_aligned[:, :2]

    step = max(1, len(xy) // max_pts)
    return xy[::step]


def make_traj_grid():
    fig = plt.figure(figsize=(15, 10))
    gs  = GridSpec(2, 3, figure=fig, hspace=0.44, wspace=0.32)

    for idx, (seq, lbl) in enumerate(zip(SEQUENCES, LABELS)):
        row, col = divmod(idx, 3)
        ax = fig.add_subplot(gs[row, col])

        d = BASE / seq
        try:
            traj_gt = file_interface.read_tum_trajectory_file(str(d / 'ground_truth.tum'))
            traj_fc = file_interface.read_tum_trajectory_file(str(d / 'fusioncore.tum'))
            traj_rl = file_interface.read_tum_trajectory_file(str(d / 'rl_ekf.tum'))
        except Exception:
            ax.text(0.5, 0.5, f'Missing:\n{seq}', ha='center', va='center',
                    transform=ax.transAxes, color='grey')
            ax.set_title(lbl, fontweight='bold')
            continue

        # Clip GT to the time window the filters actually ran over.
        # ground_truth.tum covers the full NCLT session (90 min);
        # the benchmark only ran ~200 s: clip so GT matches filter extent.
        t0 = min(traj_fc.timestamps[0],  traj_rl.timestamps[0])
        t1 = max(traj_fc.timestamps[-1], traj_rl.timestamps[-1])
        mask   = (traj_gt.timestamps >= t0 - 2.0) & (traj_gt.timestamps <= t1 + 2.0)
        gt_pos = traj_gt.positions_xyz[mask, :2]
        step   = max(1, len(gt_pos) // 2500)
        gt_xy  = gt_pos[::step]

        # FC and RL: align FULL trajectory to GT using SE3 from synced pairs
        fc_xy = align_to_ref(traj_fc, traj_gt)
        rl_xy = align_to_ref(traj_rl, traj_gt)

        ax.plot(gt_xy[:, 0], gt_xy[:, 1], color=GT_C, lw=2.0,
                label='Ground truth', zorder=1, alpha=0.85, solid_capstyle='round')
        ax.plot(rl_xy[:, 0], rl_xy[:, 1], color=RL_C, lw=1.2,
                label='RL-EKF', zorder=2, alpha=0.80, solid_capstyle='round')
        ax.plot(fc_xy[:, 0], fc_xy[:, 1], color=FC_C, lw=1.5,
                label='FusionCore', zorder=3, alpha=0.95, solid_capstyle='round')

        # Start dot
        ax.plot(0, 0, 'o', color='black', ms=5, zorder=7, label='Start')

        fc_ate = FC_ATE[idx]
        rl_ate = RL_ATE[idx]
        fc_wins = fc_ate < rl_ate
        wc  = WIN_C if fc_wins else LOS_C
        wtx = 'FC wins' if fc_wins else 'RL wins'

        # ATE annotation (top-left)
        ax.text(0.03, 0.97,
                f'FC  {fc_ate:.1f} m\nRL  {rl_ate:.1f} m',
                transform=ax.transAxes, va='top', ha='left', fontsize=8,
                fontfamily='monospace',
                bbox=dict(boxstyle='round,pad=0.35', fc='white', alpha=0.88,
                          ec='#CCCCCC', lw=0.8))

        # Winner badge (top-right)
        ax.text(0.97, 0.97, wtx,
                transform=ax.transAxes, va='top', ha='right', fontsize=8.5,
                fontweight='bold', color=wc,
                bbox=dict(boxstyle='round,pad=0.35', fc='white', alpha=0.88,
                          ec=wc, lw=1.2))

        ax.set_title(lbl, fontsize=11, fontweight='bold')
        ax.set_aspect('equal', adjustable='datalim')
        ax.tick_params(labelsize=7.5)
        ax.set_xlabel('East (m)', fontsize=8.5)
        ax.set_ylabel('North (m)', fontsize=8.5)
        ax.grid(True, linestyle='--', alpha=0.22, zorder=0)

    # Shared legend
    handles = [
        mpatches.Patch(color=GT_C, label='Ground truth (RTK GPS)'),
        mpatches.Patch(color=FC_C, label='FusionCore'),
        mpatches.Patch(color=RL_C, label='robot_localization EKF'),
    ]
    fig.legend(handles=handles, loc='lower center', ncol=3, fontsize=11,
               bbox_to_anchor=(0.5, -0.03), framealpha=0.95, edgecolor='#CCCCCC')

    fig.suptitle(
        'Trajectory comparison: 6 NCLT sequences, all seasons, same config, same pipeline',
        fontsize=13, fontweight='bold', y=1.02)

    out = OUT / 'fig2_traj_grid.png'
    fig.savefig(out, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  ✓ {out}')


# ── Figure 3: Coast mode before/after ────────────────────────────────────────

def make_coast_chart():
    fig, ax = plt.subplots(figsize=(7.5, 5.5))

    labels = ['No coast mode\n(original)',
              'With coast mode\n(this PR)',
              'robot_localization\nEKF (no gate)']
    values = [COAST_BEFORE, COAST_AFTER, RL_NOV]
    colors = [FC_C, FC_C, RL_C]
    alphas = [0.35, 1.0, 1.0]

    for i, (lbl, val, clr, alp) in enumerate(zip(labels, values, colors, alphas)):
        ax.bar(i, val, color=clr, alpha=alp, width=0.52, zorder=3, linewidth=0)
        label_clr = clr if alp == 1.0 else '#777777'
        ax.text(i, val + 0.8, f'{val:.1f} m', ha='center', va='bottom',
                fontsize=12, fontweight='bold', color=label_clr)

    # Improvement arrow (bar 0 → bar 1)
    ax.annotate(
        '', xy=(1, COAST_AFTER + 2), xytext=(0, COAST_BEFORE - 2),
        arrowprops=dict(arrowstyle='->', color=WIN_C, lw=2.0, connectionstyle='arc3,rad=0.0'))
    pct = (COAST_BEFORE - COAST_AFTER) / COAST_BEFORE * 100
    ax.text(0.5, (COAST_BEFORE + COAST_AFTER) / 2,
            f'−{pct:.0f}%', ha='center', va='center',
            fontsize=11, color=WIN_C, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.25', fc='white', alpha=0.9, ec=WIN_C, lw=0.8))

    # RL dashed reference line
    ax.axhline(RL_NOV, color=RL_C, linestyle='--', lw=1.2, alpha=0.5, zorder=1)

    ax.set_xticks(range(3))
    ax.set_xticklabels(labels, fontsize=10.5)
    ax.set_ylabel('ATE RMSE (m)', fontsize=11)
    ax.set_title('2012-11-04 (Fall: degraded GPS)\nInertial coast mode: what changed',
                 fontsize=12, fontweight='bold', pad=12)
    ax.set_ylim(0, COAST_BEFORE * 1.28)
    ax.yaxis.grid(True, linestyle='--', alpha=0.35, zorder=0)
    ax.set_axisbelow(True)

    # Honest footnote:
    ax.text(0.98, 0.025,
            'RL still wins on this sequence.\nNo rejection gate = immediate GPS self-correction.',
            transform=ax.transAxes, ha='right', va='bottom',
            fontsize=8.5, color='#666666', style='italic')

    plt.tight_layout()
    out = OUT / 'fig3_coast_mode.png'
    fig.savefig(out, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  ✓ {out}')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Generating figures...\n')
    make_bar_chart()
    make_traj_grid()
    make_coast_chart()
    print(f'\nAll saved to {OUT}/')
