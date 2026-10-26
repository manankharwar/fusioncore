#!/usr/bin/env python3
"""
Regenerate the two paper figures that contain version-specific content.

  fig_state_vector.png  -- 23-state diagram (was 22, missing b_ewz)
  fig_trajectory.png    -- full-run 2012-01-08 trajectory (ATE 18.6 m)

Run from the repo root:
  python3 tools/make_paper_figures.py
"""

import copy
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
import numpy as np
import argparse
import sys

OUT = Path('paper')

# ── Palette ──────────────────────────────────────────────────────────────────
FC_C  = '#0072B2'
RL_C  = '#D55E00'
GT_C  = '#555555'

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 10,
})


# ── Figure 1: 23-State vector diagram ────────────────────────────────────────

def make_state_vector():
    groups = [
        ('Position',     ['p_x\n[0]', 'p_y\n[1]', 'p_z\n[2]'],               '#1e3a5f', '#c7d9f0'),
        ('Orientation',  ['q_w\n[3]', 'q_x\n[4]', 'q_y\n[5]', 'q_z\n[6]'],  '#1a4f7a', '#a8cce8'),
        ('Lin. Velocity',['v_x\n[7]', 'v_y\n[8]', 'v_z\n[9]'],               '#0e6b6b', '#a8dede'),
        ('Ang. Velocity',['ω_x\n[10]','ω_y\n[11]','ω_z\n[12]'],              '#0d7a7a', '#90d4d4'),
        ('Lin. Accel.',  ['a_x\n[13]','a_y\n[14]','a_z\n[15]'],              '#1a6b3a', '#a8dab5'),
        ('Gyro Bias',    ['b_gx\n[16]','b_gy\n[17]','b_gz\n[18]'],           '#2d7a2d', '#88cc88'),
        ('Accel. Bias',  ['b_ax\n[19]','b_ay\n[20]','b_az\n[21]'],           '#3d8c3d', '#a0d4a0'),
        ('Enc. WZ Bias', ['b_ewz\n[22]'],                                      '#7b3f00', '#f5c89a'),
    ]

    subtitles = [
        'x, y, z (m)\nENU frame',
        'Unit quaternion\n(SO3 manifold)',
        'Body frame\n(m/s)',
        'Body frame\n(rad/s)',
        'Body frame\n(m/s²)',
        'Estimated online\n(rad/s)',
        'Estimated online\n(m/s²)',
        'Estimated online\n(rad/s)\nvia GPS heading',
    ]

    box_w = 0.9
    box_h = 1.0
    gap_x = 0.25
    group_gap = 0.55

    # compute total width
    total_states = sum(len(g[1]) for g in groups)
    total_gaps = len(groups) - 1
    fig_w = total_states * box_w + (total_states - len(groups)) * gap_x + total_gaps * group_gap + 2.0

    fig, ax = plt.subplots(figsize=(fig_w, 6.5))
    ax.set_xlim(0, fig_w)
    ax.set_ylim(-3.0, 3.5)
    ax.axis('off')

    ax.text(fig_w / 2, 3.2,
            'FusionCore 23-Dimensional UKF State Vector',
            ha='center', va='center', fontsize=15, fontweight='bold',
            color='#1a1a2e')
    ax.text(fig_w / 2, 2.75,
            'State indices [0] → [22]  ·  Orientation stored as unit quaternion  ·  '
            'Biases estimated online at 100 Hz',
            ha='center', va='center', fontsize=9, color='#555555', style='italic')

    x = 0.7
    group_centers = []
    for g_idx, (group_name, states, dark, light) in enumerate(groups):
        g_start = x
        for s_idx, label in enumerate(states):
            rect = mpatches.FancyBboxPatch(
                (x, 1.0), box_w, box_h,
                boxstyle='round,pad=0.06',
                linewidth=1.4 if g_idx < 7 else 2.2,
                edgecolor=dark,
                facecolor=light,
                zorder=3)
            ax.add_patch(rect)
            ax.text(x + box_w / 2, 1.0 + box_h / 2, label,
                    ha='center', va='center', fontsize=8.5,
                    fontweight='bold', color=dark, zorder=4,
                    fontfamily='monospace')
            x += box_w + gap_x
        g_center = (g_start + x - gap_x) / 2
        group_centers.append((g_center, g_start, x - gap_x + box_w, dark, light, group_name))
        x += group_gap - gap_x

    # group label bars below boxes
    bar_y = 0.72
    bar_h = 0.22
    for (gc, g_start, g_end, dark, light, name) in group_centers:
        rect = mpatches.FancyBboxPatch(
            (g_start, bar_y), g_end - g_start, bar_h,
            boxstyle='round,pad=0.04',
            linewidth=0, facecolor=dark, zorder=2)
        ax.add_patch(rect)
        ax.text(gc, bar_y + bar_h / 2, name,
                ha='center', va='center', fontsize=8.5,
                fontweight='bold', color='white', zorder=3)

    # connector lines
    for (gc, g_start, g_end, dark, light, name) in group_centers:
        ax.plot([gc, gc], [1.0, bar_y + bar_h],
                color=dark, lw=1.0, zorder=1)

    # subtitles
    for i, ((gc, g_start, g_end, dark, light, name), sub) in enumerate(zip(group_centers, subtitles)):
        color = '#8b4513' if i == 7 else '#666666'
        weight = 'bold' if i == 7 else 'normal'
        ax.text(gc, 0.55, sub,
                ha='center', va='top', fontsize=7.5,
                color=color, fontweight=weight,
                multialignment='center')

    # b_ewz callout box
    ewz_center = group_centers[7][0]
    ax.annotate(
        '23rd state: systematic ω_z offset\nfrom wheel radius mismatch.\nIdentified via GPS heading\ncross-covariance; subtracted\nduring GPS blackouts.',
        xy=(ewz_center, 1.0),
        xytext=(ewz_center - 1.8, -2.2),
        fontsize=7.8, color='#7b3f00',
        ha='center', va='top',
        bbox=dict(boxstyle='round,pad=0.4', fc='#fff5eb', ec='#f5c89a', lw=1.5),
        arrowprops=dict(arrowstyle='->', color='#7b3f00', lw=1.4,
                        connectionstyle='arc3,rad=0.15'),
        zorder=5)

    plt.tight_layout(pad=0.4)
    out = OUT / 'fig_state_vector.png'
    fig.savefig(out, dpi=180, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {out}')


# ── Figure 2: Full-run trajectory 2012-01-08 ─────────────────────────────────

def make_trajectory():
    seq = '2012-01-08'
    d   = Path('benchmarks/nclt') / seq

    try:
        from evo.tools import file_interface
        from evo.core  import sync
    except ImportError:
        print('  evo not available, skipping trajectory figure')
        return

    try:
        traj_gt = file_interface.read_tum_trajectory_file(str(d / 'ground_truth.tum'))
        traj_fc = file_interface.read_tum_trajectory_file(str(d / 'fusioncore.tum'))
        traj_rl = file_interface.read_tum_trajectory_file(str(d / 'rl_ekf.tum'))
    except Exception as e:
        print(f'  could not load tum files: {e}')
        return

    def align_and_xy(traj_est, traj_ref, max_pts=4000):
        ref_s, est_s = sync.associate_trajectories(traj_ref, traj_est, max_diff=1.0)
        if len(ref_s.timestamps) < 10:
            xy = traj_est.positions_xyz[:, :2]
            return xy[::max(1, len(xy) // max_pts)] - xy[0]
        est_copy = copy.deepcopy(est_s)
        R, t, _ = est_copy.align(ref_s, correct_scale=False)
        pos = traj_est.positions_xyz
        aligned = (R @ pos.T + t.reshape(3, 1)).T
        xy = aligned[:, :2]
        step = max(1, len(xy) // max_pts)
        return xy[::step]

    gt_pos = traj_gt.positions_xyz[:, :2]
    # Remove single-point spikes: keep only points where the jump to/from neighbors < 80m
    dists = np.linalg.norm(np.diff(gt_pos, axis=0), axis=1)
    spike_mask = np.ones(len(gt_pos), dtype=bool)
    for i in range(1, len(gt_pos) - 1):
        if dists[i-1] > 80 and dists[i] > 80:
            spike_mask[i] = False
    gt_pos = gt_pos[spike_mask]
    step  = max(1, len(gt_pos) // 4000)
    gt_xy = gt_pos[::step]
    gt_xy = gt_xy - gt_xy[0]

    fc_xy = align_and_xy(traj_fc, traj_gt)
    rl_xy = align_and_xy(traj_rl, traj_gt)

    fig, ax = plt.subplots(figsize=(8, 7))

    ax.plot(gt_xy[:, 0], gt_xy[:, 1], color=GT_C, lw=2.0,
            label='Ground Truth (RTK GPS)', zorder=1, alpha=0.75,
            linestyle='--', solid_capstyle='round')
    ax.plot(rl_xy[:, 0], rl_xy[:, 1], color=RL_C, lw=1.2,
            label=f'robot_localization EKF  (ATE 41.2 m)', zorder=2, alpha=0.75,
            solid_capstyle='round')
    ax.plot(fc_xy[:, 0], fc_xy[:, 1], color=FC_C, lw=1.8,
            label=f'FusionCore  (ATE 18.6 m)', zorder=3, alpha=0.95,
            solid_capstyle='round')
    ax.plot(0, 0, 'o', color='black', ms=7, zorder=7, label='Start')

    ax.set_title('NCLT 2012-01-08 — FusionCore vs RTK Ground Truth\n'
                 'Full sequence: 92 min, single config, no tuning',
                 fontsize=12, fontweight='bold', pad=10)
    ax.set_xlabel('East (m)', fontsize=11)
    ax.set_ylabel('North (m)', fontsize=11)
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, linestyle='--', alpha=0.25)
    ax.legend(loc='lower left', fontsize=9.5, framealpha=0.92,
              edgecolor='#cccccc')

    plt.tight_layout()
    out = OUT / 'fig_trajectory.png'
    fig.savefig(out, dpi=180, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {out}')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.parse_args(argv)
    print('Generating paper figures...\n')
    make_state_vector()
    make_trajectory()
    print('\nDone.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
