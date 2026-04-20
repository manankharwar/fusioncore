#!/usr/bin/env python3
"""
FusionCore NCLT benchmark visualizer.
Outputs one PNG per result — each is self-contained and presentation-ready.

Usage:
  python3 tools/plot_benchmark.py --seq_dir benchmarks/nclt/2012-01-08
"""

import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

# ── Palette ────────────────────────────────────────────────────────────────
BG     = '#FFFFFF'
PANEL  = '#F8FAFC'
BORDER = '#E2E8F0'
TEXT   = '#0F172A'
MUTED  = '#64748B'
C_FC   = '#2563EB'
C_EKF  = '#DC2626'
C_UKF  = '#7C3AED'
C_GT   = '#94A3B8'


# ── Helpers ────────────────────────────────────────────────────────────────
def load_tum(path):
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


def align_se2_temporal(src_ts, src_xy, ref_ts, ref_xy):
    """SE(2) alignment using timestamp-matched pairs (correct for partial trajectories)."""
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


def interp_error_2d(est_ts, est_x, est_y, gt_ts, gt_x, gt_y):
    errs = np.full(len(est_ts), np.nan)
    for i, t in enumerate(est_ts):
        idx = np.searchsorted(gt_ts, t)
        if idx == 0 or idx >= len(gt_ts):
            continue
        t0, t1 = gt_ts[idx - 1], gt_ts[idx]
        if t1 == t0:
            continue
        a = (t - t0) / (t1 - t0)
        gx = gt_x[idx - 1] + a * (gt_x[idx] - gt_x[idx - 1])
        gy = gt_y[idx - 1] + a * (gt_y[idx] - gt_y[idx - 1])
        errs[i] = math.hypot(est_x[i] - gx, est_y[i] - gy)
    return errs


def base_fig(w=12, h=7.5):
    fig, ax = plt.subplots(figsize=(w, h), facecolor=BG)
    ax.set_facecolor(BG)
    ax.tick_params(colors=MUTED, labelsize=10)
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER)
    return fig, ax


def set_titles(fig, title, subtitle):
    fig.text(0.5, 0.96, title, ha='center', fontsize=17,
             fontweight='bold', color=TEXT)
    fig.text(0.5, 0.915, subtitle, ha='center', fontsize=10.5, color=MUTED)


def badge(ax, x, y, text, good=True, size=10):
    bg = '#DCFCE7' if good else '#FEE2E2'
    tc = '#15803D' if good else '#B91C1C'
    ax.text(x, y, text, transform=ax.transAxes, fontsize=size,
            color=tc, fontweight='bold', va='top',
            bbox=dict(boxstyle='round,pad=0.4', facecolor=bg, edgecolor='none'))


def save(fig, path):
    fig.savefig(str(path), dpi=160, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f'  → {path}')


# ── Chart 1: Trajectory (two stacked panels) ─────────────────────────────
def plot_trajectory(seq, out_dir):
    gt_ts, gt_x, gt_y, _ = load_tum(str(seq / 'ground_truth.tum'))
    fc_ts, fc_x, fc_y, _ = load_tum(str(seq / 'fusioncore.tum'))
    ek_ts, ek_x, ek_y, _ = load_tum(str(seq / 'rl_ekf.tum'))

    gt_xy = np.stack([gt_x, gt_y], 1)
    fc_al = align_se2_temporal(fc_ts, np.stack([fc_x, fc_y], 1), gt_ts, gt_xy)
    ek_al = align_se2_temporal(ek_ts, np.stack([ek_x, ek_y], 1), gt_ts, gt_xy)

    t_end = max(fc_ts[-1], ek_ts[-1])
    gt_mask = gt_ts <= t_end

    pad = 60
    all_x = np.concatenate([fc_al[:, 0], ek_al[:, 0], gt_x[gt_mask]])
    all_y = np.concatenate([fc_al[:, 1], ek_al[:, 1], gt_y[gt_mask]])
    xlo, xhi = all_x.min() - pad, all_x.max() + pad
    ylo, yhi = all_y.min() - pad, all_y.max() + pad

    fig, (ax_top, ax_bot) = plt.subplots(1, 2, figsize=(18, 10), facecolor=BG)
    fig.subplots_adjust(left=0.06, right=0.97, top=0.93, bottom=0.07, wspace=0.08)

    fig.text(0.5, 0.97, 'Route Accuracy — 600 s Campus Drive',
             ha='center', fontsize=17, fontweight='bold', color=TEXT)
    fig.text(0.5, 0.955, 'NCLT 2012-01-08  •  RTK GPS ground truth  •  SE(2) aligned',
             ha='center', fontsize=10.5, color=MUTED)

    def style_ax(ax):
        ax.set_facecolor(BG)
        ax.tick_params(colors=MUTED, labelsize=9)
        for sp in ax.spines.values():
            sp.set_edgecolor(BORDER)
        ax.set_xlim(xlo, xhi)
        ax.set_ylim(ylo, yhi)
        ax.set_aspect('equal')
        ax.grid(color=BORDER, lw=0.7, zorder=0)

    def plot_gt(ax):
        ax.plot(gt_x[gt_mask], gt_y[gt_mask],
                color='#111827', lw=1.8, ls='--', alpha=0.85,
                label='Ground Truth (RTK GPS)', zorder=3)

    # Top panel — RL-EKF
    style_ax(ax_top)
    ax_top.plot(ek_al[:, 0], ek_al[:, 1],
                color=C_EKF, lw=2.0, alpha=0.85,
                label='RL-EKF  (ATE 23.4 m)', zorder=2)
    plot_gt(ax_top)
    ax_top.plot(ek_al[0, 0], ek_al[0, 1], 'o', color=TEXT, ms=6, zorder=5)
    ax_top.set_ylabel('North (m)', fontsize=10, color=MUTED)
    ax_top.tick_params(labelbottom=True)
    leg = ax_top.legend(fontsize=10, loc='upper left',
                        facecolor='white', edgecolor=BORDER, framealpha=1)
    for t in leg.get_texts(): t.set_color(TEXT)

    # Bottom panel — FusionCore
    style_ax(ax_bot)
    ax_bot.plot(fc_al[:, 0], fc_al[:, 1],
                color=C_FC, lw=2.0, alpha=0.85,
                label='FusionCore  (ATE 5.5 m)', zorder=2)
    plot_gt(ax_bot)
    ax_bot.plot(fc_al[0, 0], fc_al[0, 1], 'o', color=TEXT, ms=6, zorder=5)
    ax_bot.set_xlabel('East (m)', fontsize=10, color=MUTED)
    ax_bot.tick_params(labelleft=False)
    leg = ax_bot.legend(fontsize=10, loc='upper left',
                        facecolor='white', edgecolor=BORDER, framealpha=1)
    for t in leg.get_texts(): t.set_color(TEXT)

    save(fig, out_dir / '01_trajectory.png')


# ── Chart 2: ATE bar chart ────────────────────────────────────────────────
def plot_ate(out_dir):
    fig, ax = base_fig(9, 7)
    fig.subplots_adjust(left=0.12, right=0.82, top=0.86, bottom=0.12)
    set_titles(fig,
               'FusionCore is 4.2× More Accurate',
               'Absolute Trajectory Error (ATE RMSE)  •  lower is better')

    names  = ['FusionCore', 'RL-EKF']
    vals   = [5.517, 23.434]
    colors = [C_FC, C_EKF]
    alphas = [1.0, 0.85]

    bars = ax.bar(names, vals, color=colors, width=0.45, zorder=3,
                  alpha=1.0, edgecolor='none')

    # value labels on top of bars
    for bar, v in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 0.3,
                f'{v:.1f} m', ha='center', va='bottom',
                color=TEXT, fontsize=15, fontweight='bold')

    # improvement arrow
    x_arrow = 1.38
    ax.annotate('', xy=(x_arrow, vals[0]), xytext=(x_arrow, vals[1]),
                xycoords=('data', 'data'),
                arrowprops=dict(arrowstyle='<->', color=MUTED, lw=1.8))
    ax.text(x_arrow + 0.06, (vals[0] + vals[1]) / 2,
            '4.2×\nmore\naccurate', ha='left', va='center',
            color=TEXT, fontsize=12, fontweight='bold', linespacing=1.4,
            transform=ax.get_xaxis_transform() if False else ax.transData)

    ax.set_ylim(0, vals[1] * 1.3)
    ax.set_xlim(-0.6, 1.9)
    ax.set_ylabel('ATE RMSE (m)', fontsize=11, color=MUTED)
    ax.tick_params(axis='x', labelsize=13, colors=TEXT)
    ax.grid(axis='y', color=BORDER, lw=0.7, zorder=0)
    ax.set_axisbelow(True)

    badge(ax, 0.04, 0.97, '✓  Winner', good=True, size=10)

    save(fig, out_dir / '02_ate.png')


# ── Chart 3: GPS spike ────────────────────────────────────────────────────
def plot_spike(seq, out_dir):
    gt_ts, gt_x, gt_y, _ = load_tum(str(seq / 'ground_truth.tum'))
    fc_ts, fc_x, fc_y, _ = load_tum(str(seq / 'fusioncore_spike.tum'))
    ek_ts, ek_x, ek_y, _ = load_tum(str(seq / 'rl_ekf_spike.tum'))

    SPIKE_T = 120.0
    t0 = gt_ts[0]

    def rel_errs(ts, x, y):
        rel = ts - t0
        errs = interp_error_2d(ts, x, y, gt_ts, gt_x, gt_y)
        mask = (rel >= SPIKE_T - 35) & (rel <= SPIKE_T + 50)
        return rel[mask] - SPIKE_T, errs[mask]

    fig, ax = base_fig(12, 7)
    fig.subplots_adjust(left=0.1, right=0.96, top=0.86, bottom=0.13)
    set_titles(fig,
               'FusionCore Rejects Corrupted GPS — RL-EKF Jumps 93 m',
               'A single GPS fix was corrupted to +707 m NE  •  injected at t = 120 s')

    fc_t, fc_e = rel_errs(fc_ts, fc_x, fc_y)
    ek_t, ek_e = rel_errs(ek_ts, ek_x, ek_y)

    ax.fill_between(ek_t, ek_e, alpha=0.12, color=C_EKF)
    ax.plot(ek_t, ek_e, color=C_EKF, lw=2.2, label='RL-EKF — accepted fake fix, jumped 93 m')
    ax.plot(fc_t, fc_e, color=C_FC,  lw=2.5, label='FusionCore — Mahalanobis gate blocked spike')

    # spike line
    ax.axvline(0, color='#EF4444', lw=1.8, ls='--', alpha=0.85, zorder=5)
    ymax = np.nanmax(ek_e) if len(ek_e) else 100
    ax.text(1.5, ymax * 0.97, '← 707 m fake fix\n   injected here',
            color='#EF4444', fontsize=9.5, va='top', linespacing=1.5)

    ax.set_xlabel('Seconds relative to spike injection', fontsize=11, color=MUTED)
    ax.set_ylabel('Position error vs ground truth (m)', fontsize=11, color=MUTED)
    ax.set_ylim(bottom=0)
    ax.grid(color=BORDER, lw=0.7, zorder=0)
    ax.set_axisbelow(True)

    leg = ax.legend(fontsize=11, loc='upper left',
                    facecolor='white', edgecolor=BORDER, framealpha=1)
    for t in leg.get_texts():
        t.set_color(TEXT)

    badge(ax, 0.01, 0.97, '✓  FusionCore: +1 m  (REJECTED)', good=True,  size=10)
    badge(ax, 0.01, 0.84, '✗  RL-EKF: +93 m  (JUMPED)',      good=False, size=10)

    save(fig, out_dir / '03_spike.png')


# ── Chart 4: RL-UKF divergence ────────────────────────────────────────────
def plot_ukf(seq, out_dir):
    gt_ts, gt_x, gt_y, _ = load_tum(str(seq / 'ground_truth.tum'))
    uk_ts, uk_x, uk_y, _ = load_tum(str(seq / 'rl_ukf.tum'))
    fc_ts, fc_x, fc_y, _ = load_tum(str(seq / 'fusioncore.tum'))

    fig, ax = base_fig(12, 7)
    fig.subplots_adjust(left=0.12, right=0.96, top=0.86, bottom=0.13)
    set_titles(fig,
               'RL-UKF Numerically Diverges at t = 31 s',
               'FusionCore ran stably for 600 s on identical IMU data  •  RL-UKF published NaN from t = 31 s onward')

    t0 = gt_ts[0]

    # FC error vs GT (first 90s for context)
    fc_rel = fc_ts - t0
    fc_err = interp_error_2d(fc_ts, fc_x, fc_y, gt_ts, gt_x, gt_y)
    mask90 = fc_rel <= 90
    ax.plot(fc_rel[mask90], fc_err[mask90], color=C_FC, lw=2.2,
            label='FusionCore — stays on route', zorder=3)

    # UKF: only plot pre-divergence poses (magnitude < 1000 m is sane)
    uk_rel = uk_ts - t0
    valid = np.hypot(uk_x, uk_y) < 1000
    uk_err = interp_error_2d(uk_ts[valid], uk_x[valid], uk_y[valid], gt_ts, gt_x, gt_y)
    die_t = float(uk_rel[valid][-1]) if valid.any() else 31.0

    ax.plot(uk_rel[valid], uk_err, color=C_UKF, lw=2.2,
            label='RL-UKF — valid output before divergence', zorder=2)

    # "Dead zone" shading after divergence
    ax.axvspan(die_t, 90, color='#FEE2E2', alpha=0.35, zorder=0)
    ax.axvline(die_t, color='#EF4444', lw=2.0, ls='--', zorder=5)

    # Place annotation after axes are drawn so y-limits are known
    ax.set_xlim(0, 90)
    ax.set_ylim(bottom=0)
    ax.figure.canvas.draw()
    ymax = ax.get_ylim()[1]
    ax.text(die_t + 2, ymax * 0.55,
            f'RL-UKF dies\nt = {die_t:.0f} s\n\nAll subsequent\noutput: NaN',
            color='#B91C1C', fontsize=9.5, va='center', linespacing=1.6)

    ax.set_xlabel('Time (s)', fontsize=11, color=MUTED)
    ax.set_ylabel('Position error vs ground truth (m)', fontsize=11, color=MUTED)
    ax.grid(color=BORDER, lw=0.7, zorder=0)
    ax.set_axisbelow(True)

    leg = ax.legend(fontsize=11, loc='upper left',
                    facecolor='white', edgecolor=BORDER, framealpha=1)
    for t in leg.get_texts():
        t.set_color(TEXT)

    badge(ax, 0.01, 0.97, '✗  RL-UKF: dead in 31 s — NaN explosion', good=False, size=10)
    badge(ax, 0.01, 0.84, '✓  FusionCore: stable for 600 s', good=True, size=10)

    save(fig, out_dir / '04_ukf_divergence.png')


# ── Entry point ───────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seq_dir', default='benchmarks/nclt/2012-01-08')
    parser.add_argument('--out_dir', default=None)
    args = parser.parse_args()

    seq     = Path(args.seq_dir)
    out_dir = Path(args.out_dir) if args.out_dir else seq / 'results'
    out_dir.mkdir(parents=True, exist_ok=True)

    print('Generating benchmark charts...')
    plot_trajectory(seq, out_dir)
    plot_ate(out_dir)
    plot_spike(seq, out_dir)
    plot_ukf(seq, out_dir)
    print(f'Done. All charts saved to {out_dir}/')


if __name__ == '__main__':
    main()
