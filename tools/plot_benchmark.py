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


def align_se2(src_xy, ref_xy):
    step = max(1, len(src_xy) // 2000)
    s, r = src_xy[::step], ref_xy[::step]
    n = min(len(s), len(r))
    s, r = s[:n], r[:n]
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


# ── Chart 1: Trajectory ───────────────────────────────────────────────────
def plot_trajectory(seq, out_dir):
    gt_ts, gt_x, gt_y, _ = load_tum(str(seq / 'ground_truth.tum'))
    fc_ts, fc_x, fc_y, _ = load_tum(str(seq / 'fusioncore.tum'))
    ek_ts, ek_x, ek_y, _ = load_tum(str(seq / 'rl_ekf.tum'))

    fig, ax = base_fig(10, 10)
    fig.subplots_adjust(left=0.1, right=0.92, top=0.88, bottom=0.08)
    set_titles(fig,
               'Route Accuracy — 600 s Campus Drive',
               'NCLT 2012-01-08  •  RTK GPS ground truth  •  SE(2) aligned')

    gt_xy = np.stack([gt_x, gt_y], 1)
    fc_al = align_se2(np.stack([fc_x, fc_y], 1), gt_xy)
    ek_al = align_se2(np.stack([ek_x, ek_y], 1), gt_xy)

    # Crop view to the region the filters actually cover, with padding
    pad = 80
    all_x = np.concatenate([fc_al[:, 0], ek_al[:, 0]])
    all_y = np.concatenate([fc_al[:, 1], ek_al[:, 1]])
    xlo, xhi = all_x.min() - pad, all_x.max() + pad
    ylo, yhi = all_y.min() - pad, all_y.max() + pad

    # Clip GT to the same view window for context
    gt_mask = (gt_x >= xlo) & (gt_x <= xhi) & (gt_y >= ylo) & (gt_y <= yhi)

    # GT drawn last so it sits on top as the reference to beat
    ax.plot(ek_al[:, 0], ek_al[:, 1],
            color=C_EKF, lw=2.2, alpha=0.8, label='RL-EKF  (ATE 23.4 m)', zorder=2)
    ax.plot(fc_al[:, 0], fc_al[:, 1],
            color=C_FC, lw=2.2, alpha=0.9, label='FusionCore  (ATE 5.5 m)', zorder=3)
    ax.plot(gt_x[gt_mask], gt_y[gt_mask],
            color='#111827', lw=2.5, alpha=0.9,
            label='Ground Truth — where the robot actually was', zorder=4)

    ax.plot(fc_al[0, 0], fc_al[0, 1], 'o', color=TEXT, ms=7, zorder=5)
    ax.text(fc_al[0, 0] + 8, fc_al[0, 1] + 8, 'Start', fontsize=9, color=TEXT)

    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)
    ax.set_aspect('equal')
    ax.set_xlabel('East (m)', fontsize=10, color=MUTED)
    ax.set_ylabel('North (m)', fontsize=10, color=MUTED)
    ax.grid(color=BORDER, lw=0.7, zorder=0)

    ax.text(0.5, -0.07,
            'Goal: stay as close to the black line as possible.',
            transform=ax.transAxes, ha='center', fontsize=10,
            color=MUTED, style='italic')

    leg = ax.legend(fontsize=10.5, loc='upper left',
                    facecolor='white', edgecolor=BORDER, framealpha=1)
    for t in leg.get_texts():
        t.set_color(TEXT)

    save(fig, out_dir / '01_trajectory.png')


# ── Chart 2: ATE — horizontal bar so ratio is obvious ────────────────────
def plot_ate(out_dir):
    fig, ax = base_fig(11, 5)
    fig.subplots_adjust(left=0.18, right=0.88, top=0.82, bottom=0.18)
    set_titles(fig,
               'How Far Off Was Each Filter, On Average?',
               'Absolute Trajectory Error (ATE RMSE)  •  lower = closer to where the robot actually was')

    labels = ['RL-EKF', 'FusionCore']
    vals   = [23.434, 5.517]
    colors = [C_EKF, C_FC]

    bars = ax.barh(labels, vals, color=colors, height=0.45,
                   edgecolor='none', zorder=3)

    # value + real-world anchor label at end of each bar
    anchors = ['≈ width of a city intersection', '≈ length of a car']
    for bar, v, anchor in zip(bars, vals, anchors):
        ax.text(v + 0.3, bar.get_y() + bar.get_height() / 2,
                f'  {v:.1f} m  —  {anchor}',
                va='center', fontsize=11, color=TEXT, fontweight='bold')

    # "perfect" reference line
    ax.axvline(0, color='#111827', lw=2.0, zorder=5)
    ax.text(0.3, -0.62, 'Perfect\n(0 m error)', fontsize=8.5,
            color='#111827', va='top', fontweight='bold')

    # bracket showing the gap
    ax.annotate('', xy=(vals[1], 0.55), xytext=(vals[0], 0.55),
                arrowprops=dict(arrowstyle='<->', color=MUTED, lw=1.8))
    ax.text((vals[0] + vals[1]) / 2, 0.62,
            '4.2× more accurate',
            ha='center', va='bottom', fontsize=11,
            color=TEXT, fontweight='bold')

    ax.set_xlim(0, vals[0] * 1.55)
    ax.set_xlabel('Average position error (m)', fontsize=11, color=MUTED)
    ax.tick_params(axis='y', labelsize=13, colors=TEXT)
    ax.grid(axis='x', color=BORDER, lw=0.7, zorder=0)
    ax.set_axisbelow(True)
    ax.invert_yaxis()

    save(fig, out_dir / '02_ate.png')


# ── Chart 3: GPS spike — delta from own baseline ─────────────────────────
def plot_spike(seq, out_dir):
    gt_ts, gt_x, gt_y, _ = load_tum(str(seq / 'ground_truth.tum'))
    fc_ts, fc_x, fc_y, _ = load_tum(str(seq / 'fusioncore_spike.tum'))
    ek_ts, ek_x, ek_y, _ = load_tum(str(seq / 'rl_ekf_spike.tum'))

    SPIKE_T = 120.0
    WINDOW  = 30.0
    t0 = gt_ts[0]

    def get_errs(ts, x, y, t_lo, t_hi):
        rel = ts - t0
        errs = interp_error_2d(ts, x, y, gt_ts, gt_x, gt_y)
        mask = (rel >= t_lo) & (rel <= t_hi)
        return rel[mask] - SPIKE_T, errs[mask]

    # baseline = mean error in 30s before spike
    _, fc_pre = get_errs(fc_ts, fc_x, fc_y, SPIKE_T - WINDOW, SPIKE_T)
    _, ek_pre = get_errs(ek_ts, ek_x, ek_y, SPIKE_T - WINDOW, SPIKE_T)
    fc_base = float(np.nanmean(fc_pre))
    ek_base = float(np.nanmean(ek_pre))

    # full window: 30s before → 45s after
    fc_t, fc_e = get_errs(fc_ts, fc_x, fc_y, SPIKE_T - WINDOW, SPIKE_T + 45)
    ek_t, ek_e = get_errs(ek_ts, ek_x, ek_y, SPIKE_T - WINDOW, SPIKE_T + 45)

    fc_delta = fc_e - fc_base
    ek_delta = ek_e - ek_base

    fig, ax = base_fig(12, 7)
    fig.subplots_adjust(left=0.11, right=0.96, top=0.84, bottom=0.13)
    set_titles(fig,
               'How Much Did Each Filter Move When the Fake GPS Arrived?',
               '707 m corrupted fix injected at t = 0  •  both lines start at 0 = their own normal baseline')

    ax.axhline(0, color='#111827', lw=1.5, zorder=1)
    ax.fill_between(ek_t, ek_delta, alpha=0.12, color=C_EKF, zorder=2)
    ax.plot(ek_t, ek_delta, color=C_EKF, lw=2.4,
            label='RL-EKF — blindly accepted the fake fix', zorder=3)
    ax.plot(fc_t, fc_delta, color=C_FC, lw=2.4,
            label='FusionCore — Mahalanobis gate rejected it', zorder=4)

    ax.axvline(0, color='#EF4444', lw=2.0, ls='--', zorder=5)

    ax.figure.canvas.draw()
    ymax = ax.get_ylim()[1]

    ax.text(1.5, ymax * 0.96, '← fake fix\n   injected',
            color='#EF4444', fontsize=9.5, va='top', linespacing=1.5)

    # annotate peak EKF jump
    ek_peak = int(np.nanargmax(ek_delta))
    ax.annotate(f'+{ek_delta[ek_peak]:.0f} m',
                xy=(ek_t[ek_peak], ek_delta[ek_peak]),
                xytext=(ek_t[ek_peak] - 14, ek_delta[ek_peak] * 0.78),
                color=C_EKF, fontsize=12, fontweight='bold',
                arrowprops=dict(arrowstyle='->', color=C_EKF, lw=1.5))

    ax.text(12, ymax * 0.08, '≈ 0 m change', color=C_FC,
            fontsize=11, fontweight='bold')

    ax.set_xlabel('Seconds relative to fake GPS injection', fontsize=11, color=MUTED)
    ax.set_ylabel('Position change from own baseline (m)', fontsize=11, color=MUTED)
    ax.grid(color=BORDER, lw=0.7, zorder=0)
    ax.set_axisbelow(True)

    leg = ax.legend(fontsize=11, loc='upper left',
                    facecolor='white', edgecolor=BORDER, framealpha=1)
    for t in leg.get_texts():
        t.set_color(TEXT)

    badge(ax, 0.73, 0.97, '✓  FC: +1 m — REJECTED', good=True,  size=10)
    badge(ax, 0.73, 0.84, '✗  EKF: +93 m — JUMPED', good=False, size=10)

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
