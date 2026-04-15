#!/usr/bin/env python3
"""
FusionCore Ground Control Dashboard
Aerospace-grade real-time sensor fusion visualization
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyBboxPatch, Circle, FancyArrowPatch
import numpy as np
import threading
import time
from collections import deque
import math

# ── Palette ───────────────────────────────────────────────────────────────────
BG       = '#02060e'
BG2      = '#050d1a'
PANEL    = '#080f1e'
BORDER   = '#0d2040'
CYAN     = '#00e5ff'
CYAN2    = '#00b8d4'
CYAN_DIM = '#003d4d'
GREEN    = '#00ff88'
GREEN2   = '#00c966'
RED      = '#ff1744'
RED2     = '#ff5252'
ORANGE   = '#ff6d00'
YELLOW   = '#ffd600'
WHITE    = '#e8f4fd'
DIM      = '#1a3a5c'
DIM2     = '#0d2035'
GREY     = '#2a4a6a'

THRESHOLD = 16.27

# ── State ─────────────────────────────────────────────────────────────────────
class S:
    lock       = threading.Lock()
    fuse_x     = deque(maxlen=800)
    fuse_y     = deque(maxlen=800)
    gps_x      = deque(maxlen=80)
    gps_y      = deque(maxlen=80)
    spike_x    = deque(maxlen=30)
    spike_y    = deque(maxlen=30)
    pos        = (0.0, 0.0)
    speed      = 0.0
    heading    = 0.0
    dist       = 0.0
    vx_hist    = deque([0.0]*300, maxlen=300)
    vy_hist    = deque([0.0]*300, maxlen=300)
    mahal_hist = deque([0.0]*300, maxlen=300)
    last_d2    = 0.0
    rejected   = 0
    log        = deque(maxlen=12)
    spike_t    = 0.0
    gps_ref    = None
    prev_pos   = None
    gps_origin_pos = (0.0, 0.0)

def enu(lat, lon, rl, ro):
    R = 6371000.0
    x = R * np.radians(lon - ro) * np.cos(np.radians(rl))
    y = R * np.radians(lat - rl)
    return x, y

# ── ROS ───────────────────────────────────────────────────────────────────────
class GCSNode(Node):
    def __init__(self):
        super().__init__('fusioncore_gcs')
        self.create_subscription(Odometry,  '/fusion/odom', self.odom_cb, 10)
        self.create_subscription(NavSatFix, '/gnss/fix',    self.gps_cb,  10)

    _odom_count = 0
    def odom_cb(self, msg):
        GCSNode._odom_count += 1
        if GCSNode._odom_count % 20 != 0:
            return
        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        q  = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        with S.lock:
            if S.prev_pos:
                dx = x - S.prev_pos[0]
                dy = y - S.prev_pos[1]
                S.dist += math.sqrt(dx*dx + dy*dy)
            S.prev_pos = (x, y)
            S.fuse_x.append(x)
            S.fuse_y.append(y)
            S.pos     = (x, y)
            S.speed   = math.sqrt(vx*vx + vy*vy)
            S.heading = math.degrees(yaw) % 360
            S.vx_hist.append(vx)
            S.vy_hist.append(vy)

    def gps_cb(self, msg):
        with S.lock:
            if S.gps_ref is None:
                S.gps_ref = (msg.latitude, msg.longitude)
                S.gps_origin_pos = S.pos  # align with FusionCore origin
            gx_raw, gy_raw = enu(msg.latitude, msg.longitude,
                         S.gps_ref[0], S.gps_ref[1])
            # Offset GPS to match FusionCore coordinate frame
            ox, oy = getattr(S, 'gps_origin_pos', (0.0, 0.0))
            gx = gx_raw + ox
            gy = gy_raw + oy
            fx, fy = S.pos
            dx, dy = gx - fx, gy - fy
            d2 = (dx*dx + dy*dy) / 4.0
            S.mahal_hist.append(min(d2, 600.0))
            S.last_d2 = d2
            if d2 > THRESHOLD:
                S.spike_x.append(gx)
                S.spike_y.append(gy)
                S.rejected += 1
                S.spike_t = time.time()
                ts = time.strftime('%H:%M:%S')
                dist_m = math.sqrt((gx-fx)**2 + (gy-fy)**2)
                S.log.append(f'{ts}  BLOCKED  +{dist_m:.0f}m  d²={d2:.0f}')
            else:
                S.gps_x.append(gx)
                S.gps_y.append(gy)

def spin_node(node):
    rclpy.spin(node)

# ── Dashboard ─────────────────────────────────────────────────────────────────
def build():
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(18, 10), facecolor=BG)
    fig.canvas.manager.set_window_title('FusionCore GCS  //  Ground Control Station')

    gs = gridspec.GridSpec(4, 6, figure=fig,
                           left=0.02, right=0.99,
                           top=0.95, bottom=0.04,
                           hspace=0.55, wspace=0.35)

    # ── Header ────────────────────────────────────────────────────────────────
    fig.text(0.5, 0.978,
             'FUSIONCORE  GCS  ·  GROUND CONTROL STATION  ·  LIVE',
             ha='center', va='top',
             color=CYAN, fontsize=13,
             fontfamily='monospace', fontweight='bold', alpha=0.95)
    fig.text(0.02, 0.978, '◉ SYSTEM ACTIVE',
             ha='left', va='top',
             color=GREEN, fontsize=8, fontfamily='monospace')
    fig.text(0.98, 0.978, 'UKF · 21D STATE · 100Hz · JAZZY',
             ha='right', va='top',
             color=GREY, fontsize=8, fontfamily='monospace')

    # ═══════════════════════════════════════════════════════════════════════════
    # PANEL 1: Main map (left, full height)
    # ═══════════════════════════════════════════════════════════════════════════
    ax_map = fig.add_subplot(gs[0:4, 0:3])
    ax_map.set_facecolor(BG2)
    ax_map.tick_params(colors=GREY, labelsize=7)
    for sp in ax_map.spines.values():
        sp.set_color(BORDER)
        sp.set_linewidth(1.5)
    ax_map.set_xlabel('EAST  (m)', color=GREY, fontsize=7,
                      fontfamily='monospace', labelpad=4)
    ax_map.set_ylabel('NORTH  (m)', color=GREY, fontsize=7,
                      fontfamily='monospace', labelpad=4)
    ax_map.grid(True, color=DIM2, linewidth=0.4, alpha=0.7)
    ax_map.set_title('POSITION MAP  ·  REAL-TIME TRACKING',
                     color=CYAN2, fontsize=8,
                     fontfamily='monospace', pad=6)

    # Range rings
    for r in [5, 10, 15, 20]:
        circle = plt.Circle((0, 0), r, color=DIM2,
                             fill=False, linewidth=0.3, alpha=0.5)
        ax_map.add_patch(circle)

    # FusionCore trail: glowing effect with two layers
    trail_glow, = ax_map.plot([], [], color=CYAN2, linewidth=5,
                               alpha=0.15, zorder=3)
    trail,      = ax_map.plot([], [], color=CYAN, linewidth=2,
                               alpha=0.9, zorder=4, solid_capstyle='round')

    # Robot dot: pulsing
    robot_outer, = ax_map.plot([], [], 'o', color=CYAN, markersize=18,
                                alpha=0.2, zorder=7)
    robot_dot,   = ax_map.plot([], [], 'o', color=CYAN, markersize=8,
                                zorder=8, markeredgecolor=WHITE,
                                markeredgewidth=1.2)

    # Heading arrow
    heading_arrow = ax_map.annotate('', xy=(0,0), xytext=(0,0),
                                     arrowprops=dict(
                                         arrowstyle='->', color=CYAN,
                                         lw=2.0, mutation_scale=15),
                                     zorder=9)

    # GPS scatter: small dim orange dots
    gps_sc = ax_map.scatter([], [], s=12, c=ORANGE,
                             alpha=0.4, zorder=2,
                             label='Raw GPS (noise)')

    # Spike markers
    spike_sc = ax_map.scatter([], [], s=300, c=RED,
                               marker='x', linewidths=3,
                               zorder=10, label='GPS spike (BLOCKED)')

    # Spike ring animation
    spike_ring = plt.Circle((0,0), 0, color=RED,
                              fill=False, linewidth=2, alpha=0,
                              zorder=6)
    ax_map.add_patch(spike_ring)

    # Spike label
    spike_lbl = ax_map.text(0, 0, '', color=RED,
                             fontsize=9, fontfamily='monospace',
                             fontweight='bold', ha='center', zorder=11,
                             bbox=dict(boxstyle='round,pad=0.4',
                                       facecolor=BG, edgecolor=RED,
                                       linewidth=1.5, alpha=0.92))
    spike_lbl.set_visible(False)

    ax_map.legend(loc='lower left', fontsize=7,
                  facecolor=PANEL, edgecolor=BORDER,
                  labelcolor=WHITE, markerscale=1.3)

    # ═══════════════════════════════════════════════════════════════════════════
    # PANEL 2: Velocity waveform (top center)
    # ═══════════════════════════════════════════════════════════════════════════
    ax_vel = fig.add_subplot(gs[0, 3:6])
    ax_vel.set_facecolor(PANEL)
    ax_vel.tick_params(colors=GREY, labelsize=6)
    for sp in ax_vel.spines.values(): sp.set_color(BORDER)
    ax_vel.set_title('VELOCITY  ·  Vx  /  Vy  (m/s)',
                     color=GREY, fontsize=7,
                     fontfamily='monospace', pad=3)
    ax_vel.set_ylim(-2, 2)
    ax_vel.set_xlim(0, 300)
    ax_vel.grid(True, color=DIM2, linewidth=0.3, alpha=0.6, axis='y')
    ax_vel.axhline(0, color=GREY, linewidth=0.5, alpha=0.4)

    vel_vx, = ax_vel.plot([], [], color=CYAN, linewidth=1.2,
                           label='Vx', alpha=0.9)
    vel_vy, = ax_vel.plot([], [], color=GREEN, linewidth=1.2,
                           label='Vy', alpha=0.7, linestyle='--')
    ax_vel.legend(loc='upper right', fontsize=6,
                  facecolor=PANEL, edgecolor=BORDER,
                  labelcolor=WHITE, ncol=2)

    # ═══════════════════════════════════════════════════════════════════════════
    # PANEL 3: Mahalanobis waveform (second row center)
    # ═══════════════════════════════════════════════════════════════════════════
    ax_mah = fig.add_subplot(gs[1, 3:6])
    ax_mah.set_facecolor(PANEL)
    ax_mah.tick_params(colors=GREY, labelsize=6)
    for sp in ax_mah.spines.values(): sp.set_color(BORDER)
    ax_mah.set_title('MAHALANOBIS DISTANCE  d²  ·  GPS QUALITY GATE',
                     color=GREY, fontsize=7,
                     fontfamily='monospace', pad=3)
    ax_mah.set_ylim(0, 150)
    ax_mah.set_xlim(0, 300)
    ax_mah.grid(True, color=DIM2, linewidth=0.3, alpha=0.6, axis='y')

    # Zone fills
    ax_mah.fill_between([0,300], 0, THRESHOLD,
                         color=GREEN, alpha=0.04)
    ax_mah.fill_between([0,300], THRESHOLD, 150,
                         color=RED, alpha=0.06)
    ax_mah.axhline(THRESHOLD, color=RED, linewidth=1,
                   linestyle='--', alpha=0.7)
    ax_mah.text(295, THRESHOLD+2, f'REJECT  {THRESHOLD}',
                ha='right', va='bottom', color=RED,
                fontsize=6, fontfamily='monospace', alpha=0.8)
    ax_mah.text(295, THRESHOLD/2, 'NOMINAL',
                ha='right', va='center', color=GREEN,
                fontsize=6, fontfamily='monospace', alpha=0.5)

    mah_line, = ax_mah.plot([], [], color=YELLOW,
                             linewidth=1.5, zorder=5)
    mah_fill_ref = [None]

    d2_text = ax_mah.text(0.02, 0.85, 'd² = 0.0',
                           transform=ax_mah.transAxes,
                           color=YELLOW, fontsize=10,
                           fontfamily='monospace', fontweight='bold')

    # ═══════════════════════════════════════════════════════════════════════════
    # PANEL 4: Stats (third row right)
    # ═══════════════════════════════════════════════════════════════════════════
    ax_stats = fig.add_subplot(gs[2, 3:6])
    ax_stats.set_facecolor(PANEL)
    ax_stats.axis('off')

    ax_stats.text(0.5, 0.97, 'FILTER  STATE  VECTOR',
                  ha='center', va='top', color=CYAN2,
                  fontsize=8, fontfamily='monospace', fontweight='bold',
                  transform=ax_stats.transAxes)

    # Stat rows
    labels = ['POSITION X', 'POSITION Y', 'SPEED', 'HEADING', 'DISTANCE']
    units  = ['m', 'm', 'm/s', '°', 'm']
    y_pos  = [0.75, 0.58, 0.42, 0.26, 0.10]
    stat_vals = []
    for i, (lbl, unit, yp) in enumerate(zip(labels, units, y_pos)):
        ax_stats.text(0.02, yp+0.09, lbl, ha='left', va='top',
                      color=GREY, fontsize=6, fontfamily='monospace',
                      transform=ax_stats.transAxes)
        v = ax_stats.text(0.02, yp, '—',
                          ha='left', va='top', color=CYAN,
                          fontsize=12, fontfamily='monospace',
                          fontweight='bold',
                          transform=ax_stats.transAxes)
        ax_stats.text(0.95, yp, unit, ha='right', va='top',
                      color=GREY, fontsize=8, fontfamily='monospace',
                      transform=ax_stats.transAxes)
        stat_vals.append(v)

    # ═══════════════════════════════════════════════════════════════════════════
    # PANEL 5: Event log (bottom right)
    # ═══════════════════════════════════════════════════════════════════════════
    ax_log = fig.add_subplot(gs[3, 3:6])
    ax_log.set_facecolor(PANEL)
    ax_log.axis('off')

    ax_log.text(0.5, 0.97, 'OUTLIER  REJECTION  LOG',
                ha='center', va='top', color=RED2,
                fontsize=8, fontfamily='monospace', fontweight='bold',
                transform=ax_log.transAxes)

    log_lines = []
    for i in range(6):
        t = ax_log.text(0.02, 0.82 - i*0.14, '',
                        ha='left', va='top', color=RED,
                        fontsize=7, fontfamily='monospace',
                        transform=ax_log.transAxes)
        log_lines.append(t)

    counter_text = ax_log.text(0.97, 0.15, '0',
                                ha='right', va='bottom', color=RED,
                                fontsize=28, fontfamily='monospace',
                                fontweight='bold',
                                transform=ax_log.transAxes)
    ax_log.text(0.97, 0.05, 'SPIKES BLOCKED',
                ha='right', va='bottom', color=GREY,
                fontsize=6, fontfamily='monospace',
                transform=ax_log.transAxes)

    # ── Animation ──────────────────────────────────────────────────────────────
    frame_count = [0]

    def update(frame):
        frame_count[0] += 1
        t = time.time()

        with S.lock:
            fx   = list(S.fuse_x)
            fy   = list(S.fuse_y)
            gx   = list(S.gps_x)
            gy   = list(S.gps_y)
            sx   = list(S.spike_x)
            sy   = list(S.spike_y)
            pos  = S.pos
            spd  = S.speed
            hdg  = S.heading
            dst  = S.dist
            vxh  = list(S.vx_hist)
            vyh  = list(S.vy_hist)
            mhh  = list(S.mahal_hist)
            d2   = S.last_d2
            cnt  = S.rejected
            st   = S.spike_t
            log  = list(S.log)

        spike_age = t - st if st > 0 else 999
        is_spike  = spike_age < 4.0

        # ── Map ────────────────────────────────────────────────────────────────
        if fx:
            trail.set_data(fx, fy)
            trail_glow.set_data(fx, fy)
            cx, cy = pos
            span = max(12, max(
                abs(max(fx)-min(fx)) if len(fx)>1 else 0,
                abs(max(fy)-min(fy)) if len(fy)>1 else 0
            ) * 0.6 + 6)
            ax_map.set_xlim(cx-span, cx+span)
            ax_map.set_ylim(cy-span*0.65, cy+span*0.65)

            # Robot dot pulsing
            pulse = 0.15 + 0.1 * abs(math.sin(t * 2.5))
            robot_outer.set_data([cx], [cy])
            robot_outer.set_alpha(pulse)
            robot_dot.set_data([cx], [cy])

            # Heading arrow
            hrad = math.radians(hdg)
            arrow_len = span * 0.08
            dx_arr = arrow_len * math.sin(hrad)
            dy_arr = arrow_len * math.cos(hrad)
            heading_arrow.set_visible(False)

            # Range ring overlay
            for patch in ax_map.patches:
                if isinstance(patch, plt.Circle) and patch != spike_ring:
                    patch.center = (cx, cy)

        # GPS dots
        if gx:
            gps_sc.set_offsets(np.c_[gx, gy])
        else:
            gps_sc.set_offsets(np.empty((0,2)))

        # Spike markers
        if sx:
            spike_sc.set_offsets(np.c_[sx, sy])
        else:
            spike_sc.set_offsets(np.empty((0,2)))

        # Spike ring + label
        if is_spike and sx and fx:
            ring_r = 1.0 + spike_age * 3.0
            ring_a = max(0, 0.8 - spike_age * 0.2)
            spike_ring.center = (sx[-1], sy[-1])
            spike_ring.set_radius(ring_r)
            spike_ring.set_alpha(ring_a)
            spike_ring.set_color(RED)

            cx2, cy2 = pos
            dist_m = math.sqrt((sx[-1]-cx2)**2 + (sy[-1]-cy2)**2)
            spike_lbl.set_position((sx[-1], sy[-1]))
            spike_lbl.set_text(f'GPS CLAIMS  +{dist_m:.0f}m\n✗  OUTLIER BLOCKED')
            spike_lbl.set_visible(True)

            blink = 0.5 + 0.5 * abs(math.sin(t * 8))
            ax_map.set_facecolor(
                f'#{int(0x05 + 8*blink):02x}0d1a')
            for sp in ax_map.spines.values():
                sp.set_color(RED)
                sp.set_linewidth(2.5)
        else:
            spike_ring.set_alpha(0)
            spike_lbl.set_visible(False)
            ax_map.set_facecolor(BG2)
            for sp in ax_map.spines.values():
                sp.set_color(BORDER)
                sp.set_linewidth(1.5)

        # ── Velocity waveform ──────────────────────────────────────────────────
        xs = list(range(len(vxh)))
        vel_vx.set_data(xs, vxh)
        vel_vy.set_data(xs, vyh)

        # ── Mahalanobis ────────────────────────────────────────────────────────
        xm = list(range(len(mhh)))
        mah_line.set_data(xm, mhh)
        recent_spike = any(v > THRESHOLD for v in mhh[-20:])
        mah_line.set_color(RED if recent_spike else YELLOW)
        mah_line.set_linewidth(2.5 if recent_spike else 1.5)
        d2_text.set_text(f'd² = {d2:.1f}')
        d2_text.set_color(RED if d2 > THRESHOLD else YELLOW)

        # ── Stats ──────────────────────────────────────────────────────────────
        vals = [f'{pos[0]:+.3f}', f'{pos[1]:+.3f}',
                f'{spd:.3f}', f'{hdg:.1f}', f'{dst:.1f}']
        for sv, val in zip(stat_vals, vals):
            sv.set_text(val)
            sv.set_color(RED if is_spike else CYAN)

        # ── Log ────────────────────────────────────────────────────────────────
        recent = list(reversed(log))[:6]
        for i, lt in enumerate(log_lines):
            if i < len(recent):
                lt.set_text(recent[i])
                lt.set_color(RED if i == 0 and is_spike else RED2)
                lt.set_alpha(1.0 - i * 0.12)
            else:
                lt.set_text('')

        counter_text.set_text(str(cnt))
        if is_spike:
            blink2 = 0.6 + 0.4 * abs(math.sin(t * 6))
            counter_text.set_color(RED)
            counter_text.set_alpha(blink2)
            ax_log.set_facecolor(
                f'#{int(0x08 + 6*blink2):02x}0f1e')
        else:
            counter_text.set_color(RED2)
            counter_text.set_alpha(1.0)
            ax_log.set_facecolor(PANEL)

        return (trail, trail_glow, robot_dot, robot_outer,
                gps_sc, spike_sc, spike_ring, spike_lbl,
                vel_vx, vel_vy, mah_line, d2_text,
                *stat_vals, *log_lines, counter_text)

    ani = FuncAnimation(fig, update, interval=80,
                        blit=False, cache_frame_data=False)
    plt.show()
    return ani

def main():
    rclpy.init()
    node = GCSNode()
    threading.Thread(target=spin_node, args=(node,), daemon=True).start()
    print('\033[96m╔══════════════════════════════════════════╗\033[0m')
    print('\033[96m║   FusionCore GCS  //  Ground Control     ║\033[0m')
    print('\033[96m╚══════════════════════════════════════════╝\033[0m')
    build()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()