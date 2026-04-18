#!/usr/bin/env python3
"""
FusionCore vs robot_localization benchmark runner.

Single-threaded design: rclpy.spin_once() is called inside every loop so
callbacks fire reliably without executor threads.

Prerequisites:
  Terminal 1: ros2 launch fusioncore_gazebo benchmark.launch.py
  Wait for 'FusionCore active. Listening for sensors.' then:
  Terminal 2 (same env vars): ros2 run fusioncore_gazebo benchmark_runner
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage

# ── GPS / ENU constants ─────────────────────────────────────────────────────────
ORIGIN_LAT = 43.2557
ORIGIN_LON = -79.8711
ORIGIN_ALT = 100.0
A  = 6378137.0
E2 = 0.00669437999014
GPS_NOISE_H = 0.5
GPS_NOISE_V = 0.3

_W = 62


def _line(char="─"):
    return char * _W


def _header(title):
    print("\n" + _line("═"))
    pad = (_W - len(title) - 2) // 2
    print(" " * pad + f" {title} ")
    print(_line("═"))


def _row(label, fc_val, rl_val, unit=""):
    try:
        mark_fc = " ✓" if float(fc_val) <= float(rl_val) else "  "
        mark_rl = " ✓" if float(rl_val) <  float(fc_val) else "  "
    except Exception:
        mark_fc = mark_rl = "  "
    print(f"  {label:<28} FC: {fc_val:>7}{unit}{mark_fc}   RL: {rl_val:>7}{unit}{mark_rl}")


def enu_to_lla(x, y, z):
    lat0 = math.radians(ORIGIN_LAT)
    lon0 = math.radians(ORIGIN_LON)
    alt0 = ORIGIN_ALT
    sl = math.sin(lat0); cl = math.cos(lat0)
    sn = math.sin(lon0); cn = math.cos(lon0)
    N0 = A / math.sqrt(1.0 - E2 * sl * sl)
    X0 = (N0 + alt0) * cl * cn
    Y0 = (N0 + alt0) * cl * sn
    Z0 = (N0 * (1 - E2) + alt0) * sl
    dX = -sn * x - sl * cn * y + cl * cn * z
    dY =  cn * x - sl * sn * y + cl * sn * z
    dZ =  cl * y + sl * z
    Xp = X0 + dX; Yp = Y0 + dY; Zp = Z0 + dZ
    p  = math.sqrt(Xp * Xp + Yp * Yp)
    lat = math.atan2(Zp, p * (1.0 - E2))
    for _ in range(5):
        s = math.sin(lat)
        N = A / math.sqrt(1.0 - E2 * s * s)
        lat = math.atan2(Zp + E2 * N * s, p)
    s = math.sin(lat)
    N = A / math.sqrt(1.0 - E2 * s * s)
    return math.degrees(lat), math.degrees(math.atan2(Yp, Xp)), p / math.cos(lat) - N


def dist2d(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


class BenchmarkRunner(Node):

    def __init__(self):
        super().__init__("fusioncore_benchmark")

        # Use BEST_EFFORT QoS to match Gazebo bridge publishers
        be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._gt_pos    = None
        self._fc_pos    = None
        self._rl_pos    = None

        self._publish_gps    = True
        self._outlier_active = False
        self._gps_ref_sent   = False
        self._last_gps_t     = 0.0   # manual 5 Hz gate

        # GPS acceptance tracking via /diagnostics
        self._gnss_outliers_count = 0   # cumulative rejected GPS fixes from FC
        self._gnss_outliers_prev  = 0   # snapshot at last DIAG print
        self._gps_fixes_published = 0   # total GPS fixes we published

        self.create_subscription(TFMessage, "/world/fusioncore_test/pose/info",
                                 self._gt_cb,  be)
        self.create_subscription(Odometry,  "/fusion/odom",
                                 self._fc_cb, be)
        self.create_subscription(Odometry,  "/rl/odom",
                                 self._rl_cb, be)
        # FusionCore diagnostics: track gnss_outliers to see GPS rejection rate
        self.create_subscription(DiagnosticArray, "/diagnostics",
                                 self._diag_cb, 10)

        self._cmd_pub = self.create_publisher(Twist,     "/cmd_vel",   10)
        self._gps_pub = self.create_publisher(NavSatFix, "/gnss/fix",  10)

        self.get_logger().info("Benchmark runner ready.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _gt_cb(self, msg):
        # Gazebo Harmonic Pose_V bridge leaves all frame_ids empty.
        # Identify the robot chassis by z-height: base_link sits at z≈0.15m.
        # Tighter range (0.13-0.18) excludes IMU link (z=0.1) and wheels (z≈0).
        for tf in msg.transforms:
            t = tf.transform.translation
            if 0.13 < t.z < 0.18:
                self._gt_pos = (t.x, t.y)
                return

    def _fc_cb(self, msg):
        p = msg.pose.pose.position
        self._fc_pos = (p.x, p.y)

    def _rl_cb(self, msg):
        p = msg.pose.pose.position
        self._rl_pos = (p.x, p.y)

    def _diag_cb(self, msg):
        for status in msg.status:
            if status.name == "fusioncore: GNSS":
                for kv in status.values:
                    if kv.key == "outlier_count":
                        try:
                            self._gnss_outliers_count = int(kv.value)
                        except ValueError:
                            pass

    # ── GPS publication (called manually inside spin loops at ~5 Hz) ──────────

    def _maybe_publish_gps(self):
        now = time.time()
        if now - self._last_gps_t < 0.2:
            return
        self._last_gps_t = now

        if not self._publish_gps or self._gt_pos is None:
            return

        self._gps_fixes_published += 1

        # Diagnostic: log positions + GPS acceptance every ~5 s
        if not hasattr(self, '_last_diag_t'):
            self._last_diag_t = 0.0
        if now - self._last_diag_t >= 5.0:
            dt_diag = now - self._last_diag_t if self._last_diag_t > 0.0 else 5.0
            self._last_diag_t = now
            gt  = self._gt_pos
            fc  = self._fc_pos
            rl  = self._rl_pos
            err_fc = dist2d(gt, fc) if fc else float('nan')
            err_rl = dist2d(gt, rl) if rl else float('nan')
            # GPS rejection rate since last print
            new_rejects = self._gnss_outliers_count - self._gnss_outliers_prev
            self._gnss_outliers_prev = self._gnss_outliers_count
            fixes_in_window = max(1, round(dt_diag / 0.2))
            reject_pct = 100.0 * new_rejects / fixes_in_window
            gps_status = (f"GPS_REJECTED={new_rejects}/{fixes_in_window} "
                          f"({reject_pct:.0f}%  total_rej={self._gnss_outliers_count})")
            if fc is not None:
                fc_str = f"FC=({fc[0]:.2f},{fc[1]:.2f}) err={err_fc:.2f}m"
            else:
                fc_str = "FC=None"
            if rl is not None:
                rl_str = f"RL=({rl[0]:.2f},{rl[1]:.2f}) err={err_rl:.2f}m"
            else:
                rl_str = "RL=None"
            print(f"[DIAG] GT=({gt[0]:.2f},{gt[1]:.2f})  "
                  f"{fc_str}  {rl_str}  {gps_status}")

        if self._outlier_active:
            lat_off = 500.0 / 111111.0
            fix = NavSatFix()
            fix.header.stamp     = self.get_clock().now().to_msg()
            fix.header.frame_id  = "gnss_link"
            fix.status.status    = NavSatStatus.STATUS_FIX
            fix.status.service   = NavSatStatus.SERVICE_GPS
            fix.latitude         = ORIGIN_LAT + lat_off
            fix.longitude        = ORIGIN_LON
            fix.altitude         = ORIGIN_ALT
            fix.position_covariance      = [GPS_NOISE_H**2, 0, 0,
                                            0, GPS_NOISE_H**2, 0,
                                            0, 0, GPS_NOISE_V**2]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self._gps_pub.publish(fix)
            return

        x = self._gt_pos[0] + random.gauss(0, GPS_NOISE_H)
        y = self._gt_pos[1] + random.gauss(0, GPS_NOISE_H)
        z = 0.15 if not self._gps_ref_sent else 0.15 + random.gauss(0, GPS_NOISE_V)
        self._gps_ref_sent = True

        lat, lon, alt = enu_to_lla(x, y, z)
        fix = NavSatFix()
        fix.header.stamp     = self.get_clock().now().to_msg()
        fix.header.frame_id  = "gnss_link"
        fix.status.status    = NavSatStatus.STATUS_FIX
        fix.status.service   = NavSatStatus.SERVICE_GPS
        fix.latitude  = lat;  fix.longitude = lon;  fix.altitude = alt
        fix.position_covariance      = [GPS_NOISE_H**2, 0, 0,
                                        0, GPS_NOISE_H**2, 0,
                                        0, 0, GPS_NOISE_V**2]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(fix)

    # ── Spin helpers ──────────────────────────────────────────────────────────

    def spin_for(self, duration, linear=0.0, angular=0.0):
        """Spin rclpy for `duration` seconds, optionally driving the robot."""
        end = time.time() + duration
        while rclpy.ok() and time.time() < end:
            if linear != 0.0 or angular != 0.0:
                msg = Twist()
                msg.linear.x  = linear
                msg.angular.z = angular
                self._cmd_pub.publish(msg)
            self._maybe_publish_gps()
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self._cmd_pub.publish(Twist())

    # ── Wait for data ─────────────────────────────────────────────────────────

    def wait_for_data(self, timeout=60.0):
        """Wait until ground truth, FusionCore, AND robot_localization are all live."""
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            self._maybe_publish_gps()
            rclpy.spin_once(self, timeout_sec=0.5)
            have_gt = self._gt_pos is not None
            have_fc = self._fc_pos is not None
            have_rl = self._rl_pos is not None
            if have_gt and have_fc and have_rl:
                return True
            missing = []
            if not have_gt: missing.append("/world/fusioncore_test/pose/info")
            if not have_fc: missing.append("/fusion/odom")
            if not have_rl: missing.append("/rl/odom")
            self.get_logger().info(
                f"Waiting for: {', '.join(missing)}", throttle_duration_sec=5.0)
        return False

    # ── Scenario 1: 100 m loop ────────────────────────────────────────────────

    def scenario_loop_accuracy(self):
        _header("Scenario 1 — 100 m Loop (GPS active)")
        print("  Path: straight 25m → left circle → straight 25m → right circle\n")

        fc_errors = []
        rl_errors  = []

        def _run(duration, linear, angular):
            end = time.time() + duration
            while rclpy.ok() and time.time() < end:
                msg = Twist()
                msg.linear.x  = linear
                msg.angular.z = angular
                self._cmd_pub.publish(msg)
                self._maybe_publish_gps()
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._gt_pos and self._fc_pos:
                    fc_errors.append(dist2d(self._gt_pos, self._fc_pos))
                if self._gt_pos and self._rl_pos:
                    rl_errors.append(dist2d(self._gt_pos, self._rl_pos))

        print("  → Straight 25 m...")
        _run(50.0, 0.5, 0.0)

        print("  → Left circle (r=4 m)...")
        _run(50.0, 0.5, 0.125)

        print("  → Straight 25 m...")
        _run(50.0, 0.5, 0.0)

        print("  → Right circle (r=4 m)...")
        _run(50.0, 0.5, -0.125)

        # Compute final error immediately when motion ends — before stopping —
        # so neither filter gets a ZUPT benefit before the snapshot is taken.
        fc_rmse  = math.sqrt(sum(e*e for e in fc_errors) / max(len(fc_errors), 1))
        rl_rmse  = math.sqrt(sum(e*e for e in rl_errors)  / max(len(rl_errors), 1))
        fc_final = dist2d(self._gt_pos, self._fc_pos) if self._gt_pos and self._fc_pos else float("nan")
        rl_final = dist2d(self._gt_pos, self._rl_pos) if self._gt_pos and self._rl_pos else float("nan")
        self.stop()

        print()
        _row("RMSE vs ground truth",  f"{fc_rmse:.3f}",  f"{rl_rmse:.3f}",  " m")
        _row("Final position error",  f"{fc_final:.3f}", f"{rl_final:.3f}", " m")
        return {"fc_rmse": fc_rmse, "rl_rmse": rl_rmse,
                "fc_final": fc_final, "rl_final": rl_final}

    # ── Scenario 2: GPS outlier spike ─────────────────────────────────────────

    def scenario_outlier_spike(self):
        _header("Scenario 2 — 500 m GPS Outlier Spike")
        print("  Robot stopped. Injecting 500 m northward GPS spike for 4 s.")
        print("  FusionCore: Mahalanobis chi2 gate (chi2(3,0.999)=16.27) hard-rejects")
        print("  the spike — fix is dropped, filter state unchanged.")
        print("  robot_localization: navsat_transform continuously re-anchors its datum,")
        print("  which absorbs the spike at the coordinate-transform level rather than")
        print("  rejecting it. This also prevents large jumps but is NOT outlier rejection —")
        print("  the datum is silently shifted, corrupting future GPS accuracy until it")
        print("  re-converges. mahalanobis_threshold is not set (RL default).\n")

        self.stop()
        self.spin_for(3.0)

        fc_before = self._fc_pos
        rl_before  = self._rl_pos
        gt_before  = self._gt_pos
        print(f"  Before spike: GT=({gt_before[0]:.2f}, {gt_before[1]:.2f})" if gt_before else "  Before spike: GT=unknown")

        fc_peak = 0.0
        rl_peak  = 0.0
        self._outlier_active = True
        end = time.time() + 4.0
        while rclpy.ok() and time.time() < end:
            self._maybe_publish_gps()
            rclpy.spin_once(self, timeout_sec=0.05)
            if fc_before and self._fc_pos:
                fc_peak = max(fc_peak, dist2d(fc_before, self._fc_pos))
            if rl_before and self._rl_pos:
                rl_peak = max(rl_peak, dist2d(rl_before, self._rl_pos))
        self._outlier_active = False

        print("  Spike ended. Waiting 5 s for recovery...")
        self.spin_for(5.0)

        fc_rec = dist2d(self._gt_pos, self._fc_pos) if self._gt_pos and self._fc_pos else float("nan")
        rl_rec  = dist2d(self._gt_pos, self._rl_pos) if self._gt_pos and self._rl_pos else float("nan")

        print()
        _row("Peak jump during spike", f"{fc_peak:.2f}", f"{rl_peak:.2f}", " m")
        _row("Error after recovery",   f"{fc_rec:.2f}",  f"{rl_rec:.2f}",  " m")
        return {"fc_peak": fc_peak, "rl_peak": rl_peak,
                "fc_recovery": fc_rec, "rl_recovery": rl_rec}

    # ── Scenario 3: GPS outage ─────────────────────────────────────────────────

    def scenario_gps_outage(self):
        _header("Scenario 3 — 45 s GPS Outage (dead reckoning)")
        print("  GPS active 20 s to initialise both filters.")
        print("  Then GPS cut for 45 s while driving circles.")
        print("  FusionCore: UKF + ZUPT (zero-velocity updates) + adaptive noise")
        print("              + gyro/accel bias estimation. Both at 100 Hz.")
        print("  robot_localization: EKF, fixed noise, no ZUPT equivalent.")
        print("  This tests architectural dead-reckoning quality, not just algorithm.\n")

        print("  → Warm-up 20 s (GPS active)...")
        self.spin_for(20.0, linear=0.3, angular=0.2)
        self.stop(); self.spin_for(1.0)

        gt_off = self._gt_pos
        print(f"  GPS-off reference: GT=({gt_off[0]:.2f}, {gt_off[1]:.2f})" if gt_off else "  GPS-off reference: unknown")

        print("  → GPS CUT. Driving circles for 45 s...")
        self._publish_gps = False
        fc_err_out = []
        rl_err_out  = []
        end = time.time() + 45.0
        while rclpy.ok() and time.time() < end:
            msg = Twist()
            msg.linear.x  = 0.5
            msg.angular.z = 0.4
            self._cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._gt_pos and self._fc_pos:
                fc_err_out.append(dist2d(self._gt_pos, self._fc_pos))
            if self._gt_pos and self._rl_pos:
                rl_err_out.append(dist2d(self._gt_pos, self._rl_pos))
        self.stop(); rclpy.spin_once(self, timeout_sec=0.1)

        fc_at_restore = dist2d(self._gt_pos, self._fc_pos) if self._gt_pos and self._fc_pos else float("nan")
        rl_at_restore  = dist2d(self._gt_pos, self._rl_pos) if self._gt_pos and self._rl_pos else float("nan")

        print("  → GPS RESTORED. Waiting 10 s for filters to re-anchor...")
        self._publish_gps = True
        self.spin_for(10.0)

        fc_after = dist2d(self._gt_pos, self._fc_pos) if self._gt_pos and self._fc_pos else float("nan")
        rl_after  = dist2d(self._gt_pos, self._rl_pos) if self._gt_pos and self._rl_pos else float("nan")

        fc_rmse = math.sqrt(sum(e*e for e in fc_err_out) / max(len(fc_err_out), 1))
        rl_rmse  = math.sqrt(sum(e*e for e in rl_err_out)  / max(len(rl_err_out), 1))

        print()
        _row("RMSE during outage",        f"{fc_rmse:.3f}",      f"{rl_rmse:.3f}",      " m")
        _row("Drift at GPS restoration",  f"{fc_at_restore:.3f}",f"{rl_at_restore:.3f}"," m")
        _row("Error 10 s after GPS back", f"{fc_after:.3f}",     f"{rl_after:.3f}",     " m")
        return {"fc_rmse_out": fc_rmse,        "rl_rmse_out": rl_rmse,
                "fc_err_out":  fc_at_restore,  "rl_err_out":  rl_at_restore,
                "fc_err_after":fc_after,        "rl_err_after":rl_after}

    # ── Scorecard ─────────────────────────────────────────────────────────────

    def print_scorecard(self, s1, s2, s3):
        print("\n\n")
        _header("BENCHMARK RESULTS: FusionCore vs robot_localization")
        print(f"\n  {'Metric':<32} {'FusionCore':>12}   {'robot_loc.':>12}   {'Winner'}")
        print("  " + _line())

        def row(label, fc, rl, unit):
            if math.isnan(fc) or math.isnan(rl):
                winner = "N/A"
            else:
                winner = "FusionCore" if fc <= rl else "robot_loc."
            print(f"  {label:<32} {fc:>10.3f}{unit}   {rl:>10.3f}{unit}   {winner}")

        print("\n  ── Scenario 1: Loop accuracy (GPS active) ──────────────────")
        row("RMSE vs ground truth",       s1["fc_rmse"],     s1["rl_rmse"],     " m")
        row("Final position error",       s1["fc_final"],    s1["rl_final"],    " m")
        print("\n  ── Scenario 2: GPS outlier spike (500 m) ───────────────────")
        row("Peak jump during spike",     s2["fc_peak"],     s2["rl_peak"],     " m")
        row("Error 5 s after spike",      s2["fc_recovery"], s2["rl_recovery"], " m")
        print("\n  ── Scenario 3: 45 s GPS outage ─────────────────────────────")
        row("RMSE during outage",         s3["fc_rmse_out"], s3["rl_rmse_out"], " m")
        row("Drift at GPS restoration",   s3["fc_err_out"],  s3["rl_err_out"],  " m")
        row("Error 10 s after GPS back",  s3["fc_err_after"],s3["rl_err_after"]," m")
        print("\n  " + _line("═"))

        wins = sum(1 for fc, rl in [
            (s1["fc_rmse"], s1["rl_rmse"]), (s1["fc_final"], s1["rl_final"]),
            (s2["fc_peak"], s2["rl_peak"]), (s2["fc_recovery"], s2["rl_recovery"]),
            (s3["fc_rmse_out"], s3["rl_rmse_out"]),
            (s3["fc_err_out"],  s3["rl_err_out"]),
            (s3["fc_err_after"],s3["rl_err_after"]),
        ] if not (math.isnan(fc) or math.isnan(rl)) and fc <= rl)
        total = sum(1 for fc, rl in [
            (s1["fc_rmse"], s1["rl_rmse"]), (s1["fc_final"], s1["rl_final"]),
            (s2["fc_peak"], s2["rl_peak"]), (s2["fc_recovery"], s2["rl_recovery"]),
            (s3["fc_rmse_out"], s3["rl_rmse_out"]),
            (s3["fc_err_out"],  s3["rl_err_out"]),
            (s3["fc_err_after"],s3["rl_err_after"]),
        ] if not (math.isnan(fc) or math.isnan(rl)))
        print(f"\n  FusionCore won {wins}/{total} metrics.\n")
        print("  Architecture notes:")
        print("  FusionCore : 100 Hz UKF | built-in Mahalanobis outlier rejection")
        print("               ZUPT | adaptive noise | gyro+accel bias estimation")
        print("  robot_loc. : 100 Hz EKF | fixed noise | default config")
        print("               (no outlier rejection, no ZUPT — both are available")
        print("               in RL but require manual parameter tuning)")
        print("  Sensors    : identical — same IMU, encoder, GPS feed to both")
        print("  " + _line("═") + "\n")


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = BenchmarkRunner()

    print("\n" + "═" * 62)
    print("  FusionCore vs robot_localization — Benchmark")
    print("═" * 62)
    print("\nWaiting for ground truth, FusionCore, and robot_localization (up to 90 s)...")
    print("(FusionCore takes ~15 s to configure+activate; RL starts immediately)\n")

    if not node.wait_for_data(timeout=90.0):
        print("\n[ERROR] Not all data sources live after 90 s.")
        print("  Check each topic individually:")
        print("    ros2 topic echo /world/fusioncore_test/pose/info --once")
        print("    ros2 topic echo /fusion/odom --once")
        print("    ros2 topic echo /rl/odom --once")
        rclpy.shutdown()
        return

    print("\nAll sources live.")
    print("GPS warm-up: 15 s stationary — anchoring GPS reference in both filters...")
    print("(FC sets GPS reference on first fix; RL navsat anchors its datum.)")
    print("Robot will not move until warm-up completes.\n")
    node.spin_for(15.0)   # robot stationary, GPS published at ~5 Hz

    gt  = node._gt_pos
    fc  = node._fc_pos
    rl  = node._rl_pos
    err_fc = dist2d(gt, fc) if (gt and fc) else float('nan')
    err_rl = dist2d(gt, rl) if (gt and rl) else float('nan')
    print(f"Warm-up complete. Post-warm-up positions:")
    print(f"  GT =({gt[0]:.3f}, {gt[1]:.3f})" if gt else "  GT=None")
    print(f"  FC =({fc[0]:.3f}, {fc[1]:.3f})  err={err_fc:.3f}m  "
          f"(GPS rejected so far: {node._gnss_outliers_count})" if fc else
          f"  FC=None")
    print(f"  RL =({rl[0]:.3f}, {rl[1]:.3f})  err={err_rl:.3f}m" if rl else
          f"  RL=None")
    print("Starting benchmark scenarios...\n")

    s1 = node.scenario_loop_accuracy()
    print("\nPausing 5 s...")
    node.spin_for(5.0)

    s2 = node.scenario_outlier_spike()
    print("\nPausing 5 s...")
    node.spin_for(5.0)

    s3 = node.scenario_gps_outage()

    node.stop()
    node.print_scorecard(s1, s2, s3)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
