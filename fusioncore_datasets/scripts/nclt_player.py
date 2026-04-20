#!/usr/bin/env python3
"""
NCLT dataset player for FusionCore benchmarking.

Reads NCLT CSV files and publishes as ROS2 sensor topics with simulated clock.

NCLT download: http://robots.engin.umich.edu/nclt/

File formats (verified against 2012-01-08 sequence):
  ms25.csv            utime, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
  ms25_euler.csv      utime, roll, pitch, yaw       (Euler angles from on-board AHRS)
  gps.csv             utime, mode, ?, lat_rad, lon_rad, alt_m, ?, ?
  odometry_mu_100hz.csv  utime, x, y, z, roll, pitch, yaw   (integrated wheel odometry)

Frame convention:
  NCLT IMU (Microstrain 3DM-GX3-45) is mounted NED-like: x-forward, y-right, z-down.
  FusionCore expects ENU body frame: x-forward, y-left, z-up.
  Conversion: ay_enu = -ay_ned, az_enu = -az_ned, wy_enu = -wy_ned, wz_enu = -wz_ned.
  After conversion: az ≈ +9.81 m/s² when flat and stationary (correct for FusionCore).

Optional test modes (set via ROS parameters):
  GPS spike injection:
    gps_spike_time_s    : sim-time seconds after start to inject spike (default -1 = off)
    gps_spike_magnitude_m: spike size in meters (default 500.0)

  GPS outage simulation:
    gps_outage_start_s  : sim-time seconds after start to begin outage (default -1 = off)
    gps_outage_duration_s: how long GPS is cut (default 45.0)

Usage:
  ros2 run fusioncore_datasets nclt_player.py \
    --ros-args -p data_dir:=/path/to/nclt/2012-01-08 \
               -p playback_rate:=1.0
"""

import csv
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


# ─── coordinate helpers ───────────────────────────────────────────────────────

def euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    """ZYX Euler (rad) → quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:  d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def utime_to_ros(utime_us: int) -> Time:
    ns = utime_us * 1000
    t = Time()
    t.sec = int(ns // 1_000_000_000)
    t.nanosec = int(ns % 1_000_000_000)
    return t


def meters_to_lat_deg(meters: float) -> float:
    """Convert north offset in meters to degrees of latitude."""
    return meters / 111_000.0


def meters_to_lon_deg(meters: float, lat_deg: float) -> float:
    """Convert east offset in meters to degrees of longitude at given latitude."""
    return meters / (111_000.0 * math.cos(math.radians(lat_deg)))


# ─── node ─────────────────────────────────────────────────────────────────────

class NCLTPlayer(Node):

    def __init__(self):
        super().__init__('nclt_player')

        self.declare_parameter('data_dir', '')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('duration_s', 0.0)

        # GPS spike injection
        self.declare_parameter('gps_spike_time_s', -1.0)
        self.declare_parameter('gps_spike_magnitude_m', 500.0)

        # GPS outage simulation
        self.declare_parameter('gps_outage_start_s', -1.0)
        self.declare_parameter('gps_outage_duration_s', 45.0)


        data_dir = self.get_parameter('data_dir').value
        if not data_dir:
            raise RuntimeError('nclt_player: data_dir parameter is required')

        self._rate     = self.get_parameter('playback_rate').value
        self._duration = self.get_parameter('duration_s').value

        self._spike_time_s  = self.get_parameter('gps_spike_time_s').value
        self._spike_mag_m   = self.get_parameter('gps_spike_magnitude_m').value
        self._outage_start  = self.get_parameter('gps_outage_start_s').value
        self._outage_dur    = self.get_parameter('gps_outage_duration_s').value

        self._clock_pub = self.create_publisher(Clock,     '/clock',       10)
        self._imu_pub   = self.create_publisher(Imu,       '/imu/data',    50)
        self._gps_pub   = self.create_publisher(NavSatFix, '/gnss/fix',    10)
        self._odom_pub  = self.create_publisher(Odometry,  '/odom/wheels', 50)

        self.get_logger().info(f'Loading NCLT data from: {data_dir}')
        if self._spike_time_s >= 0:
            self.get_logger().info(
                f'GPS spike: {self._spike_mag_m:.0f}m at t={self._spike_time_s:.1f}s')
        if self._outage_start >= 0:
            self.get_logger().info(
                f'GPS outage: {self._outage_dur:.0f}s starting at t={self._outage_start:.1f}s')

        # Load Euler angles separately (for orientation embed in IMU message)
        euler_table = self._load_euler(os.path.join(data_dir, 'ms25_euler.csv'))

        self._events = []
        self._load_imu(os.path.join(data_dir, 'ms25.csv'), euler_table)
        self._load_gps(os.path.join(data_dir, 'gps.csv'))
        self._load_odom(self._find_odom(data_dir))

        self._events.sort(key=lambda e: e[0])
        self.get_logger().info(
            f'Loaded {len(self._events)} events  '
            f'({self._count("imu")} IMU, {self._count("gps")} GPS, '
            f'{self._count("odom")} odom)  rate={self._rate}x')

        threading.Thread(target=self._play, daemon=True).start()

    # ── loaders ───────────────────────────────────────────────────────────────

    def _load_euler(self, path: str) -> list:
        """
        Load ms25_euler.csv → sorted list of (utime, roll, pitch, yaw).
        Format: utime, roll, pitch, yaw   (4 columns, angles in radians)
        Frame: NED-like. Convert pitch/yaw sign for ENU: pitch_enu=-pitch, yaw_enu=-yaw.
        """
        rows = []
        with open(path) as f:
            for row in csv.reader(f):
                if not row or row[0].startswith('#'):
                    continue
                try:
                    utime = int(row[0])
                    roll  = float(row[1])
                    pitch = -float(row[2])  # NED→ENU: negate pitch
                    yaw   = -float(row[3])  # NED→ENU: negate yaw
                    rows.append((utime, roll, pitch, yaw))
                except (ValueError, IndexError):
                    continue
        rows.sort(key=lambda r: r[0])
        self.get_logger().info(f'  Euler: {len(rows)} rows from {os.path.basename(path)}')
        return rows

    def _nearest_euler(self, euler_table: list, utime: int):
        """Binary-search euler_table for the entry nearest to utime."""
        lo, hi = 0, len(euler_table) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if euler_table[mid][0] < utime:
                lo = mid + 1
            else:
                hi = mid
        if lo > 0 and abs(euler_table[lo-1][0] - utime) < abs(euler_table[lo][0] - utime):
            lo -= 1
        return euler_table[lo]

    def _load_imu(self, path: str, euler_table: list):
        """
        Load ms25.csv → IMU events.
        Format: utime, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        NED→ENU conversion: ay_enu=-ay, az_enu=-az, wy_enu=-wy, wz_enu=-wz
        Orientation is taken from the nearest ms25_euler.csv sample.
        """
        count = 0
        with open(path) as f:
            for row in csv.reader(f):
                if not row or row[0].startswith('#'):
                    continue
                try:
                    utime = int(row[0])
                    # cols 1-3: magnetometer (ignored)
                    ax =  float(row[4])
                    ay = -float(row[5])   # NED y-right → ENU y-left
                    az = -float(row[6])   # NED z-down  → ENU z-up  (+9.81 when flat)
                    wx =  float(row[7])
                    wy = -float(row[8])   # NED→ENU
                    wz = -float(row[9])   # NED→ENU
                    _, roll, pitch, yaw = self._nearest_euler(euler_table, utime)
                    self._events.append((utime, 'imu', [wx, wy, wz, ax, ay, az, roll, pitch, yaw]))
                    count += 1
                except (ValueError, IndexError):
                    continue
        self.get_logger().info(f'  IMU:  {count} rows from {os.path.basename(path)}')

    def _load_gps(self, path: str):
        """
        Load gps.csv → GPS events.
        Format: utime, mode, ?, lat_rad, lon_rad, alt_m, ?, ?
        lat/lon are in RADIANS: must convert to degrees.
        mode: 2=2D fix (no altitude), 3=3D fix.
        """
        count, skipped = 0, 0
        with open(path) as f:
            for row in csv.reader(f):
                if not row or row[0].startswith('#'):
                    continue
                try:
                    utime = int(row[0])
                    mode  = int(float(row[1]))
                    lat_r = float(row[3])
                    lon_r = float(row[4])
                    alt_s = row[5].strip()

                    if mode < 3:   # require 3D fix: 2D fixes have nan altitude (set to 0), causing 273m z-spikes
                        skipped += 1
                        continue

                    lat_deg = math.degrees(lat_r)
                    lon_deg = math.degrees(lon_r)
                    alt_m   = float(alt_s) if alt_s.lower() != 'nan' else 0.0

                    self._events.append((utime, 'gps', [lat_deg, lon_deg, alt_m, mode]))
                    count += 1
                except (ValueError, IndexError):
                    continue
        self.get_logger().info(f'  GPS:  {count} fixes ({skipped} no-fix skipped)')

    def _find_odom(self, data_dir: str) -> str:
        for name in ('odometry_mu_100hz.csv', 'odometry_100hz.csv'):
            p = os.path.join(data_dir, name)
            if os.path.exists(p):
                return p
        raise RuntimeError(f'No wheel odometry file found in {data_dir}')

    def _load_odom(self, path: str):
        """
        Load odometry_mu_100hz.csv → velocity events.
        Format: utime, x, y, z, roll, pitch, yaw   (7 columns)
        Differentiate position to get body-frame velocity.
        Heading = col[6] (yaw).
        """
        rows = []
        with open(path) as f:
            for row in csv.reader(f):
                if not row or row[0].startswith('#'):
                    continue
                try:
                    utime = int(row[0])
                    x     = float(row[1])
                    y     = float(row[2])
                    yaw   = float(row[6])    # heading is col 6, NOT col 3
                    rows.append((utime, x, y, yaw))
                except (ValueError, IndexError):
                    continue

        count = 0
        for i in range(1, len(rows)):
            utime, x, y, h = rows[i]
            p_utime, px, py, ph = rows[i - 1]
            dt = (utime - p_utime) / 1e6
            if dt <= 0 or dt > 0.5:
                continue
            dx, dy = x - px, y - py
            vx    = ( dx * math.cos(h) + dy * math.sin(h)) / dt
            vy    = (-dx * math.sin(h) + dy * math.cos(h)) / dt  # ≈0 for diff drive
            omega = angle_diff(h, ph) / dt
            self._events.append((utime, 'odom', [vx, vy, omega]))
            count += 1
        self.get_logger().info(f'  Odom: {count} velocity estimates from {os.path.basename(path)}')

    def _count(self, kind: str) -> int:
        return sum(1 for e in self._events if e[1] == kind)

    # ── playback ──────────────────────────────────────────────────────────────

    def _play(self):
        if not self._events:
            self.get_logger().error('No events loaded')
            return

        sim_start_us = self._events[0][0]
        wall_start   = time.monotonic()
        spike_fired  = False

        for utime, kind, data in self._events:
            sim_elapsed_s = (utime - sim_start_us) / 1e6

            if self._duration > 0 and sim_elapsed_s > self._duration:
                break

            sleep_s = wall_start + sim_elapsed_s / self._rate - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

            ros_time = utime_to_ros(utime)

            clk = Clock()
            clk.clock = ros_time
            self._clock_pub.publish(clk)

            if kind == 'imu':
                self._pub_imu(ros_time, data)
            elif kind == 'odom':
                self._pub_odom(ros_time, data)
            elif kind == 'gps':
                # GPS outage: suppress all GPS during window
                if self._outage_start >= 0:
                    outage_end = self._outage_start + self._outage_dur
                    if self._outage_start <= sim_elapsed_s <= outage_end:
                        continue  # drop this GPS message

                # GPS spike: inject one corrupted fix at spike_time
                if self._spike_time_s >= 0 and not spike_fired:
                    if sim_elapsed_s >= self._spike_time_s:
                        lat, lon, alt, mode = data
                        spike_lat = lat + meters_to_lat_deg(self._spike_mag_m)
                        spike_lon = lon + meters_to_lon_deg(self._spike_mag_m, lat)
                        self._pub_gps(ros_time, [spike_lat, spike_lon, alt, mode])
                        self.get_logger().warn(
                            f'GPS spike injected at t={sim_elapsed_s:.1f}s '
                            f'({self._spike_mag_m:.0f}m NE offset)')
                        spike_fired = True
                        continue

                self._pub_gps(ros_time, data)

        self.get_logger().info('Playback complete.')

    # ── publishers ────────────────────────────────────────────────────────────

    def _pub_imu(self, ros_time: Time, data: list):
        wx, wy, wz, ax, ay, az, roll, pitch, yaw = data
        msg = Imu()
        msg.header.stamp    = ros_time
        msg.header.frame_id = 'imu_link'

        msg.orientation = euler_to_quat(roll, pitch, yaw)
        msg.orientation_covariance = [
            1e-4, 0.0,  0.0,
            0.0,  1e-4, 0.0,
            0.0,  0.0,  1e6,    # large yaw covariance → FC ignores yaw (6-axis mode)
        ]

        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz
        # Microstrain 3DM-GX3-45: angular random walk ≈ 0.07°/√hr → σ ≈ 0.003 rad/s
        msg.angular_velocity_covariance = [
            9e-6, 0.0,  0.0,
            0.0,  9e-6, 0.0,
            0.0,  0.0,  9e-6,
        ]

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az   # ≈ +9.81 when flat (after NED→ENU)
        msg.linear_acceleration_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01,
        ]
        self._imu_pub.publish(msg)

    def _pub_gps(self, ros_time: Time, data: list):
        lat, lon, alt, mode = data
        msg = NavSatFix()
        msg.header.stamp    = ros_time
        msg.header.frame_id = 'gnss_link'
        msg.status.service  = NavSatStatus.SERVICE_GPS
        msg.status.status   = NavSatStatus.STATUS_FIX
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = alt
        # NCLT standard GPS: ~3m CEP → σ_xy ≈ 3m → var ≈ 9m²
        msg.position_covariance = [
            9.0, 0.0,  0.0,
            0.0, 9.0,  0.0,
            0.0, 0.0, 25.0,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(msg)

    def _pub_odom(self, ros_time: Time, data: list):
        vx, vy, omega = data
        msg = Odometry()
        msg.header.stamp     = ros_time
        msg.header.frame_id  = 'odom'
        msg.child_frame_id   = 'base_link'
        msg.twist.twist.linear.x  = vx
        msg.twist.twist.linear.y  = vy
        msg.twist.twist.angular.z = omega
        msg.twist.covariance[0]  = 0.04
        msg.twist.covariance[7]  = 0.01
        msg.twist.covariance[35] = 0.001
        self._odom_pub.publish(msg)


def main():
    rclpy.init()
    node = NCLTPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
