#!/usr/bin/env python3
"""
Create a small FusionCore demo bag from NCLT CSV source data.

Extracts the first --duration seconds of the NCLT 2012-01-08 sequence and
writes a self-contained MCAP bag with raw sensor topics:
  /imu/data         sensor_msgs/Imu            (ms25.csv)
  /odom/wheels      nav_msgs/Odometry          (wheels.csv)
  /gnss/fix         sensor_msgs/NavSatFix      (gps.csv)
  /clock            rosgraph_msgs/Clock

This bag is uploaded to GitHub Releases as nclt_demo_120s.mcap so users
can replay it through FusionCore without downloading the full NCLT dataset.

Requirements:
  pip install rosbags

Usage:
  # From the repo root, with the full NCLT sequence at benchmarks/nclt/2012-01-08/
  python3 demo/make_demo_bag.py \
    --data_dir benchmarks/nclt/2012-01-08 \
    --out      demo/nclt_demo_120s.mcap \
    --duration 120
"""

import argparse
import csv
import math
import struct
import sys
from pathlib import Path

try:
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    sys.exit(
        "Missing dependency: pip install rosbags\n"
        "rosbags is a pure-Python ROS2 bag library with no ROS installation required."
    )

# ── NCLT CSV format constants ─────────────────────────────────────────────────
# ms25.csv:    utime_us, mag_x,mag_y,mag_z, accel_x,accel_y,accel_z, gyro_x,gyro_y,gyro_z
# wheels.csv:  utime_us, v_left_mps, v_right_mps  (both rear wheels)
# gps.csv:     utime_us, mode, num_sats, lat_rad, lon_rad, alt_m, hdop, ?

WHEEL_TRACK_M = 0.522   # NCLT Segway RMP track width (from NCLT paper)


def utime_to_ns(utime_us: int) -> int:
    return utime_us * 1000


def read_imu(path: Path, t_start_us: int, duration_us: int):
    """Yield (utime_us, wx, wy, wz, ax, ay, az) for the first duration_us after t_start_us."""
    t_end = t_start_us + duration_us
    with open(path) as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            t = int(row[0])
            if t < t_start_us:
                continue
            if t > t_end:
                break
            # ms25: col 4,5,6 = accel x,y,z (body NED), col 7,8,9 = gyro x,y,z (body NED)
            # NED -> ENU: ay_enu = -ay_ned, az_enu = -az_ned, wy_enu = -wy_ned, wz_enu = -wz_ned
            ax_ned = float(row[4])
            ay_ned = float(row[5])
            az_ned = float(row[6])
            gx_ned = float(row[7])
            gy_ned = float(row[8])
            gz_ned = float(row[9])
            ax_enu =  ax_ned
            ay_enu = -ay_ned
            az_enu = -az_ned
            wx_enu =  gx_ned
            wy_enu = -gy_ned
            wz_enu = -gz_ned
            yield t, wx_enu, wy_enu, wz_enu, ax_enu, ay_enu, az_enu


def read_wheels(path: Path, t_start_us: int, duration_us: int):
    """Yield (utime_us, vx_mps, wz_radps) from wheel encoder data."""
    t_end = t_start_us + duration_us
    with open(path) as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            t = int(row[0])
            if t < t_start_us:
                continue
            if t > t_end:
                break
            vl = float(row[1])
            vr = float(row[2])
            vx = (vl + vr) / 2.0
            wz = (vr - vl) / WHEEL_TRACK_M
            yield t, vx, wz


def read_gps(path: Path, t_start_us: int, duration_us: int):
    """Yield (utime_us, lat_rad, lon_rad, alt_m, hdop, fix_mode)."""
    t_end = t_start_us + duration_us
    with open(path) as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            t = int(row[0])
            if t < t_start_us:
                continue
            if t > t_end:
                break
            mode = int(row[1])
            if mode < 2:   # 2=2D fix, 3=3D fix; skip no-fix
                continue
            lat_rad = float(row[3])
            lon_rad = float(row[4])
            alt_m   = float(row[5])
            hdop    = float(row[6]) if len(row) > 6 and row[6] else 1.5
            yield t, math.degrees(lat_rad), math.degrees(lon_rad), alt_m, hdop, mode


def make_bag(data_dir: Path, out_path: Path, duration_s: float):
    imu_csv    = data_dir / 'raw files' / 'ms25.csv'
    wheels_csv = data_dir / 'raw files' / 'wheels.csv'
    gps_csv    = data_dir / 'raw files' / 'gps.csv'

    for p in (imu_csv, wheels_csv, gps_csv):
        if not p.exists():
            sys.exit(f"Missing: {p}\nDownload the NCLT 2012-01-08 sequence first.")

    # Determine t_start from first IMU timestamp
    with open(imu_csv) as f:
        for line in f:
            if line.strip() and not line.startswith('#'):
                t_start_us = int(line.split(',')[0])
                break

    duration_us = int(duration_s * 1e6)

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    Imu       = typestore.types['sensor_msgs/msg/Imu']
    Odom      = typestore.types['nav_msgs/msg/Odometry']
    NavSatFix = typestore.types['sensor_msgs/msg/NavSatFix']
    NavSatStatus = typestore.types['sensor_msgs/msg/NavSatStatus']
    Clock     = typestore.types['rosgraph_msgs/msg/Clock']
    Header    = typestore.types['std_msgs/msg/Header']
    Time      = typestore.types['builtin_interfaces/msg/Time']
    Quaternion = typestore.types['geometry_msgs/msg/Quaternion']
    Vector3    = typestore.types['geometry_msgs/msg/Vector3']

    out_path.parent.mkdir(parents=True, exist_ok=True)

    with Writer(str(out_path)) as writer:
        conn_imu    = writer.add_connection('/imu/data',    'sensor_msgs/msg/Imu',       typestore=typestore)
        conn_odom   = writer.add_connection('/odom/wheels', 'nav_msgs/msg/Odometry',     typestore=typestore)
        conn_gps    = writer.add_connection('/gnss/fix',    'sensor_msgs/msg/NavSatFix', typestore=typestore)
        conn_clock  = writer.add_connection('/clock',       'rosgraph_msgs/msg/Clock',   typestore=typestore)

        def ros_time(utime_us: int):
            ns  = utime_to_ns(utime_us)
            sec = ns // 1_000_000_000
            nsec = ns % 1_000_000_000
            return Time(sec=int(sec), nanosec=int(nsec))

        last_clock_t = t_start_us
        CLOCK_INTERVAL_US = 10_000   # 100 Hz clock

        # Merge and sort all events by timestamp
        events = []

        for row in read_imu(imu_csv, t_start_us, duration_us):
            events.append(('imu', row))
        for row in read_wheels(wheels_csv, t_start_us, duration_us):
            events.append(('wheels', row))
        for row in read_gps(gps_csv, t_start_us, duration_us):
            events.append(('gps', row))

        events.sort(key=lambda e: e[1][0])

        n_imu, n_wheels, n_gps = 0, 0, 0

        for kind, row in events:
            t_us = row[0]
            t_ns = utime_to_ns(t_us)
            stamp = ros_time(t_us)

            # Emit clock messages to cover the gap
            while last_clock_t + CLOCK_INTERVAL_US <= t_us:
                last_clock_t += CLOCK_INTERVAL_US
                clk = Clock(clock=ros_time(last_clock_t))
                writer.write(conn_clock, utime_to_ns(last_clock_t),
                             typestore.serialize_cdr(clk, 'rosgraph_msgs/msg/Clock'))

            if kind == 'imu':
                _, wx, wy, wz, ax, ay, az = row
                h = Header(stamp=stamp, frame_id='imu_link')
                # 9-element covariance (row-major 3x3). Use small diagonal.
                cov3 = [0.0] * 9
                cov3[0] = cov3[4] = cov3[8] = 0.0025   # 0.05 rad/s sigma
                msg = Imu(
                    header=h,
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    orientation_covariance=[-1.0] + [0.0]*8,  # unknown
                    angular_velocity=Vector3(x=wx, y=wy, z=wz),
                    angular_velocity_covariance=cov3,
                    linear_acceleration=Vector3(x=ax, y=ay, z=az),
                    linear_acceleration_covariance=[0.0]*8 + [0.01],
                )
                writer.write(conn_imu, t_ns,
                             typestore.serialize_cdr(msg, 'sensor_msgs/msg/Imu'))
                n_imu += 1

            elif kind == 'wheels':
                _, vx, wz = row
                h = Header(stamp=stamp, frame_id='base_link')
                cov6 = [0.0] * 36
                cov6[0]  = 0.0025   # vx variance (0.05 m/s sigma)
                cov6[35] = 0.0004   # wz variance (0.02 rad/s sigma)
                from rosbags.typesys.stores.ros2_humble import (
                    geometry_msgs__msg__TwistWithCovariance as TwistWC,
                    geometry_msgs__msg__Twist as Twist,
                    geometry_msgs__msg__PoseWithCovariance as PoseWC,
                    geometry_msgs__msg__Pose as Pose,
                    geometry_msgs__msg__Point as Point,
                )
                msg = Odom(
                    header=h,
                    child_frame_id='base_link',
                    pose=PoseWC(pose=Pose(position=Point(x=0.0,y=0.0,z=0.0),
                                         orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)),
                                covariance=[0.0]*36),
                    twist=TwistWC(twist=Twist(linear=Vector3(x=vx,y=0.0,z=0.0),
                                             angular=Vector3(x=0.0,y=0.0,z=wz)),
                                  covariance=cov6),
                )
                writer.write(conn_odom, t_ns,
                             typestore.serialize_cdr(msg, 'nav_msgs/msg/Odometry'))
                n_wheels += 1

            elif kind == 'gps':
                _, lat, lon, alt, hdop, mode = row
                h = Header(stamp=stamp, frame_id='gnss_link')
                status_val = 2 if mode >= 3 else 0  # NavSatStatus: FIX=0, SBAS_FIX=1, GBAS_FIX=2
                cov_horiz  = (hdop * 1.5) ** 2   # approximate 1-sigma from HDOP
                msg = NavSatFix(
                    header=h,
                    status=NavSatStatus(status=status_val, service=1),
                    latitude=lat,
                    longitude=lon,
                    altitude=alt,
                    position_covariance=[cov_horiz, 0, 0,
                                         0, cov_horiz, 0,
                                         0, 0, cov_horiz * 4.0],
                    position_covariance_type=1,  # DIAGONAL
                )
                writer.write(conn_gps, t_ns,
                             typestore.serialize_cdr(msg, 'sensor_msgs/msg/NavSatFix'))
                n_gps += 1

        print(f"Written {n_imu} IMU  |  {n_wheels} wheel  |  {n_gps} GPS  messages")
        print(f"Output: {out_path}  ({out_path.stat().st_size // 1024} KB)")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--data_dir', default='benchmarks/nclt/2012-01-08',
                        help='Path to NCLT sequence directory containing "raw files/"')
    parser.add_argument('--out', default='demo/nclt_demo_120s.mcap',
                        help='Output MCAP path')
    parser.add_argument('--duration', type=float, default=120.0,
                        help='Duration to extract in seconds (default: 120)')
    args = parser.parse_args()

    make_bag(Path(args.data_dir), Path(args.out), args.duration)


if __name__ == '__main__':
    main()
