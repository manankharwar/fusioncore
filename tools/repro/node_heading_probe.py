#!/usr/bin/env python3
"""Drive the real fusioncore_node with synthetic sensors and report what it does
with a dual-antenna heading when the robot starts facing START_DEG.

Parked 5 s, then 1 m/s straight. IMU level and still at 100 Hz, wheel odometry
at 50 Hz, heading exact at 5 Hz with 0.02 rad declared. Wall clock, so the
numbers move by a few tens of milliseconds between runs.

On 0.5.0 a 120 degree start had every heading rejected for 55.70 s and a 170
degree start never got one accepted in 75 s. Fixed in 0.5.1.

usage: node_heading_probe.py START_DEG [T_END] [platform.profile]
Needs a sourced workspace. Uses its own ROS_DOMAIN_ID unless one is set.
"""
import math
import os
import subprocess
import sys
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', str(60 + os.getpid() % 30))

import rclpy  # noqa: E402
from lifecycle_msgs.msg import Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

START_DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 170.0
T_END = float(sys.argv[2]) if len(sys.argv) > 2 else 70.0
PROFILE = sys.argv[3] if len(sys.argv) > 3 else 'general'
PARK = 5.0
yaw_true = math.radians(START_DEG)

LOG = tempfile.TemporaryFile(mode='w+')
proc = subprocess.Popen(
    ['ros2', 'run', 'fusioncore_ros', 'fusioncore_node', '--ros-args',
     '-r', '__node:=fusioncore',
     '-p', 'base_frame:=base_link', '-p', 'init.wait_for_all_sensors:=false',
     '-p', 'gnss.heading_topic:=/gnss/heading', '-p', f'platform.profile:={PROFILE}',
     '-p', 'publish_tf:=false'],
    stdout=LOG, stderr=subprocess.STDOUT)

rclpy.init()
node = rclpy.create_node('node_heading_probe')
cli = node.create_client(ChangeState, '/fusioncore/change_state')
assert cli.wait_for_service(timeout_sec=30.0), 'no lifecycle service'
for tid in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
    req = ChangeState.Request()
    req.transition.id = tid
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
    # Autostart may already have activated it; a refused ACTIVATE is fine then.

imu_pub = node.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
odom_pub = node.create_publisher(Odometry, '/odom/wheels', qos_profile_sensor_data)
hdg_pub = node.create_publisher(Imu, '/gnss/heading', qos_profile_sensor_data)

samples = []


def on_odom(msg):
    q = msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    samples.append((time.monotonic(), yaw, msg.twist.twist.angular.z))


node.create_subscription(Odometry, '/fusion/odom', on_odom, 50)

time.sleep(1.5)
t0 = time.monotonic()
i = 0
while True:
    t = time.monotonic() - t0
    if t > T_END:
        break
    now = node.get_clock().now().to_msg()
    imu = Imu()
    imu.header.stamp = now
    imu.header.frame_id = 'base_link'
    imu.orientation.w = 1.0
    imu.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    imu.linear_acceleration.z = 9.80665
    imu_pub.publish(imu)
    if i % 2 == 0:
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.twist.twist.linear.x = 0.0 if t < PARK else 1.0
        odom_pub.publish(od)
    if i % 20 == 0:
        h = Imu()
        h.header.stamp = now
        h.header.frame_id = 'base_link'
        h.orientation.w = math.cos(yaw_true / 2)
        h.orientation.z = math.sin(yaw_true / 2)
        h.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0e-4]
        hdg_pub.publish(h)
    i += 1
    rclpy.spin_once(node, timeout_sec=0.0)
    d = t0 + i * 0.01 - time.monotonic()
    if d > 0:
        time.sleep(d)


def err_deg(y):
    e = math.degrees(y - yaw_true)
    return (e + 180) % 360 - 180


first_ok = next((ts - t0 for ts, y, _ in samples if abs(err_deg(y)) < 5.0), None)
print(f'start_deg={START_DEG:7.1f} profile={PROFILE} odom_msgs={len(samples)} '
      f'({len(samples) / T_END:.1f} Hz)')
for mark in (1, 4, 6, 10, 20, 30, 45, 60, T_END - 0.5):
    s = next(((ts - t0, y, w) for ts, y, w in samples if ts - t0 >= mark), None)
    if s:
        print(f'  t={s[0]:6.2f}s  yaw={math.degrees(s[1]):8.2f} deg  '
              f'err={err_deg(s[1]):8.2f} deg  wz={s[2]:8.4f}')
print('  first time yaw within 5 deg of truth:',
      'never' if first_ok is None else f'{first_ok:.2f} s')
if samples:
    print(f'  final yaw error: {err_deg(samples[-1][1]):.2f} deg')

node.destroy_node()
rclpy.shutdown()
proc.terminate()
try:
    proc.wait(timeout=10)
except subprocess.TimeoutExpired:
    proc.kill()
    proc.wait()
LOG.seek(0)
out = LOG.read()
print('  node log: "GNSS heading update rejected" warnings (throttled 5 s): '
      f'{out.count("GNSS heading update rejected")}')
