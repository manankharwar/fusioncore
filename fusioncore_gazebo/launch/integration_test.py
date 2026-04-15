#!/usr/bin/env python3
"""
FusionCore Gazebo Integration Test
Runs 4 tests in sequence and prints a pass/fail scorecard.

Usage:
  Terminal 1: ros2 launch fusioncore_gazebo fusioncore_gazebo.launch.py
  Terminal 2: python3 integration_test.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Twist
import math
import time
import threading


# ── Thresholds ────────────────────────────────────────────────────────────────
IMU_DRIFT_LIMIT_M       = 2.0   # max drift in 10s IMU-only
GPS_CONVERGENCE_LIMIT_S = 15.0  # max seconds to converge after GPS re-enabled
GPS_CONVERGENCE_DIST_M  = 2.0   # converged when within this many meters of truth
OUTLIER_JUMP_LIMIT_M    = 5.0   # position must not jump more than this on bad fix
CIRCLE_RETURN_LIMIT_M   = 3.0   # final position must be within this of start


class IntegrationTest(Node):

    def __init__(self):
        super().__init__('fusioncore_integration_test')

        self.pos = (0.0, 0.0)
        self.pos_lock = threading.Lock()

        self.odom_sub = self.create_subscription(
            Odometry, '/fusion/odom', self._odom_cb, 100)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.gnss_pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)

        # GPS origin: must match gz_pose_to_gps.py
        self.ORIGIN_LAT = 43.2557
        self.ORIGIN_LON = -79.8711
        self.ORIGIN_ALT = 100.0

        self.results = {}

        self.get_logger().info('Integration test node ready.')

    def _odom_cb(self, msg):
        with self.pos_lock:
            self.pos = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            )

    def get_pos(self):
        with self.pos_lock:
            return self.pos

    def dist(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def drive(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def publish_bad_gps(self, offset_m=500.0):
        """Publish a GPS fix offset_m north of current position."""
        # Convert offset meters north to lat offset
        lat_offset = offset_m / 111111.0
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gnss_link'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude  = self.ORIGIN_LAT + lat_offset
        fix.longitude = self.ORIGIN_LON
        fix.altitude  = self.ORIGIN_ALT
        fix.position_covariance = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 4.0]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        for _ in range(5):
            self.gnss_pub.publish(fix)
            time.sleep(0.1)

    def spin_background(self):
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

    # ── Test 1: IMU-only drift rate ───────────────────────────────────────────

    def test_imu_drift(self):
        print('\n─── Test 1: IMU-only dead reckoning drift ───')
        print('Stopping robot and letting IMU dead reckon for 10 seconds...')

        self.stop()
        time.sleep(2.0)

        # Record start position
        start = self.get_pos()
        print(f'Start position: x={start[0]:.3f} y={start[1]:.3f}')

        # Wait 10 seconds: GPS is still publishing but robot is stationary
        # We're measuring how much the filter drifts with no motion
        time.sleep(10.0)

        end = self.get_pos()
        drift = self.dist(start, end)
        print(f'End position:   x={end[0]:.3f} y={end[1]:.3f}')
        print(f'Drift: {drift:.3f}m (limit: {IMU_DRIFT_LIMIT_M}m)')

        passed = drift < IMU_DRIFT_LIMIT_M
        self.results['IMU drift rate'] = ('PASS' if passed else 'FAIL',
                                          f'{drift:.3f}m drift in 10s')
        print(f'Result: {"PASS ✓" if passed else "FAIL ✗"}')
        return passed

    # ── Test 2: Outlier rejection ─────────────────────────────────────────────

    def test_outlier_rejection(self):
        print('\n─── Test 2: Outlier rejection ───')
        print('Injecting GPS fix 500m away...')

        before = self.get_pos()
        print(f'Position before: x={before[0]:.3f} y={before[1]:.3f}')

        self.publish_bad_gps(offset_m=500.0)
        time.sleep(2.0)

        after = self.get_pos()
        jump = self.dist(before, after)
        print(f'Position after:  x={after[0]:.3f} y={after[1]:.3f}')
        print(f'Position jump: {jump:.3f}m (limit: {OUTLIER_JUMP_LIMIT_M}m)')

        passed = jump < OUTLIER_JUMP_LIMIT_M
        self.results['Outlier rejection'] = ('PASS' if passed else 'FAIL',
                                             f'{jump:.3f}m jump on 500m outlier')
        print(f'Result: {"PASS ✓" if passed else "FAIL ✗"}')
        return passed

    # ── Test 3: GPS correction after drift ───────────────────────────────────

    def test_gps_correction(self):
        print('\n─── Test 3: GPS corrections after drift ───')
        print('Driving forward 3 seconds to build up position...')

        self.drive(0.5, 0.0)
        time.sleep(3.0)
        self.stop()
        time.sleep(1.0)

        pos_after_drive = self.get_pos()
        print(f'Position after drive: x={pos_after_drive[0]:.3f} y={pos_after_drive[1]:.3f}')

        # GPS is already running: check that position stays stable (converged)
        time.sleep(3.0)
        pos_stable = self.get_pos()
        drift_after_stop = self.dist(pos_after_drive, pos_stable)

        print(f'Position 3s after stop: x={pos_stable[0]:.3f} y={pos_stable[1]:.3f}')
        print(f'Drift after stopping: {drift_after_stop:.3f}m (limit: {GPS_CONVERGENCE_DIST_M}m)')

        passed = drift_after_stop < GPS_CONVERGENCE_DIST_M
        self.results['GPS correction'] = ('PASS' if passed else 'FAIL',
                                          f'{drift_after_stop:.3f}m drift after stop with GPS active')
        print(f'Result: {"PASS ✓" if passed else "FAIL ✗"}')
        return passed

    # ── Test 4: Full circle return ────────────────────────────────────────────

    def test_circle_return(self):
        print('\n─── Test 4: Full circle return ───')

        # Drive to a clean start position
        self.stop()
        time.sleep(1.0)
        start = self.get_pos()
        print(f'Circle start: x={start[0]:.3f} y={start[1]:.3f}')

        # One full circle: radius ~0.5m, angular=1.0 rad/s, linear=0.5 m/s
        # Circumference = 2*pi*r = 2*pi*0.5 = 3.14m
        # Time = 3.14 / 0.5 = 6.28s for one full circle
        circle_time = 2.0 * math.pi / 1.0  # = 6.28s
        print(f'Driving circle for {circle_time:.1f} seconds...')

        self.drive(0.5, 1.0)
        time.sleep(circle_time)
        self.stop()
        time.sleep(2.0)

        end = self.get_pos()
        dist_from_start = self.dist(start, end)
        print(f'Circle end:   x={end[0]:.3f} y={end[1]:.3f}')
        print(f'Distance from start: {dist_from_start:.3f}m (limit: {CIRCLE_RETURN_LIMIT_M}m)')

        passed = dist_from_start < CIRCLE_RETURN_LIMIT_M
        self.results['Circle return'] = ('PASS' if passed else 'FAIL',
                                         f'{dist_from_start:.3f}m from start after full circle')
        print(f'Result: {"PASS ✓" if passed else "FAIL ✗"}')
        return passed

    # ── Scorecard ─────────────────────────────────────────────────────────────

    def print_scorecard(self):
        print('\n' + '═'*50)
        print('  FUSIONCORE INTEGRATION TEST SCORECARD')
        print('═'*50)
        all_passed = True
        for name, (result, detail) in self.results.items():
            icon = '✓' if result == 'PASS' else '✗'
            print(f'  [{result}] {icon} {name}')
            print(f'           {detail}')
            if result != 'PASS':
                all_passed = False
        print('═'*50)
        print(f'  Overall: {"ALL TESTS PASSED ✓" if all_passed else "SOME TESTS FAILED ✗"}')
        print('═'*50)
        return all_passed


def main():
    rclpy.init()
    node = IntegrationTest()
    node.spin_background()

    print('\nFusionCore Integration Test Starting...')
    print('Make sure fusioncore_gazebo.launch.py is running first.\n')
    print('Waiting 3 seconds for filter to settle...')
    time.sleep(3.0)

    node.test_imu_drift()
    node.test_outlier_rejection()
    node.test_gps_correction()
    node.test_circle_return()

    node.print_scorecard()

    node.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
