#!/usr/bin/env python3
"""
Drives the robot in a slow circle so the comparison demo is fully autonomous.
Starts publishing after a warm-up delay so FusionCore and robot_localization
have time to initialize before the robot moves.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

WARMUP_S      = 20.0   # seconds to wait before driving (let filters initialize)
LINEAR_MPS    = 0.4    # forward speed
ANGULAR_RPS   = 0.25   # turning rate - produces ~2.5 m radius circle


class DriveCircle(Node):
    def __init__(self):
        super().__init__("drive_circle")
        self._pub   = self.create_publisher(Twist, "/cmd_vel", 10)
        self._start = self.get_clock().now().nanoseconds * 1e-9
        self._timer = self.create_timer(0.1, self._tick)
        self._driving = False
        self.get_logger().info(f"Waiting {WARMUP_S}s before driving...")

    def _tick(self):
        now     = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._start

        if elapsed < WARMUP_S:
            return

        if not self._driving:
            self.get_logger().info("Starting circle drive")
            self._driving = True

        twist = Twist()
        twist.linear.x  = LINEAR_MPS
        twist.angular.z = ANGULAR_RPS
        self._pub.publish(twist)


def main():
    rclpy.init()
    rclpy.spin(DriveCircle())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
