#!/usr/bin/env python3
"""
Accumulates Odometry messages from FusionCore and robot_localization into
nav_msgs/Path topics so RViz2 can show smooth trajectory trails.

Only adds a new point when the robot has moved MIN_DIST_M metres from the
previous recorded point. This eliminates high-frequency jitter from the
100Hz filter output without losing any real trajectory information.
"""
import math
import copy
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

MAX_POSES   = 5000
MIN_DIST_M  = 0.05   # only record a new point after moving 5 cm
RL_Y_OFFSET = 8.0    # shift RL trail beside FusionCore trail in Y


class PathRecorder(Node):
    def __init__(self):
        super().__init__("path_recorder")

        self._fc_path = Path()
        self._rl_path = Path()

        self._fc_path.header.frame_id = "odom"
        self._rl_path.header.frame_id = "odom"

        self.fc_pub = self.create_publisher(Path, "/demo/fusioncore_path", 10)
        self.rl_pub = self.create_publisher(Path, "/demo/rl_path",         10)

        self._fc_last = None   # last recorded (x, y)
        self._rl_last = None

        self.create_subscription(Odometry, "/fusion/odom",          self._fc_cb, 10)
        self.create_subscription(Odometry, "/rl/odometry/filtered", self._rl_cb, 10)

        self.get_logger().info("Path recorder ready")

    def _moved(self, last, x, y):
        if last is None:
            return True
        dx = x - last[0]
        dy = y - last[1]
        return math.sqrt(dx*dx + dy*dy) >= MIN_DIST_M

    def _append(self, path, msg, x_override=None, y_override=None):
        x = x_override if x_override is not None else msg.pose.pose.position.x
        y = y_override if y_override is not None else msg.pose.pose.position.y
        ps = PoseStamped()
        ps.header.stamp    = msg.header.stamp
        ps.header.frame_id = path.header.frame_id
        ps.pose            = copy.deepcopy(msg.pose.pose)
        ps.pose.position.x = x
        ps.pose.position.y = y
        path.poses.append(ps)
        if len(path.poses) > MAX_POSES:
            path.poses.pop(0)
        path.header.stamp = msg.header.stamp

    def _fc_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not self._moved(self._fc_last, x, y):
            return
        self._fc_last = (x, y)
        self._append(self._fc_path, msg)
        self.fc_pub.publish(self._fc_path)

    def _rl_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y + RL_Y_OFFSET
        if not self._moved(self._rl_last, x, y):
            return
        self._rl_last = (x, y)
        self._append(self._rl_path, msg, x_override=x, y_override=y)
        self.rl_pub.publish(self._rl_path)


def main():
    rclpy.init()
    rclpy.spin(PathRecorder())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
