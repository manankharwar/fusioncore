#!/usr/bin/env python3
# Accumulates Odometry into Path for clean trajectory lines in RViz.
# Uses per-source distance decimation to suppress jitter from high-rate
# topics and GPS noise: only adds a pose if the robot has moved MIN_DIST
# from the previous pose.

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

MAX_POSES = 8000

# FC and RL paths: fine detail, but skip micro-jitter under 5 cm
# GPS path: GPS has 0.5 m sigma noise - decimate more aggressively
_MIN_DIST = {
    "/fusion/odom":       0.06,
    "/odometry/filtered": 0.06,
    "/gps/odometry":      0.35,
}


class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")

        self._pub_fc  = self.create_publisher(Path, "/fusion/path", 10)
        self._pub_rl  = self.create_publisher(Path, "/rl/path",     10)
        self._pub_gps = self.create_publisher(Path, "/gps/path",    10)

        self._path_fc  = Path(); self._path_fc.header.frame_id  = "odom"
        self._path_rl  = Path(); self._path_rl.header.frame_id  = "odom"
        self._path_gps = Path(); self._path_gps.header.frame_id = "odom"

        self.create_subscription(
            Odometry, "/fusion/odom",
            lambda m: self._append(m, self._pub_fc,  self._path_fc,  "/fusion/odom"),      10)
        self.create_subscription(
            Odometry, "/odometry/filtered",
            lambda m: self._append(m, self._pub_rl,  self._path_rl,  "/odometry/filtered"), 10)
        self.create_subscription(
            Odometry, "/gps/odometry",
            lambda m: self._append(m, self._pub_gps, self._path_gps, "/gps/odometry"),      10)

    def _append(self, odom: Odometry, pub, path: Path, topic: str):
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        min_d = _MIN_DIST.get(topic, 0.1)

        if path.poses:
            last = path.poses[-1].pose.position
            if math.hypot(px - last.x, py - last.y) < min_d:
                return

        ps = PoseStamped()
        ps.header = odom.header
        ps.pose   = odom.pose.pose
        path.header.stamp = odom.header.stamp
        path.poses.append(ps)
        if len(path.poses) > MAX_POSES:
            path.poses.pop(0)
        pub.publish(path)


def main():
    rclpy.init()
    rclpy.spin(PathPublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
