#!/usr/bin/env python3
"""
Accumulates Odometry messages from FusionCore and robot_localization into
nav_msgs/Path topics so RViz2 can show full trajectory trails.
Also extracts ground truth from the Gazebo TF broadcast.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

MAX_POSES = 10000   # prevent unbounded growth on long runs


class PathRecorder(Node):
    def __init__(self):
        super().__init__("path_recorder")

        self._fc_path  = Path()
        self._rl_path  = Path()
        self._gt_path  = Path()

        self._fc_path.header.frame_id  = "odom"
        self._rl_path.header.frame_id  = "odom"
        self._gt_path.header.frame_id  = "odom"

        self.fc_pub  = self.create_publisher(Path, "/demo/fusioncore_path",  10)
        self.rl_pub  = self.create_publisher(Path, "/demo/rl_path",          10)
        self.gt_pub  = self.create_publisher(Path, "/demo/ground_truth_path", 10)

        self.create_subscription(Odometry, "/fusion/odom",          self._fc_cb, 10)
        self.create_subscription(Odometry, "/rl/odometry/filtered", self._rl_cb, 10)
        self.create_subscription(
            TFMessage, "/world/fusioncore_test/pose/info", self._gt_cb, 10)

        # ground truth ENU origin for the pose extraction
        self._gt_origin_x = None
        self._gt_origin_y = None

        self.get_logger().info("Path recorder ready")

    def _append(self, path, odom_msg):
        ps = PoseStamped()
        ps.header = odom_msg.header
        ps.pose   = odom_msg.pose.pose
        path.poses.append(ps)
        if len(path.poses) > MAX_POSES:
            path.poses.pop(0)
        path.header.stamp = odom_msg.header.stamp

    def _fc_cb(self, msg):
        self._append(self._fc_path, msg)
        self.fc_pub.publish(self._fc_path)

    def _rl_cb(self, msg):
        self._append(self._rl_path, msg)
        self.rl_pub.publish(self._rl_path)

    def _gt_cb(self, tf_msg):
        # Find the robot base_link in the Gazebo world TF broadcast
        for tf in tf_msg.transforms:
            tail = tf.child_frame_id.rsplit("::", 1)[-1].rsplit("/", 1)[-1]
            if tail != "base_link":
                continue
            t = tf.transform.translation
            if not (0.05 < t.z < 0.4):
                continue

            if self._gt_origin_x is None:
                self._gt_origin_x = t.x
                self._gt_origin_y = t.y

            ps = PoseStamped()
            ps.header.stamp    = tf_msg.transforms[0].header.stamp
            ps.header.frame_id = "odom"
            ps.pose.position.x = t.x - self._gt_origin_x
            ps.pose.position.y = t.y - self._gt_origin_y
            ps.pose.position.z = 0.0
            ps.pose.orientation = tf.transform.rotation
            self._gt_path.poses.append(ps)
            if len(self._gt_path.poses) > MAX_POSES:
                self._gt_path.poses.pop(0)
            self._gt_path.header.stamp = ps.header.stamp
            self.gt_pub.publish(self._gt_path)
            break


def main():
    rclpy.init()
    rclpy.spin(PathRecorder())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
