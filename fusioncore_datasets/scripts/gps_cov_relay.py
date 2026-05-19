#!/usr/bin/env python3
"""
Relay node that republishes /gnss/fix with inflated position covariance.

Used in benchmarks to give robot_localization a more realistic GPS noise
model without changing the covariance seen by FusionCore.

FusionCore:         /gnss/fix          (var_xy=9,  sigma=3m, stated spec)
RL navsat_transform: /gnss/fix_rl      (var_xy=35, sigma=5.9m, closer to actual)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import copy


class GpsCovRelay(Node):
    def __init__(self):
        super().__init__('gps_cov_relay')
        self.declare_parameter('var_xy', 35.0)
        self.declare_parameter('var_z',  25.0)

        var_xy = self.get_parameter('var_xy').value
        var_z  = self.get_parameter('var_z').value
        self._cov = [
            var_xy, 0.0,  0.0,
            0.0, var_xy,  0.0,
            0.0,    0.0, var_z,
        ]

        self._sub = self.create_subscription(
            NavSatFix, '/gnss/fix', self._cb, 10)
        self._pub = self.create_publisher(NavSatFix, '/gnss/fix_rl', 10)

    def _cb(self, msg: NavSatFix):
        out = copy.copy(msg)
        out.position_covariance = self._cov
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GpsCovRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
