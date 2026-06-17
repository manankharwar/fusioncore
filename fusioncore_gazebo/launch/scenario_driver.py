#!/usr/bin/env python3
# Open-loop agricultural lawnmower driver.
# Executes a pre-computed segment list so the path is fully deterministic
# and jitter-free: no feedback loop, no oscillation.
#
# Pattern: 4 survey rows (East-West) spaced 5 m apart, 22 m long each.
# Robot starts at origin facing East.

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

_TURN = 0.9                          # rad/s pivot speed
_T90  = (math.pi / 2.0) / _TURN     # 1.745 s for a 90-degree pivot

# (linear_x  m/s, angular_z  rad/s, duration_s)
# yaw 0=East, 90=North, 180=West
_SEGMENTS = [
    # Row 1 East
    (1.0,    0.0,  22.0),
    # U-turn left: pivot CCW to North, north jog, pivot CCW to West
    (0.0,  _TURN,  _T90),
    (1.0,    0.0,   5.0),
    (0.0,  _TURN,  _T90),
    # Row 2 West
    (1.0,    0.0,  22.0),
    # U-turn right: pivot CW to North, north jog, pivot CW to East
    (0.0, -_TURN,  _T90),
    (1.0,    0.0,   5.0),
    (0.0, -_TURN,  _T90),
    # Row 3 East
    (1.0,    0.0,  22.0),
    # U-turn left
    (0.0,  _TURN,  _T90),
    (1.0,    0.0,   5.0),
    (0.0,  _TURN,  _T90),
    # Row 4 West
    (1.0,    0.0,  22.0),
    # Stop
    (0.0,    0.0,   6.0),
]


class ScenarioDriver(Node):
    def __init__(self):
        super().__init__("scenario_driver")
        self.declare_parameter("start_delay_s", 18.0)

        self._pub        = self.create_publisher(Twist, "/cmd_vel", 10)
        self._start_ns   = self.get_clock().now().nanoseconds
        self._last_ns    = self._start_ns
        self._seg_idx    = -1     # -1 = holding until start_delay
        self._seg_t      = 0.0
        self._started    = False

        self.create_timer(0.05, self._tick)   # 20 Hz

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        dt     = (now_ns - self._last_ns) * 1e-9
        self._last_ns = now_ns

        delay   = self.get_parameter("start_delay_s").get_parameter_value().double_value
        elapsed = (now_ns - self._start_ns) * 1e-9

        if elapsed < delay:
            return

        if not self._started:
            self._started  = True
            self._seg_idx  = 0
            self._seg_t    = 0.0
            self.get_logger().info("scenario_driver: lawnmower started")

        if self._seg_idx >= len(_SEGMENTS):
            self._pub.publish(Twist())
            return

        lin, ang, dur = _SEGMENTS[self._seg_idx]
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self._pub.publish(msg)

        self._seg_t += dt
        if self._seg_t >= dur:
            self._seg_t    = 0.0
            self._seg_idx += 1
            if self._seg_idx < len(_SEGMENTS):
                self.get_logger().info(
                    f"segment {self._seg_idx + 1}/{len(_SEGMENTS)}: "
                    f"lin={_SEGMENTS[self._seg_idx][0]:.1f} "
                    f"ang={_SEGMENTS[self._seg_idx][1]:.2f} "
                    f"dur={_SEGMENTS[self._seg_idx][2]:.1f}s")


def main():
    rclpy.init()
    rclpy.spin(ScenarioDriver())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
