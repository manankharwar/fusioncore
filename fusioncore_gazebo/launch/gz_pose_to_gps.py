#!/usr/bin/env python3
# Converts Gazebo ground truth pose to GPS outputs with configurable disruptions.
#
# Publishes:
#   /gnss/fix          (sensor_msgs/NavSatFix)   for FusionCore
#   /gps/odometry      (nav_msgs/Odometry)        ENU for robot_localization
#   /gps/status_marker (visualization_msgs/Marker) RViz text overlay
#
# Disruption schedule (times relative to first published fix):
#   Spike 1:  spike_at_s  for spike_duration_s   (+spike_dx_m east)
#   Outage:   outage_at_s for outage_duration_s  (no output at all)
#   Spike 2:  spike2_at_s for spike2_duration_s  (+spike2_dx_m, from any direction)

import math
import random
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

ORIGIN_LAT = 43.2557
ORIGIN_LON = -79.8711
ORIGIN_ALT = 100.0
A  = 6378137.0
E2 = 0.00669437999014


def enu_to_lla(x, y, z):
    lat0 = math.radians(ORIGIN_LAT)
    lon0 = math.radians(ORIGIN_LON)
    alt0 = ORIGIN_ALT
    sl = math.sin(lat0); cl = math.cos(lat0)
    sn = math.sin(lon0); cn = math.cos(lon0)
    N0 = A / math.sqrt(1.0 - E2 * sl * sl)
    X0 = (N0 + alt0) * cl * cn
    Y0 = (N0 + alt0) * cl * sn
    Z0 = (N0 * (1 - E2) + alt0) * sl
    dX = -sn * x - sl * cn * y + cl * cn * z
    dY =  cn * x - sl * sn * y + cl * sn * z
    dZ =  cl * y + sl * z
    Xp = X0 + dX; Yp = Y0 + dY; Zp = Z0 + dZ
    p = math.sqrt(Xp * Xp + Yp * Yp)
    lat = math.atan2(Zp, p * (1.0 - E2))
    for _ in range(5):
        s = math.sin(lat)
        N = A / math.sqrt(1.0 - E2 * s * s)
        lat = math.atan2(Zp + E2 * N * s, p)
    s = math.sin(lat)
    N = A / math.sqrt(1.0 - E2 * s * s)
    alt = p / math.cos(lat) - N
    return math.degrees(lat), math.degrees(math.atan2(Yp, Xp)), alt


def _is_base_link(frame_id):
    tail = frame_id.rsplit("::", 1)[-1].rsplit("/", 1)[-1]
    return tail == "base_link"


class GzPoseToGps(Node):
    def __init__(self):
        super().__init__("gz_pose_to_gps")

        # World / noise
        self.declare_parameter("world_name",         "fusioncore_outdoor")
        self.declare_parameter("noise_h",             0.5)
        self.declare_parameter("noise_v",             0.3)
        # Robot identification. The ros_gz bridge publishes pose/info with empty
        # frame_ids, so the robot cannot be found by name. It is instead found by
        # its model height: the robot base sits at body_z and is the only entity
        # in the world at that height (scenery is taller, link poses are at 0).
        self.declare_parameter("body_z",              0.15)
        self.declare_parameter("body_z_tol",          0.03)
        # Spike 1
        self.declare_parameter("spike_at_s",         -1.0)   # <0 disables
        self.declare_parameter("spike_duration_s",    8.0)
        self.declare_parameter("spike_dx_m",         60.0)
        self.declare_parameter("spike_dy_m",          0.0)
        # GPS blackout
        self.declare_parameter("outage_at_s",        -1.0)   # <0 disables
        self.declare_parameter("outage_duration_s",  25.0)
        # Spike 2
        self.declare_parameter("spike2_at_s",        -1.0)   # <0 disables
        self.declare_parameter("spike2_duration_s",   6.0)
        self.declare_parameter("spike2_dx_m",        -60.0)  # opposite direction
        self.declare_parameter("spike2_dy_m",          0.0)

        world = self.get_parameter("world_name").get_parameter_value().string_value

        self.pub_fix    = self.create_publisher(NavSatFix, "/gnss/fix",          10)
        self.pub_odom   = self.create_publisher(Odometry,  "/gps/odometry",      10)
        self.pub_marker = self.create_publisher(Marker,    "/gps/status_marker", 10)
        self.sub = self.create_subscription(
            TFMessage, f"/world/{world}/pose/info", self.pose_cb, 10)

        self.body_frame_id = None
        self._last_xy      = None
        self.ref_published = False
        self.start_ns      = None
        self._last_status  = ""

        self.get_logger().info(f"GPS publisher ready (world={world})")

    # ── elapsed time helpers ──────────────────────────────────────────

    def _elapsed(self):
        if self.start_ns is None:
            return 0.0
        return (self.get_clock().now().nanoseconds - self.start_ns) * 1e-9

    def _window(self, at_param, dur_param):
        at = self.get_parameter(at_param).get_parameter_value().double_value
        if at < 0.0:
            return False
        dur = self.get_parameter(dur_param).get_parameter_value().double_value
        e = self._elapsed()
        return at <= e < (at + dur)

    def _outage_active(self):
        return self._window("outage_at_s", "outage_duration_s")

    def _spike_active(self):
        return self._window("spike_at_s", "spike_duration_s")

    def _spike2_active(self):
        return self._window("spike2_at_s", "spike2_duration_s")

    # ── body tracking ────────────────────────────────────────────────

    def _find_body(self, msg):
        # If frame_ids are populated (older bridge / other worlds), prefer the
        # named base_link. This keeps the node working across bridge versions.
        for tf in msg.transforms:
            if tf.child_frame_id and _is_base_link(tf.child_frame_id):
                return tf.transform.translation

        # Otherwise identify the robot by its model height. Among the entities
        # whose world pose sits at body_z (only the robot does), pick the one
        # nearest the last known robot position for continuity. Scenery (crop
        # rows at 0.25, fences at 0.75+, trees higher) and the zeroed relative
        # link poses are all excluded by the height window.
        z0  = self.get_parameter("body_z").get_parameter_value().double_value
        tol = self.get_parameter("body_z_tol").get_parameter_value().double_value
        cands = [tf.transform.translation for tf in msg.transforms
                 if abs(tf.transform.translation.z - z0) <= tol]
        if not cands:
            return None
        ref = self._last_xy if self._last_xy is not None else (0.0, 0.0)
        best = min(cands, key=lambda t: (t.x - ref[0]) ** 2 + (t.y - ref[1]) ** 2)
        self._last_xy = (best.x, best.y)
        return best

    # ── status marker ─────────────────────────────────────────────────

    def _publish_status(self, label: str, r: float, g: float, b: float):
        if label == self._last_status:
            return
        self._last_status = label
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.ns   = "gps_status"
        m.id   = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = -18.0
        m.pose.position.y =  28.0
        m.pose.position.z =   3.0
        m.pose.orientation.w = 1.0
        m.scale.z = 2.5
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
        m.text = label
        self.pub_marker.publish(m)

    # ── main callback ────────────────────────────────────────────────

    def pose_cb(self, msg):
        best = self._find_body(msg)
        if best is None:
            return

        if self.start_ns is None:
            self.start_ns = self.get_clock().now().nanoseconds

        noise_h  = self.get_parameter("noise_h").get_parameter_value().double_value
        noise_v  = self.get_parameter("noise_v").get_parameter_value().double_value

        # Determine GPS state
        outage  = self._outage_active()
        spike1  = self._spike_active()
        spike2  = self._spike2_active()

        if outage:
            self._publish_status("  GPS OUTAGE  ", 0.95, 0.45, 0.0)
            if not hasattr(self, "_outage_logged"):
                self.get_logger().warn("GPS OUTAGE ACTIVE: no fixes being published")
                self._outage_logged = True
            return   # publish nothing during blackout
        elif hasattr(self, "_outage_logged"):
            del self._outage_logged
            self.get_logger().info("GPS outage ended, fixes resuming")

        if spike1:
            dx = self.get_parameter("spike_dx_m").get_parameter_value().double_value
            dy = self.get_parameter("spike_dy_m").get_parameter_value().double_value
            self._publish_status("  GPS SPIKE +60 m  ", 0.95, 0.1, 0.1)
            if not hasattr(self, "_spike1_logged"):
                self.get_logger().warn(f"GPS SPIKE 1 ACTIVE: +{dx:.0f} m East")
                self._spike1_logged = True
        elif hasattr(self, "_spike1_logged"):
            del self._spike1_logged
            self.get_logger().info("GPS spike 1 ended")
            dx, dy = 0.0, 0.0
        elif spike2:
            dx = self.get_parameter("spike2_dx_m").get_parameter_value().double_value
            dy = self.get_parameter("spike2_dy_m").get_parameter_value().double_value
            self._publish_status("  GPS SPIKE -60 m  ", 0.95, 0.1, 0.1)
            if not hasattr(self, "_spike2_logged"):
                self.get_logger().warn(f"GPS SPIKE 2 ACTIVE: {dx:.0f} m East")
                self._spike2_logged = True
        elif hasattr(self, "_spike2_logged"):
            del self._spike2_logged
            self.get_logger().info("GPS spike 2 ended")
            dx, dy = 0.0, 0.0
        else:
            self._publish_status("  GPS OK  ", 0.1, 0.9, 0.1)
            dx, dy = 0.0, 0.0

        x = best.x + random.gauss(0, noise_h) + dx
        y = best.y + random.gauss(0, noise_h) + dy
        z = best.z if not self.ref_published else best.z + random.gauss(0, noise_v)

        now = self.get_clock().now().to_msg()

        # NavSatFix for FusionCore (chi2 gate rejects spikes)
        lat, lon, alt = enu_to_lla(x, y, z)
        fix = NavSatFix()
        fix.header.stamp    = now
        fix.header.frame_id = "gnss_link"
        fix.status.status   = NavSatStatus.STATUS_FIX
        fix.status.service  = NavSatStatus.SERVICE_GPS
        fix.latitude  = lat
        fix.longitude = lon
        fix.altitude  = alt
        fix.position_covariance = [
            noise_h ** 2, 0, 0,
            0, noise_h ** 2, 0,
            0, 0, noise_v ** 2,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)

        # ENU Odometry for robot_localization (no navsat_transform_node needed)
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        var = noise_h ** 2
        odom.pose.covariance[0]  = var
        odom.pose.covariance[7]  = var
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 1e6
        self.pub_odom.publish(odom)

        self.ref_published = True


def main():
    rclpy.init()
    rclpy.spin(GzPoseToGps())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
