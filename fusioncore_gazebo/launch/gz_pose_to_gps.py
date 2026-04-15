#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix, NavSatStatus
import math
import random

ORIGIN_LAT = 43.2557
ORIGIN_LON = -79.8711
ORIGIN_ALT = 100.0
A  = 6378137.0
E2 = 0.00669437999014
NOISE_H = 0.5   # 1-sigma horizontal GPS noise (m)
NOISE_V = 0.3   # 1-sigma vertical GPS noise (m)

def enu_to_lla(x, y, z):
    lat0 = math.radians(ORIGIN_LAT)
    lon0 = math.radians(ORIGIN_LON)
    alt0 = ORIGIN_ALT
    sl = math.sin(lat0); cl = math.cos(lat0)
    sn = math.sin(lon0); cn = math.cos(lon0)
    N0 = A / math.sqrt(1.0 - E2*sl*sl)
    X0 = (N0+alt0)*cl*cn; Y0 = (N0+alt0)*cl*sn; Z0 = (N0*(1-E2)+alt0)*sl
    dX = -sn*x - sl*cn*y + cl*cn*z
    dY =  cn*x - sl*sn*y + cl*sn*z
    dZ =  cl*y + sl*z
    Xp=X0+dX; Yp=Y0+dY; Zp=Z0+dZ
    p = math.sqrt(Xp*Xp+Yp*Yp)
    lat = math.atan2(Zp, p*(1.0-E2))
    for _ in range(5):
        s = math.sin(lat)
        N = A/math.sqrt(1.0-E2*s*s)
        lat = math.atan2(Zp+E2*N*s, p)
    s = math.sin(lat)
    N = A/math.sqrt(1.0-E2*s*s)
    alt = p/math.cos(lat)-N
    return math.degrees(lat), math.degrees(math.atan2(Yp,Xp)), alt

def _is_base_link(frame_id):
    # Matches "base_link", "fusioncore_robot::base_link", "fusioncore_robot/base_link"
    tail = frame_id.rsplit("::", 1)[-1].rsplit("/", 1)[-1]
    return tail == "base_link"

class GzPoseToGps(Node):
    def __init__(self):
        super().__init__("gz_pose_to_gps")
        self.pub = self.create_publisher(NavSatFix, "/gnss/fix", 10)
        self.sub = self.create_subscription(
            TFMessage, "/world/fusioncore_test/pose/info", self.pose_cb, 10)
        # child_frame_id of the link we've locked onto as the robot body.
        # Set on the first callback; used exclusively afterwards so that
        # the reference altitude and all subsequent ENU positions use
        # exactly the same link: preventing wheel-vs-body switch bias.
        self.body_frame_id = None
        # True after the very first NavSatFix is published.
        # The first publish uses z = best.z (no vertical noise) so that
        # fusion_node captures an accurate altitude reference.  All
        # subsequent publishes add NOISE_V to model realistic GPS error.
        self.ref_published = False
        self.get_logger().info(f"GPS publisher ready. Origin: {ORIGIN_LAT}, {ORIGIN_LON}")

    def _find_body(self, msg):
        """Return the translation of the robot body link.

        Priority:
          1. Previously locked-on frame (sticky: avoids inter-callback switching).
          2. Any frame whose tail name is "base_link" (robust across SDF / ROS naming).
          3. Max x²+y² magnitude fallback (original heuristic, kept for other models).
        """
        # 1. Sticky: reuse the frame identified on a previous callback
        if self.body_frame_id is not None:
            for tf in msg.transforms:
                if tf.child_frame_id == self.body_frame_id:
                    t = tf.transform.translation
                    if 0.05 < t.z < 0.4:
                        return t
            # If the sticky frame is no longer present / out of z range, fall through

        # 2. Name-based: find "base_link" regardless of namespace prefix
        for tf in msg.transforms:
            if _is_base_link(tf.child_frame_id):
                t = tf.transform.translation
                if 0.05 < t.z < 0.4:
                    self.body_frame_id = tf.child_frame_id
                    return t

        # 3. Fallback: original max-magnitude heuristic
        best = None
        best_mag = -1.0
        best_fid = None
        for tf in msg.transforms:
            t = tf.transform.translation
            if not (0.05 < t.z < 0.4):
                continue
            mag = t.x*t.x + t.y*t.y
            if mag > best_mag:
                best_mag = mag
                best = t
                best_fid = tf.child_frame_id
        if best_fid is not None:
            self.body_frame_id = best_fid
        return best

    def pose_cb(self, msg):
        best = self._find_body(msg)
        if best is None:
            return

        x = best.x + random.gauss(0, NOISE_H)
        y = best.y + random.gauss(0, NOISE_H)
        # First publish: use exact z so fusion_node's altitude reference has no bias.
        # Without this, a random noise_first spike permanently offsets every ENU z
        # reading for the entire run (ENU z = noise_i - noise_first).
        if not self.ref_published:
            z = best.z
            self.ref_published = True
        else:
            z = best.z + random.gauss(0, NOISE_V)

        lat, lon, alt = enu_to_lla(x, y, z)

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = "gnss_link"
        fix.status.status  = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat; fix.longitude = lon; fix.altitude = alt
        fix.position_covariance = [NOISE_H**2,0,0, 0,NOISE_H**2,0, 0,0,NOISE_V**2]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(fix)

def main():
    rclpy.init()
    rclpy.spin(GzPoseToGps())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
