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
NOISE_H = 1.0
NOISE_V = 2.0

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

class GzPoseToGps(Node):
    def __init__(self):
        super().__init__("gz_pose_to_gps")
        self.pub = self.create_publisher(NavSatFix, "/gnss/fix", 10)
        self.sub = self.create_subscription(
            TFMessage, "/world/fusioncore_test/pose/info", self.pose_cb, 10)
        # Track the largest-magnitude x,y position seen — that is the robot body
        # in world frame. Local link offsets (wheels, imu) are always small (<0.5m).
        # The robot body accumulates position as it drives.
        self.last_body_x = None
        self.last_body_y = None
        self.get_logger().info(f"GPS publisher ready. Origin: {ORIGIN_LAT}, {ORIGIN_LON}")

    def pose_cb(self, msg):
        # The robot body transform is the one with the largest x^2+y^2 magnitude
        # among all transforms with z between 0.05 and 0.4m.
        # Local link offsets (wheels=0.22m, imu=0.1m) are always < 0.5m from origin.
        # The robot body position grows as the robot drives.
        best = None
        best_mag = -1.0
        for tf in msg.transforms:
            t = tf.transform.translation
            # Must be at ground height
            if not (0.05 < t.z < 0.4):
                continue
            mag = t.x*t.x + t.y*t.y
            if mag > best_mag:
                best_mag = mag
                best = t

        if best is None:
            return

        x = best.x + random.gauss(0, NOISE_H)
        y = best.y + random.gauss(0, NOISE_H)
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
