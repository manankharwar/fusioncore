#!/usr/bin/env python3
"""
FusionCore GPS Spike Injector
Press SPACE to inject a 500m GPS spike
Press Q to quit
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import threading
import sys
import termios
import tty
import time

# GPS origin (Hamilton ON: matches simulation)
ORIGIN_LAT = 43.2557
ORIGIN_LON = -79.8711
SPIKE_METERS = 500.0  # how far to inject the spike

class SpikeInjector(Node):
    def __init__(self):
        super().__init__('spike_injector')
        self.pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        self.sub = self.create_subscription(
            NavSatFix, '/gnss/fix_real', self.real_gps_cb, 10)
        self.last_real = None
        self.spike_active = False

    def real_gps_cb(self, msg):
        self.last_real = msg

    def inject_spike(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        # 500m north of current position
        msg.latitude  = ORIGIN_LAT + (SPIKE_METERS / 111320.0)
        msg.longitude = ORIGIN_LON
        msg.altitude  = 0.0
        msg.status.status = 0
        msg.status.service = 1
        msg.position_covariance_type = 1
        msg.position_covariance[0] = 0.25
        msg.position_covariance[4] = 0.25
        msg.position_covariance[8] = 0.25
        # Publish spike 3 times to make sure it's seen
        for _ in range(3):
            self.pub.publish(msg)
            time.sleep(0.05)
        print(f'\033[91m[SPIKE INJECTED] +{SPIKE_METERS}m north: watch FusionCore reject it!\033[0m')

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rclpy.init()
    node = SpikeInjector()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('\033[92m╔══════════════════════════════════════╗\033[0m')
    print('\033[92m║   FusionCore GPS Spike Injector      ║\033[0m')
    print('\033[92m╠══════════════════════════════════════╣\033[0m')
    print('\033[92m║  SPACE  →  inject +500m GPS spike    ║\033[0m')
    print('\033[92m║  Q      →  quit                      ║\033[0m')
    print('\033[92m╚══════════════════════════════════════╝\033[0m')
    print()

    while True:
        key = get_key()
        if key == ' ':
            node.inject_spike()
        elif key in ('q', 'Q', '\x03'):
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
