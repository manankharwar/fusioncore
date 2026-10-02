#!/usr/bin/env python3
"""Tests for spike_injector.py: the spike it publishes is what its docstring says.

No ROS needed. rclpy and the message packages are replaced with small fakes, so
this checks the position, covariance and publish count, not ROS 2 itself.
Run with: python3 -m unittest tools/test_spike_injector.py
"""

import contextlib
import io
import math
import os
import sys
import types
import unittest
from types import SimpleNamespace
from unittest import mock

METRES_PER_DEGREE = 111320.0


class FakeNavSatFix:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.status = SimpleNamespace(status=0, service=0)
        self.position_covariance_type = 0
        self.position_covariance = [0.0] * 9


class FakePublisher:
    def __init__(self, topic, sink):
        self.topic = topic
        self.sink = sink

    def publish(self, msg):
        self.sink.append((self.topic, msg))


class FakeNode:
    def __init__(self, name):
        self.published = []

    def create_publisher(self, msg_type, topic, depth):
        return FakePublisher(topic, self.published)

    def create_subscription(self, msg_type, topic, callback, depth):
        return None

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp"))


def fake_module(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    return mod


STUBS = {
    "rclpy": fake_module("rclpy"),
    "rclpy.node": fake_module("rclpy.node", Node=FakeNode),
    "sensor_msgs": fake_module("sensor_msgs"),
    "sensor_msgs.msg": fake_module("sensor_msgs.msg", NavSatFix=FakeNavSatFix),
    "nav_msgs": fake_module("nav_msgs"),
    "nav_msgs.msg": fake_module("nav_msgs.msg", Odometry=object),
}

sys.path.insert(0, os.path.dirname(__file__))
# The fakes only live while the import runs, so they never leak into other tests.
with mock.patch.dict(sys.modules, STUBS):
    import spike_injector  # noqa: E402


def inject(node=None):
    """Press SPACE once. Returns the (topic, message) pairs that were published."""
    node = node or spike_injector.SpikeInjector()
    with mock.patch.object(spike_injector.time, "sleep"), \
            contextlib.redirect_stdout(io.StringIO()):
        node.inject_spike()
    return node.published


def metres_north(lat):
    return (lat - spike_injector.ORIGIN_LAT) * METRES_PER_DEGREE


class SpikeInjectorTest(unittest.TestCase):
    def test_publishes_three_identical_copies_on_gnss_fix(self):
        published = inject()
        self.assertEqual(["/gnss/fix"] * 3, [topic for topic, _ in published])
        self.assertEqual(1, len({id(msg) for _, msg in published}))

    def test_spike_is_500_metres_due_north_of_the_fixed_origin(self):
        msg = inject()[0][1]
        self.assertAlmostEqual(500.0, metres_north(msg.latitude), places=6)
        self.assertEqual(spike_injector.ORIGIN_LON, msg.longitude)

    def test_magnitude_reaches_the_code(self):
        # Sweep one knob: a size that never reached the position would show here.
        for metres in (0.0, 100.0, 2500.0):
            with self.subTest(metres=metres), \
                    mock.patch.object(spike_injector, "SPIKE_METERS", metres):
                msg = inject()[0][1]
                self.assertAlmostEqual(metres, metres_north(msg.latitude), places=6)

    def test_covariance_stays_small_so_the_spike_is_adversarial(self):
        msg = inject()[0][1]
        self.assertEqual([0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.25],
                         msg.position_covariance)
        self.assertEqual(1, msg.position_covariance_type)
        sigma = math.sqrt(msg.position_covariance[0])
        self.assertEqual(0.5, sigma)
        self.assertGreater(spike_injector.SPIKE_METERS / sigma, 100)

    def test_the_last_real_fix_is_ignored(self):
        node = spike_injector.SpikeInjector()
        node.real_gps_cb(SimpleNamespace(latitude=10.0, longitude=20.0))
        msg = inject(node)[0][1]
        self.assertAlmostEqual(500.0, metres_north(msg.latitude), places=6)
        self.assertEqual(spike_injector.ORIGIN_LON, msg.longitude)


if __name__ == "__main__":
    unittest.main()
