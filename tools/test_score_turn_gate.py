#!/usr/bin/env python3
"""Synthetic tests for the rate and angle turn gates in score_turn_gate.sim().

Regression tests for issue #157. No ROS or bag file is needed.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from score_turn_gate import sim

RATE = 0.3    # rad/s
ANGLE = 5.0   # degrees


def constant_speed(_t):
    return 1.0


class TestScoreTurnGate(unittest.TestCase):

    def test_straight_motion(self):
        # No yaw at all: neither gate latches, full distance is kept.
        gyr = [(i * 0.1, 0.0) for i in range(4)]

        rate_best, rate_latches = sim(gyr, constant_speed, "rate", RATE)
        angle_best, angle_latches = sim(gyr, constant_speed, "angle", ANGLE)

        self.assertEqual(rate_latches, 0)
        self.assertEqual(angle_latches, 0)
        self.assertAlmostEqual(rate_best, 0.3)
        self.assertAlmostEqual(angle_best, 0.3)

    def test_one_sample_yaw_spike(self):
        # One 1 rad/s sample for 0.02 s integrates to ~1.15 deg:
        # above the rate threshold, far below the angle threshold.
        gyr = [(0.0, 0.0), (0.02, 0.0), (0.04, 1.0), (0.06, 0.0)]

        rate_best, rate_latches = sim(gyr, constant_speed, "rate", RATE)
        angle_best, angle_latches = sim(gyr, constant_speed, "angle", ANGLE)

        self.assertEqual(rate_latches, 1)
        self.assertEqual(angle_latches, 0)
        self.assertAlmostEqual(rate_best, 0.04)    # distance before the spike
        self.assertAlmostEqual(angle_best, 0.06)   # never reset

    def test_sustained_90_degree_corner(self):
        # 90 deg/s held for 1 s (50 samples at 0.02 s) = 90 deg total.
        # Each sample adds 1.8 deg, so the angle gate must accumulate
        # 3 samples (5.4 deg > 5 deg) before it trips: 16 latches in 50 samples.
        gyr = [(0.0, 0.0)]
        for i in range(1, 51):
            gyr.append((i * 0.02, math.radians(90.0)))

        rate_best, rate_latches = sim(gyr, constant_speed, "rate", RATE)
        angle_best, angle_latches = sim(gyr, constant_speed, "angle", ANGLE)

        self.assertEqual(rate_latches, 50)
        self.assertEqual(angle_latches, 16)
        self.assertAlmostEqual(rate_best, 0.02)    # reset every sample
        self.assertAlmostEqual(angle_best, 0.06)   # reset every 3 samples

    def test_signed_rocking(self):
        # +1, -1, +1, ... rad/s: rate gate trips every sample, but the signed
        # angle cancels (peak ~1.15 deg), so the angle gate never trips.
        # If the code integrated abs(wz), this would trip the angle gate.
        gyr = [(0.0, 0.0)]
        for i in range(1, 11):
            wz = 1.0 if i % 2 else -1.0
            gyr.append((i * 0.02, wz))

        rate_best, rate_latches = sim(gyr, constant_speed, "rate", RATE)
        angle_best, angle_latches = sim(gyr, constant_speed, "angle", ANGLE)

        self.assertEqual(rate_latches, 10)
        self.assertEqual(angle_latches, 0)
        self.assertAlmostEqual(rate_best, 0.02)
        self.assertAlmostEqual(angle_best, 0.20)


if __name__ == "__main__":
    unittest.main()