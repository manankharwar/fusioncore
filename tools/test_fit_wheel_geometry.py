"""Tests for fit_wheel_geometry.py.

This tool produces a number that everything downstream rests on: a wrong wheel
radius is a scale error, and a 29.7% one went unnoticed on NCLT for months while
quietly taxing every turn. So it is tested against synthetic data with a KNOWN
answer, and tested to refuse a fit it should not believe.

All of it runs with no bag and no ROS: read_joints is the only part that needs
either, and it is substituted.
"""

import importlib.util
import math
import pathlib
import sys
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location(
    "fit_wheel_geometry", ROOT / "tools" / "fit_wheel_geometry.py")
fwg = importlib.util.module_from_spec(spec)
sys.modules["fit_wheel_geometry"] = fwg
spec.loader.exec_module(fwg)

TRUE_R = 0.3135          # a plausible car wheel radius, m
TRUE_WB = 1.5820         # a plausible rear track width, m
HZ = 100.0


def synth(duration=120.0, r=TRUE_R, wb=TRUE_WB, noise_m=0.0):
    """A trajectory driven by known wheel velocities, so the answer is known.

    Returns (gt_rows, joint_rows). The wheels drive the motion rather than being
    derived from it, which is the right direction: it means the fit has to invert the
    same relationship a real vehicle does.
    """
    n = int(duration * HZ)
    dt = 1.0 / HZ
    x = y = yaw = 0.0
    gt, joints = [], []
    for i in range(n):
        t = i * dt
        # a profile with straights AND turns, both directions, so BOTH stages of the
        # fit have signal. A single constant turn would fit the radius and leave the
        # track width unidentifiable.
        phase = (i // int(15 * HZ)) % 4
        if phase == 0:
            wl = wr = 20.0                    # straight
        elif phase == 1:
            wl, wr = 18.0, 22.0               # left-ish turn
        elif phase == 2:
            wl = wr = 20.0
        else:
            wl, wr = 22.0, 18.0               # the other way
        vx = (wl * r + wr * r) / 2.0
        wz = (wr * r - wl * r) / wb
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw += wz * dt
        gx, gy = x, y
        if noise_m:
            # deterministic, bounded, and uncorrelated-looking: a fixed pattern beats
            # a seeded RNG here because the test must not drift between runs.
            gx += noise_m * math.sin(i * 1.7)
            gy += noise_m * math.cos(i * 2.3)
        gt.append((t, gx, gy, yaw))
        joints.append((t, wl, wr))
    return gt, joints


def write_tum(path, gt):
    with open(path, "w") as fh:
        for t, x, y, yaw in gt:
            qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            fh.write(f"{t:.9f} {x:.6f} {y:.6f} 0.0 0.0 0.0 {qz:.9f} {qw:.9f}\n")


def run_fit(gt, joints, extra=()):
    with tempfile.TemporaryDirectory() as d:
        p = pathlib.Path(d) / "gt.tum"
        write_tum(p, gt)
        argv = ["fit_wheel_geometry.py", "/fake/bag", "--gt", str(p), "--json", *extra]
        import io
        from contextlib import redirect_stdout
        buf = io.StringIO()
        with mock.patch.object(fwg, "read_joints", lambda b: joints), \
             mock.patch.object(sys, "argv", argv), redirect_stdout(buf):
            code = fwg.main()
        import json as J
        return code, J.loads(buf.getvalue())


class LeastSquaresTest(unittest.TestCase):

    def test_recovers_a_known_slope_through_the_origin(self):
        xs = [1.0, 2.0, 3.0, 4.0]
        ys = [2.5, 5.0, 7.5, 10.0]
        m, r2, rms = fwg.lstsq_through_origin(xs, ys)
        self.assertAlmostEqual(m, 2.5, places=9)
        self.assertAlmostEqual(r2, 1.0, places=9)
        self.assertAlmostEqual(rms, 0.0, places=9)

    def test_no_intercept_is_deliberate(self):
        # Zero wheel rotation must mean zero speed. A fit WITH an intercept would
        # absorb an offset that cannot physically exist and flatter the result.
        xs = [1.0, 2.0, 3.0]
        ys = [11.0, 12.0, 13.0]           # slope 1, intercept 10
        m, r2, _ = fwg.lstsq_through_origin(xs, ys)
        self.assertGreater(m, 4.0, "through-origin must not recover slope 1 here")
        self.assertLess(r2, 1.0)

    def test_random_data_does_not_fit(self):
        xs = [math.sin(i * 1.1) for i in range(200)]
        ys = [math.cos(i * 2.7) for i in range(200)]
        _, r2, _ = fwg.lstsq_through_origin(xs, ys)
        self.assertLess(r2, 0.5)


class GroundTruthVelocityTest(unittest.TestCase):

    def test_recovers_speed_and_yaw_rate_from_a_trajectory(self):
        gt, _ = synth(duration=60.0)
        vels = fwg.gt_velocities(gt, 0.5)
        self.assertGreater(len(vels), 1000)
        # on the straight phases the true speed is r*20
        expect = TRUE_R * 20.0
        straight = [v for _, v, w in vels if abs(w) < 1e-3]
        self.assertTrue(straight)
        self.assertAlmostEqual(sum(straight) / len(straight), expect, delta=0.02)

    def test_a_window_shorter_than_the_floor_is_refused(self):
        gt, joints = synth(duration=30.0)
        with self.assertRaises(SystemExit):
            run_fit(gt, joints, extra=["--window", "0.05"])


class EndToEndTest(unittest.TestCase):

    def test_recovers_both_parameters_on_clean_data(self):
        gt, joints = synth(duration=180.0)
        code, o = run_fit(gt, joints)
        self.assertEqual(code, 0)
        self.assertEqual(o["verdict"], "OK")
        self.assertAlmostEqual(o["radius_m"], TRUE_R, delta=0.002)
        self.assertAlmostEqual(o["track_width_m"], TRUE_WB, delta=0.02)
        self.assertGreater(o["radius_r2"], 0.99)
        self.assertGreater(o["track_r2"], 0.99)

    def test_survives_centimetre_ground_truth_noise(self):
        # 1 cm of RTK noise is the realistic case, and the whole reason velocities
        # are taken over a window instead of between adjacent samples.
        gt, joints = synth(duration=180.0, noise_m=0.01)
        code, o = run_fit(gt, joints)
        self.assertEqual(code, 0)
        self.assertAlmostEqual(o["radius_m"], TRUE_R, delta=0.01)

    def test_a_wrong_model_is_refused_not_reported(self):
        # Wheel velocities unrelated to the motion. There is no radius to find, and
        # the tool must say so rather than print whatever the algebra returns.
        gt, joints = synth(duration=120.0)
        scrambled = [(t, 5.0 + 3.0 * math.sin(i * 0.9), 5.0 + 3.0 * math.cos(i * 1.3))
                     for i, (t, _, _) in enumerate(joints)]
        code, o = run_fit(gt, scrambled)
        self.assertEqual(code, 1)
        self.assertEqual(o["verdict"], "REFUSED")
        self.assertIn("does not describe this data", o["reason"])

    def test_a_straight_only_sequence_fits_radius_but_not_track(self):
        # The urban25 case: 101 s at 91 km/h with almost no turning. The radius is
        # identifiable and the track width is not, and the tool must separate those
        # rather than reporting a track width nobody should use.
        n = int(120 * HZ)
        dt = 1.0 / HZ
        x = y = 0.0
        gt, joints = [], []
        for i in range(n):
            wl = wr = 20.0
            vx = TRUE_R * 20.0
            x += vx * dt
            gt.append((i * dt, x, y, 0.0))
            joints.append((i * dt, wl, wr))
        code, o = run_fit(gt, joints)
        self.assertEqual(o["verdict"], "PARTIAL")
        self.assertAlmostEqual(o["radius_m"], TRUE_R, delta=0.002)
        self.assertIn("too little turning", o["reason"])


if __name__ == "__main__":
    unittest.main()
