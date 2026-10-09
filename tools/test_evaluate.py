#!/usr/bin/env python3
"""Tests for evaluate.py, which produces the numbers check_benchmark_regression.py gates on.

Issue #155. The checker had a test and the thing feeding it did not, so the gate was only
as good as an unmeasured ATE. Everything here builds its trajectories in the test, so no
dataset, no ROS and no NCLT download.

Read this before changing compute_ate: the alignment is what makes most of these numbers
counter-intuitive. evaluate.py aligns the estimate to the ground truth with Umeyama SE(3)
and correct_scale=False, so a rigid error (a constant offset, a rotation about any point)
is absorbed completely and scores zero, while a scale error survives. That is deliberate
and both halves are pinned below.
"""
import json
import os
import subprocess
import sys
import tempfile
import unittest

import numpy as np

os.environ.setdefault("MPLBACKEND", "Agg")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import check_benchmark_regression

# evo is a hard requirement of evaluate.py, and CI installs it and fails loudly if
# that install does not work, so this never silently skips there. The guard is for a
# developer who has not installed evo yet: a clear skip beats an import error that
# looks like a broken test. It must not become the normal path.
try:
    import evaluate
    from evo.core.geometry import GeometryException
    from evo.core.sync import SyncException
    from evo.core.trajectory import PoseTrajectory3D
    EVO = None
except ImportError as exc:  # pragma: no cover
    EVO = f"evo is not installed ({exc}). pip install evo --break-system-packages"

TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "evaluate.py")


def traj(xyz, timestamps=None):
    """A PoseTrajectory3D with identity orientation. ATE translation ignores rotation."""
    xyz = np.asarray(xyz, dtype=float)
    if timestamps is None:
        timestamps = np.arange(len(xyz), dtype=float)
    return PoseTrajectory3D(
        positions_xyz=xyz,
        orientations_quat_wxyz=np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (len(xyz), 1)),
        timestamps=np.asarray(timestamps, dtype=float),
    )


def curve(n=60):
    """A path with curvature in all three axes.

    Umeyama needs the point cloud to span more than a line. A dead straight run with
    zero Y and Z gives a rank deficient covariance and alignment raises, which
    test_a_straight_line_cannot_be_aligned pins, so every other case here curves.
    """
    i = np.arange(n)
    return np.column_stack([i * 1.0, np.sin(i * 0.2) * 4.0, np.cos(i * 0.15) * 2.0])


def write_tum(path, xyz, timestamps=None):
    xyz = np.asarray(xyz, dtype=float)
    if timestamps is None:
        timestamps = np.arange(len(xyz), dtype=float)
    with open(path, "w") as f:
        for t, (x, y, z) in zip(timestamps, xyz):
            f.write(f"{t:.6f} {x:.6f} {y:.6f} {z:.6f} 0.0 0.0 0.0 1.0\n")


@unittest.skipIf(EVO, EVO or "")
class AlignmentContract(unittest.TestCase):
    """What SE(3) alignment absorbs, and what it must not."""

    def test_identical_trajectories_score_zero(self):
        g = curve()
        self.assertAlmostEqual(0.0, evaluate.compute_ate(traj(g), traj(g))["rmse"], places=9)

    def test_a_constant_offset_is_absorbed_and_scores_zero(self):
        # The issue text for #155 predicted 1 m here. It is 0, and that is correct:
        # a pure translation is exactly what the t term of an SE(3) fit removes.
        # Anyone reading an ATE as "how far off was it on average" will be wrong
        # about a biased trajectory, which is why this case is written down.
        g = curve()
        r = evaluate.compute_ate(traj(g), traj(g + np.array([1.0, 0.0, 0.0])))
        self.assertAlmostEqual(0.0, r["rmse"], places=6)
        self.assertAlmostEqual(0.0, r["xy_rmse"], places=6)

    def test_a_rotation_about_the_origin_is_absorbed(self):
        g = curve()
        th = 0.3
        rot = np.array([[np.cos(th), -np.sin(th), 0.0],
                        [np.sin(th), np.cos(th), 0.0],
                        [0.0, 0.0, 1.0]])
        self.assertAlmostEqual(
            0.0, evaluate.compute_ate(traj(g), traj(g @ rot.T))["rmse"], places=6)

    def test_a_scale_error_is_NOT_absorbed(self):
        # align() passes correct_scale=False on purpose. A wheel radius that is 10%
        # wrong produces a trajectory 10% too long, and that has to show up in the
        # score rather than being fitted away. Pinning it because the opposite
        # default would silently hide exactly the class of error being chased in #150.
        g = curve()
        r = evaluate.compute_ate(traj(g), traj(g * 1.1))
        self.assertGreater(r["rmse"], 1.0)

    def test_a_non_rigid_error_survives_alignment(self):
        # No single rigid transform fits both halves, so something must remain.
        g = curve()
        e = g.copy()
        e[:len(g) // 2, 1] += 1.0
        e[len(g) // 2:, 1] -= 1.0
        self.assertGreater(evaluate.compute_ate(traj(g), traj(e))["rmse"], 0.1)

    def test_a_straight_line_cannot_be_aligned(self):
        # Documents why every other case here curves. A degenerate cloud raises
        # rather than returning a meaningless number, which is the right behaviour.
        n = 40
        line = np.column_stack([np.arange(n) * 1.0, np.zeros(n), np.zeros(n)])
        with self.assertRaises(GeometryException):
            evaluate.compute_ate(traj(line), traj(line + np.array([0.5, 0.0, 0.0])))


@unittest.skipIf(EVO, EVO or "")
class XyNeverExceedsThreeD(unittest.TestCase):
    """The invariant that would have caught the corrupt baseline entries.

    Both metrics come from the same aligned trajectory and the same matched ground
    truth, so pointwise the horizontal error is a leg of the triangle whose
    hypotenuse is the 3D error. XY can never be the larger of the two, and an
    entry where it is did not come from one run of this code.

    tools/benchmark_baseline.json holds two such entries. 2012-08-20 records
    xy 121.669 against 3d 116.444, and 2013-04-05 records xy 200.91 against
    3d 189.7. Both gaps sit inside each sequence's own run to run spread, so the
    fields were filled from different runs rather than miscomputed.
    """

    def test_holds_on_randomised_trajectories(self):
        rng = np.random.default_rng(7)
        worst = -np.inf
        for _ in range(200):
            n = int(rng.integers(20, 80))
            i = np.arange(n)
            g = np.column_stack([
                i * rng.uniform(0.5, 2.0),
                np.sin(i * rng.uniform(0.05, 0.4)) * rng.uniform(1.0, 8.0),
                rng.normal(0.0, 1.0, n),
            ])
            e = g + rng.normal(0.0, rng.uniform(0.1, 5.0), g.shape)
            r = evaluate.compute_ate(traj(g), traj(e))
            worst = max(worst, r["xy_rmse"] - r["rmse"])
        self.assertLessEqual(worst, 0.0, f"xy_rmse exceeded ate_rmse_3d by {worst}")

    def test_holds_when_the_error_is_purely_vertical(self):
        # The widest the two can be apart: all error in Z, so XY is zero.
        g = curve()
        e = g.copy()
        e[:, 2] += np.linspace(0.0, 20.0, len(g))
        r = evaluate.compute_ate(traj(g), traj(e))
        self.assertGreater(r["rmse"], r["xy_rmse"])


@unittest.skipIf(EVO, EVO or "")
class TimestampAssociation(unittest.TestCase):
    """Association is exercised THROUGH compute_ate, deliberately.

    An earlier version of this class called sync.associate_trajectories directly.
    That tests evo, not evaluate.py: widening evaluate.py's own max_diff from 0.1 to
    1000 s left every case green. Mutation testing caught it. Everything here now
    goes through the function under test, so the tolerance is actually guarded.
    """

    def test_samples_inside_the_tolerance_are_matched(self):
        g = curve()
        # A 20 ms lag is well inside 0.1 s, so every pose pairs with its own partner
        # and a trajectory compared against itself still scores zero.
        r = evaluate.compute_ate(traj(g), traj(g, np.arange(len(g)) + 0.02))
        self.assertAlmostEqual(0.0, r["rmse"], places=6)

    def test_an_estimate_covering_part_of_the_run_matches_only_that_part(self):
        g = curve()
        half = len(g) // 2
        r = evaluate.compute_ate(traj(g), traj(g[:half], np.arange(half, dtype=float)))
        self.assertEqual(half, len(r["errors"]))

    def test_samples_with_no_partner_are_dropped_rather_than_paired(self):
        # The failure that matters: pairing a pose with a partner hundreds of seconds
        # away produces a confident and meaningless number instead of dropping it.
        # The estimate here is the ground truth plus ten junk poses far in the future.
        # Dropped, the score is zero. Paired, it is not.
        g = curve()
        junk = np.tile(np.array([999.0, 999.0, 999.0]), (10, 1))
        est_xyz = np.vstack([g, junk])
        est_t = np.concatenate([np.arange(len(g), dtype=float),
                                np.arange(500.0, 510.0)])
        r = evaluate.compute_ate(traj(g), traj(est_xyz, est_t))
        self.assertEqual(len(g), len(r["errors"]))
        self.assertAlmostEqual(0.0, r["rmse"], places=6)

    def test_no_overlap_at_all_raises_rather_than_scoring_nothing(self):
        # A run compared against a trajectory from a different session must fail
        # loudly. Silently returning a number here is how two unrelated things get
        # compared and believed.
        g = curve()
        with self.assertRaises(SyncException):
            evaluate.compute_ate(traj(g), traj(g, np.arange(len(g)) + 500.0))


@unittest.skipIf(EVO, EVO or "")
class MetricsJsonContract(unittest.TestCase):
    """metrics.json is the interface to check_benchmark_regression.py.

    The checker reads data['sequence'], filters['FusionCore'][METRIC] and
    filters['RL-EKF'][CONTROL_METRIC]. If evaluate.py stops emitting any of those
    the gate cannot run, and a gate that cannot run must never look like one that
    passed.
    """

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        d = cls.tmp.name
        g = curve()
        rng = np.random.default_rng(3)
        write_tum(os.path.join(d, "gt.tum"), g)
        write_tum(os.path.join(d, "fc.tum"), g + rng.normal(0, 0.5, g.shape))
        write_tum(os.path.join(d, "rl.tum"), g + rng.normal(0, 2.0, g.shape))
        cls.out = os.path.join(d, "res")
        cls.proc = subprocess.run(
            [sys.executable, TOOL,
             "--gt", os.path.join(d, "gt.tum"),
             "--fusioncore", os.path.join(d, "fc.tum"),
             "--rl", os.path.join(d, "rl.tum"),
             "--sequence", "synthetic", "--out_dir", cls.out],
            capture_output=True, text=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_the_run_succeeds(self):
        self.assertEqual(0, self.proc.returncode, self.proc.stderr[-2000:])

    def test_metrics_json_is_written(self):
        self.assertTrue(os.path.isfile(os.path.join(self.out, "metrics.json")))

    def test_it_carries_every_key_the_regression_checker_reads(self):
        with open(os.path.join(self.out, "metrics.json")) as f:
            data = json.load(f)
        self.assertIn("sequence", data)
        self.assertIn("FusionCore", data["filters"])
        self.assertIn("RL-EKF", data["filters"])
        self.assertIn(check_benchmark_regression.METRIC, data["filters"]["FusionCore"])
        self.assertIn(check_benchmark_regression.CONTROL_METRIC, data["filters"]["RL-EKF"])

    def test_the_checker_can_actually_load_what_was_written(self):
        # Going through the checker's own loader rather than re-reading the keys by
        # hand, so the two cannot drift apart without this failing.
        seq, fc, rl = check_benchmark_regression.load_metrics(
            os.path.join(self.out, "metrics.json"))
        self.assertEqual("synthetic", seq)
        self.assertIsInstance(fc, float)
        self.assertIsInstance(rl, float)

    def test_the_emitted_pair_obeys_the_xy_invariant(self):
        with open(os.path.join(self.out, "metrics.json")) as f:
            data = json.load(f)
        for name, m in data["filters"].items():
            self.assertLessEqual(m["ate_rmse_xy"], m["ate_rmse_3d"],
                                 f"{name}: xy {m['ate_rmse_xy']} above 3d {m['ate_rmse_3d']}")


if __name__ == "__main__":
    unittest.main()
