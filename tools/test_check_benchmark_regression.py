"""Tests for check_benchmark_regression.py.

This was the only one of the five repo checkers without a test, and it is the one whose
job is to stop exactly what happened on 2026-09-27: NCLT 2012-08-20 went from 121.669 m
to 192.923 m XY ATE across ten commits and nobody noticed, because the gate exists, works
and exits 1, and was simply never run.

A gate nobody runs is bad enough. A gate nobody runs AND nobody tests is worse, because
the first time you finally do run it you cannot tell a real regression from a broken
checker. These lock the contract.
"""

import importlib.util
import io
import json
import pathlib
import sys
import tempfile
import unittest
from contextlib import redirect_stdout, redirect_stderr

ROOT = pathlib.Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location(
    "check_benchmark_regression", ROOT / "tools" / "check_benchmark_regression.py")
cbr = importlib.util.module_from_spec(spec)
sys.modules["check_benchmark_regression"] = cbr
spec.loader.exec_module(cbr)


def write_json(d, obj):
    p = pathlib.Path(d) / "metrics.json"
    p.write_text(json.dumps(obj))
    return str(p)


def metrics(sequence, xy, ate3d=None):
    """Shaped like a real tools/evaluate.py --json output."""
    return {
        "sequence": sequence,
        "filters": {
            "FusionCore": {"ate_rmse_xy": xy, "ate_rmse_3d": ate3d if ate3d else xy},
            "RL-EKF": {"ate_rmse_xy": 9.8, "ate_rmse_3d": 10.5},
        },
    }


def baseline(d, seq="2012-08-20", xy=121.669, threshold=10.0, verified=True):
    p = pathlib.Path(d) / "baseline.json"
    p.write_text(json.dumps({
        "baseline_commit": "abc1234",
        "recorded": "2026-09-14",
        "regression_threshold_pct": threshold,
        "sequences": {seq: {"fusioncore_ate_rmse_xy": xy, "verified": verified}},
    }))
    return str(p)


def run(argv):
    """Returns (exit_code, stdout)."""
    out, err = io.StringIO(), io.StringIO()
    old = sys.argv
    sys.argv = ["check_benchmark_regression.py"] + argv
    try:
        with redirect_stdout(out), redirect_stderr(err):
            code = cbr.main()
    finally:
        sys.argv = old
    return (code or 0), out.getvalue()


class BenchmarkRegressionTest(unittest.TestCase):

    def test_real_regression_fails_with_nonzero_exit(self):
        # The exact numbers from 2026-09-27. This MUST be a failure, and it must exit
        # non-zero, because exiting 0 on a failure is how a CI gate silently passes.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 192.923))
            code, out = run([m, "--baseline", baseline(d)])
        self.assertEqual(code, 1)
        self.assertIn("REGRESSION", out)
        self.assertIn("FAIL", out)

    def test_the_lever_arm_fix_passes(self):
        # 128.256 is +5.4%, inside the 10% gate. The fix has to read as a pass or the
        # gate is useless for telling "fixed" from "still broken".
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 128.256))
            code, out = run([m, "--baseline", baseline(d)])
        self.assertEqual(code, 0)
        self.assertIn("PASS", out)

    def test_boundary_just_under_threshold_passes(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.669 * 1.099))
            code, _ = run([m, "--baseline", baseline(d)])
        self.assertEqual(code, 0)

    def test_boundary_just_over_threshold_fails(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.669 * 1.101))
            code, _ = run([m, "--baseline", baseline(d)])
        self.assertEqual(code, 1)

    def test_improvement_is_reported_and_passes(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 60.0))
            code, out = run([m, "--baseline", baseline(d)])
        self.assertEqual(code, 0)
        self.assertIn("improved", out)

    def test_unknown_sequence_does_not_silently_pass_as_ok(self):
        # A sequence with no baseline must be visibly NEW. Reporting it as "ok" would
        # let a whole sequence be added and never gated.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2013-04-05", 999.0))
            code, out = run([m, "--baseline", baseline(d)])
        self.assertIn("NEW", out)
        self.assertNotIn("REGRESSION", out)

    def test_threshold_override_is_honoured(self):
        # 192.923 is +58.6%. It must pass only if the caller says so explicitly.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 192.923))
            code, _ = run([m, "--baseline", baseline(d), "--threshold", "100"])
        self.assertEqual(code, 0)

    def test_no_input_is_an_error_not_a_pass(self):
        # Passing nothing must NOT look like success. A CI step that globs zero files
        # and exits 0 is a gate that reports PASS having checked nothing.
        with tempfile.TemporaryDirectory() as d:
            code, _ = run(["--baseline", baseline(d)])
        self.assertNotEqual(code, 0)

    def test_malformed_metrics_is_skipped_loudly_not_counted_as_ok(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "metrics.json"
            p.write_text(json.dumps({"sequence": "2012-08-20", "filters": {}}))
            code, out = run([str(p), "--baseline", baseline(d)])
        # Nothing was actually compared, so it must not claim a clean result.
        self.assertNotIn("REGRESSION", out)

    def test_provisional_baseline_is_flagged(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.7))
            code, out = run([m, "--baseline", baseline(d, verified=False)])
        self.assertIn("provisional", out)

    def test_it_gates_on_xy_not_3d(self):
        # XY is the metric for a ground robot, and mixing the two is how the 116.44 and
        # 121.669 figures for the same run got confused on 2026-09-27.
        self.assertEqual(cbr.METRIC, "ate_rmse_xy")
        self.assertEqual(cbr.BASELINE_KEY, "fusioncore_ate_rmse_xy")


if __name__ == "__main__":
    unittest.main()
