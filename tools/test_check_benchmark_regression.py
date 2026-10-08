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


def metrics(sequence, xy, ate3d=None, rl3d=10.5, rl=True):
    """Shaped like a real tools/evaluate.py --json output.

    rl3d is the CONTROL. It defaults to the same 10.5 the baseline fixture records,
    so an unmodified call is a steady-control run and grades exactly as before.
    rl=False drops the RL-EKF block entirely, for the no-control-to-check path.
    """
    filters = {"FusionCore": {"ate_rmse_xy": xy, "ate_rmse_3d": ate3d if ate3d else xy}}
    if rl:
        filters["RL-EKF"] = {"ate_rmse_xy": rl3d * 0.999, "ate_rmse_3d": rl3d}
    return {"sequence": sequence, "filters": filters}


def baseline(d, seq="2012-08-20", xy=121.669, threshold=10.0, verified=True, rl3d=10.5):
    p = pathlib.Path(d) / "baseline.json"
    p.write_text(json.dumps({
        "baseline_commit": "abc1234",
        "recorded": "2026-09-14",
        "regression_threshold_pct": threshold,
        "sequences": {seq: {"fusioncore_ate_rmse_xy": xy, "verified": verified,
                            **({"rl_ate_rmse_3d": rl3d} if rl3d is not None else {})}},
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


class ControlVoidTest(unittest.TestCase):
    """The control decides whether a row can be read at all.

    robot_localization rides the same playback in the same launch, so its score is a
    property of the harness and the machine, never of a FusionCore change. These lock
    the behaviour that was missing on 2026-10-05, when this gate was handed a starved
    2013-04-05 run and reported a 74.4% improvement.
    """

    def test_the_20261005_void_run_is_not_called_an_improvement(self):
        # The real numbers. FusionCore 51.454 against a 200.910 baseline is -74.4%,
        # which the gate used to print as "improved" and exit 0 on. The control had
        # moved 266.700 -> 254.978, which is -4.40%, so the run measured the machine.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2013-04-05", 51.454, rl3d=254.978))
            code, out = run([m, "--baseline",
                             baseline(d, seq="2013-04-05", xy=200.910, rl3d=266.700)])
        self.assertEqual(code, 1, "a void run must not exit 0")
        self.assertIn("VOID", out)
        self.assertNotIn("improved", out)
        self.assertIn("-4.40%", out)

    def test_a_steady_control_still_grades_normally(self):
        # 2012-06-15 on 2026-10-05: control 18.487 -> 18.489 is +0.01%, well inside
        # tolerance, so the FusionCore number is readable and must be graded.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-06-15", 71.998, rl3d=18.489))
            code, out = run([m, "--baseline",
                             baseline(d, seq="2012-06-15", xy=69.425, rl3d=18.487)])
        self.assertEqual(code, 0)
        self.assertNotIn("VOID", out)
        self.assertIn("ok", out)

    def test_a_regression_with_a_steady_control_still_fails(self):
        # 2012-08-20 on 2026-10-05: control -0.11%, so the +18.3% IS real and must
        # still be reported. Voiding must not become a way to swallow regressions.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 143.976, rl3d=10.507))
            code, out = run([m, "--baseline",
                             baseline(d, seq="2012-08-20", xy=121.669, rl3d=10.519)])
        self.assertEqual(code, 1)
        self.assertIn("REGRESSION", out)
        self.assertNotIn("VOID", out)

    def test_a_void_run_fails_even_when_fusioncore_regressed_too(self):
        # Both things wrong at once: do not let a void row be quietly dropped.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 999.0, rl3d=20.0))
            code, out = run([m, "--baseline", baseline(d)])
        self.assertEqual(code, 1)
        self.assertIn("VOID", out)

    def test_tolerance_boundary_just_inside_is_graded(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.7, rl3d=10.5 * 1.009))
            code, out = run([m, "--baseline", baseline(d)])
        self.assertEqual(code, 0)
        self.assertNotIn("VOID", out)

    def test_tolerance_boundary_just_outside_is_void(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.7, rl3d=10.5 * 1.011))
            code, out = run([m, "--baseline", baseline(d)])
        self.assertEqual(code, 1)
        self.assertIn("VOID", out)

    def test_control_moving_the_other_way_is_also_void(self):
        # A control that got BETTER is just as much a changed machine.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.7, rl3d=10.5 * 0.95))
            code, out = run([m, "--baseline", baseline(d)])
        self.assertEqual(code, 1)
        self.assertIn("VOID", out)

    def test_tolerance_is_overridable(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.7, rl3d=10.5 * 1.03))
            code, out = run([m, "--baseline", baseline(d), "--control-tolerance", "5"])
        self.assertEqual(code, 0)
        self.assertNotIn("VOID", out)

    def test_a_baseline_with_no_control_still_grades_but_says_so(self):
        # Back-compat: older baseline entries have no rl_ate_rmse_3d. Grade them,
        # but never silently, or a starved run looks identical to a clean one.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.7))
            code, out = run([m, "--baseline", baseline(d, rl3d=None)])
        self.assertEqual(code, 0)
        self.assertIn("UNCHECKED", out)
        self.assertIn("starved", out)

    def test_metrics_with_no_rl_block_is_also_flagged(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.7, rl=False))
            code, out = run([m, "--baseline", baseline(d)])
        self.assertEqual(code, 0)
        self.assertIn("UNCHECKED", out)


class BaselineIntegrityTest(unittest.TestCase):
    """A reference has to be self-consistent before it can judge anything.

    Found 2026-10-06 while investigating the 2012-08-20 regression: two of the three
    entries in this repo's own baseline had 3D ATE BELOW XY ATE, which is impossible,
    since 3D adds a non-negative dz^2 to the same sum. The gate gates on XY, so a
    transposed XY quietly shifts every percentage measured against it.
    """

    def _baseline(self, d, a3d, axy):
        p = pathlib.Path(d) / "baseline.json"
        p.write_text(json.dumps({
            "baseline_commit": "abc1234", "recorded": "2026-09-14",
            "regression_threshold_pct": 10.0,
            "sequences": {"2012-08-20": {
                "fusioncore_ate_rmse_3d": a3d, "fusioncore_ate_rmse_xy": axy,
                "rl_ate_rmse_3d": 10.5, "verified": True}},
        }))
        return str(p)

    def test_3d_below_xy_is_refused(self):
        # The real 2012-08-20 entry.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.7))
            code, out = run([m, "--baseline", self._baseline(d, 116.444, 121.669)])
        self.assertEqual(code, 2, "an impossible baseline must not grade anything")
        self.assertIn("impossible", out)
        self.assertIn("2012-08-20", out)

    def test_a_consistent_baseline_is_accepted(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.7))
            code, out = run([m, "--baseline", self._baseline(d, 125.0, 121.669)])
        self.assertEqual(code, 0)
        self.assertNotIn("impossible", out)

    def test_equal_3d_and_xy_is_allowed(self):
        # Legitimate: a perfectly planar run has dz = 0 throughout.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.7))
            code, out = run([m, "--baseline", self._baseline(d, 121.669, 121.669)])
        self.assertEqual(code, 0)

    def test_it_refuses_before_grading_not_after(self):
        # Even a clean run must not be graded against a broken reference.
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 100.0))
            code, out = run([m, "--baseline", self._baseline(d, 116.444, 121.669)])
        self.assertEqual(code, 2)
        self.assertNotIn("improved", out)


class CouldNotRunTest(unittest.TestCase):
    """"Could not run" must never read as "passed".

    The previous version printed PASS and exited 0 when every file handed to it
    failed to parse, because nothing was added to the regressions list. The absence
    of a failure is not evidence of success, which is the same shape as "a tight
    prior is not an observation" elsewhere in this project.
    """

    def _baseline(self, d):
        p = pathlib.Path(d) / "baseline.json"
        p.write_text(json.dumps({
            "baseline_commit": "abc", "recorded": "2026-01-01",
            "regression_threshold_pct": 10.0,
            "sequences": {"2012-08-20": {"fusioncore_ate_rmse_3d": 130.0,
                                         "fusioncore_ate_rmse_xy": 121.669,
                                         "rl_ate_rmse_3d": 10.5, "verified": True}},
        }))
        return str(p)

    def test_an_unreadable_metrics_file_is_not_a_pass(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, {"sequence": "2012-08-20", "filters": {}})
            code, out = run([m, "--baseline", self._baseline(d)])
        self.assertEqual(code, 2, "0 sequences graded must not exit 0")
        self.assertIn("COULD NOT RUN", out)
        self.assertNotIn("PASS:", out)

    def test_a_sequence_with_no_baseline_entry_is_not_a_pass_on_its_own(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2099-01-01", 50.0))
            code, out = run([m, "--baseline", self._baseline(d)])
        self.assertEqual(code, 2)
        self.assertIn("COULD NOT RUN", out)

    def test_a_real_comparison_still_passes_and_says_how_many(self):
        with tempfile.TemporaryDirectory() as d:
            m = write_json(d, metrics("2012-08-20", 121.0))
            code, out = run([m, "--baseline", self._baseline(d)])
        self.assertEqual(code, 0)
        self.assertIn("1 sequence(s) graded", out)

    def test_one_good_and_one_bad_passes_but_warns(self):
        with tempfile.TemporaryDirectory() as d:
            good = pathlib.Path(d) / "good.json"
            good.write_text(json.dumps(metrics("2012-08-20", 121.0)))
            bad = pathlib.Path(d) / "bad.json"
            bad.write_text(json.dumps({"sequence": "2012-08-20", "filters": {}}))
            code, out = run([str(good), str(bad), "--baseline", self._baseline(d)])
        self.assertEqual(code, 0, "something was graded, so it is a pass")
        self.assertIn("WARNING", out)
        self.assertIn("could not be read", out)


if __name__ == "__main__":
    unittest.main()
