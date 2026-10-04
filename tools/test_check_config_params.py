#!/usr/bin/env python3
"""Unit tests for the value rules in check_config_params.py."""
import contextlib
import io
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(__file__))
import check_config_params


def findings_for(*parameters):
    return check_config_params.value_findings(parameters)


class ValueRulesTest(unittest.TestCase):
    def test_outlier_threshold_below_floor_is_an_error(self):
        findings = findings_for(
            ("outlier_threshold_gnss", 10, "7.0"),
        )

        self.assertEqual("ERROR", findings[0][0])
        self.assertIn("chi2(3, 0.95)", findings[0][4])

    def test_documented_tightest_gnss_gate_is_allowed(self):
        self.assertEqual([], findings_for(("outlier_threshold_gnss", 10, "7.81")))

    def test_continuity_floor_is_advisory(self):
        findings = findings_for(("gnss.continuity_max_m", 10, "2.0"))

        self.assertEqual("WARNING", findings[0][0])
        self.assertIn("2,361 fixes", findings[0][4])

    def test_field_strength_requires_magnetometer(self):
        findings = findings_for(
            ("magnetometer.enabled", 2, "false"),
            ("magnetometer.field_strength", 3, "48.0"),
        )

        self.assertEqual("WARNING", findings[0][0])
        self.assertIn("never runs", findings[0][4])

    def test_sigma_scaled_speed_gate_is_not_warned(self):
        self.assertEqual([], findings_for(
            ("gnss.max_speed", 2, "2.0"),
            ("gnss.max_speed_sigma_k", 3, "5.0"),
        ))

    def test_absolute_speed_gate_warns(self):
        findings = findings_for(
            ("gnss.max_speed", 2, "2.0"),
            ("gnss.max_speed_sigma_k", 3, "0.0"),
        )

        self.assertEqual("WARNING", findings[0][0])
        self.assertIn("157 of 500", findings[0][4])

    def test_outlier_sigma_must_be_smaller_than_sigma_quality_limit(self):
        findings = findings_for(
            ("gnss.max_sigma_xy", 2, "3.24"),
            ("gnss.outlier_sigma_xy", 3, "5.0"),
        )

        self.assertEqual("WARNING", findings[0][0])
        self.assertIn("26 m to 30 m", findings[0][4])

    def test_disabled_outlier_rejection_skips_threshold_rules(self):
        self.assertEqual([], findings_for(
            ("outlier_rejection", 2, "false"),
            ("outlier_threshold_gnss", 3, "7.0"),
        ))


def run_main(*argv):
    out = io.StringIO()
    with mock.patch.object(sys, "argv", ["check_config_params.py", *argv]), \
            contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        code = check_config_params.main()
    return code, out.getvalue()


class CommandLinePathsTest(unittest.TestCase):
    def write_config(self, body):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(body)
        self.addCleanup(os.remove, f.name)
        return f.name

    def test_named_file_is_checked_instead_of_shipped_configs(self):
        path = self.write_config("fusioncore:\n  ros__parameters:\n    gnss.max_hodp: 2.0\n")

        code, out = run_main(path)

        self.assertEqual(1, code)
        self.assertIn(path, out)
        self.assertIn("gnss.max_hodp", out)
        self.assertIn("1 config files checked", out)

    def test_missing_named_file_fails(self):
        code, out = run_main("/nonexistent/robot.yaml")

        self.assertNotEqual(0, code)
        self.assertIn("/nonexistent/robot.yaml", out)

    def test_no_paths_checks_shipped_configs(self):
        code, out = run_main("--quiet")

        self.assertEqual(0, code)
        self.assertRegex(out, r"\n[1-9]\d* config files checked")


if __name__ == "__main__":
    unittest.main()
