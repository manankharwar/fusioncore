#!/usr/bin/env python3
"""Unit tests for the value rules in check_config_params.py."""
import os
import sys
import unittest

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


if __name__ == "__main__":
    unittest.main()
