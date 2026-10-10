#!/usr/bin/env python3
"""Tests for the README benchmark table vs verified baseline checker."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import check_readme_benchmark


class ReadmeBenchmarkTest(unittest.TestCase):
    def test_parses_readme_rows_with_status(self):
        readme = """\
| Sequence | Season | Duration | FC ATE RMSE | RL-EKF ATE RMSE | Winner | Status |
|---|---|---|---|---|---|---|
| 2012-06-15 | Summer | 55 min | **82.0 m** | **45.9 m** | **RL +44%** | re-measured, n=2 |
| 2012-08-20 | Summer | 83 min | **155.6 m** | **10.4 m** | **RL +93%** | re-measured, n=2 |
| 2012-01-08 | Winter | 92 min | 18.6 m | 41.2 m | FC +55% | ‡ never verified |
"""
        self.assertEqual(
            {
                "2012-06-15": (82.0, 45.9, "re-measured, n=2"),
                "2012-08-20": (155.6, 10.4, "re-measured, n=2"),
                "2012-01-08": (18.6, 41.2, "‡ never verified"),
            },
            check_readme_benchmark.readme_rows(readme),
        )

    def test_ignores_non_benchmark_tables(self):
        readme = """\
| Other | Table |
|---|---|
| 2012-06-15 | 49.2 m |
"""
        self.assertEqual({}, check_readme_benchmark.readme_rows(readme))

    def test_re_measured_row_matching_baseline_passes(self):
        rows = {"2012-06-15": (82.0, 45.9, "re-measured, n=2")}
        baseline = {
            "2012-06-15": {
                "fusioncore_ate_rmse_3d": 82.011,
                "rl_ate_rmse_3d": 45.864,
                "repeats_post_frame_fix": [78.112, 82.011],
                "repeats_post_frame_fix_control": [45.772, 45.864],
            }
        }
        self.assertEqual([], check_readme_benchmark.violations(rows, baseline))

    def test_re_measured_row_without_baseline_entry_fails(self):
        rows = {"2012-06-15": (82.0, 45.9, "re-measured, n=2")}
        self.assertEqual(1, len(check_readme_benchmark.violations(rows, {})))

    def test_re_measured_row_with_mismatched_fc_fails(self):
        rows = {"2012-06-15": (50.0, 45.9, "re-measured, n=2")}
        baseline = {
            "2012-06-15": {
                "fusioncore_ate_rmse_3d": 82.011,
                "rl_ate_rmse_3d": 45.864,
            }
        }
        self.assertEqual(1, len(check_readme_benchmark.violations(rows, baseline)))

    def test_re_measured_row_matches_mean_of_repeats(self):
        # 2012-09-28 RL: README 87.2 is the mean of the two control runs
        # (82.598 and 91.724), not the primary 82.598.
        rows = {"2012-09-28": (18.6, 87.2, "re-measured, n=2")}
        baseline = {
            "2012-09-28": {
                "fusioncore_ate_rmse_3d": 18.594,
                "rl_ate_rmse_3d": 82.598,
                "repeats_post_frame_fix": [18.594, 18.596],
                "repeats_post_frame_fix_control": [82.598, 91.724],
            }
        }
        self.assertEqual([], check_readme_benchmark.violations(rows, baseline))

    def test_non_re_measured_row_must_carry_never_verified_mark(self):
        rows = {"2012-01-08": (18.6, 41.2, "re-measured, n=2")}
        baseline = {}
        # A row marked re-measured with no baseline entry is a violation.
        self.assertEqual(1, len(check_readme_benchmark.violations(rows, baseline)))

    def test_current_tree_readme_is_clean(self):
        rows = check_readme_benchmark.readme_rows(
            check_readme_benchmark.README.read_text()
        )
        baseline = check_readme_benchmark.load_baseline()
        self.assertEqual([], check_readme_benchmark.violations(rows, baseline))


if __name__ == "__main__":
    unittest.main()
