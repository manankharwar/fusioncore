#!/usr/bin/env python3
"""Tests for rl_to_fusioncore.py, the first code most robot_localization migrators run."""
import os
import re
import subprocess
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, os.path.dirname(__file__))
import check_config_params

TOOL = os.path.join(os.path.dirname(__file__), "rl_to_fusioncore.py")

MINIMAL = """\
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    imu0: /imu/data
    imu0_config: [false, false, false, true, true, true,
                  false, false, false, true, true, true,
                  true, true, true]
    imu0_remove_gravitational_acceleration: true
    odom0: /wheel/odom
    odom0_config: [false, false, false, false, false, false,
                   true, true, false, false, false, true,
                   false, false, false]
"""


def convert(text, *extra):
    """Run the tool on `text`. Returns (returncode, stdout, stderr)."""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(text)
    try:
        r = subprocess.run([sys.executable, TOOL, f.name, *extra],
                           capture_output=True, text=True)
    finally:
        os.unlink(f.name)
    return r.returncode, r.stdout, r.stderr


def emitted_keys(stdout):
    doc = yaml.safe_load(stdout)
    return set(doc["fusioncore"]["ros__parameters"])


class ConversionTest(unittest.TestCase):
    def test_every_emitted_key_is_declared_by_the_node(self):
        rc, out, _ = convert(MINIMAL)
        self.assertEqual(0, rc)
        declared = check_config_params.declared_parameters(check_config_params.NODE)
        self.assertEqual(set(), emitted_keys(out) - declared)

    def test_two_d_mode_maps_to_force_2d(self):
        _, out, _ = convert(MINIMAL)
        params = yaml.safe_load(out)["fusioncore"]["ros__parameters"]
        self.assertIs(True, params["publish.force_2d"])

    def test_gravity_flag_is_inverted(self):
        _, out, _ = convert(MINIMAL)
        params = yaml.safe_load(out)["fusioncore"]["ros__parameters"]
        self.assertIs(False, params["imu.remove_gravitational_acceleration"])

    def test_unmapped_feature_becomes_todo_and_summary_count_matches(self):
        text = MINIMAL + "    process_noise_covariance: [0.05]\n"
        rc, out, err = convert(text)
        self.assertEqual(0, rc)
        self.assertIn("process_noise_covariance was NOT translated", out)
        todos_in_stdout = len(re.findall(r"^\s*# TODO:", out, re.M))
        stderr_count = int(re.search(r"^(\d+) TODO\(s\)", err, re.M).group(1))
        self.assertEqual(todos_in_stdout, stderr_count)
        self.assertGreater(stderr_count, 0)

    def test_output_is_valid_yaml_with_header_disclaimer(self):
        _, out, _ = convert(MINIMAL)
        self.assertIn("NOT a finished config", out)
        yaml.safe_load(out)


class BadInputTest(unittest.TestCase):
    def assert_clear_failure(self, text):
        rc, out, err = convert(text)
        self.assertNotEqual(0, rc)
        self.assertEqual("", out)
        self.assertNotIn("Traceback", err)

    def test_empty_input_fails_clearly(self):
        self.assert_clear_failure("")

    def test_non_mapping_input_fails_clearly(self):
        self.assert_clear_failure("- just\n- a list\n")

    def test_missing_ros_parameters_fails_clearly(self):
        self.assert_clear_failure("ekf_filter_node:\n  frequency: 30\n")

    def test_malformed_yaml_fails_clearly(self):
        self.assert_clear_failure("ekf_filter_node: [\n")


if __name__ == "__main__":
    unittest.main()
