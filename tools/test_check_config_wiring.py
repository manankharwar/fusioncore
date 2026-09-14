#!/usr/bin/env python3
"""Tests for the ROS-parameter-to-FusionCore wiring checker."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import check_config_wiring


class ConfigWiringTest(unittest.TestCase):
    def test_extracts_top_level_and_nested_config_mappings(self):
        node = '''
        config.outlier_rejection = get_parameter("outlier_rejection").as_bool();
        config.encoder.vel_noise_y = get_parameter("encoder.nhc_vy_sigma").as_double();
        '''
        self.assertEqual(
            [
                ("outlier_rejection", "outlier_rejection"),
                ("encoder.vel_noise_y", "encoder.nhc_vy_sigma"),
            ],
            check_config_wiring.config_mappings(node),
        )

    def test_used_field_is_not_reported_even_when_parameter_name_differs(self):
        mappings = [("encoder.vel_noise_y", "encoder.nhc_vy_sigma")]
        core = "double vel_noise_y = 0.05;\nreturn p.vel_noise_y * p.vel_noise_y;\n"
        self.assertEqual([], check_config_wiring.dead_mappings(mappings, core))

    def test_comment_naming_dead_field_does_not_count_as_a_use(self):
        mappings = [("encoder.dead_sigma", "encoder.some_parameter")]
        core = "// dead_sigma tunes nothing\ndouble dead_sigma = 0.05;\n"
        self.assertEqual(
            [("encoder.dead_sigma", "encoder.some_parameter")],
            check_config_wiring.dead_mappings(mappings, core),
        )

    def test_declaration_only_field_is_reported(self):
        mappings = [("encoder.dead_sigma", "encoder.some_parameter")]
        core = "double dead_sigma = 0.05;\n"
        self.assertEqual(
            [("encoder.dead_sigma", "encoder.some_parameter")],
            check_config_wiring.dead_mappings(mappings, core),
        )

    def test_extracts_declared_parameters(self):
        node = '''
        declare_parameter("base_frame", "base_link");
        declare_parameter("publish.tf", true);
        '''
        self.assertEqual(
            ["base_frame", "publish.tf"],
            check_config_wiring.declared_parameters(node),
        )

    def test_current_tree_has_no_dead_mappings(self):
        mappings, core = check_config_wiring.load_tree()
        self.assertTrue(mappings)
        self.assertEqual([], check_config_wiring.dead_mappings(mappings, core))


if __name__ == "__main__":
    unittest.main()
