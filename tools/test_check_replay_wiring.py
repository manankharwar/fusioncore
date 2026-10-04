#!/usr/bin/env python3
"""Tests for the ROS-parameter-to-offline-replay wiring checker."""
import contextlib
import io
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(__file__))
import check_replay_wiring

NODE = '''
declare_parameter("gnss.outlier_sigma_xy", 3.0);
declare_parameter("ukf.q_position", 0.01);
declare_parameter("imu.topic", "/imu/data");
'''


class ReplayWiringTest(unittest.TestCase):
    def run_checker(self, node, replay):
        with tempfile.TemporaryDirectory() as d:
            node_path = os.path.join(d, "fusion_node.cpp")
            replay_path = os.path.join(d, "replay.cpp")
            with open(node_path, "w") as f:
                f.write(node)
            if replay is not None:
                with open(replay_path, "w") as f:
                    f.write(replay)
            out = io.StringIO()
            with mock.patch.object(check_replay_wiring, "NODE", node_path), \
                    mock.patch.object(check_replay_wiring, "REPLAY", replay_path), \
                    contextlib.redirect_stdout(out):
                code = check_replay_wiring.main()
        return code, out.getvalue()

    def test_unwired_core_parameter_is_reported(self):
        replay = 'get("gnss.outlier_sigma_xy", 3.0);\n'
        code, out = self.run_checker(NODE, replay)
        self.assertEqual(1, code)
        self.assertIn("NOT wired into hardware/replay.cpp", out)
        self.assertIn("  ukf.q_position", out)
        self.assertNotIn("  gnss.outlier_sigma_xy", out)
        self.assertNotIn("  imu.topic", out)

    def test_parameter_both_wired_and_node_level_is_reported(self):
        replay = '''
        get("gnss.outlier_sigma_xy", 3.0);
        get("ukf.q_position", 0.01);
        static const std::set<std::string> kNodeLevel = {"ukf.q_position"};
        '''
        code, out = self.run_checker(NODE, replay)
        self.assertEqual(1, code)
        self.assertIn("BOTH wired and listed node-level", out)
        self.assertIn("  ukf.q_position", out)

    def test_clean_tree_passes(self):
        replay = '''
        get("gnss.outlier_sigma_xy", 3.0);
        static const std::set<std::string> kNodeLevel = {"ukf.q_position"};
        '''
        code, out = self.run_checker(NODE, replay)
        self.assertEqual(0, code)
        self.assertIn("OK:", out)

    def test_missing_replay_skips(self):
        code, out = self.run_checker(NODE, None)
        self.assertEqual(0, code)
        self.assertIn("skipping", out)


if __name__ == "__main__":
    unittest.main()
