"""Every tool in SCRIPTS must answer --help with its docstring and reject unknown flags.

This imports each tool, so a tool that demands a display at import time takes the whole
file down. That is not hypothetical: fusioncore_gcs.py called matplotlib.use('TkAgg') at
module scope and turned CI red on both distros for five commits, while passing locally
because a developer machine has a display. MPLBACKEND is set before any tool is imported,
which every well-behaved matplotlib caller honours, and fusioncore_gcs also falls back on
its own now. Belt and braces, because asking a tool for --help must never need a screen.
"""
import os

os.environ.setdefault("MPLBACKEND", "Agg")
import contextlib
import importlib
import importlib.util
import io
import pathlib
import sys
import types
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

SCRIPTS = [
    "check_config_wiring",
    "check_docs_params",
    "check_node_member_wiring",
    "check_replay_wiring",
    "check_docs_test_count",
    "make_figures",
    "make_paper_figures",
    "fork_diff",
    "compare_runs",
    "score_turn_gate",
]


class HelpFlag(unittest.TestCase):
    def test_help_prints_the_docstring_and_exits_zero(self):
        for name in SCRIPTS:
            with self.subTest(script=name):
                module = importlib.import_module(name)
                out = io.StringIO()
                with contextlib.redirect_stdout(out), self.assertRaises(SystemExit) as raised:
                    module.main(["--help"])
                self.assertEqual(raised.exception.code, 0)
                first_line = module.__doc__.strip().splitlines()[0]
                self.assertIn(first_line, out.getvalue())

    def test_unknown_flag_is_an_error_not_a_silent_run(self):
        for name in SCRIPTS:
            with self.subTest(script=name):
                module = importlib.import_module(name)
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                    module.main(["--bogus"])
                self.assertEqual(raised.exception.code, 2)


def _fake(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    return mod


# fusioncore_gcs imports rclpy at module level. These fakes have no init, so a test
# fails if main ever reaches rclpy.init before the parser.
GCS_STUBS = {
    "rclpy": _fake("rclpy"),
    "rclpy.node": _fake("rclpy.node", Node=object),
    "nav_msgs": _fake("nav_msgs"),
    "nav_msgs.msg": _fake("nav_msgs.msg", Odometry=object),
    "sensor_msgs": _fake("sensor_msgs"),
    "sensor_msgs.msg": _fake("sensor_msgs.msg", NavSatFix=object),
}


@unittest.skipUnless(
    importlib.util.find_spec("matplotlib") and importlib.util.find_spec("numpy"),
    "needs matplotlib and numpy",
)
class HelpFlagStubbedRos(unittest.TestCase):
    """fusioncore_gcs cannot join SCRIPTS, so it is imported with fake ROS modules."""

    def test_gcs_help_exits_zero_and_unknown_flag_exits_two(self):
        with mock.patch.dict(sys.modules, GCS_STUBS):
            module = importlib.import_module("fusioncore_gcs")
            out = io.StringIO()
            with contextlib.redirect_stdout(out), self.assertRaises(SystemExit) as raised:
                module.main(["--help"])
            self.assertEqual(raised.exception.code, 0)
            self.assertIn(module.__doc__.strip().splitlines()[0], out.getvalue())
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                module.main(["--bogus"])
            self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
