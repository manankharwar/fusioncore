"""Tests for check_docs_params.py."""

import importlib.util
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location(
    "check_docs_params", ROOT / "tools" / "check_docs_params.py")
check_docs_params = importlib.util.module_from_spec(spec)
sys.modules["check_docs_params"] = check_docs_params
spec.loader.exec_module(check_docs_params)


class DocsParamsTest(unittest.TestCase):

    def test_matches_a_backticked_parameter(self):
        self.assertEqual(
            ["gnss.fix_topic"],
            check_docs_params.TOKEN.findall("set `gnss.fix_topic` to your topic"))

    def test_ignores_prose_without_backticks(self):
        """Otherwise "the gnss.fix topic" in a sentence trips the checker."""
        self.assertEqual([], check_docs_params.TOKEN.findall(
            "the gnss.fix_topic parameter controls this"))

    def test_ignores_an_unrelated_group(self):
        self.assertEqual([], check_docs_params.TOKEN.findall("`nav2.controller`"))

    def test_matches_a_dotted_multi_level_name(self):
        self.assertEqual(
            ["imu.lever_arm_x"],
            check_docs_params.TOKEN.findall("`imu.lever_arm_x`"))

    def test_the_declare_regex_finds_real_parameters(self):
        declared = set(check_docs_params.DECLARE.findall(
            check_docs_params.NODE.read_text()))
        self.assertIn("gnss.fix_topic", declared)
        self.assertIn("imu.lever_arm_x", declared)
        self.assertGreater(len(declared), 100, "the declare regex is matching too little")

    def test_changelog_is_skipped(self):
        """A changelog names parameters that were REMOVED. That is correct history."""
        self.assertIn("CHANGELOG.md", check_docs_params.SKIP_EXACT)

    def test_current_docs_name_only_real_parameters(self):
        """The real tree. Any doc telling a user to set a parameter that the node
        never declares fails here, because ROS 2 ignores it silently and nothing
        else in the project would notice."""
        declared = set(check_docs_params.DECLARE.findall(
            check_docs_params.NODE.read_text()))
        bad = {}
        for d in check_docs_params.tracked_docs():
            for name in set(check_docs_params.TOKEN.findall((ROOT / d).read_text())):
                if name not in declared:
                    bad.setdefault(name, []).append(d)
        self.assertEqual({}, bad, "docs name parameters that do not exist")


if __name__ == "__main__":
    unittest.main()
