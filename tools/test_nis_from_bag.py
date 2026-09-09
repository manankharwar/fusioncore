#!/usr/bin/env python3
"""Tests for nis_from_bag.py: --json must carry the same numbers the prose prints.

No ROS needed. The bag readers are replaced with synthetic messages, so this
only checks the aggregation and the two output paths, not rosbag2 itself.
Run with: python3 -m unittest tools/test_nis_from_bag.py
"""
import contextlib
import io
import json
import os
import sys
import types
import unittest
from types import SimpleNamespace as Msg
from unittest import mock

# nis_from_bag exits at import time without a sourced ROS 2 environment.
# Nothing below reads a bag, so stand in for the three imports it checks.
for _name, _attr in (("rosbag2_py", None), ("rclpy", None), ("rosidl_runtime_py", None),
                     ("rclpy.serialization", "deserialize_message"),
                     ("rosidl_runtime_py.utilities", "get_message")):
    if _name not in sys.modules:
        sys.modules[_name] = types.ModuleType(_name)
        if _attr:
            setattr(sys.modules[_name], _attr, None)

sys.path.insert(0, os.path.dirname(__file__))
import nis_from_bag  # noqa: E402


def gnss(mahalanobis_sq, reason, accepted):
    return Msg(mahalanobis_sq=mahalanobis_sq, rejection_reason=reason,
               accepted=accepted, chi2_threshold=16.27)


# NIS samples sorted: 1, 2, 4, 30 -> median 3.0, mean 9.25, p90 4.0, max 30.0.
# The -1 fix failed a quality gate and carries no NIS.
GNSS = [gnss(1.0, "ACCEPTED", True), gnss(2.0, "ACCEPTED", True),
        gnss(4.0, "ACCEPTED", True), gnss(30.0, "CHI2_FAILED", False),
        gnss(-1.0, "HDOP_HIGH", False)]
HEALTH = [Msg(position_sigma_x=0.5, heading_sigma_deg=10.0, heading_source="GPS_TRACK"),
          Msg(position_sigma_x=0.3, heading_sigma_deg=90.0, heading_source="NONE"),
          Msg(position_sigma_x=0.4, heading_sigma_deg=20.0, heading_source="GPS_TRACK")]
NAVSAT = ("/gps/fix", 2.5, 0.03, 7.21)


def run_main(*argv):
    out = io.StringIO()
    with mock.patch.object(sys, "argv", ["nis_from_bag.py", *argv]), \
            contextlib.redirect_stdout(out):
        nis_from_bag.main()
    return out.getvalue()


@mock.patch.object(nis_from_bag, "read_navsat", lambda bag: NAVSAT)
@mock.patch.object(nis_from_bag, "read", lambda bag: (GNSS, HEALTH, {}))
class NisFromBagTest(unittest.TestCase):
    def test_analyze_reports_the_numbers_the_prose_prints(self):
        s = nis_from_bag.analyze("/bags/run1/")
        self.assertEqual("run1", s["bag"])
        self.assertEqual((5, 3), (s["fixes"], s["accepted"]))
        self.assertEqual({"ACCEPTED": 3, "CHI2_FAILED": 1, "HDOP_HIGH": 1}, s["rejection_reasons"])
        self.assertEqual(16.27, s["chi2_threshold"])
        self.assertEqual({"samples": 4, "median": 3.0, "mean": 9.25, "p90": 4.0,
                          "max": 30.0, "expected": 3.0}, s["nis"])
        self.assertEqual({"position_sigma_median_m": 0.4, "heading_sigma_median_deg": 20.0,
                          "heading_sources": ["GPS_TRACK", "NONE"]}, s["health"])
        self.assertEqual({"topic": "/gps/fix", "declared_sigma_m": 2.5,
                          "observed_d2_median_m": 0.03, "expected_d2_median_m": 7.21},
                         s["navsat"])

    def test_json_is_one_object_per_bag_and_a_bad_bag_does_not_stop_the_rest(self):
        def read(bag):
            if bag == "broken":
                raise RuntimeError("no such file")
            return GNSS, HEALTH, {}   # third value: topics that failed to decode

        with mock.patch.object(nis_from_bag, "read", read):
            lines = run_main("--json", "broken", "/bags/run1").splitlines()
        self.assertEqual(2, len(lines))
        bad, good = (json.loads(line) for line in lines)
        self.assertEqual({"bag": "broken", "error": "could not be read (no such file)"}, bad)
        self.assertEqual(nis_from_bag.analyze("/bags/run1"), good)

    def test_prose_stays_the_default(self):
        out = run_main("/bags/run1")
        self.assertIn("=== run1 ===", out)
        self.assertIn("    ACCEPTED              3  (60%)", out)
        self.assertIn("NIS median 3.00   mean 9.25   p90 4.00   max 30.00   (honest is 3.0)", out)
        self.assertIn("Covariance is consistent with the errors being made.", out)
        self.assertIn("heading 1-sigma: median 20 deg, sources ['GPS_TRACK', 'NONE']", out)
        self.assertIn("receiver on /gps/fix declares 2.50 m 1-sigma", out)

    def test_no_nis_samples_when_every_fix_failed_a_quality_gate(self):
        gated = [m for m in GNSS if m.mahalanobis_sq < 0.0]
        with mock.patch.object(nis_from_bag, "read", lambda bag: (gated, [], {})), \
                mock.patch.object(nis_from_bag, "read_navsat", lambda bag: None):
            s = nis_from_bag.analyze("run")
            out = run_main("run")
        self.assertEqual({"nis": None, "health": None, "navsat": None},
                         {k: s[k] for k in ("nis", "health", "navsat")})
        self.assertIn("no NIS samples", out)


if __name__ == "__main__":
    unittest.main()


class UnreadableTopicTest(unittest.TestCase):
    """A topic that no longer deserialises must not cost you the whole bag.

    CDR is not self-describing, so adding a field to a message makes every older
    recording of it unreadable. FilterHealth has gained fields three times, most
    recently in 96d0207. The NIS numbers live on GnssStatus and are unaffected
    by that, and losing them to an unrelated topic means losing the analysis of
    a field run you cannot go back and repeat.
    """

    def test_gnss_numbers_survive_an_undecodable_health_topic(self):
        broken = {"/fusion/debug/filter_health": "Fast CDR exception"}

        def read(bag):
            return GNSS, [], broken

        with mock.patch.object(nis_from_bag, "read", read), \
                mock.patch.object(nis_from_bag, "read_navsat", lambda bag: None):
            summary = nis_from_bag.analyze("/bags/run1")
            out = run_main("/bags/run1")

        self.assertNotIn("error", summary)
        self.assertEqual(len(GNSS), summary["fixes"])
        self.assertIsNotNone(summary["nis"])
        self.assertEqual(broken, summary["unreadable_topics"])
        self.assertIn("could not be decoded and was skipped", out)
        self.assertIn("Everything below is unaffected", out)
        self.assertIn("NIS median", out)
