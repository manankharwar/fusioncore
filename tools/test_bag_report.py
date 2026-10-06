"""Tests for bag_report.py.

The analysis is a pure function of plain data, so all of this runs with no bag, no
ROS and no dataset. That matters: the tool's whole purpose is to be the thing a
stranger can get value from without installing anything, and a test suite that
needed a 1 GB bag to run would never be run.

Two of these pin bugs found by running the tool on a real recording rather than by
reading it, and both were in the checks rather than the plumbing:

  * clock skew was reported as "stamps lag arrival by 451469497014 ms" on a bag
    replayed under simulated time, where the quantity is meaningless.
  * the stationary gyro bias averaged the whole 3309 s run, because the still
    interval was taken as min-to-max of every still sample rather than as the
    longest contiguous one.
"""

import importlib.util
import json
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("bag_report", ROOT / "tools" / "bag_report.py")
br = importlib.util.module_from_spec(spec)
sys.modules["bag_report"] = br
spec.loader.exec_module(br)


def sev(findings, severity):
    return [f for f in findings if f["severity"] == severity]


def titles(findings):
    return " | ".join(f["title"] for f in findings)


def stream(n, hz, t0=0.0, **fields):
    out = []
    for i in range(n):
        m = {"t": t0 + i / hz, "recv": t0 + i / hz}
        for k, v in fields.items():
            m[k] = v(i) if callable(v) else v
        out.append(m)
    return out


class YawRateSignTest(unittest.TestCase):
    """The issue #169 class of bug, which is the tool's headline finding."""

    def _series(self, wheel_sign, ratio=1.0):
        turn = 0.5
        imu = stream(2000, 100.0, wz=turn)
        odom = stream(2000, 100.0, wz=wheel_sign * turn * ratio)
        return {"imu": imu, "odom": odom}

    def test_opposite_signs_are_a_blocker(self):
        f = br.check_yaw_rate_signs(self._series(-1))
        b = sev(f, "BLOCKER")
        self.assertEqual(len(b), 1, titles(f))
        self.assertIn("disagree in sign", b[0]["title"])
        self.assertIn("REP-103", b[0]["fix"])

    def test_agreeing_signs_are_ok_and_not_a_blocker(self):
        f = br.check_yaw_rate_signs(self._series(+1))
        self.assertEqual(sev(f, "BLOCKER"), [])
        self.assertTrue(sev(f, "OK"), titles(f))

    def test_a_scale_error_is_reported_separately_from_the_sign(self):
        # Correct sign, 1.285x magnitude: the real NCLT number. Fixing the sign does
        # not fix the scale, so they must not be one finding.
        f = br.check_yaw_rate_signs(self._series(+1, ratio=1.285))
        self.assertEqual(sev(f, "BLOCKER"), [])
        w = sev(f, "WARNING")
        self.assertTrue(any("1.28" in x["title"] or "1.29" in x["title"] for x in w),
                        titles(f))
        self.assertIn("track width", w[0]["fix"])

    def test_both_wrong_at_once_reports_both(self):
        f = br.check_yaw_rate_signs(self._series(-1, ratio=1.285))
        self.assertEqual(len(sev(f, "BLOCKER")), 1)
        self.assertTrue(sev(f, "WARNING"), titles(f))

    def test_not_enough_turning_says_so_rather_than_guessing(self):
        f = br.check_yaw_rate_signs({"imu": stream(2000, 100.0, wz=0.0),
                                     "odom": stream(2000, 100.0, wz=0.0)})
        self.assertEqual(sev(f, "BLOCKER"), [])
        self.assertIn("Not enough turning", titles(f))


class OdometryTopicChoiceTest(unittest.TestCase):
    """Merging a wheel sensor with a filter output describes no physical sensor."""

    def test_a_single_topic_needs_no_note(self):
        chosen, note = br.pick_wheel_odometry({"/odom": [1, 2, 3]})
        self.assertEqual(chosen, [1, 2, 3])
        self.assertIsNone(note)

    def test_the_real_nclt_case_picks_the_wheel_topic(self):
        by = {"/fusion/odom": ["f"], "/odom/wheels": ["w"], "/rl/odometry": ["r"]}
        chosen, note = br.pick_wheel_odometry(by)
        self.assertEqual(chosen, ["w"])
        self.assertIsNotNone(note)
        self.assertIn("/odom/wheels", note["title"])

    def test_filter_outputs_are_ranked_below_a_plain_odom(self):
        by = {"/odometry/filtered": ["f"], "/odom": ["w"]}
        chosen, _ = br.pick_wheel_odometry(by)
        self.assertEqual(chosen, ["w"])

    def test_the_choice_is_always_reported_when_ambiguous(self):
        by = {"/a": ["a"], "/b": ["b"]}
        _, note = br.pick_wheel_odometry(by)
        self.assertEqual(note["severity"], "WARNING")
        self.assertIn("do not apply", note["fix"])

    def test_no_odometry_at_all_is_not_an_error(self):
        chosen, note = br.pick_wheel_odometry({})
        self.assertEqual(chosen, [])
        self.assertIsNone(note)


class ClockSkewTest(unittest.TestCase):

    def test_a_real_lag_is_a_warning(self):
        imu = stream(100, 100.0)
        for m in imu:
            m["recv"] = m["t"] + 0.4
        f = br.check_clock_skew({"imu": imu})
        self.assertTrue(any("lag" in x["title"] for x in sev(f, "WARNING")), titles(f))

    def test_two_sensors_on_different_clocks_is_a_warning(self):
        imu = stream(100, 100.0)
        gnss = stream(100, 5.0)
        for m in gnss:
            m["recv"] = m["t"] + 0.15
        f = br.check_clock_skew({"imu": imu, "gnss": gnss})
        self.assertTrue(any("clocks differ" in x["title"] for x in f), titles(f))

    def test_a_simulated_time_bag_is_not_reported_as_skew(self):
        # The bug: a bag replayed under /clock has stamps on the capture epoch, so
        # recv - t is years. Calling that "clock skew" is worse than saying nothing.
        imu = stream(100, 100.0)
        for m in imu:
            m["recv"] = m["t"] + 451469497.0
        f = br.check_clock_skew({"imu": imu})
        self.assertEqual(sev(f, "WARNING"), [], titles(f))
        self.assertIn("different epoch", titles(f))

    def test_consistent_clocks_say_so(self):
        f = br.check_clock_skew({"imu": stream(100, 100.0), "odom": stream(100, 20.0)})
        self.assertTrue(sev(f, "OK"), titles(f))


class StationaryBiasTest(unittest.TestCase):

    def test_the_interval_is_the_longest_contiguous_one_not_min_to_max(self):
        # The bug this pins: still at the start and still at the end, moving for the
        # whole 100 s in between. min-to-max would average the gyro over everything
        # and call the result a bias.
        odom = []
        imu = []
        for i in range(1000):                 # 0 to 10 s, still
            odom.append({"t": i * 0.01, "recv": i * 0.01, "v": 0.0, "wz": 0.0})
            imu.append({"t": i * 0.01, "recv": i * 0.01, "wz": 0.001})
        for i in range(1000, 11000):          # 10 to 110 s, moving and rotating
            odom.append({"t": i * 0.01, "recv": i * 0.01, "v": 1.0, "wz": 0.5})
            imu.append({"t": i * 0.01, "recv": i * 0.01, "wz": 0.5})
        for i in range(11000, 11500):         # 110 to 115 s, still again
            odom.append({"t": i * 0.01, "recv": i * 0.01, "v": 0.0, "wz": 0.0})
            imu.append({"t": i * 0.01, "recv": i * 0.01, "wz": 0.001})
        f = br.check_stationary_bias({"imu": imu, "odom": odom})
        bias_f = [x for x in f if "bias" in x["title"]]
        self.assertTrue(bias_f, titles(f))
        # The longest still run is the first, 10 s, where the gyro reads 0.001.
        self.assertIn("+0.00100", bias_f[0]["title"])
        self.assertIn("10.0 s", bias_f[0]["detail"])

    def test_a_large_bias_is_escalated_with_the_drift_it_implies(self):
        odom = [{"t": i * 0.01, "recv": i * 0.01, "v": 0.0, "wz": 0.0}
                for i in range(1000)]
        imu = [{"t": i * 0.01, "recv": i * 0.01, "wz": 0.05} for i in range(1000)]
        f = br.check_stationary_bias({"imu": imu, "odom": odom})
        w = sev(f, "WARNING")
        self.assertTrue(w, titles(f))
        self.assertIn("deg", w[0]["detail"])
        self.assertIn("observability", w[0]["fix"])

    def test_a_run_that_never_stops_says_so(self):
        odom = [{"t": i * 0.01, "recv": i * 0.01, "v": 1.0, "wz": 0.0}
                for i in range(1000)]
        imu = [{"t": i * 0.01, "recv": i * 0.01, "wz": 0.0} for i in range(1000)]
        f = br.check_stationary_bias({"imu": imu, "odom": odom})
        self.assertIn("No stationary period", titles(f))


class ObservabilityContentTest(unittest.TestCase):
    """Whether the bag contains a manoeuvre that makes the quantities observable."""

    def test_never_stopping_is_the_blocker(self):
        # Measured: the gyro reads rate + bias, so no trajectory separates them.
        # Only a zero-velocity update does, because it observes the rate with no
        # bias term. A bag with no stop cannot answer the question.
        odom = [{"t": i * 0.01, "recv": i * 0.01, "v": 1.0,
                 "wz": 0.5 if i % 2 else -0.5} for i in range(1000)]
        f = br.check_observability({"odom": odom})
        b = sev(f, "BLOCKER")
        self.assertTrue(b, titles(f))
        self.assertIn("never stopped", b[0]["title"])

    def test_turning_one_way_is_informational_not_a_blocker(self):
        # It was a blocker in the first version, on the reasoning that reversing the
        # turn separates a bias from a rate. Measured: it does not. A figure-eight
        # driven without stopping leaves the bias exactly as wrong as a circle does.
        odom = [{"t": i * 0.01, "recv": i * 0.01, "v": 1.0, "wz": 0.5}
                for i in range(1000)]
        odom += [{"t": 10 + i * 0.01, "recv": 10 + i * 0.01, "v": 0.0, "wz": 0.0}
                 for i in range(100)]
        f = br.check_observability({"odom": odom})
        one_way = [x for x in f if "only turned one way" in x["title"]]
        self.assertTrue(one_way, titles(f))
        self.assertEqual(one_way[0]["severity"], "INFO")
        self.assertIn("Only a stop does that", one_way[0]["fix"])

    def test_a_bag_with_a_stop_is_ok(self):
        odom = [{"t": i * 0.01, "recv": i * 0.01, "v": 1.0,
                 "wz": 0.5 if i % 2 else -0.5} for i in range(1000)]
        odom += [{"t": 10 + i * 0.01, "recv": 10 + i * 0.01, "v": 0.0, "wz": 0.0}
                 for i in range(100)]
        f = br.check_observability({"odom": odom})
        self.assertEqual(sev(f, "BLOCKER"), [], titles(f))
        self.assertTrue(any("stopped at least once" in x["title"] for x in f),
                        titles(f))

    def test_no_straight_driving_is_flagged(self):
        odom = [{"t": i * 0.01, "recv": i * 0.01, "v": 1.0,
                 "wz": 0.5 if i % 2 else -0.5} for i in range(1000)]
        f = br.check_observability({"odom": odom})
        self.assertIn("straight", titles(f).lower())


class InventoryTest(unittest.TestCase):

    def test_no_imu_is_a_blocker(self):
        f = br.check_inventory({"odom": stream(100, 20.0), "gnss": stream(50, 5.0)})
        self.assertTrue(sev(f, "BLOCKER"), titles(f))
        self.assertIn("No IMU", titles(f))

    def test_missing_gnss_and_odometry_are_warnings_not_blockers(self):
        f = br.check_inventory({"imu": stream(1000, 100.0)})
        self.assertEqual(sev(f, "BLOCKER"), [])
        self.assertEqual(len(sev(f, "WARNING")), 2, titles(f))


class ReportShapeTest(unittest.TestCase):

    def test_analyse_never_raises_on_an_empty_series(self):
        self.assertIsInstance(br.analyse({}), list)

    def test_one_broken_check_does_not_kill_the_report(self):
        bad = lambda s: (_ for _ in ()).throw(ValueError("boom"))
        saved = br.CHECKS
        try:
            br.CHECKS = (br.check_inventory, bad)
            f = br.analyse({"imu": stream(1000, 100.0)})
            self.assertIn("could not run", titles(f))
            self.assertIn("boom", " ".join(x["detail"] for x in f))
        finally:
            br.CHECKS = saved

    def test_blockers_sort_first(self):
        f = br.analyse({"odom": stream(500, 20.0, v=1.0, wz=0.5)})
        if sev(f, "BLOCKER"):
            self.assertEqual(f[0]["severity"], "BLOCKER", titles(f))

    def test_render_puts_blockers_in_their_own_section(self):
        f = [br.finding("BLOCKER", "B", "d", "fix it"),
             br.finding("INFO", "I", "d")]
        md = br.render(f, "mybag", 120.0)
        self.assertIn("# Estimator report: `mybag`", md)
        self.assertIn("## Fix these first", md)
        self.assertIn("**What to do:** fix it", md)
        self.assertLess(md.index("## Fix these first"), md.index("## Everything else"))

    def test_render_survives_a_finding_with_no_fix(self):
        md = br.render([br.finding("INFO", "I", "d")], "b")
        self.assertIn("I", md)

    def test_notes_from_reading_reach_the_findings(self):
        note = br.finding("WARNING", "chose a topic", "d")
        f = br.analyse({"notes": [note], "imu": stream(1000, 100.0)})
        self.assertIn("chose a topic", titles(f))


if __name__ == "__main__":
    unittest.main()
