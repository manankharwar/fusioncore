"""Frame-convention tests for the NCLT player. Issue #169.

The player converts NCLT's NED-like body frame to the ENU frame FusionCore expects.
It applied that conversion to the IMU and not to the wheel odometry, so the encoder
fed a yaw rate of the opposite sign to the gyro on every published benchmark number.

The filter never diverged on it, which is why twelve sequences of benchmarking did
not surface it: imu.gyro_noise is tighter than encoder.yaw_noise, so the filter
leaned on the gyro and spent gain rejecting the encoder on every turn. The in-filter
detector did fire, into a launch log nobody greps.

These lock the convention so the rule cannot be applied to one reader and forgotten
on the other again.
"""

import importlib.util
import math
import os
import pathlib
import sys
import types
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent


def _load_player():
    """Import nclt_player without a ROS installation.

    It imports rclpy and five message packages at module scope, and none of them
    matter for the pure conversion helpers. Stubbing is what makes this a tools test
    that runs anywhere rather than one that needs a sourced workspace.
    """
    stubs = {
        "rclpy": ["init", "shutdown", "spin"],
        "rclpy.node": ["Node"],
        "builtin_interfaces.msg": ["Time"],
        "rosgraph_msgs.msg": ["Clock"],
        "sensor_msgs.msg": ["Imu", "NavSatFix", "NavSatStatus"],
        "nav_msgs.msg": ["Odometry"],
        "geometry_msgs.msg": ["Quaternion"],
    }
    saved = {}
    for name, attrs in stubs.items():
        saved[name] = sys.modules.get(name)
        mod = types.ModuleType(name)
        for a in attrs:
            setattr(mod, a, type(a, (), {}))
        sys.modules[name] = mod
    for pkg in ("builtin_interfaces", "rosgraph_msgs", "sensor_msgs", "nav_msgs",
                "geometry_msgs"):
        saved.setdefault(pkg, sys.modules.get(pkg))
        sys.modules.setdefault(pkg, types.ModuleType(pkg))
    try:
        spec = importlib.util.spec_from_file_location(
            "nclt_player", ROOT / "fusioncore_datasets" / "scripts" / "nclt_player.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        for name, old in saved.items():
            if old is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old


player = _load_player()


class YawRateConventionTest(unittest.TestCase):

    def test_the_conversion_negates(self):
        self.assertEqual(player.ned_yaw_rate_to_enu(0.6), -0.6)
        self.assertEqual(player.ned_yaw_rate_to_enu(-0.6), 0.6)

    def test_zero_is_unchanged_and_keeps_no_sign(self):
        self.assertEqual(player.ned_yaw_rate_to_enu(0.0), 0.0)

    def test_it_is_its_own_inverse(self):
        for w in (0.0, 0.08, -1.7, 3.14159):
            self.assertAlmostEqual(
                player.ned_yaw_rate_to_enu(player.ned_yaw_rate_to_enu(w)), w)

    def test_it_agrees_with_the_rule_applied_to_the_imu(self):
        # The player's own header states wz_enu = -wz_ned and the IMU reader does
        # `wz = -float(row[9])`. The odometry path must use the SAME rule, or the two
        # rotation sources disagree in sign. That identity is the whole bug.
        gyro_z_ned = 0.42
        imu_enu = -gyro_z_ned
        odom_enu = player.ned_yaw_rate_to_enu(gyro_z_ned)
        self.assertEqual(imu_enu, odom_enu)

    def test_a_left_turn_is_positive_under_rep103(self):
        # REP-103: turning LEFT gives a POSITIVE angular_velocity.z. NED yaw increases
        # clockwise, so a left turn is a NEGATIVE NED rate, and the conversion must
        # make it positive.
        self.assertGreater(player.ned_yaw_rate_to_enu(-0.5), 0.0)


class AgainstTheRealDatasetTest(unittest.TestCase):
    """Skipped unless the NCLT CSVs are on disk. This is the evidence, not a mock."""

    DATA = pathlib.Path(os.environ.get("NCLT_DATA", pathlib.Path.home() / "nclt"))
    SEQ = "2013-04-05"

    def setUp(self):
        self.d = self.DATA / self.SEQ
        if not (self.d / "ms25.csv").exists() or \
           not (self.d / "odometry_mu_100hz.csv").exists():
            self.skipTest(f"no NCLT CSVs at {self.d}")

    def _paired(self, limit=120000):
        import bisect, csv

        def angdiff(a, b):
            dd = a - b
            while dd > math.pi:
                dd -= 2 * math.pi
            while dd < -math.pi:
                dd += 2 * math.pi
            return dd

        odo, prev = [], None
        with open(self.d / "odometry_mu_100hz.csv") as f:
            for r in csv.reader(f):
                if not r or r[0].startswith("#"):
                    continue
                try:
                    u, h = int(r[0]), float(r[6])
                except (ValueError, IndexError):
                    continue
                if prev:
                    dt = (u - prev[0]) / 1e6
                    if 0 < dt <= 0.5:
                        odo.append((u, angdiff(h, prev[1]) / dt))
                prev = (u, h)
                if len(odo) >= limit:
                    break
        imu = []
        with open(self.d / "ms25.csv") as f:
            for r in csv.reader(f):
                if not r or r[0].startswith("#"):
                    continue
                try:
                    imu.append((int(r[0]), float(r[9])))
                except (ValueError, IndexError):
                    continue
        imu.sort()
        ut = [x[0] for x in imu]
        out = []
        for u, w in odo:
            i = bisect.bisect_left(ut, u)
            if i <= 0 or i >= len(ut):
                continue
            j = i if abs(ut[i] - u) < abs(ut[i - 1] - u) else i - 1
            if abs(ut[j] - u) <= 50000:
                out.append((w, imu[j][1]))
        return [(a, b) for a, b in out if abs(a) > 0.08 and abs(b) > 0.08]

    def test_raw_odometry_and_raw_gyro_share_a_frame(self):
        # This is the proof the odometry is NED: unconverted, the two agree.
        both = self._paired()
        self.assertGreater(len(both), 1000, "not enough turning samples to judge")
        dis = sum(1 for a, b in both if (a > 0) != (b > 0)) / len(both)
        self.assertLess(dis, 0.02, f"raw odom and raw gyro disagreed {100*dis:.1f}%")

    def test_the_fix_makes_the_two_sources_agree(self):
        both = self._paired()
        imu_enu = [-b for _, b in both]
        odo_enu = [player.ned_yaw_rate_to_enu(a) for a, _ in both]
        dis = sum(1 for a, b in zip(odo_enu, imu_enu) if (a > 0) != (b > 0)) / len(both)
        self.assertLess(dis, 0.02, f"after conversion they still disagreed {100*dis:.1f}%")

    def test_without_the_fix_they_disagree_on_essentially_every_sample(self):
        # The shipped behaviour, pinned. If this ever stops failing-to-agree, the
        # dataset or the reader changed and the whole analysis needs revisiting.
        both = self._paired()
        imu_enu = [-b for _, b in both]
        dis = sum(1 for (a, _), b in zip(both, imu_enu) if (a > 0) != (b > 0)) / len(both)
        self.assertGreater(dis, 0.98, f"expected near-total disagreement, got {100*dis:.1f}%")

    def test_the_magnitude_mismatch_is_separate_and_still_open(self):
        # The sign is fixed; the SCALE is not. NCLT's wheel odometry over-reports
        # rotation by about 30%. Recorded here so the next person does not assume the
        # sign fix closed it.
        import statistics as st
        both = self._paired()
        ratio = st.median(abs(a) / abs(b) for a, b in both if abs(b) > 1e-6)
        self.assertGreater(ratio, 1.15, f"ratio {ratio:.3f}: the over-report vanished")
        self.assertLess(ratio, 1.45, f"ratio {ratio:.3f}: moved outside the measured 1.297")


if __name__ == "__main__":
    unittest.main()
