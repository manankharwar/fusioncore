#!/usr/bin/env python3
"""platform.profile must supply defaults, never override what the caller set.

The layering is the whole point and it is easy to get backwards. A profile says "on
this kind of robot these states are not worth estimating". An explicit parameter says
"I know what I am doing on this robot". The explicit one has to win, or the profile is
a trap: a user sets a value, sees it ignored, and has no way to find out why.

The first implementation got this wrong. It used
get_node_options().parameter_overrides(), which holds only overrides passed
programmatically through NodeOptions and never sees a --ros-args -p value. It silently
overwrote an explicitly-set ukf.q_gyro_bias and reported "0 kept". These tests exist so
that cannot come back.
"""
import os
import subprocess
import sys
import time
import unittest

NODE = "fusioncore_node"


def run_node(extra_args, settle=6.0):
    """Start the node unconfigured, configure it, return (state, params, log)."""
    name = "pt%d" % (int(time.time() * 1000) % 100000)
    cmd = ["ros2", "run", "fusioncore_ros", NODE, "--ros-args",
           "-r", "__node:=" + name, "-p", "autostart:=false"] + extra_args
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True)
    try:
        for _ in range(60):
            out = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True).stdout
            if "/" + name in out.split():
                break
            time.sleep(0.25)
        subprocess.run(["ros2", "lifecycle", "set", "/" + name, "configure"],
                       capture_output=True, text=True, timeout=30)
        time.sleep(settle)
        state = subprocess.run(["ros2", "lifecycle", "get", "/" + name],
                               capture_output=True, text=True).stdout.strip()
        params = {}
        for p in ("ukf.q_gyro_bias", "ukf.q_accel_bias", "ukf.q_position",
                  "ground_constraint.z_position_sigma", "gnss.reanchor_enabled"):
            r = subprocess.run(["ros2", "param", "get", "/" + name, p],
                               capture_output=True, text=True).stdout.strip()
            params[p] = r.split(":")[-1].strip() if ":" in r else r
        return state, params
    finally:
        proc.kill()
        proc.wait(timeout=10)
        subprocess.run(["pkill", "-9", "-f", NODE], capture_output=True)
        time.sleep(1.0)


class PlatformProfile(unittest.TestCase):

    def test_default_changes_nothing(self):
        state, p = run_node([])
        self.assertIn("inactive", state)
        self.assertAlmostEqual(float(p["ukf.q_gyro_bias"]), 1e-5, places=9,
                               msg="the default profile must not touch anything")
        self.assertAlmostEqual(float(p["ground_constraint.z_position_sigma"]), 0.0, places=9)

    def test_ground_robot_flat_applies(self):
        state, p = run_node(["-p", "platform.profile:=ground_robot_flat"])
        self.assertIn("inactive", state)
        self.assertAlmostEqual(float(p["ukf.q_gyro_bias"]), 0.0, places=12)
        self.assertAlmostEqual(float(p["ukf.q_accel_bias"]), 0.0, places=12)
        self.assertAlmostEqual(float(p["ground_constraint.z_position_sigma"]), 0.3, places=9)
        # a parameter the profile does not mention must be left alone
        self.assertAlmostEqual(float(p["ukf.q_position"]), 0.01, places=9)

    def test_an_explicit_value_beats_the_profile(self):
        # THE regression test. The profile wants 0.0 here and must not get it.
        state, p = run_node(["-p", "platform.profile:=ground_robot_flat",
                             "-p", "ukf.q_gyro_bias:=0.002"])
        self.assertIn("inactive", state)
        self.assertAlmostEqual(float(p["ukf.q_gyro_bias"]), 0.002, places=9,
                               msg="the profile overwrote a value the caller set explicitly")
        # and the rest of the profile still applies
        self.assertAlmostEqual(float(p["ukf.q_accel_bias"]), 0.0, places=12)

    def test_ground_robot_turns_reanchoring_on(self):
        state, p = run_node(["-p", "platform.profile:=ground_robot"])
        self.assertIn("inactive", state)
        self.assertEqual(p["gnss.reanchor_enabled"], "True")

    def test_general_leaves_reanchoring_off(self):
        state, p = run_node([])
        self.assertEqual(p["gnss.reanchor_enabled"], "False",
                         "the library default must stay off")

    def test_an_explicit_false_beats_the_profile(self):
        # false is also the declared default, so a compare-against-default test would
        # miss it and turn re-anchoring back on behind the caller's back.
        state, p = run_node(["-p", "platform.profile:=ground_robot",
                             "-p", "gnss.reanchor_enabled:=false"])
        self.assertIn("inactive", state)
        self.assertEqual(p["gnss.reanchor_enabled"], "False",
                         "the profile overrode an explicit gnss.reanchor_enabled: false")
        self.assertAlmostEqual(float(p["ukf.q_gyro_bias"]), 0.0, places=12)

    def test_an_explicit_false_in_a_params_file_beats_the_profile(self):
        import tempfile
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write("/**:\n  ros__parameters:\n    platform.profile: ground_robot\n"
                    "    gnss.reanchor_enabled: false\n")
            path = f.name
        try:
            state, p = run_node(["--params-file", path])
        finally:
            os.unlink(path)
        self.assertIn("inactive", state)
        self.assertEqual(p["gnss.reanchor_enabled"], "False",
                         "a params-file false must survive the profile too")

    def test_an_unknown_profile_refuses_to_configure(self):
        # Silently falling back to 'general' would be worse than failing: the caller
        # asked for a specific behaviour and would not get it.
        state, p = run_node(["-p", "platform.profile:=nonsense"])
        self.assertIn("unconfigured", state,
                      "an unknown profile must refuse to configure, not fall back")
        self.assertAlmostEqual(float(p["ukf.q_gyro_bias"]), 1e-5, places=9,
                               msg="nothing may be applied when the profile is invalid")


if __name__ == "__main__":
    unittest.main()
