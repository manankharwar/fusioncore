#!/usr/bin/env python3
"""Platform profiles must only set parameters the node actually declares.

A profile is a YAML bundle a user copies into their config. If a key in it is
misspelled or renamed out from under it, ROS ignores the key ENTIRELY and silently
falls back to the library default. The user sees a profile that loaded without error
and a filter that is not configured the way the profile says it is.

That failure mode is the most repeated bug in this repo: a PR faithfully parameterised
the confirmation-count knob of issue #126 and the branch behind it turned out to be dead,
replay.cpp accepted 43 parameters that never reached the filter, and a rover params file
keyed to the wrong node name silently restored the two worst settings ever measured on
that hardware. A profile is the same shape of risk with a friendlier name.
"""
import os
import pathlib
import re
import sys
import unittest

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_config_params as ccp

PROFILES = pathlib.Path(__file__).resolve().parent.parent / \
    "fusioncore_datasets" / "config" / "profiles"


def profile_files():
    return sorted(PROFILES.glob("*.yaml"))


def params(path):
    doc = yaml.safe_load(path.read_text())
    return doc["fusioncore"]["ros__parameters"]


class Profiles(unittest.TestCase):

    def test_there_are_profiles_to_check(self):
        # Guards against the glob silently matching nothing, which would make every
        # other test here pass by doing no work.
        self.assertGreaterEqual(len(profile_files()), 2, f"no profiles found in {PROFILES}")

    def test_every_profile_is_valid_yaml_keyed_to_the_node(self):
        for f in profile_files():
            with self.subTest(profile=f.name):
                doc = yaml.safe_load(f.read_text())
                self.assertIn("fusioncore", doc,
                              "a profile keyed to the wrong node name is ignored entirely")
                self.assertIn("ros__parameters", doc["fusioncore"])

    def test_every_parameter_is_declared_by_the_node(self):
        declared = ccp.declared_parameters(ccp.NODE)
        self.assertGreater(len(declared), 50, "could not read the node's declarations")
        for f in profile_files():
            with self.subTest(profile=f.name):
                undeclared = sorted(k for k in params(f) if k not in declared)
                self.assertEqual([], undeclared,
                                 f"{f.name} sets parameters the node does not declare, "
                                 f"so ROS will ignore them silently: {undeclared}")

    def test_flat_is_a_superset_of_ground_robot(self):
        # The flat profile is documented as "ground_robot plus a Z constraint". If it
        # ever drifts from that, the documentation is lying rather than merely stale.
        base = params(PROFILES / "ground_robot.yaml")
        flat = params(PROFILES / "ground_robot_flat.yaml")
        for k, v in base.items():
            self.assertIn(k, flat, f"ground_robot_flat dropped {k}")
            self.assertEqual(v, flat[k], f"ground_robot_flat changed {k}")
        extra = set(flat) - set(base)
        self.assertEqual({"ground_constraint.z_position_sigma"}, extra,
                         "flat should add exactly the Z constraint and nothing else")

    def test_the_frozen_states_are_actually_frozen(self):
        # The whole point of ground_robot is that these four stop being driven. A
        # non-zero here would make the profile a no-op while still reading like one.
        p = params(PROFILES / "ground_robot.yaml")
        for k in ("ukf.q_acceleration", "ukf.q_accel_bias",
                  "ukf.q_gyro_bias", "ukf.q_encoder_wz_bias"):
            self.assertEqual(0.0, float(p[k]), f"{k} must be 0 for the profile to mean anything")
        # and the priors pinned, or a measurement can still move a q=0 state
        for k in ("imu.gyro_bias_initial_sigma", "imu.accel_bias_initial_sigma",
                  "encoder.wz_scale_initial_sigma"):
            self.assertLess(float(p[k]), 1e-4, f"{k} must be pinned, not merely small")


class BenchmarkMatchesTheProfile(unittest.TestCase):
    """The NCLT A/B config must set exactly what ground_robot_flat ships.

    The profile's entire claim is a measurement: 2012-06-15 going from 82.011 to
    32.149. That claim is only honest if the
    run that produced it used the settings the profile hands to a user. If the two
    drift apart, the README is quoting numbers for a configuration nobody ships.

    This is cheap to check and silent to get wrong, which is the same shape as the
    README benchmark table before #174 added a checker for it.
    """

    # Added to the profiles after the frozen benchmark ran, and measured on its own:
    # all twelve NCLT sequences in tools/eval_20261025_results.md and through the node on
    # 2012-01-08 (tools/benchmark_regression.md, 2026-10-26). The frozen config's numbers
    # describe the profile without it, which is what they were measured as.
    MEASURED_SEPARATELY = {"gnss.reanchor_enabled"}

    BENCH = pathlib.Path(__file__).resolve().parent.parent / \
        "fusioncore_datasets" / "config" / "nclt_fusioncore_frozen.yaml"

    def test_the_benchmark_config_sets_exactly_the_profile_values(self):
        self.assertTrue(self.BENCH.exists(), f"missing {self.BENCH}")
        prof = params(PROFILES / "ground_robot_flat.yaml")
        bench = yaml.safe_load(self.BENCH.read_text())["fusioncore"]["ros__parameters"]
        for k, v in prof.items():
            if k in self.MEASURED_SEPARATELY:
                continue
            self.assertIn(k, bench,
                          f"the benchmark does not set {k}, so the measured numbers do "
                          f"not describe the shipped profile")
            self.assertEqual(float(v), float(bench[k]),
                             f"{k}: profile ships {v}, benchmark measured {bench[k]}")


class TheNodeTableMatchesTheProfiles(unittest.TestCase):
    """platform.profile in the node must set exactly what the YAML files set.

    There are now two ways to ask for the same thing: load ground_robot.yaml, or set
    platform.profile to ground_robot. If they drift apart, two users following two
    pieces of documentation get two different filters and neither of them is wrong.

    The third column of the node's table is the subtler risk. It holds what the node
    DECLARES, and it is how "the user set this" is told apart from "nobody set this".
    If a declare_parameter default changes and that column does not, the profile reads
    every value as user-supplied, applies nothing, and logs that it kept everything.
    It would be a no-op that announces success, which is the worst version of this bug
    and would not show up in any other test here.
    """

    NODE = pathlib.Path(ccp.NODE)

    def node_table(self):
        """The {name, profile value, declared value} rows out of apply_platform_profile."""
        src = self.NODE.read_text()
        body = re.search(r"std::vector<Row> defaults = \{(.*?)\n    \};", src, re.S)
        self.assertIsNotNone(body, "could not find the Row table; the parser is stale")
        rows = re.findall(r'\{\s*"([^"]+)"\s*,\s*([-\d.e+]+)\s*,\s*([-\d.e+]+)\s*\}',
                          body.group(1))
        self.assertGreaterEqual(len(rows), 7, "parsed too few rows out of the node")
        flat = re.search(r'defaults\.push_back\(\{\s*"([^"]+)"\s*,\s*([-\d.e+]+)\s*,'
                         r'\s*([-\d.e+]+)\s*\}\)', src)
        self.assertIsNotNone(flat, "could not find the flat-only row")
        base = {n: (float(pv), float(dv)) for n, pv, dv in rows}
        return base, {flat.group(1): (float(flat.group(2)), float(flat.group(3)))}

    def node_bool_table(self):
        """The {name, profile value} rows of the boolean table. Booleans are judged by
        whether the caller supplied them at all (an explicit false equals the default),
        so they have no declared column."""
        src = self.NODE.read_text()
        body = re.search(r"std::vector<BoolRow> bool_defaults = \{(.*?)\n    \};", src, re.S)
        self.assertIsNotNone(body, "could not find the BoolRow table; the parser is stale")
        rows = re.findall(r'\{\s*"([^"]+)"\s*,\s*(true|false)\s*\}', body.group(1))
        self.assertGreaterEqual(len(rows), 1, "parsed no rows out of the bool table")
        return {n: v == "true" for n, v in rows}

    @staticmethod
    def same(yaml_value, node_value):
        if isinstance(node_value, bool):
            return yaml_value is node_value
        return float(yaml_value) == node_value

    def declared_defaults(self):
        src = self.NODE.read_text()
        out = {}
        for name, val in re.findall(
                r'declare_parameter\(\s*"([^"]+)"\s*,\s*([-\d.][-\d.e+]*)\s*\)', src):
            out[name] = float(val)
        self.assertGreater(len(out), 40, "could not read the node's declared defaults")
        return out

    def test_the_node_sets_the_same_values_as_ground_robot_yaml(self):
        base, _ = self.node_table()
        node = {k: v[0] for k, v in base.items()}
        node.update(self.node_bool_table())
        yaml_vals = params(PROFILES / "ground_robot.yaml")
        self.assertEqual(sorted(yaml_vals), sorted(node),
                         "the node's table and ground_robot.yaml set different parameters")
        for k, v in yaml_vals.items():
            self.assertTrue(self.same(v, node[k]),
                            f"{k}: the YAML ships {v}, platform.profile applies {node[k]}")

    def test_the_flat_row_matches_ground_robot_flat_yaml(self):
        base, extra = self.node_table()
        yaml_vals = params(PROFILES / "ground_robot_flat.yaml")
        combined = {k: v[0] for k, v in base.items()}
        combined.update({k: v[0] for k, v in extra.items()})
        combined.update(self.node_bool_table())
        self.assertEqual(sorted(yaml_vals), sorted(combined),
                         "the node's flat table and ground_robot_flat.yaml differ")
        for k, v in yaml_vals.items():
            self.assertTrue(self.same(v, combined[k]),
                            f"{k}: the YAML ships {v}, platform.profile applies {combined[k]}")

    def test_every_bool_row_differs_from_the_declared_default(self):
        # Same dead-weight check as the numeric rows, for the boolean table.
        declared = dict(re.findall(r'declare_parameter\(\s*"([^"]+)"\s*,\s*(true|false)\s*\)',
                                   self.NODE.read_text()))
        for name, want in self.node_bool_table().items():
            self.assertIn(name, declared, f"{name} is not declared as a bool")
            self.assertNotEqual(want, declared[name] == "true",
                                f"{name} sets the value the node already declares")

    def test_the_declared_column_is_what_the_node_declares(self):
        # The one that catches a silent no-op. See the class docstring.
        base, extra = self.node_table()
        declared = self.declared_defaults()
        rows = dict(base)
        rows.update(extra)
        for name, (_, claimed) in rows.items():
            self.assertIn(name, declared,
                          f"{name} is in the profile table but is not declared with a "
                          f"numeric default")
            self.assertEqual(declared[name], claimed,
                             f"{name}: the node declares {declared[name]} but the profile "
                             f"table says {claimed}, so an unset value would be read as "
                             f"user-supplied and the profile would apply nothing")

    def test_the_profile_actually_changes_every_value(self):
        # A row whose profile value equals its declared default does nothing and should
        # be deleted rather than left looking meaningful.
        base, extra = self.node_table()
        rows = dict(base)
        rows.update(extra)
        for name, (want, declared) in rows.items():
            self.assertNotEqual(want, declared,
                                f"{name} sets the value the node already declares, so the "
                                f"row is dead weight")


if __name__ == "__main__":
    unittest.main()
