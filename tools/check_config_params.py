#!/usr/bin/env python3
"""Check shipped config YAMLs for undeclared or unsafe parameters.

ROS 2 silently ignores a parameter a node did not declare. No error, no warning,
nothing in the log: the value simply has no effect, forever. A typo like
gnss.max_hodp is therefore invisible, and the user drives believing the gate is
tuned. Three certified configs carried gnss.max_hdop as though it were the active
gate for months (issue #79) for exactly this reason.

Usage:
    python3 tools/check_config_params.py [--quiet]

Warnings describe hardware-dependent settings and do not fail CI. Errors describe
settings the documented filter model says are unsafe and exit 1.
"""
import glob
import os
import re
import sys
import textwrap

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE = os.path.join(ROOT, "fusioncore_ros", "src", "fusion_node.cpp")

# Configs that configure OUR node. robot_localization and navsat_transform
# comparison configs are deliberately excluded: they configure other people's
# nodes and their keys are none of our business.
CONFIG_GLOBS = [
    "fusioncore_ros/config/*.yaml",
    "fusioncore_gazebo/config/fusioncore_gazebo.yaml",
    "fusioncore_datasets/config/nclt_fusioncore.yaml",
    "tools/quick_test_params.yaml",
    "docs/**/*.yaml",
]

# A 95% chi-squared gate is the tightest supported setting. The defaults use
# 99.9%; this lower floor still leaves room for deliberately tighter shipped
# configurations while rejecting settings known to reject ordinary sensor noise.
OUTLIER_THRESHOLD_MINIMUMS = {
    "outlier_threshold_gnss": (7.81, "chi2(3, 0.95)",
                               "At 7.0, normal GNSS noise tripped the gate in a field test."),
    "outlier_threshold_imu": (12.59, "chi2(6, 0.95)",
                              "A lower threshold rejects more than 5% of ordinary 6-DOF updates."),
    "outlier_threshold_enc": (7.81, "chi2(3, 0.95)",
                              "A lower threshold rejects more than 5% of ordinary 3-DOF updates."),
    "outlier_threshold_hdg": (3.84, "chi2(1, 0.95)",
                              "A lower threshold rejects more than 5% of ordinary heading updates."),
    "outlier_threshold_vslam": (12.59, "chi2(6, 0.95)",
                                "A lower threshold rejects more than 5% of ordinary 6-DOF updates."),
}

# Values used by the node when a config does not spell the parameter out. They
# let relationship rules reason about an effective configuration without
# duplicating every node default here.
RELATIONSHIP_DEFAULTS = {
    "outlier_rejection": True,
    "magnetometer.enabled": False,
    "gnss.max_speed_sigma_k": 5.0,
    "gnss.max_sigma_xy": 25.0,
}


def declared_parameters(path):
    """Every name passed to declare_parameter() in the node."""
    src = open(path).read()
    names = set(re.findall(r'declare_parameter[<\w,\s>]*\(\s*"([^"]+)"', src))
    if not names:
        sys.exit(f"{path}: found no declare_parameter calls, the regex is wrong")
    return names


def config_parameters(path):
    """Parameters under a node's ros__parameters as (name, line, value).

    Hand-rolled rather than yaml.safe_load because these files are heavily
    commented and the comments carry the reasoning: keeping the parser dumb means
    it reports the line number a bad key is actually on.
    """
    # Only blocks belonging to OUR node. A YAML can configure several nodes at
    # once, and the docs ship exactly such a file: a Nav2 collision monitor sits
    # alongside fusioncore and its keys (PolygonStop, observation_sources,
    # cmd_vel_in_topic) are none of our business. Keying off the node name above
    # ros__parameters is what separates them.
    out = []
    in_params = False
    params_indent = 0
    stack = []
    node_name = None
    mine = False
    for n, raw in enumerate(open(path), 1):
        line = raw.rstrip("\n")
        if not line.strip() or line.strip().startswith("#"):
            continue
        indent = len(line) - len(line.lstrip())
        key = line.strip().split(":")[0].strip()
        if key == "ros__parameters":
            in_params, params_indent, stack = True, indent, []
            # "fusioncore", "/fusioncore", "/**/fusioncore", "a200_0000/fusioncore"
            mine = bool(node_name) and (node_name.rstrip("/").split("/")[-1] == "fusioncore"
                                        or node_name in ("/**", "**"))
            continue
        if not in_params and ":" in line and not line.split(":", 1)[1].strip():
            node_name = key
        if not in_params or not mine:
            continue
        if indent <= params_indent:
            in_params = False
            continue
        stack = [(i, k) for (i, k) in stack if i < indent]
        value = line.split(":", 1)[1].strip() if ":" in line else ""
        if not value:                       # a nesting level, not a leaf
            stack.append((indent, key))
            continue
        if line.strip().startswith("-"):    # list continuation
            continue
        out.append((".".join([k for _, k in stack] + [key]), n, value))
    return out


def scalar(value):
    """Parse the scalar forms this checker needs without adding PyYAML."""
    value = re.split(r"\s+#", value, maxsplit=1)[0].strip()
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    try:
        return float(value)
    except ValueError:
        return value.strip("\"'")


def number(values, name):
    """Return a configured numeric value, or None for a non-number/missing key."""
    value = values.get(name, (RELATIONSHIP_DEFAULTS.get(name), None, None))[0]
    return value if isinstance(value, float) else None


def value_findings(parameters):
    """Return (severity, name, line, raw_value, message) value findings.

    The rules are intentionally split into strict errors and advisory warnings:
    the chi-squared floors are documented mathematical limits, whereas GNSS
    consistency depends on the specific receiver and environment.
    """
    values = {name: (scalar(raw), line, raw) for name, line, raw in parameters}
    findings = []

    if values.get("outlier_rejection", (True, None, None))[0]:
        for name, (minimum, distribution, evidence) in OUTLIER_THRESHOLD_MINIMUMS.items():
            configured = values.get(name)
            configured_number = number(values, name)
            if configured and configured_number is not None and configured_number < minimum:
                _, line, raw = configured
                findings.append((
                    "ERROR", name, line, raw,
                    f"This is below {distribution}, the lowest supported outlier gate. "
                    "Normal sensor noise will trip the gate and measurements will be "
                    f"rejected. {evidence}",
                ))

    continuity = number(values, "gnss.continuity_max_m")
    if continuity is not None and 0.0 < continuity < 3.0:
        _, line, raw = values["gnss.continuity_max_m"]
        findings.append((
            "WARNING", "gnss.continuity_max_m", line, raw,
            "This is low enough to reject good GNSS fixes. Across 2,361 fixes from "
            "seven field logs, 3.0 m rejected none while 2.0 m rejected 12 good fixes. "
            "Measure your receiver's fix-to-fix consistency before tightening it.",
        ))

    field_strength = number(values, "magnetometer.field_strength")
    if field_strength is not None and field_strength != 0.0 and not values.get(
            "magnetometer.enabled", (False, None, None))[0]:
        _, line, raw = values["magnetometer.field_strength"]
        findings.append((
            "WARNING", "magnetometer.field_strength", line, raw,
            "The magnetic-field gate is configured but never runs because "
            "magnetometer.enabled is false.",
        ))

    max_speed = number(values, "gnss.max_speed")
    sigma_k = number(values, "gnss.max_speed_sigma_k")
    if max_speed is not None and max_speed > 0.0 and sigma_k is not None and sigma_k <= 0.0:
        _, line, raw = values["gnss.max_speed_sigma_k"]
        findings.append((
            "WARNING", "gnss.max_speed_sigma_k", line, raw,
            "With gnss.max_speed enabled this makes the jump gate absolute metres, "
            "not receiver-sigma-scaled. On a u-blox M9N, max_speed 2.0 with the "
            "default margin rejected 157 of 500 good fixes and worsened loop closure "
            "from 2.62 m to 7.27 m.",
        ))

    outlier_sigma = number(values, "gnss.outlier_sigma_xy")
    max_sigma = number(values, "gnss.max_sigma_xy")
    if outlier_sigma is not None and outlier_sigma > 0.0 and max_sigma is not None \
            and outlier_sigma >= max_sigma:
        _, line, raw = values["gnss.outlier_sigma_xy"]
        findings.append((
            "WARNING", "gnss.outlier_sigma_xy", line, raw,
            "This cannot tighten the outlier gate: every accepted covariance-reporting "
            "fix has sigma no larger than gnss.max_sigma_xy. Set this to the receiver's "
            "measured short-term fix-to-fix consistency, which must be smaller than its "
            "reported sigma. On a 3.24 m receiver, 5.0 loosened the rejection threshold "
            "from 26 m to 30 m.",
        ))

    return findings


def print_finding(severity, name, line, raw, message):
    print(f"  line {line:4d}  {name}: {raw}")
    print(textwrap.fill(f"{severity}: {message}", width=84, initial_indent="             ",
                        subsequent_indent="             "))


def main():
    quiet = "--quiet" in sys.argv
    declared = declared_parameters(NODE)
    files, errors, warnings = 0, 0, 0
    for pattern in CONFIG_GLOBS:
        for path in sorted(glob.glob(os.path.join(ROOT, pattern), recursive=True)):
            rel = os.path.relpath(path, ROOT)
            parameters = config_parameters(path)
            unknown = [(k, n) for k, n, _ in parameters if k not in declared]
            findings = value_findings(parameters)
            files += 1
            if unknown or findings:
                print(f"\n{rel}")
                for k, n in unknown:
                    errors += 1
                    near = sorted(d for d in declared if d.split(".")[-1] == k.split(".")[-1])
                    hint = f"   did you mean {near[0]}?" if near else ""
                    print(f"  line {n:4d}  {k}  NOT DECLARED BY THE NODE{hint}")
                for finding in findings:
                    if finding[0] == "ERROR":
                        errors += 1
                    else:
                        warnings += 1
                    print_finding(*finding)
            elif not quiet:
                print(f"ok  {rel}")
    print(f"\n{files} config files checked, {errors} errors, {warnings} warnings")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
