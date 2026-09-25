#!/usr/bin/env python3
"""Check that every ROS parameter named in the docs actually exists.

The third member of the wiring-checker family. check_config_wiring.py catches a
parameter that reaches FusionCoreConfig and is never read;
check_node_member_wiring.py catches one that lands in a node member and is never
read. This one catches the opposite direction: documentation telling a user to set
a parameter the node never declares.

That failure is invisible from both sides. ROS 2 silently ignores an override for
an undeclared parameter, so the user sets it, `ros2 param list` never shows it,
nothing warns, and the documented behaviour simply does not happen.

It is not hypothetical and it recurs. From the 0.3.x changelog:

    configuration.md had a non-existent param: gnss.degraded_noise_multiplier was
    documented but never implemented. Removed.

and found again in this audit: `gnss.use_wall_clock_stamp` and `imu.buffer_duration`
in the delayed-GPS guide (neither has ever existed), and
`gnss.gps_track_heading_cross_check_deg` in the 0.4.0 release notes, which carries
the C++ field's `gps_` prefix rather than the parameter's real name.

CHANGELOG.md is skipped on purpose: a changelog legitimately names parameters that
were removed, and rewriting history to satisfy a checker would be worse than the
rot it prevents.

Usage:
    python3 tools/check_docs_params.py
"""

import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
NODE = ROOT / "fusioncore_ros" / "src" / "fusion_node.cpp"

DECLARE = re.compile(r'declare_parameter[<\w>]*\(\s*"([\w.]+)"')

# Only tokens in backticks, and only ones shaped like one of our parameter groups.
# Prose mentioning "the gnss topic" should not trip this; `gnss.fix_topic` should.
GROUPS = ("gnss", "imu", "imu2", "encoder", "encoder2", "vslam", "mag",
          "magnetometer", "zupt", "publish", "init", "reference", "radar")
TOKEN = re.compile(r"`((?:" + "|".join(GROUPS) + r")\.[a-z][\w.]*)`")

# A changelog describes history, including removals. Benchmark result dumps are
# machine-written records of a past run, not instructions to a user.
SKIP_EXACT = {"CHANGELOG.md"}
SKIP_PREFIX = ("benchmarks/nclt/",)


def tracked_docs():
    out = subprocess.run(["git", "ls-files", "*.md"], cwd=ROOT,
                         capture_output=True, text=True).stdout.split()
    return [d for d in out
            if d not in SKIP_EXACT and not d.startswith(SKIP_PREFIX)]


def main():
    declared = set(DECLARE.findall(NODE.read_text()))
    if not declared:
        print(f"{NODE}: found no declare_parameter calls, the regex is wrong")
        return 1

    docs = tracked_docs()
    bad = {}
    for d in docs:
        try:
            text = (ROOT / d).read_text()
        except OSError:
            continue
        for name in sorted(set(TOKEN.findall(text))):
            if name not in declared:
                bad.setdefault(name, []).append(d)

    for name in sorted(bad):
        where = ", ".join(sorted(set(bad[name])))
        print(f"{name}: named in {where}, but no such parameter is declared")

    print(f"{len(docs)} docs checked against {len(declared)} declared "
          f"parameters, {len(bad)} that do not exist")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
