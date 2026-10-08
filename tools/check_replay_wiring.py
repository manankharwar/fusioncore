#!/usr/bin/env python3
"""Every ROS parameter that reaches FusionCoreConfig must also reach the offline replay.

    python3 tools/check_replay_wiring.py

tools/check_config_wiring.py already checks that a declared ROS parameter reaches
FusionCoreConfig. Nothing checked the OTHER consumer of that same config: the offline
replay in hardware/replay.cpp, which is what every config sweep and bisect runs through.

It went unchecked and it drifted. Found 2026-09-26/27/28:

  - 43 of the rover's parameters and 11 of NCLT's were accepted and silently ignored, so
    every offline sweep before that date replayed library defaults for them and still
    printed a confident number.
  - gnss.track_heading_max_window_turn_deg was not wired, so the first #144 A/B returned
    byte-identical results for gate on and gate off.
  - ukf.max_sigma_rotation_deg was not wired, which would have done the same to #150.

A parameter that is accepted and ignored is worse than one that is rejected, because it
answers confidently. This check makes that drift fail instead of pass.

hardware/ is gitignored, so replay.cpp is absent from a fresh clone. This SKIPS cleanly
in that case rather than failing CI on a file that was never meant to be there.
"""
import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPLAY = os.path.join(ROOT, "hardware", "replay.cpp")
NODE = os.path.join(ROOT, "fusioncore_ros", "src", "fusion_node.cpp")

# Parameters that legitimately reach only the node and have no fusioncore_core field, so
# the replay cannot honour them. replay.cpp keeps the same list as kNodeLevel; this is the
# cross-check that the two agree about what "node level" means.
NODE_ONLY_HINT = re.compile(
    r"^(gnss\.(fix_topic|use_gps_fix|heading_topic|azimuth_topic|min_fix_type|lever_arm|"
    r"min_sigma|recovery_timeout_s|apply_lever_arm_pre_heading|frame_id)|"
    r"imu\.(topic|frame_id|lever_arm|remove_grav|has_magnetometer)|"
    r"magnetometer\.(topic|hard_iron|soft_iron)|"
    r"encoder2?\.(topic|nhc_auto_detect)|"
    r"(input|output|reference|publish|init)\.|"
    r"vslam\.|base_frame|odom_frame|map_frame|autostart|diagnostic|"
    r"use_sim_time|qos_|.*_topic$)")


def declared_params(path):
    src = open(path).read()
    return set(re.findall(r'declare_parameter\(\s*"([^"]+)"', src))


def replay_wired(path):
    src = open(path).read()
    wired = set(re.findall(r'get\(\s*"([^"]+)"', src))
    block = re.search(r"kNodeLevel\s*=\s*\{(.*?)\};", src, re.S)
    node_level = set(re.findall(r'"([^"]+)"', block.group(1))) if block else set()
    return wired, node_level


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.parse_args(argv)
    if not os.path.isfile(REPLAY):
        print("hardware/replay.cpp not present (it is gitignored), skipping.")
        return 0
    if not os.path.isfile(NODE):
        print(f"missing {NODE}", file=sys.stderr)
        return 2

    declared = declared_params(NODE)
    wired, node_level = replay_wired(REPLAY)

    missing = []
    for p in sorted(declared):
        if p in wired or p in node_level:
            continue
        if NODE_ONLY_HINT.match(p):
            continue                      # plainly node-level and not claimed by either
        missing.append(p)

    # A key listed as node-level AND wired is a contradiction: it says the replay both
    # can and cannot honour it, and one of the two is a lie.
    contradictory = sorted(wired & node_level)

    print(f"{len(declared)} declared parameters | {len(wired)} wired into the replay | "
          f"{len(node_level)} declared node-level")

    ok = True
    if contradictory:
        ok = False
        print("\nBOTH wired and listed node-level, which cannot both be true:")
        for p in contradictory:
            print(f"  {p}")
    if missing:
        ok = False
        print("\nReach FusionCoreConfig but are NOT wired into hardware/replay.cpp,")
        print("so a sweep of them would silently replay the library default:")
        for p in missing:
            print(f"  {p}")
        print("\nWire it with a get(\"<key>\", <library default>) call, or add it to")
        print("kNodeLevel in replay.cpp if fusioncore_core genuinely has no such field.")

    if ok:
        print("\nOK: no parameter reaches the filter without also reaching the replay.")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
