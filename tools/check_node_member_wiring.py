#!/usr/bin/env python3
"""Check that ROS parameters read into a node member are actually used.

Companion to check_config_wiring.py, which covers the parameters that flow into
FusionCoreConfig. The other parameters land in a member of the node instead
(`member_ = get_parameter("name")`), and a member that is assigned and then
never read fails the same silent way a dead config field does: the parameter is
accepted, `ros2 param get` echoes it, and nothing happens.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
NODE = ROOT / "fusioncore_ros" / "src" / "fusion_node.cpp"
COMMENT = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)
MEMBER_ASSIGN = re.compile(
    r'^\s*([a-zA-Z_]\w*_)(?:\.\w+)?\s*=\s*[^;]*get_parameter\("([\w.]+)"\)',
    re.M,
)


def member_mappings(node_source):
    """Return (node member, ROS parameter) pairs assigned by fusion_node.cpp.

    A member is written once per parameter, so `gnss.lever_arm_x/y/z` all report
    against `gnss_lever_arm_`; the caller groups them by member.
    """
    return MEMBER_ASSIGN.findall(node_source)


def dead_members(mappings, node_source):
    """Return members that are assigned from a parameter but never otherwise read.

    The declaration and the assignment are both references, so a live member
    appears at least three times. Comments are stripped first so a member merely
    named in a nearby comment does not look used.
    """
    code = COMMENT.sub("", node_source)
    params_by_member = {}
    for member, parameter in mappings:
        params_by_member.setdefault(member, []).append(parameter)

    dead = []
    for member, parameters in params_by_member.items():
        if len(re.findall(rf"\b{re.escape(member)}\b", code)) <= 2:
            dead.append((member, sorted(set(parameters))))
    return sorted(dead)


def main():
    node_source = NODE.read_text()
    mappings = member_mappings(node_source)
    if not mappings:
        print(f"{NODE}: found no member/get_parameter assignments, the regex is wrong")
        return 1

    dead = dead_members(mappings, node_source)
    for member, parameters in dead:
        names = ", ".join(parameters)
        print(f"{names}: read into {member}, but {member} is never read afterwards")

    members = {member for member, _ in mappings}
    print(
        f"{len(members)} node members are assigned from parameters, {len(dead)} dead"
    )
    return 1 if dead else 0


if __name__ == "__main__":
    sys.exit(main())
