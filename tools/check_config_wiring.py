#!/usr/bin/env python3
"""Check that ROS parameters mapped into FusionCoreConfig are actually read."""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
NODE = ROOT / "fusioncore_ros" / "src" / "fusion_node.cpp"
CORE_DIRS = (ROOT / "fusioncore_core" / "include", ROOT / "fusioncore_core" / "src")
COMMENT = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)
DECLARE_PARAMETER = re.compile(r'declare_parameter\s*\(\s*"([\w.]+)"')


def config_mappings(node_source):
    """Return (config field, ROS parameter) pairs assigned by fusion_node.cpp."""
    return re.findall(
        r'config\.((?:\w+\.)?\w+)\s*=\s*[^;]*get_parameter\("([\w.]+)"\)',
        node_source,
    )


def declared_parameters(node_source):
    """Return the parameter names declared by fusion_node.cpp."""
    return DECLARE_PARAMETER.findall(node_source)


def dead_mappings(mappings, core_source):
    """Return mappings whose config field is only declared, never read by core."""
    code = COMMENT.sub("", core_source)
    dead = []
    for field, parameter in mappings:
        member = field.rsplit(".", 1)[-1]
        if len(re.findall(rf"\b{re.escape(member)}\b", code)) <= 1:
            dead.append((field, parameter))
    return dead


def load_tree():
    """Load mappings from the ROS node and production FusionCore sources."""
    node_source = NODE.read_text()
    sources = []
    for directory in CORE_DIRS:
        for path in directory.rglob("*"):
            if path.suffix in {".cpp", ".hpp", ".h"}:
                sources.append(path.read_text())
    return config_mappings(node_source), "\n".join(sources)


def main():
    mappings, core_source = load_tree()
    if not mappings:
        print(f"{NODE}: found no config/get_parameter mappings, the regex is wrong")
        return 1

    dead = dead_mappings(mappings, core_source)
    for field, parameter in dead:
        print(
            f"{parameter}: mapped to config.{field}, but {field.rsplit('.', 1)[-1]} "
            "is never read by fusioncore_core"
        )

    declared = declared_parameters(NODE.read_text())
    node_direct = len(declared) - len(mappings)
    print(
        f"{len(mappings)} of {len(declared)} declared parameters map into "
        f"FusionCoreConfig, {len(dead)} dead"
    )
    print(f"({node_direct} reach the node directly and are out of scope here)")
    return 1 if dead else 0


if __name__ == "__main__":
    sys.exit(main())
