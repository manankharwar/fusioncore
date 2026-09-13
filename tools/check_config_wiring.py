#!/usr/bin/env python3
"""Check that ROS parameters mapped into FusionCoreConfig are actually read."""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
NODE = ROOT / "fusioncore_ros" / "src" / "fusion_node.cpp"
CORE_DIRS = (ROOT / "fusioncore_core" / "include", ROOT / "fusioncore_core" / "src")


def config_mappings(node_source):
    """Return (config field, ROS parameter) pairs assigned by fusion_node.cpp."""
    return re.findall(
        r'config\.((?:\w+\.)?\w+)\s*=\s*[^;]*get_parameter\("([\w.]+)"\)',
        node_source,
    )


def dead_mappings(mappings, core_source):
    """Return mappings whose config field is only declared, never read by core."""
    dead = []
    for field, parameter in mappings:
        member = field.rsplit(".", 1)[-1]
        if len(re.findall(rf"\b{re.escape(member)}\b", core_source)) <= 1:
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

    print(f"{len(mappings)} config mappings checked, {len(dead)} dead fields")
    return 1 if dead else 0


if __name__ == "__main__":
    sys.exit(main())
