#!/usr/bin/env python3
"""Turn a FusionCore node YAML into key=value arguments for hardware/replay.cpp.

    python3 tools/yaml_to_replay_args.py config.yaml [--drop prefix ...]

Exists because an A/B through the offline replay needs the SAME configuration the
node would have used, and typing 70 parameters by hand is how a sweep ends up
measuring something other than what it claims to. replay.cpp refuses to run on an
unrecognised key, so anything this emits that the tool cannot honour is loud.

Only numbers and booleans are emitted. String parameters are reported on stderr
rather than dropped silently, because a silently dropped key is the failure mode
this repo keeps relearning.

--drop removes keys by exact name, for the arm of an A/B that wants the library
default instead of the config's value.
"""
import argparse
import sys

import yaml


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config")
    ap.add_argument("--drop", nargs="*", default=[],
                    help="parameter names to leave out entirely")
    args = ap.parse_args(argv)

    doc = yaml.safe_load(open(args.config))
    params = None
    for value in doc.values():
        if isinstance(value, dict) and "ros__parameters" in value:
            params = value["ros__parameters"]
            break
    if params is None:
        sys.exit(f"{args.config}: no ros__parameters block found")

    out, skipped = [], []

    def walk(prefix, d):
        for key, value in d.items():
            name = f"{prefix}{key}"
            if isinstance(value, dict):
                walk(name + ".", value)
            elif name in args.drop:
                continue
            elif isinstance(value, bool):
                out.append(f"{name}={1 if value else 0}")
            elif isinstance(value, (int, float)):
                out.append(f"{name}={value}")
            else:
                skipped.append(name)

    walk("", params)
    print(" ".join(out))
    if skipped:
        print("not numeric, not emitted: " + " ".join(skipped), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
