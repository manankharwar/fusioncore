#!/usr/bin/env python3
"""Translate a robot_localization ekf_node config into a FusionCore config.

WHY
The barrier to trying FusionCore is not the filter, it is the afternoon spent writing
a config. Whoever is evaluating already HAS a working robot_localization setup, and
that config encodes everything they learned about their robot: which sensors they
trust, which channels those sensors actually measure, their frames, their rates. This
reads that and emits the FusionCore equivalent, so trying it costs a command instead
of an afternoon.

    python3 rl_to_fusioncore.py my_robot_ekf.yaml > fusioncore.yaml
    python3 rl_to_fusioncore.py my_robot_ekf.yaml --navsat navsat.yaml

IT REFUSES TO GUESS. Anything that does not map cleanly is emitted as a commented
TODO with the reason, and a summary goes to stderr so it is visible even when stdout
is redirected to a file. A config that looks complete but quietly invented a number
is worse than one that says it does not know.

THE ONE THAT BITES EVERYONE
`imu0_remove_gravitational_acceleration` is INVERTED between the two projects.
robot_localization's means "the data has gravity, take it out". FusionCore's means
"the driver already took it out, put it back", because the filter's measurement model
expects specific force. Copying it across unchanged makes FusionCore add a second
9.8 m/s^2 to data that already carries it, and a constant acceleration error
double-integrates into position. This tool inverts it and says so.
"""
import argparse
import sys

import yaml

# robot_localization's 15 element sensor_config vector, in order.
IDX = ["x", "y", "z", "roll", "pitch", "yaw",
       "vx", "vy", "vz", "vroll", "vpitch", "vyaw",
       "ax", "ay", "az"]


def cfg_of(block, key):
    """robot_localization writes booleans, YAML sometimes reads them as strings."""
    v = block.get(key)
    if not v:
        return {}
    return {name: (str(flag).lower() == "true") for name, flag in zip(IDX, v)}


def find_params(doc):
    """ekf_node configs nest under <node_name>/ros__parameters, name varies."""
    if not isinstance(doc, dict):
        raise SystemExit("input is empty or not a YAML mapping: is this an ekf_node config?")
    for k, v in doc.items():
        if isinstance(v, dict) and "ros__parameters" in v:
            return k, v["ros__parameters"]
    if "ros__parameters" in doc:
        return "(root)", doc["ros__parameters"]
    raise SystemExit("no ros__parameters block found: is this an ekf_node config?")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ekf_yaml", help="robot_localization ekf_node config")
    ap.add_argument("--navsat", help="navsat_transform config, if you use GPS")
    ap.add_argument("--name", default="fusioncore", help="output node name")
    a = ap.parse_args()

    with open(a.ekf_yaml) as f:
        try:
            node_name, p = find_params(yaml.safe_load(f))
        except yaml.YAMLError as e:
            raise SystemExit(f"{a.ekf_yaml} is not valid YAML: {e}")

    notes, todos = [], []
    out = {}

    # ── frames and rate ──────────────────────────────────────────────────────
    out["base_frame"] = p.get("base_link_frame", "base_link")
    out["odom_frame"] = p.get("odom_frame", "odom")
    out["publish_rate"] = float(p.get("frequency", 30.0))
    if p.get("world_frame") == p.get("map_frame", "map"):
        todos.append(
            "world_frame was map, so robot_localization was running the MAP instance "
            "(the one fusing GPS). FusionCore fuses GNSS directly in one filter, so "
            "there is no second instance: point gnss.fix_topic at your /fix and drop "
            "navsat_transform entirely.")

    out["publish.force_2d"] = bool(p.get("two_d_mode", False))
    if not p.get("two_d_mode", False):
        notes.append("two_d_mode was false, so publish.force_2d is false. If this is "
                     "a ground robot, true usually helps.")

    # ── motion model, which robot_localization has no concept of ─────────────
    todos.append(
        "motion_model has NO robot_localization equivalent and changes behaviour. "
        "DifferentialDrive for skid steer or diff drive, Ackermann for car-like, "
        "Omnidirectional for mecanum or holonomic. Defaulted to DifferentialDrive.")
    out["motion_model"] = "DifferentialDrive"

    # ── IMU ──────────────────────────────────────────────────────────────────
    if p.get("imu0"):
        out["imu.topic"] = p["imu0"]
        c = cfg_of(p, "imu0_config")
        out["imu.has_magnetometer"] = bool(c.get("yaw", False))
        if c.get("yaw"):
            notes.append("imu0_config fused YAW, so imu.has_magnetometer is true: "
                         "FusionCore will treat the IMU quaternion as an ABSOLUTE "
                         "heading. Only correct if the IMU really has a magnetometer "
                         "and its yaw is referenced to magnetic north. If yaw is "
                         "relative to power-on, set this false.")
        rl_grav = bool(p.get("imu0_remove_gravitational_acceleration", False))
        out["imu.remove_gravitational_acceleration"] = not rl_grav
        notes.append(
            f"imu.remove_gravitational_acceleration INVERTED: robot_localization had "
            f"{str(rl_grav).lower()}, FusionCore needs {str(not rl_grav).lower()}. "
            f"The flags mean opposite things. Verify with the robot at rest: "
            f"linear_acceleration.z near 9.8 means false, near 0.0 means true.")
    else:
        todos.append("no imu0 found. FusionCore needs an IMU: set imu.topic.")

    # ── wheel odometry and any second twist source ───────────────────────────
    odoms = [k for k in p if k.startswith("odom") and k[4:].isdigit()]
    wheel_done = False
    for key in sorted(odoms):
        topic, c = p[key], cfg_of(p, key + "_config")
        pose = any(c.get(k) for k in ("x", "y", "z"))
        twist = any(c.get(k) for k in ("vx", "vy", "vyaw"))
        if pose and not twist:
            todos.append(
                f"{key} ({topic}) fused POSITION only. If that is navsat_transform "
                f"output, delete it and set gnss.fix_topic to your raw /fix instead. "
                f"If it is visual odometry or SLAM, set vslam.topic to it.")
        elif twist and not wheel_done:
            out["encoder.topic"] = topic
            wheel_done = True
            ch = [n for n in ("vx", "vy", "wz")
                  if c.get({"wz": "vyaw"}.get(n, n))]
            if ch != ["vx", "vy", "wz"]:
                notes.append(f"{key} measured only {ch}. FusionCore fuses all three "
                             f"by default; consider encoder.channels if that matters.")
        elif twist:
            out["encoder2.topic"] = topic
            out["encoder2.channels"] = [n for n in ("vx", "vy", "wz")
                                        if c.get({"wz": "vyaw"}.get(n, n))]
            notes.append(f"{key} became encoder2 (second twist source). Its channels "
                         f"were read from {key}_config: anything it does not actually "
                         f"MEASURE must be left out, or a channel that always reads "
                         f"zero argues with your gyro on every turn.")
    if not wheel_done:
        todos.append("no twist-bearing odometry found. If you have wheel encoders, "
                     "set encoder.topic.")

    # ── GNSS ─────────────────────────────────────────────────────────────────
    if a.navsat:
        with open(a.navsat) as f:
            _, n = find_params(yaml.safe_load(f))
        todos.append(
            "navsat_transform is NOT needed. FusionCore consumes sensor_msgs/NavSatFix "
            "directly in ECEF: set gnss.fix_topic to the raw fix topic that was feeding "
            "navsat_transform, and delete the navsat_transform node.")
        if n.get("magnetic_declination_radians"):
            out["magnetometer.declination_rad"] = float(n["magnetic_declination_radians"])
            notes.append("carried magnetic_declination_radians across from "
                         "navsat_transform.")
    out.setdefault("gnss.fix_topic", "")
    if not out["gnss.fix_topic"]:
        todos.append("gnss.fix_topic is empty. Set it to your NavSatFix topic, or "
                     "leave empty for a GPS-denied robot.")

    # ── noise: deliberately NOT translated ───────────────────────────────────
    if "process_noise_covariance" in p:
        todos.append(
            "process_noise_covariance was NOT translated. It is a 15x15 matrix over "
            "robot_localization's state, and FusionCore has 23 states in a different "
            "order, so there is no element-wise mapping. The shipped defaults are "
            "tuned; change them only against a measurement.")

    # ── emit ─────────────────────────────────────────────────────────────────
    print(f"# Generated from {a.ekf_yaml} (node '{node_name}') by rl_to_fusioncore.py")
    print("#")
    print("# NOT a finished config. Every TODO below is something this tool refused to")
    print("# guess at. Read them before running the robot.")
    print(f"{a.name}:")
    print("  ros__parameters:")
    for k, v in out.items():
        # safe_dump appends a "..." document-end marker on scalars; take the
        # first line only so the emitted file is still valid YAML.
        val = yaml.safe_dump(v, default_flow_style=True).split("\n")[0].strip()
        print(f"    {k}: {val}")
    if todos:
        print("\n    # ── TODO, this tool would not guess ──────────────────────────")
        for t in todos:
            for i, line in enumerate(_wrap(t)):
                print(f"    # {'TODO: ' if i == 0 else '      '}{line}")
    if notes:
        print("\n    # ── decisions this tool made for you ─────────────────────────")
        for nt in notes:
            for i, line in enumerate(_wrap(nt)):
                print(f"    # {'NOTE: ' if i == 0 else '      '}{line}")

    print(f"\n{len(todos)} TODO(s) and {len(notes)} note(s). Read them.",
          file=sys.stderr)
    for t in todos:
        print(f"  TODO: {t}", file=sys.stderr)
    return 0


def _wrap(text, width=70):
    import textwrap
    return textwrap.wrap(text, width) or [""]


if __name__ == "__main__":
    sys.exit(main())
