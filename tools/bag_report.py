#!/usr/bin/env python3
"""Read a rosbag and report why an estimator drifts, without installing anything.

The barrier to someone evaluating FusionCore was never the filter. It is the
afternoon spent cloning, matching a ROS distro, building, and writing a config
before they learn anything at all. That filters FOR people who enjoy building
software and AGAINST the people who actually own a drifting robot.

This inverts it. They send a bag, they get a report. Nothing to install on their
side, no config, no ROS version to match, and the findings are about THEIR robot
rather than about a benchmark dataset.

Every check here is computable from raw sensor topics with NO ground truth, which
is what makes it work on a bag from a stranger:

  * inventory and achieved rates against nominal
  * clock skew per topic, and stamp offsets BETWEEN sensors
  * IMU against wheel yaw-rate sign agreement, the issue #169 class of bug
  * wheel against GNSS distance scale, the issue #169 magnitude class
  * stationary gyro bias, measured while the robot is still
  * GNSS declared sigma against how far consecutive fixes actually move
  * whether the bag contains a manoeuvre that makes heading and gyro bias
    observable at all (see docs/observability.md)

Topics are matched BY MESSAGE TYPE, not by name, because someone else's topics are
named whatever they are named.

THE RULE FOR EVERY FINDING IN THIS FILE, learned the hard way on 2026-10-06:

    A disagreement between your reader and someone's data is YOUR READER'S BUG
    until you have checked both sides.

The IMU and wheel encoder in this project's own NCLT harness disagreed in sign on
100% of turns. The first description of that was "a bug in a dataset many papers rely
on", and it was wrong: the raw files agreed perfectly, and the conversion was applied
to one sensor and not the other by the READER. Reporting it to the dataset maintainers
would have been telling them their data was broken when it was not.

So findings here describe the PIPELINE, never the data. "Your IMU and your wheels
disagree" is a statement about a robot's configuration. "Your data is wrong" is a
claim about someone else's work, and it needs both sides checked before it is made.

    python3 tools/bag_report.py /path/to/bag                  # markdown to stdout
    python3 tools/bag_report.py /path/to/bag --out report.md
    python3 tools/bag_report.py /path/to/bag --json           # machine-readable

The analysis is a pure function of plain data (analyse()), so it is testable without
a bag and without ROS. Only read_bag() needs a sourced workspace.
"""

import argparse
import json
import math
import os
import statistics
import sys

IMU_TYPE = "sensor_msgs/msg/Imu"
ODOM_TYPE = "nav_msgs/msg/Odometry"
NAVSAT_TYPE = "sensor_msgs/msg/NavSatFix"
TWIST_TYPES = ("geometry_msgs/msg/TwistStamped",
               "geometry_msgs/msg/TwistWithCovarianceStamped")
JOINT_TYPE = "sensor_msgs/msg/JointState"

# Joint-name fragments that identify a left or a right wheel. Matched on the NAME
# rather than the index, because index order is a convention and names are a contract.
LEFT_HINTS  = ("left", "_l_", "_l0", "port")
RIGHT_HINTS = ("right", "_r_", "_r0", "starboard")

# Both sources must agree the robot is turning before their signs mean anything.
TURNING_RAD_S = 0.08
# Below this the platform is treated as stationary, for bias and ZUPT questions.
STILL_SPEED = 0.02
STILL_RATE = 0.02

SEVERITY_ORDER = {"BLOCKER": 0, "WARNING": 1, "INFO": 2, "OK": 3}


def finding(severity, title, detail, fix=None):
    return {"severity": severity, "title": title, "detail": detail, "fix": fix}


# ---------------------------------------------------------------------------
# pure analysis
# ---------------------------------------------------------------------------

def _rate(stamps):
    """Achieved Hz from stamps, median-based so one gap does not dominate."""
    if len(stamps) < 3:
        return None, None
    gaps = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    if not gaps:
        return None, None
    med = statistics.median(gaps)
    span = stamps[-1] - stamps[0]
    return (1.0 / med if med > 0 else None,
            (len(stamps) - 1) / span if span > 0 else None)


def check_inventory(series):
    out = []
    if not series.get("imu"):
        out.append(finding("BLOCKER", "No IMU messages",
                           "Found no sensor_msgs/msg/Imu in the bag.",
                           "FusionCore propagates on the IMU. Without one there is "
                           "nothing to fuse into."))
    if not series.get("odom") and not series.get("twist"):
        out.append(finding("WARNING", "No wheel odometry",
                           "Found no nav_msgs/msg/Odometry or TwistStamped.",
                           "Encoders are what bound drift while GNSS is out. Without "
                           "them a blackout is pure inertial coast."))
    if not series.get("gnss"):
        out.append(finding("WARNING", "No GNSS",
                           "Found no sensor_msgs/msg/NavSatFix.",
                           "Without an absolute position source, position error grows "
                           "without bound and heading is never observable."))

    for name, nominal in (("imu", 100.0), ("odom", 20.0), ("gnss", 5.0)):
        s = series.get(name) or []
        if len(s) < 3:
            continue
        stamps = [m["t"] for m in s]
        med_hz, mean_hz = _rate(stamps)
        if med_hz is None:
            continue
        detail = (f"{len(s)} messages, median {med_hz:.1f} Hz, "
                  f"mean {mean_hz:.1f} Hz over {stamps[-1] - stamps[0]:.1f} s.")
        # A mean far below the median means gaps: dropped frames or a starved recorder.
        if mean_hz and med_hz and mean_hz < 0.8 * med_hz:
            out.append(finding("WARNING", f"{name} has gaps",
                               detail + " Mean well below median means dropped runs "
                               "of messages rather than a uniformly slow stream.",
                               "Check recorder CPU and USB bandwidth. A gap is "
                               "indistinguishable from the sensor being absent."))
        else:
            out.append(finding("INFO", f"{name} rate", detail))
    return out


def check_clock_skew(series):
    """Header stamps against bag receive time, and sensors against each other.

    A sensor whose stamps run ahead of another's makes the filter clock ride the
    leading one, and every message from the lagging sensor then looks stale. That
    path currently REJECTS rather than corrects, so the data is simply discarded.
    """
    out = []
    offsets, spreads = {}, {}
    for name in ("imu", "odom", "gnss"):
        s = series.get(name) or []
        d = [m["recv"] - m["t"] for m in s if m.get("recv") is not None]
        if len(d) < 3:
            continue
        offsets[name] = statistics.median(d)
        spreads[name] = max(d) - min(d)

    # A header-to-arrival gap of more than a day is not clock skew. It means the
    # stamps and the arrival times are on different clocks entirely: a bag replayed
    # under /clock with sim time, or a dataset whose stamps carry the original
    # capture epoch. Reporting "stamps lag arrival by 451469497014 ms" as skew would
    # be worse than saying nothing, and the cross-sensor comparison below rests on
    # the same quantity so it is not trustworthy either.
    IMPLAUSIBLE_S = 86400.0
    if offsets and max(abs(v) for v in offsets.values()) > IMPLAUSIBLE_S:
        worst = max(offsets, key=lambda k: abs(offsets[k]))
        out.append(finding(
            "INFO", "Clock skew not checked: stamps are on a different epoch",
            f"Header stamps differ from bag arrival time by "
            f"{offsets[worst] / 86400.0:.0f} days on `{worst}`.",
            "This is normal for a bag replayed under simulated time, or for a public "
            "dataset that keeps its original capture timestamps. Clock skew can only "
            "be measured on a bag recorded live from the sensors."))
        return out

    for name, off in sorted(offsets.items()):
        if abs(off) > 0.2:
            out.append(finding("WARNING", f"{name} stamps lag arrival by "
                               f"{off * 1000:.0f} ms",
                               f"Median header-to-arrival delay {off:.3f} s, "
                               f"spread {spreads[name]:.3f} s.",
                               "If this exceeds max_measurement_delay the update is "
                               "dropped as DELAY_TOO_LARGE and the filter coasts."))
    names = sorted(offsets)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            gap = offsets[a] - offsets[b]
            if abs(gap) > 0.05:
                out.append(finding(
                    "WARNING", f"{a} and {b} clocks differ by {gap * 1000:.0f} ms",
                    f"Median stamp offsets: {a} {offsets[a]:.3f} s, "
                    f"{b} {offsets[b]:.3f} s.",
                    "Two sensors on different clocks fuse measurements taken at "
                    "different instants as if simultaneous. At 1 m/s, 100 ms is "
                    "10 cm of position error injected on every update."))
    if offsets and not any(f["severity"] == "WARNING" for f in out):
        out.append(finding("OK", "Clocks are consistent",
                           "All sensor stamp offsets agree within 50 ms."))
    return out


def wheel_wz_proxy(msg_names, velocities):
    """A SIGN-CORRECT, scale-free stand-in for yaw rate from wheel joint velocities.

    JointState gives per-wheel angular velocity in rad/s. Turning it into a real yaw
    rate needs wheel radii and track width:

        wz = (w_right * r_right - w_left * r_left) / track_width

    Those are calibration values a bag does not carry, and guessing them is exactly
    how a 1.297x scale error gets introduced. So this does NOT return a yaw rate.

    It returns (w_right_mean - w_left_mean), which has the SAME SIGN as the true yaw
    rate for any positive radii and track width. That is enough for the one check
    that matters most here, whether two rotation sources agree about which way the
    robot turned, and it needs no calibration at all. The magnitude is in
    rad/s-of-wheel-difference and is NOT comparable to a gyro, so the scale check is
    skipped rather than computed from an invented geometry.
    """
    left, right = [], []
    for name, v in zip(msg_names, velocities):
        low = name.lower()
        if any(h in low for h in LEFT_HINTS):
            left.append(v)
        elif any(h in low for h in RIGHT_HINTS):
            right.append(v)
    if not left or not right:
        return None
    return sum(right) / len(right) - sum(left) / len(left)


def check_yaw_rate_signs(series):
    """The issue #169 class: two rotation sources disagreeing about which way.

    Found in this project's own NCLT harness, where the IMU was converted to ENU and
    the wheel odometry was not, so the encoder fed the opposite sign for the whole of
    every published run. It never crashed, because the gyro has tighter noise so the
    filter leaned on it and spent gain rejecting the encoder on every turn.
    """
    imu = series.get("imu") or []
    odom = series.get("odom") or []
    if len(imu) < 50 or len(odom) < 50:
        return []
    pairs = []
    j = 0
    for o in odom:
        while j + 1 < len(imu) and abs(imu[j + 1]["t"] - o["t"]) < abs(imu[j]["t"] - o["t"]):
            j += 1
        if abs(imu[j]["t"] - o["t"]) <= 0.05:
            pairs.append((o.get("wz"), imu[j].get("wz")))
    both = [(a, b) for a, b in pairs
            if a is not None and b is not None
            and abs(a) > TURNING_RAD_S and abs(b) > TURNING_RAD_S]
    if len(both) < 100:
        return [finding("INFO", "Not enough turning to check yaw-rate signs",
                        f"Only {len(both)} samples had both sources above "
                        f"{TURNING_RAD_S} rad/s.",
                        "Drive a figure-eight so both sources see real rotation.")]
    dis = sum(1 for a, b in both if (a > 0) != (b > 0)) / len(both)
    ratio = statistics.median(abs(a) / abs(b) for a, b in both if abs(b) > 1e-6)
    proxy = bool(series.get("odom_is_wheel_proxy"))
    out = []
    if dis > 0.8:
        out.append(finding(
            "BLOCKER", f"IMU and wheel yaw rates disagree in sign on "
            f"{100 * dis:.0f}% of turns",
            f"{len(both)} samples where both exceeded {TURNING_RAD_S} rad/s. "
            f"Median magnitude ratio wheel/IMU {ratio:.3f}.",
            "Something in the pipeline has a frame convention wrong: a driver, a "
            "URDF, or a conversion applied to one sensor and not the other. Check "
            "BOTH sides before concluding which. Under REP-103, turning the robot "
            "LEFT by hand must give a POSITIVE angular_velocity.z, so that is the "
            "test. A BNO085 in UART-RVC mode reports yaw increasing CLOCKWISE and "
            "needs its sign flipped in the driver.\n"
            "  This exact finding appeared in FusionCore's own benchmark harness and "
            "the first diagnosis blamed the dataset. It was the reader: the NED to "
            "ENU conversion was applied to the IMU and not to the wheel odometry. "
            "The raw files agreed perfectly.\n"
            "  Until it is fixed, every turn costs the filter gain to reject one of "
            "its two rotation sources. It will not crash, which is why it survives."))
    else:
        out.append(finding("OK", "IMU and wheel yaw rates agree in sign",
                           f"{100 * (1 - dis):.0f}% agreement over {len(both)} "
                           f"turning samples."))
    # Magnitude is a separate question from sign, and the sign fix does not close it.
    # It is only askable when the wheel source is a real yaw rate. A JointState proxy
    # is in rad/s-of-wheel-difference, so a ratio against a gyro is meaningless and
    # saying so beats printing a number nobody can act on.
    if proxy:
        out.append(finding(
            "INFO", "Wheel yaw-rate SCALE not checked: no calibration in the bag",
            "The wheel source is sensor_msgs/JointState, which gives per-wheel "
            "angular velocity. Converting that to a yaw rate needs wheel radii and "
            "track width, which a bag does not carry.",
            "The SIGN check above is still valid and needs no calibration, because "
            "sign(right - left) gives turn direction for any positive geometry. To "
            "get the scale too, supply the wheel radii and track width."))
    elif ratio > 1.15 or ratio < 0.87:
        out.append(finding(
            "WARNING", f"Wheel yaw rate is {ratio:.2f}x the gyro's",
            f"Median |wheel| / |IMU| over {len(both)} turning samples is {ratio:.3f}.",
            "A scale error, not a frame convention: wrong track width or "
            "ticks-per-rev. The encoder over-reports rotation under slip, so heading "
            "integrated from wheels alone drifts proportionally. Calibrate the track "
            "width before tuning any noise parameter."))
    return out


def check_stationary_bias(series):
    imu = series.get("imu") or []
    odom = series.get("odom") or []
    if len(imu) < 100:
        return []
    if odom and all(o.get("v") is None for o in odom):
        return [finding(
            "INFO", "Stationary gyro bias not checked: forward speed is unknown",
            "The wheel source gives per-wheel angular velocity, and forward speed "
            "needs the wheel radius, which a bag does not carry. Without speed there "
            "is no reliable way to say when the platform was still.",
            "Supply the wheel radius, or record a nav_msgs/Odometry with a twist.")]
    # The LONGEST CONTIGUOUS still interval, not min-to-max of every still sample.
    # Taking min and max spans the whole bag whenever the robot happens to be still
    # at both the start and the end, which made the first version of this average
    # the gyro over all 155,466 samples of a moving run and call it a bias.
    still = [o["t"] for o in odom
             if o.get("v") is not None and o.get("wz") is not None
             and abs(o["v"]) < STILL_SPEED and abs(o["wz"]) < STILL_RATE]
    best, run = (None, None), None
    if still:
        start = prev = still[0]
        for ts in still[1:]:
            if ts - prev > 0.5:              # a half-second hole ends the interval
                if run is None or prev - start > run:
                    best, run = (start, prev), prev - start
                start = ts
            prev = ts
        if run is None or prev - start > run:
            best, run = (start, prev), prev - start
    if best[0] is None or (run or 0.0) < 2.0:
        return [finding("INFO", "No stationary period found",
                        "No contiguous interval of 2 s or more with the platform "
                        "still (under "
                        f"{STILL_SPEED} m/s and {STILL_RATE} rad/s).",
                        "Stop for 10 s or more with the wheels still. ZUPT then lets "
                        "the accel and gyro biases settle instead of integrating "
                        "into drift, and it is the only cheap bias observation.")]
    lo, hi = best
    gz = [m["wz"] for m in imu if m.get("wz") is not None and lo <= m["t"] <= hi]
    if len(gz) < 50:
        return []
    bias = statistics.mean(gz)
    noise = statistics.pstdev(gz) if len(gz) > 1 else 0.0
    out = [finding("INFO", f"Stationary gyro-z bias {bias:+.5f} rad/s",
                   f"Mean over {len(gz)} samples in the longest still interval "
                   f"({hi - lo:.1f} s), 1-sigma {noise:.5f}.")]
    if abs(bias) > 0.01:
        out.append(finding(
            "WARNING", f"Gyro-z bias is large at {bias:+.5f} rad/s",
            f"That is {math.degrees(abs(bias)):.2f} deg/s, so "
            f"{math.degrees(abs(bias)) * 60:.1f} degrees of heading per minute if "
            f"nothing corrects it.",
            "FusionCore estimates this as B_GZ, but a gyro alone cannot separate a "
            "rate from a bias on that rate. See docs/observability.md: it needs a "
            "figure-eight, and reversing the turn is the part that matters."))
    return out


def check_gnss_quality(series):
    g = series.get("gnss") or []
    if len(g) < 10:
        return []
    out = []
    declared = [math.sqrt(m["var_xy"]) for m in g if m.get("var_xy")]
    moves = []
    for a, b in zip(g, g[1:]):
        dt = b["t"] - a["t"]
        if not (0 < dt < 2.0):
            continue
        if a.get("lat") is None or b.get("lat") is None:
            continue
        dn = (b["lat"] - a["lat"]) * 111320.0
        de = (b["lon"] - a["lon"]) * 111320.0 * math.cos(math.radians(a["lat"]))
        moves.append(math.hypot(dn, de))
    if declared:
        med_declared = statistics.median(declared)
        out.append(finding("INFO", f"GNSS declares {med_declared:.2f} m sigma",
                           f"Median of {len(declared)} reported covariances."))
        if moves:
            p95 = sorted(moves)[int(0.95 * (len(moves) - 1))]
            if p95 > 3.0 * med_declared:
                out.append(finding(
                    "WARNING", "GNSS jitter exceeds its own declared sigma",
                    f"p95 consecutive-fix movement {p95:.2f} m against a declared "
                    f"sigma of {med_declared:.2f} m.",
                    "A receiver that understates its noise makes a chi-squared gate "
                    "calibrated to the stated value reject valid fixes. Enable "
                    "adaptive.gnss so the noise model is estimated at runtime "
                    "instead of trusted."))
    gaps = [b["t"] - a["t"] for a, b in zip(g, g[1:]) if b["t"] > a["t"]]
    if gaps:
        longest = max(gaps)
        if longest > 30.0:
            out.append(finding(
                "WARNING", f"Longest GNSS blackout {longest:.0f} s",
                f"{sum(1 for x in gaps if x > 5.0)} gaps over 5 s.",
                "Through a blackout position is dead-reckoned and heading has no "
                "absolute reference, so error grows quadratically with the outage. "
                "This is the one case where a simpler 2D filter can beat a full 3D "
                "one, and an absolute heading source is the only real fix."))
    return out


def check_observability(series):
    """Does the bag contain a manoeuvre that makes heading and gyro bias observable?

    This is the question docs/observability.md exists for. A bag of pure straight-line
    driving cannot separate the yaw rate from its bias no matter how long it is, so
    no amount of tuning will fix a heading problem in that data.
    """
    odom = series.get("odom") or []
    if len(odom) < 50:
        return []
    wz = [o["wz"] for o in odom if o.get("wz") is not None]
    v = [o["v"] for o in odom if o.get("v") is not None]
    if not wz:
        return []
    speed_unknown = not v
    left = sum(1 for w in wz if w > TURNING_RAD_S)
    right = sum(1 for w in wz if w < -TURNING_RAD_S)
    straight = sum(1 for o in odom
                   if o.get("v") and o["v"] > 0.2
                   and o.get("wz") is not None and abs(o["wz"]) < TURNING_RAD_S)
    out = []
    if speed_unknown:
        out.append(finding(
            "INFO", "Straight-line content not checked: forward speed is unknown",
            "Turn direction is available from the wheels, but 'driving straight' "
            "needs a speed, and that needs the wheel radius.",
            "The turn-direction and stop findings below rest only on yaw rate and "
            "remain valid."))
    elif straight < 0.05 * len(odom):
        out.append(finding(
            "WARNING", "Almost no straight-line driving",
            f"{straight} of {len(odom)} samples were above 0.2 m/s and turning less "
            f"than {TURNING_RAD_S} rad/s.",
            "GNSS track heading is the bearing between two fixes, so it needs a "
            "straight run of 10 m or more to be usable. Without it heading is never "
            "observed and the lever arm stays inert."))
    if min(left, right) < 0.02 * len(odom):
        out.append(finding(
            "INFO", "The robot only turned one way",
            f"{left} samples turning left, {right} turning right.",
            "This limits what GNSS track heading can be cross-checked against. It "
            "does NOT affect the gyro bias: turning both ways does not separate a "
            "rate from a bias on that rate, because the gyro reads their sum at "
            "every instant whichever way you turn. Only a stop does that."))

    # The finding that actually matters for the bias, and the one the stationary
    # check above measures. A constant gyro bias and a constant yaw rate are in the
    # null space of the gyro's own measurement, so no trajectory separates them.
    # ZUPT is a different measurement, z = WZ with no bias term.
    if speed_unknown:
        out.append(finding(
            "INFO", "Cannot tell whether the robot ever stopped",
            "A stop needs forward speed below a threshold, and speed needs the wheel "
            "radius. Yaw rate alone cannot distinguish stopped from driving straight.",
            "This matters: the gyro bias is only observable while stopped or under an "
            "absolute heading, so whether this bag can constrain it is UNKNOWN rather "
            "than yes or no. Supply the wheel radius to settle it."))
        return out
    still = sum(1 for o in odom
                if o.get("v") is not None and o.get("wz") is not None
                and abs(o["v"]) < STILL_SPEED and abs(o["wz"]) < STILL_RATE)
    if still < 0.01 * len(odom):
        out.append(finding(
            "BLOCKER", "The robot never stopped, so the gyro bias is unobservable",
            f"Only {still} of {len(odom)} samples had the platform still.",
            "The gyro reports rate + bias, so any split of the two fits it equally "
            "and the pair slides at zero measurement cost. Measured at a correlation "
            "of -0.998. Stopping is the only thing that breaks it, because a "
            "zero-velocity update asserts the rate is zero with no bias term, which "
            "turns the gyro reading into a direct measurement of the bias. Measured: "
            "without a stop, yaw integrates at 85% of truth; after a two-second "
            "stop, 100%. No amount of tuning or driving substitutes for it."))
    else:
        out.append(finding("OK", "The robot stopped at least once",
                           f"{still} still samples: the gyro bias is observable in "
                           f"this data."))
    return out


# Names that mark an Odometry topic as a FILTER OUTPUT rather than a wheel sensor.
# Ordered by how strongly they imply it.
FILTER_OUTPUT_HINTS = ("fusion", "ekf", "ukf", "filtered", "odometry/filtered",
                       "rl_", "/rl/", "localization", "fused", "estimate")
WHEEL_HINTS = ("wheel", "encoder", "diff_drive", "diff_cont", "base_controller")


def pick_wheel_odometry(by_topic):
    """Choose which Odometry topic is the WHEEL sensor, and say so.

    A bag usually carries wheel odometry next to one or more filter outputs on the
    same message type. Merging them yields a stream describing no physical sensor,
    and silently picking one is worse, because every downstream finding then rests
    on a guess the reader cannot see. So: pick by an explicit heuristic and return
    a note naming the choice and the alternatives.
    """
    if not by_topic:
        return [], None
    names = sorted(by_topic)
    if len(names) == 1:
        return by_topic[names[0]], None

    def score(n):
        low = n.lower()
        s = 0
        if any(h in low for h in WHEEL_HINTS):
            s -= 10                                   # strongly a wheel sensor
        if any(h in low for h in FILTER_OUTPUT_HINTS):
            s += 10                                   # strongly an output
        if low in ("/odom", "odom"):
            s -= 5                                    # the conventional wheel topic
        return (s, n)

    ranked = sorted(names, key=score)
    chosen = ranked[0]
    note = finding(
        "WARNING", f"{len(names)} odometry topics: used `{chosen}` as the wheel sensor",
        "Found " + ", ".join(f"`{n}`" for n in names) +
        ". The others look like filter outputs, which are not a sensor and would "
        "make every rate and sign finding below meaningless if merged in.",
        f"If `{chosen}` is not your wheel odometry, the sign and scale findings do "
        f"not apply. Re-record with only the raw sensor topics, or say which topic "
        f"is the encoder.")
    return by_topic[chosen], note


CHECKS = (check_inventory, check_clock_skew, check_yaw_rate_signs,
          check_stationary_bias, check_gnss_quality, check_observability)


def analyse(series):
    findings = list(series.get("notes") or [])
    for fn in CHECKS:
        try:
            findings += fn(series) or []
        except Exception as e:                       # one bad check must not kill the report
            findings.append(finding("INFO", f"{fn.__name__} could not run", str(e)))
    findings.sort(key=lambda f: SEVERITY_ORDER.get(f["severity"], 9))
    return findings


def render(findings, bag_name, duration=None):
    icons = {"BLOCKER": "BLOCKER", "WARNING": "WARNING", "INFO": "info", "OK": "ok"}
    counts = {}
    for f in findings:
        counts[f["severity"]] = counts.get(f["severity"], 0) + 1
    lines = [f"# Estimator report: `{bag_name}`", ""]
    if duration:
        lines.append(f"{duration:.0f} s of data. "
                     + ", ".join(f"{v} {k.lower()}" for k, v in sorted(
                         counts.items(), key=lambda kv: SEVERITY_ORDER.get(kv[0], 9))))
    else:
        lines.append(", ".join(f"{v} {k.lower()}" for k, v in sorted(
            counts.items(), key=lambda kv: SEVERITY_ORDER.get(kv[0], 9))))
    lines.append("")
    blockers = [f for f in findings if f["severity"] == "BLOCKER"]
    if blockers:
        lines += ["## Fix these first", "",
                  "Each of these makes a class of estimation problem unfixable by "
                  "tuning, so they come before any parameter change.", ""]
        for f in blockers:
            lines += [f"### {f['title']}", "", f["detail"], ""]
            if f["fix"]:
                lines += [f"**What to do:** {f['fix']}", ""]
    rest = [f for f in findings if f["severity"] != "BLOCKER"]
    if rest:
        lines += ["## Everything else", ""]
        for f in rest:
            lines.append(f"- **[{icons.get(f['severity'], '?')}] {f['title']}** "
                         f"{f['detail']}")
            if f["fix"]:
                lines.append(f"  - {f['fix']}")
        lines.append("")
    lines += ["---", "",
              "Generated by `tools/bag_report.py` from FusionCore. Every check above "
              "is computed from raw sensor topics with no ground truth, so none of it "
              "depends on a reference trajectory you do not have.", ""]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# bag reading, the only part that needs ROS
# ---------------------------------------------------------------------------

def storage_id(bag):
    """rosbag2 needs telling mcap vs sqlite3. Same fallback as nis_from_bag.py:
    a recorder that was killed rather than interrupted leaves no usable metadata."""
    import yaml
    meta = os.path.join(bag, "metadata.yaml")
    try:
        with open(meta) as fh:
            info = yaml.safe_load(fh)["rosbag2_bagfile_information"]
        if info.get("storage_identifier"):
            return info["storage_identifier"]
    except Exception:
        pass
    for entry in sorted(os.listdir(bag)) if os.path.isdir(bag) else []:
        if entry.endswith(".db3"):
            return "sqlite3"
    return "mcap"


def read_bag(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id(bag)),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    # By TYPE, not by name: someone else's topics are named whatever.
    wanted = {n: ty for n, ty in types.items()
              if ty in (IMU_TYPE, ODOM_TYPE, NAVSAT_TYPE, JOINT_TYPE)
              or ty in TWIST_TYPES}
    if not wanted:
        sys.exit(f"no IMU, odometry or NavSatFix topics in {bag}. "
                 f"Found: {sorted(set(types.values()))}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))

    series = {"imu": [], "odom": [], "gnss": [], "twist": [], "joint": [],
              "odom_by_topic": {}, "topics": wanted, "notes": [],
              "odom_is_wheel_proxy": False}
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        ty = wanted[topic]
        msg = deserialize_message(data, get_message(ty))
        recv = recv_ns / 1e9
        st = getattr(msg, "header", None)
        t = (st.stamp.sec + st.stamp.nanosec / 1e9) if st else recv
        if t <= 0:
            t = recv
        if ty == IMU_TYPE:
            series["imu"].append({"t": t, "recv": recv,
                                  "wz": msg.angular_velocity.z,
                                  "az": msg.linear_acceleration.z})
        elif ty == ODOM_TYPE:
            tw = msg.twist.twist
            # Keep odometry topics SEPARATE. A bag commonly carries wheel odometry
            # alongside one or more filter OUTPUTS on the same message type, and
            # merging them produces a stream that describes no physical sensor.
            series["odom_by_topic"].setdefault(topic, []).append(
                {"t": t, "recv": recv, "wz": tw.angular.z,
                 "v": math.hypot(tw.linear.x, tw.linear.y),
                 "child": getattr(msg, "child_frame_id", "")})
        elif ty in TWIST_TYPES:
            tw = msg.twist.twist if hasattr(msg.twist, "twist") else msg.twist
            series["twist"].append({"t": t, "recv": recv, "wz": tw.angular.z,
                                    "v": math.hypot(tw.linear.x, tw.linear.y)})
        elif ty == JOINT_TYPE:
            wz = wheel_wz_proxy(list(msg.name), list(msg.velocity))
            if wz is not None:
                # v is UNKNOWN, not zero. Forward speed from JointState needs the
                # wheel radius, which a bag does not carry. Setting it to 0.0 made
                # this report state that the robot never moved, that the whole 101 s
                # run was a stationary interval, and that the gyro bias was therefore
                # observable. Three confident falsehoods from one stub.
                series["joint"].append({"t": t, "recv": recv, "wz": wz, "v": None})
        elif ty == NAVSAT_TYPE:
            c = list(msg.position_covariance)
            series["gnss"].append({"t": t, "recv": recv,
                                   "lat": msg.latitude, "lon": msg.longitude,
                                   "var_xy": (c[0] + c[4]) / 2.0 if len(c) >= 5 else None,
                                   "status": int(msg.status.status)})
    series["odom"], note = pick_wheel_odometry(series["odom_by_topic"])
    if note:
        series["notes"].append(note)
    if not series["odom"] and series["twist"]:
        series["odom"] = series["twist"]      # a TwistStamped is wheel odometry too
    if not series["odom"] and series["joint"]:
        # Last resort: a JointState wheel proxy. Sign-correct, scale-free, and the
        # report says so rather than letting a reader assume a yaw rate.
        series["odom"] = series["joint"]
        series["odom_is_wheel_proxy"] = True
        series["notes"].append(finding(
            "INFO", "Wheel data came from sensor_msgs/JointState",
            f"{len(series['joint'])} messages, using "
            f"(right - left) wheel angular velocity as a sign-correct stand-in for "
            f"yaw rate.",
            "Sign and motion-content checks are valid. Anything about MAGNITUDE is "
            "not, because converting per-wheel rad/s to a yaw rate needs wheel radii "
            "and track width that a bag does not carry."))
    return series


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--out", help="write markdown here instead of stdout")
    ap.add_argument("--json", action="store_true", help="emit findings as JSON")
    args = ap.parse_args()

    series = read_bag(args.bag)
    findings = analyse(series)
    all_t = [m["t"] for k in ("imu", "odom", "gnss") for m in series.get(k) or []]
    duration = (max(all_t) - min(all_t)) if all_t else None

    if args.json:
        print(json.dumps({"bag": os.path.basename(args.bag.rstrip("/")),
                          "duration_s": duration, "findings": findings}, indent=2))
        return 0
    text = render(findings, os.path.basename(args.bag.rstrip("/")), duration)
    if args.out:
        with open(args.out, "w") as fh:
            fh.write(text)
        print(f"wrote {args.out}")
    else:
        print(text)
    return 1 if any(f["severity"] == "BLOCKER" for f in findings) else 0


if __name__ == "__main__":
    sys.exit(main())
