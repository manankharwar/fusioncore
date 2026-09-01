#!/usr/bin/env python3
"""Filter consistency from a recorded bag: is the covariance honest?

Every other check asks whether the estimate looks right. This one asks whether
the filter's own confidence matches the errors it actually makes, which needs no
ground truth and so works on any real run.

It reads the Normalized Innovation Squared that FusionCore already records for
every GNSS fix on /fusion/debug/gnss_status:

    NIS = nu^T S^-1 nu      nu = fix minus prediction,  S = H P H^T + R

For a filter whose covariance is honest, NIS averages the measurement dimension,
3 for a GNSS x/y/z position. Reading the result:

    NIS ~ 3     honest
    NIS >> 3    OVERCONFIDENT. S is too small, so the filter is more certain than
                it has earned and the gain is too low to correct its own error.
                The chi2 gate will also fire on good fixes.
    NIS << 3    UNDERCONFIDENT. S is too large. Safe from divergence, but the
                gain is too high, so the filter chases measurement noise, and the
                chi2 gate can no longer reject anything: if NIS never approaches
                the threshold, outlier rejection is inert.

Usage:
    python3 tools/nis_from_bag.py <bag_dir> [more bags ...]
"""
import sys
import os
import statistics
import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit("needs a sourced ROS 2 environment: source /opt/ros/<distro>/setup.bash "
             "and your workspace, so the fusioncore_ros messages resolve")

GNSS_TOPIC = "/fusion/debug/gnss_status"
NAVSAT_TYPE = "sensor_msgs/msg/NavSatFix"
HEALTH_TOPIC = "/fusion/debug/filter_health"
EXPECTED_NIS = 3.0          # GNSS position is a 3-DOF measurement


def storage_id(bag):
    """rosbag2 needs to be told mcap vs sqlite3, and the metadata knows.

    A recorder that was killed rather than interrupted leaves metadata.yaml
    missing or empty, so fall back on the file extension instead of dying.
    """
    meta = os.path.join(bag, "metadata.yaml")
    try:
        with open(meta) as fh:
            info = yaml.safe_load(fh)["rosbag2_bagfile_information"]
        ident = info.get("storage_identifier")
        if ident:
            return ident
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        pass
    for entry in sorted(os.listdir(bag)) if os.path.isdir(bag) else []:
        if entry.endswith(".db3"):
            return "sqlite3"
    return "mcap"


def read_navsat(bag):
    """Declared covariance vs how much consecutive fixes actually move.

    A receiver that smooths internally reports its ABSOLUTE accuracy (metres,
    dominated by multipath and ionosphere) while delivering fixes that agree with
    each other to centimetres. The filter is handed the declared number as R, but
    its innovations only ever see the short-term consistency, so NIS collapses.
    Worth measuring, because it is invisible from the estimate alone.
    """
    import math
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id(bag)),
                rosbag2_py.ConverterOptions("", ""))
    topics = [t.name for t in reader.get_all_topics_and_types() if t.type == NAVSAT_TYPE]
    if not topics:
        return None
    topic = topics[0]
    msg_type = get_message(NAVSAT_TYPE)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    lat, lon, sig = [], [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, msg_type)
        if m.position_covariance_type == 0:
            continue
        lat.append(m.latitude)
        lon.append(m.longitude)
        sig.append(math.sqrt(max(m.position_covariance[0], 0.0)))
    if len(lat) < 10:
        return None

    earth = 6371000.0
    xs = [math.radians(v - lon[0]) * earth * math.cos(math.radians(lat[0])) for v in lon]
    ys = [math.radians(v - lat[0]) * earth for v in lat]
    # Second difference cancels constant velocity. What survives is measurement
    # noise plus the robot's real acceleration, so this BOUNDS the noise above.
    d2 = sorted(math.hypot(xs[i+1] - 2*xs[i] + xs[i-1], ys[i+1] - 2*ys[i] + ys[i-1])
                for i in range(1, len(xs) - 1))
    declared = statistics.median(sig)
    observed = d2[len(d2) // 2]
    # White noise of size s gives median |2nd difference| of about sqrt(6)*1.1774*s
    expected = math.sqrt(6.0) * 1.1774 * declared
    return topic, declared, observed, expected


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id(bag)),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if GNSS_TOPIC not in types:
        return None, None
    wanted = [t for t in (GNSS_TOPIC, HEALTH_TOPIC) if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    msg_types = {t: get_message(types[t]) for t in wanted}

    gnss, health = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, msg_types[topic])
        (gnss if topic == GNSS_TOPIC else health).append(msg)
    return gnss, health


def pct(values, q):
    idx = min(int(q * (len(values) - 1)), len(values) - 1)
    return values[idx]


def report(bag):
    name = os.path.basename(os.path.normpath(bag))
    try:
        gnss, health = read(bag)
    except Exception as exc:                      # one unreadable bag must not
        print("  %s: could not be read (%s)" % (name, exc))   # stop the rest
        return
    if gnss is None:
        print("  %s: no %s recorded, nothing to measure" % (name, GNSS_TOPIC))
        return
    if not gnss:
        print("  %s: %s is empty" % (name, GNSS_TOPIC))
        return

    # mahalanobis_sq is -1 when a quality gate failed before the chi2 test ran,
    # so those fixes carry no NIS to report.
    nis = sorted(m.mahalanobis_sq for m in gnss if m.mahalanobis_sq >= 0.0)
    reasons = {}
    for m in gnss:
        reasons[m.rejection_reason] = reasons.get(m.rejection_reason, 0) + 1
    threshold = gnss[-1].chi2_threshold

    print("\n=== %s ===" % name)
    print("  fixes %d, accepted %d" % (len(gnss), sum(1 for m in gnss if m.accepted)))
    for reason, count in sorted(reasons.items(), key=lambda kv: -kv[1]):
        print("    %-18s %4d  (%.0f%%)" % (reason, count, 100.0 * count / len(gnss)))

    if not nis:
        print("  no NIS samples: every fix failed a quality gate before the chi2 test")
        return

    med = statistics.median(nis)
    print("  NIS median %.2f   mean %.2f   p90 %.2f   max %.2f   (honest is %.1f)"
          % (med, statistics.mean(nis), pct(nis, 0.90), nis[-1], EXPECTED_NIS))

    if med > 3.0 * EXPECTED_NIS:
        print("  OVERCONFIDENT by %.0fx: the filter trusts itself more than it has "
              "earned, so its gain is too low to correct its own error."
              % (med / EXPECTED_NIS))
    elif med < EXPECTED_NIS / 3.0:
        print("  UNDERCONFIDENT by %.0fx: covariance far larger than the errors "
              "warrant." % (EXPECTED_NIS / med))
        print("  Consequence: the gain is too high, so the filter tracks GNSS noise "
              "instead of smoothing it.")
        if nis[-1] < threshold:
            print("  Consequence: outlier rejection is INERT. The chi2 threshold is "
                  "%.2f and the largest NIS in this whole run was %.2f, so no fix "
                  "could ever have been rejected by it." % (threshold, nis[-1]))
    else:
        print("  Covariance is consistent with the errors being made.")

    if health:
        hdg = sorted(m.heading_sigma_deg for m in health)
        pos = sorted(m.position_sigma_x for m in health)
        print("  filter position 1-sigma: median %.2f m" % statistics.median(pos))
        print("  heading 1-sigma: median %.0f deg, sources %s"
              % (statistics.median(hdg), sorted({m.heading_source for m in health})))
        if statistics.median(hdg) > 45.0:
            print("  Heading is effectively unknown, which inflates the position "
                  "covariance and pushes NIS down.")

    nav = read_navsat(bag)
    if nav:
        topic, declared, observed, expected = nav
        print("  receiver on %s declares %.2f m 1-sigma" % (topic, declared))
        print("    fix-to-fix scatter implies far less: median second difference "
              "%.3f m against the %.1f m that declared figure would produce"
              % (observed, expected))
        if observed * 10.0 < expected:
            print("    Consecutive fixes are %.0fx smoother than the declared "
                  "covariance implies, so that number describes ABSOLUTE accuracy "
                  "(multipath, ionosphere) while the filter's innovations only see "
                  "short-term consistency." % (expected / max(observed, 1e-9)))
            print("    This alone drives NIS down and is not a filter bug. Do not "
                  "just shrink the covariance: the absolute error really is metres, "
                  "and a small R would make the filter track that bias rigidly and "
                  "report centimetre confidence it has not earned.")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for bag in sys.argv[1:]:
        report(bag)
    print()


if __name__ == "__main__":
    main()
