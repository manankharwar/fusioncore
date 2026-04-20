#!/usr/bin/env python3
"""
Extract a nav_msgs/Odometry topic from a ROS2 bag into TUM format.

Usage:
  python3 tools/odom_to_tum.py \
    --bag   ./nclt_results \
    --topic /fusion/odom \
    --out   fusioncore.tum

  python3 tools/odom_to_tum.py \
    --bag   ./nclt_results \
    --topic /rl/odometry \
    --out   rl_ekf.tum

TUM format: timestamp tx ty tz qx qy qz qw
"""

import argparse

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bag',   required=True, help='ROS2 bag directory')
    parser.add_argument('--topic', required=True, help='Odometry topic name')
    parser.add_argument('--out',   required=True, help='Output TUM file')
    args = parser.parse_args()

    reader = SequentialReader()
    import glob as _glob, os as _os
    # If directory, find the actual bag file (handles missing metadata.yaml)
    uri = args.bag
    if _os.path.isdir(args.bag):
        mcap_files = _glob.glob(f'{args.bag}/*.mcap')
        db3_files  = _glob.glob(f'{args.bag}/*.db3')
        if mcap_files:
            uri, storage_id = mcap_files[0], 'mcap'
        elif db3_files:
            uri, storage_id = db3_files[0], 'sqlite3'
        else:
            raise RuntimeError(f'No bag file found in {args.bag}')
    else:
        storage_id = 'mcap' if args.bag.endswith('.mcap') else 'sqlite3'
    reader.open(
        StorageOptions(uri=uri, storage_id=storage_id),
        ConverterOptions('', '')
    )

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in topics:
        available = ', '.join(topics.keys())
        raise RuntimeError(f'Topic {args.topic!r} not in bag. Available: {available}')

    count = 0
    with open(args.out, 'w') as f:
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            if topic != args.topic:
                continue
            msg = deserialize_message(data, Odometry)
            # Use header stamp (sim time) not bag recording time
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            p  = msg.pose.pose.position
            o  = msg.pose.pose.orientation
            f.write(f'{ts:.9f} '
                    f'{p.x:.6f} {p.y:.6f} {p.z:.6f} '
                    f'{o.x:.6f} {o.y:.6f} {o.z:.6f} {o.w:.6f}\n')
            count += 1

    print(f'Written {count} poses from {args.topic} to {args.out}')


if __name__ == '__main__':
    main()
