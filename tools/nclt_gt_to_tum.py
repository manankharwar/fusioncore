#!/usr/bin/env python3
"""
Convert NCLT ground truth CSV to TUM trajectory format.

NCLT ground truth format (verified against groundtruth_2012-01-08.csv):
  utime, x, y, z, roll, pitch, yaw
  Local Cartesian frame — x, y, z in meters, angles in radians.
  First row often contains NaN (initialization artifact) — skipped.

Output: TUM format  timestamp tx ty tz qx qy qz qw

Usage:
  python3 tools/nclt_gt_to_tum.py \
    --gt  /path/to/groundtruth_2012-01-08.csv \
    --out ground_truth.tum
"""

import argparse
import csv
import math


def euler_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--gt',  required=True, help='NCLT groundtruth CSV')
    parser.add_argument('--out', required=True, help='Output TUM file')
    args = parser.parse_args()

    count = skipped = 0
    with open(args.gt) as fin, open(args.out, 'w') as fout:
        for row in csv.reader(fin):
            if not row or row[0].startswith('#'):
                continue
            try:
                utime = int(row[0])
                x     = float(row[1])
                y     = float(row[2])
                z     = float(row[3])
                roll  = float(row[4])
                pitch = float(row[5])
                yaw   = float(row[6])
            except (ValueError, IndexError):
                skipped += 1
                continue

            # Skip NaN rows (common at sequence start)
            if any(math.isnan(v) for v in [x, y, z, roll, pitch, yaw]):
                skipped += 1
                continue

            qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
            ts = utime / 1e6
            fout.write(f'{ts:.6f} {x:.6f} {y:.6f} {z:.6f} '
                       f'{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n')
            count += 1

    print(f'Written {count} poses to {args.out}  ({skipped} skipped due to NaN)')


if __name__ == '__main__':
    main()
