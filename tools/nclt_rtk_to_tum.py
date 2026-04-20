#!/usr/bin/env python3
"""
Convert NCLT gps_rtk.csv to TUM trajectory format.

Uses the first RTK fix as local ENU origin. Positions are projected
to local ENU (east-north-up) relative to that origin. Orientation
is set to identity (RTK has no orientation data).

Format: utime, mode, numSV, lat_rad, lon_rad, alt_m, [vel_n, vel_e]
Only mode=3 (3D fix) rows are used.

Usage:
  python3 tools/nclt_rtk_to_tum.py \
    --rtk /path/to/2012-01-08/gps_rtk.csv \
    --out ground_truth.tum
"""

import argparse
import csv
import math


def lla_to_ecef(lat_r, lon_r, alt_m):
    a = 6378137.0
    e2 = 6.6943799901377997e-3
    N = a / math.sqrt(1 - e2 * math.sin(lat_r) ** 2)
    x = (N + alt_m) * math.cos(lat_r) * math.cos(lon_r)
    y = (N + alt_m) * math.cos(lat_r) * math.sin(lon_r)
    z = (N * (1 - e2) + alt_m) * math.sin(lat_r)
    return x, y, z


def ecef_to_enu(px, py, pz, ref_lat_r, ref_lon_r, ref_x, ref_y, ref_z):
    dx, dy, dz = px - ref_x, py - ref_y, pz - ref_z
    sin_lat, cos_lat = math.sin(ref_lat_r), math.cos(ref_lat_r)
    sin_lon, cos_lon = math.sin(ref_lon_r), math.cos(ref_lon_r)
    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return e, n, u


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--rtk', required=True, help='NCLT gps_rtk.csv')
    parser.add_argument('--out', required=True, help='Output TUM file')
    args = parser.parse_args()

    rows = []
    with open(args.rtk) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('#'):
                continue
            try:
                utime = int(row[0])
                mode  = int(float(row[1]))
                lat_r = float(row[3])
                lon_r = float(row[4])
                alt_m = float(row[5])
            except (ValueError, IndexError):
                continue
            if mode < 3:
                continue
            if any(math.isnan(v) for v in [lat_r, lon_r, alt_m]):
                continue
            rows.append((utime, lat_r, lon_r, alt_m))

    if not rows:
        print('No valid RTK fixes found')
        return

    # Use first fix as ENU origin
    ref_lat, ref_lon, ref_alt = rows[0][1], rows[0][2], rows[0][3]
    ref_x, ref_y, ref_z = lla_to_ecef(ref_lat, ref_lon, ref_alt)

    count = 0
    with open(args.out, 'w') as f:
        for utime, lat_r, lon_r, alt_m in rows:
            px, py, pz = lla_to_ecef(lat_r, lon_r, alt_m)
            e, n, u = ecef_to_enu(px, py, pz, ref_lat, ref_lon, ref_x, ref_y, ref_z)
            ts = utime / 1e6
            # Identity quaternion: RTK has no orientation
            f.write(f'{ts:.6f} {e:.6f} {n:.6f} {u:.6f} 0.000000 0.000000 0.000000 1.000000\n')
            count += 1

    print(f'Written {count} poses to {args.out}  (origin: {math.degrees(ref_lat):.6f}°N, {math.degrees(ref_lon):.6f}°E)')


if __name__ == '__main__':
    main()
