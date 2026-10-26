#!/usr/bin/env python3
"""Is the output-rate jitter a heading wobble amplified by distance?

    python3 tools/jitter_heading_corr.py <attrib.csv> <exported.txt>

Hypothesis under test (2026-10-23): inside a long GNSS gap every small heading change is
back-propagated into a sideways position kick of about (distance since the last fix) x
(heading change), and the heading changes are driven by the gyro-bias estimate swinging.

Reads `hardware/replay.cpp attrib=` output (needs the yaw and b_gz columns). Inside the
longest gap of the raw fix stream, per event:

  lateral kick   the update's position change, perpendicular to the filter's heading
  lever          distance travelled by the PREDICT steps since the gap began
  dpsi           heading change across the event (yaw after this event - yaw after the last)

Prediction: |kick| and lever*|dpsi| correlate tightly and linearly, and b_gz changes lead
dpsi. Reports Pearson r for both, the fitted slope, and the lagged cross-correlation.
"""
import sys
import numpy as np


def main():
    attrib, exported = sys.argv[1], sys.argv[2]
    d = np.genfromtxt(attrib, delimiter=",", names=True, dtype=None, encoding=None)
    if "yaw" not in d.dtype.names:
        sys.exit("attrib file has no yaw column; rebuild replay")
    g = np.array(sorted(float(l.split()[1]) for l in open(exported) if l.startswith("GNSS")))
    i = int(np.argmax(np.diff(g))); g0, g1 = g[i], g[i + 1]
    t = d["t"].astype(float); m = (t > g0) & (t < g1)
    pdx, pdy = d["pred_dx"][m], d["pred_dy"][m]
    udx, udy = d["upd_dx"][m], d["upd_dy"][m]
    yaw = np.unwrap(d["yaw"][m].astype(float)); bgz = d["b_gz"][m].astype(float)

    lever = np.cumsum(np.hypot(pdx, pdy))
    lat = -udx * np.sin(yaw) + udy * np.cos(yaw)          # sideways component of the kick
    dpsi = np.diff(yaw, prepend=yaw[0])
    x = lever * np.abs(dpsi); y = np.abs(lat)
    r_abs = np.corrcoef(x, y)[0, 1]
    r_signed = np.corrcoef(lever * dpsi, lat)[0, 1]
    slope = np.polyfit(x, y, 1)[0] if x.std() > 0 else float("nan")
    print(f"  gap {g1 - g0:.1f} s, {m.sum()} events, lever reaches {lever[-1]:.1f} m")
    print(f"  |lateral kick| vs lever*|dpsi|:  r = {r_abs:+.3f}   slope {slope:.3f} (1.0 = the hypothesis)")
    print(f"  signed lateral kick vs lever*dpsi: r = {r_signed:+.3f}")
    print(f"  |lateral kick| vs lever alone:    r = {np.corrcoef(lever, y)[0, 1]:+.3f}")
    print(f"  |lateral kick| vs |dpsi| alone:   r = {np.corrcoef(np.abs(dpsi), y)[0, 1]:+.3f}")

    # Does the bias lead heading? Resample to 1 s and cross-correlate the increments.
    tt = t[m]; grid = np.arange(tt[0], tt[-1], 1.0)
    b1 = np.interp(grid, tt, bgz); p1 = np.interp(grid, tt, yaw)
    db, dp = np.diff(b1), np.diff(p1)
    print("  corr(d b_gz at t, d heading at t+lag), 1 s grid:")
    for lag in (-10, -5, -2, 0, 2, 5, 10, 20):
        a, b = (db[:len(db) - lag], dp[lag:]) if lag >= 0 else (db[-lag:], dp[:lag])
        print(f"    lag {lag:+3d} s   r = {np.corrcoef(a, b)[0, 1]:+.3f}")


if __name__ == "__main__":
    main()
