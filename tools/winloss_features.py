#!/usr/bin/env python3
"""One row per NCLT sequence: the measured verdict beside candidate explanatory columns.

    python3 tools/winloss_features.py

WHY THIS EXISTS. FusionCore wins 4 of the 9 sequences measured on corrected input and
loses 5, and nothing is known about what separates them. Baseline accuracy is about 7 m
on the wins and 23 to 46 m on the losses. No field or platform claim is supportable
until that is explained, because you cannot say which conditions you are good in.

Blackout length was the obvious candidate and it is NOT the answer: FusionCore wins with
a 266 s gap and loses with an 86 s one. So this looks at the motion and GNSS-quality
features instead, and prints an explicit separation check rather than leaving it to the
eye, because a table of 9 rows and 15 columns invites finding a pattern that is not
there.

RESULT, 2026-10-17: **no column separates the wins from the losses.** Every range
overlaps. The nine sequences are nearly interchangeable by these measures, which makes
sense once stated: NCLT is one Segway driving one campus across 15 months. So the split
is not a regime that these statistics can see, and it cannot be used to pick a vertical.

Two consequences, both in the repo record:
  - The field question is not answerable from NCLT. It needs bags from other robots.
  - The remaining hypothesis is filter-internal rather than conditional. FusionCore is
    2.5x to 7.6x worse in the MEDIAN on the losses, not only in RMSE, so it is a
    sustained bad state and not only excursions. That points at #150.

Motion comes from odometry_mu_100hz.csv because that is what nclt_player.py actually
feeds the filter. Using wheels.csv would describe a signal the filter never saw.

Note on the stop columns: stop% counts every sample under the velocity threshold, while
nstops and stop_total_s count only contiguous stops of at least MIN_STOP_S, which is
what bias observability actually needs. The gap between them is sub-second dips. Real
stops total 59 to 183 s out of 55 to 94 minutes on every sequence, so none of these runs
gives the filter much chance to observe bias while stationary, wins and losses alike.
"""
import math, os, statistics, sys

NCLT = "/home/manankharwar/nclt"

# Measured on corrected input. FC and RL are ATE RMSE 3D in metres.
RES = {
 "2012-03-31": ("WIN",   28.526, 160.574, 1),
 "2012-09-28": ("WIN",    8.872,  91.870, 2),
 "2012-10-28": ("WIN",   17.061,  25.072, 1),
 "2013-04-05": ("WIN",    9.193,  65.840, 2),
 "2012-01-08": ("loss", 185.195,  22.792, 1),
 "2012-02-04": ("loss", 147.008, 123.134, 3),
 "2012-05-11": ("loss",   9.606,   8.936, 1),
 "2012-06-15": ("loss",  32.149,  45.840, 2),
 "2012-08-20": ("loss", 157.591,  10.393, 2),
}

STOP_V      = 0.05    # m/s, below this the vehicle is stopped
STRAIGHT_W  = 0.05    # rad/s, below this it is driving straight
MIN_STOP_S  = 2.0     # a stop shorter than this does not help bias observability


def motion(seq):
    path = os.path.join(NCLT, seq, "odometry_mu_100hz.csv")
    if not os.path.isfile(path):
        return None
    t_prev = x_prev = y_prev = yaw_prev = None
    dur = 0.0
    stop_t = 0.0
    straight_t = 0.0
    move_t = 0.0
    rot = 0.0
    path_len = 0.0
    omegas = []
    stops = []          # durations of contiguous stopped stretches
    cur_stop = 0.0
    with open(path) as f:
        for line in f:
            p = line.split(",")
            if len(p) < 7:
                continue
            try:
                t = int(p[0]) / 1e6
                x, y, yaw = float(p[1]), float(p[2]), float(p[6])
            except ValueError:
                continue
            if t_prev is not None:
                dt = t - t_prev
                if 0 < dt < 1.0:                       # skip gaps and duplicates
                    d = math.hypot(x - x_prev, y - y_prev)
                    dyaw = yaw - yaw_prev
                    while dyaw >  math.pi: dyaw -= 2 * math.pi
                    while dyaw < -math.pi: dyaw += 2 * math.pi
                    v = d / dt
                    w = dyaw / dt
                    dur += dt
                    path_len += d
                    rot += abs(dyaw)
                    if v < STOP_V:
                        stop_t += dt
                        cur_stop += dt
                    else:
                        if cur_stop >= MIN_STOP_S:
                            stops.append(cur_stop)
                        cur_stop = 0.0
                        move_t += dt
                        omegas.append(abs(w))
                        if abs(w) < STRAIGHT_W:
                            straight_t += dt
            t_prev, x_prev, y_prev, yaw_prev = t, x, y, yaw
    if cur_stop >= MIN_STOP_S:
        stops.append(cur_stop)
    if dur == 0:
        return None
    return dict(
        dur_min=dur / 60.0,
        path_m=path_len,
        stop_pct=100.0 * stop_t / dur,
        n_stops=len(stops),
        stop_total_s=sum(stops),
        longest_stop_s=max(stops) if stops else 0.0,
        straight_pct=100.0 * straight_t / move_t if move_t else 0.0,
        rot_deg=math.degrees(rot),
        rot_deg_per_m=math.degrees(rot) / path_len if path_len else 0.0,
        med_w=statistics.median(omegas) if omegas else 0.0,
    )


def gnss(seq):
    path = os.path.join(NCLT, seq, "gps.csv")
    if not os.path.isfile(path):
        return None
    ts, modes = [], []
    with open(path) as f:
        for line in f:
            p = line.split(",")
            if len(p) < 3:
                continue
            try:
                t = float(p[0]) / 1e6
                m = int(float(p[1]))
            except ValueError:
                continue
            ts.append(t); modes.append(m)
    if not ts:
        return None
    order = sorted(range(len(ts)), key=lambda i: ts[i])
    ts = [ts[i] for i in order]; modes = [modes[i] for i in order]
    usable = [t for t, m in zip(ts, modes) if m >= 2]
    gaps = [b - a for a, b in zip(usable, usable[1:])] if len(usable) > 1 else [0.0]
    flips = sum(1 for a, b in zip(modes, modes[1:]) if a != b)
    span = ts[-1] - ts[0]
    mix = {}
    for m in modes:
        mix[m] = mix.get(m, 0) + 1
    return dict(
        n_fix=len(ts),
        pct_usable=100.0 * len(usable) / len(ts),
        max_gap=max(gaps),
        n_gap60=sum(1 for g in gaps if g > 60),
        flips_per_min=flips / (span / 60.0) if span else 0.0,
        mode_mix=",".join(f"{k}:{100*v//len(modes)}%" for k, v in sorted(mix.items())),
    )


rows = []
for seq in sorted(RES):
    m, g = motion(seq), gnss(seq)
    if not m or not g:
        print(f"{seq}: missing data", file=sys.stderr); continue
    rows.append((seq, RES[seq], m, g))

print()
print("MOTION FEATURES, from the odometry the player actually feeds the filter")
print("%-12s %-5s %8s %8s | %6s %7s %6s %7s | %7s %8s %7s" % (
    "sequence","verd","FC","RL","min","path_m","stop%","nstops","strght%","rot_deg","med_w"))
for seq,(v,fc,rl,n),m,g in sorted(rows, key=lambda r: (r[1][0] != "WIN", r[0])):
    print("%-12s %-5s %8.1f %8.1f | %6.0f %7.0f %6.1f %7d | %7.1f %8.0f %7.3f" % (
        seq, v, fc, rl, m["dur_min"], m["path_m"], m["stop_pct"], m["n_stops"],
        m["straight_pct"], m["rot_deg"], m["med_w"]))

print()
print("GNSS QUALITY")
print("%-12s %-5s | %7s %8s %8s %7s %9s | %s" % (
    "sequence","verd","nfix","usable%","maxgap_s","gaps>60","flips/min","mode mix"))
for seq,(v,fc,rl,n),m,g in sorted(rows, key=lambda r: (r[1][0] != "WIN", r[0])):
    print("%-12s %-5s | %7d %8.1f %8.0f %7d %9.2f | %s" % (
        seq, v, g["n_fix"], g["pct_usable"], g["max_gap"], g["n_gap60"],
        g["flips_per_min"], g["mode_mix"]))

print()
print("SEPARATION CHECK: does any column split the 4 wins from the 5 losses?")
cols = [("stop%", lambda m,g: m["stop_pct"]), ("nstops", lambda m,g: m["n_stops"]),
        ("stop_total_s", lambda m,g: m["stop_total_s"]),
        ("longest_stop_s", lambda m,g: m["longest_stop_s"]),
        ("straight%", lambda m,g: m["straight_pct"]), ("rot_deg", lambda m,g: m["rot_deg"]),
        ("rot_deg_per_m", lambda m,g: m["rot_deg_per_m"]),
        ("med_w", lambda m,g: m["med_w"]), ("path_m", lambda m,g: m["path_m"]),
        ("dur_min", lambda m,g: m["dur_min"]),
        ("max_gap", lambda m,g: g["max_gap"]), ("gaps>60", lambda m,g: g["n_gap60"]),
        ("usable%", lambda m,g: g["pct_usable"]), ("flips/min", lambda m,g: g["flips_per_min"]),
        ("n_fix", lambda m,g: g["n_fix"])]
print("%-16s %22s %22s   %s" % ("column","wins (min..max)","losses (min..max)","verdict"))
for name, fn in cols:
    w = sorted(fn(m,g) for s,(v,a,b,c),m,g in rows if v == "WIN")
    l = sorted(fn(m,g) for s,(v,a,b,c),m,g in rows if v != "WIN")
    sep = "SEPARATES" if (max(w) < min(l) or min(w) > max(l)) else ""
    print("%-16s %10.2f..%-10.2f %10.2f..%-10.2f   %s" % (name, w[0], w[-1], l[0], l[-1], sep))
