#!/usr/bin/env python3
"""Fit wheel radius and track width from a bag plus a ground-truth trajectory.

A bag that publishes sensor_msgs/JointState gives per-wheel ANGULAR velocity in
rad/s. Turning that into the (vx, wz) a state estimator consumes needs geometry:

    vx = (w_left * r + w_right * r) / 2
    wz = (w_right * r - w_left * r) / track_width

Those numbers live in a calibration file that a bag does not carry, and guessing
them is how a scale error gets introduced. Measured on NCLT, a wheel odometry that
over-reported rotation by 29.7% cost nothing visible and quietly taxed every turn.

This fits them instead, from data, which is what the KAIST authors did for their own
dataset using FOG and VRS-GPS. Two stages, each a one-parameter linear least squares,
which is more robust than a joint nonlinear fit and makes the residuals readable:

    stage 1   r  from forward speed, using (w_left + w_right)
    stage 2   wb from yaw rate, using (w_right - w_left) with r fixed

WHY A WINDOW AND NOT A DERIVATIVE. The ground truth is RTK at centimetre accuracy,
sampled around 100 Hz. Differentiating consecutive samples turns 1 cm of position
noise into 1 m/s of velocity noise, which swamps the signal. Velocities are taken
over a window (default 0.5 s), which brings that to about 0.02 m/s.

    python3 tools/fit_wheel_geometry.py <ros2_bag> --gt <gt.tum>
    python3 tools/fit_wheel_geometry.py <ros2_bag> --gt <gt.tum> --json

Ground truth is TUM: timestamp x y z qx qy qz qw, which is what tools/evaluate.py
already consumes.

It reports residuals and an R-squared per stage, and REFUSES to report a fit it does
not believe, because a confident wrong radius is worse than no radius.
"""

import argparse
import json
import math
import os
import sys

LEFT_HINTS = ("left", "_l_", "_l0", "port")
RIGHT_HINTS = ("right", "_r_", "_r0", "starboard")
JOINT_TYPE = "sensor_msgs/msg/JointState"

# Below this R-squared the model does not describe the data and the fit is refused.
MIN_R2 = 0.90
# A window shorter than this cannot beat RTK position noise when differentiated.
MIN_WINDOW_S = 0.2
# Slopes fitted on the low and high halves of the regressor must agree within this
# fraction. See slope_consistency() for why R-squared alone is not enough.
MAX_SLOPE_DISAGREEMENT = 0.10
# Below this coefficient of variation the regressor is too flat to split, so the
# proportionality assumption cannot be tested and is reported as untested.
MIN_REGRESSOR_CV = 0.05


def read_gt(path):
    """TUM ground truth into (t, x, y, yaw)."""
    out = []
    with open(path) as fh:
        for line in fh:
            f = line.split()
            if len(f) < 8:
                continue
            try:
                t, x, y = float(f[0]), float(f[1]), float(f[2])
                qx, qy, qz, qw = float(f[4]), float(f[5]), float(f[6]), float(f[7])
            except ValueError:
                continue
            yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
            out.append((t, x, y, yaw))
    out.sort()
    return out


def gt_velocities(gt, window_s):
    """Forward speed and yaw rate over a window, not between adjacent samples."""
    if len(gt) < 10:
        return []
    # index step that spans roughly window_s
    span = gt[-1][0] - gt[0][0]
    rate = (len(gt) - 1) / span if span > 0 else 0.0
    k = max(2, int(round(window_s * rate)))
    out = []
    for i in range(len(gt) - k):
        t0, x0, y0, a0 = gt[i]
        t1, x1, y1, a1 = gt[i + k]
        dt = t1 - t0
        if dt <= 0:
            continue
        # forward speed along the heading at the midpoint of the window
        mid = gt[i + k // 2][3]
        dx, dy = x1 - x0, y1 - y0
        vx = (dx * math.cos(mid) + dy * math.sin(mid)) / dt
        da = a1 - a0
        while da > math.pi:
            da -= 2 * math.pi
        while da < -math.pi:
            da += 2 * math.pi
        out.append(((t0 + t1) * 0.5, vx, da / dt))
    return out


def read_joints(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import yaml

    def storage_id(b):
        meta = os.path.join(b, "metadata.yaml")
        try:
            with open(meta) as fh:
                info = yaml.safe_load(fh)["rosbag2_bagfile_information"]
            if info.get("storage_identifier"):
                return info["storage_identifier"]
        except Exception:
            pass
        for e in sorted(os.listdir(b)) if os.path.isdir(b) else []:
            if e.endswith(".db3"):
                return "sqlite3"
        return "mcap"

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id(bag)),
                rosbag2_py.ConverterOptions("", ""))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = [n for n, ty in topics.items() if ty == JOINT_TYPE]
    if not want:
        sys.exit(f"no {JOINT_TYPE} in {bag}. Found: {sorted(set(topics.values()))}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    out = []
    msgtype = get_message(JOINT_TYPE)
    while reader.has_next():
        _, data, recv = reader.read_next()
        m = deserialize_message(data, msgtype)
        st = getattr(m, "header", None)
        t = (st.stamp.sec + st.stamp.nanosec / 1e9) if st else recv / 1e9
        if t <= 0:
            t = recv / 1e9
        left, right = [], []
        for name, v in zip(m.name, m.velocity):
            low = name.lower()
            if any(h in low for h in LEFT_HINTS):
                left.append(v)
            elif any(h in low for h in RIGHT_HINTS):
                right.append(v)
        if left and right:
            out.append((t, sum(left) / len(left), sum(right) / len(right)))
    out.sort()
    return out


def pair(joints, vels, max_dt=0.05):
    """Nearest-time join. Both are ~100 Hz so a 50 ms tolerance is generous."""
    import bisect
    jt = [j[0] for j in joints]
    out = []
    for t, vx, wz in vels:
        i = bisect.bisect_left(jt, t)
        best = None
        for k in (i - 1, i):
            if 0 <= k < len(jt) and (best is None or abs(jt[k] - t) < abs(jt[best] - t)):
                best = k
        if best is not None and abs(jt[best] - t) <= max_dt:
            out.append((vx, wz, joints[best][1], joints[best][2]))
    return out


def lstsq_through_origin(xs, ys):
    """Fit y = m*x with no intercept, and report R-squared about zero.

    No intercept on purpose: zero wheel rotation must mean zero speed. An intercept
    would absorb a bias that physically cannot exist and flatter the fit.
    """
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    if sxx <= 0:
        return None, 0.0, 0.0
    m = sxy / sxx
    ss_res = sum((y - m * x) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum(y * y for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rms = math.sqrt(ss_res / len(ys)) if ys else 0.0
    return m, r2, rms


def slope_consistency(xs, ys):
    """Fit the low and high halves of x separately and compare the slopes.

    R-squared about zero cannot tell a proportional relationship from one where both
    variables merely happen to be near-constant: a through-origin fit then just
    matches the ratio of the means, and reports a high R-squared for any data at all.
    Found by a test feeding wheel velocities unrelated to the motion, which still
    fitted at R-squared above 0.9 because both sides sat near a constant.

    A genuinely proportional relationship has the SAME slope at small and large x.
    A coincidence of means does not. That is the thing worth testing.

    Returns (disagreement_fraction, cv) where cv is the regressor's coefficient of
    variation. When cv is small the data is too flat to split and disagreement comes
    back None, which is "untested", not "passed".
    """
    n = len(xs)
    if n < 40:
        return None, 0.0
    mean = sum(xs) / n
    var = sum((x - mean) ** 2 for x in xs) / n
    cv = (math.sqrt(var) / abs(mean)) if abs(mean) > 1e-12 else float("inf")
    if cv < MIN_REGRESSOR_CV:
        return None, cv
    order = sorted(range(n), key=lambda i: xs[i])
    half = n // 2
    lo = order[:half]
    hi = order[half:]
    m_lo, _, _ = lstsq_through_origin([xs[i] for i in lo], [ys[i] for i in lo])
    m_hi, _, _ = lstsq_through_origin([xs[i] for i in hi], [ys[i] for i in hi])
    if not m_lo or not m_hi:
        return None, cv
    scale = max(abs(m_lo), abs(m_hi))
    return (abs(m_hi - m_lo) / scale if scale > 0 else None), cv


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--gt", required=True, help="ground truth in TUM format")
    ap.add_argument("--window", type=float, default=0.5,
                    help="seconds over which ground-truth velocity is taken "
                         "(default 0.5; shorter cannot beat RTK position noise)")
    ap.add_argument("--min-speed", type=float, default=0.5,
                    help="m/s below which a sample is dropped from the radius fit")
    ap.add_argument("--min-rate", type=float, default=0.05,
                    help="rad/s below which a sample is dropped from the track fit")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    if args.window < MIN_WINDOW_S:
        sys.exit(f"--window {args.window} is below {MIN_WINDOW_S} s. Differentiating "
                 f"centimetre-accurate RTK over a shorter span produces more noise "
                 f"than signal.")

    gt = read_gt(args.gt)
    if len(gt) < 100:
        sys.exit(f"only {len(gt)} usable ground-truth rows in {args.gt}")
    vels = gt_velocities(gt, args.window)
    joints = read_joints(args.bag)
    rows = pair(joints, vels)
    if len(rows) < 200:
        sys.exit(f"only {len(rows)} paired samples; not enough to fit anything")

    # stage 1: radius from forward speed.  vx = r * (wl + wr) / 2
    s1 = [(0.5 * (wl + wr), vx) for vx, wz, wl, wr in rows if abs(vx) >= args.min_speed]
    xs1 = [a for a, _ in s1]
    ys1 = [b for _, b in s1]
    r, r2_r, rms_r = lstsq_through_origin(xs1, ys1)
    dis_r, cv_r = slope_consistency(xs1, ys1)

    out = {"bag": os.path.basename(args.bag.rstrip("/")), "gt": os.path.basename(args.gt),
           "paired_samples": len(rows), "window_s": args.window,
           "radius_m": r, "radius_r2": r2_r, "radius_rms_mps": rms_r,
           "radius_samples": len(s1),
           "radius_slope_disagreement": dis_r, "radius_regressor_cv": cv_r}

    if r is None or r2_r < MIN_R2:
        out["verdict"] = "REFUSED"
        out["reason"] = (f"radius fit R^2 {r2_r:.4f} is below {MIN_R2}. The model "
                         f"vx = r*(wl+wr)/2 does not describe this data, so the "
                         f"number is not trustworthy and is not reported as one.")
        print(json.dumps(out, indent=2) if args.json else _render(out))
        return 1

    if dis_r is not None and dis_r > MAX_SLOPE_DISAGREEMENT:
        out["verdict"] = "REFUSED"
        out["reason"] = (f"the radius fitted on the slow half of the data disagrees "
                         f"with the fast half by {100*dis_r:.1f}%, above the "
                         f"{100*MAX_SLOPE_DISAGREEMENT:.0f}% limit. A real wheel "
                         f"radius is the same at every speed, so this does not "
                         f"describe this data however good R^2 looks. R^2 about zero "
                         f"cannot tell proportionality from two near-constants.")
        print(json.dumps(out, indent=2) if args.json else _render(out))
        return 1

    if dis_r is None:
        out["radius_proportionality"] = (
            f"UNTESTED: regressor coefficient of variation {cv_r:.4f} is below "
            f"{MIN_REGRESSOR_CV}, so the data is too flat to split and the "
            f"through-origin assumption could not be checked. The radius is only "
            f"identifiable if that assumption holds.")

    # stage 2: track width from yaw rate, with r fixed.  wz = r*(wr - wl)/wb
    s2 = [(r * (wr - wl), wz) for vx, wz, wl, wr in rows if abs(wz) >= args.min_rate]
    inv_wb, r2_w, rms_w = lstsq_through_origin([a for a, _ in s2], [b for _, b in s2])
    out["track_samples"] = len(s2)
    out["track_r2"] = r2_w
    out["track_rms_radps"] = rms_w
    out["track_width_m"] = (1.0 / inv_wb) if inv_wb else None

    if inv_wb is None or r2_w < MIN_R2:
        out["verdict"] = "PARTIAL"
        out["reason"] = (f"radius fitted, but the track-width fit R^2 {r2_w:.4f} is "
                         f"below {MIN_R2}. Usually too little turning in the "
                         f"sequence: {len(s2)} samples above {args.min_rate} rad/s.")
    else:
        out["verdict"] = "OK"
        out["diameter_m"] = 2.0 * r
    print(json.dumps(out, indent=2) if args.json else _render(out))
    return 0


def _render(o):
    L = [f"# Wheel geometry fit: {o['bag']} against {o['gt']}", "",
         f"{o['paired_samples']} paired samples, ground-truth velocity over a "
         f"{o['window_s']} s window.", ""]
    L.append(f"  wheel radius     {o['radius_m']:.5f} m"
             if o.get("radius_m") else "  wheel radius     FAILED")
    if o.get("radius_m"):
        L.append(f"  wheel diameter   {2*o['radius_m']:.5f} m")
    L.append(f"    R^2 {o.get('radius_r2', 0):.4f} over {o.get('radius_samples', 0)}"
             f" samples, residual RMS {o.get('radius_rms_mps', 0):.4f} m/s")
    if o.get("radius_proportionality"):
        L.append(f"    {o['radius_proportionality']}")
    elif o.get("radius_slope_disagreement") is not None:
        L.append(f"    slow-half vs fast-half slope disagreement "
                 f"{100*o['radius_slope_disagreement']:.2f}%")
    if o.get("track_width_m"):
        L.append("")
        L.append(f"  track width      {o['track_width_m']:.5f} m")
        L.append(f"    R^2 {o.get('track_r2', 0):.4f} over {o.get('track_samples', 0)}"
                 f" samples, residual RMS {o.get('track_rms_radps', 0):.4f} rad/s")
    L += ["", f"verdict: {o.get('verdict')}"]
    if o.get("reason"):
        L.append(f"  {o['reason']}")
    return "\n".join(L)


if __name__ == "__main__":
    sys.exit(main())
