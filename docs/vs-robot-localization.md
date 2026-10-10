# FusionCore vs robot_localization

A direct technical comparison based on 940 minutes of real robot data across twelve NCLT sequences.

Paper: [arXiv:2605.25239](https://arxiv.org/abs/2605.25239)

---

## The short version

| | robot_localization | FusionCore |
|---|---|---|
| GPS fusion | navsat_transform node, UTM projection | Native ECEF, no projection node |
| IMU bias | Not in state vector | Gyro + accel bias as filter states |
| Outlier rejection | Scalar Mahalanobis threshold | Chi-squared gate per sensor DOF |
| GPS noise estimation | Uses sensor-reported covariance as-is | Adapts from 50-sample innovation window |
| ZUPT | Not built-in | Auto when stationary |
| Delay compensation | `smooth_lagged_data` + `history_length` | IMU ring-buffer replay |
| GPS fix quality gating | Not built-in | HDOP, satellite count, fix type |
| Wheel encoder yaw bias | Not in state vector | 23rd state, estimated online |
| GPS velocity fusion | Not built-in | Yes (slip detection via Doppler) |
| VSLAM pose fusion | Not built-in | Yes: `vslam.topic` |
| Dual IMU | Not built-in | Yes: `imu2.topic` |
| RL-UKF on GPS sequences | Diverges (NaN) | Stable on all twelve sequences |
| ROS 2 native | Ported from ROS 1 | Written from scratch for ROS 2 |

---

## Why RL-EKF fails on GPS-heavy sequences

This is the central finding from the NCLT benchmark and it affects anyone using robot_localization with GPS, not just people evaluating FusionCore.

The NCLT GPS driver reports `var_xy = 9` (3m sigma), matching the Novatel SPAN-CPT open-sky specification. Measured against RTK ground truth, actual p95 error ranges from **9.7m to 53.1m** across sequences: 2 to 18 times the stated sigma.

robot_localization calibrates its chi-squared gate to the stated covariance. When actual GPS error reaches 40-200m, those fixes land far outside the expected window and get rejected. With GPS effectively disabled, RL-EKF reverts to wheel-encoder dead-reckoning. The two worst sequences show **31.84 m/km and 50.11 m/km drift**: diagnostic of open-loop operation across full runs.

This is not a bug in robot_localization. It is what happens when you trust the covariance your GPS driver reports. Most GPS drivers report optimistic covariance. The problem compounds in any real deployment.

FusionCore maintains a 50-sample innovation window per sensor and adapts the noise model in real time:

```
R ← (1 - α)R + α·Ĉ
```

where `Ĉ` is the empirical innovation covariance and `α = 0.01`. A floor prevents collapse. The chi-squared gate stays calibrated to actual error levels regardless of what the driver reports.

---

## Benchmark: 12 NCLT sequences, same config, no per-sequence tuning

| Sequence | Season | Duration | FC ATE | RL-EKF ATE | Winner |
|---|---|---|---|---|---|
| 2012-01-08 | Winter | 92 min | **18.6 m** | 41.2 m | FC +55% |
| 2012-02-04 | Winter | 77 min | **49.7 m** | 265.5 m | FC +81% |
| 2012-03-31 | Spring | 87 min | **22.0 m** | 156.5 m | FC +86% |
| 2012-05-11 | Spring | 84 min | **9.7 m** | 11.5 m | FC +16% |
| 2012-06-15 | Summer | 55 min | 49.2 m † | **18.2 m** | RL +63% |
| 2012-08-20 | Summer | 83 min | 98.3 m † | **10.6 m** | RL +89% |
| 2012-09-28 | Fall | 77 min | **22.4 m** | 53.8 m | FC +58% |
| 2012-10-28 | Fall | 85 min | **15.6 m** | 56.4 m | FC +72% |
| 2012-11-04 | Fall | 79 min | **60.1 m** | 122.0 m | FC +51% |
| 2012-12-01 | Winter | 75 min | **21.0 m** | 90.7 m | FC +77% |
| 2013-02-23 | Winter | 78 min | **59.4 m** | 82.2 m | FC +28% |
| 2013-04-05 | Spring | 68 min | **12.1 m** † | 268.9 m | FC +96% |

RL-UKF diverged with NaN on all twelve sequences. **The "FusionCore wins 10 of 12" claim that stood here is withdrawn:** of the five rows re-measured at n=2 after the #169 frame fix, FusionCore wins two and loses three, and the remaining seven have never been verified.

> **Status, 2026-10-10.** Five of these twelve rows have been re-measured at n=2 after
> fixing #169, a frame error in the dataset player that fed a wrong wheel-odometry yaw
> rate to **both** filters. On those five **FusionCore wins two and loses three**, and
> 2012-02-04 flipped from a published 81% win to a measured 20% loss. The other seven
> have not been re-run. Their May 2026 run records exist and match the published figures
> to three decimals, so the table was measured rather than invented, but it was measured
> through the same faulty player and nothing should be concluded from those rows in
> either direction.
>
> Measured, post-fix: 2013-04-05 **5.0x better** than robot_localization, 2012-09-28
> **4.7x better**, 2012-02-04 1.2x worse, 2012-06-15 1.8x worse, 2012-08-20 **15x worse**.
> The spread, not the average, is the open problem. Two identified defects sit inside
> every one of these numbers: #150 and #148. See `tools/benchmark_regression.md`.
>
> | Sequence | Published (May 2026) | Measured since | RL-EKF control, published vs measured |
> |---|---|---|---|
> | 2012-06-15 | 49.2 m | confirmed **73.3 m** on current `main` 2026-10-05 (73.5 m on `bcc0e09`) | 18.2 m vs 18.487 then 18.489 m |
> | 2012-08-20 | 98.3 m | **145.2 m** on current `main` 2026-10-05 (116.4 m on `bcc0e09`) | 10.6 m vs 10.519 then 10.507 m |
> | 2013-04-05 | 12.1 m | **189.7 m** (`bcc0e09`), then **50.2 m** (2026-10-03) | 268.9 m vs 266.7 to 268.8 m |
>
> The RL-EKF control held on the two rows re-measured on 2026-10-05, moving **+0.01%** and **-0.11%** against an established band of under 1%. That is what rules out the harness and places these moves in FusionCore's own column.
>
> **The 10-of-12 record still holds and nothing flipped**: 2012-06-15 and 2012-08-20 were already losses, and 2013-04-05 remains a large win. What changed is the magnitudes, and the two losses are now worse than published, RL-EKF winning by 75% and 93% rather than 63% and 89%.
>
> **2012-08-20 is a live regression, not only a stale figure.** 145.2 m against 116.4 m on `bcc0e09` and repeats of 116.97 and 122.32 is outside its 4.57% spread, and `tools/check_benchmark_regression.py` puts it at +18.3% on XY. The baseline has deliberately not been moved to match it.
>
> **CORRECTED 2026-10-09.** Five of these twelve rows have now been re-measured at n=2
> after fixing #169, a frame error in the NCLT player that fed a wrong wheel-odometry yaw
> rate to BOTH filters. **On those five, FusionCore wins two and loses three.** The other
> seven were measured in May 2026 through the faulty player and have never been re-run, so
> nothing should be claimed about them. The earlier note here, that the overstatement was
> systematic at about 1.48x, is withdrawn: across five rows the ratio runs 0.83x to 3.08x
> and the control swings 0.24x to 2.52x despite sharing no code. See the table and the
> explanation in the repository README.
>
> Harness and baseline: `tools/benchmark_regression.md` and `tools/benchmark_baseline.json`. Rows this note affects are marked † below.

Metric: ATE RMSE (meters), SE3-aligned to RTK ground truth using EVO. Same IMU, wheel odometry, and GPS inputs. Full methodology in the [benchmark reference](reference/benchmark.md).

---

## The two FusionCore losses

Both losses have identified root causes. They are documented here rather than hidden.

**2012-06-15 (FC 49.2m †, now measured 73.3m; RL 18.2m):** The dataset's GPS-sparsest sequence, with a 462-second blackout. During coast mode, residual wheel-encoder yaw bias (`b_ewz`) and gyro drift accumulate into quadratic position error over the multi-minute outage. RL-EKF's 2D mode has fewer divergence degrees of freedom. The lever for this is an absolute heading source during the outage: `magnetometer.enabled: true` bounds heading drift instead of letting it accumulate (demonstrated in a unit test with slipping wheel odometry). **Honesty caveat:** this cannot be validated against *this NCLT number*, the dataset publishes no usable magnetometer and its ground-truth orientation is too noisy to score a few-metre change. So the magnetometer is validated by construction and in test, and awaits real-hardware confirmation; it is not proven to close this specific loss.

**2012-08-20 (FC 98.3m †, now measured 145.2m; RL 10.6m):** 105 mode-3 GPS fixes located 720-840m from RTK ground truth appear in a 24-second window at a blackout boundary. Coast mode relaxes the chi-squared gate slightly to re-acquire GPS after the blackout; the adversarial cluster each individually pass the gate and collectively pull the estimate. RL-EKF incidentally rejects them through its miscalibrated gate (the same gate that causes its ten other losses). FusionCore now ships a physical-plausibility gate (`gnss.max_speed`) that rejects this cluster and cuts the peak spike. **Honesty caveat:** this does *not* fix the score. The sequence's ATE is dominated by dead-reckoning drift accumulated *during* the 211-second blackout, not by the cluster spike, so rejecting the cluster lowers the peak but not the overall number. The real lever, as with 2012-06-15, is an absolute heading source during the outage.

---

## RL issues FusionCore resolves

These are open robot_localization issues that describe problems FusionCore handles differently.

| robot_localization issue | What FusionCore does |
|---|---|
| UKF diverges with NaN on GPS sequences ([#780](https://github.com/cra-ros-pkg/robot_localization/issues/780), [#777](https://github.com/cra-ros-pkg/robot_localization/issues/777)) | Chi-squared gate on every sensor, covariance bounded at each step |
| navsat_transform crashes at UTM zone boundaries ([#951](https://github.com/cra-ros-pkg/robot_localization/issues/951), [#904](https://github.com/cra-ros-pkg/robot_localization/issues/904)) | GPS fused directly in ECEF, no UTM projection |
| No non-holonomic constraint ([#744](https://github.com/cra-ros-pkg/robot_localization/issues/744)) | Built-in NHC: lateral and vertical velocity zeroed as a virtual measurement |
| Delayed sensor messages cause missed updates ([#911](https://github.com/cra-ros-pkg/robot_localization/issues/911)) | IMU ring buffer with retrodiction up to 500ms |
| Non-deterministic output across bag replays ([#957](https://github.com/cra-ros-pkg/robot_localization/issues/957)) | Message timestamps drive everything under `use_sim_time: true` |
| IMU frame confusion ([#757](https://github.com/cra-ros-pkg/robot_localization/issues/757)) | TF lookup on every message, `imu.frame_id` override for broken drivers |

---

## See the difference in simulation

The Gazebo demo runs both filters simultaneously on the same sensor stream while the robot drives a lawnmower pattern, then injects two 60 m GPS spikes and a 25 s outage. No real hardware needed.

```bash
ros2 launch fusioncore_gazebo fusioncore_demo.launch.py
```

RViz shows green (FusionCore, which holds course through the spikes), red (robot_localization, which lurches to each spike), and yellow (raw GPS, including the spikes). See [Simulation](simulation.md#demo-fusioncore-vs-robot_localization-under-gps-spikes-and-an-outage) for details and the WSL2 transport note.

---

## Switching from robot_localization

See the [migration guide](migration_from_robot_localization.md) for a step-by-step walkthrough.

If localization is actively blocking your robot and you want help getting FusionCore running on your hardware, open a [GitHub Discussion](https://github.com/manankharwar/fusioncore/discussions) or email manan.kharwar@outlook.com directly. Fixed scope, fixed price.
