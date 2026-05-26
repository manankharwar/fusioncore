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
| Noise covariance | Fixed at config time | Adapts from innovation sequence |
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
| 2012-06-15 | Summer | 55 min | 49.2 m | **18.2 m** | RL +63% |
| 2012-08-20 | Summer | 83 min | 98.3 m | **10.6 m** | RL +89% |
| 2012-09-28 | Fall | 77 min | **10.8 m** | 55.7 m | FC +81% |
| 2012-10-28 | Fall | 85 min | **29.9 m** | 60.0 m | FC +50% |
| 2012-11-04 | Fall | 79 min | **60.1 m** | 122.0 m | FC +51% |
| 2012-12-01 | Winter | 75 min | **21.0 m** | 90.7 m | FC +77% |
| 2013-02-23 | Winter | 78 min | **59.4 m** | 82.2 m | FC +28% |
| 2013-04-05 | Spring | 68 min | **12.1 m** | 268.9 m | FC +96% |

RL-UKF diverged with NaN on all twelve sequences. FusionCore wins 10 of 12.

Metric: ATE RMSE (meters), SE3-aligned to RTK ground truth using EVO. Same IMU, wheel odometry, and GPS inputs. Full methodology in the [benchmark reference](reference/benchmark.md).

---

## The two FusionCore losses

Both losses have identified root causes. They are documented here rather than hidden.

**2012-06-15 (FC 49.2m, RL 18.2m):** The dataset's GPS-sparsest sequence, with a 462-second blackout. During coast mode, any residual wheel encoder yaw bias (`b_ewz`) compounds at 100Hz into quadratic lateral drift. RL-EKF's 2D mode has fewer divergence degrees of freedom. The fix is magnetometer integration for absolute heading during GPS outage: not yet implemented.

**2012-08-20 (FC 98.3m, RL 10.6m):** 105 mode-3 GPS fixes located 720-840m from RTK ground truth appear in a 24-second window at a blackout boundary. Coast mode relaxes the chi-squared gate slightly to re-acquire GPS after the blackout. The adversarial cluster each individually pass the gate and collectively pull the estimate. RL-EKF incidentally rejects them through its miscalibrated gate (the same gate that causes its ten other losses). The fix is a `max_implied_speed` pre-check upstream of chi-squared gating: not yet implemented.

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

## Switching from robot_localization

See the [migration guide](migration_from_robot_localization.md) for a step-by-step walkthrough.

If localization is actively blocking your robot and you want help getting FusionCore running on your hardware, open a [GitHub Discussion](https://github.com/manankharwar/fusioncore/discussions) or email manan.kharwar@outlook.com directly. Fixed scope, fixed price.
