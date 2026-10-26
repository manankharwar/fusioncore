# FusionCore vs robot_localization

An architectural comparison: what each package models and how it handles GPS. The measured
comparison on NCLT is being re-run (see [Benchmark](#benchmark) below), so this page makes no
accuracy claim in either direction.

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
| GPS velocity fusion | Any twist input (`twistN`), no slip detection | Yes, with slip detection (Doppler against wheel innovation) |
| VSLAM pose fusion | Any pose input (`poseN`) | Yes: `vslam.topic` |
| Multiple IMUs | Any number (`imu0`, `imu1`, ...) | Two: `imu.topic` and `imu2.topic` |
| ROS 2 native | Ported from ROS 1 | Written from scratch for ROS 2 |

---

## Benchmark

**There is currently no FusionCore against robot_localization comparison to cite.** The
NCLT comparison that stood here ("FusionCore wins 10 of 12") is withdrawn: it was measured
through our dataset player with two input errors that fed both filters wrong data, a wheel
odometry yaw rate in the wrong frame (#169) and an accelerometer-derived tilt declared as a
0.57 degree attitude. The explanation of robot_localization's losses that came with it (a
gate calibrated to optimistic GPS covariance) was built on those runs and is withdrawn with
them.

The corrected twelve-sequence evaluation ran on 2026-10-25 and 26. Its robot_localization arm
is withheld: the corrected player stopped supplying an attitude, and with this config
robot_localization then removed gravity using a level attitude, so real tilt leaked into the
fused acceleration and it diverged on nine sequences. That was our input change meeting that
config, not a property of robot_localization. It is being re-run on all twelve sequences with
a config that follows robot_localization's own planar wheeled-robot example, and the results
will be published per sequence and per metric, with no aggregate and no winner.

What the corrected run does show, against raw unfiltered GPS rather than robot_localization:
the shipped default does not beat raw GPS (raw GPS lower on 11 of 12, one tie), and with
opt-in re-anchoring FusionCore is lower on 2 of 12 held-out sequences. Every number is in the
[benchmark reference](reference/benchmark.md).

On hardware, FusionCore has not been shown to beat robot_localization either: on two
frozen-config rover runs robot_localization closed 0.68 m against FusionCore's 1.48 m, then
an exact 1.05 m tie.

What does not need a reference is outlier rejection, which rests on kinematics: a 60 m GPS
jump in one second is impossible for a robot moving at 1 m/s.

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
