# FusionCore

[![CI](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml/badge.svg)](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml)
[![DOI](https://img.shields.io/badge/DOI-10.5281%2Fzenodo.19834991-blue)](https://doi.org/10.5281/zenodo.19834991)
[![Docs](https://img.shields.io/badge/docs-manankharwar.github.io%2Ffusioncore-blue)](https://manankharwar.github.io/fusioncore/)
[![Paper](https://img.shields.io/badge/paper-arXiv%20preprint-b31b1b)](paper/fusioncore_arxiv.pdf)
[![Newsletter](https://img.shields.io/badge/newsletter-subscribe-orange)](https://manankharwar.substack.com)

> **Stay in the loop:** biweekly updates on new features, benchmarks, and real-world robot integrations. [Subscribe →](https://manankharwar.substack.com)

**ROS 2 UKF sensor fusion for robots that run in the real world. IMU + wheel encoders + GPS at 100 Hz. Handles bad calibration, timestamp jitter, delayed GPS, wheel slip, and ARM hardware out of the box. Apache 2.0.**

---

## Install

Supports **ROS 2 Jazzy** (Ubuntu 24.04) and **Humble** (Ubuntu 22.04).

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash  # or /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build && source install/setup.bash
```

> **Headless / Raspberry Pi:** `touch ~/ros2_ws/src/fusioncore/fusioncore_gazebo/COLCON_IGNORE` before building to skip the Gazebo package.

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

---

## Works on the hardware you actually have

Most sensor fusion tutorials assume clean data. Real robots don't have clean data. FusionCore was built around the problems you actually run into.

| The problem | How FusionCore handles it |
|---|---|
| **IMU calibration is approximate** | Gyro and accel bias are filter states, estimated continuously. `init.stationary_window: 2.0` estimates startup bias before motion begins, dropping startup drift from ~10 cm to under 1 cm. |
| **Extrinsic calibration is never exact** | Reads `frame_id` from every IMU message and looks up the TF rotation to `base_link` automatically. Set `imu.frame_id` to override broken frame names from drivers (e.g. Gazebo TurtleBot3). No manual rotation matrices. |
| **Timestamp jitter and zero-stamped drivers** | `dt` is clamped to prevent divergence from missed timer ticks. Wall clock fallback for drivers that publish `stamp={sec=0}`. |
| **GPS arrives late (50–200 ms)** | IMU ring buffer replays 1 second of buffered updates when a delayed fix arrives. The state at the GPS timestamp is reconstructed exactly, not approximated. |
| **Wheel odometry is noisy or slipping** | Adaptive noise covariance updates from the innovation sequence. GPS velocity fusion (optional) compares GPS-reported speed against wheel speed every cycle: the innovation reveals slip and the Kalman gain down-weights the slipping wheel automatically. |
| **Noise parameters require days of tuning** | Two numbers from your IMU datasheet: `imu.gyro_noise` (ARW) and `imu.accel_noise` (VRW). Everything else adapts within the first minute of operation. |
| **Robot runs on Raspberry Pi or Jetson** | Under 0.2 ms per cycle on i7. Under 1 ms on Raspberry Pi 4. Same binary on ARM (NEON auto-detected) and x86 (AVX auto-detected) via Eigen. No recompilation, no parameter changes. |
| **Two IMUs on the platform** | Set `imu2.topic` to fuse a second IMU as an independent measurement. No pre-merging with `imu_filter_madgwick` needed. |
| **GPS drops out in tunnels or canopy** | Inertial coast mode maintains position integrity during sustained GPS dropout. Outlier gate relaxes automatically to reacquire when GPS returns. |
| **Robot sits still for minutes** | ZUPT (zero velocity update) fuses a zero-velocity pseudo-measurement when encoder speed and angular rate are both below threshold. Prevents IMU noise from integrating into position drift during idle periods. |

---

## Try it without hardware

**No ROS, no robot, 30 seconds:**

```bash
git clone https://github.com/manankharwar/fusioncore && cd fusioncore
pip install numpy matplotlib
python3 tools/demo_quick.py --open
```

Shows two things from pre-baked NCLT benchmark data included in the repo:

- **GPS spike rejection:** a 707 m corrupted GPS fix is injected at t=120 s. FusionCore's chi-squared gate blocks it (position moves 1 m). robot_localization EKF accepts it and deviates 50+ m before recovering.
- **Overall accuracy:** FusionCore 5.6 m ATE vs RL-EKF 13.0 m ATE over a 600 s campus drive. RL-UKF diverges with NaN at t=31 s.

<p align="center">
  <img src="docs/assets/fig_spike_demo.png" alt="FusionCore GPS spike rejection demo: trajectory and error timeline" width="750">
</p>

**Live demo with real sensor data:**

```bash
bash demo/run_demo.sh
```

Downloads a 5 MB demo bag and runs FusionCore live against 120 seconds of real outdoor robot data (IMU + wheel odometry + GPS).

---

## Benchmark

FusionCore vs robot_localization on the [NCLT dataset](http://robots.engin.umich.edu/nclt/): same IMU + wheel odometry + GPS, no manual tuning. Six 30-minute sequences. RL-EKF run with chi-squared-equivalent thresholds at 99.9% confidence.

| Sequence | FC ATE RMSE | RL-EKF ATE RMSE | RL-UKF |
|---|---|---|---|
| 2012-01-08 | **5.6 m** | 13.0 m | NaN divergence at t=31 s |
| 2012-02-04 | **9.7 m** | 19.1 m | NaN divergence at t=22 s |
| 2012-03-31 | **4.2 m** | 54.3 m | NaN divergence at t=18 s |
| 2012-08-20 | **7.5 m** | 24.1 m | NaN divergence |
| 2012-11-04 | 28.6 m | **9.6 m** | NaN divergence |
| 2013-02-23 | **4.1 m** | 11.0 m | NaN divergence |

RL-UKF diverges with NaN on all six sequences. RL-EKF completes all six but with 2–10x higher ATE. FusionCore loses on the 2012-11-04 sequence: root cause under investigation.

<p align="center">
  <img src="docs/assets/fig2_traj_grid.png" alt="Trajectory overlay: all 6 sequences, SE3-aligned to RTK GPS ground truth" width="650">
</p>

---

## Coming from robot_localization?

If any of these have bitten you, FusionCore was built with them in mind:

| robot_localization issue | What FusionCore does instead |
|---|---|
| UKF diverges with NaN on GPS-heavy sequences ([#780](https://github.com/cra-ros-pkg/robot_localization/issues/780), [#777](https://github.com/cra-ros-pkg/robot_localization/issues/777)) | Chi-squared gate on every sensor; covariance bounded at each step. All six NCLT sequences finish without NaN. |
| navsat_transform crashes at UTM zone boundaries ([#951](https://github.com/cra-ros-pkg/robot_localization/issues/951), [#904](https://github.com/cra-ros-pkg/robot_localization/issues/904)) | GPS fused directly in ECEF. No UTM projection, no zone boundary. |
| No non-holonomic constraint for wheeled robots ([#744](https://github.com/cra-ros-pkg/robot_localization/issues/744)) | Built-in NHC: lateral and vertical velocity zeroed as a virtual measurement on every encoder update. |
| Delayed sensor messages cause missed updates ([#911](https://github.com/cra-ros-pkg/robot_localization/issues/911)) | Rolling IMU buffer with retrodiction. Late GPS fixes replay missed IMU steps automatically (up to 500 ms). |
| Non-deterministic output across bag replays ([#957](https://github.com/cra-ros-pkg/robot_localization/issues/957)) | Message timestamps drive everything under `use_sim_time:true`. Same bag + same config = identical output. |
| IMU frame confusion: body vs sensor frame ([#757](https://github.com/cra-ros-pkg/robot_localization/issues/757)) | TF lookup on every message. `imu.frame_id` override for broken driver frame names. |
| navsat_transform CPU load scales with fix rate ([#890](https://github.com/cra-ros-pkg/robot_localization/issues/890)) | No navsat_transform node. ECEF conversion is one matrix multiply per GPS message inside the filter. |

Migration guide: [manankharwar.github.io/fusioncore/migration_from_robot_localization](https://manankharwar.github.io/fusioncore/migration_from_robot_localization/)

---

## Documentation

**[manankharwar.github.io/fusioncore](https://manankharwar.github.io/fusioncore/)**

- [Getting Started](https://manankharwar.github.io/fusioncore/getting-started/)
- [Configuration reference](https://manankharwar.github.io/fusioncore/configuration/)
- [Hardware configs](https://manankharwar.github.io/fusioncore/hardware/)
- [Nav2 integration](https://manankharwar.github.io/fusioncore/nav2/)
- [Migrating from robot_localization](https://manankharwar.github.io/fusioncore/migration_from_robot_localization/)
- [How it works](https://manankharwar.github.io/fusioncore/how-it-works/)

---

## License

Apache 2.0.

---

## Citation

```bibtex
@software{kharwar2026fusioncore,
  author    = {Kharwar, Manan},
  title     = {FusionCore: ROS 2 UKF Sensor Fusion},
  year      = {2026},
  publisher = {Zenodo},
  version   = {0.2.0},
  doi       = {10.5281/zenodo.19834991},
  url       = {https://doi.org/10.5281/zenodo.19834991}
}
```

---

Issues answered within 24 hours. Open a GitHub issue or find the discussion on [ROS Discourse](https://discourse.ros.org).
