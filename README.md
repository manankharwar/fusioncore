# FusionCore

[![CI](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml/badge.svg)](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml)
[![DOI](https://img.shields.io/badge/DOI-10.5281%2Fzenodo.20091053-blue)](https://doi.org/10.5281/zenodo.20091053)
[![Docs](https://img.shields.io/badge/docs-manankharwar.github.io%2Ffusioncore-blue)](https://manankharwar.github.io/fusioncore/)
[![Paper](https://img.shields.io/badge/paper-arXiv%20preprint-b31b1b)](paper/fusioncore_arxiv.pdf)
[![Newsletter](https://img.shields.io/badge/newsletter-subscribe-orange)](https://manankharwar.substack.com)

**ROS 2 UKF sensor fusion for robots that run in the real world. IMU + wheel encoders + GPS at 100 Hz. Handles bad calibration, timestamp jitter, delayed GPS, wheel slip, and ARM hardware out of the box. Apache 2.0.**

<p align="center">
  <img width="800" height="384" alt="FusionCore running on a real robot" src="https://github.com/user-attachments/assets/9ef42175-6525-4e72-b8d0-a35b0cc5a09d"/>
</p>

---

## Install

### **Option A: From source** (ROS 2 Jazzy on Ubuntu 24.04 or Humble on Ubuntu 22.04):

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash  # or /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to fusioncore_ros
source install/setup.bash
```

**Verify it works** (single command, replaces the 4-terminal manual test):

```bash
bash tools/quick_test.sh
```

Starts FusionCore with fake sensors and checks all outputs in about 15 seconds. Prints `[PASS]` / `[FAIL]` for each check.

### **Option B: Docker (no ROS install required)**

```bash
docker pull ghcr.io/manankharwar/fusioncore:latest
docker run --rm -it ghcr.io/manankharwar/fusioncore:latest bash
```

Inside the container, verify everything works:

```bash
bash tools/quick_test.sh
```

---

## Try it without hardware

No ROS, no robot, 30 seconds:

```bash
git clone https://github.com/manankharwar/fusioncore && cd fusioncore
pip install numpy matplotlib
python3 tools/demo_quick.py --open
```

Shows two things from pre-baked NCLT benchmark data included in the repo:

- **GPS spike rejection:** a 707 m corrupted GPS fix is injected at t=120 s. FusionCore's chi-squared gate blocks it (position moves 1 m). robot_localization EKF accepts it and deviates 50+ m before recovering.
- **Overall accuracy:** FusionCore 5.6 m ATE vs RL-EKF 13.0 m ATE over a 600 s campus drive.

<p align="center">
  <img src="docs/assets/fig_spike_demo.png" alt="FusionCore GPS spike rejection demo: trajectory and error timeline" width="750">
</p>

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

## Benchmark

FusionCore vs robot_localization on the [NCLT dataset](http://robots.engin.umich.edu/nclt/): same IMU + wheel odometry + GPS, no manual tuning. Twelve full-length sequences across all seasons. RL-EKF run with chi-squared-equivalent thresholds at 99.9% confidence.

| Sequence | Season | Duration | FC ATE RMSE | RL-EKF ATE RMSE | Winner |
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

RL-UKF diverges with NaN on all twelve sequences. FusionCore wins 10/12 sequences. RL-EKF's losses trace to a single root cause: the GPS driver reports 3m sigma, but measured against RTK ground truth, actual p95 noise is 9.7-53.1m depending on the day. RL's gate is calibrated to the stated 3m and rejects valid fixes on bad-GPS days. FusionCore's adaptive noise estimation (`adaptive.gnss: true`) keeps chi2 statistics calibrated in real time.

The two FC losses are driven by a GPS data quality issue on 2012-08-20 (105 corrupt mode-3 fixes in a 24-second window at a blackout boundary) and accumulated heading error during a 462-second GPS blackout on 2012-06-15. See [benchmarks/README.md](benchmarks/README.md) for full per-sequence analysis including root causes and path-to-fix.

<p align="center">
  <img src="docs/assets/fig2_traj_grid.png" alt="Trajectory overlay: all 9 sequences, SE3-aligned to RTK GPS ground truth" width="650">
</p>

---

## Used on real hardware

Real engineers, real robots, real sensor data. Not demos.

> "The system was stable on real robot data and was relatively easy to configure. I was able to get reasonable behavior without spending excessive time on parameter tuning. The overall experience felt more deployment-oriented than research-demo-oriented."
>
> **Michał Bednarek** ([@mbed92](https://github.com/mbed92)), Robotics PhD
> Factory differential-drive robot, ROS 2 Humble: Cartographer (point-cloud localization, no preloaded map) + wheel odometry + IMU

<br>

> "Having a go at using FusionCore in an agricultural field robot. Hopefully will have a robot moving in a month or two."
>
> **Sam** ([@samuk](https://github.com/samuk)), [Agroecology Lab](https://github.com/Agroecology-Lab/feldfreund_devkit_ros)
> Outdoor agricultural robot, integration in progress

> **Russ Hall**, Andino robot (Raspberry Pi)
> OAK-D (visual odometry + IMU) + Velodyne VLP-16 + rtabmap: indoor mapping and autonomous outdoor navigation

Running FusionCore on your robot? Drop a note in [Discussions #22](https://github.com/manankharwar/fusioncore/discussions/22) and I will add you here.

---

## In the ecosystem

**rtabmap_ros (merged):** FusionCore is included as a named demo in the official [rtabmap_ros](https://github.com/introlab/rtabmap_ros) repository, maintained by @matlabbe. The demo ("Turtlebot3 Nav2, 2D LiDAR SLAM with FusionCore") shows FusionCore and icp_odometry running in a feedback loop: FusionCore's stable odom frame seeds scan matching via `guess_frame_id`, and the ICP result feeds back into FusionCore as a second velocity source. [View the demo](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos)

**Stereolabs community:** FusionCore + ZED integration guide posted on the Stereolabs developer forum, acknowledged by Stereolabs support. Under active evaluation by [@privvyledge](https://github.com/privvyledge) comparing FusionCore against Wolf, TIER IV EagleEye, and robot_localization on two platforms: an F1/10 scale car (indoor, VESC + RealSense D435i) and a full-size autonomous van (GPS + ZED 2i + 360 LiDAR).

---

## Coming from robot_localization?

If any of these have bitten you, FusionCore was built with them in mind:

| robot_localization issue | What FusionCore does instead |
|---|---|
| UKF diverges with NaN on GPS-heavy sequences ([#780](https://github.com/cra-ros-pkg/robot_localization/issues/780), [#777](https://github.com/cra-ros-pkg/robot_localization/issues/777)) | Chi-squared gate on every sensor; covariance bounded at each step. All nine NCLT sequences finish without NaN. |
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
  version   = {0.2.3},
  doi       = {10.5281/zenodo.20091053},
  url       = {https://doi.org/10.5281/zenodo.20091053}
}
```

---

Issues answered within 24 hours. Open a GitHub issue or find the discussion on [ROS Discourse](https://discourse.ros.org).
