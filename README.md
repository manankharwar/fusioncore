# FusionCore

[![CI](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml/badge.svg)](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml)
[![DOI](https://img.shields.io/badge/DOI-10.5281%2Fzenodo.19834991-blue)](https://doi.org/10.5281/zenodo.19834991)
[![Docs](https://img.shields.io/badge/docs-manankharwar.github.io%2Ffusioncore-blue)](https://manankharwar.github.io/fusioncore/)
[![Paper](https://img.shields.io/badge/paper-arXiv%20preprint-b31b1b)](paper/fusioncore_arxiv.pdf)
[![Newsletter](https://img.shields.io/badge/newsletter-subscribe-orange)](https://manankharwar.substack.com)

> **Stay in the loop:** biweekly updates on new features, benchmarks, and real-world robot integrations. [Subscribe →](https://manankharwar.substack.com)

**ROS 2 sensor fusion: IMU + wheel encoders + GPS fused via UKF at 100 Hz. 22-state filter with IMU bias estimation, adaptive noise covariance, chi-squared outlier rejection on every sensor, GPS velocity fusion for wheel slip detection, and radar Doppler ego-velocity fusion.**

<p align="center">
  <img width="900" height="500" alt="LinkedIn1-ezgif com-optimize" src="https://github.com/user-attachments/assets/e1e07cfb-74e0-48b9-9bfd-32b68ee5a6ef"/>
</p>

---

## Why I built this

I needed sensor fusion for a mobile robot project and reached for `robot_localization` like everyone does. It works well. But I wanted a filter that estimated IMU gyro and accelerometer bias as part of the state vector, adapted its noise covariance from real sensor behavior rather than config values, and rejected outliers on every sensor update: not just GPS.

So I built FusionCore. It's a 22-state UKF that fuses IMU, wheel encoders, GPS position, GPS velocity, and radar Doppler ego-velocity natively. Gyro and accelerometer bias are estimated continuously as filter states. Noise covariance adapts from the innovation sequence automatically. Every sensor update goes through a chi-squared gate before it touches the filter. GPS is handled in ECEF directly, no coordinate projection.

GPS velocity fusion (from any receiver that publishes Doppler velocity, like the u-blox F9P) compares GPS-reported speed against wheel-reported speed on every filter cycle. The innovation directly reveals wheel slip: the Kalman gain automatically down-weights a slipping wheel in proportion to how much it disagrees with GPS. Radar Doppler velocity fusion works the same way but using radio wave physics instead of satellites, so it functions indoors, in rain, fog, dust, and complete darkness.

<p align="center">
  <img src="docs/assets/fig2_traj_grid.png" alt="Trajectory overlay: all 6 sequences, SE3-aligned to RTK GPS ground truth" width="650">
</p>

---

## Benchmark

FusionCore vs robot_localization on the [NCLT dataset](http://robots.engin.umich.edu/nclt/): same IMU + wheel odometry + GPS, no manual tuning. Six sequences:

RL-EKF run with `odom0_twist_rejection_threshold: 4.03` and `odom1_pose_rejection_threshold: 3.72` (chi²-equivalent to FusionCore's thresholds at 99.9% confidence).

| Sequence | FC ATE RMSE | RL-EKF ATE RMSE | RL-UKF |
|---|---|---|---|
| 2012-01-08 | **5.6 m** | 13.0 m | NaN divergence at t=31 s |
| 2012-02-04 | **9.7 m** | 19.1 m | NaN divergence at t=22 s |
| 2012-03-31 | **4.2 m** | 54.3 m | NaN divergence at t=18 s |
| 2012-08-20 | **7.5 m** | 24.1 m | NaN divergence |
| 2012-11-04 | 28.6 m | **9.6 m** | NaN divergence |
| 2013-02-23 | **4.1 m** | 11.0 m | NaN divergence |

---

## Try it yourself

**No ROS required (30 seconds):**

```bash
git clone https://github.com/manankharwar/fusioncore && cd fusioncore
pip install numpy matplotlib
python3 tools/demo_quick.py --open
```

Generates a side-by-side trajectory comparison from the NCLT benchmark results included in the repository. No datasets to download, no ROS installation needed.

**Live demo with real sensor data (5 minutes):**

```bash
# Build FusionCore, then:
bash demo/run_demo.sh
```

Downloads a 5 MB demo bag, runs FusionCore live against 120 seconds of real outdoor robot data (IMU + wheel odometry + GPS), and generates the comparison plot automatically.

See [demo/README.md](demo/README.md) for full instructions, including running on your own robot bag.

---

## Does it work on a real robot with messy sensors?

**Does it tolerate imperfect IMU calibration?**
Yes. `adaptive.imu: true` (default) automatically adjusts the measurement noise matrix from the innovation sequence. `init.stationary_window: 2.0` estimates accelerometer bias before motion starts. In the NCLT benchmark, FusionCore was given only the two noise values from the IMU datasheet with no manual tuning of any other parameter.

**How much manual tuning is needed?**
Two numbers from your IMU datasheet: `imu.gyro_noise` (ARW spec) and `imu.accel_noise` (VRW spec). Everything else starts at default. Adaptive noise covariance handles the rest within the first minute of operation. Most companies copy approximate values from a CAD model and launch. FusionCore is designed to work under exactly those conditions.

**What about timestamp jitter and delayed GPS?**
FusionCore stores a rolling IMU buffer and replays intermediate updates when a delayed GPS fix arrives (retrodiction, up to 500 ms configurable). Timestamp jitter is handled by clamping `dt` to `[min_dt, max_dt]`. Out-of-order messages are absorbed without divergence.

**What is the CPU cost?**
A 22-state UKF at 100 Hz generates 45 sigma points per predict step. On a laptop Intel i7: under 0.2 ms per cycle. On Raspberry Pi 4: under 1 ms per cycle. Peak RAM under 50 MB. FusionCore uses Eigen for all matrix math; no external numeric libraries required.

**Does it behave the same on ARM (Raspberry Pi, Jetson)?**
Yes. Eigen auto-detects NEON on ARM and AVX on x86. The NCLT benchmark is reproducible on ARM within floating-point rounding tolerance. The same binary runs on laptop, Pi 4, and Jetson Orin without recompilation or parameter changes.

---

## Coming from robot_localization?

If any of these have bitten you, FusionCore was built with them in mind:

| robot_localization issue | What FusionCore does instead |
|---|---|
| UKF diverges with NaN on GPS-heavy sequences ([#780](https://github.com/cra-ros-pkg/robot_localization/issues/780), [#777](https://github.com/cra-ros-pkg/robot_localization/issues/777)) | Chi-squared gate on every sensor; covariance bounded at each step. All six NCLT sequences finish without NaN. |
| navsat_transform crashes or gives wrong position at UTM zone boundaries ([#951](https://github.com/cra-ros-pkg/robot_localization/issues/951), [#904](https://github.com/cra-ros-pkg/robot_localization/issues/904)) | GPS fused directly in ECEF. No UTM projection, no zone boundary. |
| No non-holonomic constraint for wheeled robots ([#744](https://github.com/cra-ros-pkg/robot_localization/issues/744)) | Built-in NHC: lateral and vertical velocity forced to zero as a virtual measurement on every cycle. |
| Delayed sensor messages cause missed updates ([#911](https://github.com/cra-ros-pkg/robot_localization/issues/911)) | Rolling IMU buffer with retrodiction. Late GPS fixes replay missed IMU steps automatically (up to 500 ms configurable). |
| Non-deterministic output across runs ([#957](https://github.com/cra-ros-pkg/robot_localization/issues/957)) | `--ros-args -p replay_mode:=true` freezes the clock and replays bags identically every time. |
| IMU frame confusion: body vs sensor frame ([#757](https://github.com/cra-ros-pkg/robot_localization/issues/757)) | Single `imu.frame_id` parameter; transform applied once at ingestion, filter runs in body frame throughout. |
| navsat_transform CPU load scales with fix rate ([#890](https://github.com/cra-ros-pkg/robot_localization/issues/890)) | No navsat_transform node. ECEF conversion is one matrix multiply per GPS message inside the filter. |

Migration guide: [manankharwar.github.io/fusioncore/migration_from_robot_localization](https://manankharwar.github.io/fusioncore/migration_from_robot_localization/)

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

---

## Quick start

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

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
