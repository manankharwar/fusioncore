# FusionCore

[![CI](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml/badge.svg)](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml)
[![arXiv](https://img.shields.io/badge/arXiv-2605.25239-b31b1b)](https://arxiv.org/abs/2605.25239)
[![DOI](https://img.shields.io/badge/DOI-10.5281%2Fzenodo.20091053-blue)](https://doi.org/10.5281/zenodo.20091053)
[![Docs](https://img.shields.io/badge/docs-manankharwar.github.io%2Ffusioncore-blue)](https://manankharwar.github.io/fusioncore/)
[![Newsletter](https://img.shields.io/badge/newsletter-subscribe-orange)](https://manankharwar.substack.com)
<a href="https://www.producthunt.com/products/fusioncore?embed=true&amp;utm_source=badge-featured&amp;utm_medium=badge&amp;utm_campaign=badge-fusioncore-0-3-2" target="_blank" rel="noopener noreferrer"><img alt="FusionCore 0.3.2 - Robust sensor fusion for ROS 2, zero manual tuning | Product Hunt" width="250" height="54" src="https://api.producthunt.com/widgets/embed-image/v1/featured.svg?post_id=1189863&amp;theme=light&amp;t=1783397121405"></a>

**ROS 2 UKF sensor fusion for robots that run in the real world. IMU + wheel encoders + GPS at 100 Hz. Handles bad calibration, timestamp jitter, delayed GPS, wheel slip, and ARM hardware out of the box. Apache 2.0.**


<img width="1080" height="608" alt="586785007-e1e07cfb-74e0-48b9-9bfd-32b68ee5a6ef" src="https://github.com/user-attachments/assets/d59b74ec-af94-4cb1-ab19-e5310a5d138b" />

<img width="1900" height="1069" alt="fig5_architecture" src="https://github.com/user-attachments/assets/c06af5c4-dcd5-48ca-8871-d13ba969d573" />

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

The easiest way to try FusionCore — no ROS 2 installation needed. The image is hosted on GitHub Container Registry (GHCR).

```bash
# Pull the image
docker pull ghcr.io/manankharwar/fusioncore:latest

# Quick test (15 seconds, no hardware)
docker run --rm ghcr.io/manankharwar/fusioncore:latest bash tools/quick_test.sh

# Interactive shell
docker run --rm -it ghcr.io/manankharwar/fusioncore:latest bash

# Run with your own YAML config + topic remaps
docker run --rm -it --net=host \
  -v ~/my_robot.yaml:/config/robot.yaml:ro \
  ghcr.io/manankharwar/fusioncore:latest \
  ros2 launch fusioncore_ros fusioncore.launch.py \
    fusioncore_config:=/config/robot.yaml \
    --ros-args \
    -r /imu/data:=/your/imu/topic \
    -r /gnss/fix:=/your/gps/topic
```

Full guide (volume mounts, topic remapping, `--net=host`): [docs/docker.md](https://manankharwar.github.io/fusioncore/docker/)

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

<img width="1200" height="675" alt="fusioncore_demo_hmm" src="https://github.com/user-attachments/assets/89e9134d-3ec1-4cd9-898b-e3a9c62852dd" />


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
| 2012-09-28 | Fall | 77 min | **22.4 m** | 53.8 m | FC +58% |
| 2012-10-28 | Fall | 85 min | **15.6 m** | 56.4 m | FC +72% |
| 2012-11-04 | Fall | 79 min | **60.1 m** | 122.0 m | FC +51% |
| 2012-12-01 | Winter | 75 min | **21.0 m** | 90.7 m | FC +77% |
| 2013-02-23 | Winter | 78 min | **59.4 m** | 82.2 m | FC +28% |
| 2013-04-05 | Spring | 68 min | **12.1 m** | 268.9 m | FC +96% |

<img width="1422" height="1245" alt="fig_trajectory" src="https://github.com/user-attachments/assets/7f78474b-e70b-4b38-95ef-c759e1fcea02" />

> **Note:** these numbers are a snapshot pending a controlled full-suite re-run on current `main`. The 10/12 result holds, but the 2013-04-05 figure (12.1 m) is stale: it has since regressed to ~19.4 m (still a 93% win). See `tools/benchmark_regression.md`.

<img width="1200" height="960" alt="fusioncore_demo" src="https://github.com/user-attachments/assets/1d6975a0-2f9b-400d-a47c-457568c01586" />


RL-UKF diverges with NaN on all twelve sequences. FusionCore wins 10/12 sequences. RL-EKF's losses trace to a single root cause: the GPS driver reports 3m sigma, but measured against RTK ground truth, actual p95 noise is 9.7-53.1m depending on the day. RL's gate is calibrated to the stated 3m and rejects valid fixes on bad-GPS days. FusionCore's adaptive noise estimation (`adaptive.gnss: true`) keeps chi2 statistics calibrated in real time.

<img width="1485" height="1035" alt="fig_adaptive_noise" src="https://github.com/user-attachments/assets/97c7b12d-8b93-48d1-bab1-3e03d21ea02f" />

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
> OAK-D (stereo depth + IMU) + Velodyne VLP-16 + rtabmap: indoor SLAM mapping

Running FusionCore on your robot? Drop a note in [Discussions #22](https://github.com/manankharwar/fusioncore/discussions/22) and I will add you here.

<img width="1431" height="1127" alt="fig_bias_estimation" src="https://github.com/user-attachments/assets/da189e76-bb54-4d90-b0a7-48926060b86a" />


---

## In the ecosystem

**rtabmap_ros (merged):** FusionCore is included as a named demo in the official [rtabmap_ros](https://github.com/introlab/rtabmap_ros) repository, maintained by @matlabbe. The demo ("Turtlebot3 Nav2, 2D LiDAR SLAM with FusionCore") shows FusionCore and icp_odometry running in a feedback loop: FusionCore's stable odom frame seeds scan matching via `guess_frame_id`, and the ICP result feeds back into FusionCore as a second velocity source. [View the demo](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos)

**Stereolabs community:** FusionCore + ZED integration guide posted on the Stereolabs developer forum, acknowledged by Stereolabs support. Under active evaluation by [@privvyledge](https://github.com/privvyledge) comparing FusionCore against Wolf, TIER IV EagleEye, and robot_localization on two platforms: an F1/10 scale car (indoor, VESC + RealSense D435i) and a full-size autonomous van (GPS + ZED 2i + 360 LiDAR).

**OpenMowerNext (integration in progress):** FusionCore is being integrated as the localization stack in [OpenMowerNext](https://github.com/jkaflik/OpenMowerNext), a community ROS 2 autonomous mowing system. The integration replaces robot_localization with a single FusionCore lifecycle node fusing RTK GPS (u-blox F9P), IMU, and wheel odometry, with ECEF datum calculated from the mower's home position. [PR #45](https://github.com/jkaflik/OpenMowerNext/pull/45)

---

## Switching from robot_localization?

FusionCore is a drop-in replacement for the robot_localization + navsat_transform stack. The migration guide covers the YAML and launch file changes: [manankharwar.github.io/fusioncore/migration_from_robot_localization](https://manankharwar.github.io/fusioncore/migration_from_robot_localization/)

<img width="1008" height="334" alt="demo_plot" src="https://github.com/user-attachments/assets/4fdec6e5-f827-4111-bd91-912992ab17fb" />

The architectural differences that matter in practice:

| Problem | How FusionCore handles it |
|---|---|
| GPS outliers corrupt the state | Chi-squared gate per sensor DOF rejects bad fixes before they reach the filter. Covariance bounded at every step — no NaN divergence. |
| UTM zone boundary near the operating area | GPS fused directly in ECEF. No UTM projection, no zone boundary edge case. |
| Wheeled robot drifts laterally without GPS | Non-holonomic constraint (NHC) zeros lateral and vertical velocity as a virtual measurement on every encoder update. |
| GPS fixes arrive 50–200 ms late | IMU ring buffer with retrodiction. Late fixes replay missed IMU steps and reconstruct the exact filter state at the GPS timestamp. |
| Bag replay gives different results each run | Message timestamps drive all updates under `use_sim_time: true`. Same bag, same config, identical output. |
| IMU mounted off-axis or with a broken frame name | TF lookup on every message. `imu.frame_id` override for drivers that publish wrong frame names. |
| navsat_transform node adds CPU load and startup ordering complexity | No navsat_transform node. ECEF conversion is one matrix multiply per GPS fix inside the filter. |

<img width="1109" height="1035" alt="fig_mahalanobis" src="https://github.com/user-attachments/assets/702427fe-9685-49f9-be2a-7c27db45691d" />
<img width="1967" height="1087" alt="fig3_two_outcomes" src="https://github.com/user-attachments/assets/792430dd-b86a-44e7-b51b-7a462cd8538d" />
---

## Documentation

**[manankharwar.github.io/fusioncore](https://manankharwar.github.io/fusioncore/)**

- [Getting Started](https://manankharwar.github.io/fusioncore/getting-started/)
- [Docker](https://manankharwar.github.io/fusioncore/docker/)
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
@article{kharwar2026fusioncore,
  author  = {Kharwar, Manan},
  title   = {FusionCore: A 23-State Unscented Kalman Filter for
             IMU, Wheel Encoder, GPS, and Visual SLAM Fusion in ROS 2},
  journal = {arXiv preprint arXiv:2605.25239},
  year    = {2026},
  url     = {https://arxiv.org/abs/2605.25239}
}
```

If you prefer to cite the software release directly:

```bibtex
@software{kharwar2026fusioncore_software,
  author    = {Kharwar, Manan},
  title     = {FusionCore: ROS 2 UKF Sensor Fusion},
  year      = {2026},
  publisher = {Zenodo},
  doi       = {10.5281/zenodo.20091053},
  url       = {https://doi.org/10.5281/zenodo.20091053}
}
```

---

Issues answered within 24 hours. Open a GitHub issue or find the discussion on [ROS Discourse](https://discourse.ros.org).

Running FusionCore on your robot? Open a [Discussion](https://github.com/manankharwar/fusioncore/discussions/22) to get listed in [ADOPTERS.md](ADOPTERS.md).

Need guaranteed results on your hardware? [Commercial support and fixed-price integration](https://manankharwar.github.io/fusioncore/support/) available.
