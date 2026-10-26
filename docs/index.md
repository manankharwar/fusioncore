# FusionCore

**ROS 2 sensor fusion for outdoor wheeled ground robots that lose GPS. IMU + wheel encoders + GPS at 100 Hz in a 23-state UKF. Two numbers from your IMU datasheet instead of days of tuning, and when it drifts it names the sensor and the reason. Handles bad calibration, timestamp jitter, delayed GPS, wheel slip, and ARM hardware out of the box. Apache 2.0.**

Built and measured for a **wheeled robot, outdoors, with wheel odometry**: differential
drive, skid steer, tracked or Ackermann. NCLT is a wheeled Segway and the hardware runs
are a four-wheeled rover, so that is the platform every number here was measured on. You
declare the platform in one line with `platform.profile` and that decides which states
the filter estimates, because a 23-state filter can estimate more than a wheeled robot
can observe. See [Declaring the platform](configuration.md#declaring-the-platform).

**Not built for drones, boats, legged robots or mecanum platforms.** The library runs on
them (the core is full 3D with no planar assumption, and `platform.profile: general`
freezes nothing), but nothing here is tuned or benchmarked for them and the
non-holonomic constraint is wrong for anything that can move sideways on purpose. For
those platforms, and for fusing an arbitrary set of sensors,
[robot_localization](https://github.com/cra-ros-pkg/robot_localization) is the right
answer: mature, general by design, and it is where this one has no evidence.

[![CI](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml/badge.svg)](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml)
[![arXiv](https://img.shields.io/badge/arXiv-2605.25239-b31b1b)](https://arxiv.org/abs/2605.25239)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.20091053.svg)](https://doi.org/10.5281/zenodo.20091053)

---

## Start without installing anything

If you have a rosbag, you can get something out of this before building it:

```bash
python3 tools/bag_report.py /path/to/your/bag
```

Every check works with no ground truth, because almost nobody has a surveyed
reference for their own robot. It reports whether your IMU and wheels disagree about
which way the robot turned, whether two sensors are on different clocks, and whether
the bag contains a manoeuvre that makes heading observable at all. See
[Bag report](bag-report.md).

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

!!! tip "Headless / Raspberry Pi"
    Add `COLCON_IGNORE` before building to skip the Gazebo package:
    ```bash
    touch ~/ros2_ws/src/fusioncore/fusioncore_gazebo/COLCON_IGNORE
    ```

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
| **No GPS, no wheels: visual SLAM only** | Set `vslam.topic` to fuse 6-DOF pose from ORB-SLAM3, RTAB-Map, Kimera, or any VSLAM that publishes `nav_msgs/Odometry`. Mahalanobis gate rejects reinitializations and tracking jumps automatically. Runs on IMU + VSLAM alone, indoors, GPS-denied. |

---

## See it before you install

**No ROS, no robot, 30 seconds:**

```bash
git clone https://github.com/manankharwar/fusioncore && cd fusioncore
pip install numpy matplotlib
python3 tools/demo_quick.py --open
```

Shows GPS spike rejection and overall accuracy from pre-baked NCLT data included in the repo. A 707 m corrupted GPS fix is injected at t=120 s. FusionCore's chi-squared gate rejects it (position changes 1 m). RL-EKF accepts it and deviates 50+ m before recovering. RL-UKF diverges with NaN before the spike even fires.

![FusionCore GPS spike rejection demo](assets/fig_spike_demo.png)

---

## How FusionCore differs from robot_localization

robot_localization is a solid, well-maintained package used on tens of thousands of robots. FusionCore makes different architectural choices:

| Capability | robot_localization | FusionCore |
|---|---|---|
| GPS fusion | navsat_transform node required; ECEF TF frame added in rolling-devel | Filter state runs natively in ECEF: no UTM projection |
| IMU bias estimation | Not in state vector | Gyro + accel bias as filter states |
| Outlier rejection | Mahalanobis threshold (manual scalars, no DOF guidance) | Mahalanobis chi-squared gate (pre-calibrated to sensor DOF) |
| Adaptive noise estimation | Uses sensor-reported covariance as-is | Adapts from 50-sample innovation window |
| ZUPT | Not built-in | Auto when stationary |
| Delay compensation | `smooth_lagged_data` + `history_length` | IMU ring buffer replay |
| GPS fix quality gating | Not built-in | HDOP, satellite count, fix type |
| Dual antenna heading | Not built-in | Yes |
| Inertial coast mode | Not built-in | Auto on sustained GPS dropout |
| GPS velocity fusion (wheel slip detection) | Any twist input (`twistN`), no slip detection | Yes (Doppler vs wheel innovation reveals slip) |
| Radar Doppler velocity fusion | Any twist input (`twistN`) | Yes (works indoors, all weather, slip detection) |
| VSLAM pose fusion | Any pose input (`poseN`) | Yes: `vslam.topic` fuses 6-DOF pose from ORB-SLAM3, RTAB-Map, Kimera, etc. |
| Multiple IMUs | Any number (`imu0`, `imu1`, ...) | Two: `imu2.topic` fuses a second IMU as an independent measurement |
| ROS 2 Jazzy / Humble | Ported from ROS 1 | Native, from scratch |

---


## Benchmark: 12 NCLT sequences, same config, no per-sequence tuning

Measured 2026-10-26 on all twelve [NCLT](http://robots.engin.umich.edu/nclt/) sequences under
a reporting rule committed before the run. Scored at GPS fix times, **the shipped default does
not beat raw unfiltered GPS**: raw GPS has the lower ATE RMSE on 11 of 12, and the twelfth is a
tie (9.186 m against 9.187 m). With opt-in re-anchoring (`gnss.reanchor_enabled`), FusionCore
is lower than raw GPS on 2 of 12 held-out sequences (2012-03-31 by 22.0%, 2012-10-28 by 5.6%).
Re-anchoring lowers RMSE against the default on 10 of 12 and raises it on none, but raises the
median on 4 and the output-rate jitter on 3, which is why it is off by default.

The comparison against robot_localization is being re-run and there is no current figure to
cite. The "10 of 12" comparison that stood here is withdrawn: it was measured through our
dataset player with two input errors that fed both filters wrong data. Every number, its
pipeline and the full history are in the [benchmark reference](reference/benchmark.md).

---


## Used on real hardware

People who have reported running or integrating FusionCore on their own robots.

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
> OAK-D (stereo depth + IMU) + Velodyne VLP-16 + rtabmap: indoor SLAM mapping, integration in progress

Running FusionCore on your robot? Drop a note in [Discussions #22](https://github.com/manankharwar/fusioncore/discussions/22) and I will add you here.

---

## In the ecosystem

**rtabmap_ros (merged):** FusionCore is included as a named demo in the official [rtabmap_ros](https://github.com/introlab/rtabmap_ros) repository, maintained by @matlabbe. The demo ("Turtlebot3 Nav2, 2D LiDAR SLAM with FusionCore") shows FusionCore and icp_odometry running in a feedback loop: FusionCore's stable odom frame seeds scan matching via `guess_frame_id`, and the ICP result feeds back into FusionCore as a second velocity source. [View the demo](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos)

**Stereolabs community:** a FusionCore + ZED integration guide is posted on the Stereolabs developer forum. [@privvyledge](https://github.com/privvyledge) set out to compare FusionCore against Wolf, TIER IV EagleEye and robot_localization in May 2026 on an F1/10 scale car (indoor, VESC + RealSense D435i) and a full-size van (GPS + ZED 2i + 360 LiDAR); no results have been reported back.

---

## Where to go next

- **New user** → [Getting Started](getting-started.md)
- **Configuring your robot** → [Configuration](configuration.md)
- **Pick a config for your hardware** → [Hardware Configs](hardware/index.md)
- **Using with Nav2** → [Nav2 Integration](nav2.md)
- **FusionCore vs robot_localization** → [Technical Comparison](vs-robot-localization.md)
- **Coming from robot_localization** → [Migration Guide](migration_from_robot_localization.md)
- **Simulation / testing without hardware** → [Simulation](simulation.md)
- **How the filter actually works** → [How It Works](how-it-works.md)

---

## Running FusionCore on real hardware?

If localization is blocking your robot and you want help getting it working, open a [GitHub Discussion](https://github.com/manankharwar/fusioncore/discussions/22) or email [manan.kharwar@outlook.com](mailto:manan.kharwar@outlook.com) directly. Fixed scope, fixed price, guaranteed result.
