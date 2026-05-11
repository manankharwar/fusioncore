# FusionCore

**ROS 2 UKF sensor fusion for robots that run in the real world. IMU + wheel encoders + GPS at 100 Hz. Handles bad calibration, timestamp jitter, delayed GPS, wheel slip, and ARM hardware out of the box. Apache 2.0.**

[![CI](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml/badge.svg)](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.19834991.svg)](https://doi.org/10.5281/zenodo.19834991)

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
| Adaptive noise | Fixed config values | Auto from innovation sequence |
| ZUPT | Not built-in | Auto when stationary |
| Delay compensation | `smooth_lagged_data` + `history_length` | IMU ring buffer replay |
| GPS fix quality gating | Not built-in | HDOP, satellite count, fix type |
| Dual antenna heading | Not built-in | Yes |
| Inertial coast mode | Not built-in | Auto on sustained GPS dropout |
| GPS velocity fusion (wheel slip detection) | Not built-in | Yes: Doppler vs wheel innovation reveals slip |
| Radar Doppler velocity fusion | Not built-in | Yes: works indoors, all weather, slip detection |
| Dual IMU | Not built-in | Yes: `imu2.topic` fuses a second IMU as an independent measurement |
| ROS 2 Jazzy / Humble | Ported from ROS 1 | Native, from scratch |

---

## Where to go next

- **New user** → [Getting Started](getting-started.md)
- **Configuring your robot** → [Configuration](configuration.md)
- **Pick a config for your hardware** → [Hardware Configs](hardware/index.md)
- **Using with Nav2** → [Nav2 Integration](nav2.md)
- **Coming from robot_localization** → [Migration Guide](migration_from_robot_localization.md)
- **Simulation / testing without hardware** → [Simulation](simulation.md)
- **How the filter actually works** → [How It Works](how-it-works.md)
