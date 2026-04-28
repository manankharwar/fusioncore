# FusionCore

**ROS 2 UKF sensor fusion. IMU + wheel encoders + GPS → one position estimate. No manual noise tuning. Apache 2.0.**

[![CI](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml/badge.svg)](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.19834991.svg)](https://doi.org/10.5281/zenodo.19834991)

---

## What it does

FusionCore is a 22-state UKF that fuses IMU, wheel encoders, and GPS into a single clean odometry output at 100 Hz. It runs as a single ROS 2 lifecycle node — no `navsat_transform`, no coordinate projection node, no feedback loop between two filters.

It publishes `/fusion/odom` and the full `odom → base_link` TF. Nav2 consumes it directly.

GPS is optional. FusionCore runs fine on IMU + wheel odometry alone for indoor robots.

---

## Why not robot_localization?

| Capability | robot_localization | FusionCore |
|---|---|---|
| GPS fusion | navsat_transform node (extra node, latency, UTM zone issues) | ECEF-native, single node |
| IMU bias estimation | No | Gyro + accel bias states |
| Outlier rejection | mahalanobis_threshold (one global threshold) | Chi-squared gating per sensor |
| Adaptive noise | Manual YAML tuning, re-tune per environment | Auto from innovation sequence |
| ZUPT | Not built-in | Auto when stationary |
| Delay compensation | history_length (approximate) | Full IMU replay retrodiction |
| GPS fix quality gating | No | GPS / DGPS / RTK_FLOAT / RTK_FIXED |
| Dual antenna heading | No | Yes |
| ROS 2 Jazzy | Ported from ROS 1 | Native, from scratch |
| License | BSD-3 | Apache 2.0 (patent grant included) |

---

## Install

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

!!! tip "Headless / Raspberry Pi users"
    Add a `COLCON_IGNORE` file before building to skip the Gazebo package:
    ```bash
    touch ~/ros2_ws/src/fusioncore/fusioncore_gazebo/COLCON_IGNORE
    ```

---

## Quick start

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

That's the full stack — FusionCore + Nav2, lifecycle managed automatically.

**No Nav2?** Use `fusioncore.launch.py` instead:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

---

## Where to go next

- **New user** → [Getting Started](getting-started.md)
- **Configuring your robot** → [Configuration](configuration.md)
- **Using with Nav2** → [Nav2 Integration](nav2.md)
- **Coming from robot_localization** → [Migration Guide](migration_from_robot_localization.md)
- **Simulation / testing without hardware** → [Simulation](simulation.md)
- **What all the parameters mean** → [Configuration](configuration.md)
- **How the filter actually works** → [How It Works](how-it-works.md)
