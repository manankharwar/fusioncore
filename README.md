# FusionCore

**ROS 2 sensor fusion SDK. UKF, 3D native, proper GNSS, zero manual tuning. Apache 2.0.**

Replaces robot_localization for ROS 2 Jazzy.

---

## Why FusionCore exists

In December 2024, Stefan at Husarion posted on ROS Discourse:

> "robot_localization is no longer being developed... the lack of open-source examples and out-of-the-box GNSS support is concerning."

27 months after the deprecation announcement, no accessible replacement existed. FusionCore is that replacement.

---

## What it does differently

| | robot_localization | Fuse | FusionCore |
|---|---|---|---|
| Core filter | EKF | Factor graph | UKF |
| 3D state space | Partial | PR open 12+ months | Native |
| IMU bias estimation | None | Complex config | Automatic |
| Noise covariance | Manual YAML | Manual | Automatic |
| GNSS fusion | UTM hack | Not implemented | ECEF + quality-aware |
| Dual antenna heading | Workaround | Not supported | Native |
| Maintenance | Deprecated | Active but slow | Active |
| License | BSD-3 | BSD-3 | Apache 2.0 |

---

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/fusioncore.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fusioncore_core fusioncore_ros
source install/setup.bash
```

---

## Stefan Configuration - Husarion Panther outdoor robot

GNSS + IMU + wheel encoders, ROS 2 Jazzy, dual antenna heading:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  config:=config/outdoor_gnss_dual_antenna.yaml
```

Topics subscribed:
- `/imu/data` - sensor_msgs/Imu
- `/odom/wheels` - nav_msgs/Odometry
- `/gnss/fix` - sensor_msgs/NavSatFix

Topics published:
- `/fusion/odom` - nav_msgs/Odometry at 100Hz
- TF: odom -> base_link

---

## Configuration

No covariance matrices. One YAML file:

```yaml
base_frame: base_link
odom_frame: odom
publish_rate: 100.0

imu:
  gyro_noise: 0.005
  accel_noise: 0.1

encoder:
  vel_noise: 0.05
  yaw_noise: 0.02

gnss:
  base_noise_xy: 1.0
  max_hdop: 4.0
  min_satellites: 4
```

---

## Technical details

- **Filter:** Unscented Kalman Filter - sigma points, no Jacobians
- **State vector:** 21-dimensional - position, orientation, velocity, angular velocity, acceleration, gyroscope bias, accelerometer bias
- **GNSS:** ECEF coordinates - no UTM zone boundaries
- **Bias estimation:** Online, automatic
- **Quality-aware GNSS:** Noise scaled by HDOP/VDOP automatically
- **Pure C++ core:** fusioncore_core has zero ROS dependency

---

## License

Apache 2.0 - commercially safe. Includes explicit patent license grant.

---

## Status

Active development. Issues answered within 24 hours.

If you are working on sensor fusion for a wheeled robot and hit problems - open an issue. This project exists because of community threads. Feedback drives what gets built next.
