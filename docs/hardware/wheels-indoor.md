# Indoor robots (IMU + wheel odometry, no GPS)

For any ground robot running indoors or in GPS-denied environments with wheel encoders as the primary motion source.

---

## Quick start

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/wheels_indoor.yaml \
  --ros-args \
  -r /odom/wheels:=/your/wheel/odom/topic
```

Common wheel odom topic remaps:

| Robot / controller | Topic | Remap |
|---|---|---|
| ROS 2 Control diff_drive | `/diff_controller/odom` | `-r /odom/wheels:=/diff_controller/odom` |
| Turtlebot3 | `/odom` | `-r /odom/wheels:=/odom` |
| Nav2 default | `/odom` | `-r /odom/wheels:=/odom` |
| Already `/odom/wheels` |: | no remap needed |

---

## What FusionCore gives you indoors

Without GPS, FusionCore gives you **clean local odometry**: not global position. It starts at the origin and builds from there. What it adds over raw wheel odom:

- IMU gyro + accel bias estimated continuously as filter states → less heading drift
- Chi-squared outlier rejection on wheel odom → encoder spikes don't corrupt the state
- ZUPT (zero-velocity updates) → drift suppressed while stationary
- Adaptive noise → covariance tracks actual sensor behavior, not guessed config values

For global positioning and loop closure, pair with a SLAM system.

---

## SLAM integration

FusionCore and SLAM divide the TF tree cleanly:

```
map → odom        ← slam_toolbox or RTABMAP
odom → base_link  ← FusionCore
```

```bash
# FusionCore (odometry)
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=wheels_indoor.yaml \
  --ros-args -r /odom/wheels:=/diff_controller/odom

# SLAM
ros2 launch slam_toolbox online_async_launch.py
```

---

## Adding LiDAR ICP as a second velocity source

If your wheel odom is unreliable (slippery floors, wheel slip) or you want cross-validation, add LiDAR ICP alongside wheel odom. Uncomment in `wheels_indoor.yaml`:

```yaml
encoder2.topic: "/kiss/odometry"    # KISS-ICP
# encoder2.topic: "/icp_odom"       # rtabmap icp_odometry
```

FusionCore fuses both independently. If wheel odom has a spike, the chi-squared gate rejects it: ICP keeps the estimate grounded.

If you have **no wheel odom at all** and only LiDAR ICP, use [`icp_indoor.yaml`](icp-indoor.md) instead.

---

## One thing to get right: startup bias init

Indoors, without GPS corrections, IMU bias matters. `wheels_indoor.yaml` has this enabled by default:

```yaml
init.stationary_window: 2.0
```

Keeps the robot still for 2 seconds at launch, estimates accelerometer bias from the mean readings, then starts. Reduces startup drift from ~10 cm to under 1 cm. Falls back automatically if the robot moves during the window.
