# Topics & TF

## Subscribes to

| Topic | Type | Notes |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | IMU angular velocity and linear acceleration |
| `/odom/wheels` | `nav_msgs/Odometry` | Wheel encoder velocity |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | GPS position (optional) |
| `/gnss/heading` | `sensor_msgs/Imu` | Dual antenna heading (optional) |
| `gnss.azimuth_topic` | `compass_msgs/Azimuth` | Azimuth heading (optional, preferred) |
| `gnss.fix2_topic` | `sensor_msgs/NavSatFix` | Second GPS receiver (optional) |
| `encoder2.topic` | `nav_msgs/Odometry` | Second velocity source: lidar odom, visual odom (optional) |

Default topic names can be changed with ROS 2 remaps:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=your_robot.yaml \
  --ros-args \
  -r /odom/wheels:=/your/wheel/odom \
  -r /gnss/fix:=/fix
```

---

## Publishes

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/fusion/odom` | `nav_msgs/Odometry` | 100 Hz | Fused position + orientation + velocity + full covariance |
| `/fusion/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 100 Hz | Same pose: compatible with AMCL, slam_toolbox, Nav2 pose initializer |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | Per-sensor health, outlier counts, heading status |
| `/tf` | TF | 100 Hz | `odom → base_link` |

---

## Services

| Service | Type | What it does |
|---|---|---|
| `~/reset` | `std_srvs/Trigger` | Re-initializes the filter and clears GPS reference. No restart needed. |

---

## TF tree

FusionCore publishes:

```
map → odom    (when GPS is active and reference.use_first_fix: true)
odom → base_link    (always, at publish_rate)
```

Your URDF provides `base_link → imu_link` and other sensor transforms.

For indoor navigation without GPS, FusionCore only publishes `odom → base_link`. A separate SLAM node (slam_toolbox, AMCL) provides `map → odom`.
