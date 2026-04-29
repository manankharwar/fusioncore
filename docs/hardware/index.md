# Hardware Configs

## Which setup are you?

Pick the row that matches your sensors. Everything else follows from there.

| My sensors | Use case | Start here |
|---|---|---|
| IMU + wheel odometry | Indoor, no GPS: warehouse, home robot, lab | [`wheels_indoor.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/wheels_indoor.yaml) |
| IMU + LiDAR ICP | Indoor, no wheel odom: using KISS-ICP or rtabmap | [`icp_indoor.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/icp_indoor.yaml) |
| IMU + wheel odom + LiDAR ICP | Indoor, both velocity sources for redundancy | [`wheels_indoor.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/wheels_indoor.yaml) + uncomment `encoder2.topic` |
| IMU + wheel odom + GPS | Outdoor, field robots, GPS navigation | [`bno085_custom.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/bno085_custom.yaml) or [`clearpath_husky.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/clearpath_husky.yaml) |

**Variations within each setup: one-line changes, not separate configs:**

| Variation | Parameter to change |
|---|---|
| Standard GPS → RTK float | `gnss.base_noise_xy: 0.5`, `gnss.min_fix_type: 3` |
| Standard GPS → RTK fixed | `gnss.base_noise_xy: 0.015`, `gnss.min_fix_type: 4` |
| 6-axis IMU → 9-axis (BNO085, VectorNav) | `imu.has_magnetometer: true` |
| Add dual GPS antenna heading | `gnss.heading_topic: "/gnss/heading"` |
| Add Nav2 | Use `fusioncore_nav2.launch.py` instead of `fusioncore.launch.py` |
| Urban vs open sky GPS noise | Layer an [environment preset](environment-presets.md) on top |

---

## All config files

| Config | Setup | Platform | IMU | GPS |
|---|---|---|---|---|
| [`wheels_indoor.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/wheels_indoor.yaml) | IMU + wheels | Any differential drive (generic) | Any 6-axis MEMS | none |
| [`icp_indoor.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/icp_indoor.yaml) | IMU + LiDAR ICP | Any indoor robot with LiDAR | Any 6-axis MEMS | none |
| [`bno085_custom.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/bno085_custom.yaml) | IMU + wheels + GPS | Any differential drive (DIY) | Bosch BNO085 | u-blox M8N class |
| [`clearpath_husky.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/clearpath_husky.yaml) | IMU + wheels + GPS | Clearpath Husky A200 | Microstrain 3DM-GX5-25 | u-blox F9P |
| [`duatic_mecanum.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/duatic_mecanum.yaml) | IMU + wheels | Duatic mecanum platform | BNO085 | none |

---

## Using a config

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/wheels_indoor.yaml
```

Add remaps if your topics differ from the defaults:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/wheels_indoor.yaml \
  --ros-args \
  -r /odom/wheels:=/diff_controller/odom \
  -r /imu/data:=/imu
```

---

## Indoor / GPS-denied robots

See [Indoor / LiDAR ICP](icp-indoor.md) for detailed setup instructions: KISS-ICP, rtabmap, OAK-D IMU remaps, SLAM integration.

---

## Layering environment presets

See [Environment Presets](environment-presets.md) to tune GPS noise for your operating conditions without editing the hardware config.

---

## Don't see your hardware?

Open a [Hardware Config Request](https://github.com/manankharwar/fusioncore/issues/new/choose) with your sensor datasheets and the community will help put one together.
