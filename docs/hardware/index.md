# Hardware Configs

Ready-made configs live in [`fusioncore_ros/config/`](https://github.com/manankharwar/fusioncore/tree/main/fusioncore_ros/config/). Each one has noise values pulled from datasheets with comments explaining every parameter.

| Config | Platform | IMU | GPS |
|---|---|---|---|
| [`clearpath_husky.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/clearpath_husky.yaml) | Clearpath Husky A200 | Microstrain 3DM-GX5-25 | u-blox F9P |
| [`bno085_custom.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/bno085_custom.yaml) | Any differential drive (DIY) | Bosch BNO085 | u-blox M8N class |
| [`duatic_mecanum.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/duatic_mecanum.yaml) | Duatic mecanum manipulator | BNO085 | none |
| [`icp_indoor.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/icp_indoor.yaml) | Any indoor robot (LiDAR ICP) | Any 6-axis MEMS | none |

Use one as your starting point, then adjust noise values to your specific hardware.

---

## Using a hardware config

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/clearpath_husky.yaml
```

---

## Clearpath Husky A200

IMU: Microstrain 3DM-GX5-25 (6-axis, no magnetometer). GPS: u-blox F9P.

The Husky publishes wheel odometry and GPS on non-default topics. Add remaps:

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/clearpath_husky.yaml \
  --ros-args \
  -r /odom/wheels:=/husky_velocity_controller/odom \
  -r /gnss/fix:=/fix
```

---

## DIY robots (BNO085)

The BNO085 is a 9-axis AHRS that outputs full orientation. Set `imu.has_magnetometer: true` in the config: this tells FusionCore to trust the orientation output directly rather than integrating gyro only.

---

## Don't see your hardware?

Open a [Hardware Config Request](https://github.com/manankharwar/fusioncore/issues/new/choose) with your sensor datasheets and the community will help put one together.

---

## Indoor / GPS-denied robots

See [Indoor / LiDAR ICP](icp-indoor.md) for robots running indoors with KISS-ICP or rtabmap odometry and no GPS.

---

## Layering environment presets

See [Environment Presets](environment-presets.md) to tune GPS noise for your operating conditions without editing the hardware config.
