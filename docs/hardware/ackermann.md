# Ackermann / car-like robots

For robots that steer like a car: F1/10 racers, full-size vans, golf carts, agricultural vehicles, anything with front-wheel steering. Unlike differential drive robots, these can't spin in place. They turn by angling the front wheels, which means the geometry of how the robot moves is different.

FusionCore has a built-in Ackermann motion model that accounts for this. It applies the same non-holonomic constraint as the DifferentialDrive model (the robot can't slide sideways) but is parameterized for car-like steering.

---

## Setting it up

Two things to set in your config:

```yaml
motion_model: "Ackermann"
motion_model_params.wheelbase: 0.32   # metres — see below
```

**Wheelbase** is the distance between the centre of your front axle and the centre of your rear axle, measured in metres. Get it wrong and the filter's motion predictions will be slightly off. Measure it physically on your robot rather than looking it up online — manufacturing tolerances vary.

| Platform | Typical wheelbase |
|---|---|
| F1/10 scale car (Traxxas Slash base) | ~0.32 m |
| F1/10 scale car (AckerBot, MIT Racecar) | ~0.32–0.34 m |
| Golf cart | ~1.8 m |
| Ford Transit full-size van | ~3.75 m |
| Mercedes Sprinter | ~3.67 m |
| Ford Transit Connect (compact van) | ~2.73 m |

---

## F1/10 indoor setup

**Sensors:** VESC wheel encoder + RealSense D435i IMU + optional VSLAM or LiDAR scan-matching

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/f1tenth_indoor.yaml \
  --ros-args \
  -r /odom/wheels:=/vesc/odom \
  -r /imu/data:=/camera/imu
```

If you're using KISS-ICP or RealSense T265 visual odometry as a second velocity source, uncomment `encoder2.topic` in `f1tenth_indoor.yaml`:

```yaml
encoder2.topic: "/kiss/odometry"         # KISS-ICP on LiDAR
# or
encoder2.topic: "/camera/odom/sample"    # RealSense T265 visual odometry
```

Note: `encoder2` requires a `nav_msgs/Odometry` topic with velocity in the `twist` field. slam_toolbox publishes a pose topic, not an odometry topic, so it won't work here directly.

The full config is at [`config/f1tenth_indoor.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/f1tenth_indoor.yaml).

---

## Van / large vehicle outdoor setup

**Sensors:** ZED 2i (IMU + visual odom) + wheel encoder or by-wire odometry + GPS

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/van_outdoor_gps.yaml \
  --ros-args \
  -r /odom/wheels:=/your/wheel/odom \
  -r /imu/data:=/zed/zed_node/imu/data \
  -r /gnss/fix:=/your/gps/fix
```

If you don't have wheel encoders yet, remap `/odom/wheels` to the ZED visual odometry and it becomes your only velocity source:

```bash
-r /odom/wheels:=/zed/zed_node/odom
```

Then set `encoder2.topic: ""` in the config (disable the second visual odom source since both would be the same topic).

The full config is at [`config/van_outdoor_gps.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/van_outdoor_gps.yaml).

---

## VESC gives zero covariance. Is that a problem?

No. FusionCore checks the covariance in every incoming odometry message. If it's zero or negative (which is what VESC outputs by default), FusionCore silently ignores the message covariance and uses the `encoder.vel_noise` and `encoder.yaw_noise` values from your config instead. You don't need to patch the VESC driver.

---

## Which IMU to use if you have two

FusionCore fuses one IMU per instance. If your platform has multiple (for example, both a VESC built-in IMU and a RealSense D435i), pick the better one and remap to it.

For most F1/10 setups: use the **RealSense D435i IMU**. It has a lower noise floor than the IMU in most VESC-based motor controllers, and its axes are more stable during the high angular rate turns that F1/10 racing involves.

```bash
-r /imu/data:=/camera/imu   # use D435i, not VESC
```

If you genuinely need both IMUs fused, run them through `imu_filter_madgwick` or `imu_tools/imu_complementary_filter` first to merge them into a single topic, then feed that to FusionCore.

---

## ZED 2i: one important config difference

The ZED SDK subtracts gravity from the IMU before publishing, which most IMU drivers don't do. If you use a ZED 2i, set this in your config:

```yaml
imu.remove_gravitational_acceleration: true
```

Without this, FusionCore will assume the IMU is publishing raw specific force (gravity included) and will try to handle gravity internally. Since it's already been removed by the ZED SDK, the filter ends up double-correcting and the state estimate goes wrong.

Quick check: look at `linear_acceleration.z` when the vehicle is stationary on flat ground. If it reads ~9.8 m/s², leave the flag false. If it reads ~0.0 m/s², set it true.

---

## Disabling TF broadcasting

By default FusionCore broadcasts the `odom -> base_link` transform. If another node already owns that transform, or you're running two FusionCore instances and only one should broadcast TF, set:

```yaml
publish.tf: false
```

`/fusion/odom` keeps publishing normally. Only the TF broadcast is suppressed.

---

## SLAM integration

Same as any other FusionCore setup:

```
map → odom        ← your SLAM system (slam_toolbox, RTABMAP, Cartographer)
odom → base_link  ← FusionCore
```

Just launch both. They don't conflict.
