# Indoor / GPS-denied robots (LiDAR ICP odometry)

For robots operating indoors, underground, or anywhere GPS isn't available. FusionCore runs fine without GPS: it starts at the origin and builds a local odometry frame from IMU + LiDAR ICP alone.

---

## Common setups this covers

- Indoor mobile robots (warehouses, offices, labs)
- Robots with a LiDAR running KISS-ICP or rtabmap `icp_odometry`
- Robots where wheel odometry is unreliable or unavailable
- OAK-D camera + Velodyne/RPLiDAR + any ground robot

---

## Quick start

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/icp_indoor.yaml \
  --ros-args \
  -r /odom/wheels:=/kiss/odometry
```

Replace `/kiss/odometry` with whatever your ICP pipeline publishes:

| ICP pipeline | Default topic |
|---|---|
| KISS-ICP | `/kiss/odometry` |
| rtabmap `icp_odometry` | `/icp_odom` |
| cartographer | `/tracked_pose` (pose only: needs twist wrapper) |

FusionCore treats LiDAR ICP odometry exactly like wheel odometry: same outlier gate, same adaptive noise, same ZUPT logic. The only difference is the noise values: ICP is more accurate than wheels so `icp_indoor.yaml` uses tighter defaults (`encoder.vel_noise: 0.02` instead of `0.05`).

---

## If you also have wheel odometry

If your wheel odom is reliable, use it as primary and add ICP as a second cross-check via `encoder2.topic`:

```yaml
# in icp_indoor.yaml
encoder2.topic: "/kiss/odometry"
```

Then launch without the remap: FusionCore subscribes to `/odom/wheels` natively and picks up ICP as a second velocity source.

---

## SLAM integration (slam_toolbox, RTABMAP)

FusionCore and your SLAM system divide the TF tree cleanly:

```
map → odom        ← your SLAM system (slam_toolbox, RTABMAP)
odom → base_link  ← FusionCore
```

They don't conflict. Just launch both:

```bash
# Terminal 1: FusionCore
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=icp_indoor.yaml \
  --ros-args -r /odom/wheels:=/kiss/odometry

# Terminal 2: slam_toolbox
ros2 launch slam_toolbox online_async_launch.py

# or RTABMAP
ros2 launch rtabmap_slam rtabmap.launch.py
```

Nav2 reads from `/fusion/odom` as usual.

---

## IMU topic remaps

Most drivers publish at `/imu/data` already and need no remap. Exceptions:

| Driver | Topic | Remap needed |
|---|---|---|
| OAK-D (depthai-ros) | `/imu` | `-r /imu/data:=/imu` |
| Realsense D435i | `/camera/imu` | `-r /imu/data:=/camera/imu` |
| Most others | `/imu/data` | none |

Also set `imu.frame_id` in your config if your driver doesn't fill in `header.frame_id`:

```yaml
imu.frame_id: "oak_imu_frame"      # OAK-D
imu.frame_id: "camera_imu_optical_frame"  # Realsense
```

---

## One thing to get right: startup bias init

Without GPS corrections, IMU bias matters more indoors. Add this to your config:

```yaml
init.stationary_window: 2.0
```

This collects 2 seconds of stationary IMU data at startup to estimate bias before the filter starts. Reduces startup drift from ~10 cm to under 1 cm. Robot must be stationary for the first 2 seconds after launch: it falls back automatically if it detects movement.

The `icp_indoor.yaml` config has this enabled by default.
