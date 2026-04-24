# Migrating from robot_localization to FusionCore

This guide maps every `robot_localization` parameter and topic to its FusionCore equivalent. If you have a working `robot_localization` setup and want to switch, start here.

## Architecture difference

`robot_localization` is two nodes wired together:

```
/imu/data ──────────────────────────────────────────┐
/odom/wheels ──────→ ekf_node → /odometry/filtered  │
/odometry/filtered ─→ navsat_transform_node ─────────┘
/fix ──────────────→ navsat_transform_node → GPS in ENU → back to ekf_node
```

FusionCore is one node:

```
/imu/data ────┐
/odom/wheels ─┤→ fusioncore_node → /fusion/odom
/gnss/fix ────┘                  → odom → base_link TF
```

GPS is handled internally: no `navsat_transform_node`, no topic wiring, no feedback loop.

---

## Step 1: Remove robot_localization from your launch

Remove or comment out:
- `ekf_node` launch
- `navsat_transform_node` launch
- Any `robot_localization` parameter files loaded in your launch

Add:
```python
from launch_ros.actions import LifecycleNode

fusioncore_node = LifecycleNode(
    package="fusioncore_ros",
    executable="fusioncore_node",
    name="fusioncore",
    namespace="",
    output="screen",
    parameters=["/path/to/fusioncore.yaml"],
)
```

Or use the provided launch file directly:
```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/fusioncore.yaml
```

---

## Step 2: Map your EKF parameters

| robot_localization (ekf_node) | FusionCore | Notes |
|---|---|---|
| `frequency` | `publish_rate` | |
| `sensor_timeout` | automatic | FC detects stale sensors per-sensor automatically |
| `two_d_mode: true` | `publish.force_2d: true` | |
| `transform_time_offset` | automatic | FC handles timing internally |
| `transform_timeout` | automatic | |
| `predict_to_current_time` | always on | FC always predicts to current time |
| `smooth_lagged_data` | automatic | FC replays buffered IMU for delayed GPS |
| `history_length` | automatic | FC uses a 1-second IMU ring buffer |
| `imu0` (topic name) | remap `/imu/data` | FC subscribes to `/imu/data` by default |
| `odom0` (topic name) | remap `/odom/wheels` | FC subscribes to `/odom/wheels` by default |
| `imu0_config` | not needed | FC fuses all available IMU axes automatically |
| `odom0_config` | not needed | FC fuses linear velocity and yaw rate from odometry |
| `imu0_differential` | not needed | FC handles this internally |
| `imu0_relative` | not needed | |
| `imu0_queue_size` | not needed | |
| `imu0_remove_gravitational_acceleration` | `imu.remove_gravitational_acceleration` | ⚠️ **logic is inverted**: see note below |
| `mahalanobis_threshold` | `outlier_threshold_gnss`, `outlier_threshold_imu`, `outlier_threshold_enc`, `outlier_threshold_hdg` | FC has per-sensor thresholds instead of one global value |
| `process_noise_covariance` | `ukf.q_position`, `ukf.q_velocity`, etc. | FC exposes named scalars instead of a 15×15 matrix |
| `initial_estimate_covariance` | not configurable | FC initializes automatically from first sensor readings |
| `print_diagnostics` | always on | FC publishes to `/diagnostics` at 1Hz |

### ⚠️ Gravity removal flag is inverted

```yaml
# robot_localization:
# true = "remove gravity from the raw IMU reading before fusing"
imu0_remove_gravitational_acceleration: true

# FusionCore:
# true = "my driver already removed gravity, add it back before fusing"
imu.remove_gravitational_acceleration: true
```

Both flags with value `true` describe the same physical situation, but from opposite directions. If you had `imu0_remove_gravitational_acceleration: true` in robot_localization, set `imu.remove_gravitational_acceleration: true` in FusionCore. When in doubt: check `linear_acceleration.z` at rest. If it reads `~9.8`, set false. If it reads `~0.0`, set true.

---

## Step 3: Map your navsat_transform parameters

| robot_localization (navsat_transform_node) | FusionCore | Notes |
|---|---|---|
| `frequency` | `publish_rate` | |
| `delay` | not needed | FC handles timing automatically |
| `magnetic_declination_radians` | not needed | FC uses ECEF (true north), not magnetic north |
| `yaw_offset` | not needed | |
| `zero_altitude` | `publish.force_2d: true` | |
| `broadcast_utm_transform` | not applicable | FC fuses in ECEF, outputs local ENU |
| `broadcast_utm_transform_as_parent_frame` | not applicable | |
| `publish_filtered_gps` | not available | not currently supported |
| `use_odometry_yaw` | automatic | FC initializes heading from motion automatically |
| `wait_for_datum` | `reference.use_first_fix: true` | true = anchor to first fix (default); false = set reference.x/y/z manually |
| `datum` | `reference.x`, `reference.y`, `reference.z` | set in your output CRS coordinates |

---

## Step 4: Remap topics

| robot_localization | FusionCore | Action |
|---|---|---|
| `imu0: /imu/data` | subscribes to `/imu/data` | no change needed if your driver uses `/imu/data` |
| `odom0: /your/wheel/odom` | subscribes to `/odom/wheels` | remap: `-r /odom/wheels:=/your/wheel/odom` |
| navsat input: `/fix` | subscribes to `/gnss/fix` | remap: `-r /gnss/fix:=/fix` |
| publishes: `/odometry/filtered` | publishes: `/fusion/odom` | update nav2_params.yaml and any downstream subscribers |
| publishes: `odom → base_link` TF | same | no change |

### Applying remaps in your launch file

```python
Node(
    package="fusioncore_ros",
    executable="fusioncore_node",
    remappings=[
        ("/odom/wheels", "/husky_velocity_controller/odom"),  # example
        ("/gnss/fix",    "/fix"),
    ],
)
```

Or on the command line:
```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=your_config.yaml \
  --ros-args \
  -r /odom/wheels:=/husky_velocity_controller/odom \
  -r /gnss/fix:=/fix
```

---

## Step 5: Update nav2_params.yaml

```yaml
# Before (robot_localization)
amcl:
  ros__parameters:
    odom_topic: /odometry/filtered

bt_navigator:
  ros__parameters:
    odom_topic: /odometry/filtered

velocity_smoother:
  ros__parameters:
    odom_topic: /odometry/filtered

# After (FusionCore)
amcl:
  ros__parameters:
    odom_topic: /fusion/odom

bt_navigator:
  ros__parameters:
    odom_topic: /fusion/odom

velocity_smoother:
  ros__parameters:
    odom_topic: /fusion/odom
```

---

## What FusionCore does automatically (no config needed)

These required manual configuration in robot_localization:

| Task | robot_localization | FusionCore |
|---|---|---|
| IMU frame → base_link transform | specify `imu0_config` axes manually | looks up TF tree automatically |
| GPS antenna offset (lever arm) | ignored | `gnss.lever_arm_x/y/z` applied per-fix |
| Sensor noise tuning | manual `process_noise_covariance` matrix | adaptive noise from innovation sequence |
| Zero-velocity updates (ZUPT) | not built-in | auto when encoder + UKF angular rate below threshold |
| IMU bias estimation | not built-in | gyro + accel bias states in the 22D state vector |
| GPS quality gating | not built-in | `gnss.max_hdop`, `gnss.min_satellites`, `gnss.min_fix_type` |
| Delayed GPS fusion | `smooth_lagged_data` + `history_length` | IMU ring buffer replay, always on |

---

## What robot_localization has that FusionCore doesn't

Be aware of these before migrating:

| Feature | robot_localization | FusionCore |
|---|---|---|
| Multiple independent odometry sources | yes (odom0, odom1, ...) | primary wheel odom + one secondary via `encoder2.topic` |
| Configurable state vector | yes (per-sensor config booleans) | fixed 22D state (position, orientation, velocity, biases) |
| Arbitrary sensor plugins | yes (extensible) | IMU, wheel odometry, GPS only |
| Published filtered GPS | `publish_filtered_gps: true` | not currently supported |
| navsat datum from ROS service | `/datum` service | not currently supported |

---

## Minimal working FusionCore config

Starting point: copy this and adjust noise values to your hardware:

```yaml
fusioncore:
  ros__parameters:
    base_frame: base_link
    odom_frame: odom
    publish_rate: 100.0
    publish.force_2d: true

    imu.has_magnetometer: false
    imu.gyro_noise: 0.005
    imu.accel_noise: 0.1
    imu.remove_gravitational_acceleration: false

    encoder.vel_noise: 0.05
    encoder.yaw_noise: 0.02

    gnss.base_noise_xy: 2.5
    gnss.base_noise_z: 5.0
    gnss.max_hdop: 4.0
    gnss.min_satellites: 4
    gnss.min_fix_type: 1

    outlier_rejection: true
    outlier_threshold_gnss: 16.27
    outlier_threshold_imu: 15.09
    outlier_threshold_enc: 11.34
    outlier_threshold_hdg: 10.83

    adaptive.imu: true
    adaptive.encoder: true
    adaptive.gnss: true
    adaptive.window: 50
    adaptive.alpha: 0.01

    ukf.q_position: 0.01
    ukf.q_orientation: 1.0e-9
    ukf.q_velocity: 0.1
    ukf.q_angular_vel: 0.1
    ukf.q_acceleration: 1.0
    ukf.q_gyro_bias: 1.0e-5
    ukf.q_accel_bias: 1.0e-5

    input.gnss_crs: "EPSG:4326"
    output.crs: "EPSG:4978"
    output.convert_to_enu_at_reference: true
    reference.use_first_fix: true
```

For hardware-specific noise values, see the configs in `fusioncore_ros/config/`:
- [`clearpath_husky.yaml`](../fusioncore_ros/config/clearpath_husky.yaml): Husky A200 + Microstrain GX5-25 + u-blox F9P
- [`bno085_custom.yaml`](../fusioncore_ros/config/bno085_custom.yaml): BNO085 + standard GPS (DIY robots)
- [`duatic_mecanum.yaml`](../fusioncore_ros/config/duatic_mecanum.yaml): Duatic mecanum + BNO085, no GPS

Or combine a hardware config with an environment preset for GPS-quality-specific tuning:
- [`env_open.yaml`](../fusioncore_ros/config/env_open.yaml): agricultural / open sky
- [`env_urban.yaml`](../fusioncore_ros/config/env_urban.yaml): urban / multipath
- [`env_canopy.yaml`](../fusioncore_ros/config/env_canopy.yaml): forest / partial sky

Questions? Open a [GitHub Discussion](https://github.com/manankharwar/fusioncore/discussions).
