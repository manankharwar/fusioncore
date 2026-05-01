# Migrating from robot_localization to FusionCore

If you're coming from `robot_localization` and want to switch, this guide walks you through it step by step: no prior knowledge of FusionCore required.

---

## Three files, one command

Everything you need is already in this repo. Here's the big picture before we dive in:

| File | What it does |
|---|---|
| `fusioncore_ros/config/fusioncore.yaml` (or your hardware config) | Tells FusionCore about your robot: IMU noise, GPS quality, sensor offsets |
| `fusioncore_ros/config/nav2_params.yaml` | A ready-to-use Nav2 config pre-wired to FusionCore's output (if you use Nav2) |
| `fusioncore_ros/launch/fusioncore_nav2.launch.py` | One launch file that starts both FusionCore and Nav2 together |

Once you have your robot config set up, the entire stack starts with:

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

That's it. FusionCore starts, configures itself, activates, and Nav2 comes up 5 seconds later once localization is publishing. No separate `navsat_transform_node`, no feedback loop, no topic wiring.

**Don't use Nav2?** Use `fusioncore.launch.py` instead:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

---

## What's changing (and what isn't)

With `robot_localization`, you wired two nodes together:

```
/imu/data ──────────────────────────────────────────────────┐
/odom/wheels ──────→ ekf_node → /odometry/filtered          │
/odometry/filtered ─→ navsat_transform_node → GPS in ENU ───┘
/fix ──────────────→ navsat_transform_node
```

Two nodes, two config files, feedback loop between them, lots of parameter tuning.

With FusionCore, it's one node:

```
/imu/data ────┐
/odom/wheels ─┤→ fusioncore_node → /fusion/odom
/gnss/fix ────┘                  → TF: odom → base_link
```

GPS fusion happens inside: no `navsat_transform_node`, no feedback loop. The output topic changes from `/odometry/filtered` to `/fusion/odom`.

**What stays the same:**
- Your robot still publishes `/imu/data`, `/odom/wheels`, `/gnss/fix` (you may need a remap: see Step 4)
- Nav2 still works the same way, it just reads from `/fusion/odom` instead of `/odometry/filtered`
- The `odom → base_link` TF is still published the same way

---

## Step 1: Remove robot_localization from your launch

In your launch file, remove or comment out:
- The `ekf_node` launch
- The `navsat_transform_node` launch
- Any parameter files you were loading for those nodes

You don't need to replace them with anything complex. Either use the provided launch file (recommended):

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

Or if you have your own launch file, add FusionCore as a lifecycle node:

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

FusionCore is a lifecycle node, so you need to configure and activate it after launch. The provided `fusioncore_nav2.launch.py` does this automatically (2-second delay, then configure → activate). If you're writing your own launch file, see the launch file source at `fusioncore_ros/launch/fusioncore_nav2.launch.py` for how to wire the lifecycle transitions.

---

## Step 2: Create your robot config

FusionCore needs one YAML file that describes your hardware. You can start from a ready-made config for your platform:

- [`config/clearpath_husky.yaml`](../fusioncore_ros/config/clearpath_husky.yaml): Husky A200 + Microstrain GX5-25 + u-blox F9P
- [`config/bno085_custom.yaml`](../fusioncore_ros/config/bno085_custom.yaml): BNO085 + standard GPS (DIY robots)
- [`config/duatic_mecanum.yaml`](../fusioncore_ros/config/duatic_mecanum.yaml): Duatic mecanum + BNO085, no GPS

Or start from scratch with this minimal template:

```yaml
fusioncore:
  ros__parameters:
    base_frame: base_link    # must match your robot's base TF frame
    odom_frame: odom
    publish_rate: 100.0
    publish.force_2d: true   # set true for ground robots

    imu.has_magnetometer: false
    imu.gyro_noise: 0.005    # rad/s: check your IMU datasheet
    imu.accel_noise: 0.1     # m/s²
    imu.remove_gravitational_acceleration: false  # see gravity note below

    encoder.vel_noise: 0.05  # m/s
    encoder.yaw_noise: 0.02  # rad/s

    gnss.base_noise_xy: 2.5  # m: standard GPS. Use 0.5 for RTK float, 0.015 for RTK fixed
    gnss.base_noise_z: 5.0
    gnss.max_hdop: 4.0
    gnss.min_satellites: 4
    gnss.min_fix_type: 1     # 1=GPS, 2=DGPS/RTK_FIXED, 4=RTK_FIXED only
                             # WARNING: sensor_msgs/NavSatFix has no RTK_FLOAT status.
                             # Setting min_fix_type: 3 will silently starve the filter. Use 2 or 4.

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
    reference.use_first_fix: true   # map origin = first GPS fix
```

**GPS quality presets**: if you want to override GPS noise thresholds for your environment without editing the robot config, you can layer an environment preset on top:

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=your_robot.yaml \
  env_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/env_urban.yaml
```

Available presets: [`env_open.yaml`](../fusioncore_ros/config/env_open.yaml), [`env_urban.yaml`](../fusioncore_ros/config/env_urban.yaml), [`env_canopy.yaml`](../fusioncore_ros/config/env_canopy.yaml).

### Gravity removal: one thing to get right

There's one parameter where the naming is confusing:

```yaml
# robot_localization:
imu0_remove_gravitational_acceleration: true
# means: "remove gravity before I fuse it"

# FusionCore:
imu.remove_gravitational_acceleration: true
# means: "my driver already removed gravity, add it back"
```

The names look the same but they mean opposite things. **Both set to `true` describe the same physical situation.** Rule of thumb: check `linear_acceleration.z` at rest. If it reads `~9.8 m/s²`, set `false`. If it reads `~0.0`, set `true`.

---

## Step 3: Nav2 setup (if you use Nav2)

This is the most common pain point when migrating: all the places in Nav2 that referenced `/odometry/filtered` need to point to `/fusion/odom`.

**The easy path:** use the bundled `nav2_params.yaml` at `fusioncore_ros/config/nav2_params.yaml`. It comes pre-wired to `/fusion/odom` and is configured for outdoor GPS navigation (differential drive, Regulated Pure Pursuit, no AMCL). The `fusioncore_nav2.launch.py` launch file uses it by default.

```bash
# This already uses the bundled nav2_params.yaml: nothing else needed
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=your_robot.yaml
```

**Customizing:** if you have your own `nav2_params.yaml`, pass it as an argument:

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=your_robot.yaml \
  nav2_params:=/path/to/your_nav2_params.yaml
```

**Updating your existing Nav2 config manually:** if you want to keep your own nav2_params.yaml, find and replace every instance of `/odometry/filtered` with `/fusion/odom`. The three places that need it:

```yaml
bt_navigator:
  ros__parameters:
    odom_topic: /fusion/odom        # was /odometry/filtered

velocity_smoother:
  ros__parameters:
    odom_topic: /fusion/odom        # was /odometry/filtered

# if you had AMCL, remove it: FusionCore handles global localization via GPS
```

Also remove AMCL if you had it. FusionCore publishes `odom → base_link` TF and a `/fromLL` service for GPS waypoint navigation. AMCL publishes `map → odom` for map-based localization, which is not needed when GPS provides global positioning and will conflict with the TF tree.

---

## Step 4: Remap your topics (if your robot uses non-default names)

FusionCore subscribes to these topics by default:

| Input | Default topic | Your topic | Remap needed? |
|---|---|---|---|
| IMU | `/imu/data` | probably the same | usually no |
| Wheel odometry | `/odom/wheels` | often different | likely yes |
| GPS fix | `/gnss/fix` | often `/fix` | often yes |

If your wheel odom topic or GPS topic is different, add remaps to your launch command:

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=your_robot.yaml \
  --ros-args \
  -r /odom/wheels:=/your/wheel/odom/topic \
  -r /gnss/fix:=/fix
```

**Clearpath Husky example** (odom at `/husky_velocity_controller/odom`, GPS at `/fix`):

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/clearpath_husky.yaml \
  --ros-args \
  -r /odom/wheels:=/husky_velocity_controller/odom \
  -r /gnss/fix:=/fix
```

FusionCore publishes to `/fusion/odom`. Any downstream node that was reading `/odometry/filtered` needs to be updated to read `/fusion/odom`: the bundled nav2_params.yaml already handles this for Nav2.

---

## Full parameter mapping reference

### EKF parameters

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
| `imu0_remove_gravitational_acceleration` | `imu.remove_gravitational_acceleration` | Logic is inverted: see Step 2 above |
| `odomN_pose_rejection_threshold`, `odomN_twist_rejection_threshold` | `outlier_threshold_gnss/imu/enc/hdg` | Both use Mahalanobis distance. RL exposes raw scalar thresholds with no guidance on chi-squared calibration per DOF: you set them manually. FC defaults are chi-squared critical values pre-calibrated to each sensor's measurement DOF. |
| `process_noise_covariance` | `ukf.q_position`, `ukf.q_velocity`, etc. | FC exposes named scalars instead of a 15×15 matrix |
| `initial_estimate_covariance` | not configurable | FC initializes automatically from first sensor readings |
| `print_diagnostics` | always on | FC publishes to `/diagnostics` at 1 Hz |

### navsat_transform parameters

| robot_localization (navsat_transform_node) | FusionCore | Notes |
|---|---|---|
| `frequency` | `publish_rate` | |
| `delay` | not needed | FC handles timing automatically |
| `magnetic_declination_radians` | not needed | FC uses ECEF (true north), not magnetic north |
| `yaw_offset` | not needed | |
| `zero_altitude` | `publish.force_2d: true` | |
| `broadcast_utm_transform` | not applicable | FC fuses in ECEF, outputs local ENU |
| `publish_filtered_gps` | not available | not currently supported |
| `use_odometry_yaw` | automatic | FC initializes heading from motion automatically |
| `wait_for_datum` | `reference.use_first_fix: true` | true = anchor to first fix (default) |
| `datum` | `reference.x`, `reference.y`, `reference.z` | set in your output CRS coordinates |

---

## What FusionCore handles automatically

These required manual config in robot_localization:

| Task | robot_localization | FusionCore |
|---|---|---|
| IMU frame → base_link transform | specify `imu0_config` axes manually | looks up TF tree automatically |
| GPS antenna offset (lever arm) | ignored | `gnss.lever_arm_x/y/z` applied per-fix |
| Sensor noise tuning | fixed config values | adaptive noise from innovation sequence |
| Zero-velocity updates (ZUPT) | not built-in | auto when encoder + UKF angular rate below threshold |
| IMU bias estimation | not built-in | gyro + accel bias states in the 22D state vector |
| GPS fix quality pre-filter | not built-in | `gnss.max_hdop`, `gnss.min_satellites`, `gnss.min_fix_type` |
| Measurement rejection | Mahalanobis threshold (per sensor, scalar) | Chi-squared gate (per sensor, calibrated to sensor DOF) |
| Delayed GPS fusion | `smooth_lagged_data` + `history_length` | IMU ring buffer replay, always on |

---

## What robot_localization has that FusionCore doesn't

Know these before you migrate:

| Feature | robot_localization | FusionCore |
|---|---|---|
| Multiple independent odometry sources | yes (odom0, odom1, ...) | primary wheel odom + one secondary via `encoder2.topic` |
| Configurable state vector | yes (per-sensor config booleans) | fixed 22D state (position, orientation, velocity, biases) |
| Arbitrary sensor plugins | yes (extensible) | IMU, wheel odometry, GPS only |
| Published filtered GPS | `publish_filtered_gps: true` | not currently supported |
| navsat datum from ROS service | `/datum` service | not currently supported |

---

Questions? Open a [GitHub Discussion](https://github.com/manankharwar/fusioncore/discussions).
