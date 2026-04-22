<p align="center">
  <img src="https://github.com/user-attachments/assets/f03a5458-761f-4bce-9e17-cc383bdcd57a" width="700">
</p>

# FusionCore

**ROS 2 sensor fusion SDK. Combines IMU, wheel encoders, and GPS into one reliable position estimate. Self-tuning noise covariance. Apache 2.0.**

---

## What problem does this solve?

Every mobile robot needs to know where it is. It gets this from multiple sensors: IMU, wheel encoders, GPS: each of which is imperfect in its own way. IMUs drift. Wheels slip. GPS jumps. You need software that intelligently combines all three into one trustworthy position estimate.

That software is called a sensor fusion package. The standard one for ROS, `robot_localization`, was officially deprecated in September 2023. Its designated replacement (`fuse`) has incomplete GPS support with no ECEF handling or RTK quality gating as of early 2026. At ROSCon UK 2025 the official workshop was still teaching both tools because no clear accessible replacement existed.

FusionCore is that replacement.

---

## Benchmark results

FusionCore vs robot_localization on the [NCLT dataset](http://robots.engin.umich.edu/nclt/) (University of Michigan): same IMU + wheel odometry + GPS, no tuning advantage:

| Filter | ATE RMSE | GPS spike response | Stability |
|--------|----------|--------------------|-----------|
| **FusionCore** | **5.5 m** | **+1 m (rejected)** | Stable for 600 s |
| robot_localization EKF | 23.4 m | +93 m jump | Stable for 600 s |
| robot_localization UKF |: |: | NaN divergence at t=31 s |

4.2× better accuracy out of the box. Full methodology, charts, and reproduce instructions in [`benchmarks/`](benchmarks/).

---

## Why FusionCore

| Capability | robot_localization | Fuse | FusionCore |
|---|---|---|---|
| Core filter | EKF or UKF | Factor graph | UKF (22D quaternion state) |
| 3D support | Yes | Yes | Full 3D, native |
| IMU bias estimation | No built-in states | Plugin-dependent | Gyro + accel bias states |
| GPS fusion | navsat_transform node | Plugin, no ECEF/RTK | ECEF-native, single node |
| Dual antenna heading | No | No | Yes |
| IMU frame transform | Manual (YAML) | Manual (YAML) | Automatic via TF |
| Message covariances | Used | Partial | Full 3×3 GNSS + odometry |
| GNSS antenna offset | Ignored | Ignored | Lever arm + observability guard |
| Outlier rejection | mahalanobis_threshold | Robust loss functions | Chi-squared gating, all sensors |
| GPS fix quality gating | No | No | GPS / DGPS / RTK_FLOAT / RTK_FIXED |
| Adaptive noise | Manual | Manual | Auto from innovation sequence |
| TF validation at startup | Basic | No | Startup check + fix commands |
| Multiple GNSS receivers | Workaround | Workaround | Native, independent lever arms |
| compass_msgs/Azimuth | No | No | Yes (ENU/NED, rad/deg) |
| Delay compensation | history_length | Factor graph inherent | Full IMU replay, 500ms |
| Ground constraint | Not built-in | Not built-in | VZ=0 pseudo-measurement |
| ZUPT | Not built-in | Not built-in | Auto when stationary |
| Sensor dropout detection | Basic | Basic | Per-sensor SensorHealth enum |
| /diagnostics | Basic | Basic | Per-sensor health + outliers |
| Published covariance | Yes | Yes | Full UKF P matrix |
| Filter reset service | No | No | ~/reset (no restart needed) |
| Maintenance | Deprecated Sep 2023 | Active | Active, 24h response |
| License | BSD-3 | BSD-3 | Apache 2.0 |
| ROS 2 Jazzy | Ported from ROS 1 | Native | Native, from scratch |

---

## Installation

### Prerequisites
- ROS 2 Jazzy Jalisco (primary) or ROS 2 Kilted (community tested)
- A colcon workspace (`~/ros2_ws`)

### Clone into your workspace
This is a monorepo with 4 independent ament_cmake packages: `compass_msgs`, `fusioncore_core`, `fusioncore_ros`, and `fusioncore_gazebo`. Each has its own `package.xml`. Colcon finds them by scanning `src/` recursively. The repo root has no package.xml and is not itself a package. The repo must live inside `src/` for colcon to find the packages.

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

> **Real robot users (no Gazebo):** `fusioncore_gazebo` depends on `ros_gz_sim` which pulls in Gazebo and its GUI components. On headless machines (Raspberry Pi, server) this install is large, unnecessary, and may fail. Skip it by adding a `COLCON_IGNORE` file before building:
> ```bash
> touch ~/ros2_ws/src/fusioncore/fusioncore_gazebo/COLCON_IGNORE
> ```
> This tells colcon to skip that package entirely. `fusioncore_core` and `fusioncore_ros` have no Gazebo dependency and build fine without it.

---

## Running the tests
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fusioncore_core --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fusioncore_core
colcon test-result --verbose
```

Expected output: `39 tests, 0 errors, 0 failures, 0 skipped`

---

## Running FusionCore

```bash
# Terminal 1: launch the node
ros2 launch fusioncore_ros fusioncore.launch.py

# Terminal 2: configure and activate the lifecycle node
ros2 lifecycle set /fusioncore configure
ros2 lifecycle set /fusioncore activate

# Verify it's publishing at 100Hz
ros2 topic hz /fusion/odom
# expected: average rate: 100.000
```

FusionCore uses a ROS 2 lifecycle node. Configure first (load parameters, validate TF tree, check transforms), then activate (start processing sensor data). This prevents the filter from starting with bad initial values or missing transforms.

> **WSL2 note:** If `ros2 lifecycle set` returns "Node not found", use the launch file's built-in auto-configure instead. The Gazebo launch file (`fusioncore_gazebo.launch.py`) configures and activates the node automatically via `EmitEvent(ChangeState(...))` 15 seconds after startup, bypassing DDS discovery latency that affects WSL2.

---

## Verifying all features work

You can test every FusionCore feature without a physical robot using fake sensor data. Replace `~/YOUR_WS` with your actual workspace path (e.g. `~/ros2_ws`, `~/fusioncore_ws`). Open 4 terminals:

**Terminal 1: Launch FusionCore:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/YOUR_WS/install/setup.bash
ros2 launch fusioncore_ros fusioncore.launch.py
```

**Terminal 2: Configure and activate:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/YOUR_WS/install/setup.bash

# Publish required TF transforms (stays running in background)
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id imu_link &
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link &
sleep 1

ros2 lifecycle set /fusioncore configure
sleep 1
ros2 lifecycle set /fusioncore activate
```

**Terminal 3: Feed fake sensors:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/YOUR_WS/install/setup.bash

# Fake IMU at 100Hz (stationary, gravity pointing up)
ros2 topic pub /imu/data sensor_msgs/msg/Imu "{
  header: {frame_id: 'base_link'},
  angular_velocity: {x: 0.0, y: 0.0, z: 0.0},
  linear_acceleration: {x: 0.0, y: 0.0, z: 9.81},
  orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}" --rate 100 &

# Fake wheel encoder at 50Hz (stationary: triggers ZUPT)
ros2 topic pub /odom/wheels nav_msgs/msg/Odometry "{
  header: {frame_id: 'odom'},
  twist: {twist: {linear: {x: 0.0}, angular: {z: 0.0}}}
}" --rate 50 &

# Fake GPS at 5Hz (Hamilton, Ontario)
ros2 topic pub /gnss/fix sensor_msgs/msg/NavSatFix "{
  header: {frame_id: 'base_link'},
  status: {status: 0},
  latitude: 43.2557,
  longitude: -79.8711,
  altitude: 100.0,
  position_covariance: [1.0, 0, 0, 0, 1.0, 0, 0, 0, 4.0],
  position_covariance_type: 2
}" --rate 5
```

**Terminal 4: Verify each feature:**

Check what topics and services are live:
```bash
source /opt/ros/jazzy/setup.bash
source ~/YOUR_WS/install/setup.bash
ros2 topic list | grep fusion
ros2 service list | grep fusioncore
```

You should see `/fusion/odom`, `/fusion/pose`, and `/fusioncore/reset`.

Test `/fusion/pose`: what Nav2, AMCL, and slam_toolbox expect:
```bash
ros2 topic echo /fusion/pose --once
```
You should see a pose message with a full 6×6 covariance matrix from the UKF.

Test `/diagnostics`: per-sensor health at 1Hz:
```bash
ros2 topic echo /diagnostics --once
```
You should see 4 status entries: `fusioncore: IMU`, `fusioncore: Encoder`, `fusioncore: GNSS`, `fusioncore: Filter`. Each shows OK or WARN with outlier counts and heading status.

Test ZUPT: velocity should stay near zero while stationary:
```bash
ros2 topic echo /fusion/odom --field twist.twist.linear
```
Values should be essentially zero (`~1e-10`) even while the IMU is running. This confirms ZUPT is suppressing velocity drift when the robot is not moving.

Test the reset service: reinitializes filter without restarting the node:
```bash
ros2 service call /fusioncore/reset std_srvs/srv/Trigger
```
Expected: `success: True, message: 'FusionCore filter reset. GPS reference cleared.'`

Within 1 second of calling reset, the node log (Terminal 1) should print:
```
Filter reset via ~/reset service.
GNSS reference set: lat=43.255700 lon=-79.871100
```
with **no** `GNSS fix rejected` warnings. GPS re-fuses immediately after reset.

---

## Sensor topics

**Subscribes to:**

| Topic | Type | What it is |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | IMU angular velocity and linear acceleration |
| `/odom/wheels` | `nav_msgs/Odometry` | Wheel encoder velocity |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | GPS position |
| `/gnss/heading` | `sensor_msgs/Imu` | Dual antenna heading (optional) |
| `gnss.azimuth_topic` | `compass_msgs/Azimuth` | Azimuth heading (optional, preferred standard) |
| `gnss.fix2_topic` | `sensor_msgs/NavSatFix` | Second GPS receiver (optional) |

**Publishes:**

| Topic | Type | What it is |
|---|---|---|
| `/fusion/odom` | `nav_msgs/Odometry` | Fused position + orientation + velocity + covariance at 100Hz |
| `/fusion/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Same pose: compatible with AMCL, slam_toolbox, Nav2 pose initializer |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Per-sensor health, outlier counts, heading status at 1Hz |
| `/tf` | TF | `odom -> base_link` for Nav2 |

**Services:**

| Service | Type | What it does |
|---|---|---|
| `~/reset` | `std_srvs/Trigger` | Re-initializes the filter and clears the GPS reference anchor without restarting the node. Useful after teleportation in simulation or after a catastrophic GPS jump in the field. |

---

## Configuration

```yaml
fusioncore:
  ros__parameters:
    base_frame: base_link
    odom_frame: odom
    publish_rate: 100.0

    imu.gyro_noise: 0.005       # rad/s: from your IMU datasheet
    imu.accel_noise: 0.1        # m/s²
    imu.has_magnetometer: false # true for 9-axis IMUs (BNO08x, VectorNav, Xsens)
                                # false for 6-axis: yaw from gyro integration drifts
    imu.remove_gravitational_acceleration: false  # set true if robot drifts in Z while stationary
                                                   # most IMUs report raw specific force (gravity included)
                                                   # FusionCore removes gravity using current filter orientation

    encoder.vel_noise: 0.05     # m/s
    encoder.yaw_noise: 0.02     # rad/s

    gnss.base_noise_xy: 1.0     # meters: scaled automatically by HDOP
    gnss.base_noise_z: 2.0      # meters
    gnss.heading_noise: 0.02    # rad: for dual antenna
    gnss.max_hdop: 4.0          # reject fixes worse than this
    gnss.min_satellites: 4
    gnss.min_fix_type: 1        # minimum fix quality: 1=GPS, 2=DGPS, 3=RTK_FLOAT, 4=RTK_FIXED
                                # note: sensor_msgs/NavSatFix status=2 maps to RTK_FIXED only.
                                # RTK_FLOAT (3) is unreachable via NavSatFix: use 2 or 4.

    # Antenna lever arm: offset from base_link to primary GPS antenna in body frame
    # x=forward, y=left, z=up (meters). Leave at 0 if antenna is above base_link.
    # Lever arm correction only activates when heading is independently validated.
    gnss.lever_arm_x: 0.0
    gnss.lever_arm_y: 0.0
    gnss.lever_arm_z: 0.0

    # Second GPS receiver lever arm (if using gnss.fix2_topic)
    gnss.lever_arm2_x: 0.0
    gnss.lever_arm2_y: 0.0
    gnss.lever_arm2_z: 0.0

    # Optional second GPS receiver
    gnss.fix2_topic: ""

    # Heading topics: pick one or both
    gnss.heading_topic: "/gnss/heading"   # sensor_msgs/Imu
    gnss.azimuth_topic: ""                # compass_msgs/Azimuth (preferred)

    # Mahalanobis outlier rejection
    outlier_rejection: true
    outlier_threshold_gnss: 16.27   # chi2(3, 0.999): 3D position
    outlier_threshold_hdg: 10.83    # chi2(1, 0.999): 1D heading
    outlier_threshold_enc: 11.34    # chi2(3, 0.999): 3D encoder
    outlier_threshold_imu: 15.09    # chi2(6, 0.999): 6D IMU

    # Adaptive noise covariance
    adaptive.imu: true
    adaptive.encoder: true
    adaptive.gnss: true
    adaptive.window: 50
    adaptive.alpha: 0.01

    ukf.q_position: 0.01
    ukf.q_orientation: 1.0e-9  # quaternion regularization only: do NOT increase this
    ukf.q_velocity: 0.1
    ukf.q_angular_vel: 0.1
    ukf.q_acceleration: 1.0
    ukf.q_gyro_bias: 1.0e-5
    ukf.q_accel_bias: 1.0e-5
```
> **Upgrading from an older config?** If your YAML has `ukf.q_orientation: 0.01`, change it to `1.0e-9` or delete the line. The old value corrupts quaternion math at typical IMU rates and causes yaw drift and Z-axis rise in simulation.

### GPS Coordinate Reference System (CRS)

FusionCore uses [PROJ](https://proj.org/) to convert incoming GNSS fixes between coordinate systems. The defaults handle any standard GPS receiver (WGS84 lat/lon → ECEF). Change these only if your receiver outputs a different CRS.

```yaml
    # PROJ coordinate reference system
    input.gnss_crs: "EPSG:4326"              # CRS of incoming NavSatFix messages
                                              # EPSG:4326 = WGS84 lat/lon (standard GPS)
                                              # EPSG:32617 = UTM zone 17N (some RTK receivers)
    output.crs: "EPSG:4978"                  # internal computation CRS
                                              # EPSG:4978 = ECEF XYZ (default, globally valid)
    output.convert_to_enu_at_reference: true  # true when output.crs is ECEF
                                              # false when output.crs is already a local projected CRS
    reference.use_first_fix: true            # anchor local ENU origin to first GPS fix
    reference.x: 0.0                         # fixed origin in output.crs (when use_first_fix: false)
    reference.y: 0.0
    reference.z: 0.0
```

**Agricultural RTK example**: receiver outputs UTM zone 17N (easting/northing) directly:
```yaml
    input.gnss_crs: "EPSG:32617"
    output.crs: "EPSG:32617"
    output.convert_to_enu_at_reference: false
    reference.use_first_fix: true
```

---

## How FusionCore handles the hard problems

### IMU frame transform

IMUs are almost never mounted at `base_link`. FusionCore reads `frame_id` from every IMU message, looks up the TF rotation to `base_link`, and rotates angular velocity and linear acceleration before fusing. If the transform is missing you get the exact command to fix it:

```
[WARN] Cannot transform IMU from imu_link to base_link.
Fix: ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id imu_link
```

### TF validation at startup

During `configure`, FusionCore checks that all required TF transforms exist before the filter starts. Missing transforms print the exact fix command: no silent failures, no mysterious drift:

```
--- TF Validation ---
  [OK]      imu_link -> base_link
  [MISSING] base_link -> odom  Fix: ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link
---------------------
```

### Mahalanobis outlier rejection

Before fusing any GPS fix, FusionCore computes how statistically implausible the measurement is given the current state estimate. The Mahalanobis distance `d² = νᵀ · S⁻¹ · ν` is compared against chi-squared thresholds at the 99.9th percentile. Fixes that exceed the threshold are rejected without updating the filter.

This handles GPS jumps, multipath errors, and encoder slip spikes. The filter position stays stable during rejection: verified by injecting a 500m GPS jump in testing and observing zero position change.

GNSS position covariance is floored before the gate is evaluated. This prevents RTK-grade receivers (typical σxy ~3mm) from triggering self-rejection when the filter has not yet converged to RTK-level accuracy.

### Zero velocity updates (ZUPT)

When the robot is stationary: encoder speed below 0.05 m/s and angular rate below 0.05 rad/s: FusionCore fuses a zero velocity pseudo-measurement with very tight noise. This stops the IMU from drifting the velocity estimate while the robot is sitting still. Every serious inertial navigation system does this. Without ZUPT, IMU noise accumulates into a false velocity estimate over time even when the robot has not moved.

All MEMS IMUs have a small accelerometer and gyro bias that is unknown at startup. By default the filter learns it over ~60 seconds, causing a small position offset at startup. Setting `init.stationary_window: 2.0` makes the filter collect 2 seconds of IMU data before starting, estimate the bias directly, and initialize with the correct values: reducing the startup transient from ~10cm to under 1cm. The robot must be stationary during the window; if it moves, the filter falls back to zero bias automatically.

### Adaptive noise covariance

FusionCore tracks a sliding window of 50 innovation sequences per sensor and estimates the actual noise covariance from the data. The noise matrix R is slowly updated toward the estimated true value using an exponential moving average with `alpha=0.01`. After a few minutes of operation, R converges to the real sensor characteristics automatically. No manual YAML tuning required.

### GPS antenna offset (lever arm)

If the GPS antenna is not at `base_link`: mounted on top of the robot, forward of center: its readings correspond to a different trajectory than `base_link`. FusionCore corrects for this using the rotation matrix from the current state: `p_antenna = p_base + R * lever_arm`. Lever arm correction only activates when heading has been independently validated: applying it with wrong heading makes things worse, not better.

Each GPS receiver has its own independent lever arm. Primary receiver uses `gnss.lever_arm_x/y/z`, secondary receiver uses `gnss.lever_arm2_x/y/z`.

### Heading observability

FusionCore tracks a `heading_validated_` flag that is only set true from a genuine independent source:

- **`DUAL_ANTENNA`**: dual antenna heading message received
- **`IMU_ORIENTATION`**: 9-axis AHRS published full orientation (only when `imu.has_magnetometer: true`: 6-axis IMUs drift in yaw and don't count)
- **`GPS_TRACK`**: robot has traveled >= 5 meters at speed >= 0.2 m/s with yaw rate <= 0.3 rad/s

Before any of these, lever arm is disabled regardless of what yaw variance says.

### GPS fix quality gating

FusionCore maps `sensor_msgs/NavSatFix.status` to an internal fix type enum and rejects fixes below a configurable minimum quality. The default accepts any valid GPS fix. Set to 4 to require RTK_FIXED:

```yaml
gnss.min_fix_type: 4   # require RTK_FIXED: reject basic GPS entirely
```

When a fix is rejected due to quality, the rejection log shows the fix type and threshold:

```
[WARN] GNSS fix rejected (fix_type=1, min=4, hdop=1.20, quality check or Mahalanobis gate)
```

Important: `sensor_msgs/NavSatFix` has no `STATUS_RTK_FLOAT`. Status 2 maps to RTK_FIXED. Setting `min_fix_type: 3` will silently starve the filter. Use 2 or 4 as meaningful thresholds.

### Per-sensor diagnostics

FusionCore publishes `/diagnostics` at 1Hz compatible with `rqt_robot_monitor` and Nav2. Four status entries: IMU, Encoder, GNSS, Filter. Each shows OK or WARN with outlier counts, heading source, distance traveled, position uncertainty, and update count.

### Filter reset service

```bash
ros2 service call /fusioncore/reset std_srvs/srv/Trigger
```

Reinitializes the UKF state and clears the GPS reference anchor. The robot re-anchors on the next GPS fix. No node restart required.

### Message covariances

FusionCore uses the covariance values sensors actually publish. GPS: full 3x3 matrix when `position_covariance_type == 3`. Wheel odometry: reads `twist.covariance` per-axis. IMU orientation: reads `orientation_covariance` from the message.

### Delay compensation

FusionCore stores a ring buffer of 100 IMU messages (1 second at 100Hz). When a delayed GPS fix arrives, it restores the closest state snapshot before the fix timestamp, re-fuses the fix at the correct time, then replays all buffered IMU messages forward to now. This eliminates motion-model approximation error for delayed measurements.

### Non-holonomic ground constraint

For wheeled ground robots, FusionCore fuses a `VZ = 0` pseudo-measurement on every encoder update. This prevents vertical velocity from drifting due to IMU noise. Do not use for aerial vehicles or robots that move vertically.

### Sensor dropout detection

FusionCore tracks the last update time for each sensor independently. If a sensor goes silent for longer than `stale_timeout` (default 1.0 second), `get_status()` returns `SensorHealth::STALE` for that sensor. The filter continues running on the remaining sensors and recovers automatically when the missing sensor resumes.

### compass_msgs/Azimuth

FusionCore ships a ROS 2 native port of `compass_msgs/Azimuth` (upstream is ROS 1 only). Handles ENU/NED convention conversion, RAD/DEG units, and warns when magnetic north reference is used instead of geographic.

---

## Simulation

FusionCore ships with a Gazebo Harmonic simulation world so you can test the full fusion pipeline without physical hardware. It includes a differential drive robot with a 100Hz IMU and GPS, in an outdoor environment with the GPS origin set to Hamilton, Ontario.

Gazebo Harmonic's built-in NavSat sensor has a known bug (gz-sim issue #2163) where it periodically outputs GPS fixes at completely wrong coordinates. Rather than fight a broken sensor, the simulation derives GPS from Gazebo's ground truth world pose and adds realistic Gaussian noise (0.5m horizontal, 0.3m vertical 1-sigma).

### Prerequisites for simulation

Gazebo Harmonic and the ROS-Gazebo bridge are not installed by `rosdep` automatically. Install them first:

```bash
sudo apt install ros-jazzy-ros-gz
```

### Running the simulation

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch fusioncore_gazebo fusioncore_gazebo.launch.py
```

Drive the robot and watch the fused position:
```bash
# Terminal 2: drive in a circle
source /opt/ros/jazzy/setup.bash
source ~/YOUR_WS/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}" --rate 10

# Terminal 3: watch position
source /opt/ros/jazzy/setup.bash
source ~/YOUR_WS/install/setup.bash
ros2 topic echo /fusion/odom --field pose.pose.position
```

### Integration tests

```bash
python3 ~/ros2_ws/src/fusioncore/fusioncore_gazebo/launch/integration_test.py
```

Four automated tests: IMU drift rate, outlier rejection, GPS correction after drift, full circle return. All four pass on a clean session.

---

## Real robot configs

FusionCore ships with configs for real hardware setups tested by community members:

- `fusioncore_ros/config/duatic_mecanum.yaml`: Duatic industrial mecanum manipulator. BNO085 IMU, no GPS, mecanum wheel odometry.
- `fusioncore_ros/launch/fusioncore_duatic.launch.py`: One-command launch for the Duatic setup, handles all topic remapping automatically.

To add your robot's config, open a GitHub issue or submit a PR.

---

## Architecture

```
fusioncore/
├── fusioncore_core/              # Pure C++17 math library. Zero ROS dependency.
│   ├── include/fusioncore/
│   │   ├── ukf.hpp               # Unscented Kalman Filter: 45 sigma points
│   │   ├── state.hpp             # 22-dimensional state vector (quaternion orientation)
│   │   ├── fusioncore.hpp        # Public API: FusionCore, FusionCoreConfig
│   │   └── sensors/
│   │       ├── imu.hpp           # Raw IMU + orientation measurement models
│   │       ├── encoder.hpp       # Wheel encoder measurement model
│   │       └── gnss.hpp          # GPS: ECEF, lever arm, covariance, quality gating
│   └── src/
│       ├── ukf.cpp               # UKF: sigma points, predict, update, predict_measurement
│       └── fusioncore.cpp        # Manager: outlier rejection, adaptive noise,
│                                 #          snapshots, observability, delay compensation
├── fusioncore_ros/               # ROS 2 Jazzy wrapper
│   ├── src/fusion_node.cpp       # Lifecycle node: all sensor callbacks, TF validation,
│   │                             #   ZUPT, diagnostics, /fusion/pose, reset service
│   ├── config/fusioncore.yaml    # Default configuration
│   ├── config/duatic_mecanum.yaml
│   └── launch/fusioncore.launch.py
└── fusioncore_gazebo/            # Simulation world
    ├── worlds/fusioncore_test.sdf
    ├── models/fusioncore_robot/
    ├── launch/fusioncore_gazebo.launch.py
    ├── launch/gz_pose_to_gps.py
    └── launch/integration_test.py
```

---

## Technical details

- **Filter:** Unscented Kalman Filter, 45 sigma points
- **State vector:** 22-dimensional: position (x,y,z), orientation (quaternion qw,qx,qy,qz), linear velocity, angular velocity, linear acceleration, gyroscope bias (x,y,z), accelerometer bias (x,y,z)
- **GPS coordinate system:** Configurable via PROJ: default ECEF (EPSG:4978, globally valid); supports any PROJ-compatible input CRS including UTM zones
- **Bias estimation:** Continuous online estimation, no calibration required
- **GPS quality scaling:** Noise covariance scaled by HDOP/VDOP, or full 3x3 message covariance when available
- **Outlier rejection:** Mahalanobis chi-squared gating at 99.9th percentile per sensor dimension
- **Adaptive noise:** Sliding window innovation tracking, exponential moving average R update
- **Delay compensation:** IMU ring buffer replay retrodiction up to 500ms
- **ZUPT:** Automatic zero velocity updates when stationary
- **Output rate:** 100Hz
- **Language:** C++17
- **License:** Apache 2.0

---

## Status

**Working and tested:**
- Hardware testing in progress: industrial mecanum manipulator (Duatic), agricultural RTK robot (Southern Ontario)
- UKF core: 39 unit tests passing via colcon test
- UKF numerical stability: P symmetrization + identity-shift Cholesky repair + angular velocity variance cap
- IMU + encoder + GPS fusion
- Automatic IMU bias estimation
- ECEF GPS conversion with quality-aware noise scaling
- Dual antenna heading: both `sensor_msgs/Imu` and `compass_msgs/Azimuth`
- IMU frame transform via TF
- TF validation at startup with exact fix commands
- GPS lever arm with heading observability guard: independent params for primary and secondary receivers
- Full 3x3 GPS covariance support
- Wheel odometry covariance support
- Multiple GPS receivers
- Heading observability tracking: DUAL_ANTENNA / IMU_ORIENTATION / GPS_TRACK
- GPS fix quality gating: configurable `gnss.min_fix_type` (GPS / DGPS / RTK_FIXED)
- Mahalanobis outlier rejection: GPS jumps verified rejected in testing
- Adaptive noise covariance: automatic R estimation from innovation sequence
- GPS delay compensation: full IMU replay retrodiction up to 500ms
- Non-holonomic ground constraint: VZ=0 pseudo-measurement for wheeled robots
- Zero velocity updates (ZUPT): automatic when encoder speed < 0.05 m/s
- Per-sensor diagnostics: `/diagnostics` at 1Hz with outlier counts and heading status
- `/fusion/pose`: PoseWithCovarianceStamped for Nav2 / AMCL / slam_toolbox
- Filter reset service: `~/reset` clears filter and GPS reference without node restart
- Sensor dropout detection: per-sensor staleness tracking via SensorHealth enum
- PROJ CRS coordinate transform: configurable input/output CRS via PROJ library (WGS84, UTM, ECEF, any EPSG code)
- ROS 2 Jazzy lifecycle node at 100Hz
- Gazebo Harmonic simulation world

**Known limitations:**
- GNSS antenna lever arm is fixed and known: does not estimate it from data
- In Gazebo simulation, residual y-axis drift (~0.3m) can occur from Gazebo physics: not a filter error
- Mecanum drive lateral velocity is not predicted by the motion model

**Roadmap:**
- Ackermann and omnidirectional steering motion models
- Mecanum drive motion model
- Auto-derive GNSS lever arm from TF header.frame_id

---

## License

Apache 2.0. Includes explicit patent license grant that BSD-3 does not provide. Commercially safe

---

## Support

Issues answered within 24 hours. Open a GitHub issue or find the original discussion on ROS Discourse.

This project exists because of a community thread from December 2024 asking for a `robot_localization` replacement that actually works on ROS 2 Jazzy. If you hit a problem: open an issue. That feedback drives the roadmap.
