<p align="center">
  <img src="https://github.com/user-attachments/assets/f03a5458-761f-4bce-9e17-cc383bdcd57a" width="700">
</p>

# FusionCore

**ROS 2 sensor fusion SDK. Combines IMU, wheel encoders, and GPS into one reliable position estimate. Zero manual tuning. Apache 2.0.**

---

## What problem does this solve?

Every mobile robot needs to know where it is. It gets this from multiple sensors: IMU, wheel encoders, GPS: each of which is imperfect in its own way. IMUs drift. Wheels slip. GPS jumps. You need software that intelligently combines all three into one trustworthy position estimate.

That software is called a sensor fusion package. The standard one for ROS, `robot_localization`, was officially deprecated in September 2023. Its designated replacement (`fuse`) still doesn't support GPS properly as of early 2026. At ROSCon UK 2025 the official workshop was still teaching both tools because no clear accessible replacement existed.

FusionCore is that replacement.

---

## Why FusionCore

| Capability | robot_localization | Fuse | FusionCore |
|---|---|---|---|
| Core filter | EKF | Factor graph | UKF |
| 3D support | Yes | Yes (configuration-dependent) | Full 3D, native |
| IMU bias estimation | No explicit bias states | No explicit bias states | Gyro + accel bias in state vector |
| GPS fusion | Via navsat_transform (separate node) | Plugin support, no ECEF or RTK gating | ECEF-native, no separate node |
| Dual antenna heading | Not supported | Not supported | Native |
| IMU frame transform | Configured in YAML | Configured in YAML | Read from message frame_id via TF at runtime |
| Message covariances | Used | Partial | Full 3×3 GNSS + diagonal odometry |
| GNSS antenna offset | Not handled | Not handled | Lever arm with heading observability guard |
| Outlier rejection | mahalanobis_threshold | Possible via robust loss functions | Mahalanobis chi-squared gating, all sensors |
| GPS fix quality gating | Not supported | Not supported | Configurable: GPS / DGPS / RTK_FLOAT / RTK_FIXED |
| Adaptive noise | Manual re-tuning | Manual re-tuning | Automatic from innovation sequence |
| TF validation at startup | Not provided | Not provided | Startup check with exact fix commands |
| Multiple GNSS receivers | Not supported | Not supported | 2× GPS with independent lever arms |
| compass_msgs/Azimuth | Not supported | Not supported | Supported: ENU/NED, rad/deg, magnetic warning |
| Delay compensation | history_length (re-linearizes at current state) | Inherent in factor graph design | Full IMU replay retrodiction up to 500ms |
| Ground constraint | Not built-in | Not built-in | VZ=0 pseudo-measurement for wheeled robots |
| Sensor dropout detection | Basic diagnostics | Basic diagnostics | Per-sensor staleness with SensorHealth enum |
| Published covariance | Yes | Yes | Yes (full UKF P → pose + twist covariance) |
| Maintenance | Deprecated Sep 2023 | Active | Active: issues answered in 24h |
| License | BSD-3 | BSD-3 | Apache 2.0 |
| ROS 2 Jazzy | Ported from ROS 1 | Native | Native, built from scratch |

---

## Installation

### Prerequisites
- ROS 2 Jazzy Jalisco (primary) or ROS 2 Kilted (community tested)
- A colcon workspace (`~/ros2_ws`)

### Clone into your workspace
The repo must live inside `src/`, colcon only searches there for packages.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fusioncore_core fusioncore_ros
source install/setup.bash
```

---

## Running the tests
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fusioncore_core --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fusioncore_core
colcon test-result --verbose
```

Expected output: `45 tests, 0 errors, 0 failures, 0 skipped`

---

## Running FusionCore

```bash
# Terminal 1
ros2 launch fusioncore_ros fusioncore.launch.py

# Terminal 2
ros2 lifecycle set /fusioncore configure
ros2 lifecycle set /fusioncore activate

# Verify
ros2 topic hz /fusion/odom
# expected: average rate: 100.000
```

FusionCore uses a ROS 2 lifecycle node. Configure first (load parameters, validate TF tree, check transforms), then activate (start processing sensor data). This prevents the filter from starting with bad initial values or missing transforms.

> **WSL2 note:** If `ros2 lifecycle set` returns "Node not found", use the launch file's built-in auto-configure instead. The Gazebo launch file (`fusioncore_gazebo.launch.py`) configures and activates the node automatically via `EmitEvent(ChangeState(...))` 12 seconds after startup, bypassing DDS discovery latency that affects WSL2.

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
| `/fusion/odom` | `nav_msgs/Odometry` | Fused position + orientation + velocity at 100Hz |
| `/tf` | TF | `odom -> base_link` for Nav2 |

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

    encoder.vel_noise: 0.05     # m/s
    encoder.yaw_noise: 0.02     # rad/s

    gnss.base_noise_xy: 1.0     # meters: scaled automatically by HDOP. Must be 1.0 when
                                # NavSatFix provides covariance: fusion_node sets
                                # fix.hdop = sqrt(var_xy), so sigma = 1.0 * sqrt(var_xy).
                                # Setting this to NOISE_H double-shrinks R.
    gnss.base_noise_z: 1.0      # meters: same reasoning: set to 1.0, not NOISE_V
    gnss.heading_noise: 0.02    # rad: for dual antenna
    gnss.max_hdop: 4.0          # reject fixes worse than this
    gnss.min_satellites: 4
    gnss.min_fix_type: 1        # minimum fix quality: 1=GPS, 2=DGPS, 3=RTK_FLOAT, 4=RTK_FIXED
                                # note: sensor_msgs/NavSatFix status=2 maps to RTK_FIXED only.
                                # RTK_FLOAT (3) is unreachable via NavSatFix alone.
                                # set to 4 to require RTK_FIXED, 2 to require any augmented fix.

    # Antenna lever arm: offset from base_link to GPS antenna in body frame
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

    # Mahalanobis outlier rejection: rejects GPS jumps, encoder spikes
    # Thresholds are chi-squared 99.9th percentile for each measurement dimension
    outlier_rejection: true
    outlier_threshold_gnss: 16.27   # chi2(3, 0.999): 3D position
    outlier_threshold_hdg: 10.83    # chi2(1, 0.999): 1D heading
    outlier_threshold_enc: 11.34    # chi2(3, 0.999): 3D encoder
    outlier_threshold_imu: 15.09    # chi2(6, 0.999): 6D IMU

    # Adaptive noise covariance: automatically estimates true sensor noise
    adaptive.imu: true
    adaptive.encoder: true
    adaptive.gnss: true
    adaptive.window: 50       # innovation window size (50 = ~0.5s at 100Hz)
    adaptive.alpha: 0.01      # learning rate (0.01 = slow, stable)

    ukf.q_position: 0.01
    ukf.q_orientation: 0.01
    ukf.q_velocity: 0.1
    ukf.q_angular_vel: 0.1
    ukf.q_acceleration: 1.0
    ukf.q_gyro_bias: 1.0e-5
    ukf.q_accel_bias: 1.0e-5
```

---

## How FusionCore handles the hard problems

### IMU frame transform

IMUs are almost never mounted at `base_link`. FusionCore reads `frame_id` from every IMU message, looks up the TF rotation to `base_link`, and rotates angular velocity and linear acceleration before fusing. If the transform is missing you get the exact command to fix it:

```
[WARN] Cannot transform IMU from imu_link to base_link.
Fix: ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link
```

### TF validation at startup

During `configure`, FusionCore checks that all required TF transforms exist before the filter starts. Missing transforms print the exact fix command: no silent failures, no mysterious drift:

```
--- TF Validation ---
  [OK]      imu_link -> base_link
  [MISSING] base_link -> odom  Fix: ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
---------------------
```

### Mahalanobis outlier rejection

Before fusing any GPS fix, FusionCore computes how statistically implausible the measurement is given the current state estimate. The Mahalanobis distance `d² = νᵀ · S⁻¹ · ν` is compared against chi-squared thresholds at the 99.9th percentile. Fixes that exceed the threshold are rejected without updating the filter.

This handles GPS jumps, multipath errors, and encoder slip spikes. The filter position stays stable during rejection: verified by injecting a 1km GPS jump in testing and observing zero position change.

### Adaptive noise covariance

robot_localization requires manually tuning noise covariance matrices in YAML. Getting these wrong causes the filter to be overconfident or underconfident.

FusionCore tracks a sliding window of 50 innovation sequences per sensor and estimates the actual noise covariance from the data. The noise matrix R is slowly updated toward the estimated true value using an exponential moving average with `alpha=0.01`. After a few minutes of operation, R converges to the real sensor characteristics automatically.

### GPS antenna offset (lever arm)

If the GPS antenna is not at `base_link`: mounted on top of the robot, forward of center: its readings correspond to a different trajectory than `base_link`. Ignoring this injects position errors proportional to lever arm length times rotation rate.

FusionCore corrects for this using the rotation matrix from the current state: `p_antenna = p_base + R * lever_arm`. But this correction depends on heading: if heading is wrong the correction makes things worse. So FusionCore only activates lever arm correction when heading has been **independently validated** from a real source.

Each GPS receiver has its own independent lever arm. Primary receiver uses `gnss.lever_arm_x/y/z`, secondary receiver uses `gnss.lever_arm2_x/y/z`.

### Heading observability

A Kalman filter can reduce its own uncertainty about heading even when it has no real heading sensor: it does this by fitting the motion model to GPS position updates. The variance goes down, but the heading might still be wrong. Using that fake confidence to activate lever arm correction can destabilize the filter.

FusionCore tracks a `heading_validated_` flag that is only set true from a genuine independent source:

- **`DUAL_ANTENNA`**: dual antenna heading message received
- **`IMU_ORIENTATION`**: 9-axis AHRS published full orientation (only when `imu.has_magnetometer: true`: 6-axis IMUs drift in yaw and don't count)
- **`GPS_TRACK`**: robot has traveled >= 5 meters at speed >= 0.2 m/s with yaw rate <= 0.3 rad/s (geometrically observable, not just accumulated distance or GPS jitter)

Before any of these, lever arm is disabled regardless of what yaw variance says. The filter cannot fake its way into applying the correction.

### Message covariances

FusionCore uses the covariance values sensors actually publish rather than ignoring them.

**GPS:** When `position_covariance_type == 3`, the full 3x3 covariance matrix is used including off-diagonal elements. RTK receivers in particular publish meaningful off-diagonal terms. Falls back to HDOP/VDOP scaling otherwise. Config params always available as override for sensors with bogus covariance.

**Wheel odometry:** Reads `twist.covariance` per-axis when available. A wheel-slip estimating odometry node that publishes real covariances gets the benefit automatically.

**IMU orientation:** Reads `orientation_covariance` from the message. Uses it directly when meaningful, falls back to config params when not.

### GPS fix quality gating

FusionCore maps `sensor_msgs/NavSatFix.status` to an internal fix type enum and rejects fixes below a configurable minimum quality. The default (`gnss.min_fix_type: 1`) accepts any valid GPS fix: identical to previous behavior. Set to 2 for DGPS-or-better, or 4 to require RTK_FIXED.

```yaml
gnss.min_fix_type: 4   # require RTK_FIXED: reject basic GPS entirely
```

When a fix is rejected due to quality, the rejection log shows the fix type and threshold so you can see exactly why:

```
[WARN] GNSS fix rejected (fix_type=1, min=4, hdop=1.20, quality check or Mahalanobis gate)
```

One important limitation: `sensor_msgs/NavSatFix` has no `STATUS_RTK_FLOAT` value. Status 2 (GBAS) maps to RTK_FIXED. Setting `min_fix_type: 3` (RTK_FLOAT) will therefore silently starve the filter on NavSatFix topics. Use 2 or 4 as meaningful thresholds.

### Non-holonomic ground constraint

For wheeled ground robots, FusionCore fuses a `VZ = 0` pseudo-measurement on every encoder update. This prevents vertical velocity from drifting due to IMU noise and keeps altitude estimation stable even when GPS has weak vertical observability.

Call `update_ground_constraint(timestamp)` after every `update_encoder()` call in your integration. Do not call this for aerial vehicles or robots that can move vertically.

### Sensor dropout detection

FusionCore tracks the last update time for each sensor independently. If a sensor goes silent for longer than `stale_timeout` (default 1.0 second), `get_status()` returns `SensorHealth::STALE` for that sensor. The filter continues running on the remaining sensors and recovers automatically when the missing sensor resumes. This is reported via `FusionCoreStatus` so your application can alert the operator or take action without polling every topic individually.

### compass_msgs/Azimuth

peci1 (Great Contributor, ROS Discourse) suggested using `compass_msgs/Azimuth` as a standard heading message format. The upstream package is ROS 1 only. FusionCore ships a ROS 2 native port with the identical message definition.

FusionCore accepts `compass_msgs/Azimuth` on a configurable topic, handles ENU/NED convention conversion, RAD/DEG units, and warns when magnetic north reference is used instead of geographic.

### Delay compensation

GPS messages arrive 100-300ms after the fix was taken. Without compensation, delayed fixes are silently dropped: the filter's clock has already moved past that timestamp.

FusionCore saves a full state snapshot (21-dimensional state + covariance) on every IMU update at 100Hz: 50 snapshots = 0.5 seconds of history. When a delayed GPS fix arrives, it finds the closest snapshot before the fix timestamp, restores that state, applies the fix at the correct time, then re-predicts forward to now.

This is approximate retrodiction: the re-prediction uses the motion model rather than replaying actual IMU history. For smooth motion at normal robot speeds the approximation error is small compared to GPS noise. Full IMU replay retrodiction is on the roadmap.

**Update:** Full IMU replay retrodiction is now implemented. Every raw IMU message is stored in a ring buffer (configurable size, default 100 messages = 1 second at 100Hz). When a delayed GPS fix arrives, FusionCore restores the closest snapshot before the fix timestamp, re-fuses the fix at the correct time, then replays all buffered IMU messages forward to now rather than using one big predict(dt). This eliminates the motion-model approximation error for delayed measurements entirely.

---

## Simulation

FusionCore ships with a Gazebo Harmonic simulation world so you can test the full fusion pipeline without physical hardware. It includes a differential drive robot with a 100Hz IMU and GPS, in an outdoor environment with the GPS origin set to Hamilton, Ontario.

One thing worth knowing up front: Gazebo Harmonic's built-in NavSat sensor has a known bug (gz-sim issue #2163) where it periodically outputs GPS fixes at completely wrong coordinates: sometimes 100km away. Rather than fight a broken sensor, the simulation derives GPS from Gazebo's ground truth world pose and adds realistic Gaussian noise (0.5m horizontal, 0.3m vertical 1-sigma). This gives you a clean, honest GPS model for testing the filter.

### Running the simulation

If you haven't built yet, add `fusioncore_gazebo` to the build command from the Installation section:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fusioncore_gazebo
source install/setup.bash
ros2 launch fusioncore_gazebo fusioncore_gazebo.launch.py
```

Launch everything: Gazebo, the ROS bridge, and FusionCore all start together and auto-configure after 12 seconds:
```bash
ros2 launch fusioncore_gazebo fusioncore_gazebo.launch.py
```

Drive the robot and watch the fused position:
```bash
# Terminal 2: drive forward
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}" --rate 10

# Terminal 3: watch position
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /fusion/odom --field pose.pose.position
```

### Integration tests

Four automated tests verify the full stack: IMU drift rate, outlier rejection, GPS correction after drift, and full circle return. Run them while the simulation is up:
```bash
python3 ~/ros2_ws/src/fusioncore/fusioncore_gazebo/launch/integration_test.py
```

All four pass on a clean session.

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
│   │   ├── ukf.hpp               # Unscented Kalman Filter: 43 sigma points
│   │   ├── state.hpp             # 21-dimensional state vector
│   │   ├── fusioncore.hpp        # Public API: FusionCore, FusionCoreConfig
│   │   └── sensors/
│   │       ├── imu.hpp           # Raw IMU + orientation measurement models
│   │       ├── encoder.hpp       # Wheel encoder measurement model
│   │       └── gnss.hpp          # GPS: ECEF, lever arm, covariance, quality
│   └── src/
│       ├── ukf.cpp               # UKF: sigma points, predict, update, predict_measurement
│       └── fusioncore.cpp        # Manager: outlier rejection, adaptive noise,
│                                 #          snapshots, observability, delay compensation
├── fusioncore_ros/               # ROS 2 Jazzy wrapper
│   ├── src/fusion_node.cpp       # Lifecycle node: all sensor callbacks, TF validation
│   ├── config/fusioncore.yaml    # Default configuration
│   ├── config/duatic_mecanum.yaml  # Duatic mecanum robot config
│   └── launch/fusioncore.launch.py
└── fusioncore_gazebo/            # Simulation world
    ├── worlds/fusioncore_test.sdf     # Outdoor world, GPS origin Hamilton ON
    ├── models/fusioncore_robot/       # Differential drive robot, IMU at 100Hz
    ├── launch/fusioncore_gazebo.launch.py
    ├── launch/gz_pose_to_gps.py       # Ground truth pose -> NavSatFix
    └── launch/integration_test.py     # 4 automated end-to-end tests
```

`fusioncore_core` has no ROS dependency by design. The core algorithm can run as firmware on embedded hardware for the Phase 2 hardware module without any ROS installation.

---

## Technical details

- **Filter:** Unscented Kalman Filter, 43 sigma points
- **State vector:** 21-dimensional: position (x,y,z), orientation (roll,pitch,yaw), linear velocity, angular velocity, linear acceleration, gyroscope bias (x,y,z), accelerometer bias (x,y,z)
- **GPS coordinate system:** ECEF: globally valid, no UTM zone boundaries or discontinuities
- **Bias estimation:** Continuous online estimation, no calibration required
- **GPS quality scaling:** Noise covariance scaled by HDOP/VDOP, or full 3x3 message covariance when available
- **Outlier rejection:** Mahalanobis chi-squared gating at 99.9th percentile per sensor dimension
- **Adaptive noise:** Sliding window innovation tracking, exponential moving average R update
- **Delay compensation:** State snapshot buffer, retrodiction up to 500ms
- **Output rate:** 100Hz
- **Language:** C++17
- **License:** Apache 2.0

---

## Status

**Working and tested:**
- Hardware testing in progress: industrial mecanum manipulator (Duatic), agricultural RTK robot (Southern Ontario)
- UKF core: 45 unit tests passing via colcon test
- UKF numerical stability: P symmetrization + identity-shift Cholesky repair
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
- GPS fix quality gating: configurable `gnss.min_fix_type` rejects fixes below a minimum quality (GPS / DGPS / RTK_FIXED)
- 6-axis IMU yaw correctly blocked: when `imu.has_magnetometer: false`, yaw from IMU orientation is not fused (prevents gyro drift from corrupting heading); roll and pitch still fused normally
- Mahalanobis outlier rejection: GPS jumps verified rejected in testing
- Adaptive noise covariance: automatic R estimation from innovation sequence
- GPS delay compensation: full IMU replay retrodiction up to 500ms (per-message replay, not approximate)
- Non-holonomic ground constraint: VZ=0 pseudo-measurement for wheeled robots
- Sensor dropout detection: per-sensor staleness tracking via FusionCoreStatus
- ROS 2 Jazzy lifecycle node at 100Hz
- Gazebo Harmonic simulation world

**Known limitations:**
- GNSS antenna lever arm is fixed and known: does not estimate it from data.
- In Gazebo simulation, residual y-axis drift (~0.3m) can occur from real Gazebo physics (wheel contact forces, slight crabbing). This is not a filter error: the robot's true center of mass drifts relative to the GPS-derived ENU origin.
- Mecanum drive lateral velocity is not predicted by the motion model: filter fuses odometry as velocity measurement but does not estimate sideways drift.

**Roadmap:**
- Ackermann and omnidirectional steering motion models
- Mecanum drive motion model
- UTM coordinate mode for GNSS (community contributed, in progress)
- Auto-derive GNSS lever arm from TF header.frame_id

---

## License

Apache 2.0. Includes explicit patent license grant that BSD-3 does not provide. Commercially safe.

---

## Support

Issues answered within 24 hours. Open a GitHub issue or find the original discussion on ROS Discourse.

This project exists because of a community thread from December 2024 asking for a `robot_localization` replacement that actually works on ROS 2 Jazzy. If you hit a problem: open an issue. That feedback drives the roadmap.
