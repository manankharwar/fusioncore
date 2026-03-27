# FusionCore

**ROS 2 sensor fusion SDK. Combines IMU, wheel encoders, and GPS into one reliable position estimate. Zero manual tuning. Apache 2.0.**

---

## What problem does this solve?

Every mobile robot needs to know where it is. It gets this from multiple sensors — IMU, wheel encoders, GPS — each of which is imperfect in its own way. IMUs drift. Wheels slip. GPS jumps. You need software that intelligently combines all three into one trustworthy position estimate.

That software is called a sensor fusion package. The standard one for ROS, `robot_localization`, was officially deprecated in September 2023. Its designated replacement (`fuse`) still doesn't support GPS properly as of early 2026. At ROSCon UK 2025 the official workshop was still teaching both tools because no clear accessible replacement existed.

FusionCore is that replacement.

---

## Why FusionCore

| Capability | robot_localization | Fuse | FusionCore |
|---|---|---|---|
| Core filter | EKF | Factor graph | UKF |
| 3D support | Partial | PR open 1+ year | Full 3D, native |
| IMU bias estimation | None | Complex | Automatic |
| GPS fusion | UTM workaround | Not implemented | ECEF, proper |
| Dual antenna heading | Hack required | Not supported | Native |
| IMU frame transform | Manual | Manual | Automatic via TF |
| Message covariances | Ignored | Partial | Full 3x3 GNSS + odometry |
| GNSS antenna offset | Ignored | Ignored | Lever arm with observability guard |
| Multiple sensor sources | No | No | Yes — 2x GPS, multiple IMUs |
| compass_msgs/Azimuth | No | No | Yes |
| Delay compensation | No | No | Yes — retrodiction up to 500ms |
| Maintenance | Abandoned | Slow | Active, issues answered in 24h |
| License | BSD-3 | BSD-3 | Apache 2.0 |
| ROS 2 Jazzy | Ported | Native | Native, built from scratch |
| Working examples | Minimal | None | Real robot configs |

---

## Installation

### Prerequisites
- ROS 2 Jazzy Jalisco
- A colcon workspace (`~/ros2_ws`)

### Clone into your workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fusioncore_core fusioncore_ros
source install/setup.bash
```

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

FusionCore uses a ROS 2 lifecycle node. Configure first (load parameters, validate config), then activate (start processing data). This prevents the filter from starting mid-motion with bad initial values.

---

## Sensor topics

**Subscribes to:**

| Topic | Type | What it is |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | IMU angular velocity and linear acceleration |
| `/odom/wheels` | `nav_msgs/Odometry` | Wheel encoder velocity |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | GPS position |
| `/gnss/heading` | `sensor_msgs/Imu` | Dual antenna heading (optional) |
| `gnss.azimuth_topic` | `compass_msgs/Azimuth` | Azimuth heading (optional, preferred) |
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

    imu.gyro_noise: 0.005       # rad/s — from your IMU datasheet
    imu.accel_noise: 0.1        # m/s²
    imu.has_magnetometer: false # true for 9-axis IMUs (BNO08x, VectorNav, Xsens)
                                # false for 6-axis — yaw from gyro integration drifts

    encoder.vel_noise: 0.05     # m/s
    encoder.yaw_noise: 0.02     # rad/s

    gnss.base_noise_xy: 1.0     # meters — scaled automatically by HDOP
    gnss.base_noise_z: 2.0      # meters
    gnss.heading_noise: 0.02    # rad — for dual antenna
    gnss.max_hdop: 4.0          # reject fixes worse than this
    gnss.min_satellites: 4

    # Antenna lever arm — offset from base_link to GPS antenna in body frame
    # x=forward, y=left, z=up (meters). Leave at 0 if antenna is above base_link.
    # Lever arm correction only activates when heading is independently validated.
    gnss.lever_arm_x: 0.0
    gnss.lever_arm_y: 0.0
    gnss.lever_arm_z: 0.0

    # Optional second GPS receiver
    gnss.fix2_topic: ""

    # Heading topics — pick one or both
    gnss.heading_topic: "/gnss/heading"   # sensor_msgs/Imu
    gnss.azimuth_topic: ""                # compass_msgs/Azimuth (preferred standard)

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

### GPS antenna offset (lever arm)

If the GPS antenna is not at `base_link` — mounted on top of the robot, forward of center — its readings correspond to a different trajectory than `base_link`. Ignoring this injects position errors proportional to lever arm length times rotation rate.

FusionCore corrects for this using the rotation matrix from the current state: `p_antenna = p_base + R * lever_arm`. But this correction depends on heading — if heading is wrong the correction makes things worse. So FusionCore only activates lever arm correction when heading has been **independently validated** from a real source.

### Heading observability

This is the subtle one. A Kalman filter can reduce its own uncertainty about heading even when it has no real heading sensor — it does this by fitting the motion model to GPS position updates. The variance goes down, but the heading might still be wrong. Using that fake confidence to activate lever arm correction can destabilize the filter.

FusionCore tracks a `heading_validated_` flag that is only set true from a genuine independent source:

- **`DUAL_ANTENNA`** — dual antenna heading message received
- **`IMU_ORIENTATION`** — 9-axis AHRS published full orientation (only when `imu.has_magnetometer: true` — 6-axis IMUs drift in yaw and don't count)
- **`GPS_TRACK`** — robot has traveled >= 5 meters at speed >= 0.2 m/s with yaw rate <= 0.3 rad/s (geometrically observable from GPS track, not just accumulated distance)

Before any of these, lever arm is disabled regardless of what yaw variance says.

### Message covariances

FusionCore uses the covariance values sensors actually publish rather than ignoring them:

**GPS:** When `position_covariance_type == 3`, the full 3x3 covariance matrix is used including off-diagonal elements (correlated X/Y errors). RTK receivers in particular publish meaningful off-diagonal terms. When covariance type is lower, falls back to HDOP/VDOP scaling. Config params always available as override for sensors with bogus covariance.

**Wheel odometry:** Reads `twist.covariance` per-axis when available. A wheel-slip estimating odometry node that publishes real covariances gets the benefit automatically.

**IMU orientation:** Reads `orientation_covariance` from the message. Uses it directly when meaningful, falls back to config params when not.

### compass_msgs/Azimuth

peci1 on ROS Discourse suggested using `compass_msgs/Azimuth` as a standard heading message format. The upstream package is ROS 1 only. FusionCore ships a ROS 2 native port with the identical message definition.

FusionCore accepts `compass_msgs/Azimuth` on a configurable topic. It handles ENU/NED convention conversion, RAD/DEG units, and warns when magnetic north reference is used instead of geographic (magnetic heading has declination error that varies by location and changes over time).

### Delay compensation

GPS messages arrive 100-300ms after the fix was taken. Without compensation, delayed fixes are silently dropped — the filter's clock has already moved past that timestamp.

FusionCore saves a state snapshot (full 21-dimensional state + covariance) on every IMU update at 100Hz — 50 snapshots = 0.5 seconds of history. When a delayed GPS fix arrives, it finds the closest snapshot before the fix timestamp, restores that state, applies the fix at the correct time, then re-predicts forward to the current time in one step.

This is approximate retrodiction — the re-prediction forward uses the motion model rather than replaying the actual IMU history. For smooth motion at normal robot speeds the approximation error is small compared to GPS noise. Full IMU replay retrodiction (replaying every IMU message between the fix timestamp and now) is on the roadmap.

---

## Architecture

```
fusioncore/
├── fusioncore_core/              # Pure C++17 math library. Zero ROS dependency.
│   ├── include/fusioncore/
│   │   ├── ukf.hpp               # Unscented Kalman Filter — 43 sigma points
│   │   ├── state.hpp             # 21-dimensional state vector
│   │   ├── fusioncore.hpp        # Public API — FusionCore, FusionCoreConfig
│   │   └── sensors/
│   │       ├── imu.hpp           # Raw IMU + orientation measurement models
│   │       ├── encoder.hpp       # Wheel encoder measurement model
│   │       └── gnss.hpp          # GPS: ECEF, lever arm, quality scaling
│   └── src/
│       ├── ukf.cpp               # UKF: sigma points, predict, update
│       └── fusioncore.cpp        # Manager: sensors, snapshots, observability
└── fusioncore_ros/               # ROS 2 Jazzy wrapper
    ├── src/fusion_node.cpp       # Lifecycle node: all sensor callbacks
    ├── config/fusioncore.yaml    # Default configuration
    └── launch/fusioncore.launch.py
```

`fusioncore_core` has no ROS dependency by design. The core algorithm can run as firmware on embedded hardware for the Phase 2 hardware module without any ROS installation.

---

## Technical details

- **Filter:** Unscented Kalman Filter, 43 sigma points
- **State vector:** 21-dimensional — position (x,y,z), orientation (roll,pitch,yaw), linear velocity, angular velocity, linear acceleration, gyroscope bias (x,y,z), accelerometer bias (x,y,z)
- **GPS coordinate system:** ECEF — globally valid, no UTM zone boundaries or discontinuities
- **Bias estimation:** Continuous online estimation, no calibration required
- **GPS quality scaling:** Noise covariance scaled by HDOP/VDOP, or full 3x3 message covariance when available
- **Output rate:** 100Hz
- **Language:** C++17
- **License:** Apache 2.0

---

## Status

**Working and tested:**
- UKF core — 36 unit tests passing
- IMU + encoder + GPS fusion
- Automatic IMU bias estimation
- ECEF GPS conversion with quality-aware noise scaling
- Dual antenna heading — both `sensor_msgs/Imu` and `compass_msgs/Azimuth`
- IMU frame transform via TF
- GPS lever arm with heading observability guard
- Full 3x3 GPS covariance support
- Wheel odometry covariance support
- Multiple GPS receivers
- Heading observability tracking with source classification
- GPS delay compensation — retrodiction up to 500ms
- ROS 2 Jazzy lifecycle node at 100Hz

**Known limitations:**
- Delay compensation uses approximate retrodiction (one forward prediction step, not full IMU replay). Accurate for smooth motion, may introduce small inconsistencies during high-acceleration maneuvers.
- GNSS antenna offset assumes lever arm is fixed and known. Does not estimate it from data.

**Roadmap:**
- Full IMU replay retrodiction
- Mahalanobis distance outlier rejection (GPS jumps, wheel slip detection)
- Adaptive noise covariance — fully automatic, no YAML needed
- Ackermann and omnidirectional steering motion models
- Phase 2: hardware module — custom PCB, ICM-42688-P + MMC5983MA, 400Hz onboard fusion

---

## License

Apache 2.0. Includes explicit patent license grant that BSD-3 does not provide. Commercially safe.

---

## Support

Issues answered within 24 hours. Open a GitHub issue or find the original discussion on ROS Discourse.

This project exists because of community threads asking for a `robot_localization` replacement that actually works on ROS 2 Jazzy. If you hit a problem — open an issue. That feedback is what drives the roadmap.