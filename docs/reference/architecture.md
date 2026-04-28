# Architecture & Technical Details

## Package structure

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
│       ├── ukf.cpp               # UKF: sigma points, predict, update
│       └── fusioncore.cpp        # Manager: outlier rejection, adaptive noise,
│                                 #          snapshots, observability, delay compensation
├── fusioncore_ros/               # ROS 2 Jazzy wrapper
│   ├── src/fusion_node.cpp       # Lifecycle node: sensor callbacks, TF validation,
│   │                             #   ZUPT, diagnostics, /fusion/pose, reset service
│   ├── config/fusioncore.yaml    # Default configuration
│   └── launch/
│       ├── fusioncore.launch.py
│       └── fusioncore_nav2.launch.py
└── fusioncore_gazebo/            # Simulation world
    ├── worlds/fusioncore_test.sdf
    └── launch/
        ├── fusioncore_gazebo.launch.py
        └── integration_test.py
```

The core library (`fusioncore_core`) has zero ROS dependency. It can be used in any C++ project independently of ROS.

---

## Technical details

| Property | Value |
|---|---|
| Filter | Unscented Kalman Filter, 45 sigma points |
| State vector | 22-dimensional: position (x,y,z), orientation (qw,qx,qy,qz), linear velocity, angular velocity, linear acceleration, gyroscope bias (x,y,z), accelerometer bias (x,y,z) |
| GPS coordinate system | Configurable via PROJ. Default: ECEF (EPSG:4978, globally valid). Supports any PROJ-compatible CRS including UTM zones. |
| Bias estimation | Continuous online estimation, no calibration required |
| GPS noise scaling | Covariance scaled by HDOP/VDOP, or full 3×3 message covariance when available |
| Outlier rejection | Mahalanobis chi-squared gating at 99.9th percentile per sensor dimension |
| Adaptive noise | Sliding window innovation tracking, EMA R update |
| Delay compensation | IMU ring buffer replay retrodiction up to 500 ms |
| ZUPT | Automatic zero velocity updates when stationary |
| Output rate | Configurable, default 100 Hz |
| Language | C++17 |
| License | Apache 2.0 (includes patent grant) |
| ROS 2 | Jazzy Jalisco, native lifecycle node |

---

## Status

**Working and tested:**

- UKF core: 39 unit tests passing
- IMU + encoder + GPS fusion
- Automatic IMU bias estimation
- ECEF GPS conversion with quality-aware noise scaling
- Dual antenna heading: `sensor_msgs/Imu` and `compass_msgs/Azimuth`
- IMU frame transform via TF
- TF validation at startup with fix commands
- GPS lever arm with heading observability guard
- Full 3×3 GPS covariance support
- Mahalanobis outlier rejection
- Adaptive noise covariance
- GPS delay compensation: full IMU replay retrodiction up to 500 ms
- Non-holonomic ground constraint (VZ=0)
- Zero velocity updates (ZUPT)
- Per-sensor diagnostics at 1 Hz
- Filter reset service
- Sensor dropout detection
- PROJ CRS coordinate transform
- Gazebo Harmonic simulation world

**Known limitations:**

- GNSS antenna lever arm is fixed and known: not estimated from data
- In Gazebo simulation, residual y-axis drift (~0.3 m) can occur from Gazebo physics: not a filter error
- Mecanum drive lateral velocity is not predicted by the motion model

**Roadmap:**

- Ackermann and omnidirectional steering motion models
- Mecanum drive motion model
- Auto-derive GNSS lever arm from TF header.frame_id
