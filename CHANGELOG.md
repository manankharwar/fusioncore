# Changelog

All notable changes to FusionCore are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
Versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [0.1.1]: 2026-04-03

### Fixed
- **UKF stability**: resolved numerical instability in predict step; covariance matrix
  no longer diverges during long runs without sensor updates.
- **GPS fusion**: corrected position bias introduced by incorrect ECEF→ENU origin
  anchoring; fixes steady-state position offset.
- **Position bias**: removed residual bias accumulation in the motion model that
  appeared after extended straight-line travel.
- **Eigen rosdep key**: added missing `eigen` entry to `rosdep` dependencies so
  the buildfarm can resolve the dependency without manual intervention.
- **Sensor dropout handling**: improved graceful degradation when IMU or GNSS
  messages stop arriving mid-run.

### Added
- **`compass_msgs`**: moved into this repo as a first-party package; provides the
  `compass_msgs/Azimuth` message type for dual-antenna and magnetometer heading.
- **Mahalanobis outlier rejection**: GPS jumps and other sensor spikes are now
  gated before the UKF update step; position remains stable during brief GNSS outages.
- **UKF numerical stability hardening**: symmetric covariance enforcement and
  near-zero variance clamping added throughout the filter.
- **IMU gravity model**: accelerometer measurement function now correctly accounts
  for the gravity vector in the body frame (ENU z-up convention).
- **Full IMU replay retrodiction**: GNSS delay compensation now replays every
  buffered IMU message rather than using a single approximate `predict(dt)` call;
  handles up to 500 ms late measurements.
- **Docs published to docs.ros.org**: all four packages (`fusioncore_core`,
  `fusioncore_ros`, `fusioncore_gazebo`, `compass_msgs`) are now live under the
  Jazzy distribution.

### Changed
- `.vscode/` removed from version control; added to `.gitignore`.
- All package versions bumped to `0.1.1` to align with the rosdistro release.

---

## [0.1.0]: initial release

### Added
- UKF core: predict, update, motion model, angle normalisation (7/7 tests passing).
- IMU sensor model: measurement function, noise matrix, bias handling.
- Encoder sensor model: velocity + yaw-rate fusion, IMU+encoder bias estimation.
- GNSS sensor model: ECEF/ENU conversion, HDOP/VDOP noise scaling, dual-antenna
  heading, full 3×3 covariance support, multiple receiver support.
- GNSS lever-arm correction with yaw-confidence gate.
- GNSS delay compensation: retrodiction with 50-snapshot state buffer.
- IMU orientation input: accepts full orientation from AHRS/IMU (e.g. BNO08x,
  VectorNav, Xsens) via `sensor_msgs/Imu.orientation`.
- Dual-antenna heading wired to ROS topic.
- Adaptive noise covariance: sliding-window innovation tracking, automatic R
  estimation for IMU / encoder / GNSS.
- TF validation: prints exact fix command when transform is missing.
- ROS 2 lifecycle node (`fusioncore_ros`): IMU / encoder / GNSS subscribers,
  odometry publisher, TF broadcaster, single YAML config.
- `compass_msgs/Azimuth` support: ENU/NED conversion, magnetic/geographic north
  warning.
- Gazebo integration tests: all 4 pass.
- 42/42 unit tests wired into `colcon test`.
- Apache 2.0 licence.
