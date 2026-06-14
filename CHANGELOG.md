# Changelog

All notable changes to FusionCore are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
Versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [0.3.1]: 2026-06-14

### Added
- **Raw magnetometer heading fusion**: FusionCore now subscribes to `sensor_msgs/MagneticField` and fuses the heading as a 1-DOF UKF update. Applies hard/soft iron correction (configurable 3-vector bias + 3x3 scale matrix) and tilt compensation using the current filter roll/pitch before fusing. Chi-squared gate (chi2(1, 0.99) = 9.21 by default) rejects magnetic spikes. Heading source hierarchy: DUAL_ANTENNA overrides MAGNETOMETER overrides GPS_TRACK. Enable with `magnetometer.enabled: true`. Requires calibration: collect data with a full 360-degree rotation and run `imu_calib` or `magneto` to get `hard_iron` and `soft_iron` values.
- **`mag_outlier_count` in FilterHealth**: cumulative magnetometer rejection count now published on `/fusion/debug/filter_health` alongside the existing GNSS, IMU, and encoder outlier counts.
- **Magnetometer diagnostics**: when `magnetometer.enabled: true`, a `fusioncore: Magnetometer` status block appears in `/diagnostics` with health state (OK/STALE/NOT_INIT) and outlier count.
- **`magnetometer.topic` subscriber row in topics reference**: documentation now lists the `/imu/mag` subscriber.
- 12 new unit tests: flat heading (east/north/west), declination offset, hard iron correction, tilt compensation, UKF convergence, chi2 gate rejection, heading source hierarchy, outlier counter.

---

## [0.3.0]: 2026-06-04

### Added
- **GNSS observability topics**: every GPS fix now publishes a structured message on `/fusion/debug/gnss_status` with the exact rejection reason (`ACCEPTED`, `CHI2_FAILED`, `HDOP_HIGH`, `MIN_SATS`, `FIX_TYPE_LOW`, `DELAY_TOO_LARGE`), Mahalanobis distance squared vs the chi2 threshold, fix metadata, and current coast mode state. Replaces the generic warning log line with auditable per-fix data.
- **Filter health topic**: `/fusion/debug/filter_health` publishes at 1 Hz with innovation norms per sensor, position and heading 1-sigma uncertainty (meters and degrees), heading source, GPS coast mode state, and cumulative outlier counts. All fields are plain `float64` — plottable directly in Foxglove, PlotJuggler, or rqt without a custom panel.
- **Two new message types**: `fusioncore_ros/msg/GnssStatus` and `fusioncore_ros/msg/FilterHealth`. No external dependencies added.
- **Lever arm sigma gating**: lever arm correction now requires heading uncertainty below `gnss.lever_arm_max_heading_sigma_deg` (default 20°) in addition to `heading_validated`. During prolonged turns where heading degrades, the lever arm is silently disabled until heading tightens. `lever_arm_used` and `heading_sigma_deg` published on `/fusion/debug/gnss_status` for every fix.
- **Configurable heading motion thresholds**: `gnss.track_heading_min_speed` and `gnss.track_heading_max_yaw_rate` were previously hardcoded at 0.2 m/s and 0.3 rad/s. Now exposed as YAML parameters so platforms with different motion profiles can tune when GPS displacement counts toward heading observability.
- **Complete config YAML**: `fusioncore.yaml` rewritten to document all 87 parameters with inline explanations. Every hardware YAML updated with missing params (`q_encoder_wz_bias`, `outlier_threshold_vslam`, `adaptive.ground_constraint`, correct motion models).

### Fixed
- **Mahalanobis distance computed once per GPS fix**: previously `predict_measurement` ran twice for GNSS updates (once in `is_outlier`, once implicitly). Now computed inline with a single LDLT factorization that is also stored for observability.
- **`configuration.md` had a non-existent param**: `gnss.degraded_noise_multiplier` was documented but never implemented. Removed. Also removed a duplicate coast mode section.
- **Husky config missing motion model**: `clearpath_husky.yaml` had no `motion_model` set. Added `DifferentialDrive` — Husky is a differential drive robot and the config should reflect that.
- **CITATION.cff stale**: was at 0.2.3 while code was at 0.2.4. Synced.

### Changed
- Rejection log messages now include structured fields: `GNSS fix rejected: CHI2_FAILED (hdop=1.20, d2=847.3, threshold=16.27)` instead of the previous generic message.

---

## [0.2.4]: 2026-05-19

### Added
- **`gps_msgs/GPSFix` support**: set `gnss.use_gps_fix: true` to subscribe to `/gnss/fix` as `gps_msgs/GPSFix` instead of `sensor_msgs/NavSatFix`. Unlocks RTK_FLOAT status (status code 20, unreachable via NavSatFix), uses receiver-native `hdop`/`vdop` fields, `satellites_used` for the quality gate, and `err_horz`/`err_vert` (95% CI bounds) as a fallback covariance source. Default is `false`; existing NavSatFix setups are unaffected.

### Changed
- `package.xml` (both packages): maintainer name corrected to Manan Kharwar, maintainer email updated
- `package.xml` (both packages): added `<url>` tags for website, repository, bugtracker, and documentation so index.ros.org renders clickable links

---

## [0.2.3]: 2026-05-10

### Added
- **VSLAM pose fusion**: accepts `nav_msgs/Odometry` from ORB-SLAM3, MOLA, slam_toolbox, or any VIO/LIO source via `vslam.topic`. Enables visual-inertial fusion without GPS.
- **Dual IMU support**: second IMU input via `imu2.topic` with independent noise and outlier parameters.
- **GPS velocity fusion**: fuses Doppler-derived velocity from a GNSS receiver via `gnss.velocity_topic`.
- **Radar Doppler velocity fusion**: fuses radar radial velocity via `radar.velocity_topic`.
- **Pluggable motion models**: select differential drive, Ackermann, or omnidirectional via `motion_model` parameter.
- **Sensor wait**: filter holds initialization until all declared sensors have published at least once.
- **Deterministic replay**: `use_sim_time`-aware replay for reproducible benchmark runs.
- **Docker container and `quick_test.sh`**: one-command environment for testing without a full ROS install.
- **`imu.topic` parameter**: override the IMU subscription topic at runtime without launch file changes.
- **Adaptive R-inflation**: breaks cascading outlier rejection loops when GPS quality degrades gradually.
- **`publish.tf` toggle**: suppress TF broadcast independently of odometry publishing for multi-robot setups.
- Ackermann vehicle configuration and documentation.
- GPS velocity and wheel slip detection documentation.

### Fixed
- VSLAM frame alignment and reinitialization recovery after GPS-denied stretches.
- `encoder2` noise parameters and config accuracy.
- `nav2_params` global_frame corrected from `map` to `odom` for GPS-only navigation.
- Dockerfile apt list errors on fresh builds.
- `quick_test.sh` four-check validation on clean setup.

---

## [0.2.2]: 2026-05-05

### Fixed
- **`init.stationary_window` hangs with zero-timestamp IMU drivers**: the bias window used message timestamps to measure elapsed time. IMU drivers that publish `stamp={sec=0, nanosec=0}` caused the window to never complete, silently blocking filter initialization and preventing `/fusion/odom` from publishing. Window timing now uses wall clock (`this->now()`), making it immune to message timestamp values.
- **`publish.force_2d` incomplete**: `force_2d: true` zeroed `position.z` in the published odometry and TF but left `twist.linear.z` (vertical velocity) non-zero. For a ground robot, publishing a non-zero VZ is misleading. Both are now zeroed consistently.

### Added
- **Troubleshooting page**: covers the most common failure modes: lifecycle not activating, Madgwick filter conflict, zero-timestamp IMU drivers, TF conflicts, outlier gate tuning, and more.
- **RTABMAP + Madgwick separation guide** in `icp-indoor.md`: documents the correct IMU topic split when running FusionCore alongside RTABMAP and `icp_odometry`.

---

## [0.2.1]: 2026-04-28

### Fixed
- **`duatic_mecanum.yaml`**: `ukf.q_orientation` was `0.01`, causing yaw drift at IMU rates. Corrected to `1.0e-9`.
- **CMakeLists versions**: `fusioncore_ros` and `fusioncore_core` project versions were out of sync with `package.xml`. Both now track `0.2.1`.

### Added
- **`wheels_indoor.yaml`**: new hardware config for any indoor robot with IMU + wheel odometry, no GPS. Covers differential drive, mecanum, Turtlebot3, ROS 2 Control, Nav2 default setups.
- **`icp_indoor.yaml`**: new hardware config for indoor robots using LiDAR ICP odometry (KISS-ICP, rtabmap `icp_odometry`) instead of or alongside wheel encoders.
- **Hardware docs**: new decision table ("Which setup are you?") and per-config setup guides for indoor wheel and ICP setups.

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
