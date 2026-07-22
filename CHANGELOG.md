# Changelog

All notable changes to FusionCore are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
Versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

---

## [0.3.3]: 2026-07-22

### Added
- **`encoder.topic` and `gnss.fix_topic` parameters.** Every optional and secondary input was already configurable (`imu.topic`, `imu2.topic`, `encoder2.topic`, `gnss.fix2_topic`, `gnss.velocity_topic`, and so on), but the two *primary* inputs were hardcoded: wheel odometry on `/odom/wheels` and GPS on `/gnss/fix`. That meant you could rename the second GPS receiver but not the first, and the only way to match your driver's topic was a launch remap, which is the less discoverable of the two mechanisms. Since almost no driver publishes on `/odom/wheels` (the ROS convention is `/odom` or a controller-namespaced variant) nearly every integration hit this, and pointing at the wrong topic fails silently: the filter runs, publishes, and simply never fuses that sensor. Both are now parameters with their previous values as defaults, so existing configs, launch files, and remaps behave exactly as before. `gnss.fix_topic` covers both message types (NavSatFix, or GPSFix when `gnss.use_gps_fix` is true). The node now also logs the IMU, encoder, and GNSS topics it actually subscribed to at startup, so the wiring is verifiable without guessing. The `/odom/wheels` default is kept deliberately: FusionCore publishes its own fused odometry, and defaulting to `/odom` would invite a feedback loop with its own output.
- **`rowcrop_rtk.yaml` config** for outdoor row-crop rovers running RTK GPS + IMU + wheel odometry (weeders, sprayers, scouts, seeders). Sets the RTK noise floor, the physical-plausibility gate (`gnss.max_speed`), gap-gated coast, and GPS-track heading for slow field speeds, and documents the dual-antenna heading options inline. The only per-robot values are the antenna lever arm and the rover's top speed.

### Fixed
- **IMU orientation was silently discarded at startup when the driver left `orientation_covariance` at zero.** The bias window required a strictly positive covariance to accept an orientation, while the runtime path correctly rejected only a negative one. Per the `sensor_msgs/Imu` spec, `-1` means "no orientation data" and all-zeros means "covariance unknown", and plenty of drivers publish a perfectly good quaternion with an unfilled covariance. Those were treated as having no orientation at all, which skipped accelerometer bias initialisation entirely: the bias stayed at zero, so whatever component of gravity the IMU's mounting tilt produced was left in the acceleration channel. A constant acceleration error double-integrates, so position ran away smoothly, reported from the field as roughly 170 m of drift on a 6 m out-and-back with a BNO085. The startup check now matches the runtime check and additionally rejects an all-zero quaternion, which is not a rotation. When no usable orientation is present the log line is now a warning that explains the consequence rather than a quiet info message. Adds `test_imu_orientation_validity` (6 tests) covering the covariance convention and the resulting bias error.
- **`tools/field_record.launch.py` recorded bags with no filter output.** The field-day recording launch started FusionCore with a plain `Node()` and never triggered the lifecycle CONFIGURE transition, so the node sat in `unconfigured` and published nothing. The bag still captured the raw sensor topics, but `/fusion/odom`, `/fusion/debug/gnss_status`, and `/fusion/debug/filter_health` came back empty, which is only discovered after the field day is over. The launch now configures and activates the node, and waits until it is active before starting the recorder so the first seconds of the bag are not missing the `/fusion` topics.
- **`fusioncore.launch.py` and `fusioncore_duatic.launch.py` now bring the node up automatically.** FusionCore is a lifecycle node, but these two launch files started the process and never triggered the initial CONFIGURE transition, so the node sat in `unconfigured` doing nothing: no error, no data, just silence. A first-time user following the README would reasonably conclude FusionCore was broken. Both launch files now emit CONFIGURE and then ACTIVATE, the same pattern `fusioncore_nav2.launch.py` and the Gazebo demo already used. Adds an `autoconfigure` launch argument (default `true`); set it to `false` when a lifecycle manager such as `nav2_lifecycle_manager` drives the node.

---

## [0.3.2]: 2026-07-06

### Fixed
- **Backward time-jumps no longer crash the filter.** A non-monotonic sensor timestamp (a clock rewind, a replayed bag, or a WSL2 clock glitch) previously produced a negative `dt` that drove the UKF covariance non-positive-definite and aborted the process. `predict_to` now re-syncs the clock on a backward jump instead of integrating a negative step, and the UKF floors covariance eigenvalues rather than throwing. Validated on a real NCLT run that used to SIGABRT within ~2 minutes and now survives end to end.
- **Sustained GPS spikes no longer defeat the outlier gate.** Previously the chi2 gate rejected a spike at first, but after `gnss.coast_n` consecutive rejections coast mode inflated the position process noise until the gate widened enough to admit the spike (on an 8 s, 60 m spike the filter lunged to ~62 m error after ~5 s). Coast mode is meant for re-acquisition after a GPS gap, so firing it for a continuously present, consistently rejected GPS (a persistent multipath spike) was the bug. Rejection-triggered coast (and the recovery P-inflate) now only fire when the rejection streak began after a real GPS gap. Validated on a deterministic repro (sustained-spike peak error 62 m to 1.9 m, post-outage re-acquisition preserved) and in the Gazebo demo (FusionCore RMSE 18.15 m to 2.76 m, now clearly below robot_localization's 18.6 m). Adds `test_gnss_coast` (sustained-spike-stays-rejected, outage-still-recovers).
- **Gazebo outdoor demo now works end to end.** The GPS publisher tracked a static crop row instead of the robot (the ros_gz bridge emits empty frame_ids, so the body finder fell through to a heuristic that locked onto scenery): the robot is now identified by its model height. The demo also mixed wall-clock nodes with sim time, which blew up the velocity estimate under headless: every node now runs on sim time. Added a `base_link -> gnss_link` static TF (removes lever-arm warning spam) and the launch now triggers the initial lifecycle CONFIGURE.

### Added
- **Physical-plausibility GNSS gate (`gnss.max_speed`)**: rejects a fix farther from the filter's predicted position than the robot could have moved or drifted since the last accepted fix (`max_speed * dt + margin`). This catches an outlier cluster arriving at a GPS-blackout boundary that a coast-relaxed chi2 gate would otherwise admit. It is a per-platform kinematic spec (like wheel radius), not per-run tuning. Off by default (0.0). New rejection reason `IMPLAUSIBLE_JUMP`; adds two gate unit tests.
- **Adaptive magnetic-disturbance rejection (`magnetometer.field_strength` / `magnetometer.field_tolerance`)**: a clean magnetometer reading's corrected magnitude equals the local Earth field, so a reading whose magnitude deviates (a nearby motor, steel, or rebar) is rejected even when its direction would pass the heading chi2 gate. This is what makes magnetometer absolute-heading robust enough to bound heading drift through multi-minute GPS blackouts on real outdoor hardware. Off by default (field_strength 0.0). Adds three unit tests, including a blackout scenario where heading runs away to ~113 deg on encoder + gyro alone but the magnetometer pins it to ~0.
- **`gnss.coast_min_gap_s`** parameter (default 1.0 s): minimum preceding GPS gap before rejection-triggered coast may fire. Set to 0 to restore the previous gap-agnostic behavior.
- **`headless` and `start_delay` launch args** for `fusioncore_demo.launch.py`: run Gazebo with no GUI (CI / offscreen), and adjust when the robot starts driving.
- **WSL2 UDP-only Fast-DDS profile** (`fusioncore_gazebo/config/fastdds_udp.xml`): the shared-memory transport fails on WSL2 (`RTPS_TRANSPORT_SHM` errors), dropping `/cmd_vel` and corrupting `/clock`. Point `FASTRTPS_DEFAULT_PROFILES_FILE` at this profile for reliable comms. See the troubleshooting and simulation docs.
- **Benchmark regression tracking**: `evaluate.py` now emits `metrics.json`, and `tools/check_benchmark_regression.py` compares a run against `tools/benchmark_baseline.json` so a tuning change that silently worsens another sequence is caught instead of shipping unnoticed. Documented in `tools/benchmark_regression.md`.

### Changed
- **Documentation accuracy pass**: the benchmark and comparison pages no longer describe the magnetometer as a roadmap item (it ships) or `gnss.max_speed` as hypothetical (it ships), the long-blackout losses are explained honestly as dead-reckoning drift rather than the visible GPS transients, and the published NCLT numbers now carry a note that they predate a controlled full-suite re-run (the 2013-04-05 figure has regressed 12.1 m to ~19.4 m, still a 93% win).

---

## [0.3.1]: 2026-06-24

### Added
- **Raw magnetometer heading fusion**: FusionCore now subscribes to `sensor_msgs/MagneticField` and fuses the heading as a 1-DOF UKF update. Applies hard/soft iron correction (configurable 3-vector bias + 3x3 scale matrix) and tilt compensation using the current filter roll/pitch before fusing. Chi-squared gate (chi2(1, 0.99) = 9.21 by default) rejects magnetic spikes. Heading source hierarchy: DUAL_ANTENNA overrides MAGNETOMETER overrides GPS_TRACK. Enable with `magnetometer.enabled: true`. Requires calibration: collect data with a full 360-degree rotation and run `imu_calib` or `magneto` to get `hard_iron` and `soft_iron` values.
- **`mag_outlier_count` in FilterHealth**: cumulative magnetometer rejection count now published on `/fusion/debug/filter_health` alongside the existing GNSS, IMU, and encoder outlier counts.
- **Magnetometer diagnostics**: when `magnetometer.enabled: true`, a `fusioncore: Magnetometer` status block appears in `/diagnostics` with health state (OK/STALE/NOT_INIT) and outlier count.
- **`magnetometer.topic` subscriber row in topics reference**: documentation now lists the `/imu/mag` subscriber.
- 12 new unit tests: flat heading (east/north/west), declination offset, hard iron correction, tilt compensation, UKF convergence, chi2 gate rejection, heading source hierarchy, outlier counter.
- **IMU lever arm centripetal compensation**: when an IMU lever arm is configured, the measurement function adds the centripetal term (omega x (omega x r)) to the predicted acceleration so an off-center IMU is modeled correctly. Falls back to the zero-allocation hot path when the lever arm is zero, so platforms without one pay no cost.
- **GPS pre-heading lever arm option**: `gnss.apply_lever_arm_pre_heading` applies the antenna lever arm from the first fix, before heading is validated, for setups where the lever arm is known up front.
- **IMU frame auto-resolve**: `imu.frame_id` override resolves the IMU TF frame when a driver publishes an empty `header.frame_id`. Leave empty to use the message frame as before.
- **Lifecycle `autostart` parameter** (default `true`): the node self-transitions `configure` to `activate` about 200 ms after `on_configure()` returns, so it runs standalone without a lifecycle manager. Set `autostart: false` when `nav2_lifecycle_manager` drives the node, to avoid a double-activate.
- **Certified hardware config registry**: curated, tested YAML configs for known hardware setups.
- **GNSS Doppler bridge package and Gazebo demo infrastructure**: new `fusioncore_ublox` package plus simulation tooling for an end-to-end outdoor scenario.
- **Production-grade Gazebo demo with multi-event GPS scenario**: scripted GPS outlier and dropout events for reproducible demos.
- **Docker support**: published image at `ghcr.io/manankharwar/fusioncore` with CI auto-tagging on release, plus a Docker tutorial in the docs.
- **Field tooling**: bag-recording launch file, a field day checklist, and four troubleshooting guides.

### Changed
- **`fusioncore_ros` migrated to `ament_cmake_auto`** for ROS 2 Lyrical support: Lyrical dropped `ament_target_dependencies`, so build dependencies now come from `package.xml`. Added `std_msgs` as an explicit dependency for the `GnssStatus`/`FilterHealth` messages. Eigen3 and PROJ stay explicit.
- **tf2 headers use `.hpp` uniformly**: Lyrical renamed `tf2/LinearMath/*.h` to `.hpp`. The temporary `__has_include` guard was dropped in favor of `.hpp` everywhere. Still builds on Jazzy.

### Fixed
- **Sensor subscriptions now use `SensorDataQoS` (BEST_EFFORT)**: all sensor subscriptions previously used the default reliable QoS, which silently fails to connect to standard sensor drivers that publish best-effort. Switching to `SensorDataQoS` makes IMU, GPS, and encoder topics connect out of the box.

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
