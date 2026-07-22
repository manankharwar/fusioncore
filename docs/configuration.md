# Configuration

FusionCore is configured with a single YAML file passed to the launch file:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

---

## Full parameter reference

```yaml
fusioncore:
  ros__parameters:
    base_frame: base_link    # must match your robot's base TF frame
    odom_frame: odom
    publish_rate: 100.0
    publish.force_2d: true   # zeroes Z position and Z velocity in published output. Use for ground robots.
    publish.tf: true         # set false to suppress the odom->base_link TF broadcast.
                             # /fusion/odom keeps publishing. Use when another node owns
                             # the odom->base_link transform, or when running two
                             # FusionCore instances where only one should broadcast TF.

    # ── IMU ──────────────────────────────────────────────────────────────────
    imu.gyro_noise: 0.005       # rad/s: from your IMU datasheet (ARW spec)
    imu.accel_noise: 0.1        # m/s² : from your IMU datasheet (VRW spec)
    imu.has_magnetometer: false # true for 9-axis (BNO08x, VectorNav, Xsens)
                                # false for 6-axis: yaw comes from gyro integration
    imu.remove_gravitational_acceleration: false
    # Set true if your IMU driver already subtracted gravity.
    # Check: echo linear_acceleration.z at rest.
    #   ~9.8 m/s² → leave false (driver publishes raw)
    #   ~0.0 m/s² → set true (driver already removed it)
    # NOTE: opposite of robot_localization's imu0_remove_gravitational_acceleration.

    imu.frame_id: ""  # override IMU TF frame. Leave empty (default) to use msg->header.frame_id.
                      #
                      # When to set this:
                      #   Gazebo Harmonic TurtleBot3 publishes "waffle/imu_link/tb3_imu" instead
                      #   of "imu_link". FusionCore can't find that frame in the TF tree.
                      #   Fix: set imu.frame_id to your URDF frame name (e.g. "imu_link").
                      #
                      # WARNING: do NOT set this to "base_link".
                      #   When imu.frame_id equals base_frame, FusionCore skips the TF lookup
                      #   entirely and treats IMU measurements as already in base_link frame.
                      #   If your IMU is mounted at any angle relative to base_link, its
                      #   measurements will be fused with the wrong rotation, silently
                      #   corrupting the orientation estimate. Leave empty unless your driver
                      #   publishes with no frame_id at all.

    # Optional second IMU. When non-empty, FusionCore subscribes to this topic
    # and fuses each message as an independent measurement of the same state.
    # Both IMUs must be in (or TF-transformable to) base_link frame.
    # The second IMU uses the same noise model as the primary.
    # Useful when your platform has two IMUs and you want redundancy without
    # pre-merging them with imu_filter_madgwick or similar. Leave empty to disable.
    imu2.topic: ""
    imu2.frame_id: ""
    imu2.remove_gravitational_acceleration: false

    # ── Wheel encoders ────────────────────────────────────────────────────────
    # Wheel odometry topic (nav_msgs/Odometry; only the twist is fused).
    # The default is deliberately NOT the conventional /odom: FusionCore publishes
    # its own fused odometry, so subscribing to /odom would invite a feedback loop
    # with its own output. Point this at your driver's topic instead.
    encoder.topic: "/odom/wheels"   # e.g. "/odom" or "/diff_drive_controller/odom"
    encoder.vel_noise: 0.05     # m/s
    encoder.yaw_noise: 0.02     # rad/s

    # Optional second velocity source (lidar odometry, visual odometry, etc.)
    # Must publish nav_msgs/Odometry with velocity in twist field.
    # FusionCore does NOT accept sensor_msgs/LaserScan or PointCloud2 directly.
    # A scan-matching node must sit between your LiDAR and FusionCore:
    #
    #   LaserScan / PointCloud2
    #       → KISS-ICP             → /kiss/odometry      (nav_msgs/Odometry)
    #       → rtabmap icp_odometry → /icp_odom           (nav_msgs/Odometry)
    #       → rf2o_laser_odometry  → /odom               (nav_msgs/Odometry)
    #
    # Note: slam_toolbox publishes PoseWithCovarianceStamped, not Odometry.
    # It cannot be used directly as encoder2; wrap it or use KISS-ICP instead.
    encoder2.topic: ""          # e.g. "/kiss/odometry" or "/icp_odom"
    encoder2.vel_noise: 0.05    # m/s  fallback when message covariance is zero
    encoder2.yaw_noise: 0.02    # rad/s fallback when message covariance is zero

    # ── GPS ───────────────────────────────────────────────────────────────────
    # Primary GPS fix topic. Read as sensor_msgs/NavSatFix, or gps_msgs/GPSFix
    # when gnss.use_gps_fix is true (below). Set this to your driver's topic
    # rather than writing a launch remap.
    gnss.fix_topic: "/gnss/fix"  # e.g. "/fix", "/ublox/fix", "/gps/fix"

    gnss.base_noise_xy: 1.0     # m: baseline sigma at HDOP=1
                                # scaled automatically by HDOP from the message
                                # standard autonomous GPS: 1.0–2.5
                                # RTK float: 0.5,  RTK fixed: 0.015
    gnss.base_noise_z: 2.0      # m
    gnss.heading_noise: 0.02    # rad: for dual antenna heading

    gnss.max_hdop: 4.0          # reject fixes with HDOP worse than this
    gnss.min_satellites: 4
    gnss.min_fix_type: 1        # 1=GPS, 2=DGPS, 3=RTK_FLOAT, 4=RTK_FIXED
                                # NavSatFix: status=2 maps to RTK_FIXED. RTK_FLOAT (3)
                                # is unreachable via NavSatFix; use gnss.use_gps_fix
                                # below if your receiver publishes gps_msgs/GPSFix.

    gnss.use_gps_fix: false     # Set true when your driver publishes gps_msgs/GPSFix
                                # on gnss.fix_topic instead of sensor_msgs/NavSatFix.
                                # GPSFix unlocks RTK_FLOAT status, uses receiver-native
                                # hdop/vdop values, satellites_used, and err_horz/err_vert
                                # as a fallback covariance. Default false: NavSatFix works
                                # with all receivers. See GPS Receiver Setup below.

    # Antenna lever arm: offset from base_link to GPS antenna in body frame
    # x=forward, y=left, z=up (meters). Leave 0.0 if antenna is above base_link.
    # Correction only activates after heading is independently validated.
    gnss.lever_arm_x: 0.0
    gnss.lever_arm_y: 0.0
    gnss.lever_arm_z: 0.0

    # Second GPS receiver (optional)
    gnss.fix2_topic: ""
    gnss.lever_arm2_x: 0.0
    gnss.lever_arm2_y: 0.0
    gnss.lever_arm2_z: 0.0

    # GPS velocity (optional): fuses horizontal GPS speed as an independent
    # measurement update, separate from wheel odometry.
    # Accepts nav_msgs/Odometry with velocity in ENU frame:
    #   twist.linear.x = east velocity (m/s)
    #   twist.linear.y = north velocity (m/s)
    #   twist.covariance[0,7] used when positive; falls back to encoder noise otherwise
    # FusionCore rotates ENU -> body frame internally using the current quaternion.
    # Enables slip detection: innovation between GPS velocity and wheel odometry
    # reveals wheel slip. Works with F9P, Septentrio, or any receiver that
    # publishes velocity. Leave empty to disable.
    gnss.velocity_topic: ""

    # Radar Doppler velocity (optional): fuses ego-velocity from a 4D imaging radar
    # (Continental ARS548, Oculii Eagle, Aptiv ESR, etc.) as an independent measurement.
    # Accepts nav_msgs/Odometry with velocity in robot body frame:
    #   linear.x = forward speed (m/s), linear.y = lateral speed (m/s)
    # A bridge node extracts ego-velocity from raw Doppler point cloud and publishes here.
    # Works indoors and in all weather: rain, fog, dust, darkness.
    # radar.vel_noise is used when the message covariance is zero or negative.
    radar.velocity_topic: ""
    radar.vel_noise: 0.1        # m/s fallback noise when message has no covariance

    # Heading input: pick one or both
    gnss.heading_topic: ""      # sensor_msgs/Imu (dual antenna heading)
    gnss.azimuth_topic: ""      # compass_msgs/Azimuth (preferred REP-145 standard)

    # ── Raw magnetometer heading ──────────────────────────────────────────────
    # Fuses sensor_msgs/MagneticField directly into the UKF as a 1-DOF heading
    # measurement. Applies hard/soft iron correction then tilt compensation using
    # the current filter roll/pitch. Useful when GPS is unavailable and the robot
    # is stationary (GPS track heading requires motion; magnetometer does not).
    # Heading source hierarchy: DUAL_ANTENNA > MAGNETOMETER > GPS_TRACK.
    # Requires calibration: collect data by rotating the robot through a full circle
    # and use imu_calib (ROS) or magneto (desktop) to get hard_iron and soft_iron values.

    magnetometer.enabled: false
    magnetometer.topic: "/imu/mag"   # sensor_msgs/MagneticField publisher

    magnetometer.noise_rad: 0.05
    # Standard deviation of heading estimate (rad) after correction.
    # 0.05 rad (~3 deg) is typical for a well-calibrated sensor in benign conditions.
    # Loosen to 0.15-0.30 near motors or variable magnetic fields.

    magnetometer.chi2_threshold: 9.21
    # Chi-squared outlier gate: chi2(1, 0.99) = 9.21 for 1-DOF heading.
    # Rejects magnetic spikes. Tighten to 3.84 (chi2(1,0.95)) in clean environments.

    magnetometer.declination_rad: 0.0
    # Magnetic declination: offset from magnetic north to true north (rad).
    # Positive east. Look up your location at https://www.magnetic-declination.com
    # Leave 0.0 when FusionCore can self-correct via GPS: the constant heading
    # offset is absorbed by the filter over time.

    magnetometer.hard_iron: [0.0, 0.0, 0.0]
    # Constant bias offset in body frame (Tesla): [x, y, z].
    # Estimated by rotating the sensor through a full circle and computing
    # (max + min) / 2 per axis. Use imu_calib or magneto for best results.

    magnetometer.soft_iron: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    # 3x3 scale+rotation matrix in row-major order. Identity = disabled.
    # Corrects for elliptical distortion in the magnetic field.
    # Estimated alongside hard iron using imu_calib or magneto.

    magnetometer.field_strength: 0.0
    # Local Earth total-field magnitude, in the SAME units as the incoming reading
    # (e.g. ~0.48 for Gauss, ~48 for microtesla). A clean reading's corrected
    # magnitude equals this; a nearby motor or steel structure distorts it and
    # produces a wrong heading the chi2 gate cannot reliably catch. When the
    # magnitude deviates by more than field_tolerance the reading is rejected.
    # Look up the total field at https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
    # 0.0 = disabled (no magnitude check).

    magnetometer.field_tolerance: 0.2
    # Allowed fractional deviation of the field magnitude before a reading is
    # treated as locally disturbed. 0.2 = accept within +-20% of field_strength.

    # ── GPS coast mode ────────────────────────────────────────────────────────
    # During GPS blackouts, inflates process noise so P grows and the chi2 gate
    # relaxes by the time GPS resumes. Prevents the filter from rejecting its
    # own recovery fixes because dead-reckoning drift made innovations look like
    # outliers. See How It Works for the full explanation.

    gnss.coast_n: 3
    # Consecutive chi2 GPS rejections before entering coast mode. 0 = disabled.

    gnss.coast_min_gap_s: 1.0
    # Rejection-triggered coast only fires if the rejection streak began after a
    # GPS gap of at least this many seconds (the receiver actually went silent and
    # the filter dead-reckoned). A continuously present GPS that keeps failing the
    # chi2 gate is a persistent outlier (e.g. a sustained multipath spike), not
    # filter drift, so inflating P to re-admit it would let the outlier defeat the
    # gate. Gating on a preceding gap keeps a sustained spike rejected for its full
    # duration while preserving post-outage re-acquisition. 0 = old behavior.

    gnss.coast_q_factor: 10.0
    # Q_position multiplier in coast mode. Controls how fast position uncertainty
    # grows during the blackout. 10.0: after 228s, sigma_xy=48m (rejects 840m
    # outliers, accepts 193m drift). After 461s, sigma_xy=68m (accepts 274m drift).

    gnss.coast_timeout_s: 30.0
    # Also enter coast if GPS is silent this long (seconds). Handles outages
    # where the receiver stops publishing entirely. 0.0 = timeout trigger disabled.

    gnss.coast_q_bias_factor: 100.0
    # Q_gyro_bias multiplier in coast mode. Loosens bias confidence so encoder
    # WZ can drive fast heading bias correction during the blackout. 100.0 typical.

    gnss.coast_imu_wz_scale: 500.0
    # R_imu[WZ,WZ] multiplier in coast mode. Makes IMU heading rate less trusted
    # so encoder WZ dominates. 1.0 = disabled. 500.0 typical for long blackouts.

    gnss.recovery_rejection_n: 0
    # After this many consecutive rejections, inflate P[x,x] and P[y,y] directly.
    # Fires once per cascade. Must be > gnss.coast_n. 0 = disabled. Typical: 15.

    gnss.p_inflate_sigma: 50.0
    # XY sigma used for the P inflation above (meters). Only used when
    # gnss.recovery_rejection_n > 0.

    gnss.recovery_timeout_s: 0.0
    # GPS absence (seconds) before entering position-injection recovery mode, which
    # bypasses chi2 for the first returning fix. Useful only when blackouts are very
    # long (>200s) AND GPS outliers are not a concern at that location.
    # 0.0 = disabled (chi2 always active, recommended). Must be >= coast_timeout_s.

    # ── GPS track heading fusion ──────────────────────────────────────────────
    gnss.track_heading_enabled: true
    # Fuses GPS displacement bearing as a yaw pseudo-measurement whenever the
    # robot has moved gnss.track_heading_min_dist meters since the last fusion.
    # This is the primary mechanism for estimating encoder WZ bias without a
    # dual-antenna GPS. Disable only with an independent heading source.

    gnss.track_heading_min_dist: 5.0
    # Minimum GPS displacement (m) between heading fusions.

    gnss.track_heading_max_sigma: 0.4
    # Maximum heading uncertainty to allow a fusion (radians). Computed as
    # gps_noise / displacement. 0.4 rad = 23 degrees.

    gnss.track_heading_min_speed: 0.2
    # Minimum robot speed (m/s) for GPS displacement steps to count toward
    # heading observability. Below this: could be GPS jitter, not real motion.
    # Increase on high-vibration platforms (construction equipment, tracked robots).

    gnss.track_heading_max_yaw_rate: 0.3
    # Maximum yaw rate (rad/s) for displacement steps to count. During fast turns
    # the bearing changes too quickly for a reliable heading measurement.
    # Decrease for robots that make tight turns at slow speed.

    # ── Lever arm heading gating ──────────────────────────────────────────────
    gnss.lever_arm_max_heading_sigma_deg: 20.0
    # Lever arm correction is only applied when heading uncertainty is below this.
    # When heading degrades (e.g. during prolonged turns), rotating the lever arm
    # by an uncertain heading adds more position error than it removes.
    # Default 20 deg disables lever arm during tight-turn sections while leaving
    # it active during straight/gentle-curve driving where it genuinely helps.
    # Rule of thumb: lever_arm_m * sin(threshold_rad) should be < GPS noise sigma.

    # ── Outlier rejection ─────────────────────────────────────────────────────
    outlier_rejection: true
    outlier_threshold_gnss: 16.27   # chi2(3, 0.999): 3D GPS position
    outlier_threshold_imu:  15.09   # chi2(6, 0.999): 6D IMU (gyro + accel)
                                   # If imu.has_magnetometer: false, IMU only fuses roll/pitch (DOF=2).
                                   # Lower to 13.82 (chi2(2, 0.999)) to maintain 99.9% confidence.
    outlier_threshold_enc:  11.34   # chi2(3, 0.999): 3D encoder
    outlier_threshold_hdg:  10.83   # chi2(1, 0.999): 1D heading
    outlier_threshold_vslam: 22.46  # chi2(6, 0.999): 6D VSLAM pose
    # Keep vslam gate at chi2(6, 0.999). ORB-SLAM3 can jump on reinitialization:
    # this gate rejects those jumps automatically when covariance is calibrated.
    # Do NOT lower these below chi2 critical values. At 7.0 normal GPS noise
    # trips the gate and every fix gets rejected.

    gnss.max_speed: 0.0
    # Physical-plausibility gate on GPS position. Rejects any fix farther from the
    # filter's predicted position than the robot could have moved or drifted since
    # the last accepted fix: max_speed * gap_seconds + max_speed_margin. This is a
    # kinematic backstop that catches an impossible GPS jump the chi2 gate may
    # admit when its covariance has been inflated during coast recovery. Set to the
    # platform's maximum plausible speed in m/s (a few times cruise is safe); this
    # is a per-robot spec like wheel radius, not per-run tuning. 0.0 = disabled.
    gnss.max_speed_margin: 5.0
    # Slack (m) added to the max_speed * gap bound: covers GPS noise and the
    # uncertainty in the predicted position. 3-5 m is typical.

    # ── Adaptive noise ────────────────────────────────────────────────────────
    adaptive.imu: true
    adaptive.encoder: true
    adaptive.gnss: true
    adaptive.window: 50       # sliding window size (updates, not seconds)
    adaptive.alpha: 0.01      # EMA learning rate. 0.01 = slow, stable.

    # ── UKF process noise ─────────────────────────────────────────────────────
    # These are per-predict-step noise values (not spectral densities).
    # At 100Hz IMU, the filter predicts 100 times per second. Each step adds
    # Q to P, so effective noise rate = q_* * 100 per second.

    ukf.q_position: 0.01
    ukf.q_orientation: 1.0e-9   # quaternion regularization ONLY: do not increase.
                                 # Orientation uncertainty propagates from q_angular_vel
                                 # through the kinematics. Large values here corrupt
                                 # quaternion norm and cause yaw/Z drift.
    ukf.q_velocity: 0.1
    ukf.q_angular_vel: 0.1
    ukf.q_acceleration: 1.0
    ukf.q_gyro_bias: 1.0e-5     # biases change slowly (MEMS thermal drift)
    ukf.q_accel_bias: 1.0e-5
    ukf.q_encoder_wz_bias: 1.0e-7  # encoder WZ bias is mechanical: very stable.
                                    # Smaller than gyro bias because it changes only
                                    # with physical wear, not thermal effects.

    # ── Startup ───────────────────────────────────────────────────────────────
    init.stationary_window: 0.0
    # Seconds of IMU data to collect before starting (robot must be stationary).
    # Estimates accelerometer bias at startup → reduces 60s warmup transient
    # from ~10cm to under 1cm. Set 2.0 if startup drift is a problem.
    # Falls back to zero-bias automatically if robot moves during window.

    init.wait_for_all_sensors: false
    # When true: hold filter initialization until every configured sensor has
    # published at least one message. Prevents the filter from drifting on IMU
    # alone while GPS and wheel odometry are still coming online at startup.
    # Replaces the sleep() workaround in launch files.
    init.sensor_wait_timeout: 10.0
    # Seconds to wait before starting anyway if a sensor never arrives.
    # A WARN lists which sensors were missing. Set 0.0 to disable the timeout.

    # ── Motion model ──────────────────────────────────────────────────────────
    motion_model: "ConstantVelocityAcceleration"
    # Controls how sigma points are propagated in the UKF predict step.
    #
    # "ConstantVelocityAcceleration" (default): no platform constraints.
    #   VY and AY grow freely between measurements. Correct for aerial vehicles.
    #   Good baseline for any platform.
    #
    # "DifferentialDrive": zeros VY and AY each predict step.
    #   The filter knows a diff-drive robot cannot slide sideways.
    #   Tighter lateral covariance, less position smear on straight runs.
    #   Use for: differential drive, skid-steer, tracked vehicles.
    #
    # "Ackermann": same lateral constraint as DifferentialDrive.
    #   wheelbase stored for future minimum-turning-radius extensions.
    #   Use for: car-like robots, forklifts, front-steered outdoor platforms.
    motion_model_params.wheelbase: 0.55    # metres (only used by Ackermann)

    # ── Deterministic replay ──────────────────────────────────────────────────
    replay.checkpoint_path: "/tmp/fusioncore_checkpoint.txt"
    # File used by ~/save_checkpoint and ~/load_checkpoint services.
    # save_checkpoint: writes the full 23-state + 23x23 covariance to this file.
    # load_checkpoint: restores that state (restarts filter from that point).
    # Workflow: replay a bag to a known-good point → save → tweak params →
    #   load (instant, no re-replay) → observe the problem window.

```

---

## GNSS Doppler velocity bridge (ublox F9P / M8U)

FusionCore itself has no dependency on any specific GPS driver. It accepts velocity from any receiver via `gnss.velocity_topic`, which expects `nav_msgs/Odometry` with ENU velocity (`linear.x=east`, `linear.y=north`).

If your receiver is a u-blox module (F9P, M8U, NEO-M9N, etc.), the `fusioncore_ublox` companion package provides a ready-made bridge. It is a separate package with its own dependency on `ublox_msgs` so the FusionCore core remains clean.

```bash
# Build the companion package alongside FusionCore
colcon build --packages-select fusioncore_ros fusioncore_ublox
```

**Launch the bridge alongside FusionCore:**

```bash
# Terminal 1: FusionCore
ros2 launch fusioncore_ros fusioncore.launch.py fusioncore_config:=your_robot.yaml

# Terminal 2: ublox bridge
ros2 launch fusioncore_ublox gnss_doppler_bridge.launch.py \
  navpvt_topic:=/ublox/navpvt \
  output_topic:=/gnss/doppler_vel
```

**Matching FusionCore config:**

```yaml
gnss.velocity_topic: "/gnss/doppler_vel"
```

**What the bridge does:**

| NavPVT field | Unit | ENU output |
|---|---|---|
| `vel_e` (east) | mm/s | `twist.linear.x` (m/s) |
| `vel_n` (north) | mm/s | `twist.linear.y` (m/s) |
| `vel_d` (down) | mm/s | `twist.linear.z` = -vel_d/1000 (m/s) |
| `s_acc` | mm/s | `covariance[0,7]` = (s_acc/1000)^2 |

Fixes with `fix_type < 3` (no 3D lock) or `gnssFixOK` flag unset are silently dropped. Speeds below 0.05 m/s are also dropped to avoid heading corruption at standstill.

**Other receivers:** publish `nav_msgs/Odometry` with ENU velocity on any topic and point `gnss.velocity_topic` at it. FusionCore doesn't care which driver produced it.

---

## Choosing a motion model

Start with the default (`ConstantVelocityAcceleration`) unless you have a specific reason to change it. It works well for all platforms and matches what robot_localization users are used to.

Switch to `DifferentialDrive` if:
- Your robot is a differential drive, skid-steer, or tracked vehicle
- You see small lateral position drift on straight runs
- Your `VY` state doesn't stay near zero between encoder updates

Switch to `Ackermann` if:
- Your robot has front-wheel steering (car-like, forklift, outdoor field robot)
- The lateral constraint is the same as `DifferentialDrive`; `wheelbase` is stored for future minimum-turning-radius enforcement

Do not use `DifferentialDrive` or `Ackermann` for:
- Aerial vehicles (no lateral constraint applies)
- Holonomic (mecanum) platforms (those can move sideways intentionally)

---

## Wait for all sensors: replacing sleep() in launch files

A common pattern in ROS launch files is `sleep(3)` before starting the navigation stack to give sensors time to come online. This is fragile: on a slow machine the sensors might need 5 seconds, and on a fast one you waste 3 seconds on every launch.

`init.wait_for_all_sensors: true` replaces this entirely. FusionCore holds initialization until it has seen at least one message from every sensor you configured. Then it starts. No sleep needed.

```yaml
init.wait_for_all_sensors: true
init.sensor_wait_timeout: 10.0
```

The timeout is a safety net: if a sensor fails to start, the filter does not hang forever. It logs which sensors were missing and starts anyway:

```
[WARN] Sensor wait timed out after 10.0s. Missing: [GNSS]. Starting anyway.
```

This is especially useful at competition startup, on-site robot power-on, or any deployment where sensor initialization order is not guaranteed.

---

## Deterministic replay: debugging without hardware

See [How It Works: Deterministic replay](how-it-works.md#deterministic-replay-and-state-checkpoints) for the full workflow. Quick reference:

```bash
# Save filter state at any point during bag replay
ros2 service call /fusioncore/save_checkpoint std_srvs/srv/Trigger

# Restore that state instantly (no need to replay from the beginning)
ros2 service call /fusioncore/load_checkpoint std_srvs/srv/Trigger
```

The checkpoint file path is set by `replay.checkpoint_path` (default `/tmp/fusioncore_checkpoint.txt`).

---

## GPS without a fix (indoor / no GPS)

Set `reference.use_first_fix: false` and leave reference coords at 0.0. The filter starts at the origin and runs on IMU + wheel odometry alone. No GPS topics needed.

---

## Agricultural RTK (UTM output)

Some RTK receivers output easting/northing directly:

```yaml
input.gnss_crs: "EPSG:32617"        # UTM zone 17N (adjust for your zone)
output.crs: "EPSG:32617"
output.convert_to_enu_at_reference: false
reference.use_first_fix: true
```

---

## GPS receiver setup: NavSatFix vs GPSFix

FusionCore supports two GPS message types on `gnss.fix_topic` (default `/gnss/fix`). The default is `sensor_msgs/NavSatFix` because every ROS GPS driver publishes it. Set `gnss.use_gps_fix: true` to switch to `gps_msgs/GPSFix` if your driver supports it.

| | `sensor_msgs/NavSatFix` | `gps_msgs/GPSFix` |
|---|---|---|
| Driver support | Universal | nmea_navsat_driver, ublox_dgnss, others |
| RTK_FLOAT status | Not expressible | Yes (status 20) |
| Separate HDOP / VDOP | No | Yes |
| Satellites used | No | Yes |
| 95% CI error bounds | No | err_horz / err_vert |
| Covariance matrix | Yes | Yes |

### When to use NavSatFix (default)

NavSatFix works with all receivers. For most setups, leave `gnss.use_gps_fix: false`.

The only thing you cannot get via NavSatFix is RTK_FLOAT. If you are using autonomous GPS (CEP 1-3m) or RTK fixed, NavSatFix is all you need.

### When to use GPSFix

Switch to `gnss.use_gps_fix: true` when:

- Your receiver can output RTK_FLOAT and you want to fuse those fixes (better than autonomous, worse than RTK fixed). Set `gnss.min_fix_type: 3` to require it or allow it.
- Your driver publishes receiver-native HDOP/VDOP rather than a covariance matrix, and you want those values used directly in the noise model.
- Your driver sets `err_horz`/`err_vert` (95% CI bounds) and you prefer that over a synthetic covariance.

```yaml
fusioncore:
  ros__parameters:
    gnss.use_gps_fix: true
    gnss.min_fix_type: 3      # require RTK_FLOAT or better (3=FLOAT, 4=FIXED)
    gnss.base_noise_xy: 0.5   # metres: baseline at HDOP=1 for RTK_FLOAT
    gnss.base_noise_z: 1.0
```

### Covariance priority (GPSFix)

When `gnss.use_gps_fix: true`, FusionCore picks the best available covariance source in this order:

1. `position_covariance_type == 3` (full 3x3): used directly, including off-diagonal terms.
2. `position_covariance_type >= 1` (diagonal): diagonal elements used, hdop/vdop derived from them.
3. `err_horz > 0` and `err_vert > 0`: 95% CI bounds converted to 1-sigma variance (divide by 1.96), used as a diagonal covariance.
4. `hdop > 0` and `vdop > 0`: receiver-native DOP values used directly in the noise model (`sigma_xy = base_noise_xy * hdop`).
5. Defaults (`hdop=1.5, vdop=2.0`).

---

!!! warning "Upgrading from an older config"
    If your YAML has `ukf.q_orientation: 0.01`, change it to `1.0e-9` or delete the line. The old value corrupts quaternion math at typical IMU rates and causes yaw drift and Z-axis rise in simulation.
