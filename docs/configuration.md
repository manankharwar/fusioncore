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

    imu.frame_id: ""  # override IMU TF frame. Leave empty to use msg header.frame_id.
                      # Set when your simulator or IMU driver stamps messages with a
                      # non-standard frame name (e.g. Gazebo Harmonic TurtleBot3 publishes
                      # "waffle/imu_link/tb3_imu"). Set to your URDF frame name instead.

    # ── Wheel encoders ────────────────────────────────────────────────────────
    encoder.vel_noise: 0.05     # m/s
    encoder.yaw_noise: 0.02     # rad/s

    # Optional second velocity source (lidar odometry, visual odometry, etc.)
    # Must publish nav_msgs/Odometry with velocity in twist field.
    encoder2.topic: ""          # e.g. "/kiss/odometry" or "/icp_odom"

    # ── GPS ───────────────────────────────────────────────────────────────────
    gnss.base_noise_xy: 1.0     # m: baseline sigma at HDOP=1
                                # scaled automatically by HDOP from the message
                                # standard autonomous GPS: 1.0–2.5
                                # RTK float: 0.5,  RTK fixed: 0.015
    gnss.base_noise_z: 2.0      # m
    gnss.heading_noise: 0.02    # rad: for dual antenna heading

    gnss.max_hdop: 4.0          # reject fixes with HDOP worse than this
    gnss.min_satellites: 4
    gnss.min_fix_type: 1        # 1=GPS, 2=DGPS, 3=RTK_FLOAT, 4=RTK_FIXED
                                # NOTE: sensor_msgs/NavSatFix status=2 → RTK_FIXED only.
                                # RTK_FLOAT (3) is unreachable via NavSatFix.
                                # Use 2 or 4 as meaningful thresholds.

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
    # Works indoors and in all weather -- rain, fog, dust, darkness.
    # radar.vel_noise is used when the message covariance is zero or negative.
    radar.velocity_topic: ""
    radar.vel_noise: 0.1        # m/s fallback noise when message has no covariance

    # Heading input: pick one or both
    gnss.heading_topic: ""      # sensor_msgs/Imu (dual antenna heading)
    gnss.azimuth_topic: ""      # compass_msgs/Azimuth (preferred REP-145 standard)

    # ── Outlier rejection ─────────────────────────────────────────────────────
    outlier_rejection: true
    outlier_threshold_gnss: 16.27   # chi2(3, 0.999): 3D GPS position
    outlier_threshold_imu:  15.09   # chi2(6, 0.999): 6D IMU (gyro + accel)
                                   # If imu.has_magnetometer: false, IMU only fuses roll/pitch (DOF=2).
                                   # Lower to 13.82 (chi2(2, 0.999)) to maintain 99.9% confidence.
    outlier_threshold_enc:  11.34   # chi2(3, 0.999): 3D encoder
    outlier_threshold_hdg:  10.83   # chi2(1, 0.999): 1D heading
    # Do NOT lower these below chi2 critical values. At 7.0 normal GPS noise
    # trips the gate and every fix gets rejected.

    # ── Adaptive noise ────────────────────────────────────────────────────────
    adaptive.imu: true
    adaptive.encoder: true
    adaptive.gnss: true
    adaptive.window: 50       # sliding window size (updates, not seconds)
    adaptive.alpha: 0.01      # EMA learning rate. 0.01 = slow, stable.

    # ── UKF process noise ─────────────────────────────────────────────────────
    ukf.q_position: 0.01
    ukf.q_orientation: 1.0e-9   # quaternion regularization ONLY: do not increase
    ukf.q_velocity: 0.1
    ukf.q_angular_vel: 0.1
    ukf.q_acceleration: 1.0
    ukf.q_gyro_bias: 1.0e-5
    ukf.q_accel_bias: 1.0e-5

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
    motion_model_params:
      wheelbase: 0.55    # meters (only used by Ackermann)

    # ── Deterministic replay ──────────────────────────────────────────────────
    replay.checkpoint_path: "/tmp/fusioncore_checkpoint.txt"
    # File used by ~/save_checkpoint and ~/load_checkpoint services.
    # save_checkpoint: writes the full 22-state + 22x22 covariance to this file.
    # load_checkpoint: restores that state (restarts filter from that point).
    # Workflow: replay a bag to a known-good point → save → tweak params →
    #   load (instant, no re-replay) → observe the problem window.

    # ── ZUPT ──────────────────────────────────────────────────────────────────
    zupt.enabled: true
    zupt.velocity_threshold: 0.05   # m/s: encoder speed below this → stationary
    zupt.angular_threshold: 0.05    # rad/s: angular rate below this → not rotating
    zupt.noise_sigma: 0.01          # m/s: tighter = stronger zero-velocity correction

    # ── GPS coordinate system ─────────────────────────────────────────────────
    input.gnss_crs: "EPSG:4326"              # WGS84 lat/lon (standard GPS)
    output.crs: "EPSG:4978"                  # ECEF XYZ (globally valid default)
    output.convert_to_enu_at_reference: true # required when output.crs is ECEF
    reference.use_first_fix: true            # map origin = first GPS fix
    reference.x: 0.0                         # fixed origin (when use_first_fix: false)
    reference.y: 0.0
    reference.z: 0.0
```

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

A common pattern in ROS launch files is `sleep(3)` before starting the navigation stack to give sensors time to come online. This is fragile -- on a slow machine the sensors might need 5 seconds, and on a fast one you waste 3 seconds on every launch.

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

See [How It Works -- Deterministic replay](how-it-works.md#deterministic-replay-and-state-checkpoints) for the full workflow. Quick reference:

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

!!! warning "Upgrading from an older config"
    If your YAML has `ukf.q_orientation: 0.01`, change it to `1.0e-9` or delete the line. The old value corrupts quaternion math at typical IMU rates and causes yaw drift and Z-axis rise in simulation.
