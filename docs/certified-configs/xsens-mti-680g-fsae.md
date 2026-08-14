# Xsens MTi-680G IMU + GNSS (Formula Student driverless race car)

**Platform:** Formula Student / FSAE driverless race car (Ackermann kinematics)
**Status: Validation in progress** — contributed by [UniNa Corse](https://www.instagram.com/uninacorse/), Università degli Studi di Napoli Federico II, 2nd place FSAE Italy 2025 driverless category. Field validation results will be published after competition season.

---

## Sensors

| Sensor | Model | Notes |
|---|---|---|
| IMU + GNSS | Xsens MTi-680G | 9-axis IMU (accel + gyro + mag) with integrated GNSS (RTK capable). Single unit. |
| Wheel / vehicle model | Custom RK4 bicycle model | Provides velocity odometry via encoder. Not a standard nav_msgs/Odometry topic. |
| VSLAM | KISS-ICP | Provides pose odometry in GPS-denied areas (e.g., tunnel sections). |

**IMU datasheet specs:**
- Gyro ARW: 0.01°/√hr → σ ≈ 0.0005 rad/s at 100 Hz
- Accel VRW: 0.003 m/s/√hr → σ ≈ 0.003 m/s² at 100 Hz
- This is a high-end tactical-grade MEMS IMU. Noise values are 6-14x lower than a BNO085.

**GNSS specs:**
- Integrated u-blox GNSS receiver with RTK support
- RTK fixed: CEP ~1cm
- Standard GPS: CEP ~1.5m

---

## Config

This configuration targets a high-vibration Ackermann vehicle with a high-quality IMU and RTK GPS. If you are running standard GPS (no RTK corrections), change `gnss.base_noise_xy` and `gnss.min_fix_type` as noted in the comments.

```yaml
fusioncore:
  ros__parameters:
    base_frame: base_link
    odom_frame: odom
    publish_rate: 100.0
    publish.force_2d: false         # race car: 3D mode, surface is not perfectly flat

    motion_model: "Ackermann"       # front-wheel steering: cannot turn in place

    # Xsens MTi-680G: high-end tactical MEMS IMU.
    # Has a magnetometer but race car environment has strong magnetic interference
    # from motors and motor controllers. Leave false unless field-calibrated.
    imu.has_magnetometer: false
    imu.gyro_noise: 0.0005          # rad/s : ARW 0.01 deg/sqrt(hr) at 100 Hz
    imu.accel_noise: 0.003          # m/s^2 : VRW 0.003 m/s/sqrt(hr) at 100 Hz
    imu.remove_gravitational_acceleration: false  # Xsens driver publishes raw specific force
    imu.frame_id: "imu_link"

    # Wheel / bicycle model odometry.
    # The MTi-680G also outputs velocity from its integrated GNSS.
    # Tune vel_noise based on your encoder resolution and wheel slip characteristics.
    # Race cars have minimal wheel slip on dry track; loosen for wet conditions.
    encoder.vel_noise: 0.05         # m/s : conservative starting value
    encoder.yaw_noise: 0.03         # rad/s : slightly looser for high-speed cornering

    # Xsens MTi-680G integrated GNSS (RTK mode for competition).
    # Competition circuits typically have good sky view: use RTK fixed when available.
    # Near buildings or in tunnel sections: outlier gate handles GPS degradation.
    #
    # RTK fixed (open track sections):
    gnss.base_noise_xy: 0.015       # m : RTK fixed CEP ~1cm, conservative floor
    gnss.min_fix_type: 4            # 4=RTK_FIXED
    #
    # To fall back to RTK float if fixed is unavailable:
    # gnss.base_noise_xy: 0.5
    # gnss.min_fix_type: 3
    #
    # Standard GPS (no NTRIP corrections available):
    # gnss.base_noise_xy: 1.5
    # gnss.min_fix_type: 1

    gnss.base_noise_z: 0.1          # m : RTK Z accuracy is better than standard GPS

    # Which of the two quality gates below actually runs depends on your driver.
    # If the NavSatFix carries a non-zero position_covariance (most drivers do),
    # FusionCore gates on the receiver's own reported sigma in METRES and the
    # max_hdop/max_vdop lines are never consulted. They are the fallback for
    # drivers that publish no covariance at all. Before 0.3.6 the DOP numbers
    # were compared against metres, which silently rejected every fix from a
    # receiver reporting honest sigma; see docs/configuration.md.
    #
    # Leave the sigma gate at its defaults (25 m / 50 m) unless you have logged
    # what your receiver actually reports. It is deliberately permissive: a gate
    # tuned to clean-sky numbers throws away the degraded fixes you most need.
    # gnss.max_sigma_xy: 25.0         # m : reject a fix reporting worse than this
    # gnss.max_sigma_z:  50.0         # m
    gnss.max_hdop: 3.0              # fallback only: the MTi-680G publishes covariance
    gnss.min_satellites: 4

    # MTi-680G is a single-antenna unit. No dual-antenna heading.
    # Heading initializes from GNSS motion after ~5m of travel.
    gnss.heading_topic: ""
    gnss.azimuth_topic: ""

    # Lever arm: distance from base_link to GNSS antenna phase center.
    # On a race car this is significant and must be measured accurately.
    # x=forward, y=left, z=up (meters).
    gnss.lever_arm_x: 0.0
    gnss.lever_arm_y: 0.0
    gnss.lever_arm_z: 0.0

    # Outlier rejection: tighter threshold appropriate for RTK accuracy.
    # A 5-sigma GPS outlier with RTK fixed (CEP 1cm) is still a very small deviation.
    # Standard chi2(3, 0.999) gate handles this correctly.
    outlier_rejection: true
    outlier_threshold_gnss: 16.27   # chi2(3, 0.999)
    outlier_threshold_enc: 11.34    # chi2(3, 0.999)
    outlier_threshold_imu: 15.09    # chi2(6, 0.999)

    # Coast mode: for GPS-denied sections (tunnel, underground pit area).
    gnss.coast_n: 3
    gnss.coast_q_factor: 10.0
    gnss.coast_timeout_s: 10.0      # tighter than default: race cars move fast

    # KISS-ICP VSLAM: provides absolute pose during GPS-denied sections.
    # Uncomment and set to your actual KISS-ICP output topic.
    # vslam.topic: "/kiss_icp/odometry"
    # vslam.pose_noise: 0.05

    adaptive.imu: true
    adaptive.encoder: true
    adaptive.gnss: true
    adaptive.window: 50
    adaptive.alpha: 0.01

    # High-quality IMU: tighter orientation process noise than generic configs.
    ukf.q_position: 0.01
    ukf.q_orientation: 1.0e-10     # tighter: MTi-680G orientation is very stable
    ukf.q_velocity: 0.1
    ukf.q_angular_vel: 0.05        # tighter: low gyro noise
    ukf.q_acceleration: 1.0
    ukf.q_gyro_bias: 1.0e-6        # tighter: tactical MEMS bias stability
    ukf.q_accel_bias: 1.0e-6
    ukf.q_encoder_wz_bias: 1.0e-7

    input.gnss_crs: "EPSG:4326"
    output.crs: "EPSG:4978"
    output.convert_to_enu_at_reference: true
    reference.use_first_fix: true
```

---

## Topic remaps

The Xsens MTi-680G uses the official `xsens_ros_mti_driver` package. Default topics:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/this-config.yaml \
  --ros-args \
  -r /imu/data:=/imu/data \          # xsens driver default: already /imu/data
  -r /gnss/fix:=/gnss \              # xsens driver publishes NavSatFix at /gnss
  -r /odom/wheels:=/vehicle/odom      # your bicycle model odometry topic
```

Check your `xsens_ros_mti_driver` configuration for the exact topic names. The driver can output `sensor_msgs/Imu` and `sensor_msgs/NavSatFix` simultaneously from the same unit.

---

## Adapting this config for standard GPS

If you do not have RTK corrections (NTRIP base station or corrections from the MTi-680G's GNSS), use standard GPS values:

```yaml
gnss.base_noise_xy: 1.5        # m : MTi-680G autonomous GPS CEP
gnss.min_fix_type: 1
gnss.base_noise_z: 3.0
```

---

## Deployer

**Pasquale Cannavacciuolo** ([@pakyCannavacciuolo05](https://github.com/pakyCannavacciuolo05)), UniNa Corse — Università degli Studi di Napoli Federico II. FSAE Italy 2025 driverless category, 2nd place overall.

Running FusionCore with this config on your Xsens platform? Open a [pull request](https://github.com/manankharwar/fusioncore/pulls) to update the status and add your validation results.
