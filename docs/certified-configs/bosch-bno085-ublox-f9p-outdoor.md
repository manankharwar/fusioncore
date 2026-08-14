# Bosch BNO085 + u-blox ZED-F9P (outdoor, differential drive)

**Platform:** Differential drive outdoor robots
**Status: Community contributed** — running on Oakland University Robotics Association "Erdferkel" (IGVC competition, Michigan) and Agroecology Lab agricultural robot (UK). Field validation results pending.

---

## Sensors

| Sensor | Model | Notes |
|---|---|---|
| IMU | Bosch BNO085 | 9-axis (accel + gyro + magnetometer). Typically 100 Hz. |
| GPS | u-blox ZED-F9P | Standard GPS (~2m CEP) or RTK float/fixed. 5-10 Hz. |
| Wheel odometry | Platform-specific | nav_msgs/Odometry via differential drive controller. |

**IMU datasheet specs used:**
- Gyro noise density: 0.014 dps/√Hz → σ ≈ 0.005 rad/s at 100 Hz
- Accel noise density: 120 μg/√Hz → σ ≈ 0.1 m/s² at 100 Hz (conservative for vibration)

**Note on the BNO085 magnetometer:** The onboard magnetometer can provide immediate yaw initialization without needing GPS motion first, but magnetic interference from motors and wiring makes it unreliable on most robot platforms. Leave `imu.has_magnetometer: false` unless you've calibrated and verified your magnetic environment is clean.

---

## Config

```yaml
fusioncore:
  ros__parameters:
    base_frame: base_link
    odom_frame: odom
    publish_rate: 100.0
    publish.force_2d: true

    motion_model: "DifferentialDrive"

    # Bosch BNO085: 9-axis consumer IMU.
    # Most platforms benefit from leaving the magnetometer off due to motor interference.
    imu.has_magnetometer: false
    imu.gyro_noise: 0.005           # rad/s : 0.014 dps/sqrt(Hz) at 100 Hz
    imu.accel_noise: 0.1            # m/s^2 : conservative for wheeled robot vibration
    imu.remove_gravitational_acceleration: false  # most BNO085 drivers output raw
    imu.frame_id: "imu_link"

    # Generic outdoor differential drive encoders.
    # Tighten vel_noise if your encoders are high resolution (>= 500 CPR).
    encoder.vel_noise: 0.05         # m/s
    encoder.yaw_noise: 0.02         # rad/s

    # u-blox ZED-F9P: CEP depends on fix type.
    # Standard GPS (no corrections): CEP ~2.0m -> base_noise_xy: 2.0
    # RTK float (NTRIP corrections):  CEP ~0.4m -> base_noise_xy: 0.5, min_fix_type: 3
    # RTK fixed:                       CEP ~0.01m -> base_noise_xy: 0.015, min_fix_type: 4
    # Start with standard GPS values. Tighten when corrections are stable.
    gnss.base_noise_xy: 2.0         # m : F9P autonomous CEP
    gnss.base_noise_z: 4.0          # m

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
    gnss.max_hdop: 3.5              # fallback only: F9P publishes covariance, so this is unused
    gnss.min_satellites: 4
    gnss.min_fix_type: 1            # 1=GPS, 3=RTK_FLOAT, 4=RTK_FIXED

    # Measure from base_link to GPS antenna phase center: x=forward, y=left, z=up.
    gnss.lever_arm_x: 0.0
    gnss.lever_arm_y: 0.0
    gnss.lever_arm_z: 0.0

    gnss.fix2_topic: ""
    gnss.heading_topic: ""          # set if using dual-antenna F9P for heading
    gnss.azimuth_topic: ""

    # Raw magnetometer heading: fuses sensor_msgs/MagneticField directly.
    # The BNO085 publishes this on /imu/mag at 100 Hz via the Hillcrest driver.
    # Requires hard/soft iron calibration (use imu_calib or magneto).
    # Useful for GPS-denied phases where track heading is unavailable.
    magnetometer.enabled: false
    magnetometer.topic: "/imu/mag"
    magnetometer.noise_rad: 0.05
    magnetometer.chi2_threshold: 9.21
    magnetometer.declination_rad: 0.0    # look up at magnetic-declination.com
    magnetometer.hard_iron: [0.0, 0.0, 0.0]
    magnetometer.soft_iron: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    outlier_rejection: true
    outlier_threshold_gnss: 16.27   # chi2(3, 0.999)
    outlier_threshold_enc: 11.34    # chi2(3, 0.999)
    outlier_threshold_imu: 15.09    # chi2(6, 0.999)

    gnss.coast_n: 3
    gnss.coast_q_factor: 10.0
    gnss.coast_timeout_s: 30.0

    adaptive.imu: true
    adaptive.encoder: true
    adaptive.gnss: true
    adaptive.window: 50
    adaptive.alpha: 0.01

    ukf.q_position: 0.01
    ukf.q_orientation: 1.0e-9
    ukf.q_velocity: 0.1
    ukf.q_angular_vel: 0.1
    ukf.q_acceleration: 1.0
    ukf.q_gyro_bias: 1.0e-5
    ukf.q_accel_bias: 1.0e-5
    ukf.q_encoder_wz_bias: 1.0e-7

    input.gnss_crs: "EPSG:4326"
    output.crs: "EPSG:4978"
    output.convert_to_enu_at_reference: true
    reference.use_first_fix: true
```

---

## Topic remaps

Topic names vary by driver. Common setups:

```bash
# Adafruit BNO085 breakout + ublox_ros2_driver
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/this-config.yaml \
  --ros-args \
  -r /imu/data:=/bno085/imu \
  -r /gnss/fix:=/fix \
  -r /odom/wheels:=/diff_controller/odom
```

**Common BNO085 drivers and their default topics:**
- `bno085_ros2` package: publishes at `/bno085/imu` → remap `-r /imu/data:=/bno085/imu`
- Adafruit CircuitPython bridge: check your specific node's topic

**Common F9P drivers:**
- `ublox_ros2_driver`: publishes NavSatFix at `/fix` → remap `-r /gnss/fix:=/fix`
- `ublox_gps_node`: check your configuration

**Wheel odometry (depends on your motor controller):**
- ROS 2 Control `diff_drive_controller`: `/diff_controller/odom`
- Nav2 default: `/odom`
- Custom: whatever your node publishes

---

## RTK upgrade

When NTRIP corrections are available and the F9P reaches RTK float or fixed:

```yaml
# RTK float (NTRIP corrections, carrier phase partial):
gnss.base_noise_xy: 0.5
gnss.min_fix_type: 3

# RTK fixed (centimeter-level, all carrier phases resolved):
gnss.base_noise_xy: 0.015
gnss.min_fix_type: 4
```

The F9P publishes fix quality in `NavSatFix.status.status`. FusionCore's `min_fix_type` maps to that field: 1=GPS, 2=DGPS/SBAS, 3=RTK_FLOAT, 4=RTK_FIXED.

---

## Deployers

- **Zachary Lain** ([@ZacharyLain](https://github.com/ZacharyLain)), Oakland University Robotics Association: "Erdferkel" differential-drive robot, u-blox F9P + BNO085, IGVC competition Michigan.
- **Sam** ([@samuk](https://github.com/samuk)), Agroecology Lab: outdoor agricultural tracked robot, dual u-blox F9P RTK + BNO085, ROS 2 Humble, UK.

Contributing a field validation result? Open a [pull request](https://github.com/manankharwar/fusioncore/pulls) or [discussion](https://github.com/manankharwar/fusioncore/discussions) with your rosbag or trajectory plot.
