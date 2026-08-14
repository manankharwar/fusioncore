# Microstrain 3DM-GX3-45 + Novatel SPAN-CPT GPS

**Platform:** Segway RMP campus rover (University of Michigan NCLT dataset)
**Status: Field validated** — 12 sequences, 940 minutes total, evaluated against RTK GPS ground truth. [Full benchmark results](../reference/benchmark.md).

---

## Sensors

| Sensor | Model | Notes |
|---|---|---|
| IMU | Microstrain 3DM-GX3-45 | 6-axis (accel + gyro). No magnetometer. 100 Hz. |
| GPS | Novatel SPAN-CPT | ~3m CEP in urban campus conditions. 5 Hz. |
| Wheel odometry | Segway RMP encoders | Differentiated from odometry at 100 Hz. |

**IMU datasheet specs used:**
- Angular random walk: 0.07°/√hr → noise density ≈ 0.003 rad/s at 100 Hz
- Velocity random walk: 0.03 m/s/√hr → noise density ≈ 0.001 m/s² at 100 Hz

---

## Config

This is the exact configuration used for the [NCLT benchmark](../reference/benchmark.md). Identical across all 12 sequences — no per-sequence tuning.

```yaml
fusioncore:
  ros__parameters:
    base_frame: base_link
    odom_frame: odom
    publish_rate: 100.0

    # Microstrain 3DM-GX3-45: 6-axis, no magnetometer.
    # Yaw initializes from GPS motion after ~5m of travel.
    imu.has_magnetometer: false
    imu.gyro_noise: 0.003           # rad/s : ARW 0.07 deg/sqrt(hr)
    imu.accel_noise: 0.1            # m/s^2 : conservative for campus vibration
    imu.remove_gravitational_acceleration: false  # driver publishes raw specific force

    # Segway RMP wheel encoders, differentiated from odometry_mu_100hz.csv.
    # Loosen vel_noise if your surface is rough or wheels are pneumatic.
    encoder.vel_noise: 0.05         # m/s
    encoder.yaw_noise: 0.02         # rad/s

    # Novatel SPAN-CPT: stated CEP 3m, but NCLT measured p95 errors range
    # from 9.7m (clean day) to 53.1m (bad multipath day).
    # Adaptive noise compensates automatically: gnss.base_noise_xy is the
    # floor, not the ceiling.
    gnss.base_noise_xy: 3.0         # m  : matches stated CEP
    gnss.base_noise_z: 5.0          # m

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
    gnss.max_hdop: 10.0             # fallback only: lenient, NCLT doesn't publish HDOP
    gnss.min_satellites: 0          # lenient: NCLT doesn't publish sat count
    gnss.min_fix_type: 1
    gnss.heading_topic: ""          # single-antenna: no heading from GPS
    gnss.azimuth_topic: ""

    gnss.lever_arm_x: 0.0
    gnss.lever_arm_y: 0.0
    gnss.lever_arm_z: 0.3           # GPS antenna ~30cm above base_link

    outlier_rejection: true
    outlier_threshold_gnss: 16.27   # chi2(3, 0.999)
    outlier_threshold_enc: 11.34    # chi2(3, 0.999)
    outlier_threshold_imu: 15.09    # chi2(6, 0.999)

    gnss.coast_n: 3                 # enter coast mode after 3 consecutive rejections
    gnss.coast_q_factor: 10.0       # inflate Q_position 10x during coast
    gnss.coast_timeout_s: 30.0      # also coast if GPS silent for 30s

    adaptive.imu: true
    adaptive.encoder: true
    adaptive.gnss: true             # key: adapts to actual GPS noise, not just stated spec
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

The NCLT benchmark uses `fusioncore_datasets/nclt_player.py` which publishes to default topics. For a real Segway RMP or Microstrain setup, remap as needed:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/this-config.yaml \
  --ros-args \
  -r /imu/data:=/microstrain/imu/data \
  -r /odom/wheels:=/segway_rmp/odom \
  -r /gnss/fix:=/novatel/fix
```

---

## Validation results

Evaluated against RTK GPS ground truth (Novatel SPAN-CPT RTK mode) using `evo_ape` with SE(3) alignment.

| Sequence | ATE 3D | vs RL-EKF |
|---|---|---|
| 2012-01-08 | 18.6 m | +55% better |
| 2012-05-11 | 9.7 m | +16% better |
| 2012-09-28 | 22.4 m | +58% better |
| 2013-04-05 | 12.1 m | +96% better |
| 2012-08-20 | 98.3 m | RL wins (adversarial GPS cluster at blackout boundary) |

> **Note:** the 2013-04-05 figure (12.1 m) is stale, it has since regressed to ~19.4 m (still a 93% win). These numbers predate a controlled full-suite re-run on current `main`. See `tools/benchmark_regression.md`.

Full results across all 12 sequences: [Benchmark Results](../reference/benchmark.md).

Reproduce with:
```bash
bash benchmarks/run_one.sh 2012-01-08
```

---

## Adapting this config to similar hardware

**Different GPS (standard consumer, 3-5m CEP):** Keep `gnss.base_noise_xy: 3.0` and `adaptive.gnss: true`. The adaptive window adjusts upward automatically.

**Better GPS (F9P RTK float, ~0.4m CEP):** Set `gnss.base_noise_xy: 0.5`, `gnss.min_fix_type: 3`.

**Different IMU with similar specs (VectorNav VN-100, Lord 3DM-GX5-25):** Change `imu.gyro_noise` to match the ARW from your datasheet: `σ ≈ ARW_deg_per_sqrthr × (π/180) / 60 × √sample_rate`.

**Mecanum or holonomic platform:** Remove `motion_model: "DifferentialDrive"` (or set to `"CVA"`) since the robot can slide laterally.
