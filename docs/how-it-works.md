# How It Works

The technical detail behind FusionCore's design decisions.

---

## IMU frame transform

IMUs are almost never mounted at `base_link`. FusionCore reads `frame_id` from every IMU message, looks up the TF rotation to `base_link`, and rotates angular velocity and linear acceleration before fusing. No manual YAML rotation required.

If the transform is missing:

```
[WARN] Cannot transform IMU from imu_link to base_link.
Fix: ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id imu_link
```

---

## TF validation at startup

During `configure`, FusionCore checks that all required TF transforms exist before the filter starts. Missing transforms print the exact fix command:

```
--- TF Validation ---
  [OK]      imu_link -> base_link
  [MISSING] base_link -> odom
  Fix: ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link
---------------------
```

No silent failures. No mysterious drift from a missing transform you didn't notice.

---

## State vector

FusionCore maintains a 22-dimensional state vector:

| Index | State | Units |
|-------|-------|-------|
| 0–2   | Position x, y, z | m (ENU frame) |
| 3–6   | Orientation quaternion qw, qx, qy, qz |: |
| 7–9   | Linear velocity vx, vy, vz | m/s (body frame) |
| 10–12 | Angular velocity wx, wy, wz | rad/s (body frame) |
| 13–15 | Linear acceleration ax, ay, az | m/s² (body frame) |
| 16–18 | Gyroscope bias bωx, bωy, bωz | rad/s |
| 19–21 | Accelerometer bias bax, bay, baz | m/s² |

Orientation is stored as a unit quaternion internally. Roll, pitch, and yaw are derived from it for output and logging only.

---

## IMU update paths

FusionCore runs two independent IMU update steps on every message:

**1. Raw IMU update (always 6D)**
Fuses gyro rates (wx, wy, wz) and accelerations (ax, ay, az) directly. This runs regardless of `imu.has_magnetometer` and is always a 6-dimensional measurement. Angular rates for all three axes are fused here.

**2. Orientation update (from imu_filter_madgwick or driver)**
Fuses the orientation derived by a separate filter (e.g. `imu_filter_madgwick`). The dimensionality depends on `imu.has_magnetometer`:

- `true` (9-axis IMU): fuses roll, pitch, and yaw → 3D update
- `false` (6-axis IMU): fuses roll and pitch only, yaw is omitted → 2D update

These are two separate Kalman update steps with independent outlier gates. The `outlier_threshold_imu` config key applies to both.

---

## Mahalanobis outlier rejection

Before fusing any measurement, FusionCore computes how statistically implausible it is given the current state estimate:

```
d² = νᵀ · S⁻¹ · ν
```

where `ν` is the innovation (predicted vs measured) and `S` is the innovation covariance. `d²` follows a chi-squared distribution and is compared against a threshold at the 99.9th percentile. Measurements that exceed the threshold are rejected without updating the filter.

Each sensor path has its own threshold and measurement dimensionality (DOF):

| Sensor path | DOF | Default threshold | Config key |
|-------------|-----|-------------------|------------|
| GNSS position | 3 | 16.27 | `outlier_threshold_gnss` |
| Raw IMU (gyro + accel) | 6 | 15.09 | `outlier_threshold_imu` |
| IMU orientation: 9-axis | 3 | 15.09 | `outlier_threshold_imu` |
| IMU orientation: 6-axis (no mag) | 2 | 15.09 | `outlier_threshold_imu` |
| Encoder | 3 | 11.34 | `outlier_threshold_enc` |
| Heading (dual antenna / azimuth) | 1 | 10.83 | `outlier_threshold_hdg` |

`outlier_threshold_imu` applies to all IMU update paths but does not auto-rescale when the orientation update drops from 6D to 2D. The default `15.09` is calibrated for the 6D raw IMU update. If you are using a 6-axis IMU (`has_magnetometer: false`), the orientation gate runs at DOF=2: lower `outlier_threshold_imu` to `13.82` (chi2(2, 0.999)) to maintain 99.9% confidence on that path.

Note: `d²` is compared against the threshold directly (not `d`). Equivalently, `d > sqrt(threshold)` produces the same rejection boundary since d is always positive: chi2 tables use `d²` by convention.

Verified by injecting a 500 m GPS jump in testing: zero position change.

GNSS position covariance is floored before the gate. This prevents RTK-grade receivers (σxy ~3 mm) from triggering self-rejection when the filter hasn't yet converged to RTK-level accuracy.

---

## Zero velocity updates (ZUPT)

When encoder speed is below 0.05 m/s and angular rate is below 0.05 rad/s, FusionCore fuses a zero velocity pseudo-measurement with tight noise. This stops IMU noise from integrating into a false velocity estimate while the robot is stationary.

Every serious inertial navigation system does this. Without ZUPT, a robot sitting still for 20 minutes accumulates visible position drift from IMU bias.

---

## IMU bias estimation at startup

All MEMS IMUs have a small accelerometer and gyro bias that varies by unit. By default the filter starts at zero bias and takes ~60 seconds to converge, causing a small position offset (~5–10 cm) at startup.

Setting `init.stationary_window: 2.0` makes the filter collect 2 seconds of IMU data before starting, estimate bias directly from the mean readings, and initialize with correct values. Startup drift drops from ~10 cm to under 1 cm. The robot must be stationary during the window: if it moves, the filter falls back to zero bias automatically.

---

## Adaptive noise covariance

FusionCore tracks a sliding window of 50 innovation sequences per sensor and estimates the actual noise covariance from the data. The noise matrix R is slowly updated toward the estimated true value using an exponential moving average with `alpha=0.01`.

After a few minutes of operation, R converges to the real sensor characteristics automatically. No manual YAML noise tuning required.

---

## GPS lever arm

If the GPS antenna is not at `base_link`: mounted on top, or forward of center: its readings correspond to a different trajectory than `base_link`. FusionCore corrects for this:

```
p_antenna = p_base + R * lever_arm
```

Lever arm correction only activates when heading has been independently validated. Applying it with an unknown yaw makes the position estimate worse, not better.

Each GPS receiver has its own independent lever arm (`gnss.lever_arm_x/y/z` for primary, `gnss.lever_arm2_x/y/z` for secondary).

---

## Heading observability

FusionCore tracks a `heading_validated_` flag that only sets true from a genuine independent heading source:

| Source | How it works |
|---|---|
| `DUAL_ANTENNA` | Dual antenna heading message received |
| `IMU_ORIENTATION` | 9-axis AHRS published full orientation (`imu.has_magnetometer: true`) |
| `GPS_TRACK` | Robot traveled ≥5 m at speed ≥0.2 m/s with yaw rate ≤0.3 rad/s |

Before any of these, lever arm correction is disabled regardless of yaw variance. A 6-axis IMU with gyro-only yaw integration does not count as validated.

---

## GPS fix quality gating

FusionCore maps `sensor_msgs/NavSatFix.status` to an internal fix type and rejects fixes below the configured minimum:

```yaml
gnss.min_fix_type: 4   # require RTK_FIXED: reject basic GPS entirely
```

Rejection log:
```
[WARN] GNSS fix rejected (fix_type=1, min=4, hdop=1.20, quality check or Mahalanobis gate)
```

!!! warning "NavSatFix RTK_FLOAT"
    `sensor_msgs/NavSatFix` has no STATUS_RTK_FLOAT. Status 2 maps to RTK_FIXED. Setting `min_fix_type: 3` will silently starve the filter. Use 2 or 4 as meaningful thresholds.

---

## Delay compensation

FusionCore stores a ring buffer of 100 IMU messages (1 second at 100 Hz). When a delayed GPS fix arrives, it restores the closest state snapshot before the fix timestamp, re-fuses the fix at the correct time, then replays all buffered IMU messages forward to the present. This eliminates motion-model approximation error for delayed measurements.

---

## Non-holonomic ground constraint

For wheeled ground robots, FusionCore fuses a `VZ = 0` pseudo-measurement on every encoder update. This prevents vertical velocity from drifting due to IMU noise on a robot that cannot move vertically. Do not use for aerial vehicles.

---

## Sensor dropout detection

FusionCore tracks the last update time for each sensor independently. If a sensor goes silent for longer than 1 second, it is marked `SensorHealth::STALE`. The filter continues running on remaining sensors and recovers automatically when the missing sensor resumes.

---

## Message covariances

FusionCore uses the covariance values sensors actually publish rather than ignoring them:

- GPS: full 3×3 matrix when `position_covariance_type == 3`
- Wheel odometry: reads `twist.covariance` per-axis
- IMU orientation: reads `orientation_covariance` from the message

---

## Filter reset service

```bash
ros2 service call /fusioncore/reset std_srvs/srv/Trigger
```

Re-initializes the UKF state and clears the GPS reference anchor. The robot re-anchors on the next GPS fix. No node restart required. Useful after teleportation in simulation or after a catastrophic GPS jump in the field.
