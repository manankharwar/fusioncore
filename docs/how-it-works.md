# How It Works

The technical detail behind FusionCore's design decisions.

---

## Verified behavior

Each robustness claim below has a corresponding test with a hard pass threshold. These are not assertions: they are measured outcomes.

| Claim | How it is tested | Verified result |
|---|---|---|
| GPS spike rejected | [Gazebo integration test 2](simulation.md#automated-integration-tests): inject +500 m GPS fix, measure position jump | < 5 m (typically < 0.5 m) |
| Position stable while stationary | [Gazebo integration test 1](simulation.md#automated-integration-tests): 10 s IMU-only with GPS active | Drift < 2 m (typically < 0.1 m) |
| GPS correction after motion | [Gazebo integration test 3](simulation.md#automated-integration-tests): drive then stop, measure stability with GPS active | Drift < 2 m in 3 s |
| Dead reckoning stays coherent | [Gazebo integration test 4](simulation.md#automated-integration-tests): full circle return error | < 3 m (typically < 0.5 m) |
| Accuracy on real outdoor data | [NCLT benchmark](https://manankharwar.github.io/fusioncore/): 12 full-length sequences, IMU + wheels + GPS | ATE 10–60 m (10/12 FC wins over RL-EKF) |
| Spike visible in real data | [Zero-dependency demo](index.md#see-it-before-you-install): pre-baked NCLT spike test | FC +1 m vs RL-EKF +50 m on 707 m fake fix |

The Gazebo tests run against the live simulation (`python3 integration_test.py`) and are reproducible on any machine with Gazebo Harmonic installed. The NCLT benchmark runs against real rosbag data. Both are described with reproduction steps.

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

FusionCore maintains a **23-dimensional state vector**:

| Index | State | Units | Notes |
|-------|-------|-------|-------|
| 0–2   | Position x, y, z | m (ENU frame) | Local ENU, origin at first GPS fix |
| 3–6   | Orientation qw, qx, qy, qz | unit quaternion | Body-to-world rotation |
| 7–9   | Linear velocity vx, vy, vz | m/s (body frame) | |
| 10–12 | Angular velocity wx, wy, wz | rad/s (body frame) | |
| 13–15 | Linear acceleration ax, ay, az | m/s² (body frame) | |
| 16–18 | Gyroscope bias bωx, bωy, bωz | rad/s | Slow MEMS drift |
| 19–21 | Accelerometer bias bax, bay, baz | m/s² | Slow MEMS drift |
| 22    | Encoder WZ bias b_ewz | rad/s | Systematic encoder heading rate offset |

Orientation is stored as a unit quaternion internally. Roll, pitch, and yaw are derived from it for output and logging only.

**Why encoder WZ bias is in the state vector**

Wheel encoders have a small systematic angular velocity bias: the robot reports a slightly non-zero turn rate even when driving perfectly straight. This happens from wheel diameter mismatch, slight mechanical asymmetry, and mounting offset. Unlike IMU gyro bias (which drifts slowly over time), the encoder WZ bias is primarily mechanical and changes only with wear.

This bias matters more than it sounds. Over a 1-minute straight run at typical campus speed (~1.5 m/s), a 0.002 rad/s encoder WZ bias produces ~0.12 rad (~7 degrees) of heading error. At 1.5 m/s, that is a lateral displacement of ~8 m. On longer sequences or repeated loops, it accumulates further.

FusionCore estimates b_ewz online using the cross-covariance between encoder WZ and GPS position: if encoder WZ has a consistent offset, the GPS positions will systematically disagree with the encoder-integrated heading. The filter converges on the bias estimate within a few minutes of driving and subtracts it automatically. During GPS blackouts, the already-estimated bias is subtracted from encoder WZ readings rather than allowed to accumulate into heading error.

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

**Second IMU (`imu2.topic`)**
When configured, FusionCore subscribes to a second IMU topic and runs the same two update steps for each message, treating the second sensor as an independent measurement of the same state. Both IMUs call `update_imu()` separately: the filter fuses their independent noise realizations rather than averaging them before ingestion. Useful on platforms with two IMUs (e.g. VESC built-in + RealSense D435i) when you want redundancy without pre-merging them externally. Both IMUs share the same noise model and `imu.has_magnetometer` flag. Leave `imu2.topic: ""` to disable.

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

Verified by injecting a 500 m GPS jump: position moves < 0.5 m in the [Gazebo integration test](simulation.md#automated-integration-tests) and < 2 m in the [NCLT spike test](index.md#see-it-before-you-install) on real rosbag data.

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
| `MAGNETOMETER` | Raw magnetometer fused via `magnetometer.enabled: true` (tilt-compensated, iron-corrected) |
| `GPS_TRACK` | Robot traveled ≥5 m at speed ≥0.2 m/s with yaw rate ≤0.3 rad/s |

Priority: DUAL_ANTENNA overrides MAGNETOMETER overrides GPS_TRACK. Before any of these, lever arm correction is disabled regardless of yaw variance. A 6-axis IMU with gyro-only yaw integration does not count as validated.

---

## GPS fix quality gating

FusionCore maps `sensor_msgs/NavSatFix.status` to an internal fix type and rejects fixes below the configured minimum:

```yaml
gnss.min_fix_type: 4   # require RTK_FIXED: reject basic GPS entirely
```

Rejection log:
```
[WARN] GNSS fix rejected: FIX_TYPE_LOW (hdop=1.20, d2=-1.0, threshold=16.27)
```

The structured reason and Mahalanobis distance are also available on `/fusion/debug/gnss_status` for every fix.

!!! warning "NavSatFix RTK_FLOAT"
    `sensor_msgs/NavSatFix` has no STATUS_RTK_FLOAT. Status 2 maps to RTK_FIXED. Setting `min_fix_type: 3` will silently starve the filter. Use 2 or 4 as meaningful thresholds.

---

## GPS velocity fusion and wheel slip detection

GPS receivers output two completely independent things: where you are (position) and how fast you are moving (velocity, derived from Doppler shift on satellite signals). FusionCore has always fused GPS position. GPS velocity is a separate measurement with separate accuracy characteristics: and it is now fused independently.

**Why this matters: wheel slip**

Wheel odometry assumes the wheels are in contact with the ground and not slipping. On wet grass, gravel, mud, or any loose surface that assumption breaks down. The wheels report 1.0 m/s but the robot is only actually moving at 0.2 m/s: because the wheels are spinning without traction.

Before GPS velocity fusion, FusionCore had no way to detect this. The filter trusted the wheels and the position estimate drifted in the direction the wheels said.

With GPS velocity fusion, the UKF compares GPS-reported speed against wheel-reported speed on every filter cycle. The innovation (the difference between the two) directly reveals slip. A large innovation means the wheels and GPS disagree: and the Kalman gain automatically down-weights the slipping wheel measurement in proportion to how much they disagree.

**What it adds on top of GPS position**

GPS position updates arrive at 1–10 Hz. GPS velocity from the same receiver is available at the same rate but carries different information: it measures the rate of change of position via Doppler, which is accurate even when positional HDOP is poor. This means:

- Velocity correction keeps working during brief GPS position degradation
- The velocity state converges faster at startup, before enough position fixes have accumulated to observe speed geometrically
- On receivers like the F9P (RTK), GPS velocity accuracy is ~0.03 m/s: tighter than wheel odometry on most terrain

**How to enable**

```yaml
gnss.velocity_topic: "/gnss/velocity"
```

Expects `nav_msgs/Odometry` with velocity in ENU frame (`twist.linear.x` = east, `twist.linear.y` = north). FusionCore rotates ENU to body frame internally using the current quaternion: no extra node needed. Covariance from the message is used directly when positive; falls back to adaptive encoder noise otherwise.

Works with any receiver that publishes velocity. For u-blox, a small bridge node converts UBX-NAV-PVT to the generic topic format: kept in a separate package so non-u-blox users have zero extra dependencies.

Leave `gnss.velocity_topic` empty (the default) to disable. No behavior change for existing configs.

---

## Radar Doppler velocity fusion

4D imaging radar outputs a point cloud where every point carries a Doppler velocity: the radial speed of that point relative to the sensor. By fitting a velocity model across all points, a bridge node can extract the robot's ego-velocity: how fast the robot itself is moving, measured purely from radio wave physics.

This ego-velocity is completely independent of wheels, GPS, and IMU. It works indoors. It works in rain, fog, dust, and complete darkness. It does not care whether the wheels are slipping.

FusionCore fuses this as a separate measurement update alongside wheel odometry. The innovation between radar velocity and wheel velocity directly reveals slip, using the same principle as GPS velocity fusion, but available in every environment instead of only outdoors with a sky view.

**Supported hardware** (via bridge nodes):

| Radar | Output format |
|---|---|
| Continental ARS548 / ARS540 | ROS 2 driver + ego-velocity bridge |
| Oculii Eagle | ROS 2 driver + ego-velocity bridge |
| Aptiv ESR | ROS 2 driver + ego-velocity bridge |
| Any radar with ROS 2 PointCloud2 + Doppler field | Generic bridge |

**How to enable**

```yaml
radar.velocity_topic: "/radar/ego_velocity"
radar.vel_noise: 0.1    # m/s, used when message has no covariance
```

Expects `nav_msgs/Odometry` with velocity in robot body frame (`linear.x` = forward, `linear.y` = lateral). The bridge node handles raw Doppler extraction and frame conversion. FusionCore receives clean body-frame velocity.

---

## GPS track heading fusion

Dual-antenna GPS systems directly measure heading. Most deployments don't have one. FusionCore compensates by fusing heading from GPS track geometry: if the robot traveled a distance d in direction theta according to GPS positions, then the robot's heading was approximately theta.

This is the same mechanism `navsat_transform` uses internally in robot_localization. FusionCore implements it as a direct yaw pseudo-measurement fused into the UKF:

```
sigma_heading = sigma_xy / displacement  (radians)
```

where `sigma_xy` is the GPS position noise and `displacement` is the distance since the last heading fusion. At 3m GPS noise and 10m displacement, heading sigma = 0.3 rad (17 degrees), which is still useful for coarse heading correction. At 25m displacement it drops to 0.12 rad (7 degrees), tight enough to observe encoder WZ bias.

The fusion only fires when:
1. Displacement since last heading fusion >= `gnss.track_heading_min_dist` (default 5m)
2. Computed heading sigma <= `gnss.track_heading_max_sigma` (default 0.4 rad)
3. The robot is not spinning (heading rate is changing slowly)

This mechanism is what allows the encoder WZ bias to be estimated: repeated GPS track heading fusions over many segments build up the cross-covariance between b_ewz and position, and the Kalman update starts attributing systematic heading error to the bias state rather than the position state.

To disable: `gnss.track_heading_enabled: false`. Disabling it means yaw is only corrected by dual-antenna heading or 9-axis IMU orientation. Encoder WZ bias estimation degrades.

---

## Inertial coast mode

During a GPS blackout (whether because the receiver stopped publishing due to tunnel or power loss, or because consecutive fixes fail the chi2 gate), FusionCore enters **inertial coast mode**.

Coast mode serves two purposes:

**1. Keep the chi2 gate from freezing the filter out of its own recovery**

When the robot drifts during a GPS blackout, the predicted GPS position diverges from the actual GPS position. On the blackout's end, valid GPS fixes may have innovations large enough to fail the chi2 gate, not because they are outliers, but because the filter has lost track of where it is. Without any intervention, the filter would reject all returning fixes and never recover.

Coast mode solves this by inflating `Q_position` by `gnss.coast_q_factor` during the blackout. This causes position covariance P to grow at an accelerated rate. The innovation covariance S = HPH^T + R grows with P, and chi2 = nu^T S^-1 nu naturally decreases. By the time GPS resumes, P has grown large enough that valid returning fixes pass the gate regardless of how far the filter drifted.

**2. Let encoder WZ correct heading bias faster**

During GPS absence, heading errors accumulate at `gyro_bias * time` with no GPS heading cross-covariance to correct them. Coast mode inflates `Q_gyro_bias` by `gnss.coast_q_bias_factor` (default 100x) to loosen the filter's confidence in its current bias estimate. `R_imu[WZ,WZ]` can optionally be inflated by `gnss.coast_imu_wz_scale` (default 1.0, disabled) to down-weight the IMU heading rate and let encoder WZ dominate heading integration. When enabled (values of 200-1000x are typical for outdoor robots), the filter rapidly re-estimates gyro bias from encoder WZ readings during the blackout instead of letting a stale bias estimate silently corrupt heading. Leave at 1.0 if you trust your IMU gyro over your encoder during blackouts, or if you have no wheel encoder.

**Coast mode triggers:**
- After `gnss.coast_n` consecutive GPS rejections (default: 5), **but only if the rejection streak began after a real GPS gap of at least `gnss.coast_min_gap_s` seconds** (default: 1.0)
- After `gnss.coast_timeout_s` seconds of GPS silence (default: 0.0, disabled; set to 30.0 to catch receiver-silent outages where no fixes arrive to reject)

The gap condition is what keeps coast from being abused by an outlier. Coast inflates `Q_position` to re-admit GPS after the filter dead-reckoned through a gap. A *continuously present* GPS that keeps failing the gate is a persistent outlier (e.g. a sustained multipath spike), not filter drift: inflating P for it would widen the gate until the outlier slips through. Gating on a preceding gap (the receiver actually went silent) blocks that: a sustained spike during continuous GPS stays rejected for its full duration, while genuine post-outage re-acquisition still works. Set `gnss.coast_min_gap_s: 0` to restore the old gap-agnostic behavior.

**Coast mode exits** when the first GPS fix passes the chi2 gate after the blackout ends. Process noise returns to normal immediately.

```yaml
# Defaults:
gnss.coast_n: 5                  # consecutive rejections before entering coast
gnss.coast_min_gap_s: 1.0        # required preceding GPS gap before coast can fire
gnss.coast_q_factor: 20.0        # position Q multiplier during coast
gnss.coast_q_bias_factor: 100.0  # gyro bias Q multiplier during coast
gnss.coast_imu_wz_scale: 1.0     # R_imu[WZ] multiplier (1.0 = no effect, disabled)
gnss.coast_timeout_s: 0.0        # silence-based entry (0.0 = disabled)

# Recommended additions for outdoor robots with frequent GPS blackouts:
# gnss.coast_imu_wz_scale: 500.0  # encoder dominates heading during blackout
# gnss.coast_timeout_s: 30.0      # enter coast if GPS receiver goes silent (no fixes to reject)
```

**Choosing coast_q_factor**

This parameter controls a tradeoff:
- Too high (200+): P grows so large that even outlier GPS fixes (100-800m off) may pass chi2 during recovery
- Too low (1.0-2.0): P may not grow enough for legitimate drift corrections to pass chi2 after long blackouts

The NCLT benchmarks were run with `coast_q_factor: 10.0` (conservative, below the default of 20.0). After the 228s first blackout in 2012-08-20, sigma_xy = 48m, which accepts drift corrections up to 193m but rejects outliers 840m off (chi2 = 306 >> threshold 16.27). After the 461s blackout in 2012-06-15, sigma_xy = 68m, which accepts drift corrections up to 274m. At the default of 20.0, P grows faster and the recovery window is wider.

---

## Delay compensation

FusionCore stores a ring buffer of 100 IMU messages (1 second at 100 Hz). When a delayed GPS fix arrives, it restores the closest state snapshot before the fix timestamp, re-fuses the fix at the correct time, then replays all buffered IMU messages forward to the present. This eliminates motion-model approximation error for delayed measurements.

---

## Non-holonomic ground constraint

For wheeled ground robots, FusionCore fuses three pseudo-measurements on every encoder callback. Each one targets a different part of the state vector.

**VZ = 0 (vertical velocity):** The robot cannot move vertically, so body-frame vertical velocity must be zero. This is the primary constraint. The noise sigma starts at `ground_constraint.vz_sigma` (default 0.1 m/s) and adapts upward automatically when the robot traverses obstacles, curbs, or rough terrain where the chassis genuinely has transient vertical motion. On flat ground it relaxes back to the configured floor. No config changes needed between terrain types.

**AZ = 0 (vertical acceleration):** Without this, a small mismatch between the IMU's local gravity reading and the WGS84 constant (9.80665 m/s²) leaks into the AZ state. Because the UKF process noise on acceleration is large, AZ absorbs the residual. AZ then integrates into VZ via the motion model, so the VZ=0 constraint above cannot fully compensate on its own. Constraining AZ directly eliminates the source of the leak. The noise sigma starts at `ground_constraint.az_sigma` (default 0.5 m/s²) and adapts the same way as VZ.

**Z position = 0 (optional, flat-terrain mode):** On flat terrain with GPS, GPS altitude corrections carry significant noise (typically 3-5m standard deviation). This noise causes Z position to oscillate, which adds to 3D ATE even when the robot is actually on flat ground. When `ground_constraint.z_position_sigma` is set to a positive value (recommended: 0.3m for known-flat terrain), FusionCore fuses a Z = 0 pseudo-measurement tight enough to dominate GPS altitude noise and keep the filter grounded.

On terrain with real elevation change, GPS issues hundreds of consistent corrections in the correct direction, which eventually overcome the position constraint. The constraint creates some resistance but does not prevent the filter from tracking real hills. Set `ground_constraint.z_position_sigma: 0.0` (the default) for any deployment where terrain elevation is unknown or variable.

```yaml
# Flat terrain (campus paths, parking lots, warehouse floors):
ground_constraint.z_position_sigma: 0.3

# General outdoor / hilly terrain (default):
ground_constraint.z_position_sigma: 0.0
```

Do not call the ground constraint for aerial vehicles.

---

## Sensor dropout detection

FusionCore tracks the last update time for each sensor independently. If a sensor goes silent for longer than 1 second, it is marked `SensorHealth::STALE`. The filter continues running on remaining sensors and recovers automatically when the missing sensor resumes.

---

## Message covariances

### Input covariances

FusionCore uses the covariance values sensors actually publish rather than ignoring them:

- GPS: full 3×3 matrix when `position_covariance_type == 3`
- Wheel odometry: reads `twist.covariance` per-axis
- IMU orientation: reads `orientation_covariance` from the message

### Output covariance: pose.covariance

FusionCore publishes `nav_msgs/Odometry` on `/fusion/odom`. The `pose.covariance` field is a 6×6 row-major matrix following the ROS convention: `[x, y, z, roll, pitch, yaw]`. The orientation block (rows and columns 3, 4, 5) must contain variances in Euler angle space, not quaternion component space.

The UKF tracks orientation as a unit quaternion `(qw, qx, qy, qz)` and maintains a 23×23 covariance matrix P in quaternion space. Placing `P(QX,QX)`, `P(QY,QY)`, `P(QZ,QZ)` directly into the orientation block would be wrong: for a robot driving flat with a small yaw uncertainty σ_yaw, `qz = sin(yaw/2) ≈ yaw/2`, so `P(QZ,QZ) ≈ σ_yaw²/4`. Nav2 AMCL reads `pose.covariance[35]` as the yaw variance for particle resampling; it would receive a number four times too small at small angles and increasingly wrong as orientation changes.

FusionCore instead computes the analytical 3×4 Jacobian `J = d(roll,pitch,yaw)/d(qw,qx,qy,qz)` at the current quaternion and propagates:

```
C_euler     = J · P_quat(4×4) · Jᵀ       ← 3×3 Euler angle covariance
C_pos_euler = P_pos_quat(3×4) · Jᵀ       ← 3×3 position–Euler cross-covariance
```

The full 6×6 `pose.covariance` is then assembled as:

```
[ P_pos(3×3)       C_pos_euler(3×3) ]
[ C_pos_euler(3×3)ᵀ  C_euler(3×3)  ]
```

This gives Nav2, AMCL, slam_toolbox, and any other consumer the correct yaw (and roll/pitch) variance, including all cross-correlation terms between position and orientation.

**Gimbal lock:** At pitch = ±90°, all three Euler Jacobian denominators go to zero simultaneously: roll and yaw become undefined and the Jacobian is singular. FusionCore clamps all three denominators to a minimum of 1e-12, producing large-but-finite covariance rather than NaN. This is the mathematically correct behavior: at the singularity the filter genuinely does not know the roll–yaw decomposition, and the inflated covariance communicates that uncertainty correctly to downstream consumers.

---

## Observability: seeing inside the filter

The most common question when setting up a sensor fusion filter is "why did that GPS fix get rejected?" or "is the filter actually converging?" Before, FusionCore logged a generic warning and you had to guess. Now every GPS fix, accepted or rejected, produces a structured message on `/fusion/debug/gnss_status` that tells you exactly what happened.

### Why a GPS fix gets rejected

When a GPS fix arrives, FusionCore runs two checks in order. If either check fails, the fix is dropped and the reason is recorded.

**Check 1: Quality gate**

Before any math runs, FusionCore looks at the fix metadata:

- Is `hdop` above `gnss.max_hdop` (default 4.0)? If yes: `HDOP_HIGH`, drop it.
- Is `satellites` below `gnss.min_satellites` (default 4)? If yes: `MIN_SATS`, drop it.
- Is `fix_type` below `gnss.min_fix_type` (default GPS_FIX)? If yes: `FIX_TYPE_LOW`, drop it.

When a quality gate fails, `mahalanobis_sq` is set to -1.0. The chi-squared math never ran.

**Check 2: Mahalanobis gate (chi-squared)**

If the fix passes quality checks, FusionCore computes how statistically surprising it is given the filter's current belief:

```
d² = ν^T · S^-1 · ν
```

where `ν` is the difference between what the GPS reported and what the filter predicted, and `S` is the innovation covariance (how uncertain that prediction was). If `d²` exceeds 16.27 (the 99.9th percentile of a chi-squared distribution with 3 degrees of freedom), the fix is rejected as a statistical outlier. The rejection reason is `CHI2_FAILED`.

This is the gate that catches GPS spikes, multipath jumps, and fixes that arrive when the filter has drifted far from reality during a blackout.

### What the Mahalanobis distance tells you

Think of it this way: if `mahalanobis_sq` is 4.3 and the threshold is 16.27, the fix was well within the acceptance region and fused normally. If it's 847.3, the fix was roughly 53 times the acceptance boundary: almost certainly a GPS spike or severe multipath.

A rising `mahalanobis_sq` that stays just below the threshold is a warning sign: GPS quality is degrading and fixes are being accepted that are increasingly noisy. The adaptive noise system will compensate, but it is a good prompt to check the GPS antenna environment.

### The two debug topics

**/fusion/debug/gnss_status** fires on every GPS fix:

```bash
ros2 topic echo /fusion/debug/gnss_status
```

```yaml
accepted: false
rejection_reason: CHI2_FAILED
mahalanobis_sq: 847.3
chi2_threshold: 16.27
hdop: 1.2
satellites: 8
in_coast_mode: false
position_sigma_x: 2.4
position_sigma_y: 2.4
```

**/fusion/debug/filter_health** fires at 1 Hz with the overall filter state:

```bash
ros2 topic echo /fusion/debug/filter_health
```

```yaml
gnss_innovation_norm: 1.2      # meters: how far GPS fixes have been from predictions
imu_innovation_norm: 0.08      # how far IMU readings have been from predictions
encoder_innovation_norm: 0.3   # same for wheel encoders
position_sigma_x: 1.4          # 1-sigma position uncertainty right now (meters)
position_sigma_y: 1.4
heading_sigma_deg: 3.2         # heading uncertainty (degrees)
heading_validated: true
heading_source: GPS_TRACK
gnss_in_coast: false
gnss_consecutive_rejects: 0
distance_traveled_m: 47.3
gnss_outlier_count: 3
```

### What these numbers look like when the filter is healthy

**Position sigma** starts large (you set it at 30-50m in the initial covariance), drops rapidly as the first few GPS fixes arrive, then slowly grows during GPS blackouts and drops again on recovery. A plot of `position_sigma_x` over time should look like a sawtooth that trends downward as the filter learns the robot's motion.

**Innovation norms** should be roughly stable during normal operation. A sudden spike in `gnss_innovation_norm` followed by a return to baseline is exactly what good GPS outlier rejection looks like. The spike means a bad fix arrived, the filter rejected it (check `gnss_status.rejection_reason`), and position continued from inertial prediction until GPS recovered.

**`heading_sigma_deg`** starts large and drops once the robot has traveled 5 m in a consistent direction and GPS track heading fires for the first time. If it stays large, the robot may not have moved enough, or may be turning a lot. Once it drops below ~10 degrees, the filter has a reliable heading estimate and lever arm correction activates.

### No new dependencies

These topics use standard ROS 2 machinery. `GnssStatus` and `FilterHealth` are custom message types defined inside `fusioncore_ros` itself. No external visualization tool is required. Any tool that can subscribe to a ROS 2 topic can read them: `ros2 topic echo`, `rqt_plot`, PlotJuggler, Foxglove, or your own Python script.

---

## Filter reset service

```bash
ros2 service call /fusioncore/reset std_srvs/srv/Trigger
```

Re-initializes the UKF state and clears the GPS reference anchor. The robot re-anchors on the next GPS fix. No node restart required. Useful after teleportation in simulation or after a catastrophic GPS jump in the field.

---

## Wait for all sensors before starting

By default FusionCore starts the filter as soon as it receives the first IMU message. That means if GPS and wheel odometry come online 2--3 seconds later (which is normal during system startup), the filter has already drifted purely on IMU integration before any other sensor can anchor it. The result is a small but visible position jump at the moment the encoder and GPS first arrive.

Setting `init.wait_for_all_sensors: true` fixes this. The filter sits in a holding state and does nothing until every sensor you have configured has published at least one message. Then it starts with a full picture from the beginning.

```yaml
init.wait_for_all_sensors: true
init.sensor_wait_timeout: 10.0   # give up and start anyway after this many seconds
```

FusionCore knows which sensors to wait for based on what you have configured:

| Sensor | Waited for when... |
|---|---|
| IMU | always |
| Encoder | always |
| GNSS | `reference.use_first_fix: true` |
| GPS velocity | `gnss.velocity_topic` is non-empty |
| Radar velocity | `radar.velocity_topic` is non-empty |
| Heading | `gnss.heading_topic` or `gnss.azimuth_topic` is non-empty |
| Second IMU | `imu2.topic` is non-empty |
| Second encoder | `encoder2.topic` is non-empty |
| Second GNSS | `gnss.fix2_topic` is non-empty |

If a sensor never arrives within `sensor_wait_timeout` seconds, the filter logs a warning listing which sensors were missing and starts anyway. You never get a filter that hangs forever.

During the wait, FusionCore logs which sensors are still missing every 2 seconds:

```
[INFO] Waiting for sensors (1.4s / 10.0s): missing [GNSS, GPSVel]
[INFO] All 4 configured sensors ready. Starting filter.
```

This replaces the `sleep(3)` workaround that appears in virtually every ROS launch file.

---

## Pluggable motion models

The predict step is the core of any UKF: it propagates the state forward in time between measurements using a mathematical model of how the robot moves. FusionCore's motion model is now configurable via YAML.

**Why it matters**

The default model (`ConstantVelocityAcceleration`) assumes the robot can move freely in all directions. It integrates body-frame velocity and acceleration without any platform-specific constraints. This is correct for aerial vehicles and a reasonable approximation for ground robots.

But a differential drive robot *cannot* move sideways. Physics forbids it. The default model does not know this: it allows lateral velocity (`VY`) to grow between encoder updates via IMU acceleration integration. The measurement model corrects it at every encoder update, but between updates the uncertainty in `VY` accumulates unnecessarily. The position estimate then carries a small lateral smear that the covariance matrix reflects.

The `DifferentialDrive` model enforces the non-holonomic constraint directly in the predict step: `VY` and `AY` are zeroed every time the UKF propagates forward. The filter now *knows* lateral velocity does not grow. The covariance in `VY` stays tight, position integration never accumulates false lateral displacement, and the encoder measurement just confirms what the model already predicted.

**How to enable**

```yaml
motion_model: "DifferentialDrive"
```

For Ackermann-steered platforms (forklifts, outdoor vehicles, any front-steered robot):

```yaml
motion_model: "Ackermann"
motion_model_params:
  wheelbase: 0.55    # meters
```

To keep the original behavior (and for aerial vehicles):

```yaml
motion_model: "ConstantVelocityAcceleration"   # or just leave the line out
```

**Available models**

| Model | Lateral constraint | Use for |
|---|---|---|
| `ConstantVelocityAcceleration` | None (default) | All platforms, aerial vehicles |
| `DifferentialDrive` | VY = 0 during prediction | Differential drive, skid-steer, tracked |
| `Ackermann` | VY = 0 during prediction | Car-like, forklifts, front-steered |

**What the DifferentialDrive model does not do**

It does not break slip detection. If the wheels are actually sliding laterally (ice, mud, ramp), the GPS velocity or radar Doppler measurement will still observe non-zero lateral velocity in the update step and correct the state accordingly. The constraint applies only to the prediction. The filter can still update away from it when the evidence demands it.

---

## Deterministic replay and state checkpoints

Debugging a sensor fusion filter running live is painful. By the time you notice a bad trajectory, the moment is gone and the only way back is replaying the entire rosbag from scratch.

FusionCore addresses this with two tools: deterministic replay and state checkpoints.

**Deterministic replay**

When replaying a bag with `ros2 bag play --clock` and `use_sim_time: true`, FusionCore uses the bag's message timestamps everywhere, including the stationary bias initialization window. This means:

- Same bag + same config = identical output every run
- Tweaking one parameter and diffing trajectories actually works
- Parameter sensitivity analysis becomes rigorous, not approximate

Previously the bias window used the wall clock, so two replays of the same bag could produce slightly different outputs even with identical config. That made parameter tuning feel like guessing.

No configuration required. Replay behavior is automatic when message timestamps are non-zero (all properly recorded bags). The wall clock fallback is preserved only for drivers that publish zero-stamped messages.

**State checkpoints**

```bash
ros2 service call /fusioncore/save_checkpoint std_srvs/srv/Trigger
ros2 service call /fusioncore/load_checkpoint std_srvs/srv/Trigger
```

`save_checkpoint` serializes the full 23-dimensional state vector and 23×23 covariance matrix to a text file (`/tmp/fusioncore_checkpoint.txt` by default, configurable via `replay.checkpoint_path`).

`load_checkpoint` restores that exact state and resumes the filter from that point.

**The workflow this enables**

Suppose you have a 20-minute bag and the trajectory goes wrong at minute 15. Without checkpoints, every parameter tweak requires a full 20-minute replay. With checkpoints:

1. Replay the bag to minute 14
2. Call `save_checkpoint`
3. Call `load_checkpoint`; the filter is now back at minute 14 in under a second
4. Let the bag play from minute 14, observe what happens at minute 15
5. Tweak a parameter, call `load_checkpoint` again, repeat

The checkpoint file is plain text: human readable, diffable, and editable. You can inspect the state vector directly to understand what the filter believed at the moment of failure.

```yaml
replay.checkpoint_path: "/tmp/fusioncore_checkpoint.txt"   # default
```
