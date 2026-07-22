# Topics & TF

## Subscribes to

| Topic | Type | Notes |
|---|---|---|
| `imu.topic` | `sensor_msgs/Imu` | Primary IMU angular velocity and linear acceleration (default `/imu/data`) |
| `imu2.topic` | `sensor_msgs/Imu` | Second IMU (optional): fused as independent measurement of same state |
| `encoder.topic` | `nav_msgs/Odometry` | Wheel encoder velocity (default `/odom/wheels`). Only the twist is fused; pose is ignored. |
| `gnss.fix_topic` | `sensor_msgs/NavSatFix` or `gps_msgs/GPSFix` | GPS position (optional, default `/gnss/fix`). Message type is NavSatFix by default. Set `gnss.use_gps_fix: true` to subscribe as GPSFix: unlocks RTK_FLOAT status, receiver-native HDOP/VDOP, and err_horz/err_vert covariance bounds. |
| `/gnss/heading` | `sensor_msgs/Imu` | Dual antenna heading (optional) |
| `gnss.azimuth_topic` | `compass_msgs/Azimuth` | Azimuth heading (optional, preferred) |
| `magnetometer.topic` | `sensor_msgs/MagneticField` | Raw 3-axis magnetometer (optional): tilt-compensated heading with hard/soft iron correction |
| `gnss.fix2_topic` | `sensor_msgs/NavSatFix` | Second GPS receiver (optional) |
| `encoder2.topic` | `nav_msgs/Odometry` | Second velocity source: lidar odom, visual odom (optional) |
| `gnss.velocity_topic` | `nav_msgs/Odometry` | GPS velocity in ENU frame: linear.x=east, linear.y=north (optional) |
| `radar.velocity_topic` | `nav_msgs/Odometry` | Radar Doppler ego-velocity in body frame: linear.x=forward, linear.y=lateral (optional) |
| `vslam.topic` | `nav_msgs/Odometry` | VSLAM 6-DOF pose: pose.pose + pose.covariance used; twist ignored (optional) |

### fusioncore_ublox bridge topics (separate package)

| Topic | Type | Notes |
|---|---|---|
| `/ublox/navpvt` (configurable) | `ublox_msgs/NavPVT` | Input: raw u-blox NavPVT message |
| `/gnss/doppler_vel` (configurable) | `nav_msgs/Odometry` | Output: ENU velocity, wire to `gnss.velocity_topic` |

The `fusioncore_ublox` package is a separate optional companion; FusionCore has no dependency on it. See [GNSS Doppler bridge](../configuration.md#gnss-doppler-velocity-bridge-ublox-f9p--m8u) for setup.

### Changing the input topic names

Every subscribed topic is a parameter, so the normal way to point FusionCore at your
driver is to set it in your config YAML:

```yaml
fusioncore:
  ros__parameters:
    imu.topic:      "/camera/imu"
    encoder.topic:  "/diff_drive_controller/odom"
    gnss.fix_topic: "/ublox/fix"
```

Note the wheel odometry default is `/odom/wheels`, not the conventional `/odom`. That is
deliberate: FusionCore publishes its own fused odometry, so subscribing to `/odom` would
invite a feedback loop with its own output. Point `encoder.topic` at your driver instead.

ROS 2 remaps still work and are applied on top of whatever name the parameter resolves to:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=your_robot.yaml \
  --ros-args \
  -r /odom/wheels:=/your/wheel/odom \
  -r /gnss/fix:=/fix
```

FusionCore logs the topics it actually subscribed to at startup, so you can confirm the
wiring without guessing:

```
[INFO] [fusioncore]: IMU topic: /imu/data
[INFO] [fusioncore]: Encoder topic: /odom/wheels
[INFO] [fusioncore]: GNSS topic: /gnss/fix (sensor_msgs/NavSatFix)
```

---

## Publishes

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/fusion/odom` | `nav_msgs/Odometry` | 100 Hz | Fused position + orientation + velocity + full covariance |
| `/fusion/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 100 Hz | Same pose: compatible with AMCL, slam_toolbox, Nav2 pose initializer |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | Per-sensor health, outlier counts, heading status |
| `/fusion/debug/gnss_status` | `fusioncore_ros/GnssStatus` | On every GPS fix | Why each fix was accepted or rejected, with the Mahalanobis distance |
| `/fusion/debug/filter_health` | `fusioncore_ros/FilterHealth` | 1 Hz | Innovation norms, position uncertainty, heading sigma, coast mode state |
| `/tf` | TF | 100 Hz | `odom → base_link` |

---

## Debug topics

These two topics are the answer to the question engineers always ask: "how do I know if FusionCore is actually working?" They expose the filter's internal reasoning so you can see exactly what is happening, not just what the output looks like.

### `/fusion/debug/gnss_status`

Published every time a GPS fix arrives, whether it was accepted or not. This is the single most useful topic for diagnosing GPS problems.

```yaml
header:
  stamp: ...
  frame_id: odom

accepted: false               # true if the fix was fused into the filter

rejection_reason: CHI2_FAILED # what went wrong (see table below)

mahalanobis_sq: 847.3         # d^2 = nu^T * S^-1 * nu
                              # how many "standard deviations away" the fix was
                              # compared to what the filter expected
                              # -1.0 means the quality gate failed before chi2 was computed

chi2_threshold: 16.27         # the acceptance boundary (configured outlier_threshold_gnss)
                              # if mahalanobis_sq > chi2_threshold the fix is rejected

hdop: 1.2                     # from the GPS message
vdop: 1.8
satellites: 8
fix_type: 1                   # 1=GPS, 2=DGPS, 3=RTK_FLOAT, 4=RTK_FIXED

in_coast_mode: false          # whether the filter is currently in GPS coast mode
consecutive_rejects: 0        # how many consecutive fixes have been rejected

position_sigma_x: 2.4         # sqrt(P[x,x]) at the moment this fix arrived
position_sigma_y: 2.4         # how uncertain the filter was about position right now
```

**`rejection_reason` values:**

| Value | What it means | Where to look |
|---|---|---|
| `ACCEPTED` | Fix was fused successfully | All good |
| `HDOP_HIGH` | hdop exceeded `gnss.max_hdop` (default 4.0) | GPS signal quality or environment |
| `VDOP_HIGH` | vdop exceeded the vertical DOP limit | Same as above |
| `MIN_SATS` | Fewer satellites than `gnss.min_satellites` (default 4) | Sky view is blocked |
| `FIX_TYPE_LOW` | fix_type below `gnss.min_fix_type` (default GPS) | Receiver hasn't locked |
| `CHI2_FAILED` | Passed quality gates but Mahalanobis distance too large | GPS jump, multipath, or filter drift |
| `DELAY_TOO_LARGE` | Fix arrived more than `max_measurement_delay` seconds late | Network or driver latency |
| `IMPLAUSIBLE_JUMP` | Fix farther from the prediction than `gnss.max_speed` allows | Impossible GPS jump (only when `gnss.max_speed` > 0) |

**Reading `mahalanobis_sq`:**

This number tells you how statistically surprising the GPS fix was given the filter's current state. The threshold is 16.27 (chi-squared at 99.9%, 3 DOF). So:

- `mahalanobis_sq: 4.3` with `accepted: true` is a normal fix, the filter expected roughly this position
- `mahalanobis_sq: 847.3` with `accepted: false` is a GPS spike, the fix was ~53x the acceptance boundary
- `mahalanobis_sq: -1.0` means the quality check (HDOP/satellites) already rejected it before the math ran

---

### `/fusion/debug/filter_health`

Published at 1 Hz alongside `/diagnostics`. Every field is a plain float64, so Foxglove, PlotJuggler, and rqt_plot can graph them directly without any custom panel.

```yaml
header:
  stamp: ...
  frame_id: odom

# How large was the last accepted measurement residual per sensor?
# "Residual" means: what the sensor reported vs what the filter predicted.
# A small norm = the sensor agrees with the filter's prediction.
# A growing norm = sensor and filter are diverging (calibration drift, bad config).
gnss_innovation_norm: 1.2       # meters: last accepted GPS position residual
imu_innovation_norm: 0.08       # mixed units: gyro (rad/s) + accel (m/s^2) 6D norm
encoder_innovation_norm: 0.3    # m/s norm: velocity residual at last encoder update

# How uncertain is the filter right now? (1-sigma from P diagonal)
# These shrink after GPS fixes and grow during GPS outages.
position_sigma_x: 1.4           # meters: east uncertainty
position_sigma_y: 1.4           # meters: north uncertainty
position_sigma_z: 2.1           # meters: altitude uncertainty

# Heading uncertainty in degrees (propagated from quaternion covariance via Jacobian)
# Shrinks as GPS heading fuses. Grows during long straight runs without heading updates.
heading_sigma_deg: 3.2

# Heading observability
heading_validated: true
heading_source: GPS_TRACK       # NONE | GPS_TRACK | IMU_ORIENTATION | DUAL_ANTENNA

# GPS coast mode: entered when GPS goes quiet or consecutively rejects
gnss_in_coast: false
gnss_consecutive_rejects: 0

# Distance driven since filter init (meters)
# Heading becomes observable from GPS track geometry above 5 m
distance_traveled_m: 47.3

# Running totals of how many measurements were rejected by the outlier gate
gnss_outlier_count: 3
imu_outlier_count: 0
encoder_outlier_count: 0
```

**What healthy looks like in a plot:**

- `position_sigma_x/y` starts high (large initial uncertainty) then drops rapidly as GPS fixes arrive, then slowly grows during GPS outages and drops again on recovery
- `gnss_innovation_norm` stays roughly constant when GPS is stable, spikes on multipath, then drops back
- `heading_sigma_deg` drops from large to small once 5 m of travel validates heading from GPS track geometry
- `gnss_in_coast` goes true during tunnels or urban canyons, false when GPS resumes

**How to view these in Foxglove:**

Open Foxglove, connect to your ROS 2 system or load a bag, add a "Plot" panel, and drag `/fusion/debug/filter_health.position_sigma_x` or any field onto it. No custom extension or plugin required. These are plain topics with plain fields.

---

## Services

| Service | Type | What it does |
|---|---|---|
| `~/reset` | `std_srvs/Trigger` | Re-initializes the filter and clears GPS reference. No restart needed. |
| `~/save_checkpoint` | `std_srvs/Trigger` | Saves full filter state (23-state + covariance) to `replay.checkpoint_path`. |
| `~/load_checkpoint` | `std_srvs/Trigger` | Restores filter state from `replay.checkpoint_path`. Resumes from that exact point. |

---

## TF tree

FusionCore publishes:

```
odom → base_link    (always, at publish_rate)
```

Your URDF provides `base_link → imu_link` and other sensor transforms.

FusionCore does **not** publish `map → odom` under any condition. For navigation with a map frame, a separate SLAM node (slam_toolbox, cartographer) provides `map → odom`. For GPS-only outdoor navigation without SLAM, configure Nav2 with `global_frame: odom` everywhere: see [Nav2 Integration](../nav2.md).
