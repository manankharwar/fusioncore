# Troubleshooting

Quick answers to the most common problems. Each section states what you see, why it happens, and the exact fix.

---

## `/fusion/odom` is not publishing

**Check 1: Is the node active?**

```bash
ros2 lifecycle get /fusioncore
```

It should say `active`. The launch files bring the node up on their own, so if you started it with one, this is already done:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

If it says `unconfigured` or `inactive`, you either started `fusioncore_node` directly (a bare `ros2 run` does not transition a lifecycle node), or you passed `autoconfigure:=false`. Drive it by hand:

```bash
ros2 lifecycle set /fusioncore configure
ros2 lifecycle set /fusioncore activate
```

**Check 2: Is the filter initialized?**

FusionCore initializes on the first IMU message. If `/imu/data` isn't publishing, the filter never starts.

```bash
ros2 topic hz /imu/data
```

If that's silent, fix your IMU driver or remap to the correct topic:

```bash
--ros-args -r /imu/data:=/your/imu/topic
```

**Check 3: `init.stationary_window` stuck**

If you have `init.stationary_window > 0` and your IMU driver publishes zero timestamps (`stamp={sec=0, nanosec=0}`), the window never completes on versions before `0.2.2`. Either upgrade to `0.2.2+` (fixed) or set `init.stationary_window: 0.0` as a workaround.

To check if your driver has zero timestamps:

```bash
ros2 topic echo /imu/data --field header.stamp --once
```

If you see `sec: 0, nanosec: 0`, upgrade FusionCore or disable the window.

---

## Filter initializes but robot position jumps or flies off screen

**Most likely: something else is also publishing `odom → base_link` TF.**

FusionCore owns the `odom → base_link` transform. If another node (ros2_control, robot_localization, a static publisher) is also publishing it, you get a TF conflict and the position jumps.

```bash
ros2 run tf2_ros tf2_monitor odom base_link
```

Look at the `Broadcasters` line. There should be exactly one: `fusioncore`. If you see others, disable them:

- **ros2_control**: set `enable_odom_tf: false` in your controller YAML
- **robot_localization**: remove it from your launch file entirely
- **static_transform_publisher**: remove it if it's publishing odom→base_link

**If running without GPS**: check `encoder2.topic`. If it points to a topic that isn't publishing (e.g. LiDAR ICP without the LiDAR on), FusionCore may receive garbage or stale data. Set `encoder2.topic: ""` to disable it until the sensor is running.

---

## Robot spins or rolls in RViz when driving straight

**Most likely: Madgwick filter conflict.**

If `imu_filter_madgwick` is running and FusionCore is receiving its output (on `/imu/data`), the Madgwick orientation quaternion has the IMU's physical mounting rotation baked in. FusionCore interprets this as raw angular velocity and the axes get swapped: the result looks like spinning or rolling.

**Fix**: remap FusionCore to raw IMU, keep Madgwick running for any other consumers (RTABMAP, ICP):

```bash
--ros-args -r /imu/data:=/imu    # wherever your raw IMU publishes
```

FusionCore estimates orientation internally. It does not need Madgwick.

See [RTABMAP + Madgwick separation](hardware/icp-indoor.md#using-rtabmap-alongside-fusioncore-madgwick-separation) for the full setup.

---

## 30° heading error at startup

The IMU bias window (`init.stationary_window`) hasn't settled before the robot starts moving, or there's no independent heading source and the gyro integration starts from an arbitrary yaw.

- Keep the robot **completely still** for the duration of `init.stationary_window` after launch
- For outdoor robots: wait for the first GPS fix: heading self-corrects from motion after 5 m of straight travel
- For indoor robots with a 9-axis IMU: set `imu.has_magnetometer: true`: the magnetometer provides absolute heading immediately

---

## Velocity drift or position error when IMU is tilted or robot is on a slope

FusionCore handles this correctly without any configuration change.

The accelerometer measures body acceleration plus gravity projected onto the sensor axes. When the IMU is tilted, gravity projects differently onto each axis. FusionCore's UKF measurement function models this explicitly: at every update step, it computes how much gravity should appear on each axis given the current quaternion estimate, and treats only the residual as signal. The filter knows exactly what the accelerometer should read at any orientation.

robot_localization removes gravity as a preprocessing step before the filter sees the data. That step assumes the robot is level. On a slope or with a physically tilted IMU, the preprocessing subtracts the wrong gravity vector, the residual bleeds into the velocity estimate, and the error accumulates over time without a way to correct it.

Because FusionCore carries the full quaternion in its state vector and the gravity projection is computed inside the measurement function at every update, tilt-induced drift self-corrects whenever the filter updates orientation (via GPS heading, magnetometer, or AHRS roll/pitch). There is no special slope mode or tilt compensation parameter to set.

If you are seeing position drift on a tilted platform and the above does not resolve it, check:

```bash
ros2 topic echo /imu/data --field linear_acceleration --once
```

When the robot is stationary on a slope, the acceleration vector magnitude should still be close to 9.81 m/s², just pointing in a different direction. If the magnitude is near zero, your driver is publishing free acceleration (gravity already removed). In that case set:

```yaml
imu.remove_gravitational_acceleration: true
```

This tells FusionCore the driver has already removed gravity, so the measurement function uses the raw body acceleration instead.

---

## SLAM map looks like a starburst or explosion pattern

Each incoming scan is placed at a different wrong position, and over time the map fans out in all directions from where the robot started.

This is caused by RTABMAP's SLAM node receiving a corrupted odometry estimate. The most common cause: RTABMAP is subscribed to FusionCore's `/fusion/odom` (or remapped to `/odom`), and FusionCore's estimate degraded at some point during the run.

FusionCore fuses ICP odometry with IMU. When the ICP node throws a registration failure mid-run (the `Registration failed: cannot compute transform` error, which often happens during fast rotation), FusionCore briefly runs on IMU alone. If the IMU has any calibration offset, the position estimate diverges rapidly. Any scans RTABMAP places during that window get stamped with wrong positions and corrupt the map permanently.

**Fix:** wire RTABMAP's SLAM node to subscribe to `icp_odom` directly instead of FusionCore's output:

```python
Node(
    package='rtabmap_slam', executable='rtabmap', output='screen',
    parameters=[{
        'frame_id': 'base_link',
        'subscribe_scan': True,
        'approx_sync': True,
    }],
    remappings=[
        ('odom', '/icp_odom'),
        ('imu', '/imu/data'),
    ]
),
```

With this setup, when ICP fails mid-run, RTABMAP stops placing scans and waits for ICP to recover rather than placing them at bad positions. The map only accumulates from frames where the position was geometrically reliable.

FusionCore still runs and still owns the `odom → base_link` TF. Nav2 still reads `/fusion/odom`. The change only affects what RTABMAP uses when deciding where to stamp each incoming scan. FusionCore is the right source for navigation. Scan-consistent ICP odometry is the right source for map building.

Also check that `frame_id` in your RTABMAP parameters is set to `base_link`, not a camera frame like `oak-d-base-frame`. Building the map relative to a camera frame causes the map to rotate around the camera origin during turns rather than the robot center, which produces subtle geometric errors at every rotation.

---

## Camera image not showing in RtabmapViz

This is not a FusionCore issue. FusionCore publishes `/fusion/odom` and `odom → base_link` TF: it has no involvement in the camera pipeline.

Check that your camera topics are live before RTABMAP starts:

```bash
ros2 topic hz /right/image_rect
ros2 topic hz /stereo/depth
ros2 topic hz /rgbd_image
```

If any are silent, `rgbd_sync` isn't syncing. Check the depthai driver and the topic remappings in your `rgbd_sync` node.

---

## `ros2 lifecycle set` returns "Node not found"

DDS discovery latency in WSL2 or slow machines. The node is up but hasn't been discovered yet.

Use the launch file's auto-configure instead: it uses timed lifecycle events which are immune to discovery latency. Or wait 2–3 seconds and retry the manual command.

---

## TF errors: "odom → base_link does not exist"

FusionCore publishes this transform only after it's **active** and has received its first IMU message. If a downstream node (Nav2, slam_toolbox) starts before FusionCore is active, it will log this error until the transform appears.

Confirm FusionCore is actually publishing:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

If transforms are printing, the error is a race condition at startup: the downstream node started before FusionCore. The `fusioncore_nav2.launch.py` launch file adds an 8-second delay before starting Nav2 to prevent this.

---

## Encoder or GPS getting rejected (outlier gate)

**Start here: the at-a-glance field.** `/fusion/debug/filter_health` carries `gnss_last_reject_reason`, the reason the most recent fix was dropped (empty until the first rejection). This is the fastest check in the field and the one that survives in a recorded bag, because it is on the lightweight health topic you are already monitoring and recording:

```bash
ros2 topic echo /fusion/debug/filter_health --field gnss_last_reject_reason
```

!!! warning "If every fix is rejected and you are on a NavSatFix"

    `sensor_msgs/NavSatFix` carries no DOP fields. FusionCore derives the fix
    quality from `position_covariance`, which is **metres**, and gates it with
    `gnss.max_sigma_xy` / `gnss.max_sigma_z`.

    Until this was fixed, those metres were compared against `gnss.max_hdop` (4.0) and
    `gnss.max_vdop` (6.0). Those read like dimensionless DOP limits, so they
    looked generous while actually meaning "reject anything worse than 4 m
    horizontal, 6 m vertical". Real receivers fail that constantly. Measured on
    a u-blox NEO-M9N over a 500-fix outdoor run: horizontal sigma 3.6 to 6.0 m,
    vertical 14.4 to 24.0 m, so **all 500 fixes were rejected** and the filter
    dead-reckoned the entire way with no error raised anywhere.

    If you are on an older release and your GPS never seems to be used, set
    `gnss.max_hdop: 25.0` and `gnss.max_vdop: 60.0` and it will start working.

Do not rely on `gnss_outlier_count` alone to tell you GPS is being rejected: that counter only counts chi2 and physical-plausibility rejects. Quality-gate rejects (`HDOP_HIGH`, `VDOP_HIGH`, `FIX_TYPE_LOW`, `MIN_SATS`) and `DELAY_TOO_LARGE` leave it at zero, so a filter dropping every fix on vertical DOP shows `gnss_outlier_count: 0` while `gnss_last_reject_reason: VDOP_HIGH`.

**For the full per-fix detail**, look at the structured debug firehose (one message per fix, accepted or not):

```bash
ros2 topic echo /fusion/debug/gnss_status
```

The `rejection_reason` field tells you exactly which gate fired. The `mahalanobis_sq` field tells you how far the fix was from the filter's prediction (-1.0 means the quality gate failed before the math ran).

```yaml
accepted: false
rejection_reason: HDOP_HIGH     # quality gate: signal was too noisy
mahalanobis_sq: -1.0            # chi2 math never ran
hdop: 6.8                       # this is why

---
accepted: false
rejection_reason: CHI2_FAILED   # passed quality gates but position was statistically implausible
mahalanobis_sq: 847.3           # 53x above the 16.27 threshold: GPS spike
chi2_threshold: 16.27
```

For encoder and IMU rejections, check the running counts:

```bash
ros2 topic echo /fusion/debug/filter_health --field encoder_outlier_count
ros2 topic echo /fusion/debug/filter_health --field imu_outlier_count
```

Common causes and fixes:

| Reason | Cause | Fix |
|---|---|---|
| `SIGMA_XY_HIGH` | Reported horizontal 1-sigma exceeds `gnss.max_sigma_xy` (default 25 m) | Genuinely poor signal. Check antenna placement and ground plane before raising it |
| `SIGMA_Z_HIGH` | Reported vertical 1-sigma exceeds `gnss.max_sigma_z` (default 50 m) | Vertical is always worse than horizontal. On a `publish.force_2d` robot it does not matter, so raise it freely |
| `HDOP_HIGH` at startup | Open sky not acquired yet | Normal: clears within 30–60 s once the receiver locks |
| `HDOP_HIGH` / `VDOP_HIGH` persistently | Receiver reports genuine DOP and the geometry is poor | Raise `gnss.max_hdop` / `gnss.max_vdop`. Note these apply **only** when the fix carries no covariance; with a NavSatFix the sigma gate above is what runs |
| `FIX_TYPE_LOW` | `gnss.min_fix_type` set above what the receiver provides (e.g. RTK required on a non-RTK M9N) | Lower `gnss.min_fix_type` to `1` (GPS) for a consumer receiver |
| `MIN_SATS` | Fewer satellites than `gnss.min_satellites` | Move to more open sky; lower `gnss.min_satellites` only if you understand the accuracy cost |
| `DELAY_TOO_LARGE` | Fix arrived older than `max_measurement_delay` (default 0.5 s). Two distinct causes: a clock-sync/timestamp problem, or real transport latency (fix stamped correctly but delivered late) | First compare each sensor's `header.stamp` to the node clock (clock problem). If stamps are fine, the messages are arriving late: on WiFi links (hotspots especially) disable power save (`iw wlan0 set power_save off`) and set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` when all nodes share one host, so DDS stays off the wireless interface. A moderately late fix is still valuable: raising `max_measurement_delay` (e.g. 2.0) lets the snapshot/IMU-replay retrodiction rewind and fuse it at its true time. Field case: a hotspot's power-save latency delivered every fix 1 to 3 s late and all were discarded; at 2.0 s the same data fused to ~2 m sigma |
| `CHI2_FAILED` after outage | Filter drifted during a blackout, the returning GPS fails the gate | Coast mode relaxes the gate automatically; no action needed |
| `CHI2_FAILED` persistently | Fix is far from what the filter predicts | Check for antenna obstruction, a multipath spike (working as intended), or a TF/lever-arm mismatch |
| `IMPLAUSIBLE_JUMP` | Fix is farther from the prediction than `max_speed * gap + max_speed_margin + max_speed_sigma_k * reported_sigma` allows | Working as intended for a genuine spike. If it fires on ordinary motion the bound is too tight for your receiver: raise `gnss.max_speed_sigma_k` (or `gnss.max_speed_margin`) rather than `gnss.max_speed`, which is a kinematic spec and should already be a few times cruise speed. Setting `gnss.max_speed: 0.0` disables the gate entirely and leaves chi2 as the outlier defence |
| `encoder_outlier_count` climbing | Noise config too tight vs actual velocity variance | Loosen `encoder.vel_noise` or enable `adaptive.encoder: true` |
| `imu_outlier_count` climbing | Driver publishing wrong scale or units | Check `linear_acceleration.z` at rest: should be ~9.81 or ~0.0 depending on `imu.remove_gravitational_acceleration` |

Do **not** lower outlier thresholds below their chi-squared critical values. At `7.0` normal GPS noise trips the gate and every fix gets rejected. The defaults are statistically calibrated.

---

## Fusion output far worse than raw wheel odometry (position runs away)

**Symptom:** your wheel odometry alone is reasonable (drives 6 m, reads ~6 m), but the fused output diverges wildly: position climbing tens or hundreds of meters, orientation matching neither the IMU nor the odometry.

**Most likely cause: your sensors are not on a common clock.** A filter fuses measurements *ordered by their timestamps*. If one driver stamps with a different clock (an IMU on a companion board, a sensor on a second machine without NTP, a driver stamping with its own epoch), that sensor's stamps can run seconds ahead of the others. The filter clock rides the leading sensor, and every message from the lagging sensors arrives looking seconds old and **cannot be fused at all**.

**How to check, in 30 seconds:**

```bash
ros2 topic echo /your/imu/topic --field header.stamp --once
ros2 topic echo /your/odom/topic --field header.stamp --once
```

The two stamps should agree within milliseconds. If they differ by more than `max_measurement_delay` (0.5 s default), that is the whole problem. FusionCore also tells you directly:

- At startup: `IMU header.stamp is +3.14s from this node's clock...`
- At runtime: `Dropping stale sensor samples at 48.2/s (totals imu=0 encoder=412): that sensor is effectively NOT being fused. IMU stamp minus encoder stamp is +3.14s against a max_measurement_delay of 0.50s`
- On the health topic: `ros2 topic echo /fusion/debug/filter_health --field encoder_stale_reject_count`

**Read the rate, not the total.** A time-base mismatch rejects nearly every sample, so the rate lands near the sensor's publish rate (tens per second) and the totals climb without bound. A handful of drops over a whole run is a different thing: a late sample on a wireless link, harmless, and the filter keeps fusing everything else. The warning above only fires above 1 rejection per second sustained; below that the drop is logged at debug level. So `encoder_stale_reject_count: 2` after ten minutes of driving is not a problem, and `412` and climbing is.

If the rate is low but nonzero and you want those stragglers fused rather than dropped, the offset is real transport latency rather than skew: raise `max_measurement_delay` above the offset you measured and retrodiction will rewind and fuse them at their true time.

**The fix is in the sensor driver, not the filter:**

- Stamp messages with the node clock (`this->get_clock()->now()` / `node.get_clock().now()`) instead of a device or companion-board clock.
- If sensors live on different machines, sync the clocks with chrony or NTP.
- If a sensor has a genuinely large, known latency (not skew), raise `max_measurement_delay`, but understand this widens the retrodiction window for everyone.

Versions before 0.3.4 did not reject the skewed sensor: they repeatedly re-based the filter clock backward and re-integrated the offset window at the fast sensor's rate, which is what produced the runaway. Upgrade if you see this signature on an older build.

---

## `init.stationary_window` aborted immediately

The encoder is publishing non-zero velocity during the window. Either the robot is moving, or the wheel odometry driver publishes a non-zero initial value even when stationary.

Check what the encoder reports at rest:

```bash
ros2 topic echo /odom/wheels --field twist.twist.linear --once
```

If `x` is non-zero at standstill, either fix the driver or set `init.stationary_window: 0.0` and rely on the UKF to converge bias naturally (~60 s).

---

## FusionCore odom drifts more than raw wheel odometry in Gazebo

This is expected in simulation. Gazebo's DiffDrive plugin produces near-perfect wheel velocities with no slip. The simulated IMU injects Gaussian noise. FusionCore fuses both: so the noisy IMU slightly degrades a perfect odometry source. The result: slightly more drift than raw wheel odometry, and larger `map → odom` corrections when SLAM loop-closing fires.

On real hardware this inverts: encoders accumulate slip, terrain variation, and mechanical error while IMU noise is small by comparison. That is where the fusion pays off.

The map quality is unaffected: SLAM corrects the drift. The issue is visual only.

**If the IMU frame name doesn't match your URDF (common in Gazebo Harmonic TurtleBot3):**

Gazebo can publish IMU messages with an internal frame name like `waffle/imu_link/tb3_imu` instead of `imu_link`. FusionCore can't find that frame in the TF tree and logs:

```
Cannot transform IMU from waffle/imu_link/tb3_imu to base_footprint: does not exist
```

Fix: set `imu.frame_id` to the URDF frame name in your config:

```yaml
imu.frame_id: "imu_link"
```

This tells FusionCore to ignore the frame stamped on incoming IMU messages and use your URDF frame instead.

---

## WSL2: robot doesn't move, `RTPS_TRANSPORT_SHM` errors, or the filter crashes

On WSL2 the default Fast-DDS shared-memory transport often fails to lock its
port files and intermittently drops or reorders messages. Symptoms:

```
RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7004: open_and_lock_file failed
```

- **Robot never drives in the Gazebo demo:** `/cmd_vel` is being dropped before it
  reaches the diff-drive plugin.
- **`Detected jump back in time. Clearing TF buffer`** in the logs: the `/clock`
  stream is being reordered.
- **`Cholesky decomposition failed after P repair` / the node aborts:** a corrupted
  (backward) clock hands the filter a bad `dt`, which blows up the covariance.

Fix: force UDP-only transport. The package ships a profile that disables shared
memory:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix fusioncore_gazebo)/share/fusioncore_gazebo/config/fastdds_udp.xml
```

Set these before launching (in the same shell). CycloneDDS is not a reliable
fallback on WSL2 here: it tends to fail with "Failed to find a free participant
index for domain 0". If you are replaying a long bag (e.g. an NCLT sequence) and
still see clock jumps because the machine cannot keep up at 1x, lower the
playback rate (`playback_rate:=0.5` or `0.3`) to give it headroom.

---

## VSLAM pose updates not being fused

**Check 1: Is the topic set and correct?**

```bash
ros2 param get /fusioncore vslam.topic
ros2 topic hz /vslam/odometry   # replace with your actual topic
```

If `vslam.topic` returns an empty string, FusionCore is not subscribed. Set it in your YAML:

```yaml
vslam.topic: "/vslam/odometry"
```

**Check 2: Is the covariance filled?**

```bash
ros2 topic echo /vslam/odometry --field pose.covariance --once
```

If all 36 values are zero, FusionCore falls back to `vslam.position_noise` and `vslam.orientation_noise`. This is fine but means you're not using ORB-SLAM3's quality estimate.

**Check 3: Are updates being rejected as outliers?**

```bash
ros2 topic echo /diagnostics --once
```

If `vslam_outliers` is climbing, the Mahalanobis gate is rejecting measurements.

Two causes and two different fixes:

- **Normal motion, normal tracking, still rejecting:** your covariance values are too tight relative to the actual VSLAM noise. Loosen `vslam.position_noise` / `vslam.orientation_noise` (used as fallback when covariance is zero), or calibrate the covariance your wrapper publishes.
- **ORB-SLAM3 just lost tracking and reinitialized:** expected. The chi-squared gate correctly rejects the discontinuous pose jump. After `vslam.reinit_n` consecutive rejections (default 10 ≈ 2 s at 5 Hz), FusionCore automatically re-anchors to the filter's current position and resumes fusion. You will see this in the log:

  ```
  [WARN] VSLAM: 10 consecutive rejections: reinitialization detected. Re-anchoring map origin.
  ```

  If this fires too eagerly during fast motion, increase `vslam.reinit_n`. If recovery after tracking loss is too slow, decrease it.

Do **not** raise `outlier_threshold_vslam` to suppress reinitialization rejections. That defeats the protection the gate provides and lets bad pose jumps corrupt the filter state.

**Check 4: VSLAM health shows STALE**

VSLAM is marked STALE when no message arrives for more than `stale_timeout` seconds (default 1 s). Check that ORB-SLAM3 is tracking: when tracking is lost, many forks stop publishing or publish with zero covariance.

**Check 5: VSLAM fuses on startup then drifts away**

The VSLAM map frame and filter odom frame have different origins. On first message, FusionCore anchors the offset between them (logged as `[INFO] VSLAM: map origin anchored`). If you don't see this log line, the first message arrived before the filter was initialized and was dropped. Use `init.wait_for_all_sensors: true` with `init.sensor_wait_timeout: 10.0` to ensure the filter waits for VSLAM before starting.

---

## `/fusion/odom` publishing at wrong rate

`publish_rate` defaults to `100.0` Hz. The actual rate is limited by your system load and the timer precision on WSL2/Raspberry Pi.

```bash
ros2 topic hz /fusion/odom
```

If you see consistently lower than expected, lower `publish_rate` to match your hardware capability (e.g. `50.0` on a Raspberry Pi 4).
