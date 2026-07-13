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

The fastest way to diagnose rejections is to look at the structured debug topic instead of parsing log lines:

```bash
ros2 topic echo /fusion/debug/gnss_status
```

Every GPS fix produces one message here, whether it was accepted or not. The `rejection_reason` field tells you exactly which gate fired. The `mahalanobis_sq` field tells you how far the fix was from the filter's prediction (-1.0 means the quality gate failed before the math ran).

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

| Symptom | Cause | Fix |
|---|---|---|
| `rejection_reason: HDOP_HIGH` at startup | Open sky not acquired yet | Normal: clears within 30–60 s once receiver locks |
| `rejection_reason: CHI2_FAILED` after outage | Filter drifted during blackout, returning GPS fails gate | Coast mode relaxes the gate automatically; no action needed |
| `rejection_reason: CHI2_FAILED` persistently | Fix is far from what filter predicts | Check for antenna obstruction or TF mismatch |
| `encoder_outlier_count` climbing | Noise config too tight vs actual velocity variance | Loosen `encoder.vel_noise` or enable `adaptive.encoder: true` |
| `imu_outlier_count` climbing | Driver publishing wrong scale or units | Check `linear_acceleration.z` at rest: should be ~9.81 or ~0.0 depending on `imu.remove_gravitational_acceleration` |

Do **not** lower outlier thresholds below their chi-squared critical values. At `7.0` normal GPS noise trips the gate and every fix gets rejected. The defaults are statistically calibrated.

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
