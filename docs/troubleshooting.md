# Troubleshooting

Quick answers to the most common problems. Each section states what you see, why it happens, and the exact fix.

---

## `/fusion/odom` is not publishing

**Check 1: Is the node active?**

```bash
ros2 lifecycle get /fusioncore
```

If it says `unconfigured` or `inactive`, the lifecycle transitions didn't fire. Run them manually:

```bash
ros2 lifecycle set /fusioncore configure
ros2 lifecycle set /fusioncore activate
```

Or use the provided launch file which wires this automatically:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
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

If transforms are printing, the error is a race condition at startup: the downstream node started before FusionCore. The `fusioncore_nav2.launch.py` launch file adds a 5-second delay before starting Nav2 to prevent this.

---

## Encoder or GPS getting rejected (outlier gate)

Check diagnostics:

```bash
ros2 topic echo /diagnostics --once
```

If you see high outlier counts for a sensor, the Mahalanobis gate is rejecting its measurements. Common causes:

| Symptom | Cause | Fix |
|---|---|---|
| GPS rejections at startup | Filter hasn't converged yet: large position uncertainty | Normal: clears after 30–60 s |
| GPS rejections after outage | Inertial drift exceeded gate | Inertial coast mode will relax the gate automatically |
| Encoder always rejected | Noise config too tight vs actual speed variance | Loosen `encoder.vel_noise` or enable `adaptive.encoder: true` |
| IMU always rejected | Driver publishing with wrong scale or units | Check `linear_acceleration.z` at rest: should be ~9.81 or ~0.0 depending on `imu.remove_gravitational_acceleration` |

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

## `/fusion/odom` publishing at wrong rate

`publish_rate` defaults to `100.0` Hz. The actual rate is limited by your system load and the timer precision on WSL2/Raspberry Pi.

```bash
ros2 topic hz /fusion/odom
```

If you see consistently lower than expected, lower `publish_rate` to match your hardware capability (e.g. `50.0` on a Raspberry Pi 4).
