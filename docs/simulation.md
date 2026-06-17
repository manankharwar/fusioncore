# Simulation

FusionCore ships with a Gazebo Harmonic simulation world so you can test the full fusion pipeline without physical hardware. It includes a differential drive robot with a 100 Hz IMU and GPS in an agricultural outdoor environment with the GPS origin set to Hamilton, Ontario.

!!! note "Gazebo NavSat bug"
    Gazebo Harmonic's built-in NavSat sensor has a known bug (gz-sim issue #2163) that periodically outputs GPS fixes at completely wrong coordinates. The simulation works around this by deriving GPS from Gazebo's ground truth world pose and adding realistic Gaussian noise (0.5 m horizontal, 0.3 m vertical 1-sigma).

---

## Prerequisites

Gazebo Harmonic and the ROS-Gazebo bridge are not installed by `rosdep` automatically:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-robot-localization
```

---

## Demo: FusionCore vs robot_localization GPS spike

The demo launch runs both FusionCore and robot_localization EKF simultaneously on the same sensor streams. At t=30 s, a 50-meter GPS spike is injected. FusionCore's chi-squared gate rejects it and holds course. robot_localization has no rejection threshold configured, accepts the spike, and diverges.

```bash
ros2 launch fusioncore_gazebo fusioncore_demo.launch.py
```

RViz opens automatically showing three trajectories:

| Color | Source |
|---|---|
| Green | FusionCore (`/fusion/path`) |
| Red | robot_localization EKF (`/rl/path`) |
| Yellow | Raw GPS position (`/gps/path`) |

**Timeline:**

| Time | Event |
|---|---|
| t=0 s | Gazebo starts, GPS and IMU publishing |
| t=15 s | FusionCore configures and activates |
| t=18 s | Robot starts driving in a circle (0.8 m/s, 12 m radius) |
| t=30 s | GPS spike: +50 m East injected for 6 seconds |
| t=36 s | Spike ends, GPS returns to normal |

During the spike, the yellow GPS track jumps 50 m east. The red RL trajectory follows it. The green FusionCore trajectory continues the circle undisturbed.

**Spike parameters are adjustable:**

```bash
ros2 launch fusioncore_gazebo fusioncore_demo.launch.py \
  spike_at_s:=45.0 \
  spike_duration_s:=10.0 \
  spike_dx_m:=100.0
```

**To run headless (no RViz):**

```bash
ros2 launch fusioncore_gazebo fusioncore_demo.launch.py rviz:=false
```

---

## Integration test world

The integration test world (`fusioncore_test.sdf`) is a minimal flat environment for automated pass/fail testing without visual distractions:

```bash
ros2 launch fusioncore_gazebo fusioncore_gazebo.launch.py
```

Drive the robot and watch the fused position:

```bash
# Terminal 2: drive in a circle
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}" --rate 10

# Terminal 3: watch position
ros2 topic echo /fusion/odom --field pose.pose.position
```

---

## Automated integration tests

```bash
python3 ~/ros2_ws/src/fusioncore/fusioncore_gazebo/launch/integration_test.py
```

Four tests run automatically against the live simulation. Each has a hard pass/fail threshold:

| Test | What it does | Pass threshold |
|---|---|---|
| **IMU dead reckoning** | Robot stationary for 10 s with GPS active. Measures position drift from IMU noise alone. | Drift < 2.0 m |
| **GPS spike rejection** | Publishes one corrupted GPS fix at +500 m north. Measures how far the filter moves in response. | Position jump < 5.0 m |
| **GPS correction** | Robot drives forward 3 s then stops. Measures position stability 3 s after stopping with GPS active. | Drift < 2.0 m |
| **Circle return** | Robot drives one full circle (radius ~0.5 m). Measures distance between start and end position. | Return error < 3.0 m |

Expected output on a clean session:

```
══════════════════════════════════════════════════
  FUSIONCORE INTEGRATION TEST SCORECARD
══════════════════════════════════════════════════
  [PASS] ✓ IMU dead reckoning
           0.041m drift in 10s stationary
  [PASS] ✓ Outlier rejection
           0.312m jump on 500m GPS outlier
  [PASS] ✓ GPS correction
           0.018m drift after stop with GPS active
  [PASS] ✓ Circle return
           0.247m from start after full circle
══════════════════════════════════════════════════
  Overall: ALL TESTS PASSED ✓
══════════════════════════════════════════════════
```

The spike rejection test (0.3 m movement on a 500 m corrupted fix) directly verifies the chi-squared gate described in [How It Works](../how-it-works.md#mahalanobis-outlier-rejection).

---

## How the GPS simulation works

Both the demo and integration test worlds derive GPS from Gazebo's internal ground truth pose rather than using Gazebo's NavSat sensor (which has the gz-sim #2163 coordinate corruption bug).

The `gz_pose_to_gps` node subscribes to the Gazebo pose broadcast, adds Gaussian noise, and publishes:

- `/gnss/fix` (`sensor_msgs/NavSatFix`) for FusionCore
- `/gps/odometry` (`nav_msgs/Odometry`, ENU frame) for robot_localization, so no `navsat_transform_node` is needed

Spike injection is done in the same node by adding an offset to both outputs simultaneously, so FusionCore and robot_localization see the same corrupted measurement at the same time.
