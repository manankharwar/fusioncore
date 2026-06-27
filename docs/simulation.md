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

## Demo: FusionCore vs robot_localization under GPS spikes and an outage

The demo launch runs FusionCore and a robot_localization EKF side by side on the
same sensor streams while the robot drives a lawnmower survey pattern. Three GPS
disruptions are injected: two 60 m spikes and a 25 s full outage. FusionCore's
chi-squared gate plus its gap-gated coast logic reject the spikes and hold
course; robot_localization has no rejection threshold and follows every spike.

```bash
ros2 launch fusioncore_gazebo fusioncore_demo.launch.py
```

RViz opens automatically showing four trajectories:

| Color | Source |
|---|---|
| Green | FusionCore (`/fusion/path`) |
| Red | robot_localization EKF (`/rl/path`) |
| Yellow | Raw GPS position, including the spikes (`/gps/path`) |
| (text overlay) | Current GPS state: `GPS OK` / `GPS SPIKE` / `GPS OUTAGE` |

**Timeline** (times are relative to the first GPS fix):

| Time | Event |
|---|---|
| t=0 s | Gazebo starts, GPS and IMU publishing |
| t=18 s | Robot starts the lawnmower pattern (adjust with `start_delay`) |
| t=50 s | Spike 1: +60 m East for 8 s. FC rejects it; RL jumps to it. |
| t=82 s | GPS outage: no fixes for 25 s. Both dead-reckon. |
| t=110 s | Spike 2: -60 m East for 6 s. FC rejects it; RL jumps to it. |
| t=131 s | Robot stops. |

On the spikes, the green FusionCore line stays on course while the red
robot_localization line lurches out to the spike and crawls back. During the
outage both dead-reckon: FusionCore stays much closer to truth than before the
gap-gated-coast fix, but a long full outage still produces some drift (this is
the open dead-reckoning limitation, see [Known Limitations](known-limitations.md)).

**Adjustable parameters** (defaults shown):

```bash
ros2 launch fusioncore_gazebo fusioncore_demo.launch.py \
  start_delay:=18.0 \        # sim seconds before the robot starts driving
  spike_at_s:=50.0 spike_dx_m:=60.0 \
  outage_at_s:=82.0 outage_duration_s:=25.0 \
  spike2_at_s:=110.0 spike2_dx_m:=-60.0
```

Set `start_delay:=2.0` to see motion almost immediately when watching live.

**Run headless (no GUI, for CI or offscreen rendering):**

```bash
ros2 launch fusioncore_gazebo fusioncore_demo.launch.py headless:=true rviz:=false
```

!!! warning "WSL2: the robot does not move / `RTPS_TRANSPORT_SHM` errors"
    On WSL2 the default Fast-DDS shared-memory transport fails to lock its port
    files and intermittently drops messages (including `/cmd_vel`, so the robot
    never drives) and can corrupt the `/clock`. Force UDP-only transport before
    launching:
    ```bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix fusioncore_gazebo)/share/fusioncore_gazebo/config/fastdds_udp.xml
    ```
    The profile ships with the package. The GUI also runs well below real time
    under WSLg software rendering; let the scenario play out.

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
