# Getting Started

## Prerequisites

- ROS 2 Jazzy Jalisco (Ubuntu 24.04) or Humble Hawksbill (Ubuntu 22.04)
- A colcon workspace (`~/ros2_ws`)

---

## Install

FusionCore is a monorepo with these packages:

| Package | Purpose | Required? |
|---|---|---|
| `compass_msgs` | Custom heading message type | Yes |
| `fusioncore_core` | Pure C++ UKF library, no ROS | Yes |
| `fusioncore_ros` | ROS 2 lifecycle node | Yes |
| `fusioncore_gazebo` | Simulation world + demo launch | Optional |
| `fusioncore_ublox` | u-blox NavPVT Doppler bridge | Optional (only builds with `ublox_msgs` installed) |

The repo must live inside `src/` for colcon to find them.

!!! note "Humble users"
    Replace `jazzy` with `humble` in all commands below.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to fusioncore_ros
source install/setup.bash
```

!!! warning "Run colcon from the workspace root, not from inside the repo"
    Always run `colcon build` from `~/ros2_ws`, not from `~/ros2_ws/src/fusioncore`.
    colcon discovers packages by scanning `src/` from the workspace root. Running it
    from inside the repo produces a nested `install/` that conflicts with the workspace.

!!! tip "Headless machines (Raspberry Pi, servers)"
    `--packages-up-to fusioncore_ros` already skips Gazebo and the ublox bridge. To be explicit:
    ```bash
    touch ~/ros2_ws/src/fusioncore/fusioncore_gazebo/COLCON_IGNORE
    touch ~/ros2_ws/src/fusioncore/fusioncore_ublox/COLCON_IGNORE
    ```

---

## Run the tests

```bash
cd ~/ros2_ws
colcon build --packages-select fusioncore_core --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fusioncore_core
colcon test-result --verbose
```

Expected: `64 tests, 0 errors, 0 failures, 0 skipped`

---

## First launch

FusionCore is a lifecycle node. It needs to be configured (load params, validate TF) and then activated (start processing) before it does anything.

The provided launch files handle this automatically:

```bash
# With Nav2
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml

# Without Nav2
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

Or manually in a second terminal:

```bash
ros2 lifecycle set /fusioncore configure
ros2 lifecycle set /fusioncore activate
```

Verify it's publishing:

```bash
ros2 topic hz /fusion/odom
# expected: average rate: 100.000
```

!!! note "WSL2"
    If `ros2 lifecycle set` returns "Node not found", use the launch file's auto-configure instead. WSL2 DDS discovery latency can prevent manual lifecycle commands from finding the node. The launch file handles this with timed events.

---

## Verify it works (single command)

```bash
bash tools/quick_test.sh
```

Starts FusionCore with fake sensors, runs through configure → activate, checks all outputs. Takes about 15 seconds. Prints `[PASS]` / `[FAIL]` for each check.

Expected output:

```
  FusionCore Quick Test
  =====================

  [....] Sourcing ROS environment...
  [PASS] ROS environment sourced
  [....] Starting TF publishers...
  [....] Launching FusionCore...
  [PASS] Lifecycle: configure → activate
  [....] Publishing fake IMU at 100 Hz (stationary, gravity pointing up)...
  [....] Publishing fake wheel odometry at 50 Hz (stationary)...
  [....] Waiting 6 s for filter to initialize...

  Checks:
  -------
  [PASS] /fusion/odom publishing (main output)
  [PASS] /fusion/pose publishing
  [PASS] /diagnostics publishing
  [PASS] /fusioncore/reset service responds

  All checks passed. FusionCore is working correctly.
```

If a check fails, the script prints the exact diagnostic command to run next.

!!! note "Docker"
    The script also runs inside the container:
    ```bash
    docker run --rm -it ghcr.io/manankharwar/fusioncore:latest bash tools/quick_test.sh
    ```

---

## Manual verification (if you want to inspect each output)

If you want to observe the filter behavior directly rather than run the automated check:

```bash
# Terminal 1: launch
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch fusioncore_ros fusioncore.launch.py

# Terminal 2: TF + lifecycle
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id imu_link &
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link &
sleep 1 && ros2 lifecycle set /fusioncore configure
sleep 1 && ros2 lifecycle set /fusioncore activate

# Terminal 3: diagnostics
ros2 topic echo /diagnostics --once
ros2 topic hz /fusion/odom
ros2 topic echo /fusion/odom --field twist.twist.linear
```

You should see `/fusion/odom`, `/fusion/pose`, and `/fusioncore/reset` in the topic/service list.
