# Getting Started

## Prerequisites

- ROS 2 Jazzy Jalisco (Ubuntu 24.04) or Humble Hawksbill (Ubuntu 22.04)
- A colcon workspace (`~/ros2_ws`)

---

## Install

FusionCore is a monorepo with 4 packages: `compass_msgs`, `fusioncore_core`, `fusioncore_ros`, and `fusioncore_gazebo`. The repo must live inside `src/` for colcon to find them.

!!! note "Humble users"
    Replace `jazzy` with `humble` in all commands below.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

!!! tip "Headless machines (Raspberry Pi, servers)"
    `fusioncore_gazebo` depends on `ros_gz_sim` which pulls in Gazebo and GUI components. On headless machines this may fail or install hundreds of MB you don't need. Skip it:
    ```bash
    touch ~/ros2_ws/src/fusioncore/fusioncore_gazebo/COLCON_IGNORE
    ```
    `fusioncore_core` and `fusioncore_ros` have no Gazebo dependency and build fine without it.

---

## Run the tests

```bash
cd ~/ros2_ws
colcon build --packages-select fusioncore_core --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fusioncore_core
colcon test-result --verbose
```

Expected: `39 tests, 0 errors, 0 failures, 0 skipped`

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

## Test every feature without hardware

You can verify all FusionCore features using fake sensor data. Open 4 terminals:

**Terminal 1: launch:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch fusioncore_ros fusioncore.launch.py
```

**Terminal 2: configure and activate:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id imu_link &
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link &
sleep 1

ros2 lifecycle set /fusioncore configure
sleep 1
ros2 lifecycle set /fusioncore activate
```

**Terminal 3: feed fake sensors:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash

# IMU at 100Hz (stationary, gravity up)
ros2 topic pub /imu/data sensor_msgs/msg/Imu "{
  header: {frame_id: 'base_link'},
  angular_velocity: {x: 0.0, y: 0.0, z: 0.0},
  linear_acceleration: {x: 0.0, y: 0.0, z: 9.81},
  orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}" --rate 100 &

# Wheel encoder at 50Hz (stationary)
ros2 topic pub /odom/wheels nav_msgs/msg/Odometry "{
  header: {frame_id: 'odom'},
  twist: {twist: {linear: {x: 0.0}, angular: {z: 0.0}}}
}" --rate 50 &

# GPS at 5Hz (Hamilton, Ontario)
ros2 topic pub /gnss/fix sensor_msgs/msg/NavSatFix "{
  header: {frame_id: 'base_link'},
  status: {status: 0},
  latitude: 43.2557,
  longitude: -79.8711,
  altitude: 100.0,
  position_covariance: [1.0, 0, 0, 0, 1.0, 0, 0, 0, 4.0],
  position_covariance_type: 2
}" --rate 5
```

**Terminal 4: verify:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash

# Check topics are live
ros2 topic list | grep fusion

# Odometry at 100Hz
ros2 topic hz /fusion/odom

# Per-sensor diagnostics
ros2 topic echo /diagnostics --once

# Velocity near zero while stationary (ZUPT working)
ros2 topic echo /fusion/odom --field twist.twist.linear

# Reset service
ros2 service call /fusioncore/reset std_srvs/srv/Trigger
```

You should see `/fusion/odom`, `/fusion/pose`, and `/fusioncore/reset` in the topic/service list.
