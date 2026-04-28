# Simulation

FusionCore ships with a Gazebo Harmonic simulation world so you can test the full fusion pipeline without physical hardware. It includes a differential drive robot with a 100 Hz IMU and GPS, in an outdoor environment with the GPS origin set to Hamilton, Ontario.

!!! note "Gazebo NavSat bug"
    Gazebo Harmonic's built-in NavSat sensor has a known bug (gz-sim issue #2163) where it periodically outputs GPS fixes at completely wrong coordinates. The simulation works around this by deriving GPS from Gazebo's ground truth world pose and adding realistic Gaussian noise (0.5 m horizontal, 0.3 m vertical 1-sigma).

---

## Prerequisites

Gazebo Harmonic and the ROS-Gazebo bridge are not installed by `rosdep` automatically. Install them first:

```bash
sudo apt install ros-jazzy-ros-gz
```

---

## Running the simulation

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
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

Four tests run automatically:

1. IMU drift rate while stationary
2. Outlier rejection (injects a 500 m GPS spike, verifies position stays stable)
3. GPS correction after IMU drift
4. Full circle return error

All four pass on a clean session.
