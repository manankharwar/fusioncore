# FusionCore

**ROS 2 sensor fusion SDK. Combines IMU, wheel encoders, and GPS into one reliable position estimate. Zero manual tuning. Apache 2.0.**

---

## What problem does this solve?

Every mobile robot needs to know where it is. It gets this information from multiple sensors:
- **IMU**: measures rotation and acceleration (like a phone's motion sensor)
- **Wheel encoders**: measure how far each wheel has turned
- **GPS/GNSS**: measures position from satellites

The problem: each sensor is imperfect. IMUs drift over time. Wheels slip. GPS jumps around. You need software that intelligently combines all three into one trustworthy position estimate.

That software is called a **sensor fusion package**. The standard one for ROS: `robot_localization`: was officially deprecated in September 2023. Its designated replacement (`fuse`) still doesn't support GPS properly as of early 2026.

**FusionCore is the replacement that actually works.**

---

## Why FusionCore is better

| Capability | robot_localization | Fuse | FusionCore |
|---|---|---|---|
| **Core filter** | EKF | Factor graph | UKF |
| **3D support** | Partial | Not ready yet | Full 3D from day one |
| **IMU bias estimation** | None | Complex to configure | Automatic |
| **Noise covariance** | Manual YAML file | Manual | Automatic |
| **GPS fusion** | Workaround (UTM) | Not implemented | Proper (ECEF) |
| **Dual antenna GPS heading** | Hack required | Not supported | Native support |
| **Outlier rejection** | None | Partial | Coming next release |
| **Maintenance** | Abandoned | Slow | Actively maintained |
| **License** | BSD-3 | BSD-3 | Apache 2.0 |
| **ROS 2 Jazzy** | Ported (imperfect) | Native | Native |
| **Working examples** | Minimal | None | Real robot configs |
| **Config complexity** | High: many YAML parameters | Very high | Low: one simple YAML |
| **100Hz real-time** | Yes | Difficult | Yes: tested |

### What those terms mean

**Core filter (EKF vs UKF):**
Both are mathematical algorithms for combining sensor data. EKF (Extended Kalman Filter) makes approximations that break down on aggressive robot motion. UKF (Unscented Kalman Filter) handles it more accurately without those approximations. Think of EKF as drawing a straight line through a curve: UKF actually follows the curve.

**3D support:**
robot_localization was originally built for 2D robots (ground robots that don't tilt). Full 3D means it correctly handles drones, outdoor robots on uneven terrain, or any robot that rolls and pitches.

**IMU bias estimation:**
Every IMU (gyroscope/accelerometer) has a slowly drifting error called bias. If you don't correct for it, your position estimate drifts over time. FusionCore estimates and corrects this bias automatically and continuously. robot_localization ignores it entirely.

**Noise covariance:**
A number that tells the filter "how much do I trust this sensor?" robot_localization requires you to manually set these numbers in a YAML file: most developers guess, and wrong values cause the filter to behave badly. FusionCore estimates them automatically from the data.

**GPS fusion (UTM vs ECEF):**
GPS gives you latitude/longitude. The filter needs meters. Converting between them requires a coordinate system. UTM (what robot_localization uses) has boundaries between zones and distortion at edges: causes jumps when crossing zone lines. ECEF (what FusionCore uses) is a single global coordinate system with no discontinuities. Better math, more reliable.

**Dual antenna GPS heading:**
One GPS antenna gives position. Two antennas separated by a known distance give you heading (which direction the robot is facing) by measuring the angle between them. robot_localization required a workaround (treating heading as IMU data). FusionCore supports it natively and correctly.

**Maintenance (Deprecated vs Active):**
Deprecated means the developers have stopped working on it and recommend you use something else. robot_localization is officially deprecated. FusionCore is actively maintained: issues are answered within 24 hours.

**License (BSD-3 vs Apache 2.0):**
Both are open source. Apache 2.0 includes an explicit patent license grant that BSD-3 does not. For commercial robotics companies, Apache 2.0 is the safer choice from a legal standpoint.

---

## Installation

### Prerequisites
- ROS 2 Jazzy Jalisco installed
- A colcon workspace (usually `~/ros2_ws`)

### Step 1: Clone FusionCore into your workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
```

**Why:** ROS 2 uses a tool called colcon to build packages. Colcon looks for packages inside the `src/` folder of your workspace. Cloning here makes FusionCore visible to colcon.

### Step 2: Install dependencies
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**Why:** FusionCore depends on ROS 2 packages like `rclcpp`, `sensor_msgs`, `tf2`. This command automatically finds and installs anything missing.

### Step 3: Build
```bash
colcon build --packages-select fusioncore_core fusioncore_ros
```

**Why:** This compiles the C++ code into a binary your robot can run. `fusioncore_core` is the pure math library (no ROS). `fusioncore_ros` is the ROS 2 node that connects it to your robot's sensors.

### Step 4: Source the workspace
```bash
source install/setup.bash
```

**Why:** This tells your terminal where to find the newly built FusionCore executables. You need to run this in every new terminal that uses FusionCore. Add it to your `~/.bashrc` to make it permanent.

---

## Running FusionCore

### Terminal 1: Start the node
```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  config:=config/fusioncore.yaml
```

**Why launch instead of run:** The launch file handles startup configuration automatically. You should see:
```
FusionCore node created
Configuring FusionCore...
FusionCore configured. base_frame=base_link odom_frame=odom rate=100Hz
```

### Terminal 2: Activate the node
```bash
ros2 lifecycle set /fusioncore configure
ros2 lifecycle set /fusioncore activate
```

**Why two steps:** FusionCore uses a ROS 2 lifecycle node. This is a design pattern that lets you configure the node first (load parameters, validate config) and then activate it (start processing sensor data) separately. This prevents the filter from starting mid-motion with bad initial values.

After activation you should see:
```
Activating FusionCore...
FusionCore active. Listening for sensors.
```

### Verify it is working
```bash
ros2 topic hz /fusion/odom
```

Expected output:
```
average rate: 100.000
```

---

## Sensor topics

FusionCore subscribes to these topics by default:

| Topic | Message type | What it is |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | IMU: angular velocity and linear acceleration |
| `/odom/wheels` | `nav_msgs/Odometry` | Wheel encoder velocity |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | GPS position (latitude, longitude, altitude) |

FusionCore publishes:

| Topic | Message type | What it is |
|---|---|---|
| `/fusion/odom` | `nav_msgs/Odometry` | Fused position + orientation + velocity at 100Hz |
| `/tf` | TF transform | `odom -> base_link` transform for Nav2 |

---

## Configuration

No covariance matrices. One YAML file with plain English parameters:
```yaml
base_frame: base_link    # your robot's body frame
odom_frame: odom         # the fixed reference frame
publish_rate: 100.0      # Hz

imu:
  gyro_noise: 0.005      # how noisy your gyroscope is (rad/s)
  accel_noise: 0.1       # how noisy your accelerometer is (m/s²)

encoder:
  vel_noise: 0.05        # how noisy your wheel velocity is (m/s)
  yaw_noise: 0.02        # how noisy your yaw rate is (rad/s)

gnss:
  base_noise_xy: 1.0     # base GPS horizontal noise (meters)
  max_hdop: 4.0          # reject GPS fixes worse than this quality
  min_satellites: 4      # reject GPS fixes with fewer satellites

ukf:
  q_gyro_bias: 1.0e-5    # how fast gyro bias can change (smaller = slower)
  q_accel_bias: 1.0e-5   # how fast accel bias can change
```

**HDOP explained:** Horizontal Dilution of Precision. A number from GPS receivers indicating fix quality. Lower is better. 1.0 = excellent. 4.0 = acceptable. Above 4.0 = FusionCore rejects the fix and relies on dead reckoning until quality improves.

---

## Example configurations

### Stefan's config: Husarion Panther outdoor robot
GPS + IMU + wheel encoders, dual antenna heading:
```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  config:=config/fusioncore.yaml
```

### Basic indoor robot: no GPS
IMU + wheel encoders only:
```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  config:=config/fusioncore.yaml
```

---

## Architecture
```
fusioncore/
├── fusioncore_core/          # Pure C++17 math library: zero ROS dependency
│   ├── include/fusioncore/
│   │   ├── ukf.hpp           # Unscented Kalman Filter: the core algorithm
│   │   ├── state.hpp         # 21-dimensional state vector definition
│   │   ├── fusioncore.hpp    # Main public API
│   │   └── sensors/
│   │       ├── imu.hpp       # IMU measurement model + bias estimation
│   │       ├── encoder.hpp   # Wheel encoder measurement model
│   │       └── gnss.hpp      # GPS: ECEF conversion, quality scaling
│   └── src/
│       ├── ukf.cpp           # UKF implementation
│       └── fusioncore.cpp    # Manager: wires all sensors to UKF
└── fusioncore_ros/           # ROS 2 Jazzy wrapper
    ├── src/fusion_node.cpp   # Lifecycle node: subscribes and publishes
    ├── config/
    │   ├── outdoor_gnss_dual_antenna.yaml   # Stefan's config
    │   └── differential_drive_basic.yaml    # Basic indoor config
    └── launch/
        └── fusioncore.launch.py
```

**Why fusioncore_core has no ROS dependency:** The pure C++ core can run as firmware on embedded hardware (Phase 2 hardware module) without any ROS installation. This is a deliberate architectural decision.

---

## Technical details

- **Filter:** Unscented Kalman Filter with 43 sigma points
- **State vector:** 21-dimensional: position (x,y,z), orientation (roll,pitch,yaw), linear velocity, angular velocity, linear acceleration, gyroscope bias (x,y,z), accelerometer bias (x,y,z)
- **GPS coordinate system:** ECEF: globally valid, no zone boundaries
- **Bias estimation:** Continuous online estimation: no calibration ritual required
- **GPS quality:** Noise covariance scaled automatically by HDOP and VDOP values from the receiver
- **Output rate:** 100Hz
- **Language:** C++17

---

## Status and roadmap

**Working now:**
- UKF core with full 3D state
- IMU + encoder + GPS fusion
- Automatic IMU bias estimation
- ECEF GPS conversion
- Quality-aware GPS noise scaling
- Dual antenna heading
- ROS 2 Jazzy lifecycle node
- 100Hz output

**Coming next:**
- Mahalanobis distance outlier rejection (GPS jumps, encoder slip detection)
- Adaptive noise covariance (fully automatic, no YAML needed at all)
- TF validation with clear error messages
- Omnidirectional and Ackermann steering motion models

---

## License

Apache 2.0: commercially safe. Includes explicit patent license grant that BSD-3 does not provide.

---

## Support

Issues answered within 24 hours. Open a GitHub issue or find the original discussion on ROS Discourse.

This project exists because of community threads asking for a robot_localization replacement that actually works. If you hit a problem: report it. That feedback is what makes this better.
