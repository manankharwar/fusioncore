# How to test FusionCore yourself

Three paths depending on how much time you have and what software is installed.

---

## Path A -- 30 seconds, no ROS required

Shows FusionCore vs robot_localization EKF vs RTK GPS ground truth from the NCLT benchmark dataset. Pre-baked results are included in the repository.

**Requirements:** Python 3, numpy, matplotlib

```bash
git clone https://github.com/manankharwar/fusioncore && cd fusioncore
pip install numpy matplotlib
python3 tools/demo_quick.py --open
```

Output: `demo_result.png` -- a side-by-side trajectory comparison.

```
FusionCore demo -- NCLT 2012-01-08 (pre-baked results)
  FusionCore  ATE RMSE:  5.55 m
  RL-EKF      ATE RMSE: 13.01 m
  FusionCore is 2.3x more accurate
  Saved: demo_result.png
```

This is the dataset result. To run FusionCore live on real sensor data, use Path B.

---

## Path B -- 5 minutes, ROS 2 required

Replays 120 seconds of real outdoor robot sensor data through FusionCore running live. Downloads a ~5 MB demo bag from GitHub Releases.

**Requirements:** ROS 2 Jazzy or Humble, FusionCore built

```bash
# Step 1: build (once)
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build && source install/setup.bash

# Step 2: run demo (downloads bag automatically, runs FusionCore live)
cd ~/ros2_ws/src/fusioncore
bash demo/run_demo.sh
```

That is the complete command. The script:
1. Downloads `nclt_demo_120s.mcap` (~5 MB) from GitHub Releases
2. Starts FusionCore as a lifecycle node
3. Replays the bag (120 seconds of IMU + wheel odometry + GPS at 2x speed)
4. Records FusionCore's `/fusion/odom` output
5. Generates a comparison plot against pre-baked robot_localization and RTK ground truth

Total wall-clock time: about 4 minutes.

### What the demo bag contains

120 seconds from the [NCLT 2012-01-08 sequence](http://robots.engin.umich.edu/nclt/) -- a Segway RMP400 driving a campus route at the University of Michigan. Topics:

| Topic | Rate | Description |
|---|---|---|
| `/imu/data` | 100 Hz | Microstrain 3DM-GX3-45 IMU (gyro + accel) |
| `/odom/wheels` | 100 Hz | Wheel encoder velocity |
| `/gnss/fix` | ~5 Hz | Standard GPS (not RTK) |

FusionCore config: `fusioncore_datasets/config/nclt_fusioncore.yaml` -- default settings, no tuning.

### Manual step-by-step (if you prefer)

```bash
# Download the bag
wget -O demo/nclt_demo_120s.mcap \
  https://github.com/manankharwar/fusioncore/releases/download/demo-data/nclt_demo_120s.mcap

# Terminal 1: run FusionCore + replay
ros2 launch fusioncore demo/nclt_demo.launch.py

# Wait for bag to finish, then Ctrl+C, then:
# Terminal 2: convert output to TUM and plot
python3 tools/odom_to_tum.py \
  --bag /tmp/fc_demo_out --topic /fusion/odom --out /tmp/fc_live.tum

python3 tools/demo_quick.py --live_tum /tmp/fc_live.tum --open
```

---

## Path C -- your own robot bag

If you have a rosbag from your own robot with `/imu/data`, `/odom/wheels`, and optionally `/gnss/fix`:

```bash
# Configure FusionCore for your robot
cp fusioncore_ros/config/example.yaml my_robot.yaml
# Edit my_robot.yaml: set imu.gyro_noise, imu.accel_noise from your IMU datasheet

# Run FusionCore against your bag
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=my_robot.yaml

# In another terminal: replay your bag
ros2 bag play your_robot_bag.mcap --clock

# Record the output
ros2 bag record /fusion/odom /fusion/pose -o my_robot_result
```

See [Getting Started](https://manankharwar.github.io/fusioncore/getting-started/) and [Configuration](https://manankharwar.github.io/fusioncore/configuration/) for full details.

---

## Create your own demo bag from raw NCLT data

If you have downloaded the full NCLT dataset and want to create the demo bag yourself (or create a longer clip):

```bash
pip install rosbags
python3 demo/make_demo_bag.py \
  --data_dir /path/to/nclt/2012-01-08 \
  --out demo/nclt_demo_120s.mcap \
  --duration 120
```

---

## Frequently asked questions

**Does it work with imperfect IMU calibration?**
Yes. `adaptive.imu: true` (default) adjusts the measurement noise matrix automatically based on the innovation sequence. The `init.stationary_window` parameter estimates accelerometer bias before motion starts. In the demo, FusionCore is given only the IMU datasheet noise values -- no manual tuning.

**How much manual tuning is needed?**
Two numbers from your IMU datasheet: `imu.gyro_noise` (ARW spec) and `imu.accel_noise` (VRW spec). Everything else can start at default values. Adaptive noise handles the rest within the first minute of operation.

**What about timestamp jitter and delayed GPS?**
FusionCore stores a rolling IMU buffer and replays intermediate updates when a delayed GPS fix arrives (delay compensation via retrodiction, up to 500 ms). Timestamp jitter is handled by clamping `dt` to `[min_dt, max_dt]`.

**What is the CPU cost?**
A 22-state UKF at 100 Hz generates 45 sigma points per predict step. On a laptop Intel i7: under 0.2 ms per cycle. On Raspberry Pi 4: under 1 ms per cycle. FusionCore uses Eigen for all matrix operations; Eigen auto-detects NEON on ARM.

**Does it behave the same on ARM (Raspberry Pi, Jetson)?**
Yes. All matrix operations use Eigen which compiles to NEON on ARM. The NCLT benchmark result is reproducible on ARM within floating-point rounding tolerance.
