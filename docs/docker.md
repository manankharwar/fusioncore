# Docker

Run FusionCore without installing ROS 2 on your host machine. The Docker image is self-contained: ROS 2 Jazzy, all dependencies, and FusionCore are pre-built inside the image.

---

## Prerequisites

- [Docker Engine](https://docs.docker.com/engine/install/) 20.10+ (or Docker Desktop on macOS / Windows)

!!! tip "No ROS install needed"
    Everything runs inside the container. You do **not** need to install ROS 2, colcon, or rosdep on your host.

---

## Pull the image

The image is hosted on GitHub Container Registry (GHCR):

```bash
docker pull ghcr.io/manankharwar/fusioncore:latest
```

To pin a specific version (see [Releases](https://github.com/manankharwar/fusioncore/releases) for available tags):

```bash
docker pull ghcr.io/manankharwar/fusioncore:0.3.3
```

!!! note "Available tags"
    - `latest` — built from the latest commit on `main`
    - `0.3.3`, `0.3.2`, `0.3.1`, … — semantic version tags matching GitHub Releases
    - `<commit-sha>` — one tag per commit that triggered the CI build

---

## Quick start (no hardware)

Verify the image works by running the built-in quick test:

```bash
docker run --rm ghcr.io/manankharwar/fusioncore:latest bash tools/quick_test.sh
```

This starts FusionCore with fake sensors and checks all outputs in about 15 seconds. You should see `[PASS]` for every check.

To get an interactive shell inside the container:

```bash
docker run --rm -it ghcr.io/manankharwar/fusioncore:latest bash
```

---

## Run with your own YAML config

FusionCore reads its parameters from a YAML configuration file. Inside the container the default config lives at:

```
/fusioncore_ws/install/fusioncore_ros/share/fusioncore_ros/config/fusioncore.yaml
```

To use your own config, **mount it into the container** with the `-v` flag:

```bash
docker run --rm -it \
  -v /path/to/your_robot.yaml:/config/your_robot.yaml:ro \
  ghcr.io/manankharwar/fusioncore:latest \
  ros2 launch fusioncore_ros fusioncore.launch.py \
    fusioncore_config:=/config/your_robot.yaml
```

### How it works

| Part | Meaning |
|---|---|
| `-v /path/to/your_robot.yaml:/config/your_robot.yaml:ro` | Bind-mounts your host file into the container as read-only (`:ro`) |
| `fusioncore_config:=/config/your_robot.yaml` | Tells the launch file to load parameters from the mounted file instead of the built-in default |

!!! tip "Multiple config files"
    You can mount a second YAML (environment preset) alongside the main config:
    ```bash
    docker run --rm -it \
      -v ~/my_robot.yaml:/config/robot.yaml:ro \
      -v ~/env_urban.yaml:/config/env.yaml:ro \
      ghcr.io/manankharwar/fusioncore:latest \
      ros2 launch fusioncore_ros fusioncore.launch.py \
        fusioncore_config:=/config/robot.yaml \
        env_config:=/config/env.yaml
    ```

!!! warning "File path on Windows / macOS"
    On Windows, use the full Windows path with forward slashes:
    ```
    -v C:/Users/you/robot.yaml:/config/robot.yaml:ro
    ```
    On macOS with Docker Desktop, `~` expands correctly:
    ```
    -v ~/robot.yaml:/config/robot.yaml:ro
    ```

---

## Topic remapping

Your robot's sensor topics may not match FusionCore's defaults. The defaults are:

| Topic | Default | Message type |
|---|---|---|
| IMU | `/imu/data` | `sensor_msgs/Imu` |
| GNSS | `/gnss/fix` | `sensor_msgs/NavSatFix` |
| Encoder (wheel odometry) | `/odom/wheels` | `nav_msgs/Odometry` |

Use ROS 2's `--ros-args -r` flag to remap topics at launch time:

```bash
docker run --rm -it \
  -v ~/my_robot.yaml:/config/robot.yaml:ro \
  ghcr.io/manankharwar/fusioncore:latest \
  ros2 launch fusioncore_ros fusioncore.launch.py \
    fusioncore_config:=/config/robot.yaml \
    --ros-args \
    -r /imu/data:=/sensors/imu_0/data \
    -r /gnss/fix:=/gps/fix \
    -r /odom/wheels:=/wheel_odom
```

### How it works

| Flag | Meaning |
|---|---|
| `--ros-args` | Signals that the following arguments are ROS 2 argument overrides |
| `-r /imu/data:=/sensors/imu_0/data` | Remaps FusionCore's default `/imu/data` subscription to `/sensors/imu_0/data` |

!!! tip "Common remaps for popular hardware"
    | Platform | IMU topic remap |
    |---|---|
    | Intel RealSense D435i | `-r /imu/data:=/camera/imu` |
    | OAK-D (Luxonis) | `-r /imu/data:=/imu` |
    | Clearpath Husky | `-r /imu/data:=/sensors/imu_0/data` |
    | TurtleBot 3 (Gazebo Harmonic) | `-r /imu/data:=/imu` |

!!! note "YAML vs. command-line remapping"
    You can also set the IMU topic in your YAML config with `imu.topic: "/your/topic"`. The YAML parameter and the `-r` remap both work. Use `-r` when you want to keep your config file generic and override per-robot at launch time.

---

## Access topics from the host

To let the container communicate with ROS 2 nodes running on the host (or other containers), share the host's network namespace:

```bash
docker run --rm -it \
  --net=host \
  -v ~/my_robot.yaml:/config/robot.yaml:ro \
  ghcr.io/manankharwar/fusioncore:latest \
  ros2 launch fusioncore_ros fusioncore.launch.py \
    fusioncore_config:=/config/robot.yaml
```

!!! warning "`--net=host` and DDS"
    ROS 2 uses DDS for discovery, which relies on UDP multicast. `--net=host` is the simplest way to make discovery work across the container boundary. Without it, the container is isolated on a Docker bridge network and will not see host topics.

---

## Quick-reference: one-liners

```bash
# Pull the latest image
docker pull ghcr.io/manankharwar/fusioncore:latest

# Run the quick test (no config, no hardware)
docker run --rm ghcr.io/manankharwar/fusioncore:latest bash tools/quick_test.sh

# Interactive shell
docker run --rm -it ghcr.io/manankharwar/fusioncore:latest bash

# Run with your config + topic remaps + host networking
docker run --rm -it \
  --net=host \
  -v ~/my_robot.yaml:/config/robot.yaml:ro \
  ghcr.io/manankharwar/fusioncore:latest \
  ros2 launch fusioncore_ros fusioncore.launch.py \
    fusioncore_config:=/config/robot.yaml \
    --ros-args \
    -r fusioncore:/imu/data:=/sensors/imu_0/data \
    -r fusioncore:/gnss/fix:=/gps/fix \
    -r fusioncore:/odom/wheels:=/wheel_odom
```

---

## Building the image locally

If you want to build the image from source (e.g. to test a local change):

```bash
git clone https://github.com/manankharwar/fusioncore.git
cd fusioncore
docker build -t fusioncore:local .
```

The Dockerfile uses a **multi-stage build**: the `builder` stage compiles everything, and the `runtime` stage copies only the install tree. The final image is under 1 GB.
