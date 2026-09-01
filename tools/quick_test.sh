#!/bin/bash
# FusionCore quick verification test.
#
# Starts the filter with fake sensors and checks that all expected outputs
# are publishing. Replaces the 4-terminal manual test in Getting Started.
#
# Usage:
#   bash tools/quick_test.sh
#
# Requirements:
#   FusionCore built in this workspace:
#     colcon build --packages-up-to fusioncore_ros
#   OR use the Docker container:
#     docker run --rm ghcr.io/manankharwar/fusioncore:latest bash tools/quick_test.sh

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# Support both standalone build (install/ inside repo) and standard workspace layout
# (repo lives at ~/ros2_ws/src/fusioncore, install at ~/ros2_ws/install)
if   [[ -f "${REPO_ROOT}/install/setup.bash"       ]]; then WORKSPACE="${REPO_ROOT}"
elif [[ -f "${REPO_ROOT}/../../install/setup.bash" ]]; then WORKSPACE="$(cd "${REPO_ROOT}/../.." && pwd)"
else WORKSPACE="${REPO_ROOT}"; fi
PIDS=()
FAIL=0

GREEN='\033[0;32m'; RED='\033[0;31m'; BLUE='\033[0;34m'; YELLOW='\033[1;33m'; NC='\033[0m'
pass() { echo -e "  ${GREEN}[PASS]${NC} $*"; }
fail() { echo -e "  ${RED}[FAIL]${NC} $*"; FAIL=1; }
info() { echo -e "  ${BLUE}[....]${NC} $*"; }

cleanup() {
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT

echo ""
echo "  FusionCore Quick Test"
echo "  ====================="
echo ""

# ── 1. Source ROS ──────────────────────────────────────────────────────────────
info "Sourcing ROS environment..."
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    if   [[ -f /opt/ros/jazzy/setup.bash  ]]; then source /opt/ros/jazzy/setup.bash
    elif [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash
    else
        fail "ROS not found. Run:  source /opt/ros/<distro>/setup.bash"
        exit 1
    fi
fi

if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    set +u; source "${WORKSPACE}/install/setup.bash"
    pass "ROS environment sourced (workspace: ${WORKSPACE})"
elif [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
    pass "ROS environment already sourced"
else
    fail "FusionCore not built. Build with colcon from your workspace root, e.g.:"
    fail "  cd ~/ros2_ws && colcon build --packages-up-to fusioncore_ros"
    exit 1
fi

# ── 2. TF publishers ──────────────────────────────────────────────────────────
info "Starting TF publishers..."
ros2 run tf2_ros static_transform_publisher \
    --frame-id base_link --child-frame-id imu_link >/dev/null 2>&1 &
PIDS+=($!)
ros2 run tf2_ros static_transform_publisher \
    --frame-id odom --child-frame-id base_link >/dev/null 2>&1 &
PIDS+=($!)
sleep 1

# ── 3. Launch FusionCore ──────────────────────────────────────────────────────
info "Launching FusionCore..."
ros2 launch fusioncore_ros fusioncore.launch.py \
    env_config:="${REPO_ROOT}/tools/quick_test_params.yaml" >/dev/null 2>&1 &
PIDS+=($!)
sleep 3

# ── 4. Wait for the node to come up ───────────────────────────────────────────
# The launch file drives the lifecycle itself (autoconfigure defaults to true),
# so this waits for that to finish instead of calling configure and activate.
# Driving them from here as well races the launch file: whichever transition
# arrives second is invalid for the state the node is already in, so the script
# failed on configure or on activate depending on which side won.
# Each ros2 CLI call can take 5-10 s to answer on a slow machine, so give one
# call room to finish rather than assuming it returns promptly. The node itself
# reaches active about a second after launch.
info "Waiting for lifecycle node to reach active..."
STATE=""
for i in 1 2 3 4 5 6; do
    STATE="$(timeout 20 ros2 lifecycle get /fusioncore 2>/dev/null | head -1 | awk '{print $1}')"
    [[ "${STATE}" == "active" ]] && break
    sleep 2
done

if [[ "${STATE}" != "active" ]]; then
    if timeout 20 ros2 node list 2>/dev/null | grep -qx "/fusioncore"; then
        fail "/fusioncore is up but stalled in '${STATE:-unknown}' instead of active"
        echo "       A bad parameter is the usual cause. Rerun the launch to see the error:"
        echo "         ros2 launch fusioncore_ros fusioncore.launch.py \\"
        echo "           env_config:=${REPO_ROOT}/tools/quick_test_params.yaml"
    else
        fail "/fusioncore never appeared"
        echo "       Check: ros2 node list"
    fi
    exit 1
fi
pass "Lifecycle: active"
sleep 1

# ── 5. Fake sensors ───────────────────────────────────────────────────────────
info "Publishing fake IMU at 100 Hz (stationary, gravity pointing up, orientation provided)..."
ros2 topic pub /imu/data sensor_msgs/msg/Imu "{
  header: {frame_id: 'imu_link'},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0},
  orientation_covariance: [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
  angular_velocity: {x: 0.0, y: 0.0, z: 0.0},
  angular_velocity_covariance: [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001],
  linear_acceleration: {x: 0.0, y: 0.0, z: 9.81},
  linear_acceleration_covariance: [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
}" --rate 100 >/dev/null 2>&1 &
PIDS+=($!)

info "Publishing fake wheel odometry at 50 Hz (stationary)..."
ros2 topic pub /odom/wheels nav_msgs/msg/Odometry "{
  header: {frame_id: 'odom'},
  twist: {twist: {linear: {x: 0.0}, angular: {z: 0.0}}}
}" --rate 50 >/dev/null 2>&1 &
PIDS+=($!)

info "Waiting 6 s for filter to initialize..."
sleep 6

# ── 6. Checks ─────────────────────────────────────────────────────────────────
echo ""
echo "  Checks:"
echo "  -------"

check_topic() {
    local topic="$1" label="$2"
    if ros2 topic echo "${topic}" --once --timeout 5 >/dev/null 2>&1; then
        pass "${label}"
    else
        fail "${label}  (topic: ${topic})"
    fi
}

check_topic /fusion/odom  "/fusion/odom publishing (main output)"
check_topic /fusion/pose  "/fusion/pose publishing"
check_topic /diagnostics  "/diagnostics publishing"

if ros2 service call /fusioncore/reset std_srvs/srv/Trigger '{}' >/dev/null 2>&1; then
    pass "/fusioncore/reset service responds"
else
    fail "/fusioncore/reset service not found"
fi

# ── 7. Result ─────────────────────────────────────────────────────────────────
echo ""
if [[ $FAIL -eq 0 ]]; then
    echo -e "  ${GREEN}All checks passed.${NC} FusionCore is working correctly."
    echo ""
    echo "  Next: point it at your robot config:"
    echo "    ros2 launch fusioncore_ros fusioncore.launch.py \\"
    echo "      fusioncore_config:=/path/to/your_robot.yaml"
    echo ""
    exit 0
else
    echo -e "  ${RED}Some checks failed.${NC} Run for diagnostics:"
    echo "    ros2 topic echo /diagnostics --once"
    echo "    ros2 lifecycle get /fusioncore"
    echo ""
    exit 1
fi
