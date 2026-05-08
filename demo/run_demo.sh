#!/bin/bash
# FusionCore demo runner
#
# Usage:
#   bash demo/run_demo.sh              # full live demo (downloads bag if needed)
#   bash demo/run_demo.sh --plot-only  # just generate the comparison plot
#   bash demo/run_demo.sh --quick      # zero-dependency plot, no ROS needed
#
# Requirements for full demo: ROS 2 Jazzy or Humble, FusionCore built
# Requirements for --quick:   python3, numpy, matplotlib

set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEMO_BAG="$REPO_ROOT/demo/nclt_demo_120s.mcap"
DEMO_OUT="/tmp/fc_demo_out"
DEMO_TUM="/tmp/fc_live.tum"
DEMO_PLOT="$REPO_ROOT/demo_result.png"
BAG_URL="https://github.com/manankharwar/fusioncore/releases/download/demo-data/nclt_demo_120s.mcap"

# ── colour helpers ─────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; BLUE='\033[0;34m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${BLUE}[demo]${NC} $*"; }
ok()    { echo -e "${GREEN}[demo]${NC} $*"; }
warn()  { echo -e "${YELLOW}[demo]${NC} $*"; }

# ── --quick: no ROS required ──────────────────────────────────────────────────
if [[ "$1" == "--quick" ]]; then
    info "Quick demo (pre-baked results, no ROS required)"
    info "Requirements: pip install numpy matplotlib"
    python3 "$REPO_ROOT/tools/demo_quick.py" --out "$DEMO_PLOT" --open
    ok "Done. Result: $DEMO_PLOT"
    exit 0
fi

# ── --plot-only: convert recorded bag and plot ────────────────────────────────
if [[ "$1" == "--plot-only" ]]; then
    if [[ ! -d "$DEMO_OUT" ]]; then
        echo "No recorded bag found at $DEMO_OUT. Run the full demo first."
        exit 1
    fi
    info "Converting recorded bag to TUM format..."
    python3 "$REPO_ROOT/tools/odom_to_tum.py" \
        --bag "$DEMO_OUT" --topic /fusion/odom --out "$DEMO_TUM"
    info "Generating comparison plot..."
    python3 "$REPO_ROOT/tools/demo_quick.py" \
        --live_tum "$DEMO_TUM" --out "$DEMO_PLOT" --open
    ok "Done. Result: $DEMO_PLOT"
    exit 0
fi

# ── full live demo ────────────────────────────────────────────────────────────
echo ""
echo "  FusionCore Live Demo"
echo "  ===================="
echo "  Replays 120 seconds of real outdoor robot data through FusionCore."
echo "  Compares the result against robot_localization EKF and RTK GPS ground truth."
echo ""

# Step 1: Download demo bag if not present
if [[ ! -f "$DEMO_BAG" ]]; then
    info "Downloading demo bag (~5 MB)..."
    if command -v wget &>/dev/null; then
        wget -q --show-progress -O "$DEMO_BAG" "$BAG_URL"
    elif command -v curl &>/dev/null; then
        curl -L --progress-bar -o "$DEMO_BAG" "$BAG_URL"
    else
        echo "wget or curl required. Install with: sudo apt install wget"
        exit 1
    fi
    ok "Downloaded: $DEMO_BAG"
else
    info "Demo bag already present: $DEMO_BAG"
fi

# Step 2: Check ROS environment
if [[ -z "$AMENT_PREFIX_PATH" ]]; then
    warn "ROS environment not sourced. Trying /opt/ros/jazzy/setup.bash ..."
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        source /opt/ros/jazzy/setup.bash
    elif [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
    else
        echo "Could not find ROS installation. Source your ROS setup.bash first."
        exit 1
    fi
fi

if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
    source "$REPO_ROOT/install/setup.bash"
else
    warn "FusionCore install not found at $REPO_ROOT/install/."
    warn "Build first: colcon build --packages-up-to fusioncore_ros fusioncore_datasets"
    exit 1
fi

# Step 3: Clean previous output
rm -rf "$DEMO_OUT"

# Step 4: Run FusionCore + bag replay in background, wait for completion
info "Starting FusionCore and replaying demo bag (120 seconds at 2x speed)..."
info "Press Ctrl+C to stop early."

ros2 launch fusioncore demo/nclt_demo.launch.py \
    bag:="$DEMO_BAG" output:="$DEMO_OUT" rate:=2.0 &
LAUNCH_PID=$!

# Wait for bag to finish (120s / 2x rate = ~65s, plus overhead)
sleep 75
kill $LAUNCH_PID 2>/dev/null || true
wait $LAUNCH_PID 2>/dev/null || true

# Step 5: Convert and plot
info "Bag playback complete. Generating comparison plot..."
python3 "$REPO_ROOT/tools/odom_to_tum.py" \
    --bag "$DEMO_OUT" --topic /fusion/odom --out "$DEMO_TUM"

python3 "$REPO_ROOT/tools/demo_quick.py" \
    --live_tum "$DEMO_TUM" --out "$DEMO_PLOT" --open

ok ""
ok "Demo complete."
ok "Result: $DEMO_PLOT"
ok ""
ok "To run FusionCore on your own robot, see:"
ok "  https://manankharwar.github.io/fusioncore/getting-started/"
