#!/bin/bash
# Verify that all prerequisites for the NCLT benchmark are in place.
# Run this before run_one.sh or run_all.sh.
#
# Usage: bash benchmarks/check_prereqs.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

PASS=0
FAIL=0

ok()   { echo "  [ok]   $1"; PASS=$((PASS + 1)); }
fail() { echo "  [FAIL] $1"; FAIL=$((FAIL + 1)); }
info() { echo "         $1"; }

echo ""
echo "FusionCore benchmark prerequisites"
echo "==================================="

# 1. ROS sourced
echo ""
echo "ROS 2"
if [ -n "${ROS_DISTRO:-}" ]; then
    ok "ROS_DISTRO=$ROS_DISTRO"
else
    fail "ROS 2 is not sourced"
    info "Fix: source /opt/ros/jazzy/setup.bash  (or your distro)"
fi

# 2. fusioncore_datasets package built
echo ""
echo "FusionCore workspace"
if ros2 pkg list 2>/dev/null | grep -q "^fusioncore_datasets$"; then
    ok "fusioncore_datasets found in ROS path"
else
    INSTALL_SETUP="$REPO/install/setup.bash"
    if [ -f "$INSTALL_SETUP" ]; then
        fail "fusioncore_datasets not found — workspace not sourced"
        info "Fix: source $INSTALL_SETUP"
    else
        fail "fusioncore_datasets not built"
        info "Fix: cd $REPO && colcon build --packages-select fusioncore_core fusioncore_ros fusioncore_datasets"
    fi
fi

# 3. robot_localization installed
echo ""
echo "robot_localization"
if ros2 pkg list 2>/dev/null | grep -q "^robot_localization$"; then
    ok "robot_localization found"
else
    fail "robot_localization not found"
    info "Fix: sudo apt install ros-\${ROS_DISTRO}-robot-localization"
fi

# 4. evo installed
echo ""
echo "evo (trajectory evaluation)"
if python3 -c "import evo" 2>/dev/null; then
    EVO_VER=$(python3 -c "import evo; print(evo.__version__)" 2>/dev/null || echo "unknown")
    ok "evo $EVO_VER"
else
    fail "evo not installed"
    info "Fix: pip install evo --break-system-packages"
fi

# 5. matplotlib installed
echo ""
echo "matplotlib (plotting)"
if python3 -c "import matplotlib" 2>/dev/null; then
    ok "matplotlib found"
else
    fail "matplotlib not installed"
    info "Fix: pip install matplotlib --break-system-packages"
fi

# 6. curl (for download script)
echo ""
echo "curl (for nclt_download.sh)"
if command -v curl &>/dev/null; then
    ok "curl found"
else
    fail "curl not found"
    info "Fix: sudo apt install curl"
fi

# 7. NCLT data check (at least one sequence)
echo ""
echo "NCLT data"
FOUND_SEQ=0
for SEQ in 2012-01-08 2012-02-04 2012-03-31 2012-05-11 2012-06-15 2012-08-20 \
           2012-09-28 2012-10-28 2012-11-04 2012-12-01 2013-02-23 2013-04-05; do
    RAW="$SCRIPT_DIR/nclt/$SEQ/raw files"
    if [ -f "$RAW/ms25.csv" ] && [ -f "$RAW/gps.csv" ] && [ -f "$RAW/gps_rtk.csv" ]; then
        ok "$SEQ  (ms25.csv, gps.csv, gps_rtk.csv present)"
        FOUND_SEQ=$((FOUND_SEQ + 1))
    fi
done
if [ $FOUND_SEQ -eq 0 ]; then
    fail "No NCLT sequences found under benchmarks/nclt/<date>/raw files/"
    info "Fix: bash benchmarks/nclt_download.sh 2012-01-08"
fi

# Summary
echo ""
echo "==================================="
echo "  Passed: $PASS   Failed: $FAIL"
echo "==================================="
echo ""

if [ $FAIL -gt 0 ]; then
    echo "Fix the issues above, then run:"
    echo "  bash benchmarks/run_one.sh 2012-01-08"
    exit 1
else
    echo "All checks passed. Ready to benchmark:"
    echo "  bash benchmarks/run_one.sh 2012-01-08"
fi
