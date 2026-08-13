#!/bin/bash
# Two 30-second reproductions of the position-vs-velocity defect. No ROS, no
# dataset, no harness. Found 2026-08-12 while chasing an NCLT "regression" that
# turned out to be a long-standing core bug.
W="${ROS_WS:-$HOME/ros_ws}"
for f in dr iso scope blackout split; do
  g++ -O2 -std=c++17 "$(dirname "$0")/$f.cpp" -o "/tmp/$f" \
    -I"$W/install/fusioncore_core/include" \
    -I"$W/install/fusioncore_core/include/fusioncore_core" \
    -I/usr/include/eigen3 \
    -L"$W/install/fusioncore_core/lib" -lfusioncore_core \
    -Wl,-rpath,"$W/install/fusioncore_core/lib" || exit 1
  echo "built /tmp/$f"
done
cat <<'TXT'

  /tmp/dr  <secs> <ground_constraint> [q_accel] [q_accel_bias] [alpha]
  /tmp/iso <encoder_updates> [alpha] [motion_model]

  Truth in both: 1.0 m/s straight, zero rotation, zero true acceleration.
  Expected: x = 1.0 * t, vx = 1.000, yaw = 0, accX = 0, bias = 0.

  Observed on 0.3.6 AND on c8b8b1f (May), bit-identical:
    alpha=0.1  accX runs to -2.8 and bias to -1.68 from zero input
    alpha=0.5  accel/bias clean, but position still advances at ~19% of vx
TXT
