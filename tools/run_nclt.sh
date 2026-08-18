#!/bin/bash
# One NCLT sequence, end to end: clean process table, play, evaluate, print.
#
# This keeps getting rewritten from scratch because it used to live in a
# scratchpad that did not survive the move off /mnt/c. It lives in the repo now.
#
#   tools/run_nclt.sh <sequence> <out_dir> [playback_rate]
#   tools/run_nclt.sh 2013-04-05 ~/nclt/alpha05 1.0
#
# Read tools/benchmark_regression.md before trusting any number this prints.
# In particular: check the achieved filter rate the summary reports. A starved
# run (well under ~85 Hz on 2013-04-05) has a meaningless ATE, because delayed
# measurements get dropped as DELAY_TOO_LARGE and the filter dead-reckons.
set -eo pipefail                      # NOT -u: ROS setup.bash reads unbound vars

SEQ="${1:?usage: run_nclt.sh <sequence> <out_dir> [rate]}"
OUT="${2:?usage: run_nclt.sh <sequence> <out_dir> [rate]}"
RATE="${3:-1.0}"
WS="${ROS_WS:-$HOME/ros_ws}"
DATA="${NCLT_DATA:-$HOME/nclt}/$SEQ"

[ -d "$DATA" ] || { echo "no data at $DATA (copy it from /mnt/c to ext4 first)"; exit 1; }
case "$DATA" in /mnt/c/*) echo "refusing to run off /mnt/c, it corrupts timing"; exit 1;; esac

# Kill anything left from a previous run. An orphaned player competes for CPU and
# an orphaned recorder appends to the NEXT run's bag, which has produced a bag
# with exactly double the fixes. Match on cmdline, but never on a pattern that
# appears in this script's own command line or pkill takes out our shell.
cleanup() {
  for pat in nclt_player.py fusion_node ekf_node navsat_transform 'ros2 bag record'; do
    for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
      [ "$pid" = "$$" ] && continue
      grep -qs . "/proc/$pid/cmdline" && kill -INT "$pid" 2>/dev/null || true
    done
  done
  sleep 3
  for pat in nclt_player.py fusion_node ekf_node; do pkill -9 -f -- "$pat" 2>/dev/null || true; done
}
cleanup
trap cleanup EXIT

mkdir -p "$OUT"
rm -rf "$OUT/bag" "$OUT/res"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

echo "sequence $SEQ   rate ${RATE}x   out $OUT"
echo "alpha compiled in: $(grep -oP 'double alpha = \K[0-9.]+' "$WS/src/fusioncore/fusioncore_core/include/fusioncore/ukf.hpp")"
START=$(date +%s)

ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
  data_dir:="$DATA" output_bag:="$OUT/bag" playback_rate:="$RATE" \
  > "$OUT/launch.log" 2>&1 || true

echo "playback finished in $(( ($(date +%s) - START) / 60 )) min"
cleanup                               # belt and braces: the bag needs SIGINT to write metadata
[ -f "$OUT/bag/metadata.yaml" ] || { echo "bag has no metadata.yaml, it is unreadable"; exit 1; }

# evaluate.py takes TUM trajectories, not the bag, so extract both first.
[ -f "$DATA/ground_truth.tum" ] || python3 "$WS/src/fusioncore/tools/nclt_rtk_to_tum.py" \
  --rtk "$DATA/gps_rtk.csv" --out "$DATA/ground_truth.tum"
python3 "$WS/src/fusioncore/tools/odom_to_tum.py" --bag "$OUT/bag" --topic /fusion/odom  --out "$OUT/fc.tum"
python3 "$WS/src/fusioncore/tools/odom_to_tum.py" --bag "$OUT/bag" --topic /rl/odometry --out "$OUT/rl.tum"

python3 "$WS/src/fusioncore/tools/evaluate.py" \
  --gt "$DATA/ground_truth.tum" --fusioncore "$OUT/fc.tum" --rl "$OUT/rl.tum" \
  --sequence "$SEQ" --out_dir "$OUT/res" 2>&1 | tail -20

# Duration comes from the ground truth, never from a hardcoded guess. Guessing
# 4200 s for a 3311 s sequence once made a healthy 81 Hz run look like a starved
# 64 Hz one, which would have thrown away a valid result.
GT_SPAN=$(python3 -c "
ls=[l for l in open('$DATA/ground_truth.tum') if l.strip()]
print('%.1f' % (float(ls[-1].split()[0]) - float(ls[0].split()[0])))")

python3 - "$OUT/res/metrics.json" "$GT_SPAN" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
dur = float(sys.argv[2])
fc = m["filters"]["FusionCore"]
print("\n  %-12s %10s %10s" % ("", "FusionCore", "RL-EKF"))
for k, lbl in (("ate_rmse_3d", "ATE 3D m"), ("rpe10_rmse", "RPE@10m"), ("path_length_ratio", "path ratio")):
    print("  %-12s %10.3f %10.3f" % (lbl, fc[k], m["filters"]["RL-EKF"][k]))
hz = m["poses"]["FusionCore"] / dur
print("\n  filter rate %.1f Hz over %.0f s  %s" % (hz, dur, "OK" if hz > 80 else "STARVED, ATE IS MEANINGLESS"))
PY
