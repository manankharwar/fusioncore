#!/bin/bash
# Re-measure every sequence in benchmark_baseline.json, sequentially, in one
# process tree, and write one summary.
#
#   tools/rerun_baseline.sh [out_root]
#
# WHY THIS EXISTS AS A SCRIPT RATHER THAN A SEQUENCE OF COMMANDS:
#
# On 2026-10-05 a 75 minute run was voided by watching it. Polling for progress
# with pgrep and ros2 topic hz took enough CPU to starve robot_localization 20
# times, and the control's ATE moved 4.40%, which destroys the one number that
# proves the input was the same. The run has to be started once and then left
# alone until it finishes. Nothing in here prints progress on a timer, and
# nothing outside should ask it to.
#
# A watcher written as `until pgrep -f nclt_player; do sleep 10; done` also
# matched its own command line and returned immediately, so a conversion that
# was believed to be running never started. Hence: no watchers at all.
#
# Read tools/benchmark_regression.md before trusting any number this prints.
# The achieved filter rate is the first thing to check, not the ATE: a starved
# run drops delayed measurements as DELAY_TOO_LARGE and dead-reckons, and its
# ATE means nothing. run_nclt.sh writes that rate into each manifest.
set -eo pipefail

WS="${ROS_WS:-$HOME/ros_ws}"
REPO="$WS/src/fusioncore"
OUT_ROOT="${1:-$HOME/nclt/rerun_$(date +%Y%m%d_%H%M)}"
RATE="${RATE:-1.0}"

# The sequences come from the baseline itself, so this cannot drift from what
# the regression checker gates on.
SEQUENCES=$(python3 -c "
import json
print(' '.join(json.load(open('$REPO/tools/benchmark_baseline.json'))['sequences']))")

mkdir -p "$OUT_ROOT"
SUMMARY="$OUT_ROOT/summary.txt"

{
  echo "NCLT baseline re-measurement"
  echo "started   $(date -Is)"
  echo "commit    $(git -C "$REPO" rev-parse HEAD)"
  echo "dirty     $(git -C "$REPO" status --short | wc -l) tracked file(s) modified"
  echo "rate      ${RATE}x"
  echo "sequences $SEQUENCES"
  echo "out       $OUT_ROOT"
  echo
} | tee "$SUMMARY"

for SEQ in $SEQUENCES; do
  echo "=== $SEQ  starting $(date -Is) ===" | tee -a "$SUMMARY"
  START=$(date +%s)
  if bash "$REPO/tools/run_nclt.sh" "$SEQ" "$OUT_ROOT/$SEQ" "$RATE" \
       > "$OUT_ROOT/$SEQ.stdout" 2>&1; then
    STATUS=ok
  else
    STATUS="FAILED(exit $?)"
  fi
  MINS=$(( ($(date +%s) - START) / 60 ))

  # robot_localization prints "Failed to meet update rate!" when it is starved.
  # Past runs logged 36 on 2012-06-15 and 280 on 2012-08-20, so this is not a
  # pass or fail gate, it is a number to carry beside the ATE and compare.
  STARVED=$(grep -ciE "failed to meet update rate" "$OUT_ROOT/$SEQ/launch.log" 2>/dev/null || echo 0)
  HZ=$(grep -oP 'achieved Hz\s+\K[0-9.]+' "$OUT_ROOT/$SEQ/manifest.txt" 2>/dev/null || echo "?")

  echo "  $SEQ  $STATUS  ${MINS} min  achieved ${HZ} Hz  rl_starvation_events ${STARVED}" \
    | tee -a "$SUMMARY"
done

echo | tee -a "$SUMMARY"
echo "=== results against baseline ===" | tee -a "$SUMMARY"

python3 - "$REPO/tools/benchmark_baseline.json" "$OUT_ROOT" $SEQUENCES <<'PY' | tee -a "$SUMMARY"
import json, os, sys
base = json.load(open(sys.argv[1]))["sequences"]
root = sys.argv[2]
print("  %-12s %9s %9s %8s   %9s %9s %8s   %s" % (
    "sequence", "FC base", "FC now", "delta", "RL base", "RL now", "delta", "integrity"))
for seq in sys.argv[3:]:
    p = os.path.join(root, seq, "res", "metrics.json")
    if not os.path.exists(p):
        print("  %-12s %s" % (seq, "NO METRICS, run did not reach evaluation"))
        continue
    m = json.load(open(p))["filters"]
    fc, rl = m["FusionCore"]["ate_rmse_3d"], m["RL-EKF"]["ate_rmse_3d"]
    fb, rb = base[seq]["fusioncore_ate_rmse_3d"], base[seq]["rl_ate_rmse_3d"]
    # The control is the integrity check. If RL moved, the input or the machine
    # changed and the FusionCore delta is not attributable to FusionCore.
    rd = 100.0 * (rl - rb) / rb
    flag = "ok" if abs(rd) < 1.0 else "CONTROL MOVED %.2f%%, FC DELTA NOT ATTRIBUTABLE" % rd
    print("  %-12s %9.3f %9.3f %+7.2f%%   %9.3f %9.3f %+7.2f%%   %s" % (
        seq, fb, fc, 100.0 * (fc - fb) / fb, rb, rl, rd, flag))
PY

echo | tee -a "$SUMMARY"
echo "finished  $(date -Is)" | tee -a "$SUMMARY"
echo "summary   $SUMMARY" | tee -a "$SUMMARY"
