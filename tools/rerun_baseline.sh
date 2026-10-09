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
# the regression checker gates on. SEQUENCES overrides it for a deliberate run:
# a repeat of one sequence, or a sequence with no baseline entry yet. Repeats are
# fine, each gets its own numbered output directory.
#
#   SEQUENCES="2013-04-05 2012-08-20" bash tools/rerun_baseline.sh
if [ -n "${SEQUENCES:-}" ]; then
  echo "SEQUENCES overridden: $SEQUENCES"
else
  SEQUENCES=$(python3 -c "
import json
print(' '.join(json.load(open('$REPO/tools/benchmark_baseline.json'))['sequences']))")
fi

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

IDX=0
for SEQ in $SEQUENCES; do
  IDX=$((IDX + 1))
  # Numbered so the same sequence can appear twice in one night without the
  # second run overwriting the first. n=1 is not a measurement on the two
  # sequences whose spread is several percent.
  SLOT=$(printf "%02d_%s" "$IDX" "$SEQ")
  echo "=== $SLOT  starting $(date -Is) ===" | tee -a "$SUMMARY"
  START=$(date +%s)
  if bash "$REPO/tools/run_nclt.sh" "$SEQ" "$OUT_ROOT/$SLOT" "$RATE" \
       > "$OUT_ROOT/$SLOT.stdout" 2>&1; then
    STATUS=ok
  else
    STATUS="FAILED(exit $?)"
  fi
  MINS=$(( ($(date +%s) - START) / 60 ))

  # robot_localization prints "Failed to meet update rate!" when it is starved.
  # Past runs logged 36 on 2012-06-15 and 280 on 2012-08-20, so this is not a
  # pass or fail gate, it is a number to carry beside the ATE and compare.
  STARVED=$(grep -ciE "failed to meet update rate" "$OUT_ROOT/$SLOT/launch.log" 2>/dev/null || echo 0)
  HZ=$(grep -oP 'achieved Hz\s+\K[0-9.]+' "$OUT_ROOT/$SLOT/manifest.txt" 2>/dev/null || echo "?")

  echo "  $SLOT  $STATUS  ${MINS} min  achieved ${HZ} Hz  rl_starvation_events ${STARVED}" \
    | tee -a "$SUMMARY"
done

echo | tee -a "$SUMMARY"
echo "=== results against baseline ===" | tee -a "$SUMMARY"

python3 - "$REPO/tools/benchmark_baseline.json" "$OUT_ROOT" <<'PY' | tee -a "$SUMMARY"
import json, os, sys
base = json.load(open(sys.argv[1]))["sequences"]
root = sys.argv[2]
slots = sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))
print("  %-16s %9s %9s %8s   %9s %9s %8s   %s" % (
    "slot", "FC base", "FC now", "delta", "RL base", "RL now", "delta", "integrity"))
for slot in slots:
    seq = slot.split("_", 1)[1] if "_" in slot else slot
    p = os.path.join(root, slot, "res", "metrics.json")
    if not os.path.exists(p):
        print("  %-16s %s" % (slot, "NO METRICS, run did not reach evaluation"))
        continue
    m = json.load(open(p))["filters"]
    fc, rl = m["FusionCore"]["ate_rmse_3d"], m["RL-EKF"]["ate_rmse_3d"]
    if seq not in base:
        # No baseline entry: this row has never been benchmarked, so there is
        # nothing to compare against and saying so is the honest output.
        print("  %-16s %9s %9.3f %8s   %9s %9.3f %8s   NEW, no baseline entry" % (
            slot, "-", fc, "-", "-", rl, "-"))
        continue
    fb, rb = base[seq]["fusioncore_ate_rmse_3d"], base[seq]["rl_ate_rmse_3d"]
    # The control only validates a comparison when the INPUT is unchanged. It is
    # not a contamination detector: 2026-10-09 measured 62 starvation events
    # against 0 moving it 0.20%, while a change to the player moved it 148%.
    rd = 100.0 * (rl - rb) / rb
    flag = "ok" if abs(rd) < 1.0 else "CONTROL MOVED %.2f%%, CHECK WHETHER THE INPUT CHANGED" % rd
    print("  %-16s %9.3f %9.3f %+7.2f%%   %9.3f %9.3f %+7.2f%%   %s" % (
        slot, fb, fc, 100.0 * (fc - fb) / fb, rb, rl, rd, flag))
PY

echo | tee -a "$SUMMARY"
echo "finished  $(date -Is)" | tee -a "$SUMMARY"
echo "summary   $SUMMARY" | tee -a "$SUMMARY"
