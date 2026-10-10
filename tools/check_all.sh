#!/bin/bash
# Run every repo checker. One script, so CI and a developer run the SAME thing.
#
# Why this exists, and it is not a convenience wrapper. On 2026-10-06 CI went red on
# check_config_wiring.py for three commits while a local sweep reported everything ok.
# The local sweep grepped stdout for words like "NOT wired" and "MISSING", and that
# checker's failure message contains neither. Two verification paths, and the weaker
# one won.
#
# The contract of a checker is its EXIT CODE. Nothing else. This reads exit codes and
# nothing else, and CI calls this exact script, so a grep cannot drift from the
# contract again.
#
#   bash tools/check_all.sh            # fail fast, stop at the first failure
#   bash tools/check_all.sh --all      # run every checker, then report
#
# Exit 0 if every checker passed, 1 if any failed, 2 if any could not run.
set -o pipefail

FAIL_FAST=1
[ "$1" = "--all" ] && FAIL_FAST=0

cd "$(dirname "$0")/.." || exit 2

# Checkers that need no arguments. check_benchmark_regression.py is deliberately NOT
# here: it needs metrics.json files, and a checker that cannot run must not be
# confused with one that passed. CI runs it separately, with data.
CHECKS=(
  "check_config_params.py --quiet"
  "check_config_wiring.py"
  "check_node_member_wiring.py"
  "check_docs_params.py"
  "check_docs_test_count.py"
  "check_replay_wiring.py"
  "check_links.py"
)

pass=0; fail=0; skip=0
failed_names=()

for c in "${CHECKS[@]}"; do
  name="${c%% *}"
  if [ ! -f "tools/$name" ]; then
    printf "  %-34s SKIP (not present)\n" "$name"
    skip=$((skip + 1))
    continue
  fi
  # shellcheck disable=SC2086 -- $c intentionally splits into script + args
  out=$(python3 tools/$c 2>&1); rc=$?
  if [ $rc -eq 0 ]; then
    printf "  %-34s ok\n" "$name"
    pass=$((pass + 1))
  else
    printf "  %-34s FAIL (exit %d)\n" "$name" "$rc"
    echo "$out" | sed 's/^/      /'
    fail=$((fail + 1))
    failed_names+=("$name")
    [ $FAIL_FAST -eq 1 ] && { echo; echo "FAIL: $name exited $rc. Stopping."; exit 1; }
  fi
done

echo
if [ $fail -gt 0 ]; then
  echo "FAIL: $fail of $((pass + fail)) checkers failed: ${failed_names[*]}"
  exit 1
fi
if [ $skip -gt 0 ]; then
  echo "WARNING: $skip checker(s) were not present and did not run."
  exit 2
fi
echo "PASS: all $pass checkers exited 0."
