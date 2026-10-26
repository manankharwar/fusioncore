# Reporting rule for the 2026-10-25 twelve-sequence evaluation, committed BEFORE it runs

This is pure evaluation. Nothing is tuned, selected or re-run to improve a number. The run
exists to replace two README sections (raw GPS against FusionCore, and FusionCore against
robot_localization) with one consistent, current measurement.

## What runs

All twelve NCLT sequences, the corrected player (`nclt_player.py` publishes the IMU
orientation with `orientation_covariance[0] = -1`, so neither filter fuses the
accelerometer-derived tilt), one sequence at a time, nothing else on the machine.

| Arm | Pipeline | Configuration |
|---|---|---|
| `fc_node` | ROS node, 1x playback (`tools/run_nclt.sh`) | shipped `fusioncore_datasets/config/nclt_fusioncore.yaml` |
| `rl_node` | ROS node, same playback as `fc_node` | `fusioncore_datasets/config/rl_ekf.yaml` + `navsat_transform.yaml`, copied with their sha256 into the run directory |
| `fc_replay` | deterministic replay (`hardware/replay.cpp`) | shipped config, no orientation fused |
| `fc_reanchor` | deterministic replay | shipped config plus `gnss.reanchor_enabled: true`, every other key at its default |

The re-anchoring arm runs through replay because a second node playback per sequence would
take about 33 hours. With the corrected player the node fuses no orientation, so plain replay
is the node-faithful pipeline. That is checked, not assumed: `fc_node` and `fc_replay` are the
same configuration on the same input, and their agreement is reported per sequence.

## What is reported

Per sequence and per metric, every row tagged by `tools/score_nclt_suite.py` with its content
hash, pipeline, scoring points and coverage:

- full trajectory: ATE RMSE 3D, median, truth-rate path ratio, output-rate path ratio (jitter)
- at GPS fix times: ATE RMSE 3D and median, with raw GPS, all columns on identical timestamps

## What is NOT reported

- **No aggregate and no "winner".** No mean across sequences, no win count, no "better
  overall", no "more robust". The table is per sequence and per metric, and the reader can see
  where each filter does better and worse.
- No sequence is dropped for its result. A sequence is excluded only if a run fails its
  coverage check (under 99% of the sequence's GPS-fix span) or its node run is starved (the
  harness's own "filter rate" verdict). Every exclusion is stated with its reason.
- No re-run of a sequence because its number looks wrong. A run is repeated only if it failed
  a mechanical check above, and then the repeat replaces it with the failure noted.

## When the README changes

Only after all twelve sequences have run, from this run alone, with the scorer version and
pipeline stated beside the tables.

## Amendment, 2026-10-26, written after the run and before any re-run

**What happened.** The `rl_node` arm passed every mechanical check above (rate, coverage)
and still reached 9 to 68 km of ATE RMSE on nine of the twelve sequences (for example
2012-05-11 33,666.604 m, 2012-12-01 67,746.846 m over the full trajectory). On 2012-05-11 its
estimated speed reads 1.0 m/s at t = 650 s, 14.4 m/s at 700 s and 52.0 m/s at 800 s, with
roll and pitch held at zero. The other three read 48.267, 379.746 and 106.367 m.

**Why, from robot_localization's own source, not a guess.** `rl_ekf.yaml` fuses IMU linear
acceleration with `imu0_remove_gravitational_acceleration: true` and `two_d_mode: true`.
In robot_localization 3.8.3 (`src/ros_filter.cpp`, lines 2744 to 2754), when an IMU message
carries `orientation_covariance[0] = -1`, gravity is removed using the filter's own roll and
pitch instead of the message's. In `two_d_mode` those are pinned at zero, so any real tilt of
the platform leaks into the fused acceleration as g sin(tilt), which the filter integrates.
Before 2026-10-25 our player published the orientation as valid, so gravity was removed with
the measured tilt. The corrected player (commit 98098a6, the change this evaluation exists
to measure) marks it -1. **The divergence is a consequence of our input change meeting this
config, not a property of robot_localization**, and publishing it would misrepresent the
baseline. Why three sequences did not diverge is not known.

**What changes, and only this.** `fusioncore_datasets/config/rl_ekf.yaml` follows the
planar wheeled-robot example in robot_localization's own documentation
(`doc/configuring_robot_localization.rst`, 3.8.3): wheel odometry fuses vx, vy and vyaw
(unchanged here), the IMU fuses yaw rate, GPS arrives through navsat_transform (unchanged).
One deliberate departure from that example: it also fuses IMU x acceleration, and this
config does not, because the same page drops y acceleration on the grounds that noisy
acceleration "can cause your estimate to drift rapidly", and with no attitude source here
x acceleration carries the same gravity leak (a Segway pitches to accelerate). The example
also fuses IMU yaw; NCLT's IMU has no absolute yaw, so it is off, as it already was.
Every other parameter (process noise, rejection thresholds, frequency, two_d_mode,
navsat_transform settings) is unchanged.

**How it is re-run.** All twelve sequences, not the nine that diverged, so the choice of
what to re-run does not depend on the result. Same harness, same corrected player, same
scorer (803c158e99ff), node playback at 1x, one at a time. That playback also produces a
second FusionCore node trajectory; it is reported as a repeat measurement next to the first
and does NOT replace the published `fc_node` numbers. The robot_localization numbers from
this re-run replace the withheld arm, scored against the same raw GPS and FusionCore arms,
with its own fix count stated, and no aggregate or winner, as above.

**What was published before the re-run.** The raw GPS and FusionCore results were published
2026-10-26 from this run alone (`tools/eval_20261025_results.md`), with the robot_localization
arm withheld and the README's robot_localization section left unchanged pending the re-run.
