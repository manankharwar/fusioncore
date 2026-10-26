# Benchmark Results

FusionCore on the [NCLT dataset](http://robots.engin.umich.edu/nclt/) (University of Michigan
North Campus Long-Term): twelve sequences of a Segway robot driving the same campus across all
seasons, one config file, no per-sequence tuning. Every number on this page comes from the
2026-10-26 evaluation unless it is explicitly labelled as history.

---

## Current results, 2026-10-26

All twelve sequences were re-run under a reporting rule committed before the run started
([`tools/report_rule_20261025.md`](https://github.com/manankharwar/fusioncore/blob/main/tools/report_rule_20261025.md)).
The dataset player was corrected first: it had been declaring the IMU's accelerometer-derived
tilt as a 0.57 degree attitude measurement. That declaration was ours, not the dataset's. It
counted the accelerometer twice and made the ROS node lock onto a wrong position for about
800 s on 2012-01-08 (178.808 m full-trajectory RMSE before the correction, 57.314 m in this run).

Scored at GPS fix times, **the shipped default does not beat raw GPS**: raw GPS has the lower
RMSE on 11 of 12, and the twelfth, 2012-05-11, is a tie at 9.186 m against 9.187 m. With
opt-in re-anchoring (`gnss.reanchor_enabled`), FusionCore is lower than raw GPS on 2 of 12,
2012-03-31 by 22.0% and 2012-10-28 by 5.6%, both held-out sequences never used for tuning.
Against the default on the same pipeline and scored over the full trajectory, re-anchoring
lowers RMSE on 10 of 12 and raises it on none. It also raises the median on 4 (by 0.3% to
3.0%) and the output-rate jitter on 3 (by up to 0.124), which is why it is off by default.

ATE RMSE (3D) at GPS fix times, every column on identical timestamps:

| Sequence | Set | Raw GPS | Default, ROS node | Default, replay | Re-anchoring on, replay | Re-anchoring on, ratio to raw GPS |
|---|---|---|---|---|---|---|
| 2012-01-08 | held out | 9.883 m | 56.957 m | 50.065 m | 10.899 m | 1.103x |
| 2012-02-04 | dev | 57.564 m | 163.830 m | 145.629 m | 59.340 m | 1.031x |
| 2012-03-31 | held out | 13.825 m | 16.818 m | 16.937 m | 10.787 m | 0.780x |
| 2012-05-11 | held out | 9.187 m | 9.186 m | 9.187 m | 9.187 m | 1.000x |
| 2012-06-15 | dev | 9.293 m | 24.363 m | 12.041 m | 12.041 m | 1.296x |
| 2012-08-20 | dev | 34.250 m | 160.359 m | 172.961 m | 39.790 m | 1.162x |
| 2012-09-28 | dev | 8.960 m | 18.883 m | 22.236 m | 10.176 m | 1.136x |
| 2012-10-28 | held out | 13.923 m | 15.999 m | 16.763 m | 13.150 m | 0.944x |
| 2012-11-04 | held out | 20.267 m | 23.626 m | 24.512 m | 20.414 m | 1.007x |
| 2012-12-01 | held out | 12.681 m | 16.160 m | 16.466 m | 12.741 m | 1.005x |
| 2013-02-23 | held out | 15.724 m | 17.294 m | 16.951 m | 15.793 m | 1.004x |
| 2013-04-05 | dev | 8.624 m | 9.994 m | 10.007 m | 8.798 m | 1.020x |

Medians, path ratios, output-rate jitter and coverage for every arm and sequence, including the
full-trajectory scores, are in
[`tools/eval_20261025_results.md`](https://github.com/manankharwar/fusioncore/blob/main/tools/eval_20261025_results.md).

**What this does and does not show.** Re-anchoring was run through the deterministic replay
only, so compare it with the replay default column, not the node one. The node and the replay
still disagree: node over replay is 0.70x to 1.14x over the full trajectory, and at fix times
up to 2.0x on 2012-06-15 (24.363 m against 12.041 m). That gap is not yet explained. NCLT's
ground truth covers only about 18% of the time GPS is absent, so this dataset cannot say how
well any filter coasts through a long outage, in either direction.

## robot_localization: re-run pending

**There is no current FusionCore against robot_localization comparison on this page, and no
claim in either direction should be drawn from older ones.** In the 2026-10-26 run
robot_localization's config fused IMU acceleration. Once the corrected player stopped
supplying an attitude, robot_localization removed gravity using a level attitude, so real tilt
leaked into the acceleration and it diverged to kilometres on nine sequences. That was our
input change meeting that config, not a property of robot_localization, so those numbers are
withheld. All twelve are being re-run with the config following robot_localization's own
planar wheeled-robot example; the amendment in the rule file states exactly what changed and
why. This page will carry those results, per sequence and per metric, once they exist.

---

## What the measurements since May 2026 found

- **Most of the error is recovery, not the blackout itself.** On the five development
  sequences, 74.8% to 96.8% of squared error sits in GPS blackouts or in the re-convergence
  after them, and in that recovery window the outlier gate rejected 81% of arriving fixes. The
  filter was discarding the fixes that would have corrected it. `gnss.reanchor_enabled` is the
  fix for that part, and it stays opt-in for the costs stated above.
- **Two input errors in our own dataset player** fed both filters wrong data: a wheel
  odometry yaw rate in the wrong frame (#169) and the 0.57 degree tilt
  declaration (found 2026-10-25). Every number measured before each fix is superseded.
- **Run-to-run spread through the ROS node can reach 14.8%** on a single sequence
  (2012-02-04, n=3), while the replay is deterministic. A single node run cannot grade a small
  difference.

The full narrative, with every number and its provenance, is in
[`tools/benchmark_regression.md`](https://github.com/manankharwar/fusioncore/blob/main/tools/benchmark_regression.md).

---

## Methodology

**Dataset:** NCLT (University of Michigan, 2012-2013). Segway RMP on a large campus over
multiple seasons, replayed from the raw CSV files by `nclt_player`.

**Sensors fused:**

- IMU: Microstrain 3DM-GX3-45 at 100 Hz (accelerometer and gyro; its orientation output is
  published as invalid, `orientation_covariance[0] = -1`, because it is derived from the same
  accelerometer)
- Wheel odometry: Segway RMP at 100 Hz
- GPS: the dataset's consumer-grade receiver (`gps.csv`), 3D fixes only

**Ground truth:** NCLT's RTK GPS (`gps_rtk.csv`), projected to local ENU.

**Pipelines:** "node" is the ROS node at 1x playback (achieved filter rate 99.8 to 103.3 Hz on
every sequence). "replay" is the deterministic offline replay (`hardware/replay.cpp`) of the
same config on the same input.

**Scoring:** `tools/score_nclt_suite.py` (version `803c158e99ff`, printed on every row) through
`hardware/ate_tum.py`: Umeyama SE(3) alignment, 0.1 s association, which matches evo at the
same tolerance. "At GPS fix times" means every column, raw GPS included, is scored on
identical timestamps.

**FusionCore config:** `fusioncore_datasets/config/nclt_fusioncore.yaml`, identical across all
twelve sequences. "Held out" sequences were never used to develop or tune anything
([`tools/ship_rule.md`](https://github.com/manankharwar/fusioncore/blob/main/tools/ship_rule.md)).

---

## History: the May 2026 table (superseded)

This page used to lead with the table below, under the heading "12 sequences, 10 FC wins".
**That claim is withdrawn.** Every row was measured through the dataset player with both
input errors above, which fed both filters wrong data, and re-measurement flipped published
results in both directions (2012-01-08 went from FC 18.6 m against RL 41.2 m to FC 185.2 m
against RL 22.8 m). It is kept so the correction is visible. Nothing should be concluded from
it in either direction.

| Sequence | Season | Duration | GPS fixes | Max blackout | FC ATE (3D), May 2026 | RL-EKF ATE (3D), May 2026 |
|---|---|---|---|---|---|---|
| 2012-01-08 | Winter | 92 min | 22,041 | 203s | 18.6 m | 41.2 m |
| 2012-02-04 | Winter | 77 min | 18,808 | 184s | 49.7 m | 265.5 m |
| 2012-03-31 | Spring | 87 min | 20,482 | 262s | 22.0 m | 156.5 m |
| 2012-05-11 | Spring | 84 min | 21,621 | 120s | 9.7 m | 11.5 m |
| 2012-06-15 | Summer | 55 min | 12,399 | 462s | 49.2 m | 18.2 m |
| 2012-08-20 | Summer | 83 min | 20,025 | 228s | 98.3 m | 10.6 m |
| 2012-09-28 | Fall | 77 min | 19,191 | 196s | 22.4 m | 53.8 m |
| 2012-10-28 | Fall | 85 min | 21,060 | 256s | 15.6 m | 56.4 m |
| 2012-11-04 | Fall | 79 min | 17,840 | 400s | 60.1 m | 122.0 m |
| 2012-12-01 | Winter | 75 min | 17,941 | 173s | 21.0 m | 90.7 m |
| 2013-02-23 | Winter | 78 min | 19,333 | 240s | 59.4 m | 82.2 m |
| 2013-04-05 | Spring | 68 min | 16,297 | 275s | 12.1 m | 268.9 m |

The analysis that used to follow it ("why RL-EKF fails on 10 sequences", performance tiers,
and the two losses explained as heading drift) was built on these numbers and is withdrawn
with them. The recovery finding above replaced it.

**Corrections, in order:** 2026-10-09, five rows re-measured after the #169 frame fix,
FusionCore two wins and three losses. 2026-10-17, nine rows, four and five, with four of the
nine at n=1. 2026-10-18, raw GPS published as the baseline the benchmark never had, and found
to beat FusionCore on all five development sequences. 2026-10-26, the twelve-sequence
evaluation above, with robot_localization withheld.

---

## Reproduce

```bash
# Build
colcon build --packages-select fusioncore_core fusioncore_ros fusioncore_datasets

# One sequence through the ROS node at 1x (about 60 to 95 min)
bash tools/run_nclt.sh 2012-01-08 <output_dir> 1.0

# Score several trajectories of one sequence on identical points
python3 tools/score_nclt_suite.py <exported.txt> <ground_truth.tum> \
    NAME=path[@pipeline] [NAME=path[@pipeline] ...] --seq 2012-01-08
```

One playback at a time: a starved run's numbers are meaningless, and `tools/run_nclt.sh`
reports the achieved filter rate so you can check.
