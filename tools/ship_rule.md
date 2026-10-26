# Ship rule for changing a default, written before any holdout run (2026-10-22)

A candidate that is "4 of 5 better, 1 worse" on the sequences it was developed on proves
nothing about the next robot, because those five have been looked at hundreds of times.
This file fixes the rule before the data that decides it is touched. The commit that adds
it is the timestamp.

## Inventory, 2026-10-22

All 12 NCLT sequences have raw sensor CSVs and ground truth in `~/nclt/<seq>/`. Replay logs
(`~/nclt/exported/nclt_<seq>.txt`) exist for five:

    DEV (used for every investigation since 2026-10-17; contaminated by design)
      2012-02-04  2012-06-15  2012-08-20  2012-09-28  2013-04-05

    HOLDOUT (never replayed in that investigation; node runs exist for some, unused for tuning)
      2012-01-08  2012-03-31  2012-05-11  2012-10-28  2012-11-04  2012-12-01  2013-02-23

2012-01-08's earlier export is truncated at 66% and is NOT reused; it is rebuilt with the rest.

## Building the holdout logs, and the gate on that

The holdout logs do not exist yet. Either path is a change to the measurement chain:

1. Record inputs on seven 1x node runs and export them (about 10 h, sequential), or
2. A direct converter from the NCLT CSVs to the replay log format.

A converter is admissible ONLY if, run on the five DEV sequences, it reproduces the existing
exported logs event for event (same event types, same timestamps, values within float
formatting). If it does not, it is not used. This repo already has a dataset player whose
frame error invalidated every number measured through it (#169).

## The rule

Measured in replay (deterministic, so no run-to-run noise), shipped NCLT config plus the
candidate's keys, every arm scored through `hardware/ate_tum.py`, against the shipped config
on the same holdout sequence. A candidate becomes a default only if ALL of these hold on the
HOLDOUT:

1. **No sequence worse by more than 5%** in ATE RMSE 3D.
2. **At least 5 of 7 better by 2% or more** in ATE RMSE 3D.
3. **No sequence's median error worse by more than 3%.**
4. **No sequence's `output_rate_path_ratio` worse by more than 0.05.**
5. **No degenerate estimate:** truth-rate path length ratio within 0.10 of the shipped
   config's on every sequence. (2012-06-15 at AY sigma 0.5 scored -44.8% RMSE while
   churning in place; this clause exists for that.)
6. **One shot.** A candidate is run on the holdout once. If it fails, it does not ship, and
   a retuned version may not be re-tested on the same holdout: the holdout is then spent for
   that line of work and has to be replaced (KAIST, or a hardware run with RTK truth).

DEV results are reported alongside, but they cannot make a candidate pass.

## Candidates queued against this rule

    A   ukf.semidefinite_sqrt (the leak fix; changes every output)
    B   platform.profile ground_robot + gyro heading in coast
        (gnss.coast_imu_wz_scale 1, gnss_coast_encoder_wz_scale 1e6)
    C   A + B

Order: A first, because it changes the meaning of "frozen" in B.

## Outcome, 2026-10-23

Holdout logs built for all seven; candidates A and C run once each.

    A   FAIL   4/7 better by 2%; 2012-11-04 +20.5% RMSE, +12.4% median
    C   FAIL   3/7 better; 2012-01-08 +251.9%, 2012-03-31 +15.3%

Neither ships. B was not run: it is C without the leak fix, and its DEV result (2012-08-20
+9.2%) was already the one C was meant to repair. The holdout is spent for this line of work
(rule 6), so B cannot now be tested on it either. Detail in
tools/benchmark_regression.md, 2026-10-23.

## Addendum, 2026-10-25, written and committed BEFORE any run it describes

A separate line of work, so the holdout is fresh for it: **candidate R, re-anchoring on by
default**, the shipping re-anchor configuration (`gnss.reanchor_enabled: true`, rate trigger
only, bias trigger off: the library defaults for every other `gnss.reanchor_*` key).

**Pipeline changes for R, and why:** replay with `imuq_rp=1`, which fuses the IMU orientation
exactly as the node does. On 2026-10-25 that was shown to reproduce the node on 2012-01-08 to
0.3% where plain replay was 3.43x off. The baseline arm is the shipped config on the same
pipeline. Everything else in the rule (clauses 1 to 6, scoring through hardware/ate_tum.py)
applies unchanged. R is run once on the seven holdout sequences; DEV results are reported
alongside and cannot make it pass.

## Outcome of R, 2026-10-25

    R   FAIL   RMSE lower on 6/7 (2012-01-08 -90.5%), none higher; fails clause 3
               (median +3.3% on 2012-03-31, +4.4% on 2012-11-04) and clause 4
               (output-rate ratio +0.126 on 2012-12-01)

Re-anchoring stays opt-in. The holdout is spent for this line.

## Addendum, 2026-10-26, written and committed BEFORE either run it describes

R failed the rule above. This addendum does not reopen it. It records a PRODUCT decision to be
taken openly, and the one mechanical check that gates it.

**The decision, if the check passes:** `platform.profile: ground_robot` turns on
`gnss.reanchor_enabled`. The library default stays off. It is stated as a decision with its
measured costs, not as a pass. From `tools/eval_20261025_results.md`, re-anchoring against the
default on the same pipeline (replay), full trajectory: RMSE lower on 10 of 12 and higher on
none; median higher on 4 (+0.3% to +3.0%); output-rate jitter higher on 3 (up to +0.124).
It is a behaviour change for existing ground_robot users: CHANGELOG, version bump, release note.

**The check: one ROS node run, 2012-01-08, corrected player, shipped NCLT config plus
`gnss.reanchor_enabled: true`, 1x playback.** Its fix-time ATE RMSE (score_nclt_suite
803c158e99ff) is compared with the replay re-anchoring figure on the same sequence, 10.899 m.
The band is the node-over-replay ratio observed on the default arm across the eleven sequences
of that evaluation other than 2012-06-15, which is under separate investigation: 0.849 to
1.138. **Confirmed if the node reads 9.253 m to 12.403 m. Outside that, the preset change is
held until after 2026-11-04**, and nothing about re-anchoring through the node is claimed.

**2012-06-15 triage, from the repeat FusionCore node run that the robot_localization re-run
produces anyway.** First run: node 24.363 m, replay 12.041 m at fix times (2.023x). Using the
same band: a repeat at or below 13.703 m (12.041 x 1.138) reads as node nondeterminism (message
ordering/timing) and becomes the next node-path fix; at or above 20.684 m (24.363 x 0.849) as
systematic, something the node does differently from replay on that sequence, to be found the
way the roll/pitch input was; in between, inconclusive, and the next step is a third run, not
a theory.
