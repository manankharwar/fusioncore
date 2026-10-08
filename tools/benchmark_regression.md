# Benchmark regression tracking

FusionCore is a tuned filter with many coupled parameters. A change made to win
one scenario (for example a long GPS blackout) can quietly worsen another. This
directory holds a small system that makes those trade-offs **visible** instead of
letting benchmark numbers drift away from the code over time.

## The pieces

- `evaluate.py` writes `metrics.json` next to `BENCHMARK.md` on every run
  (machine-readable ATE / drift / RPE per filter).
- `benchmark_baseline.json` records the reference FusionCore XY ATE per sequence,
  with provenance (`commit`, `measured`, and a `verified` flag).
- `check_benchmark_regression.py` diffs fresh `metrics.json` against the baseline
  and exits non-zero if FusionCore XY ATE grew beyond the threshold.

## Workflow

1. Run a sequence (produces `results.../metrics.json`):

   ```
   scratchpad/run_nclt_fast.sh 2013-04-05 1.0 0.0
   ```

   or re-evaluate existing `.tum` trajectories without re-running the filter:

   ```
   python3 tools/evaluate.py \
     --gt <gt.tum> --fusioncore <fc.tum> --rl <rl.tum> \
     --sequence 2013-04-05 --out_dir <results_dir>
   ```

2. Check for regressions:

   ```
   python3 tools/check_benchmark_regression.py \
     --glob 'benchmarks/nclt/*/results_fast/metrics.json'
   ```

   Exit code is non-zero if any sequence regressed, so this can gate a release or
   a CI job before published numbers are updated.

3. When a run is trusted (controlled config, current `main`), update the sequence
   entry in `benchmark_baseline.json` and set `"verified": true`.

## Status and honesty notes

- The baseline is **partial**: only the sequences currently staged on disk are
  tracked. A controlled full-suite re-run on current `main` is still owed before
  any published benchmark table is updated.
- `verified: false` entries were measured mid-investigation and should be re-run
  under a controlled current-`main` config before being trusted.
- Rule of thumb: do not update a published number without a `verified` baseline
  entry and a clean regression check.

Why this exists: the published benchmark snapshot predated weeks of filter
changes, some of which improved the long-blackout sequences while regressing
another, and nothing caught it because scores were never tracked per commit.
This closes that gap.

## Working on the machine during a playback voids the control (2026-10-08)

Measured, not suspected. A baseline re-measurement was started at 15:18 and
2012-06-15 came back at 78.112 m against a recorded 73.525 m, which looks like
a 6.24% regression worth investigating. It is not a result at all, because the
robot_localization control moved from 18.487 m to 45.772 m, **+147.59%**. A
control that moves 148% cannot certify that the input was the same, so the
FusionCore delta beside it is not attributable to FusionCore.

FusionCore's own achieved rate was fine, 97.9 Hz. The control's was not: 65325
poses over 3311 s is 19.7 Hz against FusionCore's 324063. So the starvation was
specific to robot_localization, which is the process whose health the ATE
comparison depends on.

What starved it was this session doing other work on the same machine while the
playback ran: fetching URLs, running `ros2 pkg list` through check_prereqs.sh,
rewriting a script, and committing. Mapping each "Failed to meet update rate"
to wall-clock minute shows it directly:

  2012-06-15, worked on during playback   54 events in the first 31 min
                                          (10 in one minute, 19 across three)
  2012-06-15, left alone after 15:50       7 events in the next 29 min
  2012-08-20, left alone from the start    1 event in 19 min

Three orders of difference in event rate between working and not working, on the
same machine, in the same hour, on the same harness.

**THIS IS THE SAME MISTAKE AS 2026-10-05**, when polling a run with pgrep and
`ros2 topic hz` moved the control 4.40% and voided 75 minutes. The lesson was
recorded as "do not poll it", and the header of rerun_baseline.sh says so. The
lesson is actually larger and this is the corrected form:

  A playback owns the machine. Not just no polling: no builds, no package
  queries, no downloads, no test runs, nothing. Start it and go and do something
  that is not on this computer.

Historical starvation counts are in the old logs and were never read until now:
36 on 2012-06-15, 215 on 2013-04-05, 280 on 2012-08-20. So some past runs were
starved too, and every number measured beside a high count deserves the same
suspicion as this one. `rerun_baseline.sh` now records the count per sequence
beside the ATE for exactly this reason. Read it before the ATE, along with the
achieved Hz, and read the control delta before either.
