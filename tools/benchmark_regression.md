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

## Working on the machine during a playback: what it does and does not do (2026-10-08/09)

**The claim first written here on 2026-10-08 was WRONG and is withdrawn.** It said that
working on this machine during a playback starved robot_localization and moved the control
by 148%, voiding the run. The first half is real. The second is not, and a clean re-run
disproved it the same night.

2012-06-15 was run twice, hours apart, with nothing changed but whether this session was
using the machine:

```
                        starvation   RL control ATE   FusionCore ATE   achieved Hz
  afternoon, machine in use     62         45.772           78.112          97.9
  night, machine left alone      0         45.864           82.011          99.9
                                          +0.20%           +4.99%
```

The control reproduced to **0.20%** across a 62 versus 0 difference in starvation events.
Whatever those events are, they did not move the measurement. FusionCore's 4.99% sits
inside its documented 5.8% spread on this sequence, so it moved no more than it moves
anyway.

**A second error in the same note:** robot_localization was described as starved down to
19.7 Hz against FusionCore's 97.9. It was not. 20 Hz is simply the rate it runs at here,
and it logged 65325 poses in the first run against 66158 in the second, 1.28% apart. Two
different configured rates were read as a shortfall.

**What actually moved the control, and it is far more important.** robot_localization runs
no FusionCore code, so the only thing that can change its score is its INPUT. The input did
change: #169 corrected the frame of the wheel-odometry yaw rate the player publishes, and
both filters consume it. Every entry in benchmark_baseline.json is marked `pre_frame_fix`
for exactly this reason. Measured on the three baseline sequences:

```
  sequence      RL before   RL after    change
  2012-06-15      18.487     45.864    +148.1%
  2012-08-20      10.519     10.395      -1.2%
  2013-04-05     266.700     65.776     -75.3%
```

So the control is only an integrity check WHEN THE INPUT IS UNCHANGED. Comparing across a
deliberate change to the data the player emits, a moving control is the expected result and
not a fault. rerun_baseline.sh prints "CONTROL MOVED, FC DELTA NOT ATTRIBUTABLE" on two of
these three, and on this particular comparison that warning is correct about attribution
and misleading about cause. Read it as "the baseline predates a change to the input", not
as "the run was contaminated".

**What still stands, and is still worth doing.** 54 starvation events in 31 worked minutes
against 1 in 19 idle minutes is real and timestamped. Leaving the machine alone produced 0
events across all three sequences. Starvation is a cost worth avoiding and a signal worth
recording, and the 2026-10-05 incident where a run was voided after polling with
`ros2 topic hz` is a different and heavier kind of load than reading a log file. But the
rule is now stated at its measured strength rather than above it: **keep off the machine,
and do not attribute a moved control to it without a paired run.**

**The method lesson, which is the one to keep.** The afternoon's conclusion came from a
single run plus a plausible mechanism, and the repo already had a rule against exactly
that: "when a repeat disagrees with the first run, the next move is another repeat, not a
theory." A theory was published instead, into this file and into the project memory, within
an hour of the observation. The repeat cost nothing and settled it.

