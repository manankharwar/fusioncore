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
