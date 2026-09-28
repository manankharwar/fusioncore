# Golden NCLT reference, 2026-09-28

The three staged sequences, all re-measured on one commit with one config, with
robot_localization as the control on every run. This exists because
`tools/benchmark_baseline.json` records numbers without recording what produced them, so
a disagreement could not be resolved. On 2026-09-27 that cost a day: 2012-08-20 had
regressed from 121.669 m to 192.923 m and the only way to find out was to re-run it.

## What produced these

```
  commit            feat/track-heading-angle-gate at ff84820
  config            fusioncore_datasets/config/nclt_fusioncore.yaml, unmodified
  playback rate     1.0x, which is the only rate these numbers are valid at
  harness           tools/run_nclt.sh <sequence> <out_dir> 1.0
  metrics           tools/evaluate.py via evo 1.37.1
```

The lever arm reads `explicitly ZERO, so the TF auto-resolve is skipped` in all three
launch logs. Before `63482b0` it read `auto-resolved from TF ... z=0.300 m`, which is the
regression in #148.

## Results

```
  sequence      FC ATE XY   baseline   change   gate      RL ATE XY   filter Hz
  2012-06-15       71.548     69.425    +3.1%   ok           17.407       99.7
  2012-08-20      128.256    121.669    +5.4%   ok            9.883      100.0
  2013-04-05      189.605    200.910    -5.6%   ok          268.443      103.2
```

None starved. RL matched its own recorded baseline on every sequence, which is what makes
the FusionCore numbers comparable rather than a harness artefact.

**Read the shape, not just the ranking.** FusionCore wins the sequence with no long
blackout (189.6 against 268.4) and loses 4x and 13x on the two that have them. It is not
uniformly behind; it is behind where it has to dead-reckon. The mechanism is measured in
#150: position advances at 87% of a perfect velocity at t=5 s and decays to 38% by
t=150 s, while velocity and yaw both stay perfect.

## What is NOT here, and why

The input bags (849 MB for 2012-08-20 alone) and the `.tum` trajectories (46 MB each) are
too large to commit and are excluded by `benchmarks/.gitignore`. They live in
`~/nclt_out/<run>/` on the machine that produced them. Regenerate with the harness above;
the bags now record the filter's inputs as of `a3a986e`, so a future run is reproducible
from its own bag in a way these were not.

## How to use this

CI gates any `metrics.json` under `benchmarks/nclt/*/results*/metrics.json`. These files
are named differently on purpose: they are a REFERENCE for comparison by hand, not a gate
input, because gating a result against itself proves nothing. To check a new run:

```
  python3 tools/check_benchmark_regression.py <out_dir>/res/metrics.json
```

If a number here is ever beaten or regressed, replace this directory wholesale rather
than editing a value in it. A baseline that has been edited in place is not a baseline.
