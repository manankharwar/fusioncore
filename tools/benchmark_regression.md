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


---

## 2026-10-17: 2012-02-04's tight spread was two samples that landed together

`benchmark_baseline.json` recorded 2012-02-04 at **n=2 with a 0.51% spread** and said in
as many words that "a few percent is meaningful here". A third run with the same shipped
config, on `6537cf3`, read **134.045** against the recorded 153.883 and 153.097.

```
  arm                   run 1      run 2      run 3     spread
  FusionCore          153.883    153.097    134.045    14.80%
  robot_localization  123.131    123.149    123.121     0.02%
```

The control holding at 0.02% across all three is what places the 14.80% in FusionCore's
own column rather than in the harness or the input. **29x wider than the figure this file
asserted for eight days.**

The lesson is narrow and it is not "FusionCore is noisy". It is that **a tight spread at
n=2 is not evidence of a tight spread.** Two samples landing close together is the
expected outcome a good fraction of the time even from a wide distribution, and nothing
about the first two runs looked wrong. Every n=2 entry in the baseline file carries the
same exposure, and the ones with a tight recorded spread carry it most, because a tight
spread is what licenses reading a small delta as real.

Practical consequences, and the first one is the expensive one:

- **Any A/B on 2012-02-04 needs repeats per arm.** At n=1 per arm a 14.8% difference
  means nothing on this sequence. The 2026-10-16 profile run read 32.422 there, a 78.8%
  improvement, and it cannot be graded.
- The gate threshold is 10%, so this sequence cannot be graded by
  `check_benchmark_regression.py` at all until n grows per arm.
- **The 272.258 control from that same profile run is a rare failure mode, not the
  control's operating point.** Three clean runs put it at 123.1. A control that moves
  121% is still a reason to void a run; it is not a reason to believe the control
  changed.

One further observation, recorded without being used: run 3 logged **32 robot_localization
starvation events** against 0 in the first two, clustered and matching no work done on the
machine, so the load was external. Its control moved 0.02% and its pose count held, so it
was not starved in output. Attributing the FusionCore move to machine load would repeat
the 2026-10-08 mistake documented above, where a paired run measured 62 starvations
against 0 and moved the control 0.20%.

---

## 2026-10-17: no input feature separates the wins from the losses

FusionCore wins 4 of the 9 sequences measured on corrected input and loses 5, with
baseline accuracy around 7 m on the wins and 23 to 46 m on the losses. That gap is the
oldest open question here, and no field, platform or vertical claim is supportable until
it is explained, because without it you cannot say which conditions FusionCore is good
in.

`tools/winloss_features.py` puts the verdict beside fifteen candidate columns, computed
from the raw data rather than from a run: motion features from `odometry_mu_100hz.csv`,
which is what `nclt_player.py` actually feeds the filter, and GNSS-quality features from
`gps.csv`. It prints an explicit separation check, because a table of 9 rows and 15
columns invites finding a pattern that is not there.

```
  sequence     verd        FC       RL |    min  path_m  stop%  nstops | strght%  rot_deg   med_w
  2012-03-31   WIN       28.5    160.6 |     88    6094    8.1      38 |    44.6    36965   0.064
  2012-09-28   WIN        8.9     91.9 |     78    5680    5.9      17 |    42.8    34602   0.069
  2012-10-28   WIN       17.1     25.1 |     86    5821    7.8      33 |    45.6    34034   0.058
  2013-04-05   WIN        9.2     65.8 |     70    4675    9.8      37 |    48.3    23389   0.049
  2012-01-08   loss     185.2     22.8 |     94    6545    7.2      24 |    46.1    37565   0.057
  2012-02-04   loss     147.0    123.1 |     79    5625    6.6      16 |    45.3    32429   0.060
  2012-05-11   loss       9.6      8.9 |     85    6144    6.0      21 |    42.3    40530   0.074
  2012-06-15   loss      32.1     45.8 |     55    4129    5.9      14 |    44.9    22593   0.060
  2012-08-20   loss     157.6     10.4 |     84    6106    6.3      17 |    42.8    35878   0.068
```

**Every column overlaps. Nothing separates.** Stops, stop count, longest stop, straight
fraction, total rotation, rotation per metre, median yaw rate, path length, duration, max
GNSS gap, gaps over 60 s, usable-fix percentage, mode-flip rate and fix count: in all
fifteen the win range and the loss range intersect, usually heavily. Straight fraction is
42.8 to 48.3% on the wins and 42.3 to 46.1% on the losses. Median yaw rate is 0.05 to
0.07 rad/s on both.

**Which makes sense once said out loud: NCLT is one Segway driving one campus across
fifteen months.** The sequences are near-interchangeable by bulk motion statistics, so
the thing that turns a 9 m result into a 185 m result is not visible in them.

Three consequences.

**The field question cannot be answered from NCLT.** Picking a vertical needs to know
which conditions FusionCore is good in, and this dataset does not vary enough in the
right dimensions to tell you. That has to come from bags off other robots, which makes
the outstanding asks to Swap Robotics, Joseph at Upside and Aigen the path to the answer
rather than a parallel activity.

**The remaining hypothesis is filter-internal, not conditional.** FusionCore is 2.5x to
7.6x worse in the MEDIAN on the losses, not only in RMSE (23.37 against 9.18, 33.65
against 4.44, 46.12 against 22.09), so it is holding a sustained wrong state rather than
taking one excursion and recovering. A sustained offset with no input feature to explain
it points at the filter entering a bad state and staying there, which is #150.

**None of these runs gives the filter much chance to observe bias while stationary.**
Contiguous stops of 2 s or longer total 59 to 183 s out of 55 to 94 minutes, on every
sequence, wins and losses alike. So "it stopped more often" cannot be the explanation
either, and ZUPT is doing less work here than its presence suggests.

---

## 2026-10-18 (SUPERSEDED BELOW): the attitude bug was NOT the NCLT lock-in

The exact-log-map fix made heading converge in a controlled test where it had previously
random-walked the whole circle. The obvious question was whether that bug was also what
locks a losing NCLT sequence into a wrong state, since a covariance that lost 40% of
itself on every rebuild is exactly the precondition for a filter that is confident and
wrong.

It is not. 2012-08-20, the 15x loss, run through the deterministic core replay as a 2x2
over the attitude flag and the `ground_robot` bias freeze:

```
  cell          gnss rejects      path_m      ATE RMSE 3D
  OFF_free              2121    10586.56         171.393
  ON_free               2299    25011.02         158.843
  OFF_frozen            2070     9771.16         145.765
  ON_frozen             2759     9789.83         166.135
```

Encoder reference path is 6105.75 m. Nothing drops toward the 7 m that the winning
sequences achieve. The best cell is the bias freeze alone at 145.765, about 15% better
than baseline, which is the same order the profile already measured through the node.

**So two suspects are cleared and the bistability question stays open.** The attitude
representation was a real bug with a real controlled-test fix, and it is not what makes
a sequence lose.

One warning sign against turning the flag on: **ON_free produced a 25,011 m fused path
for a 6,106 m drive**, four times the true distance, while its ATE happened to read
lower. A lower ATE on a trajectory that long is not an improvement, and it is another
reason the flag stays off until the turning error is understood.

Caveats, stated because the numbers invite over-reading. This is **n=1 per cell on one
sequence**. Replay ATE is not comparable to node ATE on the same data: the node scored
157.591 here while the replay baseline is 171.393, because `export_bag.py` flattens
lat/lon with its own conversion while the node uses PROJ ECEF to ENU, which is a known
and documented gap. The comparison across cells is valid because all four share one
exported log and one projection.

`tools/yaml_to_replay_args.py` was written for this and is the supported way to drive an
offline A/B with the same configuration the node would have used.

---

## 2026-10-18, corrected: the attitude flag is ACTIVELY HARMFUL on real data

The section above measured one sequence, in ATE RMSE, against a target quoted in
MEDIAN. Those are different metrics and the comparison as framed was invalid. The data
was fine; the framing was mine and wrong, which is the third summarisation error in two
days and the reason the rule about pasting raw numbers now exists.

Re-scored with every metric the scorer already reported, and with a WIN added so there
is something to compare a loss against:

```
  2012-08-20 (LOSS, node 157.591, node median 33.65)
  cell              RMSE     median   path_ratio    drift
  OFF_free       171.393     35.436        0.888    20.24
  ON_free        158.843     30.012        0.614    12.96
  OFF_frozen     145.765     31.618        0.967    18.76
  ON_frozen      166.135     30.660        0.940    20.77

  2013-04-05 (WIN, node 9.193, node median 7.31)
  cell              RMSE     median   path_ratio    drift
  OFF_free        16.677      6.960        1.019     3.17
  ON_free         64.159     11.029        0.171     2.04
  OFF_frozen      15.355      7.001        1.073     3.07
  ON_frozen       27.444      8.165        1.049     5.37
```

**The attitude flag makes the WIN 3.8x worse**, 16.677 to 64.159, with the median going
6.960 to 11.029. And the path length ratio collapses to **0.171**, meaning the
trajectory is 83% too short.

**That reframes the apparent improvement on the loss.** ON_free looked better there
(171.393 to 158.843) only because the loss is already bad enough to hide the same
degeneracy: its path ratio is 0.614. The flag causes systematic UNDER-TRAVEL on both
sequences, and ATE flattered it on the one where there was more room to hide.

Without the path-length ratio beside ATE this would have been recorded as a partial
improvement. It is not. **Report the ratio next to ATE on every benchmark from now on.**
A ratio far from 1.0 catches jitter, oscillation and degeneracy no matter how the SE3
alignment happens to land.

**OFF_frozen is the best cell on both sequences**: 145.765 at ratio 0.967 on the loss,
15.355 at ratio 1.073 on the win. The `ground_robot` bias freeze helps a loss and a win
alike, which is consistent with what the profile already measured through the node.

**So the attitude flag stays off, and not merely pending the turning work.** It has a
systematic under-travel problem on real data. The likely mechanism is the one this filter
has had all along: the exact log map no longer under-reports large rotations, so the
attitude covariance is correctly LARGER, the sigma points spread wider, and averaging
`R(q)*v*dt` across a wider attitude cloud cancels more forward displacement. The
small-angle bug was partially masking the cancellation defect by shrinking the covariance.

Both conclusions from the superseded section survive: the attitude bug was not the NCLT
lock-in, and the bistability question is still open.


---

## 2026-10-18: the old tuning WAS compensating for the leak, and a cheap bistability repro

Path length ratio has a closed form under heading uncertainty: averaging `R(psi)*v` over
a yaw spread of sigma gives `v*exp(-sigma^2/2)`, so the ratio measures yaw sigma
directly. A ratio of 0.171 implies **sigma = 107.7 degrees**, which is approaching
uniform. So the under-travel is not a defect in the averaging. It is correct averaging
fed a covariance that is far too large.

Hypothesis tested: `ukf.q_angular_vel` was tuned against a covariance that leaked up to
40% on every rebuild, so with the leak fixed the same value inflates without limit.

2013-04-05, attitude flag ON, bias free, sweeping only `ukf.q_angular_vel`:

```
  q_angular_vel      RMSE     median   path_ratio   implied yaw sigma
  0.1  (shipped)   64.159     11.029        0.171       107.7 deg
  0.01             26.696      7.832        1.053         0.0 deg
  0.001            41.131      8.403        0.302        88.7 deg
  1e-4             28.673      7.856        0.174       107.2 deg
  1e-5             31.042      8.067        0.997         4.4 deg
```

**Confirmed.** Dropping the shipped 0.1 to 0.01 takes the path ratio from 0.171 to
1.053, RMSE from 64.159 to 26.696 and the median from 11.029 to 7.832. A parameter
tuned around a bug became wrong the moment the bug was fixed.

**But look at the column, because it is not monotonic.** 0.01 is healthy, 0.001 is
degenerate, 1e-4 is degenerate, 1e-5 is healthy again. Three orders of magnitude of
process noise and the filter flips between a path ratio near 1.0 and one near 0.17, with
nothing in between.

**That is the bistability, reproduced in seconds instead of 90 minutes.** The same input,
the same code, a small parameter change, and two qualitatively different outcomes: one
tracking, one with yaw sigma near 108 degrees. Up to now bistability was an inference
from the fact that no input feature separates the wins from the losses. This is a
controlled, cheap, repeatable handle on it.

**It does not rescue the flag.** The best ON cell is 26.696 against 16.677 for the
shipped default with the flag off, so the flag still loses on this sequence even when
tuned. It stays off.

The useful output is the repro, not the tuning. Any future work on the wrong-basin
question should start here, because it costs seconds per sample.

---

## 2026-10-18: the outlier gate IS the discrete switch, and it protects rather than locks out

Step 1 of the twin-run plan: rerun the `q_angular_vel` sweep with all outlier gates
disabled. If the flipping disappears, the gate is the switch.

2013-04-05, attitude flag ON, bias free, only `ukf.q_angular_vel` and
`outlier_rejection` changed:

```
  gates ON                                        gates OFF
  q_ang_vel     RMSE   median   ratio  rejects    RMSE   median   ratio  rejects
  0.1         64.159   11.029   0.171       61   271.480  42.400  0.153       12
  0.01        26.696    7.832   1.053      145   259.383  40.609  0.161       12
  0.001       41.131    8.403   0.302      101   238.688  35.816  0.599       12
  1e-4        28.673    7.856   0.174       68   251.513  40.034  0.184       12
  1e-5        31.042    8.067   0.997      147   227.750  36.445  0.289       12
```

**The flipping disappears with the gates off.** Every cell is uniformly degenerate,
227 to 271 m, ratio 0.15 to 0.60, and the healthy outcomes at q = 0.01 and 1e-5 are
gone. So the gate is the discrete switch, exactly as predicted.

**But the direction is the opposite of the lock-out theory, and the rejection counts say
so plainly.** With the gates on, the HEALTHY runs reject MORE: 145 and 147 rejections at
q = 0.01 and 1e-5, against 61, 68 and 101 in the degenerate ones. A locked-out filter
would show the reverse, long streaks of rejection in the bad runs.

The mechanism that fits every column: the Mahalanobis test normalises by
`S = HPH' + R`. In a degenerate run the attitude covariance is enormous, so S is
enormous, so the gate is effectively wide open and accepts bad fixes (61 rejections).
In a healthy run P is sane, S is sane, the gate does its job (147 rejections) and the
filter stays tracking. **The gate is not causing the lock-in. It is the thing preventing
it, and it stops working precisely when the covariance it normalises by has blown up.**

That makes it self-reinforcing, which is what a bistability needs: a big P disables the
only mechanism that would have kept P small.

**Consequence for the fix.** A recovery path after N consecutive rejections, the obvious
remedy under the lock-out theory, would have been aimed at the wrong end of the loop:
the bad runs are not rejecting enough to trigger it. The lever is bounding the attitude
covariance so the gate keeps working, which is also what the `ground_robot` freeze does
and is consistent with OFF_frozen being the best cell on both a win and a loss.

Still to do on this thread: a fine sweep between 0.01 and 0.001 to find neighbouring
healthy and degenerate values, then a twin-run diff logging every discrete decision with
a timestamp to find the first one that differs. And the same on a losing sequence with
the flag OFF, to confirm it is the same switch rather than one the flag created.

Caveat by the rule: "implied yaw sigma" in the section above is an INFERENCE from path
ratio, not a measurement, and path ratio also moves with jitter. Yaw sigma should be
logged from P directly before that number is quoted again.

---

## 2026-10-18: twin-run diff. P blows up FIRST, and no rejection cascade is involved

The fine sweep between 0.001 and 0.01 gives adjacent twins that differ by 33% in one
parameter and land in different worlds:

```
  q_angular_vel     RMSE   median   path_ratio   rejects
  0.001           41.131    8.403        0.302       101
  0.0015          26.096    7.619        1.001       143   HEALTHY
  0.002           67.471   11.997        0.173        67   DEGENERATE
  0.003           28.410    7.475        0.303       111
  0.004           32.709    7.688        1.002       148
  0.005           29.892    8.044        0.992       146
  0.0065          34.443    8.490        0.173        72
  0.008           43.234    8.712        0.192       112
  0.01            26.696    7.832        1.053       145
```

Path ratio is bimodal: every run is either about 1.00 or about 0.2, with nothing
between. Healthy runs reject 143 to 148, degenerate ones 67 to 112.

Running the 0.0015 / 0.002 twins with per-fix logging:

```
    fix#   t_rel    sep_m   sigma_h   sigma_d   acc_h   acc_d
       0     0.0    0.000    1.0694    1.0694       1       1
     100    22.6    0.013    1.8174    1.7065       1       1
     450    92.6    0.548    1.4135    1.5508       1       1
    1000   206.5    0.001    1.7572    1.7426       1       1
    1500   362.2    0.751    1.5058    2.9761       1       1   <- sigma doubles
    2000   464.2    0.574    1.3803    2.9405       1       1
    2500   564.4    0.880    1.7077    2.9498       1       1
    2576   579.6   13.559    1.5301    2.9337       0       1   <- first disagreement
```

**Three findings, and all three contradict the obvious story.**

**1. Both twins accepted EVERY fix for 2576 fixes. Zero rejections in either, right up to
the first disagreement.** There is no rejection cascade, no lockout, no latch. Whatever
causes this, it is not the gate throwing fixes away.

**2. The covariance diverges 217 seconds before the position does.** The degenerate
twin's sigma doubles from about 1.4 to 2.94 at fix 1500, t = 362 s, and stays there,
while the healthy twin holds 1.4 to 1.7. Position separation is still under 1 m at that
point and only jumps to 13.6 m at fix 2576, t = 580 s.

**3. So P blows up FIRST.** This answers the question the two levers hung on. It is not a
bad fix accepted while P was sane and then wrecking everything. It is the covariance
growing, which widens the gate, which lets through a fix the healthy twin rejects
(mah2 0.13 against 16.58 on the same fix, threshold 16.27).

**The lever is therefore bounding covariance growth at its source, not the GNSS
measurement model.** That is the `ground_robot` bias freeze now and IMU-as-input later,
and it is consistent with OFF_frozen winning on both a win and a loss.

The separate point about R stands on its own evidence and is unaffected: with gates off,
even a sane-P run at q = 0.01 collapses from 26.696 to 259.383, so some NCLT fixes are
far worse than the covariance they declare. That is a second, independent defect.

What is still not known is what happens at t = 362 s to double the covariance. That is
the next thing to look at, and the twins make it a seconds-long experiment.

---

## OPEN DEFECT: NCLT GNSS fixes are worse than the covariance they declare

Filed separately because it is independent of the bistability above and has its own
evidence.

With all outlier gates disabled, a run whose covariance is demonstrably sane collapses
anyway. 2013-04-05, attitude flag ON, `q_angular_vel = 0.01`, the healthiest cell in the
sweep:

```
  gates ON    RMSE  26.696   median  7.832   path ratio 1.053   rejects 145
  gates OFF   RMSE 259.383   median 40.609   path ratio 0.161   rejects  12
```

A 9.7x degradation with no change to the filter's tuning. The chi-squared gate is the
only thing standing between the filter and a set of fixes whose declared covariance does
not describe them. The record already carries the extreme case: on 2012-08-20 two raw
fixes 0.07 s apart differ by 825 m, which is 11789 m/s on a 1.5 m/s Segway, and no
reported sigma describes that.

**Consequence: the filter currently depends on the gate for correctness, not just for
robustness.** That is a fragile place to be, because the same experiment shows the gate
stops working when the covariance it normalises by grows.

The honest fix is on the measurement side rather than the filter side: set R from the
quality the receiver actually reports, reject degraded fix modes outright, and
cross-check successive fixes against each other and against odometry before trusting
them. Any absolute innovation limit added here must come with re-anchoring when several
successive fixes agree with each other but disagree with the filter, because that means
the filter is the wrong one, and without it this rebuilds the latch bug that once
disabled spike rejection for entire runs.

## Agreed plan and stop rule, 2026-10-18

1. **Find what doubles the covariance at t = 362 s.** Rerun the 0.0015 / 0.002 twins on
   2013-04-05 with the flag on, logging every discrete branch (ZUPT on and off, coast
   enter and exit, adaptive-noise switches, wheel-odometry gate) plus yaw sigma straight
   from P and the effective gate radius `sqrt(chi2 * S)` in metres, step by step from
   340 to 380 s. Report the first step where P differs by more than 1% and which branch
   fired at or just before it.
2. **Confirm it is the same loop in the shipped config.** Same twin method on
   2012-08-20 with the flag OFF, sweeping q or the gate threshold until the outcome
   flips, and check the pattern holds: P first, then the gate widens, then a bad fix
   gets through.
3. **Stop rule. After 1 and 2, or two sessions at most, pause the deep dive and ship.**
   Run the `ground_robot` freeze across all six available bags in replay; if it wins on
   most, propose it as the default for the wheeled-robot preset. Fix the projection gap
   so `export_bag.py` and the node share one lat/lon projection. Rerun the benchmarks
   once, then draft the public correction with the updated numbers.

Standing rules for all of it: replay only and never through the live node for any
benchmark; report RMSE, median and path ratio together; the attitude flag stays off.

---

## 2026-10-18, Block 1: there is NO event at 362 s. Divergence starts at 10 s.

`tools/repro/pblowup.cpp` logs the diagonal blocks of P directly, plus yaw sigma computed
from the quaternion covariance rather than inferred from path ratio. Twins q = 0.0015 and
q = 0.002 on 2013-04-05, attitude flag ON.

First step at which each block of P differs by more than 1% between the twins:

```
  block           t_rel        healthy     degenerate
  P_angvel         9.59       0.241693       0.244623
  P_quat          16.63         0.2807         0.2766
  yaw_sig         18.65        55.4783        54.9199
  P_gbias         23.70         0.1329         0.1313
  P_accel         27.21        12.1707        12.3492
  P_pos           28.72        21.0795        22.1537
  P_vel           29.22        31.2951        30.2855
```

**Everything happens in the first 30 seconds, and the first block to move is the one the
swept parameter controls directly.** `q_angular_vel` sets the angular-velocity process
noise, so `P_angvel` separating at 9.59 s is the parameter doing exactly what it says.
It reaches the quaternion 7 s later, yaw sigma 2 s after that, and the rest of the state
within another 10 s.

**So the earlier "sigma doubles at t = 362 s" was wrong, and the error was mine.** That
number came from the `sigma_x` column of the per-fix GNSS CSV, which is the adaptively
scaled measurement noise, not the filter covariance. Measured from P directly, the twins
have already parted by 30 s.

**The far more important finding is in the yaw sigma trajectory, and it applies to BOTH
twins:**

```
    t_rel    healthy      degen
     22.2    52.5504    51.0143
     44.3    44.7697    24.9704
     66.5    55.9493    19.2326
    110.8     8.1878    39.6904
    177.2    32.7259    70.1856
    243.7    43.0086    46.1270
    332.2     4.5278    25.6770
    398.6    13.1652    24.4433
```

**Yaw sigma runs 5 to 70 degrees for the whole run in both twins.** The healthy one is at
55.9 degrees at t = 66.5 s. A filter with continuous 1 Hz GNSS and working wheel odometry
should know its heading to single digits. Neither twin ever does.

**That reframes the whole question.** There is no discrete event to find and no single
branch to instrument. Heading uncertainty is pathological from the first minute, both
twins wander inside that uncertainty, and the outlier gate turns a continuous divergence
into two discrete basins, which is why the outcome is bimodal and why it disappears when
the gates are switched off. Sensitive dependence, not a switch firing at a moment.

The practical consequence is unchanged and is now better supported: **bound the attitude
covariance.** Nothing downstream can be fixed while yaw sigma spends the run between 5
and 70 degrees.

**Caveat, stated because it matters for Block 3:** these runs have the attitude flag ON,
and the shipped configuration has it OFF. Whether the shipped losses go through this same
loop is exactly what Block 3 tests, and it has not been run.

---

## 2026-10-18, Block 2: there is NO projection gap. The premise was wrong.

The plan called for one shared lat/lon projection between `export_bag.py` and the node,
on the belief that their different code paths explained node 157.591 against replay
171.393 on 2012-08-20.

**They are already the same transformation.** The node converts through PROJ with
`input.gnss_crs: EPSG:4326` and `output.crs: EPSG:4978`, which is WGS84 geodetic to
WGS84 ECEF. `export_bag.py` uses the closed form for exactly that. Tested on twelve real
2012-08-20 fixes spanning the track:

```
          lat          lon |  max|diff| m
    42.293916   -83.708331 |    2.910e-10
    42.293949   -83.715033 |    9.313e-10
    42.292802   -83.712655 |    5.821e-11
    42.290485   -83.717180 |    4.657e-10
    42.291835   -83.715965 |    1.746e-10
    42.293924   -83.711118 |    0.000e+00
```

**Worst disagreement anywhere on the track: 9.313e-10 m.** Sub-nanometre. Both paths
then call the same `fusioncore::sensors::ecef_to_enu`. The comment in the node saying
"default behavior is identical to the hand-coded WGS84 to ECEF math" is correct and was
worth believing. (The rows showing nan are fixes whose altitude is nan in the raw NCLT
`gps.csv`, which is a property of the dataset, not of either projection.)

**So the node-versus-replay difference is not a pipeline gap.** Comparing like with like,
the node's frozen-config run scored 157.591 and the replay's OFF_frozen cell scored
145.765, a 7.5% difference.

**That is well inside the node's own measured run-to-run spread.** 2012-02-04 repeated
through the node at 153.883, 153.097 and 134.045, a 14.80% spread on identical input and
an unchanged control. A 7.5% node-to-replay difference needs no explanation beyond the
node being the nondeterministic one.

**Two consequences.**

The acceptance test as written, node and replay agreeing within 0.5%, **is not
achievable, and not because of the projection.** The node does not reproduce itself to
0.5%. Asking the replay to match it to 0.5% asks it to match a number that moves.

**Absolute published numbers should come from the replay, not the node**, for the same
reason the benchmarking rule already says replay only: it is the deterministic path. The
node remains the right integration test, checking that it lands within the replay's
figure plus the node's own known spread.

Block 2 therefore passes with no code change, and Block 4 is unblocked.

---

## 2026-10-18, Block 4: the ground_robot freeze wins 6 of 6 in replay

All six exported sequences through the deterministic core replay, shipped NCLT config,
only the seven `ground_robot` keys changed between arms. Attitude flag OFF throughout.

```
  sequence     arm           RMSE    median   path_ratio    drift
  2012-01-08   free        19.974     7.183        0.848     3.61
  2012-01-08   frozen      19.423     7.123        0.877     3.63
  2012-02-04   free       142.792    34.495        1.055    18.07
  2012-02-04   frozen     142.311    34.582        1.108    18.90
  2012-06-15   free        78.139    29.978        1.113    14.84
  2012-06-15   frozen      64.295    18.164        1.180    12.94
  2012-08-20   free       171.393    35.436        0.888    20.24
  2012-08-20   frozen     145.765    31.618        0.967    18.76
  2012-09-28   free        23.638     7.235        1.048     3.43
  2012-09-28   frozen      22.501     7.075        1.084     3.38
  2013-04-05   free        16.677     6.960        1.019     3.17
  2013-04-05   frozen      15.355     7.001        1.073     3.07
```

**Frozen is better on RMSE on all six**: -2.8%, -0.3%, -17.7%, -15.0%, -4.8%, -7.9%.

**Median is better on four of six**, and the two it loses are 34.582 against 34.495 and
7.001 against 6.960, which are 0.25% and 0.6%. The biggest median gain is 2012-06-15 at
29.978 to **18.164**, a 39% improvement.

**No path-ratio degeneracy anywhere.** Every cell sits between 0.848 and 1.180, and the
freeze moves every ratio upward, which helps the two sequences that were under-travelling
(0.848 to 0.877 and 0.888 to 0.967) and slightly overshoots on the four that were already
above 1.0.

Honest reading. This is **n=1 per cell**, and 2012-02-04 carries a measured 14.80% spread
through the node, so its -0.3% means nothing on its own. The two large gains, -17.7% and
-15.0%, are outside any spread recorded for those sequences. The consistent direction
across six sequences is worth more than any single row.

**What this does and does not justify.** It is strong evidence for recommending
`platform.profile: ground_robot` to anyone running a wheeled robot with encoders, and the
profile README can now say six sequences rather than three. It is **not** on its own a
reason to change the library default from `general`: `fusioncore_core` still has to be
correct for a platform that leaves the ground plane, and the scope statement shipped
today says the library runs on those with nothing frozen. Flipping that default is a
separate decision with a separate blast radius.

---

## 2026-10-18: the WZ/bias pair hypothesis FAILS, and my column label caused it

Predictions were written down before running and are kept in
`tools/repro/wz_bias_predictions.md`. The hypothesis was that `P_angvel = 0.2417` means
the filter cannot pin its rotation rate to better than 28 deg/s while a gyro measures it
at 100 Hz, which is only consistent with the WZ / B_GZ pair at r = -0.998: each huge
individually, their SUM pinned by the gyro, and heading integrating WZ alone.

2013-04-05, t = 100 to 140 s, all four arms:

```
  arm               t_rel   yaw_sig        var_WZ       var_BGZ       var_SUM     r_wb   sum/wz
  flagOFF_free     118.15   22.1178  2.903199e-02  1.718904e-03  2.941712e-02  -0.0944    1.013
  flagOFF_frozen   118.15   10.4647  2.836724e-02  1.310965e-04  2.834013e-02  -0.0410    0.999
  flagON_free      118.15   34.5746  2.888260e-02  1.573027e-03  2.941244e-02  -0.0774    1.018
  flagON_frozen    118.15   11.3186  2.835584e-02  1.206296e-04  2.833968e-02  -0.0370    0.999
```

**P1 FAILS.** `var(WZ + B_GZ)` was predicted to be of order the gyro noise variance,
about 2.5e-5, against a `var(WZ)` of 0.24. Measured, the sum is **2.94e-2 against a WZ
variance of 2.90e-2**, a ratio of **1.013**. The sum is not small. It is the same size as
var(WZ).

**P2 FAILS.** `r(WZ, B_GZ)` was predicted near -1. Measured it runs **-0.6256 down to
-0.0370**, and it is near -0.04 to -0.10 for most of the window. The -0.998 on record was
not measured here and does not describe this run.

**P3 PARTLY HOLDS.** Freezing was predicted to give single-digit yaw sigma. It roughly
HALVES it, 37.7883 to 15.4331 at t = 101 and 22.1178 to 10.4647 at t = 118, but it lands
at 10 to 15 degrees rather than under 10.

**P4 HOLDS.** The pattern is the same with the attitude flag off and on.

**The failure is traceable to my own column label, and that is the fifth time this week.**
My Block 1 table printed a column headed `P_angvel` with the value 0.2417. That column is
`blk(WX,WZ)`, the **sum of the three angular-velocity variances**, not the variance of
WZ. Measured here, `var(WZ)` alone is 0.029. A reader given a column called `P_angvel`
reasonably treats it as a variance, builds a 28 deg/s argument on it, and the argument
inherits my error. Labelling a trace as if it were a variance is the same class of
mistake as reporting a median against an RMSE target.

**What is actually true, and it still points the same way.** `var(WZ)` of 0.029 is a
standard deviation of 0.17 rad/s, about 10 deg/s. Integrated over the seconds between
useful corrections that is tens of degrees of yaw uncertainty, which is the observed 20
to 40. So yaw uncertainty does grow out of angular-rate uncertainty, exactly as the
hypothesis said. What is NOT happening is a precise sum hiding behind two huge
anti-correlated terms. The bias variance contributes directly: freezing it drops
`var(B_GZ)` from 1.7e-3 to 1.3e-4, a factor of 13, and halves yaw sigma.

**That still supports IMU-as-input**, but for a plainer reason than the one proposed.
Integrating `gyro - bias` directly would remove WZ as a free state whose variance grows
between corrections, rather than removing a cancelling pair that is not there.

---

## 2026-10-18, PHASE 1: adapt_R inflates the gyro R by up to 1723x. Two defects found.

Prediction written before measuring, kept in `tools/repro/phase1_predictions.md`: right
after an IMU update, `var(WZ + B_GZ)` should be about `R_gz`. The NCLT config sets
`imu.gyro_noise: 0.003 rad/s`, so **R_gz = 9.000e-06 rad^2/s^2**.

2013-04-05, t = 115 to 125 s, attitude flag off, bias free:

```
  q_angular_vel  adaptive   var(WZ) [rad^2/s^2]  var(WZ+B_GZ)   var(sum)/R_gz   yaw sigma [deg]
  0.1            ON               1.723080e-02   1.551077e-02          1723.4           25.5440
  0.1            OFF              1.089873e-03   8.912238e-06             1.0           26.7427
  0.01           ON               4.200724e-03   2.797061e-03           310.8           24.7926
  0.01           OFF              1.226315e-03   8.905351e-06             1.0           31.3261
  0.001          ON               2.007583e-03   9.173582e-04           101.9           16.1546
  0.001          OFF              2.259952e-03   1.423805e-03           158.2           22.5421
```

**The prediction is confirmed exactly, with adaptive noise OFF: `var(sum)/R_gz = 1.0`.**
8.912238e-06 against a configured 9.000e-06. So the gyro IS being fused, the measurement
model maps WZ correctly, and there is no axis, frame or units bug. That rules out the
third Phase 1 outcome cleanly.

### Defect 1: `adapt_R` has a floor and no ceiling

With adaptive noise ON the same quantity is **1.551077e-02, which is 1723x the
configured R**. `FusionCore::adapt_R` moves R toward the measured innovation covariance
with an exponential average and clamps only from below:

```cpp
  R = (1.0 - config_.adaptive_alpha) * R + config_.adaptive_alpha * C_hat;
  for (int i = 0; i < z_dim; ++i)
    if (R(i,i) < R_floor(i,i)) R(i,i) = R_floor(i,i);   // floor only
```

**That is a positive feedback loop.** The filter is wrong, so innovations are large, so R
grows toward them, so the gyro update carries less weight and the chi-squared gate
(normalised by `S = HPH' + R`) opens wider, so the filter gets wronger. It is bounded
below and unbounded above, and the comment explains the floor while the ceiling was never
considered.

This is the same self-reinforcing shape found from the other direction in the gate
experiment, and it explains why it is self-reinforcing: **a wrong filter manufactures its
own permission to stay wrong.**

### Defect 2: `q_angular_vel` is per STEP and is enormous at 100 Hz

`ukf.hpp` documents `q_angular_vel = 0.1 (rad/s)^2/step` and Q is never scaled by dt. At
100 Hz that injects **10 (rad/s)^2 per second** of angular-rate uncertainty against a
gyro whose own variance is 9e-6.

Even with adaptive noise off and R correct, `var(WZ)` sits at **1.089873e-03**, which is
a standard deviation of 0.033 rad/s or 1.9 deg/s. Integrated over the seconds between
effective corrections that is the 20 to 30 degrees of yaw sigma observed, and the table
shows yaw sigma barely moves when R is fixed (26.7427 at adaptive off against 25.5440
at adaptive on).

**So the two defects are independent.** Fixing R removes the 1723x inflation and the
feedback loop. It does NOT by itself fix yaw sigma, because the per-step process noise
sets a floor on `var(WZ)` regardless.

### Consequence for the plan

The stop rule says: if Phase 1 or 2 finds a large noise bug, fix it and rerun Phase 3
before building anything in Phase 5. **Two large noise bugs were found.** The architecture
work (IMU as input, USQUE) is now explicitly not the next thing to build.

---

## 2026-10-18, PHASE 1 CONCLUDED: two real defects found and fixed, NEITHER causes the losses

Both fixes are implemented behind flags that default off, so the shipped path is byte
identical. 395 tests pass.

### What was confirmed

`var(WZ + B_GZ) / R_gz = 1.0` with adaptation off: 8.912238e-06 against a configured
9.000e-06. **The gyro is fused correctly and the measurement model maps WZ properly.**
No axis, frame or units bug. The third Phase 1 outcome is ruled out.

### Defect 1: `adapt_R` is unbounded above and double counts H*P*H'

Real and measured at **1723x** nominal. Two separate faults in one function: no ceiling,
and `estimate_covariance()` returns `E[nu*nu'] = H*P*H' + R` which is assigned straight
into R, so the filter's own uncertainty is counted as sensor noise. Both now fixable via
`adaptive_r_max_scale` and `adaptive_subtract_hph`.

**But it is not what loses the NCLT sequences.** Turning IMU adaptation off entirely:

```
  sequence      baseline RMSE   imu_adapt_OFF RMSE   median base   median off
  2013-04-05          16.677               16.793         6.960        6.971
  2012-08-20         171.393              174.558        35.436       35.628
  2012-06-15          78.139               78.882        29.978       29.659
```

Under 2% on all three, and slightly worse on two. **The encoder picks up the yaw-rate
slack**, so starving the gyro update of weight costs almost nothing here.

**A global R ceiling is actively harmful.** Setting `adaptive_r_max_scale = 10` across
all sensors sent 2012-08-20 to **16318370.904 m**. GNSS genuinely needs adaptation
headroom, because NCLT fixes really are far worse than their declared covariance, which
is the separate open defect filed above. The ceiling must be per sensor: tight on the
IMU, loose on GNSS.

### Defect 2: Q is per step and never scaled by dt

Real, documented in `ukf.hpp` as `(rad/s)^2/step`, and confirmed numerically: the
predicted steady state `sqrt(q*R)` is sqrt(0.1 * 9e-6) = 9.5e-4 against a measured
1.089873e-03, within 15%. Now fixable via `ukf.q_time_scaled`.

**At an equivalent q it is behaviour preserving**, which is what a correctness fix should
look like: 2013-04-05 goes 16.677 to 16.964 RMSE, 6.960 to 7.005 median. Larger q makes
it worse (24.398 at 3x, 27.674 at 10x).

**The rate-invariance test is INCONCLUSIVE, and not in the predicted direction.**
Decimating the IMU from 100 Hz to 50 Hz:

```
  arm            rate      RMSE     median   path_ratio
  per-step      100Hz    16.677      6.960        1.019
  per-step       50Hz    16.679      7.094        1.027
  dt-scaled     100Hz    16.964      7.005        0.984
  dt-scaled      50Hz    14.392      6.804        0.989
```

**Per-step is already rate invariant here, 16.677 against 16.679.** The prediction was
that it would not be. Two reasons it is a weak test on this data: the encoder runs at
roughly twice the IMU rate and drives most predicts, so halving the IMU removes only
about 13% of predict calls; and the 15% movement in the dt-scaled arm is comparable to
the healthy-band variation already documented on this sequence. A fair rate-invariance
test needs the encoder decimated too, and n greater than 1.

### Conclusion, and it matters for what happens next

**Two genuine correctness defects were found, and neither explains why FusionCore loses
on NCLT.** They are worth fixing because a filter that lets a wrong estimate inflate its
own sensor noise, and one whose behaviour depends on message rate rather than elapsed
time, are both wrong regardless of what they cost on a benchmark.

The stop rule said a large noise bug would mean the architecture work is not needed.
**Two were found and the architecture question is back open**, because neither moved the
result. That is the opposite of the hoped-for outcome and it is the honest one.

---

## 2026-10-18: RAW GNSS BEATS THE FILTER ON ALL SIX SEQUENCES

The dumb baseline nobody had run. Take the GNSS fixes straight out of the exported log,
score them against ground truth as a trajectory, and compare against the fused output.

**First attempt was unfair and the caveat matters.** GNSS-only associates on about 2.9%
of the points the filter does (3947 against 134368 on 2012-01-08), because it only exists
at fix times and is therefore never scored during the blackouts where the filter's error
explodes. Scoring them that way flatters GNSS.

Corrected with `tools/subsample_to_fix_times.py`, which keeps only the filter poses within
50 ms of a GNSS fix, so both are scored on identical points:

```
  sequence      GNSS RMSE   GNSS median |   FC RMSE   FC median |   pairs
  2012-01-08        9.523         7.024 |    20.606       7.116 |    3947
  2012-02-04       58.672        13.817 |   144.903      35.104 |    4970
  2012-06-15        9.046         4.937 |    10.945       4.957 |    3400
  2012-08-20       32.238         7.337 |   176.805      39.923 |    5236
  2012-09-28        8.983         6.289 |    23.956       7.180 |    3990
  2013-04-05        8.562         6.297 |    10.149       6.419 |    3199
```

**Raw GNSS has lower RMSE than the filter on all six, scored on the same points.**

**The medians say something more precise, and more useful.** On four of the six the
medians are within 2%: 7.024 against 7.116, 4.937 against 4.957, 6.289 against 7.180,
6.297 against 6.419. **Typical-case accuracy is the same as doing no filtering at all.**

> **CORRECTED 2026-10-19.** That list is wrong: `6.289 against 7.180` is **14.2%**, not
> within 2%. Computed exactly, of the five valid sequences only **2012-06-15 is within 2%**
> (+0.4%), with 2013-04-05 next at +2.1%; 2012-09-28 is +14.2% and the other two are +154.1%
> and +444.1%. So the pre-fix claim is "one sequence at parity and one close", not four.
> It holds AFTER re-anchoring, where the five run -0.4%, +0.4%, +3.0%, +1.6%, +1.9%: four
> within 2% and all within 3.1%. The conclusion that the RMSE gap is excursions survives,
> because it rests on RMSE being far worse than the median, not on how many medians tie.
The RMSE gap is therefore excursions: the filter matches GNSS most of the time and then
departs badly. On 2012-02-04 and 2012-08-20 it is worse even in the median, 13.817
against 35.104 and 7.337 against 39.923.

### What this reframes

**The sequences FusionCore "wins" are not wins.** 2012-09-28 and 2013-04-05, its two best
results at 8.872 and 9.193 RMSE, are matching a raw GNSS stream that scores 8.983 and
8.562 on its own. Fusion is adding nothing there.

**The split is not GNSS quality.** 2012-01-08 and 2013-04-05 have almost identical GNSS
error, 9.523 and 8.562 RMSE, 7.024 and 6.297 median. FusionCore scores 185.195 on the
first and 9.193 on the second. **Same input quality, 20x different output.** So the
hypothesis that the losses come from those sequences' GNSS being worse is falsified: the
split is created by the filter.

**The published benchmark omits the baseline that beats both filters.** The table compares
FusionCore against robot_localization and reports who wins. On this evidence raw GNSS,
interpolated and unfiltered, has lower RMSE than both on most of these sequences. Any
public correction has to carry that, because it is the first thing a careful reader would
check and it is not currently anywhere in the repository.

### The remaining honest caveat

This scores only where GNSS exists. It says nothing about the blackouts, which is the case
fusion is actually for and where raw GNSS has no answer at all. So the correct statement
is narrow and still damaging: **where GNSS is available, fusion is not improving on it,
and is adding excursions.** Quantifying the blackout case needs a different experiment,
scoring only inside the gaps, and that has not been run.

---

## 2026-10-18, CORRECTION: the 20x claim was wrong, and 2012-01-08's replay data is invalid

Two errors in the section above, both mine, found by a label check.

### Error 1: the "20x" mixed three different things

I wrote that 2012-01-08 and 2013-04-05 have near-identical GNSS quality while FusionCore
scores "185.195 and 9.193, same input quality, 20x different output". Those two numbers
are not comparable to each other or to the table they sat next to:

```
  185.195   node run, full12 2026-10-12, FULL trajectory, shipped config
    9.193   node run, frozen-config A/B, FULL trajectory, ground_robot profile
   20.606   replay, AT GNSS FIX TIMES, free arm      <- what the table actually showed
   10.149   replay, AT GNSS FIX TIMES, free arm
```

Different pipelines, different configs, different scoring points. **At fix times in one
pipeline the ratio is 2.0x, not 20x.**

### Error 2: 2012-01-08's exported log is TRUNCATED

It was exported from `profile4_20261017_1420`, which is the run stopped partway through on
request. Measured span against the sequence duration:

```
  sequence      log span   expected   coverage
  2012-01-08        3740       5640        66%    <- INVALID
  2012-02-04        4711       4740        99%
  2012-06-15        3308       3300       100%
  2012-08-20        5030       5040       100%
  2012-09-28        4681       4680       100%
  2013-04-05        4181       4200       100%
```

**Every 2012-01-08 replay number reported today is on two thirds of the sequence** and is
withdrawn: the free/frozen row (19.974 and 19.423) and the GNSS-only row. No complete
2012-01-08 input bag exists, because input recording postdates that run, so it cannot be
re-measured without a node run. The other five sequences are 99 to 100% and stand.

### The finding survives, at 4.7x, without 2012-01-08

Like for like, replay pipeline, free arm, full trajectory:

```
  sequence      GNSS RMSE      FC RMSE
  2012-06-15        9.046       78.139
  2013-04-05        8.562       16.677
```

**GNSS quality within 6% of each other, FusionCore 4.7x apart.** The conclusion is
unchanged and the magnitude is smaller: the split is created by the filter, not inherited
from the input.

### A fairer reading of "median equals raw GNSS"

Consumer GNSS error is strongly time correlated, multipath and slow biases rather than
white noise, and no amount of odometry averages away a slow bias without a second
absolute reference. **So matching raw GNSS in the median may be the ceiling, not a
failure.** Stated as three separable jobs:

1. **Do not make it worse where GNSS is good.** Medians pass on four of five valid
   sequences.
2. **Reject outliers without creating excursions of your own.** FAILS: RMSE is above raw
   GNSS on every sequence while the median is not.
3. **Bound drift during blackouts.** NOT MEASURED. This is the actual value proposition
   and there is still no number for it.

That is a narrower and more useful statement than "fusion adds nothing", and it is what
the public correction should say.

---

## 2026-10-18, CORRECTED AGAIN: two failure types, and the split is NOT created at fix times

The 4.7x in the section above mixed scoring points a second time: it put a **fix-times**
GNSS number beside a **full-trajectory** FusionCore number. Raw GNSS only exists at fix
times, so a full-trajectory GNSS score is not defined at all. Like for like:

```
  sequence      GNSS @fix    FC @fix    FC full_traj
  2012-06-15        9.046     10.945          78.139
  2013-04-05        8.562     10.149          16.677
```

**At fix times FusionCore is nearly identical on those two sequences, 10.945 against
10.149.** So on 2012-06-15 the filter is NOT creating the split while GNSS is present.
The 78.139 against 16.677 gap has to come from the time when GNSS is absent. **It is
losing it in the gaps.**

That withdraws "the split is created by the filter while GNSS is present" as a general
claim. It splits into two distinct failures:

**Type A, blackout drift. 2012-06-15.** Fine at fix times (10.945 against a GNSS 9.046),
bad over the full trajectory (78.139). The error accumulates inside the gaps.

**Type B, excursions with GNSS present. 2012-08-20 and 2012-02-04.** Bad even at fix
times: 176.805 against a GNSS 32.238, and 144.903 against 58.672. This is the shape
predicted for failed re-acquisition: the filter drifts during a gap, then the returning
fixes are far from its prediction, so either the gate rejects them or an inflated R pulls
it back slowly, and the excursion lasts until it reconverges.

The two need different fixes. Type A needs better coasting. Type B needs re-acquisition
and re-anchoring. Neither is an attitude representation problem.

### The blackout experiment, with predictions written first

For every GNSS gap of 10 s or more, record error at gap start, peak error, error at gap
end, the drift curve against time since the last fix, and **re-convergence time**, the
seconds after the gap before error returns to GNSS-level. Same for four dumb baselines:
hold last fix, constant-velocity extrapolation, encoder plus gyro dead reckoning, and
robot_localization.

```
  P1  On 2012-06-15 most of FusionCore's full-trajectory error sits INSIDE the gaps.
  P2  On 2012-08-20 most of it sits in the RE-CONVERGENCE windows just after the gaps.
  P3  If FusionCore does not clearly beat encoder-plus-gyro dead reckoning inside the
      gaps, the inertial coast mode is adding nothing.
```

### Process: every number now carries its provenance

`tools/provenance.py`. A measurement carries four fields and two of them refuse to be
compared unless all four match: **pipeline** (node, replay, sim), **config**, **scoring
points** (full_traj, fix_times, gaps, window), and **coverage** as a fraction of the
sequence.

Seven comparisons were published wrong this week and **not one was a wrong number**.
Every one was a correct number compared against a number from somewhere else. A
raw-numbers rule cannot catch that, because the numbers were raw and correct. Replaying
the structural failures through the rule:

```
  "185.195 vs 9.193, 20x"   CAUGHT: config full12_shipped vs frozen_profile
  "GNSS 9.046 vs FC 78.139" CAUGHT: scoring points fix_times vs full_traj
  truncated 2012-01-08      CAUGHT: covers only 66% of its sequence
  GNSS 9.046 vs FC 10.945   ALLOWED, ratio 0.826
```

Coverage is a field because of the truncated log: anything under 99% refuses to be
compared at all.

---

## 2026-10-18, THE ANSWER: 75 to 97% of the error is blackouts and failing to recover

`tools/blackout_budget.py`. Windows defined from the RAW fix stream so the
classification does not depend on the filter: a **gap** is no fix for 10 s or more,
**re-convergence** runs from the first fix after a gap until fused error has held under
the GNSS-level reference for 10 s, **steady** is everything else. Each window's share of
TOTAL SQUARED ERROR is reported, so the shares sum to 100% and answer the question
directly.

The reference is the RAW GNSS median against ground truth for that sequence. A first run
used the fused steady median instead, which is the thing under test and far too loose, and
it under-detected re-convergence badly: 2012-08-20 showed 93.4% with a 35.176 m reference
and 94.0% with the correct 7.337 m one, while 2012-02-04 went from 0 detected
re-convergence to 44.5% of the run.

Provenance for every row: `replay / nclt_shipped / windows / 100%`.

```
  sequence      gap %time  gap %err | reconv %time  reconv %err | steady %time  steady %err
  2012-02-04         0.0%      0.0% |       44.5%        94.5% |       55.5%         5.5%
  2012-06-15         8.7%     84.6% |        0.0%         0.0% |       91.3%        15.4%
  2012-08-20         2.2%      2.8% |       22.4%        94.0% |       75.5%         3.2%
  2012-09-28         0.0%      0.0% |       12.3%        86.5% |       87.7%        13.5%
  2013-04-05         1.8%     60.3% |        3.4%        14.5% |       94.9%        25.2%
```

**P1 confirmed.** 2012-06-15: **84.6%** of the squared error is inside the gaps, from
8.7% of the time. One 462 s blackout.

**P2 confirmed.** 2012-08-20: the gaps themselves are **2.8%**, and re-convergence is
**94.0%** from 22.4% of the time. The filter does not lose it in the hole, it loses it
climbing out.

**And it generalises.** On every one of the five, between **74.8% and 96.8%** of the error
lives in gaps or the recovery after them. **Steady coverage never exceeds 25.2%**, and
on the two worst sequences it is 5.5% and 3.2%.

### What this settles

**The filter is fine when GNSS is present.** That is now measured rather than inferred,
and it is consistent with the fix-times comparison where the medians matched raw GNSS.

**The entire problem is blackout handling, and mostly RE-ACQUISITION rather than
coasting.** Three of five are dominated by re-convergence (94.5%, 94.0%, 86.5%), two by
the gaps themselves (84.6%, 60.3%). 2012-02-04 spends **44.5% of the whole run** in a
state of not having recovered.

**This is a logic problem, not an architecture problem.** Nothing here points at the
state vector, the attitude representation, or the sigma-point scheme, which is where this
investigation spent two days. It points at what happens in the seconds after a fix
returns, which this repository has already shipped bugs in twice: the post-outage latch
that never cleared, and the case measured at 1 accepted fix against 1999 rejected while
690 m out.

### What to measure next, to pick the Type B fix

In the re-convergence windows specifically, log gate rejections and adapted GNSS R over
nominal. If the filter is rejecting the returning fixes, the fix is re-anchoring when
several successive fixes agree with each other and with odometry. If it is accepting them
but moving slowly, the fix is the unbounded adaptive R on the GNSS channel, which is the
same defect already found and fixed on the IMU channel and deliberately left alone on
GNSS because GNSS needs headroom.

---

## 2026-10-18, ROOT CAUSE: the gate rejects 81% of the fixes that would rescue it

Branch predictions were written before running and are in
`tools/repro/reconv_predictions.md`. Inside the re-convergence windows only:

```
  sequence      nfix_rc   rej_rc%   rej_all%   med_mah2_rc   med_Rscale_rc   max_Rscale
  2012-02-04       2399     81.0%      10.4%        47.678          253.79       502.40
  2012-08-20       2604     81.2%      10.6%        85.542         1990.28      4907.78
  2012-09-28        842     33.0%       1.5%         0.526            4.70       240.46
```

Provenance: `replay / nclt_shipped / windows / 100%`. The chi-squared threshold is 16.27.

**Branch A confirmed on the two worst sequences, and it is not subtle.** During recovery
the gate rejects **81.0% and 81.2%** of fixes, against 10.4% and 10.6% across the whole
run. An eightfold increase in rejection rate, concentrated exactly where the filter needs
the fixes most.

**Branch B is present as well, and it is the same unbounded `adapt_R` found on the IMU
channel.** The adapted GNSS R reaches **1990x nominal in the median** during 2012-08-20's
recovery windows and **4907x at peak**. On the IMU channel the same code was measured at
1723x. It was deliberately left unbounded on GNSS because GNSS genuinely needs headroom,
and that decision is what lets it reach 4907x.

**Branch C confirmed on 2012-09-28, exactly as predicted.** Only 33% rejected, a median
mahalanobis of **0.526** against a threshold of 16.27, and an R scale of 4.70. **The
fixes look completely fine to the gate and are accepted at near-normal weight, and the
error persists anyway.** That is the heading signature: position is pulled to the fix,
then walks away again because the state left the gap wrong.

**Prediction scorecard: 2 of 3.** 2012-08-20 predicted A, is A. 2012-09-28 predicted C,
is C. **2012-02-04 predicted B, is actually A**: its noisy GNSS made the adaptive-R story
tempting, and the rejection rate of 81.0% says otherwise.

### The mechanism, end to end and measured

1. GNSS blackout. The filter drifts, which is Type A and is only 2.8% of 2012-08-20's
   error.
2. Fixes return. The filter is far from truth, so innovations are enormous: median
   mahalanobis 85.5 against a 16.27 threshold.
3. **The gate rejects 81% of them.** The fixes that would fix it are discarded for
   disagreeing with a filter that is wrong.
4. The ~19% that get through are weighted at up to 1/1990th of nominal, because `adapt_R`
   grew without a ceiling, so they barely move it.
5. Recovery takes 22.4% of the run on 2012-08-20 and **44.5% on 2012-02-04**.

**A wrong filter rejects the evidence that it is wrong, and discounts what little it
accepts.** Both halves are the same failure of self-trust, and the repository has shipped
this family twice before: the post-outage latch that never cleared, and the case measured
at 1 accepted fix against 1999 rejected while 690 m out.

### What the fix has to be, and how to stop it becoming latch bug #3

Re-anchoring, specified so it is bounded by construction:

- Buffer the last N fixes. Require them to agree **with each other** and **with
  odometry**, at acceptable fix quality. Consistency among the fixes is the evidence,
  not agreement with the filter, because the filter is the thing in doubt.
- Re-anchor only after M consecutive large innovations from an otherwise consistent
  buffer. Set position from the fix, inflate `P_pos`, and re-initialise heading from the
  GNSS track when moving, because 2012-09-28 shows position alone is not enough.
- **Time bounded.** Normal gating resumes automatically afterwards, with a test that
  fails if it does not.
- Synthetic first, with injected blackout drift and known truth, covering all three
  branches.
- Acceptance on the five bags: the recovery share of error drops sharply, steady
  coverage does not get worse, no new excursions.

Five NCLT bags is a small set to tune against. Keep thresholds physically motivated, fix
quality and odometry agreement, rather than fitted, and confirm on the rover's own
blackouts before calling it done.

---

## 2026-10-18: fix B alone makes it WORSE. Prediction confirmed, do not ship B without A.

Prediction written first, in `tools/repro/fixB_prediction.md`: the gate tests against
`S = H*P*H' + R`, so the inflated R is part of what lets the ~19% of fixes through during
recovery. Shrinking R without fixing the rejection behaviour should tighten the gate on a
filter that is already rejecting what would rescue it.

2012-08-20, a GNSS R ceiling applied and nothing else changed.
Provenance: `replay / nclt_shipped / windows / 100%`.

```
  r_max_scale      rej_all%   rej_recov%   med_Rscale     final_RMSE
  0 (unbounded)       10.6%        81.2%      1990.28        186.627
  100                 11.9%        88.8%        86.02        175.075
  10                  98.2%       100.0%        10.00    2671751.429
```

**Confirmed.** A ceiling of 100 pushes recovery rejection from 81.2% to **88.8%**. A
ceiling of 10 pushes it to **100%**: every single fix rejected during recovery, 98.2%
across the whole run, and an RMSE of **2,671,751 m**.

This also explains the earlier global-ceiling result that sent this sequence to
16318370.904 m. That was not a GNSS-needs-headroom problem in the abstract. It was this
exact interaction, measured now instead of guessed at.

**So the two halves interact and the order matters.** B alone tightens the gate on a
filter whose problem is that it is already rejecting too much. A alone recovers faster
but then weights every later fix at up to 1/1990th. **Build both, and measure A only,
B only, A+B, and A+B+C separately with the gap / recovery / steady table**, so each
piece's contribution is attributable rather than assumed.

### Getting B right, since a ceiling alone is the wrong tool

- **Fix the estimator before the bound.** R should be `C_hat - H*P*H'`, not `C_hat`.
  Most of the 1990x is the filter's own error being recorded as receiver noise, so
  subtracting the filter's share removes the cause rather than clipping the symptom.
  `adaptive_subtract_hph` exists but currently no caller passes `hph`, so it is inert.
- **Freeze adaptation during recovery**, and whenever a consistency check says the filter
  is the wrong one. Innovations there describe the filter's error, not the receiver's.
- **Set the ceiling from data.** Take the real ratio of GNSS error to declared sigma from
  the Phase 2 noise table and use roughly its 99th percentile, rather than picking 10 or
  100 by hand. Both hand-picked values above are wrong in opposite directions.

---

## 2026-10-18: the defect in its purest form, and it needs no blackout

The adversarial multipath test, `tools/repro/reanchor_multipath.cpp`. A consistent 20 m
GNSS offset for 30 s, continuous healthy coverage, **no blackout anywhere in the run**:

```
  reanchor OFF   0 re-anchors   22.062 m during the offset   129.968 m 40 s after
  reanchor ON    2 re-anchors   21.308 m during the offset     1.226 m 40 s after
```

**Without re-anchoring the filter does not resist the multipath, it never recovers from
it.** 130 m out, from a 30 s offset, with GNSS present and healthy throughout.

The chain, which is the core defect stated as plainly as it can be:

1. While the offset lasts the filter reasonably follows fixes that are mutually
   consistent.
2. **That sideways step looks like sideways motion, so it corrupts heading.**
3. When the fixes snap back to truth they now disagree with the filter, so the gate
   rejects them.
4. The filter coasts on odometry **with a wrong heading**, and keeps going.

**No blackout is required. Any event that moves the filter far enough that good fixes
look like outliers will do it.** That is why 2012-02-04 spends 44.5% of its run in
re-convergence while having ZERO gaps of 10 s or more: there was never a blackout to
recover from.

This stops being a long-gap edge case and becomes the most important defect in the
repository.

### The guard that was proposed and then measured

The expected requirement was that re-anchoring must not fire while the filter looks
healthy, enforced by requiring a recent gap, an elevated `P_pos`, or a one-sided
innovation that began during degraded coverage. The reasoning was sound: a sudden
consistent offset with a confident filter is more likely multipath than drift.

**Measured, that guard would have cost 130 m to save 1.2 m**, because it suppresses the
rescue along with the trap. Not added. The pass criterion is now the OUTCOME rather than
the re-anchor count, since counting fires punishes the behaviour that rescues it.

The re-anchor count stays as a REPORTED metric so a jumpiness regression cannot become
invisible, and the real controller-facing cost, the size of the position jump, is
measured on NCLT rather than assumed away. If the jumps are large the A/B to run is a
soft re-anchor: inflate `P_pos` and yaw `P` and let the normal update pull the state in
over a few fixes, instead of setting the state directly.

## 2026-10-18  Re-anchoring on NCLT: the four arms, all five sequences

Provenance for every number below: pipeline `replay`, config `nclt_shipped` plus the
six free parameters the sweep leaves open, scoring points `full_traj` for the fused
RMSE and `fix_times` for the raw GNSS reference, coverage 100%. The exports live in
`/home/manankharwar/nclt/exported/` and are the re-exported set, not the truncated
2012-01-08 material whose numbers were withdrawn on 2026-10-17.

### The arms

A is `gnss.reanchor_enabled=1`. B is `adaptive.r_max_scale=200 adaptive.subtract_hph=1`.

    sequence     arm          RMSE    median  path_ratio
    2012-02-04   baseline  142.792    34.495       1.055
    2012-02-04   A          61.357    13.893       1.005
    2012-02-04   B         160.648    41.396       1.058
    2012-02-04   AB         61.221    13.858       1.007
    2012-08-20   baseline  171.393    35.436       0.888
    2012-08-20   A          60.674     8.678       0.870
    2012-08-20   B         167.084    34.558       0.890
    2012-08-20   AB         60.299     8.752       0.870

B alone makes 2012-02-04 worse, as predicted on 2026-10-17 before it was run. AB is
within 0.2% of A on both. B contributes nothing once A is present, so the ceiling
stays off and A ships alone.

### A on all five

    sequence     arm          RMSE    median  path_ratio
    2012-02-04   baseline  142.792    34.495       1.055
    2012-02-04   A          61.357    13.893       1.005
    2012-06-15   baseline   78.139    29.978       1.113
    2012-06-15   A          78.109    30.030       1.113
    2012-08-20   baseline  171.393    35.436       0.888
    2012-08-20   A          60.674     8.678       0.870
    2012-09-28   baseline   23.638     7.235       1.048
    2012-09-28   A          13.750     6.553       1.028
    2013-04-05   baseline   16.677     6.960       1.019
    2013-04-05   A          16.005     6.996       1.014

No sequence regresses. Two improve by more than 2x, one by 1.7x, two are flat. This
is the first change this month that improved real data without a single regression,
and the contrast with the attitude flag is the point: that one passed its clean tests
and then cost 3.8x on real data, which is why A was not believed until this table.

### Where the error moved

Windows come from the RAW fix stream. The re-convergence threshold is pinned with
`--ref` to the raw GNSS median per sequence, measured at fix times with the same
alignment and error metric the budget tool uses:

    2012-02-04  14.900   2012-06-15   5.059   2012-08-20   7.152
    2012-09-28   6.484   2013-04-05   5.822

Pinning matters. Without `--ref` the tool falls back to the fused steady median,
which is the thing under test and changes between arms, so %time in recovery is not
comparable across arms. The fallback prints a FALLBACK warning for exactly this
reason.

**CORRECTED 2026-10-22: "reproduce exactly" is wrong and overstated what was measured.**
With the reference pinned, 2012-02-04 and 2012-08-20 DO reproduce the 2026-10-17
classification exactly (94.5%/5.5% and 2.8%/94.0%/3.2%). Two do not:

    sequence      2026-10-17 table        pinned table
    2012-09-28    12.3% / 86.5% rec      8.8% / 85.8% rec
    2013-04-05     3.4% rec, 94.9% std   3.6% rec, 94.6% std

I noticed the 2012-09-28 difference when it was measured, said it came from a different
reference, and then wrote "exactly" anyway. The correct statement is that the two
recovery-dominated sequences reproduce, which is what the pinning was being validated
against, and the other two shift by 0.2 to 3.5 points because the earlier table used a
different reference. That is still evidence the choice is sound, and it is a weaker claim
than the one made.

**The two tables also use two DIFFERENT quantities under one label, "raw GNSS median."**
2012-02-04 reads 13.817 in the fix-times table and 14.900 in the pinned-reference list;
2012-08-20 reads 7.337 and 7.152. Neither is wrong: 13.817/7.337 come from
`subsample_to_fix_times.py` scored with `ate_tum.py` over matched pairs, and 14.900/7.152
come from `raw_gnss_ref.py` over ALL non-zero fixes with blackout_budget's own alignment
and association. Two computations, one name, which is the failure this file exists to
prevent. The pinned windows use the `raw_gnss_ref.py` values and that is the correct
choice for them, because the budget tool must be given a reference computed with its own
metric.

    sequence     arm      | gap %t  gap %e | rec %t  rec %e | std %t  std %e | rec RMSE
    2012-02-04   baseline |   0.0%    0.0% |  44.5%   94.5% |  55.5%    5.5% |  215.799
    2012-02-04   A        |   0.0%    0.0% |  11.5%   94.5% |  88.5%    5.5% |  180.152
    2012-06-15   baseline |   8.7%   84.6% |   0.0%    0.0% |  91.3%   15.4% |    0.000
    2012-06-15   A        |   8.7%   84.6% |   0.0%    0.0% |  91.3%   15.4% |    0.000
    2012-08-20   baseline |   2.2%    2.8% |  22.4%   94.0% |  75.5%    3.2% |  350.265
    2012-08-20   A        |   2.2%   19.2% |  10.9%   78.5% |  87.0%    2.4% |  161.680
    2012-09-28   baseline |   0.0%    0.0% |   8.8%   85.8% |  91.2%   14.2% |   75.269
    2012-09-28   A        |   0.0%    0.1% |   6.7%   60.7% |  93.3%   39.3% |   42.494
    2013-04-05   baseline |   1.8%   60.3% |   3.6%   14.5% |  94.6%   25.2% |   33.362
    2013-04-05   A        |   1.8%   65.4% |   3.7%    7.2% |  94.6%   27.5% |   22.268

The mechanism is visible: on 2012-02-04 the time spent in re-convergence falls 44.5%
to 11.5%, a factor of 3.9, and the re-convergence RMSE falls 215.8 to 180.2. The
error SHARE stays at 94.5% because re-convergence is still where this sequence's
error lives. The RMSE gain comes mostly from spending less time out there, not from
a smaller excursion. On 2012-08-20 both move: 22.4% to 10.9% of time, 350.3 to 161.7
RMSE.

The two flat sequences explain themselves. 2012-06-15 has 84.6% of its error in a
single 462 s gap and 2013-04-05 has 60-65% in its gaps. Re-anchoring acts after a
gap ends, so it cannot touch error accumulated during one. That error is dead
reckoning drift and it is P3's subject, still untouched.

### Does it ever fire when the filter is healthy

`hardware/replay.cpp` now emits a `REANCHOR <t> <jump_m>` line per event, so the
feature's behaviour is read off its own log rather than inferred from an RMSE
difference. (hardware/ is gitignored, so that change is local only.)

    sequence     events  med jump  max jump
    2012-02-04       14     34.88    281.44
    2012-06-15        1     16.84     16.84
    2012-08-20        6     59.89    787.11
    2012-09-28        4    110.57    196.97
    2013-04-05        5     14.79    173.68

Rare: 30 events across roughly 25,000 s of driving. The trajectory's largest
single-step jump equals the largest logged re-anchor on 2012-08-20, 787.11 against
787.10, so the discontinuity IS the re-anchor and not a side effect. It is also not
new behaviour: the baseline's own largest jumps are 414.29 and 716.65 on these two
sequences, so the filter was already making excursions of this size without being
told to.

Each event scored by mean error in the 10 s before against the 2-12 s after:

    sequence     better  WORSE  unscorable
    2012-02-04        7      1           6
    2012-06-15        1      0           0
    2012-08-20        3      0           3
    2012-09-28        2      0           2
    2013-04-05        2      0           3

15 better, 1 worse, 14 unscorable because ground truth is sparse there. Unscorable is
reported as its own column rather than folded into either side; an earlier version of
the script counted them as "better" because `nan > nan` is false, which is the same
class of labelling error as the seven logged on 2026-10-17.

The safety property is the lowest pre-event error across all scorable events: 48.22 m,
against raw GNSS medians of 5 to 15 m. No re-anchor fired while the filter was
healthy. The clearest single event is 2012-08-20 at t=4000.4: a 787 m jump took the
error from 798.07 m to 16.79 m.

### What this does not say

The fused RMSEs here are full-trajectory and the raw GNSS numbers are at fix times.
They are NOT comparable and no claim is made from putting them side by side; the raw
figures appear only as the window reference, which is a legitimate use of them. Raw
GNSS still beats the filter on these sequences at identical scoring points, as
measured on 2026-10-17. A closes part of that gap. It does not close it.

## 2026-10-18  P3 on NCLT: the dataset cannot answer the question

P3 asked whether the filter's coast during a GNSS blackout beats plain dead
reckoning from the same sensors. On 2012-06-15 that matters most, because 84.6% of
that sequence's error sits in a single 462 s gap that re-anchoring cannot reach.

The answer is that NCLT cannot settle it. Ground truth covers 18% of GNSS gap time:

    sequence       gaps    gap_s  truth_s     cov
    2012-02-04        5      511        0       0%
    2012-06-15        1      462      276      60%
    2012-08-20        4      545       87      16%
    2012-09-28        4      431        0       0%
    2013-04-05        5      556       77      14%
    TOTAL            19     2505      440      18%

Three of 19 gaps have enough truth to score. Two sequences have NO truth at all
inside their gaps. This is not a quirk: NCLT's ground truth comes from a SLAM
solution that degrades under the same conditions that take the GNSS out, so truth
goes missing exactly where the blackout question lives.

### A retraction, logged the same way as the seven on 2026-10-17

The first run of this comparison reported that gyro dead reckoning from the filter's
own heading beat the filter's coast by 3.4x on 2012-06-15, 22.2% of distance against
76.1%. That number is withdrawn. Truth has a 185 s hole at the START of the 462 s
gap, so dead reckoning was integrated over the full 462 s while the truth
displacement it was scored against covered only the last 276 s. It was 462 s of
integration measured against 276 s of movement.

Same class of error: right arithmetic, wrong window. The guard now clips the scoring
window to the truth-covered span and skips gaps with an internal truth hole over
30 s. `tools/gap_deadreckon.py` prints the clipped fraction and the largest hole on
every row so the next reader cannot miss it.

A second withdrawn number from the same run: 2012-08-20's 228 s gap appeared to be
the filter's best win, 28.4% against 62-74% for dead reckoning. That gap has a 140 s
truth hole inside it and is now skipped, unscorable.

### What the two scorable windows actually say

    2012-06-15, gap 462s, truth covers 276s (60%), 377.7 m travelled
                            err_m   %dist
      filter coast          425.3   112.6
      imuq @truth yaw       504.9   133.7
      gyro @truth yaw       477.2   126.4
      enc  @truth yaw       501.1   132.7
      enc_neg @truth yaw    510.4   135.1
      gyro @filter yaw      411.1   108.8
      enc  @filter yaw      385.9   102.2
      enc_neg @filter yaw   336.1    89.0

    2013-04-05, gap 275s, truth covers 84s (31%), 114.9 m travelled
                            err_m   %dist
      filter coast          153.6   133.7
      imuq @truth yaw       108.2    94.1
      gyro @truth yaw       115.5   100.5
      enc  @truth yaw       109.1    94.9
      enc_neg @truth yaw     44.5    38.7
      gyro @filter yaw      131.6   114.5
      enc  @filter yaw      140.2   122.0
      enc_neg @filter yaw   176.1   153.2

`@truth yaw` hands dead reckoning the true heading at window entry, which the filter
does not have, so those rows bound the filter's deficit rather than measure it.
`@filter yaw` anchors dead reckoning at the filter's own heading and is the fair
comparison.

Two honest readings, and no more than two. First, every method is catastrophic in
these windows: 89% to 153% of distance travelled. An error larger than the distance
driven means the estimate is worse than asserting the robot never moved. Second,
dead reckoning beats the coast on 2012-06-15 (336 against 425) and loses to it on
2013-04-05 (132-176 against 154). With n=2 and 31-60% coverage, that is not a result
in either direction.

### Consequence

The `enc_neg` column is the strongest of the eight on 2013-04-05 at truth yaw, 38.7%
against 94.9% for `enc`. Negating the encoder yaw rate more than halves the error.
That is consistent with the standing finding that the NCLT wheel yaw rate was never
converted to ENU, and it is the fourth place that sign has shown up. It is suggestive
and nothing more: on 2012-06-15 the same negation is the WORST column at filter yaw,
153.2%.

Two things follow. P3 needs a dataset whose ground truth survives the blackout, which
means KAIST with its TUM ground truth or hardware with RTK, not NCLT. And the gap
error shares in the window table above rest on the same partial coverage: 2012-06-15's
"84.6% of error in the gap" is measured across 60% of that gap, and 2013-04-05's
60-65% across 14% of its gaps. Those shares stand as written, because error can only
be scored where truth exists, but they are not a claim about the uncovered 82%.

## 2026-10-18, LATER: the bias trigger is net harmful, and the heading reset is inert

**This supersedes the labelling in the 2026-10-18 arm table above.** When that table was
measured, `gnss.reanchor_enabled=1` switched on BOTH triggers, so everything recorded
there as "arm A" is really A+C. The numbers stand for what they measured. The label does
not, and the default has since changed.

First, a correction to a caveat I attached to that table. I wrote that it was a single
run per arm against a measured 14.80% run-to-run spread. **That spread belongs to the
node, not to replay.** Replay is deterministic: three runs of the same arm on 2012-02-04
produced byte-identical trajectories, md5 `c1df7968f555f8cbd521243f90851e4a`, one of them
the published run itself. Every replay number in this file is noise-free, and the two
flat sequences are genuinely flat rather than lost in spread.

### The ablation

Isolating each trigger by pushing the other's threshold out of reach, five sequences,
RMSE and re-anchor event count:

    sequence     baseline        both      rate only      bias only
    2012-02-04    142.792   61.357/14    59.689/ 7      61.357/13
    2012-06-15     78.139   78.109/ 1    78.139/ 0      78.109/ 1
    2012-08-20    171.393   60.674/ 6    46.671/ 4      60.674/ 6
    2012-09-28     23.638   13.750/ 4    10.008/ 2      13.750/ 4
    2013-04-05     16.677   16.005/ 5    16.011/ 3      16.005/ 5

The rate trigger alone fires about half as often and lands better on three of five, by
23% on 2012-08-20 and 27% on 2012-09-28. What the bias trigger buys in exchange is 0.04%
on the two sequences where it wins, both of which are the sequences where re-anchoring
cannot help anyway. **Fewer re-anchors is better.** `gnss_reanchor_bias_trigger` now
exists and defaults to false.

Note the bias-only column reproduces the both-triggers column exactly on all five. Both
triggers fire in the same situations; the bias one simply fires more. This is not two
mechanisms, it is one mechanism at two thresholds.

### The heading reset does nothing

    sequence       both   both, no yaw   rate only   rate only, no yaw
    2012-02-04   61.357        61.308      59.689              59.689
    2012-06-15   78.109        78.108      78.139              78.139
    2012-08-20   60.674        61.351      46.671              46.671
    2012-09-28   13.750        13.748      10.008              10.008
    2013-04-05   16.005        16.005      16.011              16.011

With the bias trigger off, disabling the yaw reset changes nothing at all: bit-identical
trajectories on all five. The comment in `maybe_reanchor` claimed position alone does not
fix 2012-09-28 because the heading it left the gap with is wrong. **That claim is
withdrawn** and the comment now says so. Disabling the reset moves 2012-09-28 from 13.750
to 13.748. The reset is kept because resetting a heading the filter has just admitted is
wrong is the right thing to do and it costs nothing, not because anything measured shows
it helping.

### The shipping configuration

`gnss.reanchor_enabled=1`, bias trigger off, which is now the default pair:

    sequence     baseline        new   ratio    median  path_ratio  events
    2012-02-04    142.792     59.689   2.39x    13.665       1.012       7
    2012-06-15     78.139     78.139   1.00x    29.978       1.113       0
    2012-08-20    171.393     46.671   3.67x     8.531       0.866       4
    2012-09-28     23.638     10.008   2.36x     6.565       1.106       2
    2013-04-05     16.677     16.011   1.04x     6.993       1.016       3

**One thing got worse and it is not the RMSE.** 2012-09-28's path length ratio goes
1.048 baseline, 1.028 with both triggers, **1.106 with the rate trigger alone**. So the
configuration with the best RMSE on that sequence has the worst path distortion of the
three, worse than doing nothing. Two re-anchors buy a 2.36x RMSE improvement and cost
5.8% of path length. That is the honest trade and it is worth flagging to anyone whose
consumer integrates the path rather than reading the pose.

Where the error moved, windows pinned to the raw GNSS median as before:

    sequence     arm        | rec %t  rec %e | std %t  std %e | rec RMSE
    2012-02-04   baseline   |  44.5%   94.5% |  55.5%    5.5% |  215.799
    2012-02-04   rate only  |  11.5%   94.4% |  88.5%    5.6% |  175.275
    2012-08-20   baseline   |  22.4%   94.0% |  75.5%    3.2% |  350.265
    2012-08-20   rate only  |  10.9%   64.7% |  87.0%    3.8% |  114.029
    2012-09-28   baseline   |   8.8%   85.8% |  91.2%   14.2% |   75.269
    2012-09-28   rate only  |   6.7%   24.3% |  93.3%   75.5% |   19.312
    2013-04-05   baseline   |   3.6%   14.5% |  94.6%   25.2% |   33.362
    2013-04-05   rate only  |   3.7%    7.2% |  94.6%   27.4% |   22.364

2012-09-28 is the clearest: recovery RMSE 75.269 to 19.312 and recovery's share of error
85.8% to 24.3%, with 75.5% of what remains now in steady coverage. Recovery is no longer
that sequence's problem. Steady-state accuracy is, and raw GNSS still beats it there.

Safety on the shipping configuration, from replay's own event log: 16 events, **7 better,
0 worse, 9 unscorable** where truth is sparse. The single harmful event in the
both-triggers arm is gone. The lowest pre-event error across all scorable events rises
from 48.22 m to **138.61 m**, so the surviving trigger is the more conservative of the
two by a wide margin.

### It was unreachable from ROS

None of these parameters were declared in `fusion_node.cpp`. The feature existed, was
measured, and **no deployment could switch it on**, because replay wires the config
struct directly. Eleven `gnss.reanchor_*` parameters are now declared and wired, default
off, verified by launching the node and reading each one back after configure. The node
also publishes the event: `reanchored`, `reanchor_jump_m`, `reanchor_yaw_reset` and
`reanchor_count` on `/fusion/debug/gnss_status`, plus a WARN log line and three
`/diagnostics` fields. A re-anchor is a deliberate discontinuity, and a consumer that
cannot see it happen has to treat an 787 m correction as divergence and chase it.

## 2026-10-19  Three corrections to yesterday, and the yaw reset is gone

### 1. "Bit-identical on all five" was wrong

I wrote that disabling the yaw reset changed nothing on any sequence, bit-identical
trajectories, and used that to call it inert. The byte comparison was only ever run
on the bias-trigger arms. Run properly, with the reset removed from the code:

    2012-02-04  bit-identical      2012-06-15  bit-identical
    2012-09-28  bit-identical      2013-04-05  bit-identical
    2012-08-20  DIFFERS

On 2012-08-20 the trajectories diverge from t=1345508586.897, row 684185 of 790269:
position by up to **0.798 m** and a quaternion component by **0.934**, which is a
large rotation. Position RMSE lands on 46.671 either way.

So the reset was inert in the POSITION METRIC and not in the state. The RMSE agreeing
to three decimals is what I actually measured; "bit-identical" is what I claimed, and
it is a stronger statement than the measurement supported.

### 2. NCLT cannot score heading at all, so the truth-yaw rows are withdrawn

`2012-08-20/ground_truth.tum` carries an **exactly identity quaternion on all 11,174
rows**. Every NCLT ground truth here does. This was already on record from
2026-06-30, and `tools/gap_deadreckon.py` read yaw out of those quaternions anyway,
so every row labelled `@truth yaw` in the 2026-10-18 P3 table anchored dead reckoning
at **0 rad**, not at the true heading. All of those rows are withdrawn, including the
one I called suggestive: "enc_neg is the strongest of eight columns on 2013-04-05 at
truth yaw, 38.7% against 94.9%". It was the strongest way of facing due East.

The tool now detects an identity-orientation ground truth and says so instead of
scoring against a constant, and the truth-anchored variants are gone.

What survives, anchored at the FILTER's own heading so dead reckoning gets nothing the
filter does not already have:

    2012-06-15, gap 462s, truth covers 276s (60%), 377.7 m travelled
      filter coast          425.3 m   112.6% of distance
      gyro  @filter yaw     411.1 m   108.8%
      enc   @filter yaw     385.9 m   102.2%
      enc_neg @filter yaw   336.1 m    89.0%

    2013-04-05, gap 275s, truth covers 84s (31%), 114.9 m travelled
      filter coast          153.6 m   133.7% of distance
      gyro  @filter yaw     131.6 m   114.5%
      enc   @filter yaw     140.2 m   122.0%
      enc_neg @filter yaw   176.1 m   153.2%

Dead reckoning beats the filter's coast on **both** scorable gaps now, 336 against 425
and 132 against 154. That is a cleaner reading than yesterday's "wins one, loses one",
which came from mixing in the invalid rows. It is still n=2 at 31-60% coverage, and
every method is between 89% and 153% of distance travelled, which is worse than
asserting the robot never moved. The encoder sign does not generalise either:
`enc_neg` is the best column on 2012-06-15 and the worst on 2013-04-05.

### 3. The yaw reset is removed

Removed, along with `gnss_reanchor_min_speed` and `gnss_reanchor_sigma_yaw`, which
existed only to gate it, and the `reanchor_yaw_reset` field on GnssStatus. A re-anchor
now moves position only, so no consumer ever sees a heading discontinuity from one.
`UKF::reanchor_yaw` still exists and is still tested; nothing calls it.

It is removed because nothing shows it helping, which is weaker than showing it does
nothing. NCLT cannot tell the difference. If a future dataset with real orientation
shows heading is the problem, the evidence for putting it back has to be produced.
`test_gnss_reanchor.cpp` asserts that a re-anchor leaves the quaternion untouched:
the largest single-step yaw change is 2.11109 rad with re-anchoring and 2.11109 rad
without, across a 380 m position jump.

### Unchanged by all of this

The shipping numbers do not move: 2012-02-04 59.689, 2012-06-15 78.139, 2012-08-20
46.671, 2012-09-28 10.008, 2013-04-05 16.011. Four are bit-identical to the
pre-removal runs and the fifth matches to three decimals.

## 2026-10-19  Heading truth on NCLT after all, from positions

NCLT's ground-truth ORIENTATION is identity and unusable. Its POSITIONS are real, and
a Segway does not slide sideways, so course over ground is heading wherever the robot
is moving and not turning hard. `tools/truth_heading.py` builds that series with four
guards: a chord of at least 3 m of travel rather than adjacent samples, at least
0.5 m/s over the chord, under 10 degrees of turn across it, and a rejection of chords
implying over 3 m/s, which NCLT needs because a known ~12% of its samples imply over
5 m/s on a 1.5 m/s robot.

Coverage is better than expected:

    sequence      GT rows   heading samples   % of rows   % of run spanned
    2012-08-20      11174              7097       63.5%               99%
    2012-06-15       8197              4846       59.1%               99%

**The validation is what makes it usable.** The series is cross-checked against IMUQ,
the IMU's own absolute attitude, which is an INDEPENDENT source. They fit one constant
offset:

    2012-08-20   offset 97.33 deg,  scatter 5.19 deg median, 12.05 deg p90
    2012-06-15   offset 96.47 deg,  scatter 4.96 deg median, 12.90 deg p90

Two sequences, recorded months apart, agree on that constant to within 0.9 degrees,
and it is approximately the 90 degree NED-against-ENU convention difference. A
course-over-ground series that was mostly noise would not fit one constant across 7097
samples. So heading is now measurable here to a few degrees, and the standing claim
that "no heading claim can be made on NCLT" was too strong: it is true of the
quaternions, not of the dataset.

### The yaw reset, decided on evidence instead of its absence

The reset was removed on the grounds that nothing showed it helping. With heading truth
it can be asked properly. First, what actually differed, because my own description of
it was loose in both directions:

- The largest COMPONENT-wise quaternion difference is 0.9340 at t=1345508586.897. I
  called that "a large rotation" without computing the rotation. It is one: the actual
  angle between those two attitudes, double cover handled, is **120.45 deg**, and the
  largest anywhere in the run is 121.94 deg. So the loose quote happened to be right,
  but it was not a measurement when I wrote it.
- It is a YAW difference, up to **120.54 deg**, with roll and pitch moving 16.40 and
  29.92 deg.
- It is not persistent. 10509 of 790269 samples differ in yaw by over 1 degree, 1.33%,
  in **four episodes** of 29.5, 17.8, 29.0 and 1.9 s inside a 322 s span.

Scored against heading truth, whole run and from the divergence onward, the two arms
are indistinguishable:

    yaw reset ON  (whole run)   n=7097  median 9.01 deg  p90 34.90  RMS 30.78
    yaw reset OFF (whole run)   n=7097  median 9.01 deg  p90 34.90  RMS 30.78
    yaw reset ON  (after onset) n= 542  median 5.69 deg  p90 21.63  RMS 17.87
    yaw reset OFF (after onset) n= 542  median 5.69 deg  p90 21.67  RMS 17.90

That is because the episodes sit almost entirely where heading truth is missing, which
is the same place everything else on this dataset goes missing: 8 truth samples in the
first episode, 3 in the second, **0 in the third and fourth**. Eleven scorable samples
out of 78 seconds of disagreement.

On those eleven, with the convention offset fitted over the whole run so the episodes
cannot set it:

    mean heading error   ON 18.78 deg,  OFF 18.23 deg
    per-sample           ON closer on 3,  OFF closer on 8

**So the reset made heading slightly worse where it mattered, and removing it was the
right call.** State it at the strength the data supports and no higher: 0.55 degrees of
mean difference across eleven samples, against a heading truth whose own scatter is
5.19 degrees median. The direction is consistent and it is not evidence of a real
effect. What it does do is move the decision off "absence of evidence", which is where
it stood this morning.

### Consequence

Heading is now a scorable quantity on NCLT wherever the robot was moving, which makes
two previously unanswerable things answerable: whether a heading source helps, and
whether the coast's drift is a heading problem or a speed problem. It does NOT help
inside the blackouts, because that is exactly where truth is absent. The 18% gap
coverage figure stands unchanged.

## 2026-10-19  The node-versus-replay gap: a structural suspect, NOT yet a measurement

Replay scores 137 m where the node scores 194 m on the same sequence, same commit, same
data, about 40%. That is the largest unexplained number in this file and it is the one
that matters most to a user, because users run the node. Node run-to-run spread is
14.8% and replay's is zero, so the gap is far outside both.

**What is confirmed by reading the code and the ROS headers, not by measuring:**

Every sensor subscription in `fusion_node.cpp` uses `rclcpp::SensorDataQoS()`. From
`rmw/qos_profiles.h`, that profile is:

    KEEP_LAST, depth 5, BEST_EFFORT, VOLATILE

So the queue is five messages deep and delivery is best effort: under load, messages
are dropped, silently, with nothing in any log. Replay reads a file and drops nothing,
ever.

And all sensor callbacks share ONE callback group:

    sensor_cb_group_ = create_callback_group(CallbackGroupType::MutuallyExclusive)

which serialises them. A 100 Hz IMU, the encoder and GNSS all queue behind one thread,
and a GNSS update on a 23-state UKF is not cheap. A stall of 50 ms at 100 Hz overflows
a depth-5 queue.

**That is a mechanism, and mechanisms have been wrong here before.** Three of the
"obvious" explanations in this file were falsified by measurement, so this one is
labelled a hypothesis until the following is run:

1. Count messages the node actually CONSUMES per sensor, which it does not currently
   do, and compare against what the player published (the player logs its row counts:
   218450 IMU rows on 2012-09-28). A deficit is the proof. Equal counts falsify it
   outright and the gap is elsewhere.
2. If a deficit exists, re-run with RELIABLE and depth 1000 on the sensor topics. If
   the gap closes, that is the cause and it is also a one-line user-facing fix.
3. Only then look at the other two suspects: arrival-order rather than timestamp-order
   processing, which the node does do (there is no reorder buffer anywhere in it), and
   `predict` running once per message.

**What would make suspect 1 the wrong answer:** the achieved filter rate. `run_nclt.sh`
already reports it and calls a run under 80 Hz starved. If the node holds ~100 Hz and
still scores 194 m, it is not dropping much and the gap is arrival order or predict
cadence instead. That number comes out of the run finishing now.

Noted as a known gap in the public correction draft rather than left for a reader to
find, because the published improvements are replay numbers and I cannot yet say how
much of them reaches a deployment.

## 2026-10-19  The ~7 degree residual in the heading-truth offset, and a real defect behind it

The course-over-ground heading truth fits the IMU's own attitude with one constant
offset: 97.33 deg on 2012-08-20 and 96.47 deg on 2012-06-15. Ninety of that is the
NED-against-ENU convention. The residual is 7.33 and 6.47 degrees.

**That residual is very probably magnetic declination.** NCLT was recorded in Ann
Arbor, where magnetic north sits several degrees west of true north, and the IMU's
attitude is magnetic-referenced while course over ground is true-referenced. A
constant of that size, stable across sequences months apart, is what declination looks
like.

**Stated as a hypothesis, because NCLT cannot separate it from a mounting offset.**
Declination is a property of location and the whole dataset is one campus, so a fixed
sensor misalignment would be indistinguishable. A web search did not return an
authoritative Ann Arbor 2012 value, so the figure is not quoted here. What CAN be
tested on this dataset is consistency: all five sequences span 14 months, and if the
residual is declination they should agree to within the estimation noise, with at most
about 0.1 deg of secular drift. That is queued behind the node runs.

### The defect it led to, which is not about NCLT

`imu.has_magnetometer: true` makes the IMU's orientation quaternion the heading source
(`HeadingSource::IMU_ORIENTATION`). The comment there asserted that "9-axis IMUs with
magnetometer give true heading". **They do not. They give magnetic heading.**

`magnetometer.declination_rad` exists and is applied in exactly ONE place: `sensors/magnetometer.hpp`,
computing yaw from a raw magnetic field. Counted: `fusioncore.cpp` contains zero
references to declination. So on the IMU-orientation path nothing applies it, and a
user with a 9-axis IMU and a true-north map carries the local declination as a constant
heading bias, with nothing in the code or the docs telling them. Roughly 7 degrees in
Michigan and over 15 in parts of the US west coast.

No NCLT number is affected: `nclt_fusioncore.yaml` sets `imu.has_magnetometer: false`
and takes yaw from GPS track only, so that branch never runs in any result in this
file. This is a user-facing defect, not a benchmark one.

The comment is corrected. The fix is not in that commit, because applying a rotation to
a fused orientation is a behaviour change that needs its own test, and the build is
occupied by the node runs. What already exists and would surface the symptom: the
`heading_vs_track_deg` cross-check on `/fusion/debug/gnss_status`, whose own
documentation already names declination as a cause of an offset that stays constant
across headings. It detects it and nothing corrects it.

## 2026-10-19  Audit: did any published robot_localization number come from a starved run?

Prompted by a node run in which `ekf_node` and `navsat_transform_node` both died at
startup with exit -2. If the control can crash, then the control figures in the
published table need checking, because robot_localization IS the comparison baseline
and a degraded control flatters FusionCore.

**First, that crash was mine.** `run_nclt.sh`'s own `cleanup()` kills `ekf_node` and
`navsat_transform` by pattern, and I had run overlapping `pkill`s while tearing down a
contended run. In the clean run the control comes up fine and is still running. Not a
systematic defect, so nothing to fix there.

**The audit itself**, every entry in `tools/benchmark_baseline.json` against its own
recorded `achieved_hz` and `rl_starvation_events`:

    sequence         FC_3D     RL_3D      Hz    n  RL_starv
    2012-01-08     185.195    22.792   102.5    1         0
    2012-02-04     153.097   123.149   103.1    3         0
    2012-03-31      28.526   160.574   102.4    1         0
    2012-05-11       9.606     8.936   102.9    1         0
    2012-06-15      82.011    45.864    99.9    2         0
    2012-08-20     155.618    10.389   102.1    2         0
    2012-09-28      18.594    82.598   102.2    2        24
    2012-10-28      17.061    25.072   102.6    1         2
    2013-04-05      14.058     65.51   103.1    2         0

No FusionCore arm was starved: 99.9 to 103.1 Hz against the 80 Hz floor.

Two entries carry non-zero control starvation, and **both are rows FusionCore wins**,
which is precisely where a starved control would matter. Of the four claimed wins
(2012-03-31, 2012-09-28, 2012-10-28, 2013-04-05), two are on this list. So the question
is whether the four-of-nine win count survives.

**It does, and the entries already contained the answer:**

- **2012-09-28.** The 24 events are in the SECOND control run, which scored 91.724. The
  PUBLISHED figure is 82.598, the first run, which logged **zero**. The note also
  records that the control's pose count changed by 0.14%, so it was not starved in
  output, and FusionCore wins by 77% or 80% depending on which control run is used, so
  the verdict does not turn on it. Worth keeping in view: the control's own spread here
  is 11.05% against FusionCore's 0.01%, so on this sequence the control is the noisy
  filter.
- **2012-10-28.** 2 events across an 86 minute run against a historical rate of about
  one per 19 minutes. That is the background rate, not starvation. n=1 and the spread
  is unknown, which the entry already warns about.

**So no published control number came from a starved or crashed run, and the
correction does not need another correction.**

### The defect this did find is in the record format

`rl_starvation_events` is a single scalar per entry, and on 2012-09-28 it holds 24,
taken from the worse of two runs, while the published figure came from the run that
logged 0. Read on its own, that field says the published number is starved when it is
not. A scalar cannot carry per-run provenance once n > 1.

Fixed in the baseline: `rl_starvation_events_per_run: [0, 24]` and
`rl_published_run_index: 0` on that entry, with the scalar kept for compatibility and a
note saying which field to read. This is the same class as the claim-type confusion in
`provenance.py` earlier today: the measurement was right and the label could not say
which thing it described. `check_readme_benchmark.py` reports 0 violations after the
change.

### The prediction, written before the measurement

So the hypothesis for the node-versus-replay gap is not one mechanism but three that
compound, and reading `nclt_player.py` added the third:

1. Sensor subscriptions are `SensorDataQoS`: KEEP_LAST, **depth 5**, BEST_EFFORT.
   Overflow is a silent drop.
2. All sensor callbacks share one `MutuallyExclusive` group, so a 100 Hz IMU, the
   encoder and GNSS serialise behind a single thread, and a GNSS update on a 23-state
   UKF is not cheap.
3. **The player catches up by bursting.** Its loop computes
   `sleep_s = wall_start + sim_elapsed/rate - now` and sleeps only when that is
   positive. Fall behind and it publishes as fast as it can until it is level again.
   So load produces a burst, and the burst meets a five-deep best-effort queue.

**Predictions, so this can be wrong on the record:**

- The node CONSUMES fewer IMU messages than the player publishes. On 2012-09-28 the
  player logs 218450 IMU rows, so consumed should fall measurably short of that.
- The shortfall tracks the ATE gap: the sequences with the worst node-versus-replay
  disagreement should show the largest deficit.
- Re-running with RELIABLE and a depth of 1000 closes most of the gap.

**What falsifies it:** consumed counts matching published counts. The node would then
be seeing every measurement and the gap is arrival-order processing (there is no
reorder buffer anywhere in the node, so it does process in arrival order) or `predict`
cadence instead. The achieved filter rate is the first clue either way, and it is
already recorded: all nine baseline entries ran at 99.9 to 103.1 Hz, which is NOT the
picture of a starved node, and that is mild evidence AGAINST drops being the whole
story before the test is even run. Worth saying now rather than after.

**On the fix, if the prediction holds.** Not a multi-threaded executor: that trades
drops for ordering races, and this node currently has a single-threaded ordering
guarantee worth keeping. The shape that keeps determinism is trivial callbacks that
only push into a timestamp-ordered buffer, with one loop draining it in timestamp
order. That also fixes the arrival-order question in the same change, and it is the
only version where the node and replay can be expected to agree.

### Why nclt_player.py needs to stream, and why that is not a free change

It builds every event in memory and sorts: `self._events` holds roughly 440,000 tuples
for 2012-09-28, each IMU entry carrying a nine-float list. On a 5.6 GB machine that is
enough for the OOM killer to take it, which is exactly what happened on the first node
attempt (exit -9) when analysis scripts were running alongside.

The clean fix is a `heapq.merge` across three generators, since each CSV is already
time-sorted, which makes memory O(1) in the number of events.

**But the player is part of the measurement chain, so changing it can change published
numbers.** It will not be committed without first asserting that the new loader emits a
byte-identical event sequence to the current one for a full sequence. That check costs
nothing and is the whole reason to be careful here: this file already contains a case
where a dataset player's frame error invalidated every number measured through it.

## 2026-10-19  The node-versus-replay gap is NOT 40%, and the 137-against-194 pairing is stale

The premise I have been working from, carried in CLAUDE.md since 2026-09-27, is that
replay scores 137 m where the node scores 194 m on the same sequence and commit, about
40%, and that this is the most important user-facing defect because users run the node.

**Measured, it is not true of the current code.** Replay with re-anchoring OFF, measured
today, against the node figures in `benchmark_baseline.json`, both on
`nclt_fusioncore.yaml`:

    sequence         replay       node   node/replay   node run
    2012-02-04      142.792    153.097         1.07x   a4aae82, 2026-10-09, n=3
    2012-06-15       78.139     82.011         1.05x   a4aae82, 2026-10-09, n=2
    2012-08-20      171.393    155.618         0.91x   a4aae82, 2026-10-09, n=2
    2012-09-28       23.638     18.594         0.79x   a4aae82, 2026-10-09, n=2
    2013-04-05       16.677     14.058         0.84x   a4aae82, 2026-10-09, n=2

The node runs between **21% BETTER and 7% worse** than replay. It is not a uniform
penalty and it is signed in both directions. Three of the five differences are inside
the node's own measured 14.8% run-to-run spread.

**The configs really do match**, which was checked rather than assumed, because this is
exactly where the earlier errors came from. The replay arms drop seven keys from the
YAML and re-supply six. Each one:

    ukf.q_acceleration        YAML 1.0      replay default get(...,1.0)       same
    ukf.q_gyro_bias           YAML 1.0e-5   replay arg 1e-5                   same
    ukf.q_accel_bias          YAML 1.0e-5   replay arg 1e-5                   same
    ukf.q_encoder_wz_bias     YAML 1.0e-7   replay arg 1e-7                   same
    imu.gyro_bias_initial_sigma     not in YAML, node declares 0.175, replay 0.175
    imu.accel_bias_initial_sigma    not in YAML, node declares 0.30,  replay 0.30
    encoder.wz_scale_initial_sigma  not in YAML, node declares 0.30,  replay 0.30

`platform.profile` is NOT set in that config, which matters: the profile table would
override those three sigmas to 1.0e-6 and the configs would then differ. It does not.

**The node side reproduces.** A fresh node run today with re-anchoring off scored
**18.625** against the recorded **18.594** from 2026-10-09, 0.17% apart, at 102.6 Hz
with no starvation. So the node pipeline is stable across that commit range on this
sequence and the baseline figures can be used.

**The caveat, stated:** the replay column was measured at today's commit and the node
column at `a4aae82` from 2026-10-09. Different commits, so each ratio carries that
uncertainty. The 0.17% reproduction on one sequence is evidence the node side did not
move much, not proof for all five.

### What this changes

The 137-against-194 pairing dates from 2026-09-27, before the #169 frame fix and
everything after it. **It should not be quoted as the current state and CLAUDE.md is
corrected.** There is still a real pipeline disagreement worth understanding, up to 21%
on 2012-09-28, but it is modest, it favours the node as often as not, and it does not
mean the improvements measured on replay fail to reach a deployment.

The three mechanisms from the prediction above remain real as mechanisms: depth-5
BEST_EFFORT queues, one serialised callback group, and a player that bursts to catch up.
What they do not have is the 40% effect they were invoked to explain. The drop counter
still goes in, because a silent drop is a defect whatever its size, and because the
prediction deserves a real test rather than quiet abandonment. But it drops down the
priority list, and the honest reason is that the number it was meant to explain was
measured wrong.

## 2026-10-22  Node A/B confirms re-anchoring reaches a deployment, and the sweep clears the shipping config

Both node arms completed on 2026-10-19 (the session that launched them ended before the
results were read). 2012-09-28, 1x, two configs differing by one line, both unstarved:

    arm   ATE_3D   ATE_XY   path_ratio   drift_m/km     Hz   re-anchors
    off   18.625   15.661       0.8151         2.58  102.6            0
    on     9.996    6.446       0.8161         1.39  102.4            2

**1.86x through the ROS node**, which is the verification that was missing: the fix was
measured on replay and, until 2026-10-18, no deployment could even switch it on. Drift
2.58 to 1.39 m/km. The two events logged jumps of 157.98 m and 25.15 m through the WARN
line and `/fusion/debug/gnss_status`, so the diagnostic path works too.

`node on` 9.996 against `replay on` 10.008 agree to **0.12%**. On the off arm they do not
(18.625 against 23.638), which is the pipeline disagreement from the previous section, and
it is worth noting the two pipelines converge once the filter stops having excursions.

Path ratio barely moves here, 0.8151 to 0.8161, where replay on this sequence went 1.048
to 1.106. So the path-distortion cost recorded earlier is pipeline-dependent and not a
property of the feature.

The RL control read 91.648 and 82.190 across the two runs, an 11.5% spread, independently
reproducing the 11.05% recorded for this sequence. Both arms also reproduce their own
pipeline: the off arm landed 0.17% from the 18.594 on record.

### The shipping config is not bimodal

`ukf.q_angular_vel` swept 12 values on 2013-04-05 with the attitude flag OFF, which is the
shipping configuration, re-anchor off and on:

                 q |   RMSE_off   path_off |    RMSE_on    path_on   ev
              1e-5 |     18.919      0.984 |     18.341      0.981    3
              1e-4 |     29.284      0.966 |     28.846      0.964    3
             0.001 |     13.387      0.987 |     12.522      0.985    3
            0.0015 |     15.243      0.990 |     14.500      0.988    3
             0.002 |     15.261      0.993 |     14.521      0.990    3
             0.003 |     16.363      0.997 |     15.682      0.994    3
             0.004 |     18.294      0.997 |     17.691      0.994    3
             0.005 |     18.493      0.997 |     17.898      0.994    3
            0.0065 |     18.312      0.997 |     17.710      0.995    3
             0.008 |     17.602      0.998 |     16.986      0.996    3
              0.01 |     17.443      1.000 |     16.823      0.997    3
               0.1 |     16.677      1.019 |     16.011      1.016    3

Every path ratio is between 0.96 and 1.02. **There is no second basin.** With the attitude
flag ON the same sweep is sharply bimodal, every run either about 1.0 or about 0.17, and
re-anchoring does not remove that and flips basins in both directions. So **the bistability
is an attitude-flag-ON artifact**, the flag is off by default and already known to be 3.8x
worse on real data, and the question parks.

Re-anchoring also improves RMSE at every single q, by 2 to 6%, and fires 3 times
regardless. That consistency across a swept process noise is the robustness evidence the
feature needed and did not have.

### One row of that table was invalid and nearly got published

q=0.005 with re-anchoring on first read **5.391 m with 0 events**, which is self-
contradictory: zero events must mean a no-op. Its trajectory was **50146 lines against
662658** for every neighbour, and its log was 0 bytes. It was killed mid-flight when the
sweep was stopped to free the CPU for the node runs, and the resume guard in my own
driver, `[ -s "$out" ] && continue`, treats a non-empty truncated file as complete, so it
was skipped rather than redone. Re-run properly it gives **17.898**, path 0.994, 3 events,
in line with its neighbours at 17.691 and 17.710.

Same class as the 2012-01-08 66%-coverage withdrawal: a partial run scores well precisely
because it covers the easy part. **Any resume guard has to check completeness, not
emptiness.**

## 2026-10-22  A fresh session audited this file and the handoff, and found eight things

A new session was given `HANDOFF_20261022.md`, `CLAUDE.md` and this file, and asked to
report contradictions. It found eight. All eight were real. Recorded because the review
itself is the useful artifact:

1. **CLAUDE.md still asserted `P(X,VX)` is 0.000** with nothing marking it withdrawn, in
   the block that builds the whole position-versus-velocity mechanism on it. Now marked.
2. **CLAUDE.md restated "replay 137 m vs node 194 m" in place**, one paragraph below the
   2026-10-19 correction, so the correction did not reach the sentence making the claim.
   Now corrected inline as well.
3. **CLAUDE.md claimed `export_bag.py`'s projection diverges from the node's.** That is
   falsified by THIS file: the worst disagreement anywhere on the track is 9.313e-10 m. My
   own measurement, in my own narrative, contradicting my own instruction file, and I did
   not catch it while writing either.
4. **CLAUDE.md's block order is misleading.** The 2026-08-14 block sits ABOVE the
   2026-10-17 block that says it supersedes everything "dated below", so a reader starting
   at the top reads a superseded block first and nothing says so. A READ THIS FIRST block
   now sits above all of them and states the order of authority is by date, not position.
5. **The handoff said eleven `gnss.reanchor_*` parameters are declared. It is nine.**
   Eleven was true on 2026-10-18; removing the yaw reset on 10-19 took `min_speed` and
   `sigma_yaw` with it, and the handoff was written from the older number.
6. **"Baseline rows reproduce the classification exactly" is false for two of four
   sequences.** Corrected above.
7. **Two different computations share the label "raw GNSS median"**, 13.817 against 14.900
   on 2012-02-04. Corrected above.
8. **The handoff's git state was stale**: 14 commits ending `317dd34`, against 16 ending
   `ec593f2`, and its own commit list had 15 lines. It was written before the last two
   commits and the count was wrong even for when it was written.

Also noted and fixed: `2013-04-05` raw GNSS RMSE appears as 8.562 here and 8.561 in the
README. Both are real measurements of different things, the 8.562 from the fix-times
subsample and the 8.561 from the single-scorer rerun that the README table uses, and the
README's is the better figure because all three of its columns go through one scorer. The
0.001 m difference does not matter; sharing a label without saying which is which does.

**The pattern across 1, 2, 3 and 4 is one thing: corrections were written in the newest
place rather than in every place the claim lives.** A withdrawal has to chase the claim
into the instruction file, into the paragraph that restates it, and into the document that
summarises it. Four of the eight are that single mistake.

**A ninth, found 2026-10-22 while building a pitch deck from these numbers.** "75 to 100% of
squared error is in blackouts or recovery" was published in the README and repeated in
CLAUDE.md (twice), the handoff, the new-session prompt, a code comment in `fusioncore.hpp`,
this file (twice) and two memory files. Neither table supports the upper bound. Outage plus
recovery, computed from both the first budget table and the pinned-reference table:

    2012-02-04  94.5%   2012-06-15  84.6%   2012-08-20  96.8%
    2012-09-28  85.8% (86.5% unpinned)      2013-04-05  74.8%

So the range is **74.8% to 96.8%**. The lower bound was derived as 100 minus the largest
steady share (25.2%), and the upper bound should have been 100 minus the smallest (3.2%),
not 100. Corrected in all ten places in one change, which is the pattern above applied
rather than described. The conclusion is untouched: steady coverage is a small minority of
the error on every sequence.

## 2026-10-22  The coast variant: the premise was weaker than it looked, and two real findings instead

Queued item 1 from the handoff. During a coast, take speed only from the wheels and heading
only from the gyro, because dead reckoning anchored at the filter's heading beat the coast on
both scorable gaps (336 against 425 m, 132 against 154 m). Predictions were written first, in
`tools/repro/coast_variant_predictions.md`.

### Method

Two coast-only R multipliers were added, default 1.0: one on the accelerometer rows of R_imu,
one on R_encoder[WZ] (the mirror of the existing `gnss.coast_imu_wz_scale`). Replay with both
at default reproduces `arm_2012-06-15_baseline.tum` BYTE FOR BYTE (md5 ac7a622b...), so the
rebuilt tool changes nothing until asked. Arms, shipped NCLT config, re-anchoring OFF so the
coast is the recorded one:

    base   as shipped (coast_imu_wz_scale 500: heading mostly from the encoder)
    qb1    gnss.coast_q_bias_factor 1 instead of 100
    H      heading from the gyro: coast_imu_wz_scale 1, coast_encoder_wz_scale 1e6

### The accelerometer half cannot be tested this way, and the knob is removed

Inflating R on the accelerometer rows does not take speed from the wheels. Those rows also
carry gravity, which is what keeps roll and pitch observable, so dropping them breaks the
state: on the synthetic every seed went to 272% to 6682% of distance travelled. **So the
accelerometer hypothesis is UNTESTED, not falsified.** Testing it properly needs a structural
change (hold AX at zero during coast while keeping the gravity rows), not an R scale. The knob
was removed rather than left in the config as a trap.

### Synthetic: tools/repro/coast_variant.cpp

Turning, speed-varying truth, 120 s of 1 Hz GNSS then a 300 s gap. Gyro bias 0.005 rad/s,
accelerometer bias 0.05 m/s^2, encoder speed scale 2%, encoder WZ scale 5% plus 0.002 rad/s
bias, signs random per seed. 10 seeds. Error is displacement error over the gap as % of
distance:

    arm                  err mean   err med    path (full rate / true)
    base                   47.189    36.530    35.541
    H gyro heading         65.353    68.367    31.495
    qb1                    12.618    14.344     4.154
    q_pos 1                45.381    35.055    35.557
    DR gyro @filter yaw    10.331     9.845
    DR enc  @filter yaw     8.351     7.966

On the synthetic the 100x gyro-bias Q inflation is the whole story and the position Q
inflation is nothing. **NCLT does not agree, see below, so the synthetic is not a predictor
for qb1.**

### NCLT, whole run

ATE RMSE 3D (m), replay, all three columns re-scored through `hardware/ate_tum.py` (matches
evo to three decimals). These baseline figures differ from the 2026-10-18 table (78.428
against 78.139 on 2012-06-15) because that table came from a scorer that is no longer on disk;
nothing below is compared against it.

    sequence      base       qb1                H
    2012-02-04   142.043   162.936 (+14.7%)   140.115 (-1.4%)
    2012-06-15    78.428    66.009 (-15.8%)    62.676 (-20.1%)
    2012-08-20   165.364   159.604  (-3.5%)   142.761 (-13.7%)
    2012-09-28    22.805    22.533  (-1.2%)    19.822 (-13.1%)
    2013-04-05    16.456    16.496  (+0.2%)    27.163 (+65.1%)

**Neither arm is a no-regression change, so neither default moves.**

### NCLT, inside the gaps

`gap_deadreckon.py`, the two scorable gaps (truth covers 60% and 31% of them):

    gap                    base    qb1      H
    2012-06-15, 462 s     425.3   404.3   300.3   m at window exit
    2013-04-05, 275 s     153.6   110.8    31.0

H is better on both scorable gaps, by a lot on the second. But `blackout_budget.py` over every
gap with truth on 2013-04-05 (ref 5.822) says the gap-window RMSE goes 96.844 to **187.253**
under H: gyro heading rescued the one long gap and made the shorter ones worse. On 2012-06-15
(ref 5.059) the gap RMSE goes 243.866 to 194.033 (H) and 202.135 (qb1).

### The premise of this item was weaker than it looked

The dead-reckoning rows are NOT a fixed reference. `gyro @filter yaw` on 2012-06-15 reads
411.1 m under base and **32.7 m** under H, on identical sensor data. Each arm's dead reckoning
is anchored at THAT ARM's heading at window entry, and on 2012-06-15 window entry is 185 s into
the gap (the truth hole), so the anchor already carries 185 s of the coast being judged. "Dead
reckoning beats the coast" was therefore never independent of the coast. It should not be
quoted as evidence that the coast adds drift.

### A real defect found on the way: the fused position jitters, twice as much in coast

The synthetic's path length ratio of 35 prompted a check on real data. ate_tum's path ratio is
taken at ground-truth timestamps and cannot see this. At the filter's full output rate,
2012-06-15, three 462 s windows on the shipped baseline:

    window               fixes  full-rate path / 1 Hz path   full-rate path / wheel path
    before gap, GPS on    1737        1.27                        1.40
    inside the gap           2        2.46                        1.97
    after gap, GPS on      450        1.36                        1.33

The fused position oscillates at output rate all the time, and roughly twice as much while
coasting: 1073.0 m of fused path inside the gap against 543.9 m of wheel odometry. qb1 makes
it worse on real data (3.61) though it helped on the synthetic. A consumer that differentiates
pose (a controller, an odometry integrator) sees this directly. Not fixed; filed here.

### Prediction scorecard

    P1  accel out moves coast toward enc DR        UNTESTABLE with an R scale, knob removed
    P2  variant not uniform, heading source decides CONFIRMED: H -20.1% to +65.1% by sequence
    P3  nothing below ~89% of distance              FALSIFIED: H reached 79.5% and 27.0%
    P4  accel out beats base on the synthetic       NOT TESTED VALIDLY, same reason as P1

Provenance for every NCLT number: pipeline replay, config nclt_shipped (reanchor off) plus the
named arm keys, scoring points full_traj (ate_tum) or gaps (gap_deadreckon, blackout_budget),
coverage 100% of each sequence for full_traj.

## 2026-10-22  The jitter located, 2013-04-05's gaps explained in part, the accel hold, and declination

### 1. Where the output-rate jitter comes from

`hardware/replay.cpp attrib=` now splits every IMU and ENC event's position change into
the predict and the update that follows, by running the predict first through a new
diagnostics-only `FusionCore::predict_only(t)`. Replay with attribution on is BYTE
IDENTICAL to replay without it (md5 ac7a622b... on 2012-06-15), so the split is exact and
does not perturb the filter. `tools/jitter_attrib.py` reads it.

2012-06-15, shipped config, inside the 462 s gap:

    source            path m    net m   path/net   mean |step| mm
    predict            379.3    263.3     1.44         5.333
    IMU update         212.8     18.6    11.42         9.835
    ENC update         625.2     19.5    32.13        12.633
    combined output   1072.9    247.5

**The jitter is measurement updates, not prediction.** Velocity-type updates should barely
move position; inside the gap each encoder update moves it 12.6 mm, against 2.3 mm with GNSS
present. The kicks grow with time since the last fix (0.5 mm in the first 30 s, 10.5 mm by
60 to 90 s, ~17 mm after 200 s) and are mostly sideways (53 to 86%).

**The prediction "two sources take turns pulling in opposite directions" is NOT confirmed.**
Adjacent IMU and ENC updates point the same way more often than not (median cos +0.381,
47.7% aligned against 34.9% opposed), and gyro and encoder yaw rate agree inside this gap
(corr +0.779).

Ruled out, each by re-running the attribution with one thing changed (combined in-gap path):

    coast position Q 10x -> 1           1073.7   no change
    coast gyro sidelining 500 -> 1      1072.7   no change
    coast gyro-bias Q 100x -> 1         1657.1   worse
    q_angular_vel 0.1 -> 0.01, 0.001    1036.0, 1143.3   no better (WZ wobble fell 0.0226 to 0.0085 rad/s)
    every yaw-rate measurement off      1111.7   no change: the kicks do not come through WZ

What DOES move it:

    motion_model DifferentialDrive      698.7   ENC kicks 625.2 -> 287.7
    acceleration held at zero in coast  671.6   ENC kicks 625.2 -> 128.6

So the encoder kicks travel through the position cross-covariance with the free LATERAL
VELOCITY and ACCELERATION states, which grows with distance travelled since the last fix.
Neither lever is a fix: DifferentialDrive costs +7.8%, +14.4% and +17.2% ATE on three of
five sequences, and the accel hold below is no better. The mechanism is located; the cure
is not.

A false alarm caught on the way: replay's own comment said it "hardcodes DifferentialDrive",
which would have meant replay and node used different motion models all along. It never
did: nothing assigned cfg.motion_model, so both ran ConstantVelocityAcceleration. The
comment is corrected and `motion_model` is now a wired replay string parameter.

**New standard metric: `output_rate_path_ratio`** in `tools/evaluate.py` (metrics.json) and
`hardware/ate_tum.py`: the estimate's path at its full output rate over the associated span,
over the ground-truth path. On the 2012-06-15 baseline it reads **1.228** against a
truth-rate path ratio of 0.801. Three tests pin it, including that the old ratio cannot see
jitter between truth samples.

### 2. Why 2013-04-05's gaps hold 60.3% of its error

`tools/gap_sensors.py`, per gap from the raw fix stream:

    start  len s  gyro dpsi  enc dpsi  true dpsi   corr  mean(gyro-enc) rad/s
      588     65     -72.2     -68.3        -        0.796    +0.00195
     2103     32     -34.3     -26.3        -        0.783    -0.00433
     2140    275    -161.4    -128.8    (partial)    0.786    -0.00260
     3460    129    -133.6    -121.5        -        0.767    -0.00281
     3716     55     +36.6     +43.3        -        0.784    -0.00090

Over the 72 s of the 275 s gap that has heading truth (73 samples): truth -86.2 deg,
encoder -87.4, gyro -96.7. **The encoder is right to 1.2 deg and the gyro is 10.5 deg off,
which is a gyro bias of about -0.0026 rad/s (-0.15 deg/s) relative to both.** Not slip:
the two sensors agree in shape (corr 0.77 to 0.80) and over the truth-covered stretches
of that gap the wheels read 101.7 m against 97.6 m of truth (1.042). That bias is why
gyro-only heading in coast cost this sequence +65.1%.

**What it does NOT explain:** the shipped coast takes heading from the encoder, which matched
truth, so the 60.3% of error in these gaps is still not accounted for. Open. Also noted and
not chased: with GNSS present, a 500 s stretch reads wheels 583.2 m against truth 485.4 m
(1.201), a larger over-read than inside the gap.

### 3. The accelerometer test done properly

`gnss_coast_hold_accel_sigma` (default 0, core and replay only): during coast, after each
IMU update, a pseudo-measurement holds AX/AY/AZ at zero. The accelerometer rows still run,
so gravity keeps roll and pitch observable, which is what the R-scale attempt broke.

Synthetic (10 seeds): drift 47.189% -> 27.702% (sigma 0.1) and 35.296% (0.01), path
inflation 35.5 -> 6.6 and 8.3. NCLT, ATE RMSE 3D, replay, ate_tum:

    sequence      base      hold 0.1            hold 0.01
    2012-02-04   142.043   157.642 (+11.0%)   158.110 (+11.3%)
    2012-06-15    78.428    74.876  (-4.5%)    79.535  (+1.4%)
    2012-08-20   165.364   160.684  (-2.8%)   158.199  (-4.3%)
    2012-09-28    22.805    22.539  (-1.2%)    19.738 (-13.4%)
    2013-04-05    16.456    29.710 (+80.5%)    33.065 (+100.9%)

**Does not ship.** It cuts the in-gap jitter (1072.9 -> 671.6 m) and the output-rate path
ratio (1.228 -> 1.173), so acceleration IS connected to the jitter, but it costs two
sequences badly. The synthetic predicted a clean win again and was wrong again on real data.

### 4. Declination: fixed, and the existing path had the sign backwards

Physics: with magnetic north D east of true north, a body at true ENU yaw psi measures a
horizontal field (sin(psi+D), cos(psi+D)), so true yaw = atan2(mx, my) - D. The raw-field
path ADDED D, so the documented setting doubled the bias, and its unit test pinned the
formula's output rather than the physics. Fixed to subtract, with a forward-built test over
four declinations and four headings. The 9-axis IMU-orientation path now subtracts D too
(it applied nothing before). Every shipped config sets 0.0, so no default changes and no
NCLT number moves. Test `NineAxisYawCorrectedForDeclination` checks the settled offset is
-D to 0.005 rad; it found that at D = 0 the yaw settles 0.0144 rad above a constant
0.5 rad IMU yaw after 30 s with no other sensor, a pre-existing residual on the unchanged
path.

Provenance for every NCLT number here: pipeline replay, config nclt_shipped (reanchor off)
plus the named keys, scoring full_traj via hardware/ate_tum.py or gaps via the tools named,
coverage 100% of each sequence for full_traj.

## 2026-10-22  Four follow-ups: the AY constraint, the gyro bias, -0.998, and "frozen" is not frozen

### The lateral-acceleration constraint makes the jitter WORSE

Premise check first: the suggested fix, fuse VY = 0 as a measurement, is ALREADY SHIPPED.
`encoder_measurement_function` carries VY and `encoder.nhc_vy_sigma` is 0.05 m/s, so the
jitter was measured with it in place. What nothing constrains is AY, which the IMU sees only
as AY + B_AY + gravity. So the test was the kinematic version for a no-slip wheeled robot:
fuse AY - VX*WZ = 0 on every encoder update (`nhc_ay_sigma`, core field, replay-only).

    sequence      base      AY 0.1             AY 0.5           output-rate ratio base / 0.1 / 0.5
    2012-02-04   142.043   149.487  (+5.2%)   147.073  (+3.5%)   1.411 / 1.550 / 1.424
    2012-06-15    78.428    77.463  (-1.2%)    43.318 (-44.8%)   1.228 / 1.388 / 1.324
    2012-08-20   165.364   159.812  (-3.4%)   166.871  (+0.9%)   1.569 / 1.692 / 1.644
    2012-09-28    22.805    18.595 (-18.5%)    18.880 (-17.2%)   1.361 / 1.420 / 1.357
    2013-04-05    16.456    19.776 (+20.2%)    17.350  (+5.4%)   1.555 / 1.620 / 1.542

**Prediction falsified.** In-gap fused path on 2012-06-15 rises from 1072.9 m to 1500.0 m
and encoder kicks from 625.2 to 1122.0 m. **The -44.8% on 2012-06-15 is not an improvement:**
across the whole 462 s gap the fused position nets 17.6 m against 255.1 m of truth
displacement over the covered part, i.e. it churns in place (1500.5 m of path), which scores
well on RMSE for the same reason "assert the robot never moved" would. Does not ship.

### 2013-04-05 and the gyro bias

The shipped filter's B_GZ is NOT near zero and NOT frozen (the NCLT config sets no profile).
It wanders: whole-run median -0.00134 rad/s, p10 -0.03723, p90 +0.03111; inside the gaps it
swings over about +-0.1 rad/s with sigma 0.19 to 0.38 rad/s, the 100x coast bias Q at work.

Boot calibration: 2013-04-05's first ~60 s are effectively stationary (encoder speed median
0.04 to 0.05 m/s balancing wobble, encoder yaw rate mean -0.00001 rad/s), and the gyro
averages wx -0.00295, wy +0.00130, wz -0.00183 rad/s there. Starting replay from that bias
(`init.gyro_bias_*`, replay-only) changes NOTHING: 16.456 against 16.456, gap RMSE 96.844
both, because the shipped prior sigma of 0.175 rad/s forgets the start within 2 s (B_GZ is
+0.0015 in both arms by t+2 s). With gyro heading in coast it is still 27.163, identical to
before. **Both predictions fail as stated.**

Separately: the node's own `init.stationary_window` would ABORT on this start, because its
abort fires on any single encoder sample above 0.05 m/s and the balancing Segway's p90 is
0.087 m/s. Turning boot calibration on through the node would silently fall back to zero.

What DID move 2013-04-05 is the `ground_robot` profile combined with gyro heading in coast:

    2013-04-05            ATE      gap RMSE   gap share
    shipped             16.456     96.844      60.3%
    profile             15.168     84.884      54.5%
    profile + gyro      12.671     57.750      36.1%   (boot bias or zero: identical)

On all five (ATE RMSE 3D, replay, ate_tum, output-rate ratio):

    sequence      shipped    profile    profile + gyro heading
    2012-02-04   142.043    141.595    134.447  (-5.3%)   1.411 -> 1.224
    2012-06-15    78.428     64.537     67.927 (-13.4%)   1.228 -> 1.078
    2012-08-20   165.364    141.195    180.598  (+9.2%)   1.569 -> 1.416
    2012-09-28    22.805     21.560     19.511 (-14.4%)   1.361 -> 1.187
    2013-04-05    16.456     15.168     12.671 (-23.0%)

It cuts the output-rate jitter on every sequence measured and improves four of five, but
2012-08-20 regresses 9.2% against shipped and 27.9% against the profile alone. A candidate,
not a default.

### "Frozen" is not frozen

The state dump of the profile arm shows B_GZ sigma starting at 0 and reaching 0.026 rad/s
within 2 s; over the run its median is 0.053 rad/s (max 0.124) and the estimate's magnitude
has median 0.0106, p90 0.0465 rad/s. The profile zeroes the bias Q and prior, but
`UKF::generate_sigma_points` builds them from `(n+lambda)*P + 1e-6*I`, and a zero-variance
state makes P near-singular so the identity-shift repair raises every diagonal. Core code,
so the node does the same. The measured profile improvements stand as what the profile does;
the word "freezes" overstates the mechanism and is corrected in the README, CHANGELOG,
configuration.md, the profiles README and CLAUDE.md. Not fixed.

### -0.998

It is NOT of unknown provenance: `test_observability.cpp` asserts it, a 1.8 s in-place spin
at 0.6 rad/s from a tight heading prior, and it still passes. What it is not is a measurement
of an encoder robot on real data, where NCLT gave -0.6256 to -0.0370. Removed from the
CHANGELOG and README as the justification for `platform.profile`, replaced by the
frozen-against-free replay table re-run today with all seven profile keys applied (an
earlier attempt silently applied only three, because the YAML args supplied the other four;
the profile only fills defaults). Every other copy now says it is a synthetic spin.

### The node-versus-replay premise, again

The request called the node-versus-replay gap "137 vs 194 m" and the top priority. That
pairing was withdrawn on 2026-10-19: on current code the node runs between 21% better and 7%
worse than replay. The drop count was still run, because the prediction is on record. The
node now logs consumed callbacks per sensor once a minute, since a DDS KEEP_LAST overwrite
need not count as `messages_lost` and that counter alone could read 0 while messages are
skipped. One real node/replay difference was noted while reading for it: the node
initialises P as 0.1*I with position variance 1000, replay from the State default. Result
below when the run completes.

## 2026-10-23  The heading-wobble hypothesis fails; the leak fix and the combined candidate on DEV

### Is the jitter a heading wobble amplified by distance? No.

`tools/jitter_heading_corr.py` on the 2012-06-15 gap (461.5 s, 71126 events, shipped
config, replay byte-identical to the baseline with attribution on):

    |lateral kick| vs lever * |dpsi|     r = +0.500   slope 0.008 (hypothesis: 1.0)
    |lateral kick| vs |dpsi| alone       r = +0.550
    |lateral kick| vs lever alone        r = +0.127
    d b_gz vs d heading, lags -10..+20 s r between -0.091 and +0.112

The lever arm makes the relationship WEAKER, not stronger, and the kicks are about 1/125 of
the size the hypothesis predicts. The bias estimate does not lead heading at any lag. What
remains is a moderate same-event link with heading change, and that heading change includes
the robot turning, which is when lateral velocity and acceleration are non-zero: consistent
with the lateral-velocity/acceleration channel found on 2026-10-22, and with the earlier
result that removing every yaw-rate measurement left the jitter unchanged.

### DEV results (not a ship decision; tools/ship_rule.md decides on the holdout)

Replay, ATE RMSE 3D (m), hardware/ate_tum.py, against shipped on the same sequence.
A = `ukf.semidefinite_sqrt` alone. C = A + `platform.profile=ground_robot` (all seven keys
applied, verified per run) + gyro heading in coast.

    sequence      shipped     A                  C                  outrate shipped / A / C
    2012-02-04   142.043   158.530 (+11.6%)   146.201  (+2.9%)   1.411 / 1.428 / 1.235
    2012-06-15    78.428    74.563  (-4.9%)    61.266 (-21.9%)   1.228 / 1.234 / 1.082
    2012-08-20   165.364   150.059  (-9.3%)   159.035  (-3.8%)   1.569 / 1.547 / 1.410
    2012-09-28    22.805    18.660 (-18.2%)    19.642 (-13.9%)   1.361 / 1.316 / 1.181
    2013-04-05    16.456    12.521 (-23.9%)    17.747  (+7.8%)   1.555 / 1.544 / 1.332

The predictions for C, written before running: 2012-08-20's cost shrinks or disappears
(HOLDS: 180.598 without the leak fix, 159.035 with it); jitter drops further (HOLDS, every
sequence); 2013-04-05 beats 12.671 (FALSIFIED: 17.747). A alone reaches 12.521 there.

A barely moves the jitter, as the correlation result says it should: the jitter is not the
bias channel. Both candidates are "most better, some worse" on DEV, which is exactly what
the holdout exists to settle. Both go to it once the seven holdout logs exist.

## 2026-10-23  HOLDOUT: both candidates fail, and the node-versus-replay gap is back on one sequence

### The seven holdout logs

Built from seven 1x node runs that recorded their inputs, exported with `export_bag.py` (the
path validated by re-exporting 2012-09-28: every shared IMU/IMUQ/ENC event identical, GNSS a
constant 5.5 cm origin shift). Every run unstarved (102.0 to 103.3 Hz). Two exports matched
the player's counts exactly; five were short at the START only (the recorder starting 0.09 to
1.72 s late) and were accepted after `~/nclt/holdout/where_missing.py` showed nothing missing
inside or after, on IMU, ENC and GNSS (GNSS compared against 3D fixes only, as the player
filters them). Each accepted log has a NOTE file beside it in `~/nclt/exported/`.

### The one-shot test, per tools/ship_rule.md (committed 27aa7a9 before any holdout log)

Replay, ATE RMSE 3D (m), hardware/ate_tum.py, every arm on identical associated points, no
config or profile warnings on any of the 21 runs. `~/nclt/holdout/apply_rule.py` applies the
rule mechanically.

    sequence     shipped    A (leak fix)         C (A + profile + gyro heading)
    2012-01-08    52.152    49.890  (-4.3%)    183.529 (+251.9%)  median 12.487 -> 130.448
    2012-03-31    22.279    17.306 (-22.3%)     25.677  (+15.3%)
    2012-05-11     9.651     9.650  (-0.0%)      9.608   (-0.4%)
    2012-10-28    18.778    17.265  (-8.1%)     17.033   (-9.3%)
    2012-11-04    54.253    65.358 (+20.5%)     47.235  (-12.9%)
    2012-12-01    16.384    16.200  (-1.1%)     16.122   (-1.6%)
    2013-02-23    21.377    19.401  (-9.2%)     20.727   (-3.0%)

**A FAILS**: 4 of 7 better by 2% (needs 5), and 2012-11-04 is +20.5% RMSE, +12.4% median.
**C FAILS**: 3 of 7 better, 2012-01-08 +251.9% and 2012-03-31 +15.3%. C still cuts the
output-rate jitter on every holdout sequence (e.g. 1.666 to 1.489, 1.583 to 1.372), at an
accuracy cost the rule does not allow.

**This is the holdout doing its job.** On the five DEV sequences both read "four better, one
worse". On seven sequences neither was tuned on, A met a new bad case and C broke one
outright. Per rule 6 the holdout is now spent for this line of work: a retuned A or C may not
be re-tested on these seven. Both flags stay off.

### The node-versus-replay gap is NOT modest on every sequence

The 2026-10-19 correction ("the node runs between 21% better and 7% worse than replay") was
measured on the five DEV sequences and holds there. On the holdout, node (2026-10-23 runs,
current main) against replay (shipped config, same day):

    2012-01-08   node 178.808   replay  52.152   3.43x
    2012-11-04   node  68.232   replay  54.253   1.26x
    2012-03-31   node  25.887   replay  22.279   1.16x
    2012-05-11   node   9.605   replay   9.651   1.00x
    2012-12-01   node  15.948   replay  16.384   0.97x
    2012-10-28   node  17.220   replay  18.778   0.92x
    2013-02-23   node  16.934   replay  21.377   0.79x

2012-01-08 is 3.43x worse through the node, far outside the node's 14.8% spread, and its RL
control reproduced its record to 0.47% (22.900 against 22.792), so the input is not the cause.
The drop counter rules out lost messages on 2012-09-28 only; 2012-01-08 is untested. This
sequence is where the node and replay disagree, and it is the place to look.

## 2026-10-25  The node-versus-replay gap is explained: replay never fused the IMU orientation

The node fuses the IMU's orientation on every IMU message whenever
`orientation_covariance[0]` is not -1 (`fuse_imu_orientation_if_valid`), and the NCLT player
publishes roll/pitch at variance 1e-4 and yaw at 1e6. `hardware/replay.cpp` deliberately
skipped those IMUQ rows. New opt-in `imuq_rp=1` fuses them exactly as the node does (same
RPY convention, identity imu_link per nclt_benchmark.launch.py, same covariance array); the
default path stays byte-identical.

ATE RMSE 3D (m), shipped config, node runs from 2026-10-09 (a4aae82, four DEV sequences) and
2026-10-23 (holdout and 2012-09-28):

    sequence      replay   replay+rp      node   node/replay  node/replay+rp
    2012-01-08    52.152     178.226   178.808       3.43x          1.00x
    2012-02-04   142.043     147.927   153.097       1.08x          1.03x
    2012-03-31    22.279      21.200    25.887       1.16x          1.22x
    2012-05-11     9.651       9.628     9.605       1.00x          1.00x
    2012-06-15    78.428      81.216    82.011       1.05x          1.01x
    2012-08-20   165.364     184.052   155.618       0.94x          0.85x
    2012-09-28    22.805      22.499    18.603       0.82x          0.83x
    2012-10-28    18.778      16.326    17.220       0.92x          1.05x
    2012-11-04    54.253      65.926    68.232       1.26x          1.03x
    2012-12-01    16.384      16.074    15.948       0.97x          0.99x
    2013-02-23    21.377      21.340    16.934       0.79x          0.79x
    2013-04-05    16.456      12.056    14.058       0.85x          1.17x

On 2012-01-08 it is not a coincidence of two totals. Under one common alignment, median
error by window: t+0..4100 replay 12.31 / replay+rp 12.31 / node 12.35; t+4100..5000
**10.98 / 458.91 / 454.58**; t+5000..5700 14.26 / 14.42 / 14.46. The orientation input is
what puts the filter into the 800 s lock-out that replay escaped (replay enters the same
rejection run at t+4138 and coast frees it at t+4351). The paths inside the excursion still
differ by up to 330 m, the node's arrival-order nondeterminism.

The residual differences (0.79x to 1.22x on five sequences) are not yet explained; four of
the five DEV node numbers are from an older commit and the node's own spread is 14.8%.

**Consequence, stated plainly:** every replay number before today, including re-anchoring's
3.67x and the 2026-10-23 holdout verdicts for A and C, comes from a pipeline WITHOUT the
orientation fusion the NCLT node performs. Those numbers stand for that pipeline; they are
not what the node does. Node-confirmed results (re-anchoring 1.86x on 2012-09-28) are
unaffected. From now on the node-faithful pipeline is `imuq_rp=1`.

On 2012-01-08, plain replay with re-anchoring on: 52.152 -> 18.569 (4 re-anchors). Not yet
measured with imuq_rp=1.

## 2026-10-25  Candidate R (re-anchoring on by default), holdout: FAILS the rule, on secondary clauses

Pre-registered in tools/ship_rule.md (commit 46563a4, pushed before any run). Pipeline:
replay with `imuq_rp=1` (node-faithful). Baseline: shipped config on the same pipeline.
ATE RMSE 3D (m), hardware/ate_tum.py, identical associated points, no config warnings.

    HOLDOUT        shipped      R          RMSE      median      outrate        re-anchors
    2012-01-08    178.226    16.996   -90.5%    -94.5%    1.556->1.452       4
    2012-03-31     21.200    18.573   -12.4%     +3.3%    0.951->0.950       3
    2012-05-11      9.628     9.628    +0.0%     +0.0%    0.939->0.939       0
    2012-10-28     16.326    13.923   -14.7%     -0.3%    1.109->1.111       3
    2012-11-04     65.926    64.166    -2.7%     +4.4%    1.406->1.403       2
    2012-12-01     16.074    12.737   -20.8%     +2.1%    1.179->1.305       4
    2013-02-23     21.340    20.080    -5.9%     -0.6%    0.898->0.901       3

**FAIL** by the rule: median +3.3% (2012-03-31) and +4.4% (2012-11-04) against a 3% limit,
and output-rate ratio +0.126 on 2012-12-01 against 0.05. On the primary measure it is the
strongest held-out result this project has: RMSE lower on 6 of 7, higher on none, and the
node's 2012-01-08 lock-out goes 178.226 to 16.996. The rule exists for exactly this shape, a
headline win paid for in typical-case accuracy and smoothness, and the thresholds are not
being moved after seeing the result. Re-anchoring stays opt-in; the holdout is spent for it.

    DEV (reported only)  shipped      R        RMSE     median
    2012-02-04          147.927   58.550   -60.4%   -63.9%   (output-rate +0.059)
    2012-06-15           81.216   81.216    +0.0%    +0.0%
    2012-08-20          184.052   43.958   -76.1%   -72.1%
    2012-09-28           22.499   10.008   -55.5%    -7.9%
    2013-04-05           12.056   11.174    -7.3%    -0.2%

**Still owed:** the README's published raw-GNSS-versus-FusionCore table (Status 2026-10-18)
was measured on the pipeline without the IMU orientation fusion. It has to be re-measured on
the node-faithful pipeline before it is cited again. Not edited until then.

## 2026-10-25  Raw GNSS against FusionCore, re-measured on all twelve, node-faithful pipeline

**The README's Status 2026-10-18 table cannot be reproduced.** Its own files still exist
(`gn_2012-02-04.tum`, `fx_2012-02-04_base.tum`, `fx_2012-02-04_new.tum`) and today score
57.564 / 145.629 / 59.340 through hardware/ate_tum.py against the published 58.672 /
144.903 / 60.522; evo at t_max_diff 0.01 and 0.05 matches neither. The scoring step that
produced the table is not on disk. Same failure class as the 78.139 baseline.

Re-measured: one scorer (hardware/ate_tum.py, Umeyama SE3, 0.1 s association, which matches
evo at evaluate.py's tolerance), fix-time points (`subsample_to_fix_times.py`), all three
columns restricted to IDENTICAL timestamps (pair counts equal on every row), trajectories
from the node-faithful replay (`imuq_rp=1`). "After" = re-anchoring on (opt-in).

    sequence     pairs   raw GNSS   FC before   FC after   before/GNSS  after/GNSS
    2012-01-08   10172      9.883     170.643     10.891      17.266x      1.102x
    2012-02-04    8905     57.564     151.937     59.303       2.639x      1.030x
    2012-03-31   10038     13.828      15.212     10.729       1.100x      0.776x
    2012-05-11   10386      9.186       9.166      9.166       0.998x      0.998x
    2012-06-15    6527      9.293      12.469     12.469       1.342x      1.342x
    2012-08-20    9799     34.250     193.923     39.782       5.662x      1.162x
    2012-09-28    8404      8.960      21.560     10.166       2.406x      1.135x
    2012-10-28    8907     13.921      15.057     13.170       1.082x      0.946x
    2012-11-04    8196     20.272      27.326     22.540       1.348x      1.112x
    2012-12-01    7835     12.681      16.150     12.731       1.274x      1.004x
    2013-02-23    8568     15.725      17.442     15.786       1.109x      1.004x
    2013-04-05    7452      8.623       9.982      8.774       1.158x      1.018x

**Shipped default:** raw GNSS has the lower RMSE on 11 of 12; 2012-05-11 is a 0.2% tie.
The published conclusion holds for what ships.
**Re-anchoring on (opt-in):** FusionCore is lower than raw GNSS on 2 of 12, 2012-03-31
(-22.4%) and 2012-10-28 (-5.4%), both HOLDOUT sequences; within 0.4% on three more; worse
on seven (1.02x to 1.34x).

The README is NOT edited yet: its standing instruction says no new claim may contradict
"does not beat raw GNSS on any NCLT sequence", and this measurement does, for the opt-in
setting on two sequences. That wording is Manan's call.

## 2026-10-25  Why the IMU orientation input causes the lock-out: it is the accelerometer, which OUR player declared at 0.57 deg

> **Attribution, corrected the same day.** The 0.57 deg was declared by FusionCore's own
> `nclt_player.py`, not by the dataset. The dataset's AHRS roll/pitch simply are the
> accelerometer's tilt, which is what an accelerometer-referenced tilt is, not a defect in
> the data. Declaring it as 0.57-deg attitude was our configuration choice, the same lesson
> as the earlier "dataset bug" that was the reader. The variance sweep below was observed on
> 2012-01-08, a SPENT holdout sequence: it is diagnosis, not validation, and it did not choose
> the fix. The fix chosen is principled rather than tuned: the player now publishes the
> orientation with `orientation_covariance[0] = -1`, because it is derived from the same
> accelerometer the node already fuses, so fusing it at any covariance double-counts. The
> node also warns once when it fuses roll/pitch declared tighter than 1 deg.

Checked as suggested: in quasi-static stretches of 2012-01-08 (5464 samples), the reported
roll/pitch match the accelerometer's gravity tilt to 0.00 deg median, 0.00 MAD, corr 1.000.
The swapped and sign-flipped alternatives are clearly wrong, so it is NOT a frame or
convention bug in what is fused. (The player converts the accelerometer and gyro NED to ENU
and passes the AHRS roll/pitch/yaw through unconverted; roll/pitch nevertheless agree with
gravity, and yaw is fused at 1e6 and so ignored.)

The real finding is that the match does not break while the Segway accelerates: with
|dv/dt| > 0.3 m/s^2 (69585 samples), reported pitch still tracks accelerometer tilt to a
median 0.037 deg, where a real attitude estimate would differ by about 1.75 deg. NCLT's
"AHRS" roll/pitch is the accelerometer's tilt. The player publishes it at variance 1e-4
(0.57 deg), so the node fuses the accelerometer twice, the second time as a confident
attitude that is wrong whenever the robot accelerates.

Its variance alone decides 2012-01-08 (node-faithful replay, full trajectory ATE RMSE):

    roll/pitch sigma   0.57 deg (player)  1.81 deg   5.73 deg   18.1 deg   not fused
    ATE RMSE              178.226          55.999     51.134     26.096     52.152

`tools/bag_report.py` now checks for this (`check_imu_orientation`): on the 2012-05-11 node
bag it reports the reported tilt tracking accelerometer tilt to a median 0.08 deg over
112305 accelerating samples against an expected 4.3 deg, at a claimed 0.57 deg. It also
flags orientation inconsistent with gravity at rest (a frame error). Four tests.

The benchmark-side fix is the player's covariance, which is part of the measurement chain
and changes every node number, so it is a decision rather than an edit; with the NCLT
holdout spent, its effect on accuracy cannot be graded on NCLT any more. This closes the
NCLT line of work for now.

## 2026-10-25  FusionCore against robot_localization, same runs, one scorer (old player, interim)

robot_localization's NCLT config (`fusioncore_datasets/config/rl_ekf.yaml`, `imu0_config` row 2)
ALSO fuses roll/pitch from the IMU orientation message, so in every node run so far both
filters fused the player's 0.57-deg accelerometer-derived tilt. The player fix above changes
robot_localization's input too (it honours `orientation_covariance[0] = -1`), so the
current-state comparison needs fresh node runs on the corrected player. (CORRECTED 2026-10-26: "honours" is only half true. It stops fusing
the orientation, but its gravity removal then uses its own roll/pitch, which two_d_mode pins at
zero, so with this config it diverged on nine of twelve sequences. See the 2026-10-26 section.) Until then, the
shipped-config node runs from 2026-10-22/23, each filter's trajectory from the SAME run,
scored through hardware/ate_tum.py (full trajectory, ATE RMSE 3D):

    sequence     FusionCore   RL-EKF    better        old player, both fusing the tilt
    2012-01-08      178.808   22.900    RL 7.81x
    2012-03-31       25.887  160.482    FC 6.20x
    2012-05-11        9.605    8.936    RL 1.07x
    2012-09-28       18.603   91.766    FC 4.93x
    2012-10-28       17.220   25.117    FC 1.46x
    2012-11-04       68.232   60.704    RL 1.12x
    2012-12-01       15.948   12.153    RL 1.31x
    2013-02-23       16.934   82.425    FC 4.87x

Four each on these eight. The other four sequences have only frozen-profile node runs on
disk, so they are not in a shipped-against-shipped table. Not a public claim until the
corrected-player twelve exist.

## 2026-10-26  The twelve-sequence evaluation, published; the robot_localization arm withheld

Run under `tools/report_rule_20261025.md` (committed a6034a9 before the run), corrected player
(IMU orientation published with covariance -1), node at 1x on all twelve (filter rate 99.8 to
103.3 Hz, all OK) plus replay of the default and of re-anchoring. Scorer
`tools/score_nclt_suite.py` 803c158e99ff. Every number, generated by script from the score
files, is in `tools/eval_20261025_results.md`; the README Status 2026-10-26 block carries the
fix-time RMSE table. No sequence excluded: the run script's own 2 s start-time check flagged
2012-10-28 and 2013-02-23 (node's first pose 2.2 s and 2.5 s late), but the rule's criterion is
99% of the GPS-fix span and the scorer reports 100.0% and 99.9%.

At GPS fix times: the default (node) against raw GPS, raw lower on 11, 2012-05-11 a tie (9.186
against 9.187 m). Re-anchoring (replay) lower than raw GPS on 2 of 12: 2012-03-31 10.787 against
13.825 m (-22.0%), 2012-10-28 13.150 against 13.923 m (-5.6%), both held out. Re-anchoring against
the default, both replay, full trajectory: RMSE lower on 10, higher on none, identical on 2
(2012-05-11 and 2012-06-15, where it never fires); median higher on 4 (+0.3% to +3.0%); jitter
higher on 3 (up to +0.124). 2012-01-08 through the node: 57.314 m full trajectory, against
178.808 m before the player fix.

Node over replay, default arm: full trajectory 0.699 to 1.140; at fix times 0.849 to 1.138 on
eleven sequences and **2.023 on 2012-06-15 (node 24.363 m, replay 12.041 m), unexplained**.

**robot_localization diverged on nine of twelve** (9 to 68 km; first three sequences 48.267,
379.746, 106.367 m). Mechanism read from its source, not guessed: `rl_ekf.yaml` fused IMU linear
acceleration with gravity removal on, and robot_localization 3.8.3 (`src/ros_filter.cpp` lines
2744 to 2754) removes gravity using the filter's own roll and pitch when the IMU orientation
covariance is -1. In two_d_mode those are zero, so platform tilt leaks in as g sin(tilt) and is
integrated (2012-05-11: estimated speed 1.0 m/s at t = 650 s, 14.4 at 700 s, 52.0 at 800 s). Our
input change, not a property of robot_localization, so the arm is withheld and the README's RL
table is marked pending. `rl_ekf.yaml` now fuses IMU yaw rate only, following the planar
wheeled-robot example in robot_localization's docs minus its x acceleration (same gravity leak);
dated amendment in the rule file. Why three sequences did not diverge is not known; an
unchecked guess is less sustained tilt with GPS holding velocity in check. Not worth time.

The re-run of all twelve robot_localization sequences has NOT completed: one start on 2026-10-26
was paused when the PC slept for about three hours, the restart (batches of four) was stopped on
request. Partial output deleted. Pre-registered next (tools/ship_rule.md addendum, 1b7653c): a
re-anchoring node run on 2012-01-08, confirmed if it reads 9.253 to 12.403 m at fix times, which
gates turning re-anchoring on in `platform.profile: ground_robot`; and a 2012-06-15 repeat read
as nondeterminism at or below 13.703 m, systematic at or above 20.684 m.


## 2026-10-26, evening  The withdrawal had not reached the docs site

No measurement. The README carried the 2026-10-26 evaluation, but the published docs site still
led with "Summary: 12 sequences, 10 FC wins", the May 2026 table with winner columns, and an
explanation of robot_localization's losses (a gate calibrated to optimistic GPS covariance)
that was measured through the faulty player. Same pattern as the 2026-10-22 audit: the
correction was written in the newest place, not everywhere the claim lived. This time the
claim lived in eleven files (docs/index.md, docs/vs-robot-localization.md,
docs/reference/benchmark.md, docs/how-it-works.md, docs/configuration.md, docs/llms.txt, the
NCLT certified config and its index, two blog posts, benchmarks/README.md) plus the mkdocs
nav. All corrected in 0e1e61d and verified on the live site.

Found on the way and also corrected: robot_localization fuses any number of imuN, poseN and
twistN inputs, so "not built-in" was wrong for multiple IMUs, VSLAM pose and velocity sources;
NCLT's fused GPS is a consumer-grade receiver, not a Novatel SPAN-CPT; the 3DM-GX3-45 has a
magnetometer. The SPAN-CPT comment remains in the NCLT YAML configs until the RL re-run,
because nclt_fusioncore.yaml's sha256 is pinned in the evaluation manifest.

The check for next time: `grep -rn` the withdrawn phrase across docs/, docs/llms.txt,
benchmarks/ and mkdocs.yml, then build the site and grep the rendered HTML, which is what
caught the nav label.

## 2026-10-26  Re-anchoring through the ROS node on 2012-01-08: CONFIRMED against the pre-registered band

The check registered in `tools/ship_rule.md` (addendum, 1b7653c) before it ran: one node run,
2012-01-08, corrected player, shipped NCLT config plus `gnss.reanchor_enabled: true`, 1x.
Confirmed if fix-time ATE RMSE 3D reads 9.253 to 12.403 m (replay re-anchoring 10.899 m times the
default arm's node-over-replay band 0.849 to 1.138).

Provenance: installed nclt_fusioncore.yaml sha256 08b4a6e26030c29b, nclt_player.py bc898e768f4153fc
(both as in the published evaluation), the run's config is that file plus the one key (e23abf15e69e),
scorer 803c158e99ff, code a1929b0. Filter rate 101.9 Hz over 5495 s, OK. `ros2 param get` on the
live node read `gnss.reanchor_enabled: True`. Four re-anchor events in the node log: t+3426 s
(141.91 m), t+4048 s (131.64 m), t+4068 s (88.66 m), t+4917 s (117.40 m); replay also fired four.
`messages_lost` 0 throughout. The robot_localization control logged 20 update-rate misses; it is
not scored here.

score_nclt_suite, all arms on the same points (22026 of 22041 fixes in common; 10170 scored at fix
times):

    arm                pipeline  fix-time RMSE / median   full-trajectory RMSE / median   jitter
    raw GPS            raw         9.882 / 6.768
    re-anchoring       node       10.882 / 6.780           13.145 / 6.818                  1.449
    re-anchoring       replay     10.898 / 6.814           18.569 / 6.932                  1.608
    default            node       56.962 / 13.798          57.314 / 13.327                 1.448

**10.882 m is inside 9.253 to 12.403 m: CONFIRMED.** Node over replay for the re-anchoring arm is
0.9985 at fix times (0.9984 against the published 10.899) and 0.708 over the full trajectory. The
replay figure differs from the published 10.899 by 0.001 m because this scoring uses the 22026
fixes common to all three arms rather than 22041.

What this does and does not establish: re-anchoring reaches a deployment through the ROS node and
behaves there as it does offline on this one sequence. It is n=1 on a held-out sequence, through a
node whose run-to-run spread reached 14.8% elsewhere. It does not change the ship-rule verdict on
re-anchoring (failed on median and jitter); it clears the one mechanical gate on the product
decision to enable it in `platform.profile: ground_robot`, which is Manan's to take, with the costs
in tools/eval_20261025_results.md stated.
