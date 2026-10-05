# Benchmark Results

FusionCore vs robot_localization EKF on the [NCLT dataset](http://robots.engin.umich.edu/nclt/) (University of Michigan North Campus Long-Term). Twelve sequences across all seasons, same single config file, no per-sequence tuning.

---

## Summary: 12 sequences, 10 FC wins

| Sequence | Season | Duration | GPS Fixes | Max Blackout | FC ATE (3D) | RL-EKF ATE (3D) | Winner |
|---|---|---|---|---|---|---|---|
| 2012-01-08 | Winter | 92 min | 22,041 | 203s | **18.6 m** | 41.2 m | FC +55% |
| 2012-02-04 | Winter | 77 min | 18,808 | 184s | **49.7 m** | 265.5 m | FC +81% |
| 2012-03-31 | Spring | 87 min | 20,482 | 262s | **22.0 m** | 156.5 m | FC +86% |
| 2012-05-11 | Spring | 84 min | 21,621 | 120s | **9.7 m** | 11.5 m | FC +16% |
| 2012-06-15 | Summer | 55 min | 12,399 | **462s** | 49.2 m † | **18.2 m** | RL +63% |
| 2012-08-20 | Summer | 83 min | 20,025 | 228s | 98.3 m † | **10.6 m** | RL +89% |
| 2012-09-28 | Fall | 77 min | 19,191 | 196s | **22.4 m** | 53.8 m | FC +58% |
| 2012-10-28 | Fall | 85 min | 21,060 | 256s | **15.6 m** | 56.4 m | FC +72% |
| 2012-11-04 | Fall | 79 min | 17,840 | 400s | **60.1 m** | 122.0 m | FC +51% |
| 2012-12-01 | Winter | 75 min | 17,941 | 173s | **21.0 m** | 90.7 m | FC +77% |
| 2013-02-23 | Winter | 78 min | 19,333 | 240s | **59.4 m** | 82.2 m | FC +28% |
| 2013-04-05 | Spring | 68 min | 16,297 | 275s | **12.1 m** † | 268.9 m | FC +96% |

> **Three of these twelve rows no longer describe current `main`, and in all three the published figure is better than every later measurement.** Do not quote the table without this note.
>
> | Sequence | Published (May 2026) | Measured since | RL-EKF control, published vs measured |
> |---|---|---|---|
> | 2012-06-15 | 49.2 m | **73.5 m** (2026-09-13, `bcc0e09`) | 18.2 m vs 18.5 m |
> | 2012-08-20 | 98.3 m | **116.4 m** (2026-09-13, `bcc0e09`) | 10.6 m vs 10.5 m |
> | 2013-04-05 | 12.1 m | **189.7 m** (`bcc0e09`), then **50.2 m** (2026-10-03, current `main`) | 268.9 m vs 266.7 to 268.8 m |
>
> The RL-EKF control reproduces within 2% on all three rows. That is what rules out the harness and places the move in FusionCore's own column. On 2013-04-05 the margin over RL-EKF is 81% at the current figure and 29% at the September one, not the 93% this note claimed until 2026-10-05, and the 19.4 m it quoted is not supported by any run on file. The other nine rows have no run record and have not been re-measured in either direction.
>
> A controlled full-suite re-run is the open work. `tools/benchmark_regression.md` is the standing rule: no published number changes without a `verified` baseline entry and a clean regression check. Rows this note affects are marked † in every table on this page.

ATE = absolute trajectory error, SE3-aligned to RTK GPS ground truth. GPS Fixes = mode-3 (3D) fixes only, as published by nclt_player.

RL-UKF: NaN divergence on all twelve sequences (known numerical instability under sim-time playback, confirmed by RL maintainer). Excluded from results.

---

## Full metrics table

| Sequence | Filter | ATE 3D | ATE XY | Within 5m | Within 10m | Drift (m/km) | RPE@10m |
|---|---|---|---|---|---|---|---|
| 2012-01-08 | FusionCore | **18.6 m** | **16.9 m** | 26.7% | 74.5% | **2.55** | **22.5 m** |
|            | RL-EKF     | 41.2 m | 41.0 m | 22.8% | 76.5% | 5.64 | 25.3 m |
| 2012-02-04 | FusionCore | **49.7 m** | **31.5 m** | 6.4% | 33.4% | **5.96** | **30.0 m** |
|            | RL-EKF     | 265.5 m | 265.4 m | 0.0% | 0.1% | 31.84 | 44.3 m |
| 2012-03-31 | FusionCore | **22.0 m** | **20.2 m** | 19.9% | 67.3% | **2.27** | **21.4 m** |
|            | RL-EKF     | 156.5 m | 156.3 m | 0.2% | 0.6% | 16.16 | 42.7 m |
| 2012-05-11 | FusionCore | **9.7 m** | **4.9 m** | 45.9% | 82.6% | **1.05** | 19.0 m |
|            | RL-EKF     | 11.5 m | 9.0 m | 56.2% | 90.1% | 1.25 | **20.2 m** |
| 2012-06-15 | FusionCore | 49.2 m † | 48.4 m | 2.4% | 20.0% | 8.40 | 22.4 m |
|            | RL-EKF     | **18.2 m** | **17.1 m** | 42.8% | 78.4% | **3.11** | **22.3 m** |
| 2012-08-20 | FusionCore | 98.3 m † | 97.9 m | 0.1% | 13.8% | 13.08 | 53.7 m |
|            | RL-EKF     | **10.6 m** | **9.9 m** | 59.4% | 89.3% | **1.40** | **19.1 m** |
| 2012-09-28 | FusionCore | **22.4 m** | **19.2 m** | 24.1% | 72.1% | **3.10** | **23.5 m** |
|            | RL-EKF     | 53.8 m | 53.5 m | 3.9% | 24.9% | 7.45 | 27.6 m |
| 2012-10-28 | FusionCore | **15.6 m** | **11.4 m** | 24.2% | 61.8% | **1.93** | **27.0 m** |
|            | RL-EKF     | 56.4 m | 56.1 m | 0.4% | 11.4% | 6.96 | 28.4 m |
| 2012-11-04 | FusionCore | **60.1 m** | **59.2 m** | 3.8% | 29.5% | **9.86** | **32.3 m** |
|            | RL-EKF     | 122.0 m | 121.9 m | 0.0% | 0.0% | 20.02 | 37.0 m |
| 2012-12-01 | FusionCore | **21.0 m** | **14.6 m** | 24.3% | 65.4% | **2.90** | 32.9 m |
|            | RL-EKF     | 90.7 m | 90.5 m | 5.3% | 20.6% | 12.53 | 42.1 m |
| 2013-02-23 | FusionCore | **59.4 m** | **58.5 m** | 1.6% | 16.2% | **6.67** | **24.1 m** |
|            | RL-EKF     | 82.2 m | 81.8 m | 0.0% | 0.6% | 9.23 | 35.0 m |
| 2013-04-05 | FusionCore | **12.1 m** † | **10.1 m** | 32.8% | 81.5% | **2.26** | 30.2 m |
|            | RL-EKF     | 268.9 m | 268.7 m | 0.0% | 0.0% | 50.11 | **27.3 m** |

---

## Methodology

**Dataset:** NCLT (University of Michigan, 2012-2013). Wheeled robot (Segway RMP) driving on a large campus over multiple seasons. Raw CSV sensor files replayed at 1x real time via `nclt_player`.

**Sensors used (identical inputs to both filters):**

- IMU: Microstrain 3DM-GX3-45 at 100 Hz (raw specific force, no factory gravity removal)
- Wheel odometry: Segway RMP encoders at 100 Hz
- GPS: Novatel SPAN-CPT, ~3m CEP, 5 Hz

**Ground truth:** RTK GPS (`gps_rtk.csv`), projected to local ENU via PROJ/WGS84. Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3)-aligned ATE.

**FusionCore config:** Single YAML file (`fusioncore_datasets/config/nclt_fusioncore.yaml`), identical across all twelve sequences. No per-sequence tuning.

**RL-EKF config:** `two_d_mode: true` (flat-terrain Segway assumption), GPS fused via `navsat_transform` with a fixed datum from the first valid RTK fix. Matching chi-squared gating thresholds to FusionCore (`odom0_twist_rejection_threshold: 4.03`, `odom1_pose_rejection_threshold: 3.72`).

**RL-UKF:** Diverged with NaN on all sequences during sim-time playback (rapid timer catchup causes near-zero dt, Cholesky failure). Excluded from results.

---

## What drives the results

### Why RL-EKF fails on 10 sequences

The drift rate column tells the story most clearly. RL drift rates of 31.84 m/km (2012-02-04), 50.11 m/km (2013-04-05), and 20.02 m/km (2012-11-04) mean the filter is operating without GPS for large portions of those runs. A Segway at 1.5 m/s accumulating 31 m per kilometer traveled is in pure dead-reckoning almost the entire time.

The cause is consistent across all RL failures: `nclt_player` publishes `position_covariance var_xy=9` (3m sigma), which is the Novatel SPAN-CPT specification under ideal open-sky conditions. Measured against the RTK ground truth, actual GPS noise across all twelve sequences looks like this:

| Sequence | Median error | p95 error | p99 error | RL result |
|---|---|---|---|---|
| 2012-01-08 | 3.7 m | 20.1 m | 49.7 m | 41.2 m |
| 2012-02-04 | 5.6 m | 46.6 m | **234.9 m** | 265.5 m |
| 2012-03-31 | 5.7 m | 14.7 m | 32.7 m | 156.5 m |
| 2012-05-11 | 3.3 m | 13.3 m | 47.7 m | 11.5 m |
| 2012-06-15 | 2.6 m | 9.7 m | 21.3 m | **18.2 m** (RL wins) |
| 2012-08-20 | 3.4 m | 12.7 m | 55.0 m | **10.6 m** (RL wins) |
| 2012-09-28 | 3.5 m | 12.8 m | 43.2 m | 53.8 m |
| 2012-10-28 | 4.6 m | 16.0 m | 48.9 m | 56.4 m |
| 2012-11-04 | 5.7 m | **53.1 m** | 79.2 m | 122.0 m |
| 2012-12-01 | 4.7 m | 20.7 m | 80.4 m | 90.7 m |
| 2013-02-23 | 5.4 m | 33.0 m | 73.6 m | 82.2 m |
| 2013-04-05 | 3.7 m | 19.9 m | 87.8 m | 87.8 m |

The driver states 3m sigma (var_xy=9). The median actual error is 2.6-5.7m across all sequences (already at or above the stated 1-sigma), and p95 ranges from 9.7m to 53.1m. RL's gate is calibrated to the stated 3m; it rejects anything beyond roughly 3x that (Mahalanobis distance above the chi2 threshold). On sequences like 2012-02-04 and 2012-11-04, most GPS fixes are outliers by RL's definition of "outlier."

The two sequences where RL wins (2012-06-15 and 2012-08-20) have the cleanest GPS of the set: p95 of 9.7m and 12.7m respectively. RL's tight gate works when the actual noise matches the stated noise. It fails everywhere else.

The contrast on 2012-05-11 (RL drift: 1.25 m/km vs 31.84 m/km on 2012-02-04) makes the mechanism concrete. Same robot, same campus, same config. The only difference is GPS data quality on that specific day. When GPS covariance matches actual sensor noise, both filters perform comparably (9.7m vs 11.5m). The advantage opens when the reported covariance is too tight.

FusionCore's `adaptive.gnss: true` adjusts GPS measurement noise in real time from the innovation sequence. When actual GPS noise is higher than the driver reports, the adaptive window inflates the noise model and keeps chi2 statistics calibrated. RL has no equivalent.

**What would help RL:** Increasing `position_covariance var_xy` in nclt_player from 9 to 25 (5m sigma, closer to actual NCLT GPS accuracy in urban conditions) would reduce RL's catastrophic losses substantially without per-sequence tuning. This does not require modifying robot_localization itself, only the dataset player. However, RL would still lack adaptive noise, and the calibration burden would remain whenever the dataset or environment changes.

### What drives FC performance variation

The single best predictor of FC ATE is the longest GPS blackout in the sequence:

| Max blackout | Sequences | FC ATE range |
|---|---|---|
| < 200s | 2012-01-08 (203s), 2012-12-01 (173s) | 18-21 m |
| 200-300s | 2012-03-31, 2012-05-11, 2012-09-28, 2012-10-28, 2013-04-05 | 10-30 m |
| 300-480s | 2012-02-04, 2012-06-15, 2012-11-04, 2013-02-23 | 49-60 m |
| Adversarial GPS | 2012-08-20 (228s blackout + 105 corrupt fixes at boundary) | 98.3 m |

FC drift rate is consistent at 1-4 m/km on clean sequences. Values above 6 m/km (2012-06-15, 2012-08-20, 2012-11-04, 2013-02-23) signal heading error accumulated during coast mode. The 2012-08-20 transient is a distinct failure mode: adversarial GPS data at the blackout boundary, not heading drift.

---

## FC performance tiers

**Excellent (< 20m ATE):** 2012-05-11 (9.7m), 2013-04-05 (12.1m) †, 2012-10-28 (15.6m), 2012-01-08 (18.6m)

High GPS fix count (19k-22k), max blackout under 300s, no adversarial GPS. FC operates as intended.

**Good (20-35m ATE):** 2012-12-01 (21.0m), 2012-03-31 (22.0m), 2012-09-28 (22.4m)

Moderate GPS density, blackouts under 275s, clean GPS at boundaries. Occasional heading drift corrected quickly on GPS return.

**Moderate (35-65m ATE):** 2012-02-04 (49.7m), 2012-06-15 (49.2m) †, 2013-02-23 (59.4m), 2012-11-04 (60.1m)

Long blackouts (240-462s) or low GPS density. Heading drift compounds over coast mode duration before correction.

**Poor (> 65m ATE):** 2012-08-20 (98.3m) †

Structurally different failure: adversarial GPS cluster at blackout boundary.

These tiers are derived from the May 2026 table above. All three sequences re-measured since would move: 2013-04-05 from Excellent to Moderate (50.2 m), 2012-06-15 from Moderate to Poor (73.5 m), and 2012-08-20 stays Poor (116.4 m). The tiers are left as published rather than half-rewritten from three rows out of twelve. Outside the 2-3 minute transient windows, FC tracks at 5-10m, on-par with RL-EKF.

---

## The two FC losses: honest analysis

### 2012-06-15 (FC 49.2m, RL 18.2m)

The lowest-density GPS sequence in the set: 12,399 mode-3 fixes vs 17,000-22,000 on others. One GPS blackout of 462 seconds (7.7 minutes).

During the blackout, FC dead-reckons on encoder and IMU. Coast mode inflates `Q_position` (coast_q_factor=10) and down-weights IMU WZ (coast_imu_wz_scale=500), so encoder WZ dominates heading. The encoder WZ bias (B_EWZ) is calibrated from GPS heading cross-covariance before the blackout and subtracted during it. However, any residual B_EWZ error compounds over 7.7 minutes. At 100 Hz with even a small uncorrected heading rate, lateral position error grows quadratically.

RL-EKF wins here because its 2D mode has a simpler state vector and accumulates less uncertainty over the blackout. This is a structural advantage for RL on GPS-sparse sequences with very long blackouts on flat terrain. See [issue #63](https://github.com/manankharwar/fusioncore/issues/63).

**Path to fixing this:**

- Reduce `coast_imu_wz_scale` from 500 to 50-100 for blackouts exceeding 200s. At 500x down-weighting, the IMU WZ is essentially ignored. Both sensors sharing heading responsibility reduces sensitivity to B_EWZ residual error.
- Magnetometer integration is available now (`magnetometer.enabled: true`): an absolute heading reference during a GPS outage bounds the heading drift instead of letting it accumulate, demonstrated in a unit test with slipping wheel odometry. Honest caveat: this cannot be validated against *this NCLT number*, the dataset publishes no usable magnetometer and its ground-truth orientation is too noisy to score a few-metre change. So it is validated by construction and in test, and awaits real-hardware confirmation; it is not proven to close this specific loss.
- Duration-dependent `coast_q_factor`: the current fixed 10x multiplier was tuned for the majority of sequences. For blackouts > 300s, a nonlinear ramp may reduce heading drift without sacrificing re-acquisition on short blackouts.

### 2012-08-20 (FC 98.3m, RL 10.6m)

The raw GPS stream contains 105 mode-3 fixes 720-840m off the RTK ground truth in `gps.csv`. The ground-truth preprocessor excludes them but they appear as valid mode-3 fixes in the real data stream. They cluster in a 24-second window at the end of the second GPS blackout (211s at t=62.5 min).

This is adversarial for any chi2-based gating scheme. During coast mode recovery, FC relaxes the chi2 gate to accept the first valid returning fix after genuine position drift. A dense cluster of corrupt fixes arriving at exactly the re-acquisition moment exploits this window.

Per-minute error analysis:

| Time | FC error | Status |
|---|---|---|
| 0-42 min | 1-10 m | Normal GPS coverage, both filters tracking well |
| 43-46 min | spike to ~100m, recovers | Blackout 1 (228s): boundary GPS errors up to ~70m |
| 47-62 min | 3-10 m | Full recovery |
| 63-67 min | spike to ~788m, recovers in 2 min | Blackout 2 (211s): 105 adversarial fixes at boundary |
| 68-82 min | 5-10 m | Full recovery, remaining 15 minutes on-par with RL |

The transient spikes are large, but the 98m ATE is not the cluster alone: even with the cluster rejected (see the plausibility gate below), the ATE stays high because of the dead-reckoning drift accumulated during the 211s blackout. RL-EKF wins because its tight Mahalanobis gate (calibrated to the stated 3m sigma, which causes GPS rejection on 10 other sequences) rejects these outliers too, and its 2D model drifts less through the blackout. See [issue #64](https://github.com/manankharwar/fusioncore/issues/64).

**Path to fixing this:**

- **Physical-plausibility gate (shipped):** A GPS fix 720m from the dead-reckoned position after a 211s blackout implies impossible motion. FusionCore ships `gnss.max_speed`, which rejects any fix farther from the prediction than `max_speed * gap + margin`; on this sequence it rejects the cluster and cuts the peak spike. Honest caveat: it does *not* fix the score, because the ATE is dominated by the dead-reckoning drift accumulated *during* the blackout, not by the cluster spike.
- **Cluster consistency gate:** Five consecutive fixes all landing 720-840m from the predicted position with geometric consistency (tight cluster, not random scatter) is distinguishable from noise. A secondary check on cluster coherence catches this without affecting single-fix rejection behavior.
- **Gate hysteresis on recovery:** Instead of a step change in chi2 threshold at re-acquisition, a linear ramp from relaxed back to nominal over the first N returned fixes makes it harder for a dense cluster to slip through entirely.

---

## Reproduce

```bash
# Build
colcon build --packages-select fusioncore_core fusioncore_ros fusioncore_datasets

# Run one sequence (auto-stops on playback complete, ~60-95 min at 1x)
bash benchmarks/run_one.sh 2012-01-08

# Results written to:
# benchmarks/nclt/2012-01-08/results_full/BENCHMARK.md

# Run all 12 sequences sequentially (plan for 15-20 hours total)
bash benchmarks/run_all.sh
```

Full tooling and configs in [`benchmarks/`](https://github.com/manankharwar/fusioncore/tree/main/benchmarks).
