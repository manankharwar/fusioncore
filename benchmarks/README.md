# FusionCore NCLT Benchmarks

FusionCore vs robot_localization EKF on the [NCLT dataset](http://robots.engin.umich.edu/nclt/) (University of Michigan North Campus Long-Term). Twelve sequences across all seasons, same single config file, no per-sequence tuning.

---

## Results: 2 of the 5 verified rows are FusionCore wins

**The "10/12 FC wins" headline that stood here is withdrawn.** It came from May 2026 runs through a dataset player with a frame bug (#169) that fed a wrong wheel-odometry yaw rate to both filters. Five rows have since been re-measured at n=2; FusionCore wins two. The other seven have never been re-run.

| Sequence | Season | Duration | GPS Fixes | Max Blackout | FC ATE 3D | RL-EKF ATE 3D | Winner |
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
> | 2012-06-15 | 49.2 m | **73.5 m** (`bcc0e09`), confirmed **73.3 m** on current `main` 2026-10-05 | 18.2 m vs 18.487 then 18.489 m |
> | 2012-08-20 | 98.3 m | **116.4 m** (`bcc0e09`), now **145.2 m** on current `main` 2026-10-05 | 10.6 m vs 10.519 then 10.507 m |
> | 2013-04-05 | 12.1 m | **189.7 m** (`bcc0e09`), then **50.2 m** (2026-10-03) | 268.9 m vs 266.7 to 268.8 m |
>
> The RL-EKF control reproduces within 2% on all three rows. That is what rules out the harness and places the move in FusionCore's own column. On 2013-04-05 the margin over RL-EKF is 81% at the current figure and 29% at the September one, not the 93% this note claimed until 2026-10-05, and the 19.4 m it quoted is not supported by any run on file. The other nine rows have no run record and have not been re-measured in either direction.
>
> The two rows re-measured on 2026-10-05 are load-bearing because their controls held: the RL-EKF figure moved **+0.01%** on 2012-06-15 and **-0.11%** on 2012-08-20, against an established band of under 1%. 2012-06-15 is now confirmed three independent times and sits inside its own n=8 spread. 2012-08-20 is worse than previously recorded and is a genuine regression, outside its 4.57% spread, so the baseline has deliberately not been moved to match it. The 2013-04-05 arm of that re-run is **void**: its control moved -4.40% after 20 `rl_ekf` update-rate misses, so it was CPU-starved and says nothing. It is being re-run.
>
> **CORRECTED 2026-10-09.** Five of these twelve rows have now been re-measured at n=2
> after fixing #169, a frame error in the NCLT player that fed a wrong wheel-odometry yaw
> rate to BOTH filters. **On those five, FusionCore wins two and loses three.** The other
> seven were measured in May 2026 through the faulty player and have never been re-run, so
> nothing should be claimed about them. The earlier note here, that the overstatement was
> systematic at about 1.48x, is withdrawn: across five rows the ratio runs 0.83x to 3.08x
> and the control swings 0.24x to 2.52x despite sharing no code. See the table and the
> explanation in the repository README.
>
> `tools/benchmark_regression.md` is the standing rule: no published number changes without a `verified` baseline entry and a clean regression check. Rows this note affects are marked † in every table on this page.

ATE = absolute trajectory error, SE3-aligned to RTK GPS ground truth. GPS Fixes = mode-3 (3D) fixes only, as published by nclt_player.

RL-UKF: NaN divergence on all sequences (known numerical instability under sim-time playback, confirmed by RL maintainer).

---

## Full metrics

| Sequence | Filter | ATE 3D | ATE XY | Within 5m | Within 10m | Drift m/km | RPE@10m |
|---|---|---|---|---|---|---|---|
| 2012-01-08 | FusionCore | **18.6 m** | **16.9 m** | 26.7% | 74.5% | **2.55** | **22.5 m** |
| | RL-EKF | 41.2 m | 41.0 m | 22.8% | 76.5% | 5.64 | 25.3 m |
| 2012-02-04 | FusionCore | **49.7 m** | **31.5 m** | 6.4% | 33.4% | **5.96** | **30.0 m** |
| | RL-EKF | 265.5 m | 265.4 m | 0.0% | 0.1% | 31.84 | 44.3 m |
| 2012-03-31 | FusionCore | **22.0 m** | **20.2 m** | 19.9% | 67.3% | **2.27** | **21.4 m** |
| | RL-EKF | 156.5 m | 156.3 m | 0.2% | 0.6% | 16.16 | 42.7 m |
| 2012-05-11 | FusionCore | **9.7 m** | **4.9 m** | 45.9% | 82.6% | **1.05** | 19.0 m |
| | RL-EKF | 11.5 m | 9.0 m | 56.2% | 90.1% | 1.25 | 20.2 m |
| 2012-06-15 | FusionCore | 49.2 m † | 48.4 m | 2.4% | 20.0% | 8.40 | 22.4 m |
| | RL-EKF | **18.2 m** | **17.1 m** | 42.8% | 78.4% | **3.11** | **22.3 m** |
| 2012-08-20 | FusionCore | 98.3 m † | 97.9 m | 0.1% | 13.8% | 13.08 | 53.7 m |
| | RL-EKF | **10.6 m** | **9.9 m** | 59.4% | 89.3% | **1.40** | **19.1 m** |
| 2012-09-28 | FusionCore | **22.4 m** | **19.2 m** | 24.1% | 72.1% | **3.10** | **23.5 m** |
| | RL-EKF | 53.8 m | 53.5 m | 3.9% | 24.9% | 7.45 | 27.6 m |
| 2012-10-28 | FusionCore | **15.6 m** | **11.4 m** | 24.2% | 61.8% | **1.93** | **27.0 m** |
| | RL-EKF | 56.4 m | 56.1 m | 0.4% | 11.4% | 6.96 | 28.4 m |
| 2012-11-04 | FusionCore | **60.1 m** | **59.2 m** | 3.8% | 29.5% | **9.86** | **32.3 m** |
| | RL-EKF | 122.0 m | 121.9 m | 0.0% | 0.0% | 20.02 | 37.0 m |
| 2012-12-01 | FusionCore | **21.0 m** | **14.6 m** | 24.3% | 65.4% | **2.90** | 32.9 m |
| | RL-EKF | 90.7 m | 90.5 m | 5.3% | 20.6% | 12.53 | 42.1 m |
| 2013-02-23 | FusionCore | **59.4 m** | **58.5 m** | 1.6% | 16.2% | **6.67** | **24.1 m** |
| | RL-EKF | 82.2 m | 81.8 m | 0.0% | 0.6% | 9.23 | 35.0 m |
| 2013-04-05 | FusionCore | **12.1 m** † | **10.1 m** | 32.8% | 81.5% | **2.26** | 30.2 m |
| | RL-EKF | 268.9 m | 268.7 m | 0.0% | 0.0% | 50.11 | **27.3 m** |

---

## What drives the results

### Why RL-EKF fails on 10 sequences

The drift rate column is the clearest signal. RL drift rates of 31.84 m/km (2012-02-04), 50.11 m/km (2013-04-05), and 20.02 m/km (2012-11-04) mean the filter is operating without GPS for large portions of those runs. A Segway at 1.5 m/s accumulating 31 m/km is in pure dead-reckoning almost the entire time.

The cause is always the same: the nclt_player publishes `position_covariance var_xy = 9` (3m sigma) because that is the Novatel SPAN-CPT specification in ideal conditions. Measured against the RTK ground truth, actual GPS noise looks like this:

| Sequence | Median error | p95 error | p99 error |
|---|---|---|---|
| 2012-01-08 | 3.7 m | 20.1 m | 49.7 m |
| 2012-02-04 | 5.6 m | 46.6 m | **234.9 m** |
| 2012-03-31 | 5.7 m | 14.7 m | 32.7 m |
| 2012-05-11 | 3.3 m | 13.3 m | 47.7 m |
| 2012-06-15 | 2.6 m | 9.7 m | 21.3 m |
| 2012-08-20 | 3.4 m | 12.7 m | 55.0 m |
| 2012-09-28 | 3.5 m | 12.8 m | 43.2 m |
| 2012-10-28 | 4.6 m | 16.0 m | 48.9 m |
| 2012-11-04 | 5.7 m | **53.1 m** | 79.2 m |
| 2012-12-01 | 4.7 m | 20.7 m | 80.4 m |
| 2013-02-23 | 5.4 m | 33.0 m | 73.6 m |
| 2013-04-05 | 3.7 m | 19.9 m | 87.8 m |

The driver states 3m sigma. Median actual error is 2.6-5.7m (already at or above the stated 1-sigma on most sequences). p95 ranges from 9.7m to 53.1m. RL's gate is calibrated to the stated 3m; it rejects most fixes on sequences like 2012-02-04 and 2012-11-04. GPS is effectively off for those sequences, and the filter runs in dead-reckoning.

The contrast on 2012-05-11 (RL drift: 1.25 m/km vs 31.84 m/km on 2012-02-04) is the clearest evidence. Same robot, same campus, same config. The only difference is GPS data quality on that day. When GPS covariance is accurate, both filters perform comparably (9.7m vs 11.5m). The advantage opens when the covariance is wrong.

FusionCore's `adaptive.gnss: true` adjusts GPS measurement noise in real time from the innovation sequence. When actual GPS noise is higher than the driver reports, the adaptive window inflates the noise model and keeps chi2 statistics calibrated. RL has no equivalent.

**What would improve RL:** A single global change (not per-sequence tuning) would help substantially: increasing `position_covariance var_xy` in the nclt_player from 9 to 25 (5m sigma, reflecting actual NCLT GPS accuracy in urban conditions). This would bring RL's catastrophic losses (265m, 268m, 156m) down significantly. However, RL has no equivalent to FusionCore's `adaptive.gnss: true`. The calibration burden would remain whenever the dataset or environment changes.

### What drives FC performance variation

The single best predictor of FC ATE is the longest GPS blackout in the sequence:

| Max blackout | Sequences | FC ATE range |
|---|---|---|
| < 200s | 2012-01-08, 2012-12-01 | 18-21m |
| 200-300s | 2012-03-31, 2012-05-11, 2012-09-28, 2012-10-28, 2013-04-05 | 10-30m |
| 300-480s | 2012-02-04, 2012-06-15, 2012-11-04, 2013-02-23 | 49-60m |
| Adversarial | 2012-08-20 (228s but 105 corrupt fixes at boundary) | 98.3m |

FC drift rate is consistent at 1-4 m/km on clean sequences. Spikes above 6 m/km (2012-06-15, 2012-08-20, 2012-11-04, 2013-02-23) signal heading error accumulated during coast mode.

---

## The two FC losses

### 2012-06-15 (FC 49.2m, RL 18.2m)

The lowest-density GPS sequence in the set. 12,399 mode-3 fixes vs 17,000-22,000 on other sequences. One GPS blackout of 462 seconds (7.7 minutes).

During the blackout, FC dead-reckons on encoder and IMU. Coast mode inflates `Q_position` (coast_q_factor=10) and down-weights IMU WZ (coast_imu_wz_scale=500) so encoder WZ dominates heading. The encoder WZ bias (B_EWZ) is calibrated from GPS heading cross-covariance before the blackout and subtracted during it. However, any residual B_EWZ error compounds over 7.7 minutes. At 100 Hz with even a small uncorrected heading rate error, lateral position error grows quadratically.

RL-EKF wins here because its 2D mode has a simpler state and accumulates less uncertainty over the blackout. This is a structural advantage for RL on GPS-sparse, flat-terrain sequences with very long blackouts.

**Path to fixing this:**
- Reduce `coast_imu_wz_scale` from 500 to 50-100 for blackouts exceeding 200s. At 500x, the IMU WZ is essentially ignored during coast. Both sensors sharing heading responsibility reduces B_EWZ sensitivity.
- Magnetometer integration closes the observability gap completely: an absolute heading reference during GPS absence makes B_GZ and B_EWZ irrelevant. This is the architecturally correct fix and is on the roadmap.
- Duration-dependent `coast_q_factor`: the current fixed 10x multiplier was tuned for the majority of sequences. For blackouts > 300s, a nonlinear ramp (aggressive early, conservative late) may reduce heading drift without sacrificing re-acquisition.

### 2012-08-20 (FC 98.3m, RL 10.6m)

The raw GPS stream contains **105 mode-3 fixes 720-840m off the RTK ground truth** in `gps.csv`. The ground-truth preprocessor excludes them from `gps_rtk.csv` but they are valid mode-3 fixes in the real data stream. They cluster in a 24-second window at the end of the second GPS blackout (211s at t=62.5 min).

This is adversarial for any chi2-based gating scheme. During blackout recovery, FC's coast mode relaxes the chi2 gate to accept the first valid returning fix after genuine drift. A cluster of corrupt fixes arriving at exactly the re-acquisition moment exploits this window.

Per-minute error analysis:

| Time | FC error | Status |
|---|---|---|
| 0-42 min | 1-10m | Normal GPS coverage |
| 43-46 min | spike to ~100m, recovers in 2-3 min | Blackout 1 (228s): boundary GPS errors up to ~70m |
| 47-62 min | 3-10m | Full recovery |
| 63-67 min | spike to ~788m, recovers in 2 min | Blackout 2 (211s): 105 adversarial fixes at boundary |
| 68-82 min | 5-10m | Full recovery, remaining 15 minutes on-par with RL |

The 98m ATE RMSE is driven entirely by those two transients. RL-EKF wins here because its tight gate (which fails on 10 other sequences) accidentally rejects these outliers too.

**Path to fixing this:**
- **Velocity sanity check:** A GPS fix 720m from the dead-reckoned position after a 211s blackout implies ~3600 m/s of motion. A hard `max_implied_speed` check (e.g., 20 m/s) operating before the chi2 gate rejects this trivially and has zero effect on normal operation.
- **Cluster consistency gate:** A single outlier at 720m is handled by chi2. Five consecutive fixes all landing 720-840m from the predicted position with geometric consistency (tight cluster, not random scatter) is a distinguishable pattern. A secondary check on cluster coherence would catch this without affecting single-fix behavior.
- **Gate hysteresis on recovery:** Instead of a step change in chi2 threshold at recovery, a linear ramp from relaxed back to tight over the first N returned fixes makes it harder for a cluster to slip through entirely.

---

## FC performance tier breakdown

**Excellent (< 20m ATE):** 2012-05-11 (9.7m), 2013-04-05 (12.1m) †, 2012-10-28 (15.6m), 2012-01-08 (18.6m)
Common: high GPS fix count (19k-22k), max blackout under 300s, no adversarial data.

**Good (20-35m ATE):** 2012-12-01 (21.0m), 2012-03-31 (22.0m), 2012-09-28 (22.4m)
Common: moderate GPS density, one or two blackouts under 260s, clean GPS at boundaries.

**Moderate (35-65m ATE):** 2012-02-04 (49.7m), 2012-06-15 (49.2m) †, 2013-02-23 (59.4m), 2012-11-04 (60.1m)
Common: long blackouts (240-462s) or low GPS density, heading drift accumulates.

**Poor (> 65m ATE):** 2012-08-20 (98.3m) †
Specific cause: adversarial GPS cluster at blackout boundary. Structurally different failure mode from all other sequences.

These tiers are derived from the May 2026 table above. All three sequences re-measured since would move: 2013-04-05 from Excellent to Moderate (50.2 m), 2012-06-15 from Moderate to Poor (73.5 m), and 2012-08-20 stays Poor (116.4 m). The tiers are left as published rather than half-rewritten from three rows out of twelve.

---

## Methodology

**Filters compared:**
- **FusionCore:** 23-state UKF, full 3D, adaptive noise from innovation sequence, GPS chi2 gating with coast mode, gyro + accel + encoder WZ bias estimation, inertial coast mode, ZUPT.
- **RL-EKF:** robot_localization EKF with `two_d_mode: true`, GPS via `navsat_transform` with a fixed RTK datum. Chi2 gating set to equivalent confidence level: `odom0_twist_rejection_threshold: 4.03` (chi2(3, 0.999)), `odom1_pose_rejection_threshold: 3.72` (chi2(2, 0.999)).

**Sensor inputs (identical to both filters):**
- IMU: Microstrain 3DM-GX3-45, 100 Hz, raw specific force (gravity not removed by driver)
- Wheel odometry: Segway RMP encoders, 100 Hz, from `odometry_mu_100hz.csv`
- GPS: Novatel SPAN-CPT, 5 Hz, ~3m CEP in urban Michigan campus, published as NavSatFix with `position_covariance var_xy=9`

**Ground truth:** RTK GPS (`gps_rtk.csv`), projected to local ENU. Only fixes with RTK mode >= 3 are used.

**Evaluation:** [evo](https://github.com/MichaelGrupp/evo) `evo_ape` with `--align` (SE3 alignment). ATE computed after finding the best rigid-body transform between filter trajectory and RTK ground truth.

**Config:** Single YAML for all sequences: `fusioncore_datasets/config/nclt_fusioncore.yaml`. No per-sequence modifications.

---

## Reproduce

### 1. Install dependencies

```bash
# ROS 2 Jazzy (Ubuntu 24.04 native or your distro)
source /opt/ros/jazzy/setup.bash

# robot_localization
sudo apt install ros-jazzy-robot-localization

# Python tools
pip install evo matplotlib --break-system-packages
```

### 2. Build FusionCore

```bash
cd /path/to/fusioncore
colcon build --packages-select fusioncore_core fusioncore_ros fusioncore_datasets
source install/setup.bash
```

### 3. Check prerequisites

```bash
bash benchmarks/check_prereqs.sh
```

Every check must pass before proceeding. A `[FAIL]` on evo is the common
one: `pip install evo --break-system-packages`, or put it in a venv at
`~/.venv-evo`, which `tools/run_nclt.sh` prefers when it exists.

### 4. Download NCLT data

```bash
# The three sequences the baseline gates on (250 MB compressed, 681 MB on disk)
bash benchmarks/nclt_download.sh baseline

# One sequence (66 to 114 MB compressed)
bash benchmarks/nclt_download.sh 2012-06-15

# All 12 sequences (1.12 GB compressed)
bash benchmarks/nclt_download.sh all
```

The dataset is served from S3 as one tarball per sequence. Sizes above were
read from the server on 2026-10-08, not estimated.

Data lands in `$NCLT_DATA` (default `~/nclt/<date>/`), which is outside the
repo, and the script symlinks `benchmarks/nclt/<date>/raw files` to it so both
harnesses find it: `tools/run_nclt.sh` reads the data root directly and
`benchmarks/run_one.sh` reads the `raw files` path.

The data root is kept out of the repo deliberately. It is not ours to
redistribute, and the hourly local backup excludes it because it is
re-downloadable from the address above.

### 5. Run one sequence (full length, auto-stops)

```bash
bash benchmarks/run_one.sh 2012-01-08
```

Takes 60-95 minutes depending on sequence length (running at 1x real time). Results write to `benchmarks/nclt/2012-01-08/results_full/`.

### 6. Run all sequences sequentially

```bash
bash benchmarks/run_all.sh
```

Runs all 12 sequences in chronological order. Plan for 15-20 hours total.

### Zero-download demo (no ROS required)

Pre-baked results for the GPS spike test are committed to the repo. Reproduce the plot in seconds:

```bash
pip install numpy matplotlib --break-system-packages
python3 tools/demo_quick.py --open
```

This shows the 707 m GPS spike injection test: FusionCore chi-squared gate blocks it, RL-EKF accepts and diverges 50+ m off-course.

---

## Directory structure

```
benchmarks/
  README.md               <- this file
  run_one.sh              <- run one sequence end-to-end
  run_all.sh              <- run all sequences sequentially
  nclt/
    2012-01-08/
      raw files/          <- symlink to $NCLT_DATA/<date> (not committed)
        ms25.csv          <- IMU (100 Hz)
        ms25_euler.csv    <- IMU Euler angles
        odometry_mu_100hz.csv  <- wheel encoder (100 Hz)
        gps.csv           <- GPS fixes (5 Hz, raw including outliers)
        gps_rtk.csv       <- RTK GPS (5 Hz, clean ground truth)
      bag_full/           <- ROS 2 bag from the run (not committed, large)
      fusioncore.tum      <- FC trajectory (not committed, regenerate)
      rl_ekf.tum          <- RL-EKF trajectory (not committed, regenerate)
      ground_truth.tum    <- RTK ground truth (not committed, regenerate)
      results_full/
        BENCHMARK.md      <- metrics table (committed)
        trajectories.png  <- trajectory overlay plot (committed)
        ate_over_time.png <- ATE vs time plot (committed)
        error_distribution.png <- error histogram (committed)
        launch.log        <- full launch output (committed)
```

`.tum` files, `.mcap` bags, and `bag_full/` directories are in `.gitignore`. Only `results_full/` contents are committed.
