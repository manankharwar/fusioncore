# FusionCore NCLT Benchmarks

Tooling and data layout for running FusionCore on the [NCLT dataset](http://robots.engin.umich.edu/nclt/)
(University of Michigan North Campus Long-Term): twelve sequences across all seasons, one config
file, no per-sequence tuning.

---

## Current results, 2026-10-26

Scored at GPS fix times on all twelve sequences, **the shipped default does not beat raw
unfiltered GPS**: raw GPS has the lower ATE RMSE on 11 of 12, and the twelfth, 2012-05-11, is a
tie (9.186 m against 9.187 m). With opt-in re-anchoring (`gnss.reanchor_enabled`), FusionCore is
lower than raw GPS on 2 of 12 held-out sequences (2012-03-31 by 22.0%, 2012-10-28 by 5.6%).
Full table: the README Status 2026-10-26 note and
[`tools/eval_20261025_results.md`](../tools/eval_20261025_results.md).

**There is no current comparison against robot_localization.** The "10/12 FC wins" table and
the analysis that stood here (why RL-EKF fails, performance tiers, the two losses) are
withdrawn. They were measured through our dataset player with two input errors that fed both
filters wrong data: a wheel-odometry yaw rate in the wrong frame (#169) and an
accelerometer-derived tilt declared as a 0.57 degree attitude. A corrected robot_localization
re-run on all twelve sequences is pending, and its results will replace this note. The
withdrawn figures remain in git history and, as a labelled superseded table, in
[`docs/reference/benchmark.md`](../docs/reference/benchmark.md).

---

## Methodology

**Filters compared:**
- **FusionCore:** 23-state UKF, full 3D, adaptive noise from innovation sequence, GPS chi2 gating with coast mode, gyro + accel + encoder WZ bias estimation, inertial coast mode, ZUPT.
- **RL-EKF:** robot_localization EKF with `two_d_mode: true`, GPS via `navsat_transform` with a fixed RTK datum. Chi2 gating set to equivalent confidence level: `odom0_twist_rejection_threshold: 4.03` (chi2(3, 0.999)), `odom1_pose_rejection_threshold: 3.72` (chi2(2, 0.999)).

**Sensor inputs (identical to both filters):**
- IMU: Microstrain 3DM-GX3-45, 100 Hz, raw specific force (gravity not removed by driver); its orientation is published as invalid (`orientation_covariance[0] = -1`) because its roll and pitch are the accelerometer's own tilt
- Wheel odometry: Segway RMP encoders, 100 Hz, from `odometry_mu_100hz.csv`
- GPS: the dataset's consumer-grade receiver (`gps.csv`), 3D fixes only, published as NavSatFix with `position_covariance var_xy=9`

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
