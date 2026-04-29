# Benchmark Results

FusionCore vs robot_localization on the [NCLT dataset](http://robots.engin.umich.edu/nclt/) (University of Michigan). Same IMU + wheel odometry + GPS inputs, no manual tuning. Six sequences, same pipeline.

<p align="center">
  <img src="../../figures/fig1_bar_chart.png" alt="ATE RMSE across 6 NCLT sequences" width="500">
</p>

| Sequence | FC ATE RMSE | RL-EKF ATE RMSE | RL-UKF |
|---|---|---|---|
| 2012-01-08 | **5.6 m** | 23.4 m | NaN divergence at t=31 s |
| 2012-02-04 | **9.7 m** | 20.6 m | NaN divergence at t=22 s |
| 2012-03-31 | **4.2 m** | 10.8 m | NaN divergence at t=18 s |
| 2012-08-20 | **7.5 m** | 9.4 m | NaN divergence |
| 2012-11-04 | 28.7 m | **10.9 m** | NaN divergence |
| 2013-02-23 | **4.1 m** | 5.8 m | NaN divergence |

FusionCore wins 5 of 6 sequences. RL-UKF diverged with NaN on all six.

**Note:** These results were generated with an RL config that had misconfigured Mahalanobis threshold parameter names (parameter was silently ignored). A corrected re-run with properly configured RL gating is in progress and results will be updated here. The RL-UKF NaN divergences are unaffected — the RL maintainer has independently confirmed the UKF has numerical instability issues.

On 2012-11-04 (fall, degraded GPS), FC's chi-squared gate traps itself: sustained GPS degradation causes continuous rejection → state drift → further rejection. RL-EKF re-anchors when signal improves. FusionCore's inertial coast mode reduces this gap (61.4 m → 28.7 m) but RL still wins this sequence.

<p align="center">
  <img src="../../figures/fig2_traj_grid.png" alt="Trajectory overlay: all 6 sequences, SE3-aligned to RTK GPS ground truth" width="700">
</p>

---

## Reproduce

Full methodology, configs, and reproduce instructions in [`benchmarks/`](https://github.com/manankharwar/fusioncore/tree/main/benchmarks).
