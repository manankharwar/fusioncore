# Benchmark Results: NCLT Sequence 2012-12-01

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE 3D (m) | ATE RMSE XY (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|----------------|----------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 20.991 | 14.602 | 24.3% | 65.4% | 0.7885 | 2.90 | 32.897 |
| RL-EKF | 90.659 | 90.472 | 5.3% | 20.6% | 0.8548 | 12.53 | 42.136 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-12-01
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- All filters consume identical sensor streams: same IMU, wheel odometry, and GPS topics
- FusionCore: full 3D UKF, adaptive noise, ZUPT, IMU bias estimation
- RL-EKF: two_d_mode=true (flat-terrain Segway RMP), GPS via navsat_transform
- RL-UKF excluded: robot_localization UKF diverges under high-rate sim time playback
  (rapid timer catchup causes near-zero dt between predictions, Cholesky failure, immediate NaN)
