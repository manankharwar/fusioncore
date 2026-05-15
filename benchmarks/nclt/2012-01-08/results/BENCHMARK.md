# Benchmark Results: NCLT Sequence 2012-01-08

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|-------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 5.506 | 65.3% | 95.9% | 0.9396 | 6.47 | 17.243 |
| RL-EKF | 4.398 | 86.9% | 97.2% | 0.9221 | 5.16 | 17.634 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-01-08
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- All filters consume identical sensor streams: same IMU, wheel odometry, and GPS topics
- FusionCore: full 3D UKF, adaptive noise, ZUPT, IMU bias estimation
- RL-EKF: two_d_mode=true (flat-terrain Segway RMP), GPS via navsat_transform
- RL-UKF excluded: robot_localization UKF diverges under high-rate sim time playback
  (rapid timer catchup causes near-zero dt between predictions, Cholesky failure, immediate NaN)
