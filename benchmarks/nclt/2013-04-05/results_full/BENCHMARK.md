# Benchmark Results: NCLT Sequence 2013-04-05

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE 3D (m) | ATE RMSE XY (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|----------------|----------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 12.140 | 10.100 | 32.8% | 81.5% | 0.9020 | 2.26 | 30.196 |
| RL-EKF | 268.928 | 268.740 | 0.0% | 0.0% | 0.9140 | 50.11 | 27.307 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2013-04-05
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- All filters consume identical sensor streams: same IMU, wheel odometry, and GPS topics
- FusionCore: full 3D UKF, adaptive noise, ZUPT, IMU bias estimation
- RL-EKF: two_d_mode=true (flat-terrain Segway RMP), GPS via navsat_transform
- RL-UKF excluded: robot_localization UKF diverges under high-rate sim time playback
  (rapid timer catchup causes near-zero dt between predictions, Cholesky failure, immediate NaN)
