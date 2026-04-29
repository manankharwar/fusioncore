# Benchmark Results: NCLT Sequence 2012-08-20

## Absolute Trajectory Error (ATE)

| Filter | RMSE (m) | Max error (m) |
|--------|----------|---------------|
| FusionCore | 7.490 | 28.421 |
| RL-EKF | 24.118 | 64.475 |

## Relative Pose Error (RPE, per 10m segment)

| Filter | RMSE (m) |
|--------|----------|
| FusionCore | 16.467 |
| RL-EKF | 17.382 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-08-20
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation tool: [evo](https://github.com/MichaelGrupp/evo)
- Alignment: SE(3) alignment
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)

### Reproducing

```bash
# 1. Download NCLT sequence
# 2. Run benchmark
ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
  data_dir:=/path/to/nclt/2012-08-20 output_bag:=./nclt_results
# 3. Convert ground truth
python3 tools/nclt_rtk_to_tum.py --rtk gps_rtk.csv --out gt.tum
# 4. Extract trajectories
python3 tools/odom_to_tum.py --bag ./nclt_results --topic /fusion/odom --out fc.tum
python3 tools/odom_to_tum.py --bag ./nclt_results --topic /rl/odometry --out rl.tum
# 5. Evaluate
python3 tools/evaluate.py --gt gt.tum --fusioncore fc.tum --rl rl.tum --sequence 2012-08-20
```
