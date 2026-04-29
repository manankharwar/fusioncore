# Benchmark Results: NCLT Sequence 2013-02-23

## Absolute Trajectory Error (ATE)

| Filter | RMSE (m) | Max error (m) |
|--------|----------|---------------|
| FusionCore | 4.114 | 9.344 |
| RL-EKF | 10.969 | 31.362 |

## Relative Pose Error (RPE, per 10m segment)

| Filter | RMSE (m) |
|--------|----------|
| FusionCore | 19.076 |
| RL-EKF | 19.076 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2013-02-23
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation tool: [evo](https://github.com/MichaelGrupp/evo)
- Alignment: SE(3) alignment
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)

### Reproducing

```bash
# 1. Download NCLT sequence
# 2. Run benchmark
ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
  data_dir:=/path/to/nclt/2013-02-23 output_bag:=./nclt_results
# 3. Convert ground truth
python3 tools/nclt_rtk_to_tum.py --rtk gps_rtk.csv --out gt.tum
# 4. Extract trajectories
python3 tools/odom_to_tum.py --bag ./nclt_results --topic /fusion/odom --out fc.tum
python3 tools/odom_to_tum.py --bag ./nclt_results --topic /rl/odometry --out rl.tum
# 5. Evaluate
python3 tools/evaluate.py --gt gt.tum --fusioncore fc.tum --rl rl.tum --sequence 2013-02-23
```
