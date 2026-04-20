# NCLT Benchmark: 2012-01-08

Results from comparing FusionCore against robot_localization EKF on the
[NCLT dataset](http://robots.engin.umich.edu/nclt/) (University of Michigan).

See `BENCHMARK.md` for the full results table.

## To reproduce

```bash
# 1. Run the benchmark (from repo root)
ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
  data_dir:=/path/to/nclt/2012-01-08

# 2. Convert ground truth
python3 tools/nclt_rtk_to_tum.py \
  --rtk /path/to/nclt/2012-01-08/gps_rtk.csv \
  --out benchmarks/nclt/2012-01-08/ground_truth.tum

# 3. Extract trajectories
python3 tools/odom_to_tum.py \
  --bag benchmarks/nclt/2012-01-08/bag \
  --topic /fusion/odom \
  --out benchmarks/nclt/2012-01-08/fusioncore.tum

python3 tools/odom_to_tum.py \
  --bag benchmarks/nclt/2012-01-08/bag \
  --topic /rl/odometry \
  --out benchmarks/nclt/2012-01-08/rl_ekf.tum

# 4. Evaluate
python3 tools/evaluate.py \
  --gt          benchmarks/nclt/2012-01-08/ground_truth.tum \
  --fusioncore  benchmarks/nclt/2012-01-08/fusioncore.tum \
  --rl          benchmarks/nclt/2012-01-08/rl_ekf.tum \
  --sequence    2012-01-08
```
