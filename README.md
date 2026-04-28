# FusionCore

[![CI](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml/badge.svg)](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml)
[![DOI](https://img.shields.io/badge/DOI-10.5281%2Fzenodo.19834991-blue)](https://doi.org/10.5281/zenodo.19834991)
[![Docs](https://img.shields.io/badge/docs-manankharwar.github.io%2Ffusioncore-blue)](https://manankharwar.github.io/fusioncore/)

**ROS 2 sensor fusion: IMU + wheel encoders + GPS fused via UKF at 100 Hz. Drop-in alternative to robot_localization with native ECEF GPS handling, automatic IMU bias estimation, and zero manual noise tuning.**

---

## Why I built this

I needed sensor fusion for a mobile robot project and reached for `robot_localization` like everyone does. It works, but it has real gaps: no native ECEF GPS handling (it goes through `navsat_transform`, which adds latency and breaks at UTM zone boundaries), no IMU bias estimation, and noise covariances you have to tune by hand and re-tune whenever your environment changes.

The designated replacement (`fuse`) has been in development for a while but still has incomplete GPS support as of early 2026.

So I built FusionCore. It's a 22-state UKF that fuses IMU, wheel encoders, and GPS natively. It estimates IMU bias, adapts its noise covariance from the innovation sequence automatically, and gates outliers with a chi-squared test on every sensor. GPS is handled in ECEF directly: no coordinate projection, no extra node, no zone boundary issues.

<p align="center">
  <img src="figures/fig2_traj_grid.png" alt="Trajectory overlay: all 6 sequences, SE3-aligned to RTK GPS ground truth" width="650">
</p>

---

## Benchmark

FusionCore vs robot_localization on the [NCLT dataset](http://robots.engin.umich.edu/nclt/): same IMU + wheel odometry + GPS, no manual tuning. Six sequences:

| Sequence | FC ATE RMSE | RL-EKF ATE RMSE | RL-UKF |
|---|---|---|---|
| 2012-01-08 | **5.6 m** | 23.4 m | NaN divergence at t=31 s |
| 2012-02-04 | **9.7 m** | 20.6 m | NaN divergence at t=22 s |
| 2012-03-31 | **4.2 m** | 10.8 m | NaN divergence at t=18 s |
| 2012-08-20 | **7.5 m** | 9.4 m | NaN divergence |
| 2012-11-04 | 28.7 m | **10.9 m** | NaN divergence |
| 2013-02-23 | **4.1 m** | 5.8 m | NaN divergence |

---

## Install

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build && source install/setup.bash
```

> **Headless / Raspberry Pi:** `touch ~/ros2_ws/src/fusioncore/fusioncore_gazebo/COLCON_IGNORE` before building to skip the Gazebo package.

---

## Quick start

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

---

## Documentation

**[manankharwar.github.io/fusioncore](https://manankharwar.github.io/fusioncore/)**

- [Getting Started](https://manankharwar.github.io/fusioncore/getting-started/)
- [Configuration reference](https://manankharwar.github.io/fusioncore/configuration/)
- [Hardware configs](https://manankharwar.github.io/fusioncore/hardware/)
- [Nav2 integration](https://manankharwar.github.io/fusioncore/nav2/)
- [Migrating from robot_localization](https://manankharwar.github.io/fusioncore/migration_from_robot_localization/)
- [How it works](https://manankharwar.github.io/fusioncore/how-it-works/)

---

## License

Apache 2.0. Includes explicit patent license grant that BSD-3 does not provide.

---

## Citation

```bibtex
@software{kharwar2026fusioncore,
  author    = {Kharwar, Manan},
  title     = {FusionCore: ROS 2 UKF Sensor Fusion},
  year      = {2026},
  publisher = {Zenodo},
  version   = {0.2.0},
  doi       = {10.5281/zenodo.19834991},
  url       = {https://doi.org/10.5281/zenodo.19834991}
}
```

---

Issues answered within 24 hours. Open a GitHub issue or find the discussion on [ROS Discourse](https://discourse.ros.org).
