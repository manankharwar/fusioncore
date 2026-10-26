---
date: 2026-04-20
title: "Benchmarking FusionCore against robot_localization on real-world GPS data"
description: "Withdrawn 2026-10-26. An April 2026 FusionCore against robot_localization comparison on NCLT, measured through a dataset player later found to feed both filters wrong data."
slug: benchmarking-fusioncore-vs-robot-localization-nclt
tags:
  - ROS
---

!!! warning "Withdrawn, 2026-10-26"
    The results in this post are withdrawn and should not be cited. They were measured through
    FusionCore's own NCLT dataset player, which had two input errors that fed both filters wrong
    data: a wheel-odometry yaw rate in the wrong frame (#169) and an accelerometer-derived tilt
    declared as a 0.57 degree attitude. The explanation below of why robot_localization did
    worse rests on those runs and is withdrawn with them. Measured again on all twelve sequences
    with the player corrected, the shipped FusionCore default does not beat raw unfiltered GPS,
    and the robot_localization comparison is being re-run. Current numbers:
    [Benchmark Results](../../reference/benchmark.md). The original text is left below unchanged
    so the correction is visible.

I ran FusionCore head-to-head against `robot_localization` on the NCLT dataset from the University of Michigan: a real robot driving around a campus for 10 minutes, mixed urban/suburban environment with tree cover, buildings, and open quads. Ground truth is RTK GPS at sub-10cm accuracy.

The results were more decisive than I expected.

<!-- more -->

## Setup

The NCLT dataset provides synchronized IMU, wheel odometry, and GPS. I fed the same raw data to both filters simultaneously with no tuning advantage: equal config, equal sensors, equal everything. The only variable is the filter implementation.

Both filters received:
- IMU at 100 Hz
- Wheel odometry
- GPS (standard autonomous, no RTK on the fused input)

Ground truth: RTK GPS. Evaluation metric: Absolute Trajectory Error (ATE) RMSE, computed with `evo_ape`.

## Results

| Filter | ATE RMSE |
|--------|----------|
| FusionCore UKF | 5.5 m |
| robot_localization EKF | 23.4 m |
| robot_localization UKF | diverged (NaN at t=31s) |

robot_localization EKF was **4.2x worse** than FusionCore on the same data. robot_localization UKF diverged numerically 31 seconds in: the covariance matrix hit NaN and every output was invalid for the remaining 9 minutes.

FusionCore ran stably for the full 600 seconds.

## Why the numbers look the way they do

The difference comes down to two things: bias estimation and adaptive noise.

**Bias estimation:** robot_localization has no gyro or accelerometer bias states. Gyro drift accumulates silently into heading error. Heading error integrates into position error quadratically over time. FusionCore tracks gyro bias and accelerometer bias as live filter states, estimating and correcting them continuously from GPS cross-covariance. A filter that knows its heading is accumulating error can correct it. One that doesn't cannot.

**Adaptive noise:** robot_localization passes through the GPS-reported covariance from the sensor message as-is. FusionCore also reads the sensor-reported covariance, but additionally adapts the noise model from the live innovation sequence using a 50-sample sliding window. It scales GPS noise by HDOP from the message and applies a chi-squared Mahalanobis gate to reject statistical outliers. Bad GPS in an urban canyon gets down-weighted automatically. The NCLT campus has a lot of tree cover and building multipath: that difference matters.

**The UKF NaN:** robot_localization's UKF mode diverged at t=31s at 100 Hz IMU input. The UKF's sigma point propagation is numerically sensitive to near-singular covariance matrices, and without quaternion normalization at each step the matrix accumulates floating point errors until it loses positive semi-definiteness. FusionCore normalizes the quaternion component of the state vector at every predict step and regularizes the orientation covariance to prevent this. It ran at 100 Hz for 600 seconds without a single numerical issue.

## The 0.31 m number

After this was posted, a few people asked about the 0.31 m RMSE figure mentioned in some of the early documentation. To be explicit: that was standard autonomous GPS (not RTK) on a flat, open sequence with good sky view. It is the best case, not the median. The 5.5 m figure above is a harder sequence with real urban multipath. Both are honest: the 0.31 m shows what's possible on clean data, the 5.5 m shows performance under real urban GPS conditions.

## Running the evaluation yourself

The evaluation scripts are in [`tools/benchmark/`](https://github.com/manankharwar/fusioncore/tree/main/tools) in the repository. If you have access to NCLT data or your own ground-truth dataset, you can run the same `evo_ape` evaluation on your own hardware and see comparable numbers. I'm curious whether anyone gets different results on different platforms or environments.

## Full benchmark results

The complete benchmark covers 12 NCLT sequences with full per-sequence tables, trajectory plots, and comparison methodology: [Benchmark Results](../../reference/benchmark.md).
