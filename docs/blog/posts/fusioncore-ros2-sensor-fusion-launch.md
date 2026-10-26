---
date: 2026-03-24
title: "Introducing FusionCore: a ROS 2 sensor fusion package built to replace robot_localization"
description: "What FusionCore does, why it exists, and what makes it different from the existing options."
slug: fusioncore-ros2-sensor-fusion-launch
tags:
  - ROS
---

I have been working on FusionCore for the last several months. It is a ROS 2 Jazzy sensor fusion package: IMU + wheel encoders + GPS fused at 100 Hz into a single position estimate. The short version of why it exists is that `robot_localization` has seen very limited development since 2023, and the gaps it leaves are ones that matter in real deployments.

<!-- more -->

## What the gaps actually are

`robot_localization` uses an Extended Kalman Filter. It has no IMU bias estimation: gyro drift accumulates silently and the filter has no mechanism to identify or correct it. It does not do ECEF GPS fusion natively, which creates UTM zone boundary problems for anything operating near a zone edge. Noise covariances are fixed values you set in a config file and never change, so a bad GPS fix gets the same weight as a good one.

FusionCore addresses all three.

!!! note "Correction, 2026-10-26"
    The covariance sentence above is wrong. robot_localization uses the covariance each sensor
    message carries, so a fix that reports a large covariance is weighted less, and it supports
    per-sensor Mahalanobis rejection thresholds. What it does not do is adapt the noise model
    when a driver's reported covariance is too optimistic, which is the difference FusionCore's
    adaptive noise layer is for.

The filter is a 23-state UKF: position (3), orientation quaternion (4), body-frame velocity (3), angular velocity (3), body-frame acceleration (3), gyro bias (3), accelerometer bias (3), encoder WZ bias (1). Gyro and accelerometer biases are live states that the filter estimates and corrects continuously. GNSS is fused in ECEF (via PROJ), so UTM zone boundaries do not exist. Sensor noise covariances are adapted in real time from the innovation sequence using a sliding window, so bad GPS fixes are automatically down-weighted without any config change.

## Feature summary

- UKF with full 3D state, not an EKF port
- Native GNSS fusion in ECEF (EPSG:4978), globally valid, no UTM zone issues
- Automatic IMU gyro and accelerometer bias estimation
- Mahalanobis chi-squared outlier rejection: GPS jumps and encoder spikes are gated
- HDOP/VDOP quality-aware noise scaling: bad sky conditions reduce GPS weight automatically
- Adaptive noise covariance from innovation window: no manual R matrix tuning
- Dual antenna heading support out of the box (`gnss.heading_topic`, `gnss.azimuth_topic` via `compass_msgs`)
- IMU orientation fusion (9-axis IMUs): roll, pitch, and yaw from IMU directly
- Lever arm correction for off-center GNSS antennas
- Message covariance respected: if your GPS receiver or odometry source publishes real covariance values, the filter uses them
- Second IMU, second encoder (lidar odometry, visual odometry), second GPS receiver: all supported
- Apache 2.0, commercially safe

## One config file

The design goal was to eliminate manual covariance matrix tuning. You give FusionCore your sensor noise specs from the datasheet (`imu.gyro_noise`, `gnss.base_noise_xy`) and it takes it from there. The adaptive noise layer handles the rest at runtime. You do not fill in a 36-element matrix and hope you got it right.

## Feedback from the community

When this was posted on ROS Discourse, [peci1](https://discourse.ros.org/t/fusioncore-which-is-a-ros-2-jazzy-sensor-fusion-package-robot-localization-replacement) pointed out several areas that needed work: IMU frame rotation, off-center GNSS lever arm, respecting message covariances, IMU orientation fusion, and multiple sensor sources. All of those were either already present or have since been built:

- TF rotation: `imu.frame_id` controls the IMU-to-base_link lookup. Set it to your URDF frame name (or leave empty to read from the message header). The filter rotates measurements before fusing.
- Lever arm: `gnss.lever_arm_x/y/z` corrects the position measurement for antennas that are not directly above base_link. Lever arm correction is gated behind heading validation so it does not make things worse before heading is known.
- Message covariances: GPS full 3x3 matrix, wheel odometry per-axis twist covariance, IMU orientation covariance, all used when present.
- IMU orientation: `imu.has_magnetometer: true` enables 9-axis orientation fusion (roll + pitch + yaw).
- Multiple sources: `imu2.topic`, `gnss.fix2_topic`, `encoder2.topic` all supported.

The `compass_msgs/Azimuth` heading format peci1 mentioned is the preferred heading input: `gnss.azimuth_topic` accepts it directly.

## Try it

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to fusioncore_ros
source install/setup.bash
```

Issues and PRs are welcome. Hardware configs from real deployments are the most useful contribution right now: if you have FusionCore running on a robot, [open a PR adding a YAML](https://github.com/manankharwar/fusioncore/tree/main/fusioncore_ros/config) and your platform joins the supported hardware list.
