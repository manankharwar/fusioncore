#pragma once

#include "fusioncore/state.hpp"
#include <Eigen/Dense>

namespace fusioncore {
namespace sensors {

// IMU measurement vector (6-dimensional):
// [wx, wy, wz,   -- angular velocity (rad/s, body frame)
//  ax, ay, az]   -- linear acceleration (m/s², body frame)

constexpr int IMU_DIM = 6;

using ImuMeasurement = Eigen::Matrix<double, IMU_DIM, 1>;
using ImuNoiseMatrix = Eigen::Matrix<double, IMU_DIM, IMU_DIM>;

// IMU noise parameters
struct ImuParams {
  // Gyroscope noise (rad/s) — from datasheet or calibration
  double gyro_noise_x  = 0.005;
  double gyro_noise_y  = 0.005;
  double gyro_noise_z  = 0.005;

  // Accelerometer noise (m/s²) — from datasheet or calibration
  double accel_noise_x = 0.1;
  double accel_noise_y = 0.1;
  double accel_noise_z = 0.1;
};

// h(x): maps state vector to expected IMU measurement
// The IMU reads angular velocity and acceleration from body frame.
// Bias is already in the state — we subtract it here.
inline ImuMeasurement imu_measurement_function(const StateVector& x) {
  ImuMeasurement z;

  // Expected gyro reading = true angular velocity + bias
  // (the UKF learns to separate them)
  z[0] = x[WX] + x[B_GX];
  z[1] = x[WY] + x[B_GY];
  z[2] = x[WZ] + x[B_GZ];

  // Expected accel reading = true acceleration + bias
  z[3] = x[AX] + x[B_AX];
  z[4] = x[AY] + x[B_AY];
  z[5] = x[AZ] + x[B_AZ];

  return z;
}

// Build R matrix from IMU noise params
inline ImuNoiseMatrix imu_noise_matrix(const ImuParams& p) {
  ImuNoiseMatrix R = ImuNoiseMatrix::Zero();

  R(0,0) = p.gyro_noise_x  * p.gyro_noise_x;
  R(1,1) = p.gyro_noise_y  * p.gyro_noise_y;
  R(2,2) = p.gyro_noise_z  * p.gyro_noise_z;
  R(3,3) = p.accel_noise_x * p.accel_noise_x;
  R(4,4) = p.accel_noise_y * p.accel_noise_y;
  R(5,5) = p.accel_noise_z * p.accel_noise_z;

  return R;
}

} // namespace sensors
} // namespace fusioncore
