#pragma once
#include "fusioncore/state.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace fusioncore {
namespace sensors {

// ─── Raw IMU measurement (6-dimensional) ─────────────────────────────────────
// [wx, wy, wz,   -- angular velocity (rad/s, body frame)
//  ax, ay, az]   -- linear acceleration (m/s², body frame)

constexpr int IMU_DIM = 6;

using ImuMeasurement = Eigen::Matrix<double, IMU_DIM, 1>;
using ImuNoiseMatrix = Eigen::Matrix<double, IMU_DIM, IMU_DIM>;

struct ImuParams {
  double gyro_noise_x  = 0.005;
  double gyro_noise_y  = 0.005;
  double gyro_noise_z  = 0.005;
  double accel_noise_x = 0.1;
  double accel_noise_y = 0.1;
  double accel_noise_z = 0.1;
};

// h(x): state -> expected raw IMU measurement
// Accelerometer reads specific force = body acceleration + gravity in body frame.
// Gravity in body frame (R^T * [0, 0, g], ENU z-up): yaw drops out because
// rotating around the world z-axis does not change a z-aligned vector.
inline ImuMeasurement imu_measurement_function(const StateVector& x) {
  ImuMeasurement z;
  constexpr double g = 9.80665;
  const double cr = std::cos(x[ROLL]),  sr = std::sin(x[ROLL]);
  const double cp = std::cos(x[PITCH]), sp = std::sin(x[PITCH]);
  z[0] = x[WX] + x[B_GX];
  z[1] = x[WY] + x[B_GY];
  z[2] = x[WZ] + x[B_GZ];
  z[3] = x[AX] + x[B_AX] + (-sp      * g);
  z[4] = x[AY] + x[B_AY] + ( cp * sr * g);
  z[5] = x[AZ] + x[B_AZ] + ( cp * cr * g);
  return z;
}

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

// ─── IMU orientation measurement (3-dimensional) ─────────────────────────────
// Some IMUs (BNO08x, VectorNav, Xsens) publish a full orientation estimate
// directly as a quaternion — not just raw gyro/accel.
// peci1 fix: accept orientation as a direct measurement input.
//
// Measurement vector: [roll, pitch, yaw] in radians

constexpr int IMU_ORIENTATION_DIM = 3;

using ImuOrientationMeasurement = Eigen::Matrix<double, IMU_ORIENTATION_DIM, 1>;
using ImuOrientationNoiseMatrix = Eigen::Matrix<double, IMU_ORIENTATION_DIM, IMU_ORIENTATION_DIM>;

struct ImuOrientationParams {
  double roll_noise  = 0.01;  // rad — typical for a good AHRS
  double pitch_noise = 0.01;  // rad
  double yaw_noise   = 0.05;  // rad — yaw is always less accurate without magnetometer
};

// h(x): state -> expected orientation measurement
inline ImuOrientationMeasurement imu_orientation_measurement_function(const StateVector& x) {
  ImuOrientationMeasurement z;
  z[0] = x[ROLL];
  z[1] = x[PITCH];
  z[2] = x[YAW];
  return z;
}

inline ImuOrientationNoiseMatrix imu_orientation_noise_matrix(const ImuOrientationParams& p) {
  ImuOrientationNoiseMatrix R = ImuOrientationNoiseMatrix::Zero();
  R(0,0) = p.roll_noise  * p.roll_noise;
  R(1,1) = p.pitch_noise * p.pitch_noise;
  R(2,2) = p.yaw_noise   * p.yaw_noise;
  return R;
}

/**
 * @brief Build orientation noise matrix from an IMU message covariance array.
 *
 * Extracts the diagonal variances from a row-major 3×3 covariance matrix
 * published in sensor_msgs::msg::Imu::orientation_covariance.
 * Falls back to the config-supplied noise values when the message covariance
 * is zero or invalid (all-zero covariance is ROS convention for "unknown").
 *
 * @param cov Row-major 3×3 covariance matrix as a 9-element array.
 *            Diagonal elements [0], [4], [8] are roll, pitch, yaw variances (rad²).
 * @param fallback Default ImuOrientationParams used when cov diagonal is non-positive.
 * @return 3×3 diagonal orientation noise matrix R.
 */
inline ImuOrientationNoiseMatrix imu_orientation_noise_from_covariance(
  const double cov[9],
  const ImuOrientationParams& fallback)
{
  ImuOrientationNoiseMatrix R = ImuOrientationNoiseMatrix::Zero();

  double var_roll  = cov[0];
  double var_pitch = cov[4];
  double var_yaw   = cov[8];

  // Use message covariance if all variances are positive
  if (var_roll > 0.0 && var_pitch > 0.0 && var_yaw > 0.0) {
    R(0,0) = var_roll;
    R(1,1) = var_pitch;
    R(2,2) = var_yaw;
  } else {
    // Fall back to config params
    R(0,0) = fallback.roll_noise  * fallback.roll_noise;
    R(1,1) = fallback.pitch_noise * fallback.pitch_noise;
    R(2,2) = fallback.yaw_noise   * fallback.yaw_noise;
  }

  return R;
}

} // namespace sensors
} // namespace fusioncore
