#pragma once

#include "fusioncore/state.hpp"
#include <Eigen/Dense>

namespace fusioncore {
namespace sensors {

// Encoder measurement vector (3-dimensional):
// [vx, vy, wz]  -- linear velocity x, linear velocity y, angular velocity z
// For a differential drive robot: vy = 0 always, vx = forward speed, wz = turn rate

constexpr int ENCODER_DIM = 3;

using EncoderMeasurement = Eigen::Matrix<double, ENCODER_DIM, 1>;
using EncoderNoiseMatrix = Eigen::Matrix<double, ENCODER_DIM, ENCODER_DIM>;

struct EncoderParams {
  // Velocity noise (m/s) — depends on encoder resolution and wheel slip
  double vel_noise_x  = 0.05;
  double vel_noise_y  = 0.05;

  // Angular velocity noise (rad/s)
  double vel_noise_wz = 0.02;
};

// h(x): maps state vector to expected encoder measurement
// Encoders measure body-frame velocity directly
inline EncoderMeasurement encoder_measurement_function(const StateVector& x) {
  EncoderMeasurement z;
  z[0] = x[VX];          // forward velocity
  z[1] = x[VY];          // lateral velocity (zero for diff drive)
  z[2] = x[WZ] - x[B_GZ]; // yaw rate corrected for gyro bias
  return z;
}

// Build R matrix from encoder noise params
inline EncoderNoiseMatrix encoder_noise_matrix(const EncoderParams& p) {
  EncoderNoiseMatrix R = EncoderNoiseMatrix::Zero();
  R(0,0) = p.vel_noise_x  * p.vel_noise_x;
  R(1,1) = p.vel_noise_y  * p.vel_noise_y;
  R(2,2) = p.vel_noise_wz * p.vel_noise_wz;
  return R;
}

} // namespace sensors
} // namespace fusioncore
