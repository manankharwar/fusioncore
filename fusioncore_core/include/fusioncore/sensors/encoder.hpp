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
  // Velocity noise (m/s): depends on encoder resolution and wheel slip
  double vel_noise_x  = 0.05;
  double vel_noise_y  = 0.05;

  // Angular velocity noise (rad/s)
  double vel_noise_wz = 0.02;
};

// h(x): maps state vector to expected encoder measurement.
// Encoder WZ reading = true body WZ + encoder WZ bias (B_EWZ).
// Mirrors the IMU measurement function which adds B_GZ to WZ.
// This allows the UKF to estimate and subtract the encoder WZ bias online,
// so that heading drift during GPS blackouts is eliminated rather than accumulated.
inline EncoderMeasurement encoder_measurement_function(const StateVector& x) {
  EncoderMeasurement z;
  z[0] = x[VX];            // forward velocity (assume no VX encoder bias)
  z[1] = x[VY];            // lateral velocity
  z[2] = x[WZ] + x[B_EWZ]; // encoder WZ reading = body WZ + encoder WZ bias
  return z;
}

// h(x): maps state vector to true body velocities (no encoder bias terms).
// Used for ZUPT: the assertion is that the BODY is not moving (true WZ = 0),
// not that the encoder reads zero. Fusing with encoder_measurement_function
// would incorrectly assert WZ = -B_EWZ instead of WZ = 0.
inline EncoderMeasurement zupt_measurement_function(const StateVector& x) {
  EncoderMeasurement z;
  z[0] = x[VX];   // true body VX
  z[1] = x[VY];   // true body VY
  z[2] = x[WZ];   // true body WZ (no bias term)
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

// ─── Non-holonomic ground constraint ─────────────────────────────────────────
// For wheeled ground robots, body-frame z-velocity must be zero.
// Fusing this as a pseudo-measurement prevents UKF altitude drift when GPS
// altitude is noisy and there is no barometer.

constexpr int GROUND_CONSTRAINT_DIM = 1;

using GroundConstraintMeasurement = Eigen::Matrix<double, GROUND_CONSTRAINT_DIM, 1>;
using GroundConstraintNoiseMatrix = Eigen::Matrix<double, GROUND_CONSTRAINT_DIM, GROUND_CONSTRAINT_DIM>;

// h(x): body-frame z-velocity (must be zero for a ground robot)
inline GroundConstraintMeasurement ground_constraint_measurement_function(const StateVector& x) {
  GroundConstraintMeasurement z;
  z[0] = x[VZ];
  return z;
}

// sigma: body-frame vertical velocity uncertainty (m/s).
// Default 0.1 m/s: loose enough to stay numerically stable when applied
// back-to-back with the encoder update, tight enough to suppress altitude drift.
// Increase for rough terrain or obstacle traversal (0.3-1.0 m/s range).
inline GroundConstraintNoiseMatrix ground_constraint_noise_matrix(double sigma = 0.1) {
  GroundConstraintNoiseMatrix R = GroundConstraintNoiseMatrix::Zero();
  R(0,0) = sigma * sigma;
  return R;
}

} // namespace sensors
} // namespace fusioncore
