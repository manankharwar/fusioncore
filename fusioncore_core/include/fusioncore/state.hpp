#pragma once

#include <Eigen/Dense>

namespace fusioncore {

// Full 21-dimensional state vector:
// [x, y, z,                    -- position (meters, ENU frame)
//  roll, pitch, yaw,           -- orientation (radians, Euler ZYX)
//  vx, vy, vz,                 -- linear velocity (m/s, body frame)
//  wx, wy, wz,                 -- angular velocity (rad/s, body frame)
//  ax, ay, az,                 -- linear acceleration (m/s², body frame)
//  b_gx, b_gy, b_gz,           -- gyroscope bias (rad/s)
//  b_ax, b_ay, b_az]           -- accelerometer bias (m/s²)

constexpr int STATE_DIM = 21;

// State indices — use these everywhere, never raw numbers
enum StateIndex {
  X = 0, Y = 1, Z = 2,
  ROLL = 3, PITCH = 4, YAW = 5,
  VX = 6, VY = 7, VZ = 8,
  WX = 9, WY = 10, WZ = 11,
  AX = 12, AY = 13, AZ = 14,
  B_GX = 15, B_GY = 16, B_GZ = 17,
  B_AX = 18, B_AY = 19, B_AZ = 20
};

using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

struct State {
  StateVector x = StateVector::Zero();   // state mean
  StateMatrix P = StateMatrix::Identity(); // state covariance

  // Convenience accessors
  double position_x()   const { return x[X]; }
  double position_y()   const { return x[Y]; }
  double position_z()   const { return x[Z]; }
  double roll()         const { return x[ROLL]; }
  double pitch()        const { return x[PITCH]; }
  double yaw()          const { return x[YAW]; }
  double vel_x()        const { return x[VX]; }
  double vel_y()        const { return x[VY]; }
  double vel_z()        const { return x[VZ]; }
  double gyro_bias_x()  const { return x[B_GX]; }
  double gyro_bias_y()  const { return x[B_GY]; }
  double gyro_bias_z()  const { return x[B_GZ]; }
  double accel_bias_x() const { return x[B_AX]; }
  double accel_bias_y() const { return x[B_AY]; }
  double accel_bias_z() const { return x[B_AZ]; }
};

} // namespace fusioncore
