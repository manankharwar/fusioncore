#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace fusioncore {

// Full 22-dimensional state vector:
// [x, y, z,                    -- position (meters, ENU frame)
//  qw, qx, qy, qz,            -- orientation (unit quaternion, body-to-world)
//  vx, vy, vz,                 -- linear velocity (m/s, body frame)
//  wx, wy, wz,                 -- angular velocity (rad/s, body frame)
//  ax, ay, az,                 -- linear acceleration (m/s², body frame)
//  b_gx, b_gy, b_gz,           -- gyroscope bias (rad/s)
//  b_ax, b_ay, b_az]           -- accelerometer bias (m/s²)
//
// Quaternion convention: q = [qw, qx, qy, qz], |q| = 1.
// Rotation: v_world = R(q) * v_body.
// Initial value must be [1, 0, 0, 0] (identity), NOT zero.

constexpr int STATE_DIM = 22;

// State indices: use these everywhere, never raw numbers
enum StateIndex {
  X = 0, Y = 1, Z = 2,
  QW = 3, QX = 4, QY = 5, QZ = 6,
  VX = 7, VY = 8, VZ = 9,
  WX = 10, WY = 11, WZ = 12,
  AX = 13, AY = 14, AZ = 15,
  B_GX = 16, B_GY = 17, B_GZ = 18,
  B_AX = 19, B_AY = 20, B_AZ = 21
};

using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

// Rotation matrix (body-to-world) from quaternion state components.
// R * v_body = v_world
inline void quat_to_rotation_matrix(
  double qw, double qx, double qy, double qz,
  double R[3][3])
{
  R[0][0] = 1 - 2*(qy*qy + qz*qz);
  R[0][1] =     2*(qx*qy - qw*qz);
  R[0][2] =     2*(qx*qz + qw*qy);
  R[1][0] =     2*(qx*qy + qw*qz);
  R[1][1] = 1 - 2*(qx*qx + qz*qz);
  R[1][2] =     2*(qy*qz - qw*qx);
  R[2][0] =     2*(qx*qz - qw*qy);
  R[2][1] =     2*(qy*qz + qw*qx);
  R[2][2] = 1 - 2*(qx*qx + qy*qy);
}

// Extract roll/pitch/yaw (ZYX Euler, radians) from quaternion.
// Safe for display and measurement functions. The singularity at ±90° pitch
// is bounded (returns ±π/2) and does NOT affect the filter's internal state,
// which is always stored as a quaternion.
inline void quat_to_euler(
  double qw, double qx, double qy, double qz,
  double& roll, double& pitch, double& yaw)
{
  roll  = std::atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy));
  double sinp = 2*(qw*qy - qz*qx);
  pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
  yaw   = std::atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
}

struct State {
  StateVector x = StateVector::Zero();   // state mean
  StateMatrix P = StateMatrix::Identity(); // state covariance

  State() {
    x[QW] = 1.0;  // identity quaternion: must NOT be zero
    // Quaternion components live on S³; P for them must stay tiny.
    // Orientation uncertainty propagates via q_angular_vel in Q, not via
    // large quaternion P. See generate_sigma_points() for the clamp rationale.
    P(QW,QW) = 1e-8;
    P(QX,QX) = 1e-8;
    P(QY,QY) = 1e-8;
    P(QZ,QZ) = 1e-8;
  }

  // Convenience accessors
  double position_x()   const { return x[X]; }
  double position_y()   const { return x[Y]; }
  double position_z()   const { return x[Z]; }
  double quat_w()       const { return x[QW]; }
  double quat_x()       const { return x[QX]; }
  double quat_y()       const { return x[QY]; }
  double quat_z()       const { return x[QZ]; }
  double vel_x()        const { return x[VX]; }
  double vel_y()        const { return x[VY]; }
  double vel_z()        const { return x[VZ]; }
  double gyro_bias_x()  const { return x[B_GX]; }
  double gyro_bias_y()  const { return x[B_GY]; }
  double gyro_bias_z()  const { return x[B_GZ]; }
  double accel_bias_x() const { return x[B_AX]; }
  double accel_bias_y() const { return x[B_AY]; }
  double accel_bias_z() const { return x[B_AZ]; }

  // Euler angles derived from quaternion (for display/logging only)
  double roll()  const { double r, p, y; quat_to_euler(x[QW],x[QX],x[QY],x[QZ],r,p,y); return r; }
  double pitch() const { double r, p, y; quat_to_euler(x[QW],x[QX],x[QY],x[QZ],r,p,y); return p; }
  double yaw()   const { double r, p, y; quat_to_euler(x[QW],x[QX],x[QY],x[QZ],r,p,y); return y; }
};

} // namespace fusioncore
