#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace fusioncore {

// Full 23-dimensional state vector:
// [x, y, z,                    -- position (meters, ENU frame)
//  qw, qx, qy, qz,            -- orientation (unit quaternion, body-to-world)
//  vx, vy, vz,                 -- linear velocity (m/s, body frame)
//  wx, wy, wz,                 -- angular velocity (rad/s, body frame)
//  ax, ay, az,                 -- linear acceleration (m/s², body frame)
//  b_gx, b_gy, b_gz,           -- gyroscope bias (rad/s)
//  b_ax, b_ay, b_az,           -- accelerometer bias (m/s²)
//  b_ewz]                      -- encoder angular velocity Z bias (rad/s)
//
// Quaternion convention: q = [qw, qx, qy, qz], |q| = 1.
// Rotation: v_world = R(q) * v_body.
// Initial value must be [1, 0, 0, 0] (identity), NOT zero.
//
// B_EWZ is the systematic WZ bias of the wheel encoder (differential drive).
// It is estimated online through the GPS track heading cross-covariance:
// encoder WZ bias -> integrated into heading -> observed via GPS bearing.
// During GPS blackouts, the filter subtracts this estimated bias instead of
// accumulating it as heading error. Mirrors exactly how B_GZ is handled for IMU.

constexpr int STATE_DIM = 23;

// State indices: use these everywhere, never raw numbers
enum StateIndex {
  X = 0, Y = 1, Z = 2,
  QW = 3, QX = 4, QY = 5, QZ = 6,
  VX = 7, VY = 8, VZ = 9,
  WX = 10, WY = 11, WZ = 12,
  AX = 13, AY = 14, AZ = 15,
  B_GX = 16, B_GY = 17, B_GZ = 18,
  B_AX = 19, B_AY = 20, B_AZ = 21,
  B_EWZ = 22  // encoder WZ bias: estimated from GPS heading, subtracted during blackouts
};

using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

// ─── The ERROR state, which is where the covariance lives ─────────────────────
//
// One fewer dimension than the state vector, because a rotation has three degrees of
// freedom and a unit quaternion has four components tied by |q| = 1. Carrying a 4x4
// covariance over the quaternion asserts an error direction that does not exist, and the
// filter paid for that: sigma(QZ) reached 13.32 on a component bounded by 1, yaw flipped
// 180 degrees in a single step at t=77 s, and on real rover bags the published yaw moved
// 8.2 to 10.8 times further than the robot actually turned.
//
// The state VECTOR keeps all 23 components including the quaternion, so motion models and
// measurement models are untouched. Only the covariance changes shape:
//
//     state x (23)                  covariance P (22)
//     0..2    X  Y  Z               0..2    dX dY dZ
//     3..6    QW QX QY QZ           3..5    d_theta, a rotation vector in RADIANS
//     7..22   VX .. B_EWZ           6..21   dVX .. dB_EWZ
//
// Every physical state is still estimated, B_EWZ included. Nothing was deleted; the
// redundant fourth quaternion coordinate was never a real degree of freedom.
//
// Conversions between the two live in error_state.hpp, which needs the manifold
// operators. Only the shapes and names are here, so that state.hpp stays dependency free.

constexpr int ERROR_DIM = STATE_DIM - 1;

using ErrorVector = Eigen::Matrix<double, ERROR_DIM, 1>;
using ErrorMatrix = Eigen::Matrix<double, ERROR_DIM, ERROR_DIM>;

enum ErrorIndex {
  E_X = 0, E_Y = 1, E_Z = 2,
  E_ROLL = 3, E_PITCH = 4, E_YAW = 5,          // rotation vector, body frame, radians
  E_VX = 6, E_VY = 7, E_VZ = 8,
  E_WX = 9, E_WY = 10, E_WZ = 11,
  E_AX = 12, E_AY = 13, E_AZ = 14,
  E_B_GX = 15, E_B_GY = 16, E_B_GZ = 17,
  E_B_AX = 18, E_B_AY = 19, E_B_AZ = 20,
  E_B_EWZ = 21
};

// Ambient state index -> error index. The quaternion components are REJECTED rather than
// folded onto a representative: "the error index of QW" is a question with no answer and
// returning one anyway would hide a bug instead of surfacing it.
inline constexpr int error_index_of(int state_index)
{
  return (state_index < QW)  ? state_index
       : (state_index <= QZ) ? -1
                             : state_index - 1;
}

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
  ErrorMatrix P = ErrorMatrix::Identity(); // ERROR covariance, 22x22: see ErrorIndex

  State() {
    x[QW] = 1.0;  // identity quaternion: must NOT be zero
    // Quaternion components live on S³; P for them must stay tiny.
    // Orientation uncertainty propagates via q_angular_vel in Q, not via
    // large quaternion P. See generate_sigma_points() for the clamp rationale.
    // Attitude, now in ANGLE units. The old form set 1e-8 on each of the four
    // quaternion COMPONENTS; near the identity a rotation of theta gives a vector part
    // of about theta/2, so the equivalent angle variance is 4x that. 4e-8 rad^2 is a
    // sigma of about 0.0115 degrees, which is what the old value meant.
    P(E_ROLL,E_ROLL)   = 4e-8;
    P(E_PITCH,E_PITCH) = 4e-8;
    P(E_YAW,E_YAW)     = 4e-8;
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
  double gyro_bias_x()       const { return x[B_GX]; }
  double gyro_bias_y()       const { return x[B_GY]; }
  double gyro_bias_z()       const { return x[B_GZ]; }
  double accel_bias_x()      const { return x[B_AX]; }
  double accel_bias_y()      const { return x[B_AY]; }
  double accel_bias_z()      const { return x[B_AZ]; }
  double encoder_wz_bias()   const { return x[B_EWZ]; }

  // Euler angles derived from quaternion (for display/logging only)
  double roll()  const { double r, p, y; quat_to_euler(x[QW],x[QX],x[QY],x[QZ],r,p,y); return r; }
  double pitch() const { double r, p, y; quat_to_euler(x[QW],x[QX],x[QY],x[QZ],r,p,y); return p; }
  double yaw()   const { double r, p, y; quat_to_euler(x[QW],x[QX],x[QY],x[QZ],r,p,y); return y; }
};

} // namespace fusioncore
