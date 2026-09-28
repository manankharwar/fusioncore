#pragma once
// The error-state layout: 22 dimensions, because a rotation has three of them.
//
// STEP 2 OF #150. Definitions and conversions only. Nothing uses these yet; the UKF
// switches over in the commit after this one, so that change is a diff about the filter
// rather than a diff about indices.
//
// THE SHAPE OF THE MIGRATION, since it is not the obvious one. The state VECTOR keeps all
// 23 components including the four quaternion terms, so motion models, measurement models
// and every public accessor are untouched. What changes is the COVARIANCE, which drops to
// 22x22 with a 3-vector attitude error in place of the 4-vector block:
//
//     ambient state x (23)            error covariance P (22)
//     ------------------------        --------------------------
//     0..2    X  Y  Z                 0..2    dX dY dZ
//     3..6    QW QX QY QZ             3..5    d_theta (rotation vector, radians)
//     7..22   VX .. B_EWZ             6..21   dVX .. dB_EWZ
//
// This is the standard error-state / USQUE arrangement and it is deliberately the least
// invasive option available. Changing STATE_DIM itself would touch every motion model,
// every measurement model and every test that indexes the state; this touches the UKF
// internals and little else.
//
// WHY 22 IS THE HONEST NUMBER. The 23rd dimension was never real. A unit quaternion has
// four components tied by |q| = 1, so the four-way covariance block asserts an error
// direction that does not exist, and the filter has been paying for that assertion:
// sigma(QZ) reaching 13.32 on a component bounded by 1, a 180 degree yaw flip in a single
// step at t=77 s, and a published yaw on real rover bags that moves 8.2 to 10.8 times
// further than the robot actually turned.

#include <Eigen/Dense>

#include "fusioncore/state.hpp"
#include "fusioncore/attitude.hpp"

namespace fusioncore {

// One fewer than STATE_DIM: four quaternion components become three error angles.
constexpr int ERROR_DIM = STATE_DIM - 1;

using ErrorVector = Eigen::Matrix<double, ERROR_DIM, 1>;
using ErrorMatrix = Eigen::Matrix<double, ERROR_DIM, ERROR_DIM>;

// Error-space indices. Position keeps its numbering, attitude collapses to three, and
// everything after it shifts down by one.
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

// Map an ambient state index to its error index. The quaternion components have no
// single counterpart, so they are rejected rather than silently folded: asking for "the
// error index of QW" is a question with no answer and returning one would hide a bug.
inline constexpr int error_index_of(int state_index)
{
  return (state_index < QW)  ? state_index
       : (state_index <= QZ) ? -1                  // attitude: use E_ROLL/E_PITCH/E_YAW
                             : state_index - 1;
}

// Apply an error vector to a nominal state, producing a new state. The attitude part
// composes on the manifold; everything else adds.
inline StateVector inject_error(const StateVector & x, const ErrorVector & e)
{
  StateVector out;
  out[X] = x[X] + e[E_X];
  out[Y] = x[Y] + e[E_Y];
  out[Z] = x[Z] + e[E_Z];

  const Eigen::Quaterniond q(x[QW], x[QX], x[QY], x[QZ]);
  const Eigen::Quaterniond q_new =
    attitude::boxplus(q, Eigen::Vector3d(e[E_ROLL], e[E_PITCH], e[E_YAW]));
  out[QW] = q_new.w();
  out[QX] = q_new.x();
  out[QY] = q_new.y();
  out[QZ] = q_new.z();

  for (int i = VX; i < STATE_DIM; ++i) { out[i] = x[i] + e[i - 1]; }
  return out;
}

// The inverse: the error that takes `from` to `to`. Exactly undoes inject_error.
inline ErrorVector extract_error(const StateVector & to, const StateVector & from)
{
  ErrorVector e;
  e[E_X] = to[X] - from[X];
  e[E_Y] = to[Y] - from[Y];
  e[E_Z] = to[Z] - from[Z];

  const Eigen::Quaterniond q_to(to[QW], to[QX], to[QY], to[QZ]);
  const Eigen::Quaterniond q_from(from[QW], from[QX], from[QY], from[QZ]);
  const Eigen::Vector3d d = attitude::boxminus(q_to, q_from);
  e[E_ROLL] = d.x();
  e[E_PITCH] = d.y();
  e[E_YAW] = d.z();

  for (int i = VX; i < STATE_DIM; ++i) { e[i - 1] = to[i] - from[i]; }
  return e;
}

// Convert a legacy 23x23 covariance to the 22x22 error form.
//
// The attitude block is the part that cannot be copied across, because the two describe
// different things: the ambient block is a covariance over quaternion COMPONENTS, the
// error block a covariance over rotation ANGLES. Near the identity a small rotation of
// theta produces a vector part of about theta/2, so the angle variance is about four
// times the component variance. That factor is the whole conversion and getting it wrong
// would make every migrated attitude covariance off by 4x.
//
// This exists so an existing configuration's initial P still means what it meant. It is
// not exact for large uncertainties and does not need to be: it converts an INITIAL
// covariance, which is small by construction.
inline ErrorMatrix to_error_covariance(const StateMatrix & P)
{
  ErrorMatrix E = ErrorMatrix::Zero();
  for (int r = 0; r < STATE_DIM; ++r) {
    const int er = error_index_of(r);
    if (er < 0) { continue; }
    for (int c = 0; c < STATE_DIM; ++c) {
      const int ec = error_index_of(c);
      if (ec < 0) { continue; }
      E(er, ec) = P(r, c);
    }
  }
  // Attitude: 4x the vector-part variance, mapped QX,QY,QZ -> roll,pitch,yaw.
  const int amb[3] = {QX, QY, QZ};
  const int err[3] = {E_ROLL, E_PITCH, E_YAW};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      E(err[i], err[j]) = 4.0 * P(amb[i], amb[j]);
    }
    // Cross terms between attitude and everything else carry one factor of 2.
    for (int c = 0; c < STATE_DIM; ++c) {
      const int ec = error_index_of(c);
      if (ec < 0) { continue; }
      E(err[i], ec) = 2.0 * P(amb[i], c);
      E(ec, err[i]) = 2.0 * P(c, amb[i]);
    }
  }
  return E;
}

}  // namespace fusioncore
