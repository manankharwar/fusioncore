#pragma once
// The error-state layout: 22 dimensions, because a rotation has three of them.
//
// STEP 2 OF #150. Definitions and conversions only. Nothing uses these yet; the UKF
// switches over in the commit after this one, so that change is a diff about the filter
// rather than a diff about indices.
//
// The shapes, indices and error_index_of() live in state.hpp so that header stays
// dependency free. What is here is the part that needs the manifold operators: moving
// between an ambient state and an error, in both directions, and converting a legacy
// ambient covariance to the error form.

#include <Eigen/Dense>

#include "fusioncore/state.hpp"
#include "fusioncore/attitude.hpp"

namespace fusioncore {

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
