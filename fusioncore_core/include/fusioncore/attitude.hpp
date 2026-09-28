#pragma once
// Attitude on the manifold: the log and exp maps, and the two operators built on them.
//
// STEP 1 OF #150. These are pure functions with no filter state and no behaviour change
// anywhere. Nothing calls them yet. They exist first so the error-state migration can be
// reviewed in pieces instead of arriving as one large commit.
//
// WHY THIS IS NEEDED AT ALL. A rotation has three degrees of freedom but a quaternion has
// four components tied by |q| = 1. The filter currently carries all four in the state
// vector with an ordinary 4x4 covariance block, which claims four independent error
// directions where only three exist. Two measured consequences:
//
//   sigma(QZ) reaches 13.32, and a component bounded by 1 cannot have a standard
//   deviation above 1. At t=77 s the weighted sum of the sigma-point quaternions
//   cancels, normalising the residue aims it arbitrarily, and yaw flips 180 degrees in
//   a single step.
//
//   On real rover bags, where GNSS pins position and the covariance never explodes, the
//   filter's published yaw still moves 8.2 to 10.8 times further than the robot actually
//   turned, and 92.7% of that enters through an IMU update whose gyro reads exactly zero.
//
// Both come from the same place: a covariance in the ambient 4-space describes errors
// that are not rotations. The fix is to describe attitude error as a 3-vector in the
// tangent space at the current estimate, where every perturbation is a rotation by
// construction and there is no norm for the covariance to violate.
//
// CONVENTION, stated once because getting it wrong is silent. delta is a rotation VECTOR:
// its direction is the axis, its magnitude is the angle in radians. The composition is
// LOCAL (body frame, right multiplication):
//
//     boxplus(q, delta)  =  q * exp(delta)          apply a body-frame rotation
//     boxminus(a, b)     =  log(b^-1 * a)           the rotation taking b to a
//
// so that boxminus(boxplus(q, d), q) == d, which is the round trip the tests assert.

#include <Eigen/Dense>
#include <cmath>

namespace fusioncore {
namespace attitude {

// Rotation vector -> quaternion. The Taylor limit matters: sin(x/2)/x is 0/0 at zero and
// the filter sits near zero on every step, so the naive form would divide by zero
// constantly rather than rarely.
inline Eigen::Quaterniond exp_map(const Eigen::Vector3d & delta)
{
  const double angle = delta.norm();
  if (angle < 1e-8) {
    // sin(a/2)/a -> 1/2 as a -> 0. Second order is plenty here and stays exact at 0.
    Eigen::Quaterniond q(1.0, 0.5 * delta.x(), 0.5 * delta.y(), 0.5 * delta.z());
    q.normalize();
    return q;
  }
  const double half = 0.5 * angle;
  const double s = std::sin(half) / angle;
  return Eigen::Quaterniond(std::cos(half), s * delta.x(), s * delta.y(), s * delta.z());
}

// Quaternion -> rotation vector. Uses atan2 rather than acos(w) because atan2 keeps its
// precision near the identity, which is exactly where this is called most.
inline Eigen::Vector3d log_map(const Eigen::Quaterniond & q_in)
{
  Eigen::Quaterniond q = q_in;
  if (q.norm() < 1e-12) { return Eigen::Vector3d::Zero(); }
  q.normalize();
  // q and -q are the same rotation but log would report the long way round for one of
  // them. Always take the short way.
  if (q.w() < 0.0) { q.coeffs() *= -1.0; }

  const double vn = q.vec().norm();
  if (vn < 1e-8) {
    // Near identity: angle -> 2*|v| and the axis is v/|v|, so the whole thing -> 2*v.
    return 2.0 * q.vec();
  }
  return (2.0 * std::atan2(vn, q.w()) / vn) * q.vec();
}

// Apply a body-frame rotation to an attitude.
inline Eigen::Quaterniond boxplus(const Eigen::Quaterniond & q,
                                  const Eigen::Vector3d & delta)
{
  Eigen::Quaterniond out = q * exp_map(delta);
  out.normalize();
  return out;
}

// The body-frame rotation that takes `from` to `to`.
inline Eigen::Vector3d boxminus(const Eigen::Quaterniond & to,
                                const Eigen::Quaterniond & from)
{
  Eigen::Quaterniond a = to, b = from;
  if (a.norm() < 1e-12 || b.norm() < 1e-12) { return Eigen::Vector3d::Zero(); }
  a.normalize();
  b.normalize();
  return log_map(b.conjugate() * a);
}

// Iterative tangent-space (Karcher) mean of a weighted set of attitudes.
//
// Repeatedly: express each sample as a rotation vector relative to the current reference,
// average those 3-vectors, and move the reference by the result. Unlike a 4-vector sum
// there is nothing to cancel, so a wide spread degrades the answer gradually instead of
// flipping it. Converges in two or three passes for any realistic spread.
//
// NOTE, so this is not mistaken for the whole fix: replacing only the mean, while the
// covariance residual is still an ambient-space difference, was measured and is NOT
// sufficient. It removes the t=77 s collapse but overshoots 22% and oscillates the
// accelerometer estimate by +-5 m/s^2 on a still sensor, because residuals taken about a
// manifold mean no longer sum to zero. The mean and the residual have to move together.
template <typename QuatContainer, typename WeightContainer>
inline Eigen::Quaterniond weighted_mean(const QuatContainer & quats,
                                        const WeightContainer & weights,
                                        int count,
                                        int max_iterations = 8)
{
  if (count <= 0) { return Eigen::Quaterniond::Identity(); }
  Eigen::Quaterniond ref = quats[0];
  if (ref.norm() < 1e-12) { return Eigen::Quaterniond::Identity(); }
  ref.normalize();

  for (int iter = 0; iter < max_iterations; ++iter) {
    Eigen::Vector3d mean_err = Eigen::Vector3d::Zero();
    double w_total = 0.0;
    for (int i = 0; i < count; ++i) {
      if (quats[i].norm() < 1e-12) { continue; }
      mean_err += weights[i] * boxminus(quats[i], ref);
      w_total += weights[i];
    }
    if (std::abs(w_total) > 1e-12) { mean_err /= w_total; }
    if (mean_err.norm() < 1e-12) { break; }
    ref = boxplus(ref, mean_err);
    if (mean_err.norm() < 1e-9) { break; }
  }
  return ref;
}

}  // namespace attitude
}  // namespace fusioncore
