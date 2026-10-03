#include "fusioncore/ukf.hpp"
#include "fusioncore/motion_model.hpp"
#include <cmath>
#include <stdexcept>

namespace fusioncore {

UKF::UKF(const UKFParams& params)
  : params_(params), initialized_(false),
    motion_model_(std::make_shared<ConstantVelocityAcceleration>())
{
  // The augmented dimension follows the COVARIANCE, not the state vector: 22, because
  // that is how many independent error directions there are.
  n_aug_  = ERROR_DIM;
  lambda_ = params_.alpha * params_.alpha * (n_aug_ + params_.kappa) - n_aug_;
  compute_weights();
  build_process_noise();
}

void UKF::init(const State& initial_state) {
  state_ = initial_state;
  // Repair zero or near-zero quaternion: tests that assign x = Zero() invalidate QW.
  // A zero-norm quaternion causes division-by-zero in process_model.
  double qnorm = std::sqrt(state_.x[QW]*state_.x[QW] + state_.x[QX]*state_.x[QX] +
                            state_.x[QY]*state_.x[QY] + state_.x[QZ]*state_.x[QZ]);
  if (qnorm < 1e-10) {
    state_.x[QW] = 1.0;
    state_.x[QX] = state_.x[QY] = state_.x[QZ] = 0.0;
  } else {
    state_.x[QW] /= qnorm;
    state_.x[QX] /= qnorm;
    state_.x[QY] /= qnorm;
    state_.x[QZ] /= qnorm;
  }
  initialized_ = true;
}

void UKF::compute_weights() {
  int n_sigma = 2 * n_aug_ + 1;
  Wm_.resize(n_sigma);
  Wc_.resize(n_sigma);
  // alpha is 1.0 since 0.3.7 (commit 0da4ff0, 2026-08-13), which gives lambda = 0
  // and all 47 weights non-negative: the standard unscaled UKF. Do not lower it.
  //
  // History, because the failure is instructive. The old default was alpha = 0.1,
  // which at 23 states makes Wm[0] = -99 while the other 46 are +2.17, so every
  // mean is reconstructed as a difference of huge nearly cancelling numbers. Yaw
  // is unobservable without an absolute heading source, so the quaternion sigma
  // points spread wide, their forward displacements cancel, and what survives is
  // the correct forward-pointing centre point multiplied by -99. The filter drove
  // BACKWARDS while reporting a perfect velocity and a perfect heading.
  //
  // Measured on tools/repro/dr.cpp, perfect encoder at 1 m/s over 60 s, truth 60.00 m:
  //     alpha 0.1   Wm[0] -99.00   x = -114.06
  //     alpha 0.5   Wm[0]  -3.00   x =    8.83
  //     alpha 1.0   Wm[0]   0.00   x =   48.43
  // and on NCLT 2013-04-05 at 1x playback, 5268.80 m -> 131.85 m ATE, a 97.5
  // percent reduction, with robot_localization unchanged at ~230 m as the control.
  //
  // See the full note on UKFParams::alpha in ukf.hpp. Any change here alters every
  // filter output and must go through tools/check_benchmark_regression.py.
  Wm_[0] = lambda_ / (n_aug_ + lambda_);
  Wc_[0] = Wm_[0] + (1.0 - params_.alpha * params_.alpha + params_.beta);
  double w = 0.5 / (n_aug_ + lambda_);
  for (int i = 1; i < n_sigma; ++i) {
    Wm_[i] = w;
    Wc_[i] = w;
  }
}

void UKF::build_process_noise() {
  Q_ = ErrorMatrix::Zero();
  // Attitude now takes THREE entries, not four, and they are angle variances rather
  // than quaternion-component variances. q_orientation stays a small regularisation
  // term either way: the real orientation noise enters through q_angular_vel via the
  // kinematics. Scaled by 4 for the same reason the initial covariance is, so an
  // existing configured value keeps meaning what it meant.
  Q_.diagonal() << params_.q_position, params_.q_position, params_.q_position,
                   4.0 * params_.q_orientation, 4.0 * params_.q_orientation,
                   4.0 * params_.q_orientation,
                   params_.q_velocity, params_.q_velocity, params_.q_velocity,
                   params_.q_angular_vel, params_.q_angular_vel, params_.q_angular_vel,
                   params_.q_acceleration, params_.q_acceleration, params_.q_acceleration,
                   params_.q_gyro_bias, params_.q_gyro_bias, params_.q_gyro_bias,
                   params_.q_accel_bias, params_.q_accel_bias, params_.q_accel_bias,
                   params_.q_encoder_wz_bias;
}

Eigen::MatrixXd UKF::generate_sigma_points() {
  int n_sigma = 2 * n_aug_ + 1;
  Eigen::MatrixXd sigma(STATE_DIM, n_sigma);
  // Symmetrize P before factoring to eliminate floating-point asymmetry.
  state_.P = (state_.P + state_.P.transpose()) * 0.5;

  // Without gyro measurements, P(WX/WY/WZ) grows unboundedly via q_angular_vel
  // each predict step. The kinematic WZ→QZ coupling then makes P near-singular,
  // triggering the identity-shift Cholesky repair which introduces asymmetry that
  // Wm[0]≈-99 amplifies into QZ drift. Cap at a physically large but finite bound.
  constexpr double kMaxAngVelVar = 1.0;
  state_.P(E_WX,E_WX) = std::min(state_.P(E_WX,E_WX), kMaxAngVelVar);
  state_.P(E_WY,E_WY) = std::min(state_.P(E_WY,E_WY), kMaxAngVelVar);
  state_.P(E_WZ,E_WZ) = std::min(state_.P(E_WZ,E_WZ), kMaxAngVelVar);

  ErrorMatrix P_reg = (n_aug_ + lambda_) * state_.P;
  P_reg += ErrorMatrix::Identity() * 1e-6;

  Eigen::LLT<ErrorMatrix> llt(P_reg);
  if (llt.info() != Eigen::Success) {
    // P has developed negative eigenvalues (common under sustained high-rate IMU
    // updates where the K*S*K^T subtraction overshoots in the bias dimensions).
    // Fix: identity shift: add eps*I to raise all eigenvalues by |min_eigenvalue|.
    // Unlike the V*max(λ,ε)*V^T clamp, a shift preserves the eigenvectors and keeps
    // large position/velocity eigenvalues intact.  The clamp approach destroys them
    // because P's eigenvectors mix position and bias components after cross-coupling,
    // so clamping bias eigenvalues to 1e-9 also collapses position uncertainty when
    // P is reconstructed, making the Mahalanobis gate far too tight and rejecting GPS.
    Eigen::SelfAdjointEigenSolver<ErrorMatrix> es(state_.P, Eigen::EigenvaluesOnly);
    double min_eigen = es.eigenvalues().minCoeff();
    state_.P += ErrorMatrix::Identity() * (-min_eigen + 1e-9);
    P_reg = (n_aug_ + lambda_) * state_.P + ErrorMatrix::Identity() * 1e-6;
    llt.compute(P_reg);
    if (llt.info() != Eigen::Success) {
      // Last resort: rebuild P from its eigendecomposition with all eigenvalues
      // floored to a small positive value. This always yields a PSD matrix, so
      // the filter degrades gracefully and re-converges when good measurements
      // return, instead of aborting the whole process. Flooring can shrink some
      // uncertainty dimensions (the identity shift above is tried first precisely
      // to avoid that), but a briefly over-confident filter is recoverable; a
      // crash is not. Reaching here means the state was already badly corrupted
      // (e.g. by replay clock chaos), which the predict_to re-sync guards upstream.
      Eigen::SelfAdjointEigenSolver<ErrorMatrix> es_full(state_.P);
      Eigen::Matrix<double, ERROR_DIM, 1> ev = es_full.eigenvalues().cwiseMax(1e-9);
      state_.P = es_full.eigenvectors() * ev.asDiagonal() * es_full.eigenvectors().transpose();
      state_.P = (state_.P + state_.P.transpose()) * 0.5;
      P_reg = (n_aug_ + lambda_) * state_.P + ErrorMatrix::Identity() * 1e-6;
      llt.compute(P_reg);
      if (llt.info() != Eigen::Success) {
        // Absolute last resort: reset to a safe diagonal covariance. Never throw.
        state_.P = ErrorMatrix::Identity();
        P_reg = (n_aug_ + lambda_) * state_.P + ErrorMatrix::Identity() * 1e-6;
        llt.compute(P_reg);
      }
    }
  }
  ErrorMatrix L = llt.matrixL();
  // Each column of L is an ERROR vector, so it is INJECTED rather than added: the
  // attitude part composes on the manifold via boxplus and everything else adds. This is
  // the change that makes every sigma point a valid rotation by construction, however
  // large the attitude uncertainty becomes.
  sigma.col(0) = state_.x;
  for (int i = 0; i < n_aug_; ++i) {
    const ErrorVector e = L.col(i);
    sigma.col(i + 1)          = inject_error(state_.x,  e, err_frame());
    sigma.col(i + 1 + n_aug_) = inject_error(state_.x, -e, err_frame());
  }

  // Keep each sigma point's attitude a physically meaningful sample. See
  // UKFParams::max_sigma_rotation_deg for why, and for why this bounds the POINTS and
  // deliberately leaves P alone.
  if (params_.max_sigma_rotation_deg > 0.0) {
    const double max_rad = params_.max_sigma_rotation_deg * M_PI / 180.0;
    Eigen::Vector4d q0 = state_.x.segment<4>(QW);
    const double n0 = q0.norm();
    if (n0 > 1e-9) {
      q0 /= n0;
      for (int c = 1; c < n_sigma; ++c) {
        Eigen::Vector4d qs = sigma.col(c).segment<4>(QW);
        const double ns = qs.norm();
        if (ns < 1e-9) { sigma.col(c).segment<4>(QW) = q0; continue; }
        qs /= ns;
        // Same hemisphere, so the angle measured is the short way round.
        double dot = q0.dot(qs);
        if (dot < 0.0) { qs = -qs; dot = -dot; }
        dot = std::min(1.0, std::max(-1.0, dot));
        const double angle = 2.0 * std::acos(dot);        // rotation, radians
        if (angle > max_rad) {
          // Slerp back toward the mean until the rotation is exactly max_rad.
          const double t = max_rad / angle;
          const double theta = std::acos(dot);            // half-angle
          const double sin_theta = std::sin(theta);
          if (sin_theta > 1e-9) {
            qs = (std::sin((1.0 - t) * theta) * q0 + std::sin(t * theta) * qs) / sin_theta;
            qs.normalize();
          }
        }
        sigma.col(c).segment<4>(QW) = qs;
      }
    }
  }
  return sigma;
}

void UKF::predict(double dt) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: predict() called before init()");
  Eigen::MatrixXd sigma = generate_sigma_points();
  int n_sigma = 2 * n_aug_ + 1;
  Eigen::MatrixXd sigma_pred(STATE_DIM, n_sigma);
  for (int i = 0; i < n_sigma; ++i)
    sigma_pred.col(i) = motion_model_->predict(sigma.col(i), dt);
  // Quaternion sign consistency: q and -q represent the same rotation, but
  // their weighted sum does not. Flip any sigma point whose quaternion is in
  // the opposite hemisphere from sigma_pred[0] before averaging.
  for (int i = 1; i < n_sigma; ++i) {
    double dot = sigma_pred.col(0).segment<4>(QW).dot(sigma_pred.col(i).segment<4>(QW));
    if (dot < 0.0) sigma_pred.col(i).segment<4>(QW) *= -1.0;
  }

  // Weighted mean. Non-attitude states are a plain weighted sum; attitude is averaged on
  // the manifold about the current estimate.
  //
  // The old code summed all four quaternion components and renormalised, on the stated
  // grounds that the sigma-point spread is "guaranteed small at 100 Hz where dt is tiny".
  // That was wrong and it was the bug: the spread comes from P, not from dt, and the yaw
  // block grows without bound because yaw is unobservable. Once the points span the
  // circle their sum cancels and normalising the residue aims it anywhere, which is the
  // measured 180 degree flip at t=77 s.
  StateVector x_pred = StateVector::Zero();
  for (int i = 0; i < n_sigma; ++i)
    x_pred += Wm_[i] * sigma_pred.col(i);

  // Averaging in the tangent space about sigma point 0. There is nothing to cancel here,
  // so a wide spread degrades the answer gradually instead of inverting it.
  {
    Eigen::Quaterniond ref(sigma_pred(QW, 0), sigma_pred(QX, 0),
                           sigma_pred(QY, 0), sigma_pred(QZ, 0));
    if (ref.norm() > 1e-12) {
      ref.normalize();
      for (int iter = 0; iter < 8; ++iter) {
        Eigen::Vector3d mean_err = Eigen::Vector3d::Zero();
        for (int i = 0; i < n_sigma; ++i) {
          const Eigen::Quaterniond qi(sigma_pred(QW, i), sigma_pred(QX, i),
                                      sigma_pred(QY, i), sigma_pred(QZ, i));
          if (qi.norm() < 1e-12) { continue; }
          mean_err += Wm_[i] * attitude::boxminus(qi, ref, err_frame());
        }
        if (mean_err.norm() < 1e-13) { break; }
        ref = attitude::boxplus(ref, mean_err, err_frame());
        if (mean_err.norm() < 1e-10) { break; }
      }
      x_pred[QW] = ref.w(); x_pred[QX] = ref.x();
      x_pred[QY] = ref.y(); x_pred[QZ] = ref.z();
    }
  }
  x_pred = normalize_state(x_pred);

  // Q is added per predict step (not scaled by dt).
  // The q_* params in UKFParams are calibrated as per-step noise, not spectral densities.
  // Scaling by dt here would require re-tuning all Q values by the IMU rate (~100x),
  // which would break all existing configurations and cause GPS Mahalanobis rejections
  // as P grows too slowly to track real motion.
  ErrorMatrix P_pred = Q_;
  if (pos_noise_scale_ != 1.0) {
    P_pred(X, X) *= pos_noise_scale_;
    P_pred(Y, Y) *= pos_noise_scale_;
    P_pred(Z, Z) *= pos_noise_scale_;
  }
  if (gyro_bias_noise_scale_ != 1.0) {
    P_pred(B_GX, B_GX) *= gyro_bias_noise_scale_;
    P_pred(B_GY, B_GY) *= gyro_bias_noise_scale_;
    P_pred(B_GZ, B_GZ) *= gyro_bias_noise_scale_;
  }
  for (int i = 0; i < n_sigma; ++i) {
    // The residual is taken in the ERROR space, which is what makes it consistent with
    // the manifold mean above. Measuring it as an ambient 4-vector difference against a
    // manifold mean was tried and does not work: the residuals stop summing to zero and
    // P picks up a spurious term that leaks through the quaternion/acceleration cross
    // covariance, giving a 22% distance overshoot and +-5 m/s^2 of accelerometer
    // oscillation on a still sensor. The mean and the residual have to move together.
    const ErrorVector diff = extract_error(sigma_pred.col(i), x_pred, err_frame());
    P_pred += Wc_[i] * diff * diff.transpose();
  }
  state_.x = x_pred;
  state_.P = P_pred;
}

template <int z_dim>
Eigen::Matrix<double, z_dim, 1> UKF::update(
  const Eigen::Matrix<double, z_dim, 1>& z,
  const std::function<Eigen::Matrix<double, z_dim, 1>(const StateVector&)>& h,
  const Eigen::Matrix<double, z_dim, z_dim>& R,
  unsigned int angle_dims
) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: update() called before init()");

  using ZVector   = Eigen::Matrix<double, z_dim, 1>;
  using ZMatrix   = Eigen::Matrix<double, z_dim, z_dim>;
  using PxzMatrix = Eigen::Matrix<double, ERROR_DIM, z_dim>;

  Eigen::MatrixXd sigma = generate_sigma_points();
  int n_sigma = 2 * n_aug_ + 1;

  Eigen::Matrix<double, z_dim, Eigen::Dynamic> sigma_z(z_dim, n_sigma);
  for (int i = 0; i < n_sigma; ++i)
    sigma_z.col(i) = h(sigma.col(i));

  // Weighted mean of measurement sigma points.
  // Circular mean (atan2) is NOT used here: UKF has Wm[0] ≈ -99 and Wm[i>0] ≈ +2.38.
  // With yaw near 0: sum_cos ≈ -99 + 99*cos(spread) ≈ -0.05 (negative due to cos < 1),
  // so atan2(0, -0.05) = π instead of 0, making every z_diff ≈ π → S goes negative
  // → K*S*K.T has wrong sign → P goes non-PSD → Cholesky crash.
  ZVector z_pred = ZVector::Zero();
  for (int i = 0; i < n_sigma; ++i)
    z_pred += Wm_[i] * sigma_z.col(i);

  // Angle dimensions cannot use that linear mean: it is only valid while the
  // sigma points stay on one side of the +-pi cut. Take the weighted mean about a
  // reference sigma point instead, wrapping each offset to the short way round.
  // Because the weights sum to 1, ref + sum(Wm[i] * wrap(z[i] - ref)) IS the
  // weighted mean, and unlike a circular atan2 mean it stays correct when Wm[0]
  // is large and negative, which is what ruled the atan2 form out above.
  for (int d = 0; d < z_dim; ++d) {
    if (!(angle_dims & (1u << d))) continue;
    const double ref = sigma_z(d, 0);
    double acc = 0.0;
    for (int i = 0; i < n_sigma; ++i)
      acc += Wm_[i] * normalize_angle(sigma_z(d, i) - ref);
    z_pred[d] = normalize_angle(ref + acc);
  }

  ZMatrix   S   = R;
  PxzMatrix Pxz = PxzMatrix::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    ZVector z_diff = sigma_z.col(i) - z_pred;
    for (int d = 0; d < z_dim; ++d)
      if (angle_dims & (1u << d)) z_diff[d] = normalize_angle(z_diff[d]);
    const ErrorVector x_diff = extract_error(sigma.col(i), state_.x, err_frame());
    S   += Wc_[i] * z_diff * z_diff.transpose();
    Pxz += Wc_[i] * x_diff * z_diff.transpose();
  }

  ZVector innovation = z - z_pred;
  for (int d = 0; d < z_dim; ++d)
    if (angle_dims & (1u << d)) innovation[d] = normalize_angle(innovation[d]);

  // Use LDLT decomposition instead of direct S.inverse(): numerically stable
  // when S is near-singular. K = Pxz * S^{-1} = (S^{-1} * Pxz^T)^T.
  auto S_ldlt = S.ldlt();
  PxzMatrix K = S_ldlt.solve(Pxz.transpose()).transpose();
  const ErrorVector correction = K * innovation;
  last_pos_correction_ = std::hypot(correction[E_X], correction[E_Y]);
  // The attitude part of the correction is a rotation, so it is applied as one.
  state_.x = normalize_state(inject_error(state_.x, correction, err_frame()));
  state_.P -= K * S * K.transpose();
  // Symmetrize after each update to prevent floating-point asymmetry from
  // accumulating across the ~100 Hz IMU + 1 Hz GPS update stream.
  state_.P = (state_.P + state_.P.transpose()) * 0.5;
  return innovation;
}

template <int z_dim>
void UKF::predict_measurement(
  const Eigen::Matrix<double, z_dim, 1>& z,
  const std::function<Eigen::Matrix<double, z_dim, 1>(const StateVector&)>& h,
  const Eigen::Matrix<double, z_dim, z_dim>& R,
  Eigen::Matrix<double, z_dim, 1>& innovation_out,
  Eigen::Matrix<double, z_dim, z_dim>& S_out,
  unsigned int angle_dims
) {
  using ZVector = Eigen::Matrix<double, z_dim, 1>;
  using ZMatrix = Eigen::Matrix<double, z_dim, z_dim>;

  Eigen::MatrixXd sigma = generate_sigma_points();
  int n_sigma = 2 * n_aug_ + 1;

  Eigen::Matrix<double, z_dim, Eigen::Dynamic> sigma_z(z_dim, n_sigma);
  for (int i = 0; i < n_sigma; ++i)
    sigma_z.col(i) = h(sigma.col(i));

  // Same weighted mean as update(), and the same reference-wrapped correction for
  // angle dimensions. These two must agree: predict_measurement() is what the chi2
  // outlier gate reads, so a mean computed differently here would gate on one
  // number and fuse on another.
  ZVector z_pred = ZVector::Zero();
  for (int i = 0; i < n_sigma; ++i)
    z_pred += Wm_[i] * sigma_z.col(i);

  // Angle dimensions cannot use that linear mean: it is only valid while the
  // sigma points stay on one side of the +-pi cut. Take the weighted mean about a
  // reference sigma point instead, wrapping each offset to the short way round.
  // Because the weights sum to 1, ref + sum(Wm[i] * wrap(z[i] - ref)) IS the
  // weighted mean, and unlike a circular atan2 mean it stays correct when Wm[0]
  // is large and negative, which is what ruled the atan2 form out above.
  for (int d = 0; d < z_dim; ++d) {
    if (!(angle_dims & (1u << d))) continue;
    const double ref = sigma_z(d, 0);
    double acc = 0.0;
    for (int i = 0; i < n_sigma; ++i)
      acc += Wm_[i] * normalize_angle(sigma_z(d, i) - ref);
    z_pred[d] = normalize_angle(ref + acc);
  }

  ZMatrix S = R;
  for (int i = 0; i < n_sigma; ++i) {
    ZVector z_diff = sigma_z.col(i) - z_pred;
    for (int d = 0; d < z_dim; ++d)
      if (angle_dims & (1u << d)) z_diff[d] = normalize_angle(z_diff[d]);
    S += Wc_[i] * z_diff * z_diff.transpose();
  }

  innovation_out = z - z_pred;
  for (int d = 0; d < z_dim; ++d)
    if (angle_dims & (1u << d)) innovation_out[d] = normalize_angle(innovation_out[d]);
  S_out = S;
}

// Explicit instantiations for predict_measurement
template void UKF::predict_measurement<2>(
  const Eigen::Matrix<double, 2, 1>&,
  const std::function<Eigen::Matrix<double, 2, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 2, 2>&,
  Eigen::Matrix<double, 2, 1>&,
  Eigen::Matrix<double, 2, 2>&,
  unsigned int);

template void UKF::predict_measurement<1>(
  const Eigen::Matrix<double, 1, 1>&,
  const std::function<Eigen::Matrix<double, 1, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 1, 1>&,
  Eigen::Matrix<double, 1, 1>&,
  Eigen::Matrix<double, 1, 1>&,
  unsigned int);

template void UKF::predict_measurement<3>(
  const Eigen::Matrix<double, 3, 1>&,
  const std::function<Eigen::Matrix<double, 3, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 3, 3>&,
  Eigen::Matrix<double, 3, 1>&,
  Eigen::Matrix<double, 3, 3>&,
  unsigned int);

template void UKF::predict_measurement<6>(
  const Eigen::Matrix<double, 6, 1>&,
  const std::function<Eigen::Matrix<double, 6, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 6, 6>&,
  Eigen::Matrix<double, 6, 1>&,
  Eigen::Matrix<double, 6, 6>&,
  unsigned int);

double UKF::normalize_angle(double angle) {
  // fmod-based normalization: O(1) regardless of magnitude, safe under drift
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) angle += 2.0 * M_PI;
  return angle - M_PI;
}

StateVector UKF::normalize_state(const StateVector& x) {
  StateVector x_norm = x;
  double qnorm = std::sqrt(x[QW]*x[QW] + x[QX]*x[QX] + x[QY]*x[QY] + x[QZ]*x[QZ]);
  if (qnorm > 1e-10) {
    x_norm[QW] /= qnorm;
    x_norm[QX] /= qnorm;
    x_norm[QY] /= qnorm;
    x_norm[QZ] /= qnorm;
  } else {
    x_norm[QW] = 1.0;
    x_norm[QX] = x_norm[QY] = x_norm[QZ] = 0.0;
  }
  return x_norm;
}

// Explicit template instantiations
template Eigen::Matrix<double, 2, 1> UKF::update<2>(
  const Eigen::Matrix<double, 2, 1>&,
  const std::function<Eigen::Matrix<double, 2, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 2, 2>&,
  unsigned int
);
template Eigen::Matrix<double, 1, 1> UKF::update<1>(
  const Eigen::Matrix<double, 1, 1>&,
  const std::function<Eigen::Matrix<double, 1, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 1, 1>&,
  unsigned int
);
template Eigen::Matrix<double, 3, 1> UKF::update<3>(
  const Eigen::Matrix<double, 3, 1>&,
  const std::function<Eigen::Matrix<double, 3, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 3, 3>&,
  unsigned int
);
template Eigen::Matrix<double, 6, 1> UKF::update<6>(
  const Eigen::Matrix<double, 6, 1>&,
  const std::function<Eigen::Matrix<double, 6, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 6, 6>&,
  unsigned int
);

} // namespace fusioncore
