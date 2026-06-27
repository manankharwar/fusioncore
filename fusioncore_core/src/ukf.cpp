#include "fusioncore/ukf.hpp"
#include "fusioncore/motion_model.hpp"
#include <cmath>
#include <stdexcept>

namespace fusioncore {

UKF::UKF(const UKFParams& params)
  : params_(params), initialized_(false),
    motion_model_(std::make_shared<ConstantVelocityAcceleration>())
{
  n_aug_  = STATE_DIM;
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
  Wm_[0] = lambda_ / (n_aug_ + lambda_);
  Wc_[0] = Wm_[0] + (1.0 - params_.alpha * params_.alpha + params_.beta);
  double w = 0.5 / (n_aug_ + lambda_);
  for (int i = 1; i < n_sigma; ++i) {
    Wm_[i] = w;
    Wc_[i] = w;
  }
}

void UKF::build_process_noise() {
  Q_ = StateMatrix::Zero();
  // Quaternion has 4 components; q_orientation is a small regularization term.
  // Primary orientation noise enters through q_angular_vel via kinematics.
  Q_.diagonal() << params_.q_position, params_.q_position, params_.q_position,
                   params_.q_orientation, params_.q_orientation, params_.q_orientation, params_.q_orientation,
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
  state_.P(WX,WX) = std::min(state_.P(WX,WX), kMaxAngVelVar);
  state_.P(WY,WY) = std::min(state_.P(WY,WY), kMaxAngVelVar);
  state_.P(WZ,WZ) = std::min(state_.P(WZ,WZ), kMaxAngVelVar);

  StateMatrix P_reg = (n_aug_ + lambda_) * state_.P;
  P_reg += StateMatrix::Identity() * 1e-6;

  Eigen::LLT<StateMatrix> llt(P_reg);
  if (llt.info() != Eigen::Success) {
    // P has developed negative eigenvalues (common under sustained high-rate IMU
    // updates where the K*S*K^T subtraction overshoots in the bias dimensions).
    // Fix: identity shift: add eps*I to raise all eigenvalues by |min_eigenvalue|.
    // Unlike the V*max(λ,ε)*V^T clamp, a shift preserves the eigenvectors and keeps
    // large position/velocity eigenvalues intact.  The clamp approach destroys them
    // because P's eigenvectors mix position and bias components after cross-coupling,
    // so clamping bias eigenvalues to 1e-9 also collapses position uncertainty when
    // P is reconstructed, making the Mahalanobis gate far too tight and rejecting GPS.
    Eigen::SelfAdjointEigenSolver<StateMatrix> es(state_.P, Eigen::EigenvaluesOnly);
    double min_eigen = es.eigenvalues().minCoeff();
    state_.P += StateMatrix::Identity() * (-min_eigen + 1e-9);
    P_reg = (n_aug_ + lambda_) * state_.P + StateMatrix::Identity() * 1e-6;
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
      Eigen::SelfAdjointEigenSolver<StateMatrix> es_full(state_.P);
      Eigen::Matrix<double, STATE_DIM, 1> ev = es_full.eigenvalues().cwiseMax(1e-9);
      state_.P = es_full.eigenvectors() * ev.asDiagonal() * es_full.eigenvectors().transpose();
      state_.P = (state_.P + state_.P.transpose()) * 0.5;
      P_reg = (n_aug_ + lambda_) * state_.P + StateMatrix::Identity() * 1e-6;
      llt.compute(P_reg);
      if (llt.info() != Eigen::Success) {
        // Absolute last resort: reset to a safe diagonal covariance. Never throw.
        state_.P = StateMatrix::Identity();
        P_reg = (n_aug_ + lambda_) * state_.P + StateMatrix::Identity() * 1e-6;
        llt.compute(P_reg);
      }
    }
  }
  StateMatrix L = llt.matrixL();
  sigma.col(0) = state_.x;
  for (int i = 0; i < n_aug_; ++i) {
    sigma.col(i + 1)          = state_.x + L.col(i);
    sigma.col(i + 1 + n_aug_) = state_.x - L.col(i);
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

  // Weighted mean: normalize_state renormalizes the quaternion after summing.
  // Simple weighted sum + normalize is accurate when sigma-point spread is small
  // (guaranteed at 100 Hz where dt is tiny). No circular mean needed: the
  // quaternion representation has no angle-wrapping discontinuity.
  StateVector x_pred = StateVector::Zero();
  for (int i = 0; i < n_sigma; ++i)
    x_pred += Wm_[i] * sigma_pred.col(i);
  x_pred = normalize_state(x_pred);

  // Q is added per predict step (not scaled by dt).
  // The q_* params in UKFParams are calibrated as per-step noise, not spectral densities.
  // Scaling by dt here would require re-tuning all Q values by the IMU rate (~100x),
  // which would break all existing configurations and cause GPS Mahalanobis rejections
  // as P grows too slowly to track real motion.
  StateMatrix P_pred = Q_;
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
    // Plain Euclidean difference: no normalize_state here.
    // With quaternion state there is no angle wrapping; normalizing a diff
    // vector would treat it as a quaternion and scale it to unit length,
    // completely corrupting the covariance.
    StateVector diff = sigma_pred.col(i) - x_pred;
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
  using PxzMatrix = Eigen::Matrix<double, STATE_DIM, z_dim>;

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
  // Angle wrapping is handled correctly via normalize_angle on z_diff and innovation below.
  ZVector z_pred = ZVector::Zero();
  for (int i = 0; i < n_sigma; ++i)
    z_pred += Wm_[i] * sigma_z.col(i);

  ZMatrix   S   = R;
  PxzMatrix Pxz = PxzMatrix::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    ZVector z_diff = sigma_z.col(i) - z_pred;
    for (int d = 0; d < z_dim; ++d)
      if (angle_dims & (1u << d)) z_diff[d] = normalize_angle(z_diff[d]);
    StateVector x_diff = sigma.col(i) - state_.x;  // plain diff, no angle wrapping needed
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
  state_.x = normalize_state(state_.x + K * innovation);
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

  // Same weighted mean as update(): no circular mean for same reason.
  // Angle wrapping is handled via normalize_angle on z_diff and innovation_out below.
  ZVector z_pred = ZVector::Zero();
  for (int i = 0; i < n_sigma; ++i)
    z_pred += Wm_[i] * sigma_z.col(i);

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
