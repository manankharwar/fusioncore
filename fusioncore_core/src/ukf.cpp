#include "fusioncore/ukf.hpp"
#include <cmath>
#include <stdexcept>

namespace fusioncore {

UKF::UKF(const UKFParams& params)
  : params_(params), initialized_(false)
{
  n_aug_  = STATE_DIM;
  lambda_ = params_.alpha * params_.alpha * (n_aug_ + params_.kappa) - n_aug_;
  compute_weights();
  build_process_noise();
}

void UKF::init(const State& initial_state) {
  state_       = initial_state;
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
  Q_.diagonal() << params_.q_position, params_.q_position, params_.q_position,
                   params_.q_orientation, params_.q_orientation, params_.q_orientation,
                   params_.q_velocity, params_.q_velocity, params_.q_velocity,
                   params_.q_angular_vel, params_.q_angular_vel, params_.q_angular_vel,
                   params_.q_acceleration, params_.q_acceleration, params_.q_acceleration,
                   params_.q_gyro_bias, params_.q_gyro_bias, params_.q_gyro_bias,
                   params_.q_accel_bias, params_.q_accel_bias, params_.q_accel_bias;
}

Eigen::MatrixXd UKF::generate_sigma_points() {
  int n_sigma = 2 * n_aug_ + 1;
  Eigen::MatrixXd sigma(STATE_DIM, n_sigma);
  // Symmetrize P before factoring to eliminate floating-point asymmetry.
  state_.P = (state_.P + state_.P.transpose()) * 0.5;
  StateMatrix P_reg = (n_aug_ + lambda_) * state_.P;
  P_reg += StateMatrix::Identity() * 1e-6;

  Eigen::LLT<StateMatrix> llt(P_reg);
  if (llt.info() != Eigen::Success) {
    // P has developed negative eigenvalues (common under sustained high-rate IMU
    // updates where the K*S*K^T subtraction overshoots in the bias dimensions).
    // Fix: identity shift — add eps*I to raise all eigenvalues by |min_eigenvalue|.
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
    if (llt.info() != Eigen::Success)
      throw std::runtime_error("FusionCore: Cholesky decomposition failed after P repair");
  }
  StateMatrix L = llt.matrixL();
  sigma.col(0) = state_.x;
  for (int i = 0; i < n_aug_; ++i) {
    sigma.col(i + 1)          = state_.x + L.col(i);
    sigma.col(i + 1 + n_aug_) = state_.x - L.col(i);
  }
  return sigma;
}

StateVector UKF::process_model(const StateVector& x, double dt) const {
  StateVector x_new = x;
  double roll  = x[ROLL], pitch = x[PITCH], yaw = x[YAW];
  double cr = std::cos(roll),  sr = std::sin(roll);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cy = std::cos(yaw),   sy = std::sin(yaw);
  double vx = x[VX], vy = x[VY], vz = x[VZ];
  x_new[X] += dt * (cy*cp*vx + (cy*sp*sr - sy*cr)*vy + (cy*sp*cr + sy*sr)*vz);
  x_new[Y] += dt * (sy*cp*vx + (sy*sp*sr + cy*cr)*vy + (sy*sp*cr - cy*sr)*vz);
  x_new[Z] += dt * (-sp*vx   + cp*sr*vy             + cp*cr*vz);
  // WX/WY/WZ are the true angular rates; B_GX/B_GY/B_GZ are gyro biases.
  // The measurement model predicts z = true_rate + bias, so propagation uses
  // the true rate directly — do NOT subtract bias here.
  double wx = x[WX];
  double wy = x[WY];
  double wz = x[WZ];
  // Guard against Euler singularity (gimbal lock) at pitch = ±90° where
  // tan(pitch) and 1/cos(pitch) diverge. Clamp cos(pitch) away from zero.
  double cp_safe = (std::abs(cp) < 1e-4) ? std::copysign(1e-4, cp) : cp;
  x_new[ROLL]  += dt * (wx + sr*(sp/cp_safe)*wy + cr*(sp/cp_safe)*wz);
  x_new[PITCH] += dt * (cr*wy - sr*wz);
  x_new[YAW]   += dt * (sr/cp_safe*wy + cr/cp_safe*wz);
  // AX/AY/AZ are true body-frame accelerations (gravity already handled in
  // the measurement model). Velocity integrates true acceleration directly.
  x_new[VX] += dt * x[AX];
  x_new[VY] += dt * x[AY];
  x_new[VZ] += dt * x[AZ];
  x_new = normalize_state(x_new);
  return x_new;
}

void UKF::predict(double dt) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: predict() called before init()");
  Eigen::MatrixXd sigma = generate_sigma_points();
  int n_sigma = 2 * n_aug_ + 1;
  Eigen::MatrixXd sigma_pred(STATE_DIM, n_sigma);
  for (int i = 0; i < n_sigma; ++i)
    sigma_pred.col(i) = process_model(sigma.col(i), dt);
  // Weighted mean — normalize_state handles angle wrapping.
  // Note: circular mean (atan2) is NOT used here because when P is large (e.g.
  // during initialization with P=50*I), angle sigma points spread beyond ±π and
  // wrap around. atan2 then returns the wrong quadrant (±π instead of 0), making
  // diff_0 = ±π for every sigma point, and the large negative Wc[0] weight drives
  // P_pred[angle, angle] to be large and negative → Cholesky failure.
  // Circular mean is correct only in update()/predict_measurement() where the
  // measurement function returns a bounded heading value with small spread.
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
  for (int i = 0; i < n_sigma; ++i) {
    StateVector diff = normalize_state(sigma_pred.col(i) - x_pred);
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
  // Circular mean (atan2) is NOT used here — UKF has Wm[0] ≈ -99 and Wm[i>0] ≈ +2.38.
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
    StateVector x_diff = normalize_state(sigma.col(i) - state_.x);
    S   += Wc_[i] * z_diff * z_diff.transpose();
    Pxz += Wc_[i] * x_diff * z_diff.transpose();
  }

  ZVector innovation = z - z_pred;
  for (int d = 0; d < z_dim; ++d)
    if (angle_dims & (1u << d)) innovation[d] = normalize_angle(innovation[d]);

  // Use LDLT decomposition instead of direct S.inverse() — numerically stable
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

  // Same weighted mean as update() — no circular mean for same reason.
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
  // fmod-based normalization — O(1) regardless of magnitude, safe under drift
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) angle += 2.0 * M_PI;
  return angle - M_PI;
}

StateVector UKF::normalize_state(const StateVector& x) {
  StateVector x_norm = x;
  x_norm[ROLL]  = normalize_angle(x[ROLL]);
  x_norm[PITCH] = normalize_angle(x[PITCH]);
  x_norm[YAW]   = normalize_angle(x[YAW]);
  return x_norm;
}

// Explicit template instantiations
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
