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

Eigen::MatrixXd UKF::generate_sigma_points() const {
  int n_sigma = 2 * n_aug_ + 1;
  Eigen::MatrixXd sigma(STATE_DIM, n_sigma);
  Eigen::LLT<StateMatrix> llt((n_aug_ + lambda_) * state_.P);
  if (llt.info() != Eigen::Success)
    throw std::runtime_error("FusionCore: Cholesky decomposition failed");
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
  double wx = x[WX] - x[B_GX];
  double wy = x[WY] - x[B_GY];
  double wz = x[WZ] - x[B_GZ];
  x_new[ROLL]  += dt * (wx + sr*std::tan(pitch)*wy + cr*std::tan(pitch)*wz);
  x_new[PITCH] += dt * (cr*wy - sr*wz);
  x_new[YAW]   += dt * (sr/cp*wy + cr/cp*wz);
  x_new[VX] += dt * (x[AX] - x[B_AX]);
  x_new[VY] += dt * (x[AY] - x[B_AY]);
  x_new[VZ] += dt * (x[AZ] - x[B_AZ]);
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
  StateVector x_pred = StateVector::Zero();
  for (int i = 0; i < n_sigma; ++i)
    x_pred += Wm_[i] * sigma_pred.col(i);
  x_pred = normalize_state(x_pred);
  StateMatrix P_pred = Q_;
  for (int i = 0; i < n_sigma; ++i) {
    StateVector diff = normalize_state(sigma_pred.col(i) - x_pred);
    P_pred += Wc_[i] * diff * diff.transpose();
  }
  state_.x = x_pred;
  state_.P = P_pred;
}

template <int z_dim>
void UKF::update(
  const Eigen::Matrix<double, z_dim, 1>& z,
  const std::function<Eigen::Matrix<double, z_dim, 1>(const StateVector&)>& h,
  const Eigen::Matrix<double, z_dim, z_dim>& R
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

  ZVector z_pred = ZVector::Zero();
  for (int i = 0; i < n_sigma; ++i)
    z_pred += Wm_[i] * sigma_z.col(i);

  ZMatrix   S   = R;
  PxzMatrix Pxz = PxzMatrix::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    ZVector    z_diff = sigma_z.col(i) - z_pred;
    StateVector x_diff = normalize_state(sigma.col(i) - state_.x);
    S   += Wc_[i] * z_diff * z_diff.transpose();
    Pxz += Wc_[i] * x_diff * z_diff.transpose();
  }

  PxzMatrix K = Pxz * S.inverse();
  state_.x = normalize_state(state_.x + K * (z - z_pred));
  state_.P -= K * S * K.transpose();
}

double UKF::normalize_angle(double angle) {
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

StateVector UKF::normalize_state(const StateVector& x) {
  StateVector x_norm = x;
  x_norm[ROLL]  = normalize_angle(x[ROLL]);
  x_norm[PITCH] = normalize_angle(x[PITCH]);
  x_norm[YAW]   = normalize_angle(x[YAW]);
  return x_norm;
}

// Explicit template instantiations
template void UKF::update<1>(
  const Eigen::Matrix<double, 1, 1>&,
  const std::function<Eigen::Matrix<double, 1, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 1, 1>&
);
template void UKF::update<3>(
  const Eigen::Matrix<double, 3, 1>&,
  const std::function<Eigen::Matrix<double, 3, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 3, 3>&
);
template void UKF::update<6>(
  const Eigen::Matrix<double, 6, 1>&,
  const std::function<Eigen::Matrix<double, 6, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 6, 6>&
);
template void UKF::update<7>(
  const Eigen::Matrix<double, 7, 1>&,
  const std::function<Eigen::Matrix<double, 7, 1>(const StateVector&)>&,
  const Eigen::Matrix<double, 7, 7>&
);

} // namespace fusioncore
