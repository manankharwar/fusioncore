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


// Local rotation error between a sigma point's attitude and a reference, written
// into the quaternion slots of a state-difference vector.
//
// dq = q_ref^-1 * q_sigma is the rotation FROM the reference TO the sigma point.
// For a small rotation its vector part is dtheta/2, so 2*vec(dq) is the rotation
// vector in the reference's own frame, which is what a covariance should measure.
// The scalar slot carries dq.w - 1, which is second order and therefore ~0.
static void attitude_difference(fusioncore::StateVector& diff,
                                const fusioncore::StateVector& sigma,
                                const fusioncore::StateVector& ref)
{
  using namespace fusioncore;
  Eigen::Quaterniond qs(sigma[QW], sigma[QX], sigma[QY], sigma[QZ]);
  Eigen::Quaterniond qr(ref[QW],   ref[QX],   ref[QY],   ref[QZ]);
  if (qs.norm() < 1e-12 || qr.norm() < 1e-12) return;   // leave the plain diff alone
  qs.normalize(); qr.normalize();
  Eigen::Quaterniond dq = qr.conjugate() * qs;
  if (dq.w() < 0.0) { dq.coeffs() *= -1.0; }            // short way round

  // EXACT log map, not the small-angle 2*vec(dq).
  //
  // vec(dq) is sin(theta/2)*axis, so 2*vec(dq) equals theta only while theta is
  // small. apply_attitude_correction below uses the exact exponential map, so the
  // small-angle inverse made the two maps inverses of each other ONLY near zero.
  // Measured as a progressively lossy round trip: feeding P(QZ,QZ) back through
  // generation and differencing returned a ratio of 1.0002 at 1e-4, 0.981 at 1e-2,
  // 0.926 at 0.04 and 0.604 at 0.25. At 23 states the sigma spread is
  // sqrt((n+lambda)*P), so a variance of 0.25 means sigma points sitting about
  // 2.4 rad away, exactly where 2*sin(theta/2) badly underestimates theta.
  // The covariance therefore shrank every time it was rebuilt.
  const double vnorm = dq.vec().norm();
  Eigen::Vector3d rv;
  if (vnorm > 1e-12) {
    rv = (2.0 * std::atan2(vnorm, dq.w()) / vnorm) * dq.vec();
  } else {
    rv = 2.0 * dq.vec();
  }
  diff[QW] = dq.w() - 1.0;
  diff[QX] = rv.x();
  diff[QY] = rv.y();
  diff[QZ] = rv.z();
}

// Apply a correction whose quaternion slots are a local rotation vector.
// q_new = q_old * exp(dtheta/2), which keeps the result on the unit sphere by
// construction instead of adding and renormalising.
static void apply_attitude_correction(fusioncore::StateVector& x,
                                      const fusioncore::StateVector& correction)
{
  using namespace fusioncore;
  Eigen::Vector3d dtheta(correction[QX], correction[QY], correction[QZ]);
  Eigen::Quaterniond q(x[QW], x[QX], x[QY], x[QZ]);
  if (q.norm() < 1e-12) return;
  q.normalize();
  const double angle = dtheta.norm();
  Eigen::Quaterniond dq;
  if (angle > 1e-12) {
    const double half = 0.5 * angle;
    const Eigen::Vector3d axis = dtheta / angle;
    dq = Eigen::Quaterniond(std::cos(half),
                            std::sin(half) * axis.x(),
                            std::sin(half) * axis.y(),
                            std::sin(half) * axis.z());
  } else {
    dq = Eigen::Quaterniond(1.0, 0.5*dtheta.x(), 0.5*dtheta.y(), 0.5*dtheta.z());
  }
  Eigen::Quaterniond qn = (q * dq).normalized();
  x[QW] = qn.w(); x[QX] = qn.x(); x[QY] = qn.y(); x[QZ] = qn.z();
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

  StateMatrix L;
  bool have_L = false;
  if (params_.semidefinite_sqrt) {
    // Exact square root of a positive semidefinite matrix: A = P^T L D L^T P, so
    // S = P^T L sqrt(D) satisfies S S^T = A. Any square root gives valid sigma points.
    const StateMatrix A = (n_aug_ + lambda_) * state_.P;
    Eigen::LDLT<StateMatrix> ldlt(A);
    const Eigen::Matrix<double, STATE_DIM, 1> d = ldlt.vectorD();
    const double dmax = std::max(d.cwiseAbs().maxCoeff(), 1e-300);
    if (ldlt.info() == Eigen::Success && d.minCoeff() >= -1e-12 * dmax) {
      StateMatrix Lm = ldlt.matrixL();
      StateMatrix S = ldlt.transpositionsP().transpose() *
                      (Lm * d.cwiseMax(0.0).cwiseSqrt().asDiagonal());
      L = S;
      have_L = true;
    }
    // A genuinely negative pivot: fall through to the regularise-and-repair chain.
  }

  if (!have_L) {
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
    L = llt.matrixL();
  }
  sigma.col(0) = state_.x;
  for (int i = 0; i < n_aug_; ++i) {
    if (params_.tangent_space_attitude) {
      // The attitude slots of L are a ROTATION VECTOR once P measures rotation,
      // so they have to be composed onto the quaternion, not added to it. Adding
      // them was the fourth and last place the two representations were mixed:
      // with the other three fixed and this one left additive, heading converged
      // but position diverged to 13 km with 45 of 120 fixes rejected, because the
      // sigma cloud no longer matched the covariance that generated it.
      StateVector plus  = state_.x + L.col(i);
      StateVector minus = state_.x - L.col(i);
      plus.segment<4>(QW)  = state_.x.segment<4>(QW);
      minus.segment<4>(QW) = state_.x.segment<4>(QW);
      const StateVector dplus  =  L.col(i);
      const StateVector dminus = -L.col(i);
      apply_attitude_correction(plus,  dplus);
      apply_attitude_correction(minus, dminus);
      sigma.col(i + 1)          = plus;
      sigma.col(i + 1 + n_aug_) = minus;
    } else {
      sigma.col(i + 1)          = state_.x + L.col(i);
      sigma.col(i + 1 + n_aug_) = state_.x - L.col(i);
    }
  }

  // Keep each sigma point's attitude a physically meaningful sample. See
  // UKFParams::max_sigma_rotation_deg for why, and for why this bounds the POINTS and
  // deliberately leaves P alone.
  // A bound at or past 180 degrees cannot bind, because a shortest-path rotation
  // never exceeds it, so the relaxed case costs nothing rather than doing busy work.
  const double scaled_rot_deg = params_.max_sigma_rotation_deg * sigma_rot_scale_;
  if (params_.max_sigma_rotation_deg > 0.0 && scaled_rot_deg < 180.0) {
    const double max_rad = scaled_rot_deg * M_PI / 180.0;
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

  // Weighted mean. The non-attitude states are a plain weighted sum either way; only
  // the quaternion is at issue.
  //
  // The plain sum plus renormalise is accurate ONLY while the sigma-point spread is
  // small. An earlier comment here claimed that was guaranteed at 100 Hz because dt is
  // tiny. That is wrong: the spread comes from P, not from dt, and P(QZ,QZ) grows
  // without bound because yaw is unobservable. See tangent_space_quaternion_mean.
  StateVector x_pred = StateVector::Zero();
  for (int i = 0; i < n_sigma; ++i)
    x_pred += Wm_[i] * sigma_pred.col(i);

  if (params_.tangent_space_quaternion_mean) {
    x_pred.segment<4>(QW) = quaternion_mean_tangent(sigma_pred, Wm_, n_sigma);
  }
  x_pred = normalize_state(x_pred);

  // Q is added per predict step (not scaled by dt).
  // The q_* params in UKFParams are calibrated as per-step noise, not spectral densities.
  // Scaling by dt here would require re-tuning all Q values by the IMU rate (~100x),
  // which would break all existing configurations and cause GPS Mahalanobis rejections
  // as P grows too slowly to track real motion.
  // Q is a per-step amount by default, or a per-second density scaled by dt when
  // q_time_scaled is set. See UKFParams::q_time_scaled for why the default is wrong
  // and why it is still the default.
  StateMatrix P_pred = params_.q_time_scaled ? (Q_ * dt).eval() : Q_;
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
    if (params_.tangent_space_attitude) attitude_difference(diff, sigma_pred.col(i), x_pred);
    P_pred += Wc_[i] * diff * diff.transpose();
  }
  if (params_.tangent_space_attitude) {
    // QW carries dq.w - 1, which is second order, so its variance collapses toward
    // zero and would make P singular in that direction. Floor it.
    constexpr double kScalarAttitudeVarFloor = 1e-6;
    P_pred(QW, QW) = std::max(P_pred(QW, QW), kScalarAttitudeVarFloor);
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
    StateVector x_diff = sigma.col(i) - state_.x;  // plain diff, no angle wrapping needed
    if (params_.tangent_space_attitude) attitude_difference(x_diff, sigma.col(i), state_.x);
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
  const StateVector correction = K * innovation;
  last_pos_correction_ = std::hypot(correction[X], correction[Y]);
  if (params_.tangent_space_attitude) {
    StateVector x_new = state_.x + correction;
    x_new.segment<4>(QW) = state_.x.segment<4>(QW);   // attitude handled multiplicatively
    apply_attitude_correction(x_new, correction);
    state_.x = normalize_state(x_new);
  } else {
    state_.x = normalize_state(state_.x + correction);
  }
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

// Iterative tangent-space (Karcher) mean of the sigma points' attitudes.
//
// Repeatedly: express each sample as a rotation vector relative to the current reference
// via the log map, take the weighted mean of those 3-vectors, and move the reference by
// that much via the exp map. Unlike a 4-vector sum there is nothing to cancel, so a wide
// spread degrades the answer gradually instead of flipping it.
//
// Converges in 2-3 passes for any realistic spread; the cap is a guard, not a budget.
Eigen::Vector4d UKF::quaternion_mean_tangent(const Eigen::MatrixXd & sigma_pred,
                                             const Eigen::VectorXd & Wm,
                                             int n_sigma) {
  Eigen::Quaterniond q_ref(sigma_pred(QW, 0), sigma_pred(QX, 0),
                           sigma_pred(QY, 0), sigma_pred(QZ, 0));
  if (q_ref.norm() < 1e-12) { return Eigen::Vector4d(1.0, 0.0, 0.0, 0.0); }
  q_ref.normalize();

  for (int iter = 0; iter < 8; ++iter) {
    Eigen::Vector3d e_bar = Eigen::Vector3d::Zero();
    double w_total = 0.0;
    for (int i = 0; i < n_sigma; ++i) {
      Eigen::Quaterniond qi(sigma_pred(QW, i), sigma_pred(QX, i),
                            sigma_pred(QY, i), sigma_pred(QZ, i));
      if (qi.norm() < 1e-12) { continue; }
      qi.normalize();
      Eigen::Quaterniond dq = q_ref.conjugate() * qi;
      if (dq.w() < 0.0) { dq.coeffs() *= -1.0; }      // short way round
      const double vn = dq.vec().norm();
      // log map: 2*atan2(|v|, w) * v/|v|, with the small-angle limit handled directly.
      Eigen::Vector3d e = Eigen::Vector3d::Zero();
      if (vn >= 1e-12) { e = (2.0 * std::atan2(vn, dq.w()) / vn) * dq.vec(); }
      e_bar += Wm[i] * e;
      w_total += Wm[i];
    }
    if (std::abs(w_total) > 1e-12) { e_bar /= w_total; }

    const double an = e_bar.norm();
    if (an < 1e-10) { break; }
    // exp map back onto the manifold.
    const double half = 0.5 * an;
    Eigen::Quaterniond dq(std::cos(half), 0, 0, 0);
    dq.vec() = (std::sin(half) / an) * e_bar;
    q_ref = q_ref * dq;
    q_ref.normalize();
    if (an < 1e-8) { break; }
  }
  return Eigen::Vector4d(q_ref.w(), q_ref.x(), q_ref.y(), q_ref.z());
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
