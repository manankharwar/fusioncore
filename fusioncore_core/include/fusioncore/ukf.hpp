#pragma once

#include "fusioncore/state.hpp"
#include "fusioncore/motion_model.hpp"
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <memory>

namespace fusioncore {

// UKF tuning parameters
struct UKFParams {
  // Sigma point spread: standard defaults, rarely need changing
  double alpha = 0.1;    // spread of sigma points around mean (1e-4 to 1.0)
  double beta  = 2.0;    // prior knowledge of distribution (2.0 = Gaussian)
  double kappa = 0.0;    // secondary scaling (0.0 is standard)

  // Process noise: how much we trust the motion model
  double q_position     = 0.01;   // m²/step
  // Quaternion regularization: keeps Q positive-definite.
  // NOT the primary orientation noise source: orientation uncertainty
  // propagates from q_angular_vel through the quaternion kinematics.
  // Set to a very small value; large values corrupt the quaternion norm.
  double q_orientation  = 1e-9;   // quaternion regularization (dimensionless)
  double q_velocity     = 0.1;    // (m/s)²/step
  double q_angular_vel  = 0.1;    // (rad/s)²/step
  double q_acceleration = 1.0;    // (m/s²)²/step
  double q_gyro_bias         = 1e-5;   // (rad/s)²/step -- biases change slowly
  double q_accel_bias        = 1e-5;   // (m/s²)²/step
  double q_encoder_wz_bias   = 1e-7;   // (rad/s)²/step -- encoder WZ bias; mechanical, very stable
};

class UKF {
public:
  explicit UKF(const UKFParams& params = UKFParams{});

  // Initialize state
  void init(const State& initial_state);

  // Predict step: propagate state forward by dt seconds
  void predict(double dt);

  // Update step: fuse a measurement
  // Returns innovation vector (z - z_pred) for adaptive noise tracking
  // angle_dims: bitmask of measurement dimensions that are angles and need
  //             wrapping to [-pi, pi] during innovation computation.
  //             e.g., 0b100 means dimension 2 (zero-indexed) is an angle.
  //             Pass 0 (default) for no angle wrapping.
  template <int z_dim>
  Eigen::Matrix<double, z_dim, 1> update(
    const Eigen::Matrix<double, z_dim, 1>& z,
    const std::function<Eigen::Matrix<double, z_dim, 1>(const StateVector&)>& h,
    const Eigen::Matrix<double, z_dim, z_dim>& R,
    unsigned int angle_dims = 0
  );

  // Update step with selected state rows blocked from the Kalman correction.
  // Mask value true = state element may be corrected; false = leave it unchanged.
  template <int z_dim>
  Eigen::Matrix<double, z_dim, 1> update_masked(
    const Eigen::Matrix<double, z_dim, 1>& z,
    const std::function<Eigen::Matrix<double, z_dim, 1>(const StateVector&)>& h,
    const Eigen::Matrix<double, z_dim, z_dim>& R,
    const std::array<bool, STATE_DIM>& state_update_mask,
    unsigned int angle_dims = 0
  );

  // Predict measurement without updating state.
  // Returns innovation and innovation covariance S for Mahalanobis test.
  // Call this BEFORE update() to check if measurement should be rejected.
  // angle_dims: same bitmask as update(): wrap angle dimensions in z_diff.
  template <int z_dim>
  void predict_measurement(
    const Eigen::Matrix<double, z_dim, 1>& z,
    const std::function<Eigen::Matrix<double, z_dim, 1>(const StateVector&)>& h,
    const Eigen::Matrix<double, z_dim, z_dim>& R,
    Eigen::Matrix<double, z_dim, 1>& innovation_out,
    Eigen::Matrix<double, z_dim, z_dim>& S_out,
    unsigned int angle_dims = 0
  );

  // Get current state estimate
  const State& state() const { return state_; }

  bool is_initialized() const { return initialized_; }

  // Scale applied to position diagonal of Q during inertial coast mode.
  // 1.0 = normal operation; > 1.0 = inflated position uncertainty.
  void set_position_noise_scale(double s) { pos_noise_scale_ = s; }
  double position_noise_scale() const     { return pos_noise_scale_; }

  // Scale applied to gyro bias diagonal of Q during GPS coast mode.
  // Inflating this loosens the filter's confidence in its bias estimate so that
  // encoder WZ measurements can drive fast bias correction during GPS outages.
  // 1.0 = normal (tight); 100.0 = fast adaptation.
  void set_gyro_bias_noise_scale(double s) { gyro_bias_noise_scale_ = s; }
  double gyro_bias_noise_scale() const     { return gyro_bias_noise_scale_; }

  // Inflate P[X,X] and P[Y,Y] to at least sigma_xy_sq using max().
  // Cross-covariances are untouched so the UKF update handles the correction.
  void inflate_position_covariance(double sigma_xy_sq) {
    state_.P(X, X) = std::max(state_.P(X, X), sigma_xy_sq);
    state_.P(Y, Y) = std::max(state_.P(Y, Y), sigma_xy_sq);
  }

  // Replace the default motion model (ConstantVelocityAcceleration).
  // Call before the first predict() step.
  void set_motion_model(std::shared_ptr<MotionModelBase> model) {
    if (model) motion_model_ = std::move(model);
  }

private:
  UKFParams params_;
  State state_;
  bool   initialized_           = false;
  double pos_noise_scale_       = 1.0;
  double gyro_bias_noise_scale_ = 1.0;
  std::shared_ptr<MotionModelBase> motion_model_;

  // UKF weights
  int n_aug_;          // augmented state dimension
  double lambda_;      // scaling parameter
  Eigen::VectorXd Wm_; // weights for mean
  Eigen::VectorXd Wc_; // weights for covariance

  // Process noise matrix
  StateMatrix Q_;

  void compute_weights();
  void build_process_noise();

  // Generate 2n+1 sigma points from current state.
  // Repairs state_.P in-place if it has lost positive-definiteness.
  Eigen::MatrixXd generate_sigma_points();

  // Normalize angle to [-pi, pi]
  static double normalize_angle(double angle);

  // Normalize angle components of state vector
  static StateVector normalize_state(const StateVector& x);
};

} // namespace fusioncore
