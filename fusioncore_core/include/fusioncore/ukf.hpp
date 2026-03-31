#pragma once

#include "fusioncore/state.hpp"
#include <Eigen/Dense>
#include <functional>

namespace fusioncore {

// UKF tuning parameters
struct UKFParams {
  // Sigma point spread — standard defaults, rarely need changing
  double alpha = 0.1;    // spread of sigma points around mean (1e-4 to 1.0)
  double beta  = 2.0;    // prior knowledge of distribution (2.0 = Gaussian)
  double kappa = 0.0;    // secondary scaling (0.0 is standard)

  // Process noise — how much we trust the motion model
  double q_position     = 0.01;   // m²/s
  double q_orientation  = 0.01;   // rad²/s
  double q_velocity     = 0.1;    // (m/s)²/s
  double q_angular_vel  = 0.1;    // (rad/s)²/s
  double q_acceleration = 1.0;    // (m/s²)²/s
  double q_gyro_bias    = 1e-5;   // (rad/s)²/s  -- biases change slowly
  double q_accel_bias   = 1e-5;   // (m/s²)²/s
};

class UKF {
public:
  explicit UKF(const UKFParams& params = UKFParams{});

  // Initialize state
  void init(const State& initial_state);

  // Predict step — propagate state forward by dt seconds
  void predict(double dt);

  // Update step — fuse a measurement
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

  // Predict measurement without updating state.
  // Returns innovation and innovation covariance S for Mahalanobis test.
  // Call this BEFORE update() to check if measurement should be rejected.
  // angle_dims: same bitmask as update() — wrap angle dimensions in z_diff.
  template <int z_dim>
  void predict_measurement(
    const Eigen::Matrix<double, z_dim, 1>& z,
    const std::function<Eigen::Matrix<double, z_dim, 1>(const StateVector&)>& h,
    const Eigen::Matrix<double, z_dim, z_dim>& R,
    Eigen::Matrix<double, z_dim, 1>& innovation_out,
    Eigen::Matrix<double, z_dim, z_dim>& S_out,
    unsigned int angle_dims = 0
  ) const;

  // Get current state estimate
  const State& state() const { return state_; }

  bool is_initialized() const { return initialized_; }

private:
  UKFParams params_;
  State state_;
  bool initialized_ = false;

  // UKF weights
  int n_aug_;          // augmented state dimension
  double lambda_;      // scaling parameter
  Eigen::VectorXd Wm_; // weights for mean
  Eigen::VectorXd Wc_; // weights for covariance

  // Process noise matrix
  StateMatrix Q_;

  void compute_weights();
  void build_process_noise();

  // Generate 2n+1 sigma points from current state
  Eigen::MatrixXd generate_sigma_points() const;

  // Motion model f(x, dt): propagates a single sigma point
  StateVector process_model(const StateVector& x, double dt) const;

  // Normalize angle to [-pi, pi]
  static double normalize_angle(double angle);

  // Normalize angle components of state vector
  static StateVector normalize_state(const StateVector& x);
};

} // namespace fusioncore
