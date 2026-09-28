#pragma once

#include "fusioncore/state.hpp"
#include "fusioncore/motion_model.hpp"
#include <Eigen/Dense>
#include <functional>
#include <memory>

namespace fusioncore {

// UKF tuning parameters
struct UKFParams {
  // Sigma point spread: standard defaults, rarely need changing
  // Sigma-point spread. MUST stay at 1.0 unless you have re-measured; this is
  // not a free tuning knob at 23 states.
  //
  // The UKF mean is sum(W_i * sigma_i) with W[0] = lambda/(n+lambda) and
  // lambda = alpha^2*(n+kappa) - n. With kappa = 0 and n = 23, ANY alpha below 1
  // makes the CENTRE weight negative, and it gets extreme fast: alpha = 0.1, the
  // previous default, gives Wm[0] = -99.0 with the other 46 weights at +2.17.
  //
  // That is formally correct only for a tight sigma-point cluster. Yaw is
  // structurally unobservable without an absolute heading source, so the
  // quaternion sigma points spread wide, their forward displacements cancel each
  // other, and what survives is the centre point (the one pointing correctly
  // forward) multiplied by -99. The filter then drives BACKWARDS while reporting
  // a perfect velocity and a perfect heading.
  //
  // Measured with a perfect encoder at 1 m/s over 60 s (tools/repro/dr.cpp),
  // truth x = 60.00 m:
  //     alpha 0.1  Wm[0] -99.00   x = -114.06   <- previous default
  //     alpha 0.5  Wm[0]  -3.00   x =    8.83
  //     alpha 1.0  Wm[0]   0.00   x =   48.43
  // And on NCLT 2013-04-05 at 1x playback: 5268.80 m -> 131.85 m ATE, a 97.5%
  // reduction, with robot_localization unchanged at ~230 m as the control.
  //
  // alpha = 1.0 gives lambda = 0 and all 47 weights non-negative: the standard
  // unscaled UKF. Lowering alpha to "tighten" the sigma points does the opposite
  // of what it does in a low-dimensional filter.
  double alpha = 1.0;
  double beta  = 2.0;    // prior knowledge of distribution (2.0 = Gaussian)
  double kappa = 0.0;    // secondary scaling (0.0 is standard)

  // Largest rotation, in degrees, that a single sigma point may represent relative to
  // the mean attitude. 0 disables the limit and is the default.
  //
  // Why this exists (#150). Yaw is unobservable without an absolute heading source, so
  // P(QZ,QZ) grows without bound: measured at 177.3 after 60 s on a perfect encoder,
  // where a quaternion component lives in [-1,1] and its variance therefore cannot
  // exceed 1. The sigma points are formed as x +/- L.col(i) with the quaternion
  // perturbed ADDITIVELY and not renormalised, so a perturbation of sqrt(23*177) ~= 64
  // on qz makes (1,0,0,64), which normalises to a rotation of about 178 degrees. The
  // sigma points sit nearly antipodal to the mean, their forward displacements cancel,
  // and position advances at a fraction of a perfect velocity: 87% at t=5 s decaying to
  // 38% by t=150 s while velocity and yaw both read perfect.
  //
  // This bounds the SAMPLED POINTS, not P. That distinction is the whole point: four
  // previous attempts modified P (diagonal cap at 0.25, congruence cap at 0.05, Q
  // scaling, Markley mean) and all were reverted, because scaling a row and column
  // destroys the cross-covariance a measurement needs to correct yaw. P is left exactly
  // as it is here; only the points drawn from it are kept physical.
  //
  // A sigma point representing a 178 degree rotation is not a sample of the attitude
  // distribution, it is an artefact of representing an angle in a linear covariance.
  //
  // IT WORKS ON DEAD RECKONING AND FAILS ON REAL DATA. Keep it off. This is the SIXTH
  // attempt on #150 and it is recorded here so there is not a seventh.
  //
  // On the synthetic (tools/repro/dr.cpp, perfect encoder, still IMU, no GPS) it is a
  // complete fix, and it holds at every horizon where the unlimited filter collapses:
  //
  //     t (s)      30     60     90    120    150    300
  //     off      81.5%  80.7%  61.6%  47.1%  37.6%   7.7%     of a perfect velocity
  //     45 deg   98.6%  98.6%  98.6%  98.6%  98.6%  98.6%
  //
  // and P(QZ,QZ) comes back inside its constraint, 177.3 to 0.0047.
  //
  // On NCLT 2012-08-20 replayed through the core it is much WORSE: 135.3 m with the
  // limit off against 233.8 m at 90, 45 or 20 degrees, and 273.1 m at 10.
  //
  // The two results together are the useful part. Bounding the sampled rotations does
  // fix the cancellation, so the mechanism is confirmed. But yaw genuinely IS
  // unobservable, and a filter whose attitude covariance can no longer grow becomes
  // overconfident and under-weights the corrections it needs. The synthetic has no GPS,
  // so the overconfidence costs nothing there and the benefit shows pure.
  //
  // So the requirement is sharper than "bound the spread": the representation must let
  // yaw uncertainty be LARGE while every sampled attitude stays a valid rotation. A
  // linear covariance on a 4-vector with a norm constraint cannot do both. An error-state
  // formulation can, because the uncertainty is a 3-vector in the tangent space and every
  // sample is a rotation by construction. That is the direction #150 should take.
  double max_sigma_rotation_deg = 0.0;

  // Average the sigma points' attitudes in the TANGENT SPACE instead of summing the
  // 4-vectors and renormalising.
  //
  // WHY: the plain sum is valid only while the spread is small, and the spread is set by
  // P, not by dt. P(QZ,QZ) grows without bound because yaw is unobservable, so on a
  // straight dead-reckoning run the sigma points end up scattered over many full
  // rotations by t=77 s, their weighted sum nearly cancels, and normalising that residue
  // yields an arbitrary direction. Measured: yaw flips 0.001 to -179.721 deg in ONE step
  // while |q| stays exactly 1.0, position reverses, and the filter never recovers.
  //
  // DO NOT ENABLE THIS ON ITS OWN. It is half a migration and the half it is missing
  // matters. Measured on the dead-reckoning run, perfect encoder 1 m/s, still IMU:
  //
  //     t          25      50     100     200     300     400      (truth x = t)
  //     off      20.45   40.46   52.99   56.17   23.08   19.57
  //     ON       30.51   55.06   99.38  186.75  259.09  275.30
  //
  // So it does remove the catastrophic failure: sigma(QZ) stays 0.11-0.47 instead of
  // reaching 15.69, there is no 180 degree flip, and the filter is still advancing at
  // t=400 where the plain sum has been wandering backwards for five minutes.
  //
  // But it OVERSHOOTS by 22% early (30.51 m at t=25) and accelerometer estimates
  // oscillate +-5 m/s^2 on a still sensor where the plain sum reads 0.0000. The cause is
  // that predict() still measures the covariance residual as a plain Euclidean
  // difference sigma_pred.col(i) - x_pred. Move the mean onto the manifold and those
  // residuals no longer sum to zero, so P picks up a spurious term that leaks through the
  // quaternion/acceleration cross-covariance into the IMU update.
  //
  // Gating it on a large spread does not rescue it either: sigma(QZ) passes 1.0 within
  // 5 seconds, so the gate would fire almost immediately and the inconsistent path would
  // BE the normal path.
  //
  // The conclusion, which is the useful part: the mean and the residual have to move to
  // the tangent space together, and the residual cannot until the state carries attitude
  // error as a 3-vector. That is the full error-state rewrite (#150), and this flag is
  // the evidence that the cheap half of it is not a shortcut. Kept, off, for that reason.
  bool tangent_space_quaternion_mean = false;

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

  // Overwrite the orientation quaternion, leaving every other state and the
  // whole covariance alone. For establishing an attitude the filter never had a
  // measurement of, as opposed to correcting one it did: see
  // FusionCore::update_magnetometer, where yaw starts as a fabricated zero and
  // no amount of fusing can move it, because the gain against P(QZ,QZ) = 1e-8 is
  // nil. Not a substitute for an update, and it should be rare.
  void set_orientation(double qw, double qx, double qy, double qz) {
    const double n = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (n < 1e-12) return;
    state_.x[QW] = qw / n;
    state_.x[QX] = qx / n;
    state_.x[QY] = qy / n;
    state_.x[QZ] = qz / n;
  }

  bool is_initialized() const { return initialized_; }

  // Scale applied to position diagonal of Q during inertial coast mode.
  // 1.0 = normal operation; > 1.0 = inflated position uncertainty.
  void set_position_noise_scale(double s) { pos_noise_scale_ = s; }

  // Diagnostic: how far the LAST measurement update moved the position estimate,
  // in metres, i.e. the norm of the X/Y rows of K * innovation.
  //
  // Position should only move by integrating velocity. Anything an update adds
  // is the P position/velocity cross-covariance dragging it sideways. On the
  // 2026-08-03 field bag those corrections summed to more than the robot actually
  // travelled (161 m of correction against 127 m of real motion) while the
  // integrated velocity was accurate to 0.4 percent. Exposing it per update is
  // what makes that attributable to a specific sensor instead of guessable.
  double last_position_correction() const { return last_pos_correction_; }
  double position_noise_scale() const     { return pos_noise_scale_; }

  // Scale applied to gyro bias diagonal of Q during GPS coast mode.
  // Inflating this loosens the filter's confidence in its bias estimate so that
  // encoder WZ measurements can drive fast bias correction during GPS outages.
  // 1.0 = normal (tight); 100.0 = fast adaptation.
  void set_gyro_bias_noise_scale(double s) { gyro_bias_noise_scale_ = s; }
  double gyro_bias_noise_scale() const     { return gyro_bias_noise_scale_; }

  // Inflate P[X,X] and P[Y,Y] to at least sigma_xy_sq using max().
  // Cross-covariances are untouched so the UKF update handles the correction.
  // Raise the position covariance floor so a measurement the filter currently
  // finds impossible can be reconsidered. Z is included because the GNSS chi2
  // gate is three dimensional: inflating only X and Y leaves the vertical term
  // untouched, so a large altitude error alone can hold the gate shut no matter
  // how far the horizontal covariance is opened, and the filter then rejects
  // every fix forever while appearing to be trying. Pass 0 for sigma_z_sq to
  // leave the vertical alone.
  void inflate_position_covariance(double sigma_xy_sq, double sigma_z_sq = 0.0) {
    state_.P(X, X) = std::max(state_.P(X, X), sigma_xy_sq);
    state_.P(Y, Y) = std::max(state_.P(Y, Y), sigma_xy_sq);
    if (sigma_z_sq > 0.0)
      state_.P(Z, Z) = std::max(state_.P(Z, Z), sigma_z_sq);
  }

  // Replace the default motion model (ConstantVelocityAcceleration).
  // Call before the first predict() step.
  void set_motion_model(std::shared_ptr<MotionModelBase> model) {
    if (model) motion_model_ = std::move(model);
  }

private:
  UKFParams params_;
  double last_pos_correction_ = 0.0;
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
  static Eigen::Vector4d quaternion_mean_tangent(const Eigen::MatrixXd & sigma_pred,
                                                 const Eigen::VectorXd & Wm,
                                                 int n_sigma);
};

} // namespace fusioncore
