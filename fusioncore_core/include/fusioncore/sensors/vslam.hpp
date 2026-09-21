#pragma once

#include "fusioncore/state.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace fusioncore {
namespace sensors {

// ─── VSLAM pose measurement (6-dimensional: position + orientation) ──────────

constexpr int VSLAM_POSE_DIM = 6;  // [x, y, z, roll, pitch, yaw]

using VslamPoseMeasurement = Eigen::Matrix<double, VSLAM_POSE_DIM, 1>;
using VslamPoseNoiseMatrix = Eigen::Matrix<double, VSLAM_POSE_DIM, VSLAM_POSE_DIM>;

// ─── VSLAM quality parameters ─────────────────────────────────────────────────

struct VslamParams {
  double position_noise    = 0.1;    // m:   fallback when message has no covariance
  double orientation_noise = 0.02;   // rad: fallback when message has no covariance
};

// ─── VSLAM pose ───────────────────────────────────────────────────────────────

struct VslamPose {
  double x = 0.0, y = 0.0, z = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;

  // Per-message position covariance (3x3, diagonal used).
  // Extracted from nav_msgs/Odometry pose.covariance top-left 3x3.
  bool            has_position_cov = false;
  Eigen::Matrix3d position_cov     = Eigen::Matrix3d::Identity();

  // Per-message orientation covariance (3x3, diagonal used).
  // Extracted from nav_msgs/Odometry pose.covariance bottom-right 3x3.
  bool            has_orientation_cov = false;
  Eigen::Matrix3d orientation_cov     = Eigen::Matrix3d::Identity();

  // Same guard as GnssFix::is_finite(), for the same reason: a NaN pose passes
  // the chi2 gate instead of failing it, and a NaN reaching the state is
  // unrecoverable. The covariance is checked only where it is used: the
  // diagonal, and only when the message supplied one.
  bool is_finite() const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
           std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw) &&
           (!has_position_cov || position_cov.diagonal().allFinite()) &&
           (!has_orientation_cov || orientation_cov.diagonal().allFinite());
  }
};

// ─── Measurement function: h(x) = [X, Y, Z, roll, pitch, yaw] ───────────────

inline VslamPoseMeasurement vslam_pose_measurement_function(const StateVector& x)
{
  VslamPoseMeasurement z;
  z[0] = x[X];
  z[1] = x[Y];
  z[2] = x[Z];
  double roll, pitch, yaw;
  quat_to_euler(x[QW], x[QX], x[QY], x[QZ], roll, pitch, yaw);
  z[3] = roll;
  z[4] = pitch;
  z[5] = yaw;
  return z;
}

// ─── Noise matrix ─────────────────────────────────────────────────────────────

inline VslamPoseNoiseMatrix vslam_pose_noise_matrix(
  const VslamParams& p,
  const VslamPose&   pose)
{
  VslamPoseNoiseMatrix R = VslamPoseNoiseMatrix::Zero();

  // Position block
  constexpr double kMinVarPos = 1e-4;   // σ = 1 cm floor
  if (pose.has_position_cov &&
      pose.position_cov(0,0) > 0.0 &&
      pose.position_cov(1,1) > 0.0 &&
      pose.position_cov(2,2) > 0.0) {
    R(0,0) = std::max(pose.position_cov(0,0), kMinVarPos);
    R(1,1) = std::max(pose.position_cov(1,1), kMinVarPos);
    R(2,2) = std::max(pose.position_cov(2,2), kMinVarPos);
  } else {
    double var = p.position_noise * p.position_noise;
    R(0,0) = var;
    R(1,1) = var;
    R(2,2) = var;
  }

  // Orientation block
  constexpr double kMinVarOrient = 1e-6;  // σ = 0.001 rad floor
  if (pose.has_orientation_cov &&
      pose.orientation_cov(0,0) > 0.0 &&
      pose.orientation_cov(1,1) > 0.0 &&
      pose.orientation_cov(2,2) > 0.0) {
    R(3,3) = std::max(pose.orientation_cov(0,0), kMinVarOrient);
    R(4,4) = std::max(pose.orientation_cov(1,1), kMinVarOrient);
    R(5,5) = std::max(pose.orientation_cov(2,2), kMinVarOrient);
  } else {
    double var = p.orientation_noise * p.orientation_noise;
    R(3,3) = var;
    R(4,4) = var;
    R(5,5) = var;
  }

  return R;
}

} // namespace sensors
} // namespace fusioncore
