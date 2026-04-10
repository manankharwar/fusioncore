#pragma once

#include "fusioncore/state.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace fusioncore {
namespace sensors {

// ─── GNSS position measurement (3-dimensional) ──────────────────────────────

constexpr int GNSS_POS_DIM = 3;

using GnssPosMeasurement = Eigen::Matrix<double, GNSS_POS_DIM, 1>;
using GnssPosNoiseMatrix = Eigen::Matrix<double, GNSS_POS_DIM, GNSS_POS_DIM>;

// ─── GNSS heading measurement (1-dimensional) ───────────────────────────────

constexpr int GNSS_HDG_DIM = 1;

using GnssHdgMeasurement = Eigen::Matrix<double, GNSS_HDG_DIM, 1>;
using GnssHdgNoiseMatrix = Eigen::Matrix<double, GNSS_HDG_DIM, GNSS_HDG_DIM>;

// ─── GNSS antenna lever arm ──────────────────────────────────────────────────
// The offset from base_link to the GNSS antenna, in the robot body frame.
// peci1 fix: if the antenna is not at base_link, its readings correspond
// to a different point than base_link. Ignoring this causes position errors
// proportional to the lever arm length times the rotation rate.
//
// How to measure: use a tape measure from your robot's base_link origin
// (usually center of the rear axle or geometric center) to the antenna.
// x = forward, y = left, z = up in body frame.
//
// Example for Husarion Panther with antenna on top:
//   lever_arm_x = 0.0   (centered fore-aft)
//   lever_arm_y = 0.0   (centered left-right)
//   lever_arm_z = 0.5   (0.5m above base_link)

struct GnssLeverArm {
  double x = 0.0;  // meters, body frame forward
  double y = 0.0;  // meters, body frame left
  double z = 0.0;  // meters, body frame up

  bool is_zero() const {
    return std::abs(x) < 1e-6 && std::abs(y) < 1e-6 && std::abs(z) < 1e-6;
  }
};

// ─── GNSS quality parameters ─────────────────────────────────────────────────

struct GnssParams {
  double base_noise_xy = 1.0;
  double base_noise_z  = 2.0;
  double heading_noise = 0.02;
  double max_hdop      = 4.0;
  double max_vdop      = 6.0;
  int    min_satellites = 4;

  // Antenna offset from base_link in body frame
  GnssLeverArm lever_arm;
};

// ─── GNSS fix ────────────────────────────────────────────────────────────────

enum class GnssFixType {
  NO_FIX    = 0,
  GPS_FIX   = 1,
  DGPS_FIX  = 2,
  RTK_FLOAT = 3,
  RTK_FIXED = 4
};

struct GnssFix {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double hdop = 99.0;
  double vdop = 99.0;

  int         satellites = 0;
  GnssFixType fix_type   = GnssFixType::NO_FIX;

  // Source identifier — used when fusing multiple GNSS receivers.
  // 0 = primary, 1 = secondary, etc.
  int source_id = 0;

  // Per-measurement lever arm (antenna offset from base_link in body frame).
  // Set by the ROS node based on which receiver produced this fix.
  // Defaults to zero — no correction applied.
  GnssLeverArm lever_arm;

  // Full 3x3 position covariance matrix (row-major, ENU frame).
  // peci1 fix: real GNSS covariance often has off-diagonal elements
  // (correlated X/Y errors). When has_full_covariance is true, this
  // matrix is used directly instead of the diagonal HDOP/VDOP estimate.
  bool has_full_covariance = false;
  Eigen::Matrix3d full_covariance = Eigen::Matrix3d::Identity();

  bool is_valid(const GnssParams& p) const {
    return fix_type != GnssFixType::NO_FIX
        && hdop <= p.max_hdop
        && vdop <= p.max_vdop
        && satellites >= p.min_satellites;
  }
};

struct GnssHeading {
  double heading_rad  = 0.0;
  double accuracy_rad = 0.1;
  bool   valid        = false;

  // Source identifier — matches the source_id of the GnssFix
  // from the same receiver
  int source_id = 0;
};

// ─── Measurement functions ───────────────────────────────────────────────────

// h(x): state -> expected GNSS position at base_link (no lever arm)
inline GnssPosMeasurement gnss_pos_measurement_function(const StateVector& x) {
  GnssPosMeasurement z;
  z[0] = x[X];
  z[1] = x[Y];
  z[2] = x[Z];
  return z;
}

// h(x): state -> expected GNSS position accounting for antenna lever arm
// This is the correct version when the antenna is not at base_link.
//
// The antenna position in world frame is:
//   p_antenna = p_base + R_body_to_world * lever_arm
//
// where R_body_to_world is the rotation matrix from current roll/pitch/yaw.
//
// This function is returned as a lambda so it captures the lever arm.
inline auto gnss_pos_measurement_function_with_lever_arm(
  const GnssLeverArm& lever_arm)
{
  return [lever_arm](const StateVector& x) -> GnssPosMeasurement {
    double roll  = x[ROLL];
    double pitch = x[PITCH];
    double yaw   = x[YAW];

    // Build rotation matrix body -> world (ZYX Euler)
    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    double R00 = cy*cp;
    double R01 = cy*sp*sr - sy*cr;
    double R02 = cy*sp*cr + sy*sr;
    double R10 = sy*cp;
    double R11 = sy*sp*sr + cy*cr;
    double R12 = sy*sp*cr - cy*sr;
    double R20 = -sp;
    double R21 = cp*sr;
    double R22 = cp*cr;

    // Rotate lever arm from body frame to world frame
    double lx = lever_arm.x;
    double ly = lever_arm.y;
    double lz = lever_arm.z;

    double offset_x = R00*lx + R01*ly + R02*lz;
    double offset_y = R10*lx + R11*ly + R12*lz;
    double offset_z = R20*lx + R21*ly + R22*lz;

    GnssPosMeasurement z;
    z[0] = x[X] + offset_x;
    z[1] = x[Y] + offset_y;
    z[2] = x[Z] + offset_z;
    return z;
  };
}

// h(x): state -> expected GNSS heading
inline GnssHdgMeasurement gnss_hdg_measurement_function(const StateVector& x) {
  GnssHdgMeasurement z;
  z[0] = x[YAW];
  return z;
}

// ─── Noise matrices ───────────────────────────────────────────────────────────

inline GnssPosNoiseMatrix gnss_pos_noise_matrix(
  const GnssParams& p,
  const GnssFix& fix)
{
  // peci1 fix: use full covariance matrix when available.
  // Real GNSS receivers often report correlated X/Y errors —
  // the off-diagonal elements matter, especially with RTK.
  if (fix.has_full_covariance) {
    // Validate — all diagonal elements must be positive
    if (fix.full_covariance(0,0) > 0.0 &&
        fix.full_covariance(1,1) > 0.0 &&
        fix.full_covariance(2,2) > 0.0) {
      return fix.full_covariance;
    }
  }

  // Fall back to diagonal estimate from HDOP/VDOP
  GnssPosNoiseMatrix R = GnssPosNoiseMatrix::Zero();
  double sigma_xy = p.base_noise_xy * fix.hdop;
  double sigma_z  = p.base_noise_z  * fix.vdop;
  R(0,0) = sigma_xy * sigma_xy;
  R(1,1) = sigma_xy * sigma_xy;
  R(2,2) = sigma_z  * sigma_z;
  return R;
}

inline GnssHdgNoiseMatrix gnss_hdg_noise_matrix(
  const GnssParams& p,
  const GnssHeading& hdg)
{
  GnssHdgNoiseMatrix R;
  double sigma = std::max(p.heading_noise, hdg.accuracy_rad);
  R(0,0) = sigma * sigma;
  return R;
}

// ─── ECEF / ENU conversion ────────────────────────────────────────────────────

struct ECEFPoint {
  double x, y, z;
};

struct LLAPoint {
  double lat_rad;
  double lon_rad;
  double alt_m;
};

inline Eigen::Vector3d ecef_to_enu(
  const ECEFPoint& point,
  const ECEFPoint& ref,
  const LLAPoint&  ref_lla)
{
  double dx = point.x - ref.x;
  double dy = point.y - ref.y;
  double dz = point.z - ref.z;

  double sin_lat = std::sin(ref_lla.lat_rad);
  double cos_lat = std::cos(ref_lla.lat_rad);
  double sin_lon = std::sin(ref_lla.lon_rad);
  double cos_lon = std::cos(ref_lla.lon_rad);

  double e = -sin_lon*dx           + cos_lon*dy;
  double n = -sin_lat*cos_lon*dx   - sin_lat*sin_lon*dy + cos_lat*dz;
  double u =  cos_lat*cos_lon*dx   + cos_lat*sin_lon*dy + sin_lat*dz;

  return Eigen::Vector3d(e, n, u);
}

} // namespace sensors
} // namespace fusioncore
