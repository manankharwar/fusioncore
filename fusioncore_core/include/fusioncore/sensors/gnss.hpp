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

// ─── GNSS fix type ───────────────────────────────────────────────────────────

enum class GnssFixType {
  NO_FIX    = 0,
  GPS_FIX   = 1,
  DGPS_FIX  = 2,
  RTK_FLOAT = 3,
  RTK_FIXED = 4
};

// ─── GNSS quality parameters ─────────────────────────────────────────────────

struct GnssParams {
  // Short-term consistency of the receiver, in METRES, used ONLY by the chi2
  // outlier gate. Zero (the default) keeps the old behaviour of gating on the
  // same R the update uses.
  //
  // Those are two different quantities and conflating them is why the gate does
  // not fire. A receiver's reported covariance describes ABSOLUTE accuracy, the
  // multipath and ionospheric error that moves slowly; consecutive fixes are far
  // more consistent than that number implies. Measured on the 2026-09-06 rover
  // log: 3.24 m declared, 0.171 m median second difference, a factor of 55.
  //
  // The update wants the absolute figure, or the filter will believe GPS to
  // centimetres it has not earned and track that slow bias rigidly. The gate
  // wants the short-term figure, because an outlier IS a break in short-term
  // consistency. With the absolute figure in both places, S is so large that no
  // realistic spike looks surprising: on that log a spike had to exceed 29 m
  // before it was rejected, and an accepted 15 m spike moved position 4.5 m.
  //
  // Estimate it from a bag with tools/nis_from_bag.py, which reports the
  // fix-to-fix second difference for exactly this purpose. Set it a little
  // generously: too small and ordinary noise gets rejected, which is the failure
  // mode that has bitten this project repeatedly.
  double outlier_sigma_xy = 0.0;

  // Largest plausible SECOND DIFFERENCE between consecutive fixes, in METRES.
  // Zero (the default) disables the check.
  //
  // This exists because the chi2 gate structurally cannot see a metre-scale
  // spike. That gate tests a fix against the FILTER, so its scale is
  // S = H P H' + R, tens of square metres for a consumer receiver, and on the
  // 2026-09-06 rover log a spike had to exceed 29 m before it was rejected while
  // an accepted 15 m spike moved position 4.5 m. Neither a better gate R nor a
  // perfect heading fixes that: heading actually made it slightly worse.
  //
  // Continuity asks a different question, one that never touches P: does this fix
  // agree with where the last few put it? A least-squares line through the last
  // five accepted fixes predicts the next one, and a spike breaks that badly.
  // Measured on the same log, the median residual of a good fix is 0.171 m, so a
  // 10 m spike is roughly 58 times the normal scale. Invisible to chi2,
  // unmissable here.
  //
  // WHY FIVE POINTS AND NOT TWO. A two-point extrapolation puts an error in the
  // newest reference point into the prediction multiplied by about two, so a
  // spike small enough to pass this limit threw the NEXT good fix over it: the
  // gate kept the bad sample and rejected the good one, which is worse than not
  // gating. Over five points the weight on the newest is 0.8, so a spike that
  // passes can only move the next prediction by 0.8 of itself and can never
  // reach the limit, whatever the limit is set to.
  //
  // HOW TO SET IT. Above the p99 of the residual your own receiver produces, not
  // by feel. The cost of being wrong is rejecting good fixes, which is the
  // failure that has cost this project most, and an accepted spike of a few
  // metres does very little damage anyway: measured on the 2026-09-07 log, an
  // accepted 3 m spike moved the trajectory 0.25 m while an accepted 60 m spike
  // moved it 15.73 m. The gate earns its keep in the tail, so set it to catch
  // the tail and leave ordinary noise alone.
  //
  // Measured across 1287 fixes from six 2026-09 rover logs: median residual 0.18
  // to 0.59 m, p99 0.95 to 3.00 m, largest 3.81 m. At 4.0 there were no
  // rejections at all on clean data and every injected spike from 4 m up was
  // caught, so 4.0 is the right number FOR THAT RECEIVER. Measure yours.
  double continuity_max_m = 0.0;

  double base_noise_xy = 1.0;
  double base_noise_z  = 2.0;
  double heading_noise = 0.02;

  // Quality gate on receiver-reported DOP. These are DIMENSIONLESS geometry
  // factors and only apply when the receiver actually publishes DOP, which in
  // practice means gps_msgs/GPSFix with no covariance. A NavSatFix carries no
  // DOP at all, so for that input the sigma gate below is what runs.
  double max_hdop      = 4.0;
  double max_vdop      = 6.0;

  // Quality gate on reported position uncertainty, in METRES of one sigma.
  // Used whenever the fix carries a covariance, which is every NavSatFix with
  // position_covariance_type >= 1, so this is the gate most users actually hit.
  //
  // These are deliberately loose. A gate in absolute metres cannot tell a
  // genuinely broken fix from ordinary noise unless it sits well outside the
  // receiver's normal spread, and the chi-squared test is the real outlier
  // defence. Field-measured 2026-08: a standalone u-blox M9N reports 2 to 8 m
  // horizontal sigma in the open and worse near buildings, all of it usable.
  // Values that merely look strict (4 m, 6 m) silently veto working GPS and
  // leave the filter dead-reckoning with no error surfaced anywhere.
  double max_sigma_xy  = 25.0;
  double max_sigma_z   = 50.0;

  int    min_satellites = 4;

  // Minimum fix type required for fusion (default: any fix accepted).
  // Set to RTK_FLOAT or RTK_FIXED to reject non-RTK fixes.
  GnssFixType min_fix_type = GnssFixType::GPS_FIX;

  // Antenna offset from base_link in body frame
  GnssLeverArm lever_arm;

  // Normally the lever arm is only applied after heading_validated_ flips
  // true (dock compass, dual-GNSS, or 5 m of straight GPS track). Setting
  // this to true applies the lever arm from the very first fix, which lets
  // GPS position innovations actively observe yaw from startup, instead of
  // having to wait for the straight-line accumulation. Safe when Mahalanobis
  // gating is on AND either (a) initial yaw is roughly known (dock compass
  // available) OR (b) the receiver reports full-covariance RTK fixes that
  // let the filter weight yaw corrections correctly. Off by default to
  // preserve the original conservative behavior.
  bool apply_lever_arm_pre_heading = false;
};

// ─── GNSS fix ────────────────────────────────────────────────────────────────

struct GnssFix {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  // Receiver DOP if the driver reports it, otherwise a stand-in derived from
  // the covariance. Read the comment on sigma_xy below before using this for
  // anything: in the covariance case the number here is METRES, not DOP.
  double hdop = 99.0;
  double vdop = 99.0;

  // Reported one-sigma position uncertainty in METRES. Zero means the fix
  // carried no covariance and only DOP is available.
  //
  // This exists because hdop/vdop are overloaded. When a driver publishes a
  // covariance (every NavSatFix with position_covariance_type >= 1) the node
  // has metres, not DOP, and it puts sqrt(variance) into hdop so the noise
  // model can use it as a scale factor on base_noise_xy. That works for noise
  // but it silently broke the quality gate, which compared metres against a
  // threshold everyone reads as a dimensionless DOP. Reported as issue #73:
  // a working RTK setup had every fix vetoed as VDOP_HIGH.
  //
  // So the gate now reads these fields when they are set, and hdop/vdop keep
  // their existing meaning for the noise model. Set both or neither.
  double sigma_xy = 0.0;
  double sigma_z  = 0.0;

  int         satellites = 0;
  GnssFixType fix_type   = GnssFixType::NO_FIX;

  // Source identifier: used when fusing multiple GNSS receivers.
  // 0 = primary, 1 = secondary, etc.
  int source_id = 0;

  // Per-measurement lever arm (antenna offset from base_link in body frame).
  // Set by the ROS node based on which receiver produced this fix.
  // Defaults to zero: no correction applied.
  GnssLeverArm lever_arm;

  // Full 3x3 position covariance matrix (row-major, ENU frame).
  // peci1 fix: real GNSS covariance often has off-diagonal elements
  // (correlated X/Y errors). When has_full_covariance is true, this
  // matrix is used directly instead of the diagonal HDOP/VDOP estimate.
  bool has_full_covariance = false;
  Eigen::Matrix3d full_covariance = Eigen::Matrix3d::Identity();

  // True when the fix reported a covariance, so the quality gate has metres to
  // work with and must not compare them against the DOP thresholds.
  bool has_sigma() const { return sigma_xy > 0.0 && sigma_z > 0.0; }

  // A fix carrying NaN or infinity. Checked before anything else because every
  // comparison below silently answers false against a NaN, so a garbage fix
  // would pass every gate it was tested against rather than failing them.
  //
  // Nothing recovers from this one. A NaN reaching the state or the covariance
  // propagates through the sigma points on the next predict and every value the
  // filter reports afterwards is NaN, for the rest of the run, with no way back.
  // That makes it worth a check even though a well-behaved driver never sends
  // one: seen on a 2026-09-05 rover log, 17 of 246 fixes had NaN latitude and
  // longitude. All 17 also carried status -1, so min_fix_type happened to stop
  // them, but that is the driver being tidy rather than the filter being safe.
  bool is_finite() const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
           std::isfinite(sigma_xy) && std::isfinite(sigma_z) &&
           std::isfinite(hdop) && std::isfinite(vdop) &&
           (!has_full_covariance || full_covariance.allFinite());
  }

  bool is_valid(const GnssParams& p) const {
    if (!is_finite())                   return false;
    if (fix_type < p.min_fix_type)      return false;
    if (satellites < p.min_satellites)  return false;
    if (has_sigma())
      return sigma_xy <= p.max_sigma_xy && sigma_z <= p.max_sigma_z;
    return hdop <= p.max_hdop && vdop <= p.max_vdop;
  }
};

struct GnssHeading {
  double heading_rad  = 0.0;
  double accuracy_rad = 0.1;
  bool   valid        = false;

  // Source identifier: matches the source_id of the GnssFix
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
    double qw = x[QW], qx = x[QX], qy = x[QY], qz = x[QZ];

    // Rotation matrix body-to-world from quaternion
    double R[3][3];
    quat_to_rotation_matrix(qw, qx, qy, qz, R);

    double lx = lever_arm.x, ly = lever_arm.y, lz = lever_arm.z;
    GnssPosMeasurement z;
    z[0] = x[X] + R[0][0]*lx + R[0][1]*ly + R[0][2]*lz;
    z[1] = x[Y] + R[1][0]*lx + R[1][1]*ly + R[1][2]*lz;
    z[2] = x[Z] + R[2][0]*lx + R[2][1]*ly + R[2][2]*lz;
    return z;
  };
}

// h(x): state -> expected GNSS heading (yaw extracted from quaternion)
inline GnssHdgMeasurement gnss_hdg_measurement_function(const StateVector& x) {
  GnssHdgMeasurement z;
  double qw = x[QW], qx = x[QX], qy = x[QY], qz = x[QZ];
  z[0] = std::atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
  return z;
}

// ─── Noise matrices ───────────────────────────────────────────────────────────

inline GnssPosNoiseMatrix gnss_pos_noise_matrix(
  const GnssParams& p,
  const GnssFix& fix)
{
  // peci1 fix: use full covariance matrix when available.
  // Real GNSS receivers often report correlated X/Y errors:
  // the off-diagonal elements matter, especially with RTK.
  if (fix.has_full_covariance) {
    // Validate: all diagonal elements must be positive
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
