#pragma once

#include "fusioncore/state.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace fusioncore {
namespace sensors {

// Magnetometer heading update (1-dimensional: yaw only).
// Subscribes to sensor_msgs/MagneticField, applies hard/soft iron correction,
// tilt-compensates using current filter roll/pitch, then fuses the resulting
// heading into the UKF via the same 1-DOF update used for dual-antenna GPS.
//
// The measurement function is identical to gnss_hdg_measurement_function:
// h(x) = yaw extracted from quaternion. Both reuse GNSS_HDG_DIM / gnss_hdg_measurement_function.
//
// Frame convention: ENU (x=forward, y=left, z=up) throughout.

struct MagParams {
  // Heading noise (rad): standard deviation of the corrected heading estimate.
  // 0.05 rad (~3 deg) is a reasonable starting point for a well-calibrated
  // sensor in a benign magnetic environment. Loosen to 0.15-0.3 in environments
  // with moderate motor/cable interference.
  double noise_rad = 0.05;

  // Chi-squared outlier gate threshold: chi2(1, 0.99) = 9.21.
  // A 1-DOF gate is appropriate because magnetometer gives only yaw.
  // Tighten to chi2(1, 0.95)=3.84 in clean environments; loosen to chi2(1,0.999)=10.83
  // near motors or structures with variable magnetic fields.
  double chi2_threshold = 9.21;

  // Magnetic declination (rad): offset between magnetic north and true north.
  // Positive east (e.g. +0.07 rad for London, -0.22 rad for Michigan).
  // Look up at: https://www.magnetic-declination.com
  // Leave at 0.0 when using magnetometer only for drift suppression during GPS
  // outages: the filter self-corrects for the constant offset via GPS.
  double declination_rad = 0.0;

  // Hard iron correction: constant bias offset in body frame (Tesla).
  // Estimated once using imu_calib or magneto, or by rotating the sensor
  // through a full circle and computing (max + min) / 2 per axis.
  Eigen::Vector3d hard_iron = Eigen::Vector3d::Zero();

  // Soft iron correction: scale+rotation matrix (dimensionless).
  // Applied as: m_corrected = soft_iron * (m_raw - hard_iron).
  // Identity = no soft iron correction (default).
  // Estimated alongside hard iron using imu_calib or magneto.
  Eigen::Matrix3d soft_iron = Eigen::Matrix3d::Identity();

  // Adaptive magnetic-disturbance rejection by field magnitude.
  // A clean reading's corrected field magnitude |S*(m_raw - b)| equals the local
  // Earth field strength. A nearby motor, steel structure, or rebar distorts the
  // magnitude, and the resulting heading is wrong in a way the chi2 gate cannot
  // reliably catch: a disturbed heading can still sit within chi2 of an already
  // drifting prediction, so it gets accepted and pulls heading off. Checking the
  // magnitude catches the disturbance the heading gate misses. When |m_corrected|
  // deviates from field_strength by more than field_tolerance (fractional), the
  // reading is rejected so a transient disturbance never corrupts heading.
  // Set field_strength to the local Earth total-field magnitude in the SAME UNITS
  // as the incoming reading (e.g. ~0.48 for Gauss, ~48 for microtesla; look up the
  // total field at https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml).
  // 0.0 = disabled (default), preserving prior behavior.
  double field_strength  = 0.0;
  double field_tolerance = 0.2;  // allowed fractional deviation (0.2 = +-20%)
};

// True when the corrected field magnitude deviates from the expected Earth field
// by more than the configured tolerance: a local magnetic disturbance that makes
// the heading untrustworthy. Always false when field_strength <= 0 (disabled).
inline bool mag_field_disturbed(
  double mx, double my, double mz, const MagParams& p)
{
  if (p.field_strength <= 0.0) return false;
  Eigen::Vector3d m_c = p.soft_iron * (Eigen::Vector3d(mx, my, mz) - p.hard_iron);
  return std::abs(m_c.norm() - p.field_strength) > p.field_tolerance * p.field_strength;
}

// Apply hard/soft iron correction and tilt compensation to a raw magnetometer
// reading, returning the heading in radians (ENU yaw convention: counterclockwise
// from east, same as the UKF's quaternion-derived yaw).
//
// roll, pitch: current filter roll and pitch in radians (from quat_to_euler on
//              the UKF state). These are used for tilt compensation only.
//
// Returns the heading as yaw in ENU [-pi, pi].
inline double mag_yaw_from_field(
  double mx, double my, double mz,
  const MagParams& p,
  double roll, double pitch)
{
  // Hard/soft iron correction: m_c = S * (m_raw - b)
  Eigen::Vector3d m_raw(mx, my, mz);
  Eigen::Vector3d m_c = p.soft_iron * (m_raw - p.hard_iron);

  const double r = roll;
  const double q = pitch;

  // Tilt compensation: project body-frame field onto horizontal plane.
  // Derivation (ENU, x=forward, y=left, z=up):
  //   m_pre = Ry(pitch) * Rx(roll) * m_c
  //   yaw = atan2(m_pre.x, m_pre.y)
  // This correctly handles non-zero roll and pitch without knowing yaw.
  const double cx = m_c[0], cy = m_c[1], cz = m_c[2];
  const double mx_h = cx * std::cos(q)
                    + cy * std::sin(r) * std::sin(q)
                    + cz * std::cos(r) * std::sin(q);
  const double my_h = cy * std::cos(r)
                    - cz * std::sin(r);

  double yaw = std::atan2(mx_h, my_h) + p.declination_rad;
  // Normalize to [-pi, pi]
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

} // namespace sensors
} // namespace fusioncore
