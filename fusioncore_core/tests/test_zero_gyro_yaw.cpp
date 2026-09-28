// A gyro reading exactly zero must not produce a filter that spins.
//
// This is the sharpest available statement of #150 and it runs in milliseconds, which
// makes it worth far more than another NCLT run. The rover measurement that motivated it:
// across all seven exported field bags the filter's published yaw moved 8.2 to 10.8 times
// further than its own integrated gyro said the robot had turned.
//
// The scenario is a robot driving dead straight with a PERFECT gyro reading (0,0,0) on
// every sample, a still accelerometer reading exactly gravity, a perfect encoder, and
// 1 Hz GNSS with realistic receiver noise. Truth is yaw = 0 for the entire run.
//
// WHAT IS AND IS NOT ACCEPTABLE. With no absolute heading source yaw is not observable,
// so it is entitled to random-walk: the honest expectation is drift growing like sqrt(t),
// not a pinned zero. What is NOT acceptable is thousands of degrees of accumulated
// rotation on a stationary-yaw input, which is what happens today. So these tests bound
// the TOTAL VARIATION rather than demanding yaw stay at zero.

#include <gtest/gtest.h>
#include <cmath>
#include <random>

#include "fusioncore/state.hpp"
#include "fusioncore/fusioncore.hpp"

using namespace fusioncore;

namespace {

constexpr double kG = 9.80665;
constexpr double kSpeed = 1.0;
constexpr double kDt = 0.01;
constexpr double kGpsSigma = 2.8;      // the rover's M9N, measured in the field

double yaw_deg(const StateVector & x)
{
  return std::atan2(2 * (x[QW] * x[QZ] + x[QX] * x[QY]),
                    1 - 2 * (x[QY] * x[QY] + x[QZ] * x[QZ])) * 180.0 / M_PI;
}

double wrap(double d)
{
  while (d > 180.0) { d -= 360.0; }
  while (d < -180.0) { d += 360.0; }
  return d;
}

FusionCoreConfig rover_like(double max_sigma_rotation_deg = 0.0)
{
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.outlier_rejection = true;
  cfg.adaptive_imu = cfg.adaptive_encoder = cfg.adaptive_gnss = true;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.ukf.max_sigma_rotation_deg = max_sigma_rotation_deg;
  return cfg;
}

// Total absolute yaw motion, split by which update caused it.
struct Attribution {
  double imu = 0.0, encoder = 0.0, ground = 0.0, gnss = 0.0;
  double total() const { return imu + encoder + ground + gnss; }
  double final_x = 0.0;
};

Attribution run(double seconds, bool use_gps, double max_sigma_rotation_deg = 0.0)
{
  FusionCore fc(rover_like(max_sigma_rotation_deg));
  State s0;
  fc.init(s0, 0.0);
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, kGpsSigma);

  Attribution a;
  const int steps = static_cast<int>(seconds / kDt);
  for (int i = 1; i <= steps; ++i) {
    const double t = i * kDt;
    double y = yaw_deg(fc.get_state().x);

    fc.update_imu(t, 0, 0, 0, 0, 0, kG);           // a PERFECT still gyro
    double y2 = yaw_deg(fc.get_state().x);
    a.imu += std::abs(wrap(y2 - y));
    y = y2;

    if (i % 2 == 0) {
      fc.update_encoder(t, kSpeed, 0.0, 0.0);
      y2 = yaw_deg(fc.get_state().x); a.encoder += std::abs(wrap(y2 - y)); y = y2;
      fc.update_ground_constraint(t);
      y2 = yaw_deg(fc.get_state().x); a.ground += std::abs(wrap(y2 - y)); y = y2;
    }
    if (use_gps && i % 100 == 0) {
      sensors::GnssFix fix;
      fix.x = kSpeed * t + noise(rng);
      fix.y = noise(rng);
      fix.z = noise(rng);
      fix.sigma_xy = kGpsSigma;
      fix.sigma_z = 2.0 * kGpsSigma;
      fix.hdop = kGpsSigma;
      fix.vdop = 2.0 * kGpsSigma;
      fix.satellites = 12;
      fix.fix_type = sensors::GnssFixType::GPS_FIX;
      fc.update_gnss(t, fix);
      y2 = yaw_deg(fc.get_state().x); a.gnss += std::abs(wrap(y2 - y));
    }
  }
  a.final_x = fc.get_state().x[X];
  return a;
}

}  // namespace

// ─── THE ACCEPTANCE CRITERION FOR #150 ───────────────────────────────────────
//
// A robot driving straight for 120 s with a gyro reading exactly zero should not
// accumulate more than a few tens of degrees of yaw motion. 60 degrees is generous: it
// allows a genuine random walk while catching anything resembling today's behaviour.
//
// Run with --gtest_also_run_disabled_tests. Delete the prefix when the error-state
// rewrite lands.
TEST(ZeroGyroYaw, DISABLED_AZeroGyroDoesNotMakeTheFilterSpin)
{
  const Attribution a = run(120.0, /*use_gps=*/true);
  EXPECT_LT(a.total(), 60.0)
      << "over 120 s of driving straight with gyro = (0,0,0) the filter's yaw moved "
      << a.total() << " deg in total (IMU " << a.imu << ", GNSS " << a.gnss
      << ", encoder " << a.encoder << ", ground " << a.ground << ")";
}

TEST(ZeroGyroYaw, DISABLED_TheIMUUpdateIsNotTheDominantSourceOfRotation)
{
  // The real signature. An accelerometer cannot observe yaw at all, since gravity is
  // along Z, so the IMU update has no business being the largest contributor to yaw.
  const Attribution a = run(120.0, /*use_gps=*/true);
  EXPECT_LT(a.imu / a.total(), 0.5)
      << "the IMU update accounts for " << (100.0 * a.imu / a.total())
      << "% of all yaw motion, on a gyro reading exactly zero";
}

// ─── CHARACTERISATION: what it does today. These pin a DEFECT. ───────────────

TEST(ZeroGyroYaw, CharacteriseTheFalseRotation)
{
  const Attribution a = run(120.0, /*use_gps=*/true);
  // Position is fine throughout: GNSS pins it. It is only orientation that is wrong,
  // which is why this went unnoticed in every closure number the project has taken.
  EXPECT_NEAR(a.final_x, 120.0, 8.0)
      << "position should track truth; GNSS pins it even while yaw is nonsense";
  EXPECT_GT(a.total(), 1000.0)
      << "today this should be thousands of degrees; it measured " << a.total();
  EXPECT_GT(a.imu / a.total(), 0.7)
      << "today the IMU update should dominate; it measured "
      << (100.0 * a.imu / a.total()) << "%";
}

TEST(ZeroGyroYaw, BoundingTheSigmaPointSpreadContainsItButIsNotAFix)
{
  // ukf.max_sigma_rotation_deg is a containment mechanism, not a solution, and this
  // test exists to keep that distinction honest. It cuts the false rotation by roughly
  // an order of magnitude, which is why the rover config sets it, but the defect it is
  // containing is still there and is still #150.
  const Attribution off = run(120.0, true, 0.0);
  const Attribution on  = run(120.0, true, 2.0);
  EXPECT_LT(on.total(), off.total() * 0.5)
      << "bounding the spread should substantially cut the false rotation: "
      << off.total() << " -> " << on.total();
  EXPECT_GT(on.total(), 50.0)
      << "and it should NOT be mistaken for a fix: " << on.total()
      << " deg of rotation on a zero gyro is still wrong";
}
