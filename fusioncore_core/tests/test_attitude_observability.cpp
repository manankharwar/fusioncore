// Attitude observability: the current behaviour, pinned, and the target, executable.
//
// This file exists to make the #150 error-state migration safe. It is written BEFORE that
// work, deliberately, so the migration has a green/red signal at every step instead of
// "NCLT got worse" three hours later.
//
// Two kinds of test here, and they have opposite intent:
//
//   CHARACTERISATION  pins what the filter does TODAY. These are not statements that the
//                     behaviour is correct. Most of them pin a defect. They exist so that
//                     a migration which changes them does so visibly and in a named place.
//                     When #150 lands, these values SHOULD change, and the diff is the
//                     evidence the rewrite did what it claimed.
//
//   INVARIANT         states what must be true of any correct attitude representation.
//                     DISABLED_ today because it fails: it is the acceptance criterion
//                     for #150 written as code. Run with
//                     --gtest_also_run_disabled_tests and it goes green exactly when the
//                     rewrite is done.
//
// The scenario throughout mirrors tools/repro/dr.cpp: a PERFECT encoder at 1 m/s dead
// straight and a still IMU, so truth is x = 1.0 * t, y = 0, yaw = 0, and any deviation is
// the filter's own.
//
// THE RUN HAS TWO PHASES AND EVERY TOOL IN THIS PROJECT ONLY EVER SAW THE FIRST.
// dr.cpp, obs.cpp and split.cpp all default to 60 seconds. The transition is at 77.
//
//   PHASE 1, t < 77 s   sigma(QZ) climbs 2.76 -> 15.84, which is impossible for a
//                       component bounded by 1, but the ESTIMATE stays perfect: yaw
//                       reads -0.000 deg and position advances at a steady 80% of truth.
//                       This is the phase the standing "the estimate is fine, the
//                       covariance is not" description refers to, and for this phase it
//                       is accurate.
//
//   PHASE 2, t > 77 s   in ONE step between t=77.0 and t=78.0, sigma(QZ) collapses
//                       15.44 -> 0.34 and yaw flips 0.001 -> -179.721 deg. |q| stays
//                       exactly 1.0, so it is still a valid unit quaternion; it now
//                       points backwards. Position immediately reverses (61.83 m at
//                       t=77 down to 51.30 m by t=95) and for the rest of the run yaw
//                       tumbles through the full circle while position random-walks in
//                       a band and never advances again. At t=400 s the filter has the
//                       robot at 19.57 m after 400 m of driving.
//
// Phase 2 is the one that matters for NCLT: 2012-06-15's blackout is 461 s, so the two
// sequences FusionCore loses are entirely in it. Velocity reads a perfect 1.0000 and
// sigma(QZ) reads a healthy 0.2-0.9 throughout phase 2, so nothing in the filter's own
// output says anything is wrong.

#include <gtest/gtest.h>
#include <cmath>

#include "fusioncore/state.hpp"
#include "fusioncore/fusioncore.hpp"

using namespace fusioncore;

namespace {

constexpr double kG = 9.80665;
constexpr double kSpeed = 1.0;
constexpr double kDt = 0.01;

FusionCoreConfig dr_config()
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
  return cfg;
}

struct Result {
  double advance_ratio;   // x / (v * t), 1.0 is correct
  double sigma_qz;        // must be <= 1.0 for a unit quaternion
  double sigma_qw, sigma_qx, sigma_qy;
  double yaw_deg;         // the ESTIMATE, which stays correct even when P does not
  double q_norm;
};

Result dead_reckon(double seconds)
{
  FusionCore fc(dr_config());
  State s0;
  fc.init(s0, 0.0);

  double t = 0.0;
  const int steps = static_cast<int>(seconds / kDt);
  for (int i = 1; i <= steps; ++i) {
    t = i * kDt;
    fc.update_imu(t, 0, 0, 0, 0, 0, kG);
    if (i % 2 == 0) {
      fc.update_encoder(t, kSpeed, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }
  }

  const auto & x = fc.get_state().x;
  const auto & P = fc.get_state().P;
  Result r;
  r.advance_ratio = x[X] / (kSpeed * seconds);
  r.sigma_qw = std::sqrt(std::max(P(QW, QW), 0.0));
  r.sigma_qx = std::sqrt(std::max(P(QX, QX), 0.0));
  r.sigma_qy = std::sqrt(std::max(P(QY, QY), 0.0));
  r.sigma_qz = std::sqrt(std::max(P(QZ, QZ), 0.0));
  r.yaw_deg = std::atan2(2 * (x[QW] * x[QZ] + x[QX] * x[QY]),
                         1 - 2 * (x[QY] * x[QY] + x[QZ] * x[QZ])) * 180.0 / M_PI;
  r.q_norm = std::sqrt(x[QW]*x[QW] + x[QX]*x[QX] + x[QY]*x[QY] + x[QZ]*x[QZ]);
  return r;
}

}  // namespace

// ─── INVARIANTS: true of any correct attitude representation ─────────────────

TEST(AttitudeObservability, TheQuaternionStaysOnTheUnitSphere)
{
  // True in BOTH phases, and worth separating from the rest: the flip at t=77 is not a
  // normalisation failure. |q| is exactly 1 before and after. The filter swaps one valid
  // attitude for a different valid attitude that happens to be 180 degrees wrong.
  for (double secs : {60.0, 90.0, 300.0}) {
    EXPECT_NEAR(dead_reckon(secs).q_norm, 1.0, 1e-6)
        << "at " << secs << " s the quaternion left the unit sphere";
  }
}

TEST(AttitudeObservability, TheEstimateIsCorrectInPhaseOne)
{
  // Measured at 60 s, safely before the 77 s transition. This is what every existing
  // tool in the project measured, and within its window it is accurate.
  const Result r = dead_reckon(60.0);
  EXPECT_NEAR(r.yaw_deg, 0.0, 1.0)
      << "the robot drove dead straight and yaw reads " << r.yaw_deg << " deg";
}

TEST(AttitudeObservability, ObservableAttitudeComponentsStayWithinTheConstraint)
{
  // Roll and pitch ARE observable from gravity, so their covariance is well behaved even
  // today. Only yaw is broken. If a migration ever breaks these, it has gone backwards.
  const Result r = dead_reckon(60.0);
  EXPECT_LE(r.sigma_qw, 1.0) << "sigma(QW) = " << r.sigma_qw;
  EXPECT_LE(r.sigma_qx, 1.0) << "sigma(QX) = " << r.sigma_qx;
  EXPECT_LE(r.sigma_qy, 1.0) << "sigma(QY) = " << r.sigma_qy;
}

// THE ACCEPTANCE CRITERION FOR #150, as code.
//
// A quaternion component lives in [-1, 1], so its standard deviation cannot exceed 1.
// Measured today: 13.32 at 60 s, which is a 13x violation of a hard bound, and the
// mechanism behind the dead-reckoning collapse characterised below.
//
// DISABLED because it fails. Enable it by deleting the prefix when the error-state
// migration lands; it going green IS the definition of done. Until then:
//     ./test_attitude_observability --gtest_also_run_disabled_tests
TEST(AttitudeObservability, DISABLED_YawCovarianceRespectsTheUnitNormConstraint)
{
  for (double secs : {10.0, 30.0, 60.0, 150.0}) {
    const Result r = dead_reckon(secs);
    EXPECT_LE(r.sigma_qz, 1.0)
        << "at " << secs << " s, sigma(QZ) = " << r.sigma_qz
        << ". A quaternion component cannot have a standard deviation above 1.";
  }
}

// The sharpest statement of #150, and the one a user would actually notice. The robot
// drives dead straight for five minutes and the filter must not decide it turned round.
TEST(AttitudeObservability, DISABLED_YawSurvivesPastTheSeventySevenSecondCollapse)
{
  for (double secs : {90.0, 150.0, 300.0}) {
    const Result r = dead_reckon(secs);
    EXPECT_NEAR(r.yaw_deg, 0.0, 5.0)
        << "at " << secs << " s, driving dead straight, yaw reads " << r.yaw_deg
        << " deg. Today it flips to -179.7 at t=77 and tumbles after that.";
  }
}

// The other half of done: the deficit must go away and must not grow with time.
TEST(AttitudeObservability, DISABLED_DeadReckoningTracksAPerfectVelocity)
{
  for (double secs : {30.0, 60.0, 150.0, 300.0}) {
    const Result r = dead_reckon(secs);
    EXPECT_GT(r.advance_ratio, 0.95)
        << "at " << secs << " s the filter advanced at " << r.advance_ratio
        << " of a perfect velocity, with velocity and yaw both correct";
  }
}

// ─── CHARACTERISATION: what it does today. These pin a DEFECT. ───────────────
//
// Tolerances are loose enough not to be brittle and tight enough that a real change
// shows. If the migration lands and these fail, that is the point: read the numbers.

TEST(AttitudeObservability, CharacteriseYawCovarianceGrowth)
{
  // Grows without bound, then collapses near t=70 s when the numerical reconditioning in
  // generate_sigma_points fires. The collapse does NOT restore correct behaviour, which
  // is why the advance ratio keeps falling afterwards.
  EXPECT_NEAR(dead_reckon(10.0).sigma_qz,  2.76, 0.60);
  EXPECT_NEAR(dead_reckon(30.0).sigma_qz,  7.62, 1.50);
  EXPECT_NEAR(dead_reckon(60.0).sigma_qz, 13.32, 2.50);
}

TEST(AttitudeObservability, CharacteriseThePhaseTwoCollapse)
{
  // The transition is sharp and sits between 77 and 78 s. Pinned with margin on both
  // sides rather than at the exact step, so ordinary numerical drift does not fail it.
  const Result before = dead_reckon(70.0);
  EXPECT_GT(before.sigma_qz, 10.0) << "phase 1 should still have a huge sigma(QZ)";
  EXPECT_NEAR(before.yaw_deg, 0.0, 1.0) << "phase 1 yaw should still be perfect";

  const Result after = dead_reckon(90.0);
  EXPECT_LT(after.sigma_qz, 1.0)
      << "after the collapse sigma(QZ) looks HEALTHY at " << after.sigma_qz
      << ", which is what makes this undiagnosable from the filter's own output";
  EXPECT_GT(std::abs(after.yaw_deg), 90.0)
      << "after the collapse yaw should be grossly wrong; it reads " << after.yaw_deg;
}

TEST(AttitudeObservability, CharacteriseThatPositionStopsAdvancingEntirelyInPhaseTwo)
{
  // Not "advances slowly". At t=300 the filter has the robot BEHIND where it had it at
  // t=70, after another 230 m of driving. The cumulative ratio in the test below hides
  // this, which is why it is stated separately.
  const double x70  = dead_reckon(70.0).advance_ratio  * 70.0;
  const double x300 = dead_reckon(300.0).advance_ratio * 300.0;
  EXPECT_NEAR(x70, 56.4, 3.0);
  EXPECT_LT(x300, x70)
      << "position at 300 s (" << x300 << " m) should be less than at 70 s (" << x70
      << " m): the filter goes backwards, it does not merely lag";
}

TEST(AttitudeObservability, CharacteriseTheDeadReckoningDecay)
{
  // 87% at 5 s decaying to 38% at 150 s, while velocity reads a perfect 1.000 and yaw a
  // perfect 0.00 throughout. Over a 461 s blackout like NCLT 2012-06-15's, this is the
  // mechanism behind the losses to robot_localization.
  EXPECT_NEAR(dead_reckon(30.0).advance_ratio,  0.815, 0.05);
  EXPECT_NEAR(dead_reckon(60.0).advance_ratio,  0.807, 0.05);
  EXPECT_NEAR(dead_reckon(150.0).advance_ratio, 0.376, 0.08);
}

TEST(AttitudeObservability, CharacteriseThatVelocityStaysPerfectWhilePositionDoesNot)
{
  // The signature that hid this for months in field data: velocity right, position wrong.
  // Any explanation of #150 that predicts bad velocity is wrong.
  FusionCore fc(dr_config());
  State s0;
  fc.init(s0, 0.0);
  double t = 0.0;
  for (int i = 1; i <= 6000; ++i) {
    t = i * kDt;
    fc.update_imu(t, 0, 0, 0, 0, 0, kG);
    if (i % 2 == 0) { fc.update_encoder(t, kSpeed, 0.0, 0.0); fc.update_ground_constraint(t); }
  }
  const auto & x = fc.get_state().x;
  EXPECT_NEAR(x[VX], 1.000, 0.01) << "velocity should be perfect; it is the position that is short";
  EXPECT_LT(x[X], 0.90 * kSpeed * t) << "position should be short by roughly 19%";
}
