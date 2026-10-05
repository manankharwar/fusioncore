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

// ─── The same defect on YAW, and the probe that makes it cheap to check ──────
//
// Everything above dead-reckons in a straight line. This spins in place instead, and it
// turns the 19% position shortfall into the same ratio on YAW, in 1.8 seconds, with no
// dataset. Found while landing the rotation heading bootstrap (#67): the bootstrap would
// not fire, and the reason was that the filter does not know the rotation it just made.
//
// Why these are worth having as tests rather than a note: judging an attempt at #150 used
// to mean a 70 minute NCLT sequence. These run in milliseconds and read the mechanism
// directly off the state vector.

namespace {

struct SpinResult {
  double wz;          // the filter's angular rate
  double b_gz;        // the filter's yaw gyro bias
  double yaw;         // integrated yaw
  double yaw_sigma;   // 1-sigma from the quaternion block of P
};

// Rotate in place at a KNOWN true rate, IMU and encoder both feeding, and report what the
// filter believes afterwards. with_imu=false drops the IMU to show the two arms fail in
// opposite directions.
SpinResult spin_in_place(double true_wz, double seconds, bool with_imu = true)
{
  FusionCoreConfig cfg;
  cfg.outlier_rejection = false;
  cfg.adaptive_gnss = false;
  cfg.gps_track_heading_enabled = false;
  FusionCore fc(cfg);

  State s;
  s.P = StateMatrix::Identity() * 0.1;
  for (int i : {QW, QX, QY, QZ})
    for (int j : {QW, QX, QY, QZ})
      s.P(i, j) = (i == j) ? 1e-6 : 0.0;   // heading tightly known at init
  fc.init(s, 0.0);

  const int steps = static_cast<int>(seconds / 0.01);
  for (int i = 1; i <= steps; ++i) {
    const double t = i * 0.01;
    if (with_imu) fc.update_imu(t, 0.0, 0.0, true_wz, 0.0, 0.0, kG);
    fc.update_encoder(t, 0.0, 0.0, true_wz, 1e-4, 1e-4, 1e-4);
  }

  const auto & x = fc.get_state().x;
  const auto & P = fc.get_state().P;
  const double qw = x[QW], qz = x[QZ];
  const double t3 = 2.0 * (qw * qz);
  const double t4 = 1.0 - 2.0 * (qz * qz);
  const double den = std::max(t3 * t3 + t4 * t4, 1e-12);
  const double j0 = 2.0 * qz * t4 / den;
  const double j3 = (2.0 * qw * t4 + 4.0 * qz * t3) / den;
  const double var = j0 * j0 * P(QW, QW) + j3 * j3 * P(QZ, QZ)
                   + 2.0 * j0 * j3 * P(QW, QZ);

  SpinResult r;
  r.wz        = x[WZ];
  r.b_gz      = x[B_GZ];
  r.yaw       = std::atan2(t3, t4);
  r.yaw_sigma = std::sqrt(std::max(var, 0.0));
  return r;
}

}  // namespace

// INVARIANT. The acceptance criterion for #150 stated on the yaw rate rather than on
// yaw itself, which is the sharpest form of it: WZ and B_GZ are not separately
// observable from a gyro alone, so the filter is free to slide the true rate between
// them at zero measurement cost. Nothing that integrates WZ alone survives that, and
// yaw does exactly that.
TEST(AttitudeObservability, DISABLED_YawRateDoesNotSlideOntoTheGyroBias)
{
  const SpinResult r = spin_in_place(0.6, 1.8);
  EXPECT_NEAR(r.wz, 0.6, 0.02)
      << "WZ carries only " << r.wz << " of a true 0.6; the rest went to B_GZ";
  EXPECT_NEAR(r.b_gz, 0.0, 0.02)
      << "a zero-bias gyro produced a bias estimate of " << r.b_gz;
}

// CHARACTERISATION. The pair sums to the truth while neither half is right. This is the
// clearest statement of the #150 mechanism anywhere in the suite: the sum costs nothing
// to get right, so the filter gets it right, and the split is unconstrained.
TEST(AttitudeObservability, CharacteriseThatTheYawRatePairSumsCorrectlyWhileSplitWrong)
{
  const SpinResult r = spin_in_place(0.6, 1.8);

  EXPECT_NEAR(r.wz + r.b_gz, 0.6, 0.005)
      << "the SUM is observable and the filter tracks it";
  EXPECT_LT(r.wz, 0.55)
      << "if WZ now carries the full rate, #150 moved; measured 0.4800 on this commit";
  EXPECT_GT(r.b_gz, 0.05)
      << "if B_GZ is now near zero, #150 moved; measured 0.1200 on this commit";
}

// CHARACTERISATION. Yaw integrates WZ alone, so it inherits the split exactly. 81.6%
// here against the 80.7% that CharacteriseTheDeadReckoningDecay measures for position
// against a perfect velocity: the same number, which says these are one defect and not
// two. If a migration fixes one and not the other, this pair is where it shows.
TEST(AttitudeObservability, CharacteriseThatYawAdvancesAtTheSameFractionAsPosition)
{
  const double true_wz = 0.6, seconds = 1.8;
  const SpinResult r = spin_in_place(true_wz, seconds);
  const double ratio = r.yaw / (true_wz * seconds);

  EXPECT_NEAR(ratio, 0.816, 0.03)
      << "yaw advanced at " << ratio << " of truth; 0.816 is what this commit does, and "
         "it is the position shortfall over again";
  EXPECT_NEAR(ratio, r.wz / true_wz, 0.02)
      << "the shortfall should equal the share of the rate WZ kept, which is the mechanism";
}

// CHARACTERISATION. Why the rotation heading bootstrap needs a brisk turn, and the
// numbers configuration.md quotes. Yaw uncertainty grows while turning, so a slow turn
// has lost the rotation by the time it has swept a usable arc.
TEST(AttitudeObservability, CharacteriseYawUncertaintyGrowthWhileTurning)
{
  const SpinResult with    = spin_in_place(0.6, 1.8, true);
  const SpinResult without = spin_in_place(0.6, 1.8, false);

  EXPECT_NEAR(with.yaw_sigma,    0.361, 0.05) << "measured 0.361 rad with an IMU feeding";
  EXPECT_NEAR(without.yaw_sigma, 0.564, 0.07) << "measured 0.564 rad on encoder alone";
  EXPECT_LT(with.yaw_sigma, without.yaw_sigma)
      << "the IMU should constrain attitude; if not, the gyro is not reaching the filter";
}

// CHARACTERISATION. The two arms fail in OPPOSITE directions, which matters because it
// rules out a single sign error and should stop the next mechanism proposal that assumes
// one. With the IMU yaw falls short of truth; on encoder alone it overshoots.
TEST(AttitudeObservability, CharacteriseThatTheEncoderOnlyArmOvershootsInstead)
{
  const double true_wz = 0.6, seconds = 1.8;
  const SpinResult with    = spin_in_place(true_wz, seconds, true);
  const SpinResult without = spin_in_place(true_wz, seconds, false);
  const double denom = true_wz * seconds;

  EXPECT_LT(with.yaw / denom,    1.0) << "with IMU: measured 81.6% of truth";
  EXPECT_GT(without.yaw / denom, 1.0) << "encoder only: measured 106.6% of truth";
  EXPECT_NEAR(without.wz + without.b_gz, 0.4, 0.05)
      << "and on encoder alone even the SUM is wrong, 0.40 against a true 0.60";
}
