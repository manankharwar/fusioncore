// What the filter can see, and what to drive to fix what it cannot.
//
// Most of this project's hardest open problems are observability problems wearing a
// filter problem's clothes: #150 is WZ + B_GZ sliding along a direction nothing
// measures, "heading never fuses" was 0 bearings in 298 fixes, the GNSS lever arm
// sits inert until heading uncertainty drops, and the #67 bootstrap declines unless
// the turn is brisk. In each the filter worked as specified and the robot was never
// driven in a way that made the quantity observable.
//
// A user cannot tell those two apart from outside, so FusionCore::observability()
// says which it is and names the manoeuvre. These tests pin the real measured values
// rather than invented ones, so a change to the filter that moves them shows up here
// in milliseconds instead of in a 75 minute playback.

#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include <cmath>
#include <set>
#include <string>

using namespace fusioncore;

namespace {

FusionCoreConfig obsConfig()
{
  FusionCoreConfig c;
  c.outlier_rejection = false;
  c.adaptive_gnss = false;
  c.gps_track_heading_enabled = false;
  return c;
}

// Heading tightly known at init, so a report of UNOBSERVABLE there is about nothing
// having OBSERVED it rather than about a loose prior.
void initTight(FusionCore& fc)
{
  State s;
  s.P = StateMatrix::Identity() * 0.1;
  for (int i : {QW, QX, QY, QZ})
    for (int j : {QW, QX, QY, QZ})
      s.P(i, j) = (i == j) ? 1e-6 : 0.0;
  fc.init(s, 0.0);
}

double spin(FusionCore& fc, double wz, int steps)
{
  double t = 0.0;
  for (int i = 1; i <= steps; ++i) {
    t = i * 0.01;
    fc.update_imu(t, 0.0, 0.0, wz, 0.0, 0.0, 9.81);
    fc.update_encoder(t, 0.0, 0.0, wz, 1e-4, 1e-4, 1e-4);
  }
  return t;
}

}  // namespace

TEST(ObservabilityReport, AnUninitialisedFilterClaimsNothing)
{
  FusionCore fc(obsConfig());
  const auto r = fc.observability();
  EXPECT_EQ(r.heading,     Observability::UNKNOWN);
  EXPECT_EQ(r.gyro_bias_z, Observability::UNKNOWN);
  EXPECT_EQ(r.lever_arm,   Observability::UNKNOWN);
  EXPECT_EQ(r.next,        ExcitationManoeuvre::NONE)
      << "an uninitialised filter must not prescribe a manoeuvre";
}

// The distinction that matters most in the whole report. At init the heading sigma
// from P is 0.1 degrees, which looks excellent, and nothing has observed heading at
// all. Reporting OBSERVABLE off a tight prior is how a user ends up trusting a
// heading that never fused.
TEST(ObservabilityReport, ATightPriorIsNotAnObservation)
{
  FusionCore fc(obsConfig());
  initTight(fc);
  const auto r = fc.observability();
  EXPECT_LT(r.heading_sigma_deg, 1.0) << "the prior really is tight";
  EXPECT_EQ(r.heading, Observability::UNOBSERVABLE)
      << "heading has never been observed, however tight the prior looks";
}

// #150, read straight off P. A gyro alone cannot separate a rate from a bias on that
// rate: they enter the measurement identically, so the pair slides along
// WZ + B_GZ = const at zero measurement cost. Correlation is the right indicator
// because it approaches 1 exactly when the pair is free.
TEST(ObservabilityReport, TheYawRateAndItsBiasAreReportedUnobservable)
{
  FusionCore fc(obsConfig());
  initTight(fc);
  spin(fc, 0.6, 180);

  const auto r = fc.observability();
  EXPECT_GE(std::abs(r.wz_bgz_correlation), kWzBgzCorrelationLimit)
      << "measured -0.9982 on this commit; if it dropped, #150 moved";
  EXPECT_EQ(r.gyro_bias_z, Observability::UNOBSERVABLE);
}

// The sign is not incidental. A NEGATIVE correlation is the pair trading against each
// other to hold their sum, which is the mechanism: the sum is observable so the
// filter tracks it, the split is not so it is free, and yaw integrates WZ alone.
TEST(ObservabilityReport, TheCorrelationIsNegativeBecauseThePairHoldsItsSum)
{
  FusionCore fc(obsConfig());
  initTight(fc);
  spin(fc, 0.6, 180);
  EXPECT_LT(fc.observability().wz_bgz_correlation, -0.9)
      << "a positive correlation would mean a different mechanism entirely";
}

// Driving straight does not fix it either, which is the point of prescribing a
// figure-eight rather than any motion at all. Measured -0.9997 after 6 s straight.
TEST(ObservabilityReport, DrivingStraightDoesNotSeparateThePair)
{
  FusionCore fc(obsConfig());
  initTight(fc);
  for (int i = 1; i <= 600; ++i) {
    const double t = i * 0.01;
    fc.update_imu(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81);
    fc.update_encoder(t, 1.0, 0.0, 0.0, 1e-4, 1e-4, 1e-4);
  }
  const auto r = fc.observability();
  EXPECT_GE(std::abs(r.wz_bgz_correlation), kWzBgzCorrelationLimit);
  EXPECT_EQ(r.next, ExcitationManoeuvre::STOP_AND_WAIT)
      << "if the bias is free, the answer is to stop, not to drive differently";
}

// A low correlation alone must not read as observable. At init P is diagonal, so the
// correlation is 0 while the bias sigma is sqrt(0.1) = 0.316 rad/s, which is 18 deg/s.
// Uncorrelated and unconstrained are different things.
TEST(ObservabilityReport, AnUnconstrainedBiasIsNotObservableJustBecauseItIsUncorrelated)
{
  FusionCore fc(obsConfig());
  initTight(fc);
  const auto r = fc.observability();
  EXPECT_NEAR(r.wz_bgz_correlation, 0.0, 1e-9) << "diagonal P, so no correlation yet";
  EXPECT_GT(r.gyro_bias_z_sigma, kGyroBiasMarginalSigma);
  EXPECT_EQ(r.gyro_bias_z, Observability::UNOBSERVABLE)
      << "zero correlation with a 0.3 rad/s sigma is not observability";
}

TEST(ObservabilityReport, TheLeverArmFollowsHeadingUncertainty)
{
  auto cfg = obsConfig();
  cfg.gnss_lever_arm_max_heading_sigma_deg = 20.0;
  FusionCore fc(cfg);
  initTight(fc);
  // Tight prior: under the gate, so the lever arm would be applied.
  EXPECT_EQ(fc.observability().lever_arm, Observability::OBSERVABLE);
  // After a spin, heading uncertainty has grown past the gate.
  spin(fc, 0.6, 180);
  const auto r = fc.observability();
  EXPECT_GT(r.heading_sigma_deg, cfg.gnss_lever_arm_max_heading_sigma_deg);
  EXPECT_EQ(r.lever_arm, Observability::UNOBSERVABLE);
}

TEST(ObservabilityReport, OptingInPreHeadingOverridesTheLeverArmGate)
{
  auto cfg = obsConfig();
  cfg.gnss.apply_lever_arm_pre_heading = true;
  FusionCore fc(cfg);
  initTight(fc);
  spin(fc, 0.6, 180);
  EXPECT_EQ(fc.observability().lever_arm, Observability::OBSERVABLE)
      << "the user asked for it deliberately; the report must not contradict them";
}

TEST(ObservabilityReport, EveryManoeuvreHasADistinctUsableInstruction)
{
  const ExcitationManoeuvre all[] = {
    ExcitationManoeuvre::NONE,
    ExcitationManoeuvre::DRIVE_STRAIGHT,
    ExcitationManoeuvre::TURN_IN_PLACE_FAST,
    ExcitationManoeuvre::FIGURE_EIGHT,
    ExcitationManoeuvre::STOP_AND_WAIT,
  };
  std::set<std::string> seen;
  for (auto m : all) {
    const std::string s = excitation_instruction(m);
    EXPECT_GT(s.size(), 30u) << "an instruction a user cannot act on is not one";
    EXPECT_EQ(s.find("unknown"), std::string::npos);
    EXPECT_TRUE(seen.insert(s).second) << "two manoeuvres share instruction text";
  }
}

TEST(ObservabilityReport, EveryObservabilityValueHasAName)
{
  const Observability all[] = {
    Observability::UNKNOWN, Observability::UNOBSERVABLE,
    Observability::MARGINAL, Observability::OBSERVABLE,
  };
  std::set<std::string> seen;
  for (auto o : all) {
    const std::string s = observability_name(o);
    EXPECT_NE(s, "?");
    EXPECT_TRUE(seen.insert(s).second);
  }
}

// At startup the gyro bias is always unconstrained, so the answer is STOP, whichever
// heading path is enabled. The bias outranks heading because driving before it is
// observed is what produces the 81.6% yaw integration in the first place.
TEST(ObservabilityReport, AtStartupTheAnswerIsAlwaysToStop)
{
  for (bool rotation_enabled : {false, true}) {
    auto cfg = obsConfig();
    cfg.gps_rotation_heading_enabled = rotation_enabled;
    FusionCore fc(cfg);
    initTight(fc);
    EXPECT_EQ(fc.observability().next, ExcitationManoeuvre::STOP_AND_WAIT)
        << "rotation_enabled=" << rotation_enabled;
  }
}

// The measurement that corrected this file. An earlier version asked for a
// figure-eight on the reasoning that reversing the turn separates a constant bias
// from a constant rate. It does not, and the difference is not subtle:
//
//   spin one way only           B_GZ err +0.1000   r = -0.9997
//   figure-eight, never stops   B_GZ err +0.1000   r = -0.9978
//   figure-eight WITH stops     B_GZ err  0.0000   r = -0.1069
//
// The gyro reads WZ + B_GZ at every instant whichever way the robot turns, so the
// degeneracy is in the measurement Jacobian and no trajectory adds an independent
// equation. ZUPT is a different measurement: z = WZ with no bias term.
TEST(ObservabilityReport, AFigureEightWithoutStopsDoesNotSeparateTheBias)
{
  auto drive = [](bool with_stops) {
    auto cfg = obsConfig();
    FusionCore fc(cfg);
    initTight(fc);
    double t = 0.0;
    const double true_bias = 0.05;
    for (int lap = 0; lap < 6; ++lap) {
      const double dir = (lap % 2) ? -1.0 : 1.0;
      for (int i = 0; i < 300; ++i) {
        t += 0.01;
        fc.update_imu(t, 0.0, 0.0, 0.6 * dir + true_bias, 0.0, 0.0, 9.81);
        fc.update_encoder(t, 1.0, 0.0, 0.6 * dir, 1e-4, 1e-4, 1e-4);
      }
      if (with_stops) {
        for (int i = 0; i < 200; ++i) {
          t += 0.01;
          fc.update_imu(t, 0.0, 0.0, true_bias, 0.0, 0.0, 9.81);
          fc.update_encoder(t, 0.0, 0.0, 0.0, 1e-4, 1e-4, 1e-4);
          fc.update_zupt(t);
        }
      }
    }
    return fc.get_state().x[B_GZ];
  };

  const double no_stops   = drive(false);
  const double with_stops = drive(true);

  EXPECT_GT(std::abs(no_stops - 0.05), 0.05)
      << "a figure-eight with no stops left B_GZ at " << no_stops
      << " against a true 0.05; if this ever passes, the formulation changed";
  EXPECT_NEAR(with_stops, 0.05, 0.005)
      << "stopping must recover the true bias; got " << with_stops;
}

// The whole of #150 in one assertion: stopping first turns 81.6% into 100.1%.
TEST(ObservabilityReport, AStopBeforeMovingFixesTheYawIntegration)
{
  auto yaw_ratio = [](double still_secs) {
    auto cfg = obsConfig();
    FusionCore fc(cfg);
    initTight(fc);
    double t = 0.0;
    for (int i = 0; i < static_cast<int>(still_secs / 0.01); ++i) {
      t += 0.01;
      fc.update_imu(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81);
      fc.update_encoder(t, 0.0, 0.0, 0.0, 1e-4, 1e-4, 1e-4);
      fc.update_zupt(t);
    }
    for (int i = 0; i < 180; ++i) {
      t += 0.01;
      fc.update_imu(t, 0.0, 0.0, 0.6, 0.0, 0.0, 9.81);
      fc.update_encoder(t, 0.0, 0.0, 0.6, 1e-4, 1e-4, 1e-4);
    }
    return fc.get_state().yaw() / (0.6 * 1.8);
  };
  EXPECT_LT(yaw_ratio(0.0),  0.90) << "measured 0.816 with no stop";
  EXPECT_NEAR(yaw_ratio(10.0), 1.0, 0.03) << "measured 1.001 after a 10 s stop";
}

// Once the bias IS constrained, the manoeuvre has to match the heading path that is
// actually enabled. Telling someone to turn in place when the rotation bootstrap is
// off is instructing them to do something that cannot help them.
TEST(ObservabilityReport, WithTheBiasSettledTheManoeuvreMatchesTheEnabledPath)
{
  // The bias has to be OBSERVED, not merely tightly primed. A tight prior is not an
  // observation, so a few seconds of ZUPT is what actually settles it.
  auto settle_bias = [](FusionCore& fc) {
    State s;
    s.P = StateMatrix::Identity() * 0.1;
    for (int i : {QW, QX, QY, QZ})
      for (int j : {QW, QX, QY, QZ})
        s.P(i, j) = (i == j) ? 1e-6 : 0.0;
    s.P(B_GZ, B_GZ) = 1e-8;
    fc.init(s, 0.0);
    for (int i = 1; i <= 300; ++i) {
      const double t = i * 0.01;
      fc.update_imu(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);
      fc.update_encoder(t, 0.0, 0.0, 0.0, 1e-4, 1e-4, 1e-4);
      fc.update_zupt(t);
    }
  };
  {
    auto cfg = obsConfig();
    cfg.gps_rotation_heading_enabled = false;
    FusionCore fc(cfg);
    settle_bias(fc);
    ASSERT_EQ(fc.observability().gyro_bias_z, Observability::OBSERVABLE);
    EXPECT_EQ(fc.observability().next, ExcitationManoeuvre::DRIVE_STRAIGHT);
  }
  {
    auto cfg = obsConfig();
    cfg.gps_rotation_heading_enabled = true;
    FusionCore fc(cfg);
    settle_bias(fc);
    ASSERT_EQ(fc.observability().gyro_bias_z, Observability::OBSERVABLE);
    EXPECT_EQ(fc.observability().next, ExcitationManoeuvre::TURN_IN_PLACE_FAST);
  }
}

// The thresholds are constants in the header precisely so the doc and the code
// cannot drift. This fails if someone edits one without the other.
TEST(ObservabilityReport, TheDocumentedThresholdsAreTheOnesInUse)
{
  EXPECT_DOUBLE_EQ(kHeadingObservableDeg,     10.0);
  EXPECT_DOUBLE_EQ(kHeadingMarginalDeg,       30.0);
  EXPECT_DOUBLE_EQ(kWzBgzCorrelationLimit,    0.95);
  EXPECT_DOUBLE_EQ(kGyroBiasObservableSigma,  0.02);
  EXPECT_DOUBLE_EQ(kGyroBiasMarginalSigma,    0.05);
  EXPECT_LT(kHeadingObservableDeg, kHeadingMarginalDeg);
  EXPECT_LT(kGyroBiasObservableSigma, kGyroBiasMarginalSigma);
}

TEST(ObservabilityReport, TheStatusStructCarriesTheSameReport)
{
  FusionCore fc(obsConfig());
  initTight(fc);
  spin(fc, 0.6, 180);
  const auto direct = fc.observability();
  const auto via_status = fc.get_status().observability;
  EXPECT_EQ(via_status.heading,     direct.heading);
  EXPECT_EQ(via_status.gyro_bias_z, direct.gyro_bias_z);
  EXPECT_EQ(via_status.next,        direct.next);
  EXPECT_DOUBLE_EQ(via_status.wz_bgz_correlation, direct.wz_bgz_correlation);
}

// ─── The split is set by the bias prior, and the default was 100x too loose ──
//
// With n rate sensors each carrying its own bias, all measuring the same rate, and
// r = P_wz / P_bias, the minimum-variance split is
//
//     WZ fraction = n*r / (1 + n*r)        each bias fraction = 1 / (1 + n*r)
//
// Measured on this filter, where r works out to exactly 2 * P0_wz / P0_bias,
// confirmed across four orders of magnitude:
//
//   P0(bias)   WZ frac   implied r   r / (P0_wz/P0_bias)
//     0.001     0.9975    200.790          2.008
//     0.010     0.9756     20.010          2.001
//     0.050     0.8889      4.001          2.0005
//     0.100     0.8000      2.000          2.000
//     0.500     0.4445      0.400          2.000
//     2.000     0.1667      0.100          2.000
//
// The node initialised EVERY state at 0.1, so the gyro bias prior was a sigma of
// 0.316 rad/s, which is 18 deg/s. No MEMS gyro is that bad. Claiming it hands 20% of
// every yaw rate to the bias, and yaw integrates the rate alone, which is the whole
// of the 81.6% in #150.
//
// This does NOT make the pair observable. Only a zero-rate update or an absolute
// heading does that. It puts the split somewhere defensible until one arrives.

namespace {

// WZ's share of a known rate, after driving with both a gyro and an encoder.
double wzShare(double p0_bias, double rate = 0.6, int steps = 180)
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
      s.P(i, j) = (i == j) ? 1e-6 : 0.0;
  s.P(B_GZ, B_GZ)   = p0_bias;
  s.P(B_EWZ, B_EWZ) = p0_bias;
  fc.init(s, 0.0);
  for (int i = 1; i <= steps; ++i) {
    const double t = i * 0.01;
    fc.update_imu(t, 0.0, 0.0, rate, 0.0, 0.0, 9.80665);
    fc.update_encoder(t, 0.0, 0.0, rate, 1e-4, 1e-4, 1e-4);
  }
  return fc.get_state().x[WZ] / rate;
}

}  // namespace

TEST(ObservabilityReport, TheSplitFollowsTheCovarianceRatioFormula)
{
  // n = 2 (gyro and encoder), r = 2 * P0_wz / P0_bias, P0_wz = 0.1.
  for (double p0 : {0.001, 0.01, 0.05, 0.1, 0.5, 2.0}) {
    const double r = 2.0 * 0.1 / p0;
    const double predicted = 2.0 * r / (1.0 + 2.0 * r);
    EXPECT_NEAR(wzShare(p0), predicted, 0.01)
        << "P0(bias)=" << p0 << " predicted " << predicted;
  }
}

TEST(ObservabilityReport, ATighterBiasPriorGivesYawBackMostOfTheRate)
{
  // The direction of the effect, which is solid and is the reason the knob exists.
  // A driver that calibrates the gyro at startup can set 0.01 and get nearly all of
  // the rate back; the shipped default is deliberately looser, see below.
  EXPECT_NEAR(wzShare(0.1), 0.800, 0.01) << "the old blanket prior, pinned";
  EXPECT_GT(wzShare(0.01 * 0.01), 0.99)
      << "a tight prior, for a driver that actually calibrates, returns the rate";
}

// Why the SHIPPED default is loose rather than tight, measured in both directions.
//
// Datasheet initial zero-rate offset, which is what an uncalibrated driver hands the
// filter: MPU-6050 +/- 20 deg/s, ICM-20948 +/- 5 dps. Both are everywhere on hobby
// ROS robots. A 1.1 deg/s prior is tight and WRONG for either, and the cost measured
// at a 0.6 rad/s turn rate over 10 s before any stop was:
//
//   true bias  0 deg/s ->   +7.4 deg      true bias  5 deg/s ->  +31.4 deg
//   true bias  1 deg/s ->  +12.2 deg      true bias 20 deg/s -> +103.2 deg
//
// The loose error is bounded by the turn rate. The tight error is unbounded in the
// true bias. That asymmetry is the whole argument for the default.
TEST(ObservabilityReport, TheDefaultPriorSurvivesAnUncalibratedGyro)
{
  auto yaw_err_before_any_stop = [](double prior_sigma, double true_bias) {
    FusionCoreConfig cfg;
    cfg.outlier_rejection = false;
    cfg.adaptive_gnss = false;
    cfg.gps_track_heading_enabled = false;
    FusionCore fc(cfg);
    State s;
    s.P = StateMatrix::Identity() * 0.1;
    for (int i : {QW, QX, QY, QZ})
      for (int j : {QW, QX, QY, QZ})
        s.P(i, j) = (i == j) ? 1e-6 : 0.0;
    s.P(B_GZ, B_GZ)   = prior_sigma * prior_sigma;
    s.P(B_EWZ, B_EWZ) = prior_sigma * prior_sigma;
    fc.init(s, 0.0);
    double t = 0.0;
    for (int i = 1; i <= 1000; ++i) {
      t = i * 0.01;
      fc.update_imu(t, 0.0, 0.0, 0.6 + true_bias, 0.0, 0.0, 9.80665);
      fc.update_encoder(t, 0.0, 0.0, 0.6, 1e-4, 1e-4, 1e-4);
    }
    double yaw = fc.get_state().yaw();
    const double expect = 0.6 * t;
    while (yaw - expect >  M_PI) yaw -= 2 * M_PI;
    while (expect - yaw >  M_PI) yaw += 2 * M_PI;
    return std::abs(yaw - expect) * 180.0 / M_PI;
  };

  const double big_bias = 20.0 * M_PI / 180.0;     // a raw MPU-6050
  const double tight = yaw_err_before_any_stop(0.02,  big_bias);
  const double loose = yaw_err_before_any_stop(0.175, big_bias);

  // Measured: tight 103.2 deg, loose 77.2 deg. The loose prior IS better, which is
  // why it ships, but by 25% rather than by a lot.
  EXPECT_GT(tight, 90.0) << "the withdrawn 0.02 default, measured at 103 deg";
  EXPECT_LT(loose, tight) << "the shipped default must at least not be worse";

  // THE POINT, and it is the more useful half of this test: NEITHER prior saves you.
  // 77 degrees of yaw error in 10 s is not a working robot. A prior cannot fix an
  // uncalibrated gyro, it can only choose which way to be wrong. The fix is to
  // MEASURE the bias at boot, which robots can almost always do because they start
  // stationary, or to accept that nothing is trustworthy until the first stop.
  EXPECT_GT(loose, 30.0)
      << "if a prior alone ever makes this acceptable, the model changed and the "
         "boot-calibration argument needs revisiting; measured 77.2 deg";
}

// The false OK this prevents. With a tight bias prior the bias barely moves, so
// there is little covariance to correlate and r falls from -0.998 to -0.77 or lower,
// which a correlation-only test reads as observability. It is not: nothing has
// observed the bias, the prior merely started the split somewhere better. Measured
// verdicts with no ZUPT and no heading, which must ALL be UNOBSERVABLE:
//
//   P0(bias) 0.10000   WZ 0.8000   r -0.9982
//   P0(bias) 0.00040   WZ 0.9990   r -0.7686
//   P0(bias) 0.00001   WZ 1.0000   r -0.3727
TEST(ObservabilityReport, ATighterPriorDoesNotMakeThePairObservable)
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
      s.P(i, j) = (i == j) ? 1e-6 : 0.0;
  s.P(B_GZ, B_GZ) = 0.02 * 0.02;
  fc.init(s, 0.0);
  for (int i = 1; i <= 180; ++i) {
    const double t = i * 0.01;
    fc.update_imu(t, 0.0, 0.0, 0.6, 0.0, 0.0, 9.80665);
    fc.update_encoder(t, 0.0, 0.0, 0.6, 1e-4, 1e-4, 1e-4);
  }
  const auto r = fc.observability();
  EXPECT_LT(std::abs(r.wz_bgz_correlation), 0.95)
      << "the tight prior really does lower the correlation, which is the trap";
  EXPECT_EQ(r.gyro_bias_z, Observability::UNOBSERVABLE)
      << "nothing has observed the bias, so a low correlation must not read as OK";
  EXPECT_EQ(r.next, ExcitationManoeuvre::STOP_AND_WAIT);
}

// ─── Why the encoder bias prior is NOT the cheap fix for #150 ────────────────
//
// The hypothesis was good and half of it is confirmed. Encoders do not have a
// constant additive yaw-rate bias; their errors are scale (track width),
// speed-proportional (wheel radius) and slip events. Giving B_EWZ a free constant
// bias with a prior like the gyro's is what creates the null direction, so shrinking
// that prior should hand the disagreement back to the gyro bias where it belongs.
//
// It does, spectacularly, on a CLEAN encoder. 20 deg/s true gyro bias, 10 s at
// 0.6 rad/s, no stop and no GNSS:
//
//   P0(enc) 0.175     yaw err +77.17 deg   B_GZ 0.2297   B_EWZ -0.1193
//   P0(enc) 0.0175    yaw err  +9.20 deg   B_GZ 0.3465   B_EWZ -0.0025
//   P0(enc) 0.00175   yaw err  +7.74 deg   B_GZ 0.3490   B_EWZ -0.0000
//                                          true  0.3491
//
// B_GZ recovers the true bias to four decimals WHILE MOVING. So "the gyro bias is
// unobservable unless the robot stops" is an artifact of the encoder model, not a
// fact about the sensors. The prior does all the work: q_encoder_wz_bias from 1e-7
// to 1e-14 changes nothing.
//
// AND IT IS STILL NOT SAFE TO SHIP, because B_EWZ is load-bearing. It absorbs the
// encoder SCALE error, which at constant speed is indistinguishable from a constant
// bias. With NCLT's measured 1.297 over-report:
//
//   scenario                            P0enc .175   P0enc .00175
//   clean                                  -16.45        +7.72
//   gyro bias 20 dps, enc clean            +77.17        +7.74
//   NO gyro bias, enc scale 1.297          +31.48      +110.50   <- much worse
//   enc scale 1.297, slow turn 0.15         +8.69       +28.82
//   enc scale 1.297, fast turn 1.2         +55.61      -146.91   <- catastrophic
//
// The speed dependence is the tell: a constant-bias proxy for a scale error holds at
// one speed and falls apart across speeds, 8.69 deg at 0.15 rad/s against 55.61 at
// 1.2. Tighten the prior and nothing absorbs the scale at all.
//
// So the real fix is to model encoder error as a SCALE term plus a speed-proportional
// term plus gated slip events, rather than as a constant offset. That removes the
// null direction AND keeps the scale absorbed. Changing the prior alone trades one
// failure for a worse one, and these tests exist so that trade is not made by
// accident.

namespace {

// Yaw error in degrees after 10 s of turning at `rate`, no stop and no GNSS.
double yawErrDeg(double p0_enc, double gyro_bias_dps, double enc_scale, double rate)
{
  const double b = gyro_bias_dps * M_PI / 180.0;
  FusionCoreConfig cfg;
  cfg.outlier_rejection = false;
  cfg.adaptive_gnss = false;
  cfg.gps_track_heading_enabled = false;
  FusionCore fc(cfg);
  State s;
  s.P = StateMatrix::Identity() * 0.1;
  for (int i : {QW, QX, QY, QZ})
    for (int j : {QW, QX, QY, QZ})
      s.P(i, j) = (i == j) ? 1e-6 : 0.0;
  s.P(B_GZ, B_GZ)   = 0.175 * 0.175;
  s.P(B_EWZ, B_EWZ) = p0_enc * p0_enc;
  fc.init(s, 0.0);
  double t = 0.0;
  for (int i = 1; i <= 1000; ++i) {
    t = i * 0.01;
    fc.update_imu(t, 0.0, 0.0, rate + b, 0.0, 0.0, 9.80665);
    fc.update_encoder(t, 0.0, 0.0, rate * enc_scale, 1e-4, 1e-4, 1e-4);
  }
  double yaw = fc.get_state().yaw();
  const double e = rate * t;
  double y = yaw;
  while (y - e >  M_PI) y -= 2 * M_PI;
  while (e - y >  M_PI) y += 2 * M_PI;
  return (y - e) * 180.0 / M_PI;
}

}  // namespace

TEST(ObservabilityReport, ATightEncoderBiasPriorMakesTheGyroBiasObservableWhileMoving)
{
  // The confirmed half. On a clean encoder this is a 10x improvement and the gyro
  // bias is recovered to four decimals with no stop at all.
  EXPECT_GT(std::abs(yawErrDeg(0.175,   20.0, 1.0, 0.6)), 60.0) << "measured +77.17";
  EXPECT_LT(std::abs(yawErrDeg(0.00175, 20.0, 1.0, 0.6)), 15.0) << "measured  +7.74";
}

TEST(ObservabilityReport, ButTheEncoderBiasStateIsLoadBearingForScaleError)
{
  // The reason it cannot simply be tightened. B_EWZ absorbs the encoder scale error,
  // which at constant speed looks exactly like a constant bias. NCLT's encoder
  // over-reports rotation by 1.297.
  const double loose = std::abs(yawErrDeg(0.175,   0.0, 1.297, 0.6));
  const double tight = std::abs(yawErrDeg(0.00175, 0.0, 1.297, 0.6));
  EXPECT_LT(loose, tight)
      << "with a real scale error the LOOSE prior must win: loose " << loose
      << " deg vs tight " << tight << " deg (measured 31.48 vs 110.50)";
  EXPECT_GT(tight, 90.0) << "measured 110.50";
}

TEST(ObservabilityReport, TheConstantBiasProxyForScaleBreaksDownAcrossSpeeds)
{
  // The signature that says a scale error is being modelled as a constant offset:
  // it holds at one rate and falls apart at another. This is the argument for a
  // multiplicative term rather than a tighter prior.
  const double slow = std::abs(yawErrDeg(0.175, 0.0, 1.297, 0.15));
  const double fast = std::abs(yawErrDeg(0.175, 0.0, 1.297, 1.2));
  EXPECT_LT(slow, 20.0) << "measured  +8.69 at 0.15 rad/s";
  EXPECT_GT(fast, 40.0) << "measured +55.61 at 1.2 rad/s";
  EXPECT_GT(fast, slow * 3.0)
      << "a true constant bias would give a rate-independent error; this does not";
}
