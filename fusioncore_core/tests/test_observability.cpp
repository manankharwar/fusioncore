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
  // The practical consequence. 0.1 is the old blanket prior, 0.0004 is the square of
  // the new 0.02 rad/s default.
  const double old_default = wzShare(0.1);
  const double new_default = wzShare(0.02 * 0.02);
  EXPECT_NEAR(old_default, 0.800, 0.01) << "the prior behaviour, pinned";
  EXPECT_GT(new_default, 0.99)
      << "a physically defensible gyro bias prior should return nearly all of the "
         "rate to WZ; got " << new_default;
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
