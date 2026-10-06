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
  EXPECT_EQ(r.next, ExcitationManoeuvre::FIGURE_EIGHT)
      << "if the bias is free, straight driving is not the answer";
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

// At startup the gyro bias is always unconstrained, so a figure-eight is correctly
// the first thing asked for: it covers straights, turns and a reversal, which is
// everything. Pinning this because it means the more specific manoeuvres only ever
// surface once the bias has settled, and a reader of the enum would not guess that.
TEST(ObservabilityReport, AtStartupTheAnswerIsAlwaysAFigureEight)
{
  for (bool rotation_enabled : {false, true}) {
    auto cfg = obsConfig();
    cfg.gps_rotation_heading_enabled = rotation_enabled;
    FusionCore fc(cfg);
    initTight(fc);
    EXPECT_EQ(fc.observability().next, ExcitationManoeuvre::FIGURE_EIGHT)
        << "rotation_enabled=" << rotation_enabled
        << ": an unconstrained bias outranks the heading path, because a figure-eight "
           "fixes both and the specific manoeuvres fix only one";
  }
}

// Once the bias IS constrained, the manoeuvre has to match the heading path that is
// actually enabled. Telling someone to turn in place when the rotation bootstrap is
// off is instructing them to do something that cannot help them.
TEST(ObservabilityReport, WithTheBiasSettledTheManoeuvreMatchesTheEnabledPath)
{
  auto tight_bias = [](FusionCore& fc) {
    State s;
    s.P = StateMatrix::Identity() * 0.1;
    for (int i : {QW, QX, QY, QZ})
      for (int j : {QW, QX, QY, QZ})
        s.P(i, j) = (i == j) ? 1e-6 : 0.0;
    s.P(B_GZ, B_GZ) = 1e-8;        // sigma 1e-4 rad/s: settled
    fc.init(s, 0.0);
  };
  {
    auto cfg = obsConfig();
    cfg.gps_rotation_heading_enabled = false;
    FusionCore fc(cfg);
    tight_bias(fc);
    ASSERT_EQ(fc.observability().gyro_bias_z, Observability::OBSERVABLE);
    EXPECT_EQ(fc.observability().next, ExcitationManoeuvre::DRIVE_STRAIGHT);
  }
  {
    auto cfg = obsConfig();
    cfg.gps_rotation_heading_enabled = true;
    FusionCore fc(cfg);
    tight_bias(fc);
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
