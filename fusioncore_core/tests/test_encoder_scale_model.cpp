// The encoder yaw-error model: a multiplicative SCALE instead of an additive bias.
//
// An encoder has no constant additive yaw-rate offset. Its errors are scale (track
// width), speed-proportional (wheel radius) and slip events. Modelling scale as a
// bias creates a null direction that the GYRO bias hides in: the gyro reads WZ + b_g
// and the encoder reads WZ + b_ewz, which is two equations and three unknowns.
//
// The scale model removes it, and the reason is not obvious. ON A STRAIGHT DRIVE the
// encoder reads zero yaw rate whatever the scale is, because (1 + s) * 0 = 0. So
// every straight stretch is a KNOWN ZERO for WZ, and the gyro's reading there is its
// bias. The robot never has to stop. With the additive model the same stretch gives
// WZ = -b_ewz, which pins nothing.
//
// Five predictions were written down BEFORE running any of this. All five held.
// Measured over 30 s of alternating 3 s straight and 3 s turning:
//
//   [P1] gyro bias 20 dps, clean encoder     bias -119.12 deg   scale   +9.61 deg
//   [P2] enc scale 1.297, turn 0.15          bias  +45.51       scale   +3.17
//                         turn 0.60          bias  +66.68       scale   +9.62
//                         turn 1.20          bias  -47.51       scale   +7.66
//   [P3] both errors together                s_e recovered 0.30 against a true 0.297
//   [P4] constant-rate circle, no straight   bias  -56.98       scale -145.53
//   [P5] slip injected, chi2 gate on         s_e 0.30 with slip, 0.30 without
//
// P4 is an EXPECTED FAILURE and was predicted as one. On a circle at a single rate
// there is no straight stretch, so there is no known zero, and two equations cannot
// resolve three unknowns. It is included because a probe that only drove circles
// would "show" this model failing, which is the single-condition mistake a third time.

#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "sweep.hpp"
#include <cmath>

using namespace fusioncore;

namespace {

struct Result { double yaw_err_deg; double b_gz; double s_e; };

// motion: false = alternating 3 s straight / 3 s turning. true = constant circle.
Result drive(bool scale_model, double gyro_bias_dps, double enc_scale,
             double turn_rate, bool circle_only = false, bool slip = false)
{
  const double b = gyro_bias_dps * M_PI / 180.0;
  FusionCoreConfig cfg;
  cfg.outlier_rejection = true;        // the chi2 gate is what rejects slip
  cfg.adaptive_gnss = false;
  cfg.gps_track_heading_enabled = false;
  cfg.encoder_yaw_scale_model = scale_model;
  FusionCore fc(cfg);
  State s;
  s.P = StateMatrix::Identity() * 0.1;
  for (int i : {QW, QX, QY, QZ})
    for (int j : {QW, QX, QY, QZ})
      s.P(i, j) = (i == j) ? 1e-6 : 0.0;
  s.P(B_GZ, B_GZ)   = 0.175 * 0.175;
  s.P(B_EWZ, B_EWZ) = scale_model ? 0.30 * 0.30 : 0.175 * 0.175;
  fc.init(s, 0.0);

  double t = 0.0, true_yaw = 0.0;
  for (int i = 1; i <= 3000; ++i) {
    t = i * 0.01;
    const double w = circle_only ? turn_rate : (((i / 300) % 2) ? turn_rate : 0.0);
    const bool slipping = slip && i > 1500 && i < 1600;
    fc.update_imu(t, 0.0, 0.0, w + b, 0.0, 0.0, 9.80665);
    fc.update_encoder(t, slipping ? 3.0 : 1.0, 0.0,
                      slipping ? w * 3.0 : w * enc_scale, 1e-4, 1e-4, 1e-4);
    true_yaw += w * 0.01;
  }
  double yaw = fc.get_state().yaw();
  while (yaw - true_yaw >  M_PI) yaw -= 2 * M_PI;
  while (true_yaw - yaw >  M_PI) yaw += 2 * M_PI;
  const auto& x = fc.get_state().x;
  return {(yaw - true_yaw) * 180.0 / M_PI, x[B_GZ], x[B_EWZ]};
}

}  // namespace

// [P1] A gyro bias no longer hides in the encoder state.
TEST(EncoderScaleModel, AGyroBiasNoLongerHidesInTheEncoderState)
{
  const double bias_model  = std::abs(drive(false, 20.0, 1.0, 0.6).yaw_err_deg);
  const double scale_model = std::abs(drive(true,  20.0, 1.0, 0.6).yaw_err_deg);
  EXPECT_GT(bias_model, 90.0)  << "the additive model, measured 119.12 deg";
  EXPECT_LT(scale_model, 20.0) << "the scale model, measured 9.61 deg";
  EXPECT_LT(drive(true, 20.0, 1.0, 0.6).s_e, 0.05)
      << "a clean encoder must leave the scale state near zero";
}

// [P2] The scale is recovered, and the error stops depending on turn rate. Run as a
// GRID, because a single rate is exactly what hid this class of bug twice today.
TEST(EncoderScaleModel, TheScaleIsRecoveredAndTheErrorStopsDependingOnRate)
{
  using namespace fusioncore::sweep;
  Sweep s;
  s.axis("rate", {0.15, 0.6, 1.2}).axis("enc_scale", {1.0, 1.297});
  const auto cells = s.run([](const Point& p) {
    return std::abs(drive(true, 0.0, p.at("enc_scale"), p.at("rate")).yaw_err_deg);
  });
  ASSERT_TRUE(s.covers(cells)) << "the grid must vary every axis and include a real error";
  Sweep::report("scale model, yaw error by rate and encoder scale", cells);

  double worst = 0.0;
  for (const auto& c : cells) worst = std::max(worst, c.value);
  EXPECT_LT(worst, 25.0)
      << "the scale model must stay bounded across rates; the additive model gave "
         "+45.51, +66.68 and -47.51 at the same three rates";

  for (double rate : {0.15, 0.6, 1.2}) {
    EXPECT_NEAR(drive(true, 0.0, 1.297, rate).s_e, 0.297, 0.03)
        << "scale must be recovered at rate " << rate;
  }
}

// [P3] Both errors at once, and the states are what must be checked rather than yaw.
TEST(EncoderScaleModel, BothErrorsTogetherAreSeparated)
{
  const auto r = drive(true, 20.0, 1.297, 0.6);
  EXPECT_NEAR(r.s_e, 0.297, 0.03) << "encoder scale recovered";
  // Deliberately NOT asserting that the yaw number beats the additive model here.
  // Measured, the additive model got +4.79 deg against the scale model's +11.92,
  // purely because a wrong gyro bias and a wrong encoder bias happened to cancel.
  // Its b_gz was +0.04 against a true +0.349, so it recovered nothing. Judging a
  // model on one yaw number is how that kind of luck gets mistaken for accuracy.
  const auto additive = drive(false, 20.0, 1.297, 0.6);
  EXPECT_GT(std::abs(additive.b_gz - 20.0 * M_PI / 180.0), 0.2)
      << "the additive model's yaw number is luck, not a recovered state";
}

// [P4] THE EXPECTED FAILURE, predicted in advance. No straight stretch means no known
// zero, so two equations cannot resolve three unknowns and the scale model is worse.
TEST(EncoderScaleModel, DISABLED_AConstantRateCircleIsStillUnobservable)
{
  const auto r = drive(true, 20.0, 1.297, 0.6, /*circle_only=*/true);
  EXPECT_NEAR(r.s_e, 0.297, 0.05)
      << "this is the acceptance criterion if a future model ever handles a circle; "
         "today it measures -0.02 against a true 0.297";
}

TEST(EncoderScaleModel, CharacteriseThatACircleOnlyRunDefeatsBothModels)
{
  const auto scale    = drive(true,  20.0, 1.297, 0.6, true);
  const auto additive = drive(false, 20.0, 1.297, 0.6, true);
  EXPECT_GT(std::abs(scale.yaw_err_deg), 90.0)    << "measured -145.53 deg";
  EXPECT_GT(std::abs(additive.yaw_err_deg), 30.0) << "measured  -56.98 deg";
  EXPECT_GT(std::abs(scale.s_e - 0.297), 0.2)
      << "with no straight stretch the scale is not recoverable; measured -0.02";
}

// [P5] Slip must not move the scale. The existing chi2 gate already does this, which
// is worth pinning because the first run of this prediction appeared to fail purely
// because the probe had outlier_rejection off.
TEST(EncoderScaleModel, SlipDoesNotMoveTheScaleWhenTheGateIsOn)
{
  const double with_slip    = drive(true, 0.0, 1.297, 0.6, false, true).s_e;
  const double without_slip = drive(true, 0.0, 1.297, 0.6, false, false).s_e;
  EXPECT_NEAR(with_slip, without_slip, 0.02)
      << "a 1 s slip burst moved the scale from " << without_slip << " to " << with_slip;
  EXPECT_NEAR(with_slip, 0.297, 0.03);
}

TEST(EncoderScaleModel, TheAdditiveModelRemainsTheDefault)
{
  FusionCoreConfig cfg;
  EXPECT_FALSE(cfg.encoder_yaw_scale_model)
      << "the scale model is unvalidated on real data; it must stay opt-in until the "
         "benchmark says otherwise";
}
