// Measure the inter-sensor time offset instead of only rejecting on it.
//
// FusionCore has always GUARDED against clock disagreement: reject_stale_from_skew()
// drops a measurement whose stamp lags the filter clock by more than
// max_measurement_delay, and DELAY_TOO_LARGE counts it. That is safe and it throws
// the data away. Nothing here ever measured the offset so it could be corrected.
//
// It is measurable with no extra hardware, because the gyro's z rate and a
// differential drive's encoder yaw rate are the same signal observed twice, so the
// lag that maximises their agreement IS the offset.
//
// The numbers asserted below are measured, not chosen.

#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/time_offset.hpp"
#include <cmath>
#include <random>

using namespace fusioncore;

namespace {

// Sum of two incommensurate sines, so the signal is not periodic at the grid step.
// A single sine would admit a false peak one period away and the test would pass for
// the wrong reason.
double turnProfile(double t)
{
  return 0.6 * std::sin(0.7 * t) + 0.3 * std::sin(1.9 * t + 0.4);
}

TimeOffsetEstimator::Result runPair(double truth, double noise = 0.0,
                                    double scale = 1.0, double sign = 1.0,
                                    int steps = 3000)
{
  TimeOffsetEstimator est(0.5, 100.0, 10.0);
  std::mt19937 gen(1);
  std::normal_distribution<double> nd(0.0, noise);
  for (int i = 0; i < steps; ++i) {
    const double t = i * 0.01;
    const double w = turnProfile(t);
    est.add_imu(t, w + (noise > 0.0 ? nd(gen) : 0.0));
    est.add_encoder(t + truth, sign * scale * w + (noise > 0.0 ? nd(gen) : 0.0));
  }
  return est.estimate();
}

}  // namespace

TEST(TimeOffset, RecoversAKnownOffsetExactlyOnCleanData)
{
  for (double truth : {0.0, 0.05, -0.08, 0.12, 0.25}) {
    const auto r = runPair(truth);
    ASSERT_TRUE(r.valid) << "truth " << truth;
    EXPECT_NEAR(r.offset_s, truth, 1e-3) << "truth " << truth;
    EXPECT_GT(r.correlation, 0.99);
  }
}

TEST(TimeOffset, SurvivesRealisticGyroNoise)
{
  const auto r = runPair(0.08, 0.03);
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.offset_s, 0.08, 0.005) << "measured 0.8 ms of error on this commit";
  EXPECT_GT(r.correlation, 0.99);
}

// The property that makes this usable on a robot whose encoder scale is wrong, which
// is most of them. Correlation is normalised, so a magnitude error does not move the
// peak. 1.285x is the real ratio measured on NCLT in issue #169.
TEST(TimeOffset, IsScaleInvariant)
{
  const auto r = runPair(0.08, 0.03, /*scale=*/1.285);
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.offset_s, 0.08, 0.005)
      << "a 28.5% scale error must not move the timing estimate";
}

// Anti-correlation is a diagnosis, not a failure. It is the #169 signature: one of
// the two sources has its frame convention wrong, and no offset fixes that.
TEST(TimeOffset, DiagnosesASignFlipInsteadOfGuessingAnOffset)
{
  const auto r = runPair(0.08, 0.03, 1.0, /*sign=*/-1.0);
  EXPECT_FALSE(r.valid);
  EXPECT_TRUE(r.anti_correlated);
  EXPECT_LT(r.worst_correlation, -0.5) << "measured -0.83 on this commit";
  EXPECT_DOUBLE_EQ(r.offset_s, 0.0)
      << "an unbelieved estimate must report no offset, not the edge of the search";
}

TEST(TimeOffset, ASignFlipIsStillDiagnosedWhenTheScaleIsAlsoWrong)
{
  const auto r = runPair(0.08, 0.03, 1.285, -1.0);
  EXPECT_FALSE(r.valid);
  EXPECT_TRUE(r.anti_correlated);
}

// A robot driving down a row carries no timing information: every lag fits a constant
// equally well. Refusing is the honest answer and it is also the common case.
TEST(TimeOffset, RefusesWhenNothingIsTurning)
{
  TimeOffsetEstimator est;
  for (int i = 0; i < 3000; ++i) {
    const double t = i * 0.01;
    est.add_imu(t, 0.0);
    est.add_encoder(t, 0.0);
  }
  const auto r = est.estimate();
  EXPECT_FALSE(r.valid);
  EXPECT_FALSE(r.anti_correlated) << "flat is not anti-correlated, it is uninformative";
}

TEST(TimeOffset, RefusesOnTooShortAWindow)
{
  // 3 s of data against a 10 s minimum. A short window finds a confident peak in
  // noise, which is the failure mode that matters.
  const auto r = runPair(0.08, 0.0, 1.0, 1.0, /*steps=*/300);
  EXPECT_FALSE(r.valid);
  EXPECT_LT(r.span_s, 10.0);
}

TEST(TimeOffset, RefusesBeforeAnyDataArrives)
{
  TimeOffsetEstimator est;
  const auto r = est.estimate();
  EXPECT_FALSE(r.valid);
  EXPECT_EQ(r.samples, 0);
}

TEST(TimeOffset, NonMonotonicStampsAreDroppedRatherThanCorrupting)
{
  TimeOffsetEstimator est;
  est.add_imu(1.0, 0.1);
  est.add_imu(0.5, 0.2);        // goes backwards
  est.add_imu(2.0, 0.3);
  EXPECT_EQ(est.imu_samples(), 2u);
}

TEST(TimeOffset, TheWindowIsBoundedSoAnEstimateTracksADriftingClock)
{
  TimeOffsetEstimator est(0.5, 100.0, 10.0);
  for (int i = 0; i < 100000; ++i) est.add_imu(i * 0.01, turnProfile(i * 0.01));
  // 1000 s fed, window is 4x the 10 s minimum, so it must not have kept everything.
  EXPECT_LT(est.imu_samples(), 5000u);
  EXPECT_GT(est.imu_samples(), 1000u);
}

// ---- integration with the filter ----

TEST(TimeOffset, TheFilterReportsAnEstimateInStatus)
{
  FusionCoreConfig cfg;
  cfg.outlier_rejection = false;
  cfg.adaptive_gnss = false;
  cfg.gps_track_heading_enabled = false;
  cfg.time_offset_estimate_interval_s = 1.0;
  FusionCore fc(cfg);
  State s;
  s.P = StateMatrix::Identity() * 0.1;
  fc.init(s, 0.0);

  const double truth = 0.08;
  for (int i = 1; i <= 3000; ++i) {
    const double t = i * 0.01;
    const double w = turnProfile(t);
    fc.update_imu(t, 0.0, 0.0, w, 0.0, 0.0, 9.81);
    fc.update_encoder(t + truth, 0.0, 0.0, w, 1e-4, 1e-4, 1e-4);
  }
  const auto r = fc.get_status().time_offset;
  ASSERT_TRUE(r.valid) << "corr " << r.correlation << " span " << r.span_s;
  EXPECT_NEAR(r.offset_s, truth, 0.02);
}

TEST(TimeOffset, SettingTheIntervalToZeroDisablesTheEstimator)
{
  FusionCoreConfig cfg;
  cfg.outlier_rejection = false;
  cfg.adaptive_gnss = false;
  cfg.gps_track_heading_enabled = false;
  cfg.time_offset_estimate_interval_s = 0.0;
  FusionCore fc(cfg);
  State s;
  s.P = StateMatrix::Identity() * 0.1;
  fc.init(s, 0.0);
  for (int i = 1; i <= 2000; ++i) {
    const double t = i * 0.01;
    fc.update_imu(t, 0.0, 0.0, turnProfile(t), 0.0, 0.0, 9.81);
    fc.update_encoder(t, 0.0, 0.0, turnProfile(t), 1e-4, 1e-4, 1e-4);
  }
  EXPECT_FALSE(fc.get_status().time_offset.valid);
  EXPECT_EQ(fc.get_status().time_offset.samples, 0);
}

// Applying is a separate, manual decision from estimating, and it must actually
// shift the stream.
TEST(TimeOffset, TheConfiguredOffsetIsAppliedToEncoderStamps)
{
  auto drive = [](double applied) {
    FusionCoreConfig cfg;
    cfg.outlier_rejection = false;
    cfg.adaptive_gnss = false;
    cfg.gps_track_heading_enabled = false;
    cfg.time_offset_estimate_interval_s = 1.0;
    cfg.encoder_time_offset = applied;
    auto fc = std::make_unique<FusionCore>(cfg);
    State s;
    s.P = StateMatrix::Identity() * 0.1;
    fc->init(s, 0.0);
    const double truth = 0.08;
    for (int i = 1; i <= 3000; ++i) {
      const double t = i * 0.01;
      const double w = turnProfile(t);
      fc->update_imu(t, 0.0, 0.0, w, 0.0, 0.0, 9.81);
      fc->update_encoder(t + truth, 0.0, 0.0, w, 1e-4, 1e-4, 1e-4);
    }
    return fc->get_status().time_offset;
  };
  const auto before = drive(0.0);
  // The reported offset is what to ADD to align, so applying its negation should
  // drive the remaining offset toward zero.
  const auto after = drive(-before.offset_s);
  ASSERT_TRUE(before.valid);
  ASSERT_TRUE(after.valid);
  EXPECT_LT(std::abs(after.offset_s), std::abs(before.offset_s) * 0.5)
      << "applying the correction did not reduce the measured offset: "
      << before.offset_s << " -> " << after.offset_s;
}
