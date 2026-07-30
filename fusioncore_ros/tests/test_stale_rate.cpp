// The stale-rejection reporting policy from issue #73.
//
// The bug being locked down: the old code warned on ANY increase in the stale
// counter, so 2 dropped samples in a whole run produced the same alarm as a
// completely broken clock. These tests pin the two ends apart.

#include <gtest/gtest.h>
#include "fusioncore_ros/stale_rate_tracker.hpp"

using fusioncore_ros::StaleAlarm;
using fusioncore_ros::StaleRateTracker;

// The reported case: 2 rejections spread across a long run must not warn.
TEST(StaleRateTracker, TwoRejectionsInAWholeRunDoNotWarn)
{
  StaleRateTracker t;
  double rate = 0.0;

  EXPECT_EQ(t.update(0, 0, 0.0, rate), StaleAlarm::None);  // baseline

  // One rejection 83 s in, matching the reporter's first warning.
  EXPECT_EQ(t.update(1, 0, 83.0, rate), StaleAlarm::Debug);
  EXPECT_LT(rate, 1.0);

  // The second one nearly an hour later.
  EXPECT_EQ(t.update(2, 0, 3600.0, rate), StaleAlarm::Debug);
  EXPECT_LT(rate, 0.01);
}

// A genuine time-base mismatch rejects nearly every sample and must warn.
TEST(StaleRateTracker, EverySampleRejectedWarns)
{
  StaleRateTracker t;
  double rate = 0.0;

  EXPECT_EQ(t.update(0, 0, 0.0, rate), StaleAlarm::None);  // baseline

  // A 50 Hz IMU being rejected wholesale, sampled every 100 ms.
  int total = 0;
  for (int tick = 1; tick <= 5; ++tick) {
    total += 5;
    EXPECT_EQ(t.update(total, 0, tick * 0.1, rate), StaleAlarm::Warn);
    EXPECT_NEAR(rate, 50.0, 1e-6);
  }
}

// Rejections pending before the first update() are not swallowed by the
// baseline: they must surface on the next call rather than going silent.
TEST(StaleRateTracker, RejectionsBeforeFirstUpdateStillSurface)
{
  StaleRateTracker t;
  double rate = 0.0;

  // Clock broken from the start: 40 already rejected when diagnostics first run.
  EXPECT_EQ(t.update(40, 0, 1.0, rate), StaleAlarm::None);  // baseline only

  // One diagnostics period later, the backlog plus new ones is reported.
  EXPECT_EQ(t.update(80, 0, 1.1, rate), StaleAlarm::Warn);
  EXPECT_NEAR(rate, 800.0, 1e-6);  // 80 rejections over 0.1 s
}

// Nothing new means nothing said, however many times it is polled.
TEST(StaleRateTracker, QuietWhenNothingIsRejected)
{
  StaleRateTracker t;
  double rate = 0.0;

  EXPECT_EQ(t.update(0, 0, 0.0, rate), StaleAlarm::None);
  for (int i = 1; i < 100; ++i) {
    EXPECT_EQ(t.update(7, 3, i * 0.1, rate), i == 1 ? StaleAlarm::Warn : StaleAlarm::None);
  }
}

// ~/reset zeroes the counters. Without a re-sync the tracker would go blind
// until the new counts climbed past the pre-reset totals.
TEST(StaleRateTracker, SurvivesACounterReset)
{
  StaleRateTracker t;
  double rate = 0.0;

  EXPECT_EQ(t.update(0, 0, 0.0, rate), StaleAlarm::None);
  EXPECT_EQ(t.update(500, 0, 1.0, rate), StaleAlarm::Warn);

  // Filter reset: counters drop back to zero.
  EXPECT_EQ(t.update(0, 0, 2.0, rate), StaleAlarm::None);

  // A single new rejection after the reset is seen immediately, not ignored
  // until the count climbs back over 500.
  EXPECT_EQ(t.update(1, 0, 12.0, rate), StaleAlarm::Debug);
  EXPECT_NEAR(rate, 0.1, 1e-9);
}

// Both sensors feed the same rate, since either one lagging is the same problem.
TEST(StaleRateTracker, ImuAndEncoderRejectionsAreCombined)
{
  StaleRateTracker t;
  double rate = 0.0;

  EXPECT_EQ(t.update(0, 0, 0.0, rate), StaleAlarm::None);
  EXPECT_EQ(t.update(1, 1, 1.0, rate), StaleAlarm::Warn);
  EXPECT_NEAR(rate, 2.0, 1e-9);
}

// The boundary itself: exactly at the threshold warns, just under does not.
TEST(StaleRateTracker, ThresholdBoundaryIsInclusive)
{
  double rate = 0.0;
  {
    StaleRateTracker t;
    t.update(0, 0, 0.0, rate);
    EXPECT_EQ(t.update(1, 0, 1.0, rate), StaleAlarm::Warn);  // exactly 1.0/s
  }
  {
    StaleRateTracker t;
    t.update(0, 0, 0.0, rate);
    EXPECT_EQ(t.update(1, 0, 1.001, rate), StaleAlarm::Debug);  // just under
  }
}

// A custom threshold is honoured, so the policy is not hard-wired to 1 Hz.
TEST(StaleRateTracker, RespectsACustomThreshold)
{
  StaleRateTracker t(10.0);
  double rate = 0.0;

  EXPECT_EQ(t.update(0, 0, 0.0, rate), StaleAlarm::None);
  EXPECT_EQ(t.update(5, 0, 1.0, rate), StaleAlarm::Debug);   // 5/s, under 10
  EXPECT_EQ(t.update(25, 0, 2.0, rate), StaleAlarm::Warn);   // 20/s, over 10
}
