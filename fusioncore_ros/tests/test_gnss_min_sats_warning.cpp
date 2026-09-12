#include <gtest/gtest.h>

#include "fusioncore_ros/gnss_min_sats_warning.hpp"

using fusioncore_ros::MinSatsWarning;
using fusioncore_ros::min_sats_warning;

TEST(GnssMinSatsWarning, DefaultThresholdIsSatisfiedBySynthesisedCount)
{
  // 4 < 4 is false, which is why the default has never tripped this.
  EXPECT_EQ(min_sats_warning(true, false, false, 4), MinSatsWarning::kNone);
}

TEST(GnssMinSatsWarning, WarnsWhenNavSatFixCanNeverMeetTheThreshold)
{
  EXPECT_EQ(min_sats_warning(true, false, false, 5), MinSatsWarning::kEveryInput);
  EXPECT_EQ(min_sats_warning(true, false, false, 6), MinSatsWarning::kEveryInput);
}

TEST(GnssMinSatsWarning, LowerThresholdsStaySilent)
{
  EXPECT_EQ(min_sats_warning(true, false, false, 0), MinSatsWarning::kNone);
  EXPECT_EQ(min_sats_warning(true, false, false, 3), MinSatsWarning::kNone);
}

TEST(GnssMinSatsWarning, GpsFixCarriesARealCountSoNoWarning)
{
  EXPECT_EQ(min_sats_warning(true, true, false, 6), MinSatsWarning::kNone);
}

TEST(GnssMinSatsWarning, SecondReceiverIsNavSatFixEvenBehindGpsFix)
{
  // gnss_callback(msg, 1) synthesises the count for the second receiver too,
  // so a GPSFix primary does not make the threshold reachable for it.
  EXPECT_EQ(
    min_sats_warning(true, true, true, 6), MinSatsWarning::kSecondReceiverOnly);
}

TEST(GnssMinSatsWarning, SecondReceiverIsCoveredByTheEveryInputCase)
{
  // Primary NavSatFix plus a second receiver is still "every input".
  EXPECT_EQ(min_sats_warning(true, false, true, 6), MinSatsWarning::kEveryInput);
}

TEST(GnssMinSatsWarning, DisabledGnssNeedsNoWarning)
{
  EXPECT_EQ(min_sats_warning(false, false, false, 6), MinSatsWarning::kNone);
  EXPECT_EQ(min_sats_warning(false, true, true, 6), MinSatsWarning::kNone);
}
