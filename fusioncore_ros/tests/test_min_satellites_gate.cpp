#include <gtest/gtest.h>

#include "fusioncore_ros/min_satellites_gate.hpp"

using fusioncore_ros::min_satellites_is_unsatisfiable;

// NavSatFix has no satellite count, so the node substitutes a constant 4.
TEST(MinSatellitesGate, DefaultIsSatisfiableOnNavSatFix)
{
  EXPECT_FALSE(min_satellites_is_unsatisfiable(true, 4));
}

TEST(MinSatellitesGate, AboveTheSyntheticCountNothingCanEverPass)
{
  EXPECT_TRUE(min_satellites_is_unsatisfiable(true, 5));
  EXPECT_TRUE(min_satellites_is_unsatisfiable(true, 6));
  EXPECT_TRUE(min_satellites_is_unsatisfiable(true, 12));
}

TEST(MinSatellitesGate, BelowTheSyntheticCountIsFine)
{
  EXPECT_FALSE(min_satellites_is_unsatisfiable(true, 0));
  EXPECT_FALSE(min_satellites_is_unsatisfiable(true, 3));
}

// GPSFix carries status.satellites_used, so any threshold is meaningful there.
TEST(MinSatellitesGate, GpsFixInputIsNeverUnsatisfiable)
{
  EXPECT_FALSE(min_satellites_is_unsatisfiable(false, 6));
  EXPECT_FALSE(min_satellites_is_unsatisfiable(false, 12));
}
