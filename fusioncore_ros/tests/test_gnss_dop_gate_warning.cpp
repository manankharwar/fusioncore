#include <gtest/gtest.h>

#include "fusioncore_ros/gnss_dop_gate_warning.hpp"

TEST(GnssDopGateWarning, IgnoresDefaultDopThresholds)
{
  fusioncore_ros::GnssDopGateWarning warning;

  EXPECT_FALSE(warning.should_warn(true, 4.0, 6.0));
}

TEST(GnssDopGateWarning, WarnsOnceForTunedHdopWithCovariance)
{
  fusioncore_ros::GnssDopGateWarning warning;

  EXPECT_TRUE(warning.should_warn(true, 2.0, 6.0));
  EXPECT_FALSE(warning.should_warn(true, 2.0, 6.0));
}

TEST(GnssDopGateWarning, WarnsForTunedVdopWithCovariance)
{
  fusioncore_ros::GnssDopGateWarning warning;

  EXPECT_TRUE(warning.should_warn(true, 4.0, 3.0));
}

TEST(GnssDopGateWarning, WaitsUntilCovarianceActivatesTheSigmaGate)
{
  fusioncore_ros::GnssDopGateWarning warning;

  EXPECT_FALSE(warning.should_warn(false, 2.0, 6.0));
  EXPECT_TRUE(warning.should_warn(true, 2.0, 6.0));
}