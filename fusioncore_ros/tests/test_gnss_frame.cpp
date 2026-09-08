#include <gtest/gtest.h>

#include "fusioncore_ros/gnss_frame.hpp"

TEST(GnssFrameResolution, ConfiguredFrameWins)
{
  EXPECT_EQ(fusioncore_ros::resolve_gnss_frame("gps_override", "gps"), "gps_override");
}

TEST(GnssFrameResolution, MessageFrameUsedWhenNoOverride)
{
  EXPECT_EQ(fusioncore_ros::resolve_gnss_frame("", "gps"), "gps");
}

TEST(GnssFrameResolution, EmptyFrameRemainsUnknown)
{
  EXPECT_TRUE(fusioncore_ros::resolve_gnss_frame("", "").empty());
}

TEST(GnssFrameResolution, ExplicitBaseFrameIsPreserved)
{
  EXPECT_EQ(fusioncore_ros::resolve_gnss_frame("base_link", "gps"), "base_link");
}

TEST(GnssLeverArmTfAction, EmptyFrameMarksOneShotResolvedWithoutLookup)
{
  EXPECT_EQ(
    fusioncore_ros::gnss_lever_arm_tf_action("", "", "base_link"),
    fusioncore_ros::GnssLeverArmTfAction::MarkResolved);
}

TEST(GnssLeverArmTfAction, BaseFrameNeedsNoLookup)
{
  EXPECT_EQ(
    fusioncore_ros::gnss_lever_arm_tf_action("", "base_link", "base_link"),
    fusioncore_ros::GnssLeverArmTfAction::MarkResolved);
}

TEST(GnssLeverArmTfAction, MessageSensorFrameRequestsLookup)
{
  EXPECT_EQ(
    fusioncore_ros::gnss_lever_arm_tf_action("", "gps", "base_link"),
    fusioncore_ros::GnssLeverArmTfAction::Lookup);
}

TEST(GnssLeverArmTfAction, ConfiguredSensorFrameRequestsLookup)
{
  EXPECT_EQ(
    fusioncore_ros::gnss_lever_arm_tf_action("gps_override", "", "base_link"),
    fusioncore_ros::GnssLeverArmTfAction::Lookup);
}
