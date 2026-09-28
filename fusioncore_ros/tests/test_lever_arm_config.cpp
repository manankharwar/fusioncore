// Locks the zero-versus-unset rule for lever arms. See #148: treating a deliberate
// zero as "unset" let the TF auto-resolve override it and cost 64.56 m of ATE on NCLT
// 2012-08-20, which survived a full 90 minute benchmark run because nothing said so.

#include <gtest/gtest.h>
#include <cmath>

#include "fusioncore_ros/lever_arm_config.hpp"

using fusioncore_ros::lever_arm_or_zero;
using fusioncore_ros::lever_arm_unset;
using fusioncore_ros::lever_arm_was_set;

TEST(LeverArmConfig, UnsetIsNotFinite)
{
  // The whole scheme rests on the default not being a usable number.
  EXPECT_FALSE(std::isfinite(lever_arm_unset()));
}

TEST(LeverArmConfig, NothingSetMeansUnset)
{
  const double u = lever_arm_unset();
  EXPECT_FALSE(lever_arm_was_set(u, u, u));
}

TEST(LeverArmConfig, DeliberateZeroCountsAsSet)
{
  // THE REGRESSION. A platform whose antenna really is at base_link origin must be
  // able to say so, and must not have TF override it.
  EXPECT_TRUE(lever_arm_was_set(0.0, 0.0, 0.0));
}

TEST(LeverArmConfig, OrdinaryValueCountsAsSet)
{
  EXPECT_TRUE(lever_arm_was_set(0.0, 0.0, 0.30));
  EXPECT_TRUE(lever_arm_was_set(-0.12, 0.0, 0.0));
}

TEST(LeverArmConfig, PartiallySetTreatsTheRestAsZero)
{
  const double u = lever_arm_unset();
  // Setting only x is a complete statement: y and z are zero, not unknown.
  EXPECT_TRUE(lever_arm_was_set(0.25, u, u));
  EXPECT_DOUBLE_EQ(lever_arm_or_zero(0.25), 0.25);
  EXPECT_DOUBLE_EQ(lever_arm_or_zero(u), 0.0);
}

TEST(LeverArmConfig, ZeroSurvivesTheConversion)
{
  // A zero the user typed must come out as zero, not as the unset marker again.
  EXPECT_DOUBLE_EQ(lever_arm_or_zero(0.0), 0.0);
  EXPECT_FALSE(std::isnan(lever_arm_or_zero(0.0)));
}

TEST(LeverArmConfig, TheOldRuleWouldHaveFailedThis)
{
  // Documents precisely what was wrong, so a future refactor cannot quietly restore
  // it: the old test was `!is_zero()` with a 1e-6 tolerance, which is false for a
  // deliberate zero and therefore indistinguishable from absent.
  const bool old_rule_says_explicit =
    !(std::abs(0.0) < 1e-6 && std::abs(0.0) < 1e-6 && std::abs(0.0) < 1e-6);
  EXPECT_FALSE(old_rule_says_explicit);
  EXPECT_TRUE(lever_arm_was_set(0.0, 0.0, 0.0));
}
