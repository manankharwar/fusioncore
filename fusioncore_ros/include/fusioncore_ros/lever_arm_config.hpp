#ifndef FUSIONCORE_ROS__LEVER_ARM_CONFIG_HPP_
#define FUSIONCORE_ROS__LEVER_ARM_CONFIG_HPP_

#include <cmath>
#include <limits>

namespace fusioncore_ros
{

// "Unset" for a lever arm has to be distinguishable from "deliberately zero".
//
// It was not, and it cost 64.56 m of ATE on NCLT 2012-08-20 (#148). The explicit
// flag was `!lever_arm.is_zero()` with a 1e-6 tolerance, so a config setting 0,0,0 on
// purpose read as "the user said nothing" and the TF auto-resolve added z=0.300,
// overriding the intent. The NCLT config does exactly that, with a comment explaining
// why zero is correct for that platform, and there was no way to express it.
//
// The rule: the parameter default is NaN, and ANY finite component means the user
// spoke. Zero is then an ordinary value. A NaN component beside a finite one reads as
// 0.0, which is what setting only lever_arm_x means.
//
// The general lesson, worth applying to any other parameter where zero is meaningful:
// do not use a legitimate value as the sentinel for absent.
inline constexpr double lever_arm_unset()
{
  return std::numeric_limits<double>::quiet_NaN();
}

// True when the user set at least one component, including setting it to zero.
inline bool lever_arm_was_set(double x, double y, double z)
{
  return std::isfinite(x) || std::isfinite(y) || std::isfinite(z);
}

// A component the user did not set contributes nothing.
inline double lever_arm_or_zero(double v)
{
  return std::isfinite(v) ? v : 0.0;
}

}  // namespace fusioncore_ros

#endif  // FUSIONCORE_ROS__LEVER_ARM_CONFIG_HPP_
