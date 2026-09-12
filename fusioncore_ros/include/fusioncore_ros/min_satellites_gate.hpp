// Decides whether gnss.min_satellites can ever be satisfied on this input.
//
// sensor_msgs/NavSatFix carries no satellite count, so the node substitutes a
// fixed value. A threshold above that constant is unsatisfiable: the filter then
// rejects 100% of its GNSS as MIN_SATS for the whole run, which reads as a
// receiver problem when it is a placeholder problem. gps_msgs/GPSFix carries a
// real status.satellites_used and is unaffected.
//
// Kept out of the node so the decision can be unit tested without standing up a
// lifecycle node, matching GnssDopGateWarning.

#ifndef FUSIONCORE_ROS__MIN_SATELLITES_GATE_HPP_
#define FUSIONCORE_ROS__MIN_SATELLITES_GATE_HPP_

namespace fusioncore_ros
{

// The count the NavSatFix path substitutes. Must track fusion_node.cpp.
constexpr int kNavSatFixSyntheticSatellites = 4;

inline bool min_satellites_is_unsatisfiable(bool using_navsatfix, int min_satellites)
{
  return using_navsatfix && min_satellites > kNavSatFixSyntheticSatellites;
}

}  // namespace fusioncore_ros

#endif  // FUSIONCORE_ROS__MIN_SATELLITES_GATE_HPP_
