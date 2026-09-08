#pragma once

#include <string>

namespace fusioncore_ros
{

enum class GnssLeverArmTfAction
{
  Lookup,
  MarkResolved,
};

inline std::string resolve_gnss_frame(
  const std::string & configured_frame,
  const std::string & message_frame)
{
  if (!configured_frame.empty()) {
    return configured_frame;
  }
  return message_frame;
}

inline GnssLeverArmTfAction gnss_lever_arm_tf_action(
  const std::string & configured_frame,
  const std::string & message_frame,
  const std::string & base_frame)
{
  const std::string frame = resolve_gnss_frame(configured_frame, message_frame);
  if (frame.empty() || frame == base_frame) {
    return GnssLeverArmTfAction::MarkResolved;
  }
  return GnssLeverArmTfAction::Lookup;
}

}  // namespace fusioncore_ros
