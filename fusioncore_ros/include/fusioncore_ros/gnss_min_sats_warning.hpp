// Decides whether gnss.min_satellites can ever be met by the configured input.
//
// Kept separate from the ROS node so the decision can be unit tested without
// standing up a lifecycle node, the same way gnss_dop_gate_warning.hpp is.
// Stateless on purpose: on_configure can run again after a cleanup, and a
// one-shot latch would swallow the warning for the second run. The DOP warning
// latches because it fires from a per-message callback; this one does not.

#ifndef FUSIONCORE_ROS__GNSS_MIN_SATS_WARNING_HPP_
#define FUSIONCORE_ROS__GNSS_MIN_SATS_WARNING_HPP_

namespace fusioncore_ros
{

// sensor_msgs/NavSatFix has no satellite count field, so gnss_callback fills in
// this constant for every fix. A threshold above it can never be satisfied.
constexpr int kNavSatFixSyntheticSatellites = 4;

enum class MinSatsWarning
{
  kNone,
  // Every GNSS input in use is NavSatFix, so no fix can pass the gate.
  kEveryInput,
  // The primary is GPSFix and carries a real count; only the second receiver,
  // which is always NavSatFix, is gated out.
  kSecondReceiverOnly,
};

// gnss_callback serves the primary NavSatFix input (source 0) and the second
// receiver (source 1). The second receiver is NavSatFix whatever the primary
// is, which is why use_gps_fix alone does not clear the warning.
inline MinSatsWarning min_sats_warning(
  bool gnss_enabled,
  bool use_gps_fix,
  bool second_receiver_configured,
  int min_satellites)
{
  if (!gnss_enabled || min_satellites <= kNavSatFixSyntheticSatellites) {
    return MinSatsWarning::kNone;
  }
  if (!use_gps_fix) {
    return MinSatsWarning::kEveryInput;
  }
  return second_receiver_configured ?
    MinSatsWarning::kSecondReceiverOnly : MinSatsWarning::kNone;
}

}  // namespace fusioncore_ros

#endif  // FUSIONCORE_ROS__GNSS_MIN_SATS_WARNING_HPP_
