// Tracks the one-time warning that DOP thresholds are bypassed by covariance.
//
// Kept separate from the ROS node so the state transition can be unit tested
// without standing up a lifecycle node.

#ifndef FUSIONCORE_ROS__GNSS_DOP_GATE_WARNING_HPP_
#define FUSIONCORE_ROS__GNSS_DOP_GATE_WARNING_HPP_

namespace fusioncore_ros
{

class GnssDopGateWarning
{
public:
  bool should_warn(bool covariance_gate_active, double max_hdop, double max_vdop)
  {
    if (warned_ || !covariance_gate_active ||
        (max_hdop == kDefaultMaxHdop && max_vdop == kDefaultMaxVdop)) {
      return false;
    }

    warned_ = true;
    return true;
  }

private:
  static constexpr double kDefaultMaxHdop = 4.0;
  static constexpr double kDefaultMaxVdop = 6.0;
  bool warned_ = false;
};

}  // namespace fusioncore_ros

#endif  // FUSIONCORE_ROS__GNSS_DOP_GATE_WARNING_HPP_
