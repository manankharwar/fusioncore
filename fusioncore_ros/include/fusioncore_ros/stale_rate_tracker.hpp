// Decides how loudly to report stale sensor rejections.
//
// A stale rejection means a sample's stamp lagged the filter clock by more than
// max_measurement_delay, so it could not be retrodicted and was not fused. The
// raw count cannot tell the two causes apart. A few stragglers on a wireless
// link are harmless. A genuine time-base mismatch rejects essentially every
// sample and shows up as tens per second. Reporting both the same way is what
// made a user chase a clock problem they had already fixed (issue #73), so the
// decision keys off the rate instead.
//
// Lives in its own header so the policy can be unit tested without standing up
// a whole ROS node.

#ifndef FUSIONCORE_ROS__STALE_RATE_TRACKER_HPP_
#define FUSIONCORE_ROS__STALE_RATE_TRACKER_HPP_

namespace fusioncore_ros
{

enum class StaleAlarm
{
  None,   // nothing new to say
  Debug,  // a sample was dropped, but at a rate that is normal
  Warn    // sustained: a sensor is effectively not being fused
};

class StaleRateTracker
{
public:
  explicit StaleRateTracker(double warn_rate_hz = 1.0)
  : warn_rate_hz_(warn_rate_hz) {}

  // Feed the running totals and the current time. Writes the measured rate in
  // rejections per second and returns how it should be reported.
  StaleAlarm update(int imu_total, int encoder_total, double now_s, double & rate_out)
  {
    rate_out = 0.0;

    // First call establishes the time baseline but deliberately does NOT absorb
    // the counts. Anything already rejected stays pending and is reported on the
    // next call with a real span behind it, so a clock that is broken from the
    // very first sample still warns promptly instead of being swallowed.
    if (last_eval_s_ < 0.0) {
      last_eval_s_ = now_s;
      return StaleAlarm::None;
    }

    const int new_stale = (imu_total - prev_imu_) + (encoder_total - prev_encoder_);

    // Counters went backward, so the filter was reset under us. Re-sync, or the
    // previous totals stay above the live ones and real rejections stay invisible
    // until they climb past the pre-reset count.
    if (new_stale < 0) {
      prev_imu_ = imu_total;
      prev_encoder_ = encoder_total;
      last_eval_s_ = now_s;
      return StaleAlarm::None;
    }

    if (new_stale == 0) {
      return StaleAlarm::None;
    }

    const double span = now_s - last_eval_s_;
    rate_out = (span > 1e-6) ? (new_stale / span) : 0.0;

    prev_imu_ = imu_total;
    prev_encoder_ = encoder_total;
    last_eval_s_ = now_s;

    return (rate_out >= warn_rate_hz_) ? StaleAlarm::Warn : StaleAlarm::Debug;
  }

  double warn_rate_hz() const { return warn_rate_hz_; }

private:
  double warn_rate_hz_;
  int    prev_imu_     = 0;
  int    prev_encoder_ = 0;
  double last_eval_s_  = -1.0;  // negative until the first update()
};

}  // namespace fusioncore_ros

#endif  // FUSIONCORE_ROS__STALE_RATE_TRACKER_HPP_
