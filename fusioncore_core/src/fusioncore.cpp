#include "fusioncore/fusioncore.hpp"
#include "fusioncore/sensors/imu.hpp"
#include <stdexcept>
#include <cmath>

namespace fusioncore {

FusionCore::FusionCore(const FusionCoreConfig& config)
  : config_(config), ukf_(config.ukf)
{}

void FusionCore::init(const State& initial_state, double timestamp_seconds) {
  ukf_.init(initial_state);
  last_timestamp_    = timestamp_seconds;
  last_imu_time_     = -1.0;
  last_encoder_time_ = -1.0;
  last_gnss_time_    = -1.0;
  update_count_      = 0;
  initialized_       = true;
}

void FusionCore::reset() {
  initialized_       = false;
  last_timestamp_    = 0.0;
  last_imu_time_     = -1.0;
  last_encoder_time_ = -1.0;
  last_gnss_time_    = -1.0;
  update_count_      = 0;
}

void FusionCore::predict_to(double timestamp_seconds) {
  double dt = timestamp_seconds - last_timestamp_;
  if (dt < config_.min_dt) return;
  if (dt > config_.max_dt) {
    last_timestamp_ = timestamp_seconds;
    return;
  }
  ukf_.predict(dt);
  last_timestamp_ = timestamp_seconds;
}

void FusionCore::update_imu(
  double timestamp_seconds,
  double wx, double wy, double wz,
  double ax, double ay, double az
) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: update_imu() called before init()");

  predict_to(timestamp_seconds);

  sensors::ImuMeasurement z;
  z[0] = wx; z[1] = wy; z[2] = wz;
  z[3] = ax; z[4] = ay; z[5] = az;

  sensors::ImuNoiseMatrix R = sensors::imu_noise_matrix(config_.imu);
  ukf_.update<sensors::IMU_DIM>(z, sensors::imu_measurement_function, R);

  last_imu_time_ = timestamp_seconds;
  ++update_count_;
}

void FusionCore::update_imu_orientation(
  double timestamp_seconds,
  double roll, double pitch, double yaw,
  const double orientation_cov[9]
) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: update_imu_orientation() called before init()");

  predict_to(timestamp_seconds);

  sensors::ImuOrientationMeasurement z;
  z[0] = roll;
  z[1] = pitch;
  z[2] = yaw;

  sensors::ImuOrientationParams fallback;
  sensors::ImuOrientationNoiseMatrix R;

  if (orientation_cov != nullptr) {
    R = sensors::imu_orientation_noise_from_covariance(orientation_cov, fallback);
  } else {
    R = sensors::imu_orientation_noise_matrix(fallback);
  }

  ukf_.update<sensors::IMU_ORIENTATION_DIM>(
    z, sensors::imu_orientation_measurement_function, R);

  last_imu_time_ = timestamp_seconds;
  ++update_count_;
}

void FusionCore::update_encoder(
  double timestamp_seconds,
  double vx, double vy, double wz
) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: update_encoder() called before init()");

  predict_to(timestamp_seconds);

  sensors::EncoderMeasurement z;
  z[0] = vx; z[1] = vy; z[2] = wz;

  sensors::EncoderNoiseMatrix R = sensors::encoder_noise_matrix(config_.encoder);
  ukf_.update<sensors::ENCODER_DIM>(z, sensors::encoder_measurement_function, R);

  last_encoder_time_ = timestamp_seconds;
  ++update_count_;
}

bool FusionCore::update_gnss(
  double timestamp_seconds,
  const sensors::GnssFix& fix
) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: update_gnss() called before init()");

  // Reject poor quality fixes silently
  if (!fix.is_valid(config_.gnss)) return false;

  predict_to(timestamp_seconds);

  sensors::GnssPosMeasurement z;
  z[0] = fix.x;
  z[1] = fix.y;
  z[2] = fix.z;

  sensors::GnssPosNoiseMatrix R =
    sensors::gnss_pos_noise_matrix(config_.gnss, fix);

  // Use lever arm corrected measurement function if antenna is offset.
  // peci1 fix: only apply lever arm correction when yaw is sufficiently
  // converged. If yaw uncertainty is high, the lever arm rotation is
  // unreliable and can destabilize the filter.
  //
  // Yaw variance is P(YAW, YAW) — the diagonal element of the covariance.
  // Threshold of 0.1 rad² (~18 degrees std dev) is conservative enough
  // to avoid injecting bad corrections while still applying the correction
  // once the filter has a reasonable heading estimate.

  const double yaw_variance = ukf_.state().P(YAW, YAW);
  const double YAW_VARIANCE_THRESHOLD = 0.1;  // rad²

  if (config_.gnss.lever_arm.is_zero() || yaw_variance > YAW_VARIANCE_THRESHOLD) {
    // Either no lever arm configured, or yaw not yet converged — fuse at base_link
    if (!config_.gnss.lever_arm.is_zero() && yaw_variance > YAW_VARIANCE_THRESHOLD) {
      // Will be printed at most once per second via the caller
    }
    ukf_.update<sensors::GNSS_POS_DIM>(
      z, sensors::gnss_pos_measurement_function, R);
  } else {
    // Yaw is converged — apply lever arm correction
    auto h = sensors::gnss_pos_measurement_function_with_lever_arm(
      config_.gnss.lever_arm);
    ukf_.update<sensors::GNSS_POS_DIM>(z, h, R);
  }

  last_gnss_time_ = timestamp_seconds;
  ++update_count_;
  return true;
}

bool FusionCore::update_gnss_heading(
  double timestamp_seconds,
  const sensors::GnssHeading& heading
) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: update_gnss_heading() called before init()");

  if (!heading.valid) return false;

  predict_to(timestamp_seconds);

  sensors::GnssHdgMeasurement z;
  z[0] = heading.heading_rad;

  sensors::GnssHdgNoiseMatrix R =
    sensors::gnss_hdg_noise_matrix(config_.gnss, heading);

  ukf_.update<sensors::GNSS_HDG_DIM>(
    z, sensors::gnss_hdg_measurement_function, R
  );

  last_gnss_time_ = timestamp_seconds;
  ++update_count_;
  return true;
}

const State& FusionCore::get_state() const {
  return ukf_.state();
}

FusionCoreStatus FusionCore::get_status() const {
  FusionCoreStatus status;
  status.initialized  = initialized_;
  status.update_count = update_count_;

  if (!initialized_) return status;

  double stale = 1.0;

  status.imu_health =
    last_imu_time_ < 0.0 ? SensorHealth::NOT_INIT :
    (last_timestamp_ - last_imu_time_) > stale ? SensorHealth::STALE :
    SensorHealth::OK;

  status.encoder_health =
    last_encoder_time_ < 0.0 ? SensorHealth::NOT_INIT :
    (last_timestamp_ - last_encoder_time_) > stale ? SensorHealth::STALE :
    SensorHealth::OK;

  status.gnss_health =
    last_gnss_time_ < 0.0 ? SensorHealth::NOT_INIT :
    (last_timestamp_ - last_gnss_time_) > stale ? SensorHealth::STALE :
    SensorHealth::OK;

  const StateMatrix& P = ukf_.state().P;
  status.position_uncertainty = P(0,0) + P(1,1) + P(2,2);

  return status;
}

} // namespace fusioncore
