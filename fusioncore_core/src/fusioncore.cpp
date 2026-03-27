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

  // Reset heading observability
  heading_validated_ = false;
  heading_source_    = HeadingSource::NONE;
  gnss_pos_set_      = false;
  distance_traveled_ = 0.0;
  last_gnss_x_       = 0.0;
  last_gnss_y_       = 0.0;
}

void FusionCore::reset() {
  initialized_       = false;
  last_timestamp_    = 0.0;
  last_imu_time_     = -1.0;
  last_encoder_time_ = -1.0;
  last_gnss_time_    = -1.0;
  update_count_      = 0;
  heading_validated_ = false;
  heading_source_    = HeadingSource::NONE;
  gnss_pos_set_      = false;
  distance_traveled_ = 0.0;
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

void FusionCore::update_distance_traveled(double x, double y) {
  if (!gnss_pos_set_) {
    last_gnss_x_  = x;
    last_gnss_y_  = y;
    gnss_pos_set_ = true;
    return;
  }

  double dx = x - last_gnss_x_;
  double dy = y - last_gnss_y_;
  double dist = std::sqrt(dx*dx + dy*dy);

  // Minimum step size to filter GPS jitter — ignore sub-centimeter moves
  // This prevents GPS noise from accumulating fake distance
  const double MIN_STEP = 0.05;  // 5cm
  if (dist < MIN_STEP) return;

  // Estimate speed from this GPS step and the time since last GNSS fix
  // If dt is available, check that speed is meaningful (not jitter, not slip)
  // We use the state velocity as a cross-check
  double state_speed = std::sqrt(
    ukf_.state().x[VX] * ukf_.state().x[VX] +
    ukf_.state().x[VY] * ukf_.state().x[VY]);

  // Minimum forward speed to count as real motion
  // Below this threshold: could be GPS jitter, spinning in place, or sliding
  const double MIN_SPEED = 0.2;  // m/s

  // Maximum yaw rate — if spinning fast, heading is not observable from track
  const double MAX_YAW_RATE = 0.3;  // rad/s (~17 deg/s)
  double yaw_rate = std::abs(ukf_.state().x[WZ]);

  bool motion_is_valid = (state_speed >= MIN_SPEED) && (yaw_rate <= MAX_YAW_RATE);

  if (motion_is_valid) {
    distance_traveled_ += dist;
  }

  last_gnss_x_ = x;
  last_gnss_y_ = y;

  // Only validate heading from GPS track when motion quality is confirmed
  if (!heading_validated_ &&
      distance_traveled_ >= config_.heading_observable_distance) {
    heading_validated_ = true;
    heading_source_    = HeadingSource::GPS_TRACK;
  }
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

  // IMU orientation validates heading ONLY if the IMU has a magnetometer.
  // 6-axis IMUs integrate gyro for yaw — this drifts and is not a valid
  // heading reference. 9-axis IMUs with magnetometer give true heading.
  // peci1 fix: don't blindly trust IMU orientation as heading source.
  if (config_.imu_has_magnetometer) {
    if (!heading_validated_ ||
        heading_source_ == HeadingSource::GPS_TRACK) {
      heading_validated_ = true;
      heading_source_    = HeadingSource::IMU_ORIENTATION;
    }
  }
  // If no magnetometer: orientation still fused for roll/pitch accuracy,
  // but heading_validated_ is NOT set — lever arm stays inactive.

  last_imu_time_ = timestamp_seconds;
  ++update_count_;
}

void FusionCore::update_encoder(
  double timestamp_seconds,
  double vx, double vy, double wz,
  double var_vx,
  double var_vy,
  double var_wz
) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: update_encoder() called before init()");

  predict_to(timestamp_seconds);

  sensors::EncoderMeasurement z;
  z[0] = vx; z[1] = vy; z[2] = wz;

  // Use message covariance when provided (peci1 fix)
  // Falls back to config params when var is -1.0 (not available)
  sensors::EncoderNoiseMatrix R = sensors::encoder_noise_matrix(config_.encoder);
  if (var_vx > 0.0) R(0,0) = var_vx;
  if (var_vy > 0.0) R(1,1) = var_vy;
  if (var_wz > 0.0) R(2,2) = var_wz;

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

  if (!fix.is_valid(config_.gnss)) return false;

  predict_to(timestamp_seconds);

  // Track distance for GPS-track heading observability
  update_distance_traveled(fix.x, fix.y);

  sensors::GnssPosMeasurement z;
  z[0] = fix.x;
  z[1] = fix.y;
  z[2] = fix.z;

  sensors::GnssPosNoiseMatrix R =
    sensors::gnss_pos_noise_matrix(config_.gnss, fix);

  // Lever arm logic — only apply when heading is GENUINELY validated
  // from an independent source. This is the proper fix for peci1's
  // observability concern: we don't trust yaw_variance alone because
  // the filter can artificially reduce variance without real heading info.
  //
  // heading_validated_ is only set true when:
  //   - dual antenna heading message received (DUAL_ANTENNA)
  //   - IMU orientation message received (IMU_ORIENTATION)
  //   - robot moved >= heading_observable_distance meters (GPS_TRACK)
  //
  // Before any of these, lever arm is disabled regardless of yaw_variance.

  bool use_lever_arm = !config_.gnss.lever_arm.is_zero() && heading_validated_;

  if (use_lever_arm) {
    auto h = sensors::gnss_pos_measurement_function_with_lever_arm(
      config_.gnss.lever_arm);
    ukf_.update<sensors::GNSS_POS_DIM>(z, h, R);
  } else {
    ukf_.update<sensors::GNSS_POS_DIM>(
      z, sensors::gnss_pos_measurement_function, R);
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
    z, sensors::gnss_hdg_measurement_function, R);

  // Dual antenna heading is the strongest possible heading validation
  // Override any weaker source
  heading_validated_ = true;
  heading_source_    = HeadingSource::DUAL_ANTENNA;

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

  // Heading observability
  status.heading_validated = heading_validated_;
  status.heading_source    = heading_source_;
  status.distance_traveled = distance_traveled_;

  return status;
}

} // namespace fusioncore
