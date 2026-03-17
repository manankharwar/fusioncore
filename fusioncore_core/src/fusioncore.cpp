#include "fusioncore/fusioncore.hpp"
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
  update_count_      = 0;
  initialized_       = true;
}

void FusionCore::reset() {
  initialized_       = false;
  last_timestamp_    = 0.0;
  last_imu_time_     = -1.0;
  last_encoder_time_ = -1.0;
  update_count_      = 0;
}

void FusionCore::predict_to(double timestamp_seconds) {
  double dt = timestamp_seconds - last_timestamp_;

  // Ignore duplicate or out-of-order timestamps
  if (dt < config_.min_dt) return;

  // Sensor dropout — gap too large, skip predict
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
  if (!initialized_) {
    throw std::runtime_error("FusionCore: update_imu() called before init()");
  }

  predict_to(timestamp_seconds);

  sensors::ImuMeasurement z;
  z[0] = wx; z[1] = wy; z[2] = wz;
  z[3] = ax; z[4] = ay; z[5] = az;

  sensors::ImuNoiseMatrix R = sensors::imu_noise_matrix(config_.imu);

  ukf_.update<sensors::IMU_DIM>(z, sensors::imu_measurement_function, R);

  last_imu_time_ = timestamp_seconds;
  ++update_count_;
}

void FusionCore::update_encoder(
  double timestamp_seconds,
  double vx, double vy, double wz
) {
  if (!initialized_) {
    throw std::runtime_error("FusionCore: update_encoder() called before init()");
  }

  predict_to(timestamp_seconds);

  sensors::EncoderMeasurement z;
  z[0] = vx; z[1] = vy; z[2] = wz;

  sensors::EncoderNoiseMatrix R = sensors::encoder_noise_matrix(config_.encoder);

  ukf_.update<sensors::ENCODER_DIM>(z, sensors::encoder_measurement_function, R);

  last_encoder_time_ = timestamp_seconds;
  ++update_count_;
}

const State& FusionCore::get_state() const {
  return ukf_.state();
}

FusionCoreStatus FusionCore::get_status() const {
  FusionCoreStatus status;
  status.initialized  = initialized_;
  status.update_count = update_count_;

  if (!initialized_) return status;

  // Sensor health — stale if no data in 1 second
  double stale_threshold = 1.0;

  if (last_imu_time_ < 0.0)
    status.imu_health = SensorHealth::NOT_INIT;
  else if ((last_timestamp_ - last_imu_time_) > stale_threshold)
    status.imu_health = SensorHealth::STALE;
  else
    status.imu_health = SensorHealth::OK;

  if (last_encoder_time_ < 0.0)
    status.encoder_health = SensorHealth::NOT_INIT;
  else if ((last_timestamp_ - last_encoder_time_) > stale_threshold)
    status.encoder_health = SensorHealth::STALE;
  else
    status.encoder_health = SensorHealth::OK;

  // Position uncertainty — trace of top-left 3x3 of P
  const StateMatrix& P = ukf_.state().P;
  status.position_uncertainty = P(0,0) + P(1,1) + P(2,2);

  return status;
}

} // namespace fusioncore
