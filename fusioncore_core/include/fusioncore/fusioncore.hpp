#pragma once

#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"
#include "fusioncore/sensors/imu.hpp"
#include "fusioncore/sensors/encoder.hpp"
#include <chrono>
#include <optional>
#include <string>

namespace fusioncore {

// Configuration for the FusionCore manager
struct FusionCoreConfig {
  UKFParams     ukf;
  sensors::ImuParams     imu;
  sensors::EncoderParams encoder;

  // Minimum dt to accept (ignore duplicate timestamps)
  double min_dt = 1e-6;

  // Maximum dt before we reset (sensor dropout)
  double max_dt = 1.0;
};

// Health status of a single sensor
enum class SensorHealth {
  OK,
  STALE,      // no data received recently
  NOT_INIT    // never received data
};

// Overall FusionCore status
struct FusionCoreStatus {
  bool initialized        = false;
  SensorHealth imu_health     = SensorHealth::NOT_INIT;
  SensorHealth encoder_health = SensorHealth::NOT_INIT;
  double position_uncertainty = 0.0;  // trace of position block of P
  int update_count            = 0;
};

// The main FusionCore interface
// Usage:
//   FusionCore fc;
//   fc.init(initial_state);
//   fc.update_imu(timestamp, wx, wy, wz, ax, ay, az);
//   fc.update_encoder(timestamp, vx, vy, wz);
//   State s = fc.get_state();

class FusionCore {
public:
  explicit FusionCore(const FusionCoreConfig& config = FusionCoreConfig{});

  // Initialize with a known starting state
  // Call this before any updates
  void init(const State& initial_state, double timestamp_seconds);

  // Feed an IMU measurement
  // timestamp_seconds: absolute time in seconds
  // wx, wy, wz: angular velocity (rad/s, body frame)
  // ax, ay, az: linear acceleration (m/s², body frame)
  void update_imu(
    double timestamp_seconds,
    double wx, double wy, double wz,
    double ax, double ay, double az
  );

  // Feed a wheel encoder measurement
  // vx, vy: linear velocity (m/s, body frame)
  // wz: yaw rate (rad/s)
  void update_encoder(
    double timestamp_seconds,
    double vx, double vy, double wz
  );

  // Get current state estimate
  const State& get_state() const;

  // Get status and sensor health
  FusionCoreStatus get_status() const;

  // Reset everything
  void reset();

  bool is_initialized() const { return initialized_; }

private:
  FusionCoreConfig config_;
  UKF ukf_;
  bool initialized_ = false;

  double last_timestamp_    = 0.0;
  double last_imu_time_     = -1.0;
  double last_encoder_time_ = -1.0;

  int update_count_ = 0;

  // Predict forward to this timestamp
  void predict_to(double timestamp_seconds);
};

} // namespace fusioncore
