#pragma once

#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"
#include "fusioncore/sensors/imu.hpp"
#include "fusioncore/sensors/encoder.hpp"
#include "fusioncore/sensors/gnss.hpp"
#include <chrono>
#include <optional>
#include <string>

namespace fusioncore {

struct FusionCoreConfig {
  UKFParams              ukf;
  sensors::ImuParams     imu;
  sensors::EncoderParams encoder;
  sensors::GnssParams    gnss;

  double min_dt = 1e-6;
  double max_dt = 1.0;
};

enum class SensorHealth {
  OK,
  STALE,
  NOT_INIT
};

struct FusionCoreStatus {
  bool         initialized        = false;
  SensorHealth imu_health         = SensorHealth::NOT_INIT;
  SensorHealth encoder_health     = SensorHealth::NOT_INIT;
  SensorHealth gnss_health        = SensorHealth::NOT_INIT;
  double       position_uncertainty = 0.0;
  int          update_count       = 0;
};

class FusionCore {
public:
  explicit FusionCore(const FusionCoreConfig& config = FusionCoreConfig{});

  void init(const State& initial_state, double timestamp_seconds);

  // IMU raw update (gyro + accel)
  void update_imu(
    double timestamp_seconds,
    double wx, double wy, double wz,
    double ax, double ay, double az
  );

  // IMU orientation update — for IMUs that publish full orientation
  // (BNO08x, VectorNav, Xsens, etc.)
  // roll, pitch, yaw in radians
  // orientation_cov: 9-element row-major covariance matrix from message
  //                  pass nullptr to use config params
  void update_imu_orientation(
    double timestamp_seconds,
    double roll, double pitch, double yaw,
    const double orientation_cov[9] = nullptr
  );

  // Encoder update
  void update_encoder(
    double timestamp_seconds,
    double vx, double vy, double wz
  );

  // GNSS position update — ENU frame
  // Rejects automatically if fix quality is poor
  bool update_gnss(
    double timestamp_seconds,
    const sensors::GnssFix& fix
  );

  // GNSS dual antenna heading update
  bool update_gnss_heading(
    double timestamp_seconds,
    const sensors::GnssHeading& heading
  );

  const State&       get_state()  const;
  FusionCoreStatus   get_status() const;
  void               reset();
  bool               is_initialized() const { return initialized_; }

private:
  FusionCoreConfig config_;
  UKF              ukf_;
  bool             initialized_      = false;

  double last_timestamp_    = 0.0;
  double last_imu_time_     = -1.0;
  double last_encoder_time_ = -1.0;
  double last_gnss_time_    = -1.0;

  int update_count_ = 0;

  void predict_to(double timestamp_seconds);
};

} // namespace fusioncore
