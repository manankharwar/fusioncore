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

  // Minimum distance robot must travel (meters) before heading is considered
  // geometrically observable from GPS track alone.
  double heading_observable_distance = 5.0;

  // Does the IMU have a magnetometer (9-axis)?
  // true  — IMU orientation includes magnetically-referenced yaw (BNO08x,
  //         VectorNav, Xsens). Orientation update validates heading.
  // false — IMU is 6-axis only. Yaw is integrated gyro and drifts.
  //         Orientation update validates roll/pitch ONLY, not heading.
  //         Lever arm will not activate from IMU orientation alone.
  bool imu_has_magnetometer = false;
};

// How heading was validated — tracked per filter run
enum class HeadingSource {
  NONE           = 0,  // no independent heading — lever arm disabled
  DUAL_ANTENNA   = 1,  // dual GNSS antenna heading received
  IMU_ORIENTATION = 2, // AHRS/IMU published full orientation
  GPS_TRACK      = 3,  // robot moved enough for heading to be geometric
};

enum class SensorHealth {
  OK,
  STALE,
  NOT_INIT
};

struct FusionCoreStatus {
  bool         initialized          = false;
  SensorHealth imu_health           = SensorHealth::NOT_INIT;
  SensorHealth encoder_health       = SensorHealth::NOT_INIT;
  SensorHealth gnss_health          = SensorHealth::NOT_INIT;
  double       position_uncertainty = 0.0;
  int          update_count         = 0;

  // Heading observability — the real fix for peci1's concern
  bool          heading_validated   = false;
  HeadingSource heading_source      = HeadingSource::NONE;
  double        distance_traveled   = 0.0;  // meters since init
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
  // Calling this validates heading via HeadingSource::IMU_ORIENTATION
  void update_imu_orientation(
    double timestamp_seconds,
    double roll, double pitch, double yaw,
    const double orientation_cov[9] = nullptr
  );

  // Encoder update
  // var_vx, var_vy, var_wz: message covariance variances (m/s)²
  // Pass -1.0 to use config params for that axis
  void update_encoder(
    double timestamp_seconds,
    double vx, double vy, double wz,
    double var_vx = -1.0,
    double var_vy = -1.0,
    double var_wz = -1.0
  );

  // GNSS position update — ENU frame
  bool update_gnss(
    double timestamp_seconds,
    const sensors::GnssFix& fix
  );

  // GNSS dual antenna heading update
  // Calling this validates heading via HeadingSource::DUAL_ANTENNA
  bool update_gnss_heading(
    double timestamp_seconds,
    const sensors::GnssHeading& heading
  );

  const State&       get_state()  const;
  FusionCoreStatus   get_status() const;
  void               reset();
  bool               is_initialized()    const { return initialized_; }
  bool               is_heading_valid()  const { return heading_validated_; }
  HeadingSource      heading_source()    const { return heading_source_; }

private:
  FusionCoreConfig config_;
  UKF              ukf_;
  bool             initialized_       = false;

  double last_timestamp_    = 0.0;
  double last_imu_time_     = -1.0;
  double last_encoder_time_ = -1.0;
  double last_gnss_time_    = -1.0;
  int    update_count_      = 0;

  // Heading observability tracking
  bool          heading_validated_ = false;
  HeadingSource heading_source_    = HeadingSource::NONE;

  // For GPS track heading observability
  double last_gnss_x_     = 0.0;
  double last_gnss_y_     = 0.0;
  bool   gnss_pos_set_    = false;
  double distance_traveled_ = 0.0;

  void predict_to(double timestamp_seconds);
  void update_distance_traveled(double x, double y);
};

} // namespace fusioncore
