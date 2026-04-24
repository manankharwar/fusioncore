#pragma once
#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"
#include "fusioncore/sensors/imu.hpp"
#include "fusioncore/sensors/encoder.hpp"
#include "fusioncore/sensors/gnss.hpp"
#include <chrono>
#include <optional>
#include <string>
#include <deque>
#include <functional>
#include <array>

namespace fusioncore {

struct FusionCoreConfig {
  UKFParams              ukf;
  sensors::ImuParams     imu;
  sensors::EncoderParams encoder;
  sensors::GnssParams    gnss;
  double min_dt = 1e-6;
  double max_dt = 1.0;

  // How long without a sensor update before that sensor is marked STALE (seconds)
  double stale_timeout = 1.0;

  // Minimum distance robot must travel (meters) before heading is considered
  // geometrically observable from GPS track alone.
  double heading_observable_distance = 5.0;

  // Delay compensation: state snapshot buffer
  // Mahalanobis outlier rejection
  // Rejects measurements that are statistically implausible.
  // Threshold is chi-squared percentile for the measurement dimension.
  // 99.9th percentile recommended: rejects GPS jumps, encoder spikes.
  bool   outlier_rejection       = true;
  double outlier_threshold_gnss  = 16.27;  // chi2(3, 0.999): 3D position
  double outlier_threshold_imu   = 15.09;  // chi2(6, 0.999): 6D IMU
  double outlier_threshold_enc   = 11.34;  // chi2(3, 0.999): 3D encoder
  double outlier_threshold_hdg   = 10.83;  // chi2(1, 0.999): 1D heading

  // Adaptive noise covariance
  // Whether to enable adaptive R estimation for each sensor
  bool adaptive_imu     = true;
  bool adaptive_encoder = true;
  bool adaptive_gnss    = true;

  // Sliding window size for innovation tracking (number of updates)
  int adaptive_window = 50;

  // Learning rate: how fast R adapts to estimated noise (0.0 = off, 0.1 = fast)
  double adaptive_alpha = 0.01;

  // Max delay to compensate for (seconds). GNSS is typically 100-300ms late.
  double max_measurement_delay = 0.5;

  // How many state snapshots to keep. At 100Hz IMU, 50 = 0.5 seconds.
  int snapshot_buffer_size = 50;

  // How many IMU messages to keep for full replay retrodiction.
  // At 100Hz IMU and 500ms max delay: 50 messages minimum.
  int imu_buffer_size = 100;

  // Zero-velocity update (ZUPT) parameters
  // Velocity threshold below which the robot is considered stationary (m/s and rad/s)
  double zupt_velocity_threshold = 0.05;
  double zupt_angular_threshold  = 0.05;
  // Noise sigma applied during ZUPT (m/s). Tight = filter strongly believes zero velocity.
  double zupt_noise_sigma = 0.01;

  // Does the IMU have a magnetometer (9-axis)?
  // true : IMU orientation includes magnetically-referenced yaw (BNO08x,
  //         VectorNav, Xsens). Orientation update validates heading.
  // false: IMU is 6-axis only. Yaw is integrated gyro and drifts.
  //         Orientation update validates roll/pitch ONLY, not heading.
  //         Lever arm will not activate from IMU orientation alone.
  bool imu_has_magnetometer = false;

  // Inertial coast mode: after this many consecutive GNSS rejections, inflate
  // Q_position so P grows and the Mahalanobis gate naturally relaxes.
  // This prevents cascade failure when the filter drifts during a GPS gap
  // and then rejects the recovery fixes as apparent outliers.
  // 0 = disabled; typical value: 5
  int    gnss_coast_n        = 5;
  // Multiplier applied to q_position each predict step while in coast mode.
  // 20.0 ≈ 4.5× position sigma growth per second at 100Hz IMU.
  double gnss_coast_q_factor = 20.0;
};

// How heading was validated: tracked per filter run
enum class HeadingSource {
  NONE           = 0,  // no independent heading: lever arm disabled
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

  // Heading observability: the real fix for peci1's concern
  bool          heading_validated   = false;
  HeadingSource heading_source      = HeadingSource::NONE;
  double        distance_traveled   = 0.0;  // meters since init

  // Outlier rejection counters: cumulative since init()
  int gnss_outliers = 0;
  int imu_outliers  = 0;
  int enc_outliers  = 0;
  int hdg_outliers  = 0;
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

  // IMU orientation update: for IMUs that publish full orientation
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

  // GNSS position update: ENU frame
  bool update_gnss(
    double timestamp_seconds,
    const sensors::GnssFix& fix
  );

  // Non-holonomic ground constraint: fuses VZ=0 as a pseudo-measurement.
  // Call this every encoder update to prevent altitude drift in the UKF.
  // Only applies to wheeled ground robots; do not call for aerial vehicles.
  void update_ground_constraint(double timestamp_seconds);

  // Zero-velocity update (ZUPT): fuses [VX=0, VY=0, WZ=0] with tight noise
  // when the robot is stationary. Prevents IMU drift from corrupting velocity
  // states during standstill. Call this when encoder velocity is near zero.
  // noise_sigma: velocity uncertainty in m/s (default 0.01: very tight)
  void update_zupt(double timestamp_seconds, double noise_sigma = 0.01);

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

  // ─── Adaptive noise covariance ───────────────────────────────────────────
  // Tracks a sliding window of innovations per sensor.
  // Estimates actual noise from innovation sequence.
  // Slowly adjusts R toward estimated value.

  // Generic innovation window: stores squared innovations per dimension
  template <int z_dim>
  struct InnovationWindow {
    using ZMatrix = Eigen::Matrix<double, z_dim, z_dim>;
    std::deque<Eigen::Matrix<double, z_dim, 1>> innovations;
    int max_size = 50;

    void push(const Eigen::Matrix<double, z_dim, 1>& nu) {
      innovations.push_back(nu);
      if ((int)innovations.size() > max_size)
        innovations.pop_front();
    }

    bool ready() const { return (int)innovations.size() >= max_size / 2; }

    // Estimate covariance from innovation window.
    // Uses the sample covariance (mean-subtracted), not the autocorrelation.
    // In a well-tuned filter innovations are zero-mean, but during startup
    // or model mismatch the mean can be nonzero: subtracting it prevents
    // R from being inflated by a squared bias term.
    ZMatrix estimate_covariance() const {
      // Compute sample mean
      Eigen::Matrix<double, z_dim, 1> mean = Eigen::Matrix<double, z_dim, 1>::Zero();
      for (const auto& nu : innovations)
        mean += nu;
      mean /= (double)innovations.size();

      // Compute mean-subtracted sample covariance
      ZMatrix C = ZMatrix::Zero();
      for (const auto& nu : innovations) {
        Eigen::Matrix<double, z_dim, 1> d = nu - mean;
        C += d * d.transpose();
      }
      return C / (double)innovations.size();
    }
  };

  InnovationWindow<sensors::IMU_DIM>              imu_innovations_;
  InnovationWindow<sensors::ENCODER_DIM>          encoder_innovations_;
  InnovationWindow<sensors::GNSS_POS_DIM>         gnss_innovations_;
  InnovationWindow<sensors::IMU_ORIENTATION_DIM>  imu_orient_innovations_;

  // Current adaptive R estimates: start at config values, drift toward truth
  sensors::ImuNoiseMatrix             R_imu_;
  sensors::EncoderNoiseMatrix         R_encoder_;
  sensors::GnssPosNoiseMatrix         R_gnss_;
  sensors::ImuOrientationNoiseMatrix  R_imu_orient_;

  // Minimum R floors: adaptive R must never drop below the initially configured value.
  // A constant innovation bias (e.g. sim gravity ≠ WGS84 gravity) has zero variance
  // after mean-subtraction and would otherwise drive R toward 1e-9, causing
  // K[position, accel] to explode and Z to drift at m/s rates.
  sensors::ImuNoiseMatrix             R_imu_floor_;
  sensors::EncoderNoiseMatrix         R_encoder_floor_;
  sensors::GnssPosNoiseMatrix         R_gnss_floor_;
  sensors::ImuOrientationNoiseMatrix  R_imu_orient_floor_;

  bool adaptive_initialized_ = false;

  // Outlier rejection counters: for status reporting
  int gnss_outliers_   = 0;
  int imu_outliers_    = 0;
  int enc_outliers_    = 0;
  int hdg_outliers_    = 0;

  // Inertial coast mode tracking
  int  gnss_consecutive_rejects_ = 0;
  bool gnss_in_coast_            = false;

  // Mahalanobis distance test
  template <int z_dim>
  bool is_outlier(
    const Eigen::Matrix<double, z_dim, 1>& innovation,
    const Eigen::Matrix<double, z_dim, z_dim>& S,
    double threshold
  ) const;

  void init_adaptive_R();

  template <int z_dim>
  void adapt_R(
    Eigen::Matrix<double, z_dim, z_dim>& R,
    const Eigen::Matrix<double, z_dim, z_dim>& R_floor,
    InnovationWindow<z_dim>& window,
    const Eigen::Matrix<double, z_dim, 1>& innovation,
    bool enabled
  );

  // ─── State snapshot for delay compensation
  struct StateSnapshot {
    double timestamp;
    State  state;
    double last_imu_time;
    double last_encoder_time;
    double last_gnss_time;
  };

  std::deque<StateSnapshot> snapshot_buffer_;

  // IMU message buffer for full replay retrodiction
  // Every raw IMU message is stored so that when a delayed GNSS arrives,
  // we replay all intermediate IMU updates instead of one big predict(dt).
  struct ImuBufferEntry {
    double timestamp;
    double wx, wy, wz;
    double ax, ay, az;
    sensors::ImuNoiseMatrix R;
  };
  std::deque<ImuBufferEntry> imu_buffer_;

  // Heading observability tracking
  bool          heading_validated_ = false;
  HeadingSource heading_source_    = HeadingSource::NONE;

  // For GPS track heading observability
  double last_gnss_x_     = 0.0;
  double last_gnss_y_     = 0.0;
  bool   gnss_pos_set_    = false;
  double distance_traveled_ = 0.0;

  void predict_to(double timestamp_seconds);
  bool apply_gnss_update(double timestamp_seconds, const sensors::GnssFix& fix);
  void save_snapshot();
  bool apply_delayed_measurement(
    double measurement_timestamp,
    const std::function<void()>& apply_fn
  );
  void update_distance_traveled(double x, double y, double pre_update_speed = -1.0);
};

} // namespace fusioncore
