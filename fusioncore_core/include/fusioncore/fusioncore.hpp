#pragma once
#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"
#include "fusioncore/motion_model.hpp"
#include "fusioncore/sensors/imu.hpp"
#include "fusioncore/sensors/encoder.hpp"
#include "fusioncore/sensors/gnss.hpp"
#include "fusioncore/sensors/vslam.hpp"
#include "fusioncore/sensors/magnetometer.hpp"
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
  sensors::VslamParams   vslam;
  sensors::MagParams     mag;
  double min_dt = 1e-6;
  double max_dt = 1.0;

  // How long without a sensor update before that sensor is marked STALE (seconds)
  double stale_timeout = 1.0;

  // Minimum distance robot must travel (meters) before heading is considered
  // geometrically observable from GPS track alone.
  double heading_observable_distance = 5.0;

  // GPS track heading fusion: fuses the GPS displacement bearing as a yaw
  // pseudo-measurement whenever the robot has moved at least min_dist meters
  // since the last heading fusion. This is the same mechanism navsat_transform
  // uses internally and directly corrects heading from GPS geometry without
  // relying on gyro bias estimation.
  // max_sigma: skip fusion if position_noise / displacement > this (rad).
  // 0.4 rad (23 deg) is a reasonable ceiling; tighter GPS gives automatic improvement.
  bool   gps_track_heading_enabled  = true;
  double gps_track_heading_min_dist = 5.0;   // meters
  double gps_track_heading_max_sigma = 0.4;  // radians

  // Motion quality thresholds for GPS track heading observability.
  // A GPS displacement step only counts toward heading_observable_distance when:
  //   - robot speed >= min_speed (filters GPS jitter and standstill noise)
  //   - yaw rate <= max_yaw_rate (during fast turns the bearing changes too quickly
  //     to give a reliable heading measurement)
  // Increase min_speed on high-vibration platforms. Decrease max_yaw_rate if heading
  // is being incorrectly validated during tight turns (parking lot maneuvers).
  double gps_track_heading_min_speed    = 0.2;   // m/s
  double gps_track_heading_max_yaw_rate = 0.3;   // rad/s (~17 deg/s)

  // Lever arm correction is only applied when heading uncertainty is below this threshold.
  // When heading_sigma exceeds this value (e.g. during prolonged turns with no GPS track
  // heading fusions firing), rotating the lever arm by an uncertain heading adds more
  // position error than it removes. Lever arm silently deactivates until heading tightens.
  // Rule of thumb: lever_arm_length_m * sin(threshold_rad) should be < GPS noise sigma.
  // Default 20 deg: disables lever arm during tight-turn sections where heading degrades,
  // leaves it active during straight/gentle-curve driving where it genuinely helps.
  double gnss_lever_arm_max_heading_sigma_deg = 20.0;

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
  double outlier_threshold_vslam = 22.46;  // chi2(6, 0.999): 6D pose

  // Physical plausibility gate for GNSS position.
  // A fix cannot be farther from the filter's predicted position than the robot
  // could physically have moved or drifted since the last accepted fix:
  // dead-reckoning error is bounded by the distance traveled, which is bounded
  // by max_speed * dt. This rejects an adversarial outlier cluster arriving at a
  // GPS-blackout boundary, which a coast-relaxed chi2 gate would otherwise admit
  // (the chi2 covariance has been inflated to re-acquire, so a far outlier slips
  // through; chi2 alone cannot tell a 700 m outlier from a legitimate recovery
  // fix after a long gap, but physics can). An implausible fix is rejected and
  // does NOT count toward coast, so an outlier can never relax the gate.
  // Set to the platform's maximum plausible speed (m/s); a few times cruise
  // speed is safe. 0 = disabled (default, preserves prior behavior).
  double gnss_max_speed        = 0.0;
  // Slack added to the max_speed * dt bound (m): covers GPS noise and the fact
  // that the predicted position itself has some uncertainty. ~3-5 m is typical.
  double gnss_max_speed_margin = 5.0;

  // Adaptive noise covariance
  // Whether to enable adaptive R estimation for each sensor
  bool adaptive_imu     = true;
  bool adaptive_encoder = true;
  bool adaptive_gnss    = true;

  // Whether to enable adaptive R for ground constraint pseudo-measurements (VZ=0, AZ=0).
  // When true, VZ and AZ noise automatically inflates on rough terrain as innovations grow,
  // then relaxes back when terrain is smooth. No config changes needed across environments.
  bool adaptive_ground_constraint = true;

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

  // Optional custom motion model. nullptr = use ConstantVelocityAcceleration (default).
  // Set via create_motion_model("DifferentialDrive") etc. before passing to FusionCore.
  std::shared_ptr<MotionModelBase> motion_model = nullptr;

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

  // Non-holonomic constraint: lateral velocity (VY) tightness.
  // For differential drive robots, VY should be zero (robot can't move sideways).
  // This is the sigma on that assertion (m/s): lower = harder constraint.
  // Default 0.05 m/s matches encoder.vel_noise (previous hardcoded behavior).
  // Increase to 10.0+ to effectively disable for mecanum/omnidirectional robots.
  // Increase to 0.3-1.0 for Ackermann robots on slippery surfaces with lateral slip.
  double encoder_nhc_vy_sigma = 0.05;

  // Non-holonomic constraint: body-frame vertical velocity (VZ) tightness.
  // For ground robots, VZ should be zero during steady locomotion.
  // Default 0.1 m/s: fine for flat ground and mild terrain.
  // Increase to 0.3-1.0 for robots traversing obstacles, curbs, or rough terrain
  // where the chassis genuinely has transient vertical motion during transitions.
  double ground_constraint_vz_sigma = 0.1;

  // Non-holonomic constraint: body-frame vertical acceleration (AZ) tightness.
  // Constraining AZ prevents gravity-constant mismatch (WGS84 vs local g) from
  // leaking into AZ and integrating into VZ drift via the motion model.
  // Default 0.5 m/s²: loose enough for bumps and ramps, tight enough to stop drift.
  // Increase to 2.0+ for aggressive terrain where vertical accelerations are real.
  double ground_constraint_az_sigma = 0.5;

  // Position-level ground constraint: fuses Z=0 as a pseudo-measurement each
  // encoder callback. Tighter than GPS altitude noise (5m std dev on NCLT),
  // so it dominates and keeps the filter at ground level on flat terrain.
  // 0.0 = disabled (default: GPS altitude drives Z normally).
  // ~0.3m sigma = flat terrain mode (campus, parking lot, warehouse floor).
  double ground_z_position_sigma = 0.0;

  // Inertial coast mode: after this many consecutive GNSS rejections, inflate
  // Q_position so P grows and the Mahalanobis gate naturally relaxes.
  // This prevents cascade failure when the filter drifts during a GPS gap
  // and then rejects the recovery fixes as apparent outliers.
  // 0 = disabled; typical value: 5
  int    gnss_coast_n        = 5;
  // Rejection-triggered coast only fires when the rejection sequence began
  // after a GPS gap of at least this many seconds (i.e. the filter plausibly
  // drifted blind and is now rejecting the returning fix). A continuously
  // present GPS that keeps failing the chi2 gate is a persistent outlier (e.g.
  // a multipath spike), NOT filter drift: inflating P to admit it would let the
  // outlier defeat the gate. Gating coast on a preceding gap keeps a sustained
  // spike rejected for its whole duration while preserving post-outage
  // re-acquisition. The pure-absence coast path (gnss_coast_timeout_s) is
  // unaffected. Set to 0 to restore the old gap-agnostic behavior.
  double gnss_coast_min_gap_s = 1.0;
  // Multiplier applied to q_position each predict step while in coast mode.
  // 20.0 = 4.5x position sigma growth per second at 100Hz IMU.
  double gnss_coast_q_factor = 20.0;
  // Multiplier applied to q_gyro_bias while in coast mode.
  // Loosens the filter's confidence in its gyro bias estimate so that encoder WZ
  // can drive fast bias correction during GPS outages. Without this, a non-zero
  // gyro bias (present on every real MEMS IMU) accumulates into heading at ~bias*t
  // with no correction, producing tens of degrees of heading error per minute.
  // 100.0 is a good default for campus-scale GPS outages (30-500s).
  double gnss_coast_q_bias_factor = 100.0;

  // Multiplier applied to R_imu[WZ,WZ] during GPS coast mode.
  // Reduces the IMU's influence on heading rate so the encoder WZ (which has
  // lower systematic bias than a MEMS gyro) dominates heading integration.
  // Without GPS, gyro bias corrupts heading at ~bias*time with no correction.
  // Scale = 100: encoder provides ~70% of WZ information (vs 2% normally).
  // Scale = 1000: encoder provides ~96% (essentially RL behavior for heading).
  // 1.0 = disabled (default). Suggested: 500.0 for deployments with long GPS outages.
  double gnss_coast_imu_wz_scale = 1.0;
  // Also enter coast mode when GPS has been absent for this many seconds.
  // Handles GPS outages where the receiver stops publishing entirely (mode=2,
  // power loss, tunnel) rather than publishing fixes that fail the chi2 gate.
  // 0.0 = disabled; typical value: 30.0
  double gnss_coast_timeout_s = 0.0;

  // Enter position-injection recovery mode only after a GPS absence longer than
  // this many seconds. Recovery mode bypasses the chi2 gate for the first
  // returning GPS fix, which is needed for very long blackouts (>100s) where
  // dead-reckoning drift may exceed the chi2 acceptance range. For short
  // blackouts (30-90s), the chi2 gate handles recovery correctly and recovery
  // mode is counter-productive: it allows massive GPS outliers (bad multipath
  // at the blackout boundary) to be injected unconditionally.
  // 0.0 = enter recovery mode at the same time as coast mode (original behavior).
  // Typical: 120.0 (2 minutes). Must be >= gnss_coast_timeout_s.
  double gnss_recovery_timeout_s = 0.0;

  // After this many consecutive chi2 rejections, inflate P[x,x] and P[y,y]
  // directly so the next GPS fix passes the gate and corrects via a proper
  // Bayesian update. This breaks the cascade where GPS is present but the
  // filter has drifted far enough that all incoming fixes fail chi2.
  // Fires exactly once per cascade (when counter first reaches this value).
  // Must be > gnss_coast_n. 0 = disabled; typical value: 15.
  int    gnss_recovery_rejection_n = 0;
  // XY sigma for P inflation (meters). 50m covers any realistic drift from
  // a chi2 cascade, allowing GPS to pull the filter back from up to ~100m off.
  double gnss_p_inflate_sigma = 50.0;
};

// How heading was validated: tracked per filter run
enum class HeadingSource {
  NONE            = 0,  // no independent heading: lever arm disabled
  DUAL_ANTENNA    = 1,  // dual GNSS antenna heading received
  IMU_ORIENTATION = 2,  // AHRS/IMU published full orientation
  GPS_TRACK       = 3,  // robot moved enough for heading to be geometric
  MAGNETOMETER    = 4,  // raw magnetometer field fused directly
};

// Why a GNSS fix was rejected (or ACCEPTED if it passed)
enum class GnssRejectionReason {
  NOT_PROCESSED   = 0,  // update_gnss not yet called
  ACCEPTED        = 1,
  FIX_TYPE_LOW    = 2,  // fix_type < min_fix_type
  HDOP_HIGH       = 3,  // hdop > max_hdop
  VDOP_HIGH       = 4,  // vdop > max_vdop
  MIN_SATS        = 5,  // satellites < min_satellites
  CHI2_FAILED     = 6,  // Mahalanobis distance > threshold
  DELAY_TOO_LARGE = 7,  // measurement older than max_measurement_delay
  IMPLAUSIBLE_JUMP = 8, // fix farther from prediction than max_speed*dt allows
};

// Per-fix observability data: populated by update_gnss() on every call.
// Retrieve via get_gnss_debug() after update_gnss() returns.
struct GnssFixDebug {
  bool               accepted           = false;
  GnssRejectionReason reason            = GnssRejectionReason::NOT_PROCESSED;
  double             mahalanobis_sq     = -1.0;  // -1 = not computed (quality gate failed first)
  double             chi2_threshold     = 0.0;
  double             hdop               = 0.0;
  double             vdop               = 0.0;
  int                satellites         = 0;
  int                fix_type           = 0;
  bool               in_coast_mode      = false;
  int                consecutive_rejects = 0;
  double             position_sigma_x   = 0.0;
  double             position_sigma_y   = 0.0;
  // Lever arm observability
  bool               lever_arm_used     = false;  // was lever arm correction applied for this fix
  double             heading_sigma_deg  = 0.0;    // heading 1-sigma at time of this fix (degrees)
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

  // Heading observability
  bool          heading_validated   = false;
  HeadingSource heading_source      = HeadingSource::NONE;
  double        distance_traveled   = 0.0;

  // Outlier rejection counters: cumulative since init()
  int gnss_outliers  = 0;
  int imu_outliers   = 0;
  int enc_outliers   = 0;
  int hdg_outliers   = 0;
  int vslam_outliers = 0;
  int mag_outliers   = 0;

  SensorHealth vslam_health = SensorHealth::NOT_INIT;
  SensorHealth mag_health   = SensorHealth::NOT_INIT;

  // Innovation norms: magnitude of the last accepted measurement residual.
  // Zero until the first accepted update from that sensor.
  double gnss_innovation_norm    = 0.0;
  double imu_innovation_norm     = 0.0;
  double encoder_innovation_norm = 0.0;

  // Position 1-sigma uncertainty from the filter covariance (meters).
  double position_sigma_x = 0.0;
  double position_sigma_y = 0.0;
  double position_sigma_z = 0.0;

  // GPS coast mode state
  bool gnss_in_coast           = false;
  int  gnss_consecutive_rejects = 0;
};

class FusionCore {
public:
  explicit FusionCore(const FusionCoreConfig& config = FusionCoreConfig{});

  void init(const State& initial_state, double timestamp_seconds);

  // Runtime updater for the IMU lever arm — the ROS wrapper calls this
  // after auto-resolving base_frame -> imu_frame from TF. Cheap (one
  // struct copy) and only touches config_.imu.lever_arm.
  void set_imu_lever_arm(const sensors::ImuLeverArm& lever_arm);

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

  // VSLAM pose update: 6-DOF position + orientation in local ENU frame.
  // Ignores the twist component of nav_msgs/Odometry entirely.
  // Returns true if accepted, false if rejected by the outlier gate.
  bool update_pose(
    double timestamp_seconds,
    const sensors::VslamPose& pose
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

  // Raw magnetometer heading update.
  // Applies hard/soft iron correction, tilt-compensates using current filter
  // roll/pitch, then fuses the resulting yaw as a 1-DOF UKF measurement.
  // Call this from a sensor_msgs/MagneticField subscriber callback.
  // Returns true if the measurement was accepted (passed chi2 gate).
  bool update_magnetometer(
    double timestamp_seconds,
    double mx, double my, double mz
  );

  const State&       get_state()      const;
  FusionCoreStatus   get_status()     const;
  const GnssFixDebug& get_gnss_debug() const { return gnss_debug_; }
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
  double last_vslam_time_   = -1.0;
  double last_mag_time_     = -1.0;
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
    // Includes the bias term (mean^2) so systematic offsets (e.g. GPS multipath
    // pushing fixes consistently in one direction) inflate R, not just random scatter.
    ZMatrix estimate_covariance() const {
      Eigen::Matrix<double, z_dim, 1> mean = Eigen::Matrix<double, z_dim, 1>::Zero();
      for (const auto& nu : innovations)
        mean += nu;
      mean /= (double)innovations.size();

      ZMatrix C = ZMatrix::Zero();
      for (const auto& nu : innovations) {
        Eigen::Matrix<double, z_dim, 1> d = nu - mean;
        C += d * d.transpose();
      }
      C /= (double)innovations.size();
      C += mean * mean.transpose();  // systematic bias term
      return C;
    }
  };

  InnovationWindow<sensors::IMU_DIM>              imu_innovations_;
  InnovationWindow<sensors::ENCODER_DIM>          encoder_innovations_;
  InnovationWindow<sensors::GNSS_POS_DIM>         gnss_innovations_;
  InnovationWindow<sensors::IMU_ORIENTATION_DIM>  imu_orient_innovations_;
  InnovationWindow<sensors::VSLAM_POSE_DIM>       vslam_innovations_;
  InnovationWindow<1>                             vz_innovations_;
  InnovationWindow<1>                             az_innovations_;

  // Current adaptive R estimates: start at config values, drift toward truth
  sensors::ImuNoiseMatrix             R_imu_;
  sensors::EncoderNoiseMatrix         R_encoder_;
  sensors::GnssPosNoiseMatrix         R_gnss_;
  sensors::ImuOrientationNoiseMatrix  R_imu_orient_;
  sensors::VslamPoseNoiseMatrix       R_vslam_;
  Eigen::Matrix<double, 1, 1>         R_vz_;   // body-frame vertical velocity constraint
  Eigen::Matrix<double, 1, 1>         R_az_;   // body-frame vertical accel constraint

  // Minimum R floors: adaptive R must never drop below the initially configured value.
  // A constant innovation bias (e.g. sim gravity != WGS84 gravity) has zero variance
  // after mean-subtraction and would otherwise drive R toward 1e-9, causing
  // K[position, accel] to explode and Z to drift at m/s rates.
  sensors::ImuNoiseMatrix             R_imu_floor_;
  sensors::EncoderNoiseMatrix         R_encoder_floor_;
  sensors::GnssPosNoiseMatrix         R_gnss_floor_;
  sensors::ImuOrientationNoiseMatrix  R_imu_orient_floor_;
  sensors::VslamPoseNoiseMatrix       R_vslam_floor_;
  Eigen::Matrix<double, 1, 1>         R_vz_floor_;
  Eigen::Matrix<double, 1, 1>         R_az_floor_;

  bool adaptive_initialized_ = false;

  // Outlier rejection counters: for status reporting
  int gnss_outliers_   = 0;
  int imu_outliers_    = 0;
  int enc_outliers_    = 0;
  int hdg_outliers_    = 0;
  int vslam_outliers_  = 0;
  int mag_outliers_    = 0;

  // Per-fix observability: updated on every update_gnss() call
  GnssFixDebug gnss_debug_;

  // Last accepted innovation norms per sensor: updated on each accepted update
  double last_gnss_innovation_norm_    = 0.0;
  double last_imu_innovation_norm_     = 0.0;
  double last_encoder_innovation_norm_ = 0.0;

  // Inertial coast mode tracking
  int  gnss_consecutive_rejects_ = 0;
  bool gnss_in_coast_            = false;
  // Whether the current rejection sequence began after a GPS gap. Captured at
  // the first rejection of a sequence and used to gate rejection-triggered
  // coast so a continuous outlier (spike) cannot inflate P to defeat the gate.
  bool reject_after_gap_         = false;
  // Recovery mode: after a timeout-triggered coast, accept the first returning
  // GPS fix unconditionally (bypass chi2 gate). After 7+ minutes blind, dead
  // reckoning error can be hundreds of meters, far outside the chi2 gate.
  // Without this, coast mode inflates P but can't grow sigma fast enough to
  // accept the recovery fix, causing permanent GPS rejection.
  bool gnss_in_recovery_         = false;

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

  // Reference position for GPS track heading fusion.
  // Updated only when a heading fusion fires, so displacement accumulates
  // across multiple GPS fixes until the baseline is large enough to be reliable.
  double last_hdg_fix_x_  = 0.0;
  double last_hdg_fix_y_  = 0.0;
  bool   hdg_fix_set_     = false;

  // True after the first GPS track heading fusion has successfully fired.
  // The chi2 gate for subsequent fusions is only applied once this is true.
  // Without this guard, update_distance_traveled() sets heading_validated_=true
  // at 5m (before the 7.5m baseline needed for a reliable bearing), causing
  // the chi2 gate to reject the very first heading fusion when the initial
  // heading error exceeds ~75 degrees.
  bool   gps_track_hdg_fused_ = false;

  // Returns heading 1-sigma in radians computed from P via quaternion-to-yaw Jacobian.
  double compute_heading_sigma_rad() const;

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
