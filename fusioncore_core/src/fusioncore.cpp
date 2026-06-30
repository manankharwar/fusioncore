#include "fusioncore/fusioncore.hpp"
#include "fusioncore/sensors/imu.hpp"
#include <stdexcept>
#include <cmath>
#include <limits>

namespace fusioncore {

// ─── Adaptive noise covariance implementation ─────────────────────────────

// ─── Mahalanobis outlier rejection ────────────────────────────────────────

template <int z_dim>
bool FusionCore::is_outlier(
  const Eigen::Matrix<double, z_dim, 1>& innovation,
  const Eigen::Matrix<double, z_dim, z_dim>& S,
  double threshold) const
{
  // Mahalanobis distance squared: d² = νᵀ · S⁻¹ · ν
  // Use LDLT decomposition: numerically stable when S is near-singular.
  double d2 = innovation.dot(S.ldlt().solve(innovation));
  return d2 > threshold;
}

// Explicit instantiations
template bool FusionCore::is_outlier<2>(
  const Eigen::Matrix<double, 2, 1>&,
  const Eigen::Matrix<double, 2, 2>&,
  double) const;

template bool FusionCore::is_outlier<1>(
  const Eigen::Matrix<double, 1, 1>&,
  const Eigen::Matrix<double, 1, 1>&,
  double) const;

template bool FusionCore::is_outlier<3>(
  const Eigen::Matrix<double, 3, 1>&,
  const Eigen::Matrix<double, 3, 3>&,
  double) const;

template bool FusionCore::is_outlier<6>(
  const Eigen::Matrix<double, 6, 1>&,
  const Eigen::Matrix<double, 6, 6>&,
  double) const;

void FusionCore::init_adaptive_R() {
  R_imu_         = sensors::imu_noise_matrix(config_.imu);
  R_encoder_     = sensors::encoder_noise_matrix(config_.encoder);
  R_gnss_        = sensors::GnssPosNoiseMatrix::Identity();  // will be set per-fix
  R_imu_orient_  = sensors::imu_orientation_noise_matrix(sensors::ImuOrientationParams{});
  R_vslam_       = sensors::vslam_pose_noise_matrix(config_.vslam, sensors::VslamPose{});

  R_vz_(0,0) = config_.ground_constraint_vz_sigma * config_.ground_constraint_vz_sigma;
  R_az_(0,0) = config_.ground_constraint_az_sigma * config_.ground_constraint_az_sigma;

  // Save floors: adaptive R must never drop below the initially configured sensor noise.
  R_imu_floor_        = R_imu_;
  R_encoder_floor_    = R_encoder_;
  R_gnss_floor_       = R_gnss_;
  R_imu_orient_floor_ = R_imu_orient_;
  R_vslam_floor_      = R_vslam_;
  R_vz_floor_         = R_vz_;
  R_az_floor_         = R_az_;

  imu_innovations_.max_size         = config_.adaptive_window;
  encoder_innovations_.max_size     = config_.adaptive_window;
  gnss_innovations_.max_size        = config_.adaptive_window;
  imu_orient_innovations_.max_size  = config_.adaptive_window;
  vslam_innovations_.max_size       = config_.adaptive_window;
  vz_innovations_.max_size          = config_.adaptive_window;
  az_innovations_.max_size          = config_.adaptive_window;

  adaptive_initialized_ = true;
}

template <int z_dim>
void FusionCore::adapt_R(
  Eigen::Matrix<double, z_dim, z_dim>& R,
  const Eigen::Matrix<double, z_dim, z_dim>& R_floor,
  InnovationWindow<z_dim>& window,
  const Eigen::Matrix<double, z_dim, 1>& innovation,
  bool enabled)
{
  window.push(innovation);

  if (!enabled || !window.ready()) return;

  // Estimate actual noise covariance from innovation window
  auto C_hat = window.estimate_covariance();

  // Slow exponential moving average toward estimated value
  // R_(k+1) = (1 - alpha) * R_k + alpha * C_hat
  R = (1.0 - config_.adaptive_alpha) * R + config_.adaptive_alpha * C_hat;

  // Guard: R must never drop below the initially configured sensor noise.
  // A constant innovation bias (e.g. sim gravity ≠ WGS84 gravity) has zero
  // variance after mean-subtraction and would otherwise drive R toward 1e-9,
  // causing K[position, accel] to explode and Z to drift at m/s rates.
  for (int i = 0; i < z_dim; ++i) {
    if (R(i,i) < R_floor(i,i)) R(i,i) = R_floor(i,i);
  }
}

FusionCore::FusionCore(const FusionCoreConfig& config)
  : config_(config), ukf_(config.ukf)
{
  if (config.motion_model) {
    ukf_.set_motion_model(config.motion_model);
  }
}

void FusionCore::set_imu_lever_arm(const sensors::ImuLeverArm& lever_arm) {
  config_.imu.lever_arm = lever_arm;
}

void FusionCore::init(const State& initial_state, double timestamp_seconds) {
  ukf_.init(initial_state);
  last_timestamp_    = timestamp_seconds;
  last_imu_time_     = -1.0;
  last_encoder_time_ = -1.0;
  last_gnss_time_    = -1.0;
  last_mag_time_     = -1.0;
  update_count_      = 0;
  initialized_       = true;

  // Reset heading observability
  heading_validated_ = false;
  heading_source_    = HeadingSource::NONE;
  gnss_pos_set_      = false;
  distance_traveled_ = 0.0;
  last_gnss_x_       = 0.0;
  last_gnss_y_       = 0.0;
  hdg_fix_set_          = false;
  last_hdg_fix_x_       = 0.0;
  last_hdg_fix_y_       = 0.0;
  gps_track_hdg_fused_  = false;

  // Reset snapshot buffer
  snapshot_buffer_.clear();
  imu_buffer_.clear();

  // Reset coast mode state
  gnss_consecutive_rejects_ = 0;
  gnss_in_coast_            = false;
  gnss_in_recovery_         = false;
  reject_after_gap_         = false;
  ukf_.set_position_noise_scale(1.0);
  ukf_.set_gyro_bias_noise_scale(1.0);

  // Reset observability state
  gnss_debug_                    = GnssFixDebug{};
  last_gnss_innovation_norm_     = 0.0;
  last_imu_innovation_norm_      = 0.0;
  last_encoder_innovation_norm_  = 0.0;

  // Initialize adaptive noise matrices
  init_adaptive_R();
}

void FusionCore::reset() {
  initialized_       = false;
  last_timestamp_    = 0.0;
  last_imu_time_     = -1.0;
  last_encoder_time_ = -1.0;
  last_gnss_time_    = -1.0;
  last_vslam_time_   = -1.0;
  last_mag_time_     = -1.0;
  update_count_      = 0;
  heading_validated_ = false;
  heading_source_    = HeadingSource::NONE;
  gnss_pos_set_      = false;
  distance_traveled_ = 0.0;
  hdg_fix_set_          = false;
  last_hdg_fix_x_       = 0.0;
  last_hdg_fix_y_       = 0.0;
  gps_track_hdg_fused_  = false;
  snapshot_buffer_.clear();
  imu_buffer_.clear();
  gnss_consecutive_rejects_ = 0;
  gnss_in_coast_            = false;
  gnss_in_recovery_         = false;
  reject_after_gap_         = false;
  ukf_.set_position_noise_scale(1.0);
  ukf_.set_gyro_bias_noise_scale(1.0);

  gnss_debug_                   = GnssFixDebug{};
  last_gnss_innovation_norm_    = 0.0;
  last_imu_innovation_norm_     = 0.0;
  last_encoder_innovation_norm_ = 0.0;
}

void FusionCore::save_snapshot() {
  StateSnapshot snap;
  snap.timestamp        = last_timestamp_;
  snap.state            = ukf_.state();
  snap.last_imu_time    = last_imu_time_;
  snap.last_encoder_time = last_encoder_time_;
  snap.last_gnss_time   = last_gnss_time_;

  snapshot_buffer_.push_back(snap);

  // Keep buffer bounded
  while ((int)snapshot_buffer_.size() > config_.snapshot_buffer_size) {
    snapshot_buffer_.pop_front();
  }
}

// Apply a measurement that arrived late.
// Finds the closest snapshot before measurement_timestamp,
// restores that state, calls apply_fn (which does predict_to + update),
// then re-predicts forward to the current time.
//
// Returns false if the measurement is too old or no snapshots exist.
bool FusionCore::apply_delayed_measurement(
  double measurement_timestamp,
  const std::function<void()>& apply_fn
) {
  if (snapshot_buffer_.empty()) return false;

  double delay = last_timestamp_ - measurement_timestamp;

  // Too old: drop it
  if (delay > config_.max_measurement_delay) return false;

  // Not actually delayed: apply normally
  if (delay <= 0.0) {
    apply_fn();
    return true;
  }

  // Fix 7: copy snapshot by value: raw pointer into std::deque is invalidated
  // by any push_back/pop_front between pointer capture and use.
  StateSnapshot best_snap;
  bool found = false;
  for (auto it = snapshot_buffer_.rbegin(); it != snapshot_buffer_.rend(); ++it) {
    if (it->timestamp <= measurement_timestamp) {
      best_snap = *it;
      found = true;
      break;
    }
  }
  if (!found) {
    best_snap = snapshot_buffer_.front();
  }

  // Save current state to restore after re-prediction
  double current_time     = last_timestamp_;
  double current_imu      = last_imu_time_;
  double current_encoder  = last_encoder_time_;
  double current_gnss     = last_gnss_time_;

  // Roll back to snapshot
  ukf_.init(best_snap.state);
  last_timestamp_     = best_snap.timestamp;
  last_imu_time_      = best_snap.last_imu_time;
  last_encoder_time_  = best_snap.last_encoder_time;
  last_gnss_time_     = best_snap.last_gnss_time;

  // Apply the delayed measurement (predict_to inside apply_fn handles timing)
  apply_fn();

  // ── Full IMU replay retrodiction ──────────────────────────────────────────
  // Instead of one big predict(dt), replay every buffered IMU message
  // between the snapshot time and current_time in chronological order.
  // This correctly evolves the state through all intermediate dynamics.
  double replay_start = last_timestamp_;
  bool replayed_any   = false;

  for (const auto& imu : imu_buffer_) {
    if (imu.timestamp <= replay_start) continue;
    if (imu.timestamp >  current_time) break;

    double dt = imu.timestamp - last_timestamp_;
    if (dt > config_.min_dt && dt <= config_.max_dt) {
      ukf_.predict(dt);
      last_timestamp_ = imu.timestamp;

      // Re-apply the IMU measurement so the filter sees the real dynamics.
      // Pick the same measurement function as update_imu() to keep the
      // replay consistent with the original update (lever-arm aware when
      // the IMU is offset from base_link).
      sensors::ImuMeasurement z;
      z[0] = imu.wx; z[1] = imu.wy; z[2] = imu.wz;
      z[3] = imu.ax; z[4] = imu.ay; z[5] = imu.az;
      auto h_imu_replay = !config_.imu.lever_arm.is_zero()
        ? sensors::imu_measurement_function_with_lever_arm(config_.imu.lever_arm)
        : std::function<sensors::ImuMeasurement(const StateVector&)>(
            sensors::imu_measurement_function);
      ukf_.update<sensors::IMU_DIM>(z, h_imu_replay, imu.R);
      replayed_any = true;
    }
  }

  // If no IMU messages were in the buffer, fall back to single predict step
  if (!replayed_any) {
    double dt = current_time - last_timestamp_;
    if (dt > config_.min_dt && dt <= config_.max_dt) {
      ukf_.predict(dt);
    }
  }

  last_timestamp_    = current_time;
  last_imu_time_     = current_imu;
  last_encoder_time_ = current_encoder;
  last_gnss_time_    = current_gnss;

  return true;
}

double FusionCore::compute_heading_sigma_rad() const {
  const State& s = ukf_.state();
  const double qw = s.x[QW], qx = s.x[QX], qy = s.x[QY], qz = s.x[QZ];

  // d(yaw)/d(qw,qx,qy,qz): row 2 of the quaternion-to-Euler Jacobian
  const double t3 = 2.0 * (qw*qz + qx*qy);
  const double t4 = 1.0 - 2.0 * (qy*qy + qz*qz);
  const double safe_denom = std::max(t3*t3 + t4*t4, 1e-12);

  Eigen::Matrix<double, 1, 4> J;
  J(0,0) = 2.0*qz*t4 / safe_denom;
  J(0,1) = 2.0*qy*t4 / safe_denom;
  J(0,2) = (2.0*qx*t4 + 4.0*qy*t3) / safe_denom;
  J(0,3) = (2.0*qw*t4 + 4.0*qz*t3) / safe_denom;

  static constexpr int qi[4] = {QW, QX, QY, QZ};
  Eigen::Matrix4d P_quat;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      P_quat(i,j) = s.P(qi[i], qi[j]);

  double yaw_var = (J * P_quat * J.transpose())(0,0);
  return std::sqrt(std::max(yaw_var, 0.0));
}

void FusionCore::predict_to(double timestamp_seconds) {
  // Enter coast mode on GPS timeout: receiver went silent (mode=2, tunnel,
  // power loss) rather than publishing rejectable fixes. Consecutive-reject
  // coast mode won't fire in this case because there are no fixes to reject.
  if (config_.gnss_coast_timeout_s > 0.0 &&
      config_.gnss_coast_n > 0 &&
      last_gnss_time_ >= 0.0 &&
      !gnss_in_coast_ &&
      (timestamp_seconds - last_gnss_time_) > config_.gnss_coast_timeout_s)
  {
    gnss_in_coast_ = true;
    ukf_.set_position_noise_scale(config_.gnss_coast_q_factor);
    ukf_.set_gyro_bias_noise_scale(config_.gnss_coast_q_bias_factor);
  }

  double dt = timestamp_seconds - last_timestamp_;
  // Large backward time jump (clock reset, badly out-of-order timestamps,
  // bag-replay clock corruption on WSL2): re-sync the clock to the new time
  // base instead of freezing last_timestamp_ in the future. If we just returned,
  // last_timestamp_ would stay ahead and every following measurement would skip
  // its predict (dt stays negative) while still running its update, so P shrinks
  // with no Q injected until it goes non-PSD and the Cholesky factorization
  // fails. Small backward steps (delayed measurements within the delay window)
  // are still handled by the retrodiction path; this only catches jumps beyond
  // that window. We do not fold in the spurious measurement here: just re-base.
  if (dt < -config_.max_measurement_delay) {
    last_timestamp_ = timestamp_seconds;
    return;
  }
  if (dt < config_.min_dt) return;
  if (dt > config_.max_dt) {
    // Gap too large for a single step (sensor dropout, startup lag, etc.).
    // Step through in max_dt chunks so P accumulates Q proportionally to the
    // actual elapsed time: keeps uncertainty calibrated over long dropouts.
    double t = last_timestamp_;
    while (t + config_.max_dt < timestamp_seconds) {
      ukf_.predict(config_.max_dt);
      t += config_.max_dt;
    }
    // Fix 4: predict the remaining partial chunk: state was only propagated to t, not timestamp_seconds
    double dt_remaining = timestamp_seconds - t;
    if (dt_remaining > config_.min_dt) {
      ukf_.predict(dt_remaining);
    }
    last_timestamp_ = timestamp_seconds;
    return;
  }
  ukf_.predict(dt);
  last_timestamp_ = timestamp_seconds;
}

void FusionCore::update_distance_traveled(double x, double y, double pre_update_speed) {
  if (!gnss_pos_set_) {
    last_gnss_x_  = x;
    last_gnss_y_  = y;
    gnss_pos_set_ = true;
    return;
  }

  double dx = x - last_gnss_x_;
  double dy = y - last_gnss_y_;
  double dist = std::sqrt(dx*dx + dy*dy);

  // Minimum step size to filter GPS jitter: ignore sub-centimeter moves
  // This prevents GPS noise from accumulating fake distance
  const double MIN_STEP = 0.05;  // 5cm
  if (dist < MIN_STEP) return;

  // Fix 8: use pre-update speed captured before apply_gnss_update().
  // Post-update velocity is already GNSS-corrected and not representative of
  // motion during the GPS step. Fall back to current state if not provided.
  double state_speed = (pre_update_speed >= 0.0)
    ? pre_update_speed
    : std::sqrt(ukf_.state().x[VX] * ukf_.state().x[VX] +
                ukf_.state().x[VY] * ukf_.state().x[VY]);

  double yaw_rate = std::abs(ukf_.state().x[WZ]);

  bool motion_is_valid = (state_speed >= config_.gps_track_heading_min_speed) &&
                         (yaw_rate    <= config_.gps_track_heading_max_yaw_rate);

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

  // Use adaptive R if initialized, else config default
  sensors::ImuNoiseMatrix R = adaptive_initialized_ ? R_imu_ : sensors::imu_noise_matrix(config_.imu);

  // During GPS coast mode, inflate R[WZ,WZ] so the encoder WZ dominates heading
  // rate estimation instead of the biased IMU gyro. The IMU still contributes to
  // all other states (roll, pitch, accel, bias); only the yaw rate channel is
  // de-weighted. This matches RL-EKF behavior: heading from odometry, not IMU.
  if (gnss_in_coast_ && config_.gnss_coast_imu_wz_scale > 1.0) {
    R(2, 2) *= config_.gnss_coast_imu_wz_scale;
  }

  // Pick the measurement function: plain if IMU is at base_link origin,
  // else the lever-arm-aware variant that adds ω×(ω×r) centripetal to the
  // predicted accel. Both produce identical output when lever_arm == 0, so
  // we could always use the lambda; the explicit fork avoids the lambda
  // allocation on the hot path when no lever arm is configured.
  const bool use_imu_lever_arm = !config_.imu.lever_arm.is_zero();
  auto h_imu = use_imu_lever_arm
    ? sensors::imu_measurement_function_with_lever_arm(config_.imu.lever_arm)
    : std::function<sensors::ImuMeasurement(const StateVector&)>(
        sensors::imu_measurement_function);

  // Mahalanobis outlier rejection for IMU
  if (config_.outlier_rejection) {
    sensors::ImuMeasurement innovation_pre;
    sensors::ImuNoiseMatrix S;
    ukf_.predict_measurement<sensors::IMU_DIM>(z, h_imu, R, innovation_pre, S);
    if (is_outlier<sensors::IMU_DIM>(innovation_pre, S, config_.outlier_threshold_imu)) {
      ++imu_outliers_;
      last_imu_time_ = timestamp_seconds;
      return;
    }
  }

  auto innovation = ukf_.update<sensors::IMU_DIM>(z, h_imu, R);

  last_imu_innovation_norm_ = innovation.norm();

  // Track innovation for adaptive noise estimation
  adapt_R<sensors::IMU_DIM>(R_imu_, R_imu_floor_, imu_innovations_, innovation, config_.adaptive_imu);

  // Save snapshot for delay compensation
  save_snapshot();

  // Save IMU message for full replay retrodiction
  ImuBufferEntry entry;
  entry.timestamp = timestamp_seconds;
  entry.wx = wx; entry.wy = wy; entry.wz = wz;
  entry.ax = ax; entry.ay = ay; entry.az = az;
  entry.R  = R;
  imu_buffer_.push_back(entry);
  while ((int)imu_buffer_.size() > config_.imu_buffer_size)
    imu_buffer_.pop_front();

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

  if (!config_.imu_has_magnetometer) {
    // 6-axis IMU: fuse roll and pitch only. Yaw is omitted (not estimated from gyro integral).
    // A 3D update with R(yaw)=1e6 would still couple roll/pitch corrections into QZ
    // via the Kalman cross-covariance; a 2D update eliminates the channel entirely.
    sensors::ImuRPNoiseMatrix R_rp;
    if (orientation_cov != nullptr) {
      R_rp(0,0) = (orientation_cov[0] > 0.0) ? orientation_cov[0] : fallback.roll_noise  * fallback.roll_noise;
      R_rp(1,1) = (orientation_cov[4] > 0.0) ? orientation_cov[4] : fallback.pitch_noise * fallback.pitch_noise;
      R_rp(0,1) = R_rp(1,0) = 0.0;
    } else {
      R_rp = sensors::imu_rp_noise_matrix(fallback);
    }

    sensors::ImuRPMeasurement z_rp;
    z_rp[0] = roll;
    z_rp[1] = pitch;

    if (config_.outlier_rejection) {
      sensors::ImuRPMeasurement innovation_pre;
      sensors::ImuRPNoiseMatrix  S;
      ukf_.predict_measurement<sensors::IMU_RP_DIM>(
        z_rp, sensors::imu_rp_measurement_function, R_rp, innovation_pre, S);
      if (is_outlier<sensors::IMU_RP_DIM>(innovation_pre, S, config_.outlier_threshold_imu)) {
        ++imu_outliers_;
        last_imu_time_ = timestamp_seconds;
        return;
      }
    }

    ukf_.update<sensors::IMU_RP_DIM>(z_rp, sensors::imu_rp_measurement_function, R_rp);

  } else {
    // 9-axis IMU with magnetometer: fuse roll, pitch, and yaw.
    sensors::ImuOrientationNoiseMatrix R;
    if (orientation_cov != nullptr) {
      R = sensors::imu_orientation_noise_from_covariance(orientation_cov, fallback);
    } else {
      R = adaptive_initialized_ ? R_imu_orient_ : sensors::imu_orientation_noise_matrix(fallback);
    }

    if (config_.outlier_rejection) {
      sensors::ImuOrientationMeasurement innovation_pre;
      sensors::ImuOrientationNoiseMatrix S;
      ukf_.predict_measurement<sensors::IMU_ORIENTATION_DIM>(
        z, sensors::imu_orientation_measurement_function, R, innovation_pre, S);
      if (is_outlier<sensors::IMU_ORIENTATION_DIM>(innovation_pre, S, config_.outlier_threshold_imu)) {
        ++imu_outliers_;
        last_imu_time_ = timestamp_seconds;
        return;
      }
    }

    constexpr unsigned int IMU_ORIENT_ANGLE_DIMS = 0b100;  // bit 2 = yaw
    auto imu_orient_innovation = ukf_.update<sensors::IMU_ORIENTATION_DIM>(
      z, sensors::imu_orientation_measurement_function, R, IMU_ORIENT_ANGLE_DIMS);

    adapt_R<sensors::IMU_ORIENTATION_DIM>(
      R_imu_orient_, R_imu_orient_floor_, imu_orient_innovations_, imu_orient_innovation, config_.adaptive_imu);
  }

  // IMU orientation validates heading ONLY if the IMU has a magnetometer.
  // 6-axis IMUs integrate gyro for yaw: this drifts and is not a valid
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
  // but heading_validated_ is NOT set: lever arm stays inactive.

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

  // Use message covariance when provided, else adaptive R, else config default
  sensors::EncoderNoiseMatrix R = adaptive_initialized_ ? R_encoder_ : sensors::encoder_noise_matrix(config_.encoder);
  if (var_vx > 0.0) R(0,0) = var_vx;
  if (var_vy > 0.0) R(1,1) = var_vy;
  if (var_wz > 0.0) R(2,2) = var_wz;

  // Mahalanobis outlier rejection for encoder
  if (config_.outlier_rejection) {
    sensors::EncoderMeasurement innovation_pre;
    sensors::EncoderNoiseMatrix S;
    ukf_.predict_measurement<sensors::ENCODER_DIM>(z, sensors::encoder_measurement_function, R, innovation_pre, S);
    if (is_outlier<sensors::ENCODER_DIM>(innovation_pre, S, config_.outlier_threshold_enc)) {
      ++enc_outliers_;
      last_encoder_time_ = timestamp_seconds;
      return;
    }
  }

  auto innovation = ukf_.update<sensors::ENCODER_DIM>(z, sensors::encoder_measurement_function, R);

  last_encoder_innovation_norm_ = innovation.norm();

  // Track innovation for adaptive noise estimation
  // Only adapt axes where message covariance was not provided
  if (var_vx <= 0.0 && var_vy <= 0.0 && var_wz <= 0.0) {
    adapt_R<sensors::ENCODER_DIM>(R_encoder_, R_encoder_floor_, encoder_innovations_, innovation, config_.adaptive_encoder);
  }

  last_encoder_time_ = timestamp_seconds;
  ++update_count_;
}

void FusionCore::update_ground_constraint(double timestamp_seconds) {
  if (!initialized_) return;

  // Force a minimal predict step so Q is injected into P before this update.
  // This prevents Cholesky failure when called back-to-back with update_encoder
  // at the same timestamp (where predict_to would be a no-op and P gets two
  // consecutive reductions with no covariance recovery between them).
  // Do NOT update last_timestamp_ here: advancing it would cause every
  // subsequent GNSS message to be misclassified as delayed (triggering the
  // retrodiction path). The 1µs UKF time mismatch is negligible.
  ukf_.predict(config_.min_dt);

  // ── VZ = 0: body-frame vertical velocity must be zero for ground robots ──
  sensors::GroundConstraintMeasurement z;
  z[0] = 0.0;

  // Use adaptive R if initialized, else fall back to config value.
  // On rough terrain, VZ innovations grow and R_vz_ inflates automatically.
  // On flat ground, it relaxes back to the floor (config value) over ~1 second.
  Eigen::Matrix<double, 1, 1> R_vz;
  R_vz(0,0) = adaptive_initialized_
    ? R_vz_(0,0)
    : (config_.ground_constraint_vz_sigma * config_.ground_constraint_vz_sigma);

  auto vz_innovation = ukf_.update<sensors::GROUND_CONSTRAINT_DIM>(
    z, sensors::ground_constraint_measurement_function, R_vz);
  adapt_R<1>(R_vz_, R_vz_floor_, vz_innovations_, vz_innovation,
             config_.adaptive_ground_constraint);

  // ── AZ = 0: body-frame vertical acceleration must be zero for ground robots.
  // Without this, a mismatch between the IMU's local gravity and the WGS84
  // constant (9.80665) leaks into the AZ state. Because q_acceleration is
  // large (1.0), AZ absorbs the residual instead of B_AZ. AZ then integrates
  // into VZ via the motion model (VZ += AZ*dt), and the VZ=0 constraint above
  // cannot fully compensate because it only fires at encoder rate (~50Hz)
  // while IMU predict runs at ~100Hz. The net effect is continuous Z drift.
  // Constraining AZ directly eliminates the source of the leak.
  Eigen::Matrix<double, 1, 1> z_az;
  z_az[0] = 0.0;

  Eigen::Matrix<double, 1, 1> R_az;
  R_az(0,0) = adaptive_initialized_
    ? R_az_(0,0)
    : (config_.ground_constraint_az_sigma * config_.ground_constraint_az_sigma);

  auto h_az = [](const StateVector& x) -> Eigen::Matrix<double, 1, 1> {
    Eigen::Matrix<double, 1, 1> m;
    m[0] = x[AZ];
    return m;
  };
  auto az_innovation = ukf_.update<1>(z_az, h_az, R_az);
  adapt_R<1>(R_az_, R_az_floor_, az_innovations_, az_innovation,
             config_.adaptive_ground_constraint);

  // ── Z position = 0: flat-terrain pseudo-measurement ─────────────────────
  // When enabled, tells the filter the robot's altitude above its starting
  // reference is ~0. Sigma of 0.3m beats GPS altitude noise (5m std dev on
  // NCLT) and prevents GPS-altitude oscillations from accumulating. Only
  // enable when terrain is genuinely flat (campus, parking lots, warehouses).
  if (config_.ground_z_position_sigma > 0.0) {
    Eigen::Matrix<double, 1, 1> z_pos;
    z_pos[0] = 0.0;
    Eigen::Matrix<double, 1, 1> R_zpos;
    R_zpos(0,0) = config_.ground_z_position_sigma * config_.ground_z_position_sigma;
    auto h_zpos = [](const StateVector& x) -> Eigen::Matrix<double, 1, 1> {
      Eigen::Matrix<double, 1, 1> m;
      m[0] = x[Z];
      return m;
    };
    ukf_.update<1>(z_pos, h_zpos, R_zpos);
  }
}

void FusionCore::update_zupt(double timestamp_seconds, double noise_sigma) {
  if (!initialized_) return;

  predict_to(timestamp_seconds);

  // Fuse [VX=0, VY=0, WZ=0] using the encoder measurement function.
  // This is a pseudo-measurement: the robot asserts it is not moving.
  // Outlier rejection is intentionally skipped: ZUPT is only called when
  // the encoder already confirms near-zero velocity, so rejection would
  // fight against the one thing we know is true.
  sensors::EncoderMeasurement z = sensors::EncoderMeasurement::Zero();

  sensors::EncoderNoiseMatrix R = sensors::EncoderNoiseMatrix::Zero();
  double var = noise_sigma * noise_sigma;
  R(0,0) = var;
  R(1,1) = var;
  R(2,2) = var;

  ukf_.update<sensors::ENCODER_DIM>(z, sensors::zupt_measurement_function, R);
}

bool FusionCore::update_gnss(
  double timestamp_seconds,
  const sensors::GnssFix& fix
) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: update_gnss() called before init()");

  // Always populate what we know from the fix before any gate check
  gnss_debug_.hdop               = fix.hdop;
  gnss_debug_.vdop               = fix.vdop;
  gnss_debug_.satellites         = fix.satellites;
  gnss_debug_.fix_type           = static_cast<int>(fix.fix_type);
  gnss_debug_.chi2_threshold     = config_.outlier_threshold_gnss;
  gnss_debug_.in_coast_mode      = gnss_in_coast_;
  gnss_debug_.consecutive_rejects = gnss_consecutive_rejects_;
  const StateMatrix& P_now = ukf_.state().P;
  gnss_debug_.position_sigma_x   = std::sqrt(std::max(P_now(X, X), 0.0));
  gnss_debug_.position_sigma_y   = std::sqrt(std::max(P_now(Y, Y), 0.0));

  if (!fix.is_valid(config_.gnss)) {
    gnss_debug_.accepted       = false;
    gnss_debug_.mahalanobis_sq = -1.0;
    if (fix.fix_type < config_.gnss.min_fix_type)
      gnss_debug_.reason = GnssRejectionReason::FIX_TYPE_LOW;
    else if (fix.hdop > config_.gnss.max_hdop)
      gnss_debug_.reason = GnssRejectionReason::HDOP_HIGH;
    else if (fix.vdop > config_.gnss.max_vdop)
      gnss_debug_.reason = GnssRejectionReason::VDOP_HIGH;
    else
      gnss_debug_.reason = GnssRejectionReason::MIN_SATS;
    return false;
  }

  // Check if this measurement is delayed
  bool is_delayed = (last_timestamp_ - timestamp_seconds) > config_.min_dt;

  if (is_delayed) {
    bool gnss_fused = false;
    double pre_update_speed_delayed = 0.0;
    bool applied = apply_delayed_measurement(timestamp_seconds, [&]() {
      predict_to(timestamp_seconds);
      pre_update_speed_delayed = std::sqrt(
        ukf_.state().x[VX] * ukf_.state().x[VX] +
        ukf_.state().x[VY] * ukf_.state().x[VY]);
      gnss_fused = apply_gnss_update(timestamp_seconds, fix);
    });
    if (!applied) {
      gnss_debug_.accepted = false;
      gnss_debug_.reason   = GnssRejectionReason::DELAY_TOO_LARGE;
    }
    if (!applied || !gnss_fused) return false;
    update_distance_traveled(fix.x, fix.y, pre_update_speed_delayed);
    last_gnss_time_ = timestamp_seconds;
    ++update_count_;
    return true;
  }

  predict_to(timestamp_seconds);
  double pre_update_speed = std::sqrt(
    ukf_.state().x[VX] * ukf_.state().x[VX] +
    ukf_.state().x[VY] * ukf_.state().x[VY]);
  if (!apply_gnss_update(timestamp_seconds, fix)) return false;
  update_distance_traveled(fix.x, fix.y, pre_update_speed);
  last_gnss_time_ = timestamp_seconds;
  ++update_count_;
  return true;
}

bool FusionCore::apply_gnss_update(
  double timestamp_seconds,
  const sensors::GnssFix& fix)
{
  sensors::GnssPosMeasurement z;
  z[0] = fix.x;
  z[1] = fix.y;
  z[2] = fix.z;

  // Start with message covariance or HDOP-based estimate
  sensors::GnssPosNoiseMatrix R = sensors::gnss_pos_noise_matrix(config_.gnss, fix);

  // Keep a copy of the raw measurement noise before adaptive inflation.
  // Adaptive R captures temporal bias in GPS position (multipath, foliage) and
  // is used to reduce Kalman gain when GPS is unreliable. But sigma_hdg for GPS
  // track heading fusion is a geometric question (is the displacement long enough
  // relative to GPS noise?), which should use the per-fix measurement noise, not
  // the inflated adaptive noise. Using inflated R here would cause high-multipath
  // sequences to stop fusing GPS heading entirely, leaving encoder WZ bias
  // uncorrected for the rest of the mission.
  sensors::GnssPosNoiseMatrix R_meas = R;

  // Inflate R toward the adaptive estimate once the window has enough data.
  // Only inflate: if GPS is actually good, R_gnss_ stays near message R and max() is a no-op.
  // When GPS is consistently biased (multipath, foliage), R_gnss_ reflects the true error
  // magnitude and Kalman gain shrinks accordingly. Full-covariance fixes are left untouched:
  // the receiver already knows its own noise.
  if (adaptive_initialized_ && config_.adaptive_gnss && !fix.has_full_covariance && gnss_innovations_.ready()) {
    for (int i = 0; i < 3; ++i)
      R(i,i) = std::max(R(i,i), R_gnss_(i,i));
  }

  double heading_sigma_rad = compute_heading_sigma_rad();
  double heading_sigma_deg = heading_sigma_rad * 180.0 / M_PI;
  gnss_debug_.heading_sigma_deg = heading_sigma_deg;

  bool heading_reliable = heading_validated_ &&
    (heading_sigma_deg <= config_.gnss_lever_arm_max_heading_sigma_deg);
  // Apply the antenna lever arm when heading is reliable (validated + sigma within
  // bounds) OR when the user opted in to applying it pre-heading-validation.
  // The pre-heading option turns GPS into an active yaw observation from startup,
  // safe when Mahalanobis gating is on and fixes are RTK-grade.
  bool use_lever_arm = !fix.lever_arm.is_zero()
                       && (heading_reliable || config_.gnss.apply_lever_arm_pre_heading);
  gnss_debug_.lever_arm_used = use_lever_arm;

  auto h_gnss = use_lever_arm
    ? sensors::gnss_pos_measurement_function_with_lever_arm(fix.lever_arm)
    : std::function<sensors::GnssPosMeasurement(const StateVector&)>(
        sensors::gnss_pos_measurement_function);

  if (config_.outlier_rejection) {
    sensors::GnssPosMeasurement innovation_pre;
    sensors::GnssPosNoiseMatrix S;
    ukf_.predict_measurement<sensors::GNSS_POS_DIM>(z, h_gnss, R, innovation_pre, S);

    // Compute Mahalanobis distance squared inline so it can be surfaced for observability.
    // This avoids calling is_outlier() which would run a second LDLT internally.
    double d2 = innovation_pre.dot(S.ldlt().solve(innovation_pre));
    gnss_debug_.mahalanobis_sq = d2;

    // Physical plausibility gate: the fix cannot be farther from the predicted
    // position than the robot could have moved or drifted since the last accepted
    // fix (dead-reckoning error <= distance traveled <= max_speed * dt). This
    // catches an adversarial outlier cluster at a blackout boundary that a
    // coast-relaxed chi2 gate would admit. It is checked filter-vs-fix (not
    // GPS-to-GPS) so it scales with the gap and is immune to the cluster being
    // internally self-consistent. Rejected fixes do NOT count toward coast, so
    // an outlier can never inflate P and relax the gate.
    if (config_.gnss_max_speed > 0.0 && last_gnss_time_ >= 0.0) {
      double gap_s = timestamp_seconds - last_gnss_time_;
      double offset_xy = std::sqrt(innovation_pre[0]*innovation_pre[0] +
                                   innovation_pre[1]*innovation_pre[1]);
      double max_offset = config_.gnss_max_speed * std::max(gap_s, 0.0) +
                          config_.gnss_max_speed_margin;
      if (offset_xy > max_offset) {
        ++gnss_outliers_;
        gnss_debug_.accepted = false;
        gnss_debug_.reason   = GnssRejectionReason::IMPLAUSIBLE_JUMP;
        return false;  // do not touch the coast counters: an outlier must not relax the gate
      }
    }

    if (d2 > config_.outlier_threshold_gnss) {
      ++gnss_outliers_;
      gnss_debug_.accepted = false;
      gnss_debug_.reason   = GnssRejectionReason::CHI2_FAILED;

      if (config_.gnss_coast_n > 0) {
        // At the start of a rejection sequence, decide whether GPS was
        // continuous (a persistent outlier like a multipath spike) or is
        // returning after a gap (the filter may have drifted blind). Only the
        // latter justifies inflating P to re-admit GPS. last_gnss_time_ is the
        // last ACCEPTED fix, so the gap to it is small during a continuous
        // spike and large after an outage.
        if (gnss_consecutive_rejects_ == 0) {
          double gap = (last_gnss_time_ < 0.0)
                         ? std::numeric_limits<double>::infinity()
                         : (timestamp_seconds - last_gnss_time_);
          reject_after_gap_ = (gap >= config_.gnss_coast_min_gap_s);
        }
        ++gnss_consecutive_rejects_;
        gnss_debug_.consecutive_rejects = gnss_consecutive_rejects_;
        if (reject_after_gap_ &&
            gnss_consecutive_rejects_ >= config_.gnss_coast_n && !gnss_in_coast_) {
          gnss_in_coast_ = true;
          gnss_debug_.in_coast_mode = true;
          ukf_.set_position_noise_scale(config_.gnss_coast_q_factor);
          ukf_.set_gyro_bias_noise_scale(config_.gnss_coast_q_bias_factor);
        }
        if (reject_after_gap_ &&
            config_.gnss_recovery_rejection_n > 0 &&
            gnss_consecutive_rejects_ == config_.gnss_recovery_rejection_n) {
          double s2 = config_.gnss_p_inflate_sigma * config_.gnss_p_inflate_sigma;
          ukf_.inflate_position_covariance(s2);
        }
      }
      return false;
    }
  } else {
    gnss_debug_.mahalanobis_sq = -1.0;
  }

  // GPS accepted normally: exit coast mode and reset counter
  if (gnss_in_coast_) {
    gnss_in_coast_ = false;
    ukf_.set_position_noise_scale(1.0);
    ukf_.set_gyro_bias_noise_scale(1.0);
  }
  gnss_consecutive_rejects_ = 0;

  Eigen::Matrix<double, sensors::GNSS_POS_DIM, 1> innovation =
    ukf_.update<sensors::GNSS_POS_DIM>(z, h_gnss, R);

  // Update observability state for accepted fix
  gnss_debug_.accepted           = true;
  gnss_debug_.reason             = GnssRejectionReason::ACCEPTED;
  gnss_debug_.in_coast_mode      = false;
  gnss_debug_.consecutive_rejects = 0;
  last_gnss_innovation_norm_     = innovation.norm();

  // Track innovation for adaptive GNSS noise estimation
  adapt_R<sensors::GNSS_POS_DIM>(R_gnss_, R_gnss_floor_, gnss_innovations_, innovation, config_.adaptive_gnss);

  // GPS track heading fusion: fuse the displacement bearing as a yaw update.
  // This is the same mechanism navsat_transform uses for RL-EKF and directly
  // corrects heading from GPS geometry rather than relying on gyro bias estimation.
  // Displacement accumulates across multiple GPS fixes (using a separate reference
  // position that only advances when a heading fusion fires) so the baseline is
  // always large enough for the uncertainty to be meaningful.
  if (config_.gps_track_heading_enabled) {
    if (!hdg_fix_set_) {
      // Initialize reference on first accepted fix; no heading yet.
      last_hdg_fix_x_ = fix.x;
      last_hdg_fix_y_ = fix.y;
      hdg_fix_set_    = true;
    } else {
      double dx   = fix.x - last_hdg_fix_x_;
      double dy   = fix.y - last_hdg_fix_y_;
      double dist = std::sqrt(dx*dx + dy*dy);

      if (dist >= config_.gps_track_heading_min_dist) {
        double sigma_xy  = std::sqrt((R_meas(0,0) + R_meas(1,1)) * 0.5);
        double sigma_hdg = sigma_xy / dist;

        if (sigma_hdg <= config_.gps_track_heading_max_sigma) {
          sensors::GnssHdgMeasurement z_hdg;
          z_hdg[0] = std::atan2(dy, dx);

          sensors::GnssHdgNoiseMatrix R_hdg;
          R_hdg(0,0) = sigma_hdg * sigma_hdg;

          constexpr unsigned int HDG_ANGLE_DIMS = 0b1;

          // Apply chi2 gate only after this fusion has fired at least once.
          // update_distance_traveled() sets heading_validated_=true at 5m (before
          // the 7.5m baseline needed for a reliable bearing), so heading_validated_
          // alone is not a safe guard. A large initial heading error (>75 deg)
          // would then cause every fusion attempt to be rejected, permanently
          // blocking heading correction.
          bool fuse = true;
          if (config_.outlier_rejection && gps_track_hdg_fused_) {
            sensors::GnssHdgMeasurement innov_pre;
            sensors::GnssHdgNoiseMatrix S_pre;
            ukf_.predict_measurement<sensors::GNSS_HDG_DIM>(
              z_hdg, sensors::gnss_hdg_measurement_function, R_hdg, innov_pre, S_pre, HDG_ANGLE_DIMS);
            fuse = !is_outlier<sensors::GNSS_HDG_DIM>(innov_pre, S_pre, config_.outlier_threshold_hdg);
          }

          if (fuse) {
            ukf_.update<sensors::GNSS_HDG_DIM>(
              z_hdg, sensors::gnss_hdg_measurement_function, R_hdg, HDG_ANGLE_DIMS);
            gps_track_hdg_fused_ = true;

            if (!heading_validated_) {
              heading_validated_ = true;
              heading_source_    = HeadingSource::GPS_TRACK;
            }
          }

          // Advance reference only when sigma was acceptable. If sigma was too
          // high (distance not large enough relative to GPS noise), do NOT
          // advance; let the displacement keep accumulating until the baseline
          // is long enough for a reliable heading. With NCLT GPS (σ=3m) and
          // max_sigma=0.4 rad, fusion first fires at ~7.5m of displacement.
          last_hdg_fix_x_ = fix.x;
          last_hdg_fix_y_ = fix.y;
        }
        // else: sigma too high, keep accumulating displacement
      }
    }
  }

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

  // Dimension 0 (heading) is an angle: wrap z_diff across ±π boundary
  constexpr unsigned int HDG_ANGLE_DIMS = 0b1;  // bit 0 = heading

  // Mahalanobis outlier rejection for heading
  if (config_.outlier_rejection) {
    sensors::GnssHdgMeasurement innovation_pre;
    sensors::GnssHdgNoiseMatrix S;
    ukf_.predict_measurement<sensors::GNSS_HDG_DIM>(
      z, sensors::gnss_hdg_measurement_function, R, innovation_pre, S, HDG_ANGLE_DIMS);
    if (is_outlier<sensors::GNSS_HDG_DIM>(innovation_pre, S, config_.outlier_threshold_hdg)) {
      ++hdg_outliers_;
      return false;
    }
  }

  ukf_.update<sensors::GNSS_HDG_DIM>(
    z, sensors::gnss_hdg_measurement_function, R, HDG_ANGLE_DIMS);

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

  const double stale = config_.stale_timeout;

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

  status.vslam_health =
    last_vslam_time_ < 0.0 ? SensorHealth::NOT_INIT :
    (last_timestamp_ - last_vslam_time_) > stale ? SensorHealth::STALE :
    SensorHealth::OK;

  status.mag_health =
    last_mag_time_ < 0.0 ? SensorHealth::NOT_INIT :
    (last_timestamp_ - last_mag_time_) > stale ? SensorHealth::STALE :
    SensorHealth::OK;

  // Outlier rejection counters
  status.gnss_outliers  = gnss_outliers_;
  status.imu_outliers   = imu_outliers_;
  status.enc_outliers   = enc_outliers_;
  status.hdg_outliers   = hdg_outliers_;
  status.vslam_outliers = vslam_outliers_;
  status.mag_outliers   = mag_outliers_;

  // Innovation norms from last accepted update per sensor
  status.gnss_innovation_norm    = last_gnss_innovation_norm_;
  status.imu_innovation_norm     = last_imu_innovation_norm_;
  status.encoder_innovation_norm = last_encoder_innovation_norm_;

  // Position 1-sigma from diagonal of P
  status.position_sigma_x = std::sqrt(std::max(P(0, 0), 0.0));
  status.position_sigma_y = std::sqrt(std::max(P(1, 1), 0.0));
  status.position_sigma_z = std::sqrt(std::max(P(2, 2), 0.0));

  // GPS coast mode
  status.gnss_in_coast           = gnss_in_coast_;
  status.gnss_consecutive_rejects = gnss_consecutive_rejects_;

  return status;
}

bool FusionCore::update_pose(
  double timestamp_seconds,
  const sensors::VslamPose& pose)
{
  if (!initialized_)
    throw std::runtime_error("FusionCore: update_pose() called before init()");

  bool is_delayed = (last_timestamp_ - timestamp_seconds) > config_.min_dt;

  if (is_delayed) {
    bool fused = false;
    bool applied = apply_delayed_measurement(timestamp_seconds, [&]() {
      predict_to(timestamp_seconds);

      sensors::VslamPoseMeasurement z;
      z[0] = pose.x; z[1] = pose.y; z[2] = pose.z;
      z[3] = pose.roll; z[4] = pose.pitch; z[5] = pose.yaw;

      sensors::VslamPoseNoiseMatrix R = sensors::vslam_pose_noise_matrix(config_.vslam, pose);

      // bit 5 = yaw (index 5 in the measurement vector): wrap across ±π
      constexpr unsigned int VSLAM_ANGLE_DIMS = 0b100000;

      if (config_.outlier_rejection) {
        sensors::VslamPoseMeasurement innov_pre;
        sensors::VslamPoseNoiseMatrix S;
        ukf_.predict_measurement<sensors::VSLAM_POSE_DIM>(
          z, sensors::vslam_pose_measurement_function, R, innov_pre, S, VSLAM_ANGLE_DIMS);
        if (is_outlier<sensors::VSLAM_POSE_DIM>(innov_pre, S, config_.outlier_threshold_vslam)) {
          ++vslam_outliers_;
          return;
        }
      }

      auto innovation = ukf_.update<sensors::VSLAM_POSE_DIM>(
        z, sensors::vslam_pose_measurement_function, R, VSLAM_ANGLE_DIMS);
      adapt_R<sensors::VSLAM_POSE_DIM>(R_vslam_, R_vslam_floor_, vslam_innovations_, innovation, false);
      fused = true;
    });
    if (!applied || !fused) return false;
    update_distance_traveled(pose.x, pose.y);
    last_vslam_time_ = timestamp_seconds;
    ++update_count_;
    return true;
  }

  predict_to(timestamp_seconds);

  sensors::VslamPoseMeasurement z;
  z[0] = pose.x; z[1] = pose.y; z[2] = pose.z;
  z[3] = pose.roll; z[4] = pose.pitch; z[5] = pose.yaw;

  sensors::VslamPoseNoiseMatrix R = sensors::vslam_pose_noise_matrix(config_.vslam, pose);

  // bit 5 = yaw (index 5): wrap across ±π
  constexpr unsigned int VSLAM_ANGLE_DIMS = 0b100000;

  if (config_.outlier_rejection) {
    sensors::VslamPoseMeasurement innov_pre;
    sensors::VslamPoseNoiseMatrix S;
    ukf_.predict_measurement<sensors::VSLAM_POSE_DIM>(
      z, sensors::vslam_pose_measurement_function, R, innov_pre, S, VSLAM_ANGLE_DIMS);
    if (is_outlier<sensors::VSLAM_POSE_DIM>(innov_pre, S, config_.outlier_threshold_vslam)) {
      ++vslam_outliers_;
      return false;
    }
  }

  auto innovation = ukf_.update<sensors::VSLAM_POSE_DIM>(
    z, sensors::vslam_pose_measurement_function, R, VSLAM_ANGLE_DIMS);
  adapt_R<sensors::VSLAM_POSE_DIM>(R_vslam_, R_vslam_floor_, vslam_innovations_, innovation, false);

  update_distance_traveled(pose.x, pose.y);
  last_vslam_time_ = timestamp_seconds;
  ++update_count_;

  // Validate heading from VSLAM travel (same path as GPS track)
  if (!heading_validated_ &&
      heading_source_ == HeadingSource::NONE) {
    if (distance_traveled_ >= config_.heading_observable_distance) {
      heading_validated_ = true;
      heading_source_    = HeadingSource::GPS_TRACK;
    }
  }

  return true;
}

bool FusionCore::update_magnetometer(
  double timestamp_seconds,
  double mx, double my, double mz)
{
  if (!initialized_)
    throw std::runtime_error("FusionCore: update_magnetometer() called before init()");

  predict_to(timestamp_seconds);

  // Reject readings taken in a locally disturbed field (motor, steel, rebar):
  // the magnitude no longer matches Earth's field, so the tilt-compensated
  // heading is untrustworthy in a way the 1-DOF chi2 gate below cannot reliably
  // catch. See sensors::mag_field_disturbed. Disabled when field_strength <= 0.
  if (sensors::mag_field_disturbed(mx, my, mz, config_.mag)) {
    ++mag_outliers_;
    return false;
  }

  // Extract current roll and pitch from the filter state for tilt compensation.
  // Yaw is what we are about to measure, so we only need roll and pitch here.
  const State& s = ukf_.state();
  double roll, pitch, yaw_state;
  quat_to_euler(s.x[QW], s.x[QX], s.x[QY], s.x[QZ], roll, pitch, yaw_state);

  // Compute tilt-compensated heading from raw field vector
  double yaw_mag = sensors::mag_yaw_from_field(mx, my, mz, config_.mag, roll, pitch);

  // Fuse as a 1-DOF heading measurement, same path as dual-antenna GPS heading
  sensors::GnssHdgMeasurement z;
  z[0] = yaw_mag;

  sensors::GnssHdgNoiseMatrix R;
  R(0,0) = config_.mag.noise_rad * config_.mag.noise_rad;

  // bit 0 = dimension 0 (heading) is an angle: wrap innovation across +-pi
  constexpr unsigned int MAG_ANGLE_DIMS = 0b1;

  if (config_.outlier_rejection) {
    sensors::GnssHdgMeasurement innov_pre;
    sensors::GnssHdgNoiseMatrix S;
    ukf_.predict_measurement<sensors::GNSS_HDG_DIM>(
      z, sensors::gnss_hdg_measurement_function, R, innov_pre, S, MAG_ANGLE_DIMS);
    if (is_outlier<sensors::GNSS_HDG_DIM>(innov_pre, S, config_.mag.chi2_threshold)) {
      ++mag_outliers_;
      return false;
    }
  }

  ukf_.update<sensors::GNSS_HDG_DIM>(
    z, sensors::gnss_hdg_measurement_function, R, MAG_ANGLE_DIMS);

  // Magnetometer immediately provides valid heading.
  // Upgrade from GPS_TRACK (which requires 5m of motion) but never downgrade
  // from DUAL_ANTENNA (which is a stronger absolute source).
  if (!heading_validated_ ||
      heading_source_ == HeadingSource::NONE ||
      heading_source_ == HeadingSource::GPS_TRACK) {
    heading_validated_ = true;
    heading_source_    = HeadingSource::MAGNETOMETER;
  }

  last_mag_time_ = timestamp_seconds;
  ++update_count_;
  return true;
}

} // namespace fusioncore
