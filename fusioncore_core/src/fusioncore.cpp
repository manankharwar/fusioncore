#include "fusioncore/fusioncore.hpp"
#include "fusioncore/sensors/imu.hpp"
#include <stdexcept>
#include <cmath>

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

  // Save floors: adaptive R must never drop below the initially configured sensor noise.
  R_imu_floor_        = R_imu_;
  R_encoder_floor_    = R_encoder_;
  R_gnss_floor_       = R_gnss_;
  R_imu_orient_floor_ = R_imu_orient_;

  imu_innovations_.max_size         = config_.adaptive_window;
  encoder_innovations_.max_size     = config_.adaptive_window;
  gnss_innovations_.max_size        = config_.adaptive_window;
  imu_orient_innovations_.max_size  = config_.adaptive_window;

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

  // Reset snapshot buffer
  snapshot_buffer_.clear();
  imu_buffer_.clear();

  // Reset coast mode state
  gnss_consecutive_rejects_ = 0;
  gnss_in_coast_            = false;
  ukf_.set_position_noise_scale(1.0);

  // Initialize adaptive noise matrices
  init_adaptive_R();
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
  snapshot_buffer_.clear();
  imu_buffer_.clear();
  gnss_consecutive_rejects_ = 0;
  gnss_in_coast_            = false;
  ukf_.set_position_noise_scale(1.0);
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

      // Re-apply the IMU measurement so the filter sees the real dynamics
      sensors::ImuMeasurement z;
      z[0] = imu.wx; z[1] = imu.wy; z[2] = imu.wz;
      z[3] = imu.ax; z[4] = imu.ay; z[5] = imu.az;
      ukf_.update<sensors::IMU_DIM>(z, sensors::imu_measurement_function, imu.R);
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

void FusionCore::predict_to(double timestamp_seconds) {
  double dt = timestamp_seconds - last_timestamp_;
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

  // Minimum forward speed to count as real motion
  // Below this threshold: could be GPS jitter, spinning in place, or sliding
  const double MIN_SPEED = 0.2;  // m/s

  // Maximum yaw rate: if spinning fast, heading is not observable from track
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

  // Use adaptive R if initialized, else config default
  sensors::ImuNoiseMatrix R = adaptive_initialized_ ? R_imu_ : sensors::imu_noise_matrix(config_.imu);

  // Mahalanobis outlier rejection for IMU
  if (config_.outlier_rejection) {
    sensors::ImuMeasurement innovation_pre;
    sensors::ImuNoiseMatrix S;
    ukf_.predict_measurement<sensors::IMU_DIM>(z, sensors::imu_measurement_function, R, innovation_pre, S);
    if (is_outlier<sensors::IMU_DIM>(innovation_pre, S, config_.outlier_threshold_imu)) {
      ++imu_outliers_;
      last_imu_time_ = timestamp_seconds;
      return;
    }
  }

  auto innovation = ukf_.update<sensors::IMU_DIM>(z, sensors::imu_measurement_function, R);

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

  sensors::GroundConstraintNoiseMatrix R = sensors::ground_constraint_noise_matrix();
  ukf_.update<sensors::GROUND_CONSTRAINT_DIM>(
    z, sensors::ground_constraint_measurement_function, R);

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
  R_az(0,0) = 0.25;  // 0.5 m/s² sigma: loose enough for bumps and ramps
  auto h_az = [](const StateVector& x) -> Eigen::Matrix<double, 1, 1> {
    Eigen::Matrix<double, 1, 1> m;
    m[0] = x[AZ];
    return m;
  };
  ukf_.update<1>(z_az, h_az, R_az);
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

  ukf_.update<sensors::ENCODER_DIM>(z, sensors::encoder_measurement_function, R);
}

bool FusionCore::update_gnss(
  double timestamp_seconds,
  const sensors::GnssFix& fix
) {
  if (!initialized_)
    throw std::runtime_error("FusionCore: update_gnss() called before init()");

  if (!fix.is_valid(config_.gnss)) return false;

  // Check if this measurement is delayed
  bool is_delayed = (last_timestamp_ - timestamp_seconds) > config_.min_dt;

  if (is_delayed) {
    // apply_delayed_measurement rolls back state, calls the lambda, then
    // replays IMU forward. Capture gnss_fused so we only count fusions that
    // actually passed the outlier gate (apply_gnss_update returns bool).
    bool gnss_fused = false;
    double pre_update_speed_delayed = 0.0;
    bool applied = apply_delayed_measurement(timestamp_seconds, [&]() {
      predict_to(timestamp_seconds);
      pre_update_speed_delayed = std::sqrt(
        ukf_.state().x[VX] * ukf_.state().x[VX] +
        ukf_.state().x[VY] * ukf_.state().x[VY]);
      gnss_fused = apply_gnss_update(timestamp_seconds, fix);
    });
    if (!applied || !gnss_fused) return false;
    update_distance_traveled(fix.x, fix.y, pre_update_speed_delayed);
    last_gnss_time_ = timestamp_seconds;
    ++update_count_;
    return true;
  }

  predict_to(timestamp_seconds);
  // Fix 8: capture speed BEFORE gnss update: post-update velocity is corrected
  // and not representative of motion during this GPS step.
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

  // Blend with adaptive R if the fix does NOT have full message covariance
  // (if it does have full covariance, trust the receiver: don't override)
  if (adaptive_initialized_ && config_.adaptive_gnss && !fix.has_full_covariance) {
    R = (1.0 - config_.adaptive_alpha) * R + config_.adaptive_alpha * R_gnss_;
    // Guard diagonal
    for (int i = 0; i < 3; ++i)
      if (R(i,i) < 1e-6) R(i,i) = 1e-6;
  }

  bool use_lever_arm = !fix.lever_arm.is_zero() && heading_validated_;

  auto h_gnss = use_lever_arm
    ? sensors::gnss_pos_measurement_function_with_lever_arm(fix.lever_arm)
    : std::function<sensors::GnssPosMeasurement(const StateVector&)>(
        sensors::gnss_pos_measurement_function);

  // Mahalanobis outlier rejection for GNSS position
  if (config_.outlier_rejection) {
    sensors::GnssPosMeasurement innovation_pre;
    sensors::GnssPosNoiseMatrix S;
    ukf_.predict_measurement<sensors::GNSS_POS_DIM>(z, h_gnss, R, innovation_pre, S);
    if (is_outlier<sensors::GNSS_POS_DIM>(innovation_pre, S, config_.outlier_threshold_gnss)) {
      ++gnss_outliers_;
      // Inertial coast mode: after N consecutive rejections, inflate Q_position
      // so P grows and the gate naturally relaxes when GPS recovers.
      if (config_.gnss_coast_n > 0) {
        ++gnss_consecutive_rejects_;
        if (gnss_consecutive_rejects_ >= config_.gnss_coast_n && !gnss_in_coast_) {
          gnss_in_coast_ = true;
          ukf_.set_position_noise_scale(config_.gnss_coast_q_factor);
        }
      }
      return false;
    }
  }

  // GPS accepted: exit coast mode and reset counter
  if (gnss_in_coast_) {
    gnss_in_coast_ = false;
    ukf_.set_position_noise_scale(1.0);
  }
  gnss_consecutive_rejects_ = 0;

  Eigen::Matrix<double, sensors::GNSS_POS_DIM, 1> innovation =
    ukf_.update<sensors::GNSS_POS_DIM>(z, h_gnss, R);

  // Track innovation for adaptive GNSS noise estimation
  adapt_R<sensors::GNSS_POS_DIM>(R_gnss_, R_gnss_floor_, gnss_innovations_, innovation, config_.adaptive_gnss);
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

  // Outlier rejection counters
  status.gnss_outliers = gnss_outliers_;
  status.imu_outliers  = imu_outliers_;
  status.enc_outliers  = enc_outliers_;
  status.hdg_outliers  = hdg_outliers_;

  return status;
}

} // namespace fusioncore
