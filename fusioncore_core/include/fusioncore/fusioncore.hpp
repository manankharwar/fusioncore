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
  // Fixed slack added to the max_speed * dt bound (m), covering prediction error
  // that is not explained by the receiver's own noise.
  double gnss_max_speed_margin = 5.0;
  // Multiples of the receiver's REPORTED horizontal sigma added to the bound.
  //
  // Without this the whole bound is absolute metres, and a fixed margin cannot
  // tell an impossible jump from ordinary noise unless it happens to sit well
  // outside the receiver's spread. Measured 2026-08-03 on a u-blox M9N: the
  // bound worked out to 7 m at 1 Hz while the receiver's own sigma was ~6 m, so
  // the gate sat inside the noise distribution and rejected 157 of 500 perfectly
  // good fixes, turning a 2.62 m loop closure into 7.27 m.
  //
  // Deliberately scaled by the RECEIVER's sigma and not by the filter's P. The
  // entire point of this gate is to catch an outlier cluster that a coast-relaxed
  // chi2 would admit, and chi2 is already the P-scaled test. Scaling this one by
  // P too would just be a second chi2 and would reopen the hole it exists to plug.
  double gnss_max_speed_sigma_k = 5.0;
  // Multiples of the FILTER's own position sigma added to the bound.
  //
  // The original design assumed dead-reckoning error is bounded by distance
  // travelled, so max_speed * gap covered both how far the robot could drive AND
  // how far the prediction could be wrong. That is false: a filter whose heading
  // has drifted goes sideways further than it went forward, and after a GPS
  // blackout the prediction error is the dominant term. Measured on NCLT
  // 2013-04-05, which has a 34 s outage at t+620 s: with the gate off the filter
  // drifts to 98 m during the outage and is back to 4.4 m a minute later, but
  // with the gate on at 3.0 the FIRST post-outage fix is rejected, the filter
  // keeps drifting, and by the time the bound would admit a fix the chi2 gate
  // refuses it. One rejection at the wrong moment locks GPS out permanently:
  // 4.4 m becomes 358 m and climbing.
  //
  // Earlier revisions of this comment argued the bound must NOT scale with P,
  // on the grounds that chi2 is already the P-scaled test and coast inflation
  // would reopen the hole this gate plugs. That reasoning was wrong. Coast only
  // inflates P after a genuine GPS GAP (the gap-gating in fede6e0), which is
  // exactly the situation where a wide bound is correct. A sustained spike with
  // no preceding gap never triggers coast, so P stays tight and the spike is
  // still rejected. P is precisely the right quantity here.
  double gnss_max_speed_drift_k = 3.0;

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

  // Position process noise scale applied while ZUPT holds the robot stationary.
  // 1.0 keeps the previous behaviour exactly; below 1.0 stops the position
  // covariance growing while the wheels say the robot is not moving.
  //
  // ZUPT fuses [VX=0, VY=0, WZ=0]: it pins VELOCITY, and says nothing about
  // position. So a parked robot's P kept growing from process noise between
  // fixes, the Kalman gain stayed high, and the filter chased a wandering
  // receiver. Measured on the 2026-09-07 bags: parked for 57 s with the
  // encoders confirming stillness, the receiver's reported position moved
  // 9.76 m and the fused position followed it for 7.18 m, i.e. 74% of pure
  // GNSS wander on a robot that did not move at all.
  //
  // With process noise suppressed while stationary, P decays as fixes arrive
  // instead of being held up, the gain falls, and consecutive fixes average
  // rather than drag. Martin Pecka put the property well on ROS Discourse:
  // a parked robot's estimate should not drift toward the GNSS mean.
  double zupt_position_noise_scale = 1.0;

  // UPPER BOUND on how much less to believe GNSS while the wheels say the robot
  // is stationary. 1.0 disables it entirely and keeps the previous behaviour.
  //
  // This is a CAP on a MEASURED quantity, not a fixed policy, and the difference
  // is the whole point. A fixed "always distrust GNSS 100x while parked" is
  // right for one receiver in one place and wrong everywhere else, which is the
  // same defect that has bitten every absolute-threshold GNSS gate in this
  // project. Standing still is the one moment in a run where the filter can
  // check a sensor against evidence instead of against a tuned constant,
  // because every fix is then sampling the SAME physical point. Two things get
  // measured from those fixes, per axis:
  //
  //   1. MAGNITUDE. Their spread is the receiver's real short-term sigma. Divide
  //      by the sigma it declares and square: a receiver that is as good as it
  //      claims scores 1 and is left alone.
  //
  //   2. CORRELATION, and this one is usually the larger of the two. A Kalman
  //      filter assumes measurement errors are white, so N fixes of a fixed
  //      point shrink its uncertainty like sqrt(N). GNSS error is not white:
  //      multipath, ionosphere and satellite geometry drift over minutes, so
  //      consecutive fixes are largely the same error repeated. Measure the
  //      lag-1 autocorrelation r of the parked fixes and the honest effective
  //      sample size is N(1-r)/(1+r), so R has to carry a factor (1+r)/(1-r) for
  //      the filter's own posterior to mean what it says. At the 0.95 to 0.99
  //      typical of a parked consumer receiver at 1 Hz, that alone is 39x to
  //      199x, and it is why a hand-picked 100 happened to work.
  //
  // The applied inflation is the product, capped here. So this number bounds how
  // far one bad parked window can push the filter and does NOT decide the
  // amount, and there is nothing to retune per environment or per receiver.
  //
  // Note that (2) fires for an honest receiver too, and should: an RTK unit
  // reporting a correct 2 cm sigma still has errors correlated over minutes, so
  // a filter that parks for a minute and averages 60 of them ends up far more
  // confident than the geometry supports, and carries that over-confidence into
  // the next leg. That is a general defect, not a bad-receiver defect.
  //
  // Measured origin: on the 2026-09-07 bags the receiver's reported position
  // moved 9.76 m over 57 parked seconds while declaring 3.6 m of accuracy.
  // Suppressing position process noise alone (zupt_position_noise_scale) got the
  // fused drift down from 10.16 m, but no further than 1.64 m mean across 11
  // parked windows, because the filter still weighed that receiver by its own
  // stated covariance.
  //
  // THE COST, and it is real: a robot parked for a long time cannot re-acquire
  // if it was genuinely lost before it stopped. For a stop of tens of seconds
  // that does not matter. For one parked overnight it does.
  double zupt_gnss_noise_scale = 1.0;

  // Parked fixes needed before their spread is treated as a measurement of the
  // receiver's noise rather than as noise itself. Floored at 3 internally, since
  // the autocorrelation term needs at least two consecutive pairs.
  int zupt_gnss_min_samples = 5;

  // Catch wheel odometry that has died while the robot is still driving.
  //
  // ZUPT fires when the encoders report near-zero velocity. Encoders that lose
  // power do not go silent, they report ZERO, which is indistinguishable from a
  // parked robot. The two settings above then make that far worse than it used
  // to be: the filter holds its position covariance down AND distrusts GNSS by
  // up to the cap, so the robot drives away while the estimate sits still,
  // actively ignoring the GPS that is telling it otherwise. On this rover a
  // loose breadboard power rail took out all four encoders at once, and that
  // rail is shared, so it is a single point of failure.
  //
  // Displacement alone cannot tell the two apart: a genuinely parked receiver
  // wandered 12.85 m over 57 s on the 2026-09-07 log. What separates them is
  // STRAIGHTNESS, the ratio of net displacement to the path length through the
  // fixes. A parked receiver wanders and returns, so its path is much longer
  // than its displacement: measured 0.37 on that same window. A robot actually
  // driving goes one way, so the two converge on 1.0.
  //
  // Metres of net displacement before the check can fire. 0.0 disables it.
  // Only active while the ZUPT suppression above is doing something.
  double zupt_parked_motion_m = 5.0;
  // Straightness above which the displacement is real motion, not wander.
  //
  // 0.85, and the margin is measured rather than chosen. Across three genuinely
  // parked windows on the 2026-09 rover logs the worst straightness reached, at
  // any point where displacement exceeded the threshold above, was:
  //     2026-09-07, 57 s, 12.85 m of wander   0.59
  //     2026-09-05, 30 s,  2.70 m             0.00 (never reached 5 m)
  //     2026-09-05, 27 s,  8.83 m             0.72   <- the one that matters
  // A first attempt at 0.70 would have FIRED on that third window and disabled
  // the idle-drift fix on a robot that was sitting still. A robot genuinely
  // driving on dead encoders tracks close to 1.0, so 0.85 leaves room on both
  // sides. Note the parked worst case is not small: a receiver whose error
  // drifts one way under changing satellite geometry looks quite straight.
  double zupt_parked_motion_straightness = 0.85;

  // Nominal IMU rate in Hz. Above zero, the PREDICT step between IMU messages
  // advances by exactly 1/rate instead of the gap between two stamps. Zero
  // keeps the previous behaviour.
  //
  // Anything downstream that integrates amplifies stamp error: shifting every
  // IMU stamp on one recorded run by a single MICROSECOND changed final yaw by
  // 109 degrees. Most of that is an unbounded quaternion covariance rather than
  // the stamps, but a nominal dt removes the input sensitivity entirely. Martin
  // Pecka's team does exactly this, never computing dt in fusion from IMU
  // timestamps.
  //
  // THE HAZARD, which is why FusionCoreStatus reports a mismatch: if the rate
  // is wrong the filter integrates the wrong amount of time on EVERY step, and
  // that error is systematic rather than noisy. A BNO085 nominally at 100 Hz
  // was logged at 103 and 109. Only set a rate you have measured.
  double imu_fixed_rate_hz = 0.0;

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

// How often each outcome happened, and when it first and last did.
//
// A single "last reason" field is a snapshot and answers almost nothing. It
// cannot say whether a gate ever fired, how many times, or when it started,
// because every fix overwrites it. That gap is what made a whole run look
// healthy on 2026-09-06: every fix reported ACCEPTED while the chi2 gate sat 39x
// below its threshold and could not have rejected anything. Establishing that
// took a day of replaying bags and injecting synthetic spikes; a per-reason count
// would have said "CHI2_FAILED: 0 of 222" straight out of the recording.
//
// Timestamps are the filter clock, in seconds, and are -1.0 until the outcome
// has happened at least once.
struct OutcomeTally {
  int    count      = 0;
  double first_seen = -1.0;
  double last_seen  = -1.0;
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
  HDOP_HIGH       = 3,  // hdop > max_hdop (dimensionless DOP path only)
  VDOP_HIGH       = 4,  // vdop > max_vdop (dimensionless DOP path only)
  MIN_SATS        = 5,  // satellites < min_satellites
  CHI2_FAILED     = 6,  // Mahalanobis distance > threshold
  DELAY_TOO_LARGE = 7,  // measurement older than max_measurement_delay
  IMPLAUSIBLE_JUMP = 8, // fix farther from prediction than max_speed*dt allows
  SIGMA_XY_HIGH   = 9,  // reported horizontal sigma in METRES > max_sigma_xy
  SIGMA_Z_HIGH    = 10, // reported vertical sigma in METRES > max_sigma_z
  CONTINUITY_BREAK = 11, // fix disagrees with the two fixes before it
  NOT_FINITE      = 12, // position or covariance contained NaN or infinity
};

// Why GPS track heading did or did not fuse on a given fix.
//
// Heading has no absolute source on a rover with no magnetometer, no dual antenna
// and an IMU that publishes orientation as invalid, so GPS track is the only thing
// that can bound yaw. When it silently declines to fire, yaw uncertainty grows
// without limit, and that single number then disables the lever arm, inflates the
// position covariance and drives NIS low enough that the chi2 outlier gate can no
// longer fire. Measured on the 2026-09-06 field run: yaw 1-sigma reached 101 deg,
// the lever arm was applied on 0 of 222 fixes, and the largest innovation of the
// whole run sat 39x below the rejection threshold. Two booleans already recorded
// part of this and were never published, so a bag could not say which gate was
// responsible. This enum covers every branch and is published.
enum class TrackHeadingState {
  NOT_ATTEMPTED    = 0,  // feature disabled, or no fix processed yet
  FUSED            = 1,  // a heading measurement was applied to the filter
  STRONGER_SOURCE  = 2,  // dual antenna / magnetometer / IMU orientation already owns heading
  MOTION_UNSUITABLE = 3, // too slow, or turning faster than track_heading_max_yaw_rate
  BASELINE_SHORT   = 4,  // displacement since the reference fix < track_heading_min_dist
  SIGMA_HIGH       = 5,  // sigma_xy/dist > track_heading_max_sigma, bearing too uncertain
  CHI2_FAILED      = 6,  // bearing computed but rejected as an outlier
};

// Sizes for the tally arrays, which are indexed by static_cast<int>(reason).
// The static_asserts below hold these to the enums, so adding a reason without
// bumping the count fails the build instead of silently going uncounted.
constexpr int GNSS_REJECTION_REASON_COUNT = 12;
constexpr int MAG_REJECTION_REASON_COUNT  = 4;

// Why a magnetometer reading was rejected (or ACCEPTED if it passed).
enum class MagRejectionReason {
  NOT_PROCESSED    = 0,
  ACCEPTED         = 1,
  CHI2_FAILED      = 2,  // Mahalanobis distance > threshold
  FIELD_MAGNITUDE  = 3,  // corrected field magnitude outside configured range
};

static_assert(static_cast<int>(GnssRejectionReason::CONTINUITY_BREAK) + 1 ==
              GNSS_REJECTION_REASON_COUNT,
              "GNSS_REJECTION_REASON_COUNT must match GnssRejectionReason");
static_assert(static_cast<int>(MagRejectionReason::FIELD_MAGNITUDE) + 1 ==
              MAG_REJECTION_REASON_COUNT,
              "MAG_REJECTION_REASON_COUNT must match MagRejectionReason");

// Per-fix observability data: populated by update_gnss() on every call.
// Retrieve via get_gnss_debug() after update_gnss() returns.
struct GnssFixDebug {
  bool               accepted           = false;
  GnssRejectionReason reason            = GnssRejectionReason::NOT_PROCESSED;
  double             mahalanobis_sq     = -1.0;  // -1 = not computed (quality gate failed first)
  double             chi2_threshold     = 0.0;
  // Why GPS track heading did not fuse on this fix, when it did not.
  // Silent skipping is how the original problem stayed invisible: the user saw a
  // zig-zag path and had no way to tell which heading source caused it.
  bool               track_heading_skipped_stronger_source = false;
  bool               track_heading_skipped_motion          = false;
  // The same question answered completely, including the two cases the booleans
  // above never covered: baseline too short, and bearing sigma too high.
  TrackHeadingState  track_heading_state = TrackHeadingState::NOT_ATTEMPTED;
  double             track_heading_baseline_m = 0.0;  // displacement since the reference fix
  double             track_heading_sigma_rad  = 0.0;  // sigma_xy/dist, -1 if not computed
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

// Per-reading observability data: populated by update_magnetometer() on every call.
struct MagnetometerDebug {
  bool              accepted       = false;
  MagRejectionReason reason        = MagRejectionReason::NOT_PROCESSED;
  double            mahalanobis_sq = -1.0;
  double            chi2_threshold  = 0.0;
  double            measured_field  = 0.0;
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

  // True once the IMU and the wheel encoders have disagreed about the SIGN of the
  // yaw rate, consistently, while the robot was genuinely turning. One of the two
  // has a frame convention wrong. Latches: it describes the setup, not the moment.
  bool          yaw_rate_sign_conflict = false;
  double        yaw_rate_imu_mean      = 0.0;  // rad/s, over the samples that voted
  double        yaw_rate_encoder_mean  = 0.0;  // rad/s, same samples
  double        yaw_rate_disagree_frac = 0.0;  // of samples where BOTH were turning
  int           yaw_rate_turn_samples  = 0;    // how many that was

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

  // Reason the most recent GNSS fix was rejected (NOT_PROCESSED until the first
  // rejection). Quality-gate rejects (HDOP/VDOP/fix-type/sats) and delay rejects
  // do NOT increment gnss_outliers, so this is the only place they are reported.
  // Largest Mahalanobis distance seen by the GNSS outlier gate, -1 before any
  // fix has been judged, alongside the threshold it is compared against and the
  // number of fixes behind it.
  // Observed IMU rate, and whether it disagrees with imu_fixed_rate_hz enough
  // that the filter is integrating the wrong amount of time per step.
  double imu_rate_observed_hz    = -1.0;
  bool   imu_fixed_rate_mismatch = false;
  // What the receiver was measured to be while the wheels confirmed the robot
  // was parked, and the inflation that measurement earned. -1 before enough
  // parked fixes have been seen.
  //
  //   sigma_observed vs sigma_declared: is the receiver as good as it claims?
  //   correlation:    lag-1 autocorrelation of the parked fixes. Near 0 means
  //                   consecutive fixes are independent and averaging them is
  //                   worth what the filter assumes. Near 1 means they are
  //                   nearly the same error repeated, so averaging N of them
  //                   buys far less than sqrt(N) and the filter is otherwise
  //                   over-converging on a stationary robot.
  //   inflation:      what was actually applied, horizontal, after the cap.
  double gnss_parked_sigma_observed = -1.0;
  double gnss_parked_sigma_declared = -1.0;
  double gnss_parked_correlation    = 0.0;
  double gnss_parked_inflation      = 1.0;
  // True when ZUPT says parked but the GNSS fixes are moving in a straight line,
  // which means the wheel odometry is lying. Published so it is visible in a bag
  // rather than only in a log line nobody was watching.
  bool   zupt_parked_but_moving     = false;
  double zupt_parked_straightness   = 0.0;
  double gnss_chi2_max = -1.0;
  double gnss_chi2_threshold = 0.0;
  int    gnss_chi2_samples = 0;
  GnssRejectionReason gnss_last_rejection_reason = GnssRejectionReason::NOT_PROCESSED;
  MagRejectionReason mag_last_rejection_reason = MagRejectionReason::NOT_PROCESSED;

  // Stale-measurement rejections from inter-sensor clock skew: this sensor's
  // stamps run more than max_measurement_delay behind the filter clock while
  // another sensor's stamps drive it ahead (sensors not on a common time base).
  // A climbing counter here means that sensor is NOT being fused at all and the
  // sensor drivers' clocks need fixing. These are not outliers: the data may be
  // perfect, it is the timestamps that disagree.
  int imu_stale_rejects     = 0;
  int encoder_stale_rejects = 0;
  int mag_stale_rejects     = 0;
  int hdg_stale_rejects     = 0;
};

class FusionCore {
public:
  explicit FusionCore(const FusionCoreConfig& config = FusionCoreConfig{});

  void init(const State& initial_state, double timestamp_seconds);

  // Runtime updater for the IMU lever arm: the ROS wrapper calls this
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

  // Diagnostic passthrough: metres the last measurement update moved position.
  // See UKF::last_position_correction().
  double last_position_correction() const { return ukf_.last_position_correction(); }
  FusionCoreStatus   get_status()     const;

  // Per-outcome tallies, indexed by static_cast<int>(the reason enum).
  // Counts every fix, accepted included, so "the gate never fired" and
  // "no fix ever arrived" are distinguishable. Reset by init() and reset().
  const std::array<OutcomeTally, GNSS_REJECTION_REASON_COUNT>&
    gnss_outcome_tally() const { return gnss_tally_; }
  const std::array<OutcomeTally, MAG_REJECTION_REASON_COUNT>&
    mag_outcome_tally() const { return mag_tally_; }
  const GnssFixDebug& get_gnss_debug() const { return gnss_debug_; }
  const MagnetometerDebug& get_magnetometer_debug() const { return mag_debug_; }
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
  // Fix-to-fix continuity: the last few ACCEPTED fixes, oldest first, used to
  // predict where the next one should land.
  //
  // Only accepted fixes go in, so a REJECTED spike can never become the
  // reference that makes the next good fix look like a break. That was never the
  // whole problem though. A spike small enough to pass the limit still gets in,
  // and with a two-point extrapolation (px = x1 + (x1 - x2) * r) an error in the
  // newest reference point lands in the prediction multiplied by about two. So a
  // 1.5 m spike passed a 3 m limit and then threw the NEXT good fix over it:
  // measured on the 2026-09-07 rover log, the gate accepted the spike and
  // rejected the good fix after it, which is worse than not gating at all.
  //
  // A least-squares line over CONT_HISTORY points fixes that by arithmetic. For
  // five evenly spaced points extrapolating one step, the weight on the newest
  // is 0.8 rather than 2.0, so a spike that passes the limit can only move the
  // next prediction by 0.8 of itself and can no longer reach the limit. Five is
  // the smallest history where that holds: four gives exactly 1.0, which is
  // borderline, and three gives 1.33, which is not enough.
  static constexpr int CONT_HISTORY = 5;
  std::array<double, CONT_HISTORY> cont_x_{}, cont_y_{}, cont_t_{};
  int cont_n_ = 0;
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
  MagnetometerDebug mag_debug_;

  // Last accepted innovation norms per sensor: updated on each accepted update
  double last_gnss_innovation_norm_    = 0.0;
  double last_imu_innovation_norm_     = 0.0;
  double last_encoder_innovation_norm_ = 0.0;

  // Inertial coast mode tracking
  int  gnss_consecutive_rejects_ = 0;
  bool gnss_in_coast_            = false;
  // True while update_zupt owns the position noise scale, so only it undoes it.
  bool zupt_holds_pos_noise_     = false;
  // Largest Mahalanobis distance the GNSS gate has seen, and how many fixes it
  // has judged. Compare against outlier_threshold_gnss: a large ratio means the
  // gate cannot fire, which is invisible in any per-fix field.
  // Drop every parked-fix statistic. Called on init, on reset, and the moment
  // the wheels report motion, because a spread measured while stationary says
  // nothing about a receiver that is moving.
  void reset_parked_gnss_evidence();

  // GNSS fixes seen while the wheels confirm the robot is parked. All of them
  // sample the same physical point, so their spread and how strongly one fix
  // predicts the next are direct measurements of the receiver, not estimates.
  double parked_fix_n_ = 0.0;
  std::array<double, 3> parked_fix_s_{};       // sum
  std::array<double, 3> parked_fix_ss_{};      // sum of squares
  std::array<double, 3> parked_fix_slag_{};    // sum of consecutive products
  std::array<double, 3> parked_fix_prev_{};
  bool   parked_fix_has_prev_ = false;
  double gnss_parked_sigma_observed_ = -1.0;
  double gnss_parked_sigma_declared_ = -1.0;
  double gnss_parked_correlation_    = 0.0;
  double gnss_parked_inflation_      = 1.0;
  // Straightness check on the parked fixes, see zupt_parked_motion_m.
  double parked_ref_x_ = 0.0, parked_ref_y_ = 0.0;
  double parked_path_len_ = 0.0;
  bool   parked_moving_detected_ = false;
  double gnss_parked_straightness_ = 0.0;
  double imu_rate_prev_stamp_    = -1.0;
  double imu_rate_observed_sum_  = 0.0;
  int    imu_rate_observed_n_    = 0;
  double gnss_chi2_max_          = -1.0;
  int    gnss_chi2_samples_      = 0;
  // Persists the reason of the last rejected GNSS fix, for status reporting.
  GnssRejectionReason last_gnss_rejection_reason_ = GnssRejectionReason::NOT_PROCESSED;
  // Persists the reason of the last rejected magnetometer reading.
  MagRejectionReason last_mag_rejection_reason_ = MagRejectionReason::NOT_PROCESSED;
  std::array<OutcomeTally, GNSS_REJECTION_REASON_COUNT> gnss_tally_{};
  std::array<OutcomeTally, MAG_REJECTION_REASON_COUNT>  mag_tally_{};
  // Record the outcome sitting in gnss_debug_/mag_debug_ and stamp it. Called at
  // every terminal point so accepted and rejected fixes are both counted.
  void note_gnss_outcome(double timestamp_seconds);
  void note_mag_outcome(double timestamp_seconds);

  // Inter-sensor clock-skew protection. Raw per-stream stamps (recorded whether
  // or not the measurement was accepted, unlike last_*_time_ which only tracks
  // fused updates) let reject_stale_from_skew() tell a sensor that lags the
  // filter clock (its own stream still advances: skew, reject) from a genuine
  // time-base reset (its own stream jumped backward too: fall through and let
  // predict_to re-base). -1 = no history yet.
  double last_imu_raw_stamp_    = -1.0;
  double last_orient_raw_stamp_ = -1.0;
  double last_enc_raw_stamp_    = -1.0;
  double last_mag_raw_stamp_    = -1.0;
  double last_hdg_raw_stamp_    = -1.0;
  int imu_stale_rejects_     = 0;   // update_imu + update_imu_orientation combined
  int enc_stale_rejects_     = 0;
  int mag_stale_rejects_     = 0;
  int hdg_stale_rejects_     = 0;
  bool reject_stale_from_skew(double timestamp_seconds,
                              double& last_raw_stamp,
                              int& stale_counter);
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

  // ─── Yaw rate sign agreement ─────────────────────────────────────────────
  // A rover ran for months with its gyro yaw rate inverted: the BNO085 in
  // UART-RVC mode reports yaw increasing clockwise while REP-103 is
  // counterclockwise positive. Both sensors were healthy, the encoders were
  // right, and the filter watched them contradict each other every cycle
  // without comment. The disagreement was noticed twice and blamed on the
  // wheels both times. Because imu.gyro_noise defaults far tighter than
  // encoder.yaw_noise, the filter leans on the gyro for heading, which is
  // exactly the sensor that was wrong.
  //
  // Only samples taken while genuinely turning count, so noise around zero
  // cannot vote, and the verdict needs to persist rather than fire on one
  // sample.
  double yaw_sign_imu_wz_        = 0.0;    // most recent IMU yaw rate
  double yaw_sign_imu_stamp_     = -1.0;
  double yaw_sign_imu_sum_       = 0.0;   // summed over disagreeing samples only
  double yaw_sign_enc_sum_       = 0.0;
  int    yaw_sign_votes_         = 0;     // samples where BOTH were turning
  int    yaw_sign_disagree_      = 0;     // of those, how many disagreed in sign
  bool   yaw_sign_conflict_      = false;
  void note_yaw_rate_sign(double stamp, double enc_wz);

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
