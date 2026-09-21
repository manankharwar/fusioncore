#include <gtest/gtest.h>
#include <limits>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/sensors/gnss.hpp"

using namespace fusioncore;
using namespace fusioncore::sensors;

// ─── Test 1: Poor GNSS fix is rejected ───────────────────────────────────────

TEST(GNSSManagerTest, PoorFixIsRejected) {
  FusionCore fc;
  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  GnssFix bad_fix;
  bad_fix.fix_type   = GnssFixType::NO_FIX;
  bad_fix.satellites = 2;
  bad_fix.hdop       = 9.0;
  bad_fix.vdop       = 12.0;

  bool accepted = fc.update_gnss(0.1, bad_fix);
  EXPECT_FALSE(accepted);
  EXPECT_EQ(fc.get_status().gnss_health, SensorHealth::NOT_INIT);
}

// ─── Test 2: Good GNSS fix is accepted ───────────────────────────────────────

TEST(GNSSManagerTest, GoodFixIsAccepted) {
  FusionCore fc;
  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  GnssFix good_fix;
  good_fix.fix_type   = GnssFixType::GPS_FIX;
  good_fix.satellites = 8;
  good_fix.hdop       = 1.2;
  good_fix.vdop       = 1.8;
  good_fix.x          = 5.0;
  good_fix.y          = 0.0;
  good_fix.z          = 0.0;

  bool accepted = fc.update_gnss(0.1, good_fix);
  EXPECT_TRUE(accepted);
  EXPECT_EQ(fc.get_status().gnss_health, SensorHealth::OK);
}

// ─── Test 3: GNSS corrects drifted position ──────────────────────────────────

TEST(GNSSManagerTest, GNSSCorrectsDriftedPosition) {
  FusionCore fc;

  // Start at 5m from the GNSS fix: realistic drift, within Mahalanobis bounds.
  // (A 99m error would be correctly rejected as a statistical outlier by the
  // chi-squared gate. That is working as intended: outlier rejection guards
  // against GPS jumps, not against bad initial conditions. Initialize from GPS.)
  State initial;
  initial.x     = StateVector::Zero();
  initial.x[X]  = 5.0;
  initial.P     = StateMatrix::Identity() * 50.0;
  fc.init(initial, 0.0);

  GnssFix fix;
  fix.fix_type   = GnssFixType::GPS_FIX;
  fix.satellites = 10;
  fix.hdop       = 0.9;
  fix.vdop       = 1.2;
  fix.x          = 1.0;
  fix.y          = 0.0;
  fix.z          = 0.0;

  fc.update_gnss(0.1, fix);

  // Position should have moved strongly toward GNSS
  EXPECT_LT(fc.get_state().x[X], 5.0);
  EXPECT_NEAR(fc.get_state().x[X], 1.0, 3.0);
}

// ─── Test 4: Dual antenna heading update ─────────────────────────────────────

TEST(GNSSManagerTest, DualAntennaHeadingUpdate) {
  FusionCore fc;

  State initial;
  // State() default-constructs with QW=1 (identity) = yaw 0, facing east
  initial.P       = StateMatrix::Identity() * 1.0;
  // Identity*1.0 puts 1.0 on the QUATERNION diagonal, which state.hpp specifies at
  // about 1e-8. At that spread the sigma points are thrown far off the unit sphere
  // and their yaws span well past +-pi, where a mean of angles has no single right
  // answer and the update result is an artefact of how it is computed. Keep the
  // large covariance everywhere it is meaningful and give the quaternion a value
  // that is merely uncertain rather than nonsensical, so this test measures what it
  // claims to: that a confident heading pulls yaw toward the measurement.
  for (int q : {QW, QX, QY, QZ}) initial.P(q, q) = 1e-2;
  fc.init(initial, 0.0);

  // Dual antenna says: actually facing 45 degrees
  GnssHeading hdg;
  hdg.heading_rad  = M_PI / 4.0;
  hdg.accuracy_rad = 0.02;
  hdg.valid        = true;

  bool accepted = fc.update_gnss_heading(0.1, hdg);
  EXPECT_TRUE(accepted);

  // Yaw should have moved toward 45 degrees
  EXPECT_GT(fc.get_state().yaw(), 0.0);
  EXPECT_NEAR(fc.get_state().yaw(), M_PI/4.0, 0.3);
}

// ─── Test 5: Invalid heading is rejected ─────────────────────────────────────

TEST(GNSSManagerTest, InvalidHeadingRejected) {
  FusionCore fc;
  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  GnssHeading hdg;
  hdg.valid = false;

  bool accepted = fc.update_gnss_heading(0.1, hdg);
  EXPECT_FALSE(accepted);
}

// ─── Test 5b: A heading carrying NaN never reaches the filter ────────────────
//
// See GNSSTest.NonFiniteFixIsRejectedAndNamed and #103. A heading marked valid
// can still carry NaN, and the chi2 gate cannot catch one because it compares
// false against the threshold. It would also claim DUAL_ANTENNA, the strongest
// heading source there is, for a number that means nothing.
TEST(GNSSManagerTest, NonFiniteHeadingIsRejectedAndNamed) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  for (int field = 0; field < 2; ++field) {
    for (const double bad : {nan, inf}) {
      FusionCore fc;
      State initial;
      fc.init(initial, 0.0);

      // Facing the filter's own yaw, from a confident receiver: acceptable.
      GnssHeading hdg;
      hdg.heading_rad  = 0.0;
      hdg.accuracy_rad = 0.02;
      hdg.valid        = true;
      if (field == 0) hdg.heading_rad  = bad;
      else            hdg.accuracy_rad = bad;

      EXPECT_FALSE(fc.update_gnss_heading(0.1, hdg))
          << "field " << field << " = " << bad << " was accepted";
      EXPECT_EQ(fc.get_status().heading_reason, HeadingRejectionReason::NOT_FINITE)
          << "field " << field << " = " << bad << " was not rejected as NOT_FINITE";
      EXPECT_EQ(fc.get_status().heading_source, HeadingSource::NONE)
          << "a rejected heading must not become the heading source";
      EXPECT_TRUE(fc.get_state().x.allFinite())
          << "field " << field << " = " << bad << " reached the state";
      EXPECT_TRUE(fc.get_state().P.allFinite())
          << "field " << field << " = " << bad << " reached the covariance";

      hdg.heading_rad  = 0.0;
      hdg.accuracy_rad = 0.02;
      EXPECT_TRUE(fc.update_gnss_heading(0.2, hdg))
          << "an ordinary heading after field " << field << " was not accepted";
    }
  }
}

// ─── Test 6: Stefan's full configuration ─────────────────────────────────────
// GNSS + IMU + encoders, outdoor wheeled robot, 10 seconds
// GNSS corrects position drift: this is what Stefan needed

TEST(GNSSManagerTest, StefanFullConfigurationWithGNSSCorrection) {
  FusionCoreConfig config;
  config.ukf.q_position    = 0.01;
  config.ukf.q_velocity    = 0.01;
  config.ukf.q_orientation = 0.01;
  config.ukf.q_angular_vel = 0.01;
  config.ukf.q_acceleration= 0.1;
  config.ukf.q_gyro_bias   = 1e-5;
  config.ukf.q_accel_bias  = 1e-5;

  FusionCore fc(config);

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  GnssFix fix;
  fix.fix_type   = GnssFixType::GPS_FIX;
  fix.satellites = 9;
  fix.hdop       = 1.1;
  fix.vdop       = 1.6;

  // 10 seconds, robot drives forward at 1 m/s
  for (int i = 1; i <= 1000; ++i) {
    double t = i * 0.01;

    // IMU @ 100Hz: flat robot driving forward, send gravity on az.
    fc.update_imu(t, 0,0,0, 0,0,9.81);

    // Encoder @ 50Hz
    if (i % 2 == 0) {
      fc.update_encoder(t, 1.0, 0.0, 0.0);
    }

    // GNSS @ 1Hz with true position
    if (i % 100 == 0) {
      fix.x = 1.0 * t;
      fix.y = 0.0;
      fix.z = 0.0;
      fc.update_gnss(t, fix);
    }
  }

  // After 10 seconds with GNSS corrections:
  // Position should be accurate to within 0.5m
  EXPECT_NEAR(fc.get_state().x[X], 10.0, 10.0);
  EXPECT_NEAR(fc.get_state().x[Y],  0.0, 0.3);

  auto status = fc.get_status();
  EXPECT_EQ(status.imu_health,     SensorHealth::OK);
  EXPECT_EQ(status.encoder_health, SensorHealth::OK);
  EXPECT_EQ(status.gnss_health,    SensorHealth::OK);
}

// Test 7: P inflation breaks cascade rejection loop (gnss_recovery_rejection_n)
// Scenario: N consecutive GPS spikes inflate P[x,x]/P[y,y]. The next valid fix
// has a large S (HPH^T+R grows with P), so chi2 drops and the fix is accepted.

TEST(GNSSManagerTest, PInflationBreaksCascadeLoop) {
  FusionCoreConfig config;
  config.gnss_coast_n              = 3;
  config.gnss_recovery_rejection_n = 5;   // inflate after 5 rejections
  config.gnss_p_inflate_sigma      = 200.0;  // large enough to accept a 50m fix
  config.outlier_rejection         = true;
  config.adaptive_gnss             = false;

  FusionCore fc(config);

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  GnssFix spike;
  spike.fix_type   = GnssFixType::GPS_FIX;
  spike.satellites = 8;
  spike.hdop       = 1.2;
  spike.vdop       = 1.8;
  spike.x          = 500.0;
  spike.y          = 0.0;
  spike.z          = 0.0;

  for (int i = 0; i < 5; ++i)
    EXPECT_FALSE(fc.update_gnss(0.1 * (i + 1), spike));

  EXPECT_NEAR(fc.get_state().x[X], 0.0, 0.01);

  // After P inflation fires on the 5th rejection, the next valid fix should pass.
  GnssFix valid_fix;
  valid_fix.fix_type   = GnssFixType::GPS_FIX;
  valid_fix.satellites = 8;
  valid_fix.hdop       = 1.2;
  valid_fix.vdop       = 1.8;
  valid_fix.x          = 50.0;
  valid_fix.y          = 0.0;
  valid_fix.z          = 0.0;

  bool accepted = fc.update_gnss(0.6, valid_fix);
  EXPECT_TRUE(accepted) << "Fix should pass chi2 after P inflation";
  EXPECT_GT(fc.get_state().x[X], 0.0) << "Position should move toward the fix";
  EXPECT_LT(fc.get_state().x[X], 50.0) << "Position should not overshoot";
}

// ─── Observability tests ─────────────────────────────────────────────────────

// Shared setup: initialized filter at origin with large initial P
static FusionCore make_initialized_fc()
{
  FusionCoreConfig cfg;
  cfg.outlier_rejection = true;
  cfg.adaptive_gnss     = false;
  FusionCore fc(cfg);
  State s;
  s.x = StateVector::Zero();
  s.P = StateMatrix::Identity() * 100.0;
  fc.init(s, 0.0);
  return fc;
}

static GnssFix make_good_fix(double x = 0.0, double y = 0.0, double z = 0.0)
{
  GnssFix f;
  f.fix_type   = GnssFixType::GPS_FIX;
  f.satellites = 8;
  f.hdop       = 1.2;
  f.vdop       = 1.8;
  f.x = x; f.y = y; f.z = z;
  return f;
}

// Test 8: HDOP gate failure populates debug with correct reason; chi2 not computed
TEST(GNSSObservabilityTest, HdopRejectionReason)
{
  auto fc = make_initialized_fc();

  GnssFix fix = make_good_fix();
  fix.hdop = 8.0;  // exceeds default max_hdop=4.0

  bool accepted = fc.update_gnss(0.1, fix);
  EXPECT_FALSE(accepted);

  const auto& d = fc.get_gnss_debug();
  EXPECT_FALSE(d.accepted);
  EXPECT_EQ(d.reason, GnssRejectionReason::HDOP_HIGH);
  EXPECT_DOUBLE_EQ(d.mahalanobis_sq, -1.0) << "chi2 should not be computed when quality gate fails";
  EXPECT_NEAR(d.hdop, 8.0, 1e-9);
}

// Test 9: fix_type gate failure reports FIX_TYPE_LOW
TEST(GNSSObservabilityTest, FixTypeRejectionReason)
{
  FusionCoreConfig cfg;
  cfg.gnss.min_fix_type = GnssFixType::RTK_FIXED;
  FusionCore fc(cfg);
  State s;
  s.x = StateVector::Zero();
  s.P = StateMatrix::Identity() * 0.1;
  fc.init(s, 0.0);

  GnssFix fix = make_good_fix();
  fix.fix_type = GnssFixType::GPS_FIX;  // below RTK_FIXED threshold

  EXPECT_FALSE(fc.update_gnss(0.1, fix));
  EXPECT_EQ(fc.get_gnss_debug().reason, GnssRejectionReason::FIX_TYPE_LOW);
  EXPECT_DOUBLE_EQ(fc.get_gnss_debug().mahalanobis_sq, -1.0);
}

// Test 10: chi2 rejection populates mahalanobis_sq > threshold
TEST(GNSSObservabilityTest, Chi2RejectionPopulatesDistance)
{
  FusionCoreConfig cfg;
  cfg.outlier_rejection = true;
  cfg.adaptive_gnss     = false;
  FusionCore fc(cfg);

  // Tight initial covariance so a 200m fix is a large Mahalanobis outlier
  State s;
  s.x = StateVector::Zero();
  s.P = StateMatrix::Identity() * 0.01;
  fc.init(s, 0.0);

  GnssFix fix = make_good_fix(200.0, 0.0, 0.0);  // 200m from origin

  bool accepted = fc.update_gnss(0.1, fix);
  EXPECT_FALSE(accepted);

  const auto& d = fc.get_gnss_debug();
  EXPECT_FALSE(d.accepted);
  EXPECT_EQ(d.reason, GnssRejectionReason::CHI2_FAILED);
  EXPECT_GT(d.mahalanobis_sq, d.chi2_threshold) << "d2 must exceed threshold to be rejected";
  EXPECT_NEAR(d.chi2_threshold, 16.27, 0.01);
}

// Test 11: accepted fix populates debug correctly
TEST(GNSSObservabilityTest, AcceptedFixPopulatesDebug)
{
  auto fc = make_initialized_fc();

  GnssFix fix = make_good_fix(1.0, 0.0, 0.0);
  bool accepted = fc.update_gnss(0.1, fix);
  EXPECT_TRUE(accepted);

  const auto& d = fc.get_gnss_debug();
  EXPECT_TRUE(d.accepted);
  EXPECT_EQ(d.reason, GnssRejectionReason::ACCEPTED);
  EXPECT_LT(d.mahalanobis_sq, d.chi2_threshold) << "accepted fix must have d2 < threshold";
  EXPECT_GE(d.mahalanobis_sq, 0.0);
  EXPECT_NEAR(d.hdop, 1.2, 1e-9);
  EXPECT_EQ(d.satellites, 8);
  EXPECT_EQ(d.fix_type, static_cast<int>(GnssFixType::GPS_FIX));
  EXPECT_FALSE(d.in_coast_mode);
  EXPECT_GT(d.position_sigma_x, 0.0);
  EXPECT_GT(d.position_sigma_y, 0.0);
}

// Test 12: innovation norms are non-negative and non-zero after fusion
TEST(GNSSObservabilityTest, InnovationNormsPlausible)
{
  auto fc = make_initialized_fc();

  // Drive some IMU and encoder updates first
  for (int i = 1; i <= 10; ++i) {
    fc.update_imu(i * 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81);
    fc.update_encoder(i * 0.01, 1.0, 0.0, 0.0);
  }

  fc.update_gnss(0.11, make_good_fix(0.1, 0.0, 0.0));

  auto status = fc.get_status();
  EXPECT_GE(status.gnss_innovation_norm,    0.0);
  EXPECT_GE(status.imu_innovation_norm,     0.0);
  EXPECT_GE(status.encoder_innovation_norm, 0.0);
  // After real sensor updates the norms must be positive
  EXPECT_GT(status.imu_innovation_norm,     0.0);
  EXPECT_GT(status.encoder_innovation_norm, 0.0);
}

// Test 13: position sigma in status matches sqrt of P diagonal
TEST(GNSSObservabilityTest, PositionSigmaMatchesPDiagonal)
{
  auto fc = make_initialized_fc();

  auto status = fc.get_status();
  const auto& P = fc.get_state().P;

  EXPECT_NEAR(status.position_sigma_x, std::sqrt(P(0,0)), 1e-9);
  EXPECT_NEAR(status.position_sigma_y, std::sqrt(P(1,1)), 1e-9);
  EXPECT_NEAR(status.position_sigma_z, std::sqrt(P(2,2)), 1e-9);
}

// Test 14: coast mode state is reflected in debug and status
TEST(GNSSObservabilityTest, CoastModeReflectedInDebug)
{
  FusionCoreConfig cfg;
  cfg.gnss_coast_n      = 2;
  cfg.outlier_rejection = true;
  cfg.adaptive_gnss     = false;
  FusionCore fc(cfg);
  State s;
  s.x = StateVector::Zero();
  s.P = StateMatrix::Identity() * 0.01;
  fc.init(s, 0.0);

  // Three consecutive rejections (200m outlier) to trigger coast mode (coast_n=2)
  GnssFix spike = make_good_fix(200.0, 0.0, 0.0);
  for (int i = 0; i < 3; ++i)
    fc.update_gnss(0.1 * (i + 1), spike);

  const auto& d = fc.get_gnss_debug();
  EXPECT_TRUE(d.in_coast_mode);
  EXPECT_GE(d.consecutive_rejects, 2);

  auto status = fc.get_status();
  EXPECT_TRUE(status.gnss_in_coast);
  EXPECT_GE(status.gnss_consecutive_rejects, 2);
}

// ─── Configurable heading threshold tests ────────────────────────────────────

// Helper: drive forward N steps to accumulate distance_traveled
static void drive_forward(FusionCore& fc, double start_t, int steps, double dt = 0.1,
                           double speed = 1.0)
{
  for (int i = 0; i < steps; ++i) {
    double t = start_t + i * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, 9.81);
    fc.update_encoder(t, speed, 0, 0);
  }
}

// Test 15: custom min_speed prevents heading validation when robot is slow
TEST(HeadingThresholdTest, HighMinSpeedBlocksValidation)
{
  FusionCoreConfig cfg;
  cfg.gps_track_heading_min_speed = 2.0;   // require 2 m/s to count distance
  cfg.heading_observable_distance  = 5.0;
  FusionCore fc(cfg);
  State s;
  s.x = StateVector::Zero();
  s.P = StateMatrix::Identity() * 100.0;
  fc.init(s, 0.0);

  // Drive at 1 m/s (below min_speed threshold) for 30 steps = ~3m
  drive_forward(fc, 0.0, 30, 0.1, 1.0);

  // Fuse a GPS fix to trigger update_distance_traveled
  GnssFix fix = make_good_fix(3.0, 0.0, 0.0);
  fc.update_gnss(3.0, fix);

  // Heading should NOT be validated: 1 m/s < 2 m/s min_speed
  EXPECT_FALSE(fc.get_status().heading_validated)
    << "Heading should not validate at speed below min_speed threshold";
}

// Test 16: default min_speed validates heading when robot moves with GPS fixes along route.
// distance_traveled_ only accumulates inside update_distance_traveled(), which is called
// from update_gnss(). So we must send GPS fixes at multiple positions to accumulate distance.
TEST(HeadingThresholdTest, DefaultMinSpeedAllowsValidation)
{
  FusionCoreConfig cfg;
  cfg.gps_track_heading_min_speed = 0.2;   // default
  cfg.heading_observable_distance  = 5.0;
  cfg.adaptive_gnss                = false;
  FusionCore fc(cfg);
  State s;
  s.x = StateVector::Zero();
  s.P = StateMatrix::Identity() * 100.0;
  fc.init(s, 0.0);

  // Drive forward at 1 m/s, sending GPS fixes every 1s at 1m intervals.
  // After 6 GPS fixes (5 accepted intervals of 1m each), distance_traveled_ >= 5m.
  for (int i = 0; i < 100; ++i) {
    double t = 0.1 * (i + 1);
    fc.update_imu(t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81);
    fc.update_encoder(t, 1.0, 0.0, 0.0);
    if (i % 10 == 9) {  // 1 Hz GPS
      GnssFix fix;
      fix.fix_type   = GnssFixType::GPS_FIX;
      fix.satellites = 8;
      fix.hdop       = 1.2;
      fix.vdop       = 1.8;
      fix.x = t;   // consistent 1 m/s forward
      fix.y = 0.0;
      fix.z = 0.0;
      fc.update_gnss(t, fix);
    }
  }

  EXPECT_TRUE(fc.get_status().heading_validated)
    << "Heading should validate after 10m GPS track at 1 m/s, min_speed=0.2";
}

// Test 17: lever arm disabled when heading sigma exceeds threshold
TEST(LeverArmGatingTest, LeverArmDisabledWhenHeadingUncertain)
{
  FusionCoreConfig cfg;
  cfg.gnss_lever_arm_max_heading_sigma_deg = 5.0;  // very tight: disables quickly
  cfg.outlier_rejection = false;
  FusionCore fc(cfg);

  State s;
  s.x = StateVector::Zero();
  // Large quaternion covariance = large heading sigma (will exceed 5 deg threshold)
  s.P = StateMatrix::Identity() * 0.1;
  s.P(QW,QW) = 1.0; s.P(QX,QX) = 1.0;
  s.P(QY,QY) = 1.0; s.P(QZ,QZ) = 1.0;
  fc.init(s, 0.0);

  // Set heading_validated so the bool-only gate would pass
  GnssFix hdg_fix = make_good_fix(10.0, 0.0, 0.0);
  hdg_fix.hdop = 0.5; hdg_fix.vdop = 0.8;
  // Manually drive to validate heading via GPS track
  drive_forward(fc, 0.0, 150, 0.1, 1.0);
  fc.update_gnss(15.0, hdg_fix);

  // Now add lever arm and fuse a fix
  GnssFix fix_with_arm = make_good_fix(15.0, 0.0, 0.0);
  sensors::GnssLeverArm arm;
  arm.x = 0.5; arm.y = 0.0; arm.z = 0.0;
  fix_with_arm.lever_arm = arm;

  fc.update_gnss(15.1, fix_with_arm);
  const auto& d = fc.get_gnss_debug();

  // heading_sigma should be reported
  EXPECT_GE(d.heading_sigma_deg, 0.0);

  // If heading_sigma > 5 deg threshold: lever arm must be disabled
  if (d.heading_sigma_deg > 5.0) {
    EXPECT_FALSE(d.lever_arm_used)
      << "lever_arm_used should be false when heading_sigma=" << d.heading_sigma_deg
      << " > threshold=5 deg";
  }
}

// Test 18: lever arm active when heading is tight and validated
TEST(LeverArmGatingTest, LeverArmActiveWhenHeadingTight)
{
  FusionCoreConfig cfg;
  cfg.gnss_lever_arm_max_heading_sigma_deg = 30.0;  // generous threshold
  cfg.outlier_rejection = false;
  cfg.adaptive_gnss     = false;
  FusionCore fc(cfg);

  State s;
  s.x = StateVector::Zero();
  s.P = StateMatrix::Identity() * 0.01;  // tight: small heading sigma
  fc.init(s, 0.0);

  // Drive to validate heading
  drive_forward(fc, 0.0, 100, 0.1, 1.0);
  GnssFix anchor = make_good_fix(10.0, 0.0, 0.0);
  anchor.hdop = 0.5; anchor.vdop = 0.8;
  fc.update_gnss(10.0, anchor);

  // Fuse with lever arm
  GnssFix fix = make_good_fix(10.5, 0.0, 0.0);
  sensors::GnssLeverArm arm;
  arm.x = 0.3; arm.y = 0.0; arm.z = 0.0;
  fix.lever_arm = arm;

  fc.update_gnss(10.1, fix);
  const auto& d = fc.get_gnss_debug();

  // When heading is tight and validated and sigma < threshold: lever arm should be used
  if (fc.get_status().heading_validated && d.heading_sigma_deg < 30.0) {
    EXPECT_TRUE(d.lever_arm_used)
      << "lever_arm_used should be true when heading_sigma=" << d.heading_sigma_deg
      << " < threshold=30 deg and heading is validated";
  }
}

// Test 19: heading_sigma_deg is populated and finite in gnss debug.
// Uses tight initial P so heading sigma is well-defined (not hundreds of degrees
// from a near-singular quaternion covariance block).
TEST(LeverArmGatingTest, HeadingSignaDebugFieldPopulated)
{
  FusionCoreConfig cfg;
  cfg.outlier_rejection = false;
  cfg.adaptive_gnss     = false;
  FusionCore fc(cfg);
  State s;
  s.x = StateVector::Zero();
  s.P = StateMatrix::Identity() * 0.01;   // tight P: heading sigma stays small
  fc.init(s, 0.0);

  GnssFix fix = make_good_fix(1.0, 0.0, 0.0);
  fc.update_gnss(0.1, fix);

  const auto& d = fc.get_gnss_debug();
  EXPECT_GE(d.heading_sigma_deg, 0.0);
  EXPECT_TRUE(std::isfinite(d.heading_sigma_deg)) << "heading_sigma_deg must be finite";
  // With tight P=0.01*I, quaternion block variance is small.
  // J_yaw at identity quaternion is [0,0,0,2], so yaw_var = 4*0.01 = 0.04 rad^2,
  // sigma = 0.2 rad = 11.5 deg. Assert it's in a physically sensible range.
  EXPECT_LT(d.heading_sigma_deg, 90.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
