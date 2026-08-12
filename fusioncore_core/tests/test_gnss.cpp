#include <gtest/gtest.h>
#include <cmath>
#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"
#include "fusioncore/sensors/gnss.hpp"
#include "fusioncore/fusioncore.hpp"

using namespace fusioncore;
using namespace fusioncore::sensors;

// ─── Test 1: Position measurement function maps state ────────────────────────

TEST(GNSSTest, PosMeasurementFunctionMapsState) {
  StateVector x = StateVector::Zero();
  x[X] = 10.0;
  x[Y] = 20.0;
  x[Z] =  5.0;

  GnssPosMeasurement z = gnss_pos_measurement_function(x);

  EXPECT_DOUBLE_EQ(z[0], 10.0);
  EXPECT_DOUBLE_EQ(z[1], 20.0);
  EXPECT_DOUBLE_EQ(z[2],  5.0);
}

// ─── Test 2: Heading measurement function maps yaw ───────────────────────────

TEST(GNSSTest, HdgMeasurementFunctionMapsYaw) {
  StateVector x = StateVector::Zero();
  // yaw = π/2 → quaternion [cos(π/4), 0, 0, sin(π/4)]
  x[QW] = std::cos(M_PI / 4.0);
  x[QX] = 0.0;
  x[QY] = 0.0;
  x[QZ] = std::sin(M_PI / 4.0);

  GnssHdgMeasurement z = gnss_hdg_measurement_function(x);
  EXPECT_NEAR(z[0], M_PI / 2.0, 1e-9);
}

// ─── Test 3: Quality-aware noise scales with HDOP ────────────────────────────

TEST(GNSSTest, NoiseScalesWithHDOP) {
  GnssParams params;
  params.base_noise_xy = 1.0;

  GnssFix good_fix;
  good_fix.fix_type   = GnssFixType::GPS_FIX;
  good_fix.satellites = 8;
  good_fix.hdop       = 1.0;
  good_fix.vdop       = 1.5;

  GnssFix poor_fix;
  poor_fix.fix_type   = GnssFixType::GPS_FIX;
  poor_fix.satellites = 5;
  poor_fix.hdop       = 3.5;
  poor_fix.vdop       = 5.0;

  GnssPosNoiseMatrix R_good = gnss_pos_noise_matrix(params, good_fix);
  GnssPosNoiseMatrix R_poor = gnss_pos_noise_matrix(params, poor_fix);

  // Poor fix should have larger noise
  EXPECT_GT(R_poor(0,0), R_good(0,0));
  EXPECT_GT(R_poor(2,2), R_good(2,2));
}

// ─── Test 4: Fix validity check works ────────────────────────────────────────

TEST(GNSSTest, FixValidityCheck) {
  GnssParams params;

  GnssFix valid_fix;
  valid_fix.fix_type   = GnssFixType::GPS_FIX;
  valid_fix.satellites = 6;
  valid_fix.hdop       = 1.5;
  valid_fix.vdop       = 2.0;
  EXPECT_TRUE(valid_fix.is_valid(params));

  GnssFix no_fix;
  no_fix.fix_type = GnssFixType::NO_FIX;
  EXPECT_FALSE(no_fix.is_valid(params));

  GnssFix poor_hdop;
  poor_hdop.fix_type   = GnssFixType::GPS_FIX;
  poor_hdop.satellites = 6;
  poor_hdop.hdop       = 5.0;  // exceeds max_hdop=4.0
  poor_hdop.vdop       = 2.0;
  EXPECT_FALSE(poor_hdop.is_valid(params));

  GnssFix few_sats;
  few_sats.fix_type   = GnssFixType::GPS_FIX;
  few_sats.satellites = 3;     // below min_satellites=4
  few_sats.hdop       = 1.5;
  few_sats.vdop       = 2.0;
  EXPECT_FALSE(few_sats.is_valid(params));
}

// ─── Test 4b: min_fix_type gating ───────────────────────────────────────────

TEST(GNSSTest, MinFixTypeGating) {
  GnssParams params;
  params.min_fix_type = GnssFixType::RTK_FLOAT;

  GnssFix gps_fix;
  gps_fix.fix_type   = GnssFixType::GPS_FIX;
  gps_fix.satellites = 8;
  gps_fix.hdop       = 1.0;
  gps_fix.vdop       = 1.5;
  EXPECT_FALSE(gps_fix.is_valid(params));  // GPS_FIX < RTK_FLOAT

  GnssFix dgps_fix;
  dgps_fix.fix_type   = GnssFixType::DGPS_FIX;
  dgps_fix.satellites = 8;
  dgps_fix.hdop       = 1.0;
  dgps_fix.vdop       = 1.5;
  EXPECT_FALSE(dgps_fix.is_valid(params));  // DGPS < RTK_FLOAT

  GnssFix rtk_float;
  rtk_float.fix_type   = GnssFixType::RTK_FLOAT;
  rtk_float.satellites = 8;
  rtk_float.hdop       = 1.0;
  rtk_float.vdop       = 1.5;
  EXPECT_TRUE(rtk_float.is_valid(params));  // RTK_FLOAT == min

  GnssFix rtk_fixed;
  rtk_fixed.fix_type   = GnssFixType::RTK_FIXED;
  rtk_fixed.satellites = 8;
  rtk_fixed.hdop       = 1.0;
  rtk_fixed.vdop       = 1.5;
  EXPECT_TRUE(rtk_fixed.is_valid(params));  // RTK_FIXED > min
}

// ─── Test 5: ECEF to ENU conversion ─────────────────────────────────────────

TEST(GNSSTest, ECEFtoENUAtOriginIsZero) {
  // Reference point: somewhere in Hamilton Ontario
  LLAPoint ref_lla;
  ref_lla.lat_rad = 43.25 * M_PI / 180.0;
  ref_lla.lon_rad = -79.87 * M_PI / 180.0;
  ref_lla.alt_m   = 100.0;

  ECEFPoint ref;
  ref.x = 918151.0;
  ref.y = -4346071.0;
  ref.z = 4561977.0;

  // Same point: should give ENU = (0,0,0)
  Eigen::Vector3d enu = ecef_to_enu(ref, ref, ref_lla);

  EXPECT_NEAR(enu[0], 0.0, 1e-6);
  EXPECT_NEAR(enu[1], 0.0, 1e-6);
  EXPECT_NEAR(enu[2], 0.0, 1e-6);
}

// ─── Test 6: GNSS position update corrects drifted position ──────────────────

TEST(GNSSTest, GNSSUpdateCorrectedDriftedPosition) {
  UKF ukf;

  State initial;
  initial.x     = StateVector::Zero();
  initial.x[X]  = 50.0;   // drifted far from truth
  initial.x[Y]  = 30.0;
  initial.P     = StateMatrix::Identity() * 10.0;

  ukf.init(initial);

  // GNSS says: actually at (1, 1, 0)
  GnssPosMeasurement z;
  z[0] = 1.0; z[1] = 1.0; z[2] = 0.0;

  GnssParams params;
  GnssFix fix;
  fix.fix_type   = GnssFixType::GPS_FIX;
  fix.satellites = 8;
  fix.hdop       = 1.0;
  fix.vdop       = 1.5;

  GnssPosNoiseMatrix R = gnss_pos_noise_matrix(params, fix);

  ukf.update<GNSS_POS_DIM>(z, gnss_pos_measurement_function, R);

  // Should have pulled strongly toward GNSS measurement
  EXPECT_LT(ukf.state().x[X], 50.0);
  EXPECT_LT(ukf.state().x[Y], 30.0);
  EXPECT_NEAR(ukf.state().x[X], 1.0, 5.0);
}

// ─── Test 7: GNSS + IMU + encoder: full Stefan configuration ────────────────
// Outdoor wheeled robot, GNSS + IMU + encoders
// This is the exact use case Stefan posted on ROS Discourse Dec 2024

TEST(GNSSTest, StefanConfigurationFullFusion) {
  FusionCoreConfig config;
  config.ukf.q_position    = 1e-4;
  config.ukf.q_velocity    = 1e-4;
  config.ukf.q_orientation = 1e-4;
  config.ukf.q_angular_vel = 1e-4;
  config.ukf.q_acceleration= 1e-4;
  config.ukf.q_gyro_bias   = 1e-6;
  config.ukf.q_accel_bias  = 1e-6;

  FusionCore fc(config);

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  GnssParams gnss_params;
  GnssFix fix;
  fix.fix_type   = GnssFixType::GPS_FIX;
  fix.satellites = 8;
  fix.hdop       = 1.2;
  fix.vdop       = 1.8;

  // Simulate 5 seconds: robot drives forward 5 meters
  // IMU @ 100Hz, encoder @ 50Hz, GNSS @ 1Hz
  for (int i = 1; i <= 500; ++i) {
    double t = i * 0.01;

    // IMU
    fc.update_imu(t, 0,0,0, 0,0,0);

    // Encoder @ 50Hz
    if (i % 2 == 0) {
      fc.update_encoder(t, 1.0, 0.0, 0.0);
    }

    // GNSS @ 1Hz: truth position
    if (i % 100 == 0) {
      double true_x = 1.0 * t;  // 1 m/s forward
      GnssPosMeasurement z_gnss;
      z_gnss[0] = true_x;
      z_gnss[1] = 0.0;
      z_gnss[2] = 0.0;

      GnssPosNoiseMatrix R = gnss_pos_noise_matrix(gnss_params, fix);
      fc.get_state();  // read current state

      // Direct UKF update via FusionCore: we'll add update_gnss in next step
      // For now verify the manager is stable through the full run
    }
  }

  // After 5 seconds at 1 m/s, should be near x=5
  // GNSS not yet wired to manager: position will drift
  // This test proves stability of the full pipeline
  auto status = fc.get_status();
  EXPECT_TRUE(status.initialized);
  EXPECT_EQ(status.imu_health,     SensorHealth::OK);
  EXPECT_EQ(status.encoder_health, SensorHealth::OK);
  EXPECT_GT(status.update_count,   0);
}

// ─── Rejection reason surfaces in status (field + offline observability) ─────
// Quality-gate rejects (HDOP/VDOP/fix-type/sats) and delay rejects do NOT
// increment gnss_outliers, so before this the only signal was a throttled log
// line. The reason of the last rejected fix must reach get_status() so it lands
// in filter_health for live field debugging and offline bag analysis. Grounded
// in a real hardware session where an M9N was silently rejected on VDOP.
TEST(GNSSTest, RejectionReasonSurfacesInStatus) {
  FusionCoreConfig config;                 // defaults: max_hdop 4, max_vdop 6, min_sats 4
  FusionCore fc(config);

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  // Fresh filter: nothing rejected yet.
  EXPECT_EQ(fc.get_status().gnss_last_rejection_reason,
            GnssRejectionReason::NOT_PROCESSED);

  // Horizontally fine, vertically poor: must report VDOP_HIGH. This is the exact
  // case the rover's M9N hit, which used to fail silently.
  GnssFix vbad;
  vbad.fix_type   = GnssFixType::GPS_FIX;
  vbad.satellites = 10;
  vbad.hdop       = 1.0;
  vbad.vdop       = 20.0;                  // > max_vdop 6.0
  EXPECT_FALSE(fc.update_gnss(1.0, vbad));
  EXPECT_EQ(fc.get_status().gnss_last_rejection_reason,
            GnssRejectionReason::VDOP_HIGH);

  // Too few satellites: must report MIN_SATS.
  GnssFix sbad;
  sbad.fix_type   = GnssFixType::GPS_FIX;
  sbad.satellites = 2;                     // < min_satellites 4
  sbad.hdop       = 1.0;
  sbad.vdop       = 1.0;
  EXPECT_FALSE(fc.update_gnss(2.0, sbad));
  EXPECT_EQ(fc.get_status().gnss_last_rejection_reason,
            GnssRejectionReason::MIN_SATS);
}


// ─── The issue #73 gate: metres must not be judged against DOP thresholds ────
//
// A NavSatFix carries no DOP. The node derives sqrt(variance) from the message
// covariance, which is METRES, and that used to be compared against max_hdop
// (4.0) and max_vdop (6.0). Those read like dimensionless DOP limits, so they
// looked generous while actually meaning "reject anything worse than 4 m
// horizontal". A working RTK setup had every fix vetoed as VDOP_HIGH and the
// filter silently dead-reckoned. Reported by JR-pT on issue #73.

TEST(GNSSTest, SigmaGateAcceptsOrdinaryStandaloneGps) {
  GnssParams params;   // defaults: max_sigma_xy 25 m, max_sigma_z 50 m

  // Exactly what a healthy standalone receiver reports, and precisely the
  // numbers from issue #73: 2 m horizontal, 8 m vertical. Both of these
  // exceed max_hdop 4.0 / max_vdop 6.0 if misread as DOP.
  GnssFix fix;
  fix.fix_type   = GnssFixType::GPS_FIX;
  fix.satellites = 8;
  fix.hdop       = 2.0;   // metres, kept for the noise scale factor
  fix.vdop       = 8.0;
  fix.sigma_xy   = 2.0;   // metres, what the gate must read
  fix.sigma_z    = 8.0;

  EXPECT_TRUE(fix.has_sigma());
  EXPECT_TRUE(fix.is_valid(params))
      << "a 2 m / 8 m fix is ordinary standalone GPS and must be fused";
}

TEST(GNSSTest, SigmaGateStillRejectsGarbage) {
  GnssParams params;

  GnssFix wide;
  wide.fix_type   = GnssFixType::GPS_FIX;
  wide.satellites = 8;
  wide.hdop = wide.sigma_xy = 40.0;   // 40 m horizontal: not usable
  wide.vdop = wide.sigma_z  = 10.0;
  EXPECT_FALSE(wide.is_valid(params));

  GnssFix tall;
  tall.fix_type   = GnssFixType::GPS_FIX;
  tall.satellites = 8;
  tall.hdop = tall.sigma_xy = 3.0;
  tall.vdop = tall.sigma_z  = 80.0;   // 80 m vertical
  EXPECT_FALSE(tall.is_valid(params));
}

TEST(GNSSTest, SigmaPathIgnoresDopThresholds) {
  GnssParams params;
  params.max_hdop = 0.1;   // absurdly strict, must not apply
  params.max_vdop = 0.1;

  GnssFix fix;
  fix.fix_type   = GnssFixType::GPS_FIX;
  fix.satellites = 8;
  fix.hdop = fix.sigma_xy = 3.0;
  fix.vdop = fix.sigma_z  = 5.0;

  EXPECT_TRUE(fix.is_valid(params))
      << "when a covariance is present the DOP limits are the wrong units "
         "and must be bypassed entirely";
}

TEST(GNSSTest, DopGateStillAppliesWithoutCovariance) {
  // Regression guard: a receiver reporting genuine DOP and no covariance must
  // keep the original behaviour, so max_hdop/max_vdop stay meaningful.
  GnssParams params;

  GnssFix fix;
  fix.fix_type   = GnssFixType::GPS_FIX;
  fix.satellites = 8;
  fix.hdop       = 5.0;    // > max_hdop 4.0
  fix.vdop       = 2.0;
  EXPECT_FALSE(fix.has_sigma());
  EXPECT_FALSE(fix.is_valid(params));
}

TEST(GNSSTest, SigmaRejectionNamesTheGateThatFired) {
  FusionCoreConfig config;
  FusionCore fc(config);

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  // Wide horizontally: must say SIGMA_XY_HIGH, not HDOP_HIGH. Naming the wrong
  // gate is what sent issue #73 tuning a parameter that was never involved.
  GnssFix wide;
  wide.fix_type   = GnssFixType::GPS_FIX;
  wide.satellites = 8;
  wide.hdop = wide.sigma_xy = 40.0;
  wide.vdop = wide.sigma_z  = 5.0;
  EXPECT_FALSE(fc.update_gnss(1.0, wide));
  EXPECT_EQ(fc.get_status().gnss_last_rejection_reason,
            GnssRejectionReason::SIGMA_XY_HIGH);

  GnssFix tall;
  tall.fix_type   = GnssFixType::GPS_FIX;
  tall.satellites = 8;
  tall.hdop = tall.sigma_xy = 2.0;
  tall.vdop = tall.sigma_z  = 90.0;
  EXPECT_FALSE(fc.update_gnss(2.0, tall));
  EXPECT_EQ(fc.get_status().gnss_last_rejection_reason,
            GnssRejectionReason::SIGMA_Z_HIGH);

  // And a good one gets through, so the gate is not simply rejecting everything.
  GnssFix ok;
  ok.fix_type   = GnssFixType::GPS_FIX;
  ok.satellites = 8;
  ok.hdop = ok.sigma_xy = 2.0;
  ok.vdop = ok.sigma_z  = 8.0;
  EXPECT_TRUE(fc.update_gnss(3.0, ok));
}


// ─── GPS track heading must not fight a better heading source (issue #73) ────
//
// GPS track heading is course over ground. On a curved path that differs from
// body heading by a real bias, not just noise, so fusing it corrupts the
// estimate no matter how honest its R is. A user on a Nav2 path reported a
// straight route without GPS turning into a zig-zag with it, on a robot whose
// magnetometer was perfectly stable.

namespace {
// Drive east in a straight line at `speed`, feeding GPS every `gps_every_s`.
// Returns the filter so the caller can inspect what the heading fusion did.
// Drive at `speed` with a constant `yaw_rate`, feeding GPS at 1 Hz. The GPS
// track follows the SAME arc the robot is driving, so a turning case stays
// physically consistent and the fix is not rejected as an outlier.
// with_orientation: also call update_imu_orientation, which is what a 9-axis
// driver does and what makes heading_source_ become IMU_ORIENTATION.
void drive_arc(FusionCore& fc, double to_s, double speed,
               double yaw_rate, bool with_orientation) {
  const double dt = 0.01, g = 9.80665;
  for (int step = 1; step * dt <= to_s + 1e-9; ++step) {
    double t   = step * dt;
    double yaw = yaw_rate * t;
    fc.update_imu(t, 0, 0, yaw_rate, 0, 0, g);
    if (with_orientation) fc.update_imu_orientation(t, 0.0, 0.0, yaw);
    if (step % 2 == 0) fc.update_encoder(t, speed, 0.0, yaw_rate);
    if (step % 100 == 0) {
      // Exact arc: straight line degenerates correctly as yaw_rate -> 0.
      double x, y;
      if (std::abs(yaw_rate) < 1e-9) { x = speed * t;               y = 0.0; }
      else                           { x = speed / yaw_rate * std::sin(yaw);
                                       y = speed / yaw_rate * (1.0 - std::cos(yaw)); }
      GnssFix f;
      f.x = x; f.y = y; f.z = 0.0;
      f.hdop = f.sigma_xy = 2.0;
      f.vdop = f.sigma_z  = 3.0;
      f.satellites = 10;
      f.fix_type = GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
    }
  }
}
} // namespace

TEST(GNSSTest, TrackHeadingSkippedWhenImuOrientationIsAbsolute) {
  // A 9-axis IMU makes heading_source_ IMU_ORIENTATION, which outranks
  // GPS_TRACK. Track heading must then stand down entirely.
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = true;          // 9-axis: orientation IS a heading
  cfg.motion_model = create_motion_model("DifferentialDrive");
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  drive_arc(fc, 20.0, 1.0, 0.0, /*with_orientation=*/true);

  EXPECT_EQ(fc.get_status().heading_source, HeadingSource::IMU_ORIENTATION);
  EXPECT_TRUE(fc.get_gnss_debug().track_heading_skipped_stronger_source)
      << "GPS track heading fused on top of an absolute IMU heading";
}

TEST(GNSSTest, TrackHeadingStillRunsWithoutAnAbsoluteSource) {
  // Regression guard for the opposite direction: a 6-axis IMU has no absolute
  // heading, so GPS track heading is the only thing that can supply one and
  // must keep working exactly as before.
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;         // 6-axis: gyro yaw drifts
  cfg.motion_model = create_motion_model("DifferentialDrive");
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  drive_arc(fc, 20.0, 1.0, 0.0, /*with_orientation=*/false);

  EXPECT_FALSE(fc.get_gnss_debug().track_heading_skipped_stronger_source);
  EXPECT_TRUE(fc.get_status().heading_validated)
      << "GPS track heading failed to validate heading on a 6-axis robot";
  EXPECT_EQ(fc.get_status().heading_source, HeadingSource::GPS_TRACK);
}

TEST(GNSSTest, TrackHeadingSkippedWhileTurningHard) {
  // gps_track_heading_max_yaw_rate has always been documented as guarding this
  // fusion, but it only gated the heading_validated_ flag: the fusion itself ran
  // unguarded, so a turning robot fused its course over ground as body heading.
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;         // no stronger source, so only the
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.gps_track_heading_max_yaw_rate = 0.3; // motion gate can be responsible
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  // Turning at 0.8 rad/s, well above the 0.3 limit, on a consistent arc.
  drive_arc(fc, 20.0, 1.0, 0.8, /*with_orientation=*/false);

  EXPECT_TRUE(fc.get_gnss_debug().track_heading_skipped_motion)
      << "track heading fused while the robot was turning at 0.8 rad/s";
}

TEST(GNSSTest, TrackHeadingSkippedWhenTooSlowToHaveACourse) {
  // Below min_speed the displacement is GPS noise, not travel, so its bearing
  // is meaningless.
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.gps_track_heading_min_speed = 0.2;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  drive_arc(fc, 20.0, 0.05, 0.0, /*with_orientation=*/false);  // 5 cm/s vs 0.2 limit

  EXPECT_TRUE(fc.get_gnss_debug().track_heading_skipped_motion)
      << "track heading fused a bearing derived from noise at 0.05 m/s";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
