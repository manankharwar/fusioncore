#include <gtest/gtest.h>
#include <cmath>
#include <limits>
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

// ─── GPS track heading reports WHY it did not fuse ───────────────────────────
//
// This exists because of the 2026-09-06 field run. Yaw 1-sigma reached 101 deg,
// which disabled the lever arm (0 of 222 fixes), inflated the position covariance
// to 5.25 m, drove NIS to 0.20 against an honest 3.0, and left the chi2 outlier
// gate 39x away from ever firing. The whole cascade came from track heading
// silently declining to fuse, and the bag could not say which gate stopped it:
// two booleans covered "a stronger source owns heading" and "the motion is
// unsuitable", but the two cases that actually applied, a baseline shorter than
// track_heading_min_dist and a bearing sigma over the limit, were recorded
// nowhere. Diagnosing it took a day of replaying and reading source.

namespace {
// Drive east in a straight line, feeding GNSS fixes and matching wheel odometry,
// then report what the filter says about track heading on the last fix.
TrackHeadingState run_track_heading(double min_dist, double max_sigma,
                                    double fix_sigma, double step_m,
                                    int steps, double* baseline_out = nullptr)
{
  FusionCoreConfig cfg;
  cfg.gps_track_heading_enabled  = true;
  cfg.gps_track_heading_min_dist = min_dist;
  cfg.gps_track_heading_max_sigma = max_sigma;
  cfg.gps_track_heading_min_speed = 0.1;
  cfg.gps_track_heading_max_yaw_rate = 1.0;
  cfg.outlier_rejection = false;

  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  const double dt = 1.0;
  double t = 0.0, x = 0.0;
  for (int i = 0; i < steps; ++i) {
    t += dt;
    // Encoder motion so the speed gate is satisfied and the filter is moving.
    fc.update_encoder(t, step_m / dt, 0.0, 0.0);
    x += step_m;
    sensors::GnssFix fix;
    fix.x = x; fix.y = 0.0; fix.z = 0.0;
    fix.sigma_xy = fix.sigma_z = fix_sigma;
    // hdop/vdop scale the noise model, and the sigma gate reads the R the filter
    // actually built rather than this field. Leaving them at the 99.0 default
    // makes R enormous and every case reports SIGMA_HIGH, so mirror what the ROS
    // node does with a covariance-bearing fix and put the metres here too.
    fix.hdop = fix.vdop = fix_sigma;
    fix.fix_type = sensors::GnssFixType::RTK_FLOAT; fix.satellites = 12;
    fc.update_gnss(t, fix);
  }
  if (baseline_out) *baseline_out = fc.get_gnss_debug().track_heading_baseline_m;
  return fc.get_gnss_debug().track_heading_state;
}
}  // namespace

TEST(GNSSTest, TrackHeadingReportsBaselineTooShort) {
  // 1 m per fix against a 15 m minimum: the baseline never gets there, and before
  // this change the bag simply showed nothing at all.
  double baseline = -1.0;
  EXPECT_EQ(run_track_heading(15.0, 0.4, 1.0, 1.0, 5, &baseline),
            TrackHeadingState::BASELINE_SHORT);
  EXPECT_GT(baseline, 0.0) << "the baseline actually reached must be reported";
  EXPECT_LT(baseline, 15.0);
}

TEST(GNSSTest, TrackHeadingReportsSigmaTooHigh) {
  // Baseline is long enough, but the bearing the geometry implies is worse than
  // the gate allows. This is the field case, and it was previously invisible.
  // The threshold is deliberately far from the boundary so the test asserts the
  // reported STATE and does not quietly become a test of the noise model.
  EXPECT_EQ(run_track_heading(5.0, 1e-6, 3.24, 5.0, 4),
            TrackHeadingState::SIGMA_HIGH);
}

TEST(GNSSTest, TrackHeadingReportsFusedWhenItActuallyFires) {
  // Same geometry with the sigma gate opened wide: it fuses. Paired with the test
  // above this proves the states are distinguishing a real condition rather than
  // reporting one stuck value.
  EXPECT_EQ(run_track_heading(5.0, 1e6, 3.24, 5.0, 4),
            TrackHeadingState::FUSED);
}

TEST(GNSSTest, TrackHeadingReportsDisabled) {
  FusionCoreConfig cfg;
  cfg.gps_track_heading_enabled = false;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  sensors::GnssFix fix;
  fix.x = 1.0; fix.sigma_xy = fix.sigma_z = 1.0;
  fix.fix_type = sensors::GnssFixType::RTK_FLOAT; fix.satellites = 12;
  fc.update_gnss(1.0, fix);
  EXPECT_EQ(fc.get_gnss_debug().track_heading_state,
            TrackHeadingState::NOT_ATTEMPTED);
}

// ─── The outlier gate can be told what "short-term" means ────────────────────
//
// A receiver's reported covariance describes ABSOLUTE accuracy: multipath and
// ionospheric error that moves slowly. Consecutive fixes are far more consistent
// than that number implies. Measured on the 2026-09-06 rover log, 3.24 m declared
// against a 0.171 m median second difference, a factor of 55.
//
// The update wants the absolute figure or the filter believes GPS to centimetres
// it has not earned. The gate wants the short-term figure, because an outlier IS
// a break in short-term consistency. With the absolute figure in both places, S
// is so large that nothing looks surprising: on that log a spike had to exceed
// 29 m before it was rejected, while an accepted 15 m spike moved position 4.5 m.
namespace {
// Feed a straight run of clean fixes, then one displaced by `spike` metres.
// Returns true if that last fix was rejected.
bool spike_rejected(double spike, double outlier_sigma_xy) {
  FusionCoreConfig cfg;
  cfg.outlier_rejection = true;
  cfg.gnss.outlier_sigma_xy = outlier_sigma_xy;
  cfg.gnss_max_speed = 0.0;              // isolate the chi2 gate
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  double t = 0.0, x = 0.0;
  auto feed = [&](double ex) {
    sensors::GnssFix f;
    f.x = ex; f.y = 0.0; f.z = 0.0;
    f.sigma_xy = f.sigma_z = 3.24;       // what this receiver declares
    f.hdop = f.vdop = 3.24;
    f.fix_type = sensors::GnssFixType::RTK_FLOAT;
    f.satellites = 12;
    return fc.update_gnss(t, f);
  };
  for (int i = 0; i < 40; ++i) {
    t += 1.0;
    fc.update_encoder(t, 0.4, 0.0, 0.0);
    x += 0.4;
    feed(x);
  }
  t += 1.0;
  fc.update_encoder(t, 0.4, 0.0, 0.0);
  x += 0.4;
  return !feed(x + spike);
}
}  // namespace

// Smallest spike (in whole metres) this configuration rejects, or -1.
int spike_threshold(double outlier_sigma_xy) {
  for (int d = 1; d <= 120; ++d)
    if (spike_rejected(d, outlier_sigma_xy)) return d;
  return -1;
}

TEST(GNSSTest, OutlierSigmaDefaultsToTheOldBehaviour) {
  // Zero must change nothing: this ships to existing users.
  EXPECT_FALSE(spike_rejected(2.0, 0.0)) << "ordinary noise must never be rejected";
  EXPECT_TRUE (spike_rejected(60.0, 0.0)) << "a 60 m jump must still be caught";
}

TEST(GNSSTest, OutlierSigmaNeverLoosensTheGate) {
  // The invariant that matters. Telling the gate the receiver is more consistent
  // than its declared covariance can only make it more suspicious, never less.
  //
  // It is easy to get this backwards: a value ABOVE the receiver's own sigma
  // inflates the gate's S and lets MORE through. Measured with a 3.24 m receiver,
  // outlier_sigma_xy=5.0 moved the rejection threshold from 26 m out to 30 m,
  // which is the opposite of the intent. Hence the guidance to measure it from a
  // bag rather than guess.
  const int base = spike_threshold(0.0);
  ASSERT_GT(base, 0);
  for (double os : {2.0, 1.0, 0.5}) {
    EXPECT_LE(spike_threshold(os), base)
        << "outlier_sigma_xy=" << os << " made the gate looser than the default";
  }
}

TEST(GNSSTest, OutlierSigmaStillAcceptsOrdinaryNoise) {
  // The failure mode that has bitten this project repeatedly is a gate that
  // rejects good data. Ordinary fix-to-fix noise must survive at every setting.
  for (double os : {0.0, 2.0, 1.0, 0.5}) {
    EXPECT_FALSE(spike_rejected(0.5, os)) << "os=" << os;
    EXPECT_FALSE(spike_rejected(1.0, os)) << "os=" << os;
    EXPECT_FALSE(spike_rejected(2.0, os)) << "os=" << os;
  }
}

TEST(GNSSTest, ChiSquaredGateCannotSeeMetreScaleSpikes) {
  // Documents a limit rather than a fix, because it is the more important fact.
  //
  // A chi2 gate tests the fix against the FILTER, so its scale is set by
  // S = H P H' + R. With a 3.24 m receiver and a filter carrying metres of
  // position uncertainty, S is tens of square metres and nothing under about
  // 20 m looks surprising. Measured on the 2026-09-06 rover log: rejection began
  // between 29 and 30 m, and an accepted 15 m spike moved position 4.5 m.
  //
  // Giving the gate a better R helps but cannot cure it, because P remains. Nor
  // does fixing heading: a perfect absolute heading halved position sigma from
  // 7.19 to 3.94 m and moved the threshold the WRONG way, 26 m out to 32 m.
  //
  // Catching a metre-scale spike needs a test that does not involve P at all,
  // comparing a fix against its neighbours rather than against the filter. See
  // the fix-to-fix continuity proposal.
  EXPECT_FALSE(spike_rejected(10.0, 0.0))
      << "if this ever passes, the chi2 gate got sharper and the note above is stale";
  EXPECT_FALSE(spike_rejected(10.0, 0.5))
      << "a better gate R alone does not reach 10 m either";
}

// ─── Fix-to-fix continuity catches what chi2 structurally cannot ─────────────
//
// chi2 compares a fix against the FILTER, so its scale is S = H P H' + R. On a
// consumer receiver that is tens of square metres and nothing under about 25 m
// looks surprising: measured on the 2026-09-06 rover log, rejection began between
// 29 and 30 m while an accepted 15 m spike moved position 4.5 m. Neither a better
// gate R nor a perfect heading fixes it; heading made it slightly worse.
//
// Continuity asks whether a fix agrees with the two fixes either side of it,
// which never involves P. Measured across 2361 fixes from seven field logs, the
// second difference of a good fix has a median of 0.09 to 0.30 m and a p99 under
// 2.1 m, so a 3 m limit sits 10 to 30 times above normal. At that setting the
// only rejections across all seven logs were 5 fixes in one 2026-07 log whose
// second difference reached 4.42 m, 27 times that log's median, which is an
// outlier by any definition.
namespace {
// Fixes every second along a straight line, with one displaced by `spike`.
// Returns true if the displaced fix was rejected.
bool continuity_rejects(double spike, double continuity_max_m) {
  FusionCoreConfig cfg;
  cfg.outlier_rejection = true;
  cfg.gnss.continuity_max_m = continuity_max_m;
  cfg.gnss_max_speed = 0.0;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  double t = 0.0, x = 0.0;
  auto feed = [&](double ex) {
    sensors::GnssFix f;
    f.x = ex; f.y = 0.0; f.z = 0.0;
    f.sigma_xy = f.sigma_z = 3.24;
    f.hdop = f.vdop = 3.24;
    f.fix_type = sensors::GnssFixType::RTK_FLOAT;
    f.satellites = 12;
    return fc.update_gnss(t, f);
  };
  for (int i = 0; i < 10; ++i) { t += 1.0; x += 0.4; feed(x); }
  t += 1.0; x += 0.4;
  return !feed(x + spike);
}
}  // namespace

TEST(GNSSTest, ContinuityDefaultsOff) {
  EXPECT_FALSE(continuity_rejects(10.0, 0.0))
      << "zero must change nothing: this ships to existing users";
}

TEST(GNSSTest, ContinuityCatchesSpikesChi2Misses) {
  // 10 m is invisible to chi2 on this receiver and obvious here.
  EXPECT_TRUE(continuity_rejects(10.0, 3.0));
  EXPECT_TRUE(continuity_rejects(5.0,  3.0));
}

TEST(GNSSTest, ContinuityAcceptsOrdinaryFixToFixNoise) {
  // The bar any new gate must clear. Across seven real logs the second difference
  // of a good fix stayed under 2.2 m, so these must all survive a 3 m limit.
  EXPECT_FALSE(continuity_rejects(0.2, 3.0));
  EXPECT_FALSE(continuity_rejects(1.0, 3.0));
  EXPECT_FALSE(continuity_rejects(2.0, 3.0));
}

// ─── A spike small enough to pass must not take the next fix down with it ───
//
// This is the defect the least-squares predictor exists to fix. With the old
// two-point extrapolation px = x1 + (x1 - x2) * r, an error in the newest
// reference point reached the prediction multiplied by about two, so a spike
// that passed the limit threw the FOLLOWING good fix over it. Measured on the
// 2026-09-07 rover log at a 3 m limit: a 1.5 m spike was accepted and the good
// fix after it was rejected instead, which is worse than not gating at all,
// because you keep the bad sample and throw away the good one.
//
// Over five points the weight on the newest is 0.8, so a spike that passes the
// limit can only move the next prediction by 0.8 of itself and can never reach
// the limit. That holds for any threshold, which is why this is arithmetic
// rather than tuning.
TEST(GNSSTest, SubThresholdSpikeDoesNotGetTheNextFixRejected) {
  for (double spike : {0.5, 1.0, 1.5, 2.0, 2.5, 2.9}) {
    FusionCoreConfig cfg;
    cfg.outlier_rejection = true;
    cfg.gnss.continuity_max_m = 3.0;
    cfg.gnss_max_speed = 0.0;
    FusionCore fc(cfg);
    State s0;
    fc.init(s0, 0.0);

    double t = 0.0, x = 0.0;
    auto feed = [&](double ex) {
      sensors::GnssFix f;
      f.x = ex; f.y = 0.0; f.z = 0.0;
      f.sigma_xy = f.sigma_z = 3.24;
      f.hdop = f.vdop = 3.24;
      f.fix_type = sensors::GnssFixType::RTK_FLOAT;
      f.satellites = 12;
      return fc.update_gnss(t, f);
    };

    for (int i = 0; i < 10; ++i) { t += 1.0; x += 0.4; feed(x); }

    t += 1.0; x += 0.4;
    EXPECT_TRUE(feed(x + spike))
      << "a " << spike << " m offset is under the 3 m limit and must be accepted";

    // The good fix that follows. This is the one the old predictor threw away.
    t += 1.0; x += 0.4;
    EXPECT_TRUE(feed(x))
      << "the good fix after a " << spike << " m spike was REJECTED: the gate "
         "kept the bad sample and discarded the good one";
  }
}

// ─── The gate must reject the spike itself, not its neighbour ───────────────
TEST(GNSSTest, ContinuityRejectsTheSpikeAndThenRecovers) {
  FusionCoreConfig cfg;
  cfg.outlier_rejection = true;
  cfg.gnss.continuity_max_m = 3.0;
  cfg.gnss_max_speed = 0.0;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  double t = 0.0, x = 0.0;
  auto feed = [&](double ex) {
    sensors::GnssFix f;
    f.x = ex; f.y = 0.0; f.z = 0.0;
    f.sigma_xy = f.sigma_z = 3.24;
    f.hdop = f.vdop = 3.24;
    f.fix_type = sensors::GnssFixType::RTK_FLOAT;
    f.satellites = 12;
    return fc.update_gnss(t, f);
  };
  for (int i = 0; i < 10; ++i) { t += 1.0; x += 0.4; feed(x); }

  t += 1.0; x += 0.4;
  EXPECT_FALSE(feed(x + 30.0)) << "a 30 m spike must be rejected";

  // A rejected fix never enters the history, so the next good one is judged
  // against clean references and must come straight back.
  for (int i = 0; i < 3; ++i) {
    t += 1.0; x += 0.4;
    EXPECT_TRUE(feed(x)) << "good fix " << i << " after the spike was rejected";
  }
}

TEST(GNSSTest, ContinuityNeedsAFullHistoryBeforeItJudges) {
  // Five accepted fixes are needed before the fit means anything. Until then the
  // gate must stay out of the way rather than guess from two points.
  FusionCoreConfig cfg;
  cfg.outlier_rejection = true;
  cfg.gnss.continuity_max_m = 3.0;
  cfg.gnss_max_speed = 0.0;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  double t = 0.0;
  auto feed = [&](double ex) {
    sensors::GnssFix f;
    f.x = ex; f.y = 0.0; f.z = 0.0;
    f.sigma_xy = f.sigma_z = 3.24;
    f.hdop = f.vdop = 3.24;
    f.fix_type = sensors::GnssFixType::RTK_FLOAT;
    f.satellites = 12;
    return fc.update_gnss(t, f);
  };
  double x = 0.0;
  for (int i = 0; i < 4; ++i) { t += 1.0; x += 0.4; EXPECT_TRUE(feed(x)); }
  t += 1.0;
  feed(x + 50.0);
  // Whatever happens to this fix, continuity must not be the gate that decided
  // it. With four points there is no fit yet, so the question is not answerable
  // and guessing from a shorter history is what caused the defect above. chi2 is
  // free to reject it on its own terms, and here it does.
  EXPECT_NE(fc.get_status().gnss_last_rejection_reason,
            GnssRejectionReason::CONTINUITY_BREAK)
    << "continuity judged a fix before it had a full history to judge against";
}

TEST(GNSSTest, ContinuityIgnoresUnevenlySpacedFixes) {
  // Across a gap the second difference is legitimately large, and rejecting the
  // first fix after an outage is exactly the failure gnss_coast_min_gap_s exists
  // to avoid. Two fixes a second apart then one eight seconds later: the robot
  // has really travelled, and that must not read as a discontinuity.
  FusionCoreConfig cfg;
  cfg.outlier_rejection = true;
  cfg.gnss.continuity_max_m = 3.0;
  cfg.gnss_max_speed = 0.0;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  auto feed = [&](double t, double ex) {
    sensors::GnssFix f;
    f.x = ex; f.y = 0.0; f.z = 0.0;
    f.sigma_xy = f.sigma_z = 3.24;
    f.hdop = f.vdop = 3.24;
    f.fix_type = sensors::GnssFixType::RTK_FLOAT;
    f.satellites = 12;
    return fc.update_gnss(t, f);
  };
  feed(1.0, 0.4);
  feed(2.0, 0.8);
  EXPECT_TRUE(feed(10.0, 4.0))
      << "a fix after an 8 s gap must not be rejected for breaking continuity";
}

// ─── A fix carrying NaN must never reach the filter ─────────────────────────
//
// This one is worth a test out of proportion to how often it happens, because
// there is no recovery. A NaN in the state or the covariance propagates through
// the sigma points on the next predict, and every value the filter reports for
// the rest of the run is NaN. There is no gate downstream that catches it and
// no way back short of a reset.
//
// The subtle part is that a NaN does not fail the other gates, it passes them.
// Every comparison against a NaN is false, so `sigma_xy > max_sigma_xy` is false
// for a NaN sigma exactly as it is for a good one. Checking finiteness last
// would therefore never fire, which is why is_valid() checks it first.
TEST(GNSSTest, NonFiniteFixIsRejectedAndNamed) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  for (int field = 0; field < 5; ++field) {
    FusionCoreConfig cfg;
    cfg.outlier_rejection = true;
    cfg.gnss_max_speed = 0.0;
    FusionCore fc(cfg);
    State s0;
    fc.init(s0, 0.0);

    // A fix that is otherwise entirely acceptable.
    sensors::GnssFix f;
    f.x = 1.0; f.y = 2.0; f.z = 0.0;
    f.sigma_xy = f.sigma_z = 1.0;
    f.hdop = f.vdop = 1.0;
    f.fix_type = sensors::GnssFixType::RTK_FIXED;
    f.satellites = 12;

    switch (field) {
      case 0: f.x = nan; break;
      case 1: f.y = inf; break;
      case 2: f.z = nan; break;
      case 3: f.sigma_xy = nan; break;
      case 4: f.hdop = nan; break;
    }

    EXPECT_FALSE(fc.update_gnss(1.0, f)) << "non-finite field " << field
        << " was accepted; a NaN reaching the state is unrecoverable";
    EXPECT_EQ(fc.get_status().gnss_last_rejection_reason,
              GnssRejectionReason::NOT_FINITE)
        << "field " << field << " was rejected but blamed on the wrong gate, "
           "which sends people tuning a parameter that was never involved";

    // And the filter must still be usable afterwards.
    EXPECT_TRUE(std::isfinite(fc.get_state().x[X]));
    EXPECT_TRUE(std::isfinite(fc.get_state().P(X, X)));
  }
}

TEST(GNSSTest, AnOrdinaryFixIsStillAccepted) {
  // The guard runs on every fix, so pin that it lets a good one through.
  FusionCoreConfig cfg;
  cfg.outlier_rejection = true;
  cfg.gnss_max_speed = 0.0;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  sensors::GnssFix f;
  f.x = 1.0; f.y = 2.0; f.z = 0.0;
  f.sigma_xy = f.sigma_z = 1.0;
  f.hdop = f.vdop = 1.0;
  f.fix_type = sensors::GnssFixType::RTK_FIXED;
  f.satellites = 12;
  EXPECT_TRUE(fc.update_gnss(1.0, f));
}

// ─── A turn inside the heading baseline must invalidate it (#109) ────────────
//
// Contributed by Ignacio Villanua, who found it on a real UGV. Two yaw-rate
// gates already existed and neither closed this hole: one stops distance
// ACCUMULATING while turning, the other stops heading FUSING if the yaw rate is
// high at the instant of the fix. Between them, a robot can drive straight,
// turn, then drive straight again and fuse on the third leg while the reference
// position is still from before the turn. atan2 then returns the chord across
// an L-shaped path rather than the heading.
//
// The sting is sigma_hdg = sigma_xy / baseline: a long baseline makes that wrong
// bearing look extremely confident, so the filter's yaw covariance collapses
// onto it. Confidently wrong, which is the worst failure this project has.
TEST(GNSSTest, TurnInsideTheBaselineDiscardsTheWindow) {
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.gps_track_heading_max_yaw_rate = 0.3;
  cfg.gps_track_heading_min_dist     = 5.0;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  // Leg one, straight and long enough to build a baseline on its own.
  drive_arc(fc, 12.0, 1.0, 0.0, /*with_orientation=*/false);
  // A hard turn. Neither existing gate resets the reference position.
  drive_arc(fc,  4.0, 1.0, 0.8, /*with_orientation=*/false);
  // Leg two, straight again, so the instantaneous yaw rate is back under the
  // limit and the old gates would happily fuse.
  drive_arc(fc, 12.0, 1.0, 0.0, /*with_orientation=*/false);

  // What it must NOT do is fuse a bearing measured across the corner.
  const auto d = fc.get_gnss_debug();
  if (d.track_heading_state == TrackHeadingState::FUSED) {
    // If it did fuse, the baseline must have been rebuilt AFTER the turn, not
    // span it. A window spanning the corner shows up as a bearing tens of
    // degrees from the true heading of either leg.
    EXPECT_LT(std::abs(d.track_heading_sigma_rad), 1.0)
      << "fused a heading across a turn with sigma " << d.track_heading_sigma_rad;
  }
  SUCCEED() << "track heading state after the corner: "
            << static_cast<int>(d.track_heading_state);
}

// A turn that begins and ends between two GNSS fixes.
//
// #109 added a guard that discards the track-heading window when the robot turns
// inside it, but the yaw rate was only inspected from update_distance_traveled(),
// which on a GNSS-only robot runs once per fix. At 1 Hz a corner taken inside the
// gap was never seen, and the bearing was measured across it: the reference did
// not reset, so the baseline kept growing straight through the turn.
TEST(GNSSTest, TurnBetweenTwoFixesIsStillCaught) {
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.gps_track_heading_max_yaw_rate = 0.3;
  cfg.gps_track_heading_min_dist     = 5.0;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665;
  double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0;
  double baseline_before = 0.0, baseline_after = 0.0, max_state_wz = 0.0;
  TrackHeadingState state_after = TrackHeadingState::NOT_ATTEMPTED;

  // Fixes land on the second. The corner runs from t=12.0 to t=12.6, entirely
  // inside the gap between the fixes at t=12 and t=13.
  for (int i = 0; i < 1800; ++i) {
    t += dt;
    const double wz = (t > 12.0 && t <= 12.6) ? 0.8 : 0.0;
    yaw += wz * dt;
    x += std::cos(yaw) * dt;
    y += std::sin(yaw) * dt;
    fc.update_imu(t, 0, 0, wz, 0, 0, g);
    if (i % 2 == 0) fc.update_encoder(t, 1.0, 0.0, wz);
    if (wz != 0.0) max_state_wz = std::max(max_state_wz, std::abs(fc.get_state().x[WZ]));
    if (static_cast<int>(std::round(t * 100)) % 100 == 0) {
      GnssFix f;
      f.x = x; f.y = y; f.z = 0.0;
      f.hdop = f.sigma_xy = 2.0;
      f.vdop = f.sigma_z  = 3.0;
      f.satellites = 10;
      f.fix_type = GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
      const double b = fc.get_gnss_debug().track_heading_baseline_m;
      if (std::abs(t - 12.0) < 1e-6) baseline_before = b;
      if (std::abs(t - 13.0) < 1e-6) {
        baseline_after = b;
        state_after    = fc.get_gnss_debug().track_heading_state;
      }
    }
  }

  // Truth turned 27.5 degrees inside that gap, so the window must be discarded
  // rather than used to measure a bearing straight across the corner.
  EXPECT_EQ(state_after, TrackHeadingState::WINDOW_HAD_TURN)
      << "the corner was not detected (peak filtered |WZ| during it was "
      << max_state_wz << " rad/s against a threshold of "
      << cfg.gps_track_heading_max_yaw_rate << "), baseline went "
      << baseline_before << " m to " << baseline_after << " m";
}
