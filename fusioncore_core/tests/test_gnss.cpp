#include <gtest/gtest.h>
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
  x[YAW] = 1.57;

  GnssHdgMeasurement z = gnss_hdg_measurement_function(x);
  EXPECT_DOUBLE_EQ(z[0], 1.57);
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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
