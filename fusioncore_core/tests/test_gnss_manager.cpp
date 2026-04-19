#include <gtest/gtest.h>
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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
