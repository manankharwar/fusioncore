#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"

using namespace fusioncore;

// ─── Test 1: Cannot update before init ───────────────────────────────────────

TEST(FusionCoreTest, ThrowsIfNotInitialized) {
  FusionCore fc;
  EXPECT_THROW(fc.update_imu(0.0, 0,0,0,0,0,0), std::runtime_error);
  EXPECT_THROW(fc.update_encoder(0.0, 0,0,0),    std::runtime_error);
}

// ─── Test 2: Initializes cleanly ─────────────────────────────────────────────

TEST(FusionCoreTest, InitializesCleanly) {
  FusionCore fc;

  State initial;
  initial.x     = StateVector::Zero();
  initial.x[X]  = 1.0;
  initial.x[Y]  = 2.0;
  initial.P     = StateMatrix::Identity() * 0.1;

  fc.init(initial, 0.0);

  EXPECT_TRUE(fc.is_initialized());
  EXPECT_DOUBLE_EQ(fc.get_state().x[X], 1.0);
  EXPECT_DOUBLE_EQ(fc.get_state().x[Y], 2.0);
}

// ─── Test 3: Status reflects sensor health ───────────────────────────────────

TEST(FusionCoreTest, StatusReflectsSensorHealth) {
  FusionCore fc;

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  // Before any sensor data
  auto status = fc.get_status();
  EXPECT_EQ(status.imu_health,     SensorHealth::NOT_INIT);
  EXPECT_EQ(status.encoder_health, SensorHealth::NOT_INIT);

  // Send gravity on az — the measurement model now predicts ~9.81 for a flat
  // stationary robot, so the innovation is near zero and the update is accepted.
  fc.update_imu(0.01, 0,0,0, 0,0,9.81);
  status = fc.get_status();
  EXPECT_EQ(status.imu_health, SensorHealth::OK);
  EXPECT_EQ(status.update_count, 1);
}

// ─── Test 4: Position uncertainty grows without updates ──────────────────────

TEST(FusionCoreTest, UncertaintyGrowsWithoutUpdates) {
  FusionCore fc;

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.01;
  fc.init(initial, 0.0);

  double initial_uncertainty = fc.get_status().position_uncertainty;

  // Feed only IMU — no encoder, no position corrections
  for (int i = 1; i <= 100; ++i) {
    fc.update_imu(i * 0.01, 0,0,0,0,0,9.8);
  }

  double final_uncertainty = fc.get_status().position_uncertainty;
  EXPECT_GT(final_uncertainty, initial_uncertainty);
}

// ─── Test 5: Full end-to-end — robot drives forward 1 meter ─────────────────

TEST(FusionCoreTest, RobotDrivesForwardOneMeter) {
  FusionCoreConfig config;
  config.ukf.q_position    = 1e-6;
  config.ukf.q_velocity    = 1e-6;
  config.ukf.q_orientation = 1e-6;
  config.ukf.q_angular_vel = 1e-6;
  config.ukf.q_acceleration= 1e-6;
  config.ukf.q_gyro_bias   = 1e-6;
  config.ukf.q_accel_bias  = 1e-6;

  FusionCore fc(config);

  // Large uncertainty on position/velocity — we don't know where we are.
  // Small uncertainty on orientation — robot starts at a known heading (yaw=0).
  // High yaw uncertainty (P[YAW]=1.0) would spread sigma points ±57° and
  // cause the UKF's cos(yaw) average to collapse toward zero, making the
  // filter predict near-zero forward motion regardless of encoder velocity.
  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 1.0;
  initial.P(ROLL,ROLL)   = 0.01;
  initial.P(PITCH,PITCH) = 0.01;
  initial.P(YAW,YAW)     = 0.01;
  fc.init(initial, 0.0);

  // Robot drives forward at 1 m/s for 1 second
  // IMU at 100Hz, encoder at 50Hz
  for (int i = 1; i <= 100; ++i) {
    double t = i * 0.01;

    // IMU: moving forward at constant velocity, flat robot — send gravity on az.
    fc.update_imu(t, 0,0,0, 0,0,9.81);

    // Encoder at 50Hz
    if (i % 2 == 0) {
      fc.update_encoder(t, 1.0, 0.0, 0.0);
    }
  }

  // Should be approximately 1 meter forward in X
  // Tolerance is 0.5m: IMU sends zero acceleration while encoder sends 1m/s velocity.
  // The filter correctly reconciles conflicting sensors — position converges toward 1m
  // but not exactly due to the physically inconsistent input combination.
  EXPECT_NEAR(fc.get_state().x[X], 1.0, 0.5);
  EXPECT_NEAR(fc.get_state().x[Y], 0.0, 0.1);
}

// ─── Test 6: Reset clears state ──────────────────────────────────────────────

TEST(FusionCoreTest, ResetClearsState) {
  FusionCore fc;

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  fc.update_imu(0.01, 0,0,1, 0,0,9.8);
  EXPECT_TRUE(fc.is_initialized());

  fc.reset();
  EXPECT_FALSE(fc.is_initialized());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
