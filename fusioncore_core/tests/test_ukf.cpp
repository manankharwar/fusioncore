#include <gtest/gtest.h>
#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"

using namespace fusioncore;

// ─── Test 1: Initialization ─────────────────────────────────────────────────

TEST(UKFTest, InitializesCorrectly) {
  UKF ukf;

  State initial;
  initial.x[X]   = 1.0;
  initial.x[Y]   = 2.0;
  initial.x[Z]   = 0.0;
  initial.x[YAW] = 0.5;

  ukf.init(initial);

  EXPECT_TRUE(ukf.is_initialized());
  EXPECT_DOUBLE_EQ(ukf.state().x[X],   1.0);
  EXPECT_DOUBLE_EQ(ukf.state().x[Y],   2.0);
  EXPECT_DOUBLE_EQ(ukf.state().x[YAW], 0.5);
}

// ─── Test 2: Predict doesn't explode ────────────────────────────────────────

TEST(UKFTest, PredictRunsWithoutError) {
  UKF ukf;

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;

  ukf.init(initial);

  EXPECT_NO_THROW(ukf.predict(0.01));  // 10ms timestep
  EXPECT_NO_THROW(ukf.predict(0.01));
  EXPECT_NO_THROW(ukf.predict(0.01));
}

// ─── Test 3: Stationary robot stays stationary ──────────────────────────────

TEST(UKFTest, StationaryRobotRemainsStationary) {
  UKF ukf;

  State initial;
  initial.x = StateVector::Zero();  // zero velocity, zero position
  initial.P = StateMatrix::Identity() * 0.01;

  ukf.init(initial);

  // Predict forward 1 second (100 steps of 10ms)
  for (int i = 0; i < 100; ++i) {
    ukf.predict(0.01);
  }

  // Position should stay near zero (small drift from process noise only)
  EXPECT_NEAR(ukf.state().x[X], 0.0, 0.01);
  EXPECT_NEAR(ukf.state().x[Y], 0.0, 0.01);
  EXPECT_NEAR(ukf.state().x[Z], 0.0, 0.01);
}

// ─── Test 4: Robot moving forward integrates position ───────────────────────

TEST(UKFTest, ForwardMotionIntegratesPosition) {
  UKF ukf;

  State initial;
  initial.x          = StateVector::Zero();
  initial.x[VX]      = 1.0;   // 1 m/s forward
  initial.x[YAW]     = 0.0;   // facing east (ENU)
  initial.P          = StateMatrix::Identity() * 0.01;

  ukf.init(initial);

  // Predict forward 1 second
  for (int i = 0; i < 100; ++i) {
    ukf.predict(0.01);
  }

  // Should have moved ~1 meter in X
  EXPECT_NEAR(ukf.state().x[X], 1.0, 0.05);
  EXPECT_NEAR(ukf.state().x[Y], 0.0, 0.05);
}

// ─── Test 5: Covariance grows during predict (uncertainty increases) ─────────

TEST(UKFTest, CovarianceGrowsDuringPredict) {
  UKF ukf;

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.01;

  ukf.init(initial);

  double initial_trace = initial.P.trace();

  for (int i = 0; i < 50; ++i) {
    ukf.predict(0.01);
  }

  double final_trace = ukf.state().P.trace();

  // Uncertainty should have grown
  EXPECT_GT(final_trace, initial_trace);
}

// ─── Test 6: Update with position measurement corrects state ────────────────

TEST(UKFTest, PositionUpdateCorrectState) {
  UKF ukf;

  State initial;
  initial.x      = StateVector::Zero();
  initial.x[X]   = 5.0;   // we think we're at x=5
  initial.P      = StateMatrix::Identity() * 1.0;  // high uncertainty

  ukf.init(initial);

  // Measurement: we're actually at x=0, y=0, z=0
  Eigen::Vector3d z = Eigen::Vector3d::Zero();

  // Measurement function: extract position from state
  auto h = [](const StateVector& x) -> Eigen::Vector3d {
    return Eigen::Vector3d(x[X], x[Y], x[Z]);
  };

  // Low measurement noise — trust the sensor
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.01;

  ukf.update<3>(z, h, R);

  // State should have moved toward measurement (x=0)
  EXPECT_LT(ukf.state().x[X], 5.0);
  EXPECT_NEAR(ukf.state().x[X], 0.0, 0.5);
}

// ─── Test 7: Angle normalization wraps correctly ─────────────────────────────

TEST(UKFTest, AngleNormalizationAfterPredict) {
  UKF ukf;

  State initial;
  initial.x        = StateVector::Zero();
  initial.x[YAW]   = 3.0;    // near +pi
  initial.x[WZ]    = 1.0;    // rotating — will push yaw past +pi
  initial.P        = StateMatrix::Identity() * 0.01;

  ukf.init(initial);

  for (int i = 0; i < 50; ++i) {
    ukf.predict(0.01);
  }

  // Yaw must always stay within [-pi, pi]
  EXPECT_LE(ukf.state().x[YAW],  M_PI);
  EXPECT_GE(ukf.state().x[YAW], -M_PI);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
