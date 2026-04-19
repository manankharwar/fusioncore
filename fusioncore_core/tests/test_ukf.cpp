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
  // yaw = 0.5 rad → quaternion [cos(0.25), 0, 0, sin(0.25)]
  initial.x[QW]  = std::cos(0.25);
  initial.x[QX]  = 0.0;
  initial.x[QY]  = 0.0;
  initial.x[QZ]  = std::sin(0.25);

  ukf.init(initial);

  EXPECT_TRUE(ukf.is_initialized());
  EXPECT_DOUBLE_EQ(ukf.state().x[X], 1.0);
  EXPECT_DOUBLE_EQ(ukf.state().x[Y], 2.0);
  EXPECT_NEAR(ukf.state().yaw(), 0.5, 1e-9);
}

// ─── Test 2: Predict doesn't explode ────────────────────────────────────────

TEST(UKFTest, PredictRunsWithoutError) {
  UKF ukf;

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;

  ukf.init(initial);

  EXPECT_NO_THROW(ukf.predict(0.01));
  EXPECT_NO_THROW(ukf.predict(0.01));
  EXPECT_NO_THROW(ukf.predict(0.01));
}

// ─── Test 3: Stationary robot stays stationary ──────────────────────────────

TEST(UKFTest, StationaryRobotRemainsStationary) {
  UKF ukf;

  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.01;

  ukf.init(initial);

  for (int i = 0; i < 100; ++i) {
    ukf.predict(0.01);
  }

  EXPECT_NEAR(ukf.state().x[X], 0.0, 0.01);
  EXPECT_NEAR(ukf.state().x[Y], 0.0, 0.01);
  EXPECT_NEAR(ukf.state().x[Z], 0.0, 0.01);
}

// ─── Test 4: Forward motion integrates position ──────────────────────────────
// Use very low process noise so sigma points don't overwhelm the velocity signal

TEST(UKFTest, ForwardMotionIntegratesPosition) {
  // Tight process noise params for this test
  UKFParams params;
  params.q_position     = 1e-6;
  params.q_orientation  = 1e-6;
  params.q_velocity     = 1e-6;
  params.q_angular_vel  = 1e-6;
  params.q_acceleration = 1e-6;
  params.q_gyro_bias    = 1e-6;
  params.q_accel_bias   = 1e-6;

  UKF ukf(params);

  State initial;
  initial.x[VX]  = 1.0;   // 1 m/s forward
  // State() default-constructs with QW=1 (identity quaternion) = yaw 0, facing east
  initial.P      = StateMatrix::Identity() * 1e-4;

  ukf.init(initial);

  for (int i = 0; i < 100; ++i) {
    ukf.predict(0.01);
  }

  // With low noise, 1 m/s for 1 second should integrate to ~1m in X
  EXPECT_NEAR(ukf.state().x[X], 1.0, 0.1);
  EXPECT_NEAR(ukf.state().x[Y], 0.0, 0.1);
}

// ─── Test 5: Covariance grows during predict ─────────────────────────────────

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
  EXPECT_GT(final_trace, initial_trace);
}

// ─── Test 6: Update with position measurement corrects state ────────────────

TEST(UKFTest, PositionUpdateCorrectState) {
  UKF ukf;

  State initial;
  initial.x    = StateVector::Zero();
  initial.x[X] = 5.0;
  initial.P    = StateMatrix::Identity() * 1.0;

  ukf.init(initial);

  Eigen::Vector3d z = Eigen::Vector3d::Zero();

  auto h = [](const StateVector& x) -> Eigen::Vector3d {
    return Eigen::Vector3d(x[X], x[Y], x[Z]);
  };

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.01;

  ukf.update<3>(z, h, R);

  EXPECT_LT(ukf.state().x[X], 5.0);
  EXPECT_NEAR(ukf.state().x[X], 0.0, 0.5);
}

// ─── Test 7: Quaternion stays normalized after many predicts ─────────────────

TEST(UKFTest, QuaternionRemainsNormalizedAfterPredict) {
  UKF ukf;

  State initial;
  // yaw = 3.0 rad → quaternion [cos(1.5), 0, 0, sin(1.5)]
  initial.x[QW] = std::cos(1.5);
  initial.x[QX] = 0.0;
  initial.x[QY] = 0.0;
  initial.x[QZ] = std::sin(1.5);
  initial.x[WZ] = 1.0;  // spinning
  initial.P = StateMatrix::Identity() * 0.01;

  ukf.init(initial);

  for (int i = 0; i < 50; ++i) {
    ukf.predict(0.01);
  }

  // Quaternion must remain unit norm after 50 predict steps
  double qnorm = std::sqrt(
    std::pow(ukf.state().x[QW], 2) + std::pow(ukf.state().x[QX], 2) +
    std::pow(ukf.state().x[QY], 2) + std::pow(ukf.state().x[QZ], 2));
  EXPECT_NEAR(qnorm, 1.0, 1e-9);

  // Derived yaw must be in [-π, π]
  EXPECT_LE(ukf.state().yaw(),  M_PI);
  EXPECT_GE(ukf.state().yaw(), -M_PI);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
