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
  initial.P = ErrorMatrix::Identity() * 0.1;

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
  initial.P = ErrorMatrix::Identity() * 0.01;

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
  initial.P      = ErrorMatrix::Identity() * 1e-4;

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
  initial.P = ErrorMatrix::Identity() * 0.01;

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
  initial.P    = ErrorMatrix::Identity() * 1.0;

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
  initial.P = ErrorMatrix::Identity() * 0.01;

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

// ─── Measurement mean of an angle must not break across the +-pi cut ─────────
//
// Found on the 2026-09-06 rover log. update()/predict_measurement() took a plain
// weighted mean of the measurement sigma points. That is fine for metres and
// wrong for radians: once the sigma points straddle +-pi, averaging values near
// -pi with values near +pi lands nowhere near either, so the innovation and S are
// both built from a mean that is not the mean. On real data with the magnetometer
// enabled the filter reached a 213 km position error in 3 of 24 tuning
// combinations, every one of them at the DEFAULT magnetometer.declination_rad of
// 0.0 (issue #92); any non-zero declination shifted the trajectory off the cut and
// hid it. The fix takes the mean about a reference sigma point, wrapping each
// offset the short way round, which stays correct for any weights including the
// large negative Wm[0] that ruled out an atan2 circular mean.
//
// The state here sits exactly on the cut with a merely uncertain quaternion
// (1e-3, not the 1.0 that some older fixtures used, which throws sigma points off
// the unit sphere entirely and makes any angle mean meaningless). Measured on this
// exact setup: correct gives innovation 0.0500 and S 0.0042, the plain mean gives
// 0.1866 and 0.0601.
TEST(UKFTest, AngleMeasurementMeanSurvivesThePiBoundary) {
  UKF ukf;
  State initial;
  initial.x = StateVector::Zero();
  initial.x[QW] = std::cos(M_PI / 2.0);   // yaw = pi, right on the wrap
  initial.x[QZ] = std::sin(M_PI / 2.0);
  initial.P = ErrorMatrix::Identity() * 1e-4;
  for (int q : {QW, QX, QY, QZ}) initial.P(q, q) = 1e-3;
  ukf.init(initial);

  auto h = [](const StateVector& s) {
    Eigen::Matrix<double, 1, 1> z;
    z[0] = std::atan2(2.0 * (s[QW] * s[QZ] + s[QX] * s[QY]),
                      1.0 - 2.0 * (s[QY] * s[QY] + s[QZ] * s[QZ]));
    return z;
  };
  // 0.05 rad the other side of the cut: the same heading, expressed as -pi+0.05.
  Eigen::Matrix<double, 1, 1> z;
  z[0] = std::atan2(std::sin(M_PI + 0.05), std::cos(M_PI + 0.05));
  Eigen::Matrix<double, 1, 1> R;  R(0, 0) = 0.02 * 0.02;
  Eigen::Matrix<double, 1, 1> innov, S;
  ukf.predict_measurement<1>(z, h, R, innov, S, 0b1);

  EXPECT_NEAR(innov[0], 0.05, 0.02)
      << "innovation " << innov[0] << " rad, expected 0.05. A plain mean of the "
         "measurement sigma points reports 0.187 here.";
  EXPECT_LT(S(0, 0), 0.02)
      << "S = " << S(0, 0) << ", expected about 0.004. A mean taken across the pi "
         "cut inflates it by more than an order of magnitude.";
}
