#include <gtest/gtest.h>
#include <limits>
#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"
#include "fusioncore/sensors/imu.hpp"
#include "fusioncore/fusioncore.hpp"

using namespace fusioncore;
using namespace fusioncore::sensors;

// ─── Test 1: IMU measurement function maps state correctly ───────────────────

TEST(IMUTest, MeasurementFunctionMapsState) {
  StateVector x = StateVector::Zero();

  x[WX] = 0.1;  x[WY] = 0.2;  x[WZ] = 0.3;
  x[AX] = 1.0;  x[AY] = 2.0;  x[AZ] = 9.8;
  x[B_GX] = 0.0; x[B_GY] = 0.0; x[B_GZ] = 0.0;
  x[B_AX] = 0.0; x[B_AY] = 0.0; x[B_AZ] = 0.0;

  ImuMeasurement z = imu_measurement_function(x);

  EXPECT_DOUBLE_EQ(z[0], 0.1);
  EXPECT_DOUBLE_EQ(z[1], 0.2);
  EXPECT_DOUBLE_EQ(z[2], 0.3);
  constexpr double g = 9.80665;  // gravity (cp=1, cr=1 at zero roll/pitch)
  EXPECT_DOUBLE_EQ(z[3], 1.0);   // AX, no pitch → no gravity x-component
  EXPECT_DOUBLE_EQ(z[4], 2.0);   // AY, no roll  → no gravity y-component
  EXPECT_DOUBLE_EQ(z[5], 9.8 + g); // AZ + gravity at zero roll/pitch
}

// ─── Test 2: Bias shifts the expected measurement ────────────────────────────

TEST(IMUTest, BiasShiftsMeasurement) {
  StateVector x = StateVector::Zero();

  x[WX]   = 0.1;
  x[B_GX] = 0.05;

  x[AZ]   = 9.8;
  x[B_AZ] = 0.2;

  ImuMeasurement z = imu_measurement_function(x);

  constexpr double g = 9.80665;
  EXPECT_DOUBLE_EQ(z[0], 0.15);
  EXPECT_DOUBLE_EQ(z[5], 9.8 + 0.2 + g); // AZ + B_AZ + gravity at zero roll/pitch
}

// ─── Test 3: Noise matrix is diagonal and positive ───────────────────────────

TEST(IMUTest, NoiseMatrixIsDiagonalAndPositive) {
  ImuParams params;
  ImuNoiseMatrix R = imu_noise_matrix(params);

  for (int i = 0; i < IMU_DIM; ++i) {
    EXPECT_GT(R(i,i), 0.0);
  }

  for (int i = 0; i < IMU_DIM; ++i) {
    for (int j = 0; j < IMU_DIM; ++j) {
      if (i != j) EXPECT_DOUBLE_EQ(R(i,j), 0.0);
    }
  }
}

// ─── Test 4: UKF fuses IMU: total signal (WZ + B_GZ) matches measurement ────
// Observability note: IMU alone measures WZ + B_GZ, it cannot separate them.
// That separation happens when motion changes (encoder + IMU together).
// What we CAN verify: after fusion, the predicted measurement matches reality.

TEST(IMUTest, UKFUpdateFusesIMUMeasurement) {
  UKFParams ukf_params;
  ukf_params.q_gyro_bias  = 1e-4;
  ukf_params.q_accel_bias = 1e-4;

  UKF ukf(ukf_params);

  State initial;
  initial.x       = StateVector::Zero();
  initial.x[B_GZ] = 0.1;   // initial bias estimate
  initial.P       = StateMatrix::Identity() * 0.1;

  ukf.init(initial);

  // IMU reads 0.5 rad/s on Z
  ImuMeasurement z = ImuMeasurement::Zero();
  z[2] = 0.5;

  ImuParams imu_params;
  ImuNoiseMatrix R = imu_noise_matrix(imu_params);

  for (int i = 0; i < 200; ++i) {
    ukf.predict(0.01);
    ukf.update<IMU_DIM>(z, imu_measurement_function, R);
  }

  // What the filter predicts the IMU should read
  double predicted_reading = ukf.state().x[WZ] + ukf.state().x[B_GZ];

  // The predicted measurement must match the actual measurement
  EXPECT_NEAR(predicted_reading, 0.5, 0.01);
}

// ─── Test 5: Custom noise params change R matrix values ──────────────────────

TEST(IMUTest, CustomNoiseParamsApplied) {
  ImuParams params;
  params.gyro_noise_x  = 0.01;
  params.accel_noise_z = 0.5;

  ImuNoiseMatrix R = imu_noise_matrix(params);

  EXPECT_DOUBLE_EQ(R(0,0), 0.01 * 0.01);
  EXPECT_DOUBLE_EQ(R(5,5), 0.5  * 0.5);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// ─── IMU and encoders must be told when they disagree about turn direction ───
//
// A rover ran for months with its gyro yaw rate inverted: the BNO085 in UART-RVC
// mode reports yaw increasing clockwise while REP-103 is counterclockwise
// positive. Both sensors were healthy, the encoders were right, and the filter
// watched them contradict each other every cycle without comment. The
// disagreement was noticed twice and blamed on the wheels both times. Because
// imu.gyro_noise defaults far tighter than encoder.yaw_noise, the filter leans on
// the gyro, which was the sensor that was wrong.
//
// Votes are counted rather than requiring an unbroken stretch: a hand-driven
// rover corrects constantly, and on the 2026-09-06 log the longest interval with
// both sensors above even 0.05 rad/s was 0.9 s, so a continuity rule would never
// fire on real driving. Replaying that log: 6 percent of turning samples disagree
// with the correct sign, 80 percent with the gyro inverted.
namespace {
// Drive a weaving course, feeding the encoder yaw rate and the IMU yaw rate with
// a chosen relative sign.
bool yaw_sign_conflict_after(double imu_sign, int seconds = 40) {
  FusionCoreConfig cfg;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  const double dt = 0.02;
  double t = 0.0;
  for (int i = 0; i < seconds * 50; ++i) {
    t += dt;
    // Alternate left and right every 2 s, well above the 0.08 rad/s gate.
    const double wz = ((i / 100) % 2 == 0) ? 0.30 : -0.30;
    fc.update_imu(t, 0.0, 0.0, imu_sign * wz, 0.0, 0.0, 9.80665);
    fc.update_encoder(t, 0.4, 0.0, wz);
  }
  return fc.get_status().yaw_rate_sign_conflict;
}
}  // namespace

TEST(IMUTest, YawRateSignConflictFlaggedWhenGyroIsInverted) {
  EXPECT_TRUE(yaw_sign_conflict_after(-1.0))
      << "an inverted gyro must be reported, not silently trusted";
}

TEST(IMUTest, YawRateSignConflictQuietWhenTheyAgree) {
  EXPECT_FALSE(yaw_sign_conflict_after(+1.0))
      << "sensors that agree must never raise a frame-convention alarm";
}

TEST(IMUTest, YawRateSignIgnoresSlowDriftThatIsNotATurn) {
  // Below the turning gate the sign carries no information: a straight-driving
  // differential rover fabricates small yaw from wheel scale mismatch, and gyro
  // noise crosses zero freely. Neither may vote.
  FusionCoreConfig cfg;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  double t = 0.0;
  for (int i = 0; i < 4000; ++i) {
    t += 0.02;
    fc.update_imu(t, 0.0, 0.0, 0.02, 0.0, 0.0, 9.80665);   // opposite signs,
    fc.update_encoder(t, 0.4, 0.0, -0.02);                  // but far too slow
  }
  EXPECT_FALSE(fc.get_status().yaw_rate_sign_conflict);
  EXPECT_EQ(fc.get_status().yaw_rate_turn_samples, 0)
      << "samples below the turning gate must not be counted at all";
}

// ─── A sample carrying NaN must never reach the filter ──────────────────────
//
// The same hole GNSSTest.NonFiniteFixIsRejectedAndNamed closed for GNSS, on the
// sensor that feeds the filter most often (#103). The chi2 gate cannot catch it:
// a NaN innovation compares false against the threshold, so it passes the gate
// rather than failing it, and one NaN in the state or the covariance makes every
// later estimate NaN with no way back short of a reset.
TEST(IMUTest, NonFiniteSampleIsRejectedAndNamed) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const double g = 9.80665;

  for (int field = 0; field < 6; ++field) {
    for (const double bad : {nan, inf}) {
      FusionCoreConfig cfg;
      FusionCore fc(cfg);
      State s0;
      fc.init(s0, 0.0);

      // A still, level IMU: otherwise entirely acceptable.
      double sample[6] = {0.0, 0.0, 0.0, 0.0, 0.0, g};
      sample[field] = bad;
      fc.update_imu(0.01, sample[0], sample[1], sample[2],
                    sample[3], sample[4], sample[5]);

      EXPECT_EQ(fc.get_status().imu_reason, ImuRejectionReason::NOT_FINITE)
          << "field " << field << " = " << bad << " was not rejected as NOT_FINITE";
      EXPECT_TRUE(fc.get_state().x.allFinite())
          << "field " << field << " = " << bad << " reached the state";
      EXPECT_TRUE(fc.get_state().P.allFinite())
          << "field " << field << " = " << bad << " reached the covariance";

      // And the filter must still take an ordinary sample afterwards.
      fc.update_imu(0.02, 0.0, 0.0, 0.0, 0.0, 0.0, g);
      EXPECT_EQ(fc.get_status().imu_reason, ImuRejectionReason::ACCEPTED)
          << "an ordinary sample after field " << field << " was not accepted";
    }
  }
}

// The orientation path is the other IMU entry point, with the same exposure.
// 9-axis, so roll, pitch and yaw all reach the filter.
TEST(IMUTest, NonFiniteOrientationIsRejectedAndNamed) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  for (int field = 0; field < 6; ++field) {
    for (const double bad : {nan, inf}) {
      FusionCoreConfig cfg;
      cfg.imu_has_magnetometer = true;
      FusionCore fc(cfg);
      State s0;
      fc.init(s0, 0.0);

      // Level, facing the filter's own yaw, with an AHRS-like covariance.
      double angles[3] = {0.0, 0.0, 0.0};
      double cov[9] = {1e-4, 0.0, 0.0,
                       0.0, 1e-4, 0.0,
                       0.0, 0.0, 1e-3};
      if (field < 3) angles[field] = bad;
      else           cov[(field - 3) * 4] = bad;   // the variances: 0, 4 and 8
      fc.update_imu_orientation(0.01, angles[0], angles[1], angles[2], cov);

      EXPECT_EQ(fc.get_status().imu_reason, ImuRejectionReason::NOT_FINITE)
          << "field " << field << " = " << bad << " was not rejected as NOT_FINITE";
      EXPECT_TRUE(fc.get_state().x.allFinite())
          << "field " << field << " = " << bad << " reached the state";
      EXPECT_TRUE(fc.get_state().P.allFinite())
          << "field " << field << " = " << bad << " reached the covariance";

      const double ordinary_cov[9] = {1e-4, 0.0, 0.0,
                                      0.0, 1e-4, 0.0,
                                      0.0, 0.0, 1e-3};
      fc.update_imu_orientation(0.02, 0.0, 0.0, 0.0, ordinary_cov);
      EXPECT_EQ(fc.get_status().imu_reason, ImuRejectionReason::ACCEPTED)
          << "an ordinary orientation after field " << field << " was not accepted";
    }
  }
}
