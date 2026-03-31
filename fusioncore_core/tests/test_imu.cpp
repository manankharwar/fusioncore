#include <gtest/gtest.h>
#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"
#include "fusioncore/sensors/imu.hpp"

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

// ─── Test 4: UKF fuses IMU — total signal (WZ + B_GZ) matches measurement ────
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
