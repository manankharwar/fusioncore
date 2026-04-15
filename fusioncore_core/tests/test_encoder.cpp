#include <gtest/gtest.h>
#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"
#include "fusioncore/sensors/encoder.hpp"
#include "fusioncore/sensors/imu.hpp"

using namespace fusioncore;
using namespace fusioncore::sensors;

// ─── Test 1: Measurement function maps state correctly ───────────────────────

TEST(EncoderTest, MeasurementFunctionMapsState) {
  StateVector x = StateVector::Zero();
  x[VX] = 1.5;
  x[VY] = 0.0;
  x[WZ] = 0.3;
  x[B_GZ] = 0.0;

  EncoderMeasurement z = encoder_measurement_function(x);

  EXPECT_DOUBLE_EQ(z[0], 1.5);
  EXPECT_DOUBLE_EQ(z[1], 0.0);
  EXPECT_DOUBLE_EQ(z[2], 0.3);
}

// ─── Test 2: Encoder yaw rate maps directly to WZ (no gyro bias subtraction) ─
// Encoders are wheel-based: they have no gyro bias. The gyro bias state B_GZ
// belongs only in the IMU measurement function. Subtracting B_GZ here would
// incorrectly couple encoder updates to the gyro bias estimate.

TEST(EncoderTest, EncoderYawRateMapsDirectlyToWZ) {
  StateVector x = StateVector::Zero();
  x[WZ]   = 0.5;    // angular velocity state
  x[B_GZ] = 0.1;    // gyro bias: should NOT affect encoder measurement

  EncoderMeasurement z = encoder_measurement_function(x);

  // Encoder measures WZ directly, not (WZ - B_GZ)
  EXPECT_DOUBLE_EQ(z[2], 0.5);
}

// ─── Test 3: Noise matrix is diagonal and positive ───────────────────────────

TEST(EncoderTest, NoiseMatrixIsDiagonalAndPositive) {
  EncoderParams params;
  EncoderNoiseMatrix R = encoder_noise_matrix(params);

  for (int i = 0; i < ENCODER_DIM; ++i) {
    EXPECT_GT(R(i,i), 0.0);
  }
  for (int i = 0; i < ENCODER_DIM; ++i) {
    for (int j = 0; j < ENCODER_DIM; ++j) {
      if (i != j) EXPECT_DOUBLE_EQ(R(i,j), 0.0);
    }
  }
}

// ─── Test 4: Encoder alone corrects velocity ─────────────────────────────────

TEST(EncoderTest, EncoderUpdateCorrectVelocity) {
  UKF ukf;

  State initial;
  initial.x     = StateVector::Zero();
  initial.x[VX] = 5.0;   // wrong: think we're going 5 m/s
  initial.P     = StateMatrix::Identity() * 1.0;

  ukf.init(initial);

  // Encoder says: actually going 1 m/s
  EncoderMeasurement z = EncoderMeasurement::Zero();
  z[0] = 1.0;

  EncoderParams params;
  EncoderNoiseMatrix R = encoder_noise_matrix(params);

  for (int i = 0; i < 50; ++i) {
    ukf.predict(0.01);
    ukf.update<ENCODER_DIM>(z, encoder_measurement_function, R);
  }

  EXPECT_NEAR(ukf.state().x[VX], 1.0, 0.1);
}

// ─── Test 5: IMU + encoder together break bias/velocity coupling ─────────────
// This is the key test: the moment FusionCore does something no single sensor can.
// IMU alone: cannot separate WZ from B_GZ (observability problem)
// Encoder alone: gives WZ - B_GZ (independent velocity reference)
// Together: the coupling breaks: filter estimates both correctly

TEST(EncoderTest, IMUAndEncoderTogetherEstimateBias) {
  UKFParams ukf_params;
  ukf_params.q_gyro_bias = 1e-5;  // bias changes very slowly

  UKF ukf(ukf_params);

  State initial;
  initial.x       = StateVector::Zero();
  initial.x[B_GZ] = 0.1;   // wrong bias: reality has zero bias
  initial.P       = StateMatrix::Identity() * 0.1;

  ukf.init(initial);

  // Reality: robot turning at 0.5 rad/s, zero gyro bias
  // IMU reads: WZ + B_GZ = 0.5 + 0.0 = 0.5  (but filter thinks bias=0.1)
  // Encoder reads: WZ - B_GZ = 0.5 - 0.0 = 0.5

  ImuMeasurement     z_imu     = ImuMeasurement::Zero();
  EncoderMeasurement z_encoder = EncoderMeasurement::Zero();

  z_imu[2]     = 0.5;   // IMU wz reading
  z_encoder[2] = 0.5;   // encoder wz reading

  ImuParams     imu_params;
  EncoderParams enc_params;
  ImuNoiseMatrix     R_imu = imu_noise_matrix(imu_params);
  EncoderNoiseMatrix R_enc = encoder_noise_matrix(enc_params);

  for (int i = 0; i < 300; ++i) {
    ukf.predict(0.01);
    ukf.update<IMU_DIM>(z_imu, imu_measurement_function, R_imu);
    ukf.update<ENCODER_DIM>(z_encoder, encoder_measurement_function, R_enc);
  }

  // With both sensors, bias should converge toward zero
  // and WZ should converge toward 0.5
  EXPECT_NEAR(ukf.state().x[B_GZ], 0.0, 0.05);
  EXPECT_NEAR(ukf.state().x[WZ],   0.5, 0.05);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
