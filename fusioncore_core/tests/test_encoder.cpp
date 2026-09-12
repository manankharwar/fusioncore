#include <gtest/gtest.h>
#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"
#include "fusioncore/sensors/encoder.hpp"
#include "fusioncore/sensors/imu.hpp"
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"
#include <cmath>

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
// IMU alone: cannot separate WZ from B_GZ (observability problem).
// Encoder alone: measures WZ + B_EWZ (its own bias, calibrated via GPS heading).
// Together (with B_EWZ known from GPS): encoder pins WZ, IMU can estimate B_GZ.
//
// This test simulates the GPS-blackout phase where B_EWZ is already calibrated
// (tight prior on B_EWZ). Without that prior, WZ and the two biases are individually
// unobservable from IMU + encoder alone.

TEST(EncoderTest, IMUAndEncoderTogetherEstimateBias) {
  UKFParams ukf_params;
  ukf_params.q_gyro_bias = 1e-5;  // bias changes very slowly

  UKF ukf(ukf_params);

  State initial;
  initial.x       = StateVector::Zero();
  initial.x[B_GZ] = 0.1;   // wrong bias: reality has zero bias
  initial.P       = StateMatrix::Identity() * 0.1;
  // B_EWZ is calibrated from prior GPS updates: treat as known zero.
  initial.P(B_EWZ, B_EWZ) = 1e-8;

  ukf.init(initial);

  // Reality: robot turning at 0.5 rad/s, zero gyro bias, zero encoder WZ bias
  // IMU reads: WZ + B_GZ = 0.5 + 0.0 = 0.5  (but filter thinks B_GZ=0.1)
  // Encoder reads: WZ + B_EWZ = 0.5 + 0.0 = 0.5  (B_EWZ pinned ~0 by tight prior)

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

// ─── Ignoring a channel a sensor cannot measure (#107, #108) ─────────────────
//
// A Twist message always carries vx, vy and wz, so a source measuring only some
// of them still publishes a number for the rest, and that number is 0.0. The
// PMW3901 optical flow driver is the reported case: it never assigns angular.z.
//
// These two tests pin both halves of that. The first shows the damage a
// zero-filled yaw channel does at ordinary noise. The second shows that feeding
// the PREDICTED value with a huge variance leaves the channel untouched, which
// is what the ROS layer does for any channel omitted from encoder2.channels.
namespace {

// Spin the filter up and get it turning, so WZ is genuinely non-zero.
void spin_up(FusionCore & fc, double & t) {
  const double g = 9.80665;
  for (int i = 0; i < 400; ++i) {
    t += 0.01;
    fc.update_imu(t, 0.0, 0.0, 0.5, 0.0, 0.0, g);   // 0.5 rad/s yaw rate
    if (i % 2 == 0) fc.update_encoder(t, 1.0, 0.0, 0.5);
  }
}

FusionCoreConfig turning_config() {
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  return cfg;
}

}  // namespace

TEST(EncoderTest, ZeroFilledYawChannelCorruptsTheTurnRate) {
  // The bug, stated as a measurement. A second source that publishes wz = 0.0
  // because the field was never assigned, at the 0.02 default noise, drags the
  // filter's turn rate away from the truth every time it arrives.
  FusionCore fc(turning_config());
  State s0;
  fc.init(s0, 0.0);
  double t = 0.0;
  spin_up(fc, t);

  const double wz_before = fc.get_state().x[WZ];
  ASSERT_GT(std::abs(wz_before), 0.2) << "the filter should be turning by now";

  // 20 samples of "definitely not rotating" at the encoder2.yaw_noise default.
  for (int i = 0; i < 20; ++i) {
    t += 0.02;
    fc.update_encoder(t, 1.0, 0.0, 0.0, 0.05 * 0.05, 0.05 * 0.05, 0.02 * 0.02);
  }
  const double wz_after = fc.get_state().x[WZ];

  EXPECT_LT(std::abs(wz_after), std::abs(wz_before) * 0.9)
    << "a zero-filled yaw channel at default noise should visibly drag the turn "
       "rate down. before " << wz_before << " after " << wz_after
    << ". If this ever stops being true the motivation for encoder2.channels is "
       "stale and the docs should say so.";
}

TEST(EncoderTest, AnIgnoredYawChannelHasNoInfluenceWhatever) {
  // The fix, tested the only way that actually proves it: run two identical
  // filters, hand the ignored channel a DIFFERENT number in each, and require
  // the outcome to be bit-identical. If the channel still had any influence, the
  // two would diverge.
  //
  // Comparing against "unchanged from before" would not work, because the same
  // update carries real vx and vy, and those legitimately move WZ through the
  // covariance cross terms. That is the filter working, not leakage.
  auto run = [](double ignored_wz_value) {
    FusionCore fc(turning_config());
    State s0;
    fc.init(s0, 0.0);
    double t = 0.0;
    spin_up(fc, t);
    for (int i = 0; i < 20; ++i) {
      t += 0.02;
      fc.update_encoder(t, 1.0, 0.0, ignored_wz_value,
                        0.05 * 0.05, 0.05 * 0.05, 1e12);
    }
    return fc.get_state().x;
  };

  // One plausible, one absurd. Neither should meaningfully reach the state.
  const double provocation = 99.0;   // rad/s of difference in the ignored channel
  const auto a = run(0.0);
  const auto b = run(provocation);

  // Stated as an influence RATIO rather than a bare epsilon, because that is the
  // quantity with meaning: how much of a change in the ignored channel survives
  // into the state. Measured at 1.5e-11 (1.5e-9 rad/s of WZ from 99 rad/s in).
  // It is not exactly zero because a variance of 1e12 makes the gain tiny rather
  // than identically zero, and the bar below leaves three orders of headroom so
  // this fails on a real regression rather than on floating point weather.
  const double kMaxInfluence = 1e-8;
  EXPECT_LT(std::abs(a[WZ]    - b[WZ])    / provocation, kMaxInfluence)
    << "the ignored yaw value reached WZ";
  EXPECT_LT(std::abs(a[B_EWZ] - b[B_EWZ]) / provocation, kMaxInfluence)
    << "the ignored yaw value moved the encoder WZ bias";
  EXPECT_LT(std::abs(a[VX]    - b[VX])    / provocation, kMaxInfluence)
    << "the ignored yaw value even reached VX";
}

TEST(EncoderTest, PredictingTheYawChannelNeedsTheEncoderBias) {
  // Guards the reason the ROS layer adds B_EWZ when it substitutes a predicted
  // value. encoder_measurement_function maps yaw to WZ + B_EWZ, so bare WZ
  // leaves exactly the bias as innovation rather than zero. Anyone simplifying
  // that line sees here why it cannot be simplified.
  StateVector x = StateVector::Zero();
  x[WZ]    = 0.4;
  x[B_EWZ] = 0.03;

  const auto z = encoder_measurement_function(x);
  EXPECT_NEAR(z[2], x[WZ] + x[B_EWZ], 1e-12) << "encoder yaw reading is WZ + B_EWZ";
  EXPECT_NEAR(z[2] - x[WZ], x[B_EWZ], 1e-12)
    << "so predicting with bare WZ leaves exactly the bias as innovation";
  EXPECT_NEAR(z[2] - (x[WZ] + x[B_EWZ]), 0.0, 1e-12)
    << "and predicting with the bias leaves nothing";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// A channel a sensor cannot measure must contribute NOTHING.
//
// #113: the radar and GNSS velocity inputs passed a literal 0.0 for yaw rate
// with a large variance, on the theory that a large variance makes a channel
// harmless. Large is not silent. The test is not "is the filter still roughly
// right", it is "did adding this source change the answer at all", because that
// is the property being claimed.
TEST(EncoderTest, AVelocityOnlySourceMustNotMoveYawRate) {
  // Truth: constant turn. Feeding the matching centripetal acceleration keeps
  // the IMU consistent with the motion, so any yaw movement comes from the
  // channel under test and not from a contrived setup.
  auto run = [](int mode) {           // 0 = absent, 1 = predicted, 2 = zero+1e6
    FusionCoreConfig cfg;
    cfg.imu_has_magnetometer = false;
    cfg.motion_model = create_motion_model("DifferentialDrive");
    FusionCore fc(cfg);
    State s0;
    fc.init(s0, 0.0);

    const double dt = 0.01, g = 9.80665, speed = 1.0, truth_wz = 0.5;
    for (int step = 1; step * dt <= 30.0 + 1e-9; ++step) {
      const double t = step * dt;
      fc.update_imu(t, 0, 0, truth_wz, 0, speed * truth_wz, g);
      if (step % 2 == 0) {
        fc.update_encoder(t, speed, 0.0, truth_wz);      // the real encoder
        if (mode == 1) {
          const auto & x = fc.get_state().x;
          fc.update_encoder(t, speed, 0.0, x[WZ] + x[B_EWZ],
                            0.05 * 0.05, 0.05 * 0.05, 1e12);
        } else if (mode == 2) {
          fc.update_encoder(t, speed, 0.0, 0.0,
                            0.05 * 0.05, 0.05 * 0.05, 1e6);
        }
      }
    }
    return fc.get_state().x[WZ];
  };

  const double absent    = run(0);
  const double predicted = run(1);
  const double zeroed    = run(2);

  std::cerr << "  filter yaw rate after 30 s turning at 0.5 rad/s\n"
            << "    source absent                 : " << absent    << " rad/s\n"
            << "    source present, predicted     : " << predicted << " rad/s\n"
            << "    source present, zero + 1e6    : " << zeroed    << " rad/s\n";

  // What this actually measured, and it is worth being straight about it: at a
  // variance of 1e6 the zero substitution pulls yaw rate by less than 1e-5 rad/s
  // here, so the defect #113 describes is real in principle and negligible in
  // magnitude on this scenario. The predicted substitution is not bit-identical
  // to absence either, because the update still touches P even when the
  // innovation is exactly zero.
  //
  // The fix is kept on correctness grounds rather than performance: a zero in
  // the measurement vector is a claim the sensor never made, the encoder2 path
  // already does it this way, and a future change to the variance or the gain
  // would turn a negligible pull into a real one with nothing to catch it.
  EXPECT_NEAR(predicted, absent, 1e-3)
      << "substituting the prediction should leave the estimate where it was";
  EXPECT_NEAR(zeroed, absent, 1e-3)
      << "if this ever grows, the zero substitution has started to matter and "
         "the variance or the gain has changed underneath it";
}

// A rejected encoder update must say so.
//
// #124: seven rejection sites discarded a measurement with nothing recorded but
// a counter that never left the core. The encoder is the one that matters most,
// because nearly every ground robot has one and a silently rejected encoder
// update is a direct cause of the drift users report.
TEST(EncoderTest, RejectionRecordsWhyAndHowSurprising) {
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665;
  for (int step = 1; step <= 400; ++step) {
    const double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) fc.update_encoder(t, 1.0, 0.0, 0.0);
  }

  const auto good = fc.get_status();
  EXPECT_EQ(good.encoder_reason, EncoderRejectionReason::ACCEPTED);
  EXPECT_GE(good.encoder_chi2, 0.0)  << "the gate ran, so its distance must be reported";
  EXPECT_LT(good.encoder_chi2, good.encoder_chi2_threshold);

  // Something no differential drive does: 40 m/s sideways.
  fc.update_encoder(4.01, 1.0, 40.0, 0.0);
  const auto bad = fc.get_status();

  std::cerr << "  accepted: chi2 " << good.encoder_chi2
            << " against " << good.encoder_chi2_threshold << "\n"
            << "  rejected: chi2 " << bad.encoder_chi2
            << " against " << bad.encoder_chi2_threshold << "\n";

  EXPECT_EQ(bad.encoder_reason, EncoderRejectionReason::CHI2_FAILED)
      << "an encoder update was thrown away without recording why";
  EXPECT_GT(bad.encoder_chi2, bad.encoder_chi2_threshold)
      << "the published distance must actually explain the rejection";
}
