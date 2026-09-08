#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"
#include <cmath>

using namespace fusioncore;

// imu_fixed_rate_hz: propagate by 1/rate instead of by the gap between stamps,
// so stamp jitter cannot reach the integrator. The reason it is worth having is
// that anything downstream of an integrator amplifies timestamp error, and the
// reason it is dangerous is that a wrong rate is a SYSTEMATIC error rather than
// a noisy one. Both properties are pinned here.
namespace {

FusionCoreConfig dt_config(double fixed_rate) {
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.imu_fixed_rate_hz = fixed_rate;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  return cfg;
}

// Drive a constant yaw rate for 20 s, with a deterministic wobble applied to
// every IMU stamp. Returns final yaw in degrees.
double yaw_after_jittered_run(double fixed_rate, double jitter_s) {
  FusionCore fc(dt_config(fixed_rate));
  State s0;
  fc.init(s0, 0.0);
  const double g = 9.80665, rate = 100.0, wz = 0.3;
  for (int k = 1; k * (1.0 / rate) <= 20.0 + 1e-9; ++k) {
    const double clean = k / rate;
    // Deterministic, zero-mean, and small: the kind of jitter a non-realtime
    // OS puts on a serial IMU. It must not change the answer.
    const double t = clean + jitter_s * std::sin(k * 0.9);
    fc.update_imu(t, 0.0, 0.0, wz, 0.0, 0.0, g);
  }
  const auto & x = fc.get_state().x;
  return std::atan2(2.0 * (x[QW] * x[QZ] + x[QX] * x[QY]),
                    1.0 - 2.0 * (x[QY] * x[QY] + x[QZ] * x[QZ])) * 180.0 / M_PI;
}

} // namespace

// ─── Stamp jitter must not change the answer when the rate is nominal ───────
TEST(FixedDtTest, NominalRateIsImmuneToStampJitter) {
  const double clean   = yaw_after_jittered_run(100.0, 0.0);
  const double jittered = yaw_after_jittered_run(100.0, 0.002);   // 2 ms wobble
  EXPECT_NEAR(clean, jittered, 1e-6)
    << "with a nominal dt the propagation must not depend on stamp jitter at all";
}

// ─── Without it, the same jitter does reach the integrator ──────────────────
TEST(FixedDtTest, TimestampDerivedDtIsNotImmune) {
  const double clean    = yaw_after_jittered_run(0.0, 0.0);
  const double jittered = yaw_after_jittered_run(0.0, 0.002);
  EXPECT_GT(std::abs(clean - jittered), 1e-9)
    << "stamp jitter should reach the integrator when dt comes from stamps; "
    << "if this passes trivially the test above proves nothing";
}

// ─── Off by default ─────────────────────────────────────────────────────────
TEST(FixedDtTest, DisabledByDefault) {
  EXPECT_DOUBLE_EQ(FusionCoreConfig{}.imu_fixed_rate_hz, 0.0);
}

// ─── A wrong configured rate must be reported, not silently integrated ──────
// This is the hazard the feature introduces: at a wrong rate the filter
// integrates the wrong amount of time on every single step, and unlike jitter
// that error does not average out.
TEST(FixedDtTest, RateMismatchIsDetected) {
  FusionCore fc(dt_config(100.0));   // claims 100 Hz
  State s0;
  fc.init(s0, 0.0);
  const double g = 9.80665, actual_rate = 109.0;   // a real BNO085 reading
  for (int k = 1; k <= 900; ++k) {
    fc.update_imu(k / actual_rate, 0.0, 0.0, 0.0, 0.0, 0.0, g);
  }
  const auto st = fc.get_status();
  EXPECT_NEAR(st.imu_rate_observed_hz, actual_rate, 1.0);
  EXPECT_TRUE(st.imu_fixed_rate_mismatch)
    << "configured 100 Hz against an actual " << st.imu_rate_observed_hz
    << " Hz was not flagged, so the filter would integrate the wrong dt silently";
}

TEST(FixedDtTest, MatchingRateIsNotFlagged) {
  FusionCore fc(dt_config(100.0));
  State s0;
  fc.init(s0, 0.0);
  const double g = 9.80665;
  for (int k = 1; k <= 900; ++k) fc.update_imu(k / 100.0, 0, 0, 0, 0, 0, g);
  EXPECT_FALSE(fc.get_status().imu_fixed_rate_mismatch);
}

// ─── A wrong rate does not merely drift, it starts rejecting the IMU ────────
// The clock advances at the nominal rate while stamps advance at the real one,
// so the two separate at exactly the rate error. Once that exceeds
// max_measurement_delay the stale-skew guard begins discarding IMU messages.
// Worth knowing, because "my IMU is being rejected" is a much more visible
// symptom than "my dt is 9% wrong", and this is where it comes from.
TEST(FixedDtTest, WrongRateEventuallyRejectsTheImu) {
  FusionCore fc(dt_config(100.0));
  State s0;
  fc.init(s0, 0.0);
  const double g = 9.80665, actual_rate = 109.0;
  for (int k = 1; k <= 3000; ++k) fc.update_imu(k / actual_rate, 0, 0, 0, 0, 0, g);

  const auto st = fc.get_status();
  EXPECT_TRUE(st.imu_fixed_rate_mismatch) << "the mismatch itself must be flagged";
  EXPECT_GT(st.imu_stale_rejects, 0)
    << "a 9% rate error should eventually push the clock past "
       "max_measurement_delay and start rejecting IMU messages";
}
