#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"
#include <cmath>

using namespace fusioncore;

// Regression tests for issue #73: sensors on different time bases.
//
// A user's IMU driver stamped messages ~3 s ahead of the encoder's (correct)
// clock. The filter clock rode the IMU; every encoder message then arrived
// ~3 s "in the past", predict_to() re-based the clock backward, and the next
// IMU message re-integrated the whole 3 s window forward through the motion
// model again, at the IMU rate. Perfect wheel odometry in, ~10 m/s position
// runaway out. The fix rejects measurements from a lagging stream as STALE
// (its own stamps still advance: skew, not a reset) while preserving the
// ff16c65 re-base for a genuine time-base reset (the stream's own stamps
// jump backward too).

namespace {

FusionCoreConfig skew_config() {
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.outlier_rejection = true;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  return cfg;
}

constexpr double g = 9.80665;

} // namespace

// ─── The issue #73 reproduction ──────────────────────────────────────────────
// IMU stamps run +3 s ahead of the encoder. The encoder reports a healthy
// 1 m/s drive. Before the fix this diverged at ~10x the real speed because the
// clock oscillated between the two bases and the offset window was
// re-integrated at the IMU rate. Now the encoder must be rejected as stale
// (counted, visible) and the position must stay bounded instead of exploding.
TEST(ClockSkewTest, SkewedSensorPairDoesNotDiverge) {
  FusionCore fc(skew_config());
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01;          // 100 Hz IMU
  const double skew = 3.0;         // IMU clock ahead by 3 s

  for (int step = 1; step * dt <= 10.0 + 1e-9; ++step) {
    double t = step * dt;
    fc.update_imu(t + skew, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      // 50 Hz encoder on the CORRECT clock: robot driving 1 m/s
      fc.update_encoder(t, 1.0, 0.0, 0.0);
    }
  }

  auto status = fc.get_status();
  const auto& x = fc.get_state().x;

  // Every encoder update must have been rejected as stale, and counted.
  EXPECT_GT(status.encoder_stale_rejects, 400);
  // The IMU (the clock-driving stream) must still be fusing normally.
  EXPECT_GT(status.update_count, 500);
  EXPECT_EQ(status.imu_stale_rejects, 0);
  // The whole point: no runaway. Before the fix |position| reached hundreds
  // of meters here; with the encoder rejected and zero acceleration the
  // filter has no reason to move.
  double dist = std::sqrt(x[X] * x[X] + x[Y] * x[Y]);
  EXPECT_LT(dist, 5.0);
}

// ─── The ff16c65 case must keep working ──────────────────────────────────────
// A genuine time-base reset (bag replay restart, clock correction) shows up as
// the SAME stream jumping backward. The filter must re-base and keep fusing:
// no crash, no permanent rejection of the new base.
TEST(ClockSkewTest, TrueClockResetStillRebases) {
  FusionCore fc(skew_config());
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01;

  // Phase 1: both sensors on one base, out to t=10.
  for (int step = 1; step * dt <= 10.0 + 1e-9; ++step) {
    double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) fc.update_encoder(t, 0.0, 0.0, 0.0);
  }
  int count_before = fc.get_status().update_count;

  // Phase 2: the time base resets to t=1 and both streams continue on it.
  for (int step = 1; step * dt <= 1.0 + 1e-9; ++step) {
    double t = 1.0 + step * dt;
    EXPECT_NO_THROW(fc.update_imu(t, 0, 0, 0, 0, 0, g));
    if (step % 2 == 0) fc.update_encoder(t, 0.0, 0.0, 0.0);
  }

  auto status = fc.get_status();
  // Updates kept flowing after the reset: the re-base path still works.
  EXPECT_GT(status.update_count, count_before + 50);
  // A reset is not skew: nothing should have been counted as stale.
  EXPECT_EQ(status.encoder_stale_rejects, 0);
  // State stayed finite through the jump.
  const auto& x = fc.get_state().x;
  EXPECT_TRUE(std::isfinite(x[X]));
  EXPECT_TRUE(std::isfinite(x[Y]));
}

// ─── Small offsets inside the delay window are unaffected ────────────────────
// An encoder 0.3 s behind the IMU (network latency, driver queue) is within
// max_measurement_delay and must keep fusing exactly as before the fix.
TEST(ClockSkewTest, SlightlyDelayedEncoderStillFuses) {
  FusionCore fc(skew_config());
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01;
  const double lag = 0.3;   // within the 0.5 s window

  for (int step = 1; step * dt <= 5.0 + 1e-9; ++step) {
    double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0 && t - lag > 0.0) {
      fc.update_encoder(t - lag, 0.0, 0.0, 0.0);
    }
  }

  auto status = fc.get_status();
  EXPECT_EQ(status.encoder_stale_rejects, 0);
  EXPECT_EQ(status.imu_stale_rejects, 0);
  // The encoder actually fused: its health reflects a live sensor.
  EXPECT_EQ(status.encoder_health, SensorHealth::OK);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
