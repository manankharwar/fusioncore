#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"

using namespace fusioncore;

// Self-calibrating fix-to-fix continuity gate (issue #116).
//
// This gate is the only one that can see a metre-scale GPS spike, because chi2
// tests a fix against the filter and its scale is S = HPH' + R. It used to ship
// disabled, so an out-of-box FusionCore accepted spikes up to about 29 m on a
// consumer receiver. The threshold is a property of the receiver, and the
// receiver is present, so the filter measures it rather than asking.
namespace {

FusionCoreConfig base_config() {
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.gnss.base_noise_xy = 1.0;
  cfg.gnss.base_noise_z  = 1.0;
  cfg.outlier_rejection = true;
  cfg.outlier_threshold_gnss = 16.27;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  // Opt in explicitly. The shipped default is OFF, because the gate currently
  // cancels post-blackout re-acquisition (see GnssParams::continuity_auto), and
  // DefaultsToOffUntilRecoveryIsGateAgnostic below is what holds that.
  cfg.gnss.continuity_auto = true;
  return cfg;
}

sensors::GnssFix fix_at(double x, double y) {
  sensors::GnssFix f;
  f.x = x; f.y = y; f.z = 0.0;
  f.hdop = 1.0; f.vdop = 1.0;
  f.satellites = 12;
  f.fix_type = sensors::GnssFixType::DGPS_FIX;
  return f;
}

// Deterministic, repeatable jitter so the learned threshold is stable between
// runs. Amplitude is roughly the scatter a consumer receiver shows at 5 Hz.
double jitter(int k, double amp) {
  return amp * std::sin(k * 2.399963) * std::cos(k * 0.7853981);
}

struct Result {
  double learned_m   = 0.0;
  bool   learned     = false;
  int    clean_rejects = 0;
  bool   spike_rejected = false;
};

Result run(const FusionCoreConfig& cfg, double spike_m, double jitter_amp = 0.35) {
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665, speed = 1.0;
  const double T_LEARN = 90.0;          // 450 fixes at 5 Hz, well past CONT_LEARN_N
  const double T_END   = T_LEARN + 60.0;
  const double t_spike = T_LEARN + 30.0;

  Result r;
  double true_x = 0.0;
  int k = 0;

  for (int step = 1; step * dt <= T_END + 1e-9; ++step) {
    const double t = step * dt;
    true_x += speed * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      fc.update_encoder(t, speed, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }
    if (step % 20 == 0) {                       // 5 Hz
      ++k;
      const bool is_spike = (spike_m > 0.0) &&
                            (std::abs(t - t_spike) < dt * 10.0);
      const double gx = true_x + jitter(k, jitter_amp) + (is_spike ? spike_m : 0.0);
      const double gy = jitter(k + 7, jitter_amp);
      fc.update_gnss(t, fix_at(gx, gy));

      const auto d = fc.get_gnss_debug();
      const bool cont_reject =
        (!d.accepted && d.reason == GnssRejectionReason::CONTINUITY_BREAK);
      if (is_spike) {
        if (cont_reject) r.spike_rejected = true;
      } else if (t > T_LEARN && cont_reject) {
        ++r.clean_rejects;
      }
    }
  }
  const auto st = fc.get_status();
  r.learned_m = st.continuity_limit_m;
  r.learned   = st.continuity_learned;
  return r;
}

} // namespace

TEST(ContinuityAutoTest, LearnsAThresholdFromTheReceiver) {
  const Result r = run(base_config(), /*spike_m=*/0.0);
  std::cerr << "  learned limit: " << r.learned_m << " m\n";
  EXPECT_TRUE(r.learned) << "the gate never finished calibrating";
  EXPECT_GT(r.learned_m, 0.0);
  EXPECT_LE(r.learned_m, 25.0) << "clamped to the documented ceiling";
  EXPECT_GE(r.learned_m, 2.0)  << "never tighter than the documented floor";
}

TEST(ContinuityAutoTest, DoesNotRejectTheReceiverItCalibratedOn) {
  const Result r = run(base_config(), /*spike_m=*/0.0);
  std::cerr << "  clean fixes rejected after calibration: " << r.clean_rejects << "\n";
  EXPECT_EQ(r.clean_rejects, 0)
      << "a self-calibrated gate rejecting the very noise it measured is the "
         "failure this project has paid for most";
}

TEST(ContinuityAutoTest, CatchesASpikeChi2CannotSee) {
  const Result r = run(base_config(), /*spike_m=*/12.0);
  std::cerr << "  learned " << r.learned_m << " m, 12 m spike rejected: "
            << (r.spike_rejected ? "yes" : "no") << "\n";
  EXPECT_TRUE(r.spike_rejected)
      << "a 12 m spike passed a gate calibrated at " << r.learned_m << " m";
}

TEST(ContinuityAutoTest, ExplicitValueWinsAndSkipsLearning) {
  FusionCoreConfig cfg = base_config();
  cfg.gnss.continuity_max_m = 4.0;
  const Result r = run(cfg, /*spike_m=*/12.0);
  EXPECT_DOUBLE_EQ(r.learned_m, 4.0) << "an explicit setting must be used verbatim";
  EXPECT_FALSE(r.learned)            << "explicit means no learning happened";
  EXPECT_TRUE(r.spike_rejected);
}

// The shipped default. Measured on NCLT 2012-06-15: with this gate armed, error
// 300 s after a 461 s blackout is 259.5 m, against 13.4 m with recovery alone,
// because the P inflation that re-admits GNSS sits inside the chi2 block and a
// continuity rejection returns before reaching it. Flip this back when the
// recovery path fires from whichever gate did the rejecting.
TEST(ContinuityAutoTest, DefaultsToOffUntilRecoveryIsGateAgnostic) {
  FusionCoreConfig shipped;                       // untouched defaults
  EXPECT_FALSE(shipped.gnss.continuity_auto)
      << "the learned gate is on by default again: confirm post-blackout "
         "re-acquisition still works on NCLT before landing that";
}

TEST(ContinuityAutoTest, CanStillBeTurnedOff) {
  FusionCoreConfig cfg = base_config();
  cfg.gnss.continuity_auto = false;
  const Result r = run(cfg, /*spike_m=*/12.0);
  EXPECT_FALSE(r.learned);
  EXPECT_DOUBLE_EQ(r.learned_m, 0.0);
  EXPECT_FALSE(r.spike_rejected) << "disabled must mean the gate never fires";
}

// The tests above all land on the 2 m floor, because the synthetic scatter is
// small. That proves the clamp and not the arithmetic. This drives two receivers
// of genuinely different quality and checks the threshold tracks the data:
// a noisier receiver must be given a wider gate, and neither must be rejected
// on the noise it was calibrated on.
TEST(ContinuityAutoTest, ThresholdTracksHowNoisyTheReceiverIs) {
  struct Row { double amp; double learned; int rejects; };
  Row rows[3];
  const double amps[3] = {0.35, 2.0, 5.0};

  for (int i = 0; i < 3; ++i) {
    const Result r = run(base_config(), /*spike_m=*/0.0, amps[i]);
    rows[i] = {amps[i], r.learned_m, r.clean_rejects};
    std::cerr << "  jitter amplitude " << amps[i] << " m -> learned "
              << r.learned_m << " m, clean rejections " << r.clean_rejects << "\n";
    EXPECT_EQ(r.clean_rejects, 0)
        << "rejected good fixes from the very receiver it calibrated on, at "
           "jitter amplitude " << amps[i];
  }

  EXPECT_GT(rows[1].learned, rows[0].learned)
      << "a noisier receiver must get a wider gate, not the same one";
  EXPECT_GT(rows[2].learned, rows[1].learned)
      << "the threshold stopped responding to the data";
  EXPECT_GT(rows[2].learned, 2.0)
      << "still pinned to the floor, so the learned value is never exercised";
}
