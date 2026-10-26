#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"

using namespace fusioncore;

// Re-anchoring after a blackout, and the two properties that make it safe to ship
// with rather than the RMSE that makes it worth shipping.
//
// Measured on NCLT 2026-10-18, replay, full trajectory:
//
//   2012-02-04  142.792 -> 59.689    2012-08-20  171.393 -> 46.671
//   2012-09-28   23.638 -> 10.008    2013-04-05   16.677 -> 16.011
//   2012-06-15   78.139 -> 78.139 (one 462 s gap holds 84.6% of its error)
//
// The ablation that set the defaults: with the persistent-bias trigger ON as well,
// the same five land at 61.357 / 60.674 / 13.750 / 16.005 / 78.109 while firing
// twice as often, so the bias trigger is OFF by default. These tests pin that
// decision, because an RMSE table in a markdown file does not fail a build.
namespace {

FusionCoreConfig reanchor_config() {
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
  cfg.adaptive_imu = cfg.adaptive_encoder = cfg.adaptive_gnss = true;
  cfg.motion_model = create_motion_model("DifferentialDrive");
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

struct Outcome {
  double err_30s  = 0.0;
  double err_300s = 0.0;
  long   reanchors = 0;
  double last_jump = 0.0;
  double max_yaw_step = 0.0;   // largest single-step heading change, radians
};

// Truth drives straight East. During the blackout the wheels over-report, so the
// filter runs several hundred metres ahead of the robot by the time fixes return.
// blackout_s = 0 gives a healthy run with GNSS available throughout.
Outcome run(FusionCore& fc, double blackout_s) {
  State s0;
  fc.init(s0, 0.0);
  const double dt = 0.01, g = 9.80665;
  const double TRUE_SPEED = 1.5, SLIP_SPEED = 2.1;
  const double T_PRE = 120.0, T_POST = 300.0;
  const double t_out_start = T_PRE, t_out_end = T_PRE + blackout_s;
  const double t_end = t_out_end + T_POST;

  Outcome o;
  double true_x = 0.0;
  double prev_yaw = 0.0;
  bool   have_prev_yaw = false;
  for (int step = 1; step * dt <= t_end + 1e-9; ++step) {
    const double t = step * dt;
    true_x += TRUE_SPEED * dt;
    const bool blackout = (blackout_s > 0.0 && t >= t_out_start && t < t_out_end);

    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      fc.update_encoder(t, blackout ? SLIP_SPEED : TRUE_SPEED, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }
    if (step % 20 == 0 && !blackout) {       // 5 Hz GNSS at the true position
      fc.update_gnss(t, fix_at(true_x, 0.0));
    }
    const double err = std::abs(fc.get_state().x[X] - true_x);
    if (std::abs(t - (t_out_end +  30.0)) < dt * 0.5) o.err_30s  = err;
    if (std::abs(t - (t_out_end + 300.0)) < dt * 0.5) o.err_300s = err;

    // Largest single-step heading change anywhere in the run. A re-anchor must not
    // show up here: it moves position only.
    const auto& x = fc.get_state().x;
    const double yaw = std::atan2(2.0 * (x[QW] * x[QZ] + x[QX] * x[QY]),
                                  1.0 - 2.0 * (x[QY] * x[QY] + x[QZ] * x[QZ]));
    if (have_prev_yaw) {
      double d = yaw - prev_yaw;
      while (d >  M_PI) d -= 2.0 * M_PI;
      while (d < -M_PI) d += 2.0 * M_PI;
      o.max_yaw_step = std::max(o.max_yaw_step, std::abs(d));
    }
    prev_yaw = yaw;
    have_prev_yaw = true;
  }
  o.reanchors = fc.gnss_reanchor_count();
  o.last_jump = fc.get_status().gnss_reanchor_last_jump_m;
  return o;
}

} // namespace

// The default must not change behaviour for anyone upgrading.
TEST(GnssReanchorTest, DefaultIsOffAndNothingFires) {
  auto cfg = reanchor_config();
  EXPECT_FALSE(cfg.gnss_reanchor_enabled);
  EXPECT_FALSE(cfg.gnss_reanchor_bias_trigger)
    << "the bias trigger must default OFF: with it on, NCLT 2012-08-20 lands at "
       "60.674 against 46.671 with it off, and 2012-09-28 at 13.750 against 10.008";
  FusionCore fc(cfg);
  const auto o = run(fc, 460.0);
  EXPECT_EQ(o.reanchors, 0);
  std::cerr << "  default off: error +300 s = " << o.err_300s << " m\n";
}

// The property that makes it safe: a filter that is tracking fine never re-anchors.
// Measured on NCLT, the lowest error any re-anchor fired at was 138.61 m against raw
// GNSS medians of 5 to 15 m.
TEST(GnssReanchorTest, NeverFiresWhileHealthy) {
  auto cfg = reanchor_config();
  cfg.gnss_reanchor_enabled = true;
  FusionCore fc(cfg);
  const auto o = run(fc, 0.0);          // GNSS available throughout
  EXPECT_EQ(o.reanchors, 0)
    << "re-anchored on a healthy run, which is the one thing this must never do";
  EXPECT_LT(o.err_300s, 5.0);
}

// The property that makes it worth shipping, stated against what this synthetic
// actually shows.
//
// A CLEAN blackout does not reproduce the NCLT failure: the existing recovery
// inflation handles it, and the no-re-anchor arm here converges to 0.013 m by
// +300 s on its own. That was found on 2026-10-17 and it is why the synthetic work
// had to add a gyro bias to throw the filter sideways before the defect appeared.
// So this test asserts on the RECOVERY TIME at +30 s, where the difference is real
// and large, and not on the settled error at +300 s, where both arms agree to
// within 2% and an assertion would be measuring noise.
//
// That matches what the NCLT windows say the fix does: on 2012-02-04 the time spent
// re-converging falls 44.5% to 11.5% while re-convergence RMSE falls only 215.8 to
// 175.3. The win is less time wrong, not a smaller worst case.
TEST(GnssReanchorTest, ShortensRecoveryAfterABlackout) {
  auto off = reanchor_config();
  FusionCore fc_off(off);
  const auto baseline = run(fc_off, 460.0);

  auto on = reanchor_config();
  on.gnss_reanchor_enabled = true;
  FusionCore fc_on(on);
  const auto fixed = run(fc_on, 460.0);

  std::cerr << "  off: +30 s " << baseline.err_30s << " m, +300 s "
            << baseline.err_300s << " m, re-anchors " << baseline.reanchors << "\n"
            << "  on : +30 s " << fixed.err_30s << " m, +300 s "
            << fixed.err_300s << " m, re-anchors " << fixed.reanchors
            << ", last jump " << fixed.last_jump << " m\n";

  EXPECT_GT(fixed.reanchors, 0) << "nothing fired, so this test proves nothing";
  EXPECT_LT(fixed.err_30s, 0.5 * baseline.err_30s)
    << "re-anchoring must cut the error 30 s after fixes return; measured 0.571 m "
       "without it against 0.013 m with it";
  EXPECT_LT(fixed.err_300s, 2.0 * baseline.err_300s)
    << "and must not be WORSE once settled; both arms converge here, so this is a "
       "guard against regression and not a demonstration of benefit";
  EXPECT_GT(fixed.last_jump, 0.0)
    << "the jump must be reported, or a consumer cannot tell a correction from "
       "divergence and will chase it";
}

// Re-anchoring is bounded. A storm of them would be its own failure mode, and the
// cooldown is what prevents it: 16 events across five NCLT sequences and roughly
// 25,000 s of driving.
TEST(GnssReanchorTest, DoesNotStorm) {
  auto cfg = reanchor_config();
  cfg.gnss_reanchor_enabled = true;
  FusionCore fc(cfg);
  const auto o = run(fc, 460.0);
  EXPECT_GT(o.reanchors, 0);
  EXPECT_LE(o.reanchors, 10)
    << "re-anchored " << o.reanchors << " times in 300 s of recovery";
}

// A re-anchor moves POSITION ONLY. This is the evidence for that decision, kept so
// that adding a heading reset back has to be justified by a measurement rather than
// by the argument that it sounds right.
//
// There WAS a yaw reset, taking heading from the bearing between the first and last
// buffered fix. Measured 2026-10-18: disabling it moved NCLT 2012-09-28 from 13.750
// to 13.748, and position RMSE was unchanged to three decimals on all five
// sequences. Four of the five were bit-identical; on 2012-08-20 the state diverged,
// a quaternion component by 0.934 and position by up to 0.798 m, while the RMSE
// landed on 46.671 either way. So it was inert in the position metric, not in the
// state, and NCLT cannot judge the difference because its ground truth carries an
// exactly identity quaternion on every row. It was removed because nothing shows it
// helping, which is a weaker claim than proving it does nothing.
TEST(GnssReanchorTest, ReanchoringDoesNotTouchHeading) {
  auto off = reanchor_config();
  FusionCore fc_off(off);
  const auto baseline = run(fc_off, 460.0);

  auto on = reanchor_config();
  on.gnss_reanchor_enabled = true;
  FusionCore fc_on(on);
  const auto fixed = run(fc_on, 460.0);

  std::cerr << "  largest single-step yaw change: " << baseline.max_yaw_step
            << " rad without re-anchoring, " << fixed.max_yaw_step << " rad with, "
            << "over a " << fixed.last_jump << " m position jump\n";

  ASSERT_GT(fixed.reanchors, 0);
  // The jump is hundreds of metres. If any of it reached heading, the largest yaw
  // step would separate clearly from the no-re-anchor arm.
  EXPECT_LT(fixed.max_yaw_step, baseline.max_yaw_step + 1e-3)
    << "a re-anchor changed heading; it is supposed to move position only";
}
