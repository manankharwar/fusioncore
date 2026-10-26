// The sigma-point rotation bound, and relaxing it as GNSS goes silent.
//
// Context, because the numbers below are only meaningful against it. Bounding how far
// a sigma point's attitude may rotate from the mean fixes the dead-reckoning
// cancellation outright: the synthetic repro goes from 80.7% of a perfect velocity to
// 98.6% at 45 degrees. Measured on NCLT on 2026-10-10 it is a TRADEOFF, helping
// 2013-04-05 by 7.9% and 2012-08-20 by 7.2% while harming 2012-06-15 by 15.0%, and
// 2012-06-15 is the sequence with the longest single GPS gap at 416 s.
//
// The reading is that the bound's benefit is continuous while its cost only arrives
// once GNSS has been gone long enough that the filter needs a wide attitude prior to
// accept the returning fix. max_sigma_rotation_relax_s separates them in time.
//
// These tests assert the MECHANISM, not the NCLT numbers: that the bound binds while
// fixes arrive and stops binding after sustained silence.
#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include <cmath>

using namespace fusioncore;

namespace {

sensors::GnssFix make_fix(double x, double y) {
  sensors::GnssFix fix;
  fix.x = x; fix.y = y; fix.z = 0.0;
  fix.hdop = 1.0; fix.vdop = 1.0;
  fix.satellites = 12;
  fix.fix_type = sensors::GnssFixType::DGPS_FIX;
  return fix;
}

// Dead reckoning with a perfect encoder, after an optional period of GNSS fixes.
// Returns x at the end. Truth is 1.0 m/s straight, so truth x = total seconds.
double run(double max_rot_deg, double relax_s, double gnss_s, double blackout_s) {
  const double DT = 0.01, G = 9.80665, SPEED = 1.0;
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.outlier_rejection = true;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.ukf.max_sigma_rotation_deg    = max_rot_deg;
  cfg.ukf.max_sigma_rotation_relax_s = relax_s;

  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  const int n = static_cast<int>((gnss_s + blackout_s) / DT);
  for (int step = 1; step <= n; ++step) {
    const double t = step * DT;
    fc.update_imu(t, 0, 0, 0, 0, 0, G);
    if (step % 2 == 0) {
      fc.update_encoder(t, SPEED, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }
    // 1 Hz GNSS on the truth, then silence.
    if (t <= gnss_s && step % 100 == 0) {
      fc.update_gnss(t, make_fix(SPEED * t, 0.0));
    }
  }
  return fc.get_state().x[X];
}

}  // namespace

// The bound has to actually do something, or everything below is vacuous.
TEST(SigmaRotationRelax, TheBoundBindsDuringPureDeadReckoning)
{
  const double unbounded = run(0.0,  0.0, 0.0, 60.0);
  const double bounded   = run(45.0, 0.0, 0.0, 60.0);
  EXPECT_LT(unbounded, 52.0) << "unbounded should fall well short of the 60 m truth";
  EXPECT_GT(bounded,   57.0) << "the bound should recover most of the deficit";
  EXPECT_GT(bounded, unbounded);
}

// relax_s = 0 is the shipped behaviour and must be bit-identical to no relaxation.
TEST(SigmaRotationRelax, ZeroRelaxIsExactlyTheUnrelaxedBound)
{
  EXPECT_DOUBLE_EQ(run(45.0, 0.0, 10.0, 120.0), run(45.0, 0.0, 10.0, 120.0));
  // And a relax_s so large it cannot widen the bound within the run is the same thing.
  EXPECT_NEAR(run(45.0, 0.0, 10.0, 60.0), run(45.0, 1.0e9, 10.0, 60.0), 1e-9);
}

// The mechanism: after sustained silence the bound stops binding, so the filter should
// behave like the unbounded one rather than like the permanently bounded one.
TEST(SigmaRotationRelax, SustainedSilenceRelaxesTheBoundTowardUnbounded)
{
  const double gnss_s = 20.0, blackout_s = 300.0;
  const double unbounded = run(0.0,  0.0,  gnss_s, blackout_s);
  const double held      = run(45.0, 0.0,  gnss_s, blackout_s);
  const double relaxed   = run(45.0, 50.0, gnss_s, blackout_s);

  // 45 deg widens past 180 (inert) after relax_s * (180/45 - 1) = 150 s of silence,
  // so half this blackout runs effectively unbounded.
  const double d_relaxed   = std::fabs(relaxed - unbounded);
  const double d_held      = std::fabs(held    - unbounded);
  EXPECT_LT(d_relaxed, d_held)
      << "relaxed=" << relaxed << " held=" << held << " unbounded=" << unbounded
      << ": relaxing should move behaviour back toward the unbounded filter";
}

// And the converse, which is the half that makes it a tradeoff rather than a toggle:
// while fixes are still arriving the bound must still bind.
TEST(SigmaRotationRelax, TheBoundStillBindsWhileFixesAreArriving)
{
  const double gnss_s = 60.0, blackout_s = 0.0;
  const double unbounded = run(0.0,  0.0,  gnss_s, blackout_s);
  const double relaxed   = run(45.0, 50.0, gnss_s, blackout_s);
  // No silence means scale 1.0 throughout, so this must equal the held bound exactly.
  EXPECT_DOUBLE_EQ(relaxed, run(45.0, 0.0, gnss_s, blackout_s));
  (void)unbounded;
}
