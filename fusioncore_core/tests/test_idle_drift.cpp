#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"
#include <cmath>

using namespace fusioncore;

// A parked robot's estimate should not follow a wandering receiver.
//
// Martin Pecka described this on ROS Discourse as the property that matters for
// GNSS integration: when the robot is idle and the motion model says zero
// motion, the pose must not drift toward the GNSS mean. FusionCore failed it.
// Measured on the 2026-09-07 field bags, parked for 57 s with wheel encoders
// confirming stillness, the receiver's reported position moved 9.76 m and the
// fused position followed it for 10.16 m.
//
// The cause is that ZUPT fuses [VX=0, VY=0, WZ=0], which pins VELOCITY and says
// nothing about position. Position process noise kept growing between fixes, so
// the Kalman gain stayed high and every fix dragged the estimate.
namespace {

FusionCoreConfig idle_config(double zupt_pos_scale, double zupt_gnss_scale = 1.0) {
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.gnss.base_noise_xy = 1.0;
  cfg.gnss.base_noise_z  = 1.0;
  cfg.outlier_rejection = true;
  cfg.zupt_position_noise_scale = zupt_pos_scale;
  cfg.zupt_gnss_noise_scale     = zupt_gnss_scale;
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

// Deterministic multipath-shaped wander: a few metres, slowly, with no net
// drift. Incommensurate sinusoids rather than a ramp, because a ramp is
// indistinguishable from real motion and any filter is entitled to follow it.
// What a parked robot must not do is chase a signal that keeps returning.
double wander(double t) {
  return 2.0 * std::sin(0.07 * t)
       + 1.2 * std::sin(0.19 * t + 1.0)
       + 0.9 * std::sin(0.31 * t + 2.0);
}

// Hold a stationary robot for 60 s while the receiver wanders around it. The
// robot never moves, so its true position stays at the origin and the worst
// excursion of the estimate IS the error. Returns that excursion in metres.
double idle_drift(double zupt_pos_scale, double zupt_gnss_scale = 1.0) {
  FusionCore fc(idle_config(zupt_pos_scale, zupt_gnss_scale));
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665;
  double worst = 0.0;

  for (int step = 1; step * dt <= 60.0 + 1e-9; ++step) {
    const double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);      // perfectly still

    if (step % 2 == 0) {                     // 50 Hz encoder: zero velocity
      fc.update_encoder(t, 0.0, 0.0, 0.0);
      fc.update_ground_constraint(t);
      fc.update_zupt(t, 0.01);               // the wheels assert stillness
    }
    if (step % 100 == 0) {                   // 1 Hz GNSS, wandering
      fc.update_gnss(t, fix_at(wander(t), wander(t + 40.0)));
    }
    if (t >= 5.0) {                          // skip initial convergence
      worst = std::max(worst,
        std::hypot(fc.get_state().x[X], fc.get_state().x[Y]));
    }
  }
  return worst;
}

} // namespace

// ─── A parked robot must not chase a drifting receiver ───────────────────────
TEST(IdleDriftTest, SuppressedPositionNoiseHoldsAParkedRobot) {
  const double baseline = idle_drift(1.0);     // previous behaviour
  const double held     = idle_drift(0.001);   // noise suppressed while still

  EXPECT_GT(baseline, 1.5) << "test is not exercising the drift at all";
  EXPECT_LT(held, baseline * 0.7)
    << "suppressing position process noise under ZUPT did not reduce idle drift: "
    << "baseline " << baseline << " m, held " << held << " m";
}

// ─── The default must not change anyone's filter ────────────────────────────
TEST(IdleDriftTest, DefaultIsUnchangedBehaviour) {
  EXPECT_DOUBLE_EQ(FusionCoreConfig{}.zupt_position_noise_scale, 1.0);
  EXPECT_DOUBLE_EQ(idle_drift(1.0), idle_drift(1.0)) << "not deterministic";
}

// ─── Moving again must hand the noise scale back ────────────────────────────
// Otherwise a robot that parks once stays over-confident for the rest of the
// run, which would be a far worse bug than the one being fixed.
TEST(IdleDriftTest, MotionRestoresTheNoiseScale) {
  FusionCore fc(idle_config(0.001));
  State s0;
  fc.init(s0, 0.0);
  const double dt = 0.01, g = 9.80665;

  for (int step = 1; step * dt <= 10.0 + 1e-9; ++step) {   // parked
    const double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) { fc.update_encoder(t, 0.0, 0.0, 0.0); fc.update_zupt(t, 0.01); }
  }
  const double p_parked = fc.get_state().P(X, X);

  for (int step = 1001; step * dt <= 25.0 + 1e-9; ++step) { // driving
    const double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) fc.update_encoder(t, 1.0, 0.0, 0.0);
  }
  const double p_moving = fc.get_state().P(X, X);

  EXPECT_GT(p_moving, p_parked)
    << "position covariance did not grow again after the robot started moving, "
    << "so the ZUPT noise suppression was never handed back";
}

// ─── Suppressing process noise alone cannot finish the job ──────────────────
// Holding the covariance down stops it GROWING, but the filter still weighs
// each fix by the covariance the receiver claims, and a receiver that wanders
// 9.76 m while declaring 3.6 m of accuracy is wrong by more than it admits.
// While the wheels say stationary, every fix measures the SAME point, so their
// spread is itself evidence that the claim is not credible.
//
// Measured on the 2026-09-07 bag, parked 57 s: 10.16 m of drift with neither,
// 3.23 m with process noise alone, 0.10 m with both.
TEST(IdleDriftTest, DistrustingGnssWhileParkedFinishesTheJob) {
  const double baseline = idle_drift(1.0);            // neither
  const double noise    = idle_drift(0.001);          // process noise only
  const double both     = idle_drift(0.001, 100.0);   // and distrust GNSS

  EXPECT_LT(noise, baseline)
    << "suppressing position process noise should already help";
  EXPECT_LT(both, noise * 0.5)
    << "distrusting GNSS while the wheels confirm stillness should go "
       "substantially further: baseline " << baseline << " m, process noise "
    << noise << " m, both " << both << " m";
}

TEST(IdleDriftTest, GnssDistrustIsOffByDefault) {
  EXPECT_DOUBLE_EQ(FusionCoreConfig{}.zupt_gnss_noise_scale, 1.0);
}
