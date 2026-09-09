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
// Holding the covariance down stops it GROWING, but the filter still counts N
// parked fixes as N independent looks at a fixed point and shrinks P like
// sqrt(N). GNSS error does not work that way over a minute, so the filter ends
// up more certain than the geometry supports and each fix keeps dragging it.
//
// Measured on the 2026-09-07 bag, parked 57 s, with the rover config:
// 8.73 m of drift with neither, 0.56 m with process noise alone, 0.10 m once
// the parked evidence is measured and applied.
TEST(IdleDriftTest, DistrustingGnssWhileParkedFinishesTheJob) {
  const double baseline = idle_drift(1.0);            // neither
  const double noise    = idle_drift(0.001);          // process noise only
  const double both     = idle_drift(0.001, 100.0);   // and measure the receiver

  EXPECT_LT(noise, baseline)
    << "suppressing position process noise should already help";
  EXPECT_LT(both, noise * 0.85)
    << "measuring the receiver while the wheels confirm stillness should go "
       "further still: baseline " << baseline << " m, process noise "
    << noise << " m, both " << both << " m";
}

// ─── Once the evidence is in, the estimate should stop moving ───────────────
// The whole-window number above is dominated by the first few seconds, before
// enough parked fixes exist to measure anything, and during those the filter is
// entitled to follow the receiver. What the mechanism actually promises is
// about the part after that, so measure that part on its own.
TEST(IdleDriftTest, EstimateStopsMovingOnceTheEvidenceIsIn) {
  auto travel_after = [](double zupt_gnss_scale) {
    FusionCore fc(idle_config(0.001, zupt_gnss_scale));
    State s0;
    fc.init(s0, 0.0);
    const double dt = 0.01, g = 9.80665;
    double px = 0.0, py = 0.0, path = 0.0;
    for (int step = 1; step * dt <= 60.0 + 1e-9; ++step) {
      const double t = step * dt;
      fc.update_imu(t, 0, 0, 0, 0, 0, g);
      if (step % 2 == 0) {
        fc.update_encoder(t, 0.0, 0.0, 0.0);
        fc.update_ground_constraint(t);
        fc.update_zupt(t, 0.01);
      }
      if (step % 100 == 0) fc.update_gnss(t, fix_at(wander(t), wander(t + 40.0)));
      if (t >= 20.0 && step % 100 == 0) {
        const double x = fc.get_state().x[X], y = fc.get_state().x[Y];
        if (path > 0.0 || (px != 0.0 || py != 0.0)) path += std::hypot(x - px, y - py);
        px = x; py = y;
      }
    }
    return path;
  };
  const double off = travel_after(1.0);
  const double on  = travel_after(100.0);
  EXPECT_LT(on, off * 0.25)
    << "a robot that has been parked for 20 s with the evidence already "
       "collected should have a nearly frozen estimate: it travelled "
    << on << " m with the measurement on and " << off << " m with it off";
}

// ─── An honest receiver must not be penalised ───────────────────────────────
// This is the property that separates a measurement from a policy. Feed the
// same parked robot a receiver whose errors are white and whose magnitude
// matches exactly what it declares, and nothing should happen to it, no matter
// how large the cap is set. If this test ever fails, the mechanism has become a
// tuned constant wearing a measurement's clothes.
TEST(IdleDriftTest, AnHonestReceiverEarnsNoInflation) {
  FusionCoreConfig cfg = idle_config(0.001, 1000.0);
  const double sigma = cfg.gnss.base_noise_xy;   // hdop is 1.0 below, so this
                                                 // is exactly the declared sigma
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  // Deterministic white-ish sequence: successive draws share no structure, so
  // the lag-1 autocorrelation is near zero and the spread is sigma by
  // construction. A real RNG would work too but would not be reproducible.
  auto white = [](int k) {
    const double u = std::fmod(std::sin(k * 12.9898) * 43758.5453, 1.0);
    return (u < 0.0 ? u + 1.0 : u) - 0.5;               // uniform on [-0.5, 0.5]
  };
  const double uniform_sigma = 1.0 / std::sqrt(12.0);

  const double dt = 0.01, g = 9.80665;
  int k = 0;
  for (int step = 1; step * dt <= 120.0 + 1e-9; ++step) {
    const double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      fc.update_encoder(t, 0.0, 0.0, 0.0);
      fc.update_ground_constraint(t);
      fc.update_zupt(t, 0.01);
    }
    if (step % 100 == 0) {
      const double s = sigma / uniform_sigma;
      fc.update_gnss(t, fix_at(white(k) * s, white(k + 5000) * s));
      ++k;
    }
  }

  const auto st = fc.get_status();
  ASSERT_GE(st.gnss_parked_sigma_observed, 0.0) << "no parked evidence collected";
  EXPECT_NEAR(st.gnss_parked_sigma_observed, st.gnss_parked_sigma_declared,
              st.gnss_parked_sigma_declared * 0.35)
    << "the synthetic receiver is not actually as noisy as it declares, so this "
       "test is not testing what it claims to";
  EXPECT_LT(st.gnss_parked_correlation, 0.35)
    << "the synthetic receiver is not actually white, correlation "
    << st.gnss_parked_correlation;
  EXPECT_LT(st.gnss_parked_inflation, 3.0)
    << "an honest receiver was inflated " << st.gnss_parked_inflation
    << "x despite a cap of 1000, so the mechanism is not measuring, it is "
       "applying a policy";
}

// ─── A correlated receiver must be caught even when its sigma is honest ─────
// The magnitude check alone would pass this receiver: its spread is exactly
// what it claims. But its errors barely change from one fix to the next, so a
// minute of them is worth a couple of independent looks, not sixty, and a
// filter that averages them as if they were independent over-converges.
TEST(IdleDriftTest, CorrelatedErrorIsCaughtEvenWhenTheSigmaIsHonest) {
  FusionCoreConfig cfg = idle_config(0.001, 1000.0);
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665;
  for (int step = 1; step * dt <= 120.0 + 1e-9; ++step) {
    const double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      fc.update_encoder(t, 0.0, 0.0, 0.0);
      fc.update_ground_constraint(t);
      fc.update_zupt(t, 0.01);
    }
    if (step % 100 == 0) {
      // A slow drift with the same standard deviation the receiver declares.
      const double a = cfg.gnss.base_noise_xy * std::sqrt(2.0);
      fc.update_gnss(t, fix_at(a * std::sin(0.05 * t), a * std::cos(0.05 * t)));
    }
  }

  const auto st = fc.get_status();
  ASSERT_GE(st.gnss_parked_sigma_observed, 0.0) << "no parked evidence collected";
  EXPECT_GT(st.gnss_parked_correlation, 0.9)
    << "this receiver is meant to be strongly correlated, measured "
    << st.gnss_parked_correlation;
  EXPECT_GT(st.gnss_parked_inflation, 10.0)
    << "a receiver whose consecutive errors are nearly identical was inflated "
       "only " << st.gnss_parked_inflation << "x, so the filter is still "
       "treating them as independent samples";
}

TEST(IdleDriftTest, GnssDistrustIsOffByDefault) {
  EXPECT_DOUBLE_EQ(FusionCoreConfig{}.zupt_gnss_noise_scale, 1.0);
  EXPECT_EQ(FusionCoreConfig{}.zupt_gnss_min_samples, 5);
}

