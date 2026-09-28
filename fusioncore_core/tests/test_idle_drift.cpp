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
// 8.73 m of drift with neither, 0.53 m with process noise alone, 0.10 m once
// the parked evidence is measured and applied. Across all 11 parked windows in
// six runs the means are 3.49 m, 1.64 m and 0.74 m.
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


// ─── Dead wheel odometry must not freeze the estimate ───────────────────────
//
// Encoders that lose power report ZERO, not silence, which is indistinguishable
// from a parked robot. ZUPT then fires, and once zupt.position_noise_scale and
// zupt.gnss_noise_scale are enabled the filter holds its covariance down AND
// distrusts GNSS by the cap, so the robot drives away while the estimate sits
// still, actively ignoring the GPS telling it otherwise. On this rover all four
// encoders share one breadboard power rail, so that is one loose wire away.
//
// The discriminator cannot be displacement: a genuinely parked receiver wandered
// 12.85 m over 57 s on the 2026-09-07 log. It is STRAIGHTNESS, displacement over
// path length. That window measured 0.37. A driving robot approaches 1.0.
TEST(IdleDriftTest, DeadEncodersDoNotFreezeTheEstimate) {
  FusionCore fc(idle_config(0.001, 100.0));
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665;
  double t = 0.0;
  // The robot really is driving east at 0.5 m/s, but every encoder reads zero.
  // It also SHAKES, because a robot rolling over ground does. Measured on six
  // 2026-09 rover logs the accel-magnitude std is 1.69 to 2.25 m/s^2 driving
  // against 0.013 to 0.021 parked, and the detector now requires that evidence
  // before calling the wheels liars. A perfectly still accelerometer on a
  // moving robot is not a robot this test should be describing.
  for (int step = 1; step * dt <= 40.0 + 1e-9; ++step) {
    t = step * dt;
    // On the gravity axis. The published figure is the std of accelerometer
    // MAGNITUDE, and a 2 m/s^2 wobble sideways of a 9.81 vector barely changes
    // its length, so shaking in y would not describe a robot that is rolling.
    const double shake = 2.0 * std::sin(37.0 * t);
    fc.update_imu(t, 0, 0, 0, 0, 0, g + shake);
    if (step % 2 == 0) {
      fc.update_encoder(t, 0.0, 0.0, 0.0);     // the lie
      fc.update_zupt(t, 0.01);                 // which ZUPT believes
    }
    if (step % 100 == 0) fc.update_gnss(t, fix_at(0.5 * t, 0.0));  // the truth
  }

  const auto st = fc.get_status();
  EXPECT_TRUE(st.zupt_parked_but_moving)
    << "the filter never noticed its wheel odometry had died. straightness was "
    << st.zupt_parked_straightness;
  EXPECT_GT(fc.get_state().x[X], 8.0)
    << "the estimate stayed put while the robot drove 20 m. It reached only "
    << fc.get_state().x[X] << " m, which is the failure this check exists to stop.";
}

// ─── The indoor false positive, 2026-09-15 ─────────────────────────────────
// Indoors the receiver reported sigma_xy 14 m, so two consecutive fixes sat 6 m
// apart. That is one segment, and over one segment displacement and path length
// are the same line, so straightness is exactly 1.00 no matter what the receiver
// did. The check fired 2.8 s after the first fix of the run, announced that the
// wheel odometry was lying, and disabled ZUPT and the parked-GNSS suppression
// for the rest of the run. The robot was sitting on a table.
TEST(IdleDriftTest, TwoFixesAreNotEvidenceOfAnything) {
  FusionCore fc(idle_config(0.001, 100.0));
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665;
  int fix_n = 0;
  for (int step = 1; step * dt <= 12.0 + 1e-9; ++step) {
    const double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);   // dead still, it is on a table
    if (step % 2 == 0) {
      fc.update_encoder(t, 0.0, 0.0, 0.0);
      fc.update_zupt(t, 0.01);
    }
    // 1 Hz, and each fix lands 6 m further out: the worst case for this check,
    // a receiver whose error walks one way under bad indoor geometry.
    if (step % 100 == 0) fc.update_gnss(t, fix_at(6.0 * (++fix_n), 0.0));
  }

  const auto st = fc.get_status();
  EXPECT_FALSE(st.zupt_parked_but_moving)
    << "a stationary robot was told its wheel odometry was lying, on "
    << fix_n << " fixes at straightness " << st.zupt_parked_straightness
    << ". Straightness is 1.00 by construction over one segment.";
}

// The same shape, but the accelerometer is what saves it. Ten-plus segments of
// genuinely straight drift satisfy every geometric test the check has, so
// geometry alone would fire. The robot is still sitting on a table.
TEST(IdleDriftTest, StraightDriftOnAStillRobotIsNotDeadEncoders) {
  FusionCore fc(idle_config(0.001, 100.0));
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665;
  int fix_n = 0;
  for (int step = 1; step * dt <= 40.0 + 1e-9; ++step) {
    const double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      fc.update_encoder(t, 0.0, 0.0, 0.0);
      fc.update_zupt(t, 0.01);
    }
    if (step % 100 == 0) fc.update_gnss(t, fix_at(1.5 * (++fix_n), 0.0));
  }

  const auto st = fc.get_status();
  EXPECT_GE(st.zupt_parked_straightness, 0.85)
    << "the scenario is meant to be geometrically indistinguishable from real "
       "motion; if it is not, it is not testing what it claims to";
  EXPECT_FALSE(st.zupt_parked_but_moving)
    << "geometry alone fired on a robot whose accelerometer read dead flat";
}

// ─── And it must NOT fire on a robot that is genuinely parked ───────────────
// The bar that matters. A parked receiver wandering several metres must not be
// mistaken for a dead encoder, or the check would disable the idle-drift fix on
// exactly the runs it was built for.
TEST(IdleDriftTest, GenuineWanderDoesNotTripTheDeadEncoderCheck) {
  FusionCore fc(idle_config(0.001, 100.0));
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665;
  for (int step = 1; step * dt <= 60.0 + 1e-9; ++step) {
    const double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      fc.update_encoder(t, 0.0, 0.0, 0.0);
      fc.update_ground_constraint(t);
      fc.update_zupt(t, 0.01);
    }
    // Genuinely parked. The receiver wanders and returns, as a real one does.
    if (step % 100 == 0) fc.update_gnss(t, fix_at(wander(t), wander(t + 40.0)));
  }

  const auto st = fc.get_status();
  EXPECT_FALSE(st.zupt_parked_but_moving)
    << "wander was mistaken for motion at straightness "
    << st.zupt_parked_straightness << ", which would disable the idle-drift fix "
       "on exactly the runs it exists for";
  // 1.54 m is what this scenario produces with the suppression working (see
  // EstimateStopsMovingOnceTheEvidenceIsIn), against 3.36 m with it off. The bar
  // is that the suppression is still ACTIVE, not that drift is zero.
  EXPECT_LT(std::hypot(fc.get_state().x[X], fc.get_state().x[Y]), 2.0)
    << "the parked robot drifted as if the suppression had been released";
}

// The wheels saying stopped is not evidence the robot is stopped.
//
// Raised by Martin Pecka on ROS Discourse, and it had already cost a run here:
// an encoder died mid-drive and kept reporting zero, so the filter concluded
// parked, suppressed position noise, and then refused to let GNSS move the
// estimate. It recovered 7 m of the 20 m actually driven.
//
// The accelerometer cannot be fooled that way. Vibration levels below are the
// ones measured over 1 s windows on six 2026-09 rover logs: 0.013 to 0.021 m/s^2
// stationary, 1.69 to 2.25 m/s^2 driving.
namespace {

// Deterministic pseudo-vibration, so the test does not depend on a RNG.
double wobble(int k) {
  return std::sin(k * 2.399963) * std::cos(k * 0.7853981) +
         0.5 * std::sin(k * 1.1071487);
}

// Feeds IMU at 100 Hz with the given vibration level while the WHEELS insist the
// robot is stationary, then reports whether ZUPT was allowed to fire.
bool zupt_fired_with_vibration(double accel_std_target, double threshold = -1.0) {
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  if (threshold >= 0.0) cfg.zupt_accel_std_threshold = threshold;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665;
  bool fired = false;
  for (int step = 1; step <= 400; ++step) {
    const double t = step * dt;
    const double v = accel_std_target * wobble(step);
    fc.update_imu(t, 0, 0, 0, v, v * 0.7, g + v);
    if (step % 2 == 0) {
      // The lie: wheels report a dead stop throughout.
      fc.update_encoder(t, 0.0, 0.0, 0.0);
      const auto before = fc.get_state().x[VX];
      fc.update_zupt(t, 0.01);
      (void)before;
      if (!fc.get_status().zupt_blocked_by_imu && step > 250) fired = true;
    }
  }
  return fired;
}

} // namespace

TEST(IdleDriftTest, ZuptRefusedWhenTheAccelerometerSaysMoving) {
  // 2.0 m/s^2 is the measured driving regime. The wheels claim stopped, so
  // without the IMU check ZUPT would fire and pin the filter.
  EXPECT_FALSE(zupt_fired_with_vibration(2.0))
      << "ZUPT fired on a dead encoder while the accelerometer showed "
         "driving-level vibration";
}

TEST(IdleDriftTest, ZuptStillFiresWhenGenuinelyParked) {
  // 0.02 m/s^2 is the measured stationary regime. ZUPT must be unaffected,
  // otherwise this check costs the idle drift suppression it sits next to.
  EXPECT_TRUE(zupt_fired_with_vibration(0.02))
      << "the IMU check blocked ZUPT on a genuinely parked robot";
}

// Proves the guard is what does the work, rather than the scenario simply never
// firing ZUPT for some other reason. Same driving-level vibration, check
// disabled: ZUPT must fire, which is exactly the old behaviour that lost 13 m.
TEST(IdleDriftTest, WithoutTheImuCheckTheDeadEncoderIsBelieved) {
  EXPECT_TRUE(zupt_fired_with_vibration(2.0, /*threshold=*/0.0))
      << "with the check disabled ZUPT should still fire on the lying wheels, "
         "otherwise the two tests above prove nothing";
}

// ─── #145: the parked inflation must stand down for a BIASED filter ──────────
//
// The inflation cannot tell a receiver repeating a correlated error from a filter that
// is simply wrong, and it refuses the correction for both. Measured on
// fc_field_20260926_1755: the filter sat 8.20 m from a receiver scattering 2.8 m,
// accepted all 150 parked fixes, and moved 0.279 m in total. Effective gain 0.0002.

namespace {

// Park the filter, feed it fixes offset by `offset_m` from where it thinks it is, and
// report how far it actually moved toward them.
double parked_pull(double offset_m, double bias_ratio)
{
  FusionCoreConfig cfg = idle_config(0.001, 100.0);
  cfg.zupt_gnss_bias_ratio = bias_ratio;
  // Match the FIELD geometry, or the scenario tests a different mechanism. On
  // fc_field_20260926_1755 the offset was 8.20 m against a receiver reporting 2.8 m, so
  // the chi2 distance was about 2 sigma and every fix was ACCEPTED. With the harness
  // default of 1.0 m noise the same offset is a gross outlier: it fails the gate, arms
  // coast, inflates P and then jumps, which is not the defect under test.
  cfg.gnss.base_noise_xy = 2.8;
  cfg.gnss.base_noise_z  = 2.8;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665;
  double t = 0.0;
  // Settle a parked window first so the scatter statistics exist.
  for (int i = 0; i < 6000; ++i) {
    t += dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (i % 2 == 0) fc.update_encoder(t, 0.0, 0.0, 0.0);
    if (i % 100 == 0) fc.update_gnss(t, fix_at(wander(t), wander(t + 7.0)));
  }
  const double x_before = fc.get_state().x[X];
  // Now every fix sits offset_m away, which is what a biased filter sees.
  for (int i = 0; i < 20000; ++i) {
    t += dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (i % 2 == 0) fc.update_encoder(t, 0.0, 0.0, 0.0);
    if (i % 100 == 0) fc.update_gnss(t, fix_at(offset_m + wander(t), wander(t + 7.0)));
  }
  return std::abs(fc.get_state().x[X] - x_before);
}

}  // namespace

TEST(IdleDrift, BiasRatioDefaultIsTheMeasuredValue) {
  // Swept on the rover bags: 5.0 keeps ALL the idle-drift benefit (1810 excursion 0.36 m
  // against 0.37 m with the check off) while halving the closure cost (1755 3.80 -> 1.83).
  // At 3.0 closure is better still at 1.18 m but idle drift degrades to 1.59 m, which
  // gives away the thing the inflation exists for.
  EXPECT_DOUBLE_EQ(FusionCoreConfig{}.zupt_gnss_bias_ratio, 5.0);
}

// NOT TESTED HERE, deliberately: the full freeze.
//
// On fc_field_20260926_1755 the effective per-fix gain was 0.0002 and the filter moved
// 0.279 m across 150 fixes showing it 8.20 m of error. This synthetic reaches a gain
// around 0.01 and converges 7.88 m over 200 fixes even with the inflation at 100, so it
// does NOT reproduce the freeze and an assertion here would be testing something else.
//
// What makes the field case 50x stiffer is not captured by a smooth synthetic wander:
// the real receiver's lag-1 correlation saturates the cap for far longer while the held
// covariance stays small. The freeze is covered against the REAL bag instead, by
// hardware/replay.cpp on fc_field_20260926_1755, where closure goes 3.80 m to 1.83 m
// with only zupt.gnss_bias_ratio changed. Do not add a synthetic assertion for it
// without first reproducing the 0.0002 gain, or it will pass for the wrong reason.

TEST(IdleDrift, BiasCheckLetsAGrossOffsetCorrect) {
  const double moved = parked_pull(8.2, 5.0);
  EXPECT_GT(moved, 5.0)
      << "the bias check should let an 8.2 m offset pull the estimate in, it moved " << moved;
}

TEST(IdleDrift, BiasCheckDoesNotFireOnOrdinaryParkedWander) {
  // The other half. A receiver wandering within its own scatter must STILL be
  // suppressed, or the check has simply disabled the feature by another route.
  const double moved = parked_pull(0.0, 5.0);
  EXPECT_LT(moved, 2.0)
      << "ordinary parked wander must stay suppressed, it moved " << moved;
}
