#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"
#include <cmath>

using namespace fusioncore;

// Shared setup: a differential-drive robot driving straight East at 1 m/s on
// flat ground, matching the Gazebo demo config (outlier rejection on, coast
// defaults). IMU at 100 Hz, encoder + ground constraint at 50 Hz, GNSS at 5 Hz.
// gps_offset_x lets a caller inject a sustained spike; gps_present lets a caller
// drop GNSS entirely for an outage window.
namespace {

FusionCoreConfig demo_config() {
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

sensors::GnssFix make_fix(double x, double y) {
  sensors::GnssFix fix;
  fix.x = x; fix.y = y; fix.z = 0.0;
  fix.hdop = 1.0; fix.vdop = 1.0;
  fix.satellites = 12;
  fix.fix_type = sensors::GnssFixType::DGPS_FIX;
  return fix;
}

} // namespace

// ─── A sustained GPS spike must stay rejected for its whole duration ─────────
// Regression for the coast-defeats-the-gate bug: an 8 s continuous +60 m spike
// used to slip past the chi2 gate after ~5 s because rejection-triggered coast
// inflated P until the gate opened (peak error ~62 m). With gap-gated coast a
// continuous spike never triggers coast, so the filter holds.
TEST(GnssCoastTest, SustainedSpikeStaysRejected) {
  FusionCore fc(demo_config());
  State s0;                       // identity quaternion, origin, facing East
  fc.init(s0, 0.0);

  const double dt = 0.01, speed = 1.0, g = 9.80665;
  double max_err_in_spike = 0.0;

  for (int step = 1; step * dt <= 45.0 + 1e-9; ++step) {
    double t = step * dt;
    double true_x = speed * t;

    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      fc.update_encoder(t, speed, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }

    bool in_spike = (t >= 30.0 && t < 38.0);
    if (step % 20 == 0) {                          // 5 Hz GNSS, continuous
      double gx = true_x + (in_spike ? 60.0 : 0.0);
      fc.update_gnss(t, make_fix(gx, 0.0));
    }

    if (in_spike) {
      double err = std::hypot(fc.get_state().x[X] - true_x, fc.get_state().x[Y]);
      max_err_in_spike = std::max(max_err_in_spike, err);
    }
  }

  // The spike is +60 m. A filter that swallows it lands ~60 m off. We require
  // the estimate to stay within a few meters of truth throughout the spike.
  EXPECT_LT(max_err_in_spike, 5.0)
      << "sustained spike defeated the gate: max error " << max_err_in_spike << " m";
}

// ─── A genuine outage must still re-acquire after GPS returns ────────────────
// The spike fix gates coast on a preceding GPS gap, so it must NOT break the
// case coast exists for: drift during a real blackout, then re-acquisition.
TEST(GnssCoastTest, OutageStillRecovers) {
  FusionCore fc(demo_config());
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, speed = 1.0, g = 9.80665;

  for (int step = 1; step * dt <= 55.0 + 1e-9; ++step) {
    double t = step * dt;
    double true_x = speed * t;

    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      fc.update_encoder(t, speed, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }

    bool in_outage = (t >= 20.0 && t < 45.0);      // 25 s blackout
    if (step % 20 == 0 && !in_outage) {
      fc.update_gnss(t, make_fix(true_x, 0.0));
    }
  }

  // 10 s after GPS returned, the filter should be back on truth.
  double final_err = std::hypot(fc.get_state().x[X] - 55.0, fc.get_state().x[Y]);
  EXPECT_LT(final_err, 2.0)
      << "filter failed to re-acquire after outage: final error " << final_err << " m";
}

// ─── Physical plausibility gate: an impossible jump after a gap is rejected ───
// After a GPS gap, a fix far beyond what the robot could have moved/drifted
// (max_speed * dt) must be rejected even though a coast-relaxed chi2 gate would
// admit it. This is the #64 adversarial-cluster-at-blackout-boundary case.
TEST(GnssCoastTest, ImplausibleJumpAfterGapRejected) {
  FusionCoreConfig cfg = demo_config();
  cfg.gnss_max_speed = 3.0;  // platform max ~3 m/s
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);
  const double dt = 0.01, speed = 1.0, g = 9.80665;

  // Drive with good GPS to t = 10 s.
  for (int step = 1; step * dt <= 10.0 + 1e-9; ++step) {
    double t = step * dt, tx = speed * t;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) { fc.update_encoder(t, speed, 0.0, 0.0); fc.update_ground_constraint(t); }
    if (step % 20 == 0) fc.update_gnss(t, make_fix(tx, 0.0));
  }
  // 10 s outage (keep driving, no GPS).
  for (int step = 1001; step * dt <= 20.0 + 1e-9; ++step) {
    double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) { fc.update_encoder(t, speed, 0.0, 0.0); fc.update_ground_constraint(t); }
  }
  // Outlier 700 m east of truth. Bound is 3 m/s * 10 s + 5 m = 35 m, so reject.
  double t = 20.0, tx = speed * t;
  bool accepted = fc.update_gnss(t, make_fix(tx + 700.0, 0.0));
  EXPECT_FALSE(accepted);
  EXPECT_EQ(fc.get_gnss_debug().reason, GnssRejectionReason::IMPLAUSIBLE_JUMP);
  EXPECT_LT(fc.get_state().x[X], tx + 50.0)  // must not have lurched toward the outlier
      << "filter jumped to the implausible fix";
}

// The gate must NOT block a legitimate recovery fix after the same outage.
TEST(GnssCoastTest, LegitRecoveryAfterGapAccepted) {
  FusionCoreConfig cfg = demo_config();
  cfg.gnss_max_speed = 3.0;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);
  const double dt = 0.01, speed = 1.0, g = 9.80665;

  for (int step = 1; step * dt <= 10.0 + 1e-9; ++step) {
    double t = step * dt, tx = speed * t;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) { fc.update_encoder(t, speed, 0.0, 0.0); fc.update_ground_constraint(t); }
    if (step % 20 == 0) fc.update_gnss(t, make_fix(tx, 0.0));
  }
  for (int step = 1001; step * dt <= 15.0 + 1e-9; ++step) {  // 5 s outage
    double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) { fc.update_encoder(t, speed, 0.0, 0.0); fc.update_ground_constraint(t); }
  }
  // Fix at the true position (small innovation = the drift). Bound 3*5+5 = 20 m.
  double t = 15.0, tx = speed * t;
  bool accepted = fc.update_gnss(t, make_fix(tx, 0.0));
  EXPECT_TRUE(accepted) << "legitimate recovery fix was wrongly rejected";
}


// ─── The jump gate must not sit inside the receiver's own noise ──────────────
//
// Measured 2026-08-03 on a u-blox M9N at 1 Hz with gnss.max_speed = 2.0: the
// bound worked out to 2*1 + 5 = 7 m while the receiver's reported sigma was ~6 m,
// so ordinary noise routinely exceeded it. 157 of 500 good fixes were rejected
// and the loop closure went from 2.62 m to 7.27 m. The bound now adds
// max_speed_sigma_k multiples of the RECEIVER's reported sigma.

namespace {
// Same as make_fix but reporting a realistic standalone-GPS uncertainty, which
// is what puts the fix on the sigma-aware branch of the gate.
sensors::GnssFix make_noisy_fix(double x, double y, double sigma) {
  sensors::GnssFix fix = make_fix(x, y);
  fix.hdop = sigma;          // metres, as the ROS node derives from covariance
  fix.vdop = sigma;
  fix.sigma_xy = sigma;
  fix.sigma_z  = sigma;
  return fix;
}

// Drive east at 1 m/s to `until_s`, feeding GPS at 1 Hz with the given sigma.
void drive_with_gps(FusionCore& fc, double until_s, double sigma) {
  const double dt = 0.01, speed = 1.0, g = 9.80665;
  for (int step = 1; step * dt <= until_s + 1e-9; ++step) {
    double t = step * dt, tx = speed * t;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) { fc.update_encoder(t, speed, 0.0, 0.0); fc.update_ground_constraint(t); }
    if (step % 100 == 0) fc.update_gnss(t, make_noisy_fix(tx, 0.0, sigma));
  }
}
} // namespace

TEST(GnssCoastTest, JumpGateToleratesOrdinaryReceiverNoise) {
  FusionCoreConfig cfg = demo_config();
  cfg.gnss_max_speed = 2.0;      // the rover's setting
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  const double sigma = 6.0;      // what the M9N actually reports
  drive_with_gps(fc, 10.0, sigma);

  // A fix 15 m off truth: 2.5 sigma for this receiver, i.e. ordinary noise.
  // Old bound was 2*1 + 5 = 7 m, so this was rejected. New bound is
  // 2*1 + 5 + 5*6 = 37 m, so it must be fused.
  bool accepted = fc.update_gnss(11.0, make_noisy_fix(11.0 + 15.0, 0.0, sigma));
  EXPECT_TRUE(accepted)
      << "the jump gate rejected ordinary noise from a 6 m-sigma receiver; "
      << "reason=" << static_cast<int>(fc.get_gnss_debug().reason);
}

TEST(GnssCoastTest, JumpGateStillCatchesRealSpikeOnNoisyReceiver) {
  // The noise term must not become a loophole: widening the bound for a noisy
  // receiver still has to leave a genuine multipath spike outside it.
  FusionCoreConfig cfg = demo_config();
  cfg.gnss_max_speed = 2.0;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  const double sigma = 6.0;
  drive_with_gps(fc, 10.0, sigma);

  // 700 m east. Bound is 37 m even with the noise term, so this is impossible.
  bool accepted = fc.update_gnss(11.0, make_noisy_fix(11.0 + 700.0, 0.0, sigma));
  EXPECT_FALSE(accepted);
  EXPECT_EQ(fc.get_gnss_debug().reason, GnssRejectionReason::IMPLAUSIBLE_JUMP);
  EXPECT_LT(fc.get_state().x[X], 11.0 + 50.0)
      << "filter lurched toward the spike";
}

TEST(GnssCoastTest, JumpGateNoiseTermIsIndependentOfFilterCovariance) {
  // The gate exists to catch what a coast-relaxed chi2 admits, so its bound must
  // NOT grow with P. Same fix, same gap, but one filter has been coasting and has
  // a much larger P: the verdict must be identical.
  auto verdict = [](double outage_s) {
    FusionCoreConfig cfg = demo_config();
    cfg.gnss_max_speed = 2.0;
    FusionCore fc(cfg);
    State s0; fc.init(s0, 0.0);
    const double dt = 0.01, speed = 1.0, g = 9.80665;
    // 5 s of drive-up, not 10: five GPS fixes is plenty to converge and to set
    // last_gnss_time_, and this helper runs twice so the saving is doubled.
    drive_with_gps(fc, 5.0, 6.0);
    for (int step = 501; step * dt <= 5.0 + outage_s + 1e-9; ++step) {
      double t = step * dt;
      fc.update_imu(t, 0, 0, 0, 0, 0, g);
      if (step % 2 == 0) { fc.update_encoder(t, speed, 0.0, 0.0); fc.update_ground_constraint(t); }
    }
    // 700 m off truth. Physics says impossible regardless of how lost we feel.
    double t = 5.0 + outage_s, tx = speed * t;
    fc.update_gnss(t, make_noisy_fix(tx + 700.0, 0.0, 6.0));
    return fc.get_gnss_debug().reason;
  };
  EXPECT_EQ(verdict(1.0),  GnssRejectionReason::IMPLAUSIBLE_JUMP);
  EXPECT_EQ(verdict(10.0), GnssRejectionReason::IMPLAUSIBLE_JUMP)
      << "a long coast inflated P enough to let a 700 m outlier through, which is "
         "exactly the hole this gate exists to plug";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
