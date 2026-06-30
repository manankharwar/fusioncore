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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
