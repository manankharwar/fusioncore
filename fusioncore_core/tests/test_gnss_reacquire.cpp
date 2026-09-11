#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"

using namespace fusioncore;

// Re-acquisition after a long GNSS blackout.
//
// Measured on NCLT 2012-06-15 (issue #63), aligning the trajectories on the
// pre-blackout segment so a late divergence cannot drag the fit:
//
//   before the 461 s blackout   FusionCore median 4.74 m,  robot_localization 3.65 m
//   at the end of the blackout  FusionCore        397.7 m, robot_localization 72.8 m
//   30 s after fixes return     FusionCore        224.1 m, robot_localization 17.4 m
//   300 s after fixes return    FusionCore        277.2 m, robot_localization 14.2 m
//
// robot_localization snaps back within 30 s. FusionCore never does: it keeps
// diverging with 5 Hz GNSS available for another 150 s. Drifting more than RL
// during the blackout is a known consequence of the 3D model; not coming back
// afterwards is a separate defect, and it is the one that matters to a robot
// that drives under a bridge.
//
// The mechanism: after minutes of dead reckoning the filter's position error is
// far larger than its own P says, so every returning fix looks like a gross
// outlier to the chi2 gate and is rejected. Rejections never end, so the filter
// never sees the data that would fix it.
namespace {

FusionCoreConfig blackout_config() {
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

struct Recovery {
  double err_at_return   = 0.0;   // error the moment fixes come back
  double err_5s          = 0.0;
  double err_30s         = 0.0;
  double err_300s        = 0.0;
  int    accepted_after  = 0;     // fixes accepted after the blackout
  int    rejected_after  = 0;
  int    reason_counts[GNSS_REJECTION_REASON_COUNT] = {0};
};

// Truth drives straight East at TRUE_SPEED throughout. During the blackout the
// wheels over-report (slip on a loose surface), so the filter runs ahead of the
// robot and is several hundred metres out by the time fixes return, which is
// the NCLT condition without needing NCLT.
Recovery run_blackout(FusionCore& fc, double p_inflate_sigma_unused = 0.0) {
  (void)p_inflate_sigma_unused;
  const double dt = 0.01, g = 9.80665;
  const double TRUE_SPEED = 1.5, SLIP_SPEED = 2.1;
  const double T_PRE = 120.0, T_BLACKOUT = 460.0, T_POST = 300.0;
  const double t_out_start = T_PRE, t_out_end = T_PRE + T_BLACKOUT;
  const double t_end = t_out_end + T_POST;

  Recovery r;
  double true_x = 0.0;
  bool   measured_return = false;

  for (int step = 1; step * dt <= t_end + 1e-9; ++step) {
    const double t = step * dt;
    true_x += TRUE_SPEED * dt;

    const bool blackout = (t >= t_out_start && t < t_out_end);
    const double enc_speed = blackout ? SLIP_SPEED : TRUE_SPEED;

    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      fc.update_encoder(t, enc_speed, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }

    if (step % 20 == 0 && !blackout) {          // 5 Hz GNSS, at the true position
      const auto before = fc.get_gnss_debug();
      (void)before;
      fc.update_gnss(t, fix_at(true_x, 0.0));
      if (t >= t_out_end) {
        if (fc.get_gnss_debug().accepted) {
          ++r.accepted_after;
        } else {
          ++r.rejected_after;
          ++r.reason_counts[static_cast<int>(fc.get_gnss_debug().reason)];
        }
      }
    }

    if (t >= t_out_end && !measured_return) {
      r.err_at_return = std::abs(fc.get_state().x[X] - true_x);
      measured_return = true;
    }
    const double err = std::abs(fc.get_state().x[X] - true_x);
    if (std::abs(t - (t_out_end +   5.0)) < dt * 0.5) r.err_5s   = err;
    if (std::abs(t - (t_out_end +  30.0)) < dt * 0.5) r.err_30s  = err;
    if (std::abs(t - (t_out_end + 300.0)) < dt * 0.5) r.err_300s = err;
  }
  return r;
}

void report(const char* label, const Recovery& r) {
  std::cerr << "  " << label << "\n"
            << "    error when fixes return : " << r.err_at_return << " m\n"
            << "    error  +5 s             : " << r.err_5s   << " m\n"
            << "    error +30 s             : " << r.err_30s  << " m\n"
            << "    error +300 s            : " << r.err_300s << " m\n"
            << "    fixes accepted / rejected after the blackout: "
            << r.accepted_after << " / " << r.rejected_after << "\n";
  for (int i = 0; i < GNSS_REJECTION_REASON_COUNT; ++i)
    if (r.reason_counts[i])
      std::cerr << "      rejection reason " << i << ": " << r.reason_counts[i] << "\n";
}

} // namespace

// Documents today's behaviour. Expected to FAIL once re-acquisition works, at
// which point it becomes the assertion that it does.
TEST(GnssReacquireTest, RecoversAfterALongBlackout) {
  FusionCore fc(blackout_config());
  State s0;
  fc.init(s0, 0.0);

  const Recovery r = run_blackout(fc);
  report("default config", r);

  EXPECT_GT(r.err_at_return, 100.0)
      << "the blackout did not produce a large enough error to test recovery";
  EXPECT_LT(r.err_300s, 10.0)
      << "GNSS came back at 5 Hz and the filter never re-acquired: "
      << r.accepted_after << " fixes accepted, " << r.rejected_after << " rejected";
}

// What the existing knobs can and cannot do. gnss_recovery_rejection_n with
// gnss_p_inflate_sigma is the only machinery in the filter aimed at this, it is
// off by default in both the core and the ROS node, and its header says 50 m
// "covers any realistic drift from a chi2 cascade". A multi-minute blackout is
// not that cascade.
TEST(GnssReacquireTest, SweepTheExistingRecoveryKnobs) {
  struct Case { const char* label; int n; double sigma; };
  const Case cases[] = {
    {"off (the default everywhere)",            0,    0.0},
    {"recovery_rejection_n=15, sigma=50 m",    15,   50.0},
    {"recovery_rejection_n=15, sigma=200 m",   15,  200.0},
    {"recovery_rejection_n=15, sigma=500 m",   15,  500.0},
    {"recovery_rejection_n=6,  sigma=500 m",    6,  500.0},
  };
  for (const auto& c : cases) {
    FusionCoreConfig cfg = blackout_config();
    cfg.gnss_recovery_rejection_n = c.n;
    if (c.sigma > 0.0) cfg.gnss_p_inflate_sigma = c.sigma;
    FusionCore fc(cfg);
    State s0;
    fc.init(s0, 0.0);
    const Recovery r = run_blackout(fc);
    report(c.label, r);
  }
  SUCCEED();
}

// The interaction that makes the recovery path dangerous, and the thing that is
// supposed to contain it.
//
// Recovery inflates P so a returning fix can be believed. An adversarial outlier
// cluster sitting at the blackout boundary (NCLT 2012-08-20, issue #64) arrives
// in exactly the same position: after a gap, far from the prediction, and
// internally self-consistent, so it looks like a legitimate recovery fix to
// every test based on the fix alone. The header for gnss_max_speed says so
// outright: "chi2 alone cannot tell a 700 m outlier from a legitimate recovery
// fix after a long gap, but physics can".
//
// The physical gate is what tells them apart, and it defaults to OFF. This
// measures what the filter does in both configurations.
TEST(GnssReacquireTest, OutlierClusterAtTheBlackoutBoundary) {
  struct Case { const char* label; double max_speed; };
  const Case cases[] = {
    {"gnss.max_speed off (the default)", 0.0},
    {"gnss.max_speed 3.0 m/s",           3.0},
  };

  for (const auto& c : cases) {
    FusionCoreConfig cfg = blackout_config();
    cfg.gnss_max_speed = c.max_speed;
    cfg.gnss_max_speed_margin = 5.0;
    FusionCore fc(cfg);
    State s0;
    fc.init(s0, 0.0);

    const double dt = 0.01, g = 9.80665;
    const double TRUE_SPEED = 1.5;
    const double T_PRE = 120.0, T_BLACKOUT = 460.0, T_CLUSTER = 20.0, T_POST = 120.0;
    const double t_out_start = T_PRE, t_out_end = T_PRE + T_BLACKOUT;
    const double t_cluster_end = t_out_end + T_CLUSTER;
    const double t_end = t_cluster_end + T_POST;
    const double CLUSTER_OFFSET = 700.0;      // metres of pure lie, off to the side

    double true_x = 0.0, worst_after_cluster = 0.0;
    double final_lateral = 0.0, final_along = 0.0;
    int accepted_cluster = 0;

    for (int step = 1; step * dt <= t_end + 1e-9; ++step) {
      const double t = step * dt;
      true_x += TRUE_SPEED * dt;
      const bool blackout = (t >= t_out_start && t < t_out_end);
      const bool cluster  = (t >= t_out_end && t < t_cluster_end);

      fc.update_imu(t, 0, 0, 0, 0, 0, g);
      if (step % 2 == 0) {
        fc.update_encoder(t, blackout ? 2.1 : TRUE_SPEED, 0.0, 0.0);
        fc.update_ground_constraint(t);
      }
      if (step % 20 == 0 && !blackout) {
        const double gy = cluster ? CLUSTER_OFFSET : 0.0;
        fc.update_gnss(t, fix_at(true_x, gy));
        if (cluster && fc.get_gnss_debug().accepted) ++accepted_cluster;
      }
      if (t >= t_cluster_end) {
        worst_after_cluster = std::max(worst_after_cluster,
                                       std::abs(fc.get_state().x[Y] - 0.0));
      }
      final_lateral = std::abs(fc.get_state().x[Y] - 0.0);
      final_along   = std::abs(fc.get_state().x[X] - true_x);
    }
    std::cerr << "  " << c.label << "\n"
              << "    cluster fixes accepted         : " << accepted_cluster << "\n"
              << "    worst lateral error afterwards : " << worst_after_cluster << " m\n"
              << "    lateral error at end of run    : " << final_lateral << " m\n"
              << "    along-track error at end       : " << final_along << " m\n";
  }
  SUCCEED();
}
