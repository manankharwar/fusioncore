// GPS for 20 s, then a blackout. Truth: 1 m/s dead straight the whole time.
// Measures how far the estimate is from truth at the end of the blackout.
//
// A blackout is the worst case for the sigma-weight bug: with no GPS, yaw has no
// absolute reference, the quaternion sigma points spread, and a negative centre
// weight subtracts the one point that is still pointing the right way. That is
// why the estimate used to travel BACKWARDS here while reporting yaw = 0.
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
using namespace fusioncore;

static double run(double bias_factor, double blackout_s, double alpha, bool verbose) {
  const double DT = 0.01, SPEED = 1.0, G = 9.80665, T_GPS = 20.0;
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.gnss.base_noise_xy = 1.0;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.gnss_coast_q_bias_factor = bias_factor;
  cfg.ukf.alpha = alpha;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  const double T_END = T_GPS + blackout_s;
  for (int step = 1; step * DT <= T_END + 1e-9; ++step) {
    double t = step * DT, tx = SPEED * t;
    fc.update_imu(t, 0, 0, 0, 0, 0, G);
    if (step % 2 == 0) { fc.update_encoder(t, SPEED, 0.0, 0.0); fc.update_ground_constraint(t); }
    if (t <= T_GPS && step % 100 == 0) {          // 1 Hz GPS, then it stops
      sensors::GnssFix f;
      f.x = tx; f.y = 0.0; f.z = 0.0;
      f.hdop = f.sigma_xy = 1.0; f.vdop = f.sigma_z = 2.0;
      f.satellites = 10; f.fix_type = sensors::GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
    }
    if (verbose && step % 1000 == 0) {
      const auto& x = fc.get_state().x;
      double yaw = std::atan2(2*(x[QW]*x[QZ]+x[QX]*x[QY]), 1-2*(x[QY]*x[QY]+x[QZ]*x[QZ]));
      printf("      t=%5.1f%s  x=%8.2f (truth %6.2f)  err=%7.2f  yaw=%7.2f  b_gz=%8.5f\n",
             t, (t > T_GPS ? " BLK" : "    "), x[X], tx,
             std::hypot(x[X]-tx, x[Y]), yaw*180.0/M_PI, x[B_GZ]);
    }
  }
  const auto& x = fc.get_state().x;
  return std::hypot(x[X] - SPEED*T_END, x[Y]);
}

int main() {
  printf("  GPS for 20 s, then blackout. Truth is 1 m/s dead straight throughout.\n");
  printf("  Error is |estimate - truth| at the end of the blackout. Lower is better.\n\n");

  printf("  %-12s %14s %14s %14s\n", "blackout", "alpha=0.1", "alpha=0.5", "alpha=1.0");
  printf("  %-12s %14s %14s %14s\n", "", "(Wm0 -99)", "(Wm0 -3)", "(Wm0 0, new)");
  for (double bo : {30.0, 60.0, 120.0, 240.0}) {
    double a = run(100.0, bo, 0.1, false);
    double b = run(100.0, bo, 0.5, false);
    double c = run(100.0, bo, 1.0, false);
    printf("  %-10.0f s %12.2f m %12.2f m %12.2f m\n", bo, a, b, c);
  }

  printf("\n  Does coast_q_bias_factor matter now? (120 s blackout, alpha = 1.0)\n");
  printf("      factor=100  %8.2f m\n", run(100.0, 120.0, 1.0, false));
  printf("      factor=1    %8.2f m\n", run(1.0,   120.0, 1.0, false));

  printf("\n  --- 120 s blackout, alpha = 0.1 (the old default) ---\n");
  run(100.0, 120.0, 0.1, true);
  printf("\n  --- 120 s blackout, alpha = 1.0 (the new default) ---\n");
  run(100.0, 120.0, 1.0, true);
  return 0;
}
