// GPS for 20 s, then a blackout. Truth: 1 m/s dead straight the whole time.
// Measures how far the estimate is from truth at the end of the blackout, and
// what coast's 100x gyro-bias noise inflation does to that.
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
using namespace fusioncore;

static double run(double bias_factor, double blackout_s, bool verbose) {
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
  printf("  coast_q_bias_factor inflates gyro-bias process noise during coast.\n\n");
  printf("  %-14s %12s %12s %12s\n", "blackout", "factor=100", "factor=1", "improvement");
  for (double bo : {30.0, 60.0, 120.0}) {
    double a = run(100.0, bo, false);
    double b = run(1.0,   bo, false);
    printf("  %-14.0f %10.2f m %10.2f m %11.1fx\n", bo, a, b, (b > 1e-9 ? a/b : 0.0));
  }
  printf("\n  --- 60 s blackout, factor=100 (the shipped default) ---\n");
  run(100.0, 60.0, true);
  printf("\n  --- 60 s blackout, factor=1 (no inflation) ---\n");
  run(1.0, 60.0, true);
  return 0;
}
