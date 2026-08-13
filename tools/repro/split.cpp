// Where does position motion come from during a GPS blackout?
// Truth: 1 m/s dead straight. Velocity and yaw are both perfect throughout,
// so predict SHOULD advance x by exactly integral(vx dt) and updates should
// contribute ~nothing. Measure both.
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
using namespace fusioncore;

int main() {
  const double DT = 0.01, SPEED = 1.0, G = 9.80665, T_GPS = 20.0, T_END = 80.0;
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.gnss.base_noise_xy = 1.0;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  double vel_integral = 0.0;     // what velocity alone accounts for
  double x_prev = fc.get_state().x[X];
  double moved_by_updates = 0.0; // signed X motion NOT explained by predict
  double moved_by_predict = 0.0;
  double by_imu = 0.0, by_enc = 0.0, by_gc = 0.0, by_gps = 0.0;

  printf("  %6s %9s %8s %10s %9s %9s %10s %11s\n",
         "t", "x", "truth", "imu_innov", "AX", "B_AX", "P(X,AX)", "P(X,VX)");
  for (int step = 1; step * DT <= T_END + 1e-9; ++step) {
    double t = step * DT, tx = SPEED * t;

    // --- predict-only leg: IMU advances the clock, nothing corrects ---
    double x_before = fc.get_state().x[X];
    double vx_before = fc.get_state().x[VX];
    fc.update_imu(t, 0, 0, 0, 0, 0, G);
    double x_after_imu = fc.get_state().x[X];
    // The IMU call = predict + IMU update. Attribute the modelled part to
    // predict and the rest to the update.
    double modelled = DT * vx_before;
    moved_by_predict += modelled;
    moved_by_updates += (x_after_imu - x_before) - modelled;
    by_imu           += (x_after_imu - x_before) - modelled;
    vel_integral += modelled;

    if (step % 2 == 0) {
      double xb = fc.get_state().x[X];
      fc.update_encoder(t, SPEED, 0.0, 0.0);
      double xm = fc.get_state().x[X];
      by_enc += xm - xb;
      fc.update_ground_constraint(t);
      by_gc  += fc.get_state().x[X] - xm;
      moved_by_updates += fc.get_state().x[X] - xb;   // pure update, no predict
    }
    if (t <= T_GPS && step % 100 == 0) {
      double xb = fc.get_state().x[X];
      sensors::GnssFix f;
      f.x = tx; f.y = 0.0; f.z = 0.0;
      f.hdop = f.sigma_xy = 1.0; f.vdop = f.sigma_z = 2.0;
      f.satellites = 10; f.fix_type = sensors::GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
      moved_by_updates += fc.get_state().x[X] - xb;
      by_gps += fc.get_state().x[X] - xb;
    }
    if (step % 1000 == 0) {
      const auto& x = fc.get_state().x;
      const auto& P = fc.get_state().P;
      // What the IMU update actually sees: measured is (0,0,g); predicted is
      // AX + B_AX + gravity-from-attitude. Innovation drives every correlated
      // correction, position included.
      double qw=x[QW],qx=x[QX],qy=x[QY],qz=x[QZ];
      double pred_ax = x[AX] + x[B_AX] + 2*(qx*qz - qw*qy)*9.80665;
      printf("  %6.1f %9.2f %8.2f %10.4f %9.4f %9.4f %10.4f %11.3f%s\n",
             t, x[X], tx, pred_ax - 0.0, x[AX], x[B_AX], P(X,AX), P(X,VX),
             (t > T_GPS ? "  BLK" : ""));
    }
    x_prev = fc.get_state().x[X];
  }
  (void)x_prev;
  printf("\n  integral of vx dt (what velocity allows) : %.2f m\n", vel_integral);
  printf("  truth                                    : %.2f m\n", SPEED * T_END);
  printf("  final x                                  : %.2f m\n\n", fc.get_state().x[X]);
  printf("  WHICH UPDATE MOVES POSITION (signed X, metres)\n");
  printf("    IMU update        : %9.2f\n", by_imu);
  printf("    encoder update    : %9.2f\n", by_enc);
  printf("    ground constraint : %9.2f\n", by_gc);
  printf("    GNSS update       : %9.2f\n", by_gps);
  return 0;
}
