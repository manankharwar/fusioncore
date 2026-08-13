// Minimal dead-reckoning check. No GPS, no ROS, no dataset.
// Feed a PERFECT encoder (1 m/s straight) and a still IMU, and watch what the
// filter does. Truth is trivially x = 1.0 * t, y = 0, yaw = 0.
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
using namespace fusioncore;

int main(int argc, char** argv) {
  const double SPEED = 1.0, DT = 0.01, G = 9.80665;
  const double T_END = argc > 1 ? atof(argv[1]) : 60.0;
  const bool   USE_GC = argc > 2 ? atoi(argv[2]) != 0 : true;
  const double Q_ACC  = argc > 3 ? atof(argv[3]) : -1.0;   // <0 = leave default
  const double Q_BACC = argc > 4 ? atof(argv[4]) : -1.0;
  const double ALPHA  = argc > 5 ? atof(argv[5]) : -1.0;
  const double Q_WZ   = argc > 6 ? atof(argv[6]) : -1.0;

  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.outlier_rejection = true;
  cfg.adaptive_imu = cfg.adaptive_encoder = cfg.adaptive_gnss = true;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  if (Q_ACC  >= 0.0) cfg.ukf.q_acceleration = Q_ACC;
  if (Q_BACC >= 0.0) cfg.ukf.q_accel_bias   = Q_BACC;
  if (ALPHA  >  0.0) cfg.ukf.alpha           = ALPHA;
  if (Q_WZ   >= 0.0) cfg.ukf.q_angular_vel    = Q_WZ;

  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  printf("  perfect encoder %.1f m/s straight, still IMU, ground_constraint=%d\n",
         SPEED, (int)USE_GC);
  printf("  %6s %9s %8s %7s %8s %8s %8s %8s %10s\n",
         "t", "x", "vx", "yaw", "accX", "biasAX", "P_xx", "P_xvx", "lastCorr");
  for (int step = 1; step * DT <= T_END + 1e-9; ++step) {
    double t = step * DT;
    fc.update_imu(t, 0, 0, 0, 0, 0, G);
    if (step % 2 == 0) {
      fc.update_encoder(t, SPEED, 0.0, 0.0);
      if (USE_GC) fc.update_ground_constraint(t);
    }
    if (step % 500 == 0) {                       // every 5 s
      const auto& x = fc.get_state().x;
      const auto& P = fc.get_state().P;
      double tx = SPEED * t;
      double yaw = std::atan2(2*(x[QW]*x[QZ] + x[QX]*x[QY]),
                              1 - 2*(x[QY]*x[QY] + x[QZ]*x[QZ]));
      (void)tx;
      printf("  %6.1f %9.2f %8.3f %7.2f %8.3f %8.4f %8.2f %8.3f %10.4f\n",
             t, x[X], x[VX], yaw*180.0/M_PI, x[AX], x[B_AX],
             P(X,X), P(X,VX), fc.last_position_correction());
    }
  }
  return 0;
}
