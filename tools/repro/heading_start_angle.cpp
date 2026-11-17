// Does a dual-antenna heading get accepted when the robot starts facing away
// from yaw = 0? update_gnss_heading() gates every heading, the first included,
// against a filter that initialises at yaw 0. Core library only, no ROS.
//
// Init mirrors fusion_node.cpp: P = 0.1 I with position 1000, bias priors as
// apply_bias_priors() sets them (gyro 0.175 rad/s, accel 0.30 m/s^2, encoder wz
// bias = gyro), and a ZUPT whenever the encoder reports the platform still.
// Truth: level, still IMU, perfect encoder, heading exact at 5 Hz, 0.02 rad.
//
// Measured 2026-11-16 on 0.5.0, before the fix in 0.5.1, parked 5 s then 1 m/s, 120 s:
//   120 deg first accepted at 51.00 s, 150 deg at 92.00 s, 170 and 180 never.
// args: true_yaw_deg  park_seconds(0 = already moving at init)  zupt(0/1)  T_END
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
using namespace fusioncore;
int main(int argc, char** argv) {
  const double DT = 0.01, G = 9.80665;
  const double TRUE_DEG = argc > 1 ? atof(argv[1]) : 45.0;
  const double PARK = argc > 2 ? atof(argv[2]) : 5.0;
  const bool   ZUPT = argc > 3 ? atoi(argv[3]) != 0 : true;
  const double T_END = argc > 4 ? atof(argv[4]) : 120.0;
  const double yaw_true = TRUE_DEG * M_PI / 180.0;
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.outlier_rejection = true;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  FusionCore fc(cfg);
  State s0; s0.P = StateMatrix::Identity() * 0.1;
  const double g = 0.175, a = 0.30;
  s0.P(B_GX,B_GX) = s0.P(B_GY,B_GY) = s0.P(B_GZ,B_GZ) = g*g;
  s0.P(B_AX,B_AX) = s0.P(B_AY,B_AY) = s0.P(B_AZ,B_AZ) = a*a;
  s0.P(B_EWZ,B_EWZ) = g*g;
  s0.P(0,0) = s0.P(1,1) = s0.P(2,2) = 1000.0;
  fc.init(s0, 0.0);
  int accepted = 0, rejected = 0; double t_first = -1.0, max_abs_wz = 0.0;
  for (int step = 1; step * DT <= T_END + 1e-9; ++step) {
    const double t = step * DT;
    const double v = t < PARK ? 0.0 : 1.0;
    fc.update_imu(t, 0, 0, 0, 0, 0, G);
    if (step % 2 == 0) {
      fc.update_encoder(t, v, 0.0, 0.0);
      fc.update_ground_constraint(t);
      if (ZUPT && v < 0.05) fc.update_zupt(t, 0.01);
    }
    if (step % 20 == 0) {
      sensors::GnssHeading h; h.heading_rad = yaw_true; h.accuracy_rad = 0.02; h.valid = true;
      if (fc.update_gnss_heading(t, h)) { ++accepted; if (t_first < 0) t_first = t; } else ++rejected;
    }
    max_abs_wz = std::max(max_abs_wz, std::abs(fc.get_state().x[WZ]));
  }
  const auto& x = fc.get_state().x;
  const double yaw = std::atan2(2*(x[QW]*x[QZ] + x[QX]*x[QY]), 1 - 2*(x[QY]*x[QY] + x[QZ]*x[QZ]));
  double err = (yaw - yaw_true) * 180.0 / M_PI;
  while (err > 180) err -= 360; while (err < -180) err += 360;
  printf("yaw0=%7.1f park=%4.1f zupt=%d  accepted=%4d rejected=%4d  t_first=%6.2f  "
         "final_err_deg=%8.2f  max|wz|=%7.4f  b_gz=%8.4f\n",
         TRUE_DEG, PARK, (int)ZUPT, accepted, rejected, t_first, err, max_abs_wz, x[B_GZ]);
  return 0;
}
