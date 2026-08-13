// Isolate predict from updates. Start with vx = 1 m/s already in the state and
// feed ONLY the IMU (which advances time). If position does not advance at
// 1 m/s, the motion model is the problem. If it does, an update is dragging it.
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
using namespace fusioncore;
int main(int argc, char** argv) {
  const bool WITH_ENC = argc > 1 ? atoi(argv[1]) != 0 : false;
  const double ALPHA  = argc > 2 ? atof(argv[2]) : 0.5;
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  const char* MM = argc > 3 ? argv[3] : "DifferentialDrive";
  cfg.motion_model = create_motion_model(MM);
  cfg.ukf.alpha = ALPHA;
  FusionCore fc(cfg);
  State s0;
  s0.x[VX] = 1.0;                       // already moving at 1 m/s, body frame
  fc.init(s0, 0.0);
  printf("  model=%s encoder=%d alpha=%.2f   truth: x advances 1.00 m/s\n",
         MM, (int)WITH_ENC, ALPHA);
  const double DT = 0.01, G = 9.80665;
  for (int step = 1; step * DT <= 20.0 + 1e-9; ++step) {
    double t = step * DT;
    fc.update_imu(t, 0, 0, 0, 0, 0, G);
    if (WITH_ENC && step % 2 == 0) fc.update_encoder(t, 1.0, 0.0, 0.0);
    if (step % 500 == 0) {
      const auto& x = fc.get_state().x;
      printf("  t=%5.1f  x=%8.3f (truth %6.2f)  vx=%7.4f  ratio=%5.1f%%\n",
             t, x[X], t, x[VX], 100.0 * x[X] / t);
    }
  }
  return 0;
}
