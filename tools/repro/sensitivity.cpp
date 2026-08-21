// Why does the same input give 38 m one run and 345 m the next?
//
// NCLT 2013-04-05 measured, 8 runs, identical code and data:
//   FusionCore  38 - 345 m ATE   (9.0x spread)
//   RL-EKF     230 - 268 m       (1.17x spread)
// Achieved filter rate does NOT explain it (r = +0.05 against ATE), so the cause
// is not "the CPU was slower". Something else about a run differs.
//
// The only things that CAN differ between two replays of the same data are the
// order in which same-timestamp callbacks fire, sub-microsecond stamp jitter, and
// how many predict() calls the executor squeezes in. This feeds the filter the
// same measurements with exactly those perturbations and measures the divergence.
//
// A well conditioned filter answers the same to 3 decimal places. If a 1 us stamp
// shift moves the final position by metres, that is the whole 9x mystery, and it
// becomes a 30 second test instead of a 70 minute benchmark.
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
using namespace fusioncore;

struct Out { double x, y, yaw, path; };

// mode 0: baseline
// mode 1: swap encoder / ground-constraint order at the same timestamp
// mode 2: +1 microsecond on every IMU stamp
// mode 3: one extra predict() per second (a scheduling hiccup)
static Out run(int mode, double blackout_s) {
  const double DT = 0.01, SPEED = 1.0, G = 9.80665, T_GPS = 20.0;
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

  const double T_END = T_GPS + blackout_s;
  double px = 0, py = 0, path = 0;
  for (int step = 1; step * DT <= T_END + 1e-9; ++step) {
    double t = step * DT, tx = SPEED * t;
    double t_imu = (mode == 2) ? t + 1e-6 : t;
    fc.update_imu(t_imu, 0, 0, 0, 0, 0, G);
    if (mode == 3 && step % 100 == 0) fc.update_imu(t_imu, 0, 0, 0, 0, 0, G);
    if (step % 2 == 0) {
      if (mode == 1) {                       // ground constraint first
        fc.update_ground_constraint(t);
        fc.update_encoder(t, SPEED, 0.0, 0.0);
      } else {                               // encoder first (the normal order)
        fc.update_encoder(t, SPEED, 0.0, 0.0);
        fc.update_ground_constraint(t);
      }
    }
    if (t <= T_GPS && step % 100 == 0) {
      sensors::GnssFix f;
      f.x = tx; f.y = 0.0; f.z = 0.0;
      f.hdop = f.sigma_xy = 1.0; f.vdop = f.sigma_z = 2.0;
      f.satellites = 10; f.fix_type = sensors::GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
    }
    const auto& x = fc.get_state().x;
    path += std::hypot(x[X] - px, x[Y] - py);
    px = x[X]; py = x[Y];
  }
  const auto& x = fc.get_state().x;
  double yaw = std::atan2(2*(x[QW]*x[QZ] + x[QX]*x[QY]),
                          1 - 2*(x[QY]*x[QY] + x[QZ]*x[QZ])) * 180.0 / M_PI;
  return { x[X], x[Y], yaw, path };
}

int main() {
  const char* name[4] = { "baseline",
                          "callback order swapped",
                          "IMU stamp +1 microsecond",
                          "one extra predict per second" };
  printf("  Same measurements, tiny perturbations. Truth: straight, 1 m/s.\n");
  printf("  A well conditioned filter gives the same answer to every perturbation.\n");

  for (double bo : {0.0, 30.0, 120.0}) {
    printf("\n  --- blackout %.0f s (total %.0f s) ---\n", bo, 20.0 + bo);
    printf("  %-32s %10s %10s %10s %12s\n", "", "x", "y", "yaw", "vs baseline");
    Out base = run(0, bo);
    for (int m = 0; m < 4; ++m) {
      Out o = run(m, bo);
      double d = std::hypot(o.x - base.x, o.y - base.y);
      printf("  %-32s %10.3f %10.3f %10.2f %11.3f m%s\n",
             name[m], o.x, o.y, o.yaw, d,
             (m && d > 1.0) ? "   <-- DIVERGED" : "");
    }
  }
  printf("\n  Truth for the 120 s case: x = 140.00, y = 0.00, yaw = 0.00\n");
  return 0;
}
