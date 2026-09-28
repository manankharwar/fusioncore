// Which states are actually observable, and which couplings carry the information?
//
// Phase 5 of the engineering plan says establish the physics before changing the state
// vector. This measures it instead of arguing about it. No ROS, no dataset, milliseconds.
//
// Two suspected unobservable groups:
//
//   ACCELERATION SPLIT   an accelerometer measures AX + B_AX + gravity. One measurement,
//                        two unknowns. If they are unobservable as a pair their
//                        correlation goes to -1 and their variances grow together while
//                        the SUM stays pinned.
//
//   YAW RATE TRIPLE      encoder gives wz + B_EWZ, gyro gives wz + B_GZ. Two
//                        measurements, three unknowns (WZ, B_GZ, B_EWZ).
//
// And the coupling question from the 2026-09-27 measurements: position advances at 80.7%
// of a perfect velocity, the IMU update subtracts the whole deficit, and P(X,VX) reads
// 0.000. Is the problem that position is not coupled to velocity, or that velocity is not
// coupled to acceleration so the encoder cannot constrain the split?
//
// build: see build.sh   usage: /tmp/obs [seconds]
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
#include <cstdlib>
using namespace fusioncore;

static double corr(const StateMatrix & P, int a, int b)
{
  const double da = P(a, a), db = P(b, b);
  if (da <= 0.0 || db <= 0.0) { return 0.0; }
  return P(a, b) / std::sqrt(da * db);
}

int main(int argc, char ** argv)
{
  const double SPEED = 1.0, DT = 0.01, G = 9.80665;
  const double T_END = argc > 1 ? atof(argv[1]) : 60.0;

  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.outlier_rejection = true;
  cfg.adaptive_imu = cfg.adaptive_encoder = cfg.adaptive_gnss = true;
  cfg.motion_model = create_motion_model("DifferentialDrive");

  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  std::printf("  perfect encoder %.1f m/s straight, still IMU, %.0f s\n\n", SPEED, T_END);
  std::printf("  CORRELATIONS (a value near -1 or +1 means the pair is not separable)\n");
  std::printf("  %5s %8s %9s %9s %9s %9s %9s %9s\n",
              "t", "x", "r(AX,bAX)", "r(X,VX)", "r(VX,AX)", "r(X,AX)",
              "r(WZ,bGZ)", "r(bGZ,bEWZ)");

  for (int step = 1; step * DT <= T_END + 1e-9; ++step) {
    const double t = step * DT;
    fc.update_imu(t, 0, 0, 0, 0, 0, G);
    if (step % 2 == 0) {
      fc.update_encoder(t, SPEED, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }
    if (step % 1000 == 0) {                       // every 10 s
      const auto & x = fc.get_state().x;
      const auto & P = fc.get_state().P;
      std::printf("  %5.0f %8.2f %9.3f %9.3f %9.3f %9.3f %9.3f %9.3f\n",
                  t, x[X],
                  corr(P, AX, B_AX), corr(P, X, VX), corr(P, VX, AX), corr(P, X, AX),
                  corr(P, WZ, B_GZ), corr(P, B_GZ, B_EWZ));
    }
  }

  const auto & P = fc.get_state().P;
  const auto & x = fc.get_state().x;
  std::printf("\n  VARIANCES at t=%.0f  (an unobservable direction grows without bound)\n", T_END);
  std::printf("    P(AX,AX)      %12.6f      P(B_AX,B_AX)   %12.6f\n", P(AX, AX), P(B_AX, B_AX));
  std::printf("    P(VX,VX)      %12.6f      P(X,X)         %12.6f\n", P(VX, VX), P(X, X));
  std::printf("    P(WZ,WZ)      %12.6f      P(B_GZ,B_GZ)   %12.6f\n", P(WZ, WZ), P(B_GZ, B_GZ));
  std::printf("    P(B_EWZ,B_EWZ)%12.6f\n", P(B_EWZ, B_EWZ));

  std::printf("\n  THE SUM that the accelerometer actually measures\n");
  std::printf("    AX + B_AX            %10.6f  (truth 0.0)\n", x[AX] + x[B_AX]);
  std::printf("    var(AX + B_AX)       %10.6f  (pinned if the SUM is observable)\n",
              P(AX, AX) + 2.0 * P(AX, B_AX) + P(B_AX, B_AX));
  std::printf("    var(AX) + var(B_AX)  %10.6f  (what it would be if independent)\n",
              P(AX, AX) + P(B_AX, B_AX));

  std::printf("\n  THE YAW TRIPLE, same test\n");
  std::printf("    var(WZ + B_GZ)       %10.6f  (gyro measures this sum)\n",
              P(WZ, WZ) + 2.0 * P(WZ, B_GZ) + P(B_GZ, B_GZ));
  std::printf("    var(WZ + B_EWZ)      %10.6f  (encoder measures this sum)\n",
              P(WZ, WZ) + 2.0 * P(WZ, B_EWZ) + P(B_EWZ, B_EWZ));
  std::printf("    var(B_GZ - B_EWZ)    %10.6f  (the unobservable DIFFERENCE, if any)\n",
              P(B_GZ, B_GZ) - 2.0 * P(B_GZ, B_EWZ) + P(B_EWZ, B_EWZ));
  return 0;
}
