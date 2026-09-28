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
  // THE PREDICT HYPOTHESIS. Position advances as R(q)*v*dt averaged over the sigma
  // points. If the yaw sigma points spread by delta, that average shrinks by E[cos
  // delta] = exp(-sigma_yaw^2 / 2) for a small spread. So a measured yaw sigma predicts
  // the advance ratio with no free parameters, and either it matches or the hypothesis
  // is wrong.
  {
    const auto & P2 = fc.get_state().P;
    const auto & x2 = fc.get_state().x;
    // Small-angle: yaw ~ 2*qz when qw ~ 1, so sigma_yaw ~ 2*sigma_qz.
    const double sig_qz  = std::sqrt(std::max(P2(QZ, QZ), 0.0));
    const double sig_yaw = 2.0 * sig_qz / std::max(std::abs(x2[QW]), 1e-9);
    const double predicted_ratio = std::exp(-0.5 * sig_yaw * sig_yaw);
    const double actual_ratio = x2[X] / (SPEED * T_END);
    std::printf("\n  PREDICT HYPOTHESIS: does the yaw spread explain the short advance?\n");
    std::printf("    sigma(qz)                 %10.6f\n", sig_qz);
    std::printf("    implied sigma(yaw)        %10.4f rad  = %.1f deg\n",
                sig_yaw, sig_yaw * 180.0 / M_PI);
    std::printf("    predicted advance E[cos]  %10.4f\n", predicted_ratio);
    std::printf("    ACTUAL advance x/(v*t)    %10.4f\n", actual_ratio);
    std::printf("    -> %s\n", (std::abs(predicted_ratio - actual_ratio) < 0.05)
                ? "MATCHES. The sigma-point spread in YAW is the mechanism."
                : "DOES NOT MATCH. Something else is shortening the advance.");
  }

  // A quaternion component lives in [-1,1], so its variance cannot exceed 1 and its
  // standard deviation cannot exceed 1 either. Anything above that is not a large
  // uncertainty, it is a covariance inconsistent with the manifold the state lives on.
  {
    const auto & P3 = fc.get_state().P;
    const auto & x3 = fc.get_state().x;
    std::printf("\n  QUATERNION COVARIANCE vs the unit-norm constraint\n");
    const char * nm[4] = {"QW", "QX", "QY", "QZ"};
    const int idx[4] = {QW, QX, QY, QZ};
    for (int i = 0; i < 4; ++i) {
      const double v = P3(idx[i], idx[i]);
      std::printf("    P(%s,%s) %12.4f   sigma %10.4f   %s\n", nm[i], nm[i], v,
                  std::sqrt(std::max(v, 0.0)),
                  std::sqrt(std::max(v, 0.0)) > 1.0 ? "IMPOSSIBLE (>1)" : "ok");
    }
    const double nrm = std::sqrt(x3[QW]*x3[QW] + x3[QX]*x3[QX] +
                                 x3[QY]*x3[QY] + x3[QZ]*x3[QZ]);
    std::printf("    |q| = %.9f  (the ESTIMATE is fine; the COVARIANCE is not)\n", nrm);
  }

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
