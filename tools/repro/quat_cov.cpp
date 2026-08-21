// Which part of the quaternion covariance is blowing up?
//
// A quaternion has 4 components but only 3 degrees of freedom: |q| = 1 removes one.
// FusionCore stores it as 4 free scalars inside a 23x23 P, so there is a 4th,
// NON-PHYSICAL direction (along q itself, the "radial" or norm direction) that no
// measurement constrains and no code bounds. P(QZ,QZ) reaches 4.30 on a component
// that cannot leave [-1,1], so something unphysical is growing.
//
// Split the 4x4 quaternion block into:
//   radial      = q^T P_qq q                    the norm direction. SHOULD BE ~0.
//   tangential  = trace(P_qq) - radial          real rotation uncertainty.
//
// If radial dominates, the fix is a tangent-space projection, which is a congruence
// transform and therefore preserves positive semi-definiteness. If tangential
// dominates, the uncertainty is genuine and the fix has to be an error-state
// reformulation. This decides which.
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
using namespace fusioncore;

int main(int argc, char** argv) {
  const double DT = 0.01, SPEED = 1.0, G = 9.80665, T_GPS = 20.0;
  const double T_END = (argc > 1) ? atof(argv[1]) : 140.0;

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

  printf("  quaternion covariance, split into its physical and non-physical parts\n");
  printf("  GPS until %.0f s, then blackout. state.hpp initialises each diagonal to 1e-8.\n\n", T_GPS);
  printf("  %6s %11s %11s %11s %9s %9s %8s\n",
         "t", "radial", "tangential", "P(QZ,QZ)", "|q|", "rad share", "yaw");

  for (int step = 1; step * DT <= T_END + 1e-9; ++step) {
    double t = step * DT, tx = SPEED * t;
    fc.update_imu(t, 0, 0, 0, 0, 0, G);
    if (step % 2 == 0) { fc.update_encoder(t, SPEED, 0.0, 0.0); fc.update_ground_constraint(t); }
    if (t <= T_GPS && step % 100 == 0) {
      sensors::GnssFix f;
      f.x = tx; f.y = 0.0; f.z = 0.0;
      f.hdop = f.sigma_xy = 1.0; f.vdop = f.sigma_z = 2.0;
      f.satellites = 10; f.fix_type = sensors::GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
    }
    if (step % 1000 == 0) {
      const auto& st = fc.get_state();
      Eigen::Vector4d q(st.x[QW], st.x[QX], st.x[QY], st.x[QZ]);
      double qn = q.norm();
      Eigen::Matrix4d Pqq = st.P.block<4,4>(QW, QW);
      Eigen::Vector4d qhat = (qn > 1e-12) ? (q / qn).eval() : Eigen::Vector4d(1,0,0,0);
      double radial = qhat.transpose() * Pqq * qhat;      // variance along q itself
      double total  = Pqq.trace();
      double tang   = total - radial;
      double yaw = std::atan2(2*(st.x[QW]*st.x[QZ] + st.x[QX]*st.x[QY]),
                              1 - 2*(st.x[QY]*st.x[QY] + st.x[QZ]*st.x[QZ])) * 180.0 / M_PI;
      printf("  %6.1f %11.3e %11.3e %11.3e %9.6f %8.1f%% %8.2f%s\n",
             t, radial, tang, st.P(QZ,QZ), qn,
             100.0 * radial / std::max(total, 1e-30), yaw, (t > T_GPS ? "  BLK" : ""));
    }
  }
  printf("\n  radial is the NON-PHYSICAL direction. Any share of it above a few percent\n");
  printf("  means the filter is spending uncertainty on a degree of freedom that does\n");
  printf("  not exist, and that is what pushes the sigma points off the manifold.\n");
  return 0;
}
