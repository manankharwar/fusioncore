// Does GNSS position alone pull heading back to truth, and how fast?
//
// The expert's point: a non-zero P(Y,QZ) proves the COUPLING exists, not that the
// CORRECTION is right. If var(QZ) exceeds 1 the covariance is unphysical, because a
// unit-quaternion component lives in [-1,1], so GNSS would correct yaw in the right
// direction with the wrong magnitude.
//
// Setup: truth drives straight along +x at 1 m/s. The filter starts with 30 degrees of
// yaw error. Encoder says "1 m/s forward, no rotation". GNSS gives truth position at
// 1 Hz. No heading source of any kind. If heading correction works, yaw goes to 0.
#include <cstdio>
#include <cmath>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"
using namespace fusioncore;

static double yaw_of(const State& s) {
  double qw=s.x[QW], qx=s.x[QX], qy=s.x[QY], qz=s.x[QZ];
  return std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
}

int main(int argc, char** argv) {
  const double SPEED = 1.0, DT = 0.01, G = 9.80665;
  const double YAW0 = (argc > 1 ? atof(argv[1]) : 30.0) * M_PI / 180.0;
  const double T_END = argc > 2 ? atof(argv[2]) : 120.0;
  const double TURN  = argc > 3 ? atof(argv[3]) : 0.0;   // rad/s amplitude, 0 = straight
  const bool   TANGENT = argc > 4 ? atoi(argv[4]) != 0 : false;  // tangent-space quaternion mean
  const double QORI  = argc > 5 ? atof(argv[5]) : -1.0;  // attitude process noise override
  const bool   FREEZE = argc > 6 ? atoi(argv[6]) != 0 : false;  // platform.profile ground_robot freeze

  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.outlier_rejection = true;
  cfg.adaptive_imu = cfg.adaptive_encoder = cfg.adaptive_gnss = true;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.ukf.tangent_space_quaternion_mean = TANGENT;
  cfg.ukf.tangent_space_attitude = TANGENT;
  if (QORI > 0.0) cfg.ukf.q_orientation = QORI;
  if (FREEZE) {
    // exactly what platform.profile ground_robot sets
    cfg.ukf.q_acceleration = 0.0; cfg.ukf.q_accel_bias = 0.0;
    cfg.ukf.q_gyro_bias = 0.0; cfg.ukf.q_encoder_wz_bias = 0.0;
  }

  FusionCore fc(cfg);
  State s0;
  if (FREEZE) {
    s0.P(B_GX,B_GX)=1e-12; s0.P(B_GY,B_GY)=1e-12; s0.P(B_GZ,B_GZ)=1e-12;
    s0.P(B_AX,B_AX)=1e-12; s0.P(B_AY,B_AY)=1e-12; s0.P(B_AZ,B_AZ)=1e-12;
    s0.P(B_EWZ,B_EWZ)=1e-12;
  }
  s0.x[QW] = std::cos(YAW0/2.0);
  s0.x[QZ] = std::sin(YAW0/2.0);          // the planted heading error
  fc.init(s0, 0.0);

  printf("\n  truth: straight along +x at %.1f m/s. filter starts %.1f deg off.\n", SPEED, YAW0*180/M_PI);
  printf("  GNSS position at 1 Hz, no heading source. tangent_mean=%d\n\n", (int)TANGENT);
  printf("  %7s %10s %10s %12s %12s %10s %11s\n",
         "t", "yaw_deg", "err_deg", "var(QZ)", "P(Y,QZ)", "pos_err", "B_GZ");

  int n_gnss = 0, n_rej = 0;
  double tx = 0.0, ty = 0.0, tyaw = 0.0;
  for (int k = 1; k*DT <= T_END; ++k) {
    double t = k*DT;
    double wz_t = TURN * std::sin(2.0*M_PI*t/40.0);
    tyaw += wz_t*DT; tx += SPEED*std::cos(tyaw)*DT; ty += SPEED*std::sin(tyaw)*DT;
    double wz = TURN * std::sin(2.0*M_PI*t/40.0);   // slow weave, 40 s period
    fc.update_imu(t, 0,0,wz, 0,0,G);
    fc.update_encoder(t, SPEED, 0.0, wz);
    if (k % 100 == 0) {                              // 1 Hz GNSS at TRUTH
      sensors::GnssFix fix;
      fix.x = tx; fix.y = ty; fix.z = 0.0;
      fix.hdop = 1.0; fix.vdop = 1.5; fix.sigma_xy = 1.0; fix.sigma_z = 1.5;
      fix.satellites = 12; fix.fix_type = sensors::GnssFixType::RTK_FIXED;
      ++n_gnss;
      if (!fc.update_gnss(t, fix)) ++n_rej;
    }
    if (k % 1000 == 0 || k == 100) {
      const State& s = fc.get_state();
      double y = yaw_of(s);
      printf("  %7.1f %10.3f %10.3f %12.5f %12.6f %10.3f %11.6f\n",
             t, y*180/M_PI, std::atan2(std::sin(y-tyaw),std::cos(y-tyaw))*180/M_PI,
             s.P(QZ,QZ), s.P(Y,QZ), std::hypot(s.x[X]-tx, s.x[Y]-ty), s.x[B_GZ]);
    }
  }
  printf("\n  GNSS fixes %d, rejected %d\n", n_gnss, n_rej);
  printf("  var(QZ) above 1.0 is UNPHYSICAL: a unit-quaternion component is in [-1,1].\n\n");
  return 0;
}
