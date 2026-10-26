// Two checks on why the filter is wrong-and-confident about heading.
//
// RESULTS, 2026-10-18, with ukf.tangent_space_attitude ON:
//
//   VY_body = 0.0000 exactly, always. CRAB IS RULED OUT. DifferentialDrive::predict
//   forces VY and AY to zero on every step, so the filter cannot explain the GNSS
//   track by sliding sideways even if it wanted to.
//
//   Cumulative change in var(QZ) over 120 s, by sensor:
//       IMU      +0.2138      adds heading uncertainty, correct, that is gyro noise
//       encoder  -0.0003      removes essentially nothing
//       GNSS     -0.1979      removes a lot
//   So GNSS is the false source. Note this test never calls update_imu_orientation,
//   so fusing a relative IMU yaw is NOT the cause either.
//
//   It is systematic, not random. Planting +30 and -30 deg gives exactly mirrored
//   trajectories, and var(QZ) is identical across planted errors, which is correct
//   for a Kalman filter since P does not depend on measurement values. Planting
//   +10 deg ends at +87 deg, so the correction drives heading AWAY from truth.
//
//   Not a sign error. Negating the attitude correction makes var(QZ) blow up to
//   1.6-3.0 and the per-sensor deltas reach +-21000.
//
// So GNSS updates remove heading uncertainty while moving heading away from truth.
// That is an inconsistent update, and the remaining suspect is that there is no
// error-state RESET after the multiplicative injection: in an ESKF, once the error
// is injected into the nominal state the tangent space has moved, and P must be
// transported by the reset Jacobian G = I - [0.5*dtheta]x. Omitting that is a
// known classic bug and would produce exactly this, a covariance that shrinks
// while the estimate wanders.
//
// 1. CRAB: if body-frame VY settles near v*sin(heading error), the filter is
//    explaining the GNSS track by sliding sideways instead of rotating.
// 2. FALSE CONFIDENCE: log var(QZ) across every update type. Whichever sensor
//    shrinks it while yaw stays wrong is claiming to observe yaw and is not.
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
  const double SPEED=1.0, DT=0.01, G=9.80665;
  const double YAW0 = (argc>1?atof(argv[1]):30.0)*M_PI/180.0;
  const bool   TANGENT = argc>2 ? atoi(argv[2])!=0 : true;
  const bool   NO_ACCEL = argc>3 ? atoi(argv[3])!=0 : false;

  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x=cfg.imu.gyro_noise_y=cfg.imu.gyro_noise_z=0.005;
  cfg.imu.accel_noise_x=cfg.imu.accel_noise_y=cfg.imu.accel_noise_z=0.1;
  cfg.imu_has_magnetometer=false;
  cfg.encoder.vel_noise_x=cfg.encoder.vel_noise_y=0.05;
  cfg.encoder.vel_noise_wz=0.02;
  cfg.outlier_rejection=true;
  cfg.adaptive_imu=cfg.adaptive_encoder=cfg.adaptive_gnss=true;
  cfg.motion_model=create_motion_model("DifferentialDrive");
  cfg.ukf.tangent_space_attitude = TANGENT;
  cfg.ukf.tangent_space_quaternion_mean = TANGENT;

  FusionCore fc(cfg);
  State s0; s0.x[QW]=std::cos(YAW0/2); s0.x[QZ]=std::sin(YAW0/2);
  fc.init(s0,0.0);

  double d_imu=0, d_enc=0, d_gnss=0;   // cumulative var(QZ) change per sensor
  printf("\n tangent=%d  no_accel=%d   planted yaw error %.0f deg\n", (int)TANGENT,(int)NO_ACCEL, YAW0*180/M_PI);
  printf(" expected crab VY if it is sliding instead of turning: %.3f m/s\n\n", SPEED*std::sin(YAW0));
  printf("  %7s %9s %9s %11s %11s %11s %11s\n","t","yaw_deg","VY_body","var(QZ)","d_imu","d_enc","d_gnss");

  for (int k=1;k*DT<=120.0;++k){
    double t=k*DT; double v0,v1;
    v0=fc.get_state().P(QZ,QZ);
    if (!NO_ACCEL) fc.update_imu(t,0,0,0, 0,0,G); else fc.update_imu(t,0,0,0, 0,0,0);
    v1=fc.get_state().P(QZ,QZ); d_imu += (v1-v0);

    v0=v1; fc.update_encoder(t,SPEED,0.0,0.0);
    v1=fc.get_state().P(QZ,QZ); d_enc += (v1-v0);

    if (k%100==0){
      v0=v1;
      sensors::GnssFix fix; fix.x=SPEED*t; fix.y=0; fix.z=0;
      fix.hdop=1.0; fix.vdop=1.5; fix.sigma_xy=1.0; fix.sigma_z=1.5;
      fix.satellites=12; fix.fix_type=sensors::GnssFixType::RTK_FIXED;
      fc.update_gnss(t,fix);
      v1=fc.get_state().P(QZ,QZ); d_gnss += (v1-v0);
    }
    if (k%2000==0||k==100){
      const State&s=fc.get_state();
      printf("  %7.1f %9.2f %9.4f %14.9f %11.4f %11.4f %11.4f\n",
             t, yaw_of(s)*180/M_PI, s.x[VY], s.P(QZ,QZ), d_imu, d_enc, d_gnss);
    }
  }
  printf("\n  d_* are CUMULATIVE change in var(QZ) from each sensor.\n"
         "  A large NEGATIVE value means that sensor is removing heading\n"
         "  uncertainty. If yaw is still wrong, it is not entitled to.\n\n");
  return 0;
}
