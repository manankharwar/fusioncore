// Expert check 3: round-trip. Generate sigma points from a known P, convert them
// back to tangent deviations, and confirm P is recovered. A factor-of-2 or
// half-angle mismatch between the four places would show up as a constant ratio.
//
// Done through the public API: set P, predict with a tiny dt and the attitude
// process noise zeroed, and read P back. Ideal round trip returns the same value.
#include <cstdio>
#include "fusioncore/ukf.hpp"
#include "fusioncore/motion_model.hpp"
using namespace fusioncore;

static double trip(double var_qz, bool tangent, double dt)
{
  UKFParams p; p.alpha = 1.0;
  p.tangent_space_attitude = tangent;
  p.tangent_space_quaternion_mean = tangent;
  p.q_orientation = 0.0; p.q_angular_vel = 0.0; p.q_position = 0.0;
  p.q_velocity = 0.0; p.q_acceleration = 0.0;
  p.q_gyro_bias = 0.0; p.q_accel_bias = 0.0; p.q_encoder_wz_bias = 0.0;
  UKF ukf(p);
  ukf.set_motion_model(create_motion_model("DifferentialDrive", {}));
  State s;
  s.P(QZ,QZ) = var_qz;
  s.P(QX,QX) = var_qz; s.P(QY,QY) = var_qz;
  ukf.init(s);
  ukf.predict(dt);
  return ukf.state().P(QZ,QZ);
}

int main()
{
  const double dt = 1e-9;   // effectively no motion, so only the round trip acts
  printf("\n  round trip of the attitude covariance, dt=%g, all Q zeroed\n\n", dt);
  printf("  %12s | %14s %9s | %14s %9s\n", "P(QZ,QZ) in", "OFF out","ratio","ON out","ratio");
  for (double v : {1e-6, 1e-4, 1e-2, 0.04, 0.25}) {
    double off = trip(v, false, dt);
    double on  = trip(v, true,  dt);
    printf("  %12.6g | %14.8g %9.4f | %14.8g %9.4f\n", v, off, off/v, on, on/v);
  }
  printf("\n  ratio 1.0 means the representation round trips exactly.\n"
         "  a constant ratio of 4 or 0.25 would be a half-angle factor-of-2 error.\n\n");
  return 0;
}
