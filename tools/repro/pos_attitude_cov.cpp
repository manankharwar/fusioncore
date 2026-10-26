// Expert's point 10: "GNSS position innovations correct yaw through the
// position-velocity-yaw correlation". That REQUIRES P(position, quaternion) to
// be non-zero while moving. If it is zero, GNSS can fix position but can never
// pull heading back, and yaw covariance grows without bound.
//
// Invariant to check: moving forward along +x with yaw uncertainty, a yaw error
// moves the robot sideways, so P(Y, QZ) must become non-zero and grow with vx*dt.
#include <cstdio>
#include "fusioncore/ukf.hpp"
#include "fusioncore/motion_model.hpp"
using namespace fusioncore;

int main()
{
  UKFParams p; p.alpha = 1.0;
  UKF ukf(p);
  ukf.set_motion_model(create_motion_model("DifferentialDrive", {}));
  State s;
  s.x[VX] = 1.0;                 // driving forward at 1 m/s
  s.P(VX,VX) = 0.01;
  s.P(QZ,QZ) = 1e-4;             // a little yaw uncertainty, as a 4-vector component
  s.P(X,X) = 1.0; s.P(Y,Y) = 1.0;
  ukf.init(s);

  const double dt = 0.01;
  printf("\n driving +x at 1 m/s, yaw uncertain. Does position couple to attitude?\n");
  printf("\n step      t        P(Y,QZ)         P(X,VX)        P(QZ,QZ)\n");
  for (int k = 1; k <= 1000; ++k) {
    ukf.predict(dt);
    if (k==1||k==2||k==10||k==100||k==1000) {
      const State& o = ukf.state();
      printf("%5d %7.2f  %14.9f %14.8f %14.9f\n",
             k, k*dt, o.P(Y,QZ), o.P(X,VX), o.P(QZ,QZ));
    }
  }
  printf("\nNon-zero and growing P(Y,QZ) means a GNSS Y innovation CAN pull yaw.\n"
         "Zero would mean heading is uncorrectable from position, which is the\n"
         "mechanism the expert proposed for unbounded yaw covariance.\n\n");
  return 0;
}
