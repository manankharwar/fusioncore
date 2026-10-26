// Does P(X,AX) stay zero, or is it just the first-order integrator?
// x+ = x + v*dt uses v BEFORE the acceleration update, so after ONE step
// position cannot depend on acceleration. Over many steps it should, via velocity.
#include <cstdio>
#include "fusioncore/ukf.hpp"
#include "fusioncore/motion_model.hpp"
using namespace fusioncore;

int main()
{
  UKFParams p; p.alpha = 1.0;
  UKF ukf(p);
  ukf.set_motion_model(create_motion_model("ConstantVelocityAcceleration", {}));
  State s;
  s.x[VX] = 1.0;
  s.P(VX,VX)=0.25; s.P(VY,VY)=0.25; s.P(VZ,VZ)=0.25;
  s.P(AX,AX)=0.09; s.P(AY,AY)=0.09; s.P(AZ,AZ)=0.09;
  s.P(X,X)=1.0; s.P(Y,Y)=1.0; s.P(Z,Z)=1.0;
  ukf.init(s);

  const double dt = 0.01;
  printf("\n step      t       P(X,VX)        P(X,AX)       P(VX,AX)    P(X,AX)/P(X,VX)\n");
  for (int k = 1; k <= 1000; ++k) {
    ukf.predict(dt);
    if (k==1||k==2||k==3||k==10||k==100||k==500||k==1000) {
      const State& o = ukf.state();
      printf("%5d %7.2f  %13.8f %14.8f %13.8f   %8.4f\n",
             k, k*dt, o.P(X,VX), o.P(X,AX), o.P(VX,AX),
             o.P(X,VX)!=0 ? o.P(X,AX)/o.P(X,VX) : 0.0);
    }
  }
  printf("\nIf P(X,AX) is non-zero from step 2 on, the single-step zero is just\n"
         "first-order Euler (x += v*dt before v += a*dt), not a missing coupling.\n\n");
  return 0;
}
