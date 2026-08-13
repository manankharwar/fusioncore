// Scope test: which shipped indoor configurations actually suffer the
// position-vs-velocity collapse? Truth is always x = 1.0 * t, straight, level.
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
#include <string>
using namespace fusioncore;

struct Result { double x, err, yaw_sigma_deg; };

// mode: 0 = encoder only          (wheels_indoor / f1tenth_indoor)
//       1 = encoder + encoder2    (icp_indoor option B: ICP as 2nd velocity)
//       2 = encoder + VSLAM pose  (vslam_imu: pose INCLUDING yaw)
//       3 = encoder + 9-axis IMU orientation (has_magnetometer: true)
static Result run(int mode, double t_end) {
  const double DT = 0.01, SPEED = 1.0, G = 9.80665;
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = (mode == 3);
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  for (int step = 1; step * DT <= t_end + 1e-9; ++step) {
    double t = step * DT;
    fc.update_imu(t, 0, 0, 0, 0, 0, G);
    if (mode == 3) fc.update_imu_orientation(t, 0.0, 0.0, 0.0);
    if (step % 2 == 0) {
      fc.update_encoder(t, SPEED, 0.0, 0.0);
      fc.update_ground_constraint(t);
      // A second VELOCITY source, which is what encoder2 / ICP odometry is.
      if (mode == 1) fc.update_encoder(t, SPEED, 0.0, 0.0, 0.01, 0.01, 0.004);
    }
    // VSLAM at 10 Hz: full 6-DOF POSE, so it observes yaw directly.
    if (mode == 2 && step % 10 == 0) {
      sensors::VslamPose p;
      p.x = SPEED * t; p.y = 0.0; p.z = 0.0;
      p.roll = p.pitch = p.yaw = 0.0;
      fc.update_pose(t, p);
    }
  }
  const auto& x = fc.get_state().x;
  const auto& P = fc.get_state().P;
  Result r;
  r.x = x[X];
  r.err = std::hypot(x[X] - SPEED * t_end, x[Y]);
  r.yaw_sigma_deg = 2.0 * std::sqrt(std::max(P(QZ,QZ), 0.0)) * 180.0 / M_PI;
  return r;
}

int main() {
  const double T = 60.0;
  const char* names[] = {
    "encoder + 6-axis IMU        (wheels_indoor, f1tenth_indoor)",
    "+ 2nd VELOCITY source       (icp_indoor: ICP odometry)",
    "+ VSLAM POSE (yaw observed) (vslam_imu)",
    "+ 9-axis IMU orientation    (has_magnetometer: true, e.g. JR)",
  };
  printf("  Truth after %.0f s: x = %.1f m, error 0.00 m\n\n", T, T);
  printf("  %-58s %9s %9s %11s\n", "configuration", "x (m)", "err (m)", "yaw sigma");
  for (int m = 0; m < 4; ++m) {
    Result r = run(m, T);
    printf("  %-58s %9.2f %9.2f %9.0f deg%s\n",
           names[m], r.x, r.err, r.yaw_sigma_deg,
           (r.err < 5.0 ? "   OK" : "   BROKEN"));
  }
  return 0;
}
