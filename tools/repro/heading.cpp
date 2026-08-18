// Where does heading go during a GPS blackout, when nothing is turning?
//
// Truth: 1 m/s dead straight, zero rotation. The encoder reports wz = 0 exactly
// and the gyro reports wz = 0 exactly, so there is no rotation anywhere in the
// input. With ukf.alpha = 1.0 (0.3.7) position now follows velocity correctly,
// but yaw still runs away, and it does NOT do so smoothly: it sits near -22 deg
// for 80 s and then moves ~82 deg inside ten seconds.
//
// A gyro bias of 0.0104 rad/s ramping over 120 s only accounts for ~36 deg, so
// integration of the bias is not the whole story. This prints per-step so the
// jump can be located exactly, and attributes each step's yaw change to the
// predict step versus each individual measurement update.
#include "fusioncore/fusioncore.hpp"
#include <cstdio>
#include <cmath>
using namespace fusioncore;

static double yaw_of(const StateVector& x) {
  return std::atan2(2*(x[QW]*x[QZ] + x[QX]*x[QY]),
                    1 - 2*(x[QY]*x[QY] + x[QZ]*x[QZ])) * 180.0 / M_PI;
}
static double wrap(double d) {                    // shortest signed difference
  while (d >  180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

int main(int argc, char** argv) {
  const double DT = 0.01, SPEED = 1.0, G = 9.80665, T_GPS = 20.0;
  const double T_END    = (argc > 1) ? atof(argv[1]) : 140.0;
  const double fine_from = (argc > 2) ? atof(argv[2]) : -1.0;

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

  printf("  per-step yaw attribution. gyro wz = 0, encoder wz = 0, truth straight.\n");
  printf("  GPS stops at %.0f s. Reporting the 12 largest single-step yaw changes,\n", T_GPS);
  printf("  then a 1 Hz trace of the states that could drive it.\n\n");

  struct Ev { double t, d_imu, d_enc, d_gnd, total; } worst[12] = {};
  double prev_yaw = yaw_of(fc.get_state().x);

  printf("  %6s %9s %9s %9s %9s %10s %10s %11s\n",
         "t", "yaw", "b_gz", "b_ewz", "wz", "sd_yaw", "P(QZ,QZ)", "P(QZ,BGZ)");

  for (int step = 1; step * DT <= T_END + 1e-9; ++step) {
    const double t = step * DT;
    Ev e{t, 0, 0, 0, 0};

    double y0 = yaw_of(fc.get_state().x);
    fc.update_imu(t, 0, 0, 0, 0, 0, G);          // predict + IMU update together
    double y1 = yaw_of(fc.get_state().x);
    e.d_imu = wrap(y1 - y0);

    if (step % 2 == 0) {
      fc.update_encoder(t, SPEED, 0.0, 0.0);
      double y2 = yaw_of(fc.get_state().x);
      e.d_enc = wrap(y2 - y1);
      fc.update_ground_constraint(t);
      double y3 = yaw_of(fc.get_state().x);
      e.d_gnd = wrap(y3 - y2);
    }
    if (t <= T_GPS && step % 100 == 0) {
      sensors::GnssFix f;
      f.x = SPEED * t; f.y = 0.0; f.z = 0.0;
      f.hdop = f.sigma_xy = 1.0; f.vdop = f.sigma_z = 2.0;
      f.satellites = 10; f.fix_type = sensors::GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
    }

    const auto& st = fc.get_state();
    double yaw = yaw_of(st.x);
    e.total = wrap(yaw - prev_yaw);
    prev_yaw = yaw;

    // keep the 12 biggest single-step jumps
    for (int i = 0; i < 12; ++i) {
      if (std::fabs(e.total) > std::fabs(worst[i].total)) {
        for (int j = 11; j > i; --j) worst[j] = worst[j-1];
        worst[i] = e;
        break;
      }
    }

    if (step % 100 == 0) {
      printf("  %6.1f %9.2f %9.5f %9.5f %9.5f %10.3f %10.2e %11.2e%s\n",
             t, yaw, st.x[B_GZ], st.x[B_EWZ], st.x[WZ],
             std::sqrt(std::max(st.P(QZ,QZ), 0.0)) * 2.0 * 180.0 / M_PI,
             st.P(QZ,QZ), st.P(QZ,B_GZ), (t > T_GPS ? "  BLK" : ""));
    }

    // Fine trace around a window, to see the gain that lets a VZ=0 update
    // rotate heading. K(QZ) from a scalar VZ measurement is
    // P(QZ,VZ) / (P(VZ,VZ) + R), so a big cross term against a small
    // denominator turns a millimetre-per-second innovation into degrees.
    if (fine_from > 0.0 && t >= fine_from && t <= fine_from + 0.30) {
      printf("    %8.3f yaw %9.3f  d_gnd %8.3f | P(QZ,VZ) %10.2e  P(VZ,VZ) %10.2e"
             "  ratio %9.2e | P(QZ,QZ) %9.2e  vz %9.2e\n",
             t, yaw, e.d_gnd, st.P(QZ,VZ), st.P(VZ,VZ),
             st.P(QZ,VZ) / std::max(st.P(VZ,VZ), 1e-12), st.P(QZ,QZ), st.x[VZ]);
    }
  }

  printf("\n  12 largest SINGLE-STEP yaw changes (deg), and what moved them:\n");
  printf("  %8s %10s %10s %10s %10s\n", "t", "total", "predict+imu", "encoder", "ground");
  for (int i = 0; i < 12; ++i) {
    if (worst[i].t == 0.0) continue;
    printf("  %8.2f %10.3f %10.3f %10.3f %10.3f\n",
           worst[i].t, worst[i].total, worst[i].d_imu, worst[i].d_enc, worst[i].d_gnd);
  }
  printf("\n  A single step is 10 ms. Anything above ~0.1 deg in one step is not\n");
  printf("  integration of a 0.01 rad/s bias (that would be 0.006 deg/step).\n");
  return 0;
}
