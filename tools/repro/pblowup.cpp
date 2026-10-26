// What doubles the covariance at t = 362 s in the degenerate twin?
//
// Rather than logging every if/else, log WHICH BLOCK OF P MOVES FIRST. The state
// vector is partitioned, so the first block whose variance departs localises the
// mechanism without guessing which branch to instrument.
//
// Reads an exported sensor log, replays it through fusioncore_core, and prints the
// diagonal blocks of P over a time window.
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"
using namespace fusioncore;

static double yaw_sigma_deg(const State& s) {
  // 1-sigma of yaw from the quaternion covariance, via the small-angle Jacobian
  // d(yaw)/d(QZ) = 2 near identity. Reported as an actual measurement rather than
  // inferred from path ratio.
  const double qw = s.x[QW], qz = s.x[QZ];
  const double denom = qw*qw + qz*qz;
  if (denom < 1e-12) return -1.0;
  const double dyaw_dqz = 2.0 * qw / denom;
  const double dyaw_dqw = -2.0 * qz / denom;
  const double var = dyaw_dqz*dyaw_dqz*s.P(QZ,QZ)
                   + dyaw_dqw*dyaw_dqw*s.P(QW,QW)
                   + 2.0*dyaw_dqz*dyaw_dqw*s.P(QW,QZ);
  return var > 0 ? std::sqrt(var) * 180.0 / M_PI : 0.0;
}

int main(int argc, char** argv) {
  if (argc < 4) { std::fprintf(stderr, "usage: pblowup <log> <q_angular_vel> <t_lo> <t_hi> [tangent] [freeze]\n"); return 2; }
  const std::string path = argv[1];
  const double q   = std::atof(argv[2]);
  const double tlo = argc > 3 ? std::atof(argv[3]) : 340.0;
  const double thi = argc > 4 ? std::atof(argv[4]) : 380.0;
  const bool tangent = argc > 5 ? std::atoi(argv[5]) != 0 : true;
  const bool freeze  = argc > 6 ? std::atoi(argv[6]) != 0 : false;
  const bool adapt   = argc > 7 ? std::atoi(argv[7]) != 0 : true;
  const bool qscale  = argc > 8 ? std::atoi(argv[8]) != 0 : false;
  const double rceil = argc > 9 ? std::atof(argv[9]) : 0.0;

  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x=cfg.imu.gyro_noise_y=cfg.imu.gyro_noise_z=0.003;  // NCLT config
  cfg.imu.accel_noise_x=cfg.imu.accel_noise_y=cfg.imu.accel_noise_z=0.1;
  cfg.imu_has_magnetometer=false;
  cfg.outlier_rejection=true;
  cfg.adaptive_imu=cfg.adaptive_encoder=cfg.adaptive_gnss=adapt;
  cfg.motion_model=create_motion_model("DifferentialDrive");
  cfg.ukf.tangent_space_attitude=tangent;
  cfg.ukf.tangent_space_quaternion_mean=tangent;
  if (freeze) { cfg.ukf.q_acceleration=0; cfg.ukf.q_accel_bias=0;
                cfg.ukf.q_gyro_bias=0;    cfg.ukf.q_encoder_wz_bias=0; }
  cfg.ukf.q_orientation=1e-9;
  cfg.ukf.q_angular_vel=q;
  cfg.ukf.q_time_scaled=qscale;
  cfg.adaptive_r_max_scale=rceil;
  cfg.adaptive_subtract_hph=(rceil>0.0);

  FusionCore fc(cfg);
  State s_init;
  if (freeze) { s_init.P(B_GX,B_GX)=1e-12; s_init.P(B_GY,B_GY)=1e-12; s_init.P(B_GZ,B_GZ)=1e-12;
                s_init.P(B_AX,B_AX)=1e-12; s_init.P(B_AY,B_AY)=1e-12; s_init.P(B_AZ,B_AZ)=1e-12;
                s_init.P(B_EWZ,B_EWZ)=1e-12; }
  bool inited=false; double t0=-1;
  std::ifstream in(path);
  std::string line;
  std::printf("# q_angular_vel=%g window %.0f..%.0f s\n", q, tlo, thi);
  std::printf("# tangent=%d freeze=%d adaptive=%d qscale=%d rceil=%.1f  R_gz=%.3e rad^2/s^2\n",
              (int)tangent, (int)freeze, (int)adapt, (int)qscale, rceil,
              cfg.imu.gyro_noise_z*cfg.imu.gyro_noise_z);
  std::printf("# %9s %10s %12s %12s %12s %9s %12s\n",
              "t_rel","yaw_sig","var_WZ","var_BGZ","var_SUM","r_wb","sum/wz");
  double last_print=-1;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string kind; ss >> kind;
    double t;
    if (kind=="IMU") {
      double wx,wy,wz,ax,ay,az; ss >> t >> wx >> wy >> wz >> ax >> ay >> az;
      if (t0<0) t0=t;
      if (!inited) { fc.init(s_init, t); inited=true; }
      fc.update_imu(t,wx,wy,wz,ax,ay,az);
    } else if (kind=="ENC") {
      double vx,vy,wz; ss >> t >> vx >> vy >> wz;
      if (!inited) continue;
      fc.update_encoder(t,vx,vy,wz);
    } else if (kind=="GNSS") {
      double e,n,u,hd,vd,sats,ft; ss >> t >> e >> n >> u >> hd >> vd >> sats >> ft;
      if (!inited) continue;
      sensors::GnssFix fix; fix.x=e; fix.y=n; fix.z=u;
      fix.hdop=hd; fix.vdop=vd; fix.sigma_xy=hd; fix.sigma_z=vd;
      fix.satellites=(int)sats;
      fix.fix_type=(sensors::GnssFixType)(int)ft;
      fc.update_gnss(t,fix);
    } else continue;
    if (t0<0) continue;
    const double tr = t - t0;
    if (tr < tlo || tr > thi) { if (tr > thi) break; continue; }
    if (tr - last_print < 0.5) continue;
    last_print = tr;
    const State& s = fc.get_state();
    auto blk=[&](int a,int b){ double v=0; for(int i=a;i<=b;++i) v+=s.P(i,i); return v; };
    FusionCoreStatus st = fc.get_status();
    (void)blk; (void)st;
    const double pww = s.P(WZ,WZ), pbb = s.P(B_GZ,B_GZ), pwb = s.P(WZ,B_GZ);
    const double vsum = pww + pbb + 2.0*pwb;
    const double r = (pww>0 && pbb>0) ? pwb/std::sqrt(pww*pbb) : 0.0;
    std::printf("  %9.2f %10.4f %12.6e %12.6e %12.6e %9.4f %12.3e\n",
                tr, yaw_sigma_deg(s), pww, pbb, vsum, r, pww>0 ? vsum/pww : 0.0);
  }
  return 0;
}
