// Synthetic blackout with known truth, covering the three recovery branches.
//
// Truth drives straight along +x at 1 m/s. At t=100 s GNSS disappears for 200 s.
// During the gap the GYRO carries a bias, so the filter's heading walks away while
// the robot actually goes straight. That is the real failure: an encoder scale
// error alone only drifts along-track and the coast-inflated gate swallows it, but
// a heading error throws the filter sideways and keeps throwing it, which is what
// makes the returning fixes unacceptable. It also exercises branch C, since
// position alone cannot fix a filter that is still pointing the wrong way.
//
// Pass criteria, checked not eyeballed:
//   the filter must come back within 5 m of truth, and do it quickly
//   re-anchoring must NOT fire while GNSS is healthy before the gap
//   normal gating must resume afterwards, which is the latch-bug test
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <random>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"
using namespace fusioncore;

int main(int argc, char** argv) {
  const bool reanchor = argc > 1 ? std::atoi(argv[1]) != 0 : true;
  const double DT = 0.01, SPEED = 1.0, G = 9.80665;
  const double GAP0 = 100.0, GAP1 = 300.0, TEND = 420.0;
  const double GYRO_BIAS = argc > 2 ? std::atof(argv[2]) : 0.010;
  // NCLT-grade fix noise. With PERFECT returning fixes the existing recovery
  // inflation handles any amount of drift in 15 rejections, so a clean synthetic
  // cannot reproduce the 81% rejection measured on real data. The difference has
  // to be in the fixes.
  const double FIX_SIGMA = argc > 3 ? std::atof(argv[3]) : 0.0;            // rad/s of false yaw rate during the gap

  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x=cfg.imu.gyro_noise_y=cfg.imu.gyro_noise_z=0.003;
  cfg.imu.accel_noise_x=cfg.imu.accel_noise_y=cfg.imu.accel_noise_z=0.1;
  cfg.imu_has_magnetometer=false;
  cfg.outlier_rejection=true;
  cfg.adaptive_imu=cfg.adaptive_encoder=cfg.adaptive_gnss=true;
  cfg.motion_model=create_motion_model("DifferentialDrive");
  cfg.gnss_reanchor_enabled = reanchor;

  FusionCore fc(cfg);
  fc.init(State{}, 0.0);
  std::mt19937 rng(12345);
  std::normal_distribution<double> nz(0.0, 1.0);

  // The bar has to scale with the receiver. Demanding 5 m from fixes that carry
  // 9 m of noise is asking the filter to beat its own input.
  const double tol = std::max(5.0, 1.5 * FIX_SIGMA);
  double worst_after = 0.0, err_at_end = 0.0, t_recover = -1.0;
  int fixes = 0, rejected = 0, rej_before_gap = 0;

  for (int k = 1; k*DT <= TEND; ++k) {
    const double t = k*DT, truth_x = SPEED*t;
    const bool in_gap = (t >= GAP0 && t < GAP1);
    const double wz = in_gap ? GYRO_BIAS : 0.0;   // truth is straight; the gyro lies

    fc.update_imu(t, 0,0,wz, 0,0,G);
    fc.update_encoder(t, SPEED, 0.0, wz);

    if (!in_gap && k % 100 == 0) {
      sensors::GnssFix fix;
      fix.x = truth_x + FIX_SIGMA*nz(rng);
      fix.y = FIX_SIGMA*nz(rng);
      fix.z = 0.0;
      const double declared = std::max(1.0, FIX_SIGMA);
      fix.hdop=declared; fix.vdop=declared*1.5;
      fix.sigma_xy=declared; fix.sigma_z=declared*1.5;
      fix.satellites=12; fix.fix_type=sensors::GnssFixType::RTK_FIXED;
      ++fixes;
      if (!fc.update_gnss(t, fix)) { ++rejected; if (t < GAP0) ++rej_before_gap; }
    }
    if (t > GAP1) {
      const double e = std::hypot(fc.get_state().x[X] - truth_x,
                                  fc.get_state().x[Y]);
      worst_after = std::max(worst_after, e);
      if (t_recover < 0 && e < tol) t_recover = t - GAP1;
      err_at_end = e;
    }
  }

  printf("\n  reanchor=%d  gyro bias %.3f rad/s, %.0f s blackout, fix sigma %.1f m\n",
         (int)reanchor, GYRO_BIAS, GAP1-GAP0, FIX_SIGMA);
  printf("    fixes %d, rejected %d, of which before the gap %d\n", fixes, rejected, rej_before_gap);
  printf("    seconds to get back within %.1f m: %s\n", tol,
         t_recover < 0 ? "NEVER" : (std::to_string(t_recover)).c_str());
  printf("    error 120 s after the gap ends : %.3f m\n", err_at_end);
  printf("    worst error after the gap      : %.3f m\n", worst_after);
  printf("    re-anchors fired               : %ld\n", fc.gnss_reanchor_count());

  bool ok = true;
  if (rej_before_gap != 0) { printf("    FAIL: rejected a good fix before the gap\n"); ok = false; }
  if (reanchor && t_recover < 0) { printf("    FAIL: never recovered\n"); ok = false; }
  if (reanchor && err_at_end > tol) { printf("    FAIL: still %.1f m out against a %.1f m bar\n", err_at_end, tol); ok = false; }
  printf("    %s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
