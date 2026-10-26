// ADVERSARIAL: a consistent but WRONG GNSS track, with no blackout.
//
// Multipath beside a building can produce a smooth, self-consistent track that is
// simply offset. Its fix-to-fix displacements agree with each other AND with
// odometry, so a consistency check based only on mutual agreement passes, and the
// filter gets re-anchored onto a wrong position.
//
// Truth drives straight at 1 m/s with healthy GNSS throughout. From t=150 s to
// t=180 s every fix is offset by 20 m in +y, then the offset ends.
//
// The expected requirement was "no re-anchor, or at most one that is undone", on
// the reasoning that the filter has no cause to doubt itself here and a sudden
// consistent offset is more likely multipath than drift.
//
// MEASURED, and it inverts that. Without re-anchoring the filter does not merely
// resist the multipath, it NEVER RECOVERS FROM IT: 129.968 m out 40 s after the
// offset ends, from a 30 s offset with continuous healthy GNSS and no blackout at
// all. With re-anchoring it fires twice, once onto the offset and once back off
// it, and ends 1.226 m out.
//
// So the pass criterion is the OUTCOME, not the count. A guard that suppressed
// re-anchoring while the filter looks healthy would also suppress the rescue, and
// on this scenario that trade is 1.2 m against 130 m. The cost of the current
// behaviour is real and is a position JUMP, which matters to a downstream
// controller; that is measured as largest-jump on NCLT rather than assumed here.
//
// It is also a finding in its own right: the filter cannot recover from a
// transient multipath offset unaided. Same re-acquisition defect, no blackout
// required.
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
  const double OFF0 = 150.0, OFF1 = 180.0, TEND = 260.0, OFFSET = 20.0;

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
  std::mt19937 rng(777);
  std::normal_distribution<double> nz(0.0, 1.0);

  double worst_during = 0.0, err_end = 0.0;
  for (int k = 1; k*DT <= TEND; ++k) {
    const double t = k*DT, truth_x = SPEED*t;
    fc.update_imu(t, 0,0,0, 0,0,G);
    fc.update_encoder(t, SPEED, 0.0, 0.0);
    if (k % 100 == 0) {
      const bool bad = (t >= OFF0 && t < OFF1);
      sensors::GnssFix fix;
      fix.x = truth_x + 1.0*nz(rng);
      fix.y = (bad ? OFFSET : 0.0) + 1.0*nz(rng);
      fix.z = 0.0;
      fix.hdop=1.0; fix.vdop=1.5; fix.sigma_xy=1.0; fix.sigma_z=1.5;
      fix.satellites=12; fix.fix_type=sensors::GnssFixType::RTK_FIXED;
      fc.update_gnss(t, fix);
    }
    const double e = std::hypot(fc.get_state().x[X]-truth_x, fc.get_state().x[Y]);
    if (t >= OFF0 && t < OFF1) worst_during = std::max(worst_during, e);
    if (t > OFF1 + 40.0) err_end = e;
  }
  const long n = fc.gnss_reanchor_count();
  printf("\n  reanchor=%d   consistent %.0f m multipath offset for %.0f s, NO blackout\n",
         (int)reanchor, OFFSET, OFF1-OFF0);
  printf("    re-anchors fired            : %ld\n", n);
  printf("    worst error during offset   : %.3f m\n", worst_during);
  printf("    error 40 s after it ends    : %.3f m\n", err_end);
  // Judge the outcome. During the offset the filter is allowed to be pulled
  // about as far as the offset itself; what matters is that it comes back.
  bool ok = (worst_during < 2.0*OFFSET) && (err_end < 5.0);
  if (worst_during >= 2.0*OFFSET)
    printf("    FAIL: pulled %.1f m by a %.0f m offset\n", worst_during, OFFSET);
  if (err_end>=5.0) printf("    FAIL: still %.1f m out after the offset ended\n", err_end);
  printf("    %s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
