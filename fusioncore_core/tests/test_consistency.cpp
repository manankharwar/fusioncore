#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"

using namespace fusioncore;

// Filter consistency: is the covariance this filter reports honest?
//
// Every other test here asks whether the estimate is close to truth. None of
// them ask whether the filter's own confidence matches the error it actually
// makes. That gap is how ukf.hpp's alpha=0.1 survived from May to August: the
// filter reported a tight covariance and a perfect heading while driving
// position backwards, and no test compared the two.
//
// Two standard measures, both chi-squared distributed when the filter is right:
//
//   NIS  = nu^T S^-1 nu    innovation vs its predicted covariance.
//                          Needs no ground truth, so it also runs on real data.
//                          E[NIS] = measurement dimension (3 for GNSS x/y/z).
//
//   NEES = e^T P^-1 e      estimation error vs the reported state covariance.
//                          Needs ground truth, so simulation only.
//                          E[NEES] = state dimension of the block tested.
//
// Reading the result:
//   NEES ~ n   the covariance is honest
//   NEES >> n  OVERCONFIDENT. P is too small, the filter believes itself more
//              than it deserves, and the Kalman gain is too low to correct the
//              error it is making. This is the dangerous direction.
//   NEES << n  underconfident. Safe but wasteful: good measurements get
//              under-weighted because P is bloated.
namespace {

// Wilson-Hilferty: chi-squared quantile without pulling in a stats library.
// Accurate to well under a percent for the DOF counts used here (k >= 90).
double chi2_quantile(double z, double k) {
  const double a = 2.0 / (9.0 * k);
  return k * std::pow(1.0 - a + z * std::sqrt(a), 3.0);
}
constexpr double Z_LO = -2.575829;   // 0.5th percentile
constexpr double Z_HI =  2.575829;   // 99.5th percentile

// Declared sensor sigmas. The injected noise uses exactly these, so any
// mismatch the test reports belongs to the filter, not to the setup.
constexpr double kGnssSigmaXY = 1.0;    // m
constexpr double kGnssSigmaZ  = 1.0;    // m
constexpr double kGyroSigma   = 0.005;  // rad/s
constexpr double kAccelSigma  = 0.1;    // m/s^2
constexpr double kEncSigmaV   = 0.05;   // m/s
constexpr double kEncSigmaWz  = 0.02;   // rad/s
constexpr double kMagSigmaRad = 0.05;   // rad, about 3 deg

FusionCoreConfig consistency_config(double alpha, double gnss_sigma_xy) {
  FusionCoreConfig cfg;
  cfg.ukf.alpha = alpha;
  cfg.imu.gyro_noise_x  = cfg.imu.gyro_noise_y  = cfg.imu.gyro_noise_z  = kGyroSigma;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = kAccelSigma;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = kEncSigmaV;
  cfg.encoder.vel_noise_wz = kEncSigmaWz;
  cfg.gnss.base_noise_xy = gnss_sigma_xy;
  cfg.gnss.base_noise_z  = kGnssSigmaZ;

  // Adaptive noise off on purpose: it rescales R at runtime, so the filter would
  // no longer be using the R this test injects against, and NIS would be
  // measuring the adaptation rather than the filter.
  cfg.adaptive_imu = cfg.adaptive_encoder = cfg.adaptive_gnss = false;

  // Rejection stays ON because the Mahalanobis distance is only computed inside
  // that branch, and it is the NIS this test reads. At the default 16.27, the
  // 99.9th percentile of chi-squared with 3 DOF, it censors about one sample in
  // a thousand, which does not move the mean.
  cfg.outlier_rejection = true;
  cfg.outlier_threshold_gnss = 16.27;

  cfg.mag.noise_rad      = kMagSigmaRad;
  cfg.mag.chi2_threshold = 9.21;      // chi2(1, 0.99), the 1-DOF gate
  cfg.mag.declination_rad = 0.0;

  cfg.motion_model = create_motion_model("DifferentialDrive");
  return cfg;
}

sensors::GnssFix make_fix(double x, double y, double z) {
  sensors::GnssFix fix;
  fix.x = x; fix.y = y; fix.z = z;
  fix.hdop = 1.0; fix.vdop = 1.0;
  fix.satellites = 12;
  fix.fix_type = sensors::GnssFixType::DGPS_FIX;
  return fix;
}

struct RunResult {
  std::vector<double> nis;              // one per GNSS fix, after burn-in
  std::vector<double> nees_pos;         // sampled at 5 Hz, after burn-in
  std::vector<double> nees_pos_blackout;// same, restricted to the blackout window
  double max_err_m               = 0.0;
  double final_heading_sigma_deg = 0.0;
};

// Straight East at 1 m/s, flat ground. IMU 100 Hz, encoder + ground constraint
// 50 Hz, GNSS 5 Hz. Every sensor is corrupted with exactly the noise the filter
// was told to expect, so any mismatch reported belongs to the filter and not to
// the setup. Statistics start after burn_in_s so the initial covariance has
// settled. NEES is sampled on a timer rather than on GNSS fixes, because the
// interesting case is a blackout, when there are no fixes.
// Scenario knobs. Defaults describe a well-conditioned run: clean 5 Hz GNSS at
// 1 m sigma, which is the case where the filter should look its best.
struct Scenario {
  double   duration_s      = 60.0;
  double   burn_in_s       = 20.0;
  double   alpha           = 1.0;
  double   blackout_start  = -1.0;
  double   blackout_end    = -1.0;
  bool     use_mag         = false;
  double   gnss_sigma_xy   = kGnssSigmaXY;
  int      gnss_period_steps = 20;    // 20 steps at dt=0.01 is 5 Hz
};

RunResult run(unsigned seed, const Scenario& sc) {
  const double duration_s = sc.duration_s, burn_in_s = sc.burn_in_s;
  const double alpha = sc.alpha;
  const double gnss_blackout_start = sc.blackout_start;
  const double gnss_blackout_end   = sc.blackout_end;
  const bool   use_mag = sc.use_mag;
  std::mt19937 rng(seed);
  std::normal_distribution<double> n01(0.0, 1.0);

  FusionCore fc(consistency_config(alpha, sc.gnss_sigma_xy));
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, speed = 1.0, g = 9.80665;
  RunResult out;

  for (int step = 1; step * dt <= duration_s + 1e-9; ++step) {
    const double t = step * dt;
    const double true_x = speed * t, true_y = 0.0, true_z = 0.0;

    fc.update_imu(t,
                  kGyroSigma * n01(rng), kGyroSigma * n01(rng), kGyroSigma * n01(rng),
                  kAccelSigma * n01(rng), kAccelSigma * n01(rng), g + kAccelSigma * n01(rng));

    if (step % 2 == 0) {
      fc.update_encoder(t, speed + kEncSigmaV * n01(rng), 0.0, kEncSigmaWz * n01(rng));
      fc.update_ground_constraint(t);
    }

    // Absolute heading at 20 Hz. True yaw is 0 throughout this run, so the
    // reading is that plus its own noise. mx = sin(yaw), my = cos(yaw) is the
    // convention update_magnetometer expects.
    if (use_mag && step % 5 == 0) {
      const double yaw_meas = kMagSigmaRad * n01(rng);
      fc.update_magnetometer(t, std::sin(yaw_meas), std::cos(yaw_meas), 0.0);
    }

    const bool blacked_out = (gnss_blackout_start >= 0.0 &&
                              t >= gnss_blackout_start && t < gnss_blackout_end);

    if (step % sc.gnss_period_steps == 0 && !blacked_out) {
      fc.update_gnss(t, make_fix(true_x + sc.gnss_sigma_xy * n01(rng),
                                 true_y + sc.gnss_sigma_xy * n01(rng),
                                 true_z + kGnssSigmaZ    * n01(rng)));
      if (t >= burn_in_s) {
        const auto& dbg = fc.get_gnss_debug();
        if (dbg.mahalanobis_sq >= 0.0) out.nis.push_back(dbg.mahalanobis_sq);
      }
    }

    if (step % sc.gnss_period_steps == 0 && t >= burn_in_s) {
      // NEES on the position block only. The full 23-state vector cannot be used
      // directly: the quaternion is 4 numbers carrying 3 degrees of freedom, so
      // its P block is singular by construction, and the bias states have no
      // ground truth to difference against.
      const State& st = fc.get_state();
      Eigen::Vector3d e(st.x[X] - true_x, st.x[Y] - true_y, st.x[Z] - true_z);
      Eigen::Matrix3d P_pos = st.P.block<3,3>(X, X);
      const double nees = e.transpose() * P_pos.inverse() * e;
      out.nees_pos.push_back(nees);
      if (blacked_out) out.nees_pos_blackout.push_back(nees);
      out.max_err_m = std::max(out.max_err_m, e.norm());
    }
  }

  const State& st = fc.get_state();
  out.final_heading_sigma_deg = std::sqrt(std::max(0.0, st.P(QZ, QZ))) * 2.0 * 180.0 / M_PI;
  return out;
}

double mean(const std::vector<double>& v) {
  double t = 0.0;
  for (double x : v) t += x;
  return v.empty() ? 0.0 : t / static_cast<double>(v.size());
}

// Blackout NEES is heavy-tailed: across 16 seeds it ran from 0.5 to 24.3, so one
// unlucky seed drags the mean by a factor of four and a 6-run mean says almost
// nothing. tools/compare_runs.py already judges NCLT runs by median and p90 for
// the same reason. Same rule here.
double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const double idx = q * (static_cast<double>(v.size()) - 1.0);
  const size_t lo = static_cast<size_t>(std::floor(idx));
  const size_t hi = static_cast<size_t>(std::ceil(idx));
  return v[lo] + (v[hi] - v[lo]) * (idx - static_cast<double>(lo));
}

} // namespace

// Measurement pass. Prints the numbers this filter actually produces so the
// thresholds below are set from observation rather than from hope.
// ─── With GPS up, the filter must not claim more accuracy than it has ───────
// The dangerous direction is NEES above 3: P smaller than the error actually
// made, so the gain is too low to correct that error and the filter goes on
// believing a wrong answer. Below 3 is the safe direction, bounded only loosely
// here to catch a P that has exploded.
//
// These are engineering bounds, not strict chi-squared intervals. NEES is
// sampled at 5 Hz and successive samples are correlated, so the effective sample
// count is far below the nominal one and the textbook interval would be too
// narrow. Seeds are fixed, so the numbers reproduce run to run.
//
// Measured on 0.3.7: NIS 2.342, NEES 1.958. Both sit below 3, so the reported
// covariance is roughly 1.5x larger than the errors warrant. Conservative rather
// than dangerous, though it does inflate the Kalman gain, which makes the filter
// track GPS noise more closely than it needs to.
TEST(ConsistencyTest, SteadyStateCovarianceIsNotOverconfident) {
  const int kRuns = 6;
  std::vector<double> nis, nees;
  for (int i = 0; i < kRuns; ++i) {
    RunResult r = run(1000 + i, Scenario{});
    nis.insert(nis.end(), r.nis.begin(), r.nis.end());
    nees.insert(nees.end(), r.nees_pos.begin(), r.nees_pos.end());
  }
  ASSERT_GT(nis.size(),  500u);
  ASSERT_GT(nees.size(), 500u);
  const double nis_mean = mean(nis), nees_mean = mean(nees);

  EXPECT_LT(nees_mean, 3.5)
      << "OVERCONFIDENT: position error exceeds what the reported covariance "
         "allows (NEES " << nees_mean << ", honest is near 3.0)";
  EXPECT_LT(nis_mean, 3.5)
      << "OVERCONFIDENT: GNSS innovations exceed what S predicts (NIS "
      << nis_mean << ", honest is near 3.0)";
  EXPECT_GT(nees_mean, 1.2) << "covariance inflated beyond the error being made "
                               "(NEES " << nees_mean << ")";
  EXPECT_GT(nis_mean, 1.2)  << "innovation covariance inflated (NIS " << nis_mean << ")";
}

// ─── Dead reckoning, and the sigma-weight fix that only shows up there ──────
// Both properties come from one sweep because the runs are the expensive part.
//
// The alpha=0.1 bug (at 23 states with kappa=0 the centre sigma weight is -99)
// was invisible for months, and this test shows why: with GPS up it changes
// nothing measurable, NIS and NEES match to three decimals across alpha 0.1,
// 0.5 and 1.0. It only bites once the quaternion sigma points spread, which
// needs dead reckoning.
//
// Measured across 16 seeds, median blackout NEES:
//     alpha 0.1   6.50   overconfident by about 2x, the dangerous direction
//     alpha 1.0   1.35   conservative, the safe direction
// Judged on medians on purpose. The per-seed spread is 0.5 to 24.3, so a mean
// over a handful of seeds is dominated by whichever tail it happened to draw.
TEST(ConsistencyTest, BlackoutCovarianceAndSigmaWeights) {
  const int kRuns = 16;
  std::vector<double> good, bad;
  for (int i = 0; i < kRuns; ++i) {
    Scenario g; g.duration_s = 120.0; g.blackout_start = 30.0; g.blackout_end = 90.0;
    Scenario b = g; b.alpha = 0.1;
    good.push_back(mean(run(2000 + i, g).nees_pos_blackout));
    bad .push_back(mean(run(2000 + i, b).nees_pos_blackout));
  }
  const double good_med = percentile(good, 0.5), good_p90 = percentile(good, 0.9);
  const double bad_med  = percentile(bad,  0.5);

  printf("\n  blackout NEES over %d seeds (3.0 is honest)\n"
         "    alpha 1.0  median %.2f  p90 %.2f\n"
         "    alpha 0.1  median %.2f\n\n",
         kRuns, good_med, good_p90, bad_med);

  EXPECT_LT(good_med, 3.5)
      << "OVERCONFIDENT while dead reckoning: median NEES " << good_med
      << ". If P is too small during a blackout, the gain when GPS returns is "
         "too small to pull the estimate back, which is what slow re-acquisition "
         "after an outage looks like.";

  EXPECT_GT(bad_med, good_med * 2.0)
      << "alpha 0.1 should be clearly less consistent than alpha 1.0, but "
         "measured median NEES " << bad_med << " vs " << good_med
      << ". If these have converged, the sigma-weight scaling changed and the "
         "0.3.7 fix may have been undone.";
}

// ─── Does an absolute heading source actually repair the covariance? ────────
// Measured before the hardware has it, because the field runs say the covariance
// is 88x to 171x underconfident and the diagnosis is unobservable yaw. If a
// magnetometer is the fix, NIS should climb back toward 3 and the heading sigma
// should collapse. If it does not, that is worth knowing before spending a field
// day on it.
TEST(ConsistencyTest, MagnetometerRepairsTheCovariance) {
  const int kRuns = 5;
  std::vector<double> nis_off, nis_on, nees_off, nees_on;
  double hdg_off = 0.0, hdg_on = 0.0;

  for (int i = 0; i < kRuns; ++i) {
    Scenario off_sc; off_sc.duration_s = 90.0;
    Scenario on_sc = off_sc; on_sc.use_mag = true;
    RunResult off = run(3000 + i, off_sc);
    RunResult on  = run(3000 + i, on_sc);
    nis_off.insert(nis_off.end(), off.nis.begin(), off.nis.end());
    nis_on .insert(nis_on .end(), on .nis.begin(), on .nis.end());
    nees_off.insert(nees_off.end(), off.nees_pos.begin(), off.nees_pos.end());
    nees_on .insert(nees_on .end(), on .nees_pos.begin(), on .nees_pos.end());
    hdg_off += off.final_heading_sigma_deg;
    hdg_on  += on .final_heading_sigma_deg;
  }

  printf("\n  magnetometer off/on, %d runs (honest NIS and NEES are 3.0)\n"
         "    NIS   %.2f -> %.2f\n"
         "    NEES  %.2f -> %.2f\n"
         "    heading 2-sigma  %.1f -> %.1f deg\n\n",
         kRuns, mean(nis_off), mean(nis_on), mean(nees_off), mean(nees_on),
         hdg_off / kRuns, hdg_on / kRuns);

  EXPECT_LT(hdg_on / kRuns, hdg_off / kRuns)
      << "an absolute heading source should reduce heading uncertainty, but "
         "measured " << hdg_on / kRuns << " deg with it against "
      << hdg_off / kRuns << " deg without";
}

// ─── An absolute heading source keeps the covariance honest as GNSS degrades ──
// Without one, the filter grows steadily more overconfident as GNSS noise rises:
// NEES climbs from about 2 at clean 1 m fixes to about 7 at 10 m fixes, meaning
// it claims an accuracy it no longer has. A magnetometer supplying absolute yaw
// holds NEES near 2 across the whole range and pins heading under a degree.
//
// This is the measured case for adding one, and it is also the honest limit of
// what it buys: it fixes the part of the covariance error that comes from
// unobservable yaw. It does nothing about a receiver whose reported covariance
// describes absolute accuracy while its consecutive fixes agree to centimetres,
// which is a property of the measurement. See docs/guides/filter-consistency.md.
TEST(ConsistencyTest, MagnetometerHoldsCovarianceAsGnssDegrades) {
  const int kRuns = 4;
  struct Case { const char* name; double sigma; int period; };
  const Case cases[] = {
    {"clean  1.0 m @5Hz",  1.0,  20},
    {"noisy  3.0 m @1Hz",  3.0, 100},
    {"rover  6.6 m @1Hz",  6.6, 100},
    {"poor  10.0 m @1Hz", 10.0, 100},
  };

  printf("\n  %-19s %10s %10s %12s %10s\n",
         "GNSS quality", "NEES", "NEES+mag", "hdg2sig", "hdg2sig+mag");
  double worst_no_mag = 0.0, worst_with_mag = 0.0;
  for (const auto& c : cases) {
    double nees[2] = {0, 0}, hdg[2] = {0, 0};
    for (int m = 0; m < 2; ++m) {
      std::vector<double> acc;
      for (int i = 0; i < kRuns; ++i) {
        Scenario sc;
        sc.duration_s = 110.0; sc.burn_in_s = 30.0;
        sc.gnss_sigma_xy = c.sigma; sc.gnss_period_steps = c.period;
        sc.use_mag = (m == 1);
        RunResult r = run(4000 + i, sc);
        acc.insert(acc.end(), r.nees_pos.begin(), r.nees_pos.end());
        hdg[m] += r.final_heading_sigma_deg;
      }
      nees[m] = mean(acc);
    }
    printf("  %-19s %10.2f %10.2f %9.1f deg %6.1f deg\n",
           c.name, nees[0], nees[1], hdg[0] / kRuns, hdg[1] / kRuns);
    worst_no_mag   = std::max(worst_no_mag,   nees[0]);
    worst_with_mag = std::max(worst_with_mag, nees[1]);
  }
  printf("  (3.0 is honest; above it means overconfident)\n\n");

  EXPECT_GT(worst_no_mag, 4.0)
      << "without an absolute heading source the filter should become clearly "
         "overconfident as GNSS degrades, but the worst NEES was only "
      << worst_no_mag << ". If this has improved, the scenario or the filter "
         "changed and the magnetometer case below needs rechecking.";
  EXPECT_LT(worst_with_mag, 3.0)
      << "with a magnetometer the covariance should stay honest across all GNSS "
         "qualities, but the worst NEES was " << worst_with_mag;
}
