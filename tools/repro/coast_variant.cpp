// Coast variant on a synthetic blackout with known truth.
//
// Question: during a GNSS coast, does taking speed only from the wheels and heading
// only from the gyro drift less than the shipped coast? Predictions were written
// first, in coast_variant_predictions.md.
//
// Truth turns and changes speed, so the accelerometer, gyro and encoder all carry
// information and all carry error: gyro bias, accelerometer bias, an encoder speed
// scale error and an encoder yaw-rate scale error plus bias. Every seed draws the
// signs of those errors at random so no arm is tuned to one direction.
//
// Scored the way tools/gap_deadreckon.py scores NCLT: the error is the difference
// between the estimate's displacement over the gap and the truth's, as a percentage
// of distance travelled. Dead reckoning from the same sensor stream, anchored at the
// filter's own heading at gap entry, is computed alongside so the arms can be read
// against the same bar as on real data.
//
//   coast_variant [seeds] [accel_bias m/s^2] [gyro_bias rad/s]
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <random>
#include <vector>
#include <algorithm>
#include <string>
#include "fusioncore/fusioncore.hpp"
using namespace fusioncore;

struct Arm { const char* name; double imu_wz, enc_wz, q_bias = 100.0, q_pos = 10.0, hold = 0.0; };

struct Errors {
  double gyro_bias, accel_bias_x, accel_bias_y, enc_scale, enc_wz_scale, enc_wz_bias;
};

static const double DT = 0.01, G = 9.80665;
static const double GAP0 = 120.0, GAP1 = 420.0;

static double v_true(double t)  { return 1.0 + 0.3 * std::sin(2 * M_PI * t / 40.0); }
static double dv_true(double t) { return 0.3 * 2 * M_PI / 40.0 * std::cos(2 * M_PI * t / 40.0); }
static double w_true(double t)  { return 0.15 * std::sin(2 * M_PI * t / 90.0); }

static double yaw_of(const StateVector& x) {
  return std::atan2(2 * (x[QW] * x[QZ] + x[QX] * x[QY]),
                    1 - 2 * (x[QY] * x[QY] + x[QZ] * x[QZ]));
}

struct Result { double filt, dr_gyro, dr_enc, dist, path, along, yaw_sig_deg; };

static Result run(const Arm& arm, const Errors& e, unsigned seed) {
  FusionCoreConfig cfg;  // shipped NCLT values where they differ from the defaults
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.003;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.gnss.base_noise_xy = 1.0;
  cfg.outlier_rejection = true;
  cfg.adaptive_imu = cfg.adaptive_encoder = cfg.adaptive_gnss = true;
  cfg.gnss_coast_n = 3;
  cfg.gnss_coast_q_factor = arm.q_pos;
  cfg.gnss_coast_timeout_s = 30.0;
  cfg.gnss_coast_q_bias_factor = arm.q_bias;
  cfg.gnss_coast_imu_wz_scale = arm.imu_wz;
  cfg.gnss_coast_encoder_wz_scale = arm.enc_wz;
  cfg.gnss_coast_hold_accel_sigma = arm.hold;

  FusionCore fc(cfg);
  fc.init(State{}, 0.0);
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, 1.0);

  double tx = 0, ty = 0, tyaw = 0;              // truth
  double gx0 = 0, gy0 = 0, ex0 = 0, ey0 = 0;    // filter and truth at gap entry
  double drg_x = 0, drg_y = 0, drg_yaw = 0;     // DR, gyro heading
  double dre_x = 0, dre_y = 0, dre_yaw = 0;     // DR, encoder heading
  double dist = 0, path = 0, px = 0, py = 0;
  bool entered = false;

  for (int k = 1; k * DT <= GAP1 + 1e-9; ++k) {
    const double t = k * DT, v = v_true(t), w = w_true(t);
    tyaw += w * DT;
    tx += v * std::cos(tyaw) * DT;
    ty += v * std::sin(tyaw) * DT;

    const double gz = w + e.gyro_bias + 0.003 * n(rng);
    const double ax = dv_true(t) + e.accel_bias_x + 0.1 * n(rng);
    const double ay = v * w + e.accel_bias_y + 0.1 * n(rng);
    const double az = G + 0.1 * n(rng);
    const double ev = v * (1 + e.enc_scale) + 0.05 * n(rng);
    const double ew = w * (1 + e.enc_wz_scale) + e.enc_wz_bias + 0.02 * n(rng);

    fc.update_imu(t, 0.003 * n(rng), 0.003 * n(rng), gz, ax, ay, az);
    fc.update_encoder(t, ev, 0.0, ew);

    if (t < GAP0 && k % 100 == 0) {
      sensors::GnssFix f;
      f.x = tx + 1.5 * n(rng); f.y = ty + 1.5 * n(rng); f.z = 0.0;
      f.hdop = f.sigma_xy = 1.5; f.vdop = f.sigma_z = 2.5;
      f.satellites = 10; f.fix_type = sensors::GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
    }

    if (t >= GAP0) {
      if (!entered) {
        const auto& x = fc.get_state().x;
        gx0 = x[X]; gy0 = x[Y];
        ex0 = tx; ey0 = ty;
        drg_yaw = dre_yaw = yaw_of(x);
        px = x[X]; py = x[Y];
        entered = true;
      }
      { const auto& x = fc.get_state().x; path += std::hypot(x[X] - px, x[Y] - py); px = x[X]; py = x[Y]; }
      drg_yaw += gz * DT;
      dre_yaw += ew * DT;
      drg_x += ev * std::cos(drg_yaw) * DT; drg_y += ev * std::sin(drg_yaw) * DT;
      dre_x += ev * std::cos(dre_yaw) * DT; dre_y += ev * std::sin(dre_yaw) * DT;
      dist += v * DT;
    }
  }
  const auto& x = fc.get_state().x;
  const double dtx = tx - ex0, dty = ty - ey0;
  Result r;
  r.filt    = std::hypot((x[X] - gx0) - dtx, (x[Y] - gy0) - dty);
  r.dr_gyro = std::hypot(drg_x - dtx, drg_y - dty);
  r.dr_enc  = std::hypot(dre_x - dtx, dre_y - dty);
  r.dist    = dist;
  r.path    = path / dist;
  {
    const double ux = dtx / std::hypot(dtx, dty), uy = dty / std::hypot(dtx, dty);
    r.along = ((x[X] - gx0) * ux + (x[Y] - gy0) * uy) / std::hypot(dtx, dty);
  }
  {
    const auto& P = fc.get_state().P;
    r.yaw_sig_deg = 2.0 * std::sqrt(std::max(0.0, P(QZ, QZ))) * 180.0 / M_PI;
  }
  return r;
}

int main(int argc, char** argv) {
  const int seeds = argc > 1 ? std::atoi(argv[1]) : 10;
  const double ab = argc > 2 ? std::atof(argv[2]) : 0.05;
  const double gb = argc > 3 ? std::atof(argv[3]) : 0.005;

  const std::vector<Arm> arms = {
    {"base",            500.0, 1.0},               // shipped NCLT coast
    {"H gyro heading",    1.0, 1e6},               // heading from gyro only
    {"q_bias 1",        500.0, 1.0, 1.0},          // no 100x gyro-bias Q in coast
    {"q_pos 1",         500.0, 1.0, 100.0, 1.0},   // no 10x position Q in coast
    {"q_bias 1, q_pos 1", 500.0, 1.0, 1.0, 1.0},
    {"hold accel 0.01",   500.0, 1.0, 100.0, 10.0, 0.01},  // AX/AY/AZ held at zero in coast
    {"hold accel 0.1",    500.0, 1.0, 100.0, 10.0, 0.1},
  };

  std::printf("  %d seeds, %.0f s gap after %.0f s of 1 Hz GNSS. accel bias %.3f m/s^2,"
              " gyro bias %.4f rad/s, encoder speed scale 2%%, encoder WZ scale 5%%"
              " plus 0.002 rad/s bias. Signs random per seed.\n",
              seeds, GAP1 - GAP0, GAP0, ab, gb);
  std::printf("  err = |estimated displacement - true displacement| over the gap, %% of distance.\n"
              "  path = filter path length / true path length over the gap.\n"
              "  along = filter displacement projected on the true displacement, / its length.\n"
              "  yaw2s = approx 2-sigma yaw from P(QZ,QZ) at gap exit, degrees.\n\n");

  struct Acc { std::vector<double> err, path, along, ys; };
  std::vector<Acc> acc(arms.size());
  std::vector<double> drg, dre;
  std::mt19937 er(777);
  std::uniform_int_distribution<int> sgn(0, 1);
  auto s = [&]() { return sgn(er) ? 1.0 : -1.0; };
  for (int i = 0; i < seeds; ++i) {
    const Errors e{gb * s(), ab * s(), ab * s(), 0.02 * s(), 0.05 * s(), 0.002 * s()};
    for (size_t a = 0; a < arms.size(); ++a) {
      Result r = run(arms[a], e, 1000 + i);
      acc[a].err.push_back(100.0 * r.filt / r.dist);
      acc[a].path.push_back(r.path);
      acc[a].along.push_back(r.along);
      acc[a].ys.push_back(r.yaw_sig_deg);
      if (a == 0) {
        drg.push_back(100.0 * r.dr_gyro / r.dist);
        dre.push_back(100.0 * r.dr_enc / r.dist);
      }
    }
  }
  auto med = [](std::vector<double> v) { std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
  auto mean = [](const std::vector<double>& v) { double m = 0; for (double x : v) m += x; return m / v.size(); };
  std::printf("  %-22s %9s %9s %8s %8s %8s\n", "arm", "err mean", "err med", "path", "along", "yaw2s");
  for (size_t a = 0; a < arms.size(); ++a)
    std::printf("  %-22s %9.3f %9.3f %8.3f %8.3f %8.2f\n", arms[a].name,
                mean(acc[a].err), med(acc[a].err), med(acc[a].path), med(acc[a].along), med(acc[a].ys));
  std::printf("  %-22s %9.3f %9.3f\n", "DR gyro @filter yaw", mean(drg), med(drg));
  std::printf("  %-22s %9.3f %9.3f\n", "DR enc  @filter yaw", mean(dre), med(dre));
  std::printf("  (path, along and yaw2s columns are medians across seeds)\n");
  return 0;
}
