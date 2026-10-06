#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

namespace fusioncore {

// Measure the time offset between two sensors instead of only rejecting on it.
//
// FusionCore has always treated a clock disagreement as something to GUARD against:
// reject_stale_from_skew() drops a measurement whose stamp lags the filter clock by
// more than max_measurement_delay, and DELAY_TOO_LARGE counts how often that happens.
// That is safe and it throws the data away. Nothing here ever measured the offset so
// that it could be corrected instead.
//
// The offset is measurable without any extra hardware, because two sensors already
// report the same physical quantity. A gyro's z rate and a differential drive's
// encoder yaw rate are the same signal observed twice, so the lag that maximises
// their agreement IS the offset between their clocks.
//
// Method: normalised cross-correlation over a grid of candidate lags, then a
// parabolic refinement through the peak and its two neighbours, which gets below the
// grid step without a finer grid. Both series are resampled onto a common uniform
// grid first, because sensors at different rates cannot be correlated sample to
// sample.
//
// This deliberately does NOT apply the correction on its own. It reports an offset
// and a confidence, and a human decides whether to set encoder.time_offset. An
// estimator that silently shifted a sensor stream would be very hard to debug the
// first time it was wrong, and the whole reason this exists is that the previous
// behaviour was silent.
class TimeOffsetEstimator {
public:
  struct Result {
    bool   valid       = false;
    double offset_s    = 0.0;   // ADD this to the encoder stamps to align them
    double correlation = 0.0;   // peak normalised correlation, 0 to 1
    int    samples     = 0;     // overlapping resampled samples the peak used
    double span_s      = 0.0;   // wall span the estimate was drawn from
    // The two streams are ANTI-correlated: no positive peak exists anywhere in the
    // search, and a strong negative one does. That is not a timing problem, it is a
    // sign convention problem, and it falls out of this search for free. Measured at
    // -0.83 on a synthetic flip, and it is the issue #169 signature: one of the two
    // sources has its frame convention wrong. No offset will fix it, so offset_s is
    // left at 0 rather than reporting the edge of the search as if it meant something.
    bool   anti_correlated = false;
    double worst_correlation = 0.0;
  };

  // max_lag: widest offset considered, each way. 0.5 s covers every real case seen
  //   (a 100 ms USB stall, a companion board on an unsynced clock) without letting
  //   the search wander onto a periodic false peak.
  // rate_hz: the common grid both series are resampled onto.
  // min_span_s: refuse to answer from less data than this. A short window finds a
  //   confident peak in noise, which is the failure mode that matters.
  explicit TimeOffsetEstimator(double max_lag = 0.5,
                               double rate_hz = 100.0,
                               double min_span_s = 10.0)
    : max_lag_(max_lag), rate_hz_(rate_hz), min_span_s_(min_span_s) {}

  void add_imu(double t, double wz)     { push(imu_, t, wz); }
  void add_encoder(double t, double wz) { push(enc_, t, wz); }

  void reset() { imu_.clear(); enc_.clear(); }

  std::size_t imu_samples() const     { return imu_.size(); }
  std::size_t encoder_samples() const { return enc_.size(); }

  Result estimate() const {
    Result r;
    if (imu_.size() < 20 || enc_.size() < 20) return r;

    const double lo = std::max(imu_.front().t, enc_.front().t);
    const double hi = std::min(imu_.back().t, enc_.back().t);
    r.span_s = hi - lo;
    if (r.span_s < min_span_s_) return r;

    const double dt = 1.0 / rate_hz_;
    const int n = static_cast<int>(r.span_s / dt);
    if (n < 40) return r;

    // Both onto a common uniform grid. Without this, two sensors at different rates
    // have no sample-to-sample correspondence to correlate.
    std::vector<double> a(n), b(n);
    for (int i = 0; i < n; ++i) {
      a[i] = sample(imu_, lo + i * dt);
      b[i] = sample(enc_, lo + i * dt);
    }

    // A signal that never turns carries no timing information: every lag fits a
    // constant equally well. Refusing is the honest answer, and it is also the
    // common case on a robot driving down a row.
    if (spread(a) < 0.02 || spread(b) < 0.02) return r;

    const int max_shift = std::min(static_cast<int>(max_lag_ / dt), n / 4);
    if (max_shift < 2) return r;

    int best_k = 0;
    double best_c = -2.0, worst_c = 2.0;
    std::vector<double> curve(2 * max_shift + 1, -2.0);
    for (int k = -max_shift; k <= max_shift; ++k) {
      const double c = correlate(a, b, k);
      curve[k + max_shift] = c;
      if (c > best_c) { best_c = c; best_k = k; }
      if (c < worst_c && c > -1.5) worst_c = c;
    }
    r.worst_correlation = (worst_c < 1.5) ? worst_c : 0.0;

    // Anti-correlation is diagnostic rather than a failure to estimate.
    if (best_c < 0.5 && r.worst_correlation < -0.5) {
      r.anti_correlated = true;
      r.correlation = best_c;
      r.samples = n;
      return r;                 // valid stays false, offset stays 0
    }

    // Parabolic refinement through the peak and its neighbours. Gets well below the
    // grid step, which matters because at 100 Hz the grid alone is only 10 ms.
    double refined = static_cast<double>(best_k);
    const int idx = best_k + max_shift;
    if (idx > 0 && idx + 1 < static_cast<int>(curve.size())) {
      const double y0 = curve[idx - 1], y1 = curve[idx], y2 = curve[idx + 1];
      const double denom = (y0 - 2.0 * y1 + y2);
      if (std::abs(denom) > 1e-12) {
        const double adj = 0.5 * (y0 - y2) / denom;
        if (std::abs(adj) <= 1.0) refined += adj;
      }
    }

    r.valid       = best_c > 0.5;   // below this the two are not the same signal
    // Only report an offset we believe. The edge of the search is not an estimate,
    // and a reader who sees a number will use it.
    r.offset_s    = r.valid ? refined * dt : 0.0;
    r.correlation = best_c;
    r.samples     = n;
    return r;
  }

private:
  struct S { double t; double v; };

  void push(std::deque<S>& q, double t, double v) {
    if (!q.empty() && t <= q.back().t) return;      // non-monotonic stamp, drop it
    q.push_back({t, v});
    // Keep a window a few times the minimum span, so the estimate tracks a clock
    // that drifts rather than averaging over the whole run.
    const double keep = std::max(4.0 * min_span_s_, 4.0);
    while (!q.empty() && q.back().t - q.front().t > keep) q.pop_front();
  }

  // Linear interpolation at t. Zero-order hold would alias a rate signal.
  static double sample(const std::deque<S>& q, double t) {
    if (q.empty()) return 0.0;
    if (t <= q.front().t) return q.front().v;
    if (t >= q.back().t)  return q.back().v;
    std::size_t lo = 0, hi = q.size() - 1;
    while (hi - lo > 1) {
      const std::size_t mid = (lo + hi) / 2;
      if (q[mid].t <= t) lo = mid; else hi = mid;
    }
    const double span = q[hi].t - q[lo].t;
    if (span <= 0.0) return q[lo].v;
    const double w = (t - q[lo].t) / span;
    return q[lo].v * (1.0 - w) + q[hi].v * w;
  }

  static double spread(const std::vector<double>& v) {
    if (v.size() < 2) return 0.0;
    double mean = 0.0;
    for (double x : v) mean += x;
    mean /= static_cast<double>(v.size());
    double var = 0.0;
    for (double x : v) var += (x - mean) * (x - mean);
    return std::sqrt(var / static_cast<double>(v.size()));
  }

  // Normalised correlation of a against b shifted by k samples. Normalised so the
  // peak is comparable between a brisk turn and a gentle one, which is what lets a
  // single confidence threshold mean the same thing on different robots.
  static double correlate(const std::vector<double>& a,
                          const std::vector<double>& b, int k) {
    const int n = static_cast<int>(a.size());
    const int i0 = std::max(0, -k), i1 = std::min(n, n - k);
    const int m = i1 - i0;
    if (m < 20) return -2.0;
    double ma = 0.0, mb = 0.0;
    for (int i = i0; i < i1; ++i) { ma += a[i]; mb += b[i + k]; }
    ma /= m; mb /= m;
    double num = 0.0, da = 0.0, db = 0.0;
    for (int i = i0; i < i1; ++i) {
      const double x = a[i] - ma, y = b[i + k] - mb;
      num += x * y; da += x * x; db += y * y;
    }
    if (da <= 0.0 || db <= 0.0) return -2.0;
    return num / std::sqrt(da * db);
  }

  double max_lag_, rate_hz_, min_span_s_;
  std::deque<S> imu_, enc_;
};

}  // namespace fusioncore
