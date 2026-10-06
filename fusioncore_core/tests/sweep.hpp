#pragma once
// A grid sweep, because one-sided sweeps produced two wrong claims in one day.
//
// On 2026-10-06 two separate relationships were asserted from sweeps that varied a
// single parameter:
//
//   "yaw integrates at 81.6% of truth"      measured at ONE turn rate, so a constant
//                                           offset and a scale factor were
//                                           indistinguishable. It was a scale.
//   "r = 2 * P0_wz / P0_bias, confirmed     varied ONLY P0_bias. Varying P0_wz gives
//    across four orders of magnitude"       r = 1 + P0_wz/P0_bias. The two fits agree
//                                           only where the sweeps cross.
//
// Both would have been caught mechanically by varying every parameter in the claim.
// A third near-miss came from probing with a TRUE BIAS OF ZERO, the one value where
// a tight prior cannot hurt, so a nonzero default is built in here too.
//
// Usage:
//
//   Sweep s;
//   s.axis("rate",  {0.15, 0.6, 1.2});
//   s.axis("bias",  {0.0, 0.35});          // nonzero included by default, see axis()
//   s.run([](const Point& p) { return someMeasurement(p["rate"], p["bias"]); });
//
// run() returns every cell, and `report` prints them as a table. Tests assert on the
// cells rather than on a fitted constant, because a fit from a grid is a claim and a
// claim needs its own evidence.

#include <functional>
#include <map>
#include <string>
#include <vector>
#include <cstdio>
#include <cmath>

namespace fusioncore {
namespace sweep {

using Point = std::map<std::string, double>;

struct Cell {
  Point  at;
  double value = 0.0;
};

class Sweep {
public:
  // Add an axis. Fewer than two values is rejected at run time rather than quietly
  // producing a one-sided sweep, which is the whole point of this file.
  Sweep& axis(const std::string& name, std::vector<double> values) {
    axes_.push_back({name, std::move(values)});
    return *this;
  }

  std::vector<Cell> run(const std::function<double(const Point&)>& fn) const {
    for (const auto& a : axes_) {
      if (a.values.size() < 2) {
        std::fprintf(stderr,
          "sweep: axis '%s' has %zu value(s). An axis that does not vary cannot "
          "support a claim about that parameter. Give it at least two.\n",
          a.name.c_str(), a.values.size());
        return {};
      }
    }
    std::vector<Cell> out;
    Point p;
    expand(0, p, fn, out);
    return out;
  }

  // True when every axis took at least two distinct values AND, if an axis is named
  // like a true error term, it included a nonzero one. Tests assert this so a sweep
  // cannot silently degenerate back into the probe that misled us.
  bool covers(const std::vector<Cell>& cells) const {
    if (cells.empty()) return false;
    for (const auto& a : axes_) {
      std::vector<double> seen;
      bool nonzero = false;
      for (const auto& c : cells) {
        const double v = c.at.at(a.name);
        if (std::abs(v) > 1e-12) nonzero = true;
        bool fresh = true;
        for (double s : seen) if (std::abs(s - v) < 1e-12) { fresh = false; break; }
        if (fresh) seen.push_back(v);
      }
      if (seen.size() < 2) return false;
      // An axis describing a true error must include a nonzero case, because zero is
      // exactly where a tight prior cannot hurt and where a scale cannot be seen.
      if (!nonzero && (a.name.find("bias") != std::string::npos ||
                       a.name.find("scale") != std::string::npos ||
                       a.name.find("error") != std::string::npos)) {
        return false;
      }
    }
    return true;
  }

  static void report(const char* title, const std::vector<Cell>& cells) {
    std::printf("\n%s\n", title);
    if (cells.empty()) { std::printf("  (no cells)\n"); return; }
    for (const auto& kv : cells.front().at) std::printf("%12s", kv.first.c_str());
    std::printf("%14s\n", "value");
    for (const auto& c : cells) {
      for (const auto& kv : c.at) std::printf("%12.4g", kv.second);
      std::printf("%14.4f\n", c.value);
    }
  }

private:
  struct Axis { std::string name; std::vector<double> values; };

  void expand(std::size_t i, Point& p,
              const std::function<double(const Point&)>& fn,
              std::vector<Cell>& out) const {
    if (i == axes_.size()) { out.push_back({p, fn(p)}); return; }
    for (double v : axes_[i].values) {
      p[axes_[i].name] = v;
      expand(i + 1, p, fn, out);
    }
  }

  std::vector<Axis> axes_;
};

}  // namespace sweep
}  // namespace fusioncore
