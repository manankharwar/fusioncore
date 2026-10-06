// Tests for the grid sweep helper. It exists to make a specific class of mistake
// mechanically impossible, so these check that it actually refuses that mistake.
#include <gtest/gtest.h>
#include "sweep.hpp"
#include <cmath>

using namespace fusioncore::sweep;

TEST(Sweep, VisitsTheFullCartesianProduct)
{
  Sweep s;
  s.axis("a", {1.0, 2.0, 3.0}).axis("b", {10.0, 20.0});
  const auto cells = s.run([](const Point& p) { return p.at("a") * p.at("b"); });
  ASSERT_EQ(cells.size(), 6u);
  double total = 0.0;
  for (const auto& c : cells) total += c.value;
  EXPECT_DOUBLE_EQ(total, (1 + 2 + 3) * (10 + 20));
}

TEST(Sweep, RefusesAnAxisThatDoesNotVary)
{
  // The exact failure that produced "r = 2 * P0_wz / P0_bias": one side held fixed.
  Sweep s;
  s.axis("varies", {1.0, 2.0}).axis("fixed", {5.0});
  const auto cells = s.run([](const Point&) { return 1.0; });
  EXPECT_TRUE(cells.empty())
      << "a one-value axis cannot support a claim about that parameter";
}

TEST(Sweep, CoversRejectsAZeroOnlyErrorAxis)
{
  // The failure that produced the 0.02 bias prior: probed with a true bias of zero,
  // the one value where a tight prior cannot hurt.
  Sweep s;
  s.axis("rate", {0.3, 0.6}).axis("true_bias", {0.0});
  const auto cells = s.run([](const Point&) { return 1.0; });
  EXPECT_TRUE(cells.empty()) << "single-valued axis is refused first";

  Sweep s2;
  s2.axis("rate", {0.3, 0.6}).axis("true_bias", {0.0, 0.0});
  const auto c2 = s2.run([](const Point&) { return 1.0; });
  EXPECT_FALSE(s2.covers(c2))
      << "an axis named for a true error must include a nonzero case";
}

TEST(Sweep, CoversAcceptsAProperGrid)
{
  Sweep s;
  s.axis("rate", {0.15, 0.6, 1.2}).axis("true_bias", {0.0, 0.35});
  const auto cells = s.run([](const Point& p) { return p.at("rate") + p.at("true_bias"); });
  EXPECT_EQ(cells.size(), 6u);
  EXPECT_TRUE(s.covers(cells));
}

TEST(Sweep, DistinguishesAnOffsetFromAScaleWhichOneRateCannot)
{
  // The 81.6% mistake, reproduced and then caught. At a single rate these two models
  // are indistinguishable; across rates they separate immediately.
  auto offset = [](const Point& p) { return p.at("rate") - 0.12; };
  auto scale  = [](const Point& p) { return p.at("rate") * 0.8; };

  Sweep one;
  one.axis("rate", {0.6, 0.6});        // the old probe, in effect
  const auto o1 = one.run(offset), s1 = one.run(scale);
  EXPECT_NEAR(o1.front().value, s1.front().value, 1e-9)
      << "at one rate the two models agree, which is why the probe was fooled";

  Sweep grid;
  grid.axis("rate", {0.15, 0.6, 1.2});
  const auto og = grid.run(offset), sg = grid.run(scale);
  bool differ = false;
  for (std::size_t i = 0; i < og.size(); ++i)
    if (std::abs(og[i].value - sg[i].value) > 1e-6) differ = true;
  EXPECT_TRUE(differ) << "across rates they must separate";
}
