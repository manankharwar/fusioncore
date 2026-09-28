// The manifold operators, tested in isolation before anything uses them.
//
// Step 1 of #150. These functions touch no filter state, so if one of them is wrong it is
// wrong here, visibly, rather than showing up later as a benchmark number that moved.

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "fusioncore/attitude.hpp"

using namespace fusioncore::attitude;

namespace {

Eigen::Quaterniond yaw_q(double deg)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(deg * M_PI / 180.0,
                                              Eigen::Vector3d::UnitZ()));
}

double yaw_deg(const Eigen::Quaterniond & q)
{
  const auto & c = q.normalized();
  return std::atan2(2 * (c.w() * c.z() + c.x() * c.y()),
                    1 - 2 * (c.y() * c.y() + c.z() * c.z())) * 180.0 / M_PI;
}

}  // namespace

// ─── the round trip, which is the whole contract ─────────────────────────────

TEST(Attitude, BoxminusUndoesBoxplus)
{
  const Eigen::Quaterniond q = yaw_q(37.0);
  const std::vector<Eigen::Vector3d> deltas = {
    {0.0, 0.0, 0.0},
    {1e-9, 0.0, 0.0},                 // the Taylor branch
    {0.01, -0.02, 0.03},
    {0.5, 0.5, 0.5},
    {M_PI * 0.9, 0.0, 0.0},           // close to the half-turn boundary
    {0.0, 0.0, -2.0},
  };
  for (const auto & d : deltas) {
    const Eigen::Vector3d back = boxminus(boxplus(q, d), q);
    EXPECT_NEAR(back.x(), d.x(), 1e-9) << "delta = " << d.transpose();
    EXPECT_NEAR(back.y(), d.y(), 1e-9) << "delta = " << d.transpose();
    EXPECT_NEAR(back.z(), d.z(), 1e-9) << "delta = " << d.transpose();
  }
}

TEST(Attitude, ExpAndLogAreInverses)
{
  for (double a : {0.0, 1e-10, 1e-7, 0.001, 0.5, 1.5, 3.0}) {
    const Eigen::Vector3d v = a * Eigen::Vector3d(0.267, -0.535, 0.802).normalized();
    const Eigen::Vector3d back = log_map(exp_map(v));
    EXPECT_NEAR(back.norm(), v.norm(), 1e-9) << "angle " << a;
    if (a > 1e-9) {
      EXPECT_NEAR((back.normalized() - v.normalized()).norm(), 0.0, 1e-6) << "angle " << a;
    }
  }
}

TEST(Attitude, TheZeroRotationIsExactlyTheIdentity)
{
  // Called on nearly every step, so an epsilon here would be a permanent small bias.
  const Eigen::Quaterniond q = exp_map(Eigen::Vector3d::Zero());
  EXPECT_DOUBLE_EQ(q.w(), 1.0);
  EXPECT_DOUBLE_EQ(q.x(), 0.0);
  EXPECT_DOUBLE_EQ(q.y(), 0.0);
  EXPECT_DOUBLE_EQ(q.z(), 0.0);
  EXPECT_EQ(log_map(Eigen::Quaterniond::Identity()), Eigen::Vector3d::Zero());
}

// ─── the properties that make it safe for a covariance ───────────────────────

TEST(Attitude, AMagnitudeIsAnAngleInRadians)
{
  // The convention the whole migration rests on: |delta| is the rotation angle. If this
  // is ever off by a factor of two, every attitude covariance is off by a factor of four.
  const Eigen::Vector3d d(0.0, 0.0, 30.0 * M_PI / 180.0);
  EXPECT_NEAR(yaw_deg(boxplus(Eigen::Quaterniond::Identity(), d)), 30.0, 1e-9);
  EXPECT_NEAR(boxminus(yaw_q(30.0), Eigen::Quaterniond::Identity()).norm(),
              30.0 * M_PI / 180.0, 1e-9);
}

TEST(Attitude, TheSignConventionIsBodyFrameAndRightHanded)
{
  // Composition order is not a detail: q*exp(d) and exp(d)*q differ whenever the
  // rotations do not commute, and picking the wrong one is silent on yaw-only tests.
  const Eigen::Quaterniond q = yaw_q(90.0);
  const Eigen::Vector3d roll(0.3, 0.0, 0.0);      // about BODY x
  const Eigen::Quaterniond got = boxplus(q, roll);
  const Eigen::Quaterniond want = q * Eigen::Quaterniond(
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));
  EXPECT_NEAR(std::abs(got.dot(want)), 1.0, 1e-12)
      << "boxplus must post-multiply, i.e. rotate in the BODY frame";
}

TEST(Attitude, OppositeSignQuaternionsAreTheSameRotation)
{
  // q and -q are one rotation. A log that does not fold them together reports the long
  // way round and poisons any mean taken over sigma points.
  Eigen::Quaterniond q = yaw_q(20.0);
  Eigen::Quaterniond neg = q;
  neg.coeffs() *= -1.0;
  EXPECT_NEAR((log_map(q) - log_map(neg)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(boxminus(neg, q).norm(), 0.0, 1e-12);
}

TEST(Attitude, ALargeRotationNeverLeavesTheUnitSphere)
{
  // The failure this whole exercise exists to prevent: a perturbation must always be a
  // valid rotation, however large the uncertainty gets.
  for (double a : {0.1, 1.0, 3.0, 6.0, 12.0, 100.0}) {
    const Eigen::Quaterniond q = boxplus(yaw_q(10.0), Eigen::Vector3d(0, 0, a));
    EXPECT_NEAR(q.norm(), 1.0, 1e-12) << "angle " << a;
  }
}

// ─── the weighted mean ───────────────────────────────────────────────────────

TEST(Attitude, TheMeanOfIdenticalAttitudesIsThatAttitude)
{
  std::vector<Eigen::Quaterniond> q(5, yaw_q(42.0));
  std::vector<double> w(5, 0.2);
  EXPECT_NEAR(yaw_deg(weighted_mean(q, w, 5)), 42.0, 1e-9);
}

TEST(Attitude, TheMeanOfASymmetricSpreadIsTheCentre)
{
  std::vector<Eigen::Quaterniond> q = {yaw_q(0.0), yaw_q(-30.0), yaw_q(30.0)};
  std::vector<double> w = {0.5, 0.25, 0.25};
  EXPECT_NEAR(yaw_deg(weighted_mean(q, w, 3)), 0.0, 1e-6);
}

TEST(Attitude, AWideSpreadDegradesGraduallyInsteadOfFlipping)
{
  // The t=77 s failure, reproduced at the level of the operator. Summing these as
  // 4-vectors and renormalising gives a near-cancelling residue whose direction is
  // arbitrary. The tangent-space mean must stay near the centre instead.
  std::vector<Eigen::Quaterniond> q = {yaw_q(0.0), yaw_q(-140.0), yaw_q(140.0)};
  std::vector<double> w = {1.0 / 3, 1.0 / 3, 1.0 / 3};
  const double got = yaw_deg(weighted_mean(q, w, 3));
  EXPECT_LT(std::abs(got), 90.0)
      << "a symmetric spread averaged to " << got << " deg, which is not near its centre";
}

TEST(Attitude, TheMeanIsAlwaysAUnitQuaternion)
{
  std::vector<Eigen::Quaterniond> q = {yaw_q(10.0), yaw_q(170.0), yaw_q(-170.0)};
  std::vector<double> w = {0.4, 0.3, 0.3};
  EXPECT_NEAR(weighted_mean(q, w, 3).norm(), 1.0, 1e-12);
}
