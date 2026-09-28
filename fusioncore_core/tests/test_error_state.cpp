// The error-state layout and its conversions, tested before anything depends on them.
//
// Step 2 of #150. These are definitions only; the UKF switches over in a later commit.
// The point of testing them alone is that an index or factor mistake here would show up
// later as a benchmark number that moved, with nothing to point at.

#include <gtest/gtest.h>
#include <cmath>

#include "fusioncore/error_state.hpp"

using namespace fusioncore;

namespace {

StateVector make_state()
{
  StateVector x = StateVector::Zero();
  x[X] = 1.5; x[Y] = -2.5; x[Z] = 0.25;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, -0.3, 0.9).normalized()));
  x[QW] = q.w(); x[QX] = q.x(); x[QY] = q.y(); x[QZ] = q.z();
  for (int i = VX; i < STATE_DIM; ++i) { x[i] = 0.1 * (i - VX + 1); }
  return x;
}

}  // namespace

TEST(ErrorState, TheDimensionIsOneLessThanTheState)
{
  // Not cosmetic: the 23rd dimension was never real. A unit quaternion has four
  // components and three degrees of freedom.
  EXPECT_EQ(ERROR_DIM, 22);
  EXPECT_EQ(ERROR_DIM, STATE_DIM - 1);
  EXPECT_EQ(E_B_EWZ, ERROR_DIM - 1) << "the last error index must be the last dimension";
}

TEST(ErrorState, TheIndexMapIsContiguousAndShiftsAfterTheQuaternion)
{
  EXPECT_EQ(error_index_of(X), E_X);
  EXPECT_EQ(error_index_of(Z), E_Z);
  EXPECT_EQ(error_index_of(VX), E_VX);
  EXPECT_EQ(error_index_of(WZ), E_WZ);
  EXPECT_EQ(error_index_of(B_EWZ), E_B_EWZ);
  // Every non-quaternion state index must land on a distinct valid error index.
  int seen[ERROR_DIM] = {0};
  for (int i = 0; i < STATE_DIM; ++i) {
    const int e = error_index_of(i);
    if (e < 0) { continue; }
    ASSERT_GE(e, 0);
    ASSERT_LT(e, ERROR_DIM);
    EXPECT_EQ(seen[e], 0) << "error index " << e << " claimed twice";
    seen[e] = 1;
  }
}

TEST(ErrorState, TheQuaternionComponentsHaveNoSingleErrorIndex)
{
  // Rejected rather than folded. "The error index of QW" has no answer, and returning
  // one anyway would hide a bug rather than surface it.
  EXPECT_LT(error_index_of(QW), 0);
  EXPECT_LT(error_index_of(QX), 0);
  EXPECT_LT(error_index_of(QY), 0);
  EXPECT_LT(error_index_of(QZ), 0);
}

TEST(ErrorState, ExtractUndoesInject)
{
  const StateVector x = make_state();
  ErrorVector e;
  for (int i = 0; i < ERROR_DIM; ++i) { e[i] = 0.01 * (i + 1) * ((i % 2) ? -1 : 1); }

  const ErrorVector back = extract_error(inject_error(x, e), x);
  for (int i = 0; i < ERROR_DIM; ++i) {
    EXPECT_NEAR(back[i], e[i], 1e-9) << "component " << i;
  }
}

TEST(ErrorState, AZeroErrorChangesNothing)
{
  const StateVector x = make_state();
  const StateVector out = inject_error(x, ErrorVector::Zero());
  for (int i = 0; i < STATE_DIM; ++i) {
    EXPECT_NEAR(out[i], x[i], 1e-12) << "component " << i;
  }
}

TEST(ErrorState, InjectingAnErrorKeepsTheQuaternionOnTheUnitSphere)
{
  // The property the whole exercise exists for: no perturbation, however large, can
  // produce an invalid rotation.
  const StateVector x = make_state();
  for (double a : {0.001, 0.1, 1.0, 3.0, 10.0}) {
    ErrorVector e = ErrorVector::Zero();
    e[E_YAW] = a;
    const StateVector out = inject_error(x, e);
    const double n = std::sqrt(out[QW]*out[QW] + out[QX]*out[QX] +
                               out[QY]*out[QY] + out[QZ]*out[QZ]);
    EXPECT_NEAR(n, 1.0, 1e-12) << "yaw error " << a;
  }
}

TEST(ErrorState, AYawErrorIsAnAngleInRadians)
{
  // If this is ever off by a factor of two, every attitude covariance is off by four.
  StateVector x = StateVector::Zero();
  x[QW] = 1.0;
  ErrorVector e = ErrorVector::Zero();
  e[E_YAW] = 30.0 * M_PI / 180.0;
  const StateVector out = inject_error(x, e);
  const double yaw = std::atan2(2 * (out[QW]*out[QZ] + out[QX]*out[QY]),
                                1 - 2 * (out[QY]*out[QY] + out[QZ]*out[QZ])) * 180.0 / M_PI;
  EXPECT_NEAR(yaw, 30.0, 1e-9);
}

TEST(ErrorState, NonAttitudeStatesAreCopiedThroughTheCovarianceConversion)
{
  StateMatrix P = StateMatrix::Zero();
  P(X, X) = 4.0;
  P(VX, VX) = 9.0;
  P(B_EWZ, B_EWZ) = 0.25;
  P(X, VX) = P(VX, X) = 1.0;

  const ErrorMatrix E = to_error_covariance(P);
  EXPECT_DOUBLE_EQ(E(E_X, E_X), 4.0);
  EXPECT_DOUBLE_EQ(E(E_VX, E_VX), 9.0);
  EXPECT_DOUBLE_EQ(E(E_B_EWZ, E_B_EWZ), 0.25);
  EXPECT_DOUBLE_EQ(E(E_X, E_VX), 1.0);
}

TEST(ErrorState, TheAttitudeBlockCarriesTheFactorOfFour)
{
  // Near the identity a rotation of theta gives a quaternion vector part of about
  // theta/2, so an ANGLE variance is about 4x a COMPONENT variance. This is the entire
  // conversion and it is the easiest thing in the file to get silently wrong.
  StateMatrix P = StateMatrix::Zero();
  P(QZ, QZ) = 1e-8;
  const ErrorMatrix E = to_error_covariance(P);
  EXPECT_DOUBLE_EQ(E(E_YAW, E_YAW), 4e-8);
  EXPECT_NEAR(std::sqrt(E(E_YAW, E_YAW)) * 180.0 / M_PI, 0.01146, 1e-4)
      << "the shipped 1e-8 initial value should still mean about a hundredth of a degree";
}

TEST(ErrorState, AttitudeCrossTermsCarryOneFactorOfTwo)
{
  StateMatrix P = StateMatrix::Zero();
  P(QZ, X) = P(X, QZ) = 0.5;
  const ErrorMatrix E = to_error_covariance(P);
  EXPECT_DOUBLE_EQ(E(E_YAW, E_X), 1.0);
  EXPECT_DOUBLE_EQ(E(E_X, E_YAW), 1.0);
}

TEST(ErrorState, TheConvertedCovarianceIsSymmetric)
{
  StateMatrix P = StateMatrix::Zero();
  for (int i = 0; i < STATE_DIM; ++i) { P(i, i) = 0.1 * (i + 1); }
  P(QZ, X) = P(X, QZ) = 0.3;
  P(QY, VX) = P(VX, QY) = -0.2;
  const ErrorMatrix E = to_error_covariance(P);
  for (int r = 0; r < ERROR_DIM; ++r) {
    for (int c = 0; c < ERROR_DIM; ++c) {
      EXPECT_NEAR(E(r, c), E(c, r), 1e-12) << "asymmetric at (" << r << "," << c << ")";
    }
  }
}
