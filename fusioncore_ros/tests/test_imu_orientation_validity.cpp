#include <gtest/gtest.h>
#include <cmath>
#include <array>

// Regression test for the bias-window orientation check in fusion_node.cpp.
//
// The bug: the startup bias window accepted a driver's orientation only when
// the covariance was strictly positive:
//
//     has_orient = (ocov[0] > 0.0 || ocov[4] > 0.0 || ocov[8] > 0.0);
//
// while the runtime path (fuse_imu_orientation_if_valid) used the correct test,
// rejecting only a NEGATIVE covariance. Per the sensor_msgs/Imu spec:
//
//     -1  in orientation_covariance[0]  ->  "no orientation data"
//     all zeros                          ->  "covariance unknown" (data valid)
//
// Most drivers publish a good quaternion with an all-zero covariance because
// they simply never fill it in. Those were silently treated as having no
// orientation, which skipped accel-bias initialisation entirely: the bias was
// left at zero, so whatever component of gravity the IMU's mounting tilt
// produced stayed in the acceleration channel. A constant acceleration error
// double-integrates, so position ran away smoothly (reported from the field as
// hundreds of metres of drift on a 6 m out-and-back).

// Mirrors the predicate in fusion_node.cpp's bias window. Keep in sync.
static bool orientation_is_usable(const std::array<double, 9>& ocov,
                                  double qw, double qx, double qy, double qz)
{
    const double q_norm_sq = qw*qw + qx*qx + qy*qy + qz*qz;
    return (ocov[0] >= 0.0) && (q_norm_sq > 0.5);
}

static std::array<double, 9> zeros() { return std::array<double, 9>{}; }

static std::array<double, 9> with_cov0(double v)
{
    auto c = zeros();
    c[0] = v;
    return c;
}

// ─── The convention itself ──────────────────────────────────────────────────

// The case that caused the field failure: valid quaternion, unfilled covariance.
TEST(ImuOrientationValidity, ZeroCovarianceWithValidQuaternionIsUsable)
{
    EXPECT_TRUE(orientation_is_usable(zeros(), 1.0, 0.0, 0.0, 0.0));

    // A non-identity attitude must pass too.
    const double yaw = 0.7;
    EXPECT_TRUE(orientation_is_usable(zeros(), std::cos(yaw/2), 0.0, 0.0, std::sin(yaw/2)));
}

// -1 is the spec's explicit "no orientation data" flag.
TEST(ImuOrientationValidity, NegativeCovarianceMeansNoOrientation)
{
    EXPECT_FALSE(orientation_is_usable(with_cov0(-1.0), 1.0, 0.0, 0.0, 0.0));
}

// A driver with nothing to report may leave the quaternion all-zero, which is
// not a rotation. Normalising it would produce NaN.
TEST(ImuOrientationValidity, ZeroQuaternionIsRejectedEvenWithZeroCovariance)
{
    EXPECT_FALSE(orientation_is_usable(zeros(), 0.0, 0.0, 0.0, 0.0));
}

// A driver that does fill the covariance still works.
TEST(ImuOrientationValidity, PositiveCovarianceIsUsable)
{
    EXPECT_TRUE(orientation_is_usable(with_cov0(0.01), 1.0, 0.0, 0.0, 0.0));
}

// The old predicate, kept here so the regression is explicit: it rejected the
// exact message shape a BNO085 driver publishes.
TEST(ImuOrientationValidity, OldPredicateRejectedValidOrientation)
{
    const auto ocov = zeros();
    const bool old_predicate = (ocov[0] > 0.0 || ocov[4] > 0.0 || ocov[8] > 0.0);

    EXPECT_FALSE(old_predicate);
    EXPECT_TRUE(orientation_is_usable(ocov, 1.0, 0.0, 0.0, 0.0));
}

// ─── Why it mattered: the accel bias ────────────────────────────────────────

// Expected specific force for a given attitude, matching the bias window's math.
static void gravity_in_body(double qw, double qx, double qy, double qz,
                            double& gx, double& gy, double& gz)
{
    const double g = 9.80665;
    gx = 2.0*(qx*qz - qy*qw)*g;
    gy = 2.0*(qy*qz + qx*qw)*g;
    gz = (1.0 - 2.0*(qx*qx + qy*qy))*g;
}

// With the orientation used, a tilted-but-stationary IMU yields ~zero bias.
// With it discarded, the whole tilt component of gravity is left behind as a
// phantom acceleration.
TEST(ImuOrientationValidity, DiscardedOrientationLeavesGravityInAccelBias)
{
    // IMU pitched 10 degrees on its mount, robot stationary, no real bias.
    const double pitch = 10.0 * M_PI / 180.0;
    const double qw = std::cos(pitch/2), qx = 0.0, qy = std::sin(pitch/2), qz = 0.0;

    double gx, gy, gz;
    gravity_in_body(qw, qx, qy, qz, gx, gy, gz);

    // A perfect stationary accelerometer measures exactly that.
    const double meas_x = gx, meas_y = gy, meas_z = gz;

    // Orientation used: bias = measured - expected gravity ~= 0.
    EXPECT_NEAR(meas_x - gx, 0.0, 1e-9);
    EXPECT_NEAR(meas_y - gy, 0.0, 1e-9);
    EXPECT_NEAR(meas_z - gz, 0.0, 1e-9);

    // Orientation discarded: the filter starts level (identity), so it predicts
    // [0, 0, g] and the tilt component becomes an unmodelled acceleration.
    const double residual_x = meas_x - 0.0;
    EXPECT_GT(std::abs(residual_x), 1.5);   // ~1.70 m/s^2 at 10 degrees

    // That constant error double-integrates. Over a single 70 s run it is
    // enormous, which is what the field report showed.
    const double t = 70.0;
    const double drift = 0.5 * std::abs(residual_x) * t * t;
    EXPECT_GT(drift, 1000.0);
}
