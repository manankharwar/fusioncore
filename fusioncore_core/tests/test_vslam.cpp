#include <gtest/gtest.h>
#include <limits>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/sensors/vslam.hpp"

using namespace fusioncore;
using namespace fusioncore::sensors;

// ─── Test 1: VSLAM pose corrects drifted position ────────────────────────────

TEST(VSLAMTest, PoseCorrectionsDriftedPosition) {
  FusionCore fc;
  State initial;
  initial.x      = StateVector::Zero();
  initial.x[X]   = 5.0;   // filter thinks it's at x=5m
  initial.P      = StateMatrix::Identity() * 10.0;
  fc.init(initial, 0.0);

  VslamPose pose;
  pose.x     = 1.0;   // VSLAM says x=1m
  pose.y     = 0.0;
  pose.z     = 0.0;
  pose.roll  = 0.0;
  pose.pitch = 0.0;
  pose.yaw   = 0.0;
  pose.has_position_cov    = true;
  pose.position_cov(0,0)   = 0.05 * 0.05;
  pose.position_cov(1,1)   = 0.05 * 0.05;
  pose.position_cov(2,2)   = 0.05 * 0.05;
  pose.has_orientation_cov  = true;
  pose.orientation_cov(0,0) = 0.02 * 0.02;
  pose.orientation_cov(1,1) = 0.02 * 0.02;
  pose.orientation_cov(2,2) = 0.02 * 0.02;

  bool accepted = fc.update_pose(0.1, pose);
  EXPECT_TRUE(accepted);
  EXPECT_LT(fc.get_state().x[X], 5.0);
  EXPECT_NEAR(fc.get_state().x[X], 1.0, 2.0);
  EXPECT_EQ(fc.get_status().vslam_health, SensorHealth::OK);
}

// ─── Test 2: VSLAM outlier is rejected ───────────────────────────────────────

TEST(VSLAMTest, OutlierIsRejected) {
  FusionCore fc;
  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;  // tight covariance
  fc.init(initial, 0.0);

  VslamPose pose;
  pose.x     = 500.0;   // 500m jump, should be gated
  pose.y     = 0.0;
  pose.z     = 0.0;
  pose.has_position_cov  = true;
  pose.position_cov(0,0) = 0.05 * 0.05;
  pose.position_cov(1,1) = 0.05 * 0.05;
  pose.position_cov(2,2) = 0.05 * 0.05;

  bool accepted = fc.update_pose(0.1, pose);
  EXPECT_FALSE(accepted);
  EXPECT_NEAR(fc.get_state().x[X], 0.0, 0.01);
  EXPECT_EQ(fc.get_status().vslam_outliers, 1);
}

// ─── Test 3: VSLAM + IMU integration (indoor, no GPS, no encoder) ────────────

TEST(VSLAMTest, VSLAMAndIMUIndoor) {
  FusionCore fc;
  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 0.1;
  fc.init(initial, 0.0);

  VslamPose pose;
  pose.has_position_cov    = true;
  pose.has_orientation_cov = true;
  pose.position_cov    = Eigen::Matrix3d::Identity() * (0.05 * 0.05);
  pose.orientation_cov = Eigen::Matrix3d::Identity() * (0.02 * 0.02);

  // 5 seconds: IMU at 100Hz, VSLAM at 5Hz, robot moves forward at 1 m/s
  for (int i = 1; i <= 500; ++i) {
    double t = i * 0.01;
    fc.update_imu(t, 0, 0, 0, 0, 0, 9.81);

    if (i % 20 == 0) {
      pose.x = 1.0 * t;
      pose.y = 0.0;
      pose.z = 0.0;
      fc.update_pose(t, pose);
    }
  }

  EXPECT_NEAR(fc.get_state().x[X], 5.0, 1.0);
  EXPECT_NEAR(fc.get_state().x[Y], 0.0, 0.3);
  EXPECT_EQ(fc.get_status().vslam_health, SensorHealth::OK);
}

// ─── Test 4: No-covariance fallback uses config noise ────────────────────────

TEST(VSLAMTest, FallbackToConfigNoise) {
  FusionCoreConfig config;
  config.vslam.position_noise    = 0.2;
  config.vslam.orientation_noise = 0.05;
  FusionCore fc(config);

  State initial;
  initial.x    = StateVector::Zero();
  initial.x[X] = 3.0;
  initial.P    = StateMatrix::Identity() * 10.0;
  fc.init(initial, 0.0);

  VslamPose pose;
  pose.x = 0.5;
  pose.y = 0.0;
  pose.z = 0.0;
  // No covariance set: falls back to config noise (0.2m)

  bool accepted = fc.update_pose(0.1, pose);
  EXPECT_TRUE(accepted);
  EXPECT_LT(fc.get_state().x[X], 3.0);
}

// ─── Test 5: Orientation update corrects yaw ─────────────────────────────────

TEST(VSLAMTest, OrientationCorrectedFromVSLAM) {
  FusionCore fc;
  State initial;
  initial.x = StateVector::Zero();
  initial.P = StateMatrix::Identity() * 1.0;
  // Identity*1.0 puts 1.0 on the QUATERNION diagonal, which state.hpp specifies at
  // about 1e-8. At that spread the sigma points are thrown far off the unit sphere
  // and their yaws span well past +-pi, where a mean of angles has no single right
  // answer and the update result is an artefact of how it is computed. Keep the
  // large covariance everywhere it is meaningful and give the quaternion a value
  // that is merely uncertain rather than nonsensical, so this test measures what it
  // claims to: that a confident heading pulls yaw toward the measurement.
  for (int q : {QW, QX, QY, QZ}) initial.P(q, q) = 1e-2;
  fc.init(initial, 0.0);

  VslamPose pose;
  pose.x = 0.0; pose.y = 0.0; pose.z = 0.0;
  pose.roll = 0.0; pose.pitch = 0.0;
  pose.yaw = M_PI / 4.0;   // VSLAM says 45 degrees
  pose.has_orientation_cov  = true;
  pose.orientation_cov(0,0) = 0.02 * 0.02;
  pose.orientation_cov(1,1) = 0.02 * 0.02;
  pose.orientation_cov(2,2) = 0.02 * 0.02;

  bool accepted = fc.update_pose(0.1, pose);
  EXPECT_TRUE(accepted);
  EXPECT_GT(fc.get_state().yaw(), 0.0);
  EXPECT_NEAR(fc.get_state().yaw(), M_PI / 4.0, 0.3);
}

// ─── Test 6: A pose carrying NaN never reaches the filter ────────────────────
//
// See GNSSTest.NonFiniteFixIsRejectedAndNamed and #103: the chi2 gate cannot
// catch a NaN, because it compares false against the threshold. One case per
// field, the covariance included when the message supplies one.
TEST(VSLAMTest, NonFinitePoseIsRejectedAndNamed) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  // A pose where the filter already is, with its covariance: acceptable.
  auto ordinary_pose = [] {
    VslamPose pose;
    pose.has_position_cov    = true;
    pose.position_cov        = Eigen::Matrix3d::Identity() * (0.05 * 0.05);
    pose.has_orientation_cov = true;
    pose.orientation_cov     = Eigen::Matrix3d::Identity() * (0.02 * 0.02);
    return pose;
  };

  for (int field = 0; field < 8; ++field) {
    for (const double bad : {nan, inf}) {
      FusionCore fc;
      State initial;
      fc.init(initial, 0.0);

      VslamPose pose = ordinary_pose();
      switch (field) {
        case 0: pose.x = bad; break;
        case 1: pose.y = bad; break;
        case 2: pose.z = bad; break;
        case 3: pose.roll = bad; break;
        case 4: pose.pitch = bad; break;
        case 5: pose.yaw = bad; break;
        case 6: pose.position_cov(1, 1) = bad; break;
        case 7: pose.orientation_cov(2, 2) = bad; break;
      }

      EXPECT_FALSE(fc.update_pose(0.1, pose))
          << "field " << field << " = " << bad << " was accepted";
      EXPECT_EQ(fc.get_status().vslam_reason, VslamRejectionReason::NOT_FINITE)
          << "field " << field << " = " << bad << " was not rejected as NOT_FINITE";
      EXPECT_TRUE(fc.get_state().x.allFinite())
          << "field " << field << " = " << bad << " reached the state";
      EXPECT_TRUE(fc.get_state().P.allFinite())
          << "field " << field << " = " << bad << " reached the covariance";

      EXPECT_TRUE(fc.update_pose(0.2, ordinary_pose()))
          << "an ordinary pose after field " << field << " was not accepted";
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
