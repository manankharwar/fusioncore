#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"

using namespace fusioncore;

// Catching an absolute heading source that is confidently wrong.
//
// Issue #73. A 9-axis IMU reported a yaw about 23 degrees off true for a whole
// waypoint mission. FusionCore reproduced it faithfully, which is what it is
// told to do, and the robot drove a dogleg on every leg because Nav2 steers on
// the reported heading. Measured from the user's own log: FusionCore's yaw sat a
// median 23.8 deg from the GPS course over ground across 22 segments, and 23.5
// deg from its own published position track across 20. Nothing warned.
namespace {

FusionCoreConfig mag_robot_config() {
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = true;      // absolute heading present: track heading stands down
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.gnss.base_noise_xy = 1.0;
  cfg.gnss.base_noise_z  = 1.0;
  cfg.outlier_rejection = true;
  cfg.outlier_threshold_gnss = 16.27;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  return cfg;
}

sensors::GnssFix fix_at(double x, double y) {
  sensors::GnssFix f;
  f.x = x; f.y = y; f.z = 0.0;
  f.hdop = 1.0; f.vdop = 1.0;
  f.satellites = 12;
  f.fix_type = sensors::GnssFixType::DGPS_FIX;
  return f;
}

// Truth drives due East. The IMU insists the robot is pointing yaw_bias_deg to
// the left of that, exactly as a miscalibrated magnetometer would.
FusionCoreStatus drive_with_biased_heading(double yaw_bias_deg, double run_s = 200.0) {
  FusionCore fc(mag_robot_config());
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665, speed = 1.5;
  const double bias = yaw_bias_deg * M_PI / 180.0;
  double true_x = 0.0;

  for (int step = 1; step * dt <= run_s + 1e-9; ++step) {
    const double t = step * dt;
    true_x += speed * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    fc.update_imu_orientation(t, 0.0, 0.0, bias);
    if (step % 2 == 0) {
      fc.update_encoder(t, speed, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }
    if (step % 100 == 0) fc.update_gnss(t, fix_at(true_x, 0.0));   // 1 Hz, truth
  }
  return fc.get_status();
}

} // namespace

TEST(HeadingCrossCheckTest, DetectsAnAbsoluteHeadingSourceThatIsWrong) {
  const FusionCoreStatus st = drive_with_biased_heading(23.0);
  std::cerr << "  heading source      : " << static_cast<int>(st.heading_source) << "\n"
            << "  disagreement        : " << st.heading_vs_track_deg << " deg\n"
            << "  segments compared   : " << st.heading_vs_track_n << "\n";

  EXPECT_EQ(st.heading_source, HeadingSource::IMU_ORIENTATION)
      << "the absolute source must still win: this check warns, it does not override";
  EXPECT_GE(st.heading_vs_track_n, 6)
      << "not enough straight segments were compared to say anything";
  EXPECT_NEAR(st.heading_vs_track_deg, 23.0, 6.0)
      << "should recover the injected heading bias from the GPS track";
}

TEST(HeadingCrossCheckTest, StaysQuietWhenTheHeadingIsRight) {
  const FusionCoreStatus st = drive_with_biased_heading(0.0);
  std::cerr << "  disagreement        : " << st.heading_vs_track_deg << " deg over "
            << st.heading_vs_track_n << " segments\n";

  EXPECT_GE(st.heading_vs_track_n, 6);
  EXPECT_LT(std::abs(st.heading_vs_track_deg), 5.0)
      << "a correct heading must not be reported as disagreeing";
}

TEST(HeadingCrossCheckTest, OffByDefaultDoesNothing) {
  FusionCoreConfig cfg = mag_robot_config();
  cfg.gps_track_heading_cross_check_deg = 0.0;      // disabled
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);

  const double dt = 0.01, g = 9.80665, speed = 1.5;
  double true_x = 0.0;
  for (int step = 1; step * dt <= 200.0 + 1e-9; ++step) {
    const double t = step * dt;
    true_x += speed * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    fc.update_imu_orientation(t, 0.0, 0.0, 23.0 * M_PI / 180.0);
    if (step % 2 == 0) {
      fc.update_encoder(t, speed, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }
    if (step % 100 == 0) fc.update_gnss(t, fix_at(true_x, 0.0));
  }
  EXPECT_EQ(fc.get_status().heading_vs_track_n, 0)
      << "disabled must mean nothing is accumulated at all";
}
