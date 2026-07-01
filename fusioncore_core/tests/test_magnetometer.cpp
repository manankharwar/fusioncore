#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/sensors/magnetometer.hpp"
#include "fusioncore/motion_model.hpp"
#include "fusioncore/state.hpp"

using namespace fusioncore;
using namespace fusioncore::sensors;

// Helper: create and initialize a FusionCore instance with magnetometer enabled
static FusionCore make_fc_with_mag() {
  FusionCoreConfig cfg;
  cfg.outlier_rejection = false;  // disable gating so every update fires
  cfg.mag.noise_rad     = 0.05;
  cfg.mag.chi2_threshold = 9.21;
  cfg.mag.declination_rad = 0.0;

  FusionCore fc(cfg);
  State initial;
  initial.P(X, X) = 1.0;
  initial.P(Y, Y) = 1.0;
  fc.init(initial, 0.0);
  return fc;
}

// ─── Test 1: mag_yaw_from_field flat, no tilt, pointing east ─────────────────

TEST(MagnetometerTest, FlatPointingEast) {
  MagParams p;
  // Flat, no roll/pitch, pointing east (yaw=0).
  // Magnetic north is ENU +y direction. With yaw=0 (robot pointing east),
  // m_body = [0, Bh, 0].
  double yaw = mag_yaw_from_field(0.0, 1.0, 0.0, p, 0.0, 0.0);
  EXPECT_NEAR(yaw, 0.0, 1e-9);
}

// ─── Test 2: mag_yaw_from_field flat, no tilt, pointing north ────────────────

TEST(MagnetometerTest, FlatPointingNorth) {
  MagParams p;
  // Pointing north (yaw=pi/2). m_body = [Bh, 0, 0].
  double yaw = mag_yaw_from_field(1.0, 0.0, 0.0, p, 0.0, 0.0);
  EXPECT_NEAR(yaw, M_PI / 2.0, 1e-9);
}

// ─── Test 3: mag_yaw_from_field flat, pointing west ──────────────────────────

TEST(MagnetometerTest, FlatPointingWest) {
  MagParams p;
  // Pointing west (yaw=pi). m_body = [0, -Bh, 0].
  double yaw = mag_yaw_from_field(0.0, -1.0, 0.0, p, 0.0, 0.0);
  EXPECT_NEAR(std::abs(yaw), M_PI, 1e-9);
}

// ─── Test 4: Declination offset applied correctly ────────────────────────────

TEST(MagnetometerTest, DeclinationOffset) {
  MagParams p;
  p.declination_rad = 0.1;  // 0.1 rad positive declination
  // Pointing east (yaw=0 true north reference). m_body = [0, Bh, 0].
  double yaw = mag_yaw_from_field(0.0, 1.0, 0.0, p, 0.0, 0.0);
  EXPECT_NEAR(yaw, 0.1, 1e-9);
}

// ─── Test 5: Hard iron correction ────────────────────────────────────────────

TEST(MagnetometerTest, HardIronCorrection) {
  MagParams p;
  p.hard_iron = Eigen::Vector3d(0.5, 0.3, 0.1);
  // Raw reading: true field [0, 1, 0] + hard iron bias
  double mx = 0.0 + 0.5;
  double my = 1.0 + 0.3;
  double mz = 0.0 + 0.1;
  double yaw = mag_yaw_from_field(mx, my, mz, p, 0.0, 0.0);
  EXPECT_NEAR(yaw, 0.0, 1e-9);
}

// ─── Test 6: Tilt compensation (pitched forward) preserves yaw ───────────────

TEST(MagnetometerTest, TiltCompensationPitch) {
  MagParams p;
  // Robot pointing east (yaw=0), pitched forward 20 deg.
  // m_world = [0, Bh=1, -Bv=0] (horizontal only for simplicity).
  // m_body = Ry(-pitch) * [0, 1, 0] = [sin(pitch)*0 + cos(pitch)*0, 1, 0] ... let me compute:
  // R^T = Rz(0)^T * Ry(-pitch) = Ry(-pitch) for yaw=0
  // Ry(-p)*[0,1,0] = [sin(p)*0+cos(p)*0 + ... ]
  // Actually: Ry(-p) = [[cos(p),0,-sin(p)],[0,1,0],[sin(p),0,cos(p)]]
  // m_body = [0*cos(p)+1*0+0*(-sin(p)), 0*0+1*1+0*0, 0*sin(p)+1*0+0*cos(p)]
  //        = [0, 1, 0] -- no change since m_world is purely y
  // So tilt compensation should also give yaw = 0
  double pitch = 20.0 * M_PI / 180.0;
  double yaw = mag_yaw_from_field(0.0, 1.0, 0.0, p, 0.0, pitch);
  EXPECT_NEAR(yaw, 0.0, 1e-6);
}

// ─── Test 7: update_magnetometer moves yaw toward measurement ────────────────

TEST(MagnetometerTest, UpdateMovesYawTowardMeasurement) {
  FusionCoreConfig cfg;
  cfg.outlier_rejection = false;
  cfg.mag.noise_rad     = 0.05;

  FusionCore fc(cfg);
  State initial;
  // Initialize with large yaw uncertainty so the filter accepts magnetometer corrections.
  // Default P[QW/QZ] is 1e-8, which would prevent convergence in any reasonable iteration count.
  initial.P(QW, QW) = 0.5;
  initial.P(QZ, QZ) = 0.5;
  fc.init(initial, 0.0);

  for (int i = 0; i < 20; ++i) {
    fc.update_imu(0.01 * i, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);
  }

  // yaw=pi/4 (northeast): mx=sin(pi/4), my=cos(pi/4) -> atan2(mx,my)=pi/4
  double target_yaw = M_PI / 4.0;
  double mx = std::sin(target_yaw);
  double my = std::cos(target_yaw);

  for (int i = 0; i < 300; ++i) {
    fc.update_magnetometer(0.2 + 0.01 * i, mx, my, 0.0);
  }

  const State& s = fc.get_state();
  double roll, pitch, yaw;
  quat_to_euler(s.x[QW], s.x[QX], s.x[QY], s.x[QZ], roll, pitch, yaw);
  // Tolerance is 3x noise_rad: UKF residual after many updates without
  // interleaved IMU updates is bounded by the measurement noise floor.
  EXPECT_NEAR(yaw, target_yaw, 0.15);
}

// ─── Test 8: update_magnetometer returns true on acceptance ──────────────────

TEST(MagnetometerTest, UpdateReturnsTrueOnAcceptance) {
  FusionCore fc = make_fc_with_mag();
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  bool accepted = fc.update_magnetometer(0.01, 0.0, 1.0, 0.0);
  EXPECT_TRUE(accepted);
}

// ─── Test 9: Chi-squared gate rejects outlier ────────────────────────────────

TEST(MagnetometerTest, Chi2GateRejectsOutlier) {
  FusionCoreConfig cfg;
  cfg.outlier_rejection  = true;
  cfg.mag.noise_rad      = 0.001;   // very tight: any deviation is an outlier
  cfg.mag.chi2_threshold = 9.21;

  FusionCore fc(cfg);
  State initial;
  fc.init(initial, 0.0);
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  // Filter is at yaw=0. Feed a pi/2 reading with noise_rad=0.001: huge outlier.
  bool accepted = fc.update_magnetometer(0.01, 1.0, 0.0, 0.0);  // yaw=pi/2
  EXPECT_FALSE(accepted);
}

// ─── Test 10: heading_source set to MAGNETOMETER after first update ───────────

TEST(MagnetometerTest, HeadingSourceSetAfterUpdate) {
  FusionCore fc = make_fc_with_mag();
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  EXPECT_FALSE(fc.is_heading_valid());
  EXPECT_EQ(fc.heading_source(), HeadingSource::NONE);

  fc.update_magnetometer(0.01, 0.0, 1.0, 0.0);

  EXPECT_TRUE(fc.is_heading_valid());
  EXPECT_EQ(fc.heading_source(), HeadingSource::MAGNETOMETER);
}

// ─── Test 11: DUAL_ANTENNA heading not overridden by magnetometer ─────────────

TEST(MagnetometerTest, DualAntennaNotOverriddenByMag) {
  FusionCore fc = make_fc_with_mag();
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  // Set heading source to DUAL_ANTENNA manually via update_gnss_heading
  sensors::GnssHeading hdg;
  hdg.heading_rad  = 0.0;
  hdg.accuracy_rad = 0.01;
  hdg.valid        = true;
  fc.update_gnss_heading(0.01, hdg);

  EXPECT_EQ(fc.heading_source(), HeadingSource::DUAL_ANTENNA);

  // Now fuse a magnetometer update
  fc.update_magnetometer(0.02, 0.0, 1.0, 0.0);

  // Source must remain DUAL_ANTENNA
  EXPECT_EQ(fc.heading_source(), HeadingSource::DUAL_ANTENNA);
}

// ─── Test 12: mag_outliers incremented on rejection ───────────────────────────

TEST(MagnetometerTest, OutlierCounterIncremented) {
  FusionCoreConfig cfg;
  cfg.outlier_rejection  = true;
  cfg.mag.noise_rad      = 0.001;
  cfg.mag.chi2_threshold = 9.21;

  FusionCore fc(cfg);
  State initial;
  fc.init(initial, 0.0);
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  fc.update_magnetometer(0.01, 1.0, 0.0, 0.0);  // outlier

  auto status = fc.get_status();
  EXPECT_GT(status.mag_outliers, 0);
}

// ─── Test 13: field-magnitude disturbance detection (pure helper) ─────────────
// A clean reading's corrected magnitude equals the local Earth field. A nearby
// motor or steel structure distorts the magnitude. The helper flags that, which
// the heading-only chi2 gate cannot.

TEST(MagnetometerTest, FieldDisturbanceHelper) {
  MagParams p;
  // Disabled by default (field_strength = 0): never flags, whatever the input.
  EXPECT_FALSE(mag_field_disturbed(99.0, 0.0, 0.0, p));

  p.field_strength  = 1.0;
  p.field_tolerance = 0.2;
  EXPECT_FALSE(mag_field_disturbed(0.6, 0.8, 0.0, p));  // |(.6,.8,0)| = 1.0, clean
  EXPECT_TRUE (mag_field_disturbed(1.5, 0.0, 0.0, p));  // 50% high, disturbed
  EXPECT_TRUE (mag_field_disturbed(0.5, 0.0, 0.0, p));  // 50% low, disturbed
}

// ─── Test 14: update_magnetometer rejects a disturbed-magnitude reading ───────
// The disturbed reading points the SAME direction (east) as a clean one, so the
// heading chi2 gate would accept it. The magnitude check is what rejects it.

TEST(MagnetometerTest, DisturbedFieldRejected) {
  FusionCoreConfig cfg;
  cfg.outlier_rejection  = true;
  cfg.mag.noise_rad      = 0.05;
  cfg.mag.chi2_threshold = 10.83;
  cfg.mag.field_strength  = 1.0;   // expect a unit-magnitude field
  cfg.mag.field_tolerance = 0.2;

  FusionCore fc(cfg);
  State s0;
  s0.P(QW, QW) = 0.5;
  s0.P(QZ, QZ) = 0.5;
  fc.init(s0, 0.0);
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  EXPECT_TRUE (fc.update_magnetometer(0.01, 0.0, 1.0, 0.0));  // clean, accepted
  EXPECT_FALSE(fc.update_magnetometer(0.02, 0.0, 1.6, 0.0));  // 60% high, rejected
}

// ─── Test 15: magnetometer bounds heading drift from wheel slip in a blackout ─
// This is the lever for the long-blackout benchmark losses (#63/#64). During a
// multi-minute GPS outage a ground robot dead-reckons on wheel odometry + IMU.
// Wheel slip biases the odometry yaw rate and an unmodeled gyro bias compounds
// it, so heading error grows without bound (this is what makes a long blackout
// lose to a simpler 2D filter). A magnetometer adds a slip-immune absolute
// heading that bounds the error and makes the gyro bias observable. Two identical
// dead-reckoning runs, one fed the magnetometer at the true heading, prove it.
//
// Note: the magnetometer needs an independent rate reference (the wheel encoder
// here) to resolve the yaw / gyro-bias ambiguity. Mag + IMU alone, with no rate
// reference, is ill-conditioned over a long outage. Real ground robots always
// have wheel odometry, which is exactly this configuration.

TEST(MagnetometerTest, BoundsHeadingDriftFromSlipDuringBlackout) {
  auto run = [](bool use_mag) {
    FusionCoreConfig cfg;
    cfg.outlier_rejection     = true;
    cfg.outlier_threshold_imu = 15.09;
    cfg.outlier_threshold_enc = 11.34;
    cfg.mag.noise_rad         = 0.05;
    cfg.mag.chi2_threshold    = 10.83;
    cfg.encoder.vel_noise_wz  = 0.05;
    cfg.motion_model = create_motion_model("DifferentialDrive");
    FusionCore fc(cfg);
    State s0;
    s0.P(QW, QW) = 0.25;
    s0.P(QZ, QZ) = 0.25;
    fc.init(s0, 0.0);

    // Truth: driving straight east at 1 m/s (true yaw rate 0), GPS blacked out.
    // The gyro has a +0.02 rad/s bias; the wheels slip so odometry reports a
    // false +0.01 rad/s yaw rate. Both errors push heading the same way.
    // IMU 20 Hz, encoder 10 Hz, magnetometer 4 Hz over 90 s (kept coarse so the
    // test stays well under the CI timeout while still integrating enough drift).
    const double dt = 0.05, gyro_bias = 0.02, enc_slip_wz = 0.010;
    for (int step = 1; step * dt <= 90.0 + 1e-9; ++step) {
      double t = step * dt;
      fc.update_imu(t, 0.0, 0.0, gyro_bias, 0.0, 0.0, 9.80665);
      if (step % 2 == 0) fc.update_encoder(t, 1.0, 0.0, enc_slip_wz);
      if (use_mag && step % 5 == 0) fc.update_magnetometer(t, 0.0, 1.0, 0.0);
    }
    return std::fabs(fc.get_state().yaw());
  };

  double err_no_mag = run(false);
  double err_mag    = run(true);

  EXPECT_GT(err_no_mag, 0.5);                 // odometry + gyro: heading runs away (>~29 deg)
  EXPECT_LT(err_mag,    0.1);                  // magnetometer: bounded (<~6 deg)
  EXPECT_LT(err_mag,    err_no_mag * 0.2)      // mag cuts the heading error by >80%
      << "mag err " << err_mag << " vs no-mag " << err_no_mag;
}
