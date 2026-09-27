// Issue #144: discard the GPS-track-heading bearing window on the ANGLE turned
// across it, not on one instantaneous yaw-rate sample.
//
// The defect these tests encode, measured over five rover bags on ordinary grass:
// the rate gate latched 791-887 times per run, the bearing baseline never got past
// 8.8-11.9 m against the 25 m fusion needs, and ZERO bearings were ever fused in
// any run. What tripped it was not noise and not corners. Median excursion
// duration was ONE sample, median NET angle 0.43-0.52 degrees, 93-100% under 2
// degrees, and there were zero excursions while parked. Those are wheels crossing
// stones. Meanwhile the four scripted 90 degree corners were driven at 0.09 rad/s
// achieved, under the 0.3 rad/s threshold, so the gate caught none of the turns it
// exists for.
//
// The angle gate must therefore do two things that the rate gate cannot do at once:
// ignore a rocking jolt, and still catch a real corner.

#include <gtest/gtest.h>
#include <cmath>
#include "fusioncore/state.hpp"
#include "fusioncore/sensors/gnss.hpp"
#include "fusioncore/fusioncore.hpp"

using namespace fusioncore;
using namespace fusioncore::sensors;

namespace {

constexpr double kG = 9.80665;

// One run of a rover driving dead straight at 1 m/s while its wheels cross stones.
//
// Each jolt is a single 10 ms sample at 0.9 rad/s, which is 0.52 degrees, matching
// the measured median net angle, and 3x over the 0.3 rad/s rate threshold so the
// rate gate is guaranteed to latch on every one. Successive jolts ALTERNATE sign,
// because a wheel riding up over a stone and back rocks the robot one way and then
// the other. That is exactly the structure a signed integral cancels and an
// instantaneous rate cannot.
//
// Returns the longest bearing baseline the window ever reached.
double longest_baseline_with_stones(double angle_gate_deg, int * latched_out = nullptr)
{
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.gps_track_heading_max_yaw_rate = 0.3;
  cfg.gps_track_heading_min_dist     = 25.0;   // the rover's setting
  cfg.gps_track_heading_max_window_turn_deg = angle_gate_deg;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  const double dt = 0.01;
  double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0, best = 0.0;
  int latched = 0, sign = 1;

  // 90 s at 1 m/s is 90 m of straight driving, far more than the 25 m needed.
  for (int i = 0; i < 9000; ++i) {
    t += dt;
    // A jolt every 20 samples (0.2 s), one sample long, alternating direction.
    double wz = 0.0;
    if (i % 20 == 0) { wz = 0.9 * sign; sign = -sign; }

    yaw += wz * dt;
    x += std::cos(yaw) * dt;
    y += std::sin(yaw) * dt;

    fc.update_imu(t, 0.0, 0.0, wz, 0.0, 0.0, kG);
    if (i % 2 == 0) fc.update_encoder(t, 1.0, 0.0, wz);

    if (i % 100 == 0) {   // 1 Hz fixes
      GnssFix f;
      f.x = x; f.y = y; f.z = 0.0;
      f.hdop = f.sigma_xy = 0.5;
      f.vdop = f.sigma_z  = 1.0;
      f.satellites = 10;
      f.fix_type = GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
      const auto & d = fc.get_gnss_debug();
      best = std::max(best, d.track_heading_baseline_m);
      if (d.track_heading_state == TrackHeadingState::WINDOW_HAD_TURN) ++latched;
    }
  }
  if (latched_out) *latched_out = latched;
  return best;
}

}  // namespace

// ─── 1. The default must change nothing ──────────────────────────────────────

TEST(TrackHeadingTurnGate, DisabledByDefault) {
  EXPECT_DOUBLE_EQ(FusionCoreConfig{}.gps_track_heading_max_window_turn_deg, 0.0)
      << "the angle gate must ship off, so an existing config behaves as before";
}

TEST(TrackHeadingTurnGate, WindowTurnIsNotReportedWhenDisabled) {
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);
  for (int i = 1; i <= 50; ++i) fc.update_imu(i * 0.01, 0.0, 0.0, 0.5, 0.0, 0.0, kG);
  // -1 distinguishes "the gate is off" from "it measured zero rotation". Without
  // that distinction a bag cannot tell which gate was actually in force.
  EXPECT_DOUBLE_EQ(fc.get_gnss_debug().track_heading_window_turn_deg, -1.0);
}

// ─── 2. The defect itself: stones must not discard the window ────────────────

TEST(TrackHeadingTurnGate, RateGateIsDefeatedByStoneJolts) {
  int latched = 0;
  const double best = longest_baseline_with_stones(0.0, &latched);
  // This is the bug, asserted so it cannot quietly come back as the default.
  EXPECT_LT(best, 25.0)
      << "the rate gate reached " << best << " m, so this no longer reproduces #144";
  // Not one per fix: a fix only reports WINDOW_HAD_TURN if it reaches that branch,
  // and others report MOTION_UNSUITABLE or BASELINE_SHORT first. 20 of 90 fixes is
  // still the window being thrown away roughly every five seconds, forever.
  EXPECT_GT(latched, 15)
      << "expected the rate gate to latch repeatedly on single-sample jolts";
}

TEST(TrackHeadingTurnGate, AngleGateSurvivesStoneJolts) {
  int latched = 0;
  const double best = longest_baseline_with_stones(5.0, &latched);
  EXPECT_GE(best, 25.0)
      << "the angle gate only reached " << best << " m; fusion needs 25 m and the "
         "five validation bags reached 26.7-29.9 m";
  EXPECT_LT(latched, 20)
      << "the angle gate latched " << latched << " times on rocking jolts that "
         "should cancel in a signed integral";
}

// ─── 3. It must still catch a real turn ──────────────────────────────────────

TEST(TrackHeadingTurnGate, AngleGateStillCatchesARealCorner) {
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.gps_track_heading_min_dist = 5.0;
  cfg.gps_track_heading_max_window_turn_deg = 5.0;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  const double dt = 0.01;
  double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0;
  bool caught = false;
  double turned_at_catch = 0.0;

  // Straight for 10 s, then a genuine 90 degree corner at the 0.09 rad/s the
  // rover actually achieves. That rate is BELOW the 0.3 rad/s threshold, which is
  // why the rate gate misses it, and it takes 17.5 s to turn 90 degrees.
  for (int i = 0; i < 3000; ++i) {
    t += dt;
    const double wz = (t > 10.0) ? 0.09 : 0.0;
    yaw += wz * dt;
    x += std::cos(yaw) * dt;
    y += std::sin(yaw) * dt;
    fc.update_imu(t, 0.0, 0.0, wz, 0.0, 0.0, kG);
    if (i % 2 == 0) fc.update_encoder(t, 1.0, 0.0, wz);
    if (i % 100 == 0) {
      GnssFix f;
      f.x = x; f.y = y; f.z = 0.0;
      f.hdop = f.sigma_xy = 0.5;
      f.vdop = f.sigma_z  = 1.0;
      f.satellites = 10;
      f.fix_type = GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
      if (!caught &&
          fc.get_gnss_debug().track_heading_state == TrackHeadingState::WINDOW_HAD_TURN) {
        caught = true;
        turned_at_catch = std::abs(yaw) * 180.0 / M_PI;
      }
    }
  }
  EXPECT_TRUE(caught) << "a real 90 degree corner was never detected";
  // It must fire EARLY in the corner, not after the bearing has already been
  // measured across most of it.
  EXPECT_LT(turned_at_catch, 20.0)
      << "the corner was only caught after " << turned_at_catch << " degrees";
}

TEST(TrackHeadingTurnGate, RateGateMissesTheSlowCornerTheRoverActuallyDrives) {
  // The other half of #144, and the reason raising max_yaw_rate is not the fix:
  // at the rover's measured 0.09 rad/s the rate gate never fires at all.
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.gps_track_heading_max_yaw_rate = 0.3;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);
  const double dt = 0.01;
  double t = 0.0;
  for (int i = 0; i < 2000; ++i) {     // 20 s at 0.09 rad/s = 103 degrees
    t += dt;
    fc.update_imu(t, 0.0, 0.0, 0.09, 0.0, 0.0, kG);
    if (i % 2 == 0) fc.update_encoder(t, 1.0, 0.0, 0.09);
  }
  // Turned over 100 degrees and the instantaneous check never tripped.
  EXPECT_DOUBLE_EQ(fc.get_gnss_debug().track_heading_window_turn_deg, -1.0);
}

// ─── 4. The regression the old gate was written for must still hold ──────────

TEST(TrackHeadingTurnGate, TurnBetweenTwoFixesIsStillCaughtWithAngleGate) {
  // Guards the reason the check lives in update_imu() at all: a turn that starts
  // and finishes between two 1 Hz fixes used to be invisible, and the bearing was
  // then measured straight across the corner. 0.8 rad/s for 0.6 s is 27.5 degrees.
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.gps_track_heading_min_dist = 5.0;
  cfg.gps_track_heading_max_window_turn_deg = 5.0;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  const double dt = 0.01;
  double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0;
  TrackHeadingState state_after = TrackHeadingState::NOT_ATTEMPTED;

  for (int i = 0; i < 1800; ++i) {
    t += dt;
    const double wz = (t > 12.0 && t <= 12.6) ? 0.8 : 0.0;
    yaw += wz * dt;
    x += std::cos(yaw) * dt;
    y += std::sin(yaw) * dt;
    fc.update_imu(t, 0.0, 0.0, wz, 0.0, 0.0, kG);
    if (i % 2 == 0) fc.update_encoder(t, 1.0, 0.0, wz);
    if (static_cast<int>(std::round(t * 100)) % 100 == 0) {
      GnssFix f;
      f.x = x; f.y = y; f.z = 0.0;
      f.hdop = f.sigma_xy = 2.0;
      f.vdop = f.sigma_z  = 3.0;
      f.satellites = 10;
      f.fix_type = GnssFixType::GPS_FIX;
      fc.update_gnss(t, f);
      if (std::abs(t - 13.0) < 1e-6) state_after = fc.get_gnss_debug().track_heading_state;
    }
  }
  EXPECT_EQ(state_after, TrackHeadingState::WINDOW_HAD_TURN)
      << "27.5 degrees inside a single fix gap must still discard the window";
}

// ─── 5. The accumulator must not latch forever once it trips ─────────────────

TEST(TrackHeadingTurnGate, AccumulatorStaysBoundedWhileTurningContinuously) {
  // The invariant that matters: once it trips it must reset, so the reported angle
  // is a bounded sawtooth rather than the total angle ever turned. If it did NOT
  // reset it would sit permanently over threshold and latch on every subsequent
  // sample, which is the rate gate's failure reached by another route.
  //
  // Note the accumulator is NOT expected to decay when the robot drives straight.
  // It measures net angle turned since the window opened, and a window that has
  // turned 4 degrees really has turned 4 degrees however far it then goes straight.
  // It is cleared on trip and on window restart, nowhere else.
  FusionCoreConfig cfg;
  cfg.imu_has_magnetometer = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  cfg.gps_track_heading_max_window_turn_deg = 5.0;
  FusionCore fc(cfg);
  State s0; fc.init(s0, 0.0);

  const double dt = 0.01;
  double t = 0.0, worst = 0.0;
  // 20 s at 5 deg/s is 100 degrees turned in total, twenty times the threshold.
  for (int i = 0; i < 2000; ++i) {
    t += dt;
    fc.update_imu(t, 0.0, 0.0, 0.0873, 0.0, 0.0, kG);
    worst = std::max(worst, fc.get_gnss_debug().track_heading_window_turn_deg);
  }
  // One sample of overshoot past the threshold is expected and harmless.
  EXPECT_LT(worst, 5.0 + 1.0)
      << "the accumulator reached " << worst << " degrees after turning 100 in total, "
         "so it is not resetting when it trips";
  EXPECT_GT(worst, 4.0) << "the accumulator never got near the threshold at all";
}
