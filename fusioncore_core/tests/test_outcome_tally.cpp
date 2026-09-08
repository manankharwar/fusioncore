#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"
#include <cmath>

using namespace fusioncore;

// The tally exists because a "last reason" field cannot answer the two
// questions you actually have in front of a bag: how often did this gate fire,
// and when. On the 2026-09-06 run it took a day of replaying to establish that
// the chi2 gate had never fired at all.
namespace {

FusionCoreConfig rover_config() {
  FusionCoreConfig cfg;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.gnss.base_noise_xy = 1.0;
  cfg.gnss.base_noise_z  = 1.0;
  cfg.gnss.min_satellites = 6;
  cfg.outlier_rejection = true;
  cfg.outlier_threshold_gnss = 16.27;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  return cfg;
}

sensors::GnssFix good_fix(double x, double y) {
  sensors::GnssFix fix;
  fix.x = x; fix.y = y; fix.z = 0.0;
  fix.hdop = 1.0; fix.vdop = 1.0;
  fix.satellites = 12;
  fix.fix_type = sensors::GnssFixType::DGPS_FIX;
  return fix;
}

int tally_total(const std::array<OutcomeTally, GNSS_REJECTION_REASON_COUNT>& t) {
  int n = 0;
  for (const auto& e : t) n += e.count;
  return n;
}

const OutcomeTally& gnss_entry(const FusionCore& fc, GnssRejectionReason r) {
  return fc.gnss_outcome_tally()[static_cast<int>(r)];
}

} // namespace

// ─── An accepted fix is counted, not just a rejected one ────────────────────
// This is the whole point of including ACCEPTED. Without it, an empty tally is
// ambiguous: it could mean the gates never fired, or that no fix ever reached
// the filter. Those two call for completely different debugging.
TEST(OutcomeTallyTest, AcceptedFixIsCounted) {
  FusionCore fc(rover_config());
  State s0;
  fc.init(s0, 0.0);

  EXPECT_EQ(tally_total(fc.gnss_outcome_tally()), 0);

  ASSERT_TRUE(fc.update_gnss(0.2, good_fix(0.0, 0.0)));

  EXPECT_EQ(gnss_entry(fc, GnssRejectionReason::ACCEPTED).count, 1);
  EXPECT_EQ(tally_total(fc.gnss_outcome_tally()), 1);
}

// ─── Every fix lands in exactly one bucket ──────────────────────────────────
// update_gnss either throws or reaches one terminal outcome, so the tally total
// must equal the number of fixes fed in. This is the invariant that catches a
// future return path added without a note_gnss_outcome call, and it caught a
// real double count while this was being written: apply_gnss_update records its
// own rejection, so the caller recording it again counted rejects twice.
TEST(OutcomeTallyTest, EveryFixIsCountedExactlyOnce) {
  FusionCore fc(rover_config());
  State s0;
  fc.init(s0, 0.0);

  const double g = 9.80665;
  int fixes_fed = 0;

  for (int step = 1; step <= 2000; ++step) {
    const double t = step * 0.01;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);

    if (step % 20 != 0) continue;

    if (step % 200 == 0) {
      sensors::GnssFix bad = good_fix(0.0, 0.0);
      bad.satellites = 2;                 // trips the quality gate
      fc.update_gnss(t, bad);
    } else if (step % 300 == 0) {
      fc.update_gnss(t, good_fix(400.0, 400.0));  // trips the chi2 gate
    } else {
      fc.update_gnss(t, good_fix(0.0, 0.0));
    }
    ++fixes_fed;
  }

  ASSERT_GT(fixes_fed, 50);
  EXPECT_EQ(tally_total(fc.gnss_outcome_tally()), fixes_fed);
  EXPECT_GT(gnss_entry(fc, GnssRejectionReason::ACCEPTED).count, 0);
  EXPECT_GT(gnss_entry(fc, GnssRejectionReason::MIN_SATS).count, 0);
}

// ─── The reason recorded is the gate that actually fired ────────────────────
TEST(OutcomeTallyTest, ReasonLandsInItsOwnBucket) {
  FusionCore fc(rover_config());
  State s0;
  fc.init(s0, 0.0);

  sensors::GnssFix no_fix = good_fix(0.0, 0.0);
  no_fix.fix_type = sensors::GnssFixType::NO_FIX;

  for (int i = 0; i < 5; ++i) fc.update_gnss(0.1 * (i + 1), no_fix);

  EXPECT_EQ(gnss_entry(fc, GnssRejectionReason::FIX_TYPE_LOW).count, 5);
  EXPECT_EQ(gnss_entry(fc, GnssRejectionReason::ACCEPTED).count, 0);
  EXPECT_EQ(gnss_entry(fc, GnssRejectionReason::MIN_SATS).count, 0);
}

// ─── first_seen and last_seen bracket the occurrences ───────────────────────
// The timestamps are what let you jump straight to the right window in a bag
// instead of replaying the whole run to find out when a gate was active.
TEST(OutcomeTallyTest, FirstAndLastSeenBracketTheOccurrences) {
  FusionCore fc(rover_config());
  State s0;
  fc.init(s0, 0.0);

  sensors::GnssFix no_fix = good_fix(0.0, 0.0);
  no_fix.fix_type = sensors::GnssFixType::NO_FIX;

  fc.update_gnss(1.5, no_fix);
  fc.update_gnss(2.5, no_fix);
  fc.update_gnss(7.25, no_fix);

  const OutcomeTally& t = gnss_entry(fc, GnssRejectionReason::FIX_TYPE_LOW);
  EXPECT_EQ(t.count, 3);
  EXPECT_DOUBLE_EQ(t.first_seen, 1.5);
  EXPECT_DOUBLE_EQ(t.last_seen, 7.25);

  // An outcome that never happened keeps the sentinel, so a reader can tell
  // "never" apart from "at time zero".
  const OutcomeTally& never = gnss_entry(fc, GnssRejectionReason::CHI2_FAILED);
  EXPECT_EQ(never.count, 0);
  EXPECT_LT(never.first_seen, 0.0);
  EXPECT_LT(never.last_seen, 0.0);
}

// ─── Counts belong to one run ───────────────────────────────────────────────
TEST(OutcomeTallyTest, ResetClearsTheTally) {
  FusionCore fc(rover_config());
  State s0;
  fc.init(s0, 0.0);

  sensors::GnssFix no_fix = good_fix(0.0, 0.0);
  no_fix.fix_type = sensors::GnssFixType::NO_FIX;
  fc.update_gnss(1.0, no_fix);
  ASSERT_EQ(tally_total(fc.gnss_outcome_tally()), 1);

  fc.reset();
  EXPECT_EQ(tally_total(fc.gnss_outcome_tally()), 0);
  EXPECT_LT(gnss_entry(fc, GnssRejectionReason::FIX_TYPE_LOW).first_seen, 0.0);

  fc.init(s0, 0.0);
  fc.update_gnss(1.0, no_fix);
  EXPECT_EQ(gnss_entry(fc, GnssRejectionReason::FIX_TYPE_LOW).count, 1);
}

// ─── The magnetometer keeps its own tally ───────────────────────────────────
// This is the number issue #93 needs: the rejection rate, not the last reason.
TEST(OutcomeTallyTest, MagnetometerOutcomesAreCounted) {
  FusionCoreConfig cfg = rover_config();
  cfg.mag.noise_rad      = 0.001;   // very tight: a wrong heading is an outlier
  cfg.mag.chi2_threshold = 9.21;
  cfg.mag.declination_rad = 0.0;

  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);   // level, so no tilt correction

  // Heading comes out as atan2(mx, my), so (0, 1, 0) is yaw 0 and agrees with
  // the initial identity quaternion: accepted.
  ASSERT_TRUE(fc.update_magnetometer(0.01, 0.0, 1.0, 0.0));
  EXPECT_EQ(fc.mag_outcome_tally()[static_cast<int>(MagRejectionReason::ACCEPTED)].count, 1);

  // (0, -1, 0) is yaw 180 degrees out, which the tight gate rejects.
  for (int i = 0; i < 3; ++i) fc.update_magnetometer(0.02 + 0.01 * i, 0.0, -1.0, 0.0);

  const OutcomeTally& rej =
    fc.mag_outcome_tally()[static_cast<int>(MagRejectionReason::CHI2_FAILED)];
  EXPECT_EQ(rej.count, 3);
  EXPECT_DOUBLE_EQ(rej.first_seen, 0.02);
}

// ─── The tally must not change what last_reject_reason means ────────────────
// That field is documented as sticky: it names the most recent rejected fix and
// survives later accepted ones, which is what makes it useful after the fact.
// Counting ACCEPTED in the tally is the addition; it must not leak into here.
TEST(OutcomeTallyTest, AcceptedFixDoesNotClearTheLastRejectReason) {
  FusionCore fc(rover_config());
  State s0;
  fc.init(s0, 0.0);

  sensors::GnssFix no_fix = good_fix(0.0, 0.0);
  no_fix.fix_type = sensors::GnssFixType::NO_FIX;
  fc.update_gnss(0.1, no_fix);
  ASSERT_EQ(fc.get_status().gnss_last_rejection_reason,
            GnssRejectionReason::FIX_TYPE_LOW);

  ASSERT_TRUE(fc.update_gnss(0.2, good_fix(0.0, 0.0)));

  EXPECT_EQ(fc.get_status().gnss_last_rejection_reason,
            GnssRejectionReason::FIX_TYPE_LOW);
  EXPECT_EQ(gnss_entry(fc, GnssRejectionReason::ACCEPTED).count, 1);
}

// ─── The gate's own headroom is recorded, not just per-fix values ────────────
// A single small Mahalanobis distance is normal. It is the LARGEST across a
// whole run staying far below the threshold that means the gate cannot fire,
// and no per-fix field shows that because every fix honestly reports ACCEPTED.
TEST(OutcomeTallyTest, GateHeadroomIsTracked) {
  FusionCore fc(rover_config());
  State s0;
  fc.init(s0, 0.0);

  EXPECT_LT(fc.get_status().gnss_chi2_max, 0.0) << "should be -1 before any fix";
  EXPECT_EQ(fc.get_status().gnss_chi2_samples, 0);

  const double g = 9.80665;
  for (int step = 1; step <= 1500; ++step) {
    const double t = step * 0.01;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 100 == 0) fc.update_gnss(t, good_fix(0.0, 0.0));   // agrees perfectly
  }

  const auto seen = fc.get_status();
  ASSERT_GT(seen.gnss_chi2_samples, 5) << "no fix was judged by the gate";
  EXPECT_GE(seen.gnss_chi2_max, 0.0)   << "maximum never left its -1 sentinel";
  EXPECT_GT(seen.gnss_chi2_threshold, 0.0) << "threshold not reported";

  // The number that matters is a RUNNING MAXIMUM, so it must never fall and
  // must rise for a fix that genuinely disagrees. Deliberately not asserting a
  // particular headroom here: how much a fix disagrees depends on filter tuning
  // and the scenario, whereas "the maximum is tracked and responds" is the
  // property this field promises.
  const double before = seen.gnss_chi2_max;
  fc.update_gnss(15.5, good_fix(400.0, 400.0));
  const auto after = fc.get_status();
  EXPECT_GT(after.gnss_chi2_max, before)
    << "a wildly disagreeing fix did not raise the recorded maximum";
  EXPECT_GT(after.gnss_chi2_samples, seen.gnss_chi2_samples);

  // And it never goes back down when agreeing fixes resume.
  fc.update_gnss(15.6, good_fix(0.0, 0.0));
  EXPECT_GE(fc.get_status().gnss_chi2_max, after.gnss_chi2_max)
    << "the maximum decreased, so it is not a running maximum";
}
