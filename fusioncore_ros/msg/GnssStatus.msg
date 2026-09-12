std_msgs/Header header

# Whether this fix was accepted into the filter
bool accepted

# Human-readable rejection reason:
#   ACCEPTED | FIX_TYPE_LOW | HDOP_HIGH | VDOP_HIGH | MIN_SATS | CHI2_FAILED | DELAY_TOO_LARGE
string rejection_reason

# Mahalanobis distance squared (d^2 = nu^T * S^-1 * nu).
# -1.0 means the quality gate failed before the chi2 gate was reached.
float64 mahalanobis_sq

# Chi-squared acceptance threshold (configured outlier_threshold_gnss).
float64 chi2_threshold

float64 hdop
float64 vdop
int32   satellites
int32   fix_type

bool  in_coast_mode
int32 consecutive_rejects

# Filter position 1-sigma at the time of this fix (meters)
float64 position_sigma_x
float64 position_sigma_y

# Lever arm observability at the time of this fix
bool    lever_arm_used      # true if lever arm correction was applied
float64 heading_sigma_deg   # heading 1-sigma (degrees) used to decide lever arm eligibility

# Why GPS track heading did or did not fuse on this fix:
#   NOT_ATTEMPTED | FUSED | STRONGER_SOURCE | MOTION_UNSUITABLE | BASELINE_SHORT
#   | SIGMA_HIGH | CHI2_FAILED
# On a rover with no magnetometer and no dual antenna this is the ONLY thing that
# bounds yaw, and when it silently declines the whole filter degrades behind it.
string  track_heading_state
float64 track_heading_baseline_m   # displacement since the reference fix (m)
float64 track_heading_sigma_rad    # sigma_xy/baseline, -1.0 if not computed

# Cross-check on whichever ABSOLUTE heading source is in charge, in degrees:
# the filter's yaw minus the GPS track bearing, median over recent straight
# segments, plus how many segments went into it. Positive means the heading in
# use points counter-clockwise of the direction the robot is actually going.
# Only populated while an absolute source outranks GPS track heading, which is
# exactly when nothing else is checking that source. Watch this live during a
# calibration drive: a value that stays put across several headings is a
# mounting rotation or declination, one that swings with heading is hard iron.
# 0 samples means no opinion yet.
float64 heading_vs_track_deg
int32   heading_vs_track_n

# Fix-to-fix continuity limit actually in force (m), and whether it was measured
# from this receiver rather than configured. 0.0 = gate not active yet.
float64 continuity_limit_m
bool    continuity_learned
