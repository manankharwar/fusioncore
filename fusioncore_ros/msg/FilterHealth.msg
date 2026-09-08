std_msgs/Header header

# Innovation norm: magnitude of the last accepted measurement residual per sensor.
# Rising norms with accepted fixes indicate adaptive noise is absorbing sensor drift.
# Rising norms with rejected fixes indicate a sensor calibration problem.
float64 gnss_innovation_norm
float64 imu_innovation_norm
float64 encoder_innovation_norm

# Position 1-sigma uncertainty from the filter covariance (meters)
float64 position_sigma_x
float64 position_sigma_y
float64 position_sigma_z

# Heading uncertainty in degrees (propagated from quaternion covariance via Jacobian)
float64 heading_sigma_deg

# Heading observability
# True once heading COULD be observable, which is not the same as known: it is set
# on distance travelled alone (gnss.heading_observable_distance), without checking
# that any heading measurement was ever fused. Read heading_sigma_deg above to find
# out whether heading is actually known. A run on 2026-09-06 reported
# heading_validated true with a yaw 1-sigma of 101 degrees for its whole duration.
bool   heading_validated
string heading_source  # NONE | GPS_TRACK | IMU_ORIENTATION | DUAL_ANTENNA | MAGNETOMETER

# GPS coast mode state
bool  gnss_in_coast
int32 gnss_consecutive_rejects

# Reason the most recent GNSS fix was rejected (empty until the first rejection):
# FIX_TYPE_LOW | HDOP_HIGH | VDOP_HIGH | MIN_SATS | CHI2_FAILED | DELAY_TOO_LARGE |
# IMPLAUSIBLE_JUMP. Quality-gate and delay rejects do NOT count toward
# gnss_outlier_count, so this is the only field that reveals why a fix was dropped.
string gnss_last_reject_reason

# Reason the most recent magnetometer reading was rejected (empty until the first reading):
# CHI2_FAILED | FIELD_MAGNITUDE
string mag_last_reject_reason

# Distance traveled since init (meters): heading observable above 5m
float64 distance_traveled_m

# Cumulative outlier rejection counts since filter init
int32 gnss_outlier_count
int32 imu_outlier_count
int32 encoder_outlier_count
int32 mag_outlier_count

# Stale-measurement rejections from inter-sensor clock skew: the sensor's stamps
# run more than max_measurement_delay behind the filter clock while another
# sensor's stamps drive it ahead (sensors not on a common time base). A climbing
# count means that sensor is NOT being fused at all; fix the drivers' clocks.
# These are not outliers: the data may be perfect, the timestamps disagree.
int32 imu_stale_reject_count
int32 encoder_stale_reject_count

# Every gate outcome that has occurred since init, as four parallel arrays.
# A name appears only once it has happened at least once, so an absent name
# means that gate has never fired. ACCEPTED is included, which is what makes
# "the gate never fired" distinguishable from "no fix ever arrived": if
# gnss:ACCEPTED is missing too, nothing was fed to the filter at all.
# Names are prefixed gnss:, gnss2: or mag: and match the *_last_reject_reason
# strings, plus gnss:NO_FIX_REPORTED for fixes the receiver marked NO_FIX. Those
# are dropped before the filter sees them, so they appear in no other counter.
# Timestamps are the measurement stamps of the first and last occurrence, in
# the same clock the filter is driven with, so a reason can be located in a bag
# without replaying it. Establishing that the chi2 gate had never fired on the
# 2026-09-06 run took a day of replaying because only a "last reason" existed.
string[]  outcome_names
int32[]   outcome_counts
float64[] outcome_first_seen
float64[] outcome_last_seen

# Can the GNSS outlier gate actually fire?
#
# The largest Mahalanobis distance the chi2 gate has seen all run, the threshold
# it is judged against, and how many fixes are behind that. A gate whose biggest
# innovation sits far below its threshold has not been passing good fixes, it has
# been incapable of rejecting a bad one, and no per-fix field shows that: every
# fix reports ACCEPTED, which reads exactly like a healthy run. On a rover log
# from 2026-09-06 the largest of 222 fixes was 39x below firing.
#
# Read gnss_chi2_max / gnss_chi2_threshold as a ratio. Near 1 means the gate is
# working at its limit. Below about 0.1 it cannot reject anything a real
# receiver would produce. -1 means no fix has been judged yet.
float64 gnss_chi2_max
float64 gnss_chi2_threshold
int32   gnss_chi2_samples
