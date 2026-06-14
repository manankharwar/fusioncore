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
bool   heading_validated
string heading_source  # NONE | GPS_TRACK | IMU_ORIENTATION | DUAL_ANTENNA | MAGNETOMETER

# GPS coast mode state
bool  gnss_in_coast
int32 gnss_consecutive_rejects

# Distance traveled since init (meters): heading observable above 5m
float64 distance_traveled_m

# Cumulative outlier rejection counts since filter init
int32 gnss_outlier_count
int32 imu_outlier_count
int32 encoder_outlier_count
int32 mag_outlier_count
