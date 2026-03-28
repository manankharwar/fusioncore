std_msgs/Header header

# The azimuth angle in unit corresponding to the 'unit' field.
float64 azimuth

# Variance of the measurement (std dev squared) in squared 'unit' field.
float64 variance

# Unit of measurement
uint8 UNIT_RAD=0
uint8 UNIT_DEG=1
uint8 unit

# Orientation convention
uint8 ORIENTATION_ENU=0
uint8 ORIENTATION_NED=1
uint8 orientation

# North reference
uint8 REFERENCE_MAGNETIC=0
uint8 REFERENCE_GEOGRAPHIC=1
uint8 REFERENCE_UTM=2
uint8 reference
