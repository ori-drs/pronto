Header header
time curr_timestamp
time prev_timestamp
geometry_msgs/Transform relative_transform
float64[36] covariance
uint8 estimate_status

uint8 NO_DATA = 0
uint8 ESTIMATE_VALID = 1
uint8 ESTIMATE_INSUFFICIENT_FEATURES = 2
uint8 ESTIMATE_DEGENERATE = 3
uint8 ESTIMATE_REPROJECTION_ERROR = 4

