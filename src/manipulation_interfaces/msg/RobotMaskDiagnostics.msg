std_msgs/Header header
uint64 bridge_session
uint64 generation
uint64 sample_sequence
uint32 projected_pixels
uint32 masked_pixels
# Valid observed depth where robot depth is predicted; diagnostic only.
uint32 comparison_pixels
# Observed depth exceeds predicted robot depth plus tolerance; diagnostic only.
uint32 mismatch_pixels
float64 depth_tolerance_m
float64 elapsed_ms
string status
