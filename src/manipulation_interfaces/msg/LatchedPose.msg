# One object's part of InitialPoseLatch (Week 4.1 Stage 7).
# false: the executor does not wait for this object (the bin, in a scene without one).
bool required
bool latched
# "LATCHED", or "WAITING:<why>": the reason of the last unmeasured estimate, "COLLECTING n/N",
# or "INCONSISTENT <mm> mm <deg> deg" while the last N estimates disagree.
string status
# The latched pose in the world frame; NaN until latched. For the bin, z is the inner floor
# surface and yaw_rad the long side. Taken from the newest of the agreeing estimates.
geometry_msgs/Point position
float64 yaw_rad
# Which estimate it is: the bridge sample sequence and the stamp of that
# InitialBoxPose / InitialBinPose message (0 until latched).
uint64 source_sequence
builtin_interfaces/Time source_stamp
# How much the agreeing estimates differed: the larger of their x and y ranges, and the
# smallest arc that contains their yaws. NaN until latched.
float64 position_spread_m
float64 yaw_spread_rad
