# One block of an object's height band, whether or not it was taken for the object
# (Week 4.1 Stages 5 and 6). Reported so that a miss can be explained from the message alone.
uint32 pixels
# Sides of the block's minimum-area rectangle in the table plane, metres: along the yaw
# direction and across it. The block matches when both are within the side tolerance of the object's sides.
float64 side_along_m
float64 side_across_m
float64 center_x
float64 center_y
bool matches
