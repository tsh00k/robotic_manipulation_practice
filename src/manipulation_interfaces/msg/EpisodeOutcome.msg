# Published once per episode by task_executor_node when it reaches a terminal
# phase (DONE or FAILED) -- see task_executor_node.cpp's onStartEpisode()/onTimer().
# week2.md Stage J's episode runner is the intended consumer, writing one CSV row
# per message. Everything here is FSM-internal bookkeeping that only
# task_executor itself has; object pose and other physical state are read
# directly from mujoco_bridge's own ~/ground_truth topics by whoever wants them
# (including the episode runner itself), not relayed through this message.

bool success

# "NONE" when success is true. Otherwise the ExitReason (task_executor/fsm.hpp)
# that most recently sent the FSM into RECOVER -- i.e. the actual physical
# failure (TIMEOUT/SLIPPED/PLACE_MISSED/...), not RETRY_LIMIT_EXCEEDED itself,
# which only ever means the retries ran out, not what kept failing.
string failure_code

# Observation selected for this episode. The source is never inferred from the
# pose topic: it is recorded explicitly so oracle and vision runs stay separate.
string observation_source
float64 observation_confidence
float64 observation_residual_m
string observation_failure_layer
string observation_failure_reason

uint32 retries

# Parallel arrays, one entry per phase the episode actually visited, in the
# order it visited them (e.g. HOME, PREGRASP, ..., possibly RECOVER more than
# once on a retry). phase_durations_s[i] is elapsed_in_phase_s at the moment
# the FSM left phase_names[i].
string[] phase_names
float64[] phase_durations_s

# Parallel to phase_names. NaN means the selected waypoint source does not
# provide a Cartesian/IK diagnostic (e.g. legacy keyframe mode).
float64[] target_tcp_x_m
float64[] target_tcp_y_m
float64[] target_tcp_z_m
float64[] ik_position_error_m
float64[] joint_tracking_error_rad
float64[] actual_tcp_position_error_m
