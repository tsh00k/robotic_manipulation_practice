# Read-only status of the executor's wait for the initial box and bin poses (Week 4.1 Stage 7).
# Published while an episode is active in the vision source; the task targets of the episode
# come from the latched box pose and the executor sends no command before it is LATCHED.
uint8 WAITING=0
uint8 LATCHED=1
uint8 FAILED=2  # the wait ended in VISION_LATCH_TIMEOUT
std_msgs/Header header  # simulation time of the report, frame_id "world"
uint64 bridge_session
uint64 generation
uint8 state
LatchedPose box
LatchedPose bin
