# Pick-and-place with a simulated Franka Panda

A Franka Panda in MuJoCo picks a 4 cm box off a table and puts it into a bin. A fixed RGB-D camera measures where the box and the bin are; the arm follows time-parameterized joint trajectories that respect the real Panda's limits; the camera checks at the end that the box is in the bin. Each episode can be recorded as raw data for training a vision-language-action (VLA) policy.

C++17, ROS 2 Humble, MuJoCo 3.3.7. Built as a learning project alongside Russ Tedrake's *Robotic Manipulation*: the robotics parts that matter for this task are written and tested here; mature libraries are used where they exist (MoveIt's time-optimal trajectory generation and mesh filter, OpenCV, Eigen).

![The camera's view at the start of an episode](results/week5_appearance_after.png)

*The camera's view at the start of three episodes: arm at its home pose, box and bin at random poses.*

## Results

Simulation, vision as the only source of the box and bin poses (the simulator's ground truth is used only to judge the result). Numbers from the last regression run; the protocol and the per-stage measurements are in the [Week 5 notes](Job_guides/my_study/week5.md).

| Measure | Result |
| --- | --- |
| Success on 40 held-out layouts (random box and bin poses) | **40 / 40**, no retries, no false success |
| Process checks on those 40 (no motion before the poses are latched, latched poses within limits, grasp and place targets taken from them, no gripper-opening alarm, box really in the bin) | all pass on all 40 |
| Peak joint velocity / acceleration, measured on the simulated arm | ≤ 0.95 × Panda's velocity limit; ≤ 1.01 × the acceleration used for planning (¼ of Panda's) |
| Vertical approach: TCP deviation from a straight line | 0.7 – 1.5 mm |
| Box slip in the gripper by the end of the place move | 2.4 – 4.8 mm, 0.6 – 7.9° |
| Episode length | about 10 s simulated |
| Recorded frames with RGB, depth and state/action at the same timestamp | 100 % |
| Unit and lint tests | 734, 0 failures |

## How it works

```
MuJoCo ─▶ mujoco_bridge ─┬─ joint states, TCP, attachment (100 Hz) ─────────────────────▶ task_executor
                         └─ RGB-D (10 Hz) ─▶ object_pose_estimator ─ box & bin pose ─────▶     │
task_executor ── timed joint trajectory, gripper command, reset ──▶ mujoco_bridge ◀──────────────┘
```

**Simulator interface (`mujoco_bridge`).** Steps the physics at 500 Hz and is the only source of time (`/clock`), TF and ground truth. Every 10 ms it publishes one message with everything from the same physics step: joint states, TCP pose, whether the box is held, and the joint targets the servos actually used in that step (the dataset's action). It executes timed joint trajectories by interpolating them every physics step into the position servos. Whether the box is held is decided only from what a real Franka Hand reports — the commanded close, the measured opening and whether the fingers have stopped — not from contact or the box's position.

**Perception (`mujoco_perception`).** Depth only. The robot is removed from each depth image by rendering its meshes at the current joint angles and masking matching pixels. Ten masked frames are averaged; the box and the bin are found as blobs in their height bands whose minimum-area rectangle has the right size, which gives x, y, height and yaw. No colour, no ground truth, no tracking: the poses are measured at the start of an episode and again after the release.

**Task execution (`task_executor`).** A phase state machine (home, pregrasp, grasp, close, lift, preplace, place, open, retract, verify) with explicit pass conditions and retries. The initial poses are latched once five consecutive measurements agree within 3 mm and 3°; no motion is commanded before that. Each phase's goal is solved by offline IK; the path to it is a joint-space line, or a straight TCP line for the vertical approaches (solved point by point every 5 mm); MoveIt's TOTG then times the path within the Panda's joint velocity and acceleration limits. The episode ends only when the box is inside the bin by a 5 mm margin, measured by the camera after the arm is back home.

**Data.** `scripts/record_episode.py` records one episode as a rosbag (RGB, depth, state and action at 10 Hz with identical timestamps) plus the dataset contract's metadata and a sidecar with the layout, the outcome and the code version.

Details — interfaces, defaults, failure behaviour — are in [docs/architecture.md](docs/architecture.md); the decisions and the alternatives they rejected are in [docs/adr/](docs/adr/).

## Engineering notes

A few things that turned out not to be what they looked like:

- **A position servo does not limit speed.** Commanding each phase's goal directly made the arm hit 4.5 rad/s and 99 rad/s² (the Panda allows 2.175 rad/s and 12.5 rad/s²): MuJoCo's position actuator pushes at its torque limit towards a step. Time-parameterized trajectories brought the peaks within the limits and cut the box's slip in the gripper from 7–16 mm to 2.4–4.8 mm.
- **Where to sample matters as much as how to interpolate.** Cubic Hermite interpolation of 10 ms samples overshot the acceleration limit by up to 24 % where TOTG switches from accelerating to braking; 1 ms samples, which the 2 ms physics step lands on exactly, reproduce the planned motion.
- **Start a trajectory from the command, not the measurement.** Without gravity compensation the arm sags a few milliradians below its servo target; starting the first trajectory from the measured joints made a small step and a 1.7× acceleration spike.
- **Up to 80 % of the RGB frames were lost in the middleware.** RGB and depth together exceeded Fast DDS's default 512 KB shared-memory segment, and the best-effort RGB was overwritten. A 4 MB segment fixed it; nothing in the perception code was wrong.
- **The home pose was hiding the bin.** At the original home pose the hand blocked the camera's view of the far side of the table; the arm now starts, resets and verifies at Franka's standard ready pose, which keeps it out of the workspace in the image.

## Limitations

Everything is in simulation: the attachment is judged by the simulator, there is no gravity compensation and no depth noise worth the name. Vision looks at the scene only at the start and at the end, so a box lost in transit is noticed only through the gripper opening and the final check. One box, one bin, one camera, one task sentence. The full list is in [architecture §10](docs/architecture.md#10-已知局限).

## Running it

The toolchain lives in a `robotics-dev` distrobox (Ubuntu 22.04, ROS 2 Humble, MuJoCo 3.3.7 in `/opt/mujoco-3.3.7`); see [CLAUDE.md](CLAUDE.md) for the environment and its pitfalls.

```bash
distrobox enter robotics-dev
cd <repo>
colcon build --symlink-install
source install/setup.bash
ros2 launch mujoco_bridge demo.launch.py scene_enabled:=true          # bridge, estimator, executor, RViz
ros2 topic pub --once /task_executor/start_episode std_msgs/msg/Empty {}
```

Optional: `box_x:=… bin_y:=… bin_yaw:=…` place the objects; `observation_source:=oracle` uses ground truth instead of the camera. To record an episode, run `/usr/bin/python3 scripts/record_episode.py` while the demo is up. Tests: `colcon test`; the online checks are the commands of [initial_box_probe.py](src/mujoco_perception/test/initial_box_probe.py).

## Repository layout

| Path | Contents |
| --- | --- |
| `src/mujoco_bridge` | simulator node, trajectory execution, attachment, camera, scene layout |
| `src/mujoco_perception` | robot mask, initial box and bin detection |
| `src/task_executor` | episode controller, phase state machine, IK targets, trajectory planning |
| `src/arm_kinematics` | Panda FK, Jacobian, damped least-squares IK (no ROS) |
| `src/manipulation_interfaces` | messages and services |
| `robot_description/mujoco` | vendored MuJoCo Menagerie Panda and the project's scenes |
| `docs/` | architecture, ADRs, plans |
| `Job_guides/my_study/` | weekly study notes: plans, measurements, mistakes and corrections |
