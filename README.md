# robotic_manipulation_practice

A learning-oriented robotic manipulation project based on chapters 3-6 and 8 of Russ Tedrake's *Robotic Manipulation*. The core stack is C++17, ROS 2 Humble and MuJoCo.

## Current state

Weeks 1-3 established the Panda MuJoCo bridge, episode lifecycle, Cartesian task waypoints, model consistency checks, FK/Jacobian and offline damped differential IK. The next stages add RGB-D geometric perception and a policy-neutral boundary for scripted, MoveIt and learned motion backends.

The learned-policy boundary allows IL, RL, VLA and LeRobot adapters to produce trajectories or action chunks directly. A learned policy may own motion planning, collision avoidance and joint constraints; MoveIt remains an independent traditional backend and comparison baseline.

## Documentation

- [Architecture and project facts](docs/architecture.md)
- [Week 4: RGB-D and geometric pose estimation](Job_guides/my_study/week4.md)
- [Week 4.5: policy/data boundary and delivery](Job_guides/my_study/week4.5.md)
- [Container delivery plan](Job_guides/my_study/container_delivery_plan.md)

The current Distrobox environment is a development aid, not the reproducible delivery artifact. Public container validation must use a dedicated container home and an explicit `/workspace` mount; it must not depend on the host's full home directory, shell configuration or pyenv state.
