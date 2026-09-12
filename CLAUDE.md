# CLAUDE.md

## 开发环境：distrobox

本项目的 ROS2/编译工具链不在宿主机上，而是在名为 `robotics-dev` 的 distrobox 容器（Ubuntu 22.04 + ROS2 Humble）里。

**任何涉及 `ros2`、`colcon`、编译、运行仿真的命令，必须先进入这个容器，不要在宿主机上直接跑（宿主机没有安装 ROS2/colcon/MuJoCo）。**

进入方式：

```bash
distrobox enter robotics-dev -- bash -lc '<command>'
```

或交互式：

```bash
distrobox enter robotics-dev
```

容器内已确认可用：
- `colcon`、`/opt/ros/humble`（需要 `source /opt/ros/humble/setup.bash` 才能用 `ros2` 命令，`.bashrc` 里已配置好，交互式 shell 自动生效）
- **MuJoCo 3.3.7 的 C/C++ 库**安装在 `/opt/mujoco-3.3.7`（非包管理器安装，是本机手动装的）。CMake 的 `find_package(mujoco REQUIRED)` 默认就能找到它（CMake 的 `CMAKE_SYSTEM_PREFIX_PATH` 本身包含 `/opt`），不需要额外设置 `CMAKE_PREFIX_PATH`。运行 `simulate`/`compile` 等 MuJoCo 自带工具需要 `PATH`/`LD_LIBRARY_PATH`，已在 `~/.bashrc` 追加：
  ```bash
  export MUJOCO_DIR="/opt/mujoco-3.3.7"
  export PATH="$MUJOCO_DIR/bin:$PATH"
  export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:$LD_LIBRARY_PATH"
  export CMAKE_PREFIX_PATH="$MUJOCO_DIR:$CMAKE_PREFIX_PATH"
  ```
  （最后一条其实非必需，保留是为了显式、不依赖 CMake 默认搜索路径这个隐式行为。）
  注意：这些变量只在**交互式 shell**里生效（`.bashrc` 开头有非交互式 shell 直接 return 的 guard），用 `distrobox enter robotics-dev -- bash -lc '...'` 跑非交互命令时不会加载，需要用 `bash -ic '...'` 或者先手动 `source ~/.bashrc` / `export MUJOCO_DIR=...`。
- 容器内网络访问正常（能连 github.com 等），此前判断"无网络"是误判。

- **ROS2 机器人描述/MoveIt 资源已装为系统包**（apt，Humble）：`ros-humble-franka-description`（官方 Panda/`fer` URDF+mesh）、`ros-humble-moveit-resources-panda-moveit-config`（现成 MoveIt SRDF/kinematics/planning 配置）、`ros-humble-control-msgs`（`mujoco_bridge` 编译依赖）、`ros-humble-xacro`、`ros-humble-joint-state-publisher(-gui)`（这两个是上面包的依赖，自动装的）。模型来源和具体路径见 [docs/architecture.md](docs/architecture.md) 第0节。
- MuJoCo MJCF（Panda + 平行夹爪）已从 [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) vendor 进 `robot_description/mujoco/franka_emika_panda/`（约33MB，含 mesh），不走 apt（没有对应包）。

**重要坑：pyenv 会劫持 colcon 用的 python3，导致 `ModuleNotFoundError: No module named 'catkin_pkg'`。** 本仓库根目录已放置 `.python-version`（内容为 `system`），只要在仓库目录内（或其子目录）执行命令，pyenv 会自动切到系统 Python（3.10，带 `catkin_pkg`），不要删除这个文件。如果不确定是否生效，可用 `python3 -c "import catkin_pkg"` 快速验证。

容器内已确认**不可用/待装**：
- **MuJoCo 的 Python 绑定**（`pip show mujoco` 未找到，`import mujoco` 失败）。如果后续需要 Python 侧原型验证（比如快速验证 MJCF、可视化调试），需要 `pip install mujoco`。核心项目走 C++ API，这个不是阻塞项。

## 已验证可编译

`robot_description` 和 `mujoco_bridge` 两个包已用以下命令验证可以成功 `colcon build`（详见 Job_guides 第15节第一周任务）：

```bash
distrobox enter robotics-dev -- bash -ic '
source /opt/ros/humble/setup.bash
cd <repo_root>
colcon build --packages-select robot_description mujoco_bridge --symlink-install
'
```

`build/`、`install/`、`log/` 已加入 `.gitignore`，不要提交。
