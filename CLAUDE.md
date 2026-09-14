# CLAUDE.md

## 学习笔记与提问约定 → 见 [STUDY_NOTES_GUIDE.md](STUDY_NOTES_GUIDE.md)

这是一个**学习型项目**，笔记和提问过程本身是产出物的一部分。[STUDY_NOTES_GUIDE.md](STUDY_NOTES_GUIDE.md) 规定了笔记结构、每个 stage 的五步工作流、提问分类，以及 Claude 的一项固定职责：

**在每个 stage 讲解结束、写入笔记之前，主动列出"用户没问但值得注意的"问题（2~4 条）**，重点补可测试性（E 类）和可观测性（C 类）——详见该文档第 5 节。

跑验证前务必先确认没有遗留节点进程，起后台节点不要经过 `ros2 run`（见该文档 3.1，这个坑踩过两次）。

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

**坑：Claude 的沙盒工具执行环境可能没有 `docker`/`podman` 客户端，导致 `distrobox enter` 报 "we need a container manager"。** 现象：`/var/run/docker.sock` 存在（通常是软链到 `/run/host/run/docker.sock`，说明宿主机 docker 确实挂进来了），但 `docker`/`podman` 二进制不在 PATH 上，`distrobox` 内部找不到客户端就直接报错——不是权限问题，是缺 CLI。

**解法**：宿主机的 docker 二进制通常原样挂在 `/run/host/usr/bin/docker`，可以直接调用（`/run/host/usr/bin/docker --version` 验证）。把这个目录加进 PATH 前面再调 `distrobox`：

```bash
export PATH="/run/host/usr/bin:$PATH"
distrobox enter robotics-dev -- bash -ic '...'
```

**副作用，务必注意**：这样加的 PATH 会被 `distrobox`/`docker exec` 透传进容器内部 shell，导致容器里 `/run/host/usr/bin` 排到了容器自己 `/usr/bin` 前面。这会让 pyenv 的 "system" python3 判定失真——pyenv shim 找的是 PATH 上（除 shims 目录外）第一个 `python3`，现在变成了**宿主机的** python3（没有 `catkin_pkg`），而不是容器里装了 `catkin_pkg` 的那个，即使 `pyenv version` 依然正确显示 `system`。症状和下面"变体一"完全一样（`ModuleNotFoundError: No module named 'catkin_pkg'`），但根因是这条 PATH 污染，不是 `.python-version` 失效。

**对策**：进容器后，实际跑 ROS2/colcon 命令前，先把 `/run/host/*` 开头的 PATH 项过滤掉：

```bash
distrobox enter robotics-dev -- bash -ic '
  export PATH=$(echo "$PATH" | tr ":" "\n" | grep -v "^/run/host" | paste -sd: -)
  source /opt/ros/humble/setup.bash
  cd <repo_root>
  colcon build ...
'
```

这条只在"执行 `distrobox enter` 本身就需要先修 PATH 才能找到容器管理器"的沙盒环境里才会出现；在能直接访问 `docker`/`podman` 的正常环境里不会触发，可以跳过两条 PATH 处理。

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

**重要坑：pyenv 会劫持 python3，有两个不同的变体。**

**变体一（colcon）**：劫持 colcon 用的 python3，导致 `ModuleNotFoundError: No module named 'catkin_pkg'`。本仓库根目录已放置 `.python-version`（内容为 `system`），只要在仓库目录内（或其子目录）执行命令，pyenv 会自动切到系统 Python（3.10，带 `catkin_pkg`），不要删除这个文件。可用 `python3 -c "import catkin_pkg"` 快速验证。

**变体二（rclpy 脚本，Stage D 踩到）**：`.python-version` 对它**无效**。现象是跑任何 `import rclpy` 的脚本报：

```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/humble/lib/python3.10/site-packages/_rclpy_pybind11.cpython-311-...so' isn't present
```

注意错误信息里的 `python3.10` 和 `cpython-311` 打架——这是版本不匹配的特征。此时 `pyenv version` 会**正确地显示 `system`**，但 `python3` 仍然解析到 3.11.11，因为 `PATH` 最前面直接放着 `/home/anby/.pyenv/versions/3.11.11/bin`——**这个路径绕过了 pyenv 的 shim 机制，`.python-version` 管不着它**。

**对策：跑 rclpy 脚本一律显式用 `/usr/bin/python3`**，不要写 `python3`。诊断一句话：`python3 -c "import sys; print(sys.version)"` 如果不是 3.10 就是踩到了。

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
