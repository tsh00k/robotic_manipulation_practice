# AGENT.md

本文件是 [CLAUDE.md](CLAUDE.md) 的代理执行摘要。环境细节以 `CLAUDE.md` 为准；学习流程以 [STUDY_NOTES_GUIDE.md](STUDY_NOTES_GUIDE.md) 为准；frame、关节和模型等项目事实以 [docs/architecture.md](docs/architecture.md) 为唯一权威来源。

## 学习型项目约定

本项目把代码、验证、讲解和学习笔记都视为产出。每个 stage 必须按以下顺序完成：

1. 写代码。
2. `colcon build`，并实际运行或测试验证。
3. 讲解机制、至少一项权衡/替代方案，以及至少一种失败模式和验证方法。
4. 由用户针对未讲清或未涉及的内容追问；不要反过来出题考用户。
5. 将原始提问、回答、实测结果、失败模式和排查过程写入当周 `weekN.md`，之后才能进入下一 stage。

讲解结束、写笔记前，主动指出 2~4 个用户尚未问但值得注意的问题，优先覆盖可观测性和可测试性。问题按 `STUDY_NOTES_GUIDE.md` 分流：手边代码或一次实验能回答的现在处理；真正缺少参照系的才进入悬挂清单。

## 学习笔记的 stage 结构

以 `Job_guides/my_study/week3.md` 的已完成 stage 和 `STUDY_NOTES_GUIDE.md` 为范例。每个 stage 固定以 `N.0 一句话总结`、`N.1 改动清单与验证结果` 开头；随后按主题安排机制、概念、用户追问和权衡，原始提问放在对应主题中，不单设按时间堆叠的问答节。后段保留 `失败模式与验证手段`、实际发生时的 `排查记录`、`你没问但值得注意的` 和 `本阶段边界与后续`；后续决策归入后续讨论，不把新的概念讲解追加在末尾。新增主题时先判断它属于哪一段，再调整编号和目录，不改变固定骨架。

每个 `weekN.md` 的 Stage 在该周内部从 `Stage 1` 开始连续编号；新增 Stage 时重排该周标题、目录、正文交叉引用和锚点。不要沿用跨周的字母或上一周的编号。只有确实需要独立学习周期时才新增 week 文件；同一接口周的前置场景、任务和数据契约应合并到一个 week，避免人为拆分。

## 开发环境

ROS2、colcon 和 MuJoCo 工具链位于 Ubuntu 22.04 + ROS2 Humble 的 Distrobox 容器 `robotics-dev`，宿主机不保证安装这些工具。正常入口：

```bash
distrobox enter robotics-dev
```

非交互执行优先使用 `bash -ic`，让容器的 `.bashrc` 加载 ROS2 和 MuJoCo 环境：

```bash
distrobox enter robotics-dev -- bash -ic '
  source /opt/ros/humble/setup.bash
  cd <repo_root>
  colcon build
'
```

如果当前执行环境已经位于 `robotics-dev` 且工具可用，不要再嵌套进入 Distrobox。

### 沙盒找不到容器管理器

若 `distrobox enter` 报 `we need a container manager`，但 `/run/host/usr/bin/docker` 存在，只为 Docker 创建最小 PATH shim：

```bash
mkdir -p /tmp/docker_shim
ln -sf /run/host/usr/bin/docker /tmp/docker_shim/docker
export PATH="/tmp/docker_shim:$PATH"
```

不要把整个 `/run/host/usr/bin` 加入 PATH；宿主二进制可能污染 Python 解析，甚至因 glibc 版本不兼容导致 `distrobox-enter` 自身失败。进入容器后应从 PATH 过滤 `/tmp/docker_shim` 和裸的 `/home/anby/.pyenv/versions/.../bin` 路径。

## Python 注意事项

仓库根目录的 `.python-version` 固定为 `system`，不要删除。它让 colcon 使用容器内 Python 3.10 和 `catkin_pkg`。

如果 PATH 前面直接存在 `/home/anby/.pyenv/versions/3.11.11/bin`，该路径会绕过 pyenv shim，导致 colcon 或 ROS2 Python 扩展发生 3.10/3.11 混用。构建和测试前应过滤这类路径。

运行 `rclpy` 脚本一律显式使用：

```bash
/usr/bin/python3 <script>
```

可用以下命令诊断 Python 版本；应为 3.10：

```bash
python3 -c "import sys; print(sys.version)"
```

## 已安装依赖与模型

- ROS2 Humble 位于 `/opt/ros/humble`。
- MuJoCo 3.3.7 C/C++ 库位于 `/opt/mujoco-3.3.7`；交互 shell 已配置 `MUJOCO_DIR`、`PATH`、`LD_LIBRARY_PATH` 和 `CMAKE_PREFIX_PATH`。
- 未安装 MuJoCo Python 绑定；核心项目使用 C++ API。
- 系统 ROS 包包括 `franka_description`、Panda MoveIt 资源、`control_msgs`、`xacro` 和 joint-state publisher。
- MuJoCo Menagerie 的 Panda MJCF 与 mesh 已 vendor 到 `robot_description/mujoco/franka_emika_panda/`。

## 构建与测试

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --all --verbose
```

可按需添加 `--packages-select <packages...>` 缩小范围。`build/`、`install/`、`log/` 已被忽略，不得提交。

## 运行验证卫生

启动 ROS2/MuJoCo 验证前，先确认没有遗留 bridge：

```bash
ps -eo pid,comm | awk '$2 ~ /^mujoco_bridge/ {print $1}'
ros2 topic info /clock
```

前者应无输出；运行时 `/clock` 应只有一个 publisher。`ros2 node list` 会对同名节点去重，不能作为进程干净的权威判据。

后台验证直接启动安装目录中的真实可执行文件。不要通过 `ros2 run` 或 `ros2 launch` wrapper 后只杀 wrapper PID，否则可能遗留继续运行的子进程。

## 修改边界

- 对已有文档做增量修改前，先审阅目标章节的层级、顺序、目录锚点和主题重复；先确定合适的组织方式，再编辑内容，并在修改后复核结构与链接。
- 不修改 `Job_guides/` 下的原始课程计划书；学习记录写入 `Job_guides/my_study/weekN.md`。
- 结论性的项目事实必须同步到 `docs/architecture.md`，不要只留在周记。
- 架构决策按一项一文件写入 `docs/adr/`。
- 不提交构建产物、日志、个人编辑器设置或与当前任务无关的修改。
- 工作区可能已有用户修改；只暂存和提交当前任务明确涉及的文件。
