# Architecture & Conventions

> 对应 Job_guides 第15节任务2：固定 Panda 关节命名、world/base/tool/camera/object frame 约定。
> 任何后续模块都不得引入与本文档不一致的第二套约定。

## 0. 模型来源

不在本仓库 vendor 机器人描述文件，统一走 apt 安装的系统 ROS2 包，通过 `get_package_share_directory()` 引用：

| 用途 | 包名 | 安装方式 | 关键路径 |
| --- | --- | --- | --- |
| 官方 URDF/SRDF/mesh | `franka_description` (1.0.1, Humble) | `sudo apt install ros-humble-franka-description` | `share/franka_description/robots/fer/fer.urdf.xacro`（7-DoF 臂，官方新命名 `fer`，即原 "Panda"）、`share/franka_description/end_effectors/franka_hand/franka_hand.urdf.xacro`（平行夹爪） |
| MoveIt 2 现成配置（SRDF、kinematics、joint_limits、OMPL/Pilz/CHOMP planning yaml） | `moveit_resources_panda_moveit_config` (2.0.7, Humble) | `sudo apt install ros-humble-moveit-resources-panda-moveit-config` | `share/moveit_resources_panda_moveit_config/config/` |
| MuJoCo MJCF + mesh | 无 apt 包，从 [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) `franka_emika_panda/` 子目录复制 | vendor 进本仓库 | `robot_description/mujoco/franka_emika_panda/panda.xml`（含夹爪）、`panda_nohand.xml`（仅7轴臂） |

选型理由：
- Franka 官方仓库已把经典 "Panda" 型号重命名为 `fer`（Franka Emika Robot），物理几何/DH 参数与原 Panda 完全一致，`joint_limits.yaml` 数值可交叉验证。
- MJCF 没有对应 apt 包，且体积不大（~33MB mesh），vendor 进仓库比运行时依赖一个未打包的第三方仓库更可复现；已在 `.gitignore` 外单独保留（不受 build/install/log 忽略规则影响）。
- `moveit_resources_panda_moveit_config` 虽然包名仍叫 `panda`（历史遗留测试资源包名，未随官方改名同步），但里面的 joint_limits/kinematics 数值与 `franka_description` 的 `fer` 一致，可直接作为 Chap 6 `motion_planner` 包的参考配置起点。

## 1. Frame 约定

| Frame | 父 frame | 说明 | 发布方式 |
| --- | --- | --- | --- |
| `world` | - | 全局固定 frame，仿真与真实世界坐标原点 | static |
| `base_link` (= URDF `link0`) | `world` | 机器人基座，固定于桌面 | static |
| `link1` .. `link7` | 链式（`link{i}` 的父为 `link{i-1}`） | Panda 7-DoF 关节链，link/joint 命名沿用 `franka_description`（无 `panda_` 前缀，默认 `arm_prefix` 为空） | dynamic（`mujoco_bridge` 按仿真步发布） |
| `link8` | `link7` | 固定 frame（fixed joint），法兰基准，`franka_description` 中用作挂载末端执行器的连接点 | static（相对 link7 固定） |
| `hand` / `hand_tcp` | `link8` | 夹爪基座 / 工具中心点（TCP），抓取位姿以 `hand_tcp` 为参考 | static（相对 link8 固定，来自 `franka_hand` xacro 的 `hand_tcp_joint`） |
| `leftfinger` / `rightfinger` | `hand` | 两指夹爪指尖 link，关节为 `finger_joint1`/`finger_joint2` | dynamic |
| `camera_link` / `camera_optical_frame` | `world` 或固定支架 link | RGB-D 相机外参；只能由 TF 发布一份 | static |
| `object` | `world` | 目标物体 ground-truth/估计位姿 | dynamic（oracle 或感知发布） |

规则：

- 相机外参只允许在 `mujoco_bridge` 中以一份 static TF 发布，禁止感知节点手写第二套外参（对应第3.2节）。
- ground truth 与视觉估计的 `object` frame 使用同一命名，但通过不同 topic 区分（oracle vs vision），不得混用。
- 抓取/规划模块统一以 `hand_tcp` 作为末端参考 frame，不直接用 `link8` 或 `hand`（TCP 已经把夹爪长度和默认 45° 旋转的偏移量算进去，避免每个模块各自加一遍偏移）。

## 2. 关节命名与顺序

以 `franka_description`（`fer.urdf.xacro`）与 MuJoCo Menagerie（`panda.xml`）交叉核对，两者关节命名和限位数值完全一致：

| 关节 | 类型 | 下限 (rad) | 上限 (rad) | 最大速度 (rad/s) | 最大力矩 (N·m) | 备注 |
| --- | --- | --- | --- | --- | --- | --- |
| `joint1` | revolute | -2.8973 | 2.8973 | 2.175 | 87 | |
| `joint2` | revolute | -1.7628 | 1.7628 | 2.175 | 87 | |
| `joint3` | revolute | -2.8973 | 2.8973 | 2.175 | 87 | |
| `joint4` | revolute | -3.0718 | -0.0698 | 2.175 | 87 | 全负区间，注意零位不在区间中点 |
| `joint5` | revolute | -2.8973 | 2.8973 | 2.61 | 12 | |
| `joint6` | revolute | -0.0175 | 3.7525 | 2.61 | 12 | 几乎全正区间 |
| `joint7` | revolute | -2.8973 | 2.8973 | 2.61 | 12 | |
| `finger_joint1` | prismatic (slide) | 0 | 0.04 | - | - | 夹爪左指，MuJoCo 用 `class="finger"`，轴 `0 1 0` |
| `finger_joint2` | prismatic (slide) | 0 | 0.04 | - | - | 夹爪右指，与 finger_joint1 通常做镜像/等值驱动 |

- **关节顺序**：URDF `JointState.name` 顺序与 MuJoCo `qpos`/`ctrl` 顺序均为 `joint1, joint2, ..., joint7`（再加 `finger_joint1, finger_joint2`），两边一致，不需要额外映射表。若后续 `arm_prefix` 参数被设为非空（多臂场景），需要重新核对。
- **零位 / home position**：MJCF `keyframe` 中定义为 `qpos = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]`（对应 `joint1..joint7`），即 Franka 官方标准 "home" 姿态。`mujoco_bridge` 复位服务应复位到这组值。
- **轴方向**：所有主关节在 MuJoCo 里默认 `axis="0 0 1"`（各 link 自身局部 Z 轴），与 URDF xacro 里的 joint `axis` 定义在同一约定下应一致；后续第3周做 FK 对照测试时需要用有限差分/解析 Jacobian 交叉验证，不能只凭文档假设。

## 3. MuJoCo 与 MoveIt 模型一致性

对应第3.2节要求：对至少5组固定关节构型比较 MuJoCo 与 MoveIt 2 的末端 FK，位置与姿态误差超过阈值即禁止继续集成。

- 阈值：TBD（建议初始位置误差 < 5mm，姿态误差 < 1°，后续按实测调整）
- 测试位置：`test/`（第一个自动测试，对应第15节任务5）
- 碰撞几何简化记录：TBD——`franka_description` 的 collision geometry 与 MJCF 的 `*_c` collision mesh 都是简化过的凸包/近似几何，两者是否用同一份简化尚待确认，第3周对照测试时一并核对。

## 4. 待办

- [x] 选定 Panda URDF 来源：`franka_description`（apt，`fer` 型号）
- [x] 选定 MJCF 来源：MuJoCo Menagerie `franka_emika_panda/`（vendor 进 `robot_description/mujoco/`）
- [x] 填写关节命名表、零位、限位（见第2节）
- [ ] 记录相机外参数值来源与标定方式（相机型号/安装位置尚未选定）
- [ ] 确认 URDF 与 MJCF 碰撞几何是否为同一份简化（第3节）
- [ ] 编写第一个自动测试：5组固定 `q`，对比 MuJoCo 与 MoveIt FK（对应第15节任务5，需等 `motion_planner`/MoveIt 配置接入后才能跑）
