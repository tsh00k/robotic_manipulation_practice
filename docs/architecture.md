# Architecture

给开发者的当前事实：模块边界、接口契约、关键默认值和失败行为。这里只写**现在是什么**；为什么这样定见 [ADR](adr/)，实现过程、测量数据和排查见 [周记](../Job_guides/my_study/)，开发环境见 [CLAUDE.md](../CLAUDE.md)。

## 目录

1. [系统概览](#1-系统概览)
2. [模型与场景](#2-模型与场景)
3. [坐标与关节](#3-坐标与关节)
4. [mujoco_bridge：仿真与机器人接口](#4-mujoco_bridge仿真与机器人接口)
5. [mujoco_perception：初始位姿检测](#5-mujoco_perception初始位姿检测)
6. [task_executor：任务与运动](#6-task_executor任务与运动)
7. [原始记录](#7-原始记录)
8. [配置与运行](#8-配置与运行)
9. [验证入口](#9-验证入口)
10. [已知局限](#10-已知局限)

## 1. 系统概览

任务：Franka Panda 在 MuJoCo 里把桌上的一个 4 cm 方盒放进一个 bin。相机看初始场景，测出盒子和 bin 的位姿；执行器规划带时间的关节轨迹去抓、搬、放，再用相机确认盒子在 bin 里。

| 包 | 职责 | 边界 |
| --- | --- | --- |
| `robot_description` | vendor MJCF 与网格、项目自有场景 | 官方 URDF/SRDF 从系统包读 |
| `manipulation_interfaces` | 节点间的消息与服务 | 只有类型定义 |
| `mujoco_bridge` | 物理步进；执行关节轨迹与夹爪命令；reset 与生命周期；`/clock`、TF、同一步观测；RGB-D 相机；附着判定 | 仿真真值的唯一发布者；不做视觉 |
| `mujoco_perception` | 机器人遮罩；从深度测盒子与 bin 的初始位姿 | 不读真值、不读接触 |
| `arm_kinematics` | FK、雅可比、加权阻尼最小二乘 IK | 纯 C++/Eigen/yaml-cpp，不依赖 ROS、MoveIt、MuJoCo |
| `task_executor` | episode 生命周期、阶段状态机、目标与轨迹规划、结果发布 | ROS 节点包着纯 C++ 控制器 |

`src/` 下 `arm_controller`、`experiment_runner`、`grasp_planner`、`motion_planner`、`scene_perception` 是空目录（早期占位），不参与构建。

数据流：

```
MuJoCo ─▶ mujoco_bridge ─┬─ /mujoco_bridge/episode_observation (BridgeObservation, 100 Hz) ──────────────▶ task_executor
                         ├─ camera/{color,depth}/{image_raw,camera_info} (10 Hz) ─▶ object_pose_estimator ─┐
                         └─ /tf, /tf_static, /clock, /joint_states                                          │
task_executor ◀── initial_box_pose, initial_bin_pose (InitialBoxPose / InitialBinPose) ───────────────────────┘
task_executor ── joint_command (带时间的 JointTrajectory), gripper_command, reset_with_generation ──▶ mujoco_bridge
```

## 2. 模型与场景

### 2.1 模型来源与组合

| 用途 | 来源 | 路径 |
| --- | --- | --- |
| 官方 URDF/SRDF/网格、关节限值 | apt `franka_description` 1.0.1（型号名 `fer`） | `share/franka_description/robots/fer/` |
| MoveIt 参考配置 | apt `moveit_resources_panda_moveit_config` 2.0.7 | `share/moveit_resources_panda_moveit_config/config/` |
| MuJoCo 模型 | vendor [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | `robot_description/mujoco/franka_emika_panda/panda.xml` |
| 场景 | 项目自有 | [pick_place_scene.xml](../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml)（桌、盒、相机、预设状态）；[pick_place_bin_scene.xml](../robot_description/mujoco/franka_emika_panda/pick_place_bin_scene.xml)（前者加一个静态 bin） |

场景 `include` vendor 文件，不改 vendor。注意：

- include 与 meshdir 按顶层文件所在目录解析，场景文件与 `panda.xml` 同目录。
- 同名 keyframe 编译报错；而 qpos 短于模型 nq 时 MuJoCo **静默补零**，编译和 reset 成功都不能证明长度对。
- qpos 地址用 `jnt_qposadr`、qvel 用 `jnt_dofadr`，二者不可混用；`ctrl` 按执行器 id 索引。

### 2.2 场景

| 项 | 值 |
| --- | --- |
| 桌面 | 顶面 z = 0.22 m，x ∈ [0.2, 0.8]、y ∈ [−0.4, 0.4] |
| 盒子 | body `box`，自由关节；边长 40 mm、50 g；`condim=3`、`friction="1 0.03 0.003"`；默认中心 (0.5, 0, 0.241) |
| bin | body `bin`，静态；内口 140 × 130 mm，壁高 12 mm、厚 6 mm，内底面高于桌面 7 mm；默认 (0.5, 0.3)；原点为内底面中心 |
| 外观 | 桌、盒、bin 用 MuJoCo 内置程序纹理，只影响 RGB |
| 相机 | `workcell_rgbd`，位于 (0.5, −0.45, 1.0)，见 4.5 |

`scene.enabled:=true` 时 bridge 加载带 bin 的场景，并按参数 `scene.box.{x,y,z,roll,pitch,yaw}`、`scene.bin.*` 摆放（米、弧度，R = Rz(yaw)·Ry(pitch)·Rx(roll)；省略 z 时取最低角点离桌 1 mm）。位姿写入 `qpos0`、所有预设状态的 qpos 和 bin body，所以 reset 恢复同一布局。出桌、穿桌、非有限、bin 倾斜超约 20°、盒与 bin 包围盒间距 < 20 mm，都让 bridge 以状态 1 退出并说明原因。这些位姿是仿真真值，只有 bridge 声明这些参数。关闭时加载不带 bin 的场景（旧场景，固定放置点 (0.5, 0.3)）。

### 2.3 预设状态与 HOME

MJCF 的 `<keyframe>` 在本文叫**预设状态**：一份带名字的完整状态快照（qpos、qvel、act、ctrl、mocap），与动画、视频的“关键帧”无关。

| 量 | 值 |
| --- | --- |
| reset 用的预设状态 | `pick_place_home`（参数 `reset_keyframe_name`）；16 维 qpos = 7 臂 + 2 指 + 盒子 7 |
| 手臂构型 | Franka ready：`[0, −π/4, 0, −3π/4, 0, π/2, π/4]`，TCP ≈ (0.307, 0, 0.487)，工具朝下，手指张开方向沿 world y |
| 夹爪 | 每指 0.04 m（全开）；夹爪执行器的 ctrl 为 255（范围 0..255，不是米） |

bridge **启动时**就把仿真置为这个预设状态（generation 仍为 0），之后每次 reset 也是它；executor 的 HOME 是同一组关节角（`home.joint_positions`，默认 `kFrankaReadyPose`，单测逐项比对 MJCF）。选 ready 是因为这时手臂不挡相机看工作区（机器人遮罩投到桌面后离 x ∈ [0.30, 0.70]、y ∈ [−0.30, 0.40] 最近 80 mm）。

MJCF 没有重力补偿：静止时手臂在重力下比伺服目标低几毫弧度（ready 姿态下 joint4 约 6.3 mrad，TCP 约低 3.5 mm）。真机控制器自带重力补偿。

## 3. 坐标与关节

### 3.1 TF 与 TCP

frame 名用 MJCF 的 body 名，例外是合成的 `hand_tcp` 与 `camera_optical_frame`。bridge 是所有 TF 的唯一发布者。

| frame | 父 | 类型 |
| --- | --- | --- |
| `world` | — | 根 |
| `link0` | world | static，单位变换 |
| `link1`..`link7` | 上一节 | dynamic |
| `hand` | link7 | static（MJCF 合并了 URDF 的 link8；TF 里没有 link8） |
| `hand_tcp` | hand | static：平移 (0, 0, 0.1034) m，单位旋转（取自官方 `franka_hand.xacro`；代码常量 `kHandToTcpZ`，有一致性测试） |
| `left_finger`、`right_finger` | hand | dynamic |
| `camera_link` → `camera_optical_frame` | world | static：安装位姿；光学系 +X 右、+Y 下、+Z 前 |
| `box` | world | dynamic，盒子真值 |

抓取与运动学统一用 `hand_tcp`。TCP 不含手的 −45° 转角（它在 `hand` 的变换里）。

### 3.2 关节与限值

| 关节 | 范围（rad） | FCI 速度（rad/s） | FCI 加速度（rad/s²） | 力矩（N·m） |
| --- | --- | ---: | ---: | ---: |
| joint1 | ±2.8973 | 2.175 | 15 | 87 |
| joint2 | ±1.7628 | 2.175 | 7.5 | 87 |
| joint3 | ±2.8973 | 2.175 | 10 | 87 |
| joint4 | −3.0718..−0.0698 | 2.175 | 12.5 | 87 |
| joint5 | ±2.8973 | 2.61 | 15 | 12 |
| joint6 | −0.0175..3.7525 | 2.61 | 20 | 12 |
| joint7 | ±2.8973 | 2.61 | 20 | 12 |
| finger_joint1/2 | 0..0.04 m | — | — | — |

速度、力矩、范围与 `franka_description/robots/fer/joint_limits.yaml` 一致；加速度来自 FCI 文档。官方 URDF 的名字带 `fer_` 前缀（`fer_link0`、`fer_hand_tcp`…），跨库时要显式映射。bridge 的关节顺序是 joint1..7、finger_joint1、finger_joint2，消费者仍应按名字对齐。

## 4. mujoco_bridge：仿真与机器人接口

### 4.1 话题与服务

| 名字 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/clock` | Clock | 发布 | 每物理步；系统里唯一的时钟源 |
| `/joint_states` | JointState | 发布 | 9 个关节，默认 100 Hz（可视化用） |
| `/tf`、`/tf_static` | TFMessage | 发布 | 见 3.1 |
| `~/episode_observation` | BridgeObservation | 发布 | 100 Hz，见 4.3；executor 与估计器的输入 |
| `~/camera/color/image_raw`、`~/camera/depth/image_raw`、两个 `camera_info` | Image、CameraInfo | 发布 | 10 Hz，best effort，见 4.5 |
| `~/ground_truth/object_pose` | PoseStamped | 发布 | 盒子真值，评测与 oracle 用 |
| `~/ground_truth/bin_pose` | PoseStamped | 发布 | bin 真值，transient local，只在 `scene.enabled` 时启动发布一次 |
| `~/ground_truth/{left,right}_finger_contact` | Bool | 发布 | 手指是否碰到盒子（按身份），仅评测 |
| `~/joint_command` | JointTrajectory | 订阅 | 见 4.4 |
| `~/gripper_command` | GripperCommand | 订阅 | `position` = 两指总开口（m）；`max_effort` 忽略 |
| `~/reset_with_generation` | ResetScene | 服务 | 回预设状态，返回 session 与 generation |
| `~/reset` | Trigger | 服务 | 同上的手动入口 |

### 4.2 Reset 与生命周期

reset：`mj_resetDataKeyframe` 恢复预设状态（含 ctrl），`mjData::time` 保持单调，再 `mj_forward`；清除附着锁存、附着计时和正在执行的轨迹。每次 bridge 启动有新的 `bridge_session`，每次成功 reset `generation` 加一（启动时的那次不加）。消费者只接受当前 session、generation、递增 `sample_sequence` 的样本（[ADR 003](adr/003-reset-generation-observation.md)）。reset 是仿真专有接口；真机回初始位姿需要一条可取消、有反馈的轨迹。

### 4.3 同一步观测 `BridgeObservation`

同一物理步之后从同一份 `mjData` 打包，所有字段时间一致：

| 字段 | 内容 |
| --- | --- |
| `bridge_session`、`generation`、`sample_sequence` | 生命周期键；`sample_sequence` 是物理步号 |
| `joint_state` | 9 关节位置、速度、力矩；stamp 为仿真时间 |
| `object_pose` | 盒子真值（world）。vision 来源的 executor 不用它 |
| `world_to_hand_tcp` | TCP 位姿（world） |
| `attachment_state` | NOT_ATTACHED=0、ATTACHED=1、RELEASED=2，见 4.6 |
| `left_finger_contact`、`right_finger_contact` | 手指是否碰到机器人以外的东西（不带身份）。仿真专有，真实 Franka Hand 没有指尖传感器；只用于给 CLOSE 超时贴诊断标签 |
| `arm_command`、`gripper_command_width_m` | 这一步 `ctrl` 里的 7 个手臂伺服目标（rad）与夹爪目标开口（m）：控制器**实际采用**的指令，数据集的 action（[ADR 021](adr/021-applied-command-in-bridge-observation.md)） |

### 4.4 指令执行

**手臂：** `~/joint_command` 有两种形态。

- **单点**：立即把 `points[0].positions` 写进 `ctrl`（固定关节表模式、调试脚本用）。
- **多点**（`time_from_start` 递增、从 0 开始）：带时间的轨迹（[ADR 020](adr/020-path-then-time-parameterized-joint-trajectories.md)）。bridge 记下到达时的仿真时间，此后每个物理步按经过的时间在相邻两点之间**线性插值**写进 `ctrl`，走完后保持终点。新消息替换正在执行的轨迹，reset 清除它。`velocities` 不使用。executor 每 1 ms 一个点，物理步 2 ms 读参考时恰好落在采样点上，所以伺服拿到的是精确的规划位置（采样更稀时，点间插值会让加速度超限，见 Week 5 Stage 3 的 3.5）。

夹爪关节名出现在 `~/joint_command` 里会被忽略并警告一次（夹爪走 `~/gripper_command`）。

**伺服**：每个关节一个独立的位置 PD（MuJoCo `general` 执行器，`biastype="affine"`）：τ = kp·(u − q) − kd·q̇，截到力矩上限。kp/kd：joint1–2 4500/450，joint3–4 3500/350，joint5–7 2000/200。唯一输入是目标角度 u；没有速度前馈、没有重力补偿。对光滑参考，它近似为延迟 Ts = (kd + 1)/kp ≈ 0.1 s 的跟踪（各关节相同）：实际运动比参考晚约 100 ms，速度、加速度峰值不超过参考（实测 ≤ 参考上限的 1.008 倍）。

### 4.5 RGB-D 相机

| 项 | 值 |
| --- | --- |
| 启用 | bridge 参数 `enable_rgbd_camera`（默认 false）；demo launch 默认 `auto`，随 `observation_source`：vision 开、oracle 关 |
| 安装 | `camera_link` 在 (0.5, −0.45, 1.0)，`xyaxes="1 0 0 0 0.857 0.514"`，fovy 50° |
| 图像 | 320×240；RGB `rgb8`；深度 `32FC1`，米制光轴 z，无效为 NaN |
| 内参 | 两份 CameraInfo 相同，零畸变，fx = fy = 257.34 px，cx = 159.5，cy = 119.5 |
| 时间 | RGB、深度、两份 CameraInfo 与该物理步的观测同一 stamp；相机周期必须是观测周期的整数倍 |
| QoS | best effort（SensorDataQoS） |

**中间件**：RGB（230 KB）与深度（307 KB）每帧连着发，合计超过 Fast DDS 默认 512 KB 的共享内存段，best effort 的 RGB 会被深度挤掉（35%~80% 丢失）。[config/fastdds_shm.xml](../src/mujoco_bridge/config/fastdds_shm.xml) 把段加到 4 MB；demo launch 用 `FASTRTPS_DEFAULT_PROFILES_FILE` 给所有节点设置，探针也设置。不经 launch 起的节点没有这个配置，RGB 会静默丢帧。

### 4.6 附着判定

`attachment_state` 是附着的唯一权威（[ADR 019](adr/019-attachment-from-robot-side-signals.md)）。只用真实 Franka Hand 也报告的量（对应 libfranka 的 `is_grasped`）：夹爪被命令合拢（最近一条命令 < 盒宽 − 10 mm）、两指总开口与盒宽相差 < `grasp.width_epsilon_m`（10 mm）、两指已停下（速度之和 < `grasp.attach_max_finger_speed_m_s`，20 mm/s），连续 `grasp.attach_hold_s`（0.1 s 仿真时间）→ ATTACHED 并锁存；张开超过盒宽 + 15 mm（55 mm）→ RELEASED。reset 清锁存。闭爪后盒子滑出时仍为 ATTACHED（由 executor 的开度窗口发现，见 6.7）。

### 4.7 故障注入（默认关闭，launch 不暴露）

| 参数 | 作用 |
| --- | --- |
| `fault.truth_offset_x_m` | 只把**发布**的盒子真值沿 world x 平移，物理不变；用来证明 vision 来源不读真值 |
| `fault.drop_box_after_attach_s` | 附着持续这么久后把盒子移回 reset 位置（每次 bridge 运行一次）；复现“盒子滑出、附着仍锁存” |

## 5. mujoco_perception：初始位姿检测

节点 `object_pose_estimator`。只处理深度：每帧把深度图、深度 CameraInfo、`BridgeObservation` 和机器人 TF **按完全相同的时间戳**配对（不取各话题最新值拼接）；机器人 TF 等 0.5 s 还不到就报 `MISSING_ROBOT_TRANSFORM`。ATTACHED 期间不处理。

### 5.1 机器人遮罩

用 vendor 的 58 份可见网格（link0..7、hand、两指），MoveIt `moveit_mesh_filter` 按当帧 TF 渲染预测深度；观测深度与预测相差 ≤ `robot_mask.depth_tolerance_m`（12 mm）的像素视为机器人、置为无效。需要 X11/OpenGL，launch 为它设 `LIBGL_ALWAYS_SOFTWARE=1`。调试话题 `~/debug/robot_predicted_depth`、`~/debug/robot_mask`（mono8，255 = 被滤）、`~/debug/filtered_depth`、`~/debug/robot_mask_diagnostics`（像素计数与耗时）。

### 5.2 盒子与 bin 的检测

1. **深度窗口**（`DepthWindow`）：最近 `initial_box.frames`（10）帧掩膜后深度的逐像素均值；某像素有效帧不足一半、或有效帧最大最小相差超过 20 mm（窗口里见过两个表面），就无效。窗口在新的 session/generation、以及 ATTACHED → 非 ATTACHED 时清空。
2. **盒子**（`detectInitialBox`）：均值深度反投影到 world；取高度在 [顶面 − 15 mm, 顶面 + 40 mm]（[0.245, 0.30] m）、在桌面范围内的像素；8 邻域连通块（≥ 20 像素）；每块对 x–y 求最小面积外接矩形；两边都在 40 ± 5 mm 的块才是盒子，必须恰好一个。x、y 取矩形中心，z 取顶面像素中位数减半高，yaw 取边方向折到 [−45°, 45°)。
3. **bin**（`detectInitialBin`）：同样步骤，高度带 [0.2235, 0.243] m，矩形两边在 152 ± 8 mm 与 142 ± 8 mm，恰好一块；yaw 为长边方向折到 [−90°, 90°)；z 取矩形中央区域的中位数（内底面高度），中央没有像素就拒绝。

不用颜色、不读真值、不用历史。只支持平放的单个盒子与空的单个 bin，且都要在视野里。`InitialPoseEstimator` 把窗口和两个检测器合起来（同一窗口，各检测一次）；这些类不创建节点，各有单测。

### 5.3 输出

`~/initial_box_pose`（InitialBoxPose）与 `~/initial_bin_pose`（InitialBinPose），每个可处理的深度帧各发一条：

| 字段 | 含义 |
| --- | --- |
| `bridge_session`、`generation`、`sample_sequence`、`header.stamp` | 与该帧的观测相同 |
| `state` | WARMING_UP=0（窗口未满，`reason`=`WINDOW_FILLING`）、NOT_MEASURED=1、MEASURED=2 |
| `reason` | MEASURED 时为空，否则是拒绝名：`NO_VALID_DEPTH`、`NO_BOX_BAND_PIXELS`、`NO_RECTANGLE_MATCHES_BOX`、`SEVERAL_BOX_CANDIDATES`（bin：`NO_BIN_BAND_PIXELS`、`NO_RECTANGLE_MATCHES_BIN`、`SEVERAL_BIN_CANDIDATES`、`NO_BIN_FLOOR_PIXELS`），或不可用帧的 `INVALID_INPUT`、`MISSING_ROBOT_TRANSFORM`（这种帧不进窗口） |
| `position`、`yaw_rad` | world；盒子为中心，bin 为内底面中心；只在 MEASURED 时有值，否则 NaN。roll、pitch 不测 |
| `frames_averaged`、`frames_required` | 窗口进度 |
| `candidates[]` | 高度带里每一块的像素数、矩形两边长、中心、是否匹配；用来说明漏检原因 |

名字里的 “initial” 是历史：executor 在 VERIFY 时也用 `initial_box_pose` 重新测盒子。

## 6. task_executor：任务与运动

节点 `task_executor`：ROS 适配层负责话题、服务、计时器、日志；`EpisodeController`（纯 C++）负责生命周期、阶段、重试、轨迹规划与遥测。单线程执行器，20 Hz 计时器。

### 6.1 订阅与发布

| 名字 | 方向 | 用途 |
| --- | --- | --- |
| `/mujoco_bridge/episode_observation` | 订阅 | 关节、TCP、附着；oracle 来源时的盒子位姿 |
| `/object_pose_estimator/initial_box_pose`、`initial_bin_pose` | 订阅 | vision 来源的盒子与 bin |
| `/mujoco_bridge/ground_truth/bin_pose` | 订阅 | 仅 oracle 来源且有 bin 时 |
| `~/start_episode`（Empty） | 订阅 | 开始一个 episode（任何状态都可以重新开始） |
| `/mujoco_bridge/joint_command`、`gripper_command` | 发布 | 每阶段一条手臂轨迹；夹爪命令每 tick |
| `/mujoco_bridge/reset_with_generation` | 客户端 | 每个 episode 与每次重试先 reset |
| `~/initial_pose_latch`（InitialPoseLatch） | 发布 | 锁存状态：WAITING / LATCHED / FAILED，各对象的锁存值、极差、原因 |
| `~/episode_outcome`（EpisodeOutcome） | 发布 | 每个 episode 一条，见 6.8 |

### 6.2 生命周期

`Idle → ResetPending → AwaitingResetResponse → AwaitingObservation → Ready → Finished / Failed`，与阶段分开；状态机只在 Ready 推进。只接受当前 session/generation 的递增样本。看门狗（steady clock）：reset 服务与数据流断流 5 s；vision 来源等锁存 `latch.timeout_s`（10 s，墙钟）。失败码：`RESET_UNAVAILABLE`、`RESET_FAILED`、`RESET_SUPERSEDED`、`OBSERVATION_STALE`、`VISION_LATCH_TIMEOUT`、`IK_FAILED`、`TRAJECTORY_FAILED`，以及阶段失败（见下）。阶段计时用仿真时间。（[ADR 004](adr/004-episode-controller-orchestration.md)）

### 6.3 观测来源与锁存

`observation_source`：节点默认 `oracle`，demo launch 默认 `vision`。

- **oracle**：盒子用 bridge 真值，bin 用 `~/ground_truth/bin_pose`。
- **vision**：每次 reset 后，`PoseLatch` 等盒子与 bin 各自连续 `latch.frames`（5）条 MEASURED，x、y 极差 ≤ 3 mm、yaw 圆周极差 ≤ 3°（盒子按 90° 周期，bin 按 180°），锁存最新一条，直到下次 reset 不变；非 MEASURED 使累积清零。**锁存之前不发任何关节命令。** VERIFY 之前所有阶段用锁存值；VERIFY 只接受释放之后的新测量（估计器在释放时清空窗口，约 1 s 后才有第一条）。

`place.into_bin`（demo launch 随 `scene_enabled` 设置）：为 true 时放置目标是 bin（vision：锁存的 bin；oracle：真值），拿到之前不开始；为 false 时是固定点 (`target.place_x_m`, `target.place_y_m`) 与桌面。

### 6.4 阶段与通过条件

顺序 `HOME → PREGRASP → GRASP → CLOSE → LIFT → PREPLACE → PLACE → OPEN → RETRACT → VERIFY → DONE`。运动阶段的“到位”= 本段轨迹已走完 + 各关节离目标 < `fsm.position_epsilon_rad`（0.05；GRASP、PREPLACE、PLACE 用 0.3）且速度 < 0.05 rad/s + 阶段已过 `fsm.min_settle_s`（0.5 s）。

| 阶段 | 通过条件 | 失败 → RECOVER |
| --- | --- | --- |
| HOME、PREGRASP、GRASP、RETRACT | 到位 | 超时 |
| CLOSE | ATTACHED 且已持续 `fsm.close_after_attach_s`（0.2 s） | 超时：`UNEXPECTED_CONTACT` 或 `GRASP_EMPTY`（按手指接触分类） |
| LIFT | ATTACHED 且到位 | 到位但未附着且超过 `fsm.lift_settle_grace_s`（2 s）：`SLIPPED`；超时 |
| PREPLACE、PLACE | ATTACHED 且到位 | 失去附着：`SLIPPED`；超时 |
| OPEN | 两指总开口 > 0.06 m（不等 min_settle） | 超时 |
| VERIFY | RELEASED、盒子在 bin 内（6.7）、手臂回到 HOME 到位 | 超时：`PLACE_MISSED` |

超时为 `fsm.phase_timeout_s`（6 s）。RECOVER 重新 reset 并从 HOME 开始，最多 `fsm.max_retries`（3）次，之后 FAILED。

### 6.5 目标

`DiffIkWaypointSource`：每个阶段一次 IK（`arm_kinematics::solveIk`，初值为实测关节），阶段内缓存；不收敛报 `IK_FAILED`。HOME 与 VERIFY 不求 IK，直接用 `home.joint_positions`。

<a id="62-cartesian-任务与-waypoint"></a>

| 阶段 | TCP 目标（world） |
| --- | --- |
| PREGRASP、LIFT | PREGRASP 时盒子位姿上方 `target.hover_height_m`（0.15 m） |
| GRASP、CLOSE | 盒子中心 |
| PREPLACE、RETRACT | 放置点上方：支撑面 + 盒半高 + 0.05 + 0.15 m |
| PLACE、OPEN | 支撑面 + 盒半高 + `target.place_tcp_above_box_center_m`（0.05 m） |

姿态：工具朝下，绕 world z 转 `target.tool_yaw_rad` + 盒子 yaw 折到 [−45°, 45°)（`target.align_tool_to_box_yaw`，默认 true；HOME 除外）。盒子 yaw 取 PREGRASP 时的值，之后各阶段都用它，所以夹住之后不再转腕。CLOSE、LIFT、PREPLACE、PLACE 夹爪命令 0（全合），其余 0.08 m（全开）。

固定关节表模式（`waypoint_source:=keyframe`）是 Week 2 的遗留：每个阶段查一张写死的关节角表，不读物体位姿，HOME 仍是 vendor home；只作对照，不再维护。

IK（`arm_kinematics`）：任务空间为 `hand_tcp` 的 [x, y, z, rx, ry, rz]，平移权重 1.0、旋转 0.2；阻尼最小二乘（阈值 0.08、最大阻尼 0.05），零空间往关节中位拉（增益 0.02）；每步误差截到 0.05 m / 0.2 rad、每关节步长截到 0.12 rad；收敛条件 0.1 mm / 0.001 rad。只求终点，不管时间（时间由 6.6 处理）。

### 6.6 轨迹（[ADR 020](adr/020-path-then-time-parameterized-joint-trajectories.md)）

进入一个运动阶段时规划一次，发一条 `JointTrajectory`；阶段在轨迹走完之前不结束。

- **起点**：上一段轨迹的终点；reset 后的第一段从 `home.joint_positions` 出发。都是**指令**，不是实测关节（实测在重力下比伺服目标低几毫弧度，用它当起点等于一个小阶跃）。
- **路径**：GRASP、LIFT、PLACE、RETRACT 走 **TCP 直线**——两端构型的 TCP 之间每 5 mm 一个位姿（转角用四元数球面插值），逐点 IK、以上一点为初值，相邻点任一关节差 > 0.05 rad 或不收敛则 `TRAJECTORY_FAILED`；阶段目标改为这条线最后一点的解。其余阶段走**关节空间直线** q(s) = q_a + s·(q_b − q_a)。CLOSE、OPEN 手臂不动，保持上一段终点。
- **时间**：moveit_core 的 TOTG（`trajectory_processing::Path`、`Trajectory`，直接用关节向量，不需要 MoveIt 机器人模型），起止静止；TCP 直线的路径点之间允许 1 mrad 的拐角圆化。限值 `trajectory.max_velocity`（默认 FCI 的 95%：2.066 / 2.480 rad/s，给数据契约留余量）与 `trajectory.max_acceleration`（默认 FCI 的 1/4：3.75、1.875、2.5、3.125、3.75、5、5 rad/s²，与 MoveIt 官方 Panda 配置相同；这样 TOTG 的加速度突变在 1 kHz 下也满足 FCI 的加加速度上限）。
- **采样**：每 1 ms 一个点（位置、速度、`time_from_start`）。只用 TOTG 的 `getPosition`/`getVelocity`：它的 `getAcceleration` 在部分时刻报出超过上限的值，而位置本身并不超限。

### 6.7 VERIFY 与搬运期检查

**入 bin 判据**（`boxInBin`）：盒子四角（按其 yaw）在 bin 坐标系里离内口（半宽 70 × 65 mm）至少 5 mm，且盒心高度在内底面 + 20 mm ± 5 mm 内。5 mm = 盒子检测误差限 3 mm + bin 在线偏差 2 mm。VERIFY 期间日志每 0.5 s 打一行 `verify:`（位置、是否在内、最小角余量、高度差）。没有 bin 时是以放置点为圆心、`verify.place_region_radius_m`（0.08 m）为半径。

**开度窗口**（`CarryWidthMonitor`）：ATTACHED 且在 LIFT、PREPLACE、PLACE 时，两指总开口 < 34 mm 持续 0.2 s 告警一次（`CARRY_WIDTH_LOW`），只告警、不改变 episode 的走向。

### 6.8 结果 `EpisodeOutcome`

`success`、`failure_code`、`retries`；`observation_source`、`observation_failure_layer`（`perception` 或 `execution`）、`observation_failure_reason`（成功时为空）；各阶段名、时长、TCP 目标、IK 残差、跟踪误差；搬运期开度告警与最小、最大开口。`observation_confidence`、`observation_residual_m` 在 vision 来源时为 NaN（旧检测路径删除后保留的字段）。

## 7. 原始记录

[scripts/record_episode.py](../scripts/record_episode.py)：在已运行的 demo 上录一个 episode。

- 先确认只有一套 bridge 和 executor 在跑；
- `ros2 bag record --use-sim-time`，显式列出话题：相机四个（best effort 的 QoS 覆盖）、`~/episode_observation`、手臂与夹爪命令、锁存状态、outcome、`/tf_static`；
- 发 `~/start_episode`，等 outcome 后再录 1 s 停止；
- 写 `metadata.json`：数据契约规定的 `task`、`objects`、`targets`（契约的 schema 不允许其他字段）；
- 写 `sidecar.json`：布局、outcome、锁存的盒子 yaw、抓取转角及其离 ±45° 切换点的距离（同一位姿附近会出现差 90° 的两种抓法）、git 提交与是否有未提交改动。

一帧（10 Hz）= 同一时间戳的 RGB、深度和 `BridgeObservation`：state = `joint_state` 的 7 臂 + 两指之和，action = `arm_command` + `gripper_command_width_m`。一个 episode 的帧从 reset（本 generation 的第一个观测）到 outcome。深度里的 NaN 保留，由导出器按契约转成 0。导出程序（LeRobot 格式）不在本仓库。输出在 `results/episodes/`（不提交）。

## 8. 配置与运行

启动：`scripts/start_demo.sh`（构建后 `ros2 launch mujoco_bridge demo.launch.py`），或直接 `ros2 launch mujoco_bridge demo.launch.py scene_enabled:=true`。launch 起 bridge、估计器（相机开时）、executor、RViz；为所有节点设 Fast DDS 配置，为估计器和 RViz 设软件渲染。

| launch 参数 | 默认 | 作用 |
| --- | --- | --- |
| `scene_enabled` | false | 带 bin 的场景；同时设 executor 的 `place.into_bin` |
| `box_*`、`bin_*`（x, y, z, roll, pitch, yaw） | auto | 只传用户给出的值给 bridge |
| `observation_source` | vision | oracle / vision |
| `enable_rgbd_camera` | auto | 随来源；true / false 照办 |
| `camera_rate_hz`、`joint_state_rate_hz`、`tf_rate_hz` | 10、100、100 | |
| `enable_debug_viewer`、`debug_viewer_rate_hz` | false、30 | MuJoCo GLFW 窗口 |

常用节点参数（都有默认值）：bridge 的 `scene.*`、`grasp.*`、`fault.*`；估计器的 `plane_z_m`、`box_size_*`、`depth_{min,max}_m`、`initial_box.frames`、`robot_mask.depth_tolerance_m`；executor 的 `home.joint_positions`、`trajectory.max_{velocity,acceleration}`、`fsm.*`、`latch.*`、`target.*`、`verify.*`、`place.into_bin`、`waypoint_source`。

## 9. 验证入口

| 入口 | 覆盖 |
| --- | --- |
| `colcon test`（各包 gtest + lint） | 场景配置与摆放、附着判定、轨迹插值、TOTG 规划（时长、限值、1 ms 差分下的加加速度）、IK 与 TCP 直线、阶段状态机、episode 控制器、锁存、入 bin 判据、开度窗口、深度窗口与两个检测器（合成场景，答案由构造给出）、机器人遮罩、预设状态与 executor HOME 一致 |
| [test_model_consistency.cpp](../src/mujoco_bridge/test/test_model_consistency.cpp) | 7 组固定 q 下 MuJoCo、MoveIt、`arm_kinematics` 的 FK 与雅可比三方一致（位置 < 1e-6 m） |
| [initial_box_probe.py](../src/mujoco_perception/test/initial_box_probe.py) | 起真实的 bridge + 估计器（+ executor）在线核对；各命令的断言写在 docstring。常用：`held`（HELD-A 40 个布局的回归与过程断言 P1~P6）、`motion`（实际关节速度、加速度、TCP 直线度、打滑）、`record`（录制器与数据契约 D1~D5）、`timeline`（各阶段时长与等待）、`static`、`latch`、`verify` |
| [camera_probe.py](../src/mujoco_bridge/test/camera_probe.py) | 相机同步、反投影 |

以上都是仿真里的证据，不代表真机标定、真实深度噪声或真实动力学。

## 10. 已知局限

- **仿真与真机的差别**：附着由仿真 bridge 判定；手指“外部接触”是仿真专有；没有重力补偿（真机有）；伺服没有速度前馈（真机控制器是否有未查）；故障注入只在仿真里。
- **视觉**：只在开局（锁存）和 VERIFY 看盒子，搬运中没有视觉；只支持单个平放的盒子和单个空 bin、HOME 姿态、固定相机；深度几乎没有帧间噪声（多帧平均的好处在仿真里被低估）。
- **运动**：关节空间直线段 TCP 走曲线（搬运段离直线几十毫米）；轨迹起止都静止，阶段之间有停顿；加速度只用 FCI 的 1/4，比真机能做到的慢；没有碰撞规划。
- **抓取转角**：盒子 yaw 在 ±45° 附近时，几乎相同的盒子会被差 90° 的两种方式抓（方盒子的对称性，修不掉；sidecar 记录离切换点的距离）。
- **语言**：只有一种任务、一种物体，元数据里的 `task` 对模型没有区分作用。
- **中间件**：不经 launch 或探针起的节点没有 Fast DDS 配置，RGB 会静默丢帧。
- **命名与规模**：单机器人、单盒子、单 bin 的命名与假设；多物体需要重新设计目标身份与检测。

<a id="14-stage-4-实测与后续目标契约"></a>
（此锚点保留供 ADR 009 引用；机器人遮罩的历史评测见 [Week 4 Stage 4](../Job_guides/my_study/week4.md#10-stage-4同帧机器人几何掩膜)。）
