# Architecture & Conventions

本文是当前项目事实与接口约定的唯一权威来源。决策理由见 [ADR](adr/)，历史实现、实测数据与排查过程见 [周记](../Job_guides/my_study/)，环境见 [CLAUDE.md](../CLAUDE.md)。

## 目录

- [1. 系统概览与实现状态](#1-系统概览与实现状态)
- [2. 模型与场景](#2-模型与场景)
- [3. 坐标与关节约定](#3-坐标与关节约定)
- [4. 观测与 reset 契约](#4-观测与-reset-契约)
- [5. RGB-D 与视觉契约](#5-rgb-d-与视觉契约)
- [6. 任务与运动执行](#6-任务与运动执行)
- [7. 验证门禁与已知限制](#7-验证门禁与已知限制)

## 1. 系统概览与实现状态

| 模块 | 当前职责 | 边界 |
| --- | --- | --- |
| `robot_description` | vendor MJCF/mesh 与项目自有场景 | 官方 URDF 从系统包读取 |
| `mujoco_bridge` | 物理步进、命令执行、reset、clock、TF、同一步观测与 RGB-D | 仿真 ground truth 的唯一发布者，不执行视觉估计 |
| `manipulation_interfaces` | 观测、视觉、结果和 reset 消息/服务 | ROS 数据契约 |
| `mujoco_perception` | 同帧机器人掩膜、候选关联、夹持证据门与物体状态估计 | 在线不读取 oracle pose 或仿真接触位；夹持为条件性判断 |
| `arm_kinematics` | FK/Jacobian、加权 DLS 与离线 IK | 纯 C++/Eigen/yaml-cpp，不依赖 ROS、MoveIt、MuJoCo |
| `task_executor` | 观测适配、episode 编排、FSM、waypoint 与结果发布 | ROS 节点适配纯 C++ 控制器，当前单线程 executor |

主链路：`MuJoCo -> BridgeObservation (+ RGB-D/TF -> VisionObjectPose) -> task_executor -> joint/gripper command -> MuJoCo`。

成熟算法复用现有库；自写运动学作为独立学习/验证模块，项目自有代码集中在生命周期、契约适配与可测试的任务判断。选型理由进入 ADR 或周记，不在此重复平台与公开项目比较。

默认使用 `oracle` 观测与 `diff_ik` waypoint，`keyframe` 保留为对照。固定场景 oracle 抓放已回归；vision 已接质量门和机器人掩膜，但仍因局部可见目标置信度不足而停止，**尚未完成视觉抓放**。

Stage 5 已完成候选关联、独立夹持证据门、TCP 附着和四态协议的实现、构建、序列测试及运行时遮挡实验，并完成讲解、追问和正式笔记，见 [Week 4 Stage 5](../Job_guides/my_study/week4.md#11-stage-5目标关联与夹持条件下的状态估计)。oracle 驱动下曾观察到附着预测及抬升期间几何冲突后解除，仍无完整视觉成功证据。Stage 6 的任务适配与最终证据门待实现。策略依据见 [ADR 010](adr/010-grasp-conditioned-object-state.md) 和 [ADR 009](adr/009-robot-aware-stateful-vision.md)；MoveIt 规划与 learned policy 接口仍未实现。

## 2. 模型与场景

### 2.1 模型来源与组合

| 用途 | 来源 | 关键路径 |
| --- | --- | --- |
| 官方 URDF/SRDF/mesh | apt `franka_description` 1.0.1（Humble，型号名 `fer`） | `share/franka_description/robots/fer/fer.urdf.xacro`、`fer.srdf.xacro` |
| MoveIt 参考配置 | apt `moveit_resources_panda_moveit_config` 2.0.7 | `share/moveit_resources_panda_moveit_config/config/` |
| MuJoCo MJCF/mesh | vendor [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | `robot_description/mujoco/franka_emika_panda/panda.xml`、`panda_nohand.xml` |

系统模型通过 `get_package_share_directory()` 引用。[pick_place_scene.xml](../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml) include vendor `scene.xml`/`panda.xml`，不修改 vendor 文件。

- MuJoCo include 与 meshdir 路径按顶层主文件目录解析；当前场景保持与 Panda 文件同目录。此规则约束当前布局，不表示其他路径重组方案不可能。
- 同名 keyframe 导致编译错误；qpos 短于组合模型 nq 会静默补齐，编译/reset 成功不能证明长度正确。
- qpos 顺序来自合并后 body/关节的文档遍历顺序。代码按 `jnt_qposadr`/`jnt_dofadr` 查地址，二者不可混用；ctrl 按 actuator id 索引。

### 2.2 默认场景

| 项目 | 当前值或规则 |
| --- | --- |
| 桌面 | box geom：`size="0.3 0.4 0.02"`、`pos="0.5 0 0.2"`；顶面 z=0.22 m，无独立 TF |
| 物体 | body **`box`**，world 下 freejoint；4 cm 立方体、50 g，初始中心 `(0.5, 0, 0.241) m` |
| 接触 | condim=3、friction=`1 0.03 0.003`、solref=`0.01 1`；当前仅滑动摩擦生效，改 condim 需重测 |
| 放置标记 | `(0.5, 0.3, 0.221) m`，无碰撞；仅视觉参考，验收由 `verify.*` 决定 |
| `model_path` 默认 | 项目自有 `pick_place_scene.xml` |
| `reset_keyframe_name` 默认 | `pick_place_home`；16 维 qpos = 7 臂 + 2 手指 + box 3 平移/4 四元数 |

换回 panda.xml 时须同时改用 home。找不到指定 keyframe 会使 reset 显式失败；带 box 场景误用仍存在的 vendor home 则可能成功返回并把 box 置于补齐后的错误位置。

### 2.3 初始构型

| 来源 | joint1..joint7（rad） | 单指位置（m） |
| --- | --- | --- |
| vendor MJCF home | `[0, 0, 0, -1.5708, 0, 1.5708, -0.7853]` | 0.04 |
| SRDF ready / fake system initial_positions.yaml | `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]` | SRDF open 为 0.035 |

当前 reset 权威是 pick_place_home，臂构型沿用 MJCF home 并补足 box 状态。URDF 不保存初始状态，MoveIt ready 与 reset 不是同一构型。

`mj_resetDataKeyframe` 同时恢复 ctrl。vendor home 的夹爪 ctrl 为 255（actuator 范围 0..255），不是米制指位置。

## 3. 坐标与关节约定

### 3.1 TF 与 TCP

**TF frame 名采用 MJCF 原生 body 名，合成例外为 hand_tcp 和 camera_optical_frame。** bridge 从模型生成树，不做 URDF 名字翻译。

| Frame | 父 frame | 发布方式/含义 |
| --- | --- | --- |
| world | 无 | 唯一根，MuJoCo 隐式 body 0 |
| link0 | world | static，当前单位变换，不叫 base_link |
| link1..link7 | link{i-1} | dynamic，7-DoF 臂 |
| hand | link7 | static；MJCF 合并 URDF link7 -> link8 -> hand，TF 无 link8 |
| hand_tcp | hand | 合成 static：平移 `[0,0,0.1034] m`、单位旋转 |
| left_finger / right_finger | hand | dynamic，两指 body |
| camera_link / camera_optical_frame | world / camera_link | static，安装与光学轴转换 |
| box | world | dynamic，物体 ground truth；object 是语义称呼，不是当前 body 名 |

每条 TF 边仅允许一个发布者，相机外参也由 bridge 单点发布。接入 robot_state_publisher 必须隔离其 /tf、/tf_static，避免重复边；具体 MoveIt 集成方案尚未定案。视觉输出独立 pose topic，不覆盖 box 的 ground-truth TF。

抓取/运动学统一参考 hand_tcp。MJCF 无此 body/site；偏移来自官方 franka_hand.xacro 的 tcp_xyz，代码常量 kHandToTcpZ 是副本，由三方测试监控漂移。TCP 不含 -45° 手腕旋转，该旋转已在 hand 变换中。需要原生 site 消费者时使用项目 overlay，而非改 vendor。

### 3.2 URDF 映射与关节

| MJCF / TF | 官方 fer URDF |
| --- | --- |
| world | base（各自树根，非直接同名） |
| link0..link7 | fer_link0..fer_link7 |
| 无对应 TF | fer_link8 |
| hand / hand_tcp | fer_hand / fer_hand_tcp |
| left_finger / right_finger | fer_leftfinger / fer_rightfinger |

URDF hand:=true 强制加 fer_ 前缀，跨库代码须显式映射。下表采用 MJCF/bridge 无前缀名字。

| 关节 | 类型 | 下限 | 上限 | 最大速度 | 最大力矩 |
| --- | --- | ---: | ---: | ---: | ---: |
| joint1 | revolute | -2.8973 | 2.8973 | 2.175 | 87 |
| joint2 | revolute | -1.7628 | 1.7628 | 2.175 | 87 |
| joint3 | revolute | -2.8973 | 2.8973 | 2.175 | 87 |
| joint4 | revolute | -3.0718 | -0.0698 | 2.175 | 87 |
| joint5 | revolute | -2.8973 | 2.8973 | 2.61 | 12 |
| joint6 | revolute | -0.0175 | 3.7525 | 2.61 | 12 |
| joint7 | revolute | -2.8973 | 2.8973 | 2.61 | 12 |
| finger_joint1/2 | prismatic | 0 | 0.04 | 未列 | 未列 |

臂角度/速度/力矩单位为 rad、rad/s、N·m，指位置为 m。bridge 顺序为 joint1..joint7, finger_joint1, finger_joint2；消费者仍按 name 对齐。臂局部轴为 Z，两指为 Y。多臂/前缀变化需重新核对。

## 4. 观测与 reset 契约

### 4.1 Reset 与生命周期

| 服务 | 类型 | 语义 |
| --- | --- | --- |
| /mujoco_bridge/reset | std_srvs/srv/Trigger | 手动兼容入口，成功也递增 generation |
| /mujoco_bridge/reset_with_generation | manipulation_interfaces/srv/ResetScene | executor 使用，成功返回 session/generation |

两服务共用实现：恢复 keyframe 的 qpos/qvel/act/ctrl/mocap，保留 mjData::time 单调，调用 mj_forward 刷新派生量。bridge 是唯一 /clock 来源。reset 为仿真专有接口；真机回初始位姿需要可取消、有反馈的轨迹 action。

每次 bridge 启动创建新 session，每次成功 reset 严格递增 generation。controller 只接受本次 reset 的 session/generation 与递增 sample sequence；旧会话/代际/序号被拒绝，更高 generation 报 RESET_SUPERSEDED。请求 token 隔离旧回执。默认 5 s steady-clock 看门狗约束服务与观测等待/断流，失败码包括 RESET_UNAVAILABLE、RESET_FAILED、OBSERVATION_STALE。见 [ADR 003](adr/003-reset-generation-observation.md)。

### 4.2 同一步观测与 oracle

`/mujoco_bridge/episode_observation`（BridgeObservation）在同一次物理步后读取同一份 mjData，打包 session/generation/sequence、仿真 stamp、9 关节状态、box pose、world-frame TCP 与双指接触。executor 不再混用独立 joint_states 和 TF 作决策输入。

独立 joint_states、TF、ground truth 继续服务可视化与评测，但其他消费者不会自动获得 bundle 原子性。

| 独立 oracle 项目 | 契约 |
| --- | --- |
| topic/type | /mujoco_bridge/ground_truth/object_pose，geometry_msgs/msg/PoseStamped |
| frame/source | world，读取 xpos/xquat，不直接把 qpos 当 world pose |
| 频率/存在性 | 与 TF 共用 decimation，仅模型含 box body 时创建 |
| 隔离 | 视觉独立 topic，oracle 仅作仿真对照 |

## 5. RGB-D 与视觉契约

### 5.1 相机与精确同步

| 项目 | 默认值/约定 |
| --- | --- |
| 安装 | 项目 MJCF camera_link：world 平移 `(0.5,-0.45,1.0) m`，xyaxes=`1 0 0 0 0.857 0.514` |
| 光学轴 | MuJoCo +X 右/+Y 上/-Z 前；到 optical 绕 X 转 π，变成 +X 右/+Y 下/+Z 前 |
| 启用 | enable_rgbd_camera=true；bridge 默认关闭，vision 入口启用 |
| 图像 | /mujoco_bridge/camera/color/image_raw：rgb8；同前缀 depth/image_raw：32FC1 |
| 内参 | color/camera_info、depth/camera_info，相同零畸变内参；320×240、fovy=50°、fx=fy=257.34083046 px、cx=159.5/cy=119.5 |
| 深度 | 米制光轴 z-depth；OpenGL 缓冲经近远平面换算，无效/远裁剪写 NaN |
| 时间/frame | RGB、depth、两份 CameraInfo 共用物理步后仿真 stamp 与 camera_optical_frame |
| 频率 | 相机 10 Hz、TF/observation 100 Hz、物理步长 0.002 s |

反投影 `[(u-cx)z/fx,(v-cy)z/fy,z]` 后经静态 TF 转 world。四份相机消息与 BridgeObservation 按**完全相等 stamp（0 ns 容差）**配对，获取生命周期键并拒绝旧 generation，不能取各话题最新值拼接。相机 decimation 必须为 TF/observation decimation 整数倍。安装位置减轻 home 遮挡，但中心像素始终属于 box 不是不变量。

### 5.2 机器人掩膜与几何估计

<a id="14-stage-4-实测与后续目标契约"></a>

此旧锚点保留供 ADR 快照引用，历史评测见 [Week 4 Stage 4](../Job_guides/my_study/week4.md#10-stage-4同帧机器人几何掩膜)。

掩膜用 vendor 58 份可见 OBJ 网格，frame 为 link0..link7、hand、两指。geometric_shapes/Assimp 读网格，MoveIt moveit_mesh_filter 渲染预测深度，padding=0。动态机器人 TF 必须在图像同一 stamp；缺失超过 0.5 s 报 MISSING_ROBOT_TRANSFORM，不复用旧 TF。

有效观测/预测深度差绝对值 <= robot_mask.depth_tolerance_m=0.012 m 才过滤该像素，更近/更远有效深度均保留。无效输入或同过滤器生命周期中相机尺寸/内参变化报 INVALID_INPUT。当前无整帧冲突比例门，不发布 ROBOT_MODEL_MISMATCH。

管线：同帧配对/TF -> 反投影与掩膜 -> world ROI/桌面过滤 -> 水平支持平面 -> 全部候选簇 OBB -> ObjectTracker 关联/状态估计 -> VisionObjectPose。只去除配置桌面高度附近的平面，不把离桌盒体表面删作桌面；在线不再以最大簇决定目标，调试 target_cluster 是实际接受的候选。通用算法来自 image_geometry/PCL；SVD 已知对应点配准入口仍仅有单测。

先验为单个 4 cm 立方体与已知桌面。anchor_z_to_plane=true 时检查两个可见主尺寸，中心 z 为桌面加半高，yaw 为 90° 对称不确定姿态。附着/离桌时禁用支撑锚定；实际夹爪充分张开（宽度至少 55 mm）后允许重新尝试支撑测量，包括几何冲突导致附着失效后的路径，不依赖必须处于 RELEASED 状态。候选最高点超过桌面+盒体全高+support_tolerance_m 时仍禁用。张爪不直接生成测量、恢复附着或绕过关联门；尺寸不足可表示局部可见，过大、残差或内点冲突不能被预测掩盖。OBB 不提供任意形状识别或完全遮挡恢复。

PCL ICP/GICP 已在离线真值标注点云与平面 fixture 上比较，仅作为评测工具；默认保持 OBB。当前全表面模型到局部点云的 ICP 可收敛但有约 20 mm 平面偏差，GICP 的运行成本与遮挡退化尚不满足默认在线要求；此结果不否定采用可见面模型等其他配准配置。评测入口为 `registration_benchmark.py` / `compare_registration`，oracle 标签只用于离线选取评测目标像素与计算误差。

调试 topic 前缀 /object_pose_estimator/debug/：robot_predicted_depth、robot_mask、filtered_depth、foreground_points、target_cluster、robot_mask_diagnostics，共用图像 stamp。mask=mono8（255 过滤），深度=米制 32FC1，点云=world。诊断带生命周期键、投影/掩掉/比较/冲突像素数、簇点数、容差与耗时。

comparison_pixels 为有效预测/观测重合数；mismatch_pixels 仅统计观测更远且超容差的像素。零比较数时比例无定义，不能报零冲突率；计数提示投影不一致，不能直接定位原因。

moveit_mesh_filter 依赖 X11/OpenGL，demo 为感知设置 LIBGL_ALWAYS_SOFTWARE=1。无显示部署需虚拟 X 或经过验证的无头后端；当前不支持运行中改变相机标定。

### 5.3 任务观测来源与质量门

observation_source 每个 episode 选唯一来源，默认 oracle。vision 消费 /object_pose_estimator/object_pose（VisionObjectPose），关节/接触/TCP 仍来自 BridgeObservation；不回退 oracle 或沿用旧 accepted pose。demo 在 vision 模式启动 estimator，scripts/start_demo.sh --vision --viewer 同时启用相机/viewer。

vision/bridge 的 **bridge_session、generation、sample_sequence 完全相等**才配对；缓存有界，拒绝不匹配或已处理样本。拒绝样本也发布：accepted=false 终止为 VISION_REJECTED，accepted 但质量不足终止为 VISION_LOW_CONFIDENCE，均不创建新 snapshot、不发布新运动命令。

| 参数 | 通过条件 |
| --- | --- |
| vision.min_confidence=0.5 | confidence >= 阈值 |
| vision.max_residual_m=0.005 | residual_m <= 阈值 |
| vision.min_inlier_ratio=0.7 | inlier_ratio >= 阈值 |

EpisodeOutcome 记录 observation_source、observation_confidence、observation_residual_m、observation_failure_layer/reason。视觉失败层为 perception，执行失败为 execution，成功时失败层/原因为空。当前任务层仍逐帧拒绝即终止，尚未消费 Stage 5 原型的四态协议。

Stage 5 的 `VisionObjectPose` 将位姿证据与夹持状态分开：evidence_state 为 MEASURED/PREDICTED/OCCLUDED/REJECTED，grasp_state 为 UNCONFIRMED/CANDIDATE/HELD/RELEASED/INVALIDATED。pose_valid 只对可用的实测/有界预测为 true，兼容 accepted 只对 MEASURED 为 true。最近测量 stamp/sequence 仅由本帧像素支持的实测刷新；无当前测量时 residual/inlier 为 NaN、point_count=0。orientation_ambiguous 标记立方体姿态代表值的歧义，预测继承此标记，不能把 MEASURED 等同于所有姿态自由度都被唯一观测。保留预测年龄、启发式位置不确定性预算（不是统计保证）、夹持证据时间、附着建立时间/来源测量序号、依据和失效原因。

夹持门由两图像样本之间的同代际机器人反馈检查实际宽度和 TCP 稳定性；反馈按时间顺序消费至当前图像 stamp，不使用未来状态。要求先看到张开，再进入物体尺寸带，存在近期支撑目标且 TCP 靠近；通过稳定窗口后建立附着，不要求可见共同运动。初始化依据明确为近期支撑物体在闭合期间静止的先验，不能把旧 pose 伪装成确认时刻视觉测量。每根指位置允许 -10 μm 软限位下冲并归零，超限/非有限/缺指或无效 TCP 显式拒绝。

夹持后以 TCP 更新整个相对变换，合格重见测量可校正；偏差超过滑移门限解除附着。宽度增大离开尺寸带即解除，不等到完全张开，也不宣称能区分主动张爪与支撑丢失；保存解除位置供释放后关联。空抓、宽度缩小异常、状态断流、几何冲突及过期使模型失效，需要重新张开后才可再次准入。完全遮挡且输入相同的掉落/同宽错误物体不可区分，HELD 仅是条件性证据。

| Stage 5 参数（当前固定场景默认） | 值与语义 |
| --- | --- |
| grasp.width_tolerance_m / width_stability_m | 尺寸带 ±0.008 m；稳定窗口相对起点变化 ≤0.002 m |
| grasp.stable_s / measurement_max_age_s | 0.3 s；开始窗口的测量年龄 ≤1 s，确认时最多加一个稳定窗口 |
| grasp.tcp_distance_m / tcp_motion_m / tcp_angle_rad | 目标距离 ≤0.065 m；窗口内 TCP 移动 ≤0.012 m、旋转 ≤0.1 rad |
| grasp.open_width_threshold_m | 0.055 m；物体期望宽度来自 box_size_y_m=0.04 m |
| tracking.max_prediction_age_s / held_prediction_age_s / released_prediction_age_s | 支撑 0.3 s；夹持 5 s（同时约束附着总寿命，不能用重见无限续期）；释放 0.15 s 从解除时刻计 |
| tracking.initial_uncertainty_m / uncertainty_growth_m_s / held_uncertainty_growth_m_s / max_uncertainty_m | 初值 0.005 m；支撑/释放增长 0.08 m/s，夹持增长 0.003 m/s；上限 0.03 m |
| tracking.association_slack_m / max_speed_m_s / min_confidence | 0.025 m / 0.6 m/s / 0.2；非附着关联门的年龄增长在 0.3 s 截断 |
| tracking.slip_tolerance_m / max_sample_gap_s / support_tolerance_m | 可见位置偏差 0.025 m；机器人状态间隔 0.5 s；支撑高度容差 0.012 m |

这些值经固定场景回放及正反例 fixture 检查，仍不是随机场景、任意物体或真实传感器的通用标定。reset/重启清空测量、夹持和附着状态；旧序号/代际拒绝，节点不接受已退休 bridge session 回流。Stage 6 尚未修改任务消费逻辑，也尚未建立最终重新实测成功门。

## 6. 任务与运动执行

### 6.1 EpisodeController 与 FSM

EpisodeController 独占生命周期、phase、retry、失败原因与 telemetry；ROS 节点负责转换、服务/话题、timer、日志。生命周期 Idle/ResetPending/AwaitingResetResponse/AwaitingObservation/Ready/Finished/Failed 与 Phase 分开，FSM 仅在 Ready 推进。任意状态可 start 新 episode，terminal outcome 仅一次。

任务顺序 `HOME -> PREGRASP -> GRASP -> CLOSE -> LIFT -> PREPLACE -> PLACE -> OPEN -> RETRACT -> VERIFY -> DONE`，执行异常经 RECOVER 重试，耗尽后 FAILED。纯 step() 不依赖 ROS/MuJoCo，classifyGrasp() 复用 mujoco_bridge::grasp_criteria。

20 Hz tick 最多消费一份新鲜观测，设 IK seed、求当前目标、调用 FSM、记录迁移；无新观测不重发旧目标，新观测下 phase 未变则重发。动作顺序为当前 phase 目标、迁移日志、reset/outcome。阶段计时用仿真时间，看门狗用 steady clock，准入先检查超时再刷新新鲜度。见 [ADR 004](adr/004-episode-controller-orchestration.md) 与 [编排图](task_executor_episode_orchestration.html)。

### 6.2 Cartesian 任务与 waypoint

WaypointSource::jointTargetFor(Phase,ObjectPose) 返回关节目标。默认 DiffIkWaypointSource 消费独立、无 ROS 的 CartesianWaypointSource，以实测关节作 seed、阶段内缓存离线 IK 目标；HOME/retry 清抓取锁定与缓存。失败报 IK_FAILED，不发布失败解。keyframe 查表且忽略物体位姿，详表/调参见 [Week 2 Stage I](../Job_guides/my_study/week2.md#10-stage-ifsm-与-waypointsource-抽象新包-task_executor)。

| 默认几何 | world-frame hand_tcp 目标 |
| --- | --- |
| HOME | `(0.5545,0,0.5211) m` |
| GRASP/CLOSE | 首次 PREGRASP 锁定的物体中心 |
| PREGRASP/LIFT | 锁定中心上方 0.15 m |
| PLACE/OPEN | 放置 XY `(0.5,0.3) m`，桌面+半高+0.05 m，即 z=0.29 m |
| PREPLACE/RETRACT | 放置中心上方 0.15 m，即 z=0.39 m |

姿态 `R_world_tcp=Rz(tool_yaw_rad)*R_down`，R_down 绕 `(1,1,0)/sqrt(2)` 转 π；默认 yaw=0 时 TCP x/y/z 指向 world +y/+x/-z。参数不是 ZYX Euler yaw；不跟随 box yaw，也不验证最终 box 姿态。CLOSE/LIFT/PREPLACE/PLACE 全闭（总宽 0 m），其他阶段张开（0.08 m）。

target.* 表达 TCP 任务，verify.* 表达 box 验收，仅节点构造时读取，不改变 XML box/marker。diff-IK 默认两组 XY=(0.5,0.3)，不一致须 verify.allow_target_mismatch=true；keyframe 不消费 target，验收中心=(0.43,0.31)。默认半径 0.08 m，严格回归用 0.03 m。见 [ADR 001](adr/001-cartesian-task-waypoints.md)、[ADR 002](adr/002-placement-target-verification-contract.md)。

### 6.3 离线 IK 与执行容差

任务空间为 world 表达的 hand_tcp，六维 `[x,y,z,rx,ry,rz]`，平移 m、旋转轴角 rad；Jacobian 行序 `[vx,vy,vz,wx,wy,wz]`。平移/旋转权重 1.0/0.2，sigma_min、条件数、阻尼阈值仅在相同权重下可比。

DLS 默认：阻尼阈值 0.08、最大阻尼 0.05、关节居中增益 0.02；误差裁剪 0.05 m/0.2 rad、单关节步长上限 0.12 rad、位置限位内缩 1e-4 rad。先求 DLS 再裁剪/投影，不保证约束最小二乘最优。IkStatus 为 Converged/MaxIterations/Stalled，失败返回有限末状态与残差，不伪造成功。

solveIk() 反馈来自运动学模型，不是实际执行状态；不修正伺服下垂、饱和或接触扰动，没有碰撞规划。模型 IK 残差、实测 TCP 误差、最终落点误差分别记录，不可互代。

FSM 默认位置/GRASP-CLOSE 位置/速度容差为 0.05 rad/0.3 rad/0.05 rad/s；min_settle=0.5 s、close_settle=2 s、lift_settle_grace=2 s、phase_timeout=6 s、max_retries=3。当前场景经验值已回归，但未证明最小/通用；改变模型、控制或场景需重测。

## 7. 验证门禁与已知限制

### 7.1 验证入口与证据

| 验证 | 覆盖/限制 |
| --- | --- |
| [三方模型测试](../src/mujoco_bridge/test/test_model_consistency.cpp) | 7 组固定 q，直接写状态/mj_forward，对照 MoveIt/自写 FK-Jacobian，不经 servo |
| 模型门槛 | link/TCP 位置 <1e-6 m、姿态 <1e-6 rad、Jacobian 元素 <1e-8，不作为运行伺服容差 |
| [camera_probe.py](../src/mujoco_bridge/test/camera_probe.py) | 精确同步、反投影/桌面/reset；仿真 smoke，不代表真机标定/动态噪声评估 |
| [掩膜单测](../src/mujoco_perception/test/test_robot_mask.cpp) / [probe](../src/mujoco_perception/test/robot_mask_probe.py) | 表面过滤、前后景保留、缺 TF/标定变化；probe 仅离线使用 oracle 统计误掩/残留 |
| [Cartesian 回归](../Job_guides/my_study/week3.md#11-stage-ocartesian-任务几何与离线-ik-适配) / [编排回归](../Job_guides/my_study/week3.5.md#15-p5回归架构记录和收尾) | 固定 oracle 场景连续 20/20，不外推随机姿态、视觉或真机 |
| [视觉验证](../Job_guides/my_study/week4.md#10-stage-4同帧机器人几何掩膜) | 掩膜/诊断基线、缺 TF、质量拒绝；完整视觉抓放未通过 |

link0..link4、link6、link7、hand 的 collision STL 在 apt/vendor 中 SHA256 相同；link5 与手指表示不同，不能推断全身碰撞结果一致。TCP FK/Jacobian 测试不验证可见网格投影。

### 7.2 未决事项与触发条件

- 接入 MoveIt 时决定 RSP TF 隔离与 home/ready 对齐方案，各自记录 ADR。
- keyframe qpos 静默补齐仍缺专门门禁；修改场景/keyframe 时检查 nq 与完整状态。
- 冲突比例缺 TF/标定偏差注入验证，不能启用未经验证的整帧拒绝阈值。
- Stage 5 已完成夹持/附着与四态实现及学习记录，但抬升期间仍可能因几何冲突解除；Stage 6 不得用 oracle 成功掩盖此限制。完全遮挡无法确认未滑落，最终成功需释放且移开机械臂后重新实测；完整视觉验收仍未完成。当前可见滑移门仅检查位置偏差，未独立检测纯旋转滑移。
- 当前单机器人/单 box 命名与夹爪假设，引入多臂/多物体时重新设计解析与目标身份。
- executor 为纯 grasp_criteria 依赖 bridge；替换真机驱动时评估共享库归属。改多线程 executor 时重新验证同步。

维护规则：按主题更新事实并替换过时描述；实测全文写周记，决策变化新增 ADR，未来计划明确状态。此页保留接口、关键默认值、失败行为与证据入口。
