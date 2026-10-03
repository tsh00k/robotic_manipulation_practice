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
| `mujoco_bridge` | 物理步进、命令执行、reset、clock、TF、同一步观测、RGB-D 与附着生命周期 | 仿真 ground truth 的唯一发布者，不执行视觉估计 |
| `manipulation_interfaces` | 观测、视觉、结果和 reset 消息/服务 | ROS 数据契约 |
| `mujoco_perception` | 同帧机器人掩膜、几何候选检查与视觉证据 | 不读取物体真值/接触，不推断夹持或生成预测 |
| `arm_kinematics` | FK/Jacobian、加权 DLS 与离线 IK | 纯 C++/Eigen/yaml-cpp，不依赖 ROS、MoveIt、MuJoCo |
| `task_executor` | 观测适配、episode 编排、FSM、waypoint 与结果发布 | ROS 节点适配纯 C++ 控制器，当前单线程 executor |

主链路：`MuJoCo -> BridgeObservation (+ RGB-D/TF -> VisionObjectPose) -> task_executor -> joint/gripper command -> MuJoCo`。

成熟算法复用现有库；自写运动学作为独立学习/验证模块，项目自有代码集中在生命周期、契约适配与可测试的任务判断。选型理由进入 ADR 或周记，不在此重复平台与公开项目比较。

默认使用 oracle 观测与 diff_ik waypoint，keyframe 保留对照。固定 oracle 场景曾通过回归；完整 vision 抓放尚未验收。

当前 Stage 5 已简化为 bridge 权威附着、视觉暂停/释放重测与 executor 缓存准入，见 [Week 4 Stage 5](../Job_guides/my_study/week4.md#stage-5视觉接口收紧与简化状态机)。四包构建/测试通过；2026-10-03 修复抬升前附着循环依赖并补 LIFT 到位门，见 [ADR 012](adr/012-prelift-attachment-confirmation.md)。oracle 持续注入视觉拒绝的真实回归为 20/20 成功、零重试，两包 build/test 通过（工作区汇总 492 tests、0 errors、0 failures、71 skipped）；详情见 Week 4 Stage 5；当前接口 replay 和完整 vision episode 未执行。旧独立夹持与 TCP 预测实验只作为历史证据；[ADR 009](adr/009-robot-aware-stateful-vision.md)、[ADR 010](adr/010-grasp-conditioned-object-state.md)、[ADR 011](adr/011-vision-object-pose-contract-tightening.md) 保留当时决策快照，不能代替下文当前契约。Stage 6 联调验收、MoveIt 规划与 learned policy 尚未完成。

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

`/mujoco_bridge/episode_observation`（BridgeObservation）在同一次物理步后读取同一份 mjData，打包 session/generation/sequence、仿真 stamp、9 关节状态、box pose、world-frame TCP、双指接触与 attachment_state。executor 不再混用独立 joint_states 和 TF 作决策输入。

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

掩膜用 vendor 58 份可见 OBJ 网格，frame 为 link0..link7、hand、两指。geometric_shapes/Assimp 读网格，MoveIt moveit_mesh_filter 渲染预测深度，padding=0。动态机器人 TF 必须在图像同一 stamp；等待超过 0.5 s 且 depth/observation 仍在缓存时发布 MISSING_ROBOT_TRANSFORM；输入已淘汰则警告，不复用旧 TF。

有效观测/预测深度差绝对值 <= robot_mask.depth_tolerance_m=0.012 m 才过滤该像素，更近/更远有效深度均保留。无效输入或同过滤器生命周期中相机尺寸/内参变化报 INVALID_INPUT。当前无整帧冲突比例门，不发布 ROBOT_MODEL_MISMATCH。

管线：同帧配对/TF -> 反投影与掩膜 -> world ROI/桌面过滤 -> 水平支持平面 -> 全部候选簇 OBB 覆盖检查/已知盒体表面拟合 -> ObjectTracker 候选有效性/唯一性检查 -> VisionObjectPose。只去除配置桌面高度附近的平面，不把离桌盒体表面删作桌面；在线不再以最大簇决定目标，调试 target_cluster 是实际接受的候选。通用算法来自 image_geometry/PCL；SVD 已知对应点配准入口仍仅有单测。

先验为单个 4 cm 立方体与已知桌面。配置 anchor_z_to_plane=true 且 bridge 非 ATTACHED 时允许支撑锚定，两个较大 OBB 主尺寸各 ≥`box_min_visible_extent_m=0.008 m`，全部主尺寸 ≤边长加 `box_extent_tolerance_m=0.015 m`（默认 55 mm），中心 z 补为桌面加半高；立方体姿态为对称不确定的规范代表值。当前没有旧 55 mm 张爪门、Supported/Transport 模型，也没有恢复旧最高点支撑判定；新增低水平点片拒绝只过滤残留，是否真的落桌仍须验证。尺寸、残差与内点质量不能证明支撑接触。ATTACHED 时暂停正常处理链，退出附着后清缓存并重新测量。OBB 不提供任意形状识别或完全遮挡恢复。

支撑锚定下，规范立方体 XY 各枚举可见 AABB 中点、最小边界加半宽、最大边界减半宽，组合 9 个中心，选择点到已知盒体表面的平方残差和最小者；等代价优先保留局部中点。仅局部顶面仍不能唯一恢复真实 XY，残差不等于真实位姿误差；当前不支持任意 yaw 的完整搜索。水平厚度 ≤6 mm 且最高点低于模型中心的点片拒绝，防止支撑面残留被拟合为盒底。未锚定模式仍检查三个主尺寸的完整下限。

聚类最少 20 点；confidence=`min(n/box_confidence_reference_points,1) × inlier_ratio × clamp(1-residual/max_residual_m,0,1)`，点数参考值默认 40，残差上限 8 mm，内点比例下限 0.7。tracker confidence 门为 0.20；executor 仍要求 confidence ≥0.5、residual ≤5 mm、inlier ≥0.7。该分数不是校准后的统计概率。几何无效或低于 tracker confidence 门时记录每秒节流的原始候选质量日志；拒绝消息不暴露可用 pose/质量。决策见 [ADR 013](adr/013-partial-cube-geometry-admission.md)，默认场景实测见 [Week 4 Stage 5](../Job_guides/my_study/week4.md#582-排查记录局部候选尺寸与中心偏差)。

PCL ICP/GICP 已在离线真值标注点云与平面 fixture 上比较，仅作为评测工具；默认保持 OBB。当前全表面模型到局部点云的 ICP 可收敛但有约 20 mm 平面偏差，GICP 的运行成本与遮挡退化尚不满足默认在线要求；此结果不否定采用可见面模型等其他配准配置。评测入口为 `registration_benchmark.py` / `compare_registration`，oracle 标签只用于离线选取评测目标像素与计算误差。

调试 topic 前缀 /object_pose_estimator/debug/：robot_predicted_depth、robot_mask、filtered_depth、foreground_points、target_cluster、robot_mask_diagnostics，共用图像 stamp。mask=mono8（255 过滤），深度=米制 32FC1，点云=world。诊断带生命周期键、投影/掩掉/比较/冲突像素数、簇点数、容差与耗时。

comparison_pixels 为有效预测/观测重合数；mismatch_pixels 仅统计观测更远且超容差的像素。零比较数时比例无定义，不能报零冲突率；计数提示投影不一致，不能直接定位原因。

moveit_mesh_filter 依赖 X11/OpenGL，demo 为感知设置 LIBGL_ALWAYS_SOFTWARE=1。无显示部署需虚拟 X 或经过验证的无头后端；当前不支持运行中改变相机标定。

### 5.3 视觉消息与原因契约

主结果 `/object_pose_estimator/object_pose` 类型为 VisionObjectPose，header 表达 world 与图像 stamp，携带 bridge_session/generation/sample_sequence。当前生产 evidence_state 为 REJECTED=0、MEASURED=1、OCCLUDED=3；PREDICTED=2 常量仍在，但当前无生产路径。MEASURED 要求恰好一个有效候选，不排除同时存在无效候选。

有效性检查为 geometry_valid、至少 3 点、有限位置与四元数、范数误差 ≤1e-3、有限且 confidence ≥tracking.min_confidence（默认 0.20），以及有限 residual/inlier。当前没有历史运动门、速度预测、TCP 附着校正或视觉滑移检测。无测量时 residual_m/inlier_ratio 为 NaN、point_count 为 0，默认 pose 不可执行；last_measurement_sequence 只表示最近实测序号。orientation_ambiguous 标记姿态代表值歧义，processing_ms 不含配对等待或相机渲染。

`state_reason` 是视觉原因的唯一字段，诊断名对应 `DIAGNOSTIC_*` 常量：

| state_reason | evidence_state | diagnostic_stage | 触发条件与检查方向 |
| --- | --- | --- | --- |
| 空字符串 | MEASURED | NONE | 恰好一个合格候选；任务层仍需检查质量门 |
| NO_CANDIDATE | OCCLUDED | GEOMETRY | 候选为空；检查遮挡、深度、掩膜与聚类 |
| CANDIDATE_INVALID | REJECTED | GEOMETRY | 有候选但无合格候选；检查尺寸、点数和质量 |
| MULTIPLE_CANDIDATES | REJECTED | ASSOCIATION | 两个或更多合格候选；不靠历史运动门挑目标 |
| INVALID_INPUT | REJECTED | INPUT | 图像/掩膜/分割输入无效，或 tracker 序号为零、时间非法 |
| MISSING_ROBOT_TRANSFORM | REJECTED | INPUT | 同帧机器人 TF 超时或查询失败 |
| LIFECYCLE_MISMATCH | REJECTED | LIFECYCLE | tracker 的 session/generation 不匹配 |
| OUT_OF_ORDER | REJECTED | LIFECYCLE | sequence 或 stamp 小于或等于上一已处理样本 |

正常原因是空字符串，不是字符串 NONE。INPUT/GEOMETRY/ASSOCIATION/LIFECYCLE 用于当前原因，PREDICTION/GRASP 常量保留但不用于当前正常路径。节点入口过滤已退休 session 和旧 generation；LIFECYCLE_MISMATCH 是 tracker 拒绝契约，不保证每个旧消息都有结果。sequence 和 stamp 必须严格递增，相等也拒绝。

candidate_count/eligible_candidate_count 表示全部/合格候选数；support_prior_used 表示配置与生命周期允许支撑先验，不证明接触。grasp_state（GRASP_NOT_ATTACHED=0、GRASP_HELD=1、GRASP_RELEASED=2）与 attachment_valid 尚存在，直接映射 bridge 状态；这是派生副本，权威仍是 BridgeObservation，且附着期旧视觉消息不代表当前反馈。消息不再携带宽度、预测年龄、不确定性、关联距离/门限或 grasp_reason；不承诺旧布局兼容。

### 5.4 bridge 附着生命周期与视觉暂停

BridgeObservation.attachment_state 为唯一权威：ATTACHMENT_NOT_ATTACHED=0、ATTACHMENT_ATTACHED=1、ATTACHMENT_RELEASED=2。启动/reset 为 NOT_ATTACHED；bridge 用实际两指位置之和、物体真值高度、物体到 TCP 的 XY 距离及双指接触构造 GraspSignals，confirmsAttachment 在宽度差 <10 mm、双指接触、有限且非负的 XY 距离 <50 mm 时进入 ATTACHED，不要求先抬高。物体高度仅用于 classifyGrasp 的抓取诊断（成功要求 z >0.26 m），不能作为抬升前附着门。确认后锁存，接触短暂抖动或闭爪物体滑落不会直接解除；实际宽度超过 box_width_m+width_epsilon_m 才转 RELEASED。后续满足抬升前确认可再附着，reset 清锁存。

这套确认依赖仿真物体真值/接触；vision 替换的是任务物体位姿来源，不代表完全没有 oracle 信息。闭爪滑落可能仍为 ATTACHED，FSM 附着门也无法补足这一盲区。

estimator 的 tryProcess 在当前 ATTACHED 时早退，正常 RGB-D 几何处理及结果输出暂停，订阅回调/缓存仍可运行。ATTACHED→RELEASED 或 NOT_ATTACHED 时重置 tracker，清 RGB/depth/CameraInfo、pending 与 processed，避免直接消费附着期旧图像；生命周期切换也重置历史。释放状态不证明物体已落桌，须重新得到合格测量。

### 5.5 任务观测来源与质量门

observation_source 每 episode 唯一，默认 oracle。oracle 直接使用 bridge 物体真值，不受视觉 MEASURED/REJECTED 状态准入影响，但与 vision 共用附着和 FSM 阶段门。vision 消费 VisionObjectPose；关节/接触/TCP/宽度/附着仍来自 BridgeObservation。demo 的 vision 入口启动 estimator 与相机，不静默回退 oracle pose。

非附着时，vision 与 bridge 的 session/generation/sequence 完全相等才配对，缓存有界并跳过已处理序号；只准入新的 MEASURED，且各质量指标必须有限：

| 参数 | 通过条件 |
| --- | --- |
| vision.min_confidence=0.5 | confidence ≥ 阈值 |
| vision.max_residual_m=0.005 | residual_m ≤ 阈值 |
| vision.min_inlier_ratio=0.7 | inlier_ratio ≥ 阈值 |

质量失败记录 VISION_REJECTED 或 VISION_LOW_CONFIDENCE 诊断并等待，不直接结束 episode、不创建新 snapshot。持续没有可准入观测仍受 controller 的 steady-clock 新鲜度看门狗约束，可终止为 OBSERVATION_STALE；不能把等待写成无限重试或 FSM 恢复。

ATTACHED 时要求同 session/generation 的最近已接受视觉样本存在，复用其位姿数值与当前 bridge 反馈。数值保持不变，不以 TCP 外推，不发布 PREDICTED，也不证明当前物体高度；内部上下文 BRIDGE_ATTACHMENT_ATTACHED 不是视觉 state_reason。退出附着清最近视觉、已接受缓存和已处理序号，等待新的质量合格 MEASURED。释放后 VERIFY 使用新测量落点，不使用运输缓存。

EpisodeOutcome 记录来源、confidence、residual、失败层/原因；任务失败码与视觉 state_reason 属于不同接口。视觉质量门、附着门及 controller 准入共同决定是否推进，完整 vision 成功仍待集成验收。

## 6. 任务与运动执行

### 6.1 EpisodeController 与 FSM

EpisodeController 独占生命周期、phase、retry、失败原因与 telemetry；ROS 节点负责转换、服务/话题、timer、日志。生命周期 Idle/ResetPending/AwaitingResetResponse/AwaitingObservation/Ready/Finished/Failed 与 Phase 分开，FSM 仅在 Ready 推进。任意状态可 start 新 episode，terminal outcome 仅一次。

任务顺序 `HOME -> PREGRASP -> GRASP -> CLOSE -> LIFT -> PREPLACE -> PLACE -> OPEN -> RETRACT -> VERIFY -> DONE`，执行异常经 RECOVER 重试，耗尽后 FAILED。纯 step() 不依赖 ROS/MuJoCo；executor 的 classifyGrasp() 复用 mujoco_bridge::grasp_criteria，仅用于 CLOSE 超时原因分类，不作为第二套附着权威。

CLOSE 要求 ATTACHED+close_settle；LIFT 要求 ATTACHED+机械臂关节位置/速度到达抬升目标+min_settle，不以视觉高度判定。LIFT 到达臂目标但未附着且超过 grace 时 recover/slipped，阶段超时 recover/timeout。PREPLACE/PLACE 先检查 ATTACHED，缺失时 recover/slipped，否则检查运动到位；这些门依赖 bridge 锁存，不能独立检测闭爪滑落。OPEN 要求实际宽度 >0.06 m 与 settle；VERIFY 要求 RELEASED、物体 XY 在验收区域及 settle，vision 路径须释放后的新测量。

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
| [当前简化接口测试](../Job_guides/my_study/week4.md#51-改动清单与验证结果) | 2026-10-03 四包 build/test 通过，tracker 11 项通过；工作区汇总 487 tests / 0 errors / 0 failures / 71 skipped（含既有其他包结果），非完整 vision 成功证据 |

link0..link4、link6、link7、hand 的 collision STL 在 apt/vendor 中 SHA256 相同；link5 与手指表示不同，不能推断全身碰撞结果一致。TCP FK/Jacobian 测试不验证可见网格投影。

### 7.2 未决事项与触发条件

- 接入 MoveIt 时决定 RSP TF 隔离与 home/ready 对齐方案，各自记录 ADR。
- keyframe qpos 静默补齐仍缺专门门禁；修改场景/keyframe 时检查 nq 与完整状态。
- 冲突比例缺 TF/标定偏差注入验证，不能启用未经验证的整帧拒绝阈值。
- 附着锁存抗接触抖动，也可能掩盖闭爪掉落；vision 附着确认仍有仿真真值/接触依赖。释放后的新测量是最终落点证据，完整视觉验收未完成。
- 新消息布局录制与 replay 联调待做：旧 CDR 不保证兼容；tracking_replay.py 逐帧等待结果，与 ATTACHED 预期静默不兼容，须先适配断言再验收。
- 支撑锚定不证明实际落桌，候选唯一性也不保证跨遮挡目标身份；多目标或离桌场景需新证据门。
- 当前单机器人/单 box 命名与夹爪假设，引入多臂/多物体时重新设计解析与目标身份。
- executor 为纯 grasp_criteria 依赖 bridge；替换真机驱动时评估共享库归属。改多线程 executor 时重新验证同步。

维护规则：按主题更新事实并替换过时描述；实测全文写周记，决策变化新增 ADR，未来计划明确状态。此页保留接口、关键默认值、失败行为与证据入口。
