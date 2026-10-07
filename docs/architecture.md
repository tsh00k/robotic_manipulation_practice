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
| `mujoco_perception` | 同帧机器人掩膜、几何候选检查与视觉证据；初始 box 位姿（深度均值 + 最小面积矩形） | 不读取物体真值/接触，不推断夹持或生成预测 |
| `arm_kinematics` | FK/Jacobian、加权 DLS 与离线 IK | 纯 C++/Eigen/yaml-cpp，不依赖 ROS、MoveIt、MuJoCo |
| `task_executor` | 观测适配、episode 编排、FSM、waypoint 与结果发布 | ROS 节点适配纯 C++ 控制器，当前单线程 executor |

主链路：`MuJoCo -> BridgeObservation (+ RGB-D/TF -> VisionObjectPose) -> task_executor -> joint/gripper command -> MuJoCo`。

成熟算法复用现有库；自写运动学作为独立学习/验证模块，项目自有代码集中在生命周期、契约适配与可测试的任务判断。选型理由进入 ADR 或周记，不在此重复平台与公开项目比较。

默认使用 oracle 观测与 diff_ik waypoint，固定关节表模式（`waypoint_source:=keyframe`）保留对照。固定 oracle 场景曾通过回归；完整 vision 抓放尚未验收。

当前 Stage 5 已简化为 bridge 权威附着、视觉暂停/释放重测与 executor 缓存准入，见 [Week 4 Stage 5](../Job_guides/my_study/week4.md#stage-5视觉接口收紧与简化状态机)。四包构建/测试通过；2026-10-03 修复抬升前附着循环依赖并补 LIFT 到位门，见 [ADR 012](adr/012-prelift-attachment-confirmation.md)。oracle 持续注入视觉拒绝的真实回归为 20/20 成功、零重试，两包 build/test 通过（工作区汇总 492 tests、0 errors、0 failures、71 skipped）；详情见 Week 4 Stage 5；当前接口 replay 和完整 vision episode 未执行。旧独立夹持与 TCP 预测实验只作为历史证据；[ADR 009](adr/009-robot-aware-stateful-vision.md)、[ADR 010](adr/010-grasp-conditioned-object-state.md)、[ADR 011](adr/011-vision-object-pose-contract-tightening.md) 保留当时决策快照，不能代替下文当前契约。原 Stage 6（可配置 bin 与运输监控）已废弃并回退，不在代码中，见 [Week 4 6.9](../Job_guides/my_study/week4.md#69-回退决定与存档) 与 [ADR 015](adr/015-rollback-stage6-perception-side-transport-diagnostics.md)；后续改动按 [Week 4.1](../Job_guides/my_study/week4.1.md) 的阶段计划逐个加入，每个阶段通过后再更新本页。Week 4.1 Stage 5、6 已加入初始 box 与 bin 检测（`~/initial_box_pose`、`~/initial_bin_pose`，见 [5.2](#52-机器人掩膜与几何估计) 末尾与 [5.3](#53-视觉消息与原因契约)，决策见 [ADR 018](adr/018-initial-box-detection-in-the-estimator.md)），估计器不再配对 RGB；Stage 7 起 vision 来源的 executor 在 reset 后等这两个估计连续一致并锁存，抓取目标取自锁存的 box，见 [5.5](#55-任务观测来源与质量门)；Stage 8 起场景里有 bin 时放置目标与支撑高度取自 bin（vision 用锁存的视觉 bin，oracle 用 bridge 的 `~/ground_truth/bin_pose`），见 [5.5](#55-任务观测来源与质量门)。MoveIt 规划与 learned policy 尚未完成。

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
| `model_path` 默认 | 项目自有 `pick_place_scene.xml`（`scene.enabled=true` 时为 `pick_place_bin_scene.xml`，见下） |
| `reset_keyframe_name` 默认 | `pick_place_home`；16 维 qpos = 7 臂 + 2 手指 + box 3 平移/4 四元数 |

**可选 box/bin 起始布局（Week 4.1 Stage 1，bridge 专有，默认关闭）。** `scene.enabled:=true` 时 bridge 加载 [pick_place_bin_scene.xml](../robot_description/mujoco/franka_emika_panda/pick_place_bin_scene.xml)（`include` 默认场景并加一个静态 bin：底板 + 四壁，12 mm，有碰撞），并按 `scene.box.{x,y,z,roll,pitch,yaw}` / `scene.bin.*` 设置起始位姿；launch 对应 `scene_enabled`、`box_*`、`bin_*`（缺省为 `auto`，只传用户给出的值）。长度 m、角度 rad、R=Rz(yaw)Ry(pitch)Rx(roll)；`z` 省略取旋转后最低角点距桌面 1 mm；box 原点为中心，bin 原点为内底面中心。位姿写入 `qpos0`、全部 keyframe 的 qpos 与 bin body，reset 恢复同一布局。出桌、穿桌、非有限值、bin 倾斜超约 20°、box 与 bin 包围盒间距小于 20 mm、关闭时给出位姿，都使 bridge 以状态 1 退出并说明原因。**关闭时模型与旧版逐字节相同**：把隐藏的 bin 放进默认场景会使相机 RGB 有 5208 个像素变化（深度与观测不变，根因未明），所以 bin 在单独的文件里。这些位姿是仿真真值，只有 bridge 声明参数，perception 与 executor 不读取；executor 的 place 目标与验收仍是固定位置，所以开启后的抓放结果不代表入 bin（Stage 13、15 才改）。决策见 [ADR 015](adr/015-rollback-stage6-perception-side-transport-diagnostics.md)，过程与验证见 [Week 4.1 Stage 1](../Job_guides/my_study/week4.1.md#stage-1bridge-可配置场景默认关闭)。

换回 panda.xml 时须同时改用 home。找不到指定 keyframe 会使 reset 显式失败；带 box 场景误用仍存在的 vendor home 则可能成功返回并把 box 置于补齐后的错误位置。

### 2.3 初始构型

| 来源 | joint1..joint7（rad） | 单指位置（m） |
| --- | --- | --- |
| vendor MJCF home | `[0, 0, 0, -1.5708, 0, 1.5708, -0.7853]` | 0.04 |
| SRDF ready / fake system initial_positions.yaml | `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]` | SRDF open 为 0.035 |
| **pick_place_home（Week 5 Stage 2 起）** | 同 ready：`[0, -π/4, 0, -3π/4, 0, π/2, π/4]`，TCP 约 `(0.307, 0, 0.487) m`、工具朝下 | 0.04 |

**用语：** 本文的 keyframe 指 MJCF `<keyframe>` 里带名字的**预设状态**（qpos/qvel/act/ctrl/mocap 的快照），不是动画或视频的关键帧；executor 的 `waypoint_source:=keyframe` 是另一回事（固定关节表，不读 MJCF 预设状态，见 6.2）。

reset 权威是预设状态 pick_place_home。Week 5 Stage 2 起它的臂构型是 Franka ready（此前沿用 vendor home），因为 vendor home 时手和前臂挡住相机看工作区远侧（默认 bin (0.5, 0.3) 被截断，Week 4.1 14.6）；ready 时 estimator 的机器人遮罩投到桌面后离工作区 x∈[0.30,0.70]、y∈[−0.30,0.40] 最近 80 mm。**bridge 启动即进入该预设状态**（generation 仍为 0），不再从 `mj_makeData` 的 qpos0（臂近乎竖直）开始，所以第一次 reset 不移动手臂；executor 的 HOME 是同一关节构型（`home.joint_positions`，默认 `kFrankaReadyPose`，单测核对与 MJCF 一致）。URDF 不保存初始状态。

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

两服务共用实现（bridge 启动时也执行一次，但不递增 generation）：恢复预设状态的 qpos/qvel/act/ctrl/mocap，保留 mjData::time 单调，调用 mj_forward 刷新派生量。bridge 是唯一 /clock 来源。reset 为仿真专有接口；真机回初始位姿需要可取消、有反馈的轨迹 action。

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
| bin（Week 4.1 Stage 8） | /mujoco_bridge/ground_truth/bin_pose，PoseStamped，world，bin 原点 = 内底面中心；transient local，仅 `scene.enabled` 时在启动时发布一次（bin 静态，reset 恢复同一布局）；只给 oracle 来源的放置目标用 |

## 5. RGB-D 与视觉契约

### 5.1 相机与精确同步

| 项目 | 默认值/约定 |
| --- | --- |
| 安装 | 项目 MJCF camera_link：world 平移 `(0.5,-0.45,1.0) m`，xyaxes=`1 0 0 0 0.857 0.514` |
| 光学轴 | MuJoCo +X 右/+Y 上/-Z 前；到 optical 绕 X 转 π，变成 +X 右/+Y 下/+Z 前 |
| 启用 | bridge 节点参数 enable_rgbd_camera 默认 false；demo launch 的同名参数默认 `auto`，随 observation_source 走（vision 开、oracle 关），显式 true/false 照办（Week 4.1 Stage 14 修正：Stage 11 把来源默认改成 vision 后，相机仍默认关闭，直接 launch 没有图像） |
| 图像 | /mujoco_bridge/camera/color/image_raw：rgb8；同前缀 depth/image_raw：32FC1。estimator 只消费深度（Stage 5 起不订阅 RGB） |
| 内参 | color/camera_info、depth/camera_info，相同零畸变内参；320×240、fovy=50°、fx=fy=257.34083046 px、cx=159.5/cy=119.5 |
| 深度 | 米制光轴 z-depth；OpenGL 缓冲经近远平面换算，无效/远裁剪写 NaN |
| 时间/frame | RGB、depth、两份 CameraInfo 共用物理步后仿真 stamp 与 camera_optical_frame |
| 频率 | 相机 10 Hz、TF/observation 100 Hz、物理步长 0.002 s |

反投影 `[(u-cx)z/fx,(v-cy)z/fy,z]` 后经静态 TF 转 world。相机消息与 BridgeObservation 按**完全相等 stamp（0 ns 容差）**配对，获取生命周期键并拒绝旧 generation，不能取各话题最新值拼接；estimator 配对的是深度图、深度相机信息、BridgeObservation 和机器人 TF，不再等待 RGB 及彩色相机信息（去掉后，旧路径 `object_pose` 在非附着深度帧上的发布比例由 62%、76% 变为 100%，丢帧原因未查，见 [Week 4.1 5.6.5](../Job_guides/my_study/week4.1.md#565-在线-episode-检查)）。相机 decimation 必须为 TF/observation decimation 整数倍。安装位置减轻 home 遮挡，但中心像素始终属于 box 不是不变量。

### 5.2 机器人掩膜与几何估计

<a id="14-stage-4-实测与后续目标契约"></a>

此旧锚点保留供 ADR 快照引用，历史评测见 [Week 4 Stage 4](../Job_guides/my_study/week4.md#10-stage-4同帧机器人几何掩膜)。

掩膜用 vendor 58 份可见 OBJ 网格，frame 为 link0..link7、hand、两指。geometric_shapes/Assimp 读网格，MoveIt moveit_mesh_filter 渲染预测深度，padding=0。动态机器人 TF 必须在图像同一 stamp；等待超过 0.5 s 且 depth/observation 仍在缓存时发布 MISSING_ROBOT_TRANSFORM；输入已淘汰则警告，不复用旧 TF。

有效观测/预测深度差绝对值 <= robot_mask.depth_tolerance_m=0.012 m 才过滤该像素，更近/更远有效深度均保留。无效输入或同过滤器生命周期中相机尺寸/内参变化报 INVALID_INPUT。当前无整帧冲突比例门，不发布 ROBOT_MODEL_MISMATCH。

管线（旧路径；Stage 11 起 executor 不再使用，估计器仍发布）：同帧配对/TF -> 反投影与掩膜 -> world ROI/桌面过滤 -> 水平支持平面 -> 全部候选簇 OBB 覆盖检查/已知盒体表面拟合 -> ObjectTracker 候选有效性/唯一性检查 -> VisionObjectPose。只去除配置桌面高度附近的平面，不把离桌盒体表面删作桌面；在线不再以最大簇决定目标，调试 target_cluster 是实际接受的候选。通用算法来自 image_geometry/PCL；SVD 已知对应点配准入口仍仅有单测。

先验为单个 4 cm 立方体与已知桌面。配置 anchor_z_to_plane=true 且 bridge 非 ATTACHED 时允许支撑锚定，两个较大 OBB 主尺寸各 ≥`box_min_visible_extent_m=0.008 m`，全部主尺寸 ≤边长加 `box_extent_tolerance_m=0.015 m`（默认 55 mm），中心 z 补为桌面加半高；立方体姿态为对称不确定的规范代表值。当前没有旧 55 mm 张爪门、Supported/Transport 模型，也没有恢复旧最高点支撑判定；新增低水平点片拒绝只过滤残留，是否真的落桌仍须验证。尺寸、残差与内点质量不能证明支撑接触。ATTACHED 时暂停正常处理链，退出附着后清缓存并重新测量。OBB 不提供任意形状识别或完全遮挡恢复。

支撑锚定下，规范立方体 XY 各枚举可见 AABB 中点、最小边界加半宽、最大边界减半宽，组合 9 个中心，选择点到已知盒体表面的平方残差和最小者；等代价优先保留局部中点。仅局部顶面仍不能唯一恢复真实 XY，残差不等于真实位姿误差；当前不支持任意 yaw 的完整搜索。水平厚度 ≤6 mm 且最高点低于模型中心的点片拒绝，防止支撑面残留被拟合为盒底。未锚定模式仍检查三个主尺寸的完整下限。

聚类最少 20 点；confidence=`min(n/box_confidence_reference_points,1) × inlier_ratio × clamp(1-residual/max_residual_m,0,1)`，点数参考值默认 40，残差上限 8 mm，内点比例下限 0.7。tracker confidence 门为 0.20；executor 仍要求 confidence ≥0.5、residual ≤5 mm、inlier ≥0.7。该分数不是校准后的统计概率。几何无效或低于 tracker confidence 门时记录每秒节流的原始候选质量日志；拒绝消息不暴露可用 pose/质量。决策见 [ADR 013](adr/013-partial-cube-geometry-admission.md)，默认场景实测见 [Week 4 Stage 5](../Job_guides/my_study/week4.md#582-排查记录局部候选尺寸与中心偏差)。

PCL ICP/GICP 已在离线真值标注点云与平面 fixture 上比较，仅作为评测工具；默认保持 OBB。当前全表面模型到局部点云的 ICP 可收敛但有约 20 mm 平面偏差，GICP 的运行成本与遮挡退化尚不满足默认在线要求；此结果不否定采用可见面模型等其他配准配置。评测入口为 `registration_benchmark.py` / `compare_registration`，oracle 标签只用于离线选取评测目标像素与计算误差。

**初始 box 与 bin 检测（Week 4.1 Stage 5、6，与上面的旧路径并行，输入是同一帧掩膜后的深度）。** `DepthWindow` 对最近 `initial_box.frames`（默认 10）帧做逐像素均值：某像素有效帧不足一半，或有效帧的深度最大值与最小值之差超过 20 mm（它在窗口内看到过两个表面），则该像素无效。`detectInitialBox()` 对均值深度图：反投影到 world；取高度在 `[顶面 − 15 mm, 顶面 + 40 mm]`（桌面 0.22 m、盒高 40 mm 时为 [0.245, 0.30] m）、x ∈ [0.2, 0.8]、y ∈ [−0.4, 0.4] 的像素；图像上 8 邻域连通域（`cv::connectedComponentsWithStats`，至少 20 像素）；每块丢掉 z，对 x–y 求最小面积外接矩形（`cv::minAreaRect`）；两边都在 40 ± 5 mm 的块才是 box，恰好一个才算检出。位置 x、y 取矩形中心，z 取顶面像素的中位数减半高，yaw 取矩形边方向并折到 [−45°, 45°)。bin（壁顶 0.239 m）整个在高度带之下，不会成为候选。不用颜色、不读真值、不用历史。**bin 用同样的步骤、不同的参数（`detectInitialBin()`）：** 高度带 [内底面 − 3.5 mm, 壁顶 + 4 mm]（内底面高于桌面 7 mm、壁高 12 mm 时为 [0.2235, 0.243] m），块的最小面积矩形两边在 152 ± 8 mm 与 142 ± 8 mm 内且恰好一块才是 bin；yaw 取长边方向（bin 自己的 x 轴）并折到 [−90°, 90°)；x、y 取矩形中心，z 取矩形中央区域（沿长边 ±56 mm、沿短边 ±51 mm）像素的中位数，即内底面高度，中央没有像素时拒绝而不是退回先验。bin 必须平放、空着、在视野里：倾斜的 bin 与里面已有盒子的情形不支持。两个检测器共用私有的块扫描（反投影、带内像素、连通域、最小面积矩形）。窗口在新 session/generation 与 ATTACHED→非 ATTACHED 时清空，ATTACHED 时不喂入。`InitialPoseEstimator` 把窗口与两个检测器合在一起（box 与 bin 共用同一个窗口，各检测一次），每个对象各自给出 WARMING_UP / NOT_MEASURED / MEASURED 三态；检测器、窗口与估计器类不创建节点，只用 sensor_msgs 的 CameraInfo 类型和 image_geometry，各有单测。已知局限：多帧平均假设各帧噪声独立（仿真深度几乎无帧间噪声，评测噪声为人为叠加）；只验证了 HOME 位姿、平放单个盒子和单个 bin、box 在 bin 相机一侧 ≥ 21 mm 的遮挡；检测不判断场景何时变化，靠节点在已知事件清空窗口。

调试 topic 前缀 /object_pose_estimator/debug/：robot_predicted_depth、robot_mask、filtered_depth、foreground_points、target_cluster、robot_mask_diagnostics，共用图像 stamp。mask=mono8（255 过滤），深度=米制 32FC1，点云=world。诊断带生命周期键、投影/掩掉/比较/冲突像素数、簇点数、容差与耗时。

comparison_pixels 为有效预测/观测重合数；mismatch_pixels 仅统计观测更远且超容差的像素。零比较数时比例无定义，不能报零冲突率；计数提示投影不一致，不能直接定位原因。

moveit_mesh_filter 依赖 X11/OpenGL，demo 为感知设置 LIBGL_ALWAYS_SOFTWARE=1。无显示部署需虚拟 X 或经过验证的无头后端；当前不支持运行中改变相机标定。

### 5.3 视觉消息与原因契约

`/object_pose_estimator/initial_box_pose` 类型为 InitialBoxPose（Stage 5），header 的 frame 为 world、stamp 为窗口里最新一帧，携带 bridge_session/generation/sample_sequence。非附着时每个可处理的深度帧发布一条，ATTACHED 期间不发布。`state` 为 WARMING_UP=0（窗口未满，`reason`=WINDOW_FILLING）、NOT_MEASURED=1（窗口已满而检测器拒绝，或这一帧不可用）、MEASURED=2；`reason` 为空表示 MEASURED，否则是检测器的拒绝名（NO_VALID_DEPTH、NO_BOX_BAND_PIXELS、NO_RECTANGLE_MATCHES_BOX、SEVERAL_BOX_CANDIDATES、INVALID_INPUT），不可用的帧为 INVALID_INPUT 或 MISSING_ROBOT_TRANSFORM（这样的帧不进窗口）。`frames_averaged`/`frames_required` 给出窗口进度；`position`（world，盒子中心）与 `yaw_rad`（[−π/4, π/4)，正方形每 90° 重复）只在 MEASURED 时有值，否则为 NaN；不用 Pose，因为 roll、pitch 没有被测量。`candidates[]` 列出高度带里每一块的像素数、矩形两边长、中心、是否被当作 box，用来在漏检时说明原因。没有置信度、残差、内点比。 `/object_pose_estimator/initial_bin_pose` 类型为 InitialBinPose（Stage 6），字段、状态、原因和发布时机与 InitialBoxPose 相同，区别只有：`position` 是 bin 内底面中心（z 为内底面高度），`yaw_rad` 是长边方向、范围 [−π/2, π/2)，拒绝名为 NO_BIN_BAND_PIXELS、NO_RECTANGLE_MATCHES_BIN、SEVERAL_BIN_CANDIDATES、NO_BIN_FLOOR_PIXELS（另有 NO_VALID_DEPTH、INVALID_INPUT）。两者的 `candidates[]` 都是 BlockCandidate（像素数、矩形两边长、中心、`matches`）。

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

BridgeObservation.attachment_state 为唯一权威：ATTACHMENT_NOT_ATTACHED=0、ATTACHMENT_ATTACHED=1、ATTACHMENT_RELEASED=2。启动/reset 为 NOT_ATTACHED。**Week 4.1 Stage 13 起（[ADR 019](adr/019-attachment-from-robot-side-signals.md)，变更 ADR 012）** bridge 只用真实 Franka Hand 也报告的量确认（对应 libfranka 的 `is_grasped`）：夹爪被命令合拢（最后一条命令 < 盒宽 − 容差 = 30 mm）、实际两指位置之和与盒宽相差 < `grasp.width_epsilon_m`（10 mm）、两指已停下（关节速度绝对值之和 < `grasp.attach_max_finger_speed_m_s`=20 mm/s），连续 `grasp.attach_hold_s`=0.1 s 仿真时间（`AttachmentConfirmer`，输入结构里没有盒子的量，也没有接触）。确认后锁存，接触抖动不解除；实际宽度超过盒宽 + 容差 + 5 mm（55 mm，留 5 mm 滞回）才转 RELEASED。reset 清锁存与计时。`BridgeObservation.left/right_finger_contact` 的含义随之改为“外部接触”（`bodyTouchesExternal`，与 `body_rootid` 不同于机器人的物体接触，不带身份）——**仿真专有，真实 Panda 没有指尖传感器**，不进入任何决策，只给 executor 的 CLOSE 超时贴诊断标签；按身份判的“碰到盒子”只在 `~/ground_truth/*_finger_contact`，以及附着日志行里标明“仅供评测”的部分。`classifyGrasp()`（含物体高度）与 `confirmsAttachment()`（ADR 012）保留作诊断，不驱动附着。

Stage 13 起这套确认不再读物体真值（位置与接触身份），但接触本身仍由仿真产生，附着权威仍在仿真 bridge；手指被盒子以外的东西挡住、停在 30~50 mm 时会误确认（与真实 `is_grasped` 同样的局限，未在线测过）；20 mm/s 的阈值只来自一个 episode 的实测。闭爪滑落仍为 ATTACHED（锁存只在张开超过 50 mm 时解除）；Week 4.1 Stage 12 起 executor 用搬运期开度窗口发现它，见 [5.5](#55-任务观测来源与质量门)，但只告警、不改变 episode。

**仿真故障注入（默认关闭，launch 不暴露）。** Stage 14 加了 `fault.truth_offset_x_m`（默认 0）：只把 bridge **发布**的盒子真值（`BridgeObservation.object_pose`、`~/ground_truth/object_pose`）沿 world x 平移，物理不动；用来在线证明 vision 来源不读真值。下面是 Stage 12 的那一个。

**（Stage 12）** bridge 参数 `fault.drop_box_after_attach_s`（默认 −1，关闭；launch 不暴露）：附着持续到该仿真时长时，把盒子的自由关节移回 reset keyframe 里的位置、速度清零，整个 bridge 运行期间只做一次，以便重试能成功。它改的是“世界”，不是机器人；用来复现“盒子滑出、手指合拢到约 0、附着锁存不变”。

estimator 的 tryProcess 在当前 ATTACHED 时早退，正常 RGB-D 几何处理及结果输出暂停，订阅回调/缓存仍可运行。ATTACHED→RELEASED 或 NOT_ATTACHED 时重置 tracker，清 depth/CameraInfo、pending 与 processed，并清空初始 box 的深度窗口，避免直接消费附着期旧图像；生命周期切换也重置历史（含该窗口）。释放状态不证明物体已落桌，须重新得到合格测量。

### 5.5 任务观测来源与质量门

observation_source 每 episode 唯一；节点参数默认 oracle，demo launch 的默认值自 Week 4.1 Stage 11 起是 vision（vision episode 第一次能完整放进 bin）。oracle 直接使用 bridge 物体真值，不受视觉 MEASURED/REJECTED 状态准入影响，但与 vision 共用附着和 FSM 阶段门。vision 在 VERIFY 之前的所有阶段消费锁存的初始位姿（见下，Week 4.1 Stage 7），VERIFY 消费新检测器重新测得的盒子（Stage 11）；executor 不再订阅 VisionObjectPose；关节/接触/TCP/宽度/附着仍来自 BridgeObservation。demo 的 vision 入口启动 estimator 与相机，不静默回退 oracle pose。

**初始位姿锁存（Week 4.1 Stage 7，vision 来源）。** 每次 reset（含重试，generation 变化）后，executor 订阅 `/object_pose_estimator/initial_box_pose` 与 `initial_bin_pose`，由纯规则类 `PoseLatch` 判定：最近 `latch.frames`=5 条连续 MEASURED 的 x、y 极差 ≤ `latch.max_position_spread_m`=3 mm、yaw 的圆周极差 ≤ `latch.max_yaw_spread_deg`=3°（box 以 90° 为周期、bin 以 180° 为周期）才锁存，锁存值是这几条里最新那条；非 MEASURED 的消息使累积清零；session 或 generation 不符、序号不递增的消息被忽略；锁存后到下一次 reset 前不变。锁存完成之前控制器停在“等待首个观测”，不进入 READY，所以不发任何关节命令；等待上限是 `latch.timeout_s`=10 s（墙钟，控制器的 awaiting-observation 超时，与 5 s 的数据流看门狗分开），超时的 outcome 是 `VISION_LATCH_TIMEOUT`（沿用 `VISION_` 开头归入感知层），失败原因写明还缺哪个对象及其状态（如 `BIN:WAITING:NO_RECTANGLE_MATCHES_BIN`）。`place.into_bin`（默认 false，demo launch 随 `scene_enabled` 设置；Stage 7 时叫 `latch.require_bin`）决定是否也等 bin；默认场景没有 bin。锁存后，VERIFY 之前所有阶段（含 OPEN、RETRACT）的 box 位姿都取自锁存值（confidence、residual 为 NaN，质量在状态话题里）；Stage 11 起按 FSM 阶段而不是附着状态切换，所以抓取时附着状态的反复不影响它。锁存的 box yaw 决定抓取时工具的转角（Stage 9，见 [6](#6-任务与运动执行) 的姿态说明）。

**放置目标（Week 4.1 Stage 8）。** 每个 episode 一个 `PlaceTarget{x, y, support_z}`，由节点在第一次准入观测之前交给控制器（`setPlacement`），控制器用同一个值算 PREPLACE / PLACE / OPEN / RETRACT 的 IK 目标，并以它的 x、y 作为 VERIFY 放置区的中心。PLACE 的 TCP 高度 = `support_z` + 盒子半高 + `target.place_tcp_above_box_center_m`（0.05 m），PREPLACE、RETRACT 再加 `target.hover_height_m`。`place.into_bin` 为 true 时，vision 来源取锁存的视觉 bin（`support_z` 是检测到的内底面高度），oracle 来源取 bridge 的 `/mujoco_bridge/ground_truth/bin_pose`（PoseStamped，transient local，只在 `scene.enabled` 时由 bridge 启动时发布一次；与 `~/ground_truth/object_pose` 同类，只给 oracle 路径用，vision 不订阅），在拿到之前不准入任何观测、不发命令，**不退回配置**；为 false 时用 `target.place_x_m`、`target.place_y_m` 与桌面 0.22 m（旧场景，没有 bin），VERIFY 中心用 `verify.place_x_m`、`verify.place_y_m`，两者只在 `verify.allow_target_mismatch` 实验里不同。工具朝向不变，不要求盒子最终 yaw 等于 bin yaw。状态话题 `/task_executor/initial_pose_latch`（InitialPoseLatch，内含两个 LatchedPose）给出 WAITING / LATCHED / FAILED、各对象的锁存位姿、来源序号与时间戳、极差和未锁住的原因。

**VERIFY（Week 4.1 Stage 11）。** VERIFY 阶段，vision 来源准入 `/object_pose_estimator/initial_box_pose` 的 MEASURED：与 bridge 样本按 session、generation、序号完全相等配对，缓存有界并跳过已处理序号；估计器在释放时清空窗口，所以这是盒子落下后的新测量（约释放后 1 s 才有第一条）。非 MEASURED 记录 `VISION_REJECTED:<检测器原因>` 并等待，不创建 snapshot；持续没有可准入观测仍受 5 s 的数据流看门狗约束，可终止为 OBSERVATION_STALE。**只在 VERIFY 要求新测量**：OPEN、RETRACT 用锁存值，因为它们不需要盒子，而张开的手还在 bin 上方时会挡住盒子（若在那时就只准入新测量，控制器不推进、手不撤离，形成死锁，Stage 11 实测过）。oracle 来源用真值。

**入 bin 判据。** `place.into_bin` 为 true 时 VERIFY 用纯函数 `boxInBin()`（[bin_containment.hpp](../src/task_executor/include/task_executor/bin_containment.hpp)）而不是半径：盒子（边长 40 mm，yaw 取自位姿）四角在 bin 坐标系里离内口（半宽 70 × 65 mm）至少 5 mm，且盒心高度在内底面 + 20 mm ± 5 mm 内，才算放好；其它（越界、压在壁上、离壁不到 5 mm 无法确认）一律不算，VERIFY 继续等，超时为 `PLACE_MISSED` 后按原逻辑重试。5 mm = 盒子检测限 3 mm + bin 在线最大偏差向上取 2 mm。bin 来自放置目标（vision：锁存的视觉 bin，含 yaw；oracle：bridge 的真值 bin）。VERIFY 期间日志每 0.5 s 打印一行 `verify:`，给出盒子位置、是否在内、最小角余量与高度差。没有 bin 时仍是以放置目标为圆心、`verify.place_region_radius_m` 为半径的判据。

`vision.min_confidence`、`vision.max_residual_m`、`vision.min_inlier_ratio` 只用于旧的 VisionObjectPose 准入；executor 自 Stage 11 起不再读该话题，这三个参数仍被声明和校验，但不再起作用（随旧路径一起清理，见 Week 4.1 Stage 5 的 5.11）。

**搬运期开度窗口（Week 4.1 Stage 12）。** executor 对每个 bridge 样本更新纯规则类 `CarryWidthMonitor`（[carry_width_monitor.hpp](../src/task_executor/include/task_executor/carry_width_monitor.hpp)）：只在 bridge 为 ATTACHED 且阶段是 LIFT / PREPLACE / PLACE 时计时；两指之和低于 34 mm（搬运中见过 37.6~40.5 mm）持续 0.2 s 仿真时间，产生一次告警（`CARRY_WIDTH_LOW: …`，日志 WARN），每次尝试一个新监视器。告警不改变 episode 的走向；后果由后面的阶段承担（盒子不在 bin 里 → VERIFY 超时 `PLACE_MISSED` → 重试）。上限不另设，张开超过 50 mm 由 bridge 转为 RELEASED。

EpisodeOutcome 记录来源、confidence、residual、失败层/原因，以及最后一次尝试的 `carry_width_alert`、`carry_width_min_m`、`carry_width_max_m`（Stage 12；没有搬运时开度为 NaN）；任务失败码与视觉 state_reason 属于不同接口。附着门、入 bin 判据及 controller 准入共同决定是否推进。Stage 11 起 vision 在随机 box + bin 布局上完整成功（3 个布局），HELD-A 上的验收是 Stage 14。

## 6. 任务与运动执行

### 6.1 EpisodeController 与 FSM

EpisodeController 独占生命周期、phase、retry、失败原因与 telemetry；ROS 节点负责转换、服务/话题、timer、日志。生命周期 Idle/ResetPending/AwaitingResetResponse/AwaitingObservation/Ready/Finished/Failed 与 Phase 分开，FSM 仅在 Ready 推进。任意状态可 start 新 episode，terminal outcome 仅一次。

任务顺序 `HOME -> PREGRASP -> GRASP -> CLOSE -> LIFT -> PREPLACE -> PLACE -> OPEN -> RETRACT -> VERIFY -> DONE`，执行异常经 RECOVER 重试，耗尽后 FAILED。纯 step() 不依赖 ROS/MuJoCo；executor 的 classifyGrasp() 复用 mujoco_bridge::grasp_criteria，仅用于 CLOSE 超时原因分类，不作为第二套附着权威。

CLOSE 要求 ATTACHED+close_settle；LIFT 要求 ATTACHED+机械臂关节位置/速度到达抬升目标+min_settle，不以视觉高度判定。LIFT 到达臂目标但未附着且超过 grace 时 recover/slipped，阶段超时 recover/timeout。PREPLACE/PLACE 先检查 ATTACHED，缺失时 recover/slipped，否则检查运动到位；这些门依赖 bridge 锁存，不能独立检测闭爪滑落。OPEN 要求实际宽度 >0.06 m 与 settle；VERIFY 的臂目标是 HOME（Week 5 Stage 2），要求 RELEASED、物体在验收区域（有 bin 时为入 bin 判据）、机械臂回到 HOME 及 settle，vision 路径须释放后的新测量；所以 DONE 时手臂已在 reset 构型。

20 Hz tick 最多消费一份新鲜观测，设 IK seed、求当前目标、调用 FSM、记录迁移；无新观测不重发旧目标，新观测下 phase 未变则重发。动作顺序为当前 phase 目标、迁移日志、reset/outcome。阶段计时用仿真时间，看门狗用 steady clock，准入先检查超时再刷新新鲜度。见 [ADR 004](adr/004-episode-controller-orchestration.md) 与 [编排图](task_executor_episode_orchestration.html)。

### 6.2 Cartesian 任务与 waypoint

WaypointSource::jointTargetFor(Phase,ObjectPose) 返回关节目标。默认 DiffIkWaypointSource 消费独立、无 ROS 的 CartesianWaypointSource，以实测关节作 seed、阶段内缓存离线 IK 目标；HOME 与 VERIFY 不求 IK，直接用 `home.joint_positions`（Week 5 Stage 2）；HOME/retry 清抓取锁定与缓存。失败报 IK_FAILED，不发布失败解。固定关节表模式（`keyframe`）查表且忽略物体位姿，详表/调参见 [Week 2 Stage I](../Job_guides/my_study/week2.md#10-stage-ifsm-与-waypointsource-抽象新包-task_executor)。

| 默认几何 | world-frame hand_tcp 目标 |
| --- | --- |
| HOME、VERIFY | 关节目标 `home.joint_positions`（= pick_place_home 的臂构型，见 2.3），不经 IK；表中其余阶段经 IK |
| GRASP/CLOSE | 首次 PREGRASP 锁定的物体中心 |
| PREGRASP/LIFT | 锁定中心上方 0.15 m |
| PLACE/OPEN | 放置目标 XY（无 bin 时 `(0.5,0.3) m`），支撑面+半高+0.05 m：桌面时 z=0.29 m，bin 内底面时 0.297 m（Week 4.1 Stage 8） |
| PREPLACE/RETRACT | 放置目标上方 0.15 m：桌面时 z=0.39 m |

姿态 `R_world_tcp=Rz(tool_yaw_rad)*R_down`，R_down 绕 `(1,1,0)/sqrt(2)` 转 π；默认 yaw=0 时 TCP x/y/z 指向 world +y/+x/-z。参数不是 ZYX Euler yaw。**Week 4.1 Stage 9 起** `target.align_tool_to_box_yaw`（默认 true）把盒子 yaw 折到 [−45°, 45°) 加进 `tool_yaw_rad`，除 HOME 外所有阶段都用：`R_world_tcp=Rz(tool_yaw_rad+fold(box_yaw))*R_down`。盒子 yaw 取自 PREGRASP 时的盒子位姿（vision 是锁存值，oracle 是当时的真值），diff-IK 源从 PREGRASP 起对所有阶段都用这个位姿，所以抓住盒子后不再转腕；放进 bin 时不要求盒子 yaw 与 bin 对齐，也不验证最终 box 姿态。CLOSE/LIFT/PREPLACE/PLACE 全闭（总宽 0 m），其他阶段张开（0.08 m）。

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
| [DepthWindow](../src/mujoco_perception/test/test_depth_window.cpp) / [初始 box 检测器](../src/mujoco_perception/test/test_initial_box_detector.cpp) / [初始 bin 检测器](../src/mujoco_perception/test/test_initial_bin_detector.cpp) / [InitialPoseEstimator](../src/mujoco_perception/test/test_initial_pose_estimator.cpp) | 单测在精确光线投射的合成场景上，答案由构造给出（相机与场景在 [workcell_camera.hpp](../src/mujoco_perception/test/workcell_camera.hpp)），共 24 项；场景固定，所以不测非法配置、不会出现的场景和相机分辨率变化。容差是像素量化界，不是调出来的（bin 的边长用检测器自己的 ±8 mm 规则） |
| `initial_box_probe.py place` | Stage 8：随机 box + bin 布局，oracle 与 vision 各自的 PREPLACE / PLACE 目标对 bin（真值或锁存值）、释放时指尖与壁顶的余量、盒子最终是否在 bin 内（按真值判）；`--scene legacy` 核对旧场景的固定目标。仿真 |
| `initial_box_probe.py held` / `held-summary` | Week 4.1 Stage 14：留出集 HELD-A（40 个随机 box + bin 布局）上 vision 39/40、oracle 40/40；过程断言 P1~P6（锁存前无命令、锁存误差、抓取与放置目标、无开度告警、无假成功）、负向断言 N1~N5（含用 `fault.truth_offset_x_m` 平移发布的真值，vision 不受影响而 oracle 失败）。仿真，HELD-A 此后为回归集 |
| [附着确认单测](../src/mujoco_bridge/test/test_grasp_criteria.cpp) / `initial_box_probe.py carry --mode attach` | Stage 13：持续 0.1 s 才确认、空夹/手指仍在动/无合拢命令/宽度不符不确认、断一个样本重新计时；在线核对附着时手指确实碰到盒子（评测端）、合拢到附着的时间。仿真 |
| [开度窗口单测](../src/task_executor/test/test_carry_width_monitor.cpp) / `initial_box_probe.py carry` | Stage 12：窗口内不报、短暂低于下限不报、持续 0.2 s 报一次、只看搬运期；正常运行不误报，注入 `fault.drop_box_after_attach_s` 后 0.5 s 内报出。仿真 |
| [入 bin 判据单测](../src/task_executor/test/test_bin_containment.cpp) / `initial_box_probe.py verify`、`place --require-success` | Stage 11：判据的居中、越界、离壁不足余量、压壁、bin 旋转；VERIFY 时新检测器对真值的误差（6 个布局）；oracle 与 vision 在随机布局上完整成功并按真值入 bin。仿真 |
| [PoseLatch 单测](../src/task_executor/test/test_pose_latch.cpp) / `initial_box_probe.py latch` | 前者覆盖连续一致才锁存、不一致或非 MEASURED 重新累积、其它 generation 与旧序号被忽略、yaw 按物体周期比较、锁存后不变；后者起真实 bridge + 估计器 + executor 核对：锁存前无关节命令、锁存值对真值、GRASP 目标取自锁存值、重新锁存、缺 bin 时的 `VISION_LATCH_TIMEOUT`。仿真 |
| [initial_box_compare.py](../src/mujoco_perception/test/initial_box_compare.py) / [initial_box_probe.py](../src/mujoco_perception/test/initial_box_probe.py) | 前者在同一批录制帧上对照 C++ 与 Python 原型；后者起真实 bridge + estimator（+ executor）核对消息的状态、窗口计数与误差。验收规则写在各自的 docstring，在运行前写定；仿真，不代表真机 |
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
