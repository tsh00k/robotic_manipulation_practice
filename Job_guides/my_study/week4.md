# Week 4 学习笔记

> Stage P~R 已记录实现与实测；R1~R3 是视觉抓放尚待实施的后续计划，完成后再按同样流程追加代码、验证、讲解与追问。Week 4.5 的策略接口、数据链和容器交付不混入本周主线，见 [week4.5.md](week4.5.md)。

## 学习重点范围

本周进入计划书 Chap 4 的 RGB-D 与几何位姿估计，建立从 MuJoCo 相机到 `world` frame 的可验证感知链。当前项目已有 ground-truth object pose、`BridgeObservation` 和 Cartesian waypoint；本周的工作是加入相机观测和视觉估计，同时保留 oracle 对照。

本周要掌握：

1. 相机内参、深度图、点云和外参的关系。
2. camera frame 到 `world` frame 的坐标变换和时间戳语义。
3. 桌面去除、目标分割、已知对应 SVD 配准和位姿残差。
4. 视觉置信度、低质量观测拒绝和任务层重新观察。
5. oracle、vision 与后续 policy 的输入边界。oracle 和 vision 是观测来源，policy 是执行方式，不能把三者当成互斥模式。

## 目录

- [1. 从前三周继承的事实和边界](#1-从前三周继承的事实和边界)
- [2. 本周 Stage 计划](#2-本周-stage-计划)
  - [2.1 Stage P：RGB-D 相机和世界坐标观测](#21-stage-prgb-d-相机和世界坐标观测)
  - [2.2 Stage Q：几何分割和刚体位姿估计](#22-stage-q几何分割和刚体位姿估计)
  - [2.3 Stage R：oracle/vision 对照和任务接入](#23-stage-roraclevision-对照和任务接入)
  - [2.4 Stage R1：同帧机器人几何掩膜](#24-stage-r1同帧机器人几何掩膜)
  - [2.5 Stage R2：目标关联与遮挡状态估计](#25-stage-r2目标关联与遮挡状态估计)
  - [2.6 Stage R3：任务证据门与完整抓放验收](#26-stage-r3任务证据门与完整抓放验收)
- [3. 本周验收标准](#3-本周验收标准)
- [4. 失败模式和验证方法](#4-失败模式和验证方法)
- [5. 主动提示的问题](#5-主动提示的问题)
- [6. 悬挂问题](#6-悬挂问题)
- [7. Stage P 讲解与追问记录](#7-stage-p-讲解与追问记录)
  - [7.1 RGB-D 数据基础](#71-rgb-d-数据基础)
  - [7.2 坐标系、TF 和像素投影](#72-坐标系tf-和像素投影)
  - [7.3 相机摆放、遮挡和渲染坐标转换](#73-相机摆放遮挡和渲染坐标转换)
    - [GLFW 在相机渲染中的作用](#glfw-在相机渲染中的作用)
    - [`capture()` 的职责](#capture-的职责)
    - [`mujoco_bridge_node` 中的相机初始化参数](#mujoco_bridge_node-中的相机初始化参数)
  - [7.4 时间同步、消息契约和多相机边界](#74-时间同步消息契约和多相机边界)
  - [7.5 抓取判定：从 MuJoCo 接触到宽度和视觉](#75-抓取判定从-mujoco-接触到宽度和视觉)
  - [7.6 物体识别和形状先验](#76-物体识别和形状先验)
  - [7.7 性能、实测结果和验证结论](#77-性能实测结果和验证结论)
  - [7.8 尚未解决但必须保留的问题](#78-尚未解决但必须保留的问题)
- [8. Stage Q：几何分割和刚体位姿估计](#8-stage-q几何分割和刚体位姿估计)
  - [8.0 一句话总结](#80-一句话总结)
  - [8.1 改动清单与验证结果](#81-改动清单与验证结果)
  - [8.2 完整输入管线和模块关系](#82-完整输入管线和模块关系)
  - [8.3 为什么拆出独立感知包](#83-为什么拆出独立感知包)
  - [8.4 输入同步和消息契约](#84-输入同步和消息契约)
  - [8.5 几何处理链](#85-几何处理链)
  - [8.6 姿态、先验和输出语义](#86-姿态先验和输出语义)
  - [8.7 失败模式与验证手段](#87-失败模式与验证手段)
  - [8.8 你没问但值得注意的](#88-你没问但值得注意的)
  - [8.9 排查记录](#89-排查记录)
  - [8.10 本阶段边界与后续](#810-本阶段边界与后续)
  - [8.11 不引入深度模型时的泛化讨论](#811-不引入深度模型时的泛化讨论)
  - [8.12 流程鲁棒性：从初始检测到抓取过程](#812-流程鲁棒性从初始检测到抓取过程)
- [9. Stage R：oracle/vision 对照和任务接入](#9-stage-roraclevision-对照和任务接入)
  - [9.0 一句话总结](#90-一句话总结)
  - [9.1 改动清单与验证结果](#91-改动清单与验证结果)
  - [9.2 原始提问与任务边界](#92-原始提问与任务边界)
  - [9.3 两种 observation 如何共用任务执行链](#93-两种-observation-如何共用任务执行链)
  - [9.4 视觉质量门与失败语义](#94-视觉质量门与失败语义)
  - [9.5 权衡与替代方案](#95-权衡与替代方案)
  - [9.6 失败模式与验证手段](#96-失败模式与验证手段)
  - [9.7 排查记录](#97-排查记录)
  - [9.8 你没问但值得注意的](#98-你没问但值得注意的)
  - [9.9 本阶段边界与后续](#99-本阶段边界与后续)
  - [9.10 视觉抓放缺口与后续决策](#910-视觉抓放缺口与后续决策)
- [10. Week 4.5 交接](#10-week-45-交接)

## 1. 从前三周继承的事实和边界

| 事实/约束 | Week 4 的处理 |
| --- | --- |
| `world` 是任务几何和 TCP 目标的表达 frame | 视觉输出必须明确是 `world -> object` 的绝对位姿；camera-frame 结果只能作为中间值 |
| `~/ground_truth/object_pose` 是唯一 oracle 源 | 正式视觉输出使用不同 topic 或消息字段，不能覆盖 oracle |
| `BridgeObservation` 保证同一物理步的关节、物体和 TCP 观测 | 相机数据要么进入新的观测 bundle，要么通过明确的时间戳同步；不能让 executor 静默拼接不同时间的数据 |
| `EpisodeController` 管理 generation、sequence 和 stale observation | 视觉节点不能自行绕过 reset 生命周期；过期估计必须被拒绝或标记 |
| `CartesianWaypointSource` 输出任务几何，后端可以是离线 IK、MoveIt 或 learned policy | 本周只验证 vision -> Cartesian task geometry；不接入 MoveIt planner、学习策略或障碍规划 |
| 相机外参只能由一处发布 | 相机模型、安装位姿和外参来源同步写入 `docs/architecture.md` |

本周不训练深度模型，不引入 VLA/RL/IL 运行时依赖，不把 `enable_debug_viewer` 的 GPU 性能问题混入视觉正确性验收。策略和数据边界另见 Week 4.5。

## 2. 本周 Stage 计划

### 2.1 Stage P：RGB-D 相机和世界坐标观测

| 项 | 内容 |
| --- | --- |
| 改动 | 在 MuJoCo 场景加入固定 RGB-D 相机，发布标准 image、depth、CameraInfo 和唯一的相机 TF；记录相机内参、外参、分辨率、深度单位和发布频率 |
| 关键契约 | 深度像素反投影使用相机内参；相机外参通过 TF 查得；所有正式物体位姿最终表达在 `world` frame；消息带仿真时间戳 |
| 验证 | 用桌面平面、已知 box 中心和人工选取像素检查反投影；RViz2/离线脚本分别确认 camera frame 与 world frame 的方向 |
| 不做 | 不在相机节点内手写第二套外参，不把 ground truth 直接伪装成图像估计 |

相机观测和 `BridgeObservation` 的关系需要在实现时冻结：若图像和物理状态没有放入同一消息，就必须记录同步策略和允许的最大时间差。

### 2.2 Stage Q：几何分割和刚体位姿估计

| 项 | 内容 |
| --- | --- |
| 改动 | 完成 ROI/passthrough、桌面平面去除、目标聚类，以及已知对应 SVD 刚体配准；输出 pose、残差、inlier ratio、观测时间和拒绝原因 |
| 参考实现 | 先用 Eigen 自写已知对应 SVD 作为数学单测，再接工程化点云处理；ICP 或全局初始化属于后续增强，不把黑盒成功当作正确性证明 |
| 评测 | 改变物体位置、偏航、深度噪声、缺失点和遮挡，报告位置误差、姿态误差、失败率和 p50/p95 延迟 |
| 失败策略 | 错误初值、点数不足、残差过大、对称性不确定时输出低置信/拒绝，不生成可执行抓取目标 |

已知方形 box 的对称性必须显式定义姿态误差。若当前任务只关心位置和工具方向，应记录“等价姿态”规则，不能把不可观测的 yaw 误差误报为算法失败。

### 2.3 Stage R：oracle/vision 对照和任务接入

| 项 | 内容 |
| --- | --- |
| 改动 | 在相同的任务执行入口提供 `oracle` 和 `vision` 两种观测来源；下游一次 episode 只能选择一种来源；记录选择、置信度和失败层级 |
| 验证 | oracle 作为规划/控制上限，vision 作为感知闭环；固定和 held-out 场景都保留两者结果，不把 oracle 结果混进 vision 统计 |
| 任务行为 | vision 低置信时重新观察或显式失败；不继续使用上一轮未经标记的旧 pose |
| 边界 | 本周不要求 learned policy 成功，也不要求障碍环境；但输出字段必须能被 Week 4.5 的 policy observation adapter 消费 |

Stage R 的已完成验证只覆盖 vision 样本进入 `PREGRASP` 和拒绝后的停止语义，**没有完成 vision 模式的抓放**。后续 R1~R3 采用 [ADR 009](../../docs/adr/009-robot-aware-stateful-vision.md) 的机器人感知与时序跟踪决策；以下均为待实现的阶段出口，不回写为 Stage R 的实测结论。

### 2.4 Stage R1：同帧机器人几何掩膜

| 项 | 计划与出口 |
| --- | --- |
| 输入边界 | 沿用 Stage Q 的 RGB-D、CameraInfo、同时间戳 `BridgeObservation` 和现有 TF；只取其关节/夹爪状态及生命周期字段，机器人 link 几何来自与仿真模型核对过的描述。在线分割不读取 `BridgeObservation.object_pose` 或仿真接触位。 |
| 实现 | 在感知包中把机器人各 link、手指的可见几何投影到相机，生成预测深度和 robot mask；用观测深度与预测表面深度的容差判断机器人像素，保留更靠近相机的非机器人点。掩膜在桌面去除、聚类之前应用；缺同帧 link 变换或几何失配时显式拒绝。 |
| 可观测性 | 输出按 session/generation/sequence 关联的原始深度、预测机器人深度、mask、过滤后点云和候选簇调试 artifact；统计机器人残留、目标误删、mask 耗时及误差容差。 |
| 验证出口 | 固定点云/图像 fixture 覆盖空场景、机械臂靠近、夹爪与盒体相接、盒体位于机械臂前方、TF 缺失和模型偏差；实际运行时检查相机图像与 mask 叠加，不再把机械臂和盒体的连通簇直接作为目标。`colcon build/test` 与运行时 smoke 均通过后进入 R2。 |

### 2.5 Stage R2：目标关联与遮挡状态估计

| 项 | 计划与出口 |
| --- | --- |
| 实现 | 用已知盒体尺寸、上一份**实测**位姿、运动门限和候选簇质量做目标关联，替代“最大簇就是目标”。桌面支撑、夹持运动和释放后重现采用不同运动模型；离桌后禁用 `anchor_z_to_plane`。夹持时可由 TCP 相对变换产生有时限预测，但不得标成新视觉测量。 |
| 输出契约 | 在视觉消息中显式区分 `MEASURED`、`PREDICTED`、`OCCLUDED`、`REJECTED`，保留测量时间/序号、当前样本时间、预测年龄、置信度或不确定性、拒绝原因及 session/generation。`MEASURED` 必须有本帧物体点支持；预测只在有有效跟踪历史且未超过上限时可用；无可靠 pose 的状态不携带可执行抓取目标。 |
| 参数依据 | 预测最长年龄、关联门限和 mask 深度容差在 fixture/运行时回放上测量后冻结为可记录配置；不能仅凭一次固定场景成功设置阈值。 |
| 失败边界 | reset/bridge 重启清空轨迹；关联跳变、遮挡超时和模型不一致转入不可观测/拒绝，不能永久复用旧 pose，也不能把预测残差填成测量残差。完全遮挡时不宣称可以从单相机确认未滑落。 |
| 验证出口 | ROS-free 序列测试覆盖接近遮挡、局部重见、完整遮挡、真实掉落、释放后重见、错误关联和跨 generation 旧帧；断言预测年龄/不确定性增长、状态转换和无 oracle 输入。记录每阶段的实测/预测/不可观测帧数及 p50/p95 延迟。 |

### 2.6 Stage R3：任务证据门与完整抓放验收

| 项 | 计划与出口 |
| --- | --- |
| 接入 | 任务入口仍只有 `oracle`/`vision` 来源。vision 模式继续消费新鲜关节/TCP/夹爪状态，但按 R2 的状态和年龄决定运动或等待；短时 `PREDICTED` 只允许有界推进，超时、关联冲突或缺失测量进入明确失败。保持 `EpisodeController` 的 reset/generation/sequence 编排边界，调整依赖物体当前高度、滑落和最终落点的 FSM 证据门。 |
| 防止伪成功 | 当前 `BridgeObservation` 的物体真值和仿真接触位仅供离线评测，不作为 vision 模式的在线成功依据。预测位姿不能单独证明抬升、持物或落点；释放并移开机械臂后必须重新获得 `MEASURED` 的物体位姿，才能报告 vision 放置成功。无法重见则报告未能验证，不报成功。 |
| 集成验证 | `colcon build/test`、运行时 oracle 回归、vision 固定场景连续抓放和相同 seed 的 held-out 位姿对照；注入遮挡、掉落、错误目标、TF/图像缺帧及 reset，检查没有旧 pose 命令或假成功。固定场景目标为 20/20 vision 完整 episode 成功，held-out 至少 20 次报告成功率、误差与失败码；保留可重放的成功和失败 RGB-D/状态记录。 |
| 完成判据 | outcome 必须附带来源和最终实测证据；报告按 phase 划分的 `MEASURED/PREDICTED/OCCLUDED/REJECTED` 时长、最终落点误差、假成功次数和视觉端到端延迟。与 oracle 同任务对照后，再将观测契约交给 Week 4.5。 |

## 3. 本周验收标准

1. 相机内参、外参、frame、深度单位和时间戳语义写入 `docs/architecture.md`。
2. 已知几何经过 depth -> camera point -> world point 的坐标链后，位置误差和方向误差在预设容差内。
3. 已知对应 SVD 有独立单测，错误对应关系和退化点集能触发失败或低置信结果。
4. held-out 物体位姿、深度噪声和局部遮挡下报告位置误差、姿态误差、失败率和 p50/p95 延迟。
5. oracle/vision 使用同一任务执行入口，切换不会修改 FSM 或 Cartesian task geometry。
6. vision 低置信估计不能发布抓取目标；任务层能重新观察或以结构化失败码结束。
7. 保留一条至少包含成功和失败样本的 rosbag，可离线复现点云和位姿估计结果。

## 4. 失败模式和验证方法

| 失败模式 | 可能表现 | 验证方法 |
| --- | --- | --- |
| 外参方向或父子 frame 错误 | 点云在 RViz 中镜像、上下颠倒或整体偏移 | 用桌面法向、已知 box 中心和 TF 树三者交叉验证 |
| 深度单位/截断范围错误 | 物体尺度或位置按固定倍数偏差 | 检查单像素原始值、反投影点和模型边长 |
| 图像与关节状态不同步 | 估计 pose 看似合理但运动时系统性滞后 | 扰动物体/相机，比较消息时间戳和估计误差随速度的变化 |
| 桌面分割失败 | 目标点云混入桌面或目标被全部去除 | 保留中间点云并统计平面内点比例 |
| ICP/SVD 局部或对应关系失败 | 残差低但 pose 错，或迭代不收敛 | 加入错误初值、错误对应和对称物体反例 |
| 低置信结果继续下游 | 机械臂进入错误抓取位姿 | 注入残差/inlier 阈值失败，断言没有目标命令发布 |

## 5. 主动提示的问题

1. **可观测性**：如何同时记录原始 RGB-D、过滤后的点云、估计 pose、oracle pose 和最终落点，使一次视觉失败可以定位到传感器、分割、配准还是执行？
2. **可测试性**：视觉节点是否能脱离 MuJoCo，使用固定图像/点云 fixture 重放并得到稳定结果？
3. **时间语义**：相机帧和 `BridgeObservation` 的最大允许时间差是多少？这个值应由实验测量，不应只写成实现细节。
4. **姿态定义**：当前方形 box 的对称性和抓取工具 yaw 是否足以支撑后续 policy observation？如果不够，应在 Week 4.5 前冻结等价姿态规则。

## 6. 悬挂问题

- 相机安装位姿和标定方式尚未确定，解锁条件是 Stage P 的实际相机配置。
- 当前 `BridgeObservation` 没有图像字段。需要在 Stage P 决定使用原子 observation bundle 还是时间戳同步的标准话题集合，不能让 Week 4.5 再猜测。
- 点云和图像的 rosbag 体积、压缩方式和是否进入 Git 尚未决定；大文件只能进入外部 artifact 存储。
- 本周不决定是否使用 ICP、FoundationPose 或开放词汇分割；先完成可解释的几何基线。

## 7. Stage P 讲解与追问记录

本节把两轮讲解按主题整理。原始问题包括：RGB-D 基础和相机摆放、单相机与多相机、oracle 接触与宽度判定、物体形状鲁棒性，以及 ROS optical frame 和两条相机 TF 是否属于先验。

原始问题索引：

- RGB-D 相机的摆放位置、深度图格式、处理方式和常用处理包是什么？
- 当前只有一个相机吗？代码能否扩展到多个相机？
- 接入相机后，能否从 MuJoCo 接触 oracle 改成通过宽度判定抓取？
- 当前物体识别是否对形状鲁棒？有哪些形状先验？
- 什么是 ROS optical frame？`T_world_camera_link` 和 `T_camera_link_optical` 是先验吗？

### 7.1 RGB-D 数据基础

RGB-D 相机同时提供彩色图和每像素深度。常见深度编码是 `16UC1`（通常单位为毫米）和 `32FC1`（通常单位为米），但单位必须以驱动约定和 `CameraInfo` 为准。还要区分 optical frame 的 z-depth 与相机到点的欧氏距离。当前 bridge 发布 `32FC1`、米单位、沿光轴的 z-depth，无效值为 NaN。

典型处理链是：

```text
检查时间戳和 frame_id
→ 过滤无效值、统一单位
→ ROI 和滤波
→ 根据 CameraInfo 反投影为点云
→ 通过 TF 变换到 world
→ 去桌面、聚类和位姿估计
```

常用 ROS 组件包括 `cv_bridge`、`image_transport`、`image_proc`、`depth_image_proc`、`tf2_ros`、`message_filters`、OpenCV 和 PCL。当前项目还没有接入真实相机、PCL、检测器或位姿估计器；Stage P 只完成了仿真 RGB-D 输出和坐标链验证。

### 7.2 坐标系、TF 和像素投影

统一记号 `^A T_B` 表示把 B frame 中的点变换到 A frame。当前 TF 树为：

```text
world → camera_link → camera_optical_frame
```

ROS optical frame 的约定是：`+X` 向图像右方、`+Y` 向图像下方、`+Z` 沿光轴指向场景。`camera_link` 只是安装 frame，轴方向由项目或设备定义，不自动等于 optical frame。

一个深度像素先通过内参落在 optical frame：

```text
x = (u - cx) * depth / fx
y = (v - cy) * depth / fy
z = depth
```

再通过两条 TF 进入世界坐标：

```text
p_world = T_world_camera_link
        * T_camera_link_optical
        * p_optical
```

[场景文件](../../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml:25)给出相机的固定位置 `(0.5,-0.45,1.0)m` 和姿态。它形成 `^world T_camera_link`，是仿真中的已知静态外参；真机中应由安装测量或标定得到。bridge 再在 `camera_link` 下发布绕 X 轴 180°、零平移的 `^camera_link T_camera_optical`，把 MuJoCo 的 `+Y` 向上、`-Z` 向前转换成 ROS optical 的 `+Y` 向下、`+Z` 向前。[bridge](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp:436)负责发布这两条 TF，感知节点只查询。

因此：

- `CameraInfo` 内参回答“像素怎样变成相机点”；
- `^world T_camera_link` 回答“相机装在世界哪里”；
- `^camera_link T_camera_optical` 回答“安装 frame 怎样转换为图像 frame”。

这里的“运行时集成检查脚本”指
[`camera_probe.py`](../../src/mujoco_bridge/test/camera_probe.py:15)，不是新的传感器，也不是
算法模块。它作为一个 ROS 2 Python 节点运行，订阅 RGB、depth、两份 `CameraInfo`、静态 TF、
`BridgeObservation` 和 MuJoCo oracle；然后按**完全相同的时间戳**把这些消息配成一帧。
脚本还检查编码、尺寸、`frame_id`、session/generation/sequence，并用 `CameraInfo` 反投影
中心像素和桌面像素，再应用 `camera_optical_frame → camera_link → world` 的 TF。只有这些
检查都通过才打印 `PASS`；15 秒内没有匹配样本则失败。它是验证“ROS 消息、TF、深度单位和
仿真几何是否接通”的运行时 smoke test，不是单元测试，也不代表真实 RGB-D 硬件已经标定。

脚本的核心检查位于 [camera_probe.py:106-156](../../src/mujoco_bridge/test/camera_probe.py:106)：
中心像素反投影到 world 约为 `(0.5017,-0.0084,0.2605)m`；桌面参照点的 `z≈0.2206m`。
两个不同高度、不同像素的参照比只检查一个点更容易发现上下轴翻转或深度方向错误。这项
验证只证明坐标链和消息契约在当前仿真中接通，不等于已经完成物体位姿估计。

### 7.3 相机摆放、遮挡和渲染坐标转换

相机位置要在视野、工作距离、遮挡、入射角和分辨率之间折中。最初的斜视位置和随后尝试的正上方位置，在 reset 回 home 后都会让机械臂持续挡住中心视线；运行时集成检查脚本反投影出的点落在机械臂表面，而不是 box，这不是 TF 算错，而是传感器确实被遮挡。最终采用桌面负 Y 侧上方的 `(0.5,-0.45,1.0)m` 位姿，reset 前后都能看到 box；执行抓取时仍可能动态遮挡。

MuJoCo 的 `mjr_readPixels()` 使用 OpenGL 深度缓冲和左下角原点的像素数组。[相机实现](../../src/mujoco_bridge/src/rgbd_camera.cpp:67)先翻转行序，使其符合 ROS 图像的左上角原点，再用近、远裁剪面把缓冲值 `d` 转成米：

```text
depth_m = near * far / (far - d * (far - near))
```

把 `d` 直接当米会造成系统性的尺度错误；忘记翻行会让图像上下位置与深度错配。这里的上下翻转是图像存储坐标的适配，不是 `camera_link` 到 optical frame 的 TF 旋转；前者处理像素数组，后者处理三维坐标轴。远裁剪面和无效深度写为 NaN，后续点云处理必须跳过。

#### GLFW 在相机渲染中的作用

GLFW 在这里不是视觉算法，也不是负责画 MuJoCo 场景的高层模块；它负责创建一个隐藏的 OpenGL context。MuJoCo 的 `mjr_render()`/`mjr_readPixels()` 需要当前 OpenGL context，即使没有可见窗口也一样，所以 `RgbdCamera` 用隐藏 GLFW window 提供 offscreen framebuffer。debug viewer 也使用 GLFW，但那是另一条可见窗口路径；相机路径不会打开用户界面。

#### `capture()` 的职责

`capture()` 是“从当前 MuJoCo 状态得到一帧 CPU 图像”的底层函数。它的声明在
[rgbd_camera.hpp:41](../../src/mujoco_bridge/include/mujoco_bridge/rgbd_camera.hpp:41)，输入是
`mjData * data`，输出不是返回值，而是对象内部的 `rgb_` 和 `depth_` 两个 buffer。
它不创建 ROS 消息，也不发布话题；ROS 适配由
[publishCamera():971](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp:971) 完成。

实际代码位于 [rgbd_camera.cpp:96-117](../../src/mujoco_bridge/src/rgbd_camera.cpp:96)：

```cpp
void RgbdCamera::capture(mjData * data)
{
  glfwMakeContextCurrent(window_);
  api_.setBuffer(mjFB_OFFSCREEN, &context_);
  const mjrRect viewport{0, 0, width_, height_};
  api_.updateScene(model_, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
  api_.render(viewport, &scene_, &context_);
  api_.readPixels(raw_rgb_.data(), raw_depth_.data(), viewport, &context_);
  ...
}
```

这几行分别对应以下操作：

1. `glfwMakeContextCurrent(window_)`（第 98 行）把 RGB-D 相机的 OpenGL context
   切回当前线程。debug viewer 可能在同一进程中使用另一个 context；不切换的话，后续
   MuJoCo 渲染调用可能操作错误的 framebuffer。
2. `setBuffer(mjFB_OFFSCREEN, &context_)`（第 99 行）选择 MuJoCo 创建的离屏 framebuffer，
   而不是可见 debug window 的 framebuffer。`viewport`（第 100 行）规定本次读写的矩形，
   原点为 framebuffer 的左下角，尺寸是初始化时传入的 `width_ × height_`。
3. `updateScene(..., data, ..., &camera_, ...)`（第 101 行）把这次物理步的 `mjData`
   转成渲染场景。`camera_` 在构造函数中被设为 `mjCAMERA_FIXED` 和 MJCF camera ID，
   所以这里不会使用用户拖动的自由视角；`mjCAT_ALL` 表示渲染所有几何类别。
4. `render`（第 102 行）把 `mjvScene` 光栅化到离屏 framebuffer；此时还没有 ROS 图像，
   只有 GPU/图形上下文中的颜色和深度结果。
5. `readPixels`（第 103 行）把结果读回 `raw_rgb_` 和 `raw_depth_`。构造函数在
   [rgbd_camera.cpp:29-31](../../src/mujoco_bridge/src/rgbd_camera.cpp:29) 按像素分配这些
   临时数组：RGB 是每像素 3 个 `uint8_t`，深度是每像素一个 `float`。此时深度仍是
   OpenGL/MuJoCo 的归一化深度 buffer 值，不是米。

读回后，第 105-106 行用模型的裁剪面得到实际距离：

```cpp
const double near_m = model_->vis.map.znear * model_->stat.extent;
const double far_m = model_->vis.map.zfar * model_->stat.extent;
```

`znear`、`zfar` 是相对于模型尺度的裁剪参数，`stat.extent` 把它们换成米。随后
[camera_geometry.hpp:30-38](../../src/mujoco_bridge/include/mujoco_bridge/camera_geometry.hpp:30)
中的 `metricDepth()` 使用

```text
z = near × far / (far - d × (far - near))
```

把归一化值 `d` 转成沿 optical `+Z` 的 z-depth；当 `d` 是 NaN、超出 `[0,1)` 或裁剪面
非法时，函数返回 NaN，避免把背景或无效值伪装成真实距离。

最后的双重循环位于 [rgbd_camera.cpp:107-116](../../src/mujoco_bridge/src/rgbd_camera.cpp:107)：

```cpp
const int src = (height_ - 1 - y) * width_ + x;
const int dst = y * width_ + x;
for (int channel = 0; channel < 3; ++channel) {
  rgb_[3 * dst + channel] = raw_rgb_[3 * src + channel];
}
depth_[dst] = static_cast<float>(metricDepth(raw_depth_[src], near_m, far_m));
```

`src` 和 `dst` 不同，是因为 OpenGL 读回数组以左下角为原点，而 ROS 图像约定以左上角
为第一行；RGB 和 depth 必须用同一个行映射，否则颜色与深度会错位。转换后，`rgb_`
是 ROS 行序的交错 RGB 字节，`depth_` 是 ROS 行序、米单位的 optical z-depth。

随后 [publishCamera():976-1007](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp:976) 才把
两个 buffer 包装成消息：RGB 使用 `rgb8`，深度使用 `32FC1`，并给 RGB、depth 和两份
`CameraInfo` 写入同一个 `simTime()`。因此 `capture()` 负责“渲染、读回、坐标行序和深度
单位”，`publishCamera()` 负责“消息编码、frame_id、时间戳和发布”。

#### `mujoco_bridge_node` 中的相机初始化参数

这里的“初始化参数”分为三类：从 MJCF 读取的模型事实、由模型事实推导的内参，以及
为了让 ROS 消费端能够工作而作出的仿真假设。完整代码在
[mujoco_bridge_node.cpp:194-238](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp:194)。

**1. 是否启用、多久采一帧。** `enable_rgbd_camera`（第 194 行）默认 `false`，因此普通
   bridge 启动时不会创建 GLFW context、相机渲染器或四个图像 publisher。只有显式打开后，
   才执行下面的初始化。`camera_decimation_` 在第 175 行由参数 `camera_rate_hz` 计算，
   默认值是 `10.0` Hz；它不是相机曝光频率，而是仿真中每隔多少个物理步调用一次
   `capture()`。第 195-199 行要求它是 `tf_decimation_` 的整数倍，保证每个相机帧都能和
   一个 episode observation 对齐，而不是落在两个状态样本之间。

**2. 相机和安装 body 的身份。** `kCameraName = "workcell_rgbd"`、
   `kCameraBodyName = "camera_link"` 和 `kOpticalFrameName = "camera_optical_frame"`
   定义在 [mujoco_bridge_node.cpp:104-112](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp:104)。
   第 200-203 行通过 `name2id` 把前两个字符串解析成 MuJoCo 的 camera/body ID，并检查
   `model_->cam_bodyid[camera_id] == camera_body_id`。这个检查的意义是防止 MJCF 改名或换了
   挂载 body 后，程序仍然渲染一台相机却把 TF 发布到另一处。它们是模型和 TF 的命名契约，
   不是焦距或畸变标定值。

**3. `width`、`height` 和 framebuffer。** 第 208-210 行读取
   `model_->vis.global.offwidth/offheight`，当前场景文件的值是
   [pick_place_scene.xml:16](../../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml:16)
   的 `320 × 240`。这两个值同时用于 `RgbdCamera` 的像素 buffer、MuJoCo viewport 和 ROS
   `Image.width/height`；这样渲染尺寸、读回尺寸和消息尺寸一致。它们不是物理相机的“分辨率
   标定”，而是当前仿真 offscreen framebuffer 的尺寸；改 MJCF 的 framebuffer 后，ROS 图像
   尺寸也会随之变化。

**4. `fovy_rad` 和 `focal`。** MJCF 中 [pick_place_scene.xml:28-29](../../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml:28)
   的 `fovy="50"` 是垂直视场角，单位是度。第 211 行乘 `mjPI / 180` 转成弧度，因为
   `std::tan` 使用弧度。第 212 行使用针孔模型：

   ```cpp
   const double focal = height / (2.0 * std::tan(fovy_rad / 2.0));
   ```

   这里 `focal` 是像素单位的焦距，实际作为 `fy`；代码假设方形像素且没有额外的像素
     长宽比，所以令 `fx = fy = focal`。因此它不是从真实镜头标定文件读来的值，而是由
   仿真视场角和图像高度推导出的理想内参。

**5. `CameraInfo` 每个字段的定义和当前值。** 第 213-224 行把上述模型转换成 ROS
   `sensor_msgs/CameraInfo`：

   ```cpp
   camera_info_.width = width;
   camera_info_.height = height;
   camera_info_.distortion_model = "plumb_bob";
   camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
   camera_info_.k = {focal, 0.0, (width - 1) / 2.0,
     0.0, focal, (height - 1) / 2.0, 0.0, 0.0, 1.0};
   camera_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
   camera_info_.p = {focal, 0.0, (width - 1) / 2.0, 0.0,
     0.0, focal, (height - 1) / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
   camera_info_.header.frame_id = kOpticalFrameName;
   ```

   - `width`、`height`：这份 CameraInfo 所对应的图像尺寸，必须和 Image 消息一致。
   - `distortion_model`：选择 ROS 常见的针孔加径向/切向畸变模型名称；它描述 `D` 如何
       被解释，不代表仿真真的存在畸变。
   - `D`：五个畸变系数 `(k1, k2, t1, t2, k3)`。MuJoCo 当前使用理想投影，故全为零；
       真机不能默认沿用零值。
   - `K`：原始相机内参矩阵 `[[fx,0,cx],[0,fy,cy],[0,0,1]]`。`cx=(width-1)/2`
       和 `cy=(height-1)/2` 把主点放在图像中心；它们是像素坐标，不是米。
   - `R`：校正图像相对于原始图像的旋转矩阵。当前没有立体校正或额外旋转，所以是单位阵。
   - `P`：校正后的 3×4 投影矩阵。这里是单目相机，基线为零，因此它等于 K 的前三列
       加上最后一列零；立体 RGB-D 设备通常不能这样填写深度相机的 P。
   - `frame_id`：告诉消费者这些内参对应哪个坐标系。这里使用 optical frame，因为
       `backproject()` 的公式把 `z=depth` 定义在该 frame 的 `+Z` 光轴上。

**6. publisher 参数。** 第 225-234 行建立四个 publisher：RGB 图像、depth 图像、color
   CameraInfo 和 depth CameraInfo。话题名分别是 `~/camera/color/image_raw`、
   `~/camera/depth/image_raw`、`~/camera/color/camera_info` 和
   `~/camera/depth/camera_info`，QoS 使用 `SensorDataQoS` 以适合高频、可丢旧帧的传感器流。
   这也是为什么 `capture()` 不应该直接发布：渲染器只负责填充 buffer，而 node 负责 ROS
   编码和时间戳契约。

因此，代码中的 `0.0`、`1.0`、图像中心公式和 `50°` 并不是“随便写的 hardcode”：它们分别
表示零畸变、齐次相机矩阵的固定元素、理想主点和 MJCF 视场角。真正依赖具体场景的事实是
相机名字、挂载 body、offscreen 分辨率和 `fovy`；真正的仿真假设是方形像素、零畸变、单目
投影模型。真实 RGB-D 设备如果有独立 color/depth 内参、畸变或两者之间的外参，必须用驱动
提供的两份 CameraInfo 和标定 TF 替换这套理想模型。

### 7.4 时间同步、消息契约和多相机边界

时间戳匹配分为发送端和消费端两步。发送端的
[`simTime()`](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp:621)不使用墙钟，而是把
`mjData::time` 转成 ROS 时间：

```cpp
return rclcpp::Time(std::llround(data_->time * 1e9), RCL_ROS_TIME);
```

`llround` 把仿真秒数一次性量化为纳秒，避免浮点累加后直接截断造成一纳秒级的错位。
在 [onTimer():799-820](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp:799) 中，回调先
执行一次 `mj_step`，然后在同一份 `mjData` 上发布 observation 和相机。相机发布函数在
[mujoco_bridge_node.cpp:976-1007](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp:976)
只调用一次 `simTime()`，把得到的 `stamp` 复制给 RGB、depth 和两份 CameraInfo：

```cpp
camera_->capture(data_);
const auto stamp = simTime();
rgb.header.stamp = stamp;
depth.header = rgb.header;
camera_info_.header.stamp = stamp;
```

`publishEpisodeObservation()` 同样在 [mujoco_bridge_node.cpp:1022-1034](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp:1022)
使用该物理步的 `simTime()`；`BridgeObservation` 中的时间键来自它内部的
`joint_state.header.stamp`。相机每 50 个物理步采样一次，observation 每 5 步发布一次；
启动时检查相机 decimation 必须是 observation decimation 的整数倍。因此相机帧所在的
物理步一定有一个同时间戳 observation。

消费端 `camera_probe.py` 不依赖 DDS 到达顺序。每个订阅回调都把消息按
`(stamp.sec, stamp.nanosec)` 放入字典，例如 [camera_probe.py:70-103](../../src/mujoco_bridge/test/camera_probe.py:70)：

```python
self.rgb[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
self.depth[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
stamp = msg.joint_state.header.stamp
self.observations[(stamp.sec, stamp.nanosec)] = msg
```

每次收到任意一类消息，`check()` 都求六个缓存键集合的交集
（[camera_probe.py:106-115](../../src/mujoco_bridge/test/camera_probe.py:106)）：

```python
matches = (self.depth.keys() & self.rgb.keys() & self.info.keys() &
           self.depth_info.keys() & self.observations.keys() & self.oracles.keys())
if matches:
    stamp = max(matches)
```

因此只有 RGB、depth、两份 CameraInfo、`BridgeObservation` 和 oracle 的
`sec/nanosec` **完全相等**时才会配成一帧；`max(matches)` 只是从已经匹配成功的样本中
选最新的一帧，不是按到达时间选最新消息。随后脚本还检查 RGB/depth 的编码、尺寸和
`camera_optical_frame`，并从 observation 读取 `bridge_session`、`generation` 和
`sample_sequence`，所以时间戳相等只是第一道门，不能替代生命周期检查。

缓存大小也有明确限制：RGB、depth 和 CameraInfo 保留最近 5 个键，observation/oracle
保留最近 30 个键；15 秒内没有完整交集就报告 timeout。静态 TF 不参与这组时间戳交集，
脚本只确认 `camera_link` 和 `camera_optical_frame` 已收到，并按 child frame 保存变换。

采用标准图像话题便于 RViz 和现有 ROS 工具消费，但 DDS 不保证跨话题到达顺序，也可能丢帧。消费者必须按时间戳精确匹配，不能从各话题分别取“最新一条”。图像本身没有 generation；匹配到 `BridgeObservation` 后才能取得 session、generation 和 sequence，并拒绝 reset 前的旧样本。运行时集成检查脚本在 generation 0 和 reset 后的 generation 1 都配对成功。延迟、丢帧时的等待上限和失败码仍属于 Stage Q 消费端工作。

当前只有一个 `workcell_rgbd`、一组 RGB/depth/CameraInfo publisher 和一组相机 TF。底层可以扩展到多相机，但每台相机都要有独立的 MJCF camera、frame、topic、内参、渲染缓冲区和频率，还要处理多相机外参、同步、重叠视野、遮挡和融合置信度。当前代码不能声称已经支持任意多相机。

### 7.5 抓取判定：从 MuJoCo 接触到宽度和视觉

当前接触来自 MuJoCo `mjData::contact`。`classifyGrasp()` 综合：

- 夹爪宽度；
- 左右手指是否接触；
- 物体是否达到抬升高度；
- 物体是否仍靠近 TCP。

相机可以替换物体位姿来源，但不能直接提供接触力；宽度通常应读取夹爪关节状态。建议逐步迁移为：

```text
宽度误差合理
+ 抬升时物体跟随 TCP
+ 物体与 TCP 的相对位置稳定
+ 没有超时或滑落
```

仅凭宽度会把空夹爪、单侧夹持、被桌面支撑和滑落误判为成功。因此应先并行记录 width-only 和 oracle 结果，再加入视觉抬升检查，最后才在 vision/真机模式中移除真值接触依赖。

### 7.6 物体识别和形状先验

Stage P 没有物体识别，只验证固定 box 的深度反投影和坐标链。当前先验是：单个目标、固定 body 名 `box`、4 cm 立方体、已知桌面、已知相机参数和可用 MuJoCo oracle。因此当前不能称为对任意形状鲁棒。

立方体的 yaw 存在 90° 对称性。如果任务只关心抓取中心，应定义等价姿态；如果需要完整 6D pose，则必须显式处理对称性。Stage Q 再实现桌面去除、目标聚类和已知对应 SVD 配准，并输出 pose、残差、inlier ratio 和拒绝原因。

Stage Q 将在独立章节记录几何感知包、输入同步、成熟视觉组件和位姿质量输出，不把这些内容混入 Stage P 的相机适配记录。

### 7.7 性能、实测结果和验证结论

当前默认相机频率是 **10 Hz 仿真时间**，运行日志的 RTF 约为 **0.5**，因此墙钟时间不能期待稳定收到 10 帧/秒。渲染、像素读回和消息发布与物理步进同一线程执行，慢渲染会拖慢整个仿真；目前没有分别测量三者耗时，不能把 RTF 下降精确归因到某一个环节。Stage Q 测量感知延迟时应同时记录仿真时间、墙钟时间和帧序号。

已完成验证：

- `camera_probe.py` 这个运行时集成检查脚本验证 RGB、depth、两份 CameraInfo、TF、oracle pose 和 `BridgeObservation` 时间戳完全匹配；
- reset 后 generation 从 0 变为 1，配对检查仍通过；
- box 顶面反投影高度误差约 `0.6mm`、XY 误差约 `8.6mm`；
- 桌面反投影 `z≈0.2206m`，与场景顶面 `0.22m` 一致。

`camera_probe.py` 在 `LIBGL_ALWAYS_SOFTWARE=1` 下重新通过：中心深度 `0.8613m`，box 顶面误差 `0.6mm`，桌面 `z=0.2206m`。未设置软件渲染时，当前 shell 的 GLFW context 创建会卡在图形环境中；这属于运行环境问题，排查时应先确认 DISPLAY/OpenGL，再判断相机代码。该检查只验证 Stage P 的仿真相机消息和坐标链，不验证 Stage Q 的点云和姿态算法。

主要 Stage P 失败模式是外参父子方向错误导致镜像、翻转或整体偏移，以及把 OpenGL 原始深度当米导致尺度错误。时间同步、桌面分割、聚类和位姿质量属于 Stage Q 的验证范围。

### 7.8 尚未解决但必须保留的问题

- 仿真零畸变、无噪声不代表真机；真机需要重新标定并使用驱动内参。
- Stage P 只验证仿真相机消息和坐标链，不证明真实相机标定、噪声或运动中同步。
- Stage P 只支持当前单相机配置；多相机外参、重叠视野和融合策略留到后续。
- Stage Q 的视觉输出质量、oracle/vision 接口隔离、遮挡时拒绝旧 pose，以及脱离 MuJoCo 重放点云和配准，见下一章。

## 8. Stage Q：几何分割和刚体位姿估计

### 8.0 一句话总结

Stage Q 把 Stage P 发布的标准 RGB-D 输入转换成 world-frame 的目标物体位姿。它不把点云算法继续堆进 `mujoco_bridge_node`，而是在独立的 `mujoco_perception` 包中组合 `image_geometry` 和 PCL，并把时间戳、generation、质量指标和拒绝原因一起传给任务层。

### 8.1 改动清单与验证结果

本阶段的代码和接口改动如下：

- 新增 `mujoco_perception` 包，以及 `geometry_pipeline` 几何库和 `object_pose_estimator_node`；
- 使用 `image_geometry::PinholeCameraModel`、PCL `PassThrough`、`SACSegmentation`、`EuclideanClusterExtraction`、`TransformationEstimationSVD` 和 `MomentOfInertiaEstimation`；
- 新增 `manipulation_interfaces/msg/VisionObjectPose.msg`，包含 pose、accepted、confidence、residual、inlier ratio、point count、generation 和 rejection reason；
- 估计器按完全相等的仿真时间戳匹配 RGB、depth、两份 CameraInfo 和 `BridgeObservation`；
- bridge 继续负责 MuJoCo 渲染、ROS 图像封装和 TF，不负责点云或姿态算法。

包级测试覆盖 PCL SVD 正确变换、点数不匹配、共线退化、桌面去除/聚类、完整立方体 OBB、只有顶面可见和错误尺寸拒绝，共 7 个几何测试。`mujoco_perception` 的构建、gtest、copyright、cppcheck、cpplint、uncrustify 和 xmllint 全部通过。

全工作区回归结果为 6 packages、428 tests、0 errors、0 failures、64 skipped。真实 bridge + estimator 运行收到 `accepted=true`、180 个目标点、confidence 约 `0.8239`、residual 约 `0.000676m`，位置约 `(0.5000, 0.0002, 0.2400)m`。运行结束后 `/clock` 只有一个 publisher，且没有遗留 bridge 或 estimator 进程。

### 8.2 完整输入管线和模块关系

本节记录这次讲解的高层模型。Stage Q 不是把一个“大视觉函数”塞进 bridge，而是把仿真传感器、输入同步、几何处理和任务输出分成四层：

```text
MuJoCo mjData
  → mujoco_bridge：渲染 RGB-D、发布 CameraInfo/TF/BridgeObservation
  → object_pose_estimator：按仿真时间戳组成完整输入帧并校验契约
  → geometry_pipeline：反投影、world 变换、ROI（Region of Interest，感兴趣区域）、桌面去除、聚类、OBB（Oriented Bounding Box，有向包围盒）
  → VisionObjectPose：pose、质量指标、generation 和结构化拒绝原因
```

### 8.3 为什么拆出独立感知包

`mujoco_bridge` 仍拥有 MuJoCo 的 `mjData`。它的 `RgbdCamera` 是仿真专属适配层，负责从 MuJoCo 的离屏渲染缓冲区产生标准 ROS 图像；这部分不能直接由通用 ROS 视觉包替代。bridge 不负责点云聚类或物体姿态估计，也不订阅感知结果。

如果继续把 Stage Q 写进 bridge，一个节点会同时承担物理步进、渲染、状态发布、reset、点云分割和姿态估计。这样视觉计算会直接占用物理线程，算法单测也会被 MuJoCo 和 OpenGL 环境绑住。拆包后的代价是多一个节点、需要处理跨 topic 同步、并引入 PCL 版本依赖；收益是几何算法可以脱离 MuJoCo 做固定 fixture 测试，也可以以后替换为真实 RGB-D 驱动。

`depth_image_proc` 和 `pcl_ros` 是常见 ROS 运行时组件，但当前容器没有安装它们。因此本阶段使用已安装的 `image_geometry` 和 PCL C++ API。自有代码只负责参数、输入契约、TF/时间戳、生命周期字段、置信度和拒绝码，不重新实现通用点云算法。

### 8.4 输入同步和消息契约

`ObjectPoseEstimatorNode` 订阅 RGB、depth、两份 CameraInfo 和 `BridgeObservation`。每个回调只把消息按时间戳放入缓存；`tryProcess()` 只有在五类消息的时间戳完全相等时才处理。它还查询 bridge 发布的静态 `world <- camera_optical_frame` TF，并检查编码、尺寸、行步长、内参和 frame 一致性。RGB 当前主要用于同步和契约检查，几何信息来自 depth。

发送端在同一次 `mj_step` 后为 RGB、depth、两份 CameraInfo 和 `BridgeObservation` 使用相同的仿真时间戳。消费端不能按“每个 topic 最新一条”拼接，因为 DDS 到达顺序和丢帧行为不提供这个保证。

估计器用时间戳键保存五类输入。任何一类消息到达时都会尝试组帧；只有完整交集才处理。随后它还校验 RGB 为 `rgb8`、depth 为 `32FC1` 米单位、尺寸和数据长度匹配、CameraInfo 内参有效、image 与 CameraInfo 的 frame 一致，并查询 `world <- camera_optical_frame`。

时间戳匹配只回答“这些消息是不是同一仿真时刻”，generation 和 session 才回答“这是不是当前 bridge/reset 生命周期”。因此输出继续携带 `bridge_session`、`generation` 和 `sample_sequence`，不能因为时间戳相等就跳过生命周期检查。

### 8.5 几何处理链

`segmentDepth()` 是点云预处理入口。它使用 `image_geometry::PinholeCameraModel` 把 z-depth 反投影为 optical 点云，再使用 PCL 完成 world 变换、ROI 过滤、支持平面检测、桌面去除和欧氏聚类。它返回目标点簇和分割阶段的拒绝原因，而不是直接发布 ROS 消息。

处理顺序是：

```text
32FC1 z-depth
  → PinholeCameraModel 反投影到 optical 点云
  → TF/矩阵变换到 world
  → PCL PassThrough 做 world ROI（Region of Interest，感兴趣区域）
  → PCL SACSegmentation 检测水平桌面
  → 去除桌面点
  → PCL EuclideanClusterExtraction 聚类
  → 选择目标点簇
```

其中 `PinholeCameraModel` 负责内参和像素投影关系，项目代码不再手写一套相机投影公式。PCL `PassThrough` 负责轴向范围过滤，`SACSegmentation` 负责支持平面，`EuclideanClusterExtraction` 负责空间连通的目标候选。

**world ROI：动机、意义和结果。** 相机看到的不只有目标，还可能包含桌面外区域、机器人结构、场景边界和远处噪声。world ROI 用已知工作区的 `x/y/z` 范围先筛掉这些不可能成为目标的点，避免后面的平面检测和聚类处理整张视野。它的意义是把“目标应该出现在哪里”作为一个明确的场景先验，同时减少计算量和误检。当前结果是只保留 ROI 内点，再交给桌面去除和聚类；代价是物体移出 ROI 时会被当作没有目标，这不是通用检测器。

**检测并去除水平桌面：动机、意义和结果。** 桌面通常是点数最多、面积最大的平面。如果不先去掉它，桌面点会压过盒体点，或者和盒体底部连成一个错误的大簇。`SACSegmentation` 使用“接近 world Z 轴的水平平面”模型，在 ROI 内寻找支持面；检测到的平面内点被删除，只保留 foreground。这样后续聚类面对的是物体、机器人残留和噪声，而不是整张桌面。对于当前规则且很小的仿真 fixture，如果 RANSAC 没有稳定内点，则使用已知桌面高度附近的 PCL `PassThrough` 作为退化路径；结果仍是 foreground 点云，但这条路径依赖当前场景先验。

**聚类并选择目标点簇：动机、意义和结果。** 去掉桌面后，foreground 仍可能包含机械臂、支架或多个物体。Euclidean clustering 按点之间的空间距离把它们分成若干连通簇，避免把彼此分离的物体合成一个姿态。当前任务先验只有一个目标，因此选择最大的合法簇作为目标点簇。结果是 `SegmentationResult::target_cluster`，后续 OBB 只对这组点计算。这个选择不能解决多物体身份识别：多个物体都在 ROI 内时，最大的簇不一定就是任务目标，后续需要模型匹配、颜色/几何评分或显式目标 ID。

规则且很小的仿真 fixture 可能让 RANSAC 平面模型缺少足够统计特征。因此当平面分割没有内点时，代码使用已知桌面高度附近的 PCL `PassThrough` 作为确定性退化路径；这不是把桌面高度伪装成通用视觉算法，而是当前仿真任务先验的明确使用。

### 8.6 姿态、先验和输出语义

`estimateBoxPose()` 消费目标点簇，使用 PCL OBB 和已知的 4 cm 立方体先验计算位置、残差、inlier ratio 和 confidence。俯视相机通常只能看到顶面，所以隐藏的 z 厚度由桌面高度和模型先验补齐，立方体 yaw 只能作为对称不确定的规范姿态。`estimateKnownCorrespondences()` 是已测试的 PCL SVD 刚体配准入口，目前作为可复用几何能力保留，默认运行路径主要使用 OBB。

`estimateBoxPose()` 会比较可见主尺寸，计算点到盒体表面的 residual、inlier ratio 和 confidence，并把错误尺寸、点数不足、退化对应、低 inlier ratio 和高 residual 映射为结构化拒绝原因。固定俯视相机通常只能看到顶面，因此隐藏的 z 厚度不是观测量；当前只检查两个可见主尺寸，z 中心使用桌面高度加已知半高补齐。

**先解释 OBB：它输出什么。** OBB 是 Oriented Bounding Box，即有向包围盒。AABB（Axis-Aligned Bounding Box）只能沿 world 的 x/y/z 轴包住点云；OBB 则允许盒子的三个轴旋转，使包围盒跟随点云的主方向。对当前目标簇，PCL `MomentOfInertiaEstimation` 返回 OBB 的中心 `obb_position`、三个方向轴 `obb_rotation` 和每个方向上的最小/最大投影 `min_point/max_point`。代码用 `max-min` 得到三个包围尺寸，用中心和方向作为物体位姿的几何候选，再和已知立方体尺寸比较。

这里的 OBB 是“点云的几何包围和主轴估计”，不是物体识别器，也不是一定达到全局最小体积的最优包围盒；它主要提供一个快速、可解释的姿态基线。它的结果依赖输入点簇：机械臂点混入、离群点、遮挡或对称形状都会改变主轴。当前立方体的 x/y 尺寸相等，因此绕 world z 轴旋转 90 度不会改变几何证据，代码把这种 yaw 标为 `orientation_ambiguous`；俯视相机看不到厚度时，z 中心由桌面高度和半高先验补齐，而不是由 OBB 观测得到。

**再解释 source 和 target：它们不是两台相机。** 在刚体配准中，`source` 和 `target` 是两份点集：

- `source`：已知模型或模板点，通常用物体自身的 canonical/object frame 表示，例如 CAD 模型上的角点或采样点；
- `target`：RGB-D 观测到的点，当前管线会先从 camera optical frame 反投影，再变换到 world frame，通常是目标 cluster 中的对应点。

如果第 $i$ 个 source 点和第 $i$ 个 target 点确实表示同一个物理特征，SVD 求的就是把物体坐标变到世界坐标的变换：

$$
{}^{W}\mathbf{p}_i \approx {}^{W}\!R_{O}\,{}^{O}\mathbf{p}_i + {}^{W}\!\mathbf{t}_{O},
\qquad
{}^{W}\!T_{O}=\begin{bmatrix}{}^{W}\!R_{O} & {}^{W}\!\mathbf{t}_{O}\\0&1\end{bmatrix}.
$$

因此，视觉模块计算刚体变换不是在替机械臂执行运动，而是在回答任务层最需要的问题：“物体坐标系相对于 world 在哪里、朝向如何？”机械臂规划器随后才会使用这个 `object pose` 计算抓取位姿和运动轨迹。相机坐标到 world 的 TF 是传感器外参；source 到 target 的 SVD 变换是模型到观测的物体位姿，两者是不同层次的变换。

在当前实现中，`estimateKnownCorrespondences(source, target)` 并没有被默认的 `object_pose_estimator_node` 调用。真实运行路径是 `depth → world 点云 → target cluster → estimateBoxPose(OBB)`；SVD 单测使用人为构造的 source/target 对来验证这个通用配准入口。只有未来加入可靠的模型特征对应关系时，才适合把 SVD 接入主视觉路径。

**为什么在这里保留 PCL SVD。** 这里的 SVD 指 Singular Value Decomposition，奇异值分解。Stage Q 不只是要让当前这个已知尺寸的立方体通过 OBB 工作，还需要保留一个可以复用于“模型点集 ↔ 观测点集”的标准刚体配准入口。这样，后续如果换成带角点、特征点或 CAD 采样点的物体，只要上游能够建立可靠的对应关系，就可以用成熟的 PCL 求解器直接估计完整的旋转和平移，而不必再在项目中维护一套 Kabsch/SVD 实现。当前 `TransformationEstimationSVD` 的测试同时验证了正常变换、点数不匹配和共线退化；但默认盒体运行路径仍主要使用 OBB，因为俯视 RGB-D 通常只有顶面点，没有稳定的逐点对应关系。

PCL 的 `TransformationEstimationSVD` 求解“已知对应点集合之间的最佳刚体变换”：先计算 source 和 target 的质心，再对去中心化点集建立协方差矩阵，对协方差做 SVD，利用左右奇异向量恢复旋转，最后由质心差得到平移。它求的是 `R/t` 刚体变换，不改变尺度，也不负责寻找对应关系、去除外点或识别物体。

**引入 SVD 的后果。** 正面效果是求解器短小、确定、计算量低，并且显式约束结果为不含镜像的刚体旋转；输出的 RMS residual 还可以作为对应关系质量的一个检查信号。代价是 SVD 把正确性前提交给上游：第 $i$ 个 source 点必须确实对应第 $i$ 个 target 点，点集还必须具有至少非共线的几何分布。它不自动解决匹配、遮挡、外点、尺度变化或物体对称性；共线或近共线退化、错误对应和对称点集都可能使姿态不唯一，甚至产生 residual 很小但语义错误的结果。因此代码在调用前检查点数、有限值和退化，在调用后检查变换有限性、旋转行列式和 residual；未来接入真实模型配准时，还需要加入特征匹配、RANSAC/外点剔除和模型级验收。对于当前方盒，SVD 还不能消除 90 度 yaw 对称，不能把不可观测方向变成真实观测。

设第 $i$ 对对应点为 source 点 $\mathbf{x}_i$ 和 target 点 $\mathbf{y}_i$，目标是求 source $\rightarrow$ target 的刚体变换：

$$
\min_{R,t}\; \sum_i \left\|R\mathbf{x}_i + \mathbf{t} - \mathbf{y}_i\right\|^2,
\qquad R^T R = I,\; \det(R)=1.
$$

先计算两组点的质心并去中心化：

$$
\bar{\mathbf{x}}=\frac{1}{N}\sum_i\mathbf{x}_i,\qquad
\bar{\mathbf{y}}=\frac{1}{N}\sum_i\mathbf{y}_i,
$$

$$
\tilde{\mathbf{x}}_i=\mathbf{x}_i-\bar{\mathbf{x}},\qquad
\tilde{\mathbf{y}}_i=\mathbf{y}_i-\bar{\mathbf{y}},\qquad
H=\sum_i\tilde{\mathbf{x}}_i\tilde{\mathbf{y}}_i^T.
$$

对协方差矩阵做 SVD：

$$
H=U\Sigma V^T.
$$

旋转和平移为：

$$
R=V\,\operatorname{diag}\left(1,1,\det(VU^T)\right)U^T,
\qquad
\mathbf{t}=\bar{\mathbf{y}}-R\bar{\mathbf{x}}.
$$

中间的对角矩阵用于在数值计算产生镜像反射时强制 $\det(R)=1$，因为刚体旋转不能包含镜像。求出 $(R,\mathbf{t})$ 后，再用每对点的变换误差计算 RMS residual：

$$
\mathrm{RMS}=\sqrt{\frac{1}{N}\sum_i
\left\|R\mathbf{x}_i+\mathbf{t}-\mathbf{y}_i\right\|^2}.
$$

它适合“第 i 个 source 点明确对应第 i 个 target 点”的配准问题，因此当前单测专门覆盖点数不匹配和共线退化。错误对应、严重外点、对称点集或缺少可靠对应关系时，SVD 可能得到残差看似合理但语义错误的姿态。当前代码保留了这个 PCL SVD 入口并验证其退化行为，但默认盒体运行路径主要使用 OBB；如果以后接入 SVD，必须在它前面增加对应关系生成和质量验证。

对于立方体，yaw 存在 90 度对称性。当前输出采用规范姿态，但不应把它解释为已经观测到完整 6D 方向。`VisionObjectPose` 目前没有单独的 `orientation_ambiguous` 字段，任务层必须结合已知立方体先验处理这个限制。

最后，estimator 将成功或拒绝结果统一转换成 `VisionObjectPose`。拒绝样本也发布，但 `accepted=false`，这样任务层不会把上一帧 pose 静默当成新观测。oracle pose 继续使用独立 topic，视觉节点不覆盖真值接口。

### 8.7 失败模式与验证手段

验证按“纯算法 → ROS 契约 → 真实运行”三层进行：

| 层次 | 验证内容 | 通过标准 |
| --- | --- | --- |
| 纯几何单测 | PCL SVD 正确变换、点数不匹配、共线退化、桌面去除/聚类、完整 OBB、仅顶面可见、错误尺寸 | 7 项测试全部通过，失败输入返回结构化原因 |
| 包级构建和 lint | `mujoco_perception` 构建、gtest、copyright、cppcheck、cpplint、uncrustify、xmllint | 包级 7 个 CTest 项目全部通过 |
| 全工作区回归 | `colcon test` 和 `colcon test-result --all --verbose` | 6 packages，428 tests，0 failures，64 skipped |
| 运行时 smoke | 直接启动 bridge 和 estimator，启用 RGB-D，读取 `VisionObjectPose` | `accepted=true`、约 180 点、残差约 0.676 mm、位置约 `(0.5000, 0.0002, 0.2400)m` |
| 进程和时钟卫生 | 检查 `/clock` publisher 数量和结束后的进程列表 | `/clock` 只有一个 publisher，结束后无 bridge/estimator 残留 |

运行时 smoke 只能证明默认静态 fixture 的完整链路可工作，不能证明通用视觉鲁棒性。仍需单独补充随机物体位姿、深度噪声、局部遮挡、误分割、延迟 p50/p95 和固定图像/点云重放。

典型失败的定位顺序是：先检查时间戳交集和 frame，再检查 depth 单位与桌面平面，之后查看目标簇点数和尺寸，最后查看 residual/inlier ratio。若估计器没有输出，优先判断输入没有组成完整帧；若输出 `accepted=false`，再按 rejection reason 区分无目标、尺寸不符、点数不足、低 inlier 或高残差。

### 8.8 你没问但值得注意的

1. **可观测性**：后续应同时记录原始 RGB-D、目标点簇、估计 pose、oracle pose 和最终落点，否则只能知道“视觉失败”，不能区分传感器、分割、姿态估计还是执行失败。
2. **可测试性**：固定图像/点云 fixture 应脱离 MuJoCo 重放 estimator，运行时 smoke 不能替代离线回归。
3. **时间语义**：当前允许的时间差是精确 `0 ns`，适合仿真同源发布；真实设备需要测量采集延迟、传输延迟和 TF 查询年龄后再决定 exact 或 approximate policy。
4. **姿态定义**：方形 box 的对称 yaw 与抓取工具 yaw 还没有完整进入视觉任务契约，Week 4.5 前应冻结等价姿态规则。

### 8.9 排查记录

第一次真实运行把 OBB 的三个轴都强制和盒体尺寸比较，正常顶面观测被错误判定为尺寸不匹配。排查后确认相机没有观测到隐藏的盒体厚度，问题不是 PCL OBB 失败，而是把不可观测量错误地当成验收条件。修正为只比较两个可见主尺寸并用已知桌面补齐 z 后，单测和真实运行均通过。

另一次边界是小型规则平面在 PCL RANSAC 中没有稳定内点。实现保留了已知桌面高度的 PassThrough 退化路径，并在单测中覆盖桌面去除和目标聚类，避免把仿真 fixture 的统计退化误报成目标不存在。

新增代码最初还因 uncrustify 的单行大括号规则导致包级测试失败；修正格式后 `mujoco_perception` 的 7 个 CTest 项目全部通过。这类 lint 失败与几何逻辑失败必须分开记录。

### 8.10 本阶段边界与后续

Stage Q 当前是可解释的几何基线，不是通用物体识别系统。它支持单个已知尺寸立方体和已知桌面，不支持任意形状、完整可观测 6D yaw、随机遮挡恢复、多相机融合或真实相机噪声模型。

尚未完成的评测包括随机物体位姿、深度噪声、局部遮挡、误分割、长时间延迟统计和 rosbag/fixed fixture 重放。Stage R 再决定如何把视觉输出接入 oracle/vision 对照和任务执行；在此之前不能把静态 smoke test 当成完整视觉验收。

### 8.11 不引入深度模型时的泛化讨论

如果不引入深度模型，泛化的核心不是把当前阈值调得更宽，而是把固定场景假设逐层替换成可验证的几何和模型选择：

1. **场景泛化**：把固定 world ROI 改成由工作台、安全区或多个平面估计出的可配置区域；用法向和高度约束检测支持面；对深度做统计滤波、法向估计和离群点剔除；需要更大视野时使用多相机或移动相机，并显式标定外参。
2. **目标泛化**：不再只选最大簇，而是保留多个候选，使用尺寸、法向、颜色直方图、几何描述子或 CAD/template 匹配给候选打分。对已知刚体物体，可以使用 RANSAC、FPFH、基于模板的匹配和 ICP 精配准；对未知物体，只能先得到“有一个独立物体”的几何簇，不能自然得到可靠的类别和完整 6D 姿态。
3. **姿态泛化**：为每个物体模型定义可观测面、对称性和允许误差；用 RANSAC/多假设验证拒绝错误配准；对遮挡或对称物体输出等价姿态集合、低置信度或不可见，而不是伪造唯一 yaw。
4. **任务泛化**：把 ROI、平面方向、聚类阈值、候选评分和模型尺寸从代码常量改成配置或模型描述，并让每个候选携带 residual、inlier ratio、可见比例和拒绝原因。

不使用深度模型时，通常可以做到“结构化桌面场景中的少量已知刚体物体”：在标定稳定、光照和深度质量可控、遮挡中等的条件下，完成桌面分割、多个候选物体分离、已知 CAD/template 配准和毫米到厘米级的位姿估计。再往前扩展到开放类别、严重遮挡、透明/反光物体、柔性物体、杂乱背景和可靠的未知物体 6D 姿态，就会快速受到几何信息不足的限制。

因此可按三步推进：先把当前“单目标立方体”扩展为“多候选但仍有模型先验”，再加入已知模型的 RANSAC/FPFH/ICP 组合，最后评估多相机和跟踪。每一步都应保留拒绝和不确定状态；无深度模型方案的上限不是“任何场景都能识别”，而是“对结构足够强、模型足够明确的场景保持可解释的鲁棒性”。

### 8.12 流程鲁棒性：从初始检测到抓取过程

物体/环境泛化回答“换一个对象或场景还能不能识别”，流程鲁棒性回答“同一个任务进行到一半、观测条件变化后，系统会不会继续使用错误结果”。当前 Stage Q 主要是**逐帧的几何估计器**：每个完整同步帧独立执行 ROI、平面去除、聚类和 OBB，不维护目标轨迹，不知道机械臂是否正在夹持，也不拥有“抓取中/已掉落/等待重新观察”等任务状态。

因此当前行为应按场景理解：

| 过程场景 | 当前管线的实际行为 | 主要风险 |
| --- | --- | --- |
| 初始静止、目标在桌面 ROI 内 | 桌面去除后得到目标簇，OBB 输出 accepted pose | 这是当前 smoke test 覆盖的主要情况 |
| 机械臂进入视野但与物体分离 | 机械臂点可能形成另一个簇；当前选择最大合法簇 | 机械臂簇更大时可能选错，或因尺寸不符而拒绝 |
| 机械臂和物体点云接触并连成一个簇 | 二者作为一个 cluster 交给 OBB 和尺寸检查 | 通常会因尺寸、residual 或 inlier ratio 失败；但污染较小时也可能产生偏移甚至错误接受，当前没有 robot mask 或语义检查 |
| 夹爪夹住后物体被抬起 | 物体仍可能形成目标簇，但 `anchor_z_to_plane=true` 会把 z 中心按桌面高度加半高补齐 | 即使 XY 和尺寸通过，输出 z 仍代表桌面先验，不是实际抬升高度；当前不能把它当作可靠的空中 6D pose |
| 夹持时被手指遮挡 | 可见点减少或形状变残，可能在聚类阶段变成 `NO_TARGET_CLUSTER`，或在 OBB 阶段因 `LOW_INLIER_RATIO`、`HIGH_RESIDUAL`、尺寸不符而拒绝 | 也可能留下有偏但阈值内的点簇，单帧几何没有时间上下文来发现跳变 |
| 夹持失败、物体掉回桌面且仍在 ROI | 下一帧可能重新检测到桌面上的物体并发布 accepted pose | 管线不会告诉任务层“发生了掉落”，只告诉它当前看到了一个符合先验的盒子 |
| 物体掉出 ROI、被完全遮挡或深度无效 | 完整输入到达但没有目标时发布拒绝；如果五类消息没有时间戳交集，则当前节点不处理该帧 | 下游必须区分新鲜拒绝和没有新输出，不能自行复用旧 pose |

这里有一个重要区分：当前 estimator 不会在内部自动“保持上一帧 pose”。完整帧但算法失败时会发布新的 `VisionObjectPose`，其中 `accepted=false` 和新的 sequence；输入消息缺失或时间戳没有组成完整交集时，则可能没有输出。任务层如果把最后一次 accepted pose 永久当作当前目标，就会把视觉节点之外的旧值复用重新引入系统。

当前管线也没有显式的机器人背景处理。world ROI 只能限制空间范围，桌面分割只能去除支持平面，Euclidean clustering 只能按空间连通性分组；它们都不能回答“这片点属于机械臂还是物体”。要提高流程鲁棒性，至少需要：

1. 用关节状态和 TF 得到机器人 link 的几何占据，渲染或膨胀成 robot mask，在点云分割前去除已知机器人点；
2. 在抓取阶段使用 `BridgeObservation` 的夹持/接触状态和末端位姿，告诉感知层当前是“桌面检测”还是“持物检测”；
3. 引入跨帧目标跟踪和数据关联，检查位置、尺寸、速度和残差是否连续，发现突然跳变时拒绝而不是直接替换；
4. 为“桌面上”“夹持中”“可能掉落”“不可见”“重新搜索”定义显式状态，并规定每个状态允许的观测来源和超时动作；
5. 将视觉 accepted 结果与抓取状态、抬升位移和最终落点联合验证，而不是把单帧 OBB 当成抓取成功证明。

一个不依赖深度模型的可行流程是：初始阶段用桌面约束检测物体，夹持后切换到机器人 mask + 目标跟踪，抬升阶段用目标相对 TCP 的连续运动判断是否跟随，检测到遮挡或掉落时清除旧目标并重新搜索，最终用桌面重新出现和放置区域验证结果。这个方案能显著减少“机械臂混入”和“掉落后仍沿用旧目标”的风险，但仍不能从视觉单帧可靠推断接触力、滑落瞬间或完全遮挡下的真实姿态。

## 9. Stage R：oracle/vision 对照和任务接入

### 9.0 一句话总结

Stage R 把 Stage Q 的视觉物体位姿接入已有任务执行器，同时保留 oracle 作为可比较的控制上限。`observation_source` 决定一个 episode 使用哪一种物体观测；两种来源共用 `EpisodeController`、FSM 和 waypoint source，不在视觉失败时偷偷回退到旧 pose。vision 样本必须和 bridge observation 的 `bridge_session`、`generation`、`sample_sequence` 三个字段精确匹配，并通过 confidence、residual 和 inlier ratio 质量门；拒绝或低质量样本结束 episode 且不发布新的运动目标。

### 9.1 改动清单与验证结果

改动集中在以下接口和适配层：

| 文件 | 改动 |
| --- | --- |
| [`EpisodeOutcome.msg`](../../src/manipulation_interfaces/msg/EpisodeOutcome.msg) | 增加 observation source、confidence、residual、失败层级和失败原因字段 |
| [`observation_frame.hpp`](../../src/task_executor/include/task_executor/observation_frame.hpp) | 领域 observation 增加 source、confidence、residual 和 rejection reason，不依赖 ROS 消息 |
| [`task_executor_config.hpp`](../../src/task_executor/include/task_executor/task_executor_config.hpp) / [`task_executor_config.cpp`](../../src/task_executor/src/task_executor_config.cpp) | 增加 `oracle`/`vision` 选择和三个视觉质量阈值，默认 `0.5 / 0.005m / 0.7` |
| [`task_executor_node.cpp`](../../src/task_executor/src/task_executor_node.cpp) | 订阅视觉 pose，按三字段配对，做质量准入，记录 outcome，并在失败时停止 episode |
| [`demo.launch.py`](../../src/mujoco_bridge/launch/demo.launch.py) | 暴露 `observation_source` 和三个视觉阈值 launch 参数 |
| [`test_task_executor_config.cpp`](../../src/task_executor/test/test_task_executor_config.cpp) | 覆盖默认 oracle、vision override、阈值和模式配置 |

验证结果（实测）：

| 验证 | 结果 |
| --- | --- |
| `colcon build --symlink-install` | 6 packages 构建通过 |
| task_executor 单包测试、lint、XML 检查 | 16/16 通过；`cpplint`、`uncrustify` 和独立 `xmllint` 通过 |
| oracle smoke | `/clock` 只有 1 个 publisher；完整 episode 成功，`NONE`、source=`oracle`、confidence=`1.0`、residual=`0.0`、retries=`0` |
| vision smoke | accepted 样本 confidence 约 `0.824`、residual 约 `0.000676m`、inlier ratio `1.0`；`PREGRASP` 使用视觉 y 值；遮挡后 `MODEL_EXTENT_MISMATCH` 以 `VISION_REJECTED` 结束 |
| 低置信度 smoke | `vision.min_confidence:=0.99` 时同一约 `0.824` 样本以 `VISION_LOW_CONFIDENCE` 结束 |

全量回归最终结果为 **428 tests，0 errors，0 failures，64 skipped**；6 个包构建通过，`task_executor/xmllint` 单独复核也通过。此前并行执行曾出现该 XML 检查偶发 timeout，重新读取结果并单独运行后结果稳定，因此没有把那次调度问题当作代码失败。

### 9.2 原始提问与任务边界

> 你好，请看到week4.md，接下来是stageR的实现。

这次提问没有另行指定新的消息格式或失败恢复策略，因此实现沿用 Week 4 计划中已经冻结的边界：Stage R 只接入 oracle/vision 对照，不改 Stage Q 几何算法，不接入 learned policy、MoveIt 或障碍规划。实现选择“拒绝并结束当前 episode”作为当前低质量视觉样本的明确语义；重新观察和跨帧跟踪留到后续阶段。

### 9.3 两种 observation 如何共用任务执行链

`BridgeObservation` 仍是任务状态的载体：它提供关节、接触信号、`world_to_hand_tcp` 以及生命周期字段。oracle 模式直接取其中的 ground-truth object pose。vision 模式只替换 object pose：节点从 `/object_pose_estimator/object_pose` 取得 pose 和质量指标，再从同 sample sequence 的 bridge observation 取得其余状态，最后构造同一个无 ROS 的 `ObservationEnvelope`。

这样复用的是领域控制流程，而不是复用数据来源。`EpisodeController` 不知道 pose 来自 oracle 还是 vision，只消费已经通过准入的 `ObservationFrame`；`TaskExecutorNode` 负责 ROS 订阅、缓存、配对、质量判断和消息转换。FSM、Cartesian waypoint 几何和 diff-IK 求解都没有为视觉另写一份分支，因此对照结果的差异可以归因到观测链，而不是两套任务逻辑。

### 9.4 视觉质量门与失败语义

默认阈值是 `confidence >= 0.5`、`residual_m <= 0.005` 和 `inlier_ratio >= 0.7`，同时要求 `accepted=true` 且数值有限。阈值是任务层准入条件，不是 estimator 内部的几何算法定义；这样可以在不改视觉节点的情况下做严格或宽松的对照实验。

accepted 且通过三项阈值的样本会进入正常 FSM。`accepted=false` 的样本以 `VISION_REJECTED` 结束；accepted 但阈值不满足的样本以 `VISION_LOW_CONFIDENCE` 结束。两种情况都不调用 `EpisodeController::onObservation()`，所以不会产生新的 waypoint 或 joint command，也不会把上一次 accepted pose 复制到当前帧。`EpisodeOutcome` 记录 source、最后一次质量指标、`observation_failure_layer=perception` 和 estimator rejection reason，便于把感知失败与执行失败分开统计。

### 9.5 权衡与替代方案

把视觉 pose 直接写进原有 oracle topic 会少一个订阅者，但会丢失来源和拒绝语义，oracle/vision 的统计也会被混合。让视觉失败时保持上一份 pose 可以减少短暂丢帧，却会把过期状态带入抓取和 reset 后的新 generation。当前选择精确配对并立即失败，牺牲短时连续性换取可审计的生命周期和失败边界；以后若需要重试，应显式增加“重新观察”状态，而不是隐式复用缓存。

### 9.6 失败模式与验证手段

| 失败模式 | 现象 | 验证或防护 |
| --- | --- | --- |
| 视觉与 bridge 无同 sequence | vision topic 有消息但 executor 不推进 | 检查两侧三字段；配对失败时确认没有新 command |
| reset 后旧 generation 到达 | 旧 pose 可能看起来合理但属于上一 episode | 精确比较 session/generation；用 reset generation probe 验证旧样本被拒绝 |
| estimator 明确拒绝 | `accepted=false`，例如 `MODEL_EXTENT_MISMATCH` | outcome 为 `VISION_REJECTED`，检查没有后续 waypoint |
| accepted 但 confidence/residual/inlier 不达标 | 日志出现 `VISION_LOW_CONFIDENCE` | 设置 `vision.min_confidence:=0.99` 注入低置信场景 |
| 失败时复用了旧 accepted pose | 机械臂继续向旧目标运动 | 监听 joint command 与 outcome；拒绝路径不调用 `onObservation()` |
| oracle/vision 统计混在一起 | outcome 没有来源或来源被推断 | 断言 `observation_source` 明确为 `oracle` 或 `vision`，按字段分组统计 |
| 视觉质量字段为 NaN/Inf | 无效数值绕过比较 | `std::isfinite` 是质量门的一部分，并用配置单测覆盖非法阈值 |

### 9.7 排查记录

第一次 vision smoke 中，初始 accepted 样本能进入 `HOME` 和 `PREGRASP`，但抓取动作让机械臂与盒子在相机中连成不合法几何簇，estimator 发布 `MODEL_EXTENT_MISMATCH`。如果任务层只保存上一份 accepted pose，FSM 会继续执行一个视觉已经无法支持的目标。实际结果是 executor 收到新 rejected sample 后以 `VISION_REJECTED` 结束，未发布后续目标；这验证了“拒绝消息也是新观测”的语义。

低置信度实验把阈值提高到 `0.99`。estimator 的约 `0.824` confidence 样本仍是 accepted，但 executor 在任务层质量门拒绝它并记录 `VISION_LOW_CONFIDENCE`。这说明 estimator 的 accepted 与任务层的可执行质量不是同一个概念，阈值必须在 outcome 中留下可追溯配置和指标。

并行全量测试曾让 `xmllint` 偶发 timeout，单独运行通过。排查后将它视为测试调度/资源竞争信号，而不是 Stage R 逻辑失败；最终回归需要确认全量结果稳定后才能关闭本阶段。

### 9.8 你没问但值得注意的

1. **可观测性**：当前 outcome 已记录 source、质量指标和失败层级，但还没有把 RGB-D 原图、过滤点簇和 vision pose 作为同一 artifact 关联起来。后续应使用 sample sequence 和 generation 作为 rosbag/离线记录的主键，否则只能知道“视觉失败”，不能定位传感器、分割还是 OBB 阶段。
2. **可测试性**：运行时 smoke 依赖 MuJoCo 和 ROS graph，不能覆盖所有配对边界。应增加固定 `VisionObjectPose`/`BridgeObservation` fixture 的 ROS-free controller 测试，至少断言缺配对、旧 generation、重复 sequence、拒绝样本和 NaN 质量值都不发布目标。
3. **时间语义**：当前三字段匹配没有使用近似时间同步，适合 bridge 与 estimator 共享仿真 sample sequence 的场景。真实相机接入前必须测量采集和传输延迟，再决定是否引入近似同步及其最大年龄。
4. **恢复策略**：当前拒绝直接结束 episode，避免旧 pose 误用，但没有“等待下一帧并重新观察”的状态。若要支持恢复，必须同时定义超时、最大尝试次数和 outcome 中的每次拒绝记录。

### 9.9 本阶段边界与后续

Stage R 验证的是观测来源选择、生命周期配对、质量门和停止语义，不证明视觉在动态遮挡、随机物体姿态、深度噪声或真实硬件上可靠。R1~R3 将分别处理机器人掩膜、跟踪状态和任务证据门；现有 `EpisodeController` 的生命周期边界保持不变，但 vision 模式的观测准入和 FSM 成功判据需要按新证据语义调整。Week 4.5 只消费 R3 验收后的观测契约，不重新解释视觉失败。

### 9.10 视觉抓放缺口与后续决策

> 也就是说，当前现在视觉抓放还没能搞定？？？但是就我们当前无障碍的pick and place来说，不是只要object的初始位姿得到accept就可以了吗？有这个强先验还不够吗？？

> 实际上我认为应该直接做方案B。对于方案A来说，先验已经多到遮蔽了视觉模块的作用，并且还需要修改executor的原有逻辑。讲讲如果从视觉切入的话，工业界的common practice是怎么做的？

> 请你对本week的后续阶段进行设计，并且填入文档，如何划分阶段由你决定。这属于较为重大的架构决策，所以需要你写一份adr文档。

当前 vision smoke 在 `PREGRASP` 后因机器人/盒体混簇报 `MODEL_EXTENT_MISMATCH`，只验证了拒绝即停止，没有证明视觉抓放成功。单次初始位姿足以为静止盒体生成抓取目标，但不能证明后续盒体已抬升、未滑落和最终落点；现有 `makeObservationSnapshot()` 又把当前物体 pose 同时用作规划与这些验收信号。用户选择从视觉端解决遮挡和目标身份问题，因此 R1~R2 负责机器人几何过滤与时序估计，R3 只改任务层必须识别的证据状态和成功门，不把旧 pose 或仿真 oracle 伪装成实测。决策依据、替代方案和限制见 [ADR 009](../../docs/adr/009-robot-aware-stateful-vision.md)。本节是设计记录；R1~R3 尚未实现或验证。

## 10. Week 4.5 交接

R1~R3 完成并通过完整视觉抓放验收后，Week 4.5 接收三类稳定输入：带证据状态的视觉 observation、oracle observation 和 episode 生命周期事件。Week 4.5 不改变视觉算法，而是定义这些输入怎样被记录、回放并交给传统规划器或 learned policy。具体计划见 [week4.5.md](week4.5.md)。
