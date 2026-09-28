# Week 4 学习笔记

> 本文件先记录 Week 4 的开周范围和验收计划。实现、实测、讲解、追问和失败排查按 Stage P~R 完成后追加到对应章节。Week 4.5 的策略接口、数据链和容器交付不混入本周主线，见 [week4.5.md](week4.5.md)。

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
  - [2.1 Stage P：RGB-D 相机和世界坐标观测](#21-stage-p-rgb-d-相机和世界坐标观测)
  - [2.2 Stage Q：几何分割和刚体位姿估计](#22-stage-q-几何分割和刚体位姿估计)
  - [2.3 Stage R：oracle/vision 对照和任务接入](#23-stage-r-oraclevision-对照和任务接入)
- [3. 本周验收标准](#3-本周验收标准)
- [4. 失败模式和验证方法](#4-失败模式和验证方法)
- [5. 主动提示的问题](#5-主动提示的问题)
- [6. 悬挂问题](#6-悬挂问题)
- [7. Week 4.5 交接](#7-week-45-交接)

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

## 7. Week 4.5 交接

Stage R 完成后，Week 4.5 接收三类稳定输入：视觉 observation、oracle observation 和 episode 生命周期事件。Week 4.5 不改变视觉算法，而是定义这些输入怎样被记录、回放并交给传统规划器或 learned policy。具体计划见 [week4.5.md](week4.5.md)。

