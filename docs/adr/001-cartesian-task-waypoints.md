# ADR 001：Cartesian 任务 waypoint 负责抓取与放置几何

## Status

已在 Stage O 采用（2026-09-26）。

## Context

Stage N 曾通过手调关节命令的 FK 结果得到目标位姿。这些命令混入了位置伺服下垂和接触补偿，得到的 `PLACE` 目标约为 `(0.434, 0.310)m`，而 marker 位于 `(0.5, 0.3)m`。目标本身已经错误时，继续调 IK 无法修正任务几何。

## Decision

任务层输出不依赖 ROS message 的 `CartesianWaypoint`，其中包含 world-frame 的 `hand_tcp` 位姿、阶段和夹爪命令。`PickPlaceCartesianWaypointSource` 直接根据物体和场景几何定义抓取、悬停和放置位姿，并拒绝无效输入位姿。`DiffIkWaypointSource` 消费这个接口，在 `PREGRASP` 锁定物体观测，每个阶段求解一次 IK，最后输出现有的 `JointTarget`。旧的 keyframe source 作为独立的关节空间基线保留。没有采用之前提出的 `KeyframeCartesianWaypointSource` 这个名字，因为从 keyframe 推导任务几何会保留已经否定的依赖关系。

对于当前的方形 box，抓取和放置都使用相对于固定朝下基准姿态的显式 world z 轴旋转，默认值为 0。参数为 0 时，TCP 的 x/y/z 轴分别指向 world 的 +y/+x/-z；该参数不是 ZYX Euler yaw。评审后移除了基于 `atan2(target_y, target_x)` 的径向 yaw 规则，因为目标相对 world 原点的位置不能定义抓取或放置朝向。当前还没有把物体 yaw 作为抓取输入，也没有验证最终 box 朝向。IK 模式默认以 marker 中心验收，keyframe 模式保留旧的实测验收中心。

## 备选方案与后果

继续把关节 keyframe 作为 FK 输入可以保留旧行为，但也会重复已观测到的 6--7cm 目标构造误差。对于旋转物体或有朝向约束的放置，任务位姿必须包含明确的物体到工具抓取变换，以及期望的物体目标位姿。当前固定朝向只适用于已经验证的场景。后续规划或在线伺服层可以消费这个任务位姿，但仍需要各自的碰撞和执行契约。

固定场景在 3cm 放置半径下通过了 20/20 个 episode。这个结果验证的是当前几何和位置伺服的组合，不是通用抓取规划。按阶段记录的 episode 指标保留了任务目标、IK 残差、伺服跟踪和物理 box 落点之间的区别。
