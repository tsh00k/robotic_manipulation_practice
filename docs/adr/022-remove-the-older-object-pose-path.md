# ADR 022: 删除旧的逐帧物体位姿路径

- 状态：已决定
- 日期：2026-10-08
- 范围：`mujoco_perception` 估计器；`manipulation_interfaces/msg/VisionObjectPose`；executor 的 `vision.*` 参数
- 关联：废弃 [ADR 009](009-robot-aware-stateful-vision.md)、[ADR 011](011-vision-object-pose-contract-tightening.md) 定下的逐帧视觉位姿契约；[ADR 018](018-initial-box-detection-in-the-estimator.md) 第 4 条局限所说的“删除旧路径由新 ADR 记录”；[Week 5 Stage 7](../../Job_guides/my_study/week5.md#stage-7清理旧检测路径)

## 背景

Week 3~4 的估计器对每帧深度做平面分割、聚类、盒子模型拟合与跟踪，发布 `~/object_pose`（`VisionObjectPose`）。Week 4.1 起 executor 不再使用它：Stage 5 另起初始位姿检测（旧检测器 40 个布局只接受 28 个，yaw 恒为 0），Stage 7 起抓取前用锁存的初始位姿，Stage 11 起 VERIFY 也改用新检测器（旧检测器对 bin 里的盒子一律拒绝）。估计器仍每帧算一遍旧路径，三个 `vision.*` 参数声明了却不起作用。

## 决策

删除旧路径：`geometry_pipeline`、`object_tracker` 及其单测；估计器里的分割、拟合、跟踪、`~/object_pose` 与两个调试点云话题，以及约 20 个只给它用的参数；`VisionObjectPose` 消息；`RobotMaskDiagnostics` 的 `foreground_points`、`target_cluster_points` 两个字段；executor 与 demo launch 的 `vision.min_confidence`、`vision.max_residual_m`、`vision.min_inlier_ratio`；只服务于旧路径的 Week 3~4 实验工具。估计器保留机器人遮罩与初始 box、bin 检测；场景先验参数（`plane_z_m`、`box_size_*`、`depth_min_m`、`depth_max_m`）保留，直接配置检测器。

## 后果

- 估计器每帧 CPU 从 35 ms 降到 24.5 ms（Week 5 Stage 7 实测）；mujoco_perception 不再依赖 PCL。
- 视觉只剩初始位姿检测（加 VERIFY 时重测），没有逐帧跟踪；搬运期本来就不用视觉（ADR 016）。
- Week 3~4 笔记里指向被删工具的链接失效，工具的最后版本在提交 `7f33ae3`。

## 被否决的做法

- 保留旧路径、只关掉发布：代码和参数仍在，architecture 仍要描述它，维护成本不变。
- 保留 `VisionObjectPose` 消息以备将来逐帧跟踪：将来真要做时，需求会不同（多物体、带 yaw），届时按新需求定义。
