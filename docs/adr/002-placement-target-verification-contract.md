# ADR 002：区分放置任务目标与验收目标

## Status

已采用（2026-09-27）。

## Context

`target.place_x_m/y_m` 决定离线 IK 的 world-frame `hand_tcp` 放置目标；`verify.place_x_m/y_m` 决定 FSM 验收 box 的圆心。第一层重构曾漏把验收参数复制到 `FsmParams`，实际 box 距离预期圆心仅 5.67mm，FSM 却按 `(0, 0)` 判断并报告 `PLACE_MISSED`。即使映射正确，两组参数也能分别覆盖，配置漂移会使执行目标与成功判据分叉而不告警。

## Decision

`TaskExecutorConfig` 显式包含 `PlacementTask` 和 `PlacementVerification`。前者表达任务侧 TCP 目标 XY、悬停与放置高度、工具旋转；后者表达 box 落点验收圆心和半径。原有 ROS 参数名和默认值保持不变。加载配置时，验收字段继续映射到现有 `FsmParams`，任务字段转换为现有 `PickPlaceGeometry`，不修改 FSM 或 waypoint 接口。

`diff_ik` 模式默认要求任务 TCP 目标 XY 与验收 box 圆心 XY 一致；不一致时拒绝启动，并指出 `verify.allow_target_mismatch:=true`。这个参数仅用于明确允许故意不一致的实验。`keyframe` 模式保留历史默认验收中心 `(0.43, 0.31)m`，不检查任务 XY，因为 keyframe 不消费 Cartesian 任务几何。启动日志及每个 episode 的开始日志同时打印任务和验收目标。

## 备选方案与后果

直接删除独立的 `verify.*` 会破坏 keyframe 的历史对照，也无法构造故意不一致的验证实验。允许静默分叉则无法区分参数笔误与实验设计。当前只比较 XY：box 相对 TCP 的物理偏移、旋转和最终姿态尚无统一验收契约，不能据此推断任意场景下两者应严格重合。新增布尔参数和一次结构转换是保持旧执行接口的代价；CSV 消息和列保持不变，目标配对由日志提供可观测性。
