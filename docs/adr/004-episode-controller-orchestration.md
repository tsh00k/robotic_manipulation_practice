# ADR 004：用 EpisodeController 集中 episode 编排

## Status

已采用（2026-09-28）。

## Context

`TaskExecutorNode::onTimer()` 原来同时负责 reset 故障、观测准入、IK seed、waypoint 求解、FSM 输入、阶段迁移、目标发布、retry、telemetry 和 outcome。每增加一种观测来源或执行后端，都需要继续修改 ROS 回调控制流程；这些职责也难以在不启动 ROS 的情况下测试。

P2 已把 reset、generation/sequence 准入和看门狗迁入控制对象，P3 又把 IK seed、waypoint、FSM 和阶段 telemetry 迁入。P4 之后仍需要明确 ROS 节点的边界，并固定动作执行顺序。

## Decision

引入 ROS-free 的 `EpisodeController` 作为 episode 的唯一状态所有者。它持有当前 phase、reset token、bridge session/generation、观测序号、retry、失败原因和 `EpisodeTelemetry`，接收 start、reset 回执、完整 `ObservationEnvelope` 与 tick，返回值类型 `EpisodeActions`。

`EpisodeActions` 可以包含 `TargetCommand`、`PhaseTransition`、`ResetRequest`、`EpisodeFinished` 和诊断值。控制器决定这些值及其语义；`TaskExecutorNode` 只把 ROS 消息转换成领域值，并把动作转换成 topic 发布、service 请求和 ROS outcome。节点中的 `executeActions()` 固定执行顺序：目标诊断、当前 phase 目标、迁移日志、reset 请求、episode outcome。目标属于迁移前的 phase，因此 DONE、FAILED 和 retry tick 仍先发布当前目标，再执行后续副作用。

控制器只消费尚未消费的新鲜观测；phase 未变化时可以重发目标，但不追加 telemetry。IK 异常返回一次 `IK_FAILED` 且不发布无效目标；terminal 状态抑制后续动作。阶段时长使用注入的 ROS 仿真时间，看门狗使用 steady clock。当前实现保留单线程 executor 假设。

## Alternatives

继续让节点直接编排可以减少一次值对象转换，但会重新形成巨大的 `onTimer()`，并让 FSM/IK 行为依赖 ROS。把每个阶段拆成独立对象会产生大量接口和跨阶段状态传递，超过当前问题的实际复杂度。采用通用事件总线或 ROS action 会改变外部契约，增加异步幂等和调试成本，因此暂不采用。

## Consequences

控制器可以用纯 C++ 单测断言 reset、目标、迁移、retry 和 outcome 的动作序列；ROS 节点可以用少量集成测试验证消息转换和执行。代价是动作值必须定义执行顺序，且节点仍需缓存最新 ROS 观测和少量日志去重状态。若切换到 `MultiThreadedExecutor`，reset 回调、观测回调、timer 和 `executeActions()` 之间必须新增同步边界；本 ADR 不把单线程假设扩展为线程安全承诺。

## Verification

`colcon build --symlink-install` 和 `colcon test` 通过；P5 当前结果为 375 tests、0 failures。真实回归覆盖固定 diff-IK 成功、keyframe retry、不可达 IK、旧 generation 观测和连续 episode。最终架构图位于 [`docs/task_executor_episode_orchestration.html`](../task_executor_episode_orchestration.html)，其规格通过 Archify showcase 校验。
