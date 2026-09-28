# ADR 005：策略无关的数据契约与并行 motion backend

## Status

已采用（2026-09-28）。

## Context

Week 4 将加入 RGB-D 和视觉物体位姿。后续项目可能接入 LeRobot、IL、RL 和 VLA。当前系统已有 `BridgeObservation`、`EpisodeController`、Cartesian waypoint 和关节目标，但还没有策略观测、策略动作、逐步数据记录和异步推理的统一边界。

学习策略可能自行产生 motion plan、碰撞规避、关节限位和动作时长。MoveIt 也可以作为传统规划 backend、专家轨迹来源和对照实验。把 learned policy 强制接入 MoveIt 会限制策略接口，也无法表达策略自带规划的实验目标。

## Decision

项目建立策略无关的 observation/action/episode contract，并允许多个 motion backend 并行消费它：

```text
observation bundle
        -> scripted executor
        -> MoveIt planner
        -> learned policy (IL/RL/VLA/LeRobot)
        -> trajectory or action chunk
        -> execution interface
```

Observation 至少包含 session、generation、sample sequence、时间戳、关节状态、夹爪状态、TCP 状态、RGB-D、CameraInfo、object pose、source/confidence 和 task metadata。障碍几何或场景表示作为可选字段加入，等待障碍实验冻结具体编码。

Action 支持关节目标/增量、Cartesian 增量、轨迹或 action chunk 和夹爪命令，并带 control duration、valid-until timestamp 和 source。learned policy 对规划、碰撞规避、关节限位、速度限制和动作时长负责。

Execution interface 只执行进程级和接口级检查：消息维度、NaN/Inf、时间有效性、通信超时和异常停止。它不调用 MoveIt 替 learned policy 重新规划。MoveIt backend 的碰撞检查由该 backend 自己负责。

逐步数据同时保存 observation、policy action、applied action、时间延迟、phase、reward、terminal 状态、failure code 和 reset generation。原始数据先由 rosbag2 或项目中间格式保存，再转换为锁定版本的 LeRobot dataset。

策略推理采用闭环 action chunk：低频策略重新观察和决策，高频 execution interface 执行短动作窗口。推理超时或动作过期必须有明确停止、保持或失败语义；这不要求在线梯度更新。

## Alternatives

把所有策略输出统一转换成 MoveIt `JointTrajectory` 可以复用现有规划接口，但会把 learned policy 的规划能力隐藏在传统 planner 后面，也不能表达端到端 action chunk 和策略自带碰撞约束。

把 LeRobot message、Python dataclass 直接放进 C++ bridge 会让仿真核心依赖 Python 模型环境，增加部署和容器复现成本。采用项目内部契约和 Python adapter 可以把 LeRobot、RL 框架和模型版本隔离。

只记录 `EpisodeOutcome` 可以保留任务结果，但无法训练 IL、计算逐步 reward 或排查策略延迟，因此记录逐步 applied action。

## Consequences

当前 scripted executor 可以保留为 baseline；MoveIt 和 learned policy 可以在相同 observation、episode 和评测接口下比较。代价是需要新增 observation bundle、action adapter、recorder/replay 和策略超时诊断。

LeRobot、PyTorch、VLA 权重和训练框架不进入 core C++ 构建依赖。远程训练产生 checkpoint 后，通过 policy gateway 在本地或远程推理。障碍实验需要额外的场景表示和数据集，不能从当前空桌面结果推断。

## Verification

Week 4.5 至少验证 scripted backend、dummy learned backend、数据 replay、LeRobot dataset export，以及超时、NaN/Inf、过期 action 和错误维度处理。MoveIt backend 的完整规划验证按原路线单独完成，不作为 learned policy 接入门槛。

