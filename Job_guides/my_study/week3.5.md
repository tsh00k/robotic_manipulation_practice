# Week 3.5 学习笔记

> 本文件记录 `task_executor` episode 编排重构。它承接 Week 3 的 Stage O 和三层职责整理，目标是让 episode 控制逻辑脱离 ROS 回调，成为一个可以用事件和动作单独测试的控制对象。

## 学习重点范围

本阶段只处理 episode 编排边界，不扩展 FK、IK、MuJoCo bridge 或任务状态机的能力。重点是：

1. 明确 `start`、reset 回执、观测和 tick 在 episode 生命周期中的顺序。
2. 让控制对象返回 `request reset`、`publish target`、`finish episode` 等无 ROS 的动作值。
3. 让 ROS 节点只做消息转换、服务/话题通信、定时触发和动作执行。
4. 在每次搬动职责后用单测和固定场景回归证明行为没有漂移。

## 目录

- [1. 背景与现状](#1-背景与现状)
- [2. 原始提问与本阶段回答](#2-原始提问与本阶段回答)
- [3. 目标与边界](#3-目标与边界)
- [4. 控制对象契约](#4-控制对象契约)
  - [4.1 这里用了哪些设计模式](#41-这里用了哪些设计模式)
- [5. 分阶段实施计划](#5-分阶段实施计划)
- [6. 验收与测试矩阵](#6-验收与测试矩阵)
- [7. 权衡、失败模式与可观测性](#7-权衡失败模式与可观测性)
- [8. 主动补充的问题](#8-主动补充的问题)
- [9. 后续记录入口](#9-后续记录入口)
- [10. P0：现有行为基线](#10-p0现有行为基线)

---

## 1. 背景与现状

当前 `onTimer()` 从 [task_executor_node.cpp:261](../../src/task_executor/src/task_executor_node.cpp#L261) 开始，仍串联了 reset 超时、观测准入、IK seed、waypoint 求解、FSM 输入组装、目标发布、阶段 telemetry、retry 和 episode outcome。文件已经有 `TaskExecutorConfig`、`ObservationSnapshot`、`ResetGate`、`EpisodeTelemetry` 等边界，但 episode 的控制流程仍由节点成员和 ROS 回调共同持有。

已有组件不在本阶段重写：

| 组件 | 当前职责 | 本阶段处理 |
| --- | --- | --- |
| `fsm.cpp` | 根据 `FsmInputs` 和 `JointTarget` 计算阶段决策 | 保持不变，继续作为纯决策函数 |
| `WaypointSource` | 从阶段和物体位姿生成关节目标 | 保持接口不变 |
| `ObservationSnapshot` | 把一份观测整理成 FSM/IK 输入 | 作为控制对象的输入值 |
| `ResetGate` | 校验 reset 回执和观测的 session/generation/sequence | 先复用，后续再决定是否内嵌 |
| `EpisodeTelemetry` | 按阶段积累诊断并展开为旧 `EpisodeOutcome` | 由控制对象拥有 |

Week 3 已记录的 reset generation、同一步观测包和目标配置契约继续有效；本阶段不是再次修改这些 ROS 契约。

## 2. 原始提问与本阶段回答

> 我认为屎山主要堆在 task executor，而且未来明显会越来越大。FK/IK 和 mujoco bridge 都还好。
>
> 先重构 episode 编排：让一个可独立测试的控制对象接收“开始、reset 回执、观测、tick”，返回“请求 reset、发布目标、结束 episode”等动作；ROS 节点负责通信和执行这些动作。先用现有回归固定行为，再移动职责。无需为了拆文件而增加 ROS 节点，也不必给每个小步骤造一个接口。

本阶段采用这个方向，但把迁移拆成小步：先冻结动作顺序和失败语义，再建立最小控制对象，最后让节点成为适配层。不会把 ROS action、插件系统或新的进程边界提前写进设计。

## 3. 目标与边界

### 3.1 本阶段目标

- 控制对象不包含 `rclcpp`、ROS message、ROS publisher/client 或 `mjModel`。
- 控制对象拥有 episode 状态、`ResetGate`、当前阶段、retry 计数、waypoint/IK 诊断和 telemetry。
- ROS 节点把消息转换成领域输入，把领域动作转换成 ROS 发布或 service 请求。
- 原有话题名、服务名、消息类型、参数名和 `EpisodeOutcome` 字段保持不变。
- 保持当前单线程 executor 的顺序假设，并把这个假设写入测试和文档。

### 3.2 明确不做

- 不修改 `fsm.cpp` 的阶段逻辑、成功判据或 retry 语义。
- 不修改 `arm_kinematics`、`DiffIkWaypointSource` 或 MuJoCo bridge。
- 不新增 ROS 节点、ROS action server、通用事件总线或后端插件注册表。
- 不在本阶段引入多线程 executor；如果未来需要，另开阶段处理同步。
- 不以“文件行数变少”作为验收标准；验收看职责、行为和测试边界。

## 4. 控制对象契约

第一版使用少量明确的方法和一个动作批次，不为每个内部步骤建接口。建议的领域边界如下，具体命名在 P1 实施时冻结：

```text
EpisodeController
  startEpisode(sim_time_s) -> EpisodeActions
  onResetResponse(request_token, ResetReceipt) -> EpisodeActions
  onObservation(ObservationEnvelope) -> EpisodeActions
  tick(sim_time_s, wall_time) -> EpisodeActions
```

`ObservationEnvelope` 包含现有 `ObservationSnapshot` 以及 bridge session、generation、sample sequence 和仿真时间戳。控制对象只接收普通 C++ 值，不接收 ROS message。

这里有一个需要在 P1 明确处理的现状差距：当前 `ObservationSnapshot` 的头文件仍包含 ROS message 类型，`EpisodeTelemetry` 也直接提供了展开为 `EpisodeOutcome` ROS message 的方法。因此它们现在还不是完全 ROS-free 的领域层。计划中的做法是保留现有转换函数作为适配层入口，另外让控制对象使用不包含 ROS 的 `ObservationFrame`、`PhaseTelemetry` 和 `EpisodeResult`；ROS message 的组装留在节点中。

`onObservation()` 只负责按 generation/sequence 门控并保存最新完整观测；它不能因为消息到达就直接推进 FSM 或发布目标。目标重发、阶段推进和一次性 outcome 都由 `tick()` 统一产生，这样控制频率仍由现有 20 Hz timer 决定，而不会被观测发布频率隐式改变。

`EpisodeActions` 至少表达：

```text
ResetRequest { request_token }
TargetCommand { JointTarget }
EpisodeFinished { success, failure_code, retries, telemetry }
DiagnosticEvent { accepted/rejected observation, phase transition, IK failure }
```

动作是值而不是回调。节点收到 `ResetRequest` 后调用 ROS service，并把回执连同 token 送回控制对象；收到 `TargetCommand` 后发布现有两个命令话题；收到 `EpisodeFinished` 后展开为原有 `EpisodeOutcome`。动作批次可以为空，且同一个 tick 不应重复产生一次性结束动作。

时间边界也要显式化：节点把 `rclcpp::Time` 转成仿真秒数，把 `steady_clock` 只用于 reset/观测看门狗；控制对象不直接读取系统时钟。这样单测可以精确复现“仿真时间不前进但墙钟超时”和“reset 回调晚到”的场景。

### 4.1 这里用了哪些设计模式

> 这里有什么设计模式

这里不需要把代码硬套进一组经典 GoF 模板。当前代码和计划中的重构，主要使用几种反复出现的工程组织方式：

| 模式或原则 | 当前/计划中的位置 | 它解决的问题 |
| --- | --- | --- |
| **显式状态机** | `Phase` + [`step()`](../../src/task_executor/include/task_executor/fsm.hpp#L140) | 用“当前阶段 + 观测”得到“下一阶段 + 原因”。当前不是每个阶段一个类，而是一个更容易测试的纯函数。 |
| **Strategy 策略模式** | [`WaypointSource`](../../src/task_executor/include/task_executor/waypoint_source.hpp#L48)、`KeyframeWaypointSource`、`DiffIkWaypointSource` | FSM 不需要知道目标由查表还是 IK 计算；替换目标算法时不必改 FSM。 |
| **Observer / 发布订阅** | ROS topics 和 subscriptions | bridge 与 executor 通过 ROS 消息通信，不直接调用对方；消息到达顺序和延迟因此需要单独处理。 |
| **Guard / 门控** | [`ResetGate`](../../src/task_executor/include/task_executor/reset_gate.hpp#L23) | 只有 session、generation 和 sample sequence 正确的观测才能驱动 episode。这不是严格意义上的 GoF 模式，而是一个时序准入组件。 |
| **Adapter 适配器** | 计划中的 `TaskExecutorNode` | ROS message 转成领域数据，`EpisodeActions` 再转成 ROS publish/service 调用。节点负责翻译，不负责 episode 决策。 |
| **Command 命令对象** | 计划中的 `ResetRequest`、`TargetCommand`、`EpisodeFinished` | 控制对象返回“要做什么”的值，节点之后执行它们；这样单测可以直接断言动作序列，不必启动 ROS。 |
| **应用层编排器** | 计划中的 `EpisodeController` | 它安排 reset、观测、waypoint、FSM、retry 和 telemetry 的顺序。这是应用服务式的编排边界，不是一个需要大量子类的经典模式。 |

另外还有两个更基础的工程原则：`JointTarget`、`ObjectPose` 和 `FsmInputs` 是值对象，便于构造测试输入；`fsm.cpp` 依赖数据和抽象接口，而不是 ROS 或具体 IK 实现，体现了依赖倒置。

当前分层还没有完全干净：`ObservationSnapshot` 仍在头文件中包含 ROS message 类型，`EpisodeTelemetry` 也直接提供 ROS `EpisodeOutcome` 的展开方法。因此 P1 需要把 ROS message 转换留在适配层，让 `EpisodeController` 使用 ROS-free 的 `ObservationFrame`、`EpisodeResult` 等领域数据。设计模式的价值在这里不是增加类的数量，而是让每个边界的输入、输出和副作用变得清楚、可测试。

## 5. 分阶段实施计划

### P0：冻结现有行为契约

在写控制对象前整理当前行为快照：正常 tick 的目标发布顺序、terminal 阶段是否继续发布、retry 时 reset 请求的次数、IK 失败和 reset 失败的 outcome 字段，以及 `start_episode` 在运行中再次到达时的处理。记录现有单测数量和 Week 3 的固定场景、keyframe、偏移场景、旧观测探针结果。

**状态：已完成。** 结果见 [10. P0：现有行为基线](#10-p0现有行为基线)。

**出口条件：** 有一组可重复的基线命令和结果文件；所有后续差异都能归因到重构，而不是基线变化。

### P1：建立最小控制对象和事件/动作类型

新增纯 C++ 的 `EpisodeController` 及其领域类型。先接入现有 `ResetGate`、`WaypointSource` 和 `EpisodeTelemetry`，不改节点行为。单测覆盖 idle、start、reset pending、reset success、等待新鲜观测和 terminal 状态。

**出口条件：** 不启动 ROS 也能用测试构造一轮最小 episode，并断言动作序列；没有 ROS include 或 `rclcpp` 依赖。

### P2：迁移 episode 生命周期和 reset

把 `onStartEpisode()`、reset request token、reset response、retry reset 和看门狗结果迁入控制对象。节点只负责发送 service 和回传回执。保留旧日志字段，但日志内容由 `DiagnosticEvent` 携带必要数据。

**出口条件：** 连续 start、回执乱序、旧回执、服务失败、retry reset 和 bridge 重启 session 都有单测；正常链路仍只发送预期次数的 reset 请求。

### P3：迁移观测准入和 tick 决策

把当前 `onTimer()` 中的观测准入、IK seed、waypoint 求解、FSM 输入组装和阶段迁移搬入控制对象。节点的 timer 只提供时间、转发最新领域观测并执行返回动作。保留当前语义：每个可用 tick 重发当前目标，阶段改变时记录一条 telemetry，IK 异常只生成一次失败 outcome。

**出口条件：** 单测覆盖 stale/duplicate/superseded observation、IK failure、phase no-op、retry、done 和 failed；控制对象的动作序列能解释每一次命令或 outcome。

### P4：收窄 ROS 节点为适配层

删除节点中重复的 episode 成员和分支，只保留 ROS entity 创建、消息到领域值的转换、service 回调、timer 驱动、动作执行和日志格式化。不要为了让节点“看起来薄”而继续抽出零价值的小类。

**出口条件：** 节点不再直接调用 `step()`、`jointTargetFor()` 或修改 phase/retry/telemetry；这些调用只出现在控制对象中。话题、服务和参数的外部契约不变。

### P5：回归、文档和架构记录

在 `robotics-dev` 中执行 build、test 和真实可执行文件回归。覆盖固定 IK 20 次、keyframe 旧模式、偏移物体、故意不可达 IK、retry reset、延迟旧观测探针。完成后补一份 `docs/adr/004-episode-controller.md`，记录采用的控制对象边界、动作幂等性和单线程前提，并把最终项目事实同步到 `docs/architecture.md`。

**出口条件：** 所有基线结果不退化；ADR 与 architecture 只写已经实现和实测的结论；本文件追加每个 P 阶段的改动、验证和排查记录。

## 6. 验收与测试矩阵

| 场景 | 预期控制对象动作 | 验证方式 |
| --- | --- | --- |
| 未 start，只有 tick | 空动作 | 单测 |
| start 后 service 未就绪 | 保持 reset pending，不发布目标 | 单测 + 日志 |
| reset 回执失败或超时 | 一次失败 outcome，进入 terminal | 单测 |
| 旧 session/generation/sequence 观测 | 拒绝观测，不发布目标 | 单测 + 旧观测探针 |
| 新鲜完整观测 | 发布当前 `JointTarget`，允许 FSM 前进 | 单测 + ROS 回归 |
| 当前阶段未到位 | 重发目标，不追加 telemetry | 单测 |
| 阶段迁移 | 追加一条 `PhaseTelemetry`，更新 phase | 单测 + CSV |
| IK 抛错 | `IK_FAILED`，不发送无效目标，outcome 只发布一次 | 单测 + 故意不可达目标 |
| retry | 记录真实失败原因，增加 retry，发一次 reset 请求 | 单测 + retry 实跑 |
| `DONE`/`FAILED` | 发布一次 outcome，后续 tick 为空 | 单测 + 固定场景 |
| episode 运行中再次 start | 按 P0 冻结的现有语义执行，并测试旧回执不会污染新 episode | 单测 |

运行验证仍遵守项目约定：先用 `ps -eo pid,comm` 确认没有遗留 bridge，检查 `/clock` 只有一个 publisher，直接运行安装目录可执行文件；构建使用 `colcon build --symlink-install`，测试使用 `colcon test` 和 `colcon test-result --all --verbose`。

## 7. 权衡、失败模式与可观测性

### 7.1 设计权衡

采用“有状态控制对象 + 值动作批次”，而不是把 episode 写成一个泛化事件总线。控制对象需要持有当前 phase、reset gate、waypoint 缓存和 telemetry；把它强行改成无状态 reducer 会增加状态重建和事件序列复杂度。采用少量明确方法，而不是为每个阶段或每种 ROS 回调造接口，保留替换观测来源和执行后端的空间，同时避免接口数量跟着 FSM 阶段增长。

动作值的代价是节点需要做一次转换，且必须定义动作顺序；收益是单测能直接断言“发生了什么”，不会依赖 mock publisher 或 ROS graph。动作设计必须保持幂等边界：持续目标可以每 tick 重发，一次性 reset/outcome 必须由 token 和 terminal 状态防重复。

### 7.2 主要失败模式

| 失败模式 | 现象 | 防止/发现 |
| --- | --- | --- |
| reset 回执属于旧 episode | 新 episode 收到旧 generation 或旧 token | token/session 单测，日志打印 request token |
| 同一 tick 重复结束 | runner 收到两个 outcome | controller terminal guard + exactly-once 单测 |
| 把旧观测当新 seed | IK 报越界或动作突然跳变 | generation/sequence gate + 延迟观测探针 |
| 动作执行顺序改变 | phase 已变但发布了错误阶段目标 | P0 记录顺序，动作批次顺序断言 |
| sim time 与 wall time 混用 | 阶段立即超时或永不超时 | 时间作为输入，分别注入两类时钟 |
| 迁移后多线程竞态 | reset、观测和 tick 修改同一状态 | 先保持单线程；启用多线程前另开同步设计 |

## 8. 主动补充的问题

这些问题不是当前用户提问的主线，但会直接决定控制对象是否可测试：

1. **动作幂等性：** timer 重入或 service 回调重复到达时，哪些动作可以重放，哪些必须由 token/terminal 状态去重？P0/P1 用单测回答。
2. **观测保留策略：** 控制对象只保留最新完整观测，还是需要按 `sample_sequence` 排队？当前 FSM 每 tick 只消费最新值，先用单最新值并测试丢帧行为；视觉接入后再评估队列。
3. **episode 抢占：** 运行中再次 `start_episode` 是否明确表示取消旧 episode 并开启新 episode？当前实现会重置自身状态，P0 先把这个行为固定下来，不能让迁移时偶然改变。
4. **并发前提：** 如果以后改用 `MultiThreadedExecutor`，哪些成员需要锁或串行 callback group？当前不提前加锁，先把单线程前提写进 controller 的测试和 ADR。

## 9. 后续记录入口

P0 已完成，P1 尚未开始。每个 P 阶段完成后按项目约定追加：

配套的重构前后架构图：[规格 JSON](../../docs/task_executor_episode_orchestration.json) · [交互式 HTML](../../docs/task_executor_episode_orchestration.html) · [visual-check 回执](../../docs/task_executor_episode_orchestration.visual-check.json)。图中的源码证据固定到生成时的 git revision；浏览器截图检查需要本机提供 Chrome/Chromium。

1. 改动文件和真实验证输出。
2. 控制对象机制、至少一项替代方案和一个失败模式。
3. 用户追问及尚未解锁的问题。
4. 排查过程和被推翻的结论。

完成 P5 后，再把结论性的接口和线程模型同步到 [`docs/architecture.md`](../../docs/architecture.md)，并新增 ADR 004。

## 10. P0：现有行为基线

### 10.0 一句话总结

P0 没有修改 `task_executor` 的运行代码，只建立了重构前的可重复基线。构建和测试通过；固定 IK 场景 20/20 成功，keyframe 旧模式固定场景 1/1 成功，偏移场景清楚地区分了 IK 的泛化能力和 keyframe 的失败边界。reset retry、不可达 IK 和旧 generation 观测也分别得到预期的失败结果。

### 10.1 改动清单与验证结果

本阶段新增的是结果记录，不是生产代码：

| 文件 | 内容 |
| --- | --- |
| [`results/p0_fixed_20.csv`](../../results/p0_fixed_20.csv) | 默认 diff-IK 固定场景 20 次 |
| [`results/p0_keyframe.csv`](../../results/p0_keyframe.csv) | keyframe 固定场景 1 次 |
| [`results/p0_shifted_ik.csv`](../../results/p0_shifted_ik.csv) | 物体沿 `+y` 偏移 4cm，diff-IK 1 次 |
| [`results/p0_shifted_keyframe.csv`](../../results/p0_shifted_keyframe.csv) | 同一偏移场景，keyframe 1 次 |
| [`results/p0_ik_failed.csv`](../../results/p0_ik_failed.csv) | `target.place_x_m:=10.0` 的不可达 IK |
| [`results/p0_retry.csv`](../../results/p0_retry.csv) | 错误验收中心、`max_retries:=1` 的 retry |
| [`src/task_executor/test/reset_generation_probe.py`](../../src/task_executor/test/reset_generation_probe.py) | 旧 generation 观测探针，复用已有脚本 |

验证命令和结果：

| 验证 | 结果 |
| --- | --- |
| `colcon build --symlink-install` | 5 packages finished |
| `colcon test` + `colcon test-result --all --verbose` | 354 tests，0 errors，0 failures，54 skipped |
| 真实节点运行前后 `ps -eo pid,comm` | 无遗留 `mujoco_bridge` 或 `task_executor_node` |
| 实跑时 `/clock` | 只有当前 bridge 发布；进程清理后无残留 publisher |

每次实跑都直接启动 `install/*/lib/*/*_node`，没有使用 `ros2 run` wrapper。失败场景的 runner 退出码为 1 是预期的任务失败，不是 runner 崩溃；脚本仍正常写出 outcome CSV。

### 10.2 当前行为契约

下表是后续 P1 重构必须保持的行为。带“源码确认”的条目来自当前代码路径；带“实测确认”的条目同时有本次运行证据。

| 输入或时机 | 当前行为 | 证据 |
| --- | --- | --- |
| 第一次 `start_episode` | 清空本地 episode 记录，phase 设为 `HOME`，retry 清零，调用一次 reset；reset 成功后等待匹配 generation 的观测 | `onStartEpisode()`、`beginReset()`；固定场景实测确认 |
| reset pending | timer 只尝试发送一次待处理请求，服务未就绪时不发布目标 | `onTimer()` 263–295 行源码确认 |
| 接受一份完整新鲜观测 | 先给 diff-IK 设置 seed，再为当前 phase 求目标并发布关节/夹爪命令，之后才调用 FSM | `onTimer()` 297–361 行源码确认；日志按 phase 记录 |
| 当前 phase 未完成 | 继续重发当前目标，不追加阶段 telemetry | `decision.next_phase == phase_` 分支源码确认 |
| phase 发生变化 | 追加一条 `PhaseTelemetry`，更新 phase 和 `phase_start_time_` | `onTimer()` 368–441 行源码确认 |
| retry | 记录进入 `RECOVER` 的真实失败原因，retry 加一，reset generation 递增，重新从 `HOME` 开始 | [`results/p0_retry.csv`](../../results/p0_retry.csv) 和 executor 日志：generation 1 → 2 |
| `DONE` 或 `FAILED` | 发布一次 `EpisodeOutcome`，后续 timer 直接返回，不再发布目标 | `phase.hpp` 的 terminal 说明、`onTimer()` 263–268 行源码确认 |
| IK 抛错 | 当前 phase 不发布无效目标，立即发布 `IK_FAILED`，不增加 retry | [`results/p0_ik_failed.csv`](../../results/p0_ik_failed.csv)，失败阶段为 `PREPLACE` |
| 旧 session/generation/sequence 观测 | 不进入 IK/FSM；看门狗超时后发布 `OBSERVATION_STALE`，关节命令数为 0 | stale probe 实测确认 |
| episode 运行中再次 `start_episode` | 当前回调会立即重置本地状态并发起新的 reset；实测在 `CLOSE` 阶段收到第二次 start 后回到 `HOME`，generation `1 → 2`，接受 generation 2 观测，未等待第一个 episode 结束 | `onStartEpisode()`、`ResetGate` 源码和 `/tmp/p0_start_interrupt_executor.log` |

一个容易忽略的顺序约束是：当前实现会在调用 `step()` 前发布当前 phase 的目标；只有 phase 发生变化时才记录转换 telemetry。P1 的 `EpisodeActions` 必须先用单测固定这个顺序，再决定是否有理由改变它。

### 10.3 实跑基线

| 场景 | 结果 | 关键数据 |
| --- | --- | --- |
| 默认 diff-IK，固定物体，20 次 | **20/20 成功，0 retry** | 最终 box 均值 `(0.497417, 0.295315)m`；放置误差均值 `5.352mm`，最大 `6.078mm`；episode 时长均值 `6.915s` |
| keyframe，固定物体，1 次 | **1/1 成功，0 retry** | 最终 box `(0.433726, 0.309654)m`；CSV 中 Cartesian 诊断为空是既有语义 |
| diff-IK，物体沿 `+y` 偏移 4cm，1 次 | **1/1 成功，0 retry** | 最终 box `(0.497483, 0.294921)m`；相对任务 TCP 放置误差 `5.668mm` |
| keyframe，同一偏移场景，1 次 | **0/1，3 retry** | 最终 `GRASP_EMPTY`；box 仍在 `y≈0.04m`，说明查表目标没有随物体移动 |
| diff-IK，`target.place_x_m:=10.0`，1 次 | **0/1，0 retry** | `PREPLACE` 返回 `IK_FAILED`；模型位置残差约 `9.10m` |
| keyframe，验收中心设为 `(0,0)m`，`max_retries:=1` | **0/1，1 retry** | 最终 `PLACE_MISSED`；reset generation `1 → 2` |
| stale generation probe | **通过保护条件** | outcome=`OBSERVATION_STALE`，关节命令数 `0` |

固定 IK 20 次的完整 CSV 仍保留每个 phase 的持续时间、IK 残差、关节跟踪误差、实际 TCP 误差和最终放置误差；这些量不能互相替代。keyframe 的 `NaN`/空字段表示该模式没有 Cartesian 诊断，不表示误差为零。

### 10.4 机制、权衡与失败模式

P0 采用“正常成功 + 有意失败 + 时序注入”三类基线。只跑 20 次成功 episode 只能证明 happy path 没有立刻坏掉，不能固定 `IK_FAILED`、retry 或旧观测拒绝的行为；因此故意不可达目标、错误验收中心和旧 generation 探针必须一起保留。

当前基线的主要失败模式如下：

| 失败模式 | 已观察现象 | 重构时必须保留的验证 |
| --- | --- | --- |
| 目标算法不泛化 | 偏移物体时 keyframe 进入 `GRASP_EMPTY`，diff-IK 成功 | 两种 `WaypointSource` 的同场景对照 |
| IK 不可达 | `PREPLACE` 产生 `IK_FAILED`，不 retry | 单测断言无无效目标、outcome 只出现一次 |
| 验收目标错误 | `VERIFY` 超时后进入 `RECOVER`，retry 后 `FAILED` | phase/原因/retry/generation 全部可见 |
| 旧观测污染 | 旧 generation 连续到达仍不发布命令，最终 `OBSERVATION_STALE` | stale probe 和 controller 的 generation 单测 |
| 终态重复动作 | 当前 terminal tick 直接返回 | `DONE`/`FAILED` 后动作批次必须为空 |

### 10.5 P0 后仍需在 P1 明确的问题

1. 中途再次 `start_episode` 的“抢占旧 episode”语义已经由 P0 实测确认；P1 仍需用纯控制对象单测固定旧回执不能污染新 episode。
2. 当前 P0 没有统计每个 phase 的关节命令次数；P1 应把“持续目标可重复、reset/outcome 只一次”表达为动作序列断言。
3. 当前 executor 默认单线程；P0 的结果不能证明多线程 executor 下 reset 回调、观测缓存和 tick 安全，暂不把并发重构混入 P1。

### 10.6 排查记录

- `colcon test-result` 报告的 54 个 skipped 全部来自 `cppcheck` 静态检查项，测试本身没有 failure；不能把 skipped 当成运行时测试通过。
- 偏移 keyframe 场景的 runner 退出码为 1，但 CSV 和 executor 日志都正常生成；这是被测 episode 失败，不是测试工具异常。
- 所有实跑结束后重新检查进程列表，没有遗留 bridge 或 executor；因此本次基线没有被重复 publisher 污染。
