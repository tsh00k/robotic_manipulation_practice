# Week 4.5 学习笔记

> Week 4.5 是 Week 4 视觉主线和后续规划/策略实验之间的接口周。本文按 Stage S~U 记录计划、实现、验证、讲解和悬挂问题。它不承担 VLA/RL/IL 大模型训练，也不把 MoveIt 设为 learned policy 的前置依赖。

## 学习重点范围

本周要把项目整理成一个可以接入多种 motion backend 的实验平台：

```text
observation bundle
        -> motion backend
           ├── scripted / existing task executor
           ├── MoveIt planner
           └── learned policy: IL / RL / VLA / LeRobot
        -> trajectory or action chunk
        -> execution interface
```

learned policy 可以自己承担 motion planning、碰撞规避、关节限位、速度限制和动作时长。MoveIt 保留为传统 backend、专家数据来源和对照实验，不是 learned policy 的必经层。

## 目录

- [1. 从 Week 4 继承的事实](#1-从-week-4-继承的事实)
- [2. 本周 Stage 计划](#2-本周-stage-计划)
  - [2.1 Stage S：策略无关的 observation/action/episode 契约](#21-stage-s策略无关的-observationactionepisode-契约)
  - [2.2 Stage T：逐步数据记录、replay 和 LeRobot adapter](#22-stage-t逐步数据记录replay-和-lerobot-adapter)
  - [2.3 Stage U：多 backend 接入和容器交付](#23-stage-u多-backend-接入和容器交付)
- [3. 训练和推理边界](#3-训练和推理边界)
- [4. 本周验收标准](#4-本周验收标准)
- [5. 失败模式和验证方法](#5-失败模式和验证方法)
- [6. 主动提示的问题](#6-主动提示的问题)
- [7. 悬挂问题和后续顺序](#7-悬挂问题和后续顺序)

## 1. 从 Week 4 继承的事实

| 事实 | 本周处理 |
| --- | --- |
| oracle 和 vision 是 observation source | observation 中必须标记来源，训练和评测不能把 oracle 混入 vision 数据 |
| `BridgeObservation` 已有 session/generation/sequence | 这些字段进入数据记录的 episode identity 和样本排序 |
| `EpisodeController` 管理 reset、过期观测和终止原因 | recorder 记录 reset event、applied action、terminal outcome，而不是只记录最终 CSV |
| 当前执行链主要发布离散关节目标 | action contract 增加 trajectory/action chunk，但保留现有 baseline 适配器 |
| 当前场景没有障碍 | 本周定义障碍字段和 backend 边界，不宣称已经验证 learned policy 的障碍规划 |

## 2. 本周 Stage 计划

### 2.1 Stage S：策略无关的 observation/action/episode 契约

定义一个不绑定 LeRobot、PyTorch 或 MoveIt 的领域契约。

**Observation 至少包含：**

- session、generation、sample sequence、simulation timestamp；
- `q`、`dq`、夹爪状态和 TCP 状态；
- RGB、depth、CameraInfo、相机 frame 和外参版本；
- object pose、置信度、残差和 source（`oracle`/`vision`）；
- task id 或 instruction metadata；
- 可选的障碍几何/场景表示。

**Action 至少支持：**

- joint position 或 joint delta；
- Cartesian pose delta；
- joint trajectory/action chunk；
- gripper command；
- control duration、valid-until timestamp 和 action source。

策略输出的规划责任属于策略 backend。执行层只做消息完整性、NaN/Inf、过期动作和进程级异常检查，不替策略重新规划。

### 2.2 Stage T：逐步数据记录、replay 和 LeRobot adapter

数据记录必须保存观测和**实际执行的动作**，而不是只保存策略名义输出。至少记录：

```text
observation_t
policy_action_t
applied_action_t
timestamp_t / latency_t
phase / reward / terminated / truncated
failure_code / reset_generation
```

先用 rosbag2 或项目内部中间格式保存原始数据，再写 Python 转换器导出到固定版本的 LeRobot dataset。LeRobot 依赖和数据格式版本必须被锁定；C++ bridge 不直接链接 LeRobot Python 包。

replay 需要验证：同一 bag 在相同仿真配置下能重建 observation 顺序、action 时间关系和 episode outcome。它既用于视觉回归，也用于远程训练前的数据检查。

### 2.3 Stage U：多 backend 接入和容器交付

先实现一个 scripted backend 通过统一接口运行，证明策略接口不是只为某个模型定制。随后保留以下 backend 位置：

| Backend | 作用 | 是否 Week 4.5 实现完整版本 |
| --- | --- | --- |
| existing task executor | 固定抓取基线和回归 | 是 |
| MoveIt planner | 传统规划、专家数据、对照实验 | 接口预留；规划本身按原路线后续实现 |
| IL/LeRobot | 读取 checkpoint 并输出 action chunk | 只做 adapter/replay 原型 |
| RL | Gymnasium 风格 reset/step 包装 | 定义接口，可用 scripted policy smoke |
| VLA | 图像/instruction 到动作 | 定义异步接口，不要求本地训练 |

容器交付至少提供：

1. 不依赖宿主 `.bashrc`、pyenv 和完整 home 的 core 镜像。
2. 使用 Distrobox 时指定专用 `--home`，源码用显式 `--volume` 挂载到 `/workspace`。
3. policy 依赖作为可选镜像/profile，不把大模型和训练依赖塞进 core image。
4. README 给出从空容器构建、测试、启动 demo 和导出数据的命令。

## 3. 训练和推理边界

### 3.1 IL

本地生成 scripted/teleoperation demos，远程服务器训练 BC、ACT 或 diffusion policy。训练产物包含数据集版本、代码 commit、容器版本和 checkpoint 配置；本地只需要加载 checkpoint 做 replay 和评测。

### 3.2 RL

先用 oracle state 建立 Gymnasium 风格环境，固定 `reset/step`、reward、terminated/truncated 和 seed。视觉 RL 是后续观测替换，不改变 episode contract。LeRobot 可以作为数据/策略适配层，但 RL 训练框架可以独立选择。

### 3.3 VLA

VLA 的训练和微调默认在远程 GPU 服务器完成。项目本地验证 dataset export、checkpoint metadata、异步 policy gateway 和小规模推理，不承诺本机完成大模型训练。

### 3.4 推理

推理采用闭环 action chunk：

```text
observation -> inference -> short action chunk -> re-observe -> inference
```

闭环重新观测不等于在线梯度更新。低频策略推理和高频仿真/控制分离；策略超时、动作过期或返回非法值时进入明确的失败/停止路径。

## 4. 本周验收标准

1. observation、action、episode contract 写入 `docs/architecture.md` 和 ADR。
2. scripted backend 能通过统一 policy gateway 完成一次 episode。
3. 数据记录同时保存 observation、名义 action 和 applied action，并可 replay。
4. 至少一个样本集可以导出为锁定版本的 LeRobot dataset，且不要求 C++ bridge 安装 Python 模型依赖。
5. policy backend 可以输出 action chunk；gateway 能处理超时、过期、NaN/Inf 和错误维度。
6. learned policy 路径不强制经过 MoveIt；MoveIt 在架构中作为并行 backend 和对照来源。
7. core 容器在专用 home 和显式 workspace mount 下完成 build/test/smoke，不能读取作者个人 home。

## 5. 失败模式和验证方法

| 失败模式 | 后果 | 验证方法 |
| --- | --- | --- |
| oracle pose 混入 vision 数据 | 训练评测虚高 | 检查每个样本的 source 和数据集统计 |
| 名义 action 与 applied action 不同 | 训练标签错误 | 注入限幅/延迟，比较两者并记录修改原因 |
| action chunk 和 observation 时间错位 | policy 看见旧状态 | replay 时断言 sequence、timestamp 和 action validity |
| 推理进程超时 | 旧动作继续执行 | 注入延迟，确认停止/保持策略和 failure code |
| 策略输出越界或 NaN | 执行节点异常 | 单测和进程级故障注入 |
| 容器依赖宿主 home | 换机器无法复现 | 使用专用 home、空 workspace 和最小环境 smoke test |
| 把 MoveIt 当作 learned policy 必经层 | 策略接口被传统规划器限制 | 用 scripted 和 dummy learned backend 分别直连 execution interface |

## 6. 主动提示的问题

1. **可观测性**：策略延迟、action chunk 被执行了多少步、是否被丢弃或替换，是否都能从 episode 数据恢复？
2. **可测试性**：能否用 dummy backend 在不加载模型权重的情况下覆盖 replay、超时和非法动作？
3. **数据语义**：训练标签记录的是 policy 名义输出、经过安全处理的动作，还是 MuJoCo 实际实现的动作？三者是否需要同时保留？
4. **泛化边界**：障碍几何进入 observation 后，如何区分模型规划失败、感知失败和执行失败？

## 7. 悬挂问题和后续顺序

- `PolicyObservation` 是 ROS message、多个标准 topic 的时间同步 bundle，还是 Python-side dataclass，需要 Stage S 结合图像带宽决定。
- LeRobot dataset 的具体 schema 和版本需要在 Stage T 实际安装后冻结，不能提前依赖未验证的字段名称。
- 障碍场景要在 learned policy 真正接入前加入，还是与 MoveIt 场景并行加入，属于后续实验计划，不在 Week 4.5 假设中解决。
- 远程推理是否可用、模型是否必须量化以及 GPU 驱动兼容性，需要等第一个实际 checkpoint 后评估。

后续建议顺序：先完成 IL 数据链和一个小模型，再做 RL 环境包装，最后选择一个 VLA adapter。MoveIt 和 learned policy 分别作为 backend 做对照，不建立硬依赖。

