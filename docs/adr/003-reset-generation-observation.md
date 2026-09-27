# ADR 003：用 reset generation 和同一步观测包驱动 episode

## Status

已采用（2026-09-27）。

## Context

executor 原先分别缓存 `/joint_states`、物体 pose、两个无时间戳的接触布尔值，并从 TF buffer 查询 latest TCP。bridge reset 成功后清空这些本地缓存、等待 0.1s 仿真时间，仍不能清空 DDS 队列或 TF 历史。旧样本可能进入 HOME IK seed；接触信号也无法证明属于哪次 reset。仅依赖消息时间戳不够：各话题的接收顺序不同，接触 Bool 没有 stamp，时间相近不等于来自同一物理步。

## Decision

bridge 每次启动生成新的 64 位会话标识，每次成功 reset 增加一个 `generation`。会话标识防止 bridge 重启后 generation 从头计数，与旧进程滞留的样本撞号。保留原有 `~/reset` Trigger 服务，新增 `~/reset_with_generation` 服务返回成功状态、本次会话与 generation。bridge 在同一 `mj_step` 后发布 `~/episode_observation`，其中含会话、generation、单调 `sample_sequence`、带仿真时间戳的 joint state、物体 pose、world-frame TCP 变换和双指接触。既有 `/joint_states`、TF 与 ground-truth 话题继续提供给其他消费者，但 executor 的决策只使用新观测包。

executor 的 `ResetGate` 管理请求、回执、等待观测、ready 和失败状态。只有与本次 reset 回执的会话及 generation 相同且序号严格递增、内容可解析的观测才可进入 IK/FSM。旧会话、旧 generation 或重复序号被忽略；同一会话下更高的 generation 表示 episode 被其他 reset 取代，返回 `RESET_SUPERSEDED`。等待服务、回执或新观测超过 5s 墙钟时间分别报告 `RESET_UNAVAILABLE` 或 `OBSERVATION_STALE`；reset 服务显式失败报告 `RESET_FAILED`。ready 后观测流中断超过 5s 也报告 `OBSERVATION_STALE`。墙钟仅用于故障看门狗，阶段耗时仍按仿真时间计算。

## 备选方案与后果

继续延长固定等待时间无法证明样本来源。给四个旧话题分别加 generation 会改变多个公共消息类型，并仍需处理跨 topic 原子性；同一步观测包使决策输入形成一个明确边界。代价是 bridge 和 executor 增加了一份面向 episode 的共享 ROS 契约，且同一物理量会同时出现在旧话题和新观测包中。以后视觉或真机接入必须明确由谁生成等价的带时序观测，不能把仿真 oracle 直接当作感知协议。单线程 executor 与 bridge 默认互斥回调组仍是当前线程安全前提。
