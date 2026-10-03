# ADR 011: 收紧视觉物体位姿消息契约

- 状态：已决定
- 日期：2026-10-02
- 范围：`/object_pose_estimator/object_pose`、`VisionObjectPose`
- supersedes: ADR 009 中关于保留旧状态布尔值和拒绝原因别名的原型约定

## 背景

Stage 5 原型同时发布 `evidence_state`、`pose_valid`、`accepted` 和两个原因字段，并把 bridge 的夹爪宽度复制到视觉结果。它们可能描述同一帧的不同方面，却没有明确的权威关系，导致诊断和消费者实现出现歧义。当前接口尚未稳定，因此不需要兼容别名。

## 决策

`VisionObjectPose` 只用 `evidence_state` 表达位姿证据状态：

- `MEASURED`：当前帧有有效视觉测量；
- `PREDICTED`：当前没有直接测量，但历史预测仍在允许范围内；
- `OCCLUDED`：当前没有可用测量，也不能继续预测；
- `REJECTED`：当前输入或候选处理被明确拒绝。

`state_reason` 是视觉原因的唯一字段，`diagnostic_stage` 说明原因所属处理阶段；`grasp_state` 和 `attachment_valid` 独立描述夹持附着。删除 `pose_valid`、`accepted`、`rejection_reason` 和 `gripper_width_m`，不发布别名或新旧并行字段。

bridge 的反馈 topic 是夹爪宽度的唯一权威来源。视觉估计器可以读取该反馈作为 tracker 输入，但 `/object_pose` 不复制它；task executor 需要宽度时直接使用 BridgeObservation。

task executor 根据完整视觉和机器人状态决定任务是否继续。当前准入策略只接受 `evidence_state=MEASURED` 且几何质量达标的样本；这不是把任务决策重新编码成视觉消息布尔值。

## 后果与权衡

单一状态字段使消费者必须明确区分当前测量、短期预测、遮挡和拒绝，减少“可用”与“已接受”的混淆。代价是旧消费者需要同步更新，且预测准入不能靠一个通用布尔值表达。若未来需要和观测绑定的夹爪宽度快照，应设计带采样时间和有效性的独立字段。

## 验证

使用 `rg` 审计发布端、消费者、探针和回放脚本；构建 `manipulation_interfaces`、`mujoco_perception`、`task_executor`，运行 tracker 单元测试和 replay 验证新的诊断字段。`git diff --check` 作为提交前门禁。

2026-10-02 的相关包回归为 504 tests、0 errors、0 failures、71 skipped。`tracking_replay.py` 在 `LIBGL_ALWAYS_SOFTWARE=1` 下回放 54 帧，得到 `MEASURED=10`、`PREDICTED=7`、`OCCLUDED=5`、`REJECTED=32`；无效 TCP 回放为 54 帧全 `REJECTED`，原因均为 `INVALID_ROBOT_STATE`。回放 trace 只读取并记录 `state_reason`、`diagnostic_stage` 及候选诊断字段。
