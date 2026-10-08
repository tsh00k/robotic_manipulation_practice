# ADR 021: 实际采用的指令随观测一起发布

- 状态：已决定
- 日期：2026-10-08
- 范围：`manipulation_interfaces/msg/BridgeObservation`；`mujoco_bridge` 的观测发布；数据集的 action
- 关联：[Week 5 Stage 6](../../Job_guides/my_study/week5.md#stage-6满足数据契约-v02-的原始记录)；[ADR 020](020-path-then-time-parameterized-joint-trajectories.md)；数据契约 v0.2（`lerobot_begginer/pi05_remote/docs/dataset_contract.md`）

## 背景

数据契约要求每一帧的 action 是“控制器在那一刻实际采用的关节目标”。自 ADR 020 起，executor 每个阶段发一条带时间的轨迹，bridge 每个物理步按时间插值后写进 MuJoCo 的 `ctrl`；这个值只在 bridge 进程内部，没有任何话题发布。state（`BridgeObservation.joint_state`）与相机图像已经同一物理步打时间戳。

## 决策

`BridgeObservation` 增加两个字段，由 bridge 在发布观测时从 `ctrl` 读出：

- `float64[] arm_command`：本步写入的 7 个手臂位置伺服目标，joint1..joint7，rad；
- `float64 gripper_command_width_m`：夹爪目标的两指总开口，m（由夹爪执行器的 `ctrl` 换算）。

`ctrl` 在 `mj_step` 之前写入（轨迹插值、命令回调），之后不再改动，所以观测里的值就是刚刚跑完的那一步用的目标。

## 理由

- state 与 action 在同一条消息、同一个物理步里，天然对齐，录制与导出不用配对；
- 这是“命令已生效”的直接证据：bridge 实际用了什么，而不是 executor 发了什么（轨迹消息晚到、丢失或被替换时两者不同）。Week 5 Stage 6 在线核对：每个观测的 `arm_command` 都等于录下的某条轨迹的一个采样点，且按发送顺序使用。

## 局限

- 消息变大约 64 字节；所有订阅者（executor、估计器、探针）都要重新编译。
- 真机没有 `ctrl` 可读：对应的是发给 libfranka 的关节位置指令，由我们自己的控制端记录；字段的语义（“这一周期实际下发的目标”）不变。

## 被否决的做法

- **bridge 另发一个话题**（与观测同一时间戳）：不动现有消息，但录制与导出要多配对一个话题，两条消息可能一条丢一条不丢。
- **录制端用 executor 发出的轨迹自己算 action**：算出来的是“发了什么”，不是“实际用了什么”。
