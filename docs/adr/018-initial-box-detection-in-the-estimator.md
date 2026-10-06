# ADR 018: 估计器里新增并行的初始 box 检测路径

- 状态：已决定
- 日期：2026-10-06
- 范围：`mujoco_perception` 的估计器节点：初始 box 位姿的话题与消息、多帧平均放在哪里、输入配对
- 关联：[ADR 016](016-initial-pose-detection-and-gripper-width-carry-check.md)（视觉只做初始位姿）、[Week 4.1 Stage 5](../../Job_guides/my_study/week4.1.md#stage-5box-位姿检测落地xyyaw)

## 背景

ADR 016 把视觉限定为初始位姿检测。Week 4.1 Stage 5 在 Stage 3 的同一批 40 个布局上重放了现有检测器：它只接受 28/40（70%），12 个被拒的簇全部是 3D 有向包围盒边长超过 55 mm 的上限；它恒输出 yaw = 0，所以抓取朝向误差就是盒子的真实 yaw。Stage 2 测得 yaw 容差为 30°。释放后的核对（VERIFY）仍在用旧检测器。

## 决策

1. **新增一条并行的初始检测路径，旧路径保留。** 新路径发布 `~/initial_box_pose`（新消息 `InitialBoxPose`）；旧的 `~/object_pose` 和 `VisionObjectPose` 一个字段都不改，executor 继续读旧话题。新代码在单独的文件里，不放进 `geometry_pipeline`，也不引入用参数切换走哪条流水线的开关。
2. **多帧平均放在估计器里，不放在 executor。** executor 将来读到 MEASURED 就当作可用（锁存是 Week 4.1 Stage 7），自己不做平均，也不做质量判断。
3. **估计器不再把 RGB 作为输入条件。** 输入配对只要深度图、深度相机信息、bridge 观测和机器人 TF 的时间戳一致；节点不再订阅 RGB 和彩色相机信息。

## 局限

- 旧路径对“box 在 bin 里”的检测仍会拒绝（`CANDIDATE_INVALID`），bin 场景下 episode 以 `OBSERVATION_STALE` 结束；这在本决策之前就如此，由 Week 4.1 Stage 10、11 处理。
- 只在仿真里验证了 HOME 位姿、平放的单个盒子、固定相机。
- 多帧平均假设各帧深度噪声独立；仿真深度本身几乎没有帧间噪声，评测里的噪声是人为叠加的。
- 以后若让 executor 读新话题、或删除旧路径，由新 ADR 记录。

## 被否决的做法

- 把新方法塞进现有流水线、让参数决定走哪条：用户不打算实现多种几何流水线，且两者的接受规则不同。
- 由 executor 做多帧平均：executor 应当假定视觉没问题。

检测方法、阈值、多帧窗口的参数、不设质量字段、不采用开运算、验证数据与排查，都在 Week 4.1 Stage 5（5.2、5.5~5.7），不在这里重复。
