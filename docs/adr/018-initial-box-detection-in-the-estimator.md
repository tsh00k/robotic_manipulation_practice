# ADR 018: 估计器里新增“初始 box 检测”路径（深度均值 + 高度带 + 最小面积矩形）

- 状态：已决定
- 日期：2026-10-06
- 范围：`mujoco_perception` 里 box 初始位姿（x、y、yaw）的检测方法、多帧平均的位置、对外消息，以及估计器的输入配对
- 关联：[ADR 016](016-initial-pose-detection-and-gripper-width-carry-check.md)（视觉只做初始位姿）、[ADR 017](017-keep-bin-wall-height-at-12mm.md)、[Week 4.1 Stage 5](../../Job_guides/my_study/week4.1.md#stage-5box-位姿检测落地xyyaw)

## 背景

ADR 016 把视觉限定为初始位姿检测。Week 4.1 Stage 5 在 Stage 3 的同一批 40 个布局上重放了现有检测器：它只接受 28/40（70%），12 个被拒的簇全部是 3D 有向包围盒边长超过 55 mm 的上限（相机从斜前方看，盒子露出顶面加前面，成 L 形，包围盒约 56.6 mm）；它恒输出 yaw = 0，所以抓取朝向误差就是盒子的真实 yaw。Stage 2 测得 yaw 容差为 30°。位置精度两者相当，都远小于 10 mm 的位置容差。

## 决策

1. **新增一条并行的初始检测路径，旧路径保留。** 释放后的核对（VERIFY）仍用旧的 `segmentDepth()` / `estimateBoxPose()` 和 `~/object_pose`，executor 继续读它；新路径发布 `~/initial_box_pose`（`InitialBoxPose`）。新代码在单独的文件里（`depth_window`、`initial_box_detector`、`initial_box_estimator`），不放进 `geometry_pipeline`，也不引入可切换的多条流水线。
2. **检测方法。** 反投影深度 → 取世界高度落在 `[顶面 − 15 mm, 顶面 + 40 mm]` 的像素 → 图像上 8 邻域连通域（`cv::connectedComponentsWithStats`）→ 每块丢掉 z、对 x–y 求最小面积外接矩形（`cv::minAreaRect`）→ 两边都在 40 ± 5 mm 的块才是 box，恰好一个才算检出。yaw 是矩形边的方向，折到 [−45°, 45°)（正方形每 90° 重复）。连通域与矩形用成熟库，不自己实现。
3. **多帧平均放在估计器里，由 `DepthWindow` 完成。** 对最近 N = 10 帧（参数 `initial_box.frames`）做逐像素均值；某像素有效帧不足一半，或有效帧的最大深度与最小深度之差超过 20 mm，则该像素记为无效。窗口在新 session / generation、以及从 ATTACHED 退出时清空；ATTACHED 期间不喂入、不发布。executor 将来读到 MEASURED 就当作可用（锁存在 Stage 7），自己不做平均或质量判断。
4. **估计器不再要求 RGB。** 节点不再订阅 RGB 与彩色相机信息，输入配对只要深度图、深度相机信息、bridge 观测和机器人 TF 的时间戳一致。
5. **消息不带质量分数。** `InitialBoxPose` 只有三态（`WARMING_UP`、`NOT_MEASURED`、`MEASURED`）、原因字符串、窗口里的帧数、位置与 yaw（未测到为 NaN）、被检出的每个块的边长与中心。检测是二值规则（两边都在容差内、恰好一个块），连续的置信度没有可校准的含义。旧消息 `VisionObjectPose` 的字段不动。
6. **单帧上的离群点抑制不采用。** 形态学开运算只把 σ = 2 mm 下 bin 的检出从 1/120 提到 56/120（3×3）或 66/120（5×5），box 的 yaw 误差反而略大；不能替代平均。它是 Stage 6（bin 检测）的候选，不在本阶段。

## 局限

- 多帧平均假设各帧的深度噪声独立；仿真深度本身几乎没有帧间噪声，评测里的噪声是人为叠加的各帧独立高斯。真实传感器的飞点、量化和系统偏差，平均不会消除。
- 窗口里若混入“场景变了”的帧，由节点在已知事件（新 episode、释放）清空，检测器自己不判断场景是否变化；跨度规则只能挡住个别像素在两个表面之间切换。
- 只验证了 HOME 位姿（机械臂在高度带之上）、盒子平放、单个盒子、固定相机；倾斜或竖放的盒子会被当作平放的。
- 窗口在复位后立刻开始装帧，前几帧里盒子还有不到 1 mm 的落稳运动；平放的盒子没有问题，从更高处或倾斜落下的盒子没有测。
- 旧路径对“box 在 bin 里”的检测仍会拒绝（`CANDIDATE_INVALID`），bin 场景下 episode 以 `OBSERVATION_STALE` 结束；这是旧检测器的行为，本 ADR 之前就如此，由 Stage 10、11 处理。
- `VisionObjectPose` 里的 `grasp_state`、`attachment_valid` 没有消费者（executor 读 bridge 的附着状态），但删除会牵动旧 tracker、它的单测和两个离线工具（`tracking_probe.py`、`tracking_replay.py`），不在本阶段做。

## 被否决的做法

- 把新方法塞进现有流水线、让参数决定走哪条：用户不打算实现多种几何流水线，且两者的接受规则不同。
- 自己写连通域与矩形扫描：Stage 3 的原型这么做，C++ 里改用库；两者在同一批帧上逐项一致（最大差 0.0005 mm、0.0000°）。
- 由 executor 做多帧平均：executor 应当假定视觉没问题。
- 为新消息重新定义置信度、残差、内点比：见决策 5。

证据、数字与排查见 Week 4.1 Stage 5。
