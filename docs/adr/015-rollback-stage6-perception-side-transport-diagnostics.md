# ADR 015: 回退 Stage 6 的运输监控，改为感知侧只读一致性诊断；场景位姿只属于 bridge

- 状态：已决定（接口细节在对应阶段实施时冻结，见“尚未决定”）
- 日期：2026-10-05
- 范围：场景配置的归属、附着期视觉、TCP 推算与同帧比较的归属、掉落检测的排期
- 取代：[ADR 014](014-configurable-bin-continuous-transport-monitor.md) 的决策部分（共享位姿配置、executor 内的 TransportMonitor 四态、确认掉落终止、释放边界、稳定放置多帧判定）；ADR 014 保持历史快照不修改，其保留项见下文
- 关联：[ADR 010](010-grasp-conditioned-object-state.md)、[ADR 011](011-vision-object-pose-contract-tightening.md)、[ADR 012](012-prelift-attachment-confirmation.md)、[ADR 013](013-partial-cube-geometry-admission.md)；[Week 4.1](../../Job_guides/my_study/week4.1.md)

## 背景

Stage 6 一次实现了三类成熟度不同的内容：可配置 box/bin 场景与实体 bin；附着期持续视觉及固定 TCP→box 推算；掉落、主动 place 释放边界与稳定入 bin 判定。第三类没有可观测证据支撑，却已扩展到 executor、FSM、`EpisodeOutcome` 和 probe，改动与状态数量一起膨胀。收尾时用户报告夹持期持续 `MISSING_ROBOT_TRANSFORM`、在 `object_pose` 里看不到估计与校验并存，当时无法判断问题出在输入、同步还是监测逻辑。既有 20/20 回归与合成故障注入也没有证明过程可靠。

ADR 014 的共享 `SceneConfig` 还把 box/bin 位姿放进了 perception 与 executor 都能 include 的头文件。bin 现在要改由视觉获得，位姿一旦对这两个节点可见，“不读答案”只能靠自觉维护。

2026-10-04，用户与另一模型评审后决定回退，以 e37dcb2（Stage 5）为基线重新分阶段加入。Stage 6 的完整实现归档在分支 `archive/stage6-transport-monitor`（提交 3587adf），不合并，只按阶段挑选可移植部分。

## 决策

1. **场景位姿只属于 bridge。** `scene.*` 的 box/bin 位姿参数只由 bridge 声明和使用；perception、executor 不声明、不读取。共享包只放尺寸与桌面先验，不放位姿加载器。`scene.enabled` 默认 `false`，关闭时行为与 Stage 5 一致。bin 位姿在任务中的来源是视觉；oracle 对照路径需要 bin 位姿时由 bridge 发布并明确标为 oracle。
2. **附着期视觉持续运行，先只读。** estimator 在附着期继续处理并发布；executor 的控制行为不变。持续视觉带来“释放前拍摄、释放后才到达”的在途帧，由 executor 按 bridge 释放样本序号守卫，不依赖墙钟。
3. **TCP 推算与同帧比较只在 `mujoco_perception` 实现。** 同一图像时刻 t：`BridgeObservation` 提供 `T_world_tcp(t)`，视觉提供 box 测量；附着前由合格测量与同帧 TCP 建立 `T_tcp_box`，附着后 `predicted_box(t) = T_world_tcp(t) × T_tcp_box`，与当前视觉测量作同帧比较。`task_executor` 不订阅视觉后自建第二份监控，只消费 Stage 5 已有的任务观测和视觉准入结果。
4. **推算与比较只做诊断。** 用途限于：视觉短暂失效时保留可解释的 box 估计；判断当前视觉是否与 TCP 推算相容；产生 WARNING 和诊断字段。不修改任务目标、不触发 retry、不停止 FSM、不自动更新 `T_tcp_box`（沿用 ADR 010 的“偏差不被吸收进变换”，去掉其“校正”选项）、不创建异常或掉落状态。
5. **视觉测量与 TCP 推算在消息里是两个独立事实。** 沿用单一 `object_pose` 接口：`pose` 只表示当前视觉测量，`evidence_state` 只描述视觉证据，TCP 推算与比较结果用独立字段表达。视觉无效时不用零位置伪装测量，同时允许“推算仍有效”。
6. **丢弃 Stage 6 的运输监控。** 运输四态、`CONFIRMED_DROP` 终止分支、executor 的 `transport_state` 与 `box_estimated` 发布、`confirmed_drop` / `placement_valid` 任务帧字段、OPEN 命令与实际开启宽度组成的释放边界、释放前后迟到证据处理、多帧稳定放置判定、drop 专用故障注入及对应 `EpisodeOutcome` 失败码，都不进入代码基线。
7. **掉落检测后置，且以证据为入口。** 是否需要、需要什么，等有真实偏差数据和物理滑落实验后单独设计（Week 4.1 Stage 17），先有可观测证据再决定状态机，不再先写状态机再找证据。旧门限（25/65/45 mm、0.3 s）没有依据，不作为设计起点。
8. **bin 位姿由视觉获得，不由配置过滤。** Stage 6 的已知 bin 表面过滤和局部支撑拟合依赖配置位姿，不保留；由独立阶段的 bin 检测取代。在此之前，“实体 bin + vision”明确不支持验收。

保留的 ADR 014 内容：bin 为静态底板与四壁、不新增模型自由度、倾斜受限；视觉在附着期持续运行；禁止用真值 box TF 初始化视觉推算；禁止把持续偏离自动融合进变换。

## 尚未决定

以下在对应阶段开始前确认，任何一项的最终选择若与上文方向冲突，新增 ADR，不改本文。

- 附着期视觉无效时，TCP 推算是否占用 `PREDICTED` 证据状态。倾向不占用：否则 `pose` 会变成“测量或预测”，与 ADR 011 的一种证据一种含义冲突。
- 非 `MEASURED` 时 `pose` 填 NaN 还是零，取决于对全部消费者的审计。
- 比较消息的字段名、阈值与标定方式；初始化 `T_tcp_box` 所用测量的年龄上限与质量门（perception 需要与 executor 同源的门限参数）。
- bin 结果的消息形态（复用 `VisionObjectPose` 或独立消息）；锁存结果的发布形态。
- 离桌顶面的 z 恢复是否放宽，放宽后如何标记 z 来自厚度先验，及其与 ADR 013 的关系。

## 权衡与后果

把比较放在 perception 的好处是图像与 TCP 已按 `sample_sequence` 精确配对，不需要 executor 缓存配对，也避免出现“图像 t₀、到达 t₁”的两套时间；代价是 perception 需要读取 `BridgeObservation` 的 TCP，且 `VisionObjectPose` 消息变宽。比较结果只是诊断：偏差同时包含初始化偏置、真实滑移和视觉误差（部分遮挡造成的中心偏），三者不可分，因此它不是判决，也不保证能发现真实滑落。

场景位姿只在 bridge，使 oracle 对照路径需要另有一条明确标注的 bin 位姿出口，而不能让 executor 再收一份 CLI 参数。回退也意味着 Stage 6 已有的实现经验（MuJoCo 的 body/geom 碰撞缓存必须同步、旋转方块的 PCA 投影上界、释放时清缓存会丢迟到证据）只保留在归档分支和 [Week 4 Stage 6](../../Job_guides/my_study/week4.md#stage-6可配置单物体入-bin-与搬运监测) 的记录里，需要在对应阶段重新验证，不能当作已通过。

持续视觉在软件渲染下耗时大，附着期结果的到达延迟与积压需要记录；TF 同 stamp 合并必须先于任何新增 TF 广播者，否则已收到的机器人 link 可能被覆盖（基线只有 bridge 一个广播者，不触发）。

## 验证

逐阶段的预期轨迹、否定条件与基线回归见 [Week 4.1](../../Job_guides/my_study/week4.1.md)。与本决策直接对应的否定断言：话题、消息与日志中不出现 `CONFIRMED_DROP`、`ANOMALY_PENDING`、`transport_state`；executor 源码不引用比较字段；perception 与 executor 的参数列表中没有 `scene.*`；偏差 WARNING 前后阶段序列、重试次数与 outcome 不变；`scene_enabled=false` 时每个阶段都与 Stage 5 基线等价。
