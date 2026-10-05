# Week 4.1 学习笔记

> **状态：计划，尚未实施（2026-10-04 重写，2026-10-05 补充 ADR 015）。** 本周以 Stage 5 提交 `e37dcb2` 为代码基线，把原 Stage 6 中混在一起的“场景配置、持续视觉、TCP 推算、掉落/释放状态机”拆成 18 个单一出口的阶段，逐个加回。原 Stage 6 的完整实现归档在分支 `archive/stage6-transport-monitor`（提交 `3587adf`，不合并，只按阶段挑选可移植部分），该提交里同时保存了本文的上一版（以 Stage 6 代码为基线、6 个阶段）。[Week 4 Stage 6](week4.md#stage-6可配置单物体入-bin-与搬运监测) 的笔记保留作为记录，回退经过见其 [6.9](week4.md#69-回退决定与存档)；回退后的设计决策记录在 [ADR 015](../../docs/adr/015-rollback-stage6-perception-side-transport-diagnostics.md)，[ADR 014](../../docs/adr/014-configurable-bin-continuous-transport-monitor.md) 保持历史快照不修改。Week 4.5 继续 [HOLD UP](week4.5.md)，旋转打滑优化暂缓。

原 Stage 6 一次改动了二十余个代码、测试和文档文件，把三类成熟度不同的事绑在一起：场景与实体 bin、搬运期间的视觉辅助估计、掉落/主动 place/稳定判定。第三类在没有可观测证据时就长出了 `TransportMonitor` 四态、`CONFIRMED_DROP` 终止、释放边界和对应的 FSM/outcome 分支，使任何一处出问题都难以定位。用户实测夹持期持续 `MISSING_ROBOT_TRANSFORM` 时，也无法判断是输入、同步还是监测逻辑的问题。本周的方法目标是：**每个阶段只有一个可验证出口；先有可读的观测，再决定是否需要状态。**

## 学习重点范围

目标是**单摄像头、单 box、单 bin 在桌面上的不同位置与旋转，box/bin 实际位姿由视觉获得，复用现有逐阶段离线求解 IK 的 pick and place**。桌面几何、相机标定、机器人模型和 box/bin 尺寸可作先验；CLI 位姿只用于在仿真里生成场景，不能作为 vision 模式检测和任务目标的答案。

搬运期间的 TCP 推算与视觉的同帧比较**只存在于 `mujoco_perception`**，只产生诊断和 WARNING，不修改任务目标、不触发 retry、不停止 FSM、不更新 `T_tcp_box`。掉落检测、主动 place 边界和稳定放置多帧判定本周不实现，是否需要由 Stage 6 的真实偏差数据和 Stage 17 决定。本周不加入多物体、多颜色任务、VLA、数据集导出、MoveIt 或新的运动控制后端。

## 目录

- [1. 基线、回退决定与存档](#1-基线回退决定与存档)
- [2. 分阶段顺序与验证约定](#2-分阶段顺序与验证约定)
- [Stage 1：bridge 可配置场景（默认关闭）](#stage-1bridge-可配置场景默认关闭)
- [Stage 2：附着期视觉持续运行（只读）](#stage-2附着期视觉持续运行只读)
- [Stage 3：TF 同 stamp 合并](#stage-3tf-同-stamp-合并)
- [Stage 4：`predicted_pose` 与 `T_tcp_box`](#stage-4predicted_pose-与-t_tcp_box)
- [Stage 5：离桌顶面准入（证据驱动，可能跳过）](#stage-5离桌顶面准入证据驱动可能跳过)
- [Stage 6：同帧比较与 WARNING](#stage-6同帧比较与-warning)
- [Stage 7：壁高 25 mm 与几何/碰撞检查](#stage-7壁高-25-mm-与几何碰撞检查)
- [Stage 8：`BinModel` 离线拟合](#stage-8binmodel-离线拟合)
- [Stage 9：在线 bin 定位](#stage-9在线-bin-定位)
- [Stage 10：box 与 bin 点云归属分离](#stage-10box-与-bin-点云归属分离)
- [Stage 11：旋转 box 的位姿估计](#stage-11旋转-box-的位姿估计)
- [Stage 12：box/bin 双视觉锁存并发布](#stage-12boxbin-双视觉锁存并发布)
- [Stage 13：place 目标与支撑高度取自锁存 bin](#stage-13place-目标与支撑高度取自锁存-bin)
- [Stage 14：`grasp_yaw` 取自 box 视觉 yaw](#stage-14grasp_yaw-取自-box-视觉-yaw)
- [Stage 15：入 bin 成功判据](#stage-15入-bin-成功判据)
- [Stage 16：附着确认不再读 box 真值](#stage-16附着确认不再读-box-真值)
- [Stage 17：掉落与释放边界（先判断是否需要，可选）](#stage-17掉落与释放边界先判断是否需要可选)
- [Stage 18：随机场景的过程验收](#stage-18随机场景的过程验收)
- [3. 本周最终出口](#3-本周最终出口)
- [4. 悬挂问题与暂缓事项](#4-悬挂问题与暂缓事项)

---

## 1. 基线、回退决定与存档

### 回退决定与三条原则

代码基线是 `e37dcb2`（Stage 5）：默认单盒桌面视觉抓放；附着期 estimator 不处理 RGB-D，executor 复用最后一次合格视觉；bridge 确认并锁存附着；放置目标是 MJCF 中固定的 `place_marker` 位置。

2026-10-04，用户与另一模型评审后决定舍弃原 Stage 6 的大部分改动，原因有三：

1. 第三类内容（掉落、释放边界、稳定放置）没有足够成熟的观测基础，却已经扩展到 executor、FSM、`EpisodeOutcome` 和 probe，状态数量与改动范围同时膨胀。
2. 原 Stage 6 的 20/20 回归和合成故障注入没有证明过程可靠（用户的批评见 [Week 4 6.1](week4.md#61-改动清单与验证结果)）。
3. 原设计把 bin 位姿放进共享的 `scene_config.hpp`，perception 与 executor 一行 include 就能读到答案，与“bin 由视觉获得”的目标冲突。

回退后确立三条设计原则，贯穿全周，决策与“尚未决定”清单见 [ADR 015](../../docs/adr/015-rollback-stage6-perception-side-transport-diagnostics.md)：

| 原则 | 含义 | 检查方式 |
| --- | --- | --- |
| 场景位姿只属于 bridge | `scene.*` 位姿参数只被 bridge 声明；perception、executor 不声明、不读取；共享包只放尺寸/桌面先验，不放位姿加载器 | `ros2 param list` 与源码 grep 审计 |
| TCP 推算与比较只在 perception 实现 | executor 只消费 Stage 5 已有的任务观测和视觉准入结果，不再订阅视觉后另建一份监控 | 否定断言：executor 不引用比较字段 |
| 证据先于状态机 | 比较结果只做诊断；要不要自动终止，等有真实偏差数据再单独设计 | Stage 17 的入口条件 |

### 存档移植清单

存档文件只作参考，在 `main` 上不存在，下表用代码格式而非链接标注。

| 存档中的内容 | 去向 | 阶段 |
| --- | --- | --- |
| `pick_place_scene.xml` 的 bin body（底板 + 四壁，12 mm） | 移植；默认关闭时隐藏且无碰撞；壁高 25 mm 单独做 | 1、7 |
| `mujoco_bridge_node.cpp::configureScene()`，reset 恢复，body/geom 碰撞缓存同步 | 移植，改为 bridge 私有配置 | 1 |
| `scene_config.hpp` 的位姿解析与校验 | 移植到 bridge 私有头文件；尺寸先验另放共享位置 | 1、8 |
| `demo.launch.py::scene_parameters()` 与 CLI 参数 | 移植，只传给 bridge；`scene_enabled` 默认 `false`（存档默认 `true`，会改变默认 demo） | 1 |
| estimator 去掉附着期 `kAttached` 早退 | 移植 | 2 |
| executor `minimum_visual_sequence_` | 移植，按 bridge 释放样本序号守卫 | 2 |
| `geometry_pipeline.cpp` 的离桌顶面 z 分支 | 证据驱动，可能跳过 | 5 |
| PCA 投影界限（盒体对角线）与单测 `RotatedSquareDoesNotFailThePcaProjectionBound` | 移植；先在 `main` 上确认该测试为红 | 11 |
| 5° 步长（18 角度）yaw 搜索 | 移植，补专门单测 | 11 |
| 场景派生的 `support_z_m`、place 目标 | 来源改为**锁存的视觉 bin** | 13 |
| `cartesian_waypoint_source.cpp` 的 `grasp_yaw` | 移植 | 14 |
| `cubeInBin()` | 数学可参考，输入改为锁存的视觉 bin | 15 |
| `binSurface()` 与 perception 的 `in_bin` 局部拟合 | 丢弃：依赖 `scene.bin`，由 Stage 9、10 的视觉 bin 取代 | — |
| `tracking_replay.py` 读取 `scene.json` 作为 perception 参数 | 丢弃：perception 不收 `scene.*`；评测真值另存为评测元数据 | — |
| `world_roi` 放宽、`tracking.min_confidence` 调整 | 丢弃；对应阶段有证据再单独加 | — |
| `TransportMonitor` 四态、`CONFIRMED_DROP`、`transport_state`、`box_estimated` 及 TF、`confirmed_drop` / `placement_valid` 字段、OPEN 释放边界、稳定入 bin 多帧判定、`frozen_object_pose_`、drop 注入 | 丢弃；Stage 17 视证据决定 | 17 |
| `stage6_probe.py` | 不整体移植；各阶段按需写小探针，参考其进程管理思路 | 各阶段 |
| [ADR 014](../../docs/adr/014-configurable-bin-continuous-transport-monitor.md) | 保持历史快照，不修改；被 [ADR 015](../../docs/adr/015-rollback-stage6-perception-side-transport-diagnostics.md) 取代其决策部分，后续决策随各阶段新增 ADR | — |

### 基线代码事实

以下是读 `main` 上的代码得到的静态审阅结论（2026-10-04，未改代码、未运行），决定了各阶段的顺序，**不等于已验证**。

1. [object_pose_estimator_node.cpp](../../src/mujoco_perception/src/object_pose_estimator_node.cpp) 的 `onTf()` 对同一 stamp 用 `tf_frames_[key] = std::move(...)` 覆盖旧集合。基线只有 bridge 一个广播者，不触发；任何新增的同 stamp TF 广播者（原 Stage 6 的 executor `box_estimated`，或将来 perception 自己的 `box_predicted`）都可能让已收到的机器人 link 丢失。因此同步修复要排在任何新广播之前（Stage 3）。
2. `tryProcess()` 在附着期直接返回，所以基线附着期没有视觉输出。executor 在 ATTACHED 时只读 `last_accepted_vision_observation_`，不读新帧——持续视觉进入后不会进入控制；但释放时 executor 清空缓存，并不拒绝“释放前拍摄、释放后才到达”的帧。这类在途帧基线中不存在，持续视觉会引入（Stage 2）。
3. [task_executor_node.cpp](../../src/task_executor/src/task_executor_node.cpp) 的 `vision_cache_` 只有写入、裁剪、清空，没有读取（Stage 2 因同一函数被改动而纯删除）。
4. MJCF 里 bin 的地板上表面在默认位姿为 z=0.227 m，而基线 place 的盒体中心 z=0.24 m（桌面 0.22 + 半高 0.02），盒体底面低于地板上表面约 7 mm（由 MJCF 数值静态推算，未实测）。所以**开启实体 bin 后，在 place 目标改动前（Stage 13）不能验收入 bin 抓放**。
5. e37dcb2 已删除 tracker 的预测分支，`PREDICTED` 枚举存在但没有生产分支。Stage 4 是新写的 TCP 推算，不是恢复；设计出处是 [ADR 010](../../docs/adr/010-grasp-conditioned-object-state.md)，消息契约受 [ADR 011](../../docs/adr/011-vision-object-pose-contract-tightening.md) 约束。
6. bridge 的 `confirmsAttachment()` 仍使用 box 真值 XY 距离与接触物体（[ADR 012](../../docs/adr/012-prelift-attachment-confirmation.md)），所以 Stage 16 之前不能宣称 vision 路径决策无物体位姿 oracle。

### 用户发现与处理阶段

| 用户发现 | 当前事实 | 处理阶段 |
| --- | --- | --- |
| 夹持期持续 `MISSING_ROBOT_TRANSFORM`，视觉 pose 全 0 | 出现在原 Stage 6 代码上，根因未在用户场景确认；基线是否复现未知 | Stage 2 先测分布，Stage 3 复现并修复 |
| 看不到“估计”和“校验”同时存在 | 原 Stage 6 的 `object_pose` 只有视觉，推算在 executor 的另一个话题 | Stage 4、6：同一条 `object_pose` 消息 |
| 四态与 drop/place 边界不在 `object_pose` | 四态已丢弃 | Stage 17 决定是否需要 |
| bin 位姿来自 oracle/配置 | 当前无 bin 检测器 | Stage 8~10、12、13 |
| 成功率替代了过程验证 | 20/20 不证明过程可靠 | 全部阶段：预期轨迹先行 |
| 初始旋转导致打滑 | 位姿可配置不等于抓取稳定 | 暂缓，只记录，不调力/摩擦/速度 |

## 2. 分阶段顺序与验证约定

| 组 | Stage | 单一主要出口 | 本阶段不搭载 |
| --- | --- | --- | --- |
| A 场景与持续视觉输入 | 1 | bridge 可配置 box/bin 场景；默认关闭时与 Stage 5 一致 | perception、executor、入 bin 抓放 |
| | 2 | 附着期视觉持续运行但只读；释放后不接受释放前的在途帧 | 预测、比较、TF 修复 |
| | 3 | 同 stamp 多条 TF 消息合并，机器人 link 不丢 | 缓存容量/时钟调整（除非有实测依据） |
| B 感知侧 TCP 推算 | 4 | `object_pose` 携带 `predicted_pose`；视觉无效时不伪装成零位置 | 比较、WARNING |
| | 5 | 离桌顶面准入（证据驱动，可能跳过） | 旋转、PCA 界限 |
| | 6 | 同帧比较与 WARNING，不改变任务 | 任何 executor 改动 |
| C bin 视觉 | 7 | 壁高 25 mm，几何与碰撞检查 | 感知 |
| | 8 | `BinModel` 离线拟合 | 在线节点 |
| | 9 | 在线 bin 定位，不读场景配置 | box/bin 分离、任务 |
| | 10 | box 与 bin 点云归属分离 | 任务接入 |
| D 任务接入 | 11 | 旋转 box 的位姿估计 | `grasp_yaw` |
| | 12 | box/bin 双视觉锁存并发布 | 改任何目标 |
| | 13 | place 目标与支撑高度取自锁存 bin | `grasp_yaw`、成功判据 |
| | 14 | `grasp_yaw` 取自 box 视觉 yaw | 成功判据、防滑 |
| | 15 | 释放后新视觉 box 在锁存 bin 内才算成功 | 多帧稳定、掉落 |
| E 附着与验收 | 16 | 附着确认不再读 box 真值 | 掉落 |
| | 17 | 掉落/释放边界：先判断是否需要（可选） | 本周主线不实现 |
| | 18 | 随机场景过程验收 | 新功能 |

Stage 1~6 已写出完整骨架（0 一句话总结、1 改动清单、2 机制与权衡、3 预期轨迹与验收、4 失败模式、5 你没问但值得注意的、6 边界与后续）。Stage 7~18 只写出口、范围、验收要点和前置条件，依赖前面阶段的实测证据，**开始前**按同一骨架补全并先写预期轨迹。

**配置 × 阶段的支持边界。** 在对应阶段通过前，下列组合明确**不验收**：

| 配置 | 状态 |
| --- | --- |
| `scene_enabled=false`（默认） | 每个阶段都必须与 Stage 5 基线等价 |
| `scene_enabled=true` + oracle | Stage 13 之后才有完整抓放（需要 bridge 发布标明 oracle 的 bin 位姿） |
| `scene_enabled=true` + vision | Stage 10 前不支持；Stage 13 前只验收感知指标；Stage 15 后才有完整入 bin |

**验证约定。**

- 每个阶段开始前，先写**输入情形 → 预期消息/候选/状态 → 时间约束 → 预期动作 → 否定条件**，固定版本和配置；不能实验失败后改预期来凑通过。
- 有已知失败的先复现失败，修复后跑同一用例，再检查受影响的正常路径。
- build/test 与真实 ROS/MuJoCo 运行都要完成；文档中的计划项不得写成已通过。运行前按 [STUDY_NOTES_GUIDE 3.1](../../STUDY_NOTES_GUIDE.md#31-验证环境卫生两次误判后定下) 确认没有遗留节点，后台节点不经过 `ros2 run`。
- 基线回归：每个阶段出口必须保持 Stage 5 记录的结果——[oracle_attachment_probe.py](../../src/task_executor/test/oracle_attachment_probe.py) 默认配置 20/20 零重试、默认视觉抓放 3/3 零重试（见 [Week 4 5.1](week4.md#51-改动清单与验证结果)）。回归次数不替代过程断言。
- 范围：每个阶段只改上表所列的包；超出范围先改本文，再改代码。
- 逐 episode 按 session/generation/sequence、图像 stamp、结果到达时间和 phase 记录 trace；状态集合不替代顺序与持续时间。相机只有 10 Hz，视觉序号不要求连续，但必须递增且关联同一次图像。
- oracle 仅用于明确标记的对照和离线误差计算，不喂给 vision 决策。
- 每个阶段通过后单独提交（时机由用户决定），使 `main` 上每个提交对应一个出口；新增接口或决策同步 [architecture.md](../../docs/architecture.md) 并新增 ADR；已有 ADR 只做历史快照，不修改，决策变更用新 ADR 引用旧快照。

## Stage 1：bridge 可配置场景（默认关闭）

### 1.0 一句话总结

只让 bridge 能按 CLI 生成不同的 box/bin 初始位姿，并在 reset 时恢复；默认关闭时与 Stage 5 逐字段一致。perception 和 executor 不动，也不声称能入 bin 抓放。

### 1.1 改动清单与验证结果

**计划，结果待实施。** 范围：[pick_place_scene.xml](../../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml)（bin 底板和四壁，默认隐藏且无碰撞，壁高仍 12 mm）、[mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp)（场景配置、`configureScene()`、reset 恢复）、[demo.launch.py](../../src/mujoco_bridge/launch/demo.launch.py)（只把 `scene_*` / `box_*` / `bin_*` 参数传给 bridge）及对应测试。不改 perception、executor 和消息定义。

### 1.2 机制与权衡

- **配置只在 bridge。** `scene.enabled`（默认 `false`）、`scene.box.{x,y,z,roll,pitch,yaw}`、`scene.bin.*` 由 bridge 声明；解析与校验放在 bridge 私有头文件，不放进 `manipulation_interfaces`。共享包一旦带位姿加载器，后续任何节点只需一行 include 就能读到答案，“bin 由视觉获得”就只剩自觉维护。
- **尺寸来源。** 校验需要 box 与 bin 的尺寸。优先从加载的 MJCF 几何读取，避免和 C++ 常量各写一份；若实现时发现读取比常量复杂得多，再改并记录原因。
- **校验规则**（数值继承自存档 [Week 4 6.2](week4.md#62-场景配置与旋转)，作为起点而非依据）：角度为 RzRyRx（rad）；`z` 省略或 `auto` 取桌面上方旋转后最低角点 +1 mm；显式 `z` 穿桌、超出桌面范围、box 中心落入 bin 占地加边距、非有限值、bin 上轴 z 分量 <0.94（约 20°）均拒绝启动并给出具体原因。
- **reset 恢复。** 初始位姿写入 `qpos0` 与所有 keyframe 的 `qpos`，以及 bin body 的位置/四元数。MuJoCo 在编译期缓存 `body_contype/body_conaffinity`，只改 geom 会被 body 粗筛跳过（存档 [Week 4 6.6](week4.md#66-排查记录) 的实测教训），启用/禁用时两层必须同步。
- **默认关闭等价于 Stage 5。** 关闭时 bin 的 geom 透明且 `contype=0`，`place_marker` 保持可见。
- **权衡：** 校验放节点内（任何启动方式都生效）而非 launch 层；不做“每 episode 用服务改配置”，数据生成器可按不同 CLI 重新启动。

### 1.3 预期轨迹与验收

| 输入情形 | 预期 | 否定条件 |
| --- | --- | --- |
| `scene.enabled=false`（默认） | reset 后 `BridgeObservation` 的关节与 box 位姿与 `main` 基线逐字段相同（容差内）；oracle 探针 20/20、默认视觉 3/3，均零重试 | bin 可见或有碰撞；`place_marker` 被隐藏 |
| `enabled=true`，非零 xyz/rpy | reset 后 box 与 bin 位姿等于配置（容差 1e-6）；连续两次 reset 位姿相同；box 落稳后与初始位姿的差单独记录（初始倾斜会落稳） | reset 后漂移，或跨 reset 累积 |
| `z` 省略/`auto` 且有旋转 | 最低角点距桌面约 1 mm | 穿桌或明显悬空 |
| 非法参数（非有限值、出桌、穿桌、重叠、倾斜过大） | bridge 启动失败，日志给出具体原因 | 静默截断或修正 |
| 启用后的碰撞掩码 | body、geom 两层同为 1；禁用时同为 0 | 只同步了一层 |
| 启用后跑 oracle 抓放 | **不验收**：place 目标未改，bin 地板比桌面高约 7 mm（见[基线代码事实](#基线代码事实)第 4 条） | 把这一项写成通过 |

### 1.4 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现或防住 |
| --- | --- | --- |
| keyframe 未同步 | reset 后 box 回到旧位姿 | 连续两次不同配置 reset 的位姿断言 |
| 只更新 geom 不更新 body 缓存 | bin 看得见但接不住 | 沿用 [test_model_consistency.cpp](../../src/mujoco_bridge/test/test_model_consistency.cpp) 的思路加载 MJCF 断言两层掩码 |
| launch 默认值与节点默认值不一致 | 默认 demo 悄悄变了 | `--show-args` 与节点参数 dump 对照；断言 `scene_enabled` 默认 `false` |
| 校验失败只让 bridge 退出 | executor 仍在空转 | 检查 launch 是否整体退出并打印原因 |

### 1.5 你没问但值得注意的

- “与 Stage 5 一致”怎么写成自动断言：对 `BridgeObservation` 做字段级比较，而不是只看成功率（E 类，可测试性）。
- 启动校验失败时错误是否进日志、launch 是否整体退出，而不是留下半个系统（C 类，可观测性）。

### 1.6 本阶段边界与后续

通过后进入 Stage 2。不验收入 bin 抓放，也不声称 bin 能接住 box——后者留给 Stage 7、13。

## Stage 2：附着期视觉持续运行（只读）

### 2.0 一句话总结

让 estimator 在 bridge 附着期继续处理并发布视觉结果；executor 的控制行为保持 Stage 5 不变；只记录附着期的证据分布，并补上“释放前拍摄、释放后才到达”的在途帧守卫。

### 2.1 改动清单与验证结果

**计划，结果待实施。** 范围：perception 去掉 `tryProcess()` 的 `kAttached` 早退（并审阅 ATTACHED→非附着时“estimator 主动休眠”相关的清理逻辑）；executor 在释放后拒绝序号早于首个非附着 bridge 样本的视觉帧，并纯删除无读取的 `vision_cache_`；一个只读探针记录附着期 `evidence_state` / `state_reason` 计数。不改消息定义，不做预测与比较。

### 2.2 机制与权衡

- **控制行为为什么不变：** Stage 5 的 executor 在 ATTACHED 时只读最后一次合格视觉，新帧到达也被忽略。所以持续视觉先作为**只读观察**，不进控制。
- **新风险——在途帧：** estimator 在释放时 reset tracker 并清缓存，但已发布的附着期帧可能仍在传输；executor 释放后清空旧缓存，随后把它当作“释放后的新证据”。`VisionObjectPose.sample_sequence` 与对应 `BridgeObservation.sample_sequence` 同源，所以守卫键用 bridge 的释放样本序号，不依赖墙钟。
- **替代：** 用消息里已有的 `grasp_state` / `attachment_valid` 判断。它们是 bridge 状态的副本，Stage 4 审阅是否保留，不在此新增依赖。
- **算力：** 持续处理在软件渲染下耗时较大（存档 [Week 4 6.7](week4.md#67-你没问但值得注意的) 记录过 p50≈287 ms、p95≈539 ms 的量级，仅作参考，含并发与记录开销）。要记录结果到达延迟与积压。
- 不在本阶段“修” `REJECTED` / `MISSING_ROBOT_TRANSFORM`，只记录。

### 2.3 预期轨迹与验收

| 输入情形 | 预期 | 否定条件 |
| --- | --- | --- |
| 默认 vision 抓放，记录整个 episode | 附着期收到 `sample_sequence` 严格递增的 `VisionObjectPose`；`evidence_state` / `state_reason` 计数写入文档，**不预设应当 `MEASURED`** | 附着期无消息，或序号重复 |
| 同一 episode 的控制 | 阶段序列、重试次数、outcome 与 Stage 5 基线一致 | 附着期视觉改变目标或触发重试 |
| 测试节点发布序号小于释放样本的 `MEASURED` 帧 | 不被当作释放后证据，VERIFY 继续等待更大序号 | 被接受 |
| 序号不小于边界的帧 | 照常处理 | 被误拒 |
| reset 后 | 旧 generation 的帧不进入新 episode，守卫复位 | 守卫状态残留 |
| 附着期证据分布 | 记录 `MEASURED` 比例与拒绝原因分布，据此决定 Stage 3、5 是否需要 | 把分布当作通过标准 |

### 2.4 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现或防住 |
| --- | --- | --- |
| 积压使结果滞后 | 附着期结果到达延迟变大 | trace 中记录图像 stamp 到到达时间 |
| 守卫比较了错误的序号来源 | 在途帧仍被接受 | 测试节点直接发布合成帧，不依赖真实 DDS 时序 |
| 释放瞬间 estimator 清缓存与新帧交错 | 同一帧被重复处理 | 释放附近的 trace 检查 `processed_` 行为 |

### 2.5 你没问但值得注意的

- 附着期是否出现 `MISSING_ROBOT_TRANSFORM`：基线没有额外的 TF 广播者，出现与否都不能推断 Stage 3 的竞争条件不存在（E 类）。
- 在途帧怎么稳定复现：用合成帧而非等真实乱序，否则测试时有时无（E 类）。

### 2.6 本阶段边界与后续

通过后进入 Stage 3。本阶段的证据分布决定 Stage 5 是否需要做。

## Stage 3：TF 同 stamp 合并

### 3.0 一句话总结

让 estimator 对同 stamp 的多条 TF 消息做合并而不是覆盖，使额外的 TF 广播者不会让已收到的机器人 link 丢失。

### 3.1 改动清单与验证结果

**计划，结果待实施。** 范围：estimator 的 `onTf()` / `hasRobotTf()` 及其输入测试；探针内的一个**测试用**同 stamp 额外 TF 广播者（代替原 Stage 6 的 executor `box_estimated`）。不新增生产 TF 广播者；不改缓存容量和等待时间，除非本阶段有实测依据。

### 3.2 机制与权衡

`onTf()` 现在对每个 stamp 直接覆盖集合。机器人齐全的判定应只看必需的动态 link（`dynamic_robot_frames_`），额外 frame 不能清空已收到的 link。

还要区分三件事：自维护的“已收到集合”、tf2 buffer 是否已可查询该 transform、缓存淘汰（默认 30 个 stamp，100 Hz 下约 0.3 s）与 0.5 s 墙钟等待及仿真时钟速度之间的关系。只有发现实际缺失/淘汰的依据后才调缓存；不用 latest TF 或放宽精确 stamp 冒充同帧掩膜。

### 3.3 预期轨迹与验收

| 输入情形 | 预期 | 否定条件 |
| --- | --- | --- |
| 机器人 TF 先到，同 stamp 额外 TF 后到；反向顺序也测 | 两种顺序都能处理同帧，不因广播顺序变成 `MISSING_ROBOT_TRANSFORM` | 任一顺序丢机器人集合 |
| 实际缺一个必需 link，其他输入保留 | 等待后 `REJECTED` / INPUT / `MISSING_ROBOT_TRANSFORM`，不生成有效 pose | 用缺失输入产生 pose |
| 同 stamp 分拆广播、延迟、积压、reset、旧 generation | 原因、stamp、生命周期可追踪；旧数据不进新 episode | 靠重复处理制造新证据 |
| 实际遮挡，TF 齐全 | 不得标成缺 TF | 遮挡被误报为缺 TF |
| 有/无额外广播者的在线对照 | 记录附着期 `MISSING_ROBOT_TRANSFORM` 比例；若基线本来就是 0，如实记录“原路径无故障，本阶段是预防加测试”，**不声称修复了用户症状** | 以单元测试通过代替用户场景的复现 |

### 3.4 失败模式与验证手段

小输入测试覆盖广播顺序与真实缺 link；在线 trace 覆盖真实夹持与额外广播者。录制保留 TF 的消息边界，不把同 stamp 多条消息合并成一条再回放。若在线仍持续失败，按具体缺失 frame 和缓存时间排查，不直接归因于 y 轴不可观测或机械臂遮挡。

### 3.5 你没问但值得注意的

- 自维护集合显示“齐全”时，transform 是否已进入 tf2 buffer——需要一次可控回调顺序实验（E 类）。
- 缓存容量是否会随仿真速度改变拒绝原因——需要积压/时钟速度用例（C 类）。

### 3.6 本阶段边界与后续

出口通过后进入 Stage 4。replay 不能代替完整在线 TF 验证。

## Stage 4：`predicted_pose` 与 `T_tcp_box`

### 4.0 一句话总结

`VisionObjectPose` 新增 TCP 推算位姿，与“当前视觉测量”并存于同一条消息且互不覆盖；视觉无效时 `pose` 不再伪装成零位置。

### 4.1 改动清单与验证结果

**计划，结果待实施。** 范围：[VisionObjectPose.msg](../../src/manipulation_interfaces/msg/VisionObjectPose.msg) 增加 `predicted_pose`、`predicted_valid`、`predicted_reason`、`predicted_init_sequence`；perception 建立/失效 `T_tcp_box` 并发布；审阅所有 `object_pose` 消费者（executor 的 `qualified()` / `poseTransform()`、probe、replay）；对应测试。不含比较字段与 WARNING，不改 executor 行为。

### 4.2 机制与权衡

- **建立：** 附着前取最后一次合格 `MEASURED` 的 `T_world_box(t_m)` 与同一 sample 的 `T_world_tcp(t_m)`（`BridgeObservation.world_to_hand_tcp`），`T_tcp_box = inverse(T_world_tcp(t_m)) × T_world_box(t_m)`。bridge 附着状态变为 ATTACHED 时锁定。所用测量的年龄上限在阶段开始前固定，并要求同 session/generation。
- **“合格”由谁定义：** 基线中质量门（confidence/residual/inlier）在 executor 的 `vision.*` 参数里，perception 不知道。新增 perception 自己的 `prediction.init_*` 参数，launch 用同一组 `vision_*` 参数同时传给两边，避免两份门限分叉；tracker 自身的 `min_confidence`（默认 0.2）与 executor 的 0.5 不一致，存档 [Week 4 6.6](week4.md#66-排查记录) 记录过由此产生的 `MULTIPLE_CANDIDATES`，本阶段不改，只在文档中保持可见。
- **推算：** `predicted_pose(t) = T_world_tcp(t) × T_tcp_box`，`T_world_tcp(t)` 来自与图像同序号的 `BridgeObservation`。
- **失效：** 释放、reset、generation/session 变化，或从未建立。**不自动更新 `T_tcp_box`**——吸收视觉偏差会把滑移当成校正（[ADR 010](../../docs/adr/010-grasp-conditioned-object-state.md) 已列为替代方案）。
- **字段语义：** `pose` 是当前视觉测量，`predicted_pose` 是 TCP 推算，`evidence_state` 只描述视觉证据。附着期视觉无效时 `evidence_state=REJECTED/OCCLUDED` 且 `predicted_valid=true`，“视觉没更新”和“TCP 仍可推算”是两个可区分的事实。`PREDICTED=2` 保持无生产分支（与 Stage 5 一致）；若改成“视觉缺失时发 `PREDICTED`”，则 `pose` 的含义会变成“测量或预测”，违反 [ADR 011](../../docs/adr/011-vision-object-pose-contract-tightening.md) 的一种证据一种含义。该倾向列在 [ADR 015](../../docs/adr/015-rollback-stage6-perception-side-transport-diagnostics.md) 的“尚未决定”中，阶段开始前确认；若最终选择与 ADR 015 方向冲突，新增 ADR，不改 ADR 015。
- **无效 `pose` 的填充：** 倾向非 `MEASURED` 时填 NaN 而不是零；先审计所有消费者是否都先检查 `evidence_state`，任何选择都不得让零值被误当成位置。
- **权衡：** 推算放 perception 而非 executor——perception 已按 `sample_sequence` 与 TCP 精确配对，放 executor 就要缓存配对并产生第二份运输状态（原 Stage 6 的做法）。
- `grasp_state` / `attachment_valid` 是 bridge 状态副本，本阶段只审阅消费者，不删除，结论记入第 4 节。

### 4.3 预期轨迹与验收

| 输入情形 | 预期 | 否定条件 |
| --- | --- | --- |
| 附着前 | `predicted_valid=false`，`predicted_pose` 不是伪值 | 附着前出现推算 |
| 进入 ATTACHED，此前有合格测量 | 首个附着期消息 `predicted_valid=true`，`predicted_init_sequence` 指向建立所用的测量；`predicted(t_m)` 与该测量 pose 一致（单测，按构造成立） | 推算建立用了过期或跨生命周期的测量 |
| 搬运中 TCP 移动 | `predicted_pose` 随 TCP 刚体运动，用合成 TCP 序列断言 | 随视觉变化而变 |
| 附着期视觉 `REJECTED` / `OCCLUDED` | `evidence_state` 如实，`pose` 不伪装为位置，`predicted_valid=true` | 视觉无效时 `pose` 为零位置仍被消费 |
| 附着前从未有合格测量就 ATTACHED | `predicted_valid=false`，`predicted_reason` 可读 | 编造初始变换 |
| 释放、reset、新 generation/session | `T_tcp_box` 清除，旧预测不进入新 episode | 残留 |
| executor 与 Stage 5 对照 | 行为不变；无消费者依赖新字段；`pose` 填充变化不破坏 executor/probe/replay | 消费者因 NaN 出错 |

### 4.4 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现或防住 |
| --- | --- | --- |
| 初始化测量只看到部分盒体，中心偏 | `T_tcp_box` 带固定偏置，后续一直偏 | 偏置无法与“滑移”区分，只能如实记录；Stage 6 的偏差 = 初始化误差 + 滑移 + 视觉误差 |
| TCP 定义或外参错误 | 建立时刻推算与测量自洽，搬运中全程偏 | 自洽单测抓不到；用 oracle 真值**离线**比较搬运中的 `predicted` 误差分布，作为独立评测（oracle 只在评测端） |
| 建立所用测量太旧 | 夹持前盒体或 TCP 已移动 | 年龄上限与“CLOSE 期间两者静止”的假设一起验证 |

### 4.5 你没问但值得注意的

- 自洽性测试的盲区（见上表第二行）——要单独设计一个不依赖构造关系的独立评测（E 类）。
- 用户如何直接读到两个 pose：`ros2 topic echo /object_pose_estimator/object_pose --field predicted_valid` 等（C 类）。

### 4.6 本阶段边界与后续

通过后进入 Stage 5。此时仍没有比较，也没有 WARNING。

## Stage 5：离桌顶面准入（证据驱动，可能跳过）

### 5.0 一句话总结

仅当 Stage 2、4 的记录显示搬运帧大多因几何准入被拒，才放宽：离桌的 box 用可见顶面和已知厚度恢复中心 z。

### 5.1 改动清单与验证结果

**触发条件（写明数字）：** 附着期 `MEASURED` 帧占比低于 50%，且被拒帧的主要原因属于几何准入（`MODEL_EXTENT_MISMATCH` 一类）而不是 TF 或真实遮挡。不满足则跳过并记录。

**计划，结果待实施。** 范围：[geometry_pipeline.cpp](../../src/mujoco_perception/src/geometry_pipeline.cpp) 的 `estimateBoxPose()` 中 `anchor_z_to_plane=false` 分支及测试。存档参考：离桌 z 取 `max_world.z - half_extents.z()`，测试 `ElevatedTopFaceUsesKnownThicknessWithoutTableAnchor`。

### 5.2 机制与权衡

基线有一条有意的测试 `ElevatedFullGeometryDisablesSupportAnchorAndTopOnlyCannotMeasure`：离桌后没有桌面支撑锚，只看到顶面时 z 不可观测，所以不测量（[ADR 013](../../docs/adr/013-partial-cube-geometry-admission.md)）。放宽的前提是“可见面是顶面、厚度已知”，此时 z 来自先验而非测量；若实际可见的是侧面或盒体倾斜，z 会错。需要明确标记 z 来自厚度先验（标记方式在阶段开始前决定，不静默），并新增 ADR 取代 ADR 013 的相应部分。

### 5.3 预期轨迹与验收

| 输入情形 | 预期 | 否定条件 |
| --- | --- | --- |
| 抬起后的顶面点云（合成 + 录制） | `geometry_valid`，z = 顶面 − 半高 | 拒绝已满足假设的顶面 |
| 可见面是侧面或盒体倾斜 | 拒绝或明确标记，不给出确信的 z | 给出确信但错误的 z |
| 桌面上的 box | Stage 5 原有测试仍通过（被明确替换的那一条除外） | 桌面行为变化 |
| 前后对照 | 记录附着期 `MEASURED` 占比 | 只报“更多帧通过” |

### 5.4 失败模式与验证手段

先验假设被违反时静默出错最危险：用倾斜与侧面 fixture 做负向用例，并在 Stage 6 的偏差分布里检查 z 分量是否有系统偏差。

### 5.5 你没问但值得注意的

- z 偏差在 Stage 6 的比较里与“初始化偏置”不可分，需要单独看垂直分量（C 类）。
- 新旧测试的替换关系要显式写出，别让旧测试被静默删除（E 类）。

### 5.6 本阶段边界与后续

跳过或完成后进入 Stage 6。不含旋转与 PCA 界限。

## Stage 6：同帧比较与 WARNING

### 6.0 一句话总结

perception 在视觉为 `MEASURED` 且 `predicted_valid` 的帧上，比较当前视觉位置与同帧 TCP 推算位置；偏差超过阈值只输出 WARNING 和诊断字段，不影响任务。

### 6.1 改动清单与验证结果

**计划，结果待实施。** 范围：`VisionObjectPose` 增加 `comparison_valid`、`comparison_position_error_m`、`comparison_reason`；perception 的比较与限流日志；测试与探针。不改 executor、FSM、`EpisodeOutcome`，不新增话题或状态。

### 6.2 机制与权衡

- **同帧：** 两侧都由同一 `sample_sequence` 派生，不存在“图像 t₀、到达 t₁”的时间配对问题（那是把比较放在 executor 时才需要的缓存）。
- **比较量：** 位置欧氏误差（m），可附带垂直分量；**不比较旋转**，立方体 90° 对称且 `orientation_ambiguous`。
- **无法比较时：** `comparison_valid=false`，误差为 NaN，`comparison_reason` 说明（无预测 / 视觉非 `MEASURED` 等），不填零误差。
- **阈值：** 参数 `comparison.warn_position_m`。阈值由**正常搬运**的偏差分布（p50/p95/max）标定后固定；注入偏离量事先声明，不按注入结果回调阈值。存档的 25 mm 只是继承的假设，没有依据。
- **WARNING：** 每帧独立判断，没有计数、锁存或滞回；日志限流只影响日志，不影响消息字段。
- **明确不做：** 不更新 `T_tcp_box`、不修改 `pose`、不触发 retry/终止、不创建 anomaly/drop 状态。
- **诊断的含义：** 偏差同时包含初始化偏置、真实滑移、视觉误差（部分遮挡造成的中心偏），三者不可分。输出是**一致性诊断**，不是判决。
- **注入手段：** 偏离注入需要一个测试专用开关（`debug.` 前缀、默认关闭、启动日志显式提示）；比较本身另做纯函数单测。合成偏离只证明比较与警告链，不证明真实滑落能被观测到。

### 6.3 预期轨迹与验收

| 输入情形 | 预期 | 否定条件 |
| --- | --- | --- |
| 正常搬运（vision） | 有比较的帧 `comparison_valid=true`，误差分布写入文档；标定后阈值下 WARNING 为 0；任务结果与 Stage 5 基线一致 | 正常搬运 WARNING 洪泛 |
| 视觉 `OCCLUDED` / `REJECTED` 的帧 | `comparison_valid=false`，`predicted_valid=true`，无 WARNING | 用零误差代替无效比较 |
| 注入 40 mm 横向偏离（事先声明） | 对应帧误差约 40 mm，出现 WARNING；阶段序列、重试、outcome 与无注入一致；`predicted_pose` 不变 | 任务状态因 WARNING 改变 |
| 偏离持续多帧 | 每帧 WARNING（日志限流），仍无终止 | 出现任何终止或重试 |
| 恢复正常 | 无 WARNING，无状态残留 | 警告被锁存 |
| 释放后 | 比较停止（`predicted_valid=false`） | 对释放后的帧继续比较 |
| 否定断言 | 话题、消息、日志中不出现 `CONFIRMED_DROP`、`ANOMALY_PENDING`、`transport_state`；executor 源码不引用比较字段 | 以上任一出现 |

### 6.4 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现或防住 |
| --- | --- | --- |
| 阈值过紧 | 正常搬运出现警告洪泛 | 标定用的正常搬运分布 |
| 阈值过松 | 真实偏离不告警 | 注入偏离量固定在先，不回调阈值 |
| 初始化偏置被当作滑移 | 一开始就有恒定偏差 | 同时看偏差随时间的变化而不是单点值 |
| 日志限流掩盖持续偏离 | 日志少而字段仍在偏 | 以消息字段而非日志做断言 |

### 6.5 你没问但值得注意的

- 怎么断言“不改变任务状态”：同一场景有/无注入各跑一次，比较阶段序列和 outcome（E 类）。
- 用户怎么读比较结果：`ros2 topic echo /object_pose_estimator/object_pose --field comparison_position_error_m`（C 类）。

### 6.6 本阶段边界与后续

A、B 两组到此结束。通过后进入 bin 视觉。本阶段不声称能检测真实打滑。

## Stage 7：壁高 25 mm 与几何/碰撞检查

**出口：** bin 四壁从 12 mm 提高到 25 mm，MJCF 与 bridge 校验使用同一尺寸来源，碰撞真实有效。

- **范围：** MJCF 与 bridge 校验；不含感知、任务、力/摩擦/速度。共享尺寸先验头文件到 Stage 8 有第一个消费者时再建，本阶段只保证 MJCF 与 bridge 校验用同一数字来源。
- **验收要点：**
  - 一个纯 MuJoCo 单测（风格同 `test_model_consistency.cpp`）：应用场景配置后把 box 放在 bin 内口上方，`mj_step` 约 1 s，断言 box 停在地板上而不穿透、xy 在内口内；
  - 静态净间隙预算：夹爪张开宽度与指厚度、bin 内口尺寸、25 mm 壁高，算出 PLACE 时指尖到壁的最小间隙并写出数字；调整 PLACE 高度属于 Stage 13；
  - 默认关闭时 Stage 5 基线回归不变。
- **前置：** Stage 1。

## Stage 8：`BinModel` 离线拟合

**出口：** 在已知点云上，bin 的内底面中心、朝向和支撑高度被独立拟合，且不依赖任何场景位姿。

- **范围：** perception 的几何组件与单测；先沿现有 `geometry_pipeline` 提取实际共用部分（同步、robot mask、反投影、桌面过滤、空间候选），`BoxModel` 与 `BinModel` 各自检查几何、残差与覆盖，不预先搭通用多物体框架；共享尺寸先验头文件（只含尺寸与桌面常量）。不含在线节点、话题、任务。
- **机制：** bin 是底板加四壁，不能当实心 OBB 换尺寸拟合。用尺寸先验、底面/壁面法向、矩形开口和表面覆盖估计；矩形 180° 对称按等价姿态评价，不虚报唯一 yaw；顶面或壁面不可见时拒绝或明确歧义。
- **门槛（动代码前固定，失败即记录，不事后放宽）：** 中心误差 ≤8 mm，bin 支撑高度误差 ≤5 mm，对称等价 yaw ≤5°；小倾斜法向角误差单独报告（目标 ≤5°），不可由仅 yaw 的 fixture 宣称通过。继承自原计划，与 Stage 13 的净间隙预算核对。
- **fixture：** (a) 合成表面采样——自洽，只能证明实现了模型，不能证明对渲染的鲁棒；(b) 从 Stage 7 场景录制的真实渲染深度/点云，真值只在评测端使用。覆盖平移、yaw、小倾斜、局部遮挡、点不足、退化。
- **审计：** 拟合函数的输入只有点云、先验尺寸和桌面平面，没有 scene/bin 位姿。
- **前置：** Stage 7。

## Stage 9：在线 bin 定位

**出口：** estimator 在线发布视觉 bin 位姿，结果随图像里真实的 bin 移动，与任何配置无关。

- **范围：** estimator 增加 bin 结果发布；消息形态（复用 `VisionObjectPose` 加 `object_id`，还是独立消息）在阶段开始前决定并写 ADR，不为未来多物体预先泛化。不含 box/bin 分离（Stage 10）和任务接入。
- **验收要点：**
  - 静态场景，bin 换 XY/yaw/小倾斜：输出 `MEASURED`，误差对离线真值（评测端）在 Stage 8 门槛内；
  - 配置无关：`ros2 param list /object_pose_estimator` 无 `scene.*`；源码无 `scene` 引用与真值 TF 查询；bridge 的 bin 在两次启动间移动，结果随之移动；
  - 遮挡、不在视野、点不足：无有效 bin pose，原因可解释，不回退配置位姿或上一生命周期的结果；
  - 实体 bin 存在时 box 估计受到的影响（可能出现 `MULTIPLE_CANDIDATES`）只记录，不在本阶段修。
- **前置：** Stage 8。

## Stage 10：box 与 bin 点云归属分离

**出口：** 识别到 bin 后，按观测到的 bin 几何排除容器点再估计 box；容器不被当作方块，方块也不被当作容器删去。

- **范围：** perception 的候选组织。不含任务接入。
- **验收要点：**
  - `scene_enabled=true` 时 box 的位姿误差对真值（评测端）在门槛内，不因 bin 出现 `MULTIPLE_CANDIDATES`；
  - box 靠近但不接触 bin：两者归属与拟合残差分开；box 在 bin 内：用录制 fixture 覆盖（初始配置禁止重叠，释放后入 bin 的情形由 Stage 15 在线验证）；
  - 完整 episode 不在本阶段验收（place 目标未改）；执行到 LIFT 完成即可，用于确认 pick 部分不受 bin 影响。
- **前置：** Stage 9。

## Stage 11：旋转 box 的位姿估计

**出口：** 绕 z 旋转的 box 被正确测量，yaw 按 90° 对称归一化。

- **范围：** perception 几何。存档的 PCA 投影界限（盒体对角线）与 5° 步长 yaw 搜索。不含 `grasp_yaw`。
- **机制：** 4 cm 方形沿 PCA 斜轴投影可达约 56.6 mm，基线的“边长 + 容差”上界不是旋转不变量，旋转后会误判 `MODEL_EXTENT_MISMATCH`（存档 [Week 4 6.6](week4.md#66-排查记录) 记录）。
- **验收要点：**
  - 先在 `main` 上移植存档单测 `RotatedSquareDoesNotFailThePcaProjectionBound`，**确认它为红**，再修；
  - yaw 搜索补专门单测（存档只覆盖了 PCA 界限与离桌顶面），覆盖仅顶面可见与侧面可见；
  - `orientation_ambiguous` 语义：估计的是 90° 等价类的代表；
  - yaw 误差门槛在阶段开始前固定，建议与 Stage 8 的 5° 对齐。
- **顺序说明：** Stage 9、10 的 fixture 默认 box yaw=0 以避免耦合，但本阶段必须在 Stage 12 之前完成。
- **前置：** Stage 1（需要可配置 box yaw）。

## Stage 12：box/bin 双视觉锁存并发布

**出口：** reset 后等待本 generation 的 box 与 bin 的新鲜合格视觉，检查多帧一致后锁存，并发布锁存结果；不改任何目标。

- **范围：** executor 锁存逻辑与一个只读状态出口（来源、序号、stamp、质量；形态在阶段开始前决定，不是监控器，不含运输状态）。不改 IK 与任务目标。
- **验收要点：**
  - reset → 等待（`target_valid=false`，无默认原点目标）→ 锁存（`target_source=vision`，序号可读）；
  - box 与 bin 不要求同帧，但必须同生命周期、有限年龄、稳定窗口；缺 box 或 bin 时等待并明确失败层，超时给出明确 outcome，不回退配置；
  - 遮挡静态 bin 后不追目标，reset 后必须重新锁存；旧 generation 与过期帧不锁存；
  - executor 参数中没有 `scene.*`。
- **前置：** Stage 9、10、11。

## Stage 13：place 目标与支撑高度取自锁存 bin

**出口：** PREPLACE/PLACE/RETRACT 与支撑高度由锁存的视觉 bin 派生；bin 的真实位置变化，目标随之变化。

- **范围：** executor 配置、Cartesian 几何、FSM 的 place 区域参数（来自同一锁存变换，不新增第二套 task/verify 坐标）；vision 作为 launch 主入口（改变默认 demo，需在文档显式记录）；bridge 发布**标明 oracle** 的 bin 位姿，仅供 oracle 对照路径使用（形态在阶段开始前决定）。成功判据仍沿 Stage 5 的 XY 半径，留给 Stage 15。
- **存档参考：** 场景派生的 `support_z_m` 逻辑，输入来源改为锁存 bin。
- **验收要点：**
  - 移动 bin 的真实 XY/yaw，PLACE 目标随检测变动；
  - 同一 bin 位姿下 oracle 对照路径可达；不可达明确报 `IK_FAILED`，不重置到默认场景隐藏；
  - 壁间隙预算（Stage 7 数字 + Stage 8 定位误差 + 夹爪包络）核对实际通过；
  - world 向下释放，不要求最终 box yaw 等于 bin yaw。
- **前置：** Stage 12。

## Stage 14：`grasp_yaw` 取自 box 视觉 yaw

**出口：** 抓取时的 TCP yaw 与 box 的视觉 yaw（90° 等价类内取最近）对齐。

- **范围：** [cartesian_waypoint_source.cpp](../../src/task_executor/src/cartesian_waypoint_source.cpp) 的几行，存档用 `std::remainder(atan2(R10, R00), π/2)` 归一到 ±45°。
- **验收要点：** [test_cartesian_waypoint_source.cpp](../../src/task_executor/test/test_cartesian_waypoint_source.cpp) 覆盖不同 yaw 与等价类边界；真实运行检查 CLOSE 宽度与附着。**不以成功率为出口**：旋转时打滑暂缓，只记录现象，不调力/摩擦/速度。
- **前置：** Stage 11、13。

## Stage 15：入 bin 成功判据

**出口：** 释放后的新视觉 box 完整落在锁存的视觉 bin 内才算成功，替代 XY 半径判断。

- **范围：** executor/FSM 的 VERIFY 门与包含判定（旋转 box 的角点在内口内且高度接近内底面）。不含多帧稳定、掉落、OPEN 释放边界。
- **机制：** 包含的 margin 与高度容差由 Stage 8 的定位误差预算加 box 估计误差得出，阶段开始前固定；落在不确定区间内不宣称成功。数学参考存档 `cubeInBin()`，输入改为锁存 bin。
- **验收要点：**
  - 正常释放 → 成功；
  - 卡壁、越界（测试发布合成 box pose）→ 不成功；
  - 仅有序号不晚于释放样本的旧帧 → 不成功（Stage 2 的守卫）；
  - 没有任何 `CONFIRMED_DROP` 或额外运输状态。
- **前置：** Stage 13。

## Stage 16：附着确认不再读 box 真值

**出口：** bridge 的附着确认只使用闭爪意图、实际宽度、双指接触与持续时间，box 真值位置不参与判定。

- **范围：** [grasp_criteria.cpp](../../src/mujoco_bridge/src/grasp_criteria.cpp) 的 `confirmsAttachment()` 与对应测试、ADR 012 的更新。仿真接触是机器人反馈的模拟，需明确来源。
- **验收要点：**
  - 空夹（宽度不符）、单指触壁、瞬时双接触 → 不附着，原因可读；
  - 正常夹持 → 附着；改变 box 真值（注入）不改变判定；
  - 双指接触来自 bin 壁而非 box：接触 geom 的身份只在评测端核对。
- **说明：** 本阶段之后才能宣称 vision 路径的决策无物体位姿 oracle；bridge 的附着权威仍是仿真。
- **前置：** Stage 15。

## Stage 17：掉落与释放边界（先判断是否需要，可选）

**出口：** 一份决策：基于 Stage 6 的真实偏差数据，是否需要掉落检测，以及需要什么。

- **入口条件：** Stage 6 已有真实运行的偏差分布，并且用户恢复物理滑落实验；否则本阶段只产出“暂不做”的 ADR。
- **做法：** 先写设计——哪些证据可观测、是否需要状态机、是否自动终止——再实现。存档的 `TransportMonitor`、[Week 4 6.4](week4.md#64-掉落与主动-place-的证据边界) 与 ADR 014 只作设计输入，不作代码来源；旧门限（25/65/45 mm、0.3 s）没有依据。
- **本周主线不实现。**
- **前置：** Stage 6，以及真实偏差数据。

## Stage 18：随机场景的过程验收

**出口：** 用新的随机场景和预先定义的过程断言证明整条链，并明确失败范围，而不是只报 N/N 成功。

- **范围：** 整理现有 probe/fixture 和小规模运行矩阵，不另建一套 recorder。报告版本、seed、实际位置/旋转、初始落稳 pose、独立定位误差、过程断言结果、延迟、outcome；产物放仓库外，文档记录可重现命令与摘要。
- **机制：** 随机不等于桌面所有状态都可抓。声明相机视野、桌内、不重叠、允许倾角、IK/夹爪净间隙等条件，用 CLI 可复现 seed 生成初始位姿；roll/pitch 初始配置与落稳后实测姿态分开报告。存档的 20 个固定配置已用于开发，归为开发回归；新 seed 测试组事先固定，使用后调试须改称回归并建立下一组。
- **最低矩阵：** 默认、不同位置/yaw、小初始倾角、受限边缘；另测缺 TF、遮挡、合成偏离（只验证 WARNING，任务不变）、旧 generation、目标不可见/不可达、释放后不稳定。
- **负向断言：** WARNING 不改变任务状态；不存在 transport/drop 状态；oracle 不进 vision 决策；正常例不得出现系统性 `MISSING_ROBOT_TRANSFORM` 或未经解释的零有效 pose。一个断言失败即过程验收失败，不能以最终 success 抵消。
- **统计：** 所有失败都统计，不筛选不利配置；无效生成、不可观测、感知失败、IK 失败、执行/放置失败分别报告。
- **收尾：** 冻结本周文档、architecture 和 ADR；再决定后续学习周期，Week 4.5 不自动恢复。
- **前置：** Stage 16。

## 3. 本周最终出口

1. box/bin 任务位姿来自单摄像头视觉，配置只在 bridge 里生成场景；允许先验和反馈来源已列明，决策不消费物体真值位姿。
2. 搬运期间的 TCP 推算与同帧比较只存在于 perception，一条 `object_pose` 消息同时携带视觉测量与推算；偏差只产生诊断和 WARNING，没有第二套监控器或运输状态机。
3. 在线同步、持续视觉、同帧比较、入 bin 成功判据均有预先定义且实跑通过的过程断言；Stage 17 给出掉落检测的明确决定，无论做或不做。
4. 新位置/旋转组合可由 CLI/seed 复现，定位误差、过程断言与失败层可读；历史成功率只作对照。

## 4. 悬挂问题与暂缓事项

| 项目 | 当前安排 | 解锁动作 |
| --- | --- | --- |
| 旋转时打滑、夹持力/摩擦/速度控制 | 用户明确暂缓；只记录现象 | 用户恢复后先测物理接触与相对滑移，再单独规划 |
| 掉落检测、释放边界、稳定放置多帧判定（原 Stage 6 四态） | 后置到 Stage 17 的决策，本周主线不实现 | Stage 6 的真实偏差数据 + 物理滑落实验 |
| 真实物理掉落能否持续生成合格视觉 | 只有合成偏离证据；完全遮挡无保证 | 录制完整单摄像头跌落，独立评估可见性与漏检 |
| 任意 roll/pitch 的 6D 恢复、完全遮挡恢复 | 当前形状和单视角不足以宣称通用支持 | 扩大视角/模型前先建立退化 fixture 与误差范围 |
| `grasp_state` / `attachment_valid` 是 bridge 状态副本 | Stage 4 只审阅消费者 | 任何一次修改 `VisionObjectPose` 的阶段后，若无消费者则单独删除 |
| 真实 RGB-D 标定、真机安全停止 | 缺设备和安全参照 | 接设备后单独建立标定和执行边界 |
| 多物体、多颜色、VLA/LeRobot | Week 4.5 HOLD UP | 单物体本周出口通过且用户恢复对应目标后重新排期 |
