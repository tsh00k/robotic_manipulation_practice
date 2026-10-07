# Week 5 学习笔记

> **状态（2026-10-07）：进行中，Stage 1、2 已完成。** 本周承接 [Week 4.1](week4.1.md)（视觉初始位姿 + 离线 IK 的 pick-and-place，留出集验收 vision 39/40、oracle 40/40）。Week 4.5 的旧计划（多物体、多颜色）已于 2026-10-07 删除，原文见提交 `60293a2`。

## 学习重点范围

本周三个目标：

1. **更鲁棒、更稳定的抓取，更流畅的 demo**：去掉看得见的停顿和跳变，解决搬运中的打滑，并把运动约束到真实 Panda 能执行的范围内。
2. **数据质量足以给 VLA 用**：让仿真产出满足既有数据契约（`lerobot_begginer/pi05_remote/docs/dataset_contract.md`，本周改为 v0.2）的原始记录。**数据导出程序本身不在本周做。**
3. **README 是给别人看的技术报告，architecture 是给开发者看的最终细节。**

方法沿用 Week 4.1：每个阶段单一出口；先测量再改；验收规则在测量前写死；轻阶段 5 块笔记、重阶段完整骨架；提交前先问用户。

## 目录

- [1. 问题清单](#1-问题清单)
- [2. 关于运动的讨论：为什么要轨迹插值](#2-关于运动的讨论为什么要轨迹插值)
- [3. VLA 数据契约的审阅与差距](#3-vla-数据契约的审阅与差距)
- [4. 阶段计划](#4-阶段计划)
- [Stage 1：基线测量](#stage-1基线测量)
- [Stage 2：统一的新启动姿态兼 HOME](#stage-2统一的新启动姿态兼-home)
- [5. 本周最终出口](#5-本周最终出口)
- [6. 悬挂与暂缓](#6-悬挂与暂缓)

---

## 1. 问题清单

**用户在 demo 里发现的（2026-10-06）：**

| 编号 | 现象 | 已知事实 |
| --- | --- | --- |
| U1 | 开始时视觉识别停顿明显 | 窗口要装满 10 帧（相机 10 Hz，0.9 s 仿真）再连续 5 条一致才锁存；Week 4.1 Stage 7 实测约 3.4 s 墙钟。仿真 RTF 约 0.6 |
| U2 | 夹住盒子后停顿过长 | `fsm.close_settle_s` = 2.0 s：CLOSE 固定等满 2 s 才 LIFT；附着在合拢后约 0.25 s 就确认了（Week 4.1 Stage 13），中间约 1.7 s 是白等 |
| U3 | 搬运中打滑，盒子整体相对 TCP 偏移 | 实测见第 2 节：盒子在 TCP 坐标系里累计偏 16 mm、转 15°，滑动出现在高加速度段 |
| U4 | 启动后的初始状态与 start episode 后的状态之间有跳变 | （写计划时以为 bridge 启动用预设状态 `home`；Stage 2 实测是启动时不加载任何预设状态，见 2.1）executor 的 HOME 是 Cartesian 目标 (0.5545, 0, 0.5211) 经 IK 求得；HOME 时手臂还挡住相机看 bin 远侧（Week 4.1 14.6，用户用 rqt 证实） |
| U5 | 纯色的 box 和 bin 不好看 | 放到最后 |
| U6 | architecture.md 要成为给开发者的最终细节，结构简洁准确 | 放到最后 |
| U7 | README.md 还不够好 | 现在只有 18 行，停在 Week 3 的描述 |

**补充的（用户确认全部要做）：**

| 编号 | 问题 | 为什么重要 |
| --- | --- | --- |
| C1 | **关节命令是阶跃目标，没有轨迹插值**：bridge 只取 `JointTrajectory.points[0]`，PD 伺服以力矩上限冲向目标 | 速度超真实 Panda 限值约 2 倍、加速度约 3~7 倍（第 2 节）；打滑出现在加速度峰值处；VLA 的 action 是阶跃信号（第 3 节） |
| C2 | **VERIFY 时手挡住盒子**（Week 4.1 Stage 14 布局 16） | 与 U4 同源，改 HOME 和 RETRACT 时一起解决 |
| C3 | **RGB 在传输上丢帧，原因从没查过**（Week 4.1 Stage 3、5：只到达 16%~34%） | Week 4.1 因估计器不用 RGB 而绕开了；VLA 的主输入就是 RGB |
| C4 | **仿真 RTF 约 0.6** | demo 慢于真实时间 |
| C5 | **episode 结果信息不干净**：成功的 outcome 里残留 `VISION_REJECTED:…`（Stage 14：39 个里 7 个）；开度告警只保留最后一次尝试；VERIFY 看不见盒子被报成 `OBSERVATION_STALE`、执行层 | 挑选 VLA 训练数据（只要干净的成功）时会误判 |
| C6 | **旧检测路径还在**（`~/object_pose`、`VisionObjectPose`、三个已不起作用的 `vision.*` 参数） | 估计器每帧白算一遍旧流水线；architecture 越写越乱 |
| C7 | **Week 4.1 收尾没做完**：14.6 的“推断”改为“已由 rqt 证实”；补“视觉测试的 bin 都在 y ±0.22 内，Stage 14 的通过只在这个范围成立”；改 HOME 后扫 bin 的可见范围并重跑 HELD-A 回归 | 不做的话 Week 4.1 有一处结论写错 |
| C8 | 工具转角在 ±45° 处会在两个方向之间切换（Week 4.1 悬挂清单） | 同一布局两次的手臂动作可能完全不同，数据里混进无意义的分叉 |

---

## 2. 关于运动的讨论：为什么要轨迹插值

> 用户追问（2026-10-06）：“轨迹插值？我目前在 demo 里没有看到平滑轨迹的必要，似乎也没有抖动什么的？你能通过测量陈述其必要性吗？”

### 2.1 测量（一个 oracle episode，有 bin 的场景，2026-10-06）

**命令是阶跃的。** 一个 episode 只有 10 个不同的关节目标；相邻两个目标之间单个关节最多跳 0.60 rad。

**速度、加速度远超真实 Panda 的限值：**

| | 仿真实测峰值 | 真实 Panda 限值 |
| --- | --- | --- |
| 关节速度 | joint4 **4.54** rad/s，joint7 3.30，joint2 2.88，joint6 2.60 | joint1–4 **2.175**，joint5–7 **2.61** rad/s（libfranka `rate_limiting.h`） |
| 关节加速度 | joint4 **99** rad/s²，joint2 56，joint3 41 | 约 7.5~20 rad/s²（FER 限值表；这组数字来自记忆，没有在搜索结果里证实，Stage 3 开始前要查原表） |
| TCP 速度 | **2.3** m/s（夹着盒子时也是 2.3） | 1.7 m/s（旧 FER 表）或 2.0 m/s（当前 libfranka） |
| TCP 加速度 | **37** m/s²（夹着盒子时 33） | **13** m/s²（libfranka） |

真实 Panda 任何一项超限，控制器会立即中止运动（FCI 文档）。**所以现在的运动在真机上执行不了**，这一条与“看起来抖不抖”无关。

**打滑出现在高加速度段**（夹着盒子时盒子相对 TCP 的位移）：

| 时刻（附着后） | 盒子相对 TCP 的累计位移 / 转角 | 当时 TCP |
| --- | --- | --- |
| 0~1.74 s（LIFT） | 0.10 mm / 0.13° | |
| 1.83 s、2.38 s | 10 ms 内各滑 0.5~0.7 mm | 加速度 18、32 m/s² |
| 3.48 s | 累计 16 mm / 15° | |

**有一处不能只归因于加速度：** 约 3.47 s 时盒子仍在滑（每 10 ms 1~6 mm），而 TCP 几乎静止（0.005 m/s），更像夹爪开始张开或盒子已脱离下落。所以“打滑全由加速度造成”**没有被证明**，只说明加速度峰值处有滑动；Stage 1 要按阶段切开看。

**结论：** 插值的依据是**真机可执行性**和**打滑**，“demo 更流畅”只是附带效果。

### 2.2 为什么位置伺服也会超速

> 用户追问（2026-10-07）：“我印象中 mjcf 里是包含了速度和加速度上限的，既然我们只是位置控制，为什么 PD 伺服还会超过？”

**MJCF 里没有速度、加速度上限。** `panda.xml` 每个关节和执行器只有：

| 项 | 值 | 意义 |
| --- | --- | --- |
| `range` | 如 joint4 −3.07~−0.07 rad | 位置极限 |
| `ctrlrange` | ±2.8973 | 位置**目标**的允许范围 |
| `forcerange` | joint1–4 ±87 N·m，joint5–7 ±12 N·m | 力矩上限 |
| `damping` / `armature` | 1 / 0.1 | 关节阻尼、等效转子惯量 |
| `gainprm` / `biasprm` | joint4：kp 3500，kd 350 | PD 增益 |

MuJoCo 的关节没有“速度上限”这一属性。执行器是 `biastype="affine"` 的位置伺服，每步输出 τ = kp·(q_target − q) − kd·q̇，再截到 forcerange。一次跳 0.6 rad 的目标，joint4 的初始力矩是 3500 × 0.6 = 2100 N·m，被截到 87 N·m，**整个过渡都以最大力矩推**；能多快只由力矩上限、惯量和阻尼决定。前臂加手的惯量小，于是加速度到 99 rad/s²。（这是粗估，没有做实验分解。）

**位置伺服只管“去哪”，不管“多快”**，速度是增益、力矩上限和惯量的副产品。真实 Panda 不会这样，是因为它的控制器在 1 kHz 上检查每个周期的速度、加速度、加加速度，超限就中止；我们的仿真里没有这一层。

### 2.3 为什么插值能解决，约束 IK 为什么不行

> 用户追问（2026-10-07）：“为什么插值就可以缓解？能不能把当前的 Jacobian 利用带约束的最小二乘求解？但我们做的是位置控制，约束了速度和加速度怎么影响求解结果呢？”

插值管的是“目标随时间怎么变”，IK 管的是“目标是什么”，是两件事：

```
现在：  每个阶段一次 IK 求出 q_goal  ->  把 q_goal 直接当伺服目标（阶跃）
插值：  每个阶段一次 IK 求出 q_goal
        ->  在 q_current 到 q_goal 之间生成满足速度、加速度上限的 q_ref(t)
        ->  每个物理步把 q_ref(t) 当伺服目标
```

伺服始终只追“眼前”的目标，误差小、力矩小，实际速度和加速度跟着参考轨迹走，而参考轨迹本身满足限值。这是位置控制下限速的标准做法（真实 Panda、ros2_control 的 `joint_trajectory_controller` 都是这样）。

**约束 IK 对这件事没有作用，用户的直觉是对的：** 现在的求解器已有 `maximum_joint_step = 0.12 rad`，但它只约束**迭代**怎么收敛到 q_goal，最后给伺服的仍是同一个终点。IK 是离线求终点，加速度约束放进去没有意义。约束 IK 真正有用的是**在线速度级控制**（每个周期解 q̇ = J⁺v，把 q̇、q̈ 约束放进 QP），那等于把 executor 从“每阶段一个目标”改成“每周期一个速度”，改动很大。

**选择插值，代价要提前处理：** 关节空间插值时 TCP 走的不是直线，PREGRASP 到 GRASP 的下降可能偏离竖直而碰到盒子。Stage 3 要核对 TCP 偏离直线多少；需要时对“接近盒子”这类短段改为笛卡尔直线插值（每个插值点做一次 IK）。

**插值放在哪：** 倾向 executor 发带时间的多点轨迹、由 bridge 插值执行。理由：更接近真实控制器的接口；`pi05_remote_local_plan.md` 第 47 行指出 bridge 只执行 `points[0]`，VLA 推理端需要一个能按时间执行动作块的执行器，这个改动同时满足它。

---

## 3. VLA 数据契约的审阅与差距

> 用户追问（2026-10-07）：“你是否审阅过 VLA 的数据契约从 VLA 角度来看是否合理？以及我们当前的系统离这个契约有多大的 gap？”

**契约在别处已经定义**：`lerobot_begginer/pi05_remote/docs/dataset_contract.md`（数据格式）与本仓库 `docs/plans/pi05_remote_local_plan.md`（远程训练与本地仿真的整体计划，P1 是契约，P2 是采集）。最初我说“沿用”，**其实没有审阅过**；下面是审阅。

### 3.1 从 VLA 角度看契约本身

| 项 | 契约 v0.1 | 看法 |
| --- | --- | --- |
| action = 绝对关节目标 | 7 关节 + 夹爪宽度 | 合理，pi0.5 支持。但**契约没有规定 action 在时间上的形态**：按现在的系统，一个 episode 只有 10 个不同目标，大部分帧重复同一个值，是阶跃信号，模型要学的变成“什么时候跳”。这是本周改契约的原因 |
| 10 Hz | 一帧 0.1 s | 合理。现在关节命令以 20 Hz 发、目标每阶段才变一次，“10 Hz 的 action”只是对阶跃信号采样 |
| state 8 维 | 关节角 + 夹爪宽度 | 合理，pi0.5 默认不用速度；原始记录里应保留关节速度 |
| 深度 | float32，0 表示无效 | 保留合理；我们用 NaN 表示无效，导出时要转换 |
| 单个固定相机 | `table_rgb` | pi05 计划已写明“不能伪造腕部视角”；base checkpoint 期望的相机数要在 R1 核对，是已知风险 |
| 夹爪宽度（m） | 0~0.08 | 合理；但我们的夹爪是全开/全合两态，action 里只有 0.08 和 0 两个值 |
| task 文本 | 每个 episode 一个 | 合理；只有一个任务时语言不起作用（pi05 计划第 7 节已指出） |

### 3.2 契约改为 v0.2（2026-10-07，用户同意）

改动三处（v0.1 原文曾备份在 `/tmp`，2026-10-07 `/tmp` 被清空时丢失；契约所在目录不在版本控制里，此后过程文件放 `.claude/artifacts/`）：

- **action 的时间形态：** frame t 的 action 是控制器在那一刻实际采用的关节目标，来自满足速度、加速度限值的时间参数化轨迹；相邻帧平滑变化，不是“每阶段一个目标保持后跳变”。导出器拒绝相邻两帧之间任一臂关节目标变化超过“速度上限 × 帧间隔”的记录（10 Hz 时 joint1–4 0.2175 rad、joint5–7 0.261 rad）。夹爪豁免，只取 0.08 和 0.0。
- **无效深度：** 仿真用 NaN 表示，导出时把 NaN、Inf、非正值换成 0.0。
- **版本说明。**

### 3.3 当前系统离契约的差距

| 契约要求 | 现状 | 差距 |
| --- | --- | --- |
| RGB 每帧都有 | 只到达 16%~34%，原因未查 | **大** |
| action 平滑（v0.2） | 阶跃 | **大**（Stage 3 解决） |
| 原始记录 | 没有 recorder；只有 `record_demo_bag.sh`（`-a` 全话题，已知会漏话题） | **大** |
| action 是“实际采用的控制目标” | executor 每 50 ms 发一次，bridge 只取 `points[0]`，没有回执 | **中**：pi05 计划第 178 行要求命令生效证据 |
| episode 元数据（任务、物体、目标、种子） | 布局在探针里有，没写进任何记录；outcome 有残留原因（C5） | **中** |
| RGB、深度、state 同步 | bridge 的观测与相机同一物理步打戳，已验证 | 小 |
| 无 NaN/Inf | 深度用 NaN | 小，导出时处理 |
| 只用成功示范 | 成功判据可信（Week 4.1 Stage 11、14） | 小 |

**判断：** 契约格式合理，缺的是 action 的时间形态（已补）。系统离它最大的差距是 **RGB 丢帧、action 阶跃、没有可靠的记录与命令生效证据**。导出程序不在本周做，但这三条是它的前提，本周解决。

---

## 4. 阶段计划

**顺序的理由：** 先测量，再改会影响后面所有数据的起点（HOME）和运动（插值），再改抓取，然后是数据前提、清理，最后外观与文档。

| Stage | 分量 | 单一出口 | 对应 |
| --- | --- | --- | --- |
| 1 | 轻 | **基线测量**：一次 episode 的时间线（每个阶段多长、哪些是等待）；RTF；按阶段切开的打滑（搬运、张开、下落分开）；相机各话题的到达率 | U1、U2、U3、C3、C4 |
| 2 | 重 | **统一的新启动姿态兼 HOME**：bridge 启动即在该姿态，executor 的 HOME 就是它，start episode 不再跳；新 HOME 与 RETRACT 不挡相机看 bin 与 VERIFY 时的盒子；扫 bin 的可见范围；HELD-A 回归；Week 4.1 收尾 | U4、C2、C7 |
| 3 | 重 | **时间参数化的关节轨迹**：executor 发带时间的多点轨迹，bridge 按时间插值执行；速度、加速度在真实 Panda 限值内（先查 FER 加速度原表）；核对 TCP 偏离直线 | C1 |
| 4 | 重 | **抓取稳定、不打滑**：按 Stage 1 的切分找剩下的原因（夹持力、摩擦、抓取高度、张开时机），改了再量；限值在开始前写死 | U3 |
| 5 | 轻 | **去掉不必要的等待**：CLOSE 等附着确认而不是固定 2 s；锁存与 VERIFY 的等待按测量缩短；HELD-A 回归不退步 | U1、U2 |
| 6 | 重 | **满足数据契约 v0.2 的原始记录**（不写导出程序）：RGB 不丢帧、bridge 给出命令生效证据、记录器录全所需话题、episode 元数据干净；探针按契约核对 | C3、C5、C8 |
| 7 | 轻 | **清理旧检测路径** | C6 |
| 8 | 轻 | **box 与 bin 的纹理**（改外观后要重测检测器；深度不受影响，但要核对） | U5 |
| 9 | 轻 | **architecture.md 重写为开发者文档** | U6 |
| 10 | 轻 | **README.md 写成技术报告** | U7 |

C4（RTF）在 Stage 1 测量后决定要不要处理、放在哪个阶段。每个阶段开始前再细化预期、出口断言与不做的事。

**已定的：** C1~C8 全部做；启动姿态与 HOME 统一到一个新的姿态（选择标准是不挡相机）；插值由 bridge 按时间执行；数据契约改为 v0.2。

---

## Stage 1：基线测量

**状态（2026-10-07）：已完成（提交 `ef9608a`）。** 1.1 是测量之前写定的，没有改动；有效性检查 V1~V3 全部满足。**分量：轻。** 本阶段只测不改：Stage 2 起会改 HOME、运动和等待，这些数字是它们的“改之前”。

### 1.0 一句话总结

一个 episode 约 6.9 s 仿真（oracle）/ 7.8 s（vision），按 RTF 0.58 是 12~13.5 s 墙钟；其中**机械臂不动、纯粹在等的时间约占 40%（oracle）/ 47%（vision）**，最大的一块是 CLOSE 里附着之后多等的 1.7 s。打滑**全部发生在搬运期**（LIFT、PREPLACE、PLACE），到 PLACE 结束时盒子在 TCP 坐标系里偏 7~16 mm、转 5~41°，而夹爪开度一直在 37.6~40.5 mm，所以 Week 4.1 的开度窗口看不见它。RGB 在探针端也只到达 22~37%，深度和 camera_info 99~100%，丢帧发生在估计器之前。

### 1.1 测什么、怎么测（测量前写定）

**运行：** 有 bin 的场景，Week 4.1 Stage 6 起用的随机布局生成器（种子 20261008）的前 3 个布局，oracle 与 vision 各跑一遍，每个布局起一套新的 bridge + 估计器 + executor、跑一个 episode。新命令 `initial_box_probe.py timeline`。仿真是确定性的，所以不重复跑同一个布局。

| 编号 | 测量 | 怎么得到 |
| --- | --- | --- |
| M1 | **时间线：** 每个阶段的仿真时长，以及其中多少是“机械臂已经到位、在等” | 阶段时长取自 `EpisodeOutcome.phase_durations_s`，起点是本 generation 的第一个 bridge 样本（控制器在 reset 响应时开始给 HOME 计时）。“到位”用 FSM 自己的判据：各关节与该阶段目标差 < 0.05 rad 且速度 < 0.05 rad/s；目标取自探针收到的关节命令 |
| M2 | **开始前的停顿：** 从 reset 到第一条关节命令（vision 是等锁存）；以及从发出 start 到第一条命令的墙钟时间 | 第一条关节命令的接收时刻，换算到仿真时间用它之前最近的一个 bridge 样本 |
| M3 | **CLOSE 与 VERIFY 里的等待：** 合拢开始到附着、附着到 LIFT；释放到第一条新测量（vision）、VERIFY 开始到 DONE | bridge 的附着状态与估计器消息的时间戳 |
| M4 | **按阶段切开的打滑：** 附着后盒子在 TCP 坐标系里的位移与转角，每个阶段（CLOSE 剩余、LIFT、PREPLACE、PLACE、OPEN）的增量，以及该阶段的 TCP 峰值加速度与开度范围 | bridge 的盒子真值与 TCP（评测端）；OPEN 阶段一直算到盒子离开手 |
| M5 | **相机话题到达率：** RGB、深度、两个 camera_info 在 episode 期间到达探针的条数，除以“仿真时长 × 10 Hz” | 探针用与估计器相同的 SensorDataQoS（best effort）订阅 |
| M6 | **RTF：** 整个 episode 的仿真时长 / 墙钟时长 | 同上 |

**有效性检查（测量本身对不对，不是验收）：** V1 阶段时长之和 + 起点，与 outcome 发出时的仿真时间相差 ≤ 0.1 s；V2 附着时刻落在 CLOSE 阶段内；V3 episode 没有重试（有重试则该行单独标出，不混进统计）。

**没有验收门槛。** 结果用来给 Stage 2~6 定限值。

### 1.2 改动

只加了测量工具：[initial_box_probe.py](../../src/mujoco_perception/test/initial_box_probe.py) 的 `timeline` 命令。产品代码没有改。

```bash
ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_perception/test/initial_box_probe.py timeline --source oracle --count 3
ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_perception/test/initial_box_probe.py timeline --source vision --count 3
```

### 1.3 结果

**有效性：** V1 阶段时长之和与 outcome 时刻相差 +0.008~+0.014 s（≤ 0.1 s）；V2 附着都落在 CLOSE 内；V3 6 个 episode 都没有重试。

**M1 时间线（仿真秒；6 个 episode 的范围）。** “到位后在等” = 阶段时长 − 机械臂到达该阶段目标的时刻。

| 阶段 | 时长 | 到位后在等 | 这段时间里发生了什么 |
| --- | --- | --- | --- |
| HOME | oracle 0.53~0.55；vision 1.42~1.45 | 全部 | 机械臂已经在 HOME，不动；oracle 等的是最小 settle 0.5 s，vision 另加等锁存约 0.9 s |
| PREGRASP | 0.54~0.72 | 0.01~0.04 | 运动 |
| GRASP | 0.50~0.58 | 0.01~0.06 | 运动 |
| CLOSE | 2.00~2.06 | **全部（臂不动）** | 合拢后 0.24~0.30 s 附着，**之后 1.70~1.77 s 只是在等** `close_settle_s` = 2 s |
| LIFT | 0.50~0.55 | 0.02~0.03 | 运动 |
| PREPLACE | 0.51~0.68 | 0.01~0.06 | 运动 |
| PLACE | 0.50~0.55 | 0.02~0.08 | 运动 |
| OPEN | 0.50 | 臂不动 | 夹爪张开，结束时开度 79.5~79.7 mm，接近全开 |
| RETRACT | 0.50~0.54 | 0.04~0.09 | 运动 |
| VERIFY | oracle 0.50；vision 0.53~0.56 | 全部 | 等最小 settle 0.5 s |

“纯等”合计：oracle 约 2.8 s / 6.9 s，vision 约 3.7 s / 7.8 s。另一个现象：**每个运动阶段都是 0.5~0.7 s**，因为 FSM 要求“到位”且“满 0.5 s”，而阶跃目标下机械臂约 0.45~0.6 s 就冲到了。

**M2 开始前的停顿：** oracle 第一条关节命令在 reset 后 0.02~0.03 s，start 之后 0.03~0.04 s 墙钟；vision 在 reset 后 **1.40~1.44 s 仿真、2.42~2.47 s 墙钟**（等锁存）。Week 4.1 Stage 7 记的 3.4 s 是从状态话题的条数估的，这里是直接量的第一条命令。

**M3：** CLOSE 开始到附着 0.24~0.30 s，附着到 LIFT 1.70~1.77 s；vision 释放到第一条新测量 0.99~1.08 s（窗口重装 10 帧），VERIFY 本身 0.53~0.56 s。

**M4 按阶段切开的打滑**（盒子在 TCP 坐标系里相对附着时刻的位移 / 转角，阶段末的累计值）：

| 布局 | CLOSE 末 | LIFT 末 | PREPLACE 末 | PLACE 末 | 搬运期开度 |
| --- | --- | --- | --- | --- | --- |
| oracle 0 | 0.1 mm / 0.1° | 5.4 mm / 0.1° | 13.6 mm / 9.2° | 14.6 mm / 13.7° | 37.6~40.2 mm |
| oracle 1 | 0.1 / 0.2 | 4.5 / 1.6 | 11.6 / 14.2 | **15.8 / 39.4** | 38.2~40.3 |
| oracle 2 | 0.2 / 0.1 | 4.1 / 1.0 | 6.7 / 3.1 | 7.3 / 4.6 | 38.1~40.1 |
| vision 0 | 0.1 / 0.2 | 5.5 / 1.3 | 13.8 / 11.7 | 14.7 / 17.3 | 37.7~40.2 |
| vision 1 | 0.0 / 0.2 | 4.4 / 2.0 | 11.0 / 13.9 | **15.5 / 40.6** | 38.2~40.5 |
| vision 2 | 0.0 / 0.1 | 4.4 / 1.1 | 7.1 / 3.5 | 7.6 / 5.2 | 38.1~40.1 |

- CLOSE 期间几乎不动（≤ 0.2 mm）；**LIFT 一段就有 4~5.5 mm**（以平移为主），PREPLACE 再加 2.6~8.8 mm 且开始转，PLACE 主要是转（布局 1 从 14° 转到 40°）。
- 这一段的 TCP 峰值加速度：LIFT 17.7~18.9、PREPLACE 29.5~34.1、PLACE 28.1~29.8 m/s²。布局 2 的 PREPLACE 加速度和别的布局相近（29.5），滑得却少得多，所以**加速度不是唯一因素**，方向（相对手指的方向）可能更要紧，没有分解。
- 搬运期开度一直在 37.6~40.5 mm：**盒子在手里转了 40° 而开度不变**，Week 4.1 Stage 12 的开度窗口（34 mm）对它无能为力，这正是当时写下的局限。
- OPEN 阶段末盒子已离开手（附着状态不再是 ATTACHED），之后的“位移”是盒子落下、不是打滑。

**更正 2.1 节的一处推测：** 2.1 里我说“约 3.47 s 时盒子仍在滑而 TCP 几乎静止，更像夹爪开始张开或盒子已脱离下落”。按阶段切开后确认是后者：那段在 OPEN，盒子正在离开手。所以**搬运期打滑与 OPEN 无关**，它全部发生在 LIFT~PLACE。

**M5 相机到达率**（探针端，best effort，episode 期间 / 仿真时长 × 10 Hz）：

| 话题 | 6 个 episode 的范围 |
| --- | --- |
| RGB `color/image_raw` | **0.22~0.37** |
| 深度 `depth/image_raw` | 0.99~1.00 |
| 两个 `camera_info` | 0.99~1.00 |

RGB 在一个与估计器无关的订阅者那里也只到 22~37%，所以**丢帧发生在 bridge 发布或 DDS 传输这一侧**，不是估计器的配对。深度消息（320×240×4 = 307 KB）比 RGB（230 KB）大，却几乎不丢，所以不是简单的“消息太大”。原因留给 Stage 6。

**M6 RTF：** 0.58~0.59。墙钟时长约为仿真时长的 1.7 倍。

### 1.4 你没问但值得注意的

- **（E 可测试性）这些数字来自 6 个 episode、3 个布局。** 仿真确定性，所以同一布局不用重跑；但布局只有 3 个，打滑的范围（7~16 mm、5~41°）是这 3 个布局的，不是分布。Stage 4 定限值前，应当在更多布局上量打滑。
- **（C 可观测性）打滑现在只能在评测端用真值量。** 系统里没有任何信号能告诉 executor“盒子在手里转了 40°”：开度不变、附着锁存不变。真机上同样看不到（Franka Hand 没有触觉）。如果 Stage 4 之后仍有残余打滑，唯一能看见它的是相机，但搬运期视觉在 Week 4.1 已取消。
- **最小 settle 0.5 s 在两头起作用：** 它是 HOME、OPEN、VERIFY 里“纯等”的来源，也决定了运动阶段至少 0.5 s。Stage 3 加插值后，运动阶段会由轨迹时长决定（会更长），这条约束要和插值一起重新想，而不是只在 Stage 5 里砍。
- **RTF 0.58 让所有“停顿”在墙钟上放大 1.7 倍。** 用户看到的 CLOSE 停顿约 1.77 / 0.58 ≈ 3 s 墙钟。C4 要不要做，取决于 RTF 能不能提上去；Stage 2 之前先查是什么在占用时间（相机渲染、机器人遮罩、物理步）。

### 1.5 本阶段边界与后续

做完了：基线时间线、等待、按阶段切开的打滑、相机到达率、RTF。没有改产品代码。

给后续阶段的输入：Stage 3 的速度、加速度限值（第 2 节）；Stage 4 的打滑基线（PLACE 末 7~16 mm、5~41°，全部在搬运期）；Stage 5 的等待（CLOSE 多等 1.7 s、HOME 0.5 s、VERIFY 0.5 s、vision 锁存 1.4 s）；Stage 6 的 RGB 到达率（22~37%，在估计器之前就丢了）；C4 的 RTF 0.58。

## Stage 2：统一的新启动姿态兼 HOME

**状态（2026-10-07）：已完成。** 2.1、2.2 是实现之前写定的，没有改动（K4 的一个扫描点不合法，见 2.4 的更正）。**分量：重。**

### 2.0 目标

启动、每次 reset、executor 的 HOME、VERIFY 时机械臂停的位置，都是同一个关节姿态；这个姿态不挡相机看桌面工作区（初始检测 box 与 bin、VERIFY 时检测 bin 里的盒子）。

### 2.1 现状（实测）与候选

**用语：** 本节的“预设状态”指 MJCF 里的 `<keyframe>`：一份带名字、预先存好的完整仿真状态（各关节角 qpos、执行器目标 ctrl 等），`mj_resetDataKeyframe` 把仿真恢复成它。这个词借自动画，但与时间序列、图像帧无关；也不要和 executor 的**固定关节表模式**（参数值 `waypoint_source:=keyframe`，Week 2 的遗留模式，每个阶段查一张写死的关节角表，并不读 MJCF 的预设状态）混淆。

**跳变的来源（实测）：** bridge 用 `mj_makeData` 建状态，**启动时是 MuJoCo 的默认状态**（各关节约 0，手臂几乎竖直，TCP 在 (0.13, 0, 0.84)）；第一次 reset 才把它放到预设状态 `pick_place_home`，最大关节跳变 **1.52 rad**。之后每个 episode 结束在 bin 上方，下一个 reset 又把手臂瞬移回预设状态。executor 的 HOME 是 Cartesian 目标 (0.5545, 0, 0.5211) 经 IK，Stage 1 实测它与预设状态的手臂姿态一致（HOME 阶段手臂不动）。

**候选：Franka 标准 ready 姿态** `q = [0, −π/4, 0, −3π/4, 0, π/2, π/4]`，TCP 在 (0.307, 0, 0.487)，工具竖直向下，手的长轴（手指张开方向）沿世界 y。

**几何估计（只是估计，在线以机器人遮罩为准）：** 从相机 (0.5, −0.45, 1.0) 把机械臂各连杆点与手的外形投影到桌面（z = 0.22），看投影是否落进工作区 x ∈ [0.30, 0.70]、y ∈ [−0.30, 0.40]（各点按 5 cm 半径放大）：

| 候选 | 离工作区最近的距离 |
| --- | --- |
| 现在的 HOME | −199 mm（落在工作区里） |
| ready 姿态，但手腕转成手指沿世界 x（`j7 = −π/4`） | −126 mm |
| **ready 姿态（标准，`j7 = +π/4`）** | **+17.5 mm** |

手的长轴约 0.2 m，横在视线上就会挡住；标准 ready 姿态把它顺着视线方向放，所以只有它是正的。选它还因为它是 Franka 自己的标准姿态，换到真机上没有意外。

### 2.2 设计与出口断言（实现前写定，不因结果改动）

**改动：**

1. 预设状态 `pick_place_home` 的手臂部分与 ctrl 改为 ready 姿态。
2. bridge 启动时（建模、配置场景之后）就把仿真置为 reset 用的预设状态：启动即 HOME，不再从默认状态开始；generation 仍为 0。
3. executor 的 HOME 改为**关节目标**（不再经 IK）：新参数 `home.joint_positions`，默认 ready 姿态。单测读 MJCF 里的预设状态，核对两者一致。
4. **VERIFY 时机械臂回到 HOME**（RETRACT 仍是抬离 bin）：VERIFY 的目标改为 HOME 关节目标，VERIFY 的退出条件加上“机械臂到位”。这样 VERIFY 时手不在相机与 bin 之间；DONE 时机械臂已在 HOME，下一次 reset 只瞬移盒子，不瞬移手臂。
5. Week 4.1 的收尾（C7）：14.6 的“推断”改为“已由用户用 rqt 证实”，补覆盖缺口的说明。

| 编号 | 条件 | 预期 | 否定条件 |
| --- | --- | --- | --- |
| K1 | bridge 刚启动、第一次 reset 前后 | 启动时手臂就在 ready 姿态（各关节与预设状态差 ≤ 0.01 rad）；第一次 reset 的最大关节跳变 ≤ 0.01 rad | 任一超出 |
| K2 | 单测 | executor 的 `home.joint_positions` 默认值与预设状态的手臂部分逐项相等（≤ 1e-4 rad） | 不等 |
| K3 | 机械臂在 HOME，估计器的机器人遮罩 | 遮罩像素投到桌面后，落进工作区 x ∈ [0.30, 0.70]、y ∈ [−0.30, 0.40] 的：**0 个** | > 0 |
| K4 | bin 沿 y 扫：0.20、0.25、0.30、0.35（x = 0.5，yaw 0），机械臂在 HOME | 4 个位置 bin 都 MEASURED；`ros2 launch mujoco_bridge demo.launch.py scene_enabled:=true` 不加别的参数，vision 成功 | 任一检不出，或默认 launch 失败 |
| K5 | 连续两个 episode（vision，同一布局） | 第一个 DONE 时手臂在 HOME（各关节与 HOME 差 ≤ 0.05 rad）；第二个 episode 的 reset 前后手臂最大跳变 ≤ 0.05 rad | 超出 |
| K6 | HELD-A（现为回归集）40 个布局，vision 与 oracle | 不比 Week 4.1 Stage 14 差：vision ≥ 39、oracle ≥ 40 成功，零假成功；**布局 16 vision 成功**（VERIFY 不再被手挡） | 任一更差 |
| K7 | K6 同一批 | 从 HOME 到 PREGRASP 的 IK 全部收敛（没有 `IK_FAILED`） | 出现 `IK_FAILED` |

**不做：** 关节轨迹插值（Stage 3）；HOME 之外的姿态调整；固定关节表模式（`waypoint_source:=keyframe`）的 HOME 不改，它是遗留模式，记在边界里。

### 2.3 改动

| 文件 | 改动 |
| --- | --- |
| [pick_place_scene.xml](../../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml) | 预设状态 `pick_place_home` 的手臂 qpos 与 ctrl 改为 ready 姿态 |
| [mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp) | 解析出 reset 用的预设状态后立刻 `resetToKeyframe` 一次（在 `configureScene` 之后，所以盒子与 bin 的布局也在里面），不递增 generation |
| [waypoint_source.hpp](../../src/task_executor/include/task_executor/waypoint_source.hpp) | 常量 `kFrankaReadyPose` |
| [diff_ik_waypoint_source.cpp](../../src/task_executor/src/diff_ik_waypoint_source.cpp) | HOME、VERIFY 直接返回 HOME 关节目标（夹爪宽度仍取自任务），不求 IK；诊断里给出该构型的 TCP、残差 0；构造时检查 HOME 有限且在关节极限内 |
| [fsm.cpp](../../src/task_executor/src/fsm.cpp) | VERIFY 的完成条件加上“机械臂到达目标”（同运动阶段的 0.05 rad、0.05 rad/s） |
| [task_executor_config.cpp](../../src/task_executor/src/task_executor_config.cpp) | 参数 `home.joint_positions`（7 个值，默认 `kFrankaReadyPose`） |
| 单测 | `HomeIsTheBridgeResetKeyframe`（K2：用正则读 MJCF 里的预设状态，逐项比对）、`HomeAndVerifyCommandTheHomeJointsWithoutIk`（远离 HOME 的 seed 也给出同一目标，TCP 在 (0.307, 0, 0.487)、朝下）、`RejectsAHomeOutsideTheJointLimits`、`VerifyWaitsForTheArmToReturnHome`；`test_fsm` 的 HOME 常量改为新姿态，两个 VERIFY 用例补上“手臂已到位” |
| [initial_box_probe.py](../../src/mujoco_perception/test/initial_box_probe.py) | P1（及 Stage 7 的 A1）的判定改为按到达时间，见 2.4 的 P1 一段 |
| 文档 | architecture 2.3、5、6.1、6.2（HOME、启动状态、VERIFY、用语）；Week 4.1 14.6 的 C7 修订 |

**为什么 HOME 要改成关节目标，而不只是改个数值。** 改之前“HOME”有两份定义，用的不是同一种量：

| 在哪 | 存的是什么 |
| --- | --- |
| MJCF 预设状态 `pick_place_home` | 7 个**关节角** |
| executor 的 HOME | 一个 **TCP 位姿**（位置 (0.5545, 0, 0.5211)、工具朝下）；每个 episode 用 IK 从它反算出关节角再发给 bridge |

两者对得上，是因为那个 TCP 位姿当初就是从 MJCF 的关节角算出来的（Stage 1 实测 HOME 阶段手臂不动）。如果只把 executor 的 TCP 位姿改成 ready 姿态的 TCP (0.307, 0, 0.487)，会出问题：Panda 有 7 个关节，TCP 位姿只确定 6 个量（3 个位置、3 个转角），多出 1 个自由度，**同一个 TCP 位姿对应无穷多组关节角**（肘部可高可低、上臂可绕着转），IK 给出哪一组取决于迭代的初值。于是：

- IK 解出的不一定是 ready 姿态，开局手臂会动一下，“启动 = reset = HOME”不成立；
- K3 的“不挡相机”是按 ready 姿态下整条手臂的位置算的，TCP 相同而肘部不同，遮挡也不同，结论不适用。

所以 HOME（以及 VERIFY）直接发 7 个关节角 `home.joint_positions`，不经过 IK；executor 与 MJCF 存的是同一种量、同一组数，单测逐项比对。其余阶段的目标取决于盒子和 bin 在哪，只能给出 TCP 位姿，照旧用 IK。

**副作用（2.4 的 P1 就是它引起的）：** 以前 HOME 要先求一次 IK 才能发出第一条命令，锁存完成（LATCHED）和第一条命令之间总隔着一段计算；现在不用算，两条消息几乎同时发出。

一次性测量脚本（K1、K3、K4、K5、P1/P5 排查）放在 `.claude/artifacts/week5_stage2/`，不进仓库。

### 2.4 结果

| 编号 | 结果 | 判定 |
| --- | --- | --- |
| K1 | 启动后第一个样本离 ready 姿态 0.0004 rad；第一次 reset 的最大关节跳变 **0.005 rad**（改之前 1.52 rad） | 通过 |
| K2 | 单测通过（`colcon test` mujoco_bridge + task_executor：760 项，0 失败） | 通过 |
| K3 | 10 帧机器人遮罩的并集 8234 像素，投到桌面后落进工作区的 **0 个**，离工作区最近 79.6 mm；投影自检：TCP (0.306, 0, 0.484) 投到的像素在遮罩内 | 通过 |
| K4 | bin 在 y = 0.20、0.25、0.30、0.32 都 MEASURED（21/21 条，误差 ≤ 1.3 mm）；默认 launch `scene_enabled:=true` 不加参数，vision 成功、零重试 | 通过，但见下面的更正 |
| K5 | 同一 launch 连跑两个 episode，都成功、零重试；DONE 时手臂离 HOME 0.006 rad；两次之间的 reset 跳变 0.006 rad | 通过 |
| K6 | 见下 | |
| K7 | 没有 `IK_FAILED` | 通过 |

**K4 的更正（我的错误）：** 2.2 写的扫描点 y = 0.35 本身不是合法布局：bin 外沿半宽 71 mm，y = 0.35 时外沿到 0.421，超出桌面（y ≤ 0.4），bridge 按设计拒绝启动。写断言时没有核对桌面边界。bin 能放的最远处是 y ≈ 0.329，所以用 y = 0.32 代替 0.35；其余三个点不变。

**K6（HELD-A 回归，vision）：** 按改正后的 P1 判定重跑一遍：**40/40 成功、零重试、零假成功**，没有 `IK_FAILED`；**布局 16 成功**（Stage 14 里 VERIFY 被手挡住的那个）；P1~P4、P6 全部 40/40，**P5 39/40（布局 29 告警）**；估计器在 ATTACHED 样本上发出的消息 0 条（N1）。第一次回归（改判定之前）同样 40/40 成功，P1 5 个误报、P5 也只有布局 29。

HELD-A 是 Week 4.1 Stage 14 建的留出集：固定种子 20261006 生成的 40 个布局（box 与 bin 在 x ∈ [0.38, 0.62]、y ∈ [−0.22, 0.22] 内随机位置与朝向，太近的重抽），规则在第一次跑之前写定（[week4.1 14.1](week4.1.md)）。Stage 14 跑过之后它已不再“留出”，现在只当回归集，回答“改完有没有比以前差”；没碰过的 HELD-B（种子 20261009）留给本周最终验收。每个布局起一套新的 bridge、估计器、executor 跑一个 episode，记录成功与否、重试、失败归类，以及过程断言 P1~P6（如 P1：锁存完成之前没有关节命令；P5：搬运期没有开度告警）。

**用语：锁存（LATCHED）。** reset 之后估计器每帧给一个盒子、bin 位置，每帧都略有抖动；executor 先观察，等连续 5 条有效测量的 x、y 相差 ≤ 3 mm、朝向相差 ≤ 3°，就把最新那条“锁住”，这个 episode 一直用它（Week 4.1 Stage 7）。状态每 50 ms 发布到 `/task_executor/initial_pose_latch`：WAITING（还在等）、LATCHED（盒子与 bin 都锁住）、FAILED（10 s 内没锁住）。锁存之前 executor 不知道盒子在哪，按设计不发任何关节命令；Stage 1 量到的 vision 开局停顿约 1.4 s 主要就是在等它。

- **oracle 没有跑完。** 跑到 19 个（19/19 成功、零重试、P1~P6 全部成立）时，用户决定停掉 oracle、以后不再跑，所以 K6 里“oracle ≥ 40”一项**没有执行**，不算通过。
- **P1 的 5 次失败是探针的测量方法错了，不是 executor 提前发命令（第一次回归：布局 14、22、25、32、33）。** P1 原来按“两个话题的消息到达探针的先后”判定。executor 的 50 ms 定时回调里顺序固定：先发锁存状态（LATCHED），再发第一条命令；观测准入（`onObservation`）不产生命令。但这是两个话题，到达先后没有保证：重跑布局 14，命令比 LATCHED **早 0.12 ms** 到达；布局 32 晚 0.26 ms。真正提前的命令至少早一个回调（50 ms）。为什么 Stage 14 一次也没碰到：以前 HOME 要先求 IK，两条消息之间隔着一次求解；现在 HOME 不求 IK，只隔零点几毫秒（IK 的耗时没测，这是推断）。修法（用户同意）：断言不变，判定改为“命令比 LATCHED 早到超过半个周期（25 ms）才算”，Stage 7 的 A1 是同一个检查，一起改。
- **P5：布局 29 是真打滑掉盒，盒子碰巧落进 bin。** 逐帧看盒子相对 TCP 的位置：LIFT 时偏 9.6 mm，搬运到 bin 上方（TCP 峰值 1.72 m/s）累计偏到 37 mm，PLACE 下降时从指间滑出（开度降到 4 mm，bridge 的附着仍锁存为 ATTACHED），最后落在 bin 里，所以判为成功。Stage 14 里这个布局没有告警。**是不是 Stage 2 造成的没有证明**：换了起点，IK 的初值链跟着变，搬运时的关节构型也不同（这次搬运中 joint7 转了 0.43 rad），可能是原因；要确认需要用旧 HOME 对比，用户决定不做。现象与 Stage 1 的“打滑全在搬运期、伴随高速”一致，归 Stage 3、4。**布局 29 记为 Stage 3、4 必须通过的回归用例。**

**判定：** K6 没有完全满足——oracle 部分未执行，P5 比 Stage 14 多一次告警。用户同意按现状收尾（2026-10-07）。

### 2.5 你没问但值得注意的

- **（E 可测试性）跨话题的先后不能当断言。** P1 这次是按“到达顺序”判的，executor 内部一个很小的时序变化（不再求 IK）就让它误报。凡是比较两个话题先后的检查都有同样的问题；Stage 6 要做“命令生效证据”，那里应当让消息自己带时间戳或序号（executor 发命令时盖上它依据的观测序号），而不是靠到达顺序。
- **（C 可观测性）“成功”里混着掉盒。** 布局 29 的盒子从手里滑出、碰巧落进 bin，outcome 是干净的成功，只有探针的开度告警看得见。这正是 C5 担心的：挑 VLA 训练数据时，要把“搬运期出现开度告警”的 episode 排除，而现在 outcome 里没有这个字段（告警只在日志里）。
- **（E 可测试性）HELD-A 已不是留出集，而且它的 bin 只在 y ∈ [−0.22, 0.22]。** 这次 K4 单独扫了 y = 0.20~0.32，但完整 episode 的回归仍只覆盖 HELD-A 的范围。本周最终验收用 HELD-B 时，生成器的 bin 范围要按桌面实际可放的范围（y 到 ±0.329）重写，否则又是同一个覆盖缺口。
- **（C 可观测性）DONE 现在依赖“手臂回到 HOME”。** 如果将来 HOME 附近有东西挡住手臂（或 Stage 3 的轨迹回 HOME 很慢），VERIFY 会以 `PLACE_MISSED` 超时，但真实原因是手臂没到位；失败码分不出两者。

### 2.6 本阶段边界与后续

做完了：启动、reset、HOME、VERIFY 停在同一个关节姿态（Franka ready），启动与 reset 之间、episode 之间都不再瞬移手臂；HOME 时机器人遮罩不覆盖工作区；默认 bin 位置可见，默认 launch 直接成功；Week 4.1 的 C7。

没有做：
- 固定关节表模式（`waypoint_source:=keyframe`）的 HOME 仍是旧姿态（遗留模式，Stage 7 清理时一并处理）；Week 3 的对照场景 `stage_n_shifted_scene.xml` 的预设状态也没改，只有 Week 3 的实验用它。
- oracle 的回归（用户决定不再跑）。
- 布局 29 的打滑（Stage 3、4）。

给后续阶段的输入：布局 29 是打滑的回归用例；Stage 3 的轨迹从 ready 姿态出发、在 VERIFY 回到它，HOME 的时长要按轨迹重新算；Stage 6 的命令生效证据不要依赖跨话题的到达顺序。

---

## 5. 本周最终出口

1. 一次 demo 从启动到放进 bin 没有可见的跳变和不必要的停顿；关节速度、加速度在真实 Panda 限值内；搬运中盒子相对 TCP 的偏移小于 Stage 4 写定的限值。
2. 默认参数直接 launch 能成功；bin 在默认位置可见；HELD-A 回归不退步。
3. 原始记录满足数据契约 v0.2，探针核对过：RGB 不丢帧、action 平滑、命令生效有证据、元数据干净。
4. README 是技术报告，architecture 是开发者文档。

## 6. 悬挂与暂缓

| 项目 | 安排 |
| --- | --- |
| 数据导出程序（LeRobot 格式等） | 不在本周；Stage 6 的原始记录是它的输入 |
| 在线速度级控制（约束 QP） | 不做；插值已能限速，见 2.3 |
| 多物体、多颜色任务（原 Week 4.5，已删除） | 不在本周；需要时按单物体的实际需求重新规划 |
| 真机、真实相机噪声 | 继续暂缓 |
| HELD-B（种子 20261009） | 只记录种子；Stage 2 改了 HOME 后的回归用 HELD-A，HELD-B 留给本周最终验收 |

## 参考

- [libfranka rate_limiting.h](https://github.com/frankarobotics/libfranka/blob/master/include/franka/rate_limiting.h)：关节速度上限、笛卡尔加速度上限
- [FCI：Control Interface Specification and Robot Limits](https://frankarobotics.github.io/docs/robot_specifications.html)：超限即中止
- [libfranka Gripper / GripperState](https://docs.ros.org/en/humble/p/libfranka/generated/classfranka_1_1Gripper.html)：`is_grasped` 的含义（Week 4.1 Stage 13）
- `lerobot_begginer/pi05_remote/docs/dataset_contract.md`、[pi05_remote_local_plan.md](../../docs/plans/pi05_remote_local_plan.md)
