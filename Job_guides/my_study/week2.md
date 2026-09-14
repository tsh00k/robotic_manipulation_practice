# Week 2 学习笔记

> **本文件当前是开周计划稿**，只有第 1~4 节（继承事实、stage 计划、验收标准、要回头解锁的悬挂项）是现在写下的。第 5 节起的 stage 节随每个 stage 完成后就地扩写，骨架按 [STUDY_NOTES_GUIDE.md](../../STUDY_NOTES_GUIDE.md) 2.2（`N.0 一句话总结 → N.1 改动清单与验证结果 → N.x 概念讲解 → N.x 失败模式与验证手段 → N.x 排查记录`）。
>
> Stage 编号延续第1周的 A~E，本周是 **F~J**。

## 学习重点范围

本周对应计划书[第5节 Chap 3 模块](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md)与 Roadmap 第2周（"Chap 3 ground-truth pick-and-place、夹爪动作 → 固定场景连续 20 次成功；第一段 Demo"）。第1周把地基（模型加载、时间、TF、命令、复位）打完了，本周第一次出现**任务语义**：多阶段、有成功/失败、可重复跑。需要重点掌握以下几块：

1. **MJCF 场景搭建与接触参数** — 怎么在不改 vendor 文件的前提下组合出自己的场景（`<include>`、`worldbody` 追加、`<keyframe>` 必须随 `nq` 重写）；`freejoint` 的广义坐标布局（3 平移 + 4 四元数，`nq != nv` 第一次真实成立）；`condim`/`friction`/`solref` 各管什么，以及它们和 `timestep` 的关系——这是第1周悬挂清单第 2 条（timestep 凭什么取 0.002）**第一次有了实验对象**。
2. **夹爪的命令与状态** — position-servo + tendon 驱动下"夹住了"在数据上长什么样（不是力，是稳态位置误差 + 接触）；`control_msgs` 里夹爪接口的既有形状（`GripperCommand`）为什么值得现在就对齐；抓取成功怎么用**多个独立信号**判定而不是"命令发出去了"。
3. **任务状态机（FSM）** — 阶段划分、到位判据（阈值 + 超时）、失败码分层、恢复动作；为什么 FSM 应该是独立节点而不是 bridge 里的一个回调；低频决策与高频物理步进怎么通过 `/clock` + `use_sim_time` 对齐。
4. **ground-truth（oracle）接口的隔离** — 计划书第 4 节的硬要求："ground truth 必须走独立的 oracle 接口，不能偷偷混进正式感知输出"。本周要把这条边界**在代码里画出来**，第4周换视觉时只换发布者、不改下游。
5. **可测试性（E 类）从零到一** — 把和 `rclcpp`/`mjModel` 焊在一起的逻辑剥成纯函数，搭起第一个 `colcon test` 目标。这是第1周复盘里**唯一完全空缺的提问类别**（见 [STUDY_NOTES_GUIDE.md](../../STUDY_NOTES_GUIDE.md) 4.1 的 E 类），也是 week1 反向清单里三条互相咬合的待办的共同前置。

## 目录

- [1. 从 Week 1 继承的硬约束与既有事实](#1-从-week-1-继承的硬约束与既有事实)
- [2. 本周 Stage 计划](#2-本周-stage-计划)
  - [2.1 Stage F — 测试地基与胶水层收口](#21-stage-f--测试地基与胶水层收口)
    - [2.1.1 这类系统该怎么测：四层，本周只取前两层](#211-这类系统该怎么测四层本周只取前两层)
  - [2.2 Stage G — pick-and-place 场景（table + cube）与 oracle 接口](#22-stage-g--pick-and-place-场景table--cube与-oracle-接口)
  - [2.3 Stage H — 夹爪命令接口与抓取成功判据](#23-stage-h--夹爪命令接口与抓取成功判据)
  - [2.4 Stage I — FSM 与 WaypointSource 抽象（新包 `task_executor`）](#24-stage-i--fsm-与-waypointsource-抽象新包-task_executor)
  - [2.5 Stage J — episode runner、20 次连跑与第一段 Demo](#25-stage-j--episode-runner20-次连跑与第一段-demo)
- [3. 本周验收标准](#3-本周验收标准)
- [4. 要回头解锁的 Week 1 悬挂项](#4-要回头解锁的-week-1-悬挂项)
- [5. 开周就该注意的（Claude 提示，尚未讨论）](#5-开周就该注意的claude-提示尚未讨论)
- [6. 悬挂问题（本周新增）](#6-悬挂问题本周新增)
  - [6.1 清单](#61-清单)
  - [6.2 反向清单：现在就该做的](#62-反向清单现在就该做的)

---

## 1. 从 Week 1 继承的硬约束与既有事实

本节只做速查，**权威出处都在别处**，不要在这里展开或改写。

| 约束 | 内容 | 出处 |
|---|---|---|
| **不得直接链接 MuJoCo** | 新用到的 `mj_*`/`mju_*` 必须先进 `MujocoApi`，再 `resolve()`，业务代码走 `api_.xxx()`。直接 `target_link_libraries(mujoco::mujoco)` 会因 tinyxml2 符号冲突段错误 | [week1.md 7.2](week1.md#72-调试时踩到的段错误符号冲突与-dlopen-隔离) |
| **时间戳规则分两侧**（本周开周时修正，原先写成了"一律 `simTime()`"，那只对 bridge 成立） | **bridge 侧**：`simTime()` = `std::llround(data_->time * 1e9)`，**不能**用 `get_clock()->now()`——它*是* `/clock` 的源，读自己的钟只能拿到墙钟（`use_sim_time=false`）或上一步刚发布的旧值（`use_sim_time=true`），而且构造期 `/clock` 上还什么都没有。**下游侧**（executor、runner、测试脚本）：设 `use_sim_time=true`，然后正常用 `get_clock()->now()`——`TimeSource` 会订阅 `/clock` 替你更新。**两侧都禁止**的是第三种写法：`std::chrono` / `RCL_SYSTEM_TIME` / 不设 `use_sim_time` 却调 `now()`，那是绕过仿真时钟的墙钟 | [week1.md 8.2](week1.md#82-sim-time-vs-wall-time以及-use_sim_time)、[node 源码 365 行注释](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp) |
| **当前状态：全图没有一个节点设过 `use_sim_time`** | `demo.launch.py` 没给任何节点传参数，`sine_joint_test.py` / `gripper_test.py` 都在用未配置的 `get_clock().now()` 算 elapsed——**现在跑的是墙钟正弦**，只因 RTF≈1 才看不出来。Stage I 的 FSM 超时是第一个会被这件事真正咬到的地方（RTF≠1 时超时阈值的语义直接漂掉）。**Stage F 顺手修**：launch 加 `use_sim_time=true`，两个脚本一并改 | 本周开周核对发现 |
| **frame 名 = MJCF 原生名** | 基座是 `link0`（不是 `base_link`），**没有 `link8`**，`hand` 的父是 `link7`，`hand_tcp` 是合成的纯 103.4mm 平移 | [architecture.md 第1节](../../docs/architecture.md) |
| **static/dynamic 判据是结构而非名字** | `body_jntnum[i] == 0` → static。本周加的 `box`（带 freejoint）会**零代码改动**自动变成 `world -> box` 的 dynamic TF | [week1.md 9.4](week1.md#94-static--dynamic-怎么划分用结构而不是名字) |
| **`~/reset` 复位状态但不复位时间** | `mj_resetDataKeyframe` 的全部作用域（含 `ctrl`）+ 显式保护 `mjData::time`。本周的 episode runner 完全依赖它 | [architecture.md 2.2](../../docs/architecture.md) |
| **夹爪 `ctrl` 是 `0..255` 且经 tendon** | 写 `ctrl` 必须按 actuator id 索引，不能用 joint id；换算系数从模型两个 range 数组现算 | [week1.md 11.2](week1.md#112-buildactuatorindex关节驱动与-tendon-驱动的两种索引) |
| **`home`(MJCF) ≠ `ready`(SRDF)** | 两个不同构型（joint7 连符号都相反）。本周挑基准位姿只用 MJCF `home`，第6周接 MoveIt 时才定权威 | [architecture.md 2.1](../../docs/architecture.md) |
| **验证环境卫生** | `ps -eo pid,comm` + `ros2 topic info /clock` 是唯一权威判据；起后台节点直接跑可执行文件，不经 `ros2 run`；`ros2 node list`/`ros2 daemon` 都会骗人 | [STUDY_NOTES_GUIDE.md 3.1](../../STUDY_NOTES_GUIDE.md) |

## 2. 本周 Stage 计划

五个 stage，**每个都按五步走不跳步**（写代码 → build + 实跑验证 → 讲解 → 追问 → 写进本文件），见 [STUDY_NOTES_GUIDE.md 第3节](../../STUDY_NOTES_GUIDE.md)。

### 2.1 Stage F — 测试地基与胶水层收口

**为什么放在第一个**：本周要给 `MujocoApi` 再加若干字段（读接触、夹持宽度、可能的 mocap），12 个字段已经贴住 [7.2.6](week1.md#726-当前方案的扩展性代价以及怎么改善) 写下的 `decltype` 重构触发点；而 FSM + 成功判据会让节点更难测。**先剥离再加功能**，顺带把完全空缺的 E 类补上。这一步是 week1 反向清单里三条待办（`onReset` 单测、`publishTransforms` 单测、`decltype`）的共同前置——week1 自己就写了"三条应该一起做"。

| | 内容 |
|---|---|
| 改动 | ① `MujocoApi` 全部字段改 `decltype(&mj_xxx)`；② 新增 `frame_math.hpp/cpp`：绝对位姿 → 父相对变换的纯函数（不依赖 `rclcpp`、不依赖 `mjModel`，只吃 `xpos`/`xquat`/父子关系）；③ 新增 `state_ops.hpp/cpp`：把 reset 剥成 `resetToKeyframe(model, data, key)` 自由函数（不依赖 `rclcpp`）；④ 节点只负责参数、消息装配、日志 |
| 新增测试 | `test_frame_math`（纯函数，不加载模型）、`test_state_ops`（需要一个**带 free joint 的最小 MJCF fixture**，因此同时覆盖 `jnt_qposadr` / `jnt_dofadr` 写混那条——Panda 上 `nq == nv` 测不出来） |
| 验收 | `colcon test` 全绿；**并且**故意把 `mulQuat` 参数顺序写反、把 `jnt_qposadr` 改成 `jnt_dofadr`，两个单测必须变红（对测试本身的测试）。四元数断言比 $\|q_1 \cdot q_2\| \approx 1$，不逐分量比（double cover，见 [6.3.1](week1.md#631-一个单位四元数就是一个旋转)） |
| 预期要讲的概念 | gtest/`ament_cmake_gtest` 的组织方式；测试里怎么拿模型文件路径（`ament_index` vs CMake 传宏 vs 测试目录相对路径）的取舍；"剥离到什么粒度算够"——纯函数边界画在哪 |
| 顺手带上 | RTF 监控（约 6 行，[8.6](week1.md#86-失败模式与验证手段) 末尾）、`tf_rate_hz != joint_state_rate_hz` 时 `WARN`（约 3 行，[9.11](week1.md#911-权衡tf_rate_hz-独立于-joint_state_rate_hz)）。两条都是"当前够用所以没做"，本周开始跑长时间 episode，RTF 变成真的会咬人的量。**再加第三条**：给 launch 里的节点和两个测试脚本配上 `use_sim_time=true`（见第1节那两行），否则 Stage I 的 FSM 超时会建在墙钟上 |

#### 2.1.1 这类系统该怎么测：四层，本周只取前两层

> Q: 你认为有必要逐渐开始加单元测试了吗，我也好奇这样的系统应该如何去测试。

机器人系统的可测性不是一个连续谱，而是**四个成本/价值差异极大的层**。混在一起谈"要不要写单测"会得出错误结论——因为答案对第1、2层是"立刻写"，对第3层是"少量"，对第4层是"根本不该叫测试"。

| 层 | 例子 | 成本 | 价值 | 本周 |
|---|---|---|---|---|
| **1. 纯数学 / 纯函数** | 父相对变换、四元数、（第3周）FK/Jacobian、（第6周）grasp 评分 | 最低（无依赖，毫秒级） | 最高：**错误是静默的**——数值偏了不崩溃，位姿歪几度，然后你在第6周怀疑 MoveIt | **做** |
| **2. 模型契约 / 假设断言** | 关节顺序、`jnt_qposadr` vs `jnt_dofadr`、actuator 映射、keyframe 的 `qpos` 长度 == `nq`、`0.1034`、手指 `jnt_range` 下限为 0、URDF↔MJCF 一致性 | 低（加载一个 MJCF） | 最高：这层防的是**上游文件被改**，是唯一能自动抓到这类回归的手段 | **做** |
| **3. 节点 / 进程契约** | `launch_testing`：起 bridge，断言 `/clock` 在发、`/joint_states` 的 `name` 维度和顺序对、`~/reset` 能应答 | 中（起 DDS，秒级，偶发 flaky） | 中：适合**冒烟**，不适合细粒度断言 | **不做**，等第3周 MoveIt 接入后再评估 |
| **4. 任务层成功率** | episode runner 的 20 次成功率、pose error 分布 | 高（分钟级，随物理参数漂移） | 高但**不是测试，是实验**。不该当 CI gate，产物进 `results/` | Stage J 做，但**不放进 `colcon test`** |

**这类系统的 bug 主要不在第1层。** 大部分故障是"两个模块对同一个东西的假设不一致"——关节顺序、单位、frame 方向、`0..255` 还是 `0..0.04`、哪个 `pos` 是谁相对谁。单元测试对"函数算错了"很有效，对"假设不一致"几乎无能为力，**除非你把假设写成可执行的断言**，这就是第2层存在的理由。回头看第1周捡到的三颗地雷（[4.3.4](week1.md#434-每个关节的状态变量应该是什么样以及一个-stage-e-地雷)）全是第2层，而且全是在"学怎么读文件"时顺手发现的，不是靠找 bug 找到的——说明这层目前**完全靠人的注意力在维持**。

**一个能直接用的判据**：如果这个东西写错了，**第一个发现它的是人眼看 RViz**，那它就该有测试。第1周所有验证都是 `echo`/`hz`/`tf2_echo` + 肉眼，所以这个判据现在几乎命中一切。

**明确不该做的四件事**（否则会在可测试性上过度投资）：

1. **不给 MuJoCo 的物理写测试**——那是在测 MuJoCo。接触、积分、求解器的正确性不是我们的断言对象。
2. **不用 MuJoCo 验 MuJoCo**（[10.7.1](week1.md#1071-关键别用-mujoco-验-mujoco) 已定）。交叉验证必须来自独立实现（KDL/URDF）或独立推导（有限差分）。
3. **不给 `onTimer` 造 mock**——为了测一个"调 `mj_step` 然后发消息"的回调而引入一堆接口抽象，在这个规模是负收益。该做的是把**里面的决策**（reset 语义、变换数学、索引构建）剥出去，剩下的胶水就不用测了。
4. **不追覆盖率**。目标是"每条跨文件的、手抄的、隐含的假设都有一个会响的东西"，不是行数百分比。

**为什么是现在，不是第1周也不是第3周**——三条具体理由：

- 本周引入**第二个节点**（`task_executor`）和**第二个模型文件**（自己的 scene），跨模块假设的数量第一次翻倍；
- 第1周留下的地雷全是"现在验证不了、第4周加物体才炸"型（`nq != nv`），而**本周 Stage G 就是那个场景**——解锁条件提前到了；
- 剥离成本随代码量涨。`onReset` 和 `publishTransforms` 现在各几十行，剥成自由函数是半小时的事；等 FSM 和成功判据压上去就不是了。

**顺带的收益**：计划书 [8.2 第6条](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md) 要求"保存失败场景用于回归测试"。第2层的 fixture 机制（一个最小 MJCF + 一组期望值）正好就是那个回归测试的载体，第6周不用另起一套。

### 2.2 Stage G — pick-and-place 场景（table + cube）与 oracle 接口

| | 内容 |
|---|---|
| 改动 | ① 新建 **我们自己的** `mujoco_models/pick_place_scene.xml`：`<include>` vendor 的 `panda.xml`，追加 table geom、`box` body（`freejoint`）、放置区标记；**自带 `<keyframe name="home">`**（`qpos` 长度必须是新的 `nq`，vendor 的 9 元 keyframe 在新模型里失配）；② 新增 `~/ground_truth/object_pose`（`geometry_msgs/PoseStamped`，frame `world`，stamp 用 `simTime()`）作为下游**唯一允许消费的 oracle 源**；③ bridge 默认模型路径切到新场景（保留参数可切回） |
| 关键约定 | **不改 vendor 文件**（`robot_description/mujoco/franka_emika_panda/` 保持和上游一致，方便 diff 升级）。唯一例外是已在 architecture.md 待办里的 `hand_tcp` `<site>`，那是独立决定、要配 ADR |
| 验收 | ① `world -> box` 的 dynamic TF **零代码改动**自动出现（Stage C 结构判据的第一次真实检验）；② `nq != nv` 首次成立，打印两者确认；③ `~/reset` 后 box 回到标称位姿且 sim time 不回退；④ oracle 话题的位姿与 `tf2_echo world box` 一致（两条独立路径互相印证） |
| 预期要讲的概念 | `freejoint` 的 `qpos` 布局（3 平移 + 4 四元数，w 在前）与 `qvel`（6 维）为什么维度不同；`condim`/`friction`/`solref` 各控制什么；为什么 oracle 要单独一条话题而不是让下游查 TF |
| 同步文档 | 桌面高度/尺寸、box 尺寸/质量/摩擦、放置区定义、oracle 话题契约 → 全部进 `docs/architecture.md`（笔记是过程，architecture 是权威） |
| 顺带解锁 | 悬挂清单 #2（timestep）第一次有实验对象——本周**只做观察记录，不下结论**（调大 timestep 看穿透/抖动），结论留到第4~5周接触真正丰富时 |

### 2.3 Stage H — 夹爪命令接口与抓取成功判据

| | 内容 |
|---|---|
| 改动 | ① 夹爪从 `~/joint_command` 里分出独立的 `~/gripper_command`（用 `control_msgs/msg/GripperCommand`：`position` + `max_effort`，对齐真机 `ros2_control` 的接口形状，但本周仍是 topic 不是 action）；② 读夹持状态：手指关节实际宽度 + 手指与 box 之间是否存在接触（需给 `MujocoApi` 加字段）；③ 定义**抓取成功判据**和失败码枚举 |
| 成功判据（三个独立信号，缺一不可） | ① box 高度超过阈值（离开桌面）；② 夹爪实测宽度落在 `box_width ± ε`（既没完全闭合=空抓，也没保持张开）；③ box 在 `hand_tcp` 附近的期望区域内。对应计划书 7.2 第 5 条："不能只以轨迹执行结束作为成功" |
| 失败码（先定义，本周只会用到前几个） | `NO_OBJECT` / `GRASP_EMPTY` / `SLIP` / `TIMEOUT` / `PLACE_MISSED` / `UNEXPECTED_CONTACT` |
| 验收 | 手动发命令：空抓（无物体）、正常夹住、夹住后人为给 box 加初速度制造 slip，三种情况下三个信号的实测数值表——**这张表就是判据阈值的依据**，不能凭感觉取 |
| 预期要讲的概念 | position servo 下"夹住"为什么表现为稳态位置误差而不是力；`max_effort` 在只有位置伺服的模型里能不能实现（大概率不能——要说清它会被静默忽略还是该显式拒绝）；为什么不直接用 `GripperCommand` action |

### 2.4 Stage I — FSM 与 WaypointSource 抽象（新包 `task_executor`）

| | 内容 |
|---|---|
| 改动 | ① 新包 `src/task_executor`（C++ 节点，`use_sim_time=true`）；② 状态机 `HOME → PREGRASP → GRASP → CLOSE → LIFT → PREPLACE → PLACE → OPEN → RETRACT → VERIFY → DONE/RECOVER`；③ **`WaypointSource` 接口**：`jointTargetFor(Phase, object_pose) -> vector<double>`，本周只实现 `KeyframeWaypointSource`（手调关节空间 waypoint 查表），**第3周把实现换成 damped least-squares diff-IK，FSM 一行不改**；④ 恢复动作：重试、回安全位、有限次数后放弃 |
| 到位判据 | `max|q - q_target| < ε` **且** `max|qvel| < δ` **且** 带超时。**ε 不能取零**——position servo 有稳态误差（[10.7](week1.md#107-怎么快速做一次独立的-fk-验证) 实测 `hand_tcp` 差 7mm 的成因就是它）。ε/δ 的实测依据要写进笔记，数值进 architecture.md |
| 验收 | 单次 pick-and-place 全流程跑通，日志逐阶段打印：阶段名、目标构型、到位误差、耗时、退出原因。**至少人为制造一次失败**（把 box 挪到抓不到的位置）确认 FSM 走 RECOVER 而不是卡死或假成功 |
| 预期要讲的概念 | 为什么 FSM 是独立进程（对比放进 bridge 的回调里）；executor 该跑多少 Hz、和 `/clock` 的关系；"到位"这件事为什么没有通用判据；把 `WaypointSource` 做成接口的代价（多一层间接）与收益（第3周的 IK 有现成回归对照） |
| 注意 | 本周是**固定物体位姿**。随机化位置/偏航角是计划书 2.1 的下一步，等第3周 IK 到位才有意义（手调 waypoint 无法泛化） |

### 2.5 Stage J — episode runner、20 次连跑与第一段 Demo

| | 内容 |
|---|---|
| 改动 | ① `scripts/` 或新增 `experiment_runner` 里的 episode runner：调 `~/reset` → 触发一次任务 → 收结果 → 循环 N 次；② 输出 CSV 到 `results/`（每 episode：seed/物体位姿/成功与否/失败码/各阶段耗时）；③ `pick_place_demo.launch.py`（bridge + executor + rviz，`LIBGL_ALWAYS_SOFTWARE=1` 照旧）；④ launch 层暴露节点参数（解锁 week1 反向清单末条）；⑤ 录一段 rosbag |
| 验收 | **固定物体位姿下连续 20 次成功，无非预期碰撞**（计划书 5.3 第 4 条）；CSV 里 20 行全绿；失败码分布表（哪怕全零也要有这张表的产出路径） |
| 预期要讲的概念 | `~/reset` 作为 episode 边界的局限（[week1 反向清单](week1.md#132-反向清单现在就该做的属于缺一次推演)：reset 的跳变对下游不可见，而 episode runner 是**第一个真正跨 episode 比较数值的下游**）；rosbag2 记什么话题、sim time 下回放的坑；20 次"连续成功"和"20 次里成功 20 次"的区别 |

## 3. 本周验收标准

从计划书 [5.3](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md) 里挑出本周真正能达到的子集，**其余明确推给第3周**：

| # | 标准 | 出处 | 本周范围 |
|---|---|---|---|
| 1 | `T_ab * T_bc = T_ac`、inverse、四元数归一化、frame 方向有单元测试 | 5.3 第1条 | **本周做**（Stage F 的 `frame_math`，覆盖父相对变换那部分） |
| 2 | 固定物体位姿下连续成功 20 次，无非预期碰撞 | 5.3 第4条 | **本周做**（Stage J） |
| 3 | 日志能显示每个阶段、目标 frame、末端误差、失败原因 | 5.3 第5条 | **部分**：阶段/目标/误差/失败码本周做；**最小奇异值推到第3周**（需要 Jacobian） |
| 4 | MuJoCo 与 MoveIt FK 一致性固定样例测试 | 5.3 第2条 | **推到第3周**（需要 MoveIt 配置接入；Stage D 已有一次手工对照） |
| 5 | Jacobian 解析值 vs 有限差分 | 5.3 第3条 | **推到第3周** |

另外本周自加一条（不在计划书里，但是 E 类从零到一的判据）：**故意注入错误后单测必须变红**。测试不报错不等于测试有效。

## 4. 要回头解锁的 Week 1 悬挂项

按 [STUDY_NOTES_GUIDE.md 4.5](../../STUDY_NOTES_GUIDE.md)："解锁节点到了要主动回头"。第1周标注的三个解锁节点里，**"第4周加物体"这个节点本周就提前到了**（Stage G 就加物体）。

| week1 条目 | 本周能做到哪一步 |
|---|---|
| [13.1 #2](week1.md#131-清单) timestep = 0.002 凭什么 | **只做观察**：Stage G 之后第一次有接触，可以实验调大 timestep 看穿透/抖动。但接触仍很稀疏（一个方块一个桌面），**不下结论**，留到第4~5周 |
| [13.1 #6](week1.md#131-清单) `body_jntnum == 0 → static` 的例外集合 | vendor 的 `mjx_single_cube.xml` 里就有一个 `mocap="true"` body，可以直接加载它实测判据会怎么错，然后决定要不要现在就把判据改成 `body_jntnum == 0 && body_mocapid < 0` |
| [13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) `jnt_qposadr`/`jnt_dofadr` 写混无告警 | **本周解决**（Stage F 的 free joint fixture） |
| [13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) `MujocoApi` 签名手抄错编译器不报错 | **本周解决**（Stage F 的 `decltype`） |
| [13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) `publishTransforms` 数学无自动化断言 | **本周解决**（Stage F 的 `frame_math` 单测） |
| [13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) `onReset` 是第一个天然可单测的单元 | **本周解决**（Stage F 的 `state_ops` 单测） |
| [13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) 没有常驻 RTF 监控 | **本周顺手做**（Stage F） |
| [13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) `tf_rate_hz` 与 `joint_state_rate_hz` 不一致无告警 | **本周顺手做**（Stage F） |
| [13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) reset 的跳变对下游不可见 | **触发条件本周到了**：episode runner 是第一个跨 episode 比较数值的下游。Stage J 判断要不要做（发事件话题 or 序号断点标记） |
| [13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) launch 未暴露节点参数 | **本周做**（Stage J） |
| [13.1 #3](week1.md#131-清单) `effort` 填 `qfrc_actuator` 对不对 / [13.1 #4](week1.md#131-清单) 100Hz 够不够 / [13.1 #5](week1.md#131-清单) TF 由仿真发是否常规 / [13.1 #7](week1.md#131-清单) `home` vs `ready` / [13.1 #8](week1.md#131-清单) 仿真专有接口隔离 / [13.1 #9](week1.md#131-清单) 力矩控制模式 | **继续挂着**，解锁条件没到（真机 / 第6周 MoveIt / 第5周力控） |

## 5. 开周就该注意的（Claude 提示，尚未讨论）

按 [STUDY_NOTES_GUIDE.md 第5节](../../STUDY_NOTES_GUIDE.md)，这几条是"你还没问但值得注意的"。**点出来，不自问自答**，等到对应 stage 再展开或决定忽略。

1. **加了 box 之后 vendor 的 `home` keyframe 会失配**（E/C 类）— `<key qpos>` 的长度必须等于 `nq`；加一个 freejoint 就把 `nq` 从 9 推到 16。值得问的是：**长度写错时 MuJoCo 是加载失败、静默截断，还是读越界？** 三种行为对应三种完全不同的防护手段，现在不知道是哪种。
2. **"到位"阈值 ε 目前没有任何依据**（C 类）— 用零误差判据会永远卡住（position servo 的稳态误差是[实测过的](week1.md#107-怎么快速做一次独立的-fk-验证) mm 级）；但阈值取太松会让 FSM 在没真到位时就推进到下一阶段，**表现为偶发失败而不是报错**。Stage I 之前该先问"这个数该怎么测出来而不是猜出来"。
3. **"20 次连续成功"在确定性仿真里可能是假的**（F 类，知识边界）— 固定物体位姿 + 固定 waypoint + 确定性物理 = 20 次跑的其实**几乎是同一条轨迹**。那这个 20 次到底验证了什么？值得在 Stage J 之前想清楚，否则会拿一个没有信息量的数字当验收通过。
4. **`max_effort` 字段可能无法实现**（C 类）— MJCF 现在只有位置伺服 actuator（[11.4](week1.md#114-position-servo-actuator-的本质一个-pd-控制器)）。如果 `GripperCommand.max_effort` 被静默忽略，下游会以为自己在控制夹持力。**该显式拒绝、该 WARN、还是该改模型**，是 Stage H 要定的。

## 6. 悬挂问题（本周新增）

### 6.1 清单

> 待各 stage 结束后按 [4.3 的判据](../../STUDY_NOTES_GUIDE.md) 分流填入（缺参照系 → 这里）。

### 6.2 反向清单：现在就该做的

> 待填（缺一次推演 → 这里）。第1周遗留、本周不打算做的条目**不要**复制过来，仍以 [week1.md 13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) 为权威，本节只放本周新产生的。
