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
- [7. Stage F：测试地基与胶水层收口](#7-stage-f测试地基与胶水层收口)
  - [7.0 一句话总结](#70-一句话总结)
  - [7.1 改动清单与验证结果](#71-改动清单与验证结果)
  - [7.2 `decltype` 签名：把手抄错误从运行时 UB 变成编译错误](#72-decltype-签名把手抄错误从运行时-ub-变成编译错误)
  - [7.3 剥离边界画在哪：为什么 frame_math/state_ops 剥，`buildJointIndex` 暂不剥](#73-剥离边界画在哪为什么-frame_mathstate_ops-剥buildjointindex-暂不剥)
  - [7.4 测试里怎么拿模型文件路径：三个选项](#74-测试里怎么拿模型文件路径三个选项)
  - [7.5 `colcon test` / `gtest` 怎么用：三层机制](#75-colcon-test--gtest-怎么用三层机制)
  - [7.6 RTF 是什么，为什么要常驻监控，为什么 `tf_rate_hz` 要和 `joint_state_rate_hz` 一致](#76-rtf-是什么为什么要常驻监控为什么-tf_rate_hz-要和-joint_state_rate_hz-一致)
  - [7.7 失败模式与验证手段](#77-失败模式与验证手段)
  - [7.8 排查记录：`free_body.xml` 注释里的 `--` 为什么没被 gtest 挡住](#78-排查记录free_bodyxml-注释里的----为什么没被-gtest-挡住)

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

- **`ament_lint_auto` 从未通过过，涉及 `mujoco_bridge` 全部源文件**——Stage F 第一次跑 `colcon test`（本仓库此前一次都没跑过），暴露出 `cpplint`（版权头缺失、include 顺序）、`uncrustify`（`mujoco_bridge_node.cpp` 9 行、`mujoco_dl.cpp` 44 行格式差异）全部不过。gtest 部分（`test_frame_math`、`test_state_ops`）已全绿，这批是独立的风格债务，不是逻辑 bug，本周决定不修（`uncrustify --fix` 会改动 week1 笔记里逐行引用过的代码，需要专门一次处理并核对笔记引用是否还对得上行号）。
- **RTF 监控目前只打日志，没有阈值告警**（[7.6](#76-rtf-是什么为什么要常驻监控为什么-tf_rate_hz-要和-joint_state_rate_hz-一致)、[7.7](#77-失败模式与验证手段)）——FSM/episode runner 要知道"RTF 掉到多少算异常"必须人眼盯日志。解锁条件：Stage I 给 FSM 超时判据接入 RTF 时，评估要不要把这个做成可查询的话题或参数化阈值。
- **`tf_rate_hz != joint_state_rate_hz` 的告警本周加了代码但从没被真实触发过**（[7.6](#76-rtf-是什么为什么要常驻监控为什么-tf_rate_hz-要和-joint_state_rate_hz-一致)、[7.7](#77-失败模式与验证手段)）——两者目前都还是默认的 100Hz。解锁条件：以后真的把 `tf_rate_hz` 调开（比如给 FSM 提供更高频姿态反馈）时，第一次触发也是第一次验证这段代码本身是对的。

## 7. Stage F：测试地基与胶水层收口

### 7.0 一句话总结

给 `mujoco_bridge` 建了本仓库第一个 `colcon test` 目标：把 `publishTransforms` 里的相对变换数学剥成不依赖 `rclcpp`/`mjModel` 的纯函数 [frame_math](../../src/mujoco_bridge/include/mujoco_bridge/frame_math.hpp)，把 `onReset` 里对 `mjData` 的操作剥成不依赖节点生命周期的自由函数 [state_ops](../../src/mujoco_bridge/include/mujoco_bridge/state_ops.hpp)，各配一个 gtest 目标；`MujocoApi` 的 12 个函数指针字段改成 `decltype(&mj_xxx)`，把"签名手抄错"从运行时 UB 变成编译错误；新增一个带 `freejoint` 的最小 MJCF fixture，专门用来抓 `jnt_qposadr`/`jnt_dofadr` 混用（Panda 上 `nq==nv` 测不出这类 bug）；顺手加了 RTF 常驻监控、`tf_rate_hz`/`joint_state_rate_hz` 不一致告警、两个手动验证脚本的 `use_sim_time`。过程中意外抓到一个真语法错误（[7.8](#78-排查记录free_bodyxml-注释里的----为什么没被-gtest-挡住)）。

### 7.1 改动清单与验证结果

**改动**：

- [mujoco_dl.hpp](../../src/mujoco_bridge/include/mujoco_bridge/mujoco_dl.hpp)：`MujocoApi` 12 个字段全部改 `decltype(&mj_xxx)` / `decltype(&mju_xxx)`
- [frame_math.hpp](../../src/mujoco_bridge/include/mujoco_bridge/frame_math.hpp) / [frame_math.cpp](../../src/mujoco_bridge/src/frame_math.cpp)：新增 `relativePose`，从 `publishTransforms` 剥出来的纯函数
- [state_ops.hpp](../../src/mujoco_bridge/include/mujoco_bridge/state_ops.hpp) / [state_ops.cpp](../../src/mujoco_bridge/src/state_ops.cpp)：新增 `resetToKeyframe`，从 `onReset` 剥出来的自由函数
- [mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp)：`publishTransforms`/`onReset` 改为调用上面两个函数；`onTimer` 里加 `logRtfIfDue()`；`decimationFor` 两次调用后加 `tf_rate_hz != joint_state_rate_hz` 的 `WARN`
- [test/fixtures/free_body.xml](../../src/mujoco_bridge/test/fixtures/free_body.xml)（新）：一个 free-joint box + 一个 hinge joint 的最小场景，`nq=8, nv=7`
- [test/test_frame_math.cpp](../../src/mujoco_bridge/test/test_frame_math.cpp)、[test/test_state_ops.cpp](../../src/mujoco_bridge/test/test_state_ops.cpp)（新）
- [CMakeLists.txt](../../src/mujoco_bridge/CMakeLists.txt)：两个 `ament_add_gtest` 目标；`state_ops` 测试用 `target_compile_definitions` 传 fixture 目录
- [demo.launch.py](../../src/mujoco_bridge/launch/demo.launch.py)：bridge node 加 `parameters=[{'use_sim_time': True}]`
- [sine_joint_test.py](../../scripts/sine_joint_test.py) / [gripper_test.py](../../scripts/gripper_test.py)：构造函数加 `parameter_overrides=[Parameter('use_sim_time', ...)]`

**编译**：

```
Starting >>> robot_description
Finished <<< robot_description [0.08s]
Starting >>> mujoco_bridge
Finished <<< mujoco_bridge [5.95s]

Summary: 2 packages finished [6.14s]
```

**测试**（`colcon test --packages-select mujoco_bridge`，最终状态）：

```
test_frame_math.gtest.xml: 3 tests, 0 errors, 0 failures, 0 skipped
test_state_ops.gtest.xml:  6 tests, 0 errors, 0 failures, 0 skipped
xmllint.xunit.xml:         2 tests, 0 errors, 0 failures, 0 skipped
```

`ament_lint_auto` 剩余的 `cpplint`/`uncrustify` 失败是独立的历史风格债务（本仓库这是第一次跑 `colcon test`），不属于本 stage 范围，见 [6.2](#62-反向清单现在就该做的)。

**"故意注入错误，测试必须变红"实测**（本周 §3 新加的验收标准）：

| 注入的错误 | 位置 | 结果 |
|---|---|---|
| `mulQuat` 参数顺序颠倒 | `frame_math.cpp` | 最初两个用例**没有变红**——两个用例都恰好有一侧是单位四元数，四元数乘法和单位元可交换，颠倒参数结果不变。补了第三个双方都非平凡旋转的用例后，注入同样的错误：**1 个测试失败**，抓到了 |
| 去掉 `resetToKeyframe` 里保持 `d->time` 的那两行 | `state_ops.cpp` | `ResetPreservesSimTime` **变红**（1 failure / 6 tests） |
| 改回原状 | 两处都还原 | 全绿（3/0、6/0） |

第一次尝试没抓到 bug 这件事本身就是发现——见 [7.7](#77-失败模式与验证手段) 和 [7.8](#78-排查记录free_bodyxml-注释里的----为什么没被-gtest-挡住)。

**行为回归验证**（重构没有改变运行时行为）：环境卫生按 [STUDY_NOTES_GUIDE 3.1](../../STUDY_NOTES_GUIDE.md) 确认 `ps -eo pid,comm` 干净、`/clock` 无残留发布者，直接跑可执行文件（不经 `ros2 run`）：

```
[INFO] [mujoco_bridge]: Loaded .../panda.xml (nq=9, timestep=0.0020s)
[INFO] [mujoco_bridge]: 9 actuated joints: joint1, joint2, ..., finger_joint1, finger_joint2
[INFO] [mujoco_bridge]: TF: 3 static, 9 dynamic frames
[INFO] [mujoco_bridge]: actuators: 7 arm joint(s) mapped, gripper actuator found
[INFO] [mujoco_bridge]: /joint_states every 5 steps (100.0 Hz requested, 100.0 Hz actual)
[INFO] [mujoco_bridge]: /tf every 5 steps (100.0 Hz requested, 100.0 Hz actual)
[INFO] [mujoco_bridge]: RTF: 0.64
[INFO] [mujoco_bridge]: RTF: 1.00   # 之后稳定在 1.00
```

`ros2 topic echo /tf --once`：`link0 -> link1`，`z: 0.333`，旋转四元数为单位——和重构前 [9.1](week1.md#91-改动清单与验证结果) 记录的值一致，`relativePose` 替换手写数学没有改变结果。

`ros2 service call /mujoco_bridge/reset std_srvs/srv/Trigger`：`success=True, message='reset to keyframe `home`'`——`resetToKeyframe` 替换 `onReset` 里手写的三行也没有改变行为。

`tf_rate_hz`/`joint_state_rate_hz` 一致性告警本次**没有**观察到触发（两者都是默认 100Hz），这条留在了 [6.2](#62-反向清单现在就该做的) 里——代码加了但从没被验证过真的会响。

### 7.2 `decltype` 签名：把手抄错误从运行时 UB 变成编译错误

Week1 的 `MujocoApi` 是手写函数指针签名：

```cpp
void (*mulQuat)(mjtNum * res, const mjtNum * quat1, const mjtNum * quat2);
```

如果手抄时哪个参数类型、个数、顺序抄错了，`resolve()` 里的 `reinterpret_cast<FuncPtr>(dlsym(...))` **无条件成功**——`dlsym` 只按符号名字查，不关心调用方声明的签名对不对，编译器也没有真实的 `mj_mulQuat` 声明可以拿来比对。错误的后果是运行时未定义行为（参数个数不匹配可能栈错位，类型不匹配可能读错内存），且很可能"恰好没崩"，长期潜伏。

改成：

```cpp
decltype(&mju_mulQuat) mulQuat;
```

`decltype(&mju_mulQuat)` 直接从 `<mujoco/mujoco.h>` 里那个真实声明推导出函数指针类型，`resolve()` 赋值给 `api.mulQuat` 时走的是普通的指针类型检查——如果 `<mujoco/mujoco.h>` 升级后 `mju_mulQuat` 的签名变了，`resolve` 模板内部那个 `reinterpret_cast` 目标类型也会跟着变，之前"手抄的签名和头文件不一致"这类错误从"运行时可能崩可能不崩"变成"编译不过"。

值得注意的一点：这只解决"签名和头文件不一致"，解决不了"头文件声明是对的，但我们对这个函数该怎么用的理解错了"——比如上一轮已经在 `frame_math` 测试里抓到的 `mulQuat` 参数顺序错误，`decltype` 对这类错误完全无能为力，因为两种参数顺序在 C++ 类型系统看来是**同一个类型**（`void(*)(mjtNum*, const mjtNum*, const mjtNum*)` 不区分"第二个参数该传 parent 还是 child"）。这类错误仍然只能靠 [7.7](#77-失败模式与验证手段) 里说的单测抓。

### 7.3 剥离边界画在哪：为什么 frame_math/state_ops 剥，`buildJointIndex` 暂不剥

`buildJointIndex`/`buildFrameIndex`/`buildActuatorIndex` 同样在遍历 `mjModel`，但这次没有剥它们。判据不是代码长度，是**本周计划书 [2.1.1](#211-这类系统该怎么测四层本周只取前两层) 定的一句话**："如果这个东西写错了，第一个发现它的是人眼看 RViz，那它就该有测试"——反过来说，如果写错了本来就会在启动日志里立刻炸出来（比如 `buildJointIndex` 打印的关节名列表一眼就能看出顺序不对/缺关节），剥离的边际收益低。

`frame_math`/`state_ops` 剥的理由正好相反：`relativePose` 算错了，TF 显示出来的是"看起来合理但差几度"的姿态——不崩溃、不报警、只有拿基准值比对才能发现（[9.12](week1.md#912-失败模式与验证手段) 早就把这个记成"⚠️"）。`resetToKeyframe` 里漏了保持 sim time 那一步，症状是"reset 之后时间跳回 0"，这种事故会传播到整张 TF 树和所有下游节点的时间缓存，但**不会有任何一条日志或异常报出来**——这正是 [4.2 C类问题](../../STUDY_NOTES_GUIDE.md) 定义的"改了没人发现"。

### 7.4 测试里怎么拿模型文件路径：三个选项

写 `test_state_ops.cpp` 时要加载 `test/fixtures/free_body.xml`，路径怎么给测试二进制是个真实取舍：

| 方案 | 做法 | 问题 |
|---|---|---|
| `ament_index_cpp::get_package_share_directory` | 运行时按包名查 share 目录 | 要求先 `install`；纯源码树、还没装的包查不到；而且这是给"生产代码"用的机制，测试引入这个依赖只是为了找一个开发时的固定文件，方向反了 |
| 相对当前工作目录拼路径 | `"test/fixtures/free_body.xml"` | `colcon test` 从哪个 cwd 调用测试二进制不是测试作者能控制的约定，脆弱 |
| **编译期宏**（本次用的） | `CMakeLists.txt` 里 `target_compile_definitions(test_state_ops PRIVATE TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")` | 路径写死在这次编译的构建配置里，换机器要求重新配置（但 `CMAKE_CURRENT_SOURCE_DIR` 在配置阶段求值，换机器时 CMake 会重新算，其实不算真代价） |

选第三种的理由是它和 `install`、和 cwd 完全解耦——测试二进制自己知道源码在哪，不用猜运行时环境。

### 7.5 `colcon test` / `gtest` 怎么用：三层机制

> Q: 你还要讲讲测试方面的知识，比如 colcon test，gtest 等等要怎么使用，具体来说，怎么针对这个 Node 的 test 文件夹使用。

分三层，各自独立，排查时经常要跳到最底层：

**第一层，CMake 声明**：`ament_add_gtest(test_frame_math test/test_frame_math.cpp src/frame_math.cpp src/mujoco_dl.cpp)` 是对"编一个可执行文件 + 链接 gtest 的 `main` + 向 CTest 注册一条测试用例"的封装。**每个 `ament_add_gtest` 调用产出一个独立可执行文件**，不是所有 `TEST()` 挤进一个二进制——这次是两个文件对两个可执行文件。两个测试目标各自重新列出自己需要的 `.cpp`，不链接 `mujoco_bridge_node` 主可执行文件：测试不需要、也不该依赖 `main()` 和整个节点，只需要被测函数依赖的那几个源文件。这也是"剥成自由函数"在构建层面的必要性——如果 `resetToKeyframe` 还焊在 `MujocoBridgeNode` 类里，这里就没法只拉一个函数出来单独编译测试。

**第二层，`colcon test` 驱动**：

```bash
colcon build --packages-select mujoco_bridge
colcon test --packages-select mujoco_bridge
colcon test-result --verbose
```

`colcon test` 对每个包在其 CMake 构建目录里跑 `ctest`——`ament_lint_auto` 注册的那些静态检查也是通过 `add_test()` 挂到 CTest 上的，对 CTest 来说 gtest 和 lint 检查是**同一种东西**：一条测试用例，一个通过/失败。这就是为什么这次 2 条 gtest 失败和 41 条 lint 失败会混进同一份汇总（[7.1](#71-改动清单与验证结果)）。结果写到 `build/mujoco_bridge/test_results/mujoco_bridge/*.xml`（JUnit 格式），`colcon test-result` 只是读这些 xml 汇总打印，不重新跑测试。`--ctest-args -R "test_frame_math|test_state_ops"` 把正则透传给底层 `ctest`，用来只跑匹配的测试、跳过已知的 lint 失败。

**第三层，直接跑 gtest 二进制**（日常开发最快的路径）：`ament_add_gtest` 编出来的可执行文件本身是个标准 gtest 二进制，装在 `build/mujoco_bridge/` 下，可以完全绕开 colcon/ctest：

```
$ ./build/mujoco_bridge/test_frame_math --gtest_list_tests
FrameMath.
  IdentityParentReturnsChildPoseVerbatim
  RotatedParentRotatesRelativePosition
  NonCommutingRotationsCatchArgumentOrderSwap

$ ./build/mujoco_bridge/test_state_ops --gtest_filter="*ResetPreservesSimTime*"
[==========] Running 1 test from 1 test suite.
[ RUN      ] StateOps.ResetPreservesSimTime
[       OK ] StateOps.ResetPreservesSimTime (1 ms)
[  PASSED  ] 1 test.
```

常用参数：`--gtest_list_tests`（列出不执行）、`--gtest_filter=Suite.Case`（支持 `*` 通配符，`-` 前缀排除）、`--gtest_repeat=N`（重复跑，抓偶发失败——这个包目前都是确定性 fixture，用不上，Stage G/H 涉及物理仿真后会有用）、`--gtest_break_on_failure`（失败即 abort，方便接 `gdb --args`——它是个普通可执行文件，不需要 ROS 那一层）。

三层的关系：**`ament_add_gtest` 决定"编出什么"，`colcon test` 决定"批量跑哪些、汇总到哪"，直接调用二进制决定"这一次具体盯哪个用例"**。改代码→ 增量 `colcon build` → 直接跑 `./build/mujoco_bridge/test_state_ops --gtest_filter=...` 是最快的循环；提交前再跑一次 `colcon test` 做全量确认。

### 7.6 RTF 是什么，为什么要常驻监控，为什么 `tf_rate_hz` 要和 `joint_state_rate_hz` 一致

> Q: RTF是什么意思，为什么会需要常驻RTF监控，为什么joint state的发布频率要和tf的发布频率一致？

**RTF（Real-Time Factor，实时率）** = 一段窗口内 **sim time 前进的量 / wall time 经过的量**。`logRtfIfDue()`（[mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp)）按 1 秒窗口算：

```cpp
RTF = (data_->time 这次 - data_->time 上次) / (墙钟这次 - 墙钟上次)
```

`RTF ≈ 1` 是仿真跟真实时间同步前进；`RTF < 1` 是仿真变慢（"慢放"）；`RTF > 1` 是变快。这次实测日志：

```
RTF: 0.64   ← 刚启动那一秒，进程/DDS 初始化开销拉慢了这一窗口
RTF: 1.00   ← 之后稳定
```

**为什么需要常驻监控**：`onTimer()` 是墙钟定时器（[week1 8.2](week1.md#82-sim-time-vs-wall-time以及-use_sim_time)），每 `timestep_s`（2ms）触发一次 `mj_step`，隐含假设是"`mj_step` 本身的计算耗时远小于 2ms"。这个假设一旦破了——**什么都不会发生**：不报错、不崩溃、不告警。ROS2 的墙钟定时器只是"尽量按周期触发"，跟不上就在下一个机会点再触发，`data_->time` 前进的速度悄悄变慢，`/clock`/`/joint_states` 照常在发，只是它们描述的仿真世界慢了，没有任何一条日志说明这件事。这正是 [C 类问题](../../STUDY_NOTES_GUIDE.md)（"如果这里变慢了，我怎么知道？"）：本周开始才真的会咬人——Stage I 的 FSM 到位判据按 sim time 定超时阈值，RTF 掉下去时一个正常推进的任务可能被误判超时；Stage J 的 20 次连跑如果某几次 RTF 意外下降，episode 耗时这个指标就会失真，分不清是任务变难还是机器打嗝。RTF 监控把这个本来完全不可观测的失效模式变成一条每秒能看到的数字。

**为什么 `tf_rate_hz` 要和 `joint_state_rate_hz` 一致**：不是硬性物理约束，是**避免下游拿到不同步快照**的工程约定。两者都从同一条物理步进流水线按整数步数抽取（`decimationFor`），如果 decimation 不同（比如 5 步 vs 10 步），`/joint_states` 和 `/tf` 描述的**不是同一批物理步**。具体到本周场景：Stage H 的抓取成功判据要同时看夹爪实测宽度（来自 `/joint_states`）和 box 相对 `hand_tcp` 的位置（来自 `/tf`）——如果两个话题发布节奏不对齐，"取最新一帧拼起来判断"拼出来的两个数字可能来自不同的仿真时刻。大多数时候差几十毫秒看不出来，但在快速运动（抓取瞬间）时会让联合判据出现说不清楚的抖动，**且没有任何报错**，只表现成"这个 stage 的验收有时候莫名其妙不过"。代码本身不阻止把两个频率配成不同值，配成不同值就是把"同一时刻的联合判断"从自动成立变成需要额外时间对齐才能保证——而当前所有下游代码都没做这个对齐，所以这条告警才存在。目前两者都还是默认 100Hz，这条告警从没被真实触发验证过（见 [6.2](#62-反向清单现在就该做的)）。

### 7.7 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现 / 怎么防住 |
|---|---|---|
| `MujocoApi` 签名手抄错 | 以前是运行时 UB，可能崩可能不崩 | 改 `decltype(&mj_xxx)`，签名不一致直接编译失败（[7.2](#72-decltype-签名把手抄错误从运行时-ub-变成编译错误)） |
| `mulQuat`/`negQuat`/`rotVecQuat` 参数顺序颠倒 | TF 姿态"看起来合理但差几度"，不崩溃不报警 | `test_frame_math`；**但两个"一侧是单位四元数"的用例测不出参数顺序颠倒**（四元数乘法和单位元可交换），必须有一个双方都非平凡旋转的用例（[7.1](#71-改动清单与验证结果) 的注入实验当场证实） |
| `jnt_qposadr`/`jnt_dofadr` 混用 | Panda 上 `nq==nv`，两套地址数值相同，完全测不出来 | `free_body.xml` fixture 故意做出 `nq=8, nv=7`，让 hinge joint 的两个地址不同（`test_state_ops.HingeJointQposAdrAndDofAdrDiffer`） |
| `resetToKeyframe` 漏保持 sim time | reset 后时间跳回 0，传播到整张 TF 树和所有下游节点的时间缓存，**没有任何日志或异常** | `test_state_ops.ResetPreservesSimTime`；注入去掉那两行的错误，实测变红 |
| XML 注释里出现连续 `--` | 严格解析器（`xmllint`）报错，但 `mj_loadXML`（MuJoCo bundle 的宽松 tinyxml2）照常加载，gtest 全绿 | 见 [7.8](#78-排查记录free_bodyxml-注释里的----为什么没被-gtest-挡住)。**"能被某个解析器接受"不等于"合规"**，两种解析器对同一份文件的判断可以完全相反 |
| RTF 掉到 <1 | 系统只是"慢放"，不报任何错 | 常驻 RTF 监控（[7.1](#71-改动清单与验证结果) 日志里的 `RTF: 0.64 → 1.00`），继承自 week1 [8.6](week1.md#86-失败模式与验证手段) 记的缺口，本周补上 |
| `tf_rate_hz` != `joint_state_rate_hz` | `/tf` 和 `/joint_states` 样本对不齐，下游拿不到"同一时刻"的数据 | 已加告警，但**本次两者都是默认值，从没触发过**——见 [6.2](#62-反向清单现在就该做的)，这是"写了但没验证过"的已知缺口 |

### 7.8 排查记录：`free_body.xml` 注释里的 `--` 为什么没被 gtest 挡住

**现象**：上一轮 `colcon test` 有 41 条失败，全部当作"`ament_lint_auto` 风格债务、本周不修"归进了反向清单。用户追问："当前的 `free_body.xml` 是存在语法错误的，`--` 符号会被识别成特殊用途字符，所以你的测试为什么还能通过？"

**线索**：先用严格校验器直接测：

```
$ xmllint --noout test/fixtures/free_body.xml
free_body.xml:4: parser error : Double hyphen within comment: <!-- ... differ -- a jnt_qposadr ...
exit code: 1
```

确认这确实是一个真语法错误——XML 规范规定注释内部不能出现连续 `--`（唯一允许出现的位置是注释结尾的 `-->`）。再去翻上一轮 `ament_lint_auto` 跑出的 `xmllint.xunit.xml`，`test/fixtures/free_body.xml` 那条失败信息第一行就是这句 parser error——**这个 bug 早就被抓到过，只是被我整批归进"风格问题"时没有单独打开看内容**。

**根因**：`test_state_ops` 走的是 `mj_loadXML`，也就是 MuJoCo bundle 的那份 tinyxml2（[mujoco_dl.hpp](../../src/mujoco_bridge/include/mujoco_bridge/mujoco_dl.hpp) 开头那段注释讲过它为什么要 dlopen 隔离）。这份解析器处理注释的实现只找"下一个 `-->`"作为结束标记，**不检查内容里有没有出现 `--`**——一个宽松但不严格合规的实现。`xmllint` 是严格校验器，会执行这条规则。同一份文件，两个解析器给出相反的判断，因为它们检查的东西本来就不一样。

**修复**：把注释里的 `differ -- a jnt_qposadr` 改成 `differ: a jnt_qposadr`，`xmllint --noout` 退出码变 0，重新跑 `colcon test`，`xmllint.xunit.xml` 从 1 failure 变 0 failure，两个 gtest 目标不受影响（本来就没依赖这条规则）。

**留下的经验**：

1. **"能被某个解析器接受"不等于"合规"**——宽松的解析器不会替你验证规范，它只是恰好没在检查这条规则。这类问题两边跑出的结果会互相矛盾，矛盾本身就是排查的起点。
2. **批量失败列表不能整批归为同一类问题**。上一轮把 41 条失败一次性打包成"风格债务、本周不修"，其中夹带了一条真语法错误没被单独看到。以后遇到成批失败，至少要扫一遍每条失败信息的第一行，不能只看"哪个 checker 报的"就归类。
3. 这条 bug 是**用户直接读代码发现的**，不是靠工具或者我主动检查出来的——再一次印证 [4.2](../../STUDY_NOTES_GUIDE.md) 里"C类问题（如果这里写错了我怎么知道）"的价值：这次的答案是"要看你信的是哪个解析器"。
