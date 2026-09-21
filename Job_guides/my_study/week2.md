# Week 2 学习笔记

> **本文件当前是开周计划稿**，只有第 1~4 节（继承事实、stage 计划、验收标准、要回头解锁的悬挂项）是现在写下的。第 5 节起的 stage 节随每个 stage 完成后就地扩写，骨架按 [STUDY_NOTES_GUIDE.md](../../STUDY_NOTES_GUIDE.md) 2.2（`N.0 一句话总结 → N.1 改动清单与验证结果 → N.x 概念讲解 → N.x 失败模式与验证手段 → N.x 排查记录`）。
>
> Stage 编号延续第1周的 A~E，本周是 **F~J**。

## 学习重点范围

本周对应计划书[第5节 Chap 3 模块](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md)与 Roadmap 第2周（"Chap 3 ground-truth pick-and-place、夹爪动作 → 固定场景连续 20 次成功；第一段 Demo"）。第1周把地基（模型加载、时间、TF、命令、复位）打完了，本周第一次出现**任务语义**：多阶段、有成功/失败、可重复跑。实际跑下来（Stage F~J）比开周时预想的范围更广——尤其是可视化架构和后台进程管理这两块完全在计划外，但分量不亚于计划内的内容，一并补进来。需要重点掌握以下几块：

1. **MJCF 场景搭建与接触参数** — 怎么在不改 vendor 文件的前提下组合出自己的场景（`<include>`、`worldbody` 追加、`<keyframe>` 必须随 `nq` 重写）；`freejoint` 的广义坐标布局（3 平移 + 4 四元数，`nq != nv` 第一次真实成立）；`condim`/`friction`/`solref` 各管什么，以及它们和 `timestep` 的关系——这是第1周悬挂清单第 2 条（timestep 凭什么取 0.002）**第一次有了实验对象**。
2. **夹爪的命令与状态** — position-servo + tendon 驱动下"夹住了"在数据上长什么样（不是力，是稳态位置误差 + 接触）；`control_msgs` 里夹爪接口的既有形状（`GripperCommand`）为什么值得现在就对齐；抓取成功怎么用**多个独立信号**判定而不是"命令发出去了"。
3. **任务状态机（FSM）与 episode 边界** — 阶段划分、到位判据（阈值 + 超时）、失败码分层、恢复动作；为什么 FSM 应该是独立节点而不是 bridge 里的一个回调；低频决策与高频物理步进怎么通过 `/clock` + `use_sim_time` 对齐。**"episode 边界 = 进程边界"这条 Stage I 一开始的隐含假设，被 Stage J 的连续多次跑需求证伪**——换成话题触发（`~/start_episode`/`~/episode_outcome`）+ 节点常驻不重启，这一次修正比最初设计本身更能看出 FSM 到底该怎么想。
4. **ground-truth（oracle）接口的隔离** — 计划书第 4 节的硬要求："ground truth 必须走独立的 oracle 接口，不能偷偷混进正式感知输出"。本周要把这条边界**在代码里画出来**，第4周换视觉时只换发布者、不改下游。
5. **可测试性从零到一，以及"能测的"和"只能实验的"分界** — 把和 `rclcpp`/`mjModel` 焊在一起的逻辑剥成纯函数，搭起第一个 `colcon test` 目标（E 类，第1周复盘里**唯一完全空缺的提问类别**，见 [STUDY_NOTES_GUIDE.md](../../STUDY_NOTES_GUIDE.md) 4.1），也是 week1 反向清单里三条互相咬合的待办的共同前置。本周还第一次划清了四层测试判据（[2.1.1](#211-这类系统该怎么测四层本周只取前两层)）——episode 成功率、20 次连续跑、rosbag 录制这些"任务层"产物故意不进 `colcon test`，进 `results/`，测试和实验是两件事。
6. **可视化架构选型（完全在计划外）** — RViz 网格显示做过一次又被回退，换成同进程直连权威 `mjData` 的 GLFW debug viewer（`mjv_*`/`mjr_*` 走既有的 dlopen 隔离机制扩展）。这条线牵出"离权威物理源距离"的三层框架（同进程直连 / 仿真器原生跨进程 / 通用消息镜像），以及一个不直观的结论：**真机的可视化天花板结构上等同于第三层**——RViz/TF 这条路线不是退而求其次，是提前适配真机将来会有的保真度上限。过程中还实测排查出一个真实的 NVIDIA/GLX 硬件加速在嵌套 X 会话下的已知限制（外部有同款 GPU 型号的 TigerVNC 案例佐证，不是这个容器独有的配置问题）。
7. **后台进程生命周期管理与 DDS 发现竞态：从"一次性坑"到"系统性质"** — `ros2 run`/`ros2 launch` 对裸信号假死是同一个 wrapper 机制，本周把这条经验的范围从"`run` 专属"扩到"`launch` 同样如此"，还顺带实测确认了 `ros2 bag record` **不属于**这一类（裸 `kill -INT` 能正常收尾）。DDS 发现竞态本周至少第三、四次独立撞见（`~/start_episode` vs `task_executor`、bag 录制里 recorder 的动态话题发现），已经从"一次具体 bug"确认成一条通用规律——任何两个独立 ROS2 进程之间新建的第一条通信连接，都要假设对方还没发现你。

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
- [8. Stage G：pick-and-place 场景与 oracle 接口](#8-stage-gpick-and-place-场景与-oracle-接口)
  - [8.0 一句话总结](#80-一句话总结)
  - [8.1 改动清单与验证结果](#81-改动清单与验证结果)
  - [8.2 MJCF `<include>` 路径解析规则：为什么新场景文件要和 panda.xml 同目录](#82-mjcf-include-路径解析规则为什么新场景文件要和-pandaxml-同目录)
  - [8.3 qpos 顺序怎么确定：深度优先遍历 + 文档顺序](#83-qpos-顺序怎么确定深度优先遍历--文档顺序)
  - [8.4 `qpos` 与 `xpos`/`xquat`：为什么 oracle 发布读后者](#84-qpos-与-xposxquat为什么-oracle-发布读后者)
  - [8.5 `condim`/`friction`/`solref`：接触参数详解，为什么其中两个当前是死代码](#85-condimfrictionsolref接触参数详解为什么其中两个当前是死代码)
  - [8.6 三条可视化路径的定位：RViz / `simulate` / 工业界与学术界的两种范式](#86-三条可视化路径的定位rviz--simulate--工业界与学术界的两种范式)
  - [8.7 失败模式与验证手段](#87-失败模式与验证手段)
  - [8.8 排查记录：keyframe 名字冲突与长度不匹配是两件独立的事（结论被推翻）](#88-排查记录keyframe-名字冲突与长度不匹配是两件独立的事结论被推翻)
- [9. Stage H：夹爪命令接口与抓取成功判据](#9-stage-h夹爪命令接口与抓取成功判据)
  - [9.0 一句话总结](#90-一句话总结)
  - [9.1 改动清单与验证结果](#91-改动清单与验证结果)
  - [9.2 position servo 下"夹住"为什么是稳态位置误差，不是力](#92-position-servo-下夹住为什么是稳态位置误差不是力)
  - [9.3 为什么不直接用 `GripperCommand` action](#93-为什么不直接用-grippercommand-action)
  - [9.4 `mjContact` 与 `bodiesInContact`：接触检测怎么工作](#94-mjcontact-与-bodiesincontact接触检测怎么工作)
  - [9.5 `hand_tcp` 为什么要合成、为什么只有 z 轴有偏移](#95-hand_tcp-为什么要合成为什么只有-z-轴有偏移)
  - [9.6 夹爪构型：为什么同时解析 `hand` 和 `finger` 两套 body id](#96-夹爪构型为什么同时解析-hand-和-finger-两套-body-id)
  - [9.7 oracle 数据的分类：哪些会随项目推进消失，哪些需要学习方法](#97-oracle-数据的分类哪些会随项目推进消失哪些需要学习方法)
  - [9.8 失败模式与验证手段](#98-失败模式与验证手段)
- [10. Stage I：FSM 与 `WaypointSource` 抽象（新包 `task_executor`）](#10-stage-ifsm-与-waypointsource-抽象新包-task_executor)
  - [10.0 一句话总结](#100-一句话总结)
  - [10.1 改动清单与验证结果](#101-改动清单与验证结果)
  - [10.2 纯函数层为什么不用消息类型：Layer 1 的"message-free"原则](#102-纯函数层为什么不用消息类型layer-1-的message-free原则)
  - [10.3 FSM 的输入、参数、决策与消费：不看 if-else 的整体形状](#103-fsm-的输入参数决策与消费不看-if-else-的整体形状)
    - [10.3.1 `RECOVER` 完整机制：从触发到重试到力竭（此前散落在各处，没有整体讲过）](#1031-recover-完整机制从触发到重试到力竭此前散落在各处没有整体讲过)
    - [10.3.2 `FsmParams` 完整设计：这些参数怎么编码了整个抓取过程的阶段设计](#1032-fsmparams-完整设计这些参数怎么编码了整个抓取过程的阶段设计)
    - [10.3.3 `step()`/`FsmDecision` 的设计形状（不看具体实现，只看设计契约）](#1033-stepfsmdecision-的设计形状不看具体实现只看设计契约)
    - [10.3.4 `requestReset()` 是节点调 service 的一般写法吗，和命令行 `ros2 service call` 有什么关系](#1034-requestreset-是节点调-service-的一般写法吗和命令行-ros2-service-call-有什么关系)
  - [10.4 为什么选状态机模型？工程实践里的适用场景与替代方案](#104-为什么选状态机模型工程实践里的适用场景与替代方案)
  - [10.5 `WaypointSource` 接口设计：为什么现在只有一个查表实现](#105-waypointsource-接口设计为什么现在只有一个查表实现)
    - [10.5.1 这些关节数字的依据在哪：实测搜索过程（此前只在会话记录里，未落盘，这次补上）](#1051-这些关节数字的依据在哪实测搜索过程此前只在会话记录里未落盘这次补上)
  - [10.6 到位判据为什么分层：三条独立的"等一等"，不是同一件事](#106-到位判据为什么分层三条独立的等一等不是同一件事)
  - [10.7 为什么 `task_executor` 不发布/订阅一个 `GraspOutcome` 话题](#107-为什么-task_executor-不发布订阅一个-graspoutcome-话题)
    - [10.7.1 `GraspSignals`/`classifyGrasp` 为什么物理上放在 `mujoco_bridge` 包里，不是 `task_executor`](#1071-graspsignalsclassifygrasp-为什么物理上放在-mujoco_bridge-包里不是-task_executor)
  - [10.8 `task_executor` 的可扩展性：后续步骤会替换哪些部分](#108-task_executor-的可扩展性后续步骤会替换哪些部分)
  - [10.9 实测复现：`~/gripper_command` 双发布者冲突](#109-实测复现gripper_command-双发布者冲突)
  - [10.10 排查记录：三个连续 bug，都是跑起来才炸出来的](#1010-排查记录三个连续-bug都是跑起来才炸出来的)
    - [10.10.1 第一个 bug：`kRecover → kHome` 的重试转移被 `exit_reason == kNone` 误判成"没有发生"](#10101-第一个-bugkrecover--khome-的重试转移被-exit_reason--knone-误判成没有发生)
    - [10.10.2 第二个 bug：接触检测闪烁撞上到位检查（根因已查清，含一次被推翻的旧结论）](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论)
    - [10.10.3 第三个 bug：夹爪刚接触 box 瞬间的 `kSlip` 被当成"ready，进入 LIFT"](#10103-第三个-bug夹爪刚接触-box-瞬间的-kslip-被当成ready进入-lift)
    - [10.10.4 第四个（独立）问题：`kPreplace`/`kPlace` 的掉落判据看瞬时接触布尔值，被接触检测噪声误判](#10104-第四个独立问题kpreplacekplace-的掉落判据看瞬时接触布尔值被接触检测噪声误判)
    - [10.10.5 一次独立的物理调参：夹爪闭合力度不够，撑不住 `kPreplace` 的侧向摆动](#10105-一次独立的物理调参夹爪闭合力度不够撑不住-kpreplace-的侧向摆动)
  - [10.11 失败模式与验证手段](#1011-失败模式与验证手段)
  - [10.12 你没问但值得注意的](#1012-你没问但值得注意的)
- [11. Stage J：episode 边界重构与 episode runner](#11-stage-jepisode-边界重构与-episode-runner)
  - [11.0 一句话总结](#110-一句话总结)
  - [11.1 改动清单与验证结果](#111-改动清单与验证结果)
  - [11.2 为什么是连续 20 次：这个数字真正验证的是什么](#112-为什么是连续-20-次这个数字真正验证的是什么)
  - [11.3 为什么用 topic 而不是 service/action 做 episode 边界：与 `requestReset()` 是同一种死锁机制](#113-为什么用-topic-而不是-serviceaction-做-episode-边界与-requestreset-是同一种死锁机制)
  - [11.4 排查记录：`episode_runner.py` 发布 `~/start_episode` 时撞上的 DDS 发现竞态](#114-排查记录episode_runnerpy-发布-start_episode-时撞上的-dds-发现竞态)
  - [11.5 失败模式与验证手段](#115-失败模式与验证手段)
  - [11.6 你没问但值得注意的](#116-你没问但值得注意的)
  - [11.7 Debug viewer：GLFW 原生渲染，直连权威 `mjData`（替代 2.5 计划里的 RViz 集成）](#117-debug-viewerglfw-原生渲染直连权威-mjdata替代-25-计划里的-rviz-集成)
    - [11.7.1 一句话总结](#1171-一句话总结)
    - [11.7.2 改动清单与验证结果](#1172-改动清单与验证结果)
    - [11.7.3 概念讲解：可视化架构选型——RViz / GLFW 原生渲染 / 三层"离权威物理源距离"梯子](#1173-概念讲解可视化架构选型rviz--glfw-原生渲染--三层离权威物理源距离梯子)
    - [11.7.4 排查记录：GLFW 创建 GL 上下文在 distrobox 里卡死](#1174-排查记录glfw-创建-gl-上下文在-distrobox-里卡死)
    - [11.7.5 失败模式与验证手段](#1175-失败模式与验证手段)
    - [11.7.6 你没问但值得注意的](#1176-你没问但值得注意的)
  - [11.8 launch 层参数暴露与 `record_demo_bag.sh`（收尾 Stage J）](#118-launch-层参数暴露与-record_demo_bagsh收尾-stage-j)
    - [11.8.1 一句话总结](#1181-一句话总结)
    - [11.8.2 改动清单与验证结果](#1182-改动清单与验证结果)
    - [11.8.3 概念讲解：为什么不新建 `pick_place_demo.launch.py`，以及 rosbag 回放里的 sim time 陷阱](#1183-概念讲解为什么不新建-pick_place_demolaunchpy以及-rosbag-回放里的-sim-time-陷阱)
    - [11.8.4 排查记录：`ros2 launch` 和 `ros2 run` 是同一种假死](#1184-排查记录ros2-launch-和-ros2-run-是同一种假死)
    - [11.8.5 失败模式与验证手段](#1185-失败模式与验证手段)
    - [11.8.6 你没问但值得注意的](#1186-你没问但值得注意的)

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
| 改动 | ① `scripts/` 或新增 `experiment_runner` 里的 episode runner：调 `~/reset` → 触发一次任务 → 收结果 → 循环 N 次；② 输出 CSV 到 `results/`（每 episode：seed/物体位姿/成功与否/失败码/各阶段耗时）；③ `pick_place_demo.launch.py`——原计划这一项是"RViz 网格显示"（`robot_state_publisher` + `RobotModel` Display），实际做过一次又被回退，改成调试专用的 GLFW debug viewer（`enable_debug_viewer` 参数，同进程直连权威 `mjData`，不经任何话题/降采样，详见 [11.7](#117-debug-viewerglfw-原生渲染直连权威-mjdata替代-25-计划里的-rviz-集成)）；文件名也没有照搬——现有的 `demo.launch.py` 已经在做 bridge+executor+rviz、默认模型就是 `pick_place_scene.xml`，新建一个同名文件只会是同一张图的重复，改成直接在 `demo.launch.py` 上扩展（详见 [11.8](#118-launch-层参数暴露与-record_demo_bagsh收尾-stage-j)）；④ launch 层暴露节点参数（解锁 week1 反向清单末条，[11.8](#118-launch-层参数暴露与-record_demo_bagsh收尾-stage-j)）；⑤ 录一段 rosbag（`scripts/record_demo_bag.sh`，[11.8](#118-launch-层参数暴露与-record_demo_bagsh收尾-stage-j)） |
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

1. ~~**加了 box 之后 vendor 的 `home` keyframe 会失配**（E/C 类）— `<key qpos>` 的长度必须等于 `nq`；加一个 freejoint 就把 `nq` 从 9 推到 16。值得问的是：**长度写错时 MuJoCo 是加载失败、静默截断，还是读越界？** 三种行为对应三种完全不同的防护手段，现在不知道是哪种。~~ **已在 Stage G 解答**：静默补零（不是加载失败，也不是读越界），且第一次实测被一次不相关的名字冲突报错掩盖过一轮——完整过程见 [8.8](#88-排查记录keyframe-名字冲突与长度不匹配是两件独立的事结论被推翻)。
2. **"到位"阈值 ε 目前没有任何依据**（C 类）— 用零误差判据会永远卡住（position servo 的稳态误差是[实测过的](week1.md#107-怎么快速做一次独立的-fk-验证) mm 级）；但阈值取太松会让 FSM 在没真到位时就推进到下一阶段，**表现为偶发失败而不是报错**。Stage I 之前该先问"这个数该怎么测出来而不是猜出来"。
3. ~~**"20 次连续成功"在确定性仿真里可能是假的**（F 类，知识边界）— 固定物体位姿 + 固定 waypoint + 确定性物理 = 20 次跑的其实**几乎是同一条轨迹**。那这个 20 次到底验证了什么？值得在 Stage J 之前想清楚，否则会拿一个没有信息量的数字当验收通过。~~ **已在 Stage J 开工前解答**：20 次确实几乎是同一条轨迹（Stage J 实测证实：`total_duration_s` 全部落在 6.71~6.75s，box 终点坐标只在小数点第5~6位不同）——但这个"20次没有信息量"的直觉在这套系统里是错的：`mujoco_bridge`/`task_executor` 是两个独立进程、各自独立的定时器，物理确定不代表跨进程调度时序确定，[10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论) 已经实测到一次真实的跨进程时序抖动（约2%概率）。20次真正验证的是"这类抖动有没有变得更频繁"，是冒烟回归测试，不是鲁棒性证明；这次20次恰好没有撞上那个约2%的窗口，不能倒推"bug已经不在了"。详见 [11.2](#112-为什么是连续-20-次这个数字真正验证的是什么)。
4. ~~**`max_effort` 字段可能无法实现**（C 类）— MJCF 现在只有位置伺服 actuator（[11.4](week1.md#114-position-servo-actuator-的本质一个-pd-控制器)）。如果 `GripperCommand.max_effort` 被静默忽略，下游会以为自己在控制夹持力。**该显式拒绝、该 WARN、还是该改模型**，是 Stage H 要定的。~~ **已在 Stage H 解答**：选了"警告一次但继续接受命令"，不是拒绝——因为这是纯 topic（没有 service/action 那种失败返回通道），拒绝只能表现成"什么都没发生"，比警告更难诊断；而 `max_effort=0` 在一些 `control_msgs` 使用惯例里本身就是"不限制"的哨兵值，不能无脑当成用户明确要求限力后再拒绝。真要支持力限制需要换成 action 逐步斜坡加压，见 [9.3](#93-为什么不直接用-grippercommand-action)。

## 6. 悬挂问题（本周新增）

### 6.1 清单

> 待各 stage 结束后按 [4.3 的判据](../../STUDY_NOTES_GUIDE.md) 分流填入（缺参照系 → 这里）。

- **`gripper_actuator_id_`/`gripper_ctrl_scale_`（`buildActuatorIndex`）和 `kHandBodyName="hand"`（`buildFrameIndex`）都是隐式单机械臂假设**（[8.7](#87-失败模式与验证手段)）——前者是两个标量，模型里若有第二条 tendon 驱动的夹爪会被循环无条件覆盖，只留最后一个，且没有任何警告；后者硬编码查找名字恰好叫 `hand` 的 body，第二条臂必须重命名（否则和第一条臂一样撞上 keyframe 那种编译期重名错误），重命名之后这行代码就再也找不到它。**缺参照系**：现在没有第二条臂，无法验证"改成怎样的 per-arm 索引才对"，而且计划书 2.2 节明确把双臂列进"暂不进入 MVP"。解锁条件：真正引入第二条机械臂时。
- **多个可操作物体的通用化设计**（`kObjectBodyName="box"` 单一常量、单一 oracle 话题）——**缺参照系**：不知道 bin picking 阶段（Chap 5）实际需要"每个物体一条话题"还是"一条话题发数组"，现在设计只是猜。解锁条件：进入 bin picking / 杂乱清空阶段（计划书第7节，明确排在单物体 pick-and-place 稳定之后）。
- ~~**`grasp.lift_height_threshold_m`/`grasp.region_radius_m` 目前是猜的，不是测出来的**（Stage H）~~ **已在 Stage I 解锁**：`KeyframeWaypointSource` 跑出了一条真正把 box 抬过阈值的轨迹（`box_z` 0.24→0.40→0.24，详见 [10.10.5](#10105-一次独立的物理调参夹爪闭合力度不够撑不住-kpreplace-的侧向摆动)），默认值 `0.26` 在这次实测里够用，没有改。
- ~~**`classifyGrasp` 判定 `kSlip` 没有时间维度**（Stage H）~~ **已在 Stage I 解锁**：FSM 侧按阶段消歧——`kClose` 把 `kSlip` 读成"抓住了，还没试着抬"，`kLift` 把同一个 `kSlip` 读成"抬起来后真的滑了"，见 [10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论)/[10.10.3](#10103-第三个-bug夹爪刚接触-box-瞬间的-kslip-被当成ready进入-lift)。没有改 `classifyGrasp` 本身。
- ~~**`GraspOutcome` 目前只进日志，没有对应的 ROS topic**（Stage H）~~ **已在 Stage I 解锁**：答案是"不需要新话题"——`task_executor_node` 直接复用现有的 `~/ground_truth/*` 话题拼 `GraspSignals`，调导出的库函数 `classifyGrasp()` 自己算，见 [10.7](#107-为什么-task_executor-不发布订阅一个-graspoutcome-话题)。
- **是否要给 `mujoco_bridge_node` 嵌入原生渲染器**（GLFW + `mjr_render`，画同一份 `data_`，不经 TF/RViz 这一层）——**缺参照系**：现在 RViz 已经能实时看，加原生渲染器的唯一动机是"更像 `simulate` 那种可交互调试"，但值不值得为这个多接一层 GLFW 事件循环和 `onTimer`/`onReset` 的互斥访问，需要先有具体的"RViz 不够用"的场景才能判断。
- **`close_settle_s`/`lift_settle_grace_s`（Stage I，各 2.0s）是"改到实测稳定通过为止"定的，不是从物理量推出来的**（[10.12](#1012-你没问但值得注意的) 第1条）——**缺参照系**：第3周把 `KeyframeWaypointSource` 换成 IK 驱动的连续轨迹后，这两个常数背后的物理场景（"发一个离散目标、等伺服收敛"）整体改变，现在的数值大概率不能直接照搬。解锁条件：第3周接入 diff-IK 的 `WaypointSource` 之后重新测。
- **`RECOVER` 目前不区分 `ExitReason`，所有失败原因都统一回 `kHome` 重试**（Stage I，[10.8](#108-task_executor-的可扩展性后续步骤会替换哪些部分) 第5点）——`ExitReason` 枚举已经区分了 `kTimeout`/`kSlipped`/`kPlaceMissed` 等，但 `kRecover` 分支目前对所有原因一视同仁。**缺参照系**：现在只有一次真实失败模式的实测（人为制造的 `PLACE_MISSED`），不知道不同失败码是否真的需要不同恢复策略。解锁条件：Stage J 的失败码分布统计出来后，看是否有某类失败"重试无效"，值得针对性设计。
- **`task_executor` 的 `CMakeLists.txt`/`package.xml` 硬依赖 `mujoco_bridge` 这个具体包名，只为了拿 `grasp_criteria` 一段纯函数**（Stage I，[10.7.1](#1071-graspsignalsclassifygrasp-为什么物理上放在-mujoco_bridge-包里不是-task_executor)）——仓库里其实已经有一个空的 `manipulation_interfaces` 占位包，当初若把这段代码放进去，`task_executor` 就不用直接依赖 `mujoco_bridge`。**缺参照系**：现在没有真的换驱动/换包名的场景来验证这条依赖会不会真的咬人。解锁条件：第6周真正把 `mujoco_bridge` 换成真机驱动、或者需要决定 `grasp_criteria` 要不要搬进 `manipulation_interfaces` 时。
- ~~**`task_executor_node` 到达 `kDone`/`kFailed` 后没有任何自动重新开始的机制，只能重启整个进程**（Stage I，[10.3.1](#1031-recover-完整机制从触发到重试到力竭此前散落在各处没有整体讲过) 第四步）——`onTimer()` 在这两个终态直接 `return`，`phase_` 不会被任何代码再改动。**这不是缺参照系，是 Stage J 马上就要撞上的真实空白**：episode runner 要"连续跑 N 次"，现在的节点一次只能跑一个 episode。解锁条件：Stage J 设计 episode runner 时必须先决定——每次重启整个节点，还是给 `task_executor` 加一个"重新开始"的话题/服务。~~ **已在 Stage J 解决**：选了后者——新增 `~/start_episode`（订阅）+ `~/episode_outcome`（发布）两个话题，节点常驻不重启，`onStartEpisode()` 把内部状态强制拉回 `kHome`。见 [11.1](#111-改动清单与验证结果)/[11.3](#113-为什么用-topic-而不是-serviceaction-做-episode-边界与-requestreset-是同一种死锁机制)。
- **`kRecover` 的异步 `~/reset` 请求和 `phase_` 切到 `kHome` 之间没有显式同步，只靠 `min_settle_s` 的余量兜底**（Stage I，[10.3.1](#1031-recover-完整机制从触发到重试到力竭此前散落在各处没有整体讲过) 第三步）——`async_send_request` 发出去立刻返回，下一行就把 `phase_` 切了，没有等复位真正完成的确认。实测多次重试没出过问题，但这只是"跑过没翻车"，不是"验证过不会翻车"。**缺一次推演**：故意让 `~/reset` 服务响应延迟（或者让 `mujoco_bridge` 短暂不可用），观察这条竞态会不会真的被 `min_settle_s` 挡住，还是只是运气好。
- **`kOpen` 分支的开口阈值（`kOpenWidthM`/`kOpenEpsilonM`）锁死在 `fsm.cpp` 源码里的局部 `constexpr`，不像其它阈值那样是 `FsmParams`/ROS 参数**（Stage I，[10.3.2](#1032-fsmparams-完整设计这些参数怎么编码了整个抓取过程的阶段设计) 末尾）——系统里其它每一个阈值都能不改代码、不重新编译地在运行时调，唯独这两个数字不行，破坏了"可调参数都在 `FsmParams` 里"这条一致性。**缺一次推演**：现在没有真实场景需要调这两个数字，值得先想清楚要不要现在就补齐一致性，还是等真的需要调它时才动手。
- ~~**`task_executor_node` 快速重启后，`~/reset` 客户端可能在和 `mujoco_bridge` 完成 DDS 发现之前就发出第一次复位请求，静默失败**（Stage I，[10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论) 排查过程中意外撞见）——现象是 `~/reset not available yet` 警告，box 停在上一轮遗留的位置，后续动作全部建立在错误的初始状态上，且没有任何机制会重试或报错升级。**这不是缺参照系，是 Stage J 的 episode runner 会直接撞上的真实问题**：如果 runner 靠"重启进程"的方式开始新一轮 episode，第一个 episode 大概率会因为这个竞态跑错。解锁条件：Stage J 设计 episode runner 时要么改成不重启进程只调 `~/reset`（[10.3.4](#1034-requestreset-是节点调-service-的一般写法吗和命令行-ros2-service-call-有什么关系) 已经讲过节点内异步调用的写法），要么在 `requestReset()` 里给 `service_is_ready()` 失败的情况加真正的重试，不能只是打个警告就放弃。~~ **已在 Stage J 部分解决，且同一类 bug 换了个位置重新证实**：选了"不重启进程"，所以这条描述的具体场景（进程重启和 `mujoco_bridge` 竞态）不会再发生。但同一类 DDS 发现竞态换了个边立刻重现——这次是 `episode_runner.py` 新起的 `~/start_episode` publisher 和 `task_executor_node` 的 subscription 之间，第一次跑 3-episode smoke test 就实测撞见。教训不是"process 重启"这一种触发条件专属的，是"任何新引入的 pub/sub 边都要重新假设一次这个竞态会发生"。见 [11.4](#114-排查记录episode_runnerpy-发布-start_episode-时撞上的-dds-发现竞态)。
- **`classifyGrasp` 没有"宽度-only"的降级判据，完全依赖接触信号——但接触信号在真机上不存在**（Stage I，[10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论) 排查过程中确认：接触检测的逐 tick 抖动窗口长短取决于夹爪力度，是这次 `kLift` bug 的确认根因）——真机 Franka Hand 没有触觉阵列，标准做法（`franka_gripper` 的 `Grasp` action）是只看稳态宽度误差（[week2.md 9.7](#97-oracle-数据的分类哪些会随项目推进消失哪些需要学习方法)）。**这不是缺参照系**：现在就可以实现一份宽度-only 版本的判据去验证抓取流程在没有接触信号时是否还站得住，这份投入不管以后买不买得起真机都有意义，而且顺带绕开了接触信号抖动这整类问题。解锁条件：现在就可以做，没有前置依赖。

### 6.2 反向清单：现在就该做的

> 待填（缺一次推演 → 这里）。第1周遗留、本周不打算做的条目**不要**复制过来，仍以 [week1.md 13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) 为权威，本节只放本周新产生的。

- **`ament_lint_auto` 的历史债务**——Stage F 第一次跑 `colcon test`（本仓库此前一次都没跑过）时，`mujoco_bridge` 暴露出版权头缺失、include 顺序和 `uncrustify` 格式差异；gtest 部分（`test_frame_math`、`test_state_ops`）当时已全绿。这是独立的源码规范债务，不是逻辑 bug。此前以“格式化会影响 Week 1 的逐行引用”为由暂缓，但复核后确认 Week 1 使用的是代码摘录，不依赖源文件行号。2026-09-21 已补齐两包（`mujoco_bridge`、`task_executor`）的许可证头、include 顺序和格式，并通过 `colcon test`：180 项测试、0 失败、26 项因环境中的 cppcheck 慢版本跳过。
- **RTF 监控目前只打日志，没有阈值告警**（[7.6](#76-rtf-是什么为什么要常驻监控为什么-tf_rate_hz-要和-joint_state_rate_hz-一致)、[7.7](#77-失败模式与验证手段)）——FSM/episode runner 要知道"RTF 掉到多少算异常"必须人眼盯日志。解锁条件：Stage I 给 FSM 超时判据接入 RTF 时，评估要不要把这个做成可查询的话题或参数化阈值。
- **`tf_rate_hz != joint_state_rate_hz` 的告警本周加了代码但从没被真实触发过**（[7.6](#76-rtf-是什么为什么要常驻监控为什么-tf_rate_hz-要和-joint_state_rate_hz-一致)、[7.7](#77-失败模式与验证手段)）——两者目前都还是默认的 100Hz。解锁条件：以后真的把 `tf_rate_hz` 调开（比如给 FSM 提供更高频姿态反馈）时，第一次触发也是第一次验证这段代码本身是对的。
- **keyframe 长度不匹配会被 MuJoCo 静默补零，没有任何测试防住**（[8.8](#88-排查记录keyframe-名字冲突与长度不匹配是两件独立的事结论被推翻)）——现在完全靠人记得"改了 nq 就要检查所有 keyframe 长度"。缺一次推演：写一个 gtest fixture，加载一个长度不匹配的 keyframe，断言补零后的 qpos 确实是 `(0,0,0,1,0,0,0)` 而不是别的值，把这条隐藏行为钉死成一个会报警的断言。
- **`~/ground_truth/object_pose` 的发布逻辑（`resolveObjectOracle`/`publishObjectPose`）完全没有单元测试**——不像 Stage F 把 `frame_math`/`state_ops` 剥成纯函数配了 gtest，这段"直接抄 `xpos`/`xquat`"还焊在节点里。现在简单到不太会错，但和"多个物体的通用化"（[6.1](#61-清单)）是同一批要重构的代码，值得等那时候一起剥离和测试，而不是现在单独做一次半成品抽象。
- **`~/ground_truth/object_pose` 与 `/tf` 共享 `tf_decimation_`，没有独立的发布频率开关**——两者数值上刻意设计成可以互相印证同一物理步，但如果以后 FSM 需要比 `/tf` 更高频的物体反馈，现在的代码结构没有单独调节 oracle 频率的参数。解锁条件：Stage I 给 FSM 接反馈时评估是否需要。
- **`condim=3` 下 `friction` 的扭转/滚动两个分量（`0.03`、`0.003`）是抄来的死代码**——`condim` 不到 4/6 这两个数字完全不参与计算，从未验证过数值本身是否合理。解锁条件：Stage H 调抓取判据发现打滑/旋转问题，或以后主动把 `condim` 升级时。
- **`bodiesInContact` 没有检查 `mjContact::exclude`**（Stage H，[grasp_state.cpp](../../src/mujoco_bridge/src/grasp_state.cpp)）——`mjContact` 有 `margin`/`gap` 相关的 `exclude` 字段（0=计入求解，非0=因各种原因被排除，包括"在 gap 区间内但还没真正接触"），当前实现只要这对 body 出现在 `mjData::contact` 数组里就算 `true`，不看这个字段。现在 box/桌面/手指的 geom 都没配非零 `margin`（默认 0），大概率不会被触发，但没有测试验证过这个假设。缺一次推演：给某个 geom 配一个非零 `margin`，观察 `exclude!=0` 的接触是否真的会被现在的实现误判为"接触"。
- **`task_executor_node.cpp` 的 `extractArmState()`/`extractGripperWidth()` 没有剥成纯函数、没有单测**（Stage I，[10.12](#1012-你没问但值得注意的) 第3条）——两者已经是"输入一个 `JointState`、输出一个结构体"的形状，逻辑上很接近 Stage F 剥 `frame_math`/`state_ops` 时用的判据，但比 `mujoco_bridge_node.cpp` 通常的胶水层更厚。缺一次推演：按 [2.1.1](#211-这类系统该怎么测四层本周只取前两层) 的判据（"写错了人眼看 RViz 能不能发现"）评估要不要现在就剥。
- **`fsm.cpp` 的连续量判据（`box_height_m`）本身的噪声幅度没有量化过**（Stage I，[10.12](#1012-你没问但值得注意的) 第2条）——只有直觉上"连续量比逐 tick 重新生成的接触布尔值稳"，没有像 `frame_math`/`state_ops` 那样写 gtest 量化这条连续量在更剧烈运动下会不会也开始在阈值附近抖动。
- **`~/start_episode` 收到时如果上一个 episode 还没到终态就再次被调用，会无条件把 `phase_` 拽回 `kHome`，正在进行的那次尝试被静默覆盖，没有任何警告或错误**（Stage J，[11.3](#113-为什么用-topic-而不是-serviceaction-做-episode-边界与-requestreset-是同一种死锁机制)）——当前 `episode_runner.py` 严格串行、等到上一个 `~/episode_outcome` 才发下一条 `~/start_episode`，不会触发这条路径，这次决定先不修。缺一次推演：手动在 episode 中途发一条 `~/start_episode`，确认现象确实是"静默覆盖、无报错"，再决定要不要在 `onStartEpisode()` 里加一个"忙碌中拒绝并 WARN"的检查——这本质上是 [11.3](#113-为什么用-topic-而不是-serviceaction-做-episode-边界与-requestreset-是同一种死锁机制) 里讨论过的"action 会免费提供的并发目标拒绝"，topic 版本没有对应机制。

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

## 8. Stage G：pick-and-place 场景与 oracle 接口

### 8.0 一句话总结

新增 [pick_place_scene.xml](../../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml)：`<include>` 上 vendor 的 `scene.xml`（连带 `panda.xml`），追加桌面 geom、`box` body（`freejoint`）、放置区标记，自带 `pick_place_home` keyframe；`mujoco_bridge_node.cpp` 的 `model_path`/`reset_keyframe_name` 变成 ROS 参数（默认指向新场景），新增 `~/ground_truth/object_pose`（`PoseStamped`，frame `world`）作为下游唯一合法的 oracle 源，启动日志同时打印 `nq`/`nv`。过程中撞上一次 MuJoCo `<include>` 路径解析的真实限制（被迫改变文件布局），以及一次先被误判、后被推翻的 keyframe 结论（名字冲突和长度不匹配是两件独立的事，实测才分清）。

### 8.1 改动清单与验证结果

**改动**：

- [pick_place_scene.xml](../../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml)（新）：桌面 geom、`box`（`freejoint`）、放置区 marker、`pick_place_home` keyframe（详见 [docs/architecture.md 第5节](../../docs/architecture.md)）
- [mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp)：`model_path`/`reset_keyframe_name` 声明为 ROS 参数；新增 `resolveObjectOracle()`/`publishObjectPose()`；启动日志加 `nv`

**编译**：

```
Starting >>> robot_description
Finished <<< robot_description [0.08s]
Starting >>> mujoco_bridge
Finished <<< mujoco_bridge [5.06s]

Summary: 2 packages finished [5.26s]
```

**运行**（环境卫生按 [STUDY_NOTES_GUIDE 3.1](../../STUDY_NOTES_GUIDE.md) 确认后，直接跑可执行文件，不经 `ros2 run`）：

```
Loaded .../pick_place_scene.xml (nq=16, nv=15, timestep=0.0020s)
9 actuated joints: joint1, ..., finger_joint1, finger_joint2
TF: 3 static, 10 dynamic frames        # 9 -> 10，box 零代码改动自动出现
ground-truth object `box` found; publishing ~/ground_truth/object_pose
```

**四条验收标准逐一核对**：

| 验收项 | 实测 |
| --- | --- |
| ① `world -> box` 动态 TF 零代码改动出现 | `TF: 3 static, 10 dynamic frames`（panda.xml 单独跑是 9 dynamic），`tf2_echo world box` 能正常输出 |
| ② `nq != nv` 首次成立 | `nq=16, nv=15`（freejoint 的四元数比角速度多一维） |
| ③ `~/reset` 后 box 回到标称位姿且 sim time 不回退 | `success=True, message='reset to keyframe \`pick_place_home\`'`；多次 reset 间 sim time 持续单调前进 |
| ④ oracle 话题与 `tf2_echo world box` 一致 | oracle: `(0.4999999999975637, -1.77e-13, 0.23987759634993702)`；`tf2_echo`: `(0.500, -0.000, 0.240)`——同一物理步内一致，差异仅浮点噪声 |

**切回旧模型的验证**（确认参数确实"可切回"，不是只有默认值能跑）：

```
--ros-args -p model_path:=.../panda.xml -p reset_keyframe_name:=home
→ Loaded .../panda.xml (nq=9, nv=9, timestep=0.0020s)
→ TF: 3 static, 9 dynamic frames
→ no `box` body in model; ~/ground_truth/object_pose disabled
```

### 8.2 MJCF `<include>` 路径解析规则：为什么新场景文件要和 panda.xml 同目录

> Q: pick_place_scene.xml 包括了桌面和物体 + scene.xml 的场景 + panda.xml 的机械臂，之后只需要读这个总体的 xml 就可以了对吗？

不完全是——`pick_place_scene.xml` 本身不含机械臂的任何定义，只是一份"拼装清单"，真正内容分散在三层文件里（本文件 / `scene.xml` / `panda.xml`）。想要"一个文件看到全部"，正确工具是 MuJoCo 自带的 `compile`，它能把整条 `<include>` 链展开、拍平成一个不含 `<include>` 的单文件：

```bash
compile pick_place_scene.xml /tmp/flat.xml
```

最初设计时想把新场景放进独立目录 `mujoco_models/`，用 `<include file="../franka_emika_panda/scene.xml"/>` 引用，编译失败：

```
Error: Error opening file 'robot_description/mujoco/mujoco_models/assets/robot_description/mujoco/franka_emika_panda/link0.stl': No such file or directory
```

路径被拼接成三层重复，不是简单的"找不到文件"。查证后确认：**MuJoCo 无论 `<include>` 嵌套多深，`file` 属性和每个被包含文件自己的 `meshdir` 都相对"顶层主文件"的目录解析，不是相对"直接包含它的文件"**（[mujoco#974](https://github.com/google-deepmind/mujoco/issues/974)）。这就是为什么 vendor 自己的 `mjx_single_cube.xml` 也和 `panda.xml` 同目录——不是随意的组织方式，是被这条限制逼出来的唯一可行结构。改成把 `pick_place_scene.xml` 放进 `franka_emika_panda/` 目录本身后编译成功。这是**新增文件**，没有修改任何已有 vendor 文件，但确实把"vendor 目录"和"我们自己的组合文件"混在了一起——以后跟上游 diff 时要记得把新增文件排除在外。

### 8.3 qpos 顺序怎么确定：深度优先遍历 + 文档顺序

`qpos` 里没有"关节构型在前、物体自由度在后"这样的专门规则，纯粹是**合并后的 worldbody 树里谁先出现，深度优先遍历时谁先分配 qpos 槽位**。用 `compile` 展开后可以直接看到证据：

```
<geom name="floor" .../>              <- scene.xml，裸 geom，不占 qpos
<geom name="table" .../>              <- 我们的，裸 geom，不占 qpos
<geom name="place_marker" .../>       <- 我们的，裸 geom，不占 qpos
<body name="link0" ...>               <- panda.xml，链式嵌套到 link1..7, hand, fingers（9 个关节）
<body name="box" pos="0.5 0 0.241">   <- 我们的，freejoint（7 个 qpos）
```

`link0` 链排在前面是因为 `<include file="scene.xml"/>` 写在文件最前面；`box` 排在后面是因为它属于 `pick_place_scene.xml` 自己的 `<worldbody>` 块，这个块写在 `<include>` **之后**。这不是巧合，是纯粹的书写顺序决定的——如果把 `<body name="box">` 挪到 `<include>` 之前，box 就会变成 `qpos[0..6]`，机械臂关节整体后移。keyframe 里 `qpos` 前9后7的排列，是靠实际 reset 后读 oracle 话题数值吻合验证过的，不是凭猜测写的。

### 8.4 `qpos` 与 `xpos`/`xquat`：为什么 oracle 发布读后者

> Q: box 也在 qpos 里面，但是为什么代码里读的是 xpos/xquat？

两者是 `mjData` 里不同用途、不同索引方式的数组，`mj_forward`/`mj_step` 保证它们随时同步：

| 数组 | 含义 | 索引方式 | 谁算出来 |
| --- | --- | --- | --- |
| `qpos`/`qvel`/`qfrc_*` | 广义坐标，积分器实际推进的状态向量 | 按关节（`jnt_qposadr`/`jnt_dofadr`） | 物理积分本身 |
| `xpos`/`xquat` | 每个 body 在世界系下的绝对位姿 | 按 body id（每个 body 一份） | 由 `qpos` 经正运动学推导，`mj_forward` 算 |

对 `joint1..7` 这类 hinge 关节，`qpos` 只是标量转角，本身不是坐标——要知道某个 link 在世界系下的实际位置，必须把从 `link0` 开始这条链上每一级的旋转/平移全部复合，这正是正运动学，MuJoCo 已经算进 `xpos`/`xquat`。`box` 的 `freejoint` 是特例：它的 7 个 `qpos`（3 平移 + 4 四元数）**因为父级直接是 world**，定义本身就等于世界位姿——这也是为什么这次 `qpos[9..15]` 和 `xpos`/`xquat[box_id]` 数值一致。

但 `publishObjectPose()` 故意读 `xpos`/`xquat` 而不是直接切 `qpos[9..15]`：**"`qpos` 就是世界坐标"只在"父级是 world"这一个具体条件下成立**。以后如果 box 不再直接挂在 world 下（比如放进一个可移动托盘），`qpos` 就只是相对托盘的局部坐标，直接当世界坐标发布会是一个数值上完全能跑、但语义错误的 bug，而且不会报错（形状没变）。用 `xpos`/`xquat` 不依赖这个假设，因为不管父级是谁，MuJoCo 都已经把链式复合算完。这和 `buildFrameIndex()` 用 `body_jntnum==0` 判断 static/dynamic 而不是硬编码名字是同一种习惯：把"是不是绝对坐标"这类判断交给模型自己算出来的量，不要在业务代码里凭当前配置抄近路。

反过来 `publishJointState()` 必须用 `qpos` 而不是 `xpos`：`JointState` 要的是每个关节自己的标量转角，`xpos`（绝对坐标）装不下这个语义，这也是 `buildJointIndex()` 跳过 free/ball 关节的原因——box 不出现在 `/joint_states` 的 9 个名字里，它的 7 个 `qpos` 分量占着位置，只是走 TF/oracle 对外可见。

### 8.5 `condim`/`friction`/`solref`：接触参数详解，为什么其中两个当前是死代码

`friction="1 0.03 0.003"` 三个数分别是（**滑动**摩擦系数, **扭转**摩擦系数, **滚动**摩擦系数）：滑动抵抗切向滑移（抓取里最关心，"打滑 vs 咬住"直接取决于它）；扭转抵抗绕接触法线的原地旋转；滚动抵抗物体沿接触面滚动。

`condim` 决定接触点在求解器里有几个约束维度：1=只有法向（无摩擦）；3=法向+2个切向滑动（各向同性，最常用）；4=再加扭转；6=再加滚动（完整6维）。**关键点：只有 `condim` 用到的维度对应的摩擦系数才真正参与计算**——当前 `condim="3"` 只用滑动系数，扭转（0.03）和滚动（0.003）两个数字是纯粹的死代码，写了但不生效。

`solref="0.01 1"` 不是刚度系数，是接触点虚拟弹簧-阻尼器的（时间常数, 阻尼比）：0.01 = 10ms 内响应完成，1 = 临界阻尼（不震荡）。太小（接近或小于 `timestep=0.002s`）会数值不稳定；太大会看起来"软"（明显下陷后才弹回）。

这些数值照抄自 vendor 的 `mjx_single_cube.xml`，从没针对本场景的桌子+box+夹爪组合重新验证过。会不会改：几乎肯定会——Stage H 如果发现"命令夹紧了却测不到稳定接触"或"抓住了但一使力就滑出"，第一个要查的就是滑动摩擦系数；如果以后想让扭转/滚动摩擦真正生效（比如夹爪角度不完全对齐时需要抗旋转能力），需要把 `condim` 提到 4 或 6，到那时这两个"配置了但没用"的数字会突然从摆设变成真正影响任务难度的参数，且切换瞬间不会有任何日志提示。

### 8.6 三条可视化路径的定位：RViz / `simulate` / 工业界与学术界的两种范式

> Q: 之前只在 RViz 里看过 bridge 发布的 TF，如何用 MuJoCo 自己观察已构建的场景？

**RViz 不是离线数据，是对正在运行的 `mujoco_bridge_node` 的实时视图**：`demo.launch.py` 起 bridge_node + rviz2，bridge_node 每个物理步（2ms）发布一次 `/tf`/`/joint_states`，RViz 订阅重画；发给 `~/joint_command`/`~/reset` 的命令直接写进 bridge_node 那份 `mjData`（同进程同内存），下一步就体现在新发布的 `/tf` 里。这套流程本来就是实时看仿真结果。

`simulate <model.xml>` 是**完全独立的第二份物理世界**：自己加载模型、自己跑物理，和 `mujoco_bridge_node` 内部那份 `mjData` 毫无关系，不受任何 ROS 命令驱动，只用来肉眼核对 MJCF 本身（桌子高度对不对、box 会不会穿模）。实测在这个沙盒环境里加 `LIBGL_ALWAYS_SOFTWARE=1`（和 rviz2 需要的环境变量一样，同一个 GL 转发限制）后 `simulate` 能正常弹出窗口渲染 `pick_place_scene.xml`：机械臂立在桌子旁，红色 box 在桌上，绿色圆盘是放置区标记，比例符合预期。

> Q: 工业界/学术界的炫酷 demo 通常怎么做？RL policy 那种边 step 边 render 的循环是主流吗？

这对应两种不同范式：

- **单进程 step+render**（多数 RL/研究 demo）：控制器和物理仿真同进程同内存，`viewer.sync()` 直接画同一份 `data`，没有序列化/网络开销，想多快渲染多快。大规模并行训练（Isaac Lab / MJX / Brax）训练阶段直接关渲染，只在评估时录几条 rollout 做视频——很多论文/社交媒体上的"丝滑"视频其实是离线跑一次、录制帧序列剪辑出来的，不是持续运行的交互式 viewer。
- **client-server 拆分**（Gazebo 的 `gzserver`/`gzclient`、Isaac Sim 的 PhysX/Omniverse 分离、以及我们自己的 `mujoco_bridge`+RViz）：物理进程只发布状态，一个或多个可视化客户端订阅，两者独立进程不共享内存指针。代价是多一层序列化/传输延迟，换来的是**多消费者**（RViz、rosbag、未来的面板互不干扰）和**控制器与仿真解耦**——这正是本项目选这条路的原因：第6周把 `mujoco_bridge` 换成真实 `ros2_control` 驱动时，下游 FSM/感知代码不用改一行。纯粹为了录像而重写整个控制循环去接单进程渲染，在需要真机迁移的项目里是不划算的。

### 8.7 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现 / 怎么防住 |
| --- | --- | --- |
| 跨目录 `<include>` | 编译报错，mesh 路径被拼接成三层重复，报错信息不会提示"应该同目录" | 用 `compile` 工具最小复现（`test_single_include.xml`），查证 MuJoCo 的路径解析规则（[8.2](#82-mjcf-include-路径解析规则为什么新场景文件要和-pandaxml-同目录)），改用同目录布局 |
| keyframe 名字冲突 | 编译期硬错误 `repeated name 'home' in key` | 显式重命名为 `pick_place_home`；`reset_keyframe_name` 做成参数，换模型必须显式配对，不给默认值兜底成"猜一个" |
| keyframe 长度不匹配 | **不报错**，MuJoCo 静默把缺的部分补零（对 freejoint 四元数是 `(0,0,0,1,0,0,0)`），box 被传送到接近世界原点 | 目前只能靠人眼盯 reset 后的位置——本周唯一一次靠手动切换参数复现，见 [8.8](#88-排查记录keyframe-名字冲突与长度不匹配是两件独立的事结论被推翻)；反向清单已记录要补 gtest |
| `friction` 扭转/滚动分量在 `condim=3` 下不生效 | 调了参数没有任何效果，且不会报错或警告 | 目前无自动检测，只能靠理解 `condim`/`friction` 的对应关系（[8.5](#85-condimfrictionsolref接触参数详解为什么其中两个当前是死代码)） |
| oracle 与 TF 数值不一致 | 若真的出现，说明两者不再共享同一物理步或 `xpos`/`xquat` 读取有误 | 本周实测两者一致（差异仅浮点噪声），是设计上刻意的交叉验证；以后改动 decimation 逻辑要重新核对这一点 |

### 8.8 排查记录：keyframe 名字冲突与长度不匹配是两件独立的事（结论被推翻）

**现象**：第一次给 `pick_place_scene.xml` 写 `<key name="home">`（复用 vendor 的名字，长度16）时，`compile` 报错 `Error: repeated name 'home' in key`。当时把这个报错直接当成"week2.md 5.1 第1条悬挂问题（keyframe 长度写错时 MuJoCo 是加载失败、静默截断，还是读越界？）"的答案，写成"长度不匹配是编译期硬错误"。

**推翻过程**：用户追问 qpos 顺序时，为了验证顺序，用 `compile` 把整条 `<include>` 链展开成单文件重新看,才发现展开后的文件里其实**同时保留了两个 keyframe**——vendor 的 9 长度 `home` 和我们的 16 长度 `pick_place_home`，而 `home` 那一行被 MuJoCo **自动补齐**成了 `qpos="... 0.04 0.04 0 0 0 1 0 0 0"`（后7个是补的）。这说明之前那次报错的根因根本不是"长度不匹配"，是两个 keyframe **恰好都叫 `home`**——纯粹的名字唯一性冲突，和长度无关。

**验证**：把 `reset_keyframe_name` 参数手动改成 `home`（这个模型里合法存在，只是长度不对），实际调用 `~/reset`：

```
response: success=True, message='reset to keyframe `home`'   # 不报错
position: x: 0.0027813291615433416, y: 0.0008049163369827898, z: 0.014382851520445689
```

box 被静默传送到接近世界原点（`(0,0,0,1,0,0,0)` 补零的结果），砸向机械臂底座附近再弹开，全程没有一条警告或错误。

**根因**：keyframe 名字冲突和 keyframe 长度不匹配是**两个独立的检查**——前者是 MuJoCo 编译器的名字唯一性校验（硬错误），后者根本没有校验（静默补零）。只有当两个 keyframe 恰好重名时，前一种检查才会先拦住问题；只要改成不同名字（就像我们后来做的 `pick_place_home`），长度不匹配这条路径就完全不会被拦，会在运行时安静发生。

**留下的经验**：

1. **"编译报错了"不代表"报的是我以为的那个错误"**。第一次看到 `repeated name` 报错时没有细究"这到底是名字问题还是长度问题"，直接套用到了悬挂清单里现成的那个问题上——两者表面看起来很像（"keyframe 写坏了会怎样"），实际是完全不同的检查路径。
2. **之前写下的结论已被推翻，教训记在这里，不做静默修改**：week2.md 5.1 第1条问题的真实答案不是"编译期硬错误"，是"**看具体是哪种坏法**：名字冲突→硬错误；长度不匹配→静默补零"。这个更细的答案已经同步进 [docs/architecture.md 第0.1节](../../docs/architecture.md)。
3. 这次推翻是**因为多问了一步"给我看实际展开后的文件"**才发现的——再一次印证"质疑证据链优于质疑结论"（[STUDY_NOTES_GUIDE 4.4](../../STUDY_NOTES_GUIDE.md)）：第一次的结论表面自洽（有报错、看起来像回答了问题），只有去看原始展开文件才暴露出报错原因被张冠李戴。

## 9. Stage H：夹爪命令接口与抓取成功判据

### 9.0 一句话总结

把夹爪命令从 `~/joint_command` 分出独立的 `~/gripper_command`（`control_msgs/msg/GripperCommand`，对齐真机 ros2_control 的接口形状但仍是 topic 不是 action）；新增两个纯函数模块——[grasp_criteria.hpp/cpp](../../src/mujoco_bridge/include/mujoco_bridge/grasp_criteria.hpp)（`classifyGrasp`，Stage F 定义的第1层，不碰 `mjModel`/`mjData`，10 个 gtest）和 [grasp_state.hpp/cpp](../../src/mujoco_bridge/include/mujoco_bridge/grasp_state.hpp)（`gripperWidth`/`bodiesInContact`，第2层，6 个 gtest，新 fixture [contact_probe.xml](../../src/mujoco_bridge/test/fixtures/contact_probe.xml)）；节点新增 `~/ground_truth/left_finger_contact`/`right_finger_contact` 两个话题和一条"分类结果变化时打日志"的胶水代码。三场景实测（空抓/正常夹住/诱导 slip）覆盖了 `GRASP_EMPTY`/`SLIP`/`UNEXPECTED_CONTACT`，但没能真正触发 `kSuccess`——这本身就是留给 Stage I 的一条悬挂项。

### 9.1 改动清单与验证结果

**改动**：

- [grasp_criteria.hpp](../../src/mujoco_bridge/include/mujoco_bridge/grasp_criteria.hpp) / [grasp_criteria.cpp](../../src/mujoco_bridge/src/grasp_criteria.cpp)（新）：`GraspOutcome` 枚举（`kSuccess`/`kNoObject`/`kGraspEmpty`/`kSlip`/`kTimeout`/`kPlaceMissed`/`kUnexpectedContact`）、`GraspSignals`/`GraspCriteria` 结构体、纯函数 `classifyGrasp`
- [grasp_state.hpp](../../src/mujoco_bridge/include/mujoco_bridge/grasp_state.hpp) / [grasp_state.cpp](../../src/mujoco_bridge/src/grasp_state.cpp)（新）：`gripperWidth`（两指 qpos 求和）、`bodiesInContact`（遍历 `mjData::contact` 判断两个 body 是否有接触，双向查 geom 顺序）
- [mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp)：新增 `~/gripper_command`（`onGripperCommand`）；`~/joint_command`（`onJointCommand`）删掉手指特殊分支，改成遇到手指关节名 `WARN_ONCE` 提示迁移；新增 `resolveGripperFingers()`、`publishGripperContact()`；`grasp.*` 四个 ROS 参数（`box_width_m`/`width_epsilon_m`/`lift_height_threshold_m`/`region_radius_m`）
- [test/test_grasp_criteria.cpp](../../src/mujoco_bridge/test/test_grasp_criteria.cpp)、[test/test_grasp_state.cpp](../../src/mujoco_bridge/test/test_grasp_state.cpp)、[test/fixtures/contact_probe.xml](../../src/mujoco_bridge/test/fixtures/contact_probe.xml)（新）
- [CMakeLists.txt](../../src/mujoco_bridge/CMakeLists.txt) / [package.xml](../../src/mujoco_bridge/package.xml)：两个新 `ament_add_gtest` 目标；新增 `std_msgs` 依赖（`~/ground_truth/*_finger_contact` 用 `std_msgs/Bool`）
- [scripts/gripper_test.py](../../scripts/gripper_test.py)：改用 `~/gripper_command`，`position` 语义从"每指位移"改成"总开口宽度"

**编译**：

```
Starting >>> robot_description
Finished <<< robot_description [0.08s]
Starting >>> mujoco_bridge
Finished <<< mujoco_bridge [7.88s]

Summary: 2 packages finished [8.09s]
```

**测试**（直接跑 gtest 二进制）：

```
test_frame_math:      3/3 PASSED
test_state_ops:       6/6 PASSED
test_grasp_criteria: 10/10 PASSED
test_grasp_state:     6/6 PASSED
```

**"故意注入错误，测试必须变红"实测**（延续 Stage F 定的验收标准）：把 `classifyGrasp` 里 `lifted` 的比较符从 `box_height_m > threshold` 改成 `<`，重新编译后 `AllFourConditionsMetIsSuccess`/`GrippedButNeverLiftedIsSlip`/`WidthJustInsideEpsilonIsBracketed` 三个用例应声变红（`[ FAILED ]`，10 个测试里 3 个失败），改回后重新编译，10/10 恢复全绿。

**三场景实测数据表**（环境卫生按 [STUDY_NOTES_GUIDE 3.1](../../STUDY_NOTES_GUIDE.md) 确认后，直接跑可执行文件，通过 `~/joint_command`/`~/gripper_command` 手动摆位置验证）：

| 场景 | 操作 | 实测数值（width / box_z / box_to_tcp / L / R） | `classifyGrasp` 分类 |
|---|---|---|---|
| **空抓** | 手指下降到 box 附近，命令 `position=0.0`（全闭），两指都没碰到东西 | width≈0.0m, box_z≈0.240m（桌面）, box_to_tcp≈0.03~0.05m, L=0 R=0 | `GRASP_EMPTY`（在 region 内但没夹住） |
| **正常夹住** | 手指落到 box 两侧，命令 `position=0.032` | width≈0.045~0.05m, box_z≈0.239~0.243m（仍在桌面）, box_to_tcp≈0.004m, L=1 R=1 | 途中经过 `UNEXPECTED_CONTACT`（单侧先触发）→ `SLIP`（双侧接触、宽度对但还没抬起，`lifted` 未满足） |
| **诱导 slip** | 夹住后突然发一条大位移的 `~/joint_command`，相当于给 box 一个扰动 | box_to_tcp 从 0.004m 跳到 0.03m 以上，contact 从双 `true` 变成不稳定的单侧/无 | 在 `SLIP`/`UNEXPECTED_CONTACT` 之间跳动 |

**没能触发的一格**：受限于手调 waypoint、没有 IK，这次没能把 box 真正抬过 `lift_height_threshold_m=0.26m`（占位值）触发一次 `kSuccess`——这正是 Stage I 要解决的问题，见第6节悬挂清单新增项。

**手动验证摘录**（`ros2 topic pub`/`ros2 topic echo`，节点日志 `grasp outcome -> ...` 行是 `classifyGrasp` 实时结果）：

```
[INFO] [mujoco_bridge]: gripper fingers `left_finger`/`right_finger` found; publishing ~/ground_truth/*_finger_contact
...
[INFO] [mujoco_bridge]: grasp outcome -> GRASP_EMPTY (width=0.0800m box_z=0.2399m box_to_tcp=0.0489m L=0 R=0)
[INFO] [mujoco_bridge]: grasp outcome -> UNEXPECTED_CONTACT (width=0.0800m box_z=0.2399m box_to_tcp=0.0192m L=1 R=0)
[INFO] [mujoco_bridge]: grasp outcome -> SLIP (width=0.0456m box_z=0.2378m box_to_tcp=0.0045m L=1 R=1)
```

`~/gripper_command` 与 `~/joint_command` 的切换验证：给 `~/joint_command` 发手指关节名，只在第一次触发 `WARN_ONCE`（"finger joints must be commanded via ~/gripper_command now"），`ctrl` 不再被写；`~/gripper_command` 发 `max_effort != 0` 同样只警告一次，`ctrl` 正常按 `position/2 * gripper_ctrl_scale_` 写入。

### 9.2 position servo 下"夹住"为什么是稳态位置误差，不是力

MJCF 手指 actuator 是 `biastype="affine"` 的 PD 控制器（[week1 11.4](week1.md#114-position-servo-actuator-的本质一个-pd-控制器)），没有独立的力控制通道。命令一个比 box 宽度更小的目标宽度时，伺服持续施加朝闭合方向的力，box 的刚体碰撞反作用力和伺服力在稳态相互平衡——这个平衡点就是"夹住"的物理体现：**手指实际停在的位置比命令的目标位置更宽**（被 box 挡住）。这正是为什么判据要看**实测宽度**（`gripperWidth` 读 `qpos`）而不是"命令发出去了"：命令 `position=0.0`（全闭）但手指卡在 box 两侧时，实测宽度依然是 box 的宽度，不是 0——三场景表里"正常夹住"那一行 width≈0.045~0.05m 而不是命令的 0.032m，就是这个平衡点的直接证据。

### 9.3 为什么不直接用 `GripperCommand` action

真机上这确实是一个 action（有 goal/feedback/取消语义：夹爪可能需要几百毫秒才能到位，中途要能取消）。但本周这层只是"设今天的目标"，和 `~/joint_command` 是同一个哲学（Stage E 定的：没有执行语义，只有目标设定）。换成真 action 需要在这个节点里跑一个 action server 循环（判断到位、发 feedback、支持 cancel），这个复杂度现在没有对应的下游消费者去驱动设计——Stage I 的 FSM 才是第一个会问"到位了吗"的调用方，届时如果真的需要 action 语义，应该在 FSM 侧包一层，而不是现在就在 bridge 里造。

`max_effort` 被忽略而不是拒绝的理由见第5节第4条（已解答）。

### 9.4 `mjContact` 与 `bodiesInContact`：接触检测怎么工作

> Q: 讲解 mjContact 的相关知识，主要是包括在 bodiesInContact 里的使用。这个 bodiesInContact 目前主要是判断机械手的任一夹爪是否和物体接触对吗？

`mjContact`（`mjdata.h`）是碰撞检测的**逐对结果**，`mjData::contact` 是长度 `ncon` 的数组，每次 `mj_forward`/`mj_step` 的位置阶段重新生成——不是持久状态，上一步的接触在这一步会被完全覆盖，`test_grasp_state.cpp` 的 `SetSlidesAndForward` helper 每次都显式调 `api_->forward` 就是因为设了 `qpos` 不会自动触发碰撞检测，必须手动 `mj_forward`。

用到的字段：

- `geom[2]`：**是 geom id，不是 body id**。这是 `bodiesInContact` 必须转换的原因——一个 body 可能挂好几个 collision geom（`left_finger` body 就有 `finger_0` 主体 mesh + 5 个 `fingertip_pad_collision_N` 小方块，见 [panda.xml:220-228](../../robot_description/mujoco/franka_emika_panda/panda.xml)），按 geom 查询会漏掉"手指用另一块 pad 碰到"的情况，所以必须用 `m->geom_bodyid[c.geom[i]]` 转成 body id 再比较。
- `dist`：负值表示穿透，当前实现没用到，只要这对 body 出现在 `contact` 数组里就算接触。
- `frame[9]`：法线方向（`[0-2]`，从 `geom[0]` 指向 `geom[1]`），也没用到，这次只判断"有没有接触"不判断方向。

双向检查（`(b0==a && b1==b) || (b0==b && b1==a)`）的原因：MuJoCo 不保证 broadphase 排序后哪个 geom 落在 `geom[0]`，`test_grasp_state.ContactDetectionIsOrderIndependent` 专门钉住这条。

**你的理解是对的**：目前生产代码里唯一的调用点是 `publishGripperContact()`，分别查 `(left_finger, box)` 和 `(right_finger, box)`。函数本身是通用的两参数接口（不限定必须是"夹爪 vs 物体"），`test_grasp_state.cpp` 里甚至验证了"两个手指不会互相接触"（`FingersDoNotTouchEachOtherWhileBothTouchTarget`），这条测试本身没有生产代码依赖，纯粹是给通用性质的回归保护。

**一个没验证过的缺口**（已进第6节反向清单）：`mjContact` 有 `margin`/`gap` 相关的 `exclude` 字段（0=计入求解，非0=被排除，包括"还在 gap 区间内、检测到了但没真正接触"）。`bodiesInContact` 完全没检查这个字段。当前场景所有相关 geom 的 `margin` 都是默认值 0，大概率不会被触发，但这只是没测过的假设，不是验证过的事实。

### 9.5 `hand_tcp` 为什么要合成、为什么只有 z 轴有偏移

TCP（工具中心点）本周第一次被真正用上——`publishGripperContact()` 里要算 `box_to_tcp_horizontal_m`，需要 TCP 的世界坐标。`hand_tcp` 是什么、偏移量 `0.1034` 从哪来、−45° 手腕旋转为什么不在这个变换里，[docs/architecture.md 第1节](../../docs/architecture.md) 已经是权威记录（Stage C 定的，含一次结论更正），这里不重复，只补两条这次讨论里新的、architecture.md 没写的推理：

**为什么必须合成，不能直接用 `hand`**：`hand` body 的原点是机械设计上的法兰/安装基准面，不是"两个指尖之间、真正发生抓取的那个点"。规划/判据代码要的目标始终是"TCP 到哪"，不是"法兰盘到哪"——不合成这一步，`box_to_tcp_horizontal_m` 这类计算就要在每个用到它的地方各自重复硬编码 `0.1034`，现在集中在 `buildFrameIndex`（TF 合成）和 `publishGripperContact`（数值计算，两处共享同一个 `kHandToTcpZ`）两处，第4周换视觉、第6周接 MoveIt 时下游代码不用再各自算一遍。

**为什么只有 z 轴变化，没有旋转、没有 x/y**：因为 URDF 上游定义的 `tcp_rpy` 就是 `0 0 0`（architecture.md 已记）——两个指尖天然对称分布在 `hand` 局部坐标系的 z 轴（法兰安装面法线方向，也是夹爪"伸出去"的方向）两侧，抓取中心必然落在这条轴上，左右不偏、姿态和 `hand` 完全一致。这不是巧合，是 parallel 夹爪对称设计的必然结果；如果指尖不对称排布，TCP 就需要额外的 x/y 分量。代码里体现为 `publishGripperContact()` 用 `rotVecQuat` 把局部偏移 `{0,0,kHandToTcpZ}` 转到世界系再加到 `hand` 的 `xpos` 上——只转一个纯 z 向量，而不是走 `relativePose`/`frame_math.hpp` 那套完整的父子变换复合，因为这里只需要"合成"（局部→世界），不需要"分解"（世界→父相对）。

### 9.6 夹爪构型：为什么同时解析 `hand` 和 `finger` 两套 body id

> Q: 我看到你同时考虑了 finger 和 hand 的 body id，为什么？实践中会存在有 finger 没 hand 的吗？在这个 panda 里面具体来说是什么样子的。

它们回答的是两个不同问题，`publishGripperContact()` 里能看得很清楚：

- **`hand_body_id_`**（`buildFrameIndex` 里解析）→ 算 TCP 的世界坐标，回答"抓取点现在在世界的哪个位置"——用来算 `box_to_tcp_horizontal_m`（位置判据）。
- **`left_finger_body_id_`/`right_finger_body_id_`**（`resolveGripperFingers()` 里解析）→ 读 `qpos` 算实测宽度，查 `mjContact` 算接触——回答"手指实际张合到多少、有没有真的碰到东西"（宽度/接触判据）。

`classifyGrasp` 的四个条件里，`lifted` 只需要 box 自己的高度，其余三个各自需要上面两组 id 之一——这是两组都要单独解析、单独存的直接原因。

**这两处解析在代码里刻意分开、各自独立 guard**：`buildFrameIndex` 查 `hand` 失败只影响 TCP frame 合成，`resolveGripperFingers()` 查 `left_finger`/`right_finger` 失败只关掉接触/宽度这两个话题，互不影响。原因是它们在原则上可以独立缺失，只是**这个仓库 vendor 进来的两个模型恰好没有给出"有 hand 没手指"或"有手指没 hand"的例子**：

| 模型 | `hand` | `left_finger`/`right_finger` |
|---|---|---|
| `panda.xml`（当前默认用的） | 有 | 有 |
| `panda_nohand.xml` | 没有（`link7` 直接接一个叫 `attachment` 的空 body + `<site>`，留给别的末端执行器用） | 没有 |

**"有手指没 hand"在物理上基本不成立**：两个手指是通过 tendon 被 `hand` 内部的执行器驱动的，手指必须挂在某个父体上，那个父体在设计上就叫 `hand`——概念上不可能真的没有"手指的载体"这一层，只能重命名，不能去掉。

**"有 hand 没手指"是完全合理的场景**：换一个末端执行器（吸盘、三指夹爪、别的品牌夹爪），`hand` 这个法兰/壳体可能还在（TCP 参考点还需要它），但 `left_finger`/`right_finger` 这两个名字就不存在了。这种情况下代码行为是：`hand_body_id_ >= 0` → TCP frame 正常合成；`resolveGripperFingers()` 查不到手指 → 只关掉接触/宽度两个话题，`RCLCPP_INFO` 一行日志，不报错、不影响其他功能。这是比 `docs/architecture.md` 第4节标注过的"单机械臂假设"更细的一层保护——如果当时把 finger 存在性和 hand 存在性绑在一个 if 里，换夹爪时会连 TCP 都跟着消失，是过度耦合。

### 9.7 oracle 数据的分类：哪些会随项目推进消失，哪些需要学习方法

> Q: 我注意到现在似乎有很多的 oracle 数据？比如说 box 的高度、对接触点的解析、整个抓取的 criteria 设计，很多数据都是需要直接从 mujoco 仿真器的 pub 获取的（它们和机械臂的本体数据不一样），我们应该总是假定没有对应的传感器来获取这样的数据对吧？随着项目的推进，哪些 oracle 是可以逐渐去除的？哪些是需要加入学习方法（比如 RL、YOLO、VLA）才可以逐渐解决的。

先分两类，性质完全不同：

**不是 oracle，本体自带、真机上照样有的**：`/joint_states` 的 `position`/`velocity`（编码器）、`effort`（电流估算力矩的类比）、整条机械臂的 TF（由关节角做 FK 算出来，真机上 `robot_state_publisher` 干的就是这件事）、`gripperWidth`（真 Franka Hand 也有编码器读宽度）。这些不需要"去除"，本来就该留到最后。

**真正的 oracle**，按替换手段分三档：

| 信号 | 现在怎么拿 | 替换路径 | 需要学习方法吗 |
|---|---|---|---|
| `~/ground_truth/object_pose`（box 绝对位姿） | `data_->xpos`/`xquat` 直读 | **中间态**：固定场景+已知相机外参，用 ArUco/AprilTag + PnP，纯几何不需要学习；**通用态**：物体种类/纹理/遮挡不可控时，需要检测/分割（YOLO 之类）+ 6D 位姿估计网络，或深度点云配准 | 中间态不需要，通用态需要 |
| `~/ground_truth/left_finger_contact`/`right_finger_contact`（`bodiesInContact` 直查） | `mjData::contact` 直查 | 真机 Franka Hand **本身没有触觉阵列**，标准做法是靠宽度稳态值判断（`franka_gripper` 的 `Grasp` action 就是命令宽度、结束后比较实测宽度和期望宽度是否在 `epsilon` 内）。这条信号在真机上**不是被更聪明的传感器替换，而是被重新设计的判据绕开**——`classifyGrasp` 已经把 contact 和 width 设成两个独立信号，真机版本缺 contact 这一路，只剩 width，判据要退化成两条腿走路 | 不是学习问题，是硬件/判据设计问题 |
| `box_height_m`/`box_to_tcp_horizontal_m`（`classifyGrasp` 的位置项） | 同样来自 `xpos` 直读 | 一旦 `object_pose` 换成感知估计值，这两个数字**自动跟着换**——`classifyGrasp` 吃的是 `GraspSignals` 结构体，不关心信号从哪来，这是 Stage F/H 特意留的口子（`grasp_criteria.hpp` 完全不依赖 `mjModel`/`mjData`） | 跟随 `object_pose` 那一档 |

**RL/VLA 在这里的位置**，和上表是两件不同的事：上表说的是"状态估计"要不要学习方法，RL/VLA 通常解决的是**决策**——如果最终目标是训练一个从图像/本体感受直接映射到动作的策略（VLA 风格），"要不要显式估计 object_pose"这个问题本身可能被绕过：训练时 oracle 仍然存在（用来算 reward、算成功标签），但**部署时 oracle 完全消失**，策略网络直接吃像素+关节状态出动作，不存在"物体位姿"这个中间表示。这正好对应计划书"ground truth 必须走独立接口"的动机——不是为了现在删掉它，是为了保证它可以**在不改任何下游代码的前提下被整体摘除**，换成感知模块或者换成一个端到端策略，这条边界现在就在 `resolveObjectOracle()`/`resolveGripperFingers()` 的独立 guard 里画好了。

### 9.8 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现/防住 |
|---|---|---|
| 单侧手指接触被误判为空抓或抓住 | 只看 `any_finger_touches` 会把"一边卡住一边悬空"归类成正常状态 | `classifyGrasp` 显式区分 `any_finger_touches != both_fingers_touch`，`test_grasp_criteria.OnlyOneFingerTouchingIsUnexpectedContact` 钉死 |
| `bodiesInContact` 假设 geom 顺序 | MuJoCo 不保证 `contact.geom[0]/[1]` 谁是谁 | `test_grasp_state.ContactDetectionIsOrderIndependent` 双向查询都测 |
| `max_effort` 被静默忽略 | 下游以为在控力，实际什么都没发生 | 改成显式 `WARN_ONCE`，不是纯静默——但仍然只警告一次，长期运行中只有第一次调用会被看到 |
| **`kSuccess` 至今没被真实触发过** | 三场景实测覆盖了 `GRASP_EMPTY`/`SLIP`/`UNEXPECTED_CONTACT`，没有一次真正抬起 box | 已进第6节悬挂清单，等 Stage I 的 waypoint |
| `bodiesInContact` 不检查 `exclude` | `margin`/`gap` 非零时可能把"检测到但未生效"的接触误判为真接触 | 目前场景 `margin` 都是 0，未验证过；已进第6节反向清单 |
| `~/gripper_command`/`~/joint_command` 写同一个 `gripper_actuator_id_` 无互斥 | 两个话题同周期都发消息时，最后一次覆盖前一次，无警告 | 目前靠"约定不冲突"，没有运行时保护；已进第6节反向清单 |

## 10. Stage I：FSM 与 `WaypointSource` 抽象（新包 `task_executor`）

### 10.0 一句话总结

新包 [task_executor](../../src/task_executor/)：状态机 `HOME → PREGRASP → GRASP → CLOSE → LIFT → PREPLACE → PLACE → OPEN → RETRACT → VERIFY → DONE`（外加 `RECOVER`/`FAILED` 两个异常出口）的纯函数核心 [fsm.hpp/cpp](../../src/task_executor/include/task_executor/fsm.hpp)（19 个 gtest，Stage F 定的 Layer 1）、`WaypointSource` 接口 + 本周唯一实现 [KeyframeWaypointSource](../../src/task_executor/include/task_executor/keyframe_waypoint_source.hpp)（手测出来的关节空间查表，3 个 gtest）、以及节点胶水 [task_executor_node.cpp](../../src/task_executor/src/task_executor_node.cpp)。`mujoco_bridge` 的 `grasp_criteria.hpp/cpp` 被拆成独立 CMake 库目标 `mujoco_bridge::grasp_criteria` 并导出，`task_executor` 直接链接复用 `classifyGrasp`，不重新实现一遍。

**这个 stage 没有走"用户提问 → 讲解"的形状**——是在自主实现后自己发现、自己排查的（下面 10.9 详细记录）。整段实现踩了三个连续的真 bug，都是在**真跑起来看行为**这一步才发现的，光看代码逻辑看不出来：

1. `motionStep` 的到位判据里，`min_settle_s`（防止读到"上一阶段遗留的已归零速度"）没有被 `kClose`/`kLift`/`kOpen`/`kVerify` 各自的分支复用，只在共享的 `motionStep` helper 里生效；
2. `task_executor_node.cpp` 把"这一 tick 没有发生转移"错误地判定成 `exit_reason == kNone`，但 `kRecover → kHome` 的重试转移**本身**就被 `fsm.cpp` 标成了 `kNone`（它是个重定向，不是一个失败结果）——这个判据让 FSM 卡在 `RECOVER` 死循环，永远不重试；
3. `kPreplace`/`kPlace` 用瞬时的双指接触布尔值判断"掉了没掉"，但 `mujoco_bridge` 自己的抓取日志早就显示过（[9.1](#91-改动清单与验证结果)）单指接触在稳定持握时也会逐 tick 闪烁——用布尔值当判据，会把接触检测的噪声误判成真摔。

三个都在 [10.10](#1010-排查记录三个连续-bug都是跑起来才炸出来的) 详细记录；三个也都在跑通完整流程后**留下了对应的回归测试**（`test_fsm.cpp` 的 `MotionPhaseDoesNotAdvanceBeforeMinSettle`、`CloseDoesNotAdvanceOnSlipBeforeCloseSettleS`、`LiftGivesTheBoxTimeToCatchUpBeforeCallingItSlipped`、`PreplaceToleratesMomentaryFingerContactFlickerWhileStillHeldAloft`）。

### 10.1 改动清单与验证结果

**改动**：

- [src/task_executor/](../../src/task_executor/)（新包）：`package.xml`、`CMakeLists.txt`
- [include/task_executor/phase.hpp](../../src/task_executor/include/task_executor/phase.hpp) / [src/phase.cpp](../../src/task_executor/src/phase.cpp)：`Phase` 枚举 + `nextPhase()` 查表
- [include/task_executor/waypoint_source.hpp](../../src/task_executor/include/task_executor/waypoint_source.hpp)：`WaypointSource` 抽象接口、`ObjectPose`/`JointTarget` 纯结构体
- [include/task_executor/keyframe_waypoint_source.hpp](../../src/task_executor/include/task_executor/keyframe_waypoint_source.hpp)：本周唯一实现，固定查表，忽略 `object_pose`
- [include/task_executor/fsm.hpp](../../src/task_executor/include/task_executor/fsm.hpp) / [src/fsm.cpp](../../src/task_executor/src/fsm.cpp)：纯函数 `step()`，不依赖 `rclcpp`/`mjModel`，只吃手搭的结构体
- [src/task_executor_node.cpp](../../src/task_executor/src/task_executor_node.cpp)：节点胶水——订阅 `/joint_states`、`~/ground_truth/object_pose`、`~/ground_truth/{left,right}_finger_contact`、`/tf`（查 `world→hand_tcp`），发布 `~/joint_command`/`~/gripper_command`，调 `~/reset`，20Hz 定时器驱动 `step()`
- [test/test_fsm.cpp](../../src/task_executor/test/test_fsm.cpp)（19 个用例）、[test/test_keyframe_waypoint_source.cpp](../../src/task_executor/test/test_keyframe_waypoint_source.cpp)（3 个用例）
- [mujoco_bridge/CMakeLists.txt](../../src/mujoco_bridge/CMakeLists.txt) / [package.xml](../../src/mujoco_bridge/package.xml)：把 `grasp_criteria.cpp` 从 `mujoco_bridge_node` 的直接源文件列表里拆成独立库目标 `grasp_criteria`，`ament_export_targets` 导出，`test_grasp_criteria` 改成链接这个库而不是重新编译源文件
- [demo.launch.py](../../src/mujoco_bridge/launch/demo.launch.py)：加入 `task_executor_node`（`use_sim_time=true`），补全 week1 就写在注释里、当时还不存在的"第二个节点"

**编译**（`colcon build --packages-select robot_description mujoco_bridge task_executor`）：

```
Starting >>> robot_description
Finished <<< robot_description [0.08s]
Starting >>> mujoco_bridge
Finished <<< mujoco_bridge [0.10s]
Starting >>> task_executor
Finished <<< task_executor [0.64s]

Summary: 3 packages finished [0.94s]
```

**测试**（`colcon test --packages-select mujoco_bridge task_executor --ctest-args -R "test_"`，仅看 gtest 部分，lint 部分沿用 [6.2](#62-反向清单现在就该做的) 已记录的历史债务）：

```
test_frame_math:               3/3 PASSED
test_state_ops:                6/6 PASSED
test_grasp_criteria:          10/10 PASSED
test_grasp_state:               6/6 PASSED
test_fsm:                      19/19 PASSED
test_keyframe_waypoint_source:  3/3 PASSED
```

**"故意注入错误，测试必须变红"实测**（延续 Stage F/H 定的验收标准，这次在 `fsm.cpp` 上做了两次独立注入）：

| 注入的错误 | 结果 |
|---|---|
| `kClose` 分支的 `outcome == kSuccess \|\| outcome == kSlip` 改成只判 `kSuccess` | `CloseAdvancesOnSlipBecauseThatMeansGrippedNotYetLifted` 变红（1 failure / 3 tests in `Close*` 子集） |
| `motionStep` 里去掉 `elapsed_in_phase_s >= min_settle_s` 这个条件 | `MotionPhaseDoesNotAdvanceBeforeMinSettle` 变红（1 failure / 2 tests in该子集） |

两次都改回后重新编译，`test_fsm` 恢复 19/19。

**行为回归验证**（环境卫生按 [STUDY_NOTES_GUIDE 3.1](../../STUDY_NOTES_GUIDE.md) 确认后，`mujoco_bridge_node` 与 `task_executor_node` 都直接跑可执行文件、不经 `ros2 run`）：

一次完整成功的 episode，全 11 个阶段逐一到位，日志逐阶段打印阶段名/目标构型/到位误差/耗时/退出原因（计划书 [5.3 第3条](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md) 本周范围内那部分）：

```
phase HOME -> PREGRASP: pos_err=0.0066rad elapsed=0.50s exit=REACHED
phase PREGRASP -> GRASP: pos_err=0.0064rad elapsed=0.50s exit=REACHED
phase GRASP -> CLOSE: pos_err=0.1840rad elapsed=0.60s exit=REACHED
phase CLOSE -> LIFT: pos_err=0.1777rad elapsed=2.00s exit=REACHED
phase LIFT -> PREPLACE: pos_err=0.0118rad elapsed=0.50s exit=REACHED
phase PREPLACE -> PLACE: pos_err=0.0082rad elapsed=0.60s exit=REACHED
phase PLACE -> OPEN: pos_err=0.0079rad elapsed=0.50s exit=REACHED
phase OPEN -> RETRACT: pos_err=0.0083rad elapsed=0.50s exit=REACHED
phase RETRACT -> VERIFY: pos_err=0.0088rad elapsed=0.50s exit=REACHED
phase VERIFY -> DONE: pos_err=0.0081rad elapsed=0.50s exit=REACHED
episode DONE
```

`mujoco_bridge` 自己的抓取日志同一段时间窗口里没有一条 WARN/ERROR，`grasp outcome` 在 `SUCCESS`/`UNEXPECTED_CONTACT` 之间正常抖动但始终维持在"抬起来了"的高度区间（`box_z` 从 0.26 一路到 0.40），从未跌回桌面高度——这正是 [10.10](#1010-排查记录三个连续-bug都是跑起来才炸出来的) 修完三个 bug 之后才第一次看到的样子。

**至少一次人为制造失败，确认走 `RECOVER` 而不是卡死或假成功**（本 stage 验收标准里的硬要求）：把 `verify.place_x_m`/`verify.place_y_m` 参数改成物理上不可能到达的 `(99.0, 99.0)`（等价于"把 box 挪到抓不到的位置"这条要求想验证的东西——放置目标永远校验不过），`fsm.phase_timeout_s` 缩到 3s 加速复现：

```
phase VERIFY -> RECOVER: elapsed=3.05s exit=PLACE_MISSED
phase RECOVER -> HOME: exit=NONE
retry 1/3: recovering to HOME
... (完整流程原样再跑一遍，第二次 VERIFY 仍然 miss)
retry 2/3 ...
retry 3/3 ...
phase VERIFY -> RECOVER: elapsed=3.05s exit=PLACE_MISSED
phase RECOVER -> FAILED: exit=RETRY_LIMIT_EXCEEDED
episode FAILED after 3 retries
```

三次完整重试（每次都重新 `~/reset`、重新走一遍全部阶段），第三次耗尽 `max_retries` 后正确落到 `FAILED` 并停止发布命令，不是卡死、也不是在 `VERIFY` 原地假装成功。

### 10.2 纯函数层为什么不用消息类型：Layer 1 的"message-free"原则

> Q: 你提到了"纯函数不使用消息类型"，这里的"纯函数"指的是什么？

**"纯函数"不是 C++ 的语法/语言机制**（不像模板、虚函数那样有对应的关键字），是一个通用的软件工程/编程范式概念，判据是两条：

1. **没有副作用**：不修改任何超出自己参数/返回值范围的状态——不碰成员变量、不碰全局变量、不做 I/O（发消息、写日志、读文件都算）、不修改传进来的引用/指针指向的对象（除非那正是这个函数唯一的目的，比如 `resetToKeyframe(model, data, key)` 修改 `data` 是它的本职工作，但它不会因此碰任何其他状态）。
2. **确定性 / 引用透明**：同样的输入，任何时候调用都得到同样的输出——不依赖隐藏状态（当前时间、随机数、文件内容、网络、`this` 指向的对象内部状态）。

`classifyGrasp(signals, criteria)`、`relativePose(...)`、`step(in, target, params)` 都符合这两条：给定同一组结构体，调用一百次结果完全一样，且调用过程中不产生任何看不见的副作用。这正是它们能在 `test_fsm.cpp`/`test_grasp_criteria.cpp` 里"手搭结构体、不起节点、不连 DDS"就测起来的根本原因——**不是因为它们在哪个包里，而是因为它们没有隐藏输入也没有隐藏输出**。

这个概念本身跨语言通用（Python/Rust/Haskell 里说的是同一件事），只是这份笔记里"纯函数"这个词从 Stage F（[2.1.1](#211-这类系统该怎么测四层本周只取前两层) 的"Layer 1：纯数学/纯函数"）开始反复出现，却一直没有正式定义过，借这次问题补上——记在这里（`week2.md`）而不是 [cpp_concepts.md](cpp_concepts.md)，是因为它不是 C++ 语言机制本身，是贯穿整个项目可测试性纪律的设计概念（跟"要不要给 `onTimer` 造 mock"是同一类判断），按 [STUDY_NOTES_GUIDE 分流规则](../../STUDY_NOTES_GUIDE.md) 应该待在讲"为什么这段代码这样写"的地方。

> Q: 你在 `waypoint_source.hpp` 里提到 "Layer 1 stays message-free discipline"，可以再解释一下这个原则的含义吗？

"message-free" 具体指的是**不直接用 ROS 的消息类型**（`geometry_msgs::msg::Pose`、`sensor_msgs::msg::JointState` 这类由 `.msg` 文件生成的类型），不是"不能有任何外部类型依赖"——`FsmInputs` 里照样嵌了 `mujoco_bridge::GraspSignals`（Stage F/H 就是按同一原则写的纯结构体，本身也不是消息类型）。这条纪律第一次成文是在 [grasp_criteria.hpp](../../src/mujoco_bridge/include/mujoco_bridge/grasp_criteria.hpp) 的注释里——`GraspSignals` "Deliberately not mjModel/mjData"；这次 `ObjectPose`/`JointTarget`（`waypoint_source.hpp`）、`FsmInputs`/`FsmParams`/`FsmDecision`（`fsm.hpp`）是同一原则在 `task_executor` 里的延续。

不用消息类型的三个具体理由：

1. **依赖方向要对**。`geometry_msgs::msg::PoseStamped` 定义在 `geometry_msgs` 包里，链接/包含它意味着这个纯函数模块的构建依赖多了一整个 ROS 消息包——而这个函数本身要的只是 3 个 double（`x, y, z`）加 4 个 double（四元数）。[test_fsm.cpp](../../src/task_executor/test/test_fsm.cpp) 能在**不起 rclcpp、不建 DDS 参与者**的情况下手搭 19 个用例，前提就是它依赖的类型只有 `<array>`/`<cmath>` 和这几个自定义 POD struct。
2. **消息里有和这次计算无关的字段**。`PoseStamped` 还带 `header.stamp`、`header.frame_id`——`jointTargetFor()`/`step()` 根本不关心这次的 pose 是哪个 frame、什么时刻发布的（那是节点侧翻译成 `ObjectPose`/`FsmInputs` 之前就该确认好的事）。让纯函数吃一个带着无关字段的消息类型，等于给"这个函数其实依赖时间戳/frame"这种错误留了一条能编译通过的路。
3. **消息 schema 会变，纯函数的输入契约不该跟着它一起变**。`control_msgs::msg::GripperCommand`、`geometry_msgs::msg::Pose` 是别的包维护的，字段增减不受这个项目控制；`ObjectPose`/`JointTarget` 是本项目自己定义、自己维护的最小接口，只包含 `step()`/`jointTargetFor()` 真正用到的字段，改动只可能发生在这个项目自己决定的时候。

这条原则划的正是 [2.1.1](#211-这类系统该怎么测四层本周只取前两层) 里 Layer 1（纯数学/纯函数）和 Layer 2/3（模型契约、节点契约）之间的边界——**翻译**（把 ROS 消息形状转换成这几个纯结构体）这一步，永远发生在节点胶水层（`task_executor_node.cpp` 的 `onTimer()`），不发生在 `fsm.cpp`/`keyframe_waypoint_source.hpp` 内部。

### 10.3 FSM 的输入、参数、决策与消费：不看 if-else 的整体形状

> Q: 我不希望看状态机的具体 if-else，希望你讲清楚 inputs、params 怎么选的；decision 怎么做出来的；这个状态机最后怎么被使用。

`step()`（[fsm.hpp](../../src/task_executor/include/task_executor/fsm.hpp)）的签名是 `FsmDecision step(const FsmInputs&, const JointTarget&, const FsmParams&)`——三个入参、一个返回值，下面按这四个类型逐一说明，不涉及每个 `Phase` 内部具体判几个条件。

**`FsmInputs`：这一 tick 已知的全部事实**

| 字段 | 来源 | 为什么在这 |
|---|---|---|
| `phase` | 节点自己维护的状态（不是传感器读数） | `step()` 要知道"现在该用哪套判据" |
| `arm`（7 个 position + 7 个 velocity） | `/joint_states` 前 7 个关节，逐 tick 原样搬进来 | 到位判据（`armReached()`）要用 |
| `gripper_width_m` | `/joint_states` 两个手指关节 qpos 相加（沿用 Stage H `gripperWidth()` 的公式） | `kOpen` 阶段单独用 |
| `grasp_signals` | 拼给 `classifyGrasp()` 的结构体——四个字段分别来自 `~/ground_truth/object_pose`（box 高度）、`/tf` 的 `world→hand_tcp`（算 box 到 tcp 的水平距离）、`~/ground_truth/{left,right}_finger_contact` | `kClose`/`kLift` 判据要用同一个（现在导出成库的）`classifyGrasp()` |
| `box_x_m`/`box_y_m` | 同一个 `~/ground_truth/object_pose`，只是给 `kVerify` 单独核对"落点对不对" | `classifyGrasp()` 的 `GraspSignals` 不带这个（它只关心 box 相对 tcp 的距离，不关心相对"放置目标"的距离——两个不同的问题） |
| `elapsed_in_phase_s` | 节点自己算：`get_clock()->now() - phase_start_time_` | 到位判据和超时判据都要用 |
| `retry_count` | 节点自己维护的计数器 | `kRecover` 判断还要不要再给一次机会 |

**没有一个字段是"凭空编出来的"**：全部可以指到某个具体话题或节点自己的簿记状态。这张表本身就是"胶水层要做的翻译工作"的清单——`task_executor_node.cpp` 的 `onTimer()` 前半段基本就是把订阅回调缓存的最新消息，按这张表拼成一个 `FsmInputs`。

**`FsmParams`：判据里的每个数字都能指回一次实测**

不是所有参数都同一来源，按类型分三组：

- **几何/速度容差**（`position_epsilon_rad`、`grasp_position_epsilon_rad`、`velocity_epsilon_rad_s`）——继承 week1 已经确认的事实："ε 不能取零"（position servo 有稳态误差），`grasp_position_epsilon_rad` 单独放大一个量级是因为 `kGrasp`/`kClose` 那个构型下手臂对重力矩的稳态误差本身就大（[keyframe_waypoint_source.hpp](../../src/task_executor/include/task_executor/keyframe_waypoint_source.hpp) 文档已记录具体数值）。
- **三层等待时间**（`min_settle_s`、`close_settle_s`、`lift_settle_grace_s`）——[10.6](#106-到位判据为什么分层三条独立的等一等不是同一件事) 已经讲过，全部来自 [10.10](#1010-排查记录三个连续-bug都是跑起来才炸出来的) 的真实排查过程，不是预先设计好的。
- **抓取判据本身**（`grasp_criteria`）与**放置验收**（`place_x_m`/`place_y_m`/`place_region_radius_m`）——前者直接复用 Stage H 三场景实测定的四个数（[9.1](#91-改动清单与验证结果)），后者是这次自己跑出来的实测落点（[10.1](#101-改动清单与验证结果)）。

**没有一个参数是"随便设的默认值"**——但这不代表它们已经是"对的"，[10.12](#1012-你没问但值得注意的) 第1条已经记了一条悬挂项：三层等待时间的数值本身没有从物理量推导过。

**`FsmDecision`：`step()` 只回答三个问题**

```cpp
struct FsmDecision {
  Phase next_phase;      // 接下来该停在哪个阶段
  ExitReason exit_reason; // 如果发生了转移，原因是什么（用于日志/诊断，不用于流程控制）
  bool is_retry;          // 这次转移是不是 kRecover -> kHome 那个特殊的重定向
};
```

`step()` 的契约是：**给我这些事实，我告诉你接下来该停在哪、要不要报点什么**。它不发消息、不查时钟、不碰任何 ROS/MuJoCo 类型——`exit_reason` 存在的唯一目的是让调用方打日志/统计失败原因，`next_phase == phase`（阶段没变）和 `next_phase != phase`（发生了转移）才是调用方真正要做流程判断的依据（[10.10.1](#10101-第一个-bugkrecover--khome-的重试转移被-exit_reason--knone-误判成没有发生) 记录过一次把这两件事搞混的真实事故）。`is_retry` 单独存在，是因为"要不要给 `retry_count_` 加一"和"接下来去哪个阶段"是两个独立的问题——`kHome` 既是正常开局的第一个阶段，也是每次 `kRecover` 重试后要回到的地方，不能靠"下一个阶段是不是 `kHome`"来判断这次转移算不算一次重试。

**怎么被消费：`task_executor_node.cpp` 的 `onTimer()`，20Hz**

1. 订阅回调只做一件事——把收到的最新消息存进成员变量（`latest_joint_state_` 等），不在回调里做任何计算。
2. 定时器每 50ms 触发一次，做六件事，顺序固定：
   - 从缓存的消息里拼出这一 tick 的 `FsmInputs`（表格里那几行的具体实现，也是**唯一**把 ROS 消息类型翻译成 Layer 1 纯结构体的地方，对应 [10.2](#102-纯函数层为什么不用消息类型layer-1-的message-free原则) 划的边界）；
   - 用当前 `phase_` 问 `waypoint_source_` 要这一阶段该发的 `JointTarget`；
   - **无条件**把这个 `JointTarget` 发布到 `~/joint_command`/`~/gripper_command`——不等 `step()` 的结果，因为哪怕这一 tick 判定"还没到位"，仍然要持续发送同一个目标，否则伺服会松开、机械臂会掉回上一个目标；
   - 调 `step(in, target, params_)`；
   - 如果 `next_phase == phase_`（没发生转移），这一 tick 到此为止；
   - 如果发生了转移：打一行完整日志（阶段/目标/误差/耗时/退出原因，对应验收标准第3条）、按 `is_retry` 决定要不要 `++retry_count_` 并调 `~/reset`、按 `next_phase` 是否为 `kDone`/`kFailed` 决定要不要打收尾日志，最后把 `phase_`/`phase_start_time_` 更新成新值。

这个顺序里最容易漏掉的一点是"无条件发布 target"这一步和"调用 `step()` 判断要不要转移"是**两个独立的动作**，不是一回事——`step()` 从来不负责"让机械臂动起来"，它只负责"看着已经在发生的物理过程，判断该不该换阶段"。这也是为什么 `fsm.cpp` 可以完全不知道"发布"这件事存在：**执行**（把目标变成实际的电机指令）永远在节点侧，**决策**（要不要换下一个目标）被剥成了一个可以喂假数据单独测试的纯函数——和 [2.1.1](#211-这类系统该怎么测四层本周只取前两层) 第3点"不给 `onTimer` 造 mock"是同一个原则的另一次应用：这次胶水层比 `mujoco_bridge_node.cpp` 的更厚（多了拼 `FsmInputs`、查 `WaypointSource`、处理转移），但"胶水层本身不需要测试，测试保护的是它调用的那个决策函数"这条纪律没有变。

#### 10.3.1 `RECOVER` 完整机制：从触发到重试到力竭（此前散落在各处，没有整体讲过）

> Q: 你似乎从来没有记录过整个抓取过程的恢复机制？导致我对状态机的这部分不太能够理解？

这个批评是对的——之前 `is_retry`/`RECOVER` 只在讲别的东西（10.9.1 的 bug、失败模式表里的一行）时被提到过，从没把"一次失败从发生到恢复到力竭"的完整链路串起来讲。补上，按时间顺序走一遍。

**第一步：谁会把 FSM 送进 `kRecover`，条件是什么**——`fsm.cpp` 里一共 7 处 `return {Phase::kRecover, ...}`，按阶段归类：

| 阶段 | 触发条件 | `ExitReason` |
|---|---|---|
| `kHome`/`kPregrasp`/`kGrasp`/`kRetract`/`kOpen` | 走 `motionStep()`：`phase_timeout_s`（默认 6s）内没到位 | `kTimeout` |
| `kClose` | `phase_timeout_s` 内 `classifyGrasp()` 一直没报 `kSuccess`/`kSlip` | `kUnexpectedContact` 或 `kGraspEmpty`（看 `classifyGrasp()` 当时的具体输出） |
| `kLift` | 两条路：①手臂已到位（`armReached()`）且过了 `lift_settle_grace_s`，但 `classifyGrasp()` 还没报 `kSuccess` → `kSlipped`；②纯粹超时（手臂都还没到位）→ `kTimeout` |
| `kPreplace`/`kPlace` | **不等超时**，只要这一 tick `box_height_m` 掉到 `lift_height_threshold_m` 以下就立刻触发（[10.10.4](#10104-第四个独立问题kpreplacekplace-的掉落判据看瞬时接触布尔值被接触检测噪声误判) 讲过为什么看高度不看接触布尔值） | `kSlipped` |
| `kVerify` | `phase_timeout_s` 内没有"box 落在放置区半径内 **且** 松开"这个组合条件 | `kPlaceMissed` |

**没有任何路径能从 `kRecover` 之外直接跳到 `kFailed`**——所有失败都先汇合到 `kRecover` 这一个"路口"，`kFailed` 只能从 `kRecover` 里走出去。这是故意的单一出口设计，[10.4](#104-为什么选状态机模型工程实践里的适用场景与替代方案) 已经讲过"目前还没有出现需要针对不同失败码走不同应对策略的复杂度"，所以这个路口现在只做一件事——数数。

**第二步：`kRecover` 自己只问一个问题**（`fsm.cpp`）：

```cpp
case Phase::kRecover:
  if (in.retry_count < params.max_retries) {
    return {Phase::kHome, ExitReason::kNone, true};
  }
  return {Phase::kFailed, ExitReason::kRetryLimitExceeded, false};
```

`retry_count` 够不够，`max_retries` 默认 3——够就回 `kHome` 重新走一遍全流程（`is_retry=true`），不够（已经用完）就去 `kFailed`（终态）。**这一步本身不知道刚才是哪种 `ExitReason` 把它送进来的**——`fsm.cpp` 只在这个 tick 看得到 `in.phase == kRecover`，上一个 `ExitReason` 只进了日志，没有作为输入传给这次判断。这正是 [10.8](#108-task_executor-的可扩展性后续步骤会替换哪些部分) 表格里"`RECOVER` 不区分 `ExitReason`"那条悬挂项的代码级证据。

**第三步：节点侧怎么真正执行一次重试**（`task_executor_node.cpp` 的 `onTimer()`）：

```cpp
if (decision.is_retry) {
  ++retry_count_;
  RCLCPP_WARN(get_logger(), "retry %d/%d: recovering to HOME", retry_count_, params_.max_retries);
  requestReset();
}
...
phase_ = decision.next_phase;          // == kHome
phase_start_time_ = get_clock()->now();
```

`requestReset()` 调的是 `~/reset` 服务，但**是异步的**（`async_send_request` + 回调，不 `spin_until_future_complete` 等结果）：

```cpp
reset_client_->async_send_request(
  request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    const auto response = future.get();
    RCLCPP_INFO(get_logger(), "reset for retry: success=%d message='%s'", ...);
  });
```

**这里有一个没有被显式同步、只靠时序凑巧对的地方**：`phase_ = kHome` 和 `requestReset()` 几乎同一时刻发生，但没有任何代码保证"仿真那边真的先复位完，`phase_` 再切到 `kHome`"——`async_send_request` 发出去立刻返回，下一行就把阶段切了。如果下一个 50ms tick 到来时 `~/reset` 还没被 `mujoco_bridge` 处理完，`onTimer()` 读到的 `/joint_states` 可能还是重试前、卡在半路的旧姿态，`armReached(HOME 目标)` 这时候大概率是 `false`（旧姿态离 HOME 目标通常还有明显差距），所以**实际上不会误判"已经到家"**——但这依赖的是"旧姿态凑巧不满足 HOME 判据"这个偶然事实，不是显式的等待/确认机制。真正的安全网是 `min_settle_s`（0.5s）：即使复位在下一 tick 就已完成、`armReached()` 立刻为真，`min_settle_s` 还是会挡住第一个 tick 就判定"到位"——这是 `min_settle_s` 存在理由清单里**没写过的第三条**（前两条是"读到上一阶段残留状态"和"kClose 刚接触瞬间"），这次是"异步 reset 请求发出去，但还没确认真的落地"这第三种情况。三条理由不同，但挡的手法是同一个：**给了半秒钟余量，不代表任何具体机制被验证过，只是经验上够用**。实测上（[week2.md 10.9](#109-实测复现gripper_command-双发布者冲突) 那次 `~/gripper_command` 竞态演示，FSM 连续走了 3 次完整重试）从没在这个环节翻过车，但这只是"跑过几次没出问题"，不是"证明过不会出问题"。

**第四步：`kFailed` 是纯粹的死胡同，没有自动重新开始**：

```cpp
void onTimer() {
  if (phase_ == Phase::kDone || phase_ == Phase::kFailed) {
    return;  // 停止发布/决策，节点继续活着但什么都不做
  }
  ...
```

`kFailed`（以及成功的 `kDone`）一旦到达，`onTimer()` 从这行直接 `return`，`phase_` 再也不会被任何代码改动——**没有任何机制会让它自动回到 `kHome` 开始新一轮**。想再跑一次，唯一的办法是重启整个 `task_executor_node` 进程。这是一个**当前故意留白、Stage J 才要填的空白**：Stage J 的 episode runner 需要"连续跑 20 次"，而现在的 `task_executor_node` 一次只能跑一个 episode 到 `DONE`/`kFailed` 就停下——runner 到底是每次都重启整个节点，还是往 `task_executor` 加一个"重新开始"的话题/服务，这个决定还没做，值得记进悬挂清单。

**完整时间线示例**（对照一次真实失败重试，[10.1](#101-改动清单与验证结果) 末尾那次人为制造的 `PLACE_MISSED`）：

```
VERIFY  -> RECOVER: exit=PLACE_MISSED         # 第一步：某阶段触发失败
RECOVER -> HOME:    exit=NONE   (is_retry)    # 第二步：kRecover 判断"还能重试"
  [节点侧] retry_count_: 0->1，WARN 日志，requestReset() 异步发出
  [下一 tick] phase_=HOME, phase_start_time_=now()
HOME -> PREGRASP -> ... -> VERIFY             # 第三步：完整重走一遍全流程
VERIFY  -> RECOVER: exit=PLACE_MISSED         # 同样的失败，再来一次
RECOVER -> HOME:    exit=NONE   (is_retry)    # retry_count_: 1->2
... (第三次同样失败) ...
RECOVER -> HOME:    exit=NONE   (is_retry)    # retry_count_: 2->3，达到 max_retries
... (第四次同样失败) ...
VERIFY  -> RECOVER: exit=PLACE_MISSED
RECOVER -> FAILED:  exit=RETRY_LIMIT_EXCEEDED # 第四步：retry_count_(3) 不小于 max_retries(3)，终态
[ERROR] episode FAILED after 3 retries
```

`max_retries=3` 意味着**总共跑 4 次完整流程**（1 次原始尝试 + 3 次重试），不是 3 次——这个"差一次"的计数细节值得单独记一下，容易凭直觉猜错。

#### 10.3.2 `FsmParams` 完整设计：这些参数怎么编码了整个抓取过程的阶段设计

> Q: 你对 FsmParams 的设计，我希望你能够单开一个章节来介绍，这块的逻辑比较复杂。但是我觉得有利于我了解一下整个抓取过程的阶段设计？

`FsmParams` 不是一堆调参旋钮的杂货堆——每个字段都在回答"这个阶段判断'完成了'需要哪种证据"或"这个证据值得信吗"这两类问题之一。按问题分类，比按字段声明顺序看更容易看出设计意图：

**第一类：几何到位判据——"手臂到了没"**

| 字段 | 回答的问题 |
|---|---|
| `position_epsilon_rad` | 大多数阶段（`HOME`/`PREGRASP`/`RETRACT`/`VERIFY` 等）判断"到位"用的位置容差 |
| `grasp_position_epsilon_rad` | `GRASP`/`CLOSE`/`PREPLACE`/`PLACE` 专用，比上面大一个量级——这几个阶段的目标构型本身要对抗更大的重力矩，伺服稳态误差天生更大（[10.5.1](#1051-这些关节数字的依据在哪实测搜索过程此前只在会话记录里未落盘这次补上) 的搜索数据已经量化过这个现象），用同一个容差会在这些阶段永远判不到位 |
| `velocity_epsilon_rad_s` | 光看位置够不够近不够——还得确认没有在运动中"路过"目标点，两个条件（位置+速度）合在一起才是"稳定停在那"，不是"恰好经过" |

**第二类：三层等待时间——"这一刻的读数值得信吗"**（[10.6](#106-到位判据为什么分层三条独立的等一等不是同一件事) 已经详细讲过每一条背后的物理原因，这里只重复结论）：`min_settle_s`（通用，防"读到上一阶段残留状态"）、`close_settle_s`（`kClose` 专用，防"伺服刚开始收紧就误判抓稳"）、`lift_settle_grace_s`（`kLift` 专用，防"接触检测的逐 tick 抖动恰好撞上 `armReached()` 变真的那一刻"——原先记的"box 需要物理时间追赶手臂"已被实测推翻，见 [10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论)）。

**第三类：全局安全阀——"要不要放弃"**

| 字段 | 回答的问题 |
|---|---|
| `phase_timeout_s` | 单个阶段最多等多久——这不是"到位"判据的一部分，是兜底：如果前两类判据永远等不到满足的那一刻（比如目标本来就不可达），总得有个机制让 FSM 离开这个阶段，而不是永远卡住 |
| `max_retries` | 整个 episode 最多重试几次——[10.3.1](#1031-recover-完整机制从触发到重试到力竭此前散落在各处没有整体讲过) 已经讲过，这条只在 `kRecover` 里用，不参与任何单个阶段的到位判断 |

**第四类：任务语义本身的物理定义**——这两组字段不是"抓取过程的阶段判据"，是**任务成功/失败的定义**，阶段判据只是拿这些定义去比对：

| 字段组 | 定义的是什么 | 复用自哪里 |
|---|---|---|
| `grasp_criteria`（`box_width_m`/`width_epsilon_m`/`lift_height_threshold_m`/`region_radius_m`） | "抓住了"在物理上长什么样 | 原样复用 Stage H 三场景实测定的四个数（[9.1](#91-改动清单与验证结果)），`kClose`/`kLift`/`kPreplace`/`kPlace` 全部拿它去问 `classifyGrasp()` |
| `place_x_m`/`place_y_m`/`place_region_radius_m` | "放对地方了"在物理上长什么样 | Stage I 自己实测出的落点（[10.5.1](#1051-这些关节数字的依据在哪实测搜索过程此前只在会话记录里未落盘这次补上)），只有 `kVerify` 用 |

**把这套分类映射回 11 个阶段，就是整个抓取过程的阶段设计全貌**——每个阶段用了哪几类判据、有没有额外的"值得信吗"防护：

| 阶段 | "到位"用什么证据 | 用到的 `FsmParams` | 额外的读数防护 |
|---|---|---|---|
| `HOME`/`PREGRASP`/`RETRACT` | 纯几何：关节位置+速度 | `position_epsilon_rad`、`velocity_epsilon_rad_s` | `min_settle_s` |
| `GRASP` | 同上，容差放大 | `grasp_position_epsilon_rad`、`velocity_epsilon_rad_s` | `min_settle_s` |
| `CLOSE` | 抓取判据（`classifyGrasp`），不看几何 | `grasp_criteria.*` | `close_settle_s`（比 `min_settle_s` 长，等伺服真收紧） |
| `LIFT` | 抓取判据成功；或"手臂到位且总耗时过了下限却仍不成功"判定失败 | `grasp_criteria.*`、`position_epsilon_rad`、`velocity_epsilon_rad_s` | `lift_settle_grace_s`（阶段总预算下限，非"到位后另起"，见 [10.6](#106-到位判据为什么分层三条独立的等一等不是同一件事)）、`min_settle_s` |
| `PREPLACE`/`PLACE` | 几何到位 + 全程盯着 box 高度没掉 | `grasp_position_epsilon_rad`、`grasp_criteria.lift_height_threshold_m` | 无（掉落检测本身是连续量，不需要防抖，[10.10.4](#10104-第四个独立问题kpreplacekplace-的掉落判据看瞬时接触布尔值被接触检测噪声误判) 讲过为什么） |
| `OPEN` | 夹爪自己的宽度阈值，不是手臂几何 | **不在 `FsmParams` 里**（见下面的发现） | `min_settle_s` |
| `VERIFY` | 位置落在放置区半径内 + 松开 | `place_x_m`/`place_y_m`/`place_region_radius_m` | `min_settle_s` |
| `RECOVER` | 不是到位判据，是计数判据 | `max_retries` | 无 |

**做这张表时发现一个之前没注意到的设计不一致**：`kOpen` 分支自己的开口阈值——

```cpp
constexpr double kOpenWidthM = 0.08;
constexpr double kOpenEpsilonM = 0.02;
```

是写在 `fsm.cpp` 函数体内部的局部 `constexpr`，**不是** `FsmParams` 的字段。系统里所有其它阈值（容差、等待时间、抓取/放置判据）都做成了 ROS 参数（`task_executor_node.cpp` 构造函数里 `declare_parameter`），可以不改代码、不重新编译，只用命令行/launch 文件调；唯独这两个数字锁死在源码里，想改就得改 `fsm.cpp` 重新编译。这不是一个 bug（`kOpen` 目前确实不需要经常调），但破坏了"这套系统的可调参数都在 `FsmParams` 里"这条本来贯穿全局的一致性，值得记一条悬挂项。

#### 10.3.3 `step()`/`FsmDecision` 的设计形状（不看具体实现，只看设计契约）

> Q: 同样的，`FsmDecision step` 这里的逻辑也比较复杂，我希望能有一章节专门对应。对于代码，我觉得只应该学习设计，而不应该在意具体的实现（实现这块我相信你得心应手，至少我可以通过测试程序来控制这块的质量）？

**先定性 `step()` 是哪种函数**：给定"现在的状态"和"这一刻观察到的证据"，算出"下一个状态该是什么"——控制理论/自动机理论里这叫 **Mealy 型转移函数**：输出（这里是 `ExitReason`，带着"为什么"这个诊断信息）既依赖当前状态**也**依赖这一刻的输入，不是只看状态本身（那样是 Moore 型）。举个能感觉到区别的例子：同一个 `kLift` 状态，这一 tick 的 `ExitReason` 到底是 `kReached`、`kSlipped` 还是 `kTimeout`，完全取决于这一刻 `classifyGrasp()`/`armReached()`/`elapsed_in_phase_s` 这几个输入的具体取值——状态本身（"我在 `kLift`"）不能决定输出，必须结合输入才能决定。这不是重要到必须记住的术语，只是"为什么用状态机"（[10.4](#104-为什么选状态机模型工程实践里的适用场景与替代方案)）这个问题在更细的机制层面的延伸答案。

**`FsmDecision` 是一个"命令对象"，不是一次状态修改**——这是全篇最值得记住的设计判断：`step()` 自己**不修改任何状态**，它只返回一个描述"接下来该怎样"的小结构体，由调用方（`task_executor_node.cpp`）决定要不要真的照做。这正是 [10.2](#102-纯函数层为什么不用消息类型layer-1-的message-free原则) 定义的"纯函数"性质在状态机这个场景下的具体应用——**决策**和**执行**被拆成了两个独立的东西：`step()` 只做决策，"真正把 `phase_` 改掉、给 `retry_count_` 加一、调 `~/reset`"这些执行动作全部在节点侧。这个拆分是 `test_fsm.cpp` 能用手搭结构体测 19 个用例、完全不需要起节点的根本原因——不是因为凑巧测得动，是设计本身就把"可测的部分"和"必须有真实 ROS 环境才能跑的部分"物理分开了。

**无论每个阶段内部的判断逻辑多复杂，`step()` 的返回值永远只落进四种形状之一**——这是设计契约的核心，也是"学设计不学实现"这句话真正该看的地方：

| 形状 | 长什么样 | 出现在哪 |
|---|---|---|
| **停留（Hold）** | `{ in.phase, kNone, false }` | 任何阶段，判据还没满足、也没超时 |
| **前进（Advance）** | `{ nextPhase(in.phase), kReached, false }` | 任何阶段的判据满足了，走查表得到的下一个阶段 |
| **求救（Recover）** | `{ kRecover, <某个失败 ExitReason>, false }` | 任何阶段判定失败（超时/掉落/抓空等），统一送进同一个出口 |
| **`kRecover` 自己的两种特殊输出** | `{ kHome, kNone, true }` 或 `{ kFailed, kRetryLimitExceeded, false }` | 只有 `kRecover` 这一个阶段会产生，别处不会 |

**每个阶段"判断该走哪条路"用的证据千差万别**（几何位置、`classifyGrasp` 的抓取分类、夹爪宽度、box 相对放置区的距离），**但无论这次判断多复杂，最后落地的返回值形状永远是上面四种之一**——`task_executor_node.cpp` 只需要认识这四种形状（"阶段变了没有""要不要重试""是不是终态"），完全不需要知道某个阶段这次是靠什么证据做出判断的。这种"内部逻辑各自复杂、对外契约统一简单"的设计，正是让 [10.3](#103-fsm-的输入参数决策与消费不看-if-else-的整体形状) 那六步胶水逻辑能保持简单的原因。

**重复出现的判断形状被提炼成了共享函数，不是每处各写一遍**——11 个阶段里，`HOME`/`PREGRASP`/`RETRACT`（以及 `GRASP`，`PREPLACE`/`PLACE` 兜底路径）用的都是同一种判断形状："几何到位就前进，超时就求救"，这个形状被提成一个叫 `motionStep()` 的共享函数，各阶段只需要传入"用哪个容差"这一个变化的轴。这是一种可命名、可复用的设计技巧——**把转移图里重复出现的判断形状提炼成参数化的共享函数**，而不是在每个分支里复制粘贴同一段 if/else——第3周给 `WaypointSource` 加新实现、或者以后阶段数量继续增长时，同样的技巧还能再用一次。

**真正"特殊"的阶段只有 `kRecover` 一个**——`kClose`/`kLift`/`kPreplace`/`kPlace`/`kVerify` 看起来判断逻辑各不相同，但它们的判断结果**依然落进上面四种形状里的"前进"或"求救"**，只是决定走哪条路时问的问题换成了抓取/位置语义，不是纯几何。只有 `kRecover` 会产生"重试重定向"和"终态"这两种别处完全不会出现的输出——这也是为什么 [10.3.1](#1031-recover-完整机制从触发到重试到力竭此前散落在各处没有整体讲过) 把它单独拎出来整段讲：它是这套转移函数里唯一真正打破"统一四形状"这条规则（准确说是新增了两种只属于它的形状）的地方。

#### 10.3.4 `requestReset()` 是节点调 service 的一般写法吗，和命令行 `ros2 service call` 有什么关系

> Q: `requestReset()` 这里是其它节点调用 service 的一般写法吗？这和我们在命令行上使用 `ros2 service call` 来调用有什么区别和联系？

**是，这是 rclcpp 里调用服务的标准写法之一**——`create_client<ServiceType>(name)` 建一次客户端对象，之后反复用同一个对象发请求，是节点内调用 service 的规范路径。但 `requestReset()` 具体选的是这套标准写法里**两种变体中的异步那种**，这个选择不是随意的，下面分三层讲：

**第一层：两种变体，都是"标准写法"，选哪个看调用点在哪**

| 变体 | 怎么写 | 调用线程会不会被卡住 |
|---|---|---|
| **同步阻塞** | `client->async_send_request(req)` 拿到 `future`，再调 `rclcpp::spin_until_future_complete(node, future)` | 会——当前线程原地等，直到响应到达或超时 |
| **异步回调**（`requestReset()` 用的这种） | `client->async_send_request(req, callback)`，传一个 lambda 作为第二个参数 | 不会——`async_send_request` 立刻返回，请求发出去后当前函数继续往下走，响应到达时 callback 在**未来某次** executor 的 spin 里被调用 |

**第二层：为什么 `requestReset()` 必须用异步，不能用同步阻塞**——`requestReset()` 是从 `onTimer()`（一个定时器回调）内部调用的，而 `task_executor_node` 用的是默认的**单线程 executor**：所有回调（定时器、订阅、这次服务响应）排队在同一个线程上依次执行。如果这里换成同步阻塞的 `spin_until_future_complete`，会立刻死锁——`spin_until_future_complete` 要等的那个"响应到达、完成 future"的事件，恰恰需要同一个线程继续 spin 才能被处理到，但这个线程这一刻正卡在 `spin_until_future_complete` 内部动不了。**从单线程 executor 的某个回调内部，同步阻塞地等待同一个 executor 上的另一个事件，是这套并发模型里一个经典的死锁陷阱**——这也是为什么 `requestReset()` 只能选异步回调这条路，`service_is_ready()` 检查失败时也是直接 `return`（打个警告，下次重试再看），不会等着服务变可用，同样是为了不阻塞这个线程。

**第三层：和 `ros2 service call` 的关系——底层机制完全相同，只是外层包了不同的壳**——`ros2 service call` 命令本身也是一个（临时的）ROS2 节点：它临时建一个 `rclcpp`/`rclpy` 节点，对目标服务 `create_client`，发一次请求，**同步阻塞等待**（这次是安全的，因为这个临时节点除了这一件事什么都不干，没有其它回调需要同一个线程去处理，不存在死锁风险），拿到响应后打印到你的终端，然后进程退出。这正是我在这次会话里给你写的诊断探测脚本（`lift_probe.py` 的 `do_reset()`）用的同一种模式——`call_async()` + `spin_until_future_complete()`，因为那些脚本是一次性跑完就退的独立进程，跟 `ros2 service call` 是同一种"用后即抛、阻塞等结果没关系"的场景。

**一张表总结这次对比涉及的三种角色**：

| 角色 | 用同步还是异步 | 为什么这么选 |
|---|---|---|
| `ros2 service call`（命令行工具） | 同步阻塞 | 临时进程，只做这一件事，阻塞没有代价，用户就是想看到结果再退出 |
| 本次会话写的诊断脚本（`lift_probe.py` 等） | 同步阻塞 | 同上——一次性脚本，没有其它并发任务要抢同一个线程 |
| `task_executor_node::requestReset()` | 异步回调 | 长期运行的节点，`onTimer()` 之后还要继续正常工作（20Hz 定时器不能停），且调用点本身就在同一个 executor 的回调内部，同步阻塞会死锁 |

**联系是**：三者背后调的是同一套 rclcpp 客户端 API（`create_client`/`async_send_request`），传输层走的也是同一条 DDS request/reply 通道——区别只在于"这次调用之后，这个进程/这个线程还有没有别的事要做"：没有别的事，阻塞等结果最简单（`ros2 service call`、诊断脚本）；有别的事必须继续跑（长期节点的回调内部），就只能用异步回调，把"等结果"这件事交还给 executor 自己的调度。

### 10.4 为什么选状态机模型？工程实践里的适用场景与替代方案

> Q: 为什么选择使用状态机模型？在工程实践中通常什么情况下会选择状态机模型？是否还存在其他的类似模型？

**状态机适合的形状**：任务本身就是一串离散、顺序、数量固定的阶段，每个阶段有明确可判定的"到位"条件，失败模式可以枚举归类。Pick-and-place 正好长这个样子——`HOME→...→VERIFY` 这 10 个阶段是计划书直接给定的顺序，不需要运行时决定"接下来做哪个动作"，只需要判断"这一步做完了没"。这类任务的工程实践里，状态机几乎是默认选择：工业机器人的示教/回放模式、大多数 SCADA/PLC 控制逻辑，都是同一个形状。

**状态机不够用/不该用的场景**：

- **阶段数量和顺序在运行时才能确定**（比如根据感知结果动态决定先抓哪个物体、要不要先挪开障碍物）——这时候转移表会随状态数量平方级增长（"状态爆炸"），更适合**行为树（Behavior Tree）**：允许把子行为组合成树、天然支持"优先级 fallback"和并行分支，新增一种应对策略只需要接一个新子树，不需要改所有其他状态的转移表。ROS2 的 Nav2 导航栈就是用 BT.CPP 实现整个导航行为的调度层，比线性 FSM 更适合"正常路径 + N 种恢复策略"的组合爆炸。
- **需要在连续空间里实时决策，而不是在离散阶段间跳转**（比如力控接触任务、动态避障）——这类更适合基于反馈的连续控制器（MPC）或者学习到的策略（RL policy），FSM 的离散切换在连续控制问题里会产生生硬的目标跳变，而不是这次遇到的"稳态误差"这种可以靠 ε 容差解决的问题。
- **状态本身有层次结构**（比如"抓取"这个大阶段内部还有"接近/闭合/确认"三个子阶段，且这种嵌套在多个大阶段里重复出现）——这时候**分层状态机**（Hierarchical State Machine，ROS1 生态常见的是 SMACH/SMACC）能把重复的子状态机封成一个可复用单元，避免每个大阶段都手写一遍相同的转移逻辑。

**本项目现在选 FSM 而不是 BT 的理由**：计划书本身要求的形状就是"阶段划分、到位判据、失败码分层、恢复动作"（[2.4](#24-stage-i--fsm-与-waypointsource-抽象新包-task_executor)），这恰好是 FSM 的教科书场景——阶段数量小而固定（10 个 + 1 个 `RECOVER` 环），转移逻辑目前是一条线加一个统一的恢复出口，还没有出现"多种失败需要不同应对策略"的组合复杂度。这和 [architecture.md 6.2 复用 vs 自建的判断标准](../../docs/architecture.md#62-复用-vs-自建的判断标准) 是同一类判断的另一个维度——那张表回答"这段逻辑该自己写还是调库"，这次回答"该用哪种复杂度的表达工具"，两者的共同原则都是**先用能把当前问题说清楚的最简单工具，工具明显不够用了再换**，不要提前为"可能出现的组合爆炸"设计。[10.8](#108-task_executor-的可扩展性后续步骤会替换哪些部分) 第5点已经记了一条：如果 `RECOVER` 真的需要针对不同 `ExitReason` 走不同恢复策略，到那时候可能就是"FSM 不够用了"的第一个信号。

### 10.5 `WaypointSource` 接口设计：为什么现在只有一个查表实现

`jointTargetFor(Phase, ObjectPose) -> JointTarget` 这个接口本周只有 `KeyframeWaypointSource` 一个实现，且这个实现**完全忽略** `object_pose` 参数——这不是接口设计早了，是计划书本身要求的顺序：本周固定物体位姿（[2.4 注意](#24-stage-i--fsm-与-waypointsource-抽象新包-task_executor) 已经写明），第3周才把手调 waypoint 换成 damped least-squares diff-IK。接口现在就定成这个形状，是让第3周那次替换只改 `WaypointSource` 的一个新实现类，`fsm.cpp`/`task_executor_node.cpp` 一行不改——`step()` 函数签名里 `target` 是作为参数传入的（由调用方从某个 `WaypointSource` 取出来），`fsm.cpp` 本身对"这个目标从哪来"毫无所知。

代价是"多一层间接"：`KeyframeWaypointSource::jointTargetFor()` 里一个 12 行的 `switch` 语句，本可以直接写成 `task_executor_node.cpp` 里的一个查表数组。收益在 `test_keyframe_waypoint_source.cpp` 里已经能看到——`ObjectPoseIsIgnored` 这个测试**现在**看起来像多余的断言（"当然忽略，它是查表"），但它的价值是留一个自动化的哨兵：第3周把这个类换成/新增一个真正吃 `object_pose` 的 IK 实现时，如果有人在旧的 `KeyframeWaypointSource` 上手滑加了一条隐式依赖，这条测试会先炸，而不是等到集成测试才发现"欸这个类怎么还在用一个从没被正确传参的字段"。

#### 10.5.1 这些关节数字的依据在哪：实测搜索过程（此前只在会话记录里，未落盘，这次补上）

> Q: keyframe_waypoint_source 中，你提到关节查找表是实测出来的，提醒我一下依据在哪里？如果没有对应文档就加一下。

依据目前分散在三处：[keyframe_waypoint_source.hpp](../../src/task_executor/include/task_executor/keyframe_waypoint_source.hpp) 头部的三点定性发现（稳态误差随重力矩变化、直接跳跃会撞飞 box、`0.03` 撑不住侧摆需要改 `0.0`）、[docs/architecture.md 第7节](../../docs/architecture.md#7-task_executor-任务状态机stage-i) 的最终数值表、以及 [10.10.5](#10105-一次独立的物理调参夹爪闭合力度不够撑不住-kpreplace-的侧向摆动) 的闭合力度排查记录。**但这三处都只有"最终结论"，没有"怎么搜出来的"这个中间过程**——按 [STUDY_NOTES_GUIDE](../../STUDY_NOTES_GUIDE.md) "实测数据优先于叙述"的纪律，这是一个真实缺口，补在这里。

**方法**：写一个一次性探测脚本（订阅 `~/reset` 服务 + 发布 `~/joint_command`，起 `tf2_ros::Buffer`/`TransformListener` 查 `world→hand_tcp`），对每个候选构型：`~/reset` 回到标称位姿 → 发送候选关节角 → 等待伺服稳定（下面会看到这个等待时长本身就是一项发现）→ 读回 `/tf` 的 `hand_tcp` 世界坐标和 `~/ground_truth/object_pose`。

**发现1：稳态误差需要等多久才算数**——同一个候选构型，等待时长不同，读出的 `hand_tcp` 不同：

| 等待时长 | `hand_tcp` |
|---|---|
| 2s | `(0.7466, -0.0002, 0.2478)` |
| 5s | `(0.7516, -0.0001, 0.2531)` |
| 10s | `(0.7516, -0.0001, 0.2532)` |
| 20s | `(0.7516, -0.0001, 0.2532)` |

5s 之后基本不再变化——这就是 `KeyframeWaypointSource` 文档里"`>= 5 sim seconds`"这个数字的来源，不是拍脑袋定的下限。

**发现2：commanded 越过某点后，实际角度完全不再变化（伺服力矩饱和）**——固定 `joint4=-2.0`，只改 `joint2` 的命令值：

| 命令 `joint2` | 实测 `joint2` | `hand_tcp` |
|---|---|---|
| 0.9 | 0.5566 | `(0.7507, -0.0000, 0.2979)` |
| 1.0 | 0.5566 | `(0.7507, -0.0000, 0.2979)` |
| 1.1 | 0.5566 | `(0.7507, -0.0000, 0.2979)` |
| 1.2 | 0.5566 | `(0.7507, -0.0000, 0.2979)` |

命令从 0.9 加到 1.2，实测角度和末端位置**完全没变**——`joint2` 这个 actuator 在这个伸展姿态下已经被自身重力矩把 `forcerange` 撑满，命令再往前推没有意义。这是 `fsm.cpp` 里 `grasp_position_epsilon_rad` 要比其他阶段松一个量级的直接证据，不是为了让测试通过随便调大的容差。

> Q: 这个定点过程基本上就是在暴力搜索对吗，而且只搜了 joint2 和 joint4；由于只搜了这两个点，没有像 IK 那样利用到所有的关节，所以容易触发到力矩饱和的限制，导致稳态误差大？

**是暴力搜索**——手动挑候选值、发命令、等 5 秒收敛、读 `hand_tcp`，没有任何数值优化或雅可比逆解，纯靠人眼盯坐标数字试出来的。

**"只搜两个关节导致更容易撞饱和"这个因果关系需要澄清一下方向**：不是"因为只搜了这两个关节，所以更容易撞上饱和"，是反过来的——**这次的目标构型（手伸向桌面附近、够到 box 的姿态）本身就需要 `joint2`/`joint4` 承担绝大部分的重力矩负载**，这是任务的几何要求决定的，不是搜索方法造成的。`joint1`（绕竖直轴转，不对抗重力）、`joint5`/`joint6`/`joint7`（负载小，MJCF 里 `forcerange` 也确实只有 ±12，远小于 `joint2`/`joint4` 继承的 ±87）在这个姿态下天然不会饱和，固定它们、只搜 `joint2`/`joint4` 是合理的简化，不是"漏搜了别的关节才导致饱和"。

**但 IK vs 暴力搜索这个对比，抓到了一个更深层的真实差异**：问题不在"搜了几个关节"，在于**暴力搜索没有利用冗余自由度去分散负载**。Panda 是 7 自由度冗余机械臂，到达同一个末端位姿，理论上存在一整条零空间的关节角组合，其中一些组合可能让负载在关节间分摊得更均匀（甚至部分转移到本来空闲的关节），从而避开饱和、把稳态误差压得更小。加了零空间优化的 IK 确实能在解出末端位姿的同时顺便去找一个更省力的关节角组合——这是第3周换成 diff-IK 之后，这类"力度不够"的问题理论上会有所改善的原因，虽然这次没有验证过，第3周接入 IK 后才有意义验证。

**发现3：GRASP/CLOSE 一侧最终定案的过程**（固定 `joint1=0, joint6=1.5708, joint7=-0.7853`，搜 `joint2`/`joint4`）：

| `joint2`, `joint4` | `hand_tcp` |
|---|---|
| 0.2, -1.6 | `(0.5730, -0.0001, 0.3871)` → 定为 **PREGRASP**（悬停在 box 正上方，高度够安全） |
| 0.3, -1.8 | `(0.5122, -0.0001, 0.2555)` |
| 0.4, -2.0 | `(0.4749, -0.0001, 0.2357)` → 定为 **GRASP/CLOSE**（高度接近桌面 0.24，水平位置对准 box 中心 0.50） |
| 0.5, -2.4 | `(0.6443, -0.0000, 0.2871)` |

**发现4：PLACE 一侧同理**（固定 `joint1=0.62` 转到放置区角度后，搜 `joint2`/`joint4`）：

| `joint2`, `joint4` | `hand_tcp` |
|---|---|
| 0.2, -1.6 | `(0.4914, 0.2947, 0.3871)` → 定为 **PREPLACE**（同一悬停高度，只是 x/y 换到放置区上方） |
| 0.22, -1.65 | `(0.4632, 0.3168, 0.3548)` |
| 0.25, -1.72 | `(0.4466, 0.3054, 0.3106)` |
| 0.27, -1.75 | `(0.4322, 0.3084, 0.2887)` → 定为 **PLACE**（这就是 [10.10.5](#10105-一次独立的物理调参夹爪闭合力度不够撑不住-kpreplace-的侧向摆动) 里提到的、实测落点比 `place_marker` 名义位置 `(0.5, 0.3)` 少几厘米的那个数） |

**这张表本身回答了"为什么 PLACE 的实际落点和视觉标记差几厘米"**：不是 bug，是"伸展越远、重力矩越大、稳态误差越大"这条规律（发现2）在水平方向的体现——`joint2`/`joint4` 每往外推一点，`hand_tcp` 的 x/y 就比线性外推的更保守一点，最终稳定在 `(0.43, 0.31)` 附近而不是 `(0.5, 0.3)`。`verify.place_x_m/place_y_m` 参数直接抄的是这次实测的落点，不是标记的名义坐标。

### 10.6 到位判据为什么分层：三条独立的"等一等"，不是同一件事

`FsmParams` 里有三个时间常数，`min_settle_s`、`close_settle_s`、`lift_settle_grace_s`，第一眼像是同一个"防抖"参数抄了三遍，实际回答的是三个不同的物理问题：

| 常数 | 回答的问题 | 为什么不能合并 |
|---|---|---|
| `min_settle_s`（0.5s） | 这一 tick 读到的传感数据，是不是上一个阶段留下的旧值？ | 每次刚进入一个新阶段，`armReached()` 检查的速度分量可能因为"上一个目标已经稳定归零"而恰好也是零，跟"新目标已经到位"完全无法区分。这条对**所有**阶段通用（`motionStep` 内部）——除了 `kClose`，见下一行 |
| `close_settle_s`（2.0s） | 夹爪这一 tick 读到的宽度/接触，是不是刚开始收紧、还没挤紧到位？ | `kClose` 是本周三个 bug 里踩得最深的一个（[10.10.3](#10103-第三个-bug夹爪刚接触-box-瞬间的-kslip-被当成ready进入-lift)）：`classifyGrasp` 在手指刚碰到 box 的瞬间就可能报 `kSlip`（宽度+接触"看起来像抓住了"），但伺服还远没挤紧，这个夹持撑不住接下来的 `kLift`。这条比 `min_settle_s` 大一个量级（2s vs 0.5s），因为要等的是伺服**收敛**，不是单纯"跳过上一个阶段的余量" |
| `lift_settle_grace_s`（2.0s） | `armReached()` 首次持续变真的那个精确 tick，恰好撞上接触检测的一次单指假读数，怎么办？ | **原先记录的解释（"box 靠摩擦被动追赶手臂，需要将近 1 秒物理响应时间"）已被实测推翻并进一步查明确切根因**——box 高度越过成功阈值的时刻（t≈0.02s）反而比手臂自己收敛完成（t≈0.46s）早了二十多倍，不存在"box 追赶"。真正的根因（50 次实验里 1 次实测撞见，[10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论) 有完整数据）：夹爪力度较松时，接触检测的逐 tick 抖动窗口可能长达数百毫秒，恰好覆盖住手臂收敛完成的那一刻，约 2% 概率撞车。**注意计时起点**：这条预算是从进入 `kLift` 这个阶段起算，不是从"手臂到位"那一刻起算的第二个独立计时器。健康的抓取流程大概率根本不会走到这个分支——`classifyGrasp()` 的成功判定通常在阶段刚开始不久就已经满足 |

三者共同的结构是**"到位"从来不是单一时刻的判断，是要先问清楚现在这个读数值不值得信**——第1周（[week1.md 8.2](week1.md#82-sim-time-vs-wall-time以及-use_sim_time)）已经在时钟语义上撞过一次同类问题，这次是同一个教训在关节空间/接触空间的重现。

### 10.7 为什么 `task_executor` 不发布/订阅一个 `GraspOutcome` 话题

Stage H 结束时留了一条悬挂项（[6.1](#61-清单) 第4条）：`GraspOutcome` 只进日志，没有配套话题，因为"不知道 `task_executor` 想要整个分类结果、还是原始 `GraspSignals`"。这次的答案是**都不要，直接复用函数**：`mujoco_bridge` 已发布的四条话题——`~/ground_truth/object_pose`、`~/ground_truth/{left,right}_finger_contact`、`/joint_states`——已经是 `classifyGrasp()` 需要的全部输入，`task_executor_node.cpp` 自己在 `onTimer()` 里拼一个 `GraspSignals` 结构体，调同一个 `mujoco_bridge::classifyGrasp()`（现在是导出的库函数，见 [10.0](#100-一句话总结)），而不是等 `mujoco_bridge` 先算好再发过来。

理由是**谁需要额外上下文，谁就该拥有计算权**——`classifyGrasp()` 自己承认（`grasp_criteria.hpp` 的文档）分不清"抓住但还没起飞"和"起飞后真的滑了"，这个歧义只有拥有阶段信息的调用方才能消歧（[10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论) 就是这个消歧的具体样子：同一个 `kSlip` 结果，在 `kClose` 里读成"可以，进入下一步"，在 `kLift` 里读成"真摔了，去 `RECOVER`"）。如果让 `mujoco_bridge` 先分类好再发布，等于让不掌握阶段信息的一方替掌握阶段信息的一方做决定，结果只能是发一个粗粒度结果、下游再重新猜一遍上下文——不如直接把原始信号发出去（已经在发），分类逻辑作为一个纯函数库随时可以在任何知道自己在哪个阶段的地方调用。这也顺带解决了 Stage H 那条悬挂项：不是"话题契约现在定不出来"，是"这里根本不需要一个新话题"。

#### 10.7.1 `GraspSignals`/`classifyGrasp` 为什么物理上放在 `mujoco_bridge` 包里，不是 `task_executor`

> Q: 我注意到 `GraspSignals` 被放在 `mujoco_bridge` 而不是 `task_executor` 这里，我猜想原因也是：尽可能减少依赖，抓取程序的测试需要 mujoco 的 oracle 数据，所以放在 mujoco 这里可以让我们在不依赖其它节点的情况下实现抓取信号和抓取判定。（对吗）

**不完全对，猜想里的因果关系反了**——"能不依赖其它节点单独测试"这件事，靠的是 [10.2](#102-纯函数层为什么不用消息类型layer-1-的message-free原则) 讲的"纯函数"性质（不碰 `mjModel`/`mjData`/`rclcpp`，只吃手搭的结构体），跟这段代码**physically 放在哪个包**没有关系。就算把 `grasp_criteria.hpp/cpp` 整个搬进 `task_executor`，只要它继续保持纯函数的形状，一样可以脱离节点独立测试——`test_grasp_criteria.cpp` 现在能测起来，不是因为它在 `mujoco_bridge` 里，是因为它是纯函数。

**真实原因是历史顺序，不是刻意的依赖最小化设计**：`classifyGrasp()`/`GraspSignals` 是 Stage H 造出来的，那时候 `task_executor` 这个包**还不存在**。Stage H 造这套东西的直接目的是让 `mujoco_bridge_node` 自己在 `onTimer()` 里实时打印抓取状态（三场景手动验证，见 [9.1](#91-改动清单与验证结果)），所以它自然就长在 `mujoco_bridge` 包里——不是因为提前规划好"以后要给 task_executor 用，先放这里比较好测"，而是当时唯一的消费者就是 `mujoco_bridge` 自己。等 Stage I 造 `task_executor` 时才发现它也需要同一套分类逻辑，这时候面前有三个选择：①在 `task_executor` 里重新写一遍 `classifyGrasp()`（两份逻辑迟早会静默漂开，[CMakeLists.txt](../../src/mujoco_bridge/CMakeLists.txt) 里 `grasp_criteria` 库那段注释原话是"reimplementing classifyGrasp() a second time...having the two drift apart silently"）；②把这段代码从 `mujoco_bridge` 挪到一个两边都能依赖的中立位置；③原地导出成一个库目标，两边都链接同一份实现。选的是③，不是②——**这其实是个务实但不完美的选择**。

**一个值得诚实指出的架构瑕疵**：仓库里其实已经有一个空的 [manipulation_interfaces](../../src/manipulation_interfaces) 占位包（`package.xml`/`CMakeLists.txt` 都没有，纯空目录）——如果当初把 `grasp_criteria.hpp/cpp` 放进这样一个两边都不特殊依赖的中立包，`task_executor` 就不需要在 `package.xml`/`CMakeLists.txt` 里写 `find_package(mujoco_bridge REQUIRED)`。现在的写法制造了一个有点奇怪的依赖方向：**一个任务执行节点，为了拿一段纯数学分类函数，要在构建期依赖"整个仿真桥接包"**。这和 [10.8](#108-task_executor-的可扩展性后续步骤会替换哪些部分) 表格里"话题契约不变就不用改"的说法不完全一致——话题契约层面 `task_executor` 确实不关心背后是仿真还是真机，但**构建期**它现在硬链接着 `mujoco_bridge` 这个具体包名。真到了第6周换真实驱动、`mujoco_bridge` 这个包本身可能被换掉或者不再随手起时，`task_executor` 的 `CMakeLists.txt` 会因为 `find_package(mujoco_bridge REQUIRED)` 找不到包直接编译失败——这不是"缺一次推演"，是一个已知但目前决定不修的技术债，值得记进悬挂清单。

### 10.8 `task_executor` 的可扩展性：后续步骤会替换哪些部分

> Q: 解释一下 `task_executor` 的扩展性体现在哪里，在后续的步骤中哪些地方会被替换掉？

按"谁会被换、换了以后 `task_executor` 要不要跟着改"整理成一张表：

| 会被替换的部分 | 现在是什么 | 换成什么、什么时候 | `task_executor` 要不要跟着改 |
|---|---|---|---|
| `WaypointSource` 的具体实现 | `KeyframeWaypointSource`：固定查表，忽略 `object_pose` | 第3周：damped least-squares diff-IK，真正读 `object_pose` 算目标 | **不改** `fsm.cpp`/`task_executor_node.cpp`——这正是 [10.5](#105-waypointsource-接口设计为什么现在只有一个查表实现) 定的接口存在的理由 |
| `~/ground_truth/object_pose` 的发布者 | `mujoco_bridge` 直读 `xpos`/`xquat`（oracle） | 第4周：感知节点发布估计位姿 | **只要话题名/消息类型/frame 不变，不改一行**——[9.7](#97-oracle-数据的分类哪些会随项目推进消失哪些需要学习方法) 已经讨论过这条边界；如果换了话题名，只是改一处订阅参数，不是改逻辑 |
| `~/ground_truth/{left,right}_finger_contact` | 仿真专有的接触检测 | 真机上没有这个信号（[9.7](#97-oracle-数据的分类哪些会随项目推进消失哪些需要学习方法) 已定：真机是判据设计问题，不是传感器升级问题） | **需要改**——`fsm.cpp` 里看 `bothFingersHolding()`/`classifyGrasp()` 接触信号那部分逻辑要退化成只看宽度，`grasp_criteria.hpp` 现在还没有这个"无 contact 版"的分类函数 |
| `~/joint_command`/`~/gripper_command` 的接收者 | `mujoco_bridge_node` 直接写 `mjData::ctrl` | 第6周：真实 `ros2_control` 驱动 | **不改**——Stage E 就定的原则："这两个话题只设目标，不管执行"，`task_executor` 从一开始就只对着这个契约编程，不知道也不需要知道背后是仿真还是真机 |
| `RECOVER` 的恢复策略 | 不区分 `ExitReason`，统一回 `kHome` 重试 | 未预定，可能在失败码分布（Stage J 的 CSV）显示某类失败重试无效时才会做 | 目前**接口已经预留、行为还没用上**：`ExitReason` 枚举已经区分了 `kTimeout`/`kSlipped`/`kPlaceMissed` 等（[fsm.hpp](../../src/task_executor/include/task_executor/fsm.hpp)），但 `kRecover` 分支对所有原因一视同仁。这是新悬挂项，已记进第6节 |
| 物体初始位姿 | 固定（`pick_place_scene.xml` 的 `pick_place_home` keyframe） | 计划书 2.1 的下一步：随机化位置/偏航角 | `KeyframeWaypointSource` 完全忽略 `object_pose`，随机化的第一天就会立刻暴露这条短板——这正是"缺参照系"的悬挂项解锁条件（[10.5](#105-waypointsource-接口设计为什么现在只有一个查表实现) 已经指出，第3周接 IK 之后才有意义验证） |

**共同的设计原则**：`task_executor` 之所以能在这么多处"不用跟着改"，是因为它从不直接依赖任何一处**实现**，只依赖三个**契约**——`WaypointSource` 接口、`mujoco_bridge` 的话题名+消息类型、`classifyGrasp()` 的函数签名。只要这三个契约不变，背后换成什么实现都和 `task_executor` 无关；一旦某处替换连契约本身也变了（比如去掉 finger contact 信号），才需要真的改 `task_executor` 的逻辑。这个边界本身不是显式设计出来的，是延续了 Stage C/D/E 已经定下的一系列"仿真专有接口用私有名/oracle 走独立话题/joint_command 只设目标不管执行"的既有原则——`task_executor` 只是这条边界的第一个真实受益者。

### 10.9 实测复现：`~/gripper_command` 双发布者冲突

> Q: 我注意到 `~/gripper_command` 被两个模块发布（[gripper_test.py](../../scripts/gripper_test.py) 和 `task_executor_node`），如果它们会同时发布，那么最后的混杂的消息是如何处理的？

这条问题在 [6.2](#62-反向清单现在就该做的) 里已经记过一次悬挂项（Stage H："共享 `gripper_actuator_id_`，没有互斥/冲突检测"），当时是"缺一次推演"——没有真实场景触发过。这次直接跑了一次：同时起 `mujoco_bridge_node` + `task_executor_node` + `gripper_test.py`，两个客户端各自独立向 `~/gripper_command` 发消息，观察实际后果。

**机制**：ROS2 的话题是多发布者/多订阅者模型，不是"一个话题只能有一个源"——订阅端收到的是**交错到达的、彼此独立的消息流**，中间没有合并、没有优先级、没有仲裁。`control_msgs/GripperCommand` 这个消息类型本身：

```
float64 position
float64 max_effort
```

**没有 `header`，没有时间戳，没有任何字段能说明"这条消息是谁发的"**——`onGripperCommand()` 收到一条消息，唯一能做的就是全盘接受：

```cpp
data_->ctrl[gripper_actuator_id_] = (msg->position / 2.0) * gripper_ctrl_scale_;
```

这是一次无条件赋值，不是"合并"或"取平均"。两个发布者同一控制周期都发消息时，物理效果就是**最后被 `onGripperCommand()` 处理的那一条说了算**——而"哪一条最后被处理"取决于 DDS 中间件的调度和两个进程各自的发布节奏，从代码层面完全不可预测，也不会有任何日志或异常提示这件事发生过。

**实测复现**（环境卫生按 [STUDY_NOTES_GUIDE 3.1](../../STUDY_NOTES_GUIDE.md) 确认后，三个节点都直接跑可执行文件/脚本，不经 `ros2 run`）：`task_executor` 进入 `CLOSE`/`LIFT` 阶段持续发送"闭合"命令的同时，`gripper_test.py` 按固定 2 秒周期独立发送开/合切换。日志对齐后看到的真实序列：

```
[gripper_test]  target: open                                                          # sim t=781.179
[mujoco_bridge] grasp outcome -> SUCCESS (width=0.0404m box_z=0.3838m ...)             # t=781.238，box 稳定夹在半空
[mujoco_bridge] grasp outcome -> UNEXPECTED_CONTACT (width=0.0434m box_z=0.3849m ...)  # t=781.298
[mujoco_bridge] grasp outcome -> SUCCESS (width=0.0413m box_z=0.3759m ...)             # t=781.338
[mujoco_bridge] grasp outcome -> UNEXPECTED_CONTACT (width=0.0434m box_z=0.3754m ...)  # t=781.388
[mujoco_bridge] grasp outcome -> GRASP_EMPTY (width=0.0297m box_z=0.3325m ...)         # t=781.478 -- 90ms 内 width 跳水
[mujoco_bridge] grasp outcome -> NO_OBJECT (width=0.0287m box_z=0.3230m ...)           # t=781.488
[mujoco_bridge] grasp outcome -> UNEXPECTED_CONTACT (width=0.0310m box_z=0.3012m ...)  # t=781.508
[mujoco_bridge] grasp outcome -> NO_OBJECT (width=0.0295m box_z=0.2755m ...)           # t=781.528 -- box 跌回桌面高度
[task_executor] phase PREPLACE -> RECOVER: elapsed=0.20s exit=SLIPPED                  # t=781.551
```

`gripper_test.py` 的 `target: open` 一发出，`width` 在 90ms 内从 0.0434m 跳到 0.0297m——这正是伺服对新 ctrl 目标的物理响应时间，不是瞬间的（[week2.md 9.2](#92-position-servo-下夹住为什么是稳态位置误差不是力) 已经讲过这个伺服机制）。box 应声跌落（`box_z` 0.3838→0.3230→0.2755），`task_executor` 的 `fsm.cpp` 在 `PREPLACE` 阶段正确检测到高度跌破阈值，判定 `SLIPPED` 进 `RECOVER`——FSM 这一步反应是对的，但**根因**（夹爪command 被外部覆盖）从 bridge 到 FSM 全程没有一条日志指出来，只能靠下游物理状态的异常间接推断。

**结论**：`gripper_test.py` 和 `task_executor_node` 不能同时对着同一个 `mujoco_bridge_node` 实例跑——这不是理论上的边界情况，是这次实测就复现的真实故障。两者都是各自独立、互不知情的合法发布者，ROS2 的话题模型本身不提供任何"排他访问"的机制；如果真的需要排他，要么靠**运行时的应用层协议**（比如加一个"当前控制权归属"的话题/参数，命令前先检查），要么把这类命令接口从 topic 换成 service/action（有返回值，能在同一时刻拒绝第二个调用者），要么最朴素地靠**人工纪律**（写清楚"这两个东西不能同时开"，就像现在这样）。这次复现把 [6.2](#62-反向清单现在就该做的) 那条悬挂项转正为一个已确认的真实约束，记进下面的失败模式表。

### 10.10 排查记录：三个连续 bug，都是跑起来才炸出来的

**背景**：这个 stage 是自主实现，中间没有走 STUDY_NOTES_GUIDE 常规的"用户提问→讲解"流程。但仍然完整走了"写代码 → build + 实跑验证"这一步——三个 bug 全部是在**真的跑一次完整 episode** 这一步暴露的，纯读代码/纯看单测通过看不出任何异常（三个 bug 出现前，`test_fsm` 全部 16 个原始用例都是绿的）。这恰好印证了 [STUDY_NOTES_GUIDE 3](../../STUDY_NOTES_GUIDE.md) 定的第2步"不是编译通过就算完"——这次连"单测全绿"也不够,必须接一次真实的 `mujoco_bridge` + `task_executor` 联调。

#### 10.10.1 第一个 bug：`kRecover → kHome` 的重试转移被 `exit_reason == kNone` 误判成"没有发生"

**现象**：第一次跑完整流程，`LIFT → RECOVER` 之后日志永远停在这一行，`task_executor_node` 既不重试也不报错，进程仍在跑（`ps` 正常），但没有任何后续日志——比"卡死"更隐蔽，因为进程状态看起来完全正常。

**线索**：`fsm.cpp` 的 `kRecover` 分支：

```cpp
case Phase::kRecover:
  if (in.retry_count < params.max_retries) {
    return {Phase::kHome, ExitReason::kNone, true};
  }
```

`kRecover → kHome` 这条转移本身**故意**标成 `ExitReason::kNone`——它是个重定向而不是一个失败结果，`kNone` 用来表示"这不是一个需要向用户解释的退出原因"。但 `task_executor_node.cpp` 的早退条件写的是：

```cpp
if (decision.exit_reason == ExitReason::kNone) {
  return;  // Still in progress; nothing to log or transition.
}
```

`kNone` 同时被两种情况复用：`step()` 还没做出决定（阶段没变）**和** `step()` 做出了决定但决定被标成"不是失败"（阶段变了）。这段判据把两者混为一谈，选错了那个更常见、更容易先测到的情况，直到联调才暴露被选错的那个分支。

**修复**：改成比较阶段是否真的变了：

```cpp
if (decision.next_phase == phase_) {
  return;  // Still in progress; nothing to log or transition.
}
```

**留下的经验**：`FsmDecision` 里 `exit_reason` 和 `next_phase` 是两个独立的维度（"发生了什么"和"要不要继续走"），却只用其中一个字段做流程控制判据——这类"一个字段身兼两职"的设计，字段的两种用法总有一种是调用方没考虑到的，教训和 [8.8](#88-排查记录keyframe-名字冲突与长度不匹配是两件独立的事结论被推翻) 的"两个检查看起来像同一件事，实际是独立的"是同一类。

#### 10.10.2 第二个 bug：接触检测闪烁撞上到位检查（根因已查清，含一次被推翻的旧结论）

这一节记录四层内容，严格分开：**①事实**（现象和最初的实测数据）、**②被推翻的旧结论**（明确标记为错，留作教训）、**③排查过程**（两轮实验，含一次方法论错误）、**④确认的根因**（有实测证据支撑的最终结论）。

**① 事实：现象**

修完第一个 bug（[10.10.1](#10101-第一个-bugkrecover--khome-的重试转移被-exit_reason--knone-误判成没有发生)）后重新跑，流程能往前走了，但每次都在 `LIFT → RECOVER`（`exit=SLIPPED`）失败，不能继续到 `PREPLACE`。加上 `lift_settle_grace_s`（2.0s）之后，这条失败路径不再触发。

**② 被推翻的旧结论**

> ~~查同一时间窗口的日志，`box_z` 在持续上升（0.26→0.37），据此推断"box 靠摩擦力被夹爪带起来，不是刚性绑死在手上，手臂停止移动后 box 还要花将近 1 秒才能追上新高度"，`lift_settle_grace_s` 是给这段物理追赶时间留出的余量。~~

这段话唯一的依据是两个日志时间窗口大致重叠，**从没有对着逐 tick 数据核实过谁先谁后**。用户追问"这是打滑吗""是网络延迟吗"，逼着回去重新实测才发现顺序是反的：写探测脚本逐 tick 记录 `joint2`/`joint4` 位置+速度、box 高度、接触布尔值，4 次重复实验一致显示 box 高度越过成功阈值（t≈0.02s）比手臂完成收敛（t≈0.46s）早了二十多倍——box 是**先**到的，根本不存在"box 追赶手臂"。

**③ 排查过程：两轮实验**

**第一轮尝试（方法论错误，作废）**：想通过快速重启 `task_executor_node` 进程 25 次来批量制造失败样本，3/25 出现"失败"，但逐条检查发现全部是假的——新进程的 `~/reset` 客户端还没和 `mujoco_bridge` 完成 DDS 发现就抢先发了复位请求（日志有 `~/reset not available yet` 警告），静默失败，box 停在上一轮 `PLACE` 的终点，后续抓空是必然的，跟 `kLift` 这个 bug 毫无关系。**这是一个真实但完全不同的问题**（连续 episode 之间的启动竞态），已记入 [6.1 悬挂清单](#61-清单)，不是这次答案。

**第二轮实验（不重启进程，30 次独立 `CLOSE→LIFT` 循环，直接照抄 `classifyGrasp`/`armReached` 公式复现判据）**：全部使用**当前**的夹爪闭合命令 `grip(0.0)`（`KeyframeWaypointSource` 现在的值），结果 30/30 手臂到位那一刻分类结果都是 `kSuccess`，一次撞车都没有。更细的追踪（记录接触回调的原始时间戳）显示：接触闪烁只发生在 t≈0.05~0.08s（抓取刚建立的瞬间），到 t≈0.09s 就彻底稳定为 `True/True`——而 `armReached()` 变真在 t≈0.46s，比闪烁结束晚了将近 0.4 秒，**这两个窗口根本不重叠**，假说 A（接触闪烁撞车）在这个条件下不成立。

**关键的转折点（用户追问带来的洞察）**：用户问"这么干脆的抓取，怎么会撞上 `min_settle_s` 都防不住的失败"，逼着重新核对实验条件——才意识到 `grip(0.0)` 是 [10.10.5](#10105-一次独立的物理调参夹爪闭合力度不够撑不住-kpreplace-的侧向摆动) 才改过来的**较新、较紧**的力度；而按 bug 发现的时间顺序，**这次 `kLift` bug（bug #2）比夹爪力度调参（bug #5）先被发现**——也就是说，原始 bug 触发时，夹爪闭合命令用的是**更早、更松**的 `grip(0.03)`（"比 box 窄一点"的常规写法），不是现在这个更紧的 `0.0`。之前的复现实验从一开始就用错了物理条件。

**用正确的历史条件重新实验**：把探测脚本的夹爪命令改回 `grip(0.03)`，重复相同的 30 次流程，追踪接触闪烁的真实持续时长——**闪烁窗口从 t≈0.09s 结束，大幅拉长到 t≈0.8~0.9s 才结束**（10 次样本里 8 次），和之前 `0.0` 力度下"瞬间稳定"的表现完全不同：较松的挤压力度让接触本身在整个抬升过程中都不稳定，不是只在最初瞬间抖一下。

**④ 确认的根因**

把样本量加到 50 次（`grip(0.03)`），逐个核对"`armReached()` 首次持续变真的那个精确 tick，双指是否同时接触"——**第 47 次实验里真的撞车了**：

```
t=0.4519  right_finger_contact 变为 False
t=0.4619  right_finger_contact 变回 True     <- 只持续了约 10ms 的假读数窗口
armReached() 首次持续变真的时刻：t=0.4618   <- 精确落在这个假读数窗口内
```

这一刻 `left_finger_contact=True`、`right_finger_contact=False`，`both_fingers_touch` 为假，`classifyGrasp()` 不会报 `kSuccess`（不管宽度条件是否满足）——**这正是假说 A 描述的机制，第一次拿到了真实数据**：`armReached()` 的到位判断和接触检测的逐 tick 抖动是两个独立采样的信号，在这次夹爪力度较松（`0.03`）的条件下，接触闪烁窗口被拉长到能覆盖住手臂收敛完成的时刻，50 次里撞上 1 次（约 2% 概率）——低概率但真实存在，和"Stage I 排查时真实撞上过一次、之后很难再复现"这个历史描述完全吻合。

**假说 B（手臂自身瞬时假到位）在这两轮实验（共 84 次独立试验）里从未被观察到**——手臂的收敛过程每次都是一次性、干净地进入稳定状态。不能说这个假说被彻底排除（可能需要更容易产生振荡的场景才会出现），但这次的证据全部指向假说 A。

**为什么原来的推翻实验（4 次，用 `0.0` 力度）什么都没测到**：不是运气不好，是**从物理条件上就不可能撞车**——`0.0` 力度下闪烁在 t≈0.09s 就已经彻底停止，远早于手臂 t≈0.46s 收敛完成，两个窗口不重叠，撞车的必要条件都不满足。这次的 84 次实验分成"新力度 0.0"（34 次，0 次撞车）和"旧力度 0.03"（50 次，1 次撞车）两组，直接对比出了力度松紧对闪烁持续时长的影响。

**当前修复为什么依然是对的**：`lift_settle_grace_s`（2.0s）——给足够多次独立的 tick 机会，等一次干净的读数——对这个已确认的机制完全对症：一次约 10ms 的假读数窗口，2 秒的预算里有大把机会等到下一次读到 `kSuccess`。**这个数字本身依然是"远超所需但凑巧够用"**：需要覆盖的只是一次接触闪烁的窗口（毫秒级），不是任何物理追赶过程，2.0s 这个量级没有被精确校准过，只是留了足够宽裕的余量。

```cpp
if (arm_at_lift_height && in.elapsed_in_phase_s > params.lift_settle_grace_s) {
  return {Phase::kRecover, ExitReason::kSlipped, false};
}
```

**现在能明确回答的问题——真正的修复方向该往哪走**：既然根因确认是"夹爪力度较松时，接触检测的逐 tick 抖动窗口变长，可能撞上到位检查"，`lift_settle_grace_s` 这种"拖时间等一次干净读数"的方案是**在消费端兜底**，没有解决"接触信号为什么会抖这么久"这个源头问题。更彻底的修复方向：① 现在 `KeyframeWaypointSource` 已经用了更紧的 `0.0` 力度（[10.10.5](#10105-一次独立的物理调参夹爪闭合力度不够撑不住-kpreplace-的侧向摆动) 出于另一个独立原因做的调整），这本身就顺带压缩了闪烁窗口，降低了撞车概率，但没有归零；② 真正对症的方向是给接触信号本身做去抖动（连续 N 个 tick 一致才采信），但这条信号在真机上根本不存在（[week2.md 9.7](#97-oracle-数据的分类哪些会随项目推进消失哪些需要学习方法)），投入在它身上不服务于"逼近真机"这个项目目标；③ 更值得投入的方向是实现一个真机同款的"宽度-only"抓取判据（不依赖接触信号，只看稳态宽度误差，[week2.md 9.7](#97-oracle-数据的分类哪些会随项目推进消失哪些需要学习方法) 已经指出这是 `franka_gripper` 的标准做法），这样既绕开了这次的抖动问题，又是一份不管有没有真机都有意义的投入——这个方向已经记进 [6.1 悬挂清单](#61-清单)，作为比"给接触信号去抖动"更优先的选项。

**留下的经验**：

1. **"两个现象发生在同一个大致的时间窗口"不等于"一个导致了另一个，顺序是我猜的那样"**——第一次推翻旧结论靠的就是这一条。
2. **复现实验必须先核对物理条件和原始 bug 发生时是否一致，不能想当然用"现在的"配置去复现"过去的"bug**——这次的转折点正是发现"当前用的夹爪力度是后来才调整的，不是原始 bug 发生时的条件"，这条本该在设计第一轮复现实验时就想到，却是被用户追问逼出来的。
3. **小概率事件需要足够的样本量才能被观测到**——1/50 的概率，用 4 次或 10 次实验都不可能可靠地看到，需要有意识地把样本量提到能覆盖目标概率量级的规模。

#### 10.10.3 第三个 bug：夹爪刚接触 box 瞬间的 `kSlip` 被当成"ready，进入 LIFT"

**现象**：修完前两个 bug，流程能到 `PREPLACE`，但 `PREPLACE → RECOVER`（`exit=SLIPPED`）几乎每次必炸；即使把判据从"瞬时双指接触布尔值"改成"box 高度是否仍在阈值之上"（见下一条 bug）之后，**仍然**偶发失败。

**线索**：把 `kClose → kLift` 的转移时间戳和 box 高度对齐看：

```
CLOSE -> LIFT: elapsed=0.50s exit=REACHED   # 第一次尝试，close_settle_s 还不存在
```

只等了 0.5s（当时用的是通用的 `min_settle_s`）就判定"抓住了，可以进 `kLift`"。但 `classifyGrasp()` 报 `kSlip`（宽度+双指接触已经"看起来像抓住了"）的那一刻，伺服可能才刚刚开始收紧——`onGripperCommand` 写的是目标 ctrl，位置伺服达到这个目标需要真实时间，不是一个 tick 就到。半秒钟的夹持力还远没到稳态，接下来立刻抬起、再立刻做 `kPreplace` 需要的关节1旋转（一个侧向摆动），marginal 的夹持力扛不住这个额外的横向扰动。

**修复**：给 `kClose` 单独配一条比 `min_settle_s` 大一个量级的 `close_settle_s`（2.0s）——`kClose`不再是"读到 kSlip/kSuccess 立刻走"，是"读到 kSlip/kSuccess **并且**已经在这个阶段停留够久"才走：

```cpp
if ((outcome == GraspOutcome::kSuccess || outcome == GraspOutcome::kSlip) &&
  in.elapsed_in_phase_s >= params.close_settle_s)
```

**这个修复本身还不够**，配合它一起解决问题的是下面这条独立的 bug（10.9.4）——两个问题表面症状相同（都在 `PREPLACE`/`kLift` 之后掉落），根因却完全不同，是排查中第二次撞见"表面相似、根因独立"的情况（第一次是 [8.8](#88-排查记录keyframe-名字冲突与长度不匹配是两件独立的事结论被推翻) 的 keyframe 名字冲突 vs 长度不匹配）。

#### 10.10.4 第四个（独立）问题：`kPreplace`/`kPlace` 的掉落判据看瞬时接触布尔值，被接触检测噪声误判

**现象**：即使加了 `close_settle_s`，`PREPLACE → RECOVER (SLIPPED)` 仍然频繁出现，且时间点很早（`elapsed≈0.1~0.25s`，远早于任何超时）。

**线索**：这不是新现象——[9.1](#91-改动清单与验证结果) 早就记录过（当时是在讲抓取判据，不是讲 FSM）：`classifyGrasp` 在一次完全稳定的持握过程中，日志显示 `SUCCESS`/`UNEXPECTED_CONTACT` 逐 tick 交替，因为 `bodiesInContact()`（[9.4](#94-mjcontact-与-bodiesincontact接触检测怎么工作)）每个物理步都从零重新生成接触列表，单指的接触检测本身就有逐 tick 的抖动，不代表真的松开过。`fsm.cpp` 最初的 `kPreplace`/`kPlace` 判据：

```cpp
if (!bothFingersHolding(in.grasp_signals)) {
  return {Phase::kRecover, ExitReason::kSlipped, false};
}
```

直接拿这个逐 tick 会抖动的布尔值当"掉了没掉"的判据，等于把接触检测的噪声原样透传成了假摔判定——box 明明还端端正正抬在半空，只因为这一 tick `right_finger_contact` 恰好读到 `false`，就立刻宣判失败。

**修复**：改用连续量 `box_height_m`（是否仍在 `lift_height_threshold_m` 之上），而不是瞬时布尔值：

```cpp
if (in.grasp_signals.box_height_m < params.grasp_criteria.lift_height_threshold_m) {
  return {Phase::kRecover, ExitReason::kSlipped, false};
}
```

高度是积分出来的连续量，不会像逐 tick 重新生成的接触列表那样有相同幅度的高频噪声——真摔的时候，高度会持续跌落，不是单 tick 闪一下。

**留下的经验**：这是 [9.4](#94-mjcontact-与-bodiesincontact接触检测怎么工作) 早就写明的性质（"`mjContact` 每步重新生成，不是持久状态"）在下游被忽视的一次具体案例——知道一个信号有噪声，和在设计判据时真的把这条知识用上，是两件事。`classifyGrasp()` 内部对同一个噪声源是稳健的（它同时看宽度+双指接触+位置三个信号，噪声只在其中一维出现时不会翻转整体结论），但 `task_executor` 这里绕过 `classifyGrasp()` 单独看接触布尔值时，重新引入了同一个坑。

#### 10.10.5 一次独立的物理调参：夹爪闭合力度不够，撑不住 `kPreplace` 的侧向摆动

**现象**：修完上述四个逻辑 bug 后，`PREPLACE → RECOVER` 仍偶发（不是必然）——`box_height_m` 判据显示 box 是**真的**掉了，不是误判。

**排查**：手动复现 `HOME→PREGRASP→GRASP→CLOSE→LIFT→PREPLACE` 这条路径，逐步改变夹爪闭合命令的 `position` 字段对比：命令 `position=0.03`（比 box 宽度 0.04m 窄 1cm，"narrower than the box"的常规写法）能撑住纯垂直的 `kLift`，但撑不住 `kPreplace` 需要的 `joint1` 旋转（一次侧向摆动）——实测看到接触从双指变成单指、box 绕着剩下的接触点转出去，最后跌回桌面。命令 `position=0.0`（伺服朝着"完全闭合"尽力去推，被 box 挡住后停在比 0.03 更紧的挤压力度）能稳定撑过同一段侧向摆动，一路验证到 `kPlace`→松开→`box` 落在 `(0.42, 0.31)`附近（正是 `verify.place_x_m/place_y_m` 期望的区域）。

**结论写进** [KeyframeWaypointSource](../../src/task_executor/include/task_executor/keyframe_waypoint_source.hpp) 的 `kClosedWidthM`：由 `0.03` 改成 `0.0`。这不是逻辑 bug，是一次纯物理调参——"narrower than the box" 这个写法本身没错（[9.2](#92-position-servo-下夹住为什么是稳态位置误差不是力) 讲过的稳态误差机制依然成立），只是 1cm 的挤压余量在承受横向扰动时不够。

**留下的经验**：这条呼应 [6.1](#61-清单) 里"`lift_height_threshold_m`/`region_radius_m` 目前是猜的，缺一次真正抬起来的轨迹作对照"——这次终于有了那条轨迹（`box_z` 从 0.24 稳定升到 0.40 再降到 0.24 落地），阈值本身（`0.26`）在这次实测里够用，没有必要跟着改；改的是**产生这条轨迹所需的输入**（夹爪力度），不是判据本身。

### 10.11 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现/防住 |
|---|---|---|
| `kRecover→kHome` 的重定向被判据误认成"没有转移" | FSM 卡在 `RECOVER`，进程正常但永远不重试，没有任何报错 | 真实联调才暴露；已改用 `next_phase == phase_` 判断是否发生转移，不看 `exit_reason` |
| `kLift` 一到位就问 `classifyGrasp()`，接触检测的逐 tick 抖动恰好撞上就误判摔落 | `armReached()` 一到位立刻判 `kSlipped`，即使抓取本身正常。确认根因（[10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论)）：50 次实验里 1 次实测撞见——`armReached()` 首次持续变真的精确 tick，恰好落进一次约 10ms 的单指接触假读数窗口 | `lift_settle_grace_s`（2s）+ `test_fsm.LiftGivesTheBoxTimeToCatchUpBeforeCallingItSlipped`（测试名字保留了旧叙述，行为断言仍然有效——见该测试小节的说明） |
| `kClose` 一读到 `kSlip`/`kSuccess` 立刻进 `kLift`，夹持力还没收紧到位 | 半秒的夹持力扛不住紧接着的抬起+侧摆 | `close_settle_s`（2s）+ `test_fsm.CloseDoesNotAdvanceOnSlipBeforeCloseSettleS` |
| `kPreplace`/`kPlace` 用瞬时双指接触布尔值判掉落 | 接触检测本身逐 tick 抖动（[9.4](#94-mjcontact-与-bodiesincontact接触检测怎么工作) 早记录过），被误判成真摔 | 改用连续量 `box_height_m` + `test_fsm.PreplaceToleratesMomentaryFingerContactFlickerWhileStillHeldAloft` |
| 夹爪闭合力度不够，撑不住 `kPreplace` 的侧向摆动 | 纯垂直 `kLift` 能撑住，加一次旋转就掉 | 手动逐步复现找到阈值（[10.10.5](#10105-一次独立的物理调参夹爪闭合力度不够撑不住-kpreplace-的侧向摆动)），`kClosedWidthM` 改成完全闭合命令 |
| `RECOVER` 耗尽重试次数后是否真的会停 | 若卡死或假装成功，20 次连跑（Stage J）会产出没有意义的数字 | 已用人为不可达的 `verify.place_x_m/y_m` 制造 3 次必然失败，确认 `RECOVER→HOME` 重试 3 次后正确落到 `FAILED` 并停止发布命令（[10.1](#101-改动清单与验证结果) 末尾） |
| 两个独立发布者同时写 `~/gripper_command` | 90ms 内 width 从 0.0434m 跳到 0.0297m，box 从半空跌落，无任何冲突日志 | 实测复现（[10.9](#109-实测复现gripper_command-双发布者冲突)）；`GripperCommand` 无 header，订阅端无法分辨来源，只能靠人工纪律"两者不同时跑" |

### 10.12 你没问但值得注意的

按 [STUDY_NOTES_GUIDE 第5节](../../STUDY_NOTES_GUIDE.md) 的固定职责，这几条本 stage 尚未讨论：

1. **`close_settle_s`/`lift_settle_grace_s` 这两个 2 秒的数字是怎么定的？**（D 类，缺参照系，先记不深挖）——目前是"改到实测稳定通过为止"，不是从物理量（伺服带宽、box 质量/摩擦系数）推出来的。一旦第3周把 `KeyframeWaypointSource` 换成 IK 驱动的连续轨迹（不再是"发一个目标、等着收敛"这种离散跳变），这两个常数的物理含义会整体改变，现在的数值不能直接照搬过去。
2. **`box_height_m < lift_height_threshold_m` 这条判据本身也是瞬时读数，为什么就不会像接触布尔值一样抖动？**（E/C 类，值得追问但本 stage 没有专门验证）——理由在 [10.10.4](#10104-第四个独立问题kpreplacekplace-的掉落判据看瞬时接触布尔值被接触检测噪声误判) 里给了直觉解释（连续量 vs 逐 tick 重新生成的离散量），但没有像 `frame_math`/`state_ops` 那样写一个 gtest 去量化"这条连续量的物理噪声幅度到底有多大，会不会在更剧烈的运动下也开始抖动到跨过阈值"。
3. **`task_executor_node.cpp` 完全没有单测**（E 类，延续 Stage F 定的纪律）——`fsm.cpp`/`keyframe_waypoint_source.hpp` 该测的都测了，节点胶水本身（读话题、拼 `FsmInputs`、发命令）刻意留白，理由和 [2.1.1](#211-这类系统该怎么测四层本周只取前两层) 第3条一致（"不给 `onTimer` 造 mock"），但这次它比 `mujoco_bridge_node.cpp` 的胶水层更厚——`extractArmState()`/`extractGripperWidth()` 这两个从 `sensor_msgs::msg::JointState` 里按名字找索引的函数，逻辑上已经接近可以剥成纯函数（输入一个 `JointState`，输出 `optional<ArmState>`），目前没有剥，是否值得剥值得下次讨论。
4. **`WaypointSource` 只有一个实现时，这层抽象的"多一层间接"的代价现在体现在哪？**（B 类，权衡类问题，尚未展开）——`task_executor_node.cpp` 每个 tick 都要构造一次 `ObjectPose` 结构体传给 `jointTargetFor()`，即使 `KeyframeWaypointSource` 完全不看它；这是为第3周预留接口付的一点点运行时和代码复杂度成本，值不值得，要等第3周真的换实现时才能回答。

## 11. Stage J：episode 边界重构与 episode runner

> 本节覆盖 Stage J 计划（[2.5](#25-stage-j--episode-runner20-次连跑与第一段-demo)）的全部 5 项：episode runner + 连续20次（11.1~11.6）、调试可视化（[11.7](#117-debug-viewerglfw-原生渲染直连权威-mjdata替代-25-计划里的-rviz-集成)，用 GLFW debug viewer 替代了原计划里的 RViz 网格集成）、launch 层参数暴露 + rosbag 录制（[11.8](#118-launch-层参数暴露与-record_demo_bagsh收尾-stage-j)）。Stage J 到这里收尾。

### 11.0 一句话总结

给 `task_executor_node` 加了 episode 边界的话题接口——`~/start_episode`（订阅）和 `~/episode_outcome`（发布，新包 [manipulation_interfaces](../../src/manipulation_interfaces/) 的 `EpisodeOutcome.msg`），替换掉 Stage I 遗留的"episode 边界=进程边界"假设：节点常驻不重启，`onStartEpisode()` 把 `phase_`/`retry_count_`/阶段日志强制拉回 `kHome` 并触发一次 `~/reset`。新增 [scripts/episode_runner.py](../../scripts/episode_runner.py) 连续跑 N 个 episode，写 CSV 到 `results/`。开工前讨论了"为什么要连续跑20次"（此前一直是悬挂问题），开工中先设计成 service/action 被证伪，改回 topic；跑起来后第一次就撞见一个真实的 DDS 发现竞态（`episode_runner.py` 的 `~/start_episode` 发布早于被 `task_executor_node` 发现），修完后 20/20 episode 连续成功，另用不可达放置目标验证了失败路径正确上报真实失败原因。

### 11.1 改动清单与验证结果

**改动**：

- [manipulation_interfaces](../../src/manipulation_interfaces/)（新包）：`msg/EpisodeOutcome.msg`（`success`/`failure_code`/`retries`/`phase_names[]`/`phase_durations_s[]`）
- [task_executor_node.cpp](../../src/task_executor/src/task_executor_node.cpp)：新增 `~/start_episode`（`std_msgs/Empty`）订阅、`~/episode_outcome` 发布；新增 `onStartEpisode()`、`publishEpisodeOutcome()`；新增 `last_failure_reason_`（区分"真实失败原因"和"重试耗尽"）、`phase_names_log_`/`phase_durations_log_`（每 episode 的阶段轨迹）；`onTimer()` 的空闲判据从"永久终态"改成"等下一条 `~/start_episode`"
- [phase.hpp](../../src/task_executor/include/task_executor/phase.hpp)：更新过期注释——"episode 边界=进程边界"这条 Stage I 的假设已被替换
- [task_executor/CMakeLists.txt](../../src/task_executor/CMakeLists.txt) / [package.xml](../../src/task_executor/package.xml)：新增 `manipulation_interfaces` 依赖
- [scripts/episode_runner.py](../../scripts/episode_runner.py)（新）：连续跑 N 个 episode，写 `results/*.csv`

**编译**（`colcon build --packages-select robot_description mujoco_bridge manipulation_interfaces task_executor --symlink-install`）：

```
Starting >>> manipulation_interfaces
Finished <<< manipulation_interfaces [2.56s]
Starting >>> task_executor
Finished <<< task_executor [9.65s]

Summary: 4 packages finished [12.4s]
```

**测试**（直接跑各 gtest 二进制，确认这次改动没有破坏既有单测）：

```
test_frame_math:               3/3 PASSED
test_state_ops:                6/6 PASSED
test_grasp_criteria:          10/10 PASSED
test_grasp_state:               6/6 PASSED
test_fsm:                      19/19 PASSED
test_keyframe_waypoint_source:  3/3 PASSED
```

lint 部分沿用 [6.2](#62-反向清单现在就该做的) 已记录的 96 处历史债务（`copyright`/`cpplint`/`uncrustify` 三类），本 stage 没有引入新的失败类别。

**行为验证：20 次连续跑**（环境卫生按 [STUDY_NOTES_GUIDE 3.1](../../STUDY_NOTES_GUIDE.md) 确认后，`mujoco_bridge_node`/`task_executor_node` 都直接跑可执行文件，不经 `ros2 run`）：

```
$ /usr/bin/python3 scripts/episode_runner.py --episodes 20 --out results/pick_place_run.csv --timeout 60
...
episode 20: success=True failure_code=NONE retries=0 total_duration_s=6.75
wrote 20 rows to results/pick_place_run.csv
20/20 episodes succeeded (see results/pick_place_run.csv)

real	2m15.180s
```

CSV 摘录（20 行全绿，`total_duration_s` 全部落在 6.71~6.75s，`final_box_{x,y,z}_m` 只在小数点第5~6位有差异——这次实测直接证实了这套系统在固定输入下产出几乎完全相同的轨迹，[11.2](#112-为什么是连续-20-次这个数字真正验证的是什么) 详细讨论这个事实对"20次到底验证了什么"的影响）：

```
episode,seed,success,failure_code,retries,total_duration_s,...,final_box_x_m,final_box_y_m,final_box_z_m
1,0,True,NONE,0,6.714,...,0.43372460994601975,0.3096637670047532,0.23987759686370508
2,0,True,NONE,0,6.748,...,0.43372643534256605,0.30965411773162366,0.23987759686371185
```

**行为验证：强制失败路径**（[10.1](#101-改动清单与验证结果) 用过的同一手法——把 `verify.place_x_m`/`place_y_m` 改成 `(99.0, 99.0)`，缩短 `fsm.phase_timeout_s` 加速复现）：

```
episode 1: success=False failure_code=PLACE_MISSED retries=3 total_duration_s=37.23
```

`phase_names` 列显示完整 4 轮尝试（每轮 `HOME|PREGRASP|...|VERIFY|RECOVER`），`failure_code` 报告的是真实失败原因 `PLACE_MISSED`，不是更表层的"重试耗尽"信息——这正是 `last_failure_reason_` 存在的理由。

**过程中撞见并修复一个真实 bug**（不是编出来验收的，是第一次跑 3-episode smoke test 就实测到）：见 [11.4](#114-排查记录episode_runnerpy-发布-start_episode-时撞上的-dds-发现竞态)。

### 11.2 为什么是连续 20 次：这个数字真正验证的是什么

> Q: 先告诉我，我们为什么需要重复20轮？如果没必要应该砍掉。

**第一层答案**：这是外部交付物的硬性要求——[项目计划书 5.3 第4条](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md)写的是"固定物体位姿下连续成功20次，无非预期碰撞"，不是我们自己定的，不能单方面砍掉。

**但"20次没有信息量"这个质疑本身抓住了真问题**：固定物体位姿 + 固定 waypoint（`KeyframeWaypointSource` 完全忽略 `object_pose`）+ MuJoCo 物理确定性 → 如果把整个系统看成一个纯函数，20次应该输出完全相同的轨迹——这次实测（[11.1](#111-改动清单与验证结果) 的CSV）证实了这个直觉：`total_duration_s` 全部落在 6.71~6.75s，box 终点坐标只在小数点第5~6位不同,20次几乎是同一条轨迹。

**这个直觉在"20次没有信息量"这一步是错的**：`mujoco_bridge` 和 `task_executor` 是**两个独立进程、各自独立的定时器**（bridge 的物理步进定时器 + executor 的20Hz决策定时器），中间靠 DDS 传消息。物理本身确定，但**两个进程的相对调度时序不确定**——[10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论) 已经实测到一个真实案例：同样的夹爪力度、同样的轨迹，接触检测抖动窗口恰好撞上 `armReached()` 变真的那个精确 tick，50次里撞上了1次（约2%概率）。这不是"物理不确定"，是"跨进程/跨定时器的时序抖动"——这恰恰是**只有反复跑很多次才能被观测到**的一类 bug，单次跑或读代码都看不出来。

**所以20次真正验证的是什么**：不是"系统能应对多样场景"（那需要域随机化，计划书本身也推到后面），而是**"这套多进程、DDS 中介的系统，在名义上确定性的输入下，会不会因为进程间时序抖动而产生不该有的失败"**——一种回归/抗抖检测,而且我们已经知道抖动确实存在（2%量级的历史数据）。

**诚实的结论**：20次本身**不是**从"这个概率量级需要多少样本"反推出来的——1/50 的真实概率下,20次只有约33%机会撞见它（[10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论) 那次是靠50次样本才抓到的）。计划书要的20次更像一个"最低限度的冒烟测试"，不是严谨的统计验证。这次实测的20/20全绿，**不能**倒推"那个2%的bug已经不在了"——这次样本量本来就撞不上2%的事件。**结论：20次保留，但定位成"冒烟回归测试"而不是"鲁棒性证明"**——这也是为什么 [11.1](#111-改动清单与验证结果) 的 CSV 记的不只是成功/失败布尔值，还有每个阶段的耗时（`phase_durations_s`），这样"20次里有没有出现过一次异常耗时/异常路径"本身就是有信息量的观察，不是单纯验收打勾。

### 11.3 为什么用 topic 而不是 service/action 做 episode 边界：与 `requestReset()` 是同一种死锁机制

> Q: 有一点想问的是：为什么不用service或者action？

**先说 service 行不通的直接原因**——和 [10.3.4](#1034-requestreset-是节点调-service-的一般写法吗和命令行-ros2-service-call-有什么关系) 讲 `requestReset()` 时的死锁陷阱是**同一个坑**：如果 service handler 想同步跑完一整个 episode（~7秒）才返回，它必须在自己的回调内部继续等待仿真进展，但仿真进展本身要靠同一个单线程 executor 继续 spin 才能推进——回调卡在这里，executor 就动不了，死锁。所以哪怕换成 service，handler 也只能"收到就立刻返回一个'已接受'，真正完成还是要另开一个通道汇报"——这意味着 service 版本本质上还是要拆成"触发"+"结果"两条通道，跟现在的 topic+topic 结构没有本质区别，只是多了一次"确认收到"的往返。

> Q: 我不是特别理解你对service的解释。我想象中：episode_runner呼叫阻塞等待结果，然后task_executor_node执行，执行完成后回复service，episode_runner开始下一个循环继续呼叫。指出我的错误。

client 端阻塞等结果这件事本身没有问题——[10.3.4](#1034-requestreset-是节点调-service-的一般写法吗和命令行-ros2-service-call-有什么关系) 那张表里"一次性脚本，阻塞等结果没有代价"说的就是这种场景。**问题出在"task_executor_node 执行"这四个字，具体是谁在执行、在哪个线程上执行**：

`task_executor_node` 内部没有一个独立于 ROS 的"执行引擎"在后台自己跑完整个 episode。让 FSM 真正往前走的是 `onTimer()`——这个20Hz的定时器回调和节点里**其它所有回调**（包括假想中的 service handler）默认共享**同一个线程**（单线程 executor）。如果 service handler 写成"收到请求 → 一直等到 `phase_` 变成 `kDone`/`kFailed` → 才返回"：

- handler 一旦开始等，就**占着这个线程不放**，直到它自己返回。
- 但"`phase_` 什么时候变成 `kDone`"这件事，只有 `onTimer()` 被调用才会发生。
- `onTimer()` 和这个 service handler 是**同一线程上的两个不同回调**——executor 不可能一边在跑 handler（还没返回），一边又去跑 `onTimer()`（handler 还没让出线程）。

结果是 handler 在等 `onTimer()` 把 `phase_` 变成终态，`onTimer()` 却永远排不上号——因为线程被 handler 自己占着。这是死锁，不是"变慢"，这个 service 调用永远不会返回。这正是 `requestReset()` 那个坑的**同一个机制**，只是时间尺度从"等一次 reset 响应"放大到"等一整个 episode（~7秒、上百次 tick）"。

**要让"呼叫—执行—回复"这个想法真的成立，节点必须换成多线程 executor**：把 service 回调和 `onTimer()` 分到不同的 callback group/线程，handler 线程用条件变量或 `std::promise`/`future` 等着 `onTimer()` 线程在某次 tick 里把 episode 跑完后通知它。这是可以做的，但要新引入"两个线程共享 `phase_`/`retry_count_` 等状态，需要加锁或原子量"这类真实的并发复杂度——topic 版本（`onTimer()` 自己异步地、不阻塞任何人地跑完episode,跑完后**主动**发一条 `~/episode_outcome`）刚好绕开了这整个问题,因为它从不要求"有一个回调一直等到别的回调把活干完"。

**action 会比 topic+topic 更"正确"，但目前性价比不够**：`~/start_episode`+`~/episode_outcome` 本质上是在用两条独立话题手搭一个"目标—结果"配对，这正是 action 的标准形状（goal/result，`EpisodeOutcome` 可以直接做 result，不用单开一条话题），而且 action 会**免费**提供两个当前缺失的能力：① 拒绝并发目标（第二个 goal 进来、上一个还没完成时可以直接拒绝或抢占，不会像现在这样静默覆盖）；② feedback（阶段转移可以实时推给调用方）。这次的设计选择和 [9.3](#93-为什么不直接用-grippercommand-action)（为什么不直接用 `GripperCommand` action）是同一个判断逻辑——只有一个消费者、不需要 cancel/feedback，做 action server 的复杂度现在没人驱动。但这次的性价比没那么干净：action 会顺手补上一个真实存在的缺口——如果 `~/start_episode` 在上一个 episode 还没到终态时又被调用一次，`onStartEpisode()` 会无条件把 `phase_` 拽回 `kHome`，正在进行的那次尝试被静默覆盖，没有任何警告。当前 `episode_runner.py` 严格串行不会触发这条路径，这个缺口已经记进 [6.2 反向清单](#62-反向清单现在就该做的)，没有在这次决定修。

### 11.4 排查记录：`episode_runner.py` 发布 `~/start_episode` 时撞上的 DDS 发现竞态

**现象**：第一次跑 3-episode smoke test，episode 1 直接超时（60秒），`task_executor_node` 的日志里**完全没有**对应的 "episode start requested" 那一行——不是"处理了但没反应"，是这条消息从没被这个节点看到过。episode 2 的 `~/start_episode` 正常触发,日志从这里才第一次出现。

**根因**：`episode_runner.py` 的 publisher 在 DDS 还没发现 `task_executor_node` 那个订阅者之前就发了第一条消息，ROS2 的话题（这里没配置任何 durability）默认不给未发现的订阅者补发消息——这和 [10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论) 记录过的"新进程 `~/reset` 客户端抢跑 DDS 发现"是**同一类**竞态，只是这次发生在一个新写的发布者/订阅者对上，而不是 client/service 对上——[6.1](#61-清单) 那条悬挂项预测的是"进程重启"这个具体触发条件，这次实际触发条件是"新建一条 pub/sub 连接"，说明这条经验比原来写的更通用。

**修复**：`episode_runner.py` 加一个等价于 `wait_for_service()` 的手写等待——`rclpy.spin_once` 轮询 `publisher.get_subscription_count() > 0`，带15秒超时。这不是巧合：[11.3](#113-为什么用-topic-而不是-serviceaction-做-episode-边界与-requestreset-是同一种死锁机制) 已经指出 service 客户端自带 `service_is_ready()`/`wait_for_service()` 专门解决这个问题，topic 的 publisher 没有对应的现成机制，只能自己手写一个更简陋的等价物。

**验证**：修完后重跑同样的 3-episode smoke test，3/3 成功；随后跑满 20 episode，20/20 成功（[11.1](#111-改动清单与验证结果)）。

**留下的经验**：这条竞态第三次出现了（第一次是 Stage I 的 `~/reset` 客户端 vs `mujoco_bridge`，[10.10.2](#10102-第二个-bug接触检测闪烁撞上到位检查根因已查清含一次被推翻的旧结论) 排查中意外撞见；第二次是这次），说明"任何两个独立 ROS2 进程之间新建的第一条通信连接，都要假设对方还没被发现"这条规律不是某个具体场景的特例，是这套多进程系统的通用性质——以后再加新的 pub/sub 边（比如 Stage J 剩下的 `robot_state_publisher`/rosbag），应该默认先检查一次这个问题，而不是等它自己炸出来。

### 11.5 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现/防住 |
|---|---|---|
| `~/start_episode` 发布早于订阅被 DDS 发现 | 第一条消息静默丢弃，episode 超时，server 端无任何对应日志 | 实测撞见（[11.4](#114-排查记录episode_runnerpy-发布-start_episode-时撞上的-dds-发现竞态)）；修法是手写等价于 `wait_for_service()` 的订阅计数轮询，已用 3/3 与 20/20 验证 |
| 用 service 做"阻塞到完成"的 episode 边界 | 未实现（设计阶段被证伪）——单线程 executor 下，service handler 和 `onTimer()` 抢同一线程，handler 等 `onTimer()` 推进 phase，`onTimer()` 却因 handler 未让出线程而排不上号，死锁 | 与 `requestReset()`（[10.3.4](#1034-requestreset-是节点调-service-的一般写法吗和命令行-ros2-service-call-有什么关系)）同一机制，选择前先用这条已知教训排除 |
| `~/start_episode` 在上一个 episode 未到终态时被再次调用 | `onStartEpisode()` 无条件把 `phase_` 拽回 `kHome`，正在进行的尝试被静默覆盖，无警告 | 当前 `episode_runner.py` 严格串行不触发；已知缺口记入 [6.2](#62-反向清单现在就该做的)，决定先不修 |
| 20 次连续成功被当成鲁棒性证明 | 固定输入下 20 次几乎是同一条轨迹，容易误读成"系统很稳" | 已定位成冒烟回归测试而非鲁棒性证明（[11.2](#112-为什么是连续-20-次这个数字真正验证的是什么)）；CSV 记录每阶段耗时而不只是成功/失败布尔值，为将来对比留信息 |

### 11.6 你没问但值得注意的

1. **`last_failure_reason_` 的赋值逻辑没有剥成纯函数**（E 类）——"只在转移进 `kRecover` 时才更新这个字段"这条判断目前直接写在 `onTimer()` 里，逻辑很简单（一个 if），但和 Stage F/I 定的"决策逻辑该剥出去"的纪律不完全一致；值不值得剥、剥出去能不能用手搭结构体测,可以讨论。
2. **`~/start_episode` 中途重复调用的静默覆盖缺口，本质上是 topic 相对 action 缺失的"并发目标拒绝"能力**（C 类，已记入 [6.2](#62-反向清单现在就该做的)）——[11.3](#113-为什么用-topic-而不是-serviceaction-做-episode-边界与-requestreset-是同一种死锁机制) 已经讨论过这是选择的代价而不是实现疏漏，但值得在这里再点一次：这类"选了更简单的接口，隐性放弃了某个具体保护"的取舍，容易在几周后被忘记选择时权衡过什么。
3. **`episode_runner.py` 本身完全没有测试**（E 类，延续 Stage F/I 定的纪律）——它是纯粹的胶水脚本（发消息、等结果、写CSV），按 [2.1.1](#211-这类系统该怎么测四层本周只取前两层) 的四层判据属于"任务层"，不该进 `colcon test`，但目前也没有任何脚本级的自检（比如"CSV 行数应该等于 episodes 参数"这种断言）。

### 11.7 Debug viewer：GLFW 原生渲染，直连权威 `mjData`（替代 2.5 计划里的 RViz 集成）

覆盖 [2.5](#25-stage-j--episode-runner20-次连跑与第一段-demo) 计划里原本的第③项（RViz `robot_state_publisher` 集成）。这部分先按计划实现过一次 RViz 网格显示，之后又被回退，过程和原因见 [11.7.3](#1173-概念讲解可视化架构选型rviz--glfw-原生渲染--三层离权威物理源距离梯子)，最终决定换成本节记录的 GLFW debug viewer。

#### 11.7.1 一句话总结

给 `mujoco_bridge_node` 加了一个默认关闭的调试专用可视化窗口——`DebugViewer`（[debug_viewer.hpp](../../src/mujoco_bridge/include/mujoco_bridge/debug_viewer.hpp)/[.cpp](../../src/mujoco_bridge/src/debug_viewer.cpp)），走 MuJoCo 自带的 `mjv_*`/`mjr_*` 渲染管线 + GLFW 建窗，同进程、同线程、直接读节点自己正在步进的那份权威 `mjData`，不经过任何 ROS 话题或降采样。`enable_debug_viewer` 参数控制开关（默认 `false`），渲染频率用独立的 `debug_viewer_rate_hz`（默认 30Hz）通过既有的 `decimationFor()` 降采样，不跟物理步进频率绑定，也不用 vsync 卡住物理线程。这条路径替代了 [2.5](#25-stage-j--episode-runner20-次连跑与第一段-demo) 原计划里"RViz 网格显示"那部分——RViz 版本先做过一次又被回退，原因和过程见 [11.7.3](#1173-概念讲解可视化架构选型rviz--glfw-原生渲染--三层离权威物理源距离梯子)。过程中撞见一个真实的环境级故障——这个容器里硬件加速 GL 上下文创建会卡死，绕过手段和排查过程见 [11.7.4](#1174-排查记录glfw-创建-gl-上下文在-distrobox-里卡死)。

#### 11.7.2 改动清单与验证结果

**改动**：

- [mujoco_dl.hpp](../../src/mujoco_bridge/include/mujoco_bridge/mujoco_dl.hpp)：`MujocoApi` 新增 11 个 `mjv_*`/`mjr_*` 字段（`defaultCamera`/`defaultOption`/`defaultScene`/`defaultContext`/`makeScene`/`makeContext`/`updateScene`/`render`/`freeScene`/`freeContext`/`moveCamera`），同既有的 `decltype(&mj_xxx)` 写法
- [mujoco_dl.cpp](../../src/mujoco_bridge/src/mujoco_dl.cpp)：对应的 11 行 `resolve()` 调用，走同一个已有的 `dlopen(RTLD_LOCAL|RTLD_DEEPBIND)` handle——渲染符号和物理符号在同一个 `libmujoco.so` 里，不需要第二次 `dlopen`
- [debug_viewer.hpp](../../src/mujoco_bridge/include/mujoco_bridge/debug_viewer.hpp)/[debug_viewer.cpp](../../src/mujoco_bridge/src/debug_viewer.cpp)（新）：`DebugViewer` 类，封装 GLFW 窗口 + `mjvCamera`/`mjvOption`/`mjvScene`/`mjrContext`；鼠标拖拽相机旋转/平移/缩放（改自 MuJoCo 官方 `simulate` 示例的全局回调写法，换成 `glfwSetWindowUserPointer` 挂在实例上，不引入新的全局状态）
- [mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp)：`decimationFor()` 加一个可选的 `default_rate_hz` 参数（不改两个既有调用点的行为）；构造函数里 `declare_parameter("enable_debug_viewer", false)` + 条件构造 `viewer_`；析构函数里 `viewer_.reset()` 排在 `deleteData`/`deleteModel` 之前；`onTimer()` 里物理步进和既有的 `/joint_states`/`/tf` 发布之后，加 `pollEvents()`/`shouldClose()`/降采样 `render()`
- [CMakeLists.txt](../../src/mujoco_bridge/CMakeLists.txt)：`find_package(glfw3 REQUIRED)`；`debug_viewer.cpp` 加入 `mujoco_bridge_node` 的源文件；`target_link_libraries` 加 `glfw`——这里和 `mujoco::mujoco` 不一样，`glfw3` 不带冲突的 `tinyxml2`，没有必须 dlopen 隔离的理由，直接链接

**编译**（`colcon build --packages-select mujoco_bridge --symlink-install`）：

```
Starting >>> mujoco_bridge
Finished <<< mujoco_bridge [8.52s]
```

**测试**（`colcon test --packages-select mujoco_bridge`，确认扩展 `MujocoApi` 没有破坏既有单测——这四个 gtest 都会调 `loadMujocoApi()`，是"新符号能不能被 `dlsym` 解析到"这件事的真实回归检查）：

```
FrameMath:       3/3 PASSED
StateOps:        6/6 PASSED
ClassifyGrasp:  10/10 PASSED
GraspState:      6/6 PASSED
```

lint 部分沿用既有历史债务，没有新增失败类别。

**行为验证**（环境卫生按 [STUDY_NOTES_GUIDE 3.1](../../STUDY_NOTES_GUIDE.md) 确认，直接跑可执行文件，不经 `ros2 run`）：

| 场景 | 参数 | 观察到的 RTF | 备注 |
|---|---|---|---|
| 默认（关闭） | `enable_debug_viewer:=false` | 稳定 1.00 | 日志里没有任何 `mjv_*`/`mjr_*`/"Debug viewer enabled" 相关行——确认关闭时零影响 |
| 开启，30Hz（默认），软件渲染 | `enable_debug_viewer:=true`（硬件加速卡死，见 [11.7.4](#1174-排查记录glfw-创建-gl-上下文在-distrobox-里卡死)，加了 `LIBGL_ALWAYS_SOFTWARE=1`） | ~0.33 | 窗口真的弹出（`xwininfo` 确认标题 "mujoco_bridge debug viewer"，1200x900），鼠标拖拽能转相机 |
| 开启，2Hz | 同上 + `debug_viewer_rate_hz:=2.0` | ~0.87~0.94 | 降频后 RTF 回升但没回到 1.00——软件光栅化本身有开销，不是纯粹被 30Hz 拖慢 |

#### 11.7.3 概念讲解：可视化架构选型——RViz / GLFW 原生渲染 / 三层"离权威物理源距离"梯子

> Q: 我不理解，rviz太难看，这套教程的路线更加标准不是吗？你说GLFW原生渲染要转发，但是distrobox不是自带有转发吗？我手头有gpu，不能加快渲染吗？

三点分开答：① distrobox 确实默认转发 X11/Wayland，之前说"要转发更麻烦"是把"调试工具本身没挂桌面"和"distrobox 本身有没有转发能力"搞混了，是判断错误，纠正掉；② GPU 是真实优势，`mjr_render` 走 OpenGL，有硬件加速确实能撑更好的画质/帧率，但"RViz 丑"当时的根因是场景没搭好（只有 Grid+TF+RobotModel，没调光照材质），不是渲染管线算力不够，加 GPU 不解决"素"这个问题；③"更标准"要看标准的是哪一层——纯仿真研究领域 `mjv`/`mjr` + GLFW 是标准；但 ROS 机器人栈领域 RViz + TF + `robot_state_publisher` 才是标准，理由不是好看不好看，是它跟仿真器无关，只订阅 `/joint_states`/`/tf`，换成真机发布的同名话题不用改。这也是这个项目反复强调的 sim2real 可迁移目标的落点。

> Q: 你说我们代码实现的是一个通用backend，那GLFW路线难道不能作为一个解耦的前端吗？我真正好奇的是，它的耦合性到底高不高？我能不能像起一个node那么简单，不对原来代码有多余干扰的情况下实现？

耦合性低，具体靠 [mujoco_dl.hpp](../../src/mujoco_bridge/include/mujoco_bridge/mujoco_dl.hpp) 现成的 dlopen 隔离模式：`mujoco_dl.cpp` 是作为**源文件**编进每个要用 mujoco 的可执行文件的（四个既有 gtest target 各自把它加进自己的 sources），不是一个共享库——这个模式天生就是为"多个独立可执行文件各自安全用 mujoco"设计的。最初讨论时想的是"另起一个独立节点，走 `/joint_states`/`/tf` 做运动学镜像"（跟 `robot_state_publisher` 同构，只是用 MuJoCo 几何体画），这个方案确实可以完全不改 `mujoco_bridge_node`；但后来发现这本质上还是"离权威物理源"梯子的第三层（见下），达不到"直连权威 `mjData`"这个真正目标，所以最终选择直接改 `mujoco_bridge_node` 本身——但改动被限定在"新增一个默认关闭的可选分支"（`enable_debug_viewer` 参数 + 条件构造 `viewer_`），没有改变任何既有话题/参数/行为，这是这次追求的"耦合低"的具体含义：不是"物理上不碰这个文件"，是"逻辑上默认路径零变化"。

> Q: 讲讲和权威物理源仿真的区别，实践中通常使用哪种？

三层梯子，不是两层：① 同进程直连（这次选的）——渲染器和物理同一个循环，直接读同一份 `mjData`，MuJoCo 自己的 `simulate` 就是这样；② 同一仿真器的原生高频通道，跨进程——经典 Gazebo 的 `gzserver`/`gzclient`，通信走仿真器自己的高频 transport；③ 通用机器人消息镜像——RViz 订阅 `/joint_states`+`/tf` 重算 FK，最松耦合、最仿真器无关，代价是只能看到已经被显式发布、且被降采样过的量。实践里纯仿真调试阶段用①（要看清物理引擎到底算了什么，不能有第三方来源的抽样误差干扰判断）；面向要跟真机共用的生产/集成阶段用③（价值就在于跟仿真器解耦）。这次的决定是两条路径分开用，不是二选一：debug viewer 是①，专门给"调物理本身"用；[mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp) 现有的 `/joint_states`/`/tf` 发布保持③，专门给"以后要接真机"这个目标用。

> Q: 你的意思是，真机也是类似第三种吗？

对，而且是结构性必然，不是碰巧一样。第一层（同进程直连 `mjData`）对真机根本没有对应物——真实世界不是一个可查询的数据结构，唯一能拿到的通道就是驱动/控制器按自己的循环频率往外发布的关节状态，这跟 `onTimer()` 里 `decimationFor()` 降采样发布 `/joint_states` 是同一个结构：真机侧"抽样源"是编码器采样+驱动发布周期，仿真侧是这次的降采样参数。所以"RViz/TF 看到的延迟、两次发布之间看不到的东西"不是仿真特有的缺陷，是真机这条路径本来就有的、甩不掉的天花板——用第三层作为生产可视化路径，等于提前让开发过程适应真机最终会有的可视化保真度上限。

#### 11.7.4 排查记录：GLFW 创建 GL 上下文在 distrobox 里卡死

**现象**：`enable_debug_viewer:=true` 跑起来后，日志停在"debug viewer every N steps"那一行，`RCLCPP_INFO("Debug viewer enabled ...")` 那行日志永远不出现，没有任何窗口弹出；进程 CPU 几乎为零，`ps` 显示状态 `S`（sleeping）。终端里连按三次 Ctrl+C 都不能让它退出，`rclcpp` 的 `signal_handler(SIGINT/SIGTERM)` 日志确实打出来了，但进程仍然不退。

**线索**（逐步缩小范围）：

1. 查 `/proc/<pid>/wchan`，显示 `do_poll`——卡在一个阻塞的系统调用里，不是"没收到信号"，是"收到了但主线程没机会检查"：GL 上下文创建这一步还没返回，永远回不到 `rclcpp::spin()` 的事件循环，signal handler 设的标志位没人读。
2. 写一个不含 MuJoCo/`rclcpp` 的最小复现程序（只有 `glfwInit`→`glfwCreateWindow`→`glfwMakeContextCurrent`→渲染循环），同样卡死在 `glfwInit` 之后——把问题范围从"这次新代码"缩小到"纯 GLFW 建窗+建 GL 上下文，在这个环境里本身就是坏的"。
3. 加 `glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API)` 跳过 GL 上下文创建，只测纯窗口——立即成功返回。把问题精确定位到"创建 OpenGL 上下文"这一步，不是窗口/X11 连接本身。
4. 设 `LIBGL_ALWAYS_SOFTWARE=1` 强制走 Mesa 软件渲染路径，同一份代码立刻成功建窗、能跑渲染循环。

**根因**：这个容器（distrobox + 主机 X server 转发）下，NVIDIA 专有驱动的硬件加速 GLX 路径协商失败——具体卡在驱动的哪一步（direct rendering 权限、DRI 设备节点访问、还是嵌套 Xorg 会话本身不支持这条路径）没有查清，只确认了"软件渲染能绕过，硬件加速走不通"这个事实边界。这不是"沙盒工具环境独有"的问题——在真实终端里交互式跑同样卡死，因为两边其实共享同一个 `DISPLAY`/同一个容器。

**后续追查（收尾结论）**：上面这段写下时还没查清具体卡在哪一层，后来继续深挖，锁定了确切分层：① 不设 `__GLX_VENDOR_LIBRARY_NAME` 时，GLVND 的厂商自动探测本身卡死（`strace` 显示卡在对 X 连接 fd 的 `poll`，等一个永远不会来的应答）；② 强制 `__GLX_VENDOR_LIBRARY_NAME=nvidia` 后不再卡死，但改成立即报 `GLXBadFBConfig`——NVIDIA 驱动在这条嵌套 X 会话上拿不到能用的 FBConfig；③ 用 `eglQueryDevicesEXT`/`eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, ...)` 完全绕开 X11/GLX、直连 NVIDIA 的 EGL device 做了一次离屏渲染，`GL_RENDERER` 真实显示 `NVIDIA GeForce RTX 4060 Laptop GPU`、`GL_VERSION 4.6.0 NVIDIA 595.91.07`——GPU 和驱动本身完全正常，问题精确定位在"这条嵌套/转发的 X 会话上，NVIDIA 驱动的 DRI3 路径协商不出可用的 FBConfig"这一点，跟①②的现象互相印证。外部佐证：[TigerVNC#1773](https://github.com/TigerVNC/tigervnc/issues/1773) 上有人用同款 RTX 4060 在 Xvnc（另一种嵌套/虚拟 X 服务器）上撞到几乎一样的现象，且明确记录"强制 `__GLX_VENDOR_LIBRARY_NAME=nvidia` 之后 DRI3 仍然拿不到真正的硬件加速"——说明这是 NVIDIA 专有驱动在嵌套/虚拟 X 会话下的已知限制类别，不是这次这个容器独有的配置错误。

**决定**：真正的硬件加速路径存在，但要走通得装 VirtualGL（拦截 GLX 调用，用真实 GPU 离屏渲染后转发画面给这个 2D X 会话），需要新装软件、且很可能要在 X server 侧配置 VGL 相关扩展/权限，属于会改动系统级配置的操作。权衡下来选择接受软件渲染收尾——debug viewer 本来就是低频调试工具（[11.7.2](#1172-改动清单与验证结果) 实测 2Hz 时 RTF 能回到 ~0.87~0.94，够用），不为这个引入新的系统级依赖和配置风险。VirtualGL 这条路径记在这里，以后如果真的需要更高频率的硬件加速再回头看。

**修复（规避，不是根治）**：运行时加 `LIBGL_ALWAYS_SOFTWARE=1` 环境变量，强制 Mesa 软件光栅化。代价是渲染本身有真实开销（见 [11.7.2](#1172-改动清单与验证结果) 的 RTF 数据）。真正修好 NVIDIA/GLX 配置留作悬挂项。

**留下的经验**：

- "看起来卡死、Ctrl+C 打不动的进程"要先查 `ps`/`wchan` 确认是不是卡在阻塞系统调用里，而不是假设"进程没收到信号"——这类卡死只有 `kill -9` 是出路，等多久都一样，跟 [STUDY_NOTES_GUIDE 3.1](../../STUDY_NOTES_GUIDE.md) 里 `ros2 run` wrapper 假死是完全不同的机制（那个是"信号发对了地方但没人处理"，这个是"信号处理了但没人读标志位"），但表现出来都是"Ctrl+C 没用"，容易被误判成同一类问题。
- 写一个剥离掉当前项目全部依赖的最小复现程序，是分辨"是我的代码 vs 是环境"最快的手段——这次几分钟内就把范围从"MuJoCo+rclcpp+GLFW 一起用出问题"缩小到"纯 GLFW 创建 GL 上下文在这个环境里就是坏的"。
- "有 GPU"不等于"硬件加速一定能用"——这条值得明写，因为最初的直觉（"手头有 GPU，不能加快渲染吗"）默认假设了这一点，而这次实测的故障恰恰发生在"调用硬件加速"这一步本身。

#### 11.7.5 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现/防住 |
|---|---|---|
| distrobox 里硬件加速 GL 上下文创建卡死 | `glfwCreateWindow`/上下文创建永久不返回，构造函数卡住，`rclcpp::spin()` 进不去，`/joint_states`/`/tf` 等生产话题跟着一起停摆；Ctrl+C 不能退出 | 最小复现程序缩小范围；`LIBGL_ALWAYS_SOFTWARE=1` 规避；见 [11.7.4](#1174-排查记录glfw-创建-gl-上下文在-distrobox-里卡死) |
| 软件渲染拖慢物理步进 | RTF 从 1.00 掉到 0.33（30Hz）/ 0.87~0.94（2Hz），不报任何错，只是仿真变慢 | 既有的 RTF 日志（Stage F 为"变慢不报错"这类问题装的监控，这次直接派上用场）；没有主动告警，只能靠人盯日志 |
| `enable_debug_viewer` 构造阶段的故障会拖死整个节点 | 不止调试功能受影响，生产话题一起停——跟"默认关闭时零影响、开启时只影响调试路径"的设计预期不完全一致 | 目前没有防住，只是认识到了这个边界，见 [11.7.6](#1176-你没问但值得注意的) 第1条 |

#### 11.7.6 你没问但值得注意的

1. **（C类）构造阶段卡死会拖死整个节点，不只是"debug 视图卡住"**——`viewer_` 是在节点构造函数里**同步**构造的，这次实测的卡死模式下 `rclcpp::spin()` 根本进不去，`/joint_states`、`/tf`、ground-truth 话题这些生产路径会跟着一起停摆。跟"debug 工具默认关闭时零影响、开启时只影响调试路径"这个设计预期有一个没兑现的边界情况。
2. **（C类）RTF 掉线目前只能靠人盯日志发现**——开着 debug viewer 时物理会变慢，但没有任何主动告警；如果谁开着它跑 `episode_runner.py` 那种要比较耗时数据的批量实验，数据会被污染却毫无提示。
3. **（E类）唯一的验证手段是人眼看窗口**——这在"渲染到真实窗口没法脱离显示器自动化"这个约束下是合理的，但这次崩溃模式恰恰是"卡死不报错"而不是"干净失败"，人眼验证本身在这种模式下也容易被误判成"电脑卡了"而不是代码问题；现有 4 个 gtest 只验证了 `dlsym` 层，没有任何调用路径覆盖到 `render()` 本身。
4. **（F类）`LIBGL_ALWAYS_SOFTWARE=1` 的适用边界没测过**——只有 30Hz 和 2Hz 两个数据点，如果以后想用更高频率看实时接触力这类调试场景，软件渲染开销会不会变得不可接受，不知道。

### 11.8 launch 层参数暴露与 `record_demo_bag.sh`（收尾 Stage J）

覆盖 [2.5](#25-stage-j--episode-runner20-次连跑与第一段-demo) 计划里最后剩下的三项：④ launch 层暴露节点参数（week1.md [13.2](../../STUDY_NOTES_GUIDE.md) 反向清单末条），⑤ 录一段 rosbag，以及③里"要不要新建 `pick_place_demo.launch.py`"这个文件命名问题。Stage J 到这里全部收尾。

#### 11.8.1 一句话总结

在 [demo.launch.py](../../src/mujoco_bridge/launch/demo.launch.py) 里给 `bridge_node` 加了四个 `DeclareLaunchArgument`（`joint_state_rate_hz`/`tf_rate_hz`/`enable_debug_viewer`/`debug_viewer_rate_hz`，默认值跟节点自己 `declare_parameter` 的默认值逐一对齐，不传参数就是今天的行为），不新建 `pick_place_demo.launch.py`——`demo.launch.py` 已经在做同一张图（bridge+executor+rviz，默认模型就是 `pick_place_scene.xml`），新建一个同名文件只是重复。新增 [scripts/record_demo_bag.sh](../../scripts/record_demo_bag.sh)，假设 demo 已经在跑（不负责启动它，理由见 [11.8.3](#1183-概念讲解为什么不新建-pick_place_demolaunchpy以及-rosbag-回放里的-sim-time-陷阱)），录制期间跑 N 个 episode，`-a` 录全部话题。过程中实测撞见一个之前没写全的假死变体——`ros2 launch` 和 `ros2 run` 是同一种"单发 `kill -INT` 打不醒 wrapper"的坑，已经补进 [STUDY_NOTES_GUIDE.md 3.1](../../STUDY_NOTES_GUIDE.md)，原来那条只写了 `ros2 run`，范围写窄了。

#### 11.8.2 改动清单与验证结果

**改动**：

- [demo.launch.py](../../src/mujoco_bridge/launch/demo.launch.py)：四个新 `DeclareLaunchArgument`，`bridge_node` 的 `parameters=[...]` 从只有 `use_sim_time` 扩成带四个 `LaunchConfiguration(...)`。
- [scripts/record_demo_bag.sh](../../scripts/record_demo_bag.sh)（新，可执行）：环境卫生检查（`ps -eo pid,comm` 确认 `mujoco_bridge_node`/`task_executor_node` 各恰好一个实例，`ros2 topic info /clock` 确认 `Publisher count: 1`，双重权威判据，仿照 [STUDY_NOTES_GUIDE.md 3.1](../../STUDY_NOTES_GUIDE.md)）→ 后台起 `ros2 bag record -a --use-sim-time -o results/bags/pick_place_<timestamp>`，记下 `$!` → 等 3 秒 → 跑 `episode_runner.py --episodes N` → `kill -INT` 录制进程的真实 PID、`wait` 它退出 → 检查 `metadata.yaml` 是否生成。
- [.gitignore](../../.gitignore)：`results/*.bag`/`results/*.mcap` → `results/bags/`（这两条规则本来就没对过——见 [11.8.2](#1182-改动清单与验证结果) 下面这段实测）。

**实测发现并修正的一个规则错误**：`ros2 bag record` 的默认存储是 `sqlite3`（一个目录，里面是 `<name>_0.db3` + `metadata.yaml`），容器里也没装 `rosbag2_storage_mcap` 插件——原来 `.gitignore` 写的 `results/*.bag`/`results/*.mcap` 从来没匹配过这里实际产出的文件格式。这条不是这次引入的新坑，是写计划时顺手发现的历史遗留错误，按 [STUDY_NOTES_GUIDE.md 2.3](../../STUDY_NOTES_GUIDE.md) 的"结论被推翻时留档"原则记在这里，不是静默改掉。

**编译**（launch 文件是纯 Python，`colcon build --packages-select mujoco_bridge --symlink-install` 只是确认它装得进去，没有 C++ 改动）：

```
Starting >>> mujoco_bridge
Finished <<< mujoco_bridge [0.10s]
```

**行为验证：launch 参数确实到达节点**（这正是 week1 13.2 那条反向清单当初没验证过的事情）：

```
$ ros2 launch mujoco_bridge demo.launch.py \
    joint_state_rate_hz:=50.0 tf_rate_hz:=50.0 \
    enable_debug_viewer:=true debug_viewer_rate_hz:=5.0

[mujoco_bridge_node-1] /joint_states every 10 steps (50.0 Hz requested, 50.0 Hz actual)
[mujoco_bridge_node-1] /tf every 10 steps (50.0 Hz requested, 50.0 Hz actual)
[mujoco_bridge_node-1] debug viewer every 100 steps (5.0 Hz requested, 5.0 Hz actual)
[mujoco_bridge_node-1] Debug viewer enabled (GLFW window)
```

四个覆盖值全部生效，不传参数时（默认值）日志跟改动前逐字节一致（`100.0 Hz`/`false`/无 debug viewer 相关行）。

**行为验证：`record_demo_bag.sh --episodes 3`**（demo 已经跑起来的前提下）：

```
recording to results/bags/pick_place_20260920T060919Z
...
episode 1: success=True failure_code=NONE retries=0 total_duration_s=6.74
episode 2: success=True failure_code=NONE retries=0 total_duration_s=6.75
episode 3: success=True failure_code=NONE retries=0 total_duration_s=6.75
wrote 3 rows to results/bags/pick_place_20260920T060919Z/episodes.csv
stopping recorder (pid 193479)
[rosbag2_recorder]: Recording stopped
done: results/bags/pick_place_20260920T060919Z
```

`ros2 bag info` 确认产物：

```
Duration:          23.510000000s
Messages:          24465
Topic: /clock ... Count: 11757
Topic: /joint_states ... Count: 2351
Topic: /tf ... Count: 2352
Topic: /tf_static ... Count: 1
Topic: /mujoco_bridge/ground_truth/object_pose ... Count: 2351
Topic: /task_executor/episode_outcome ... Count: 3
Topic: /task_executor/start_episode ... Count: 2   # 见 11.8.3，已知限制不是 bug
```

`metadata.yaml`、`<name>_0.db3`、`episodes.csv` 三个文件都在，`git status` 确认 `results/bags/` 整个目录不出现在待提交列表里（`.gitignore` 修复生效）。

#### 11.8.3 概念讲解：为什么不新建 `pick_place_demo.launch.py`，以及 rosbag 回放里的 sim time 陷阱

**为什么直接扩展 `demo.launch.py`，不新建同名文件**：[2.5](#25-stage-j--episode-runner20-次连跑与第一段-demo) 原计划写的文件名是 `pick_place_demo.launch.py`，但那时候 `demo.launch.py` 还没写。现在 `demo.launch.py` 已经存在，而且做的就是同一件事——bridge + executor + rviz，默认模型是 `pick_place_scene.xml`。按计划字面意思新建一个文件，结果会是两个几乎一样的 launch 文件长期并存、以后改一个另一个忘了改。这是"结论被推翻时留档"的又一个例子：原计划写下时没有预见到 `demo.launch.py` 会先把这件事做了，现在补上说明，不是悄悄改掉原计划文字。

**为什么 `record_demo_bag.sh` 不负责启动 demo 本身**：如果脚本自己 `ros2 launch` 起 demo，再负责干净关掉它，等于把两类进程生命周期问题叠在一起——`ros2 launch` 本身就是个会吞掉裸 `kill -INT` 的 wrapper（[11.8.4](#1184-排查记录ros2-launch-和-ros2-run-是同一种假死)），录制器是否干净退出是另一个要单独验证的问题。让脚本假设 demo 已经在跑（`episode_runner.py` 对 bridge/executor 节点本来就是同样的假设），这次改动就只需要解决"录制器怎么干净停"这一个新问题，不用重新解一遍 `ros2 run` 那两个 session 才啃下来的旧问题。

**rosbag 回放里的 sim time 陷阱**（[2.5](#25-stage-j--episode-runner20-次连跑与第一段-demo) 计划里点名要讲的概念）：`/clock` 本身就是被录进这个 bag 的一个普通话题（`mujoco_bridge_node` 一直在发布它）。普通 `ros2 bag play` 默认按录制时的相对时间戳重放所有话题，`/clock` 也不例外，所以下游一个 `use_sim_time=true` 的节点直接订阅重放出来的 `/clock` 就能拿到正确的 sim time，不需要额外配置。**陷阱在于额外加 `--clock` 参数**：那个参数会让 `ros2 bag play` 自己根据消息时间戳再合成一份 `/clock` 发布出去——这个 bag 里已经有一个真实的 `/clock` 话题在重放，`--clock` 会造出第二个发布者，变成这个项目已经反复踩过的"同一个话题两个发布者，订阅者拿到哪个不确定"这一类问题（[STUDY_NOTES_GUIDE.md 3.2](../../STUDY_NOTES_GUIDE.md)），只是这次是在回放场景第一次撞见，不是在两个实时节点之间。

#### 11.8.4 排查记录：`ros2 launch` 和 `ros2 run` 是同一种假死

**现象**：`nohup ros2 launch mujoco_bridge demo.launch.py ... &` 后台起了 demo，`kill -INT <launch_pid>` 之后日志纹丝不动——没有任何 shutdown 相关的行，`ps` 显示 `mujoco_bridge_node`/`task_executor_node`/`rviz2`/`ros2` 四个进程原样继续跑，等了几秒依然如此。

**线索**：这跟 [STUDY_NOTES_GUIDE.md 3.1](../../STUDY_NOTES_GUIDE.md) 已经记录过的 `ros2 run` 假死现象一模一样——那条笔记当时只验证过 `ros2 run`，没验证 `ros2 launch`，因为 `ros2 launch` 那时候还没被这样用过。

**根因**：跟 `ros2 run` 是同一个机制：`ros2 launch` 也是个 Python wrapper，它的信号处理假设 SIGINT 来自终端（发给整个前台进程组，wrapper 和它启动的所有子进程一起收到），脚本/工具单独对 wrapper 这一个 PID 发 `kill -INT` 时，它的处理逻辑什么都不做——不是"处理慢"，是根本没触发对应的分支，等多久都一样。

**修复**：不指望单发信号能停掉整棵进程树。从启动日志里的 `process started with pid [N]` 行拿到每个子进程（`mujoco_bridge_node`/`task_executor_node`/`rviz2`）的真实 PID，连同 `ros2 launch` 自己的 PID 一起逐个 `kill`。

**验证**：按这个方法清理后，`ps -eo pid,comm` 确认四个进程全部消失，之后重新起 demo、跑 `record_demo_bag.sh`，行为符合预期（[11.8.2](#1182-改动清单与验证结果)）。

**留下的经验**：`ros2` 系的命令行工具（`run`、`launch`，大概率还有其他没试过的子命令）**默认都不能假设"给包装进程发一个信号就能让它和它管理的子进程一起干净退出"**——这次不是重新踩一个新坑，是发现已经写下的经验范围划窄了。已经把 [STUDY_NOTES_GUIDE.md 3.1](../../STUDY_NOTES_GUIDE.md) 那条从"`ros2 run` 是这样"改成"`ros2 run`/`ros2 launch` 都是这样"，这样以后碰到 `ros2 bag record`（已验证**不是**这样，见 [11.8.2](#1182-改动清单与验证结果)——它对裸 `kill -INT` 反应正常）之类新工具时，默认姿势是"先实测这个工具对裸信号的反应，不要预设它和 `ros2 run` 一样，也不要预设它不一样"。

#### 11.8.5 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现/防住 |
|---|---|---|
| `ros2 launch` 对裸 `kill -INT` 假死 | 子进程全部继续跑，日志无任何 shutdown 行 | 实测撞见，见 [11.8.4](#1184-排查记录ros2-launch-和-ros2-run-是同一种假死)；修法是按 PID 逐个 kill，不依赖 wrapper 转发信号 |
| `.gitignore` 的 `*.bag`/`*.mcap` 规则不匹配实际产出 | `ros2 bag record` 默认输出 `.db3` 目录，两条旧规则都不生效——第一次录制就会让 `git status` 里出现几 MB 的二进制文件 | 写计划阶段实测 `ros2 bag record --help` + 确认 `rosbag2_storage_mcap` 未安装时发现；已修，见 [11.8.2](#1182-改动清单与验证结果) |
| `-a` 录制下，episode 1 的 `~/start_episode` 触发消息可能未被录进 bag | `ros2 bag info` 显示 `/task_executor/start_episode` Count 比实际 episode 数少 1（实测：3 个 episode 只录到 2 条） | 已知限制，未修——见 [11.8.3](#1183-概念讲解为什么不新建-pick_place_demolaunchpy以及-rosbag-回放里的-sim-time-陷阱)、[record_demo_bag.sh](../../scripts/record_demo_bag.sh) 里的对应注释。该消息的*后果*（对应的 episode_outcome、物体轨迹）仍然完整录到 |
| 误加 `ros2 bag play --clock` 回放这个 bag | `/clock` 出现两个发布者，下游 `use_sim_time` 节点收到不确定是哪一个 | 概念已记录，见 [11.8.3](#1183-概念讲解为什么不新建-pick_place_demolaunchpy以及-rosbag-回放里的-sim-time-陷阱)；没有代码层面的防护，纯粹是"回放这个 bag 不要加 `--clock`"这条使用约定 |

#### 11.8.6 你没问但值得注意的

1. **（C类）`record_demo_bag.sh` 的环境卫生检查只挡"demo 没在跑"和"demo 跑了不止一份"，挡不住"demo 跑的是错的模型/参数"**——比如有人用 `model_path` 覆盖成了别的场景再录制，脚本不会发现，录出来的 bag 会显得像是 pick-and-place 演示、实际不是。
2. **（C类）`-a` 录全部话题这个决定，意味着以后新增任何话题都会自动被录进去，没有人会因为"话题变多了"收到通知**——这是选择 `-a` 而不是白名单时接受的代价（换取不用维护话题列表），但如果以后加了一个高频、大消息的调试话题（比如相机图像），第一次用这个脚本录制的人会在毫无预警的情况下得到一个远比预期大的 bag。
3. **（E类）`record_demo_bag.sh` 本身完全没有测试**——跟 `episode_runner.py` 一样是纯胶水脚本，按 [2.1.1](#211-这类系统该怎么测四层本周只取前两层) 的四层判据属于"任务层"，不该进 `colcon test`；这次的验证方式是我手动跑了两次、检查了输出结果，不是自动化断言。
4. **（F类）"`ros2 launch`/`ros2 run` 都不能信裸信号"这条经验目前只验证了 SIGINT，没测 SIGTERM**——`kill` 默认发的是 SIGTERM，这次踩坑用的是显式 `kill -INT`；两者会不会有不同表现（比如 SIGTERM 被 Python 的默认处理器直接终止、不像 SIGINT 那样被 `except KeyboardInterrupt` 特殊处理）没有验证过，是个没测的分支。
