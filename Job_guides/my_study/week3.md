# Week 3 学习笔记

> **本文按 stage 完成进度持续扩写**。第 1~6 节是开周计划与清单；第 7~9 节 Stage K/L/M 已完成，第 10 节 Stage N 保留其有效实验与设计复盘，第 11 节记录 Stage O 的代码、实测和问答。各 stage 按 [STUDY_NOTES_GUIDE.md](../../STUDY_NOTES_GUIDE.md) 2.2 的骨架组织。
>
> Stage 编号延续第1周的 A~E、第2周的 F~J，本周是 **K~O**（五个；Stage O 是 Stage N 复盘后新增的必做收束，[范围选择见 2.0](#20-为什么是五个-stage)）。

## 学习重点范围

本周对应计划书[第5节 Chap 3 模块](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md)里第2周没做完的后半段（5.2 第 4、5 条）与 Roadmap 第3周（"FK/Jacobian/damped diff-IK、模型一致性测试 → 数学单测、奇异性曲线、MoveIt/MuJoCo FK 对照"）。本周新增 Stage O 作为必须完成的架构收束，先解耦 Cartesian 任务 waypoint 与关节执行后端，再进入第4周的 RGB-D 感知和随机化评测。Stage N 的代码和回归实验已经完成，但经过复盘确认其目标构造仍依赖手调 Keyframe，不足以作为本周最终交付；本周只有 Stage O 完成后才算收周。

第2周做出来的是一条**能跑但不会算**的流水线：FSM、成功判据、episode 边界都在了，但机械臂去哪儿是[手调查表](#22-stage-l--模型一致性测试mujoco--urdf--自写三方对照)里的九组数字，物体挪 2cm 整条链就废了。本周补的是**"算"的那一半**：从物体位姿算出关节目标。需要重点掌握以下几块：

1. **FK 与 Jacobian 的数学，以及它们的三处隐含约定** — 参考点（TCP / hand / body 质心）、表达 frame（world / body）、6 维向量里平移和旋转谁在前。三个库三套默认值，**约定不一致时没有任何东西会报错**，只会得到一组"差一点但说不清差在哪"的数字。本周第一次要在代码里同时面对 MuJoCo、MoveIt 和自写实现这三套。
2. **交叉验证的独立性到底有多独立** — 已核实：`fer.urdf.xacro` 第74行直接 `xacro.load_yaml` 读 `kinematics.yaml`，所以"自写 FK（读 yaml 参数表）"和"MoveIt RobotState（读 URDF）"**共享同一份参数**，独立的只有实现，不是数据。真正独立的第二份数据源只有 MJCF。这决定了三方对照能抓到哪类错误、抓不到哪类，[2.1](#21-stage-k--自写-fkjacobian-纯函数库新包-arm_kinematics) 展开。
3. **damped least-squares diff-IK、奇异性与冗余** — λ 的量纲和自适应、最小奇异值作为奇异度量的边界条件、零空间 joint centering、速度与限位裁剪。计划书 5.3 第5条要求日志里有"最小奇异值"，这是本周第一次有这个量。
4. **运动学解与执行端的差距** — 第2周实测过：`joint2` 在 grasp 构型下稳态比命令差 0.12~0.17 rad，**命令得更远也没用**（actuator forcerange 在那个伸展位姿饱和于自重）。IK 算得再准，position servo 也到不了。本周最容易翻车的地方就在这里——"IK 对了但末端还是差几厘米"不是 IK 的 bug，把它当 bug 调会把 IK 改坏。
5. **接口抽象的第一次真实兑现** — `WaypointSource` 是第2周专门为本周留的缝（[week2.md 10.5](week2.md#105-waypointsource-接口设计为什么现在只有一个查表实现)）。换实现时 `fsm.cpp` 到底改不改一行，是对那次设计的实测判决，不是自我表扬。

## 目录

- [1. 从 Week 1/2 继承的硬约束与既有事实](#1-从-week-12-继承的硬约束与既有事实)
  - [1.1 开周核对出来的新事实（本周才第一次相关）](#11-开周核对出来的新事实本周才第一次相关)
- [2. 本周 Stage 计划](#2-本周-stage-计划)
  - [2.0 为什么是五个 stage](#20-为什么是五个-stage)
  - [2.1 Stage K — 自写 FK/Jacobian 纯函数库（新包 `arm_kinematics`）](#21-stage-k--自写-fkjacobian-纯函数库新包-arm_kinematics)
  - [2.2 Stage L — 模型一致性测试：MuJoCo ↔ URDF ↔ 自写三方对照](#22-stage-l--模型一致性测试mujoco--urdf--自写三方对照)
  - [2.3 Stage M — damped least-squares diff-IK 与奇异性观测](#23-stage-m--damped-least-squares-diff-ik-与奇异性观测)
  - [2.4 Stage N — `DiffIkWaypointSource`：FSM 一行不改地换掉查表](#24-stage-n--diffikwaypointsourcefsm-一行不改地换掉查表)
  - [2.5 Stage O — Cartesian waypoint 与执行后端解耦（本周必做，持续复盘）](#25-stage-o--cartesian-waypoint-与执行后端解耦本周必做持续复盘)
- [3. 本周验收标准](#3-本周验收标准)
- [4. 要回头解锁的 Week 1/2 悬挂项](#4-要回头解锁的-week-12-悬挂项)
- [5. 开周提示及处理状态（历史记录）](#5-开周提示及处理状态历史记录)
- [6. 悬挂问题（本周新增）](#6-悬挂问题本周新增)
  - [6.1 清单](#61-清单)
  - [6.2 反向清单：现在就该做的](#62-反向清单现在就该做的)
- [7. Stage K：自写 FK/Jacobian 纯函数库](#7-stage-k自写-fkjacobian-纯函数库)
  - [7.0 一句话总结](#70-一句话总结)
  - [7.1 改动清单与验证结果](#71-改动清单与验证结果)
  - [7.2 本阶段冻结的数学约定](#72-本阶段冻结的数学约定)
  - [7.3 JD 写“了解 FK/Jacobian”通常要求到什么程度](#73-jd-写了解-fkjacobian通常要求到什么程度)
    - [7.3.1 正向运动学应掌握到的程度](#731-正向运动学应掌握到的程度)
    - [7.3.2 Jacobian 应掌握到的程度](#732-jacobian-应掌握到的程度)
    - [7.3.3 最小补足路径](#733-最小补足路径)
    - [7.3.4 二维、三维旋转与一般位姿矩阵](#734-二维三维旋转与一般位姿矩阵)
    - [7.3.5 从物理意义理解 Jacobian 的一列](#735-从物理意义理解-jacobian-的一列)
    - [7.3.6 `model_loader` 的定位与测试路径](#736-model_loader-的定位与测试路径)
  - [7.4 这组测试证明了什么](#74-这组测试证明了什么)
  - [7.5 失败模式与验证手段](#75-失败模式与验证手段)
  - [7.6 本阶段边界与后续](#76-本阶段边界与后续)
- [8. Stage L：MuJoCo、MoveIt 与自写运动学三方一致性测试](#8-stage-lmujocomoveit-与自写运动学三方一致性测试)
  - [8.0 一句话总结](#80-一句话总结)
  - [8.1 改动清单与验证结果](#81-改动清单与验证结果)
  - [8.2 三方测试是怎样工作的](#82-三方测试是怎样工作的)
  - [8.3 `RobotModel`、`RobotState` 与 `JointModelGroup`](#83-robotmodelrobotstate-与-jointmodelgroup)
  - [8.4 测试层为什么没有破坏此前的隔离](#84-测试层为什么没有破坏此前的隔离)
  - [8.5 `hand_tcp` 不写进 MJCF 时怎样取得 Jacobian](#85-hand_tcp-不写进-mjcf-时怎样取得-jacobian)
  - [8.6 碰撞几何核对与测试边界](#86-碰撞几何核对与测试边界)
  - [8.7 失败模式与验证手段](#87-失败模式与验证手段)
  - [8.8 排查记录：首次编译与 lint 反馈](#88-排查记录首次编译与-lint-反馈)
  - [8.9 你没问但值得注意的](#89-你没问但值得注意的)
  - [8.10 本阶段边界与后续](#810-本阶段边界与后续)
- [9. Stage M：damped least-squares diff-IK 与奇异性观测](#9-stage-mdamped-least-squares-diff-ik-与奇异性观测)
  - [9.0 一句话总结](#90-一句话总结)
  - [9.1 改动清单与验证结果](#91-改动清单与验证结果)
  - [9.2 加权 DLS、误差向量与数值求解](#92-加权-dls误差向量与数值求解)
  - [9.3 阻尼、奇异值与奇异性诊断](#93-阻尼奇异值与奇异性诊断)
    - [9.3.1 奇异性扫线的读法](#931-奇异性扫线的读法)
  - [9.4 关节中心化项与冗余自由度](#94-关节中心化项与冗余自由度)
    - [9.4.1 一维冗余不等于只作用于一个关节](#941-一维冗余不等于只作用于一个关节)
  - [9.5 裁剪标准与关节限位](#95-裁剪标准与关节限位)
  - [9.6 最小二乘 IK 的计划边界](#96-最小二乘-ik-的计划边界)
    - [9.6.1 面试辨析：Jacobian 与最小二乘 IK 的区别](#961-面试辨析jacobian-与最小二乘-ik-的区别)
    - [9.6.2 `solveIk()`、在线反馈 diff-IK 与 MPC](#962-solveik在线反馈-diff-ik-与-mpc)
  - [9.7 失败模式与验证手段](#97-失败模式与验证手段)
  - [9.8 排查记录：阻尼错误注入与实验路径修正](#98-排查记录阻尼错误注入与实验路径修正)
  - [9.9 你没问但值得注意的](#99-你没问但值得注意的)
  - [9.10 本阶段边界与后续](#910-本阶段边界与后续)
- [10. Stage N：`DiffIkWaypointSource` 接入 FSM](#10-stage-ndiffikwaypointsource-接入-fsm)
  - [10.0 一句话总结](#100-一句话总结)
  - [10.1 改动清单与验证结果](#101-改动清单与验证结果)
  - [10.2 waypoint 的构造和缓存](#102-waypoint-的构造和缓存)
  - [10.3 为什么 Stage N 先使用离线 IK](#103-为什么-stage-n-先使用离线-ik)
  - [10.4 诊断信号：模型残差和执行残差必须分开](#104-诊断信号模型残差和执行残差必须分开)
  - [10.5 验收结果和失败边界](#105-验收结果和失败边界)
  - [10.6 失败模式与验证手段](#106-失败模式与验证手段)
  - [10.7 你没问但值得注意的](#107-你没问但值得注意的)
  - [10.8 排查记录：构建、运行和严格放置验证](#108-排查记录构建运行和严格放置验证)
  - [10.9 后续边界](#109-后续边界)
  - [10.10 Stage N 设计复盘：Keyframe 反推 Cartesian 目标是不合理的过渡](#1010-stage-n-设计复盘keyframe-反推-cartesian-目标是不合理的过渡)
  - [10.11 用户否定 Stage N 后的正式决策记录](#1011-用户否定-stage-n-后的正式决策记录)
- [11. Stage O：Cartesian 任务几何与离线 IK 适配](#11-stage-ocartesian-任务几何与离线-ik-适配)
  - [11.0 一句话总结](#110-一句话总结)
  - [11.1 改动清单与验证结果](#111-改动清单与验证结果)
  - [11.2 目标怎样从任务几何进入 IK](#112-目标怎样从任务几何进入-ik)
  - [11.3 工具朝向不能从目标的径向位置推出](#113-工具朝向不能从目标的径向位置推出)
  - [11.4 MJCF 初始位置、TCP 目标与验收参数的归属](#114-mjcf-初始位置tcp-目标与验收参数的归属)
  - [11.5 讲解内容的删减与流程修正](#115-讲解内容的删减与流程修正)
  - [11.6 失败模式与验证手段](#116-失败模式与验证手段)
  - [11.7 排查记录与被推翻的结论](#117-排查记录与被推翻的结论)
  - [11.8 补充核查与后续边界](#118-补充核查与后续边界)
- [12. 本周总结与第 4 周交接](#12-本周总结与第-4-周交接)
  - [12.0 本周完成情况](#120-本周完成情况)
  - [12.1 文档审阅结论](#121-文档审阅结论)
  - [12.2 第 4 周入口和边界](#122-第-4-周入口和边界)
- [13. 重构讨论与触发条件](#13-重构讨论与触发条件)
  - [13.0 现在的判断](#130-现在的判断)
  - [13.1 现在值得做的局部重构](#131-现在值得做的局部重构)
    - [13.1.1 第一层：提取配置、观测快照和 episode telemetry](#1311-第一层提取配置观测快照和-episode-telemetry)
    - [13.1.2 第二层：把任务目标和验收目标变成显式配置](#1312-第二层把任务目标和验收目标变成显式配置)
    - [13.1.3 第三层：把 reset 后等待改成可证明的新鲜度门控](#1313-第三层把-reset-后等待改成可证明的新鲜度门控)
      - [13.1.3.1 哪些组件能复用，时间戳对齐通常从哪里来](#13131-哪些组件能复用时间戳对齐通常从哪里来)
      - [13.1.3.2 旧 onReset 是否还在使用，如何处理兼容接口](#13132-旧-onreset-是否还在使用如何处理兼容接口)
      - [13.1.3.3 BridgeObservation 与独立 TF、旧话题的关系](#13133-bridgeobservation-与独立-tf旧话题的关系)
  - [13.2 暂时不要做的重构](#132-暂时不要做的重构)
  - [13.3 何时必须重构](#133-何时必须重构)

---

## 1. 从 Week 1/2 继承的硬约束与既有事实

本节只做速查，**权威出处都在别处**，不要在这里展开或改写。

| 约束 | 内容 | 出处 |
|---|---|---|
| **不得直接链接 MuJoCo** | 本周要用的 `mj_jac`/`mj_jacBody`/`mj_jacSite` **目前都不在 `MujocoApi` 里**，必须先加 `decltype(&mj_jacBody)` 字段再 `resolve()`，业务代码走 `api_.xxx()` | [week1.md 7.2](week1.md#72-调试时踩到的段错误符号冲突与-dlopen-隔离)、[week2.md 7.2](week2.md#72-decltype-签名把手抄错误从运行时-ub-变成编译错误) |
| **不用 MuJoCo 验 MuJoCo** | 交叉验证必须来自独立实现（KDL/MoveIt/URDF）或独立推导（有限差分）。本周整个 Stage L 建在这条上 | [week1.md 10.7.1](week1.md#1071-关键别用-mujoco-验-mujoco) |
| **FK 对照必须喂实际 `qpos`，不能喂 keyframe 标称值** | 否则容差要放宽到厘米级；Stage D 实测 `hand_tcp` 差 7mm 的成因是 **position servo 稳态误差，不是模型不一致** | [architecture.md 第3节](../../docs/architecture.md)、[week1.md 10.7](week1.md#107-怎么快速做一次独立的-fk-验证) |
| **四元数不能逐分量比** | `q` 与 `−q` 是同一个旋转（double cover）。断言写 $\|q_1 \cdot q_2\| \approx 1$ 或转成角度差 | [week1.md 6.3.1](week1.md#631-一个单位四元数就是一个旋转) |
| **frame 名 = MJCF 原生名** | 基座 `link0`（不是 `base_link`），**没有 `link8`**，`hand` 的父是 `link7`，`hand_tcp` 是合成的纯 103.4mm 平移（**不含** −45°，那个折进了 `hand` 自己的 `quat`） | [architecture.md 第1节](../../docs/architecture.md) |
| **URDF 侧名字全带 `fer_` 前缀且多一级 `link8`** | `left_finger` ↔ `fer_leftfinger`（**还少一个下划线**）。跨库对照代码必须显式处理这张映射表 | [architecture.md 第1节](../../docs/architecture.md) 末尾的对照表 |
| **`0.1034` 是手抄副本** | 权威在 `franka_description` 的 xacro，MJCF 侧没有这个 frame，两边没有任何构建步骤把它们焊在一起，上游改了不会告警。本周三方 FK 都要用到它 | [architecture.md 第1节](../../docs/architecture.md) |
| **`home`(MJCF) ≠ `ready`(SRDF)** | 两个不同构型（joint7 连符号都相反）。本周 Stage L 会第一次在同一个测试里同时用到它们，但**定权威仍推到第6/8周**（需要 ADR） | [architecture.md 2.1](../../docs/architecture.md) |
| **settle 常数是"改到通过为止"定的** | `close_settle_s`/`lift_settle_grace_s` 各 2.0s、`grasp_position_epsilon_rad` 0.3——全部建立在"发离散目标等伺服收敛"这个场景上。本周 Stage N 改变这个场景，**必须重测**，不能照搬 | [architecture.md 第7节](../../docs/architecture.md)、[week2.md 10.6](week2.md#106-到位判据为什么分层三条独立的等一等不是同一件事) |
| **测试分四层，本周仍只做 1、2 层** | 第3层（`launch_testing` 节点契约）week2 说"等第3周 MoveIt 接入后再评估"——本周接的是 `moveit_core` 库，不是 move_group 进程，**这个解锁条件其实没到**，继续推迟 | [week2.md 2.1.1](week2.md#211-这类系统该怎么测四层本周只取前两层) |
| **验证环境卫生** | `ps -eo pid,comm` + `ros2 topic info /clock` 是唯一权威判据；起后台节点直接跑可执行文件，不经 `ros2 run`/`ros2 launch`（两者是同一种假死） | [STUDY_NOTES_GUIDE.md 3.1](../../STUDY_NOTES_GUIDE.md) |
| **PATH 污染 / pyenv 两个变体** | `colcon build`/`colcon test` 前要过滤 `^/tmp/docker_shim` 和 `^/home/anby/.pyenv/versions`；rclpy 脚本一律显式写 `/usr/bin/python3` | [CLAUDE.md](../../CLAUDE.md) |

### 1.1 开周核对出来的新事实（本周才第一次相关）

这四条是开周时在容器里实际查出来的，之前没有任何文档记过：

1. **容器里没有 `moveit_core`**。已装的只有 `moveit_resources_panda_moveit_config` 和 `moveit_resources_panda_description` 两个**纯资源包**（yaml/srdf/urdf，没有任何库）。`architecture.md` 第0节写的"现成 MoveIt 配置"指的就是这个，不是 MoveIt 本体。本周已决定 `apt install ros-humble-moveit-core ros-humble-moveit-kinematics`（新增 36 个包，含 fcl/octomap/ruckig；对比装全量 `ros-humble-moveit` 是 64 个）。
2. **`moveit_resources_panda_description` 的 URDF 是第三套命名**（`panda_link0`/`panda_joint1`…），既不是 MJCF 的 `link0`，也不是 `franka_description` 的 `fer_link0`。**本周不用它**——`franka_description` 自己就带 `fer.srdf.xacro`（groups 定义在 `robots/common/group_definition.xacro`），URDF 和 SRDF 都从这一个包来，保持 [architecture.md 第0节](../../docs/architecture.md) 定下的"URDF 权威唯一"。
3. **`fer.urdf.xacro` 第74行 `xacro.load_yaml(.../fer/kinematics.yaml)`**——URDF 的几何参数就是从那份 yaml 生成的，两者不是两份数据。`kinematics.yaml` 是逐关节的 `x/y/z/roll/pitch/yaw` 表（`joint1`..`joint7` 加一个 `joint8` 到法兰），**格式上直接可用于自写 FK**，但用它意味着自写实现和 MoveIt 共享参数，只有实现独立。后果见 [2.1](#21-stage-k--自写-fkjacobian-纯函数库新包-arm_kinematics)。
4. **MuJoCo 侧 Jacobian 的四个入口**：`mj_jac`（任意点）、`mj_jacBody`（body 原点）、`mj_jacBodyCom`（质心）、`mj_jacSite`（site）。选哪个直接决定参考点是什么——而 `hand_tcp` 在 MJCF 里**既不是 body 也不是 site**（是 `mujoco_bridge` 合成的 TF），所以 MuJoCo 侧要拿 TCP 的 Jacobian，只能 `mj_jac` 手动传点，或者先把 `<site>` 加进 MJCF（[architecture.md 第4节](../../docs/architecture.md) 待办里那条）。这是本周第一次"那条待办不做就会卡住"。

## 2. 本周 Stage 计划

五个 stage，**每个都按五步走不跳步**（写代码 → build + 实跑验证 → 讲解 → 追问 → 写进本文件），见 [STUDY_NOTES_GUIDE.md 第3节](../../STUDY_NOTES_GUIDE.md)。Stage O 是用户在 Stage N 复盘后新增的必做架构 stage，现已实现并完成回归，讲解与追问修正记录在第 11 节。

### 2.0 为什么是五个 stage

计划书 Roadmap 第3周只写了"FK/Jacobian/damped diff-IK、模型一致性测试"，没有要求接进任务链。但**只做数学、不换掉 `KeyframeWaypointSource`，第3周结束时系统能力和第2周完全一样**（还是手调查表），而且第2周为这次替换专门留的接口会晾一周，晾着的接口通常会长歪。所以 Stage N 是必做。

随机物体位姿的批量泛化评测仍推到第4周，和 Chap 4 的 RGB-D 一起做。本周新增的 Stage O 不做大规模实验，但必须完成 Cartesian 抓取/放置目标与执行后端的解耦，并用固定场景回归证明新抽象没有退化。

### 2.1 Stage K — 自写 FK/Jacobian 纯函数库（新包 `arm_kinematics`）

| | 内容 |
|---|---|
| 为什么新建包 | 这段代码**天然是 Layer 1**（不依赖 `rclcpp`、不依赖 `mjModel`、不依赖任何消息类型），而且 `task_executor`（算 waypoint）和将来的 `motion_planner` 都要用。塞进 `mujoco_bridge` 会重演 week2 那条悬挂项（`grasp_criteria` 只因历史顺序长在 bridge 里，害得 `task_executor` 硬依赖 `mujoco_bridge`，[week2.md 10.7.1](week2.md#1071-graspsignalsclassifygrasp-为什么物理上放在-mujoco_bridge-包里不是-task_executor)）。**但不顺手搬 `grasp_criteria`**——那条的解锁条件是第6周换真机驱动包时，现在搬属于无触发条件的重构 |
| 改动 | ① 新包 `src/arm_kinematics/`（C++17 + Eigen，**不依赖 `rclcpp`**）；② 关节参数表从 `franka_description/robots/fer/kinematics.yaml` 来（[1.1 #3](#11-开周核对出来的新事实本周才第一次相关)），编译期常量还是运行时读 yaml 要在 stage 里定；③ `fk(q) -> 各 link 位姿 + hand_tcp`（`Eigen::Isometry3d`）；④ 几何法 Jacobian `jacobian(q) -> Eigen::Matrix<double,6,7>`，**参考点、表达 frame、平移/旋转分量顺序三者都要在头文件注释里写死** |
| 新增测试 | ① **解析 Jacobian vs 中心差分**（多组随机 `q`，步长扫描确认误差随 $h^2$ 下降——只测一个步长的话，常数倍错误会被容差吃掉）；② FK 链的自洽性（`T_ab * T_bc = T_ac`、逆、单位四元数）；③ 关节限位边界与近奇异构型各一组 |
| 验收 | `colcon test` 全绿；**并且**注入三类错误后必须变红：把某关节的 `roll` 符号改反、把 Jacobian 参考点从 `hand_tcp` 改成 `hand`、把角速度块和线速度块对调。延续 week2 自加的"对测试本身的测试" |
| 预期要讲的概念 | 几何法 Jacobian 每一列的物理含义（关节轴 × 从关节原点到参考点的矢量）；为什么姿态误差不能用四元数差、要用 log map / 轴角；有限差分的步长权衡（截断误差 vs 浮点抵消）；`Eigen::Isometry3d` 与手写 4×4 的取舍 |
| **必须在 stage 里明确回答的一件事** | 自写实现和 MoveIt 共享 `kinematics.yaml` 这份参数（[1.1 #3](#11-开周核对出来的新事实本周才第一次相关)），所以两者对上**只证明实现无误，不证明参数正确**；能抓到"参数抄错"的只有和 MJCF 的对照。这一条决定了 Stage L 三个对照对的权重完全不同，不能平均看待 |
| 顺手带上 | 新包从第一天起 `ament_lint_auto` 就要绿（week2 反向清单第1条是 `mujoco_bridge` 全量 lint 债务，**不要在新包上重蹈**） |

### 2.2 Stage L — 模型一致性测试：MuJoCo ↔ URDF ↔ 自写三方对照

对应计划书 [3.2](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md)（"对至少5组固定关节构型比较 MuJoCo 与 MoveIt 2 的末端 FK，误差超阈值就禁止继续集成"）和 [architecture.md 第4节](../../docs/architecture.md) 里挂了两周的那条待办。

| | 内容 |
|---|---|
| 改动 | ① `apt install ros-humble-moveit-core ros-humble-moveit-kinematics`，写进 [CLAUDE.md](../../CLAUDE.md) 的已装清单；② 从 `fer.urdf.xacro` + `fer.srdf.xacro` 构造 `moveit::core::RobotModel`，用 `RobotState::getGlobalLinkTransform()` / `getJacobian()`；③ 给 `MujocoApi` 加 `mj_jac`/`mj_jacBody`（[1.1 #4](#11-开周核对出来的新事实本周才第一次相关)）；④ 一个不依赖运行中仿真的 gtest 目标（自己 `mj_loadXML` + 写 `qpos` + `mj_forward`） |
| 构型集合（≥5组） | MJCF `home`、SRDF `ready`、全零位、一组近奇异（joint4 接近伸直）、一组贴限位边界、若干组随机。**不是随便挑5组**——每组要说明它在测什么 |
| 三个对照对（权重不同，见 2.1 末行） | **自写 ↔ MoveIt**：只验实现，参数共享。**自写/MoveIt ↔ MuJoCo**：真正的跨数据源对照，能抓参数抄错、URDF↔MJCF 的 `link8` 折叠是否真等价、`hand_tcp` 那个 `0.1034` 手抄副本是否漂了。**Jacobian 三方对照**：额外还要拉平三套约定（参考点/frame/分量顺序），三处任何一处不一致都表现成"数值对不上"，没有报错 |
| 硬性约束 | 断言输入用**直接写进 `mjData::qpos` 再 `mj_forward`** 的运动学，**不是**跑起来的仿真读数——否则测的是 position servo 的稳态误差（week1 那 7mm），不是模型一致性。这条是 [architecture.md 第3节](../../docs/architecture.md) 已经写下的约束，本周第一次真正执行 |
| 验收 | 阈值从 TBD 变成实测值并写回 [architecture.md 第3节](../../docs/architecture.md)（建议起点：位置 < 1mm、姿态 < 0.1°，**远严于**计划书建议的 5mm/1°——那是给"感知+执行"留的余量，纯运动学对照不该用它）；误差表逐构型逐 link 列出 |
| 预期要讲的概念 | MoveIt `RobotModel`/`RobotState`/`JointModelGroup` 三层是什么、不起 `move_group` 进程能不能用（本周就是这么用的）；SRDF 到底提供了 URDF 没有的什么；`getJacobian()` 的参考点默认是谁；URDF 的 `link7→link8→hand` 两步在 MJCF 折成一步，"数学等价"怎么算是被验证过了 |
| 顺带处理的待办 | ① `hand_tcp` 要不要落成 MJCF `<site>`（[1.1 #4](#11-开周核对出来的新事实本周才第一次相关) 说明了不落就得手动传点）——**要做就配 ADR**，因为它破坏"不改 vendor 文件"的约定；② URDF 与 MJCF 的碰撞几何是否同一份简化（[architecture.md 第4节](../../docs/architecture.md) 待办） |

### 2.3 Stage M — damped least-squares diff-IK 与奇异性观测

| | 内容 |
|---|---|
| 改动 | 在 `arm_kinematics` 里加求解层：$\Delta q = J^\top (JJ^\top + \lambda^2 I)^{-1} e$；λ 按最小奇异值自适应；零空间 joint centering 项；每步的速度裁剪与关节限位投影 |
| 关键设计点 | 误差向量 `e` 把米和弧度混在一起，**必须有显式加权**（否则"1cm 位置误差"和"1rad 姿态误差"被当成同一量级）；λ 的量纲跟着这个加权走。这个选择要写进头文件，不能埋在实现里 |
| 新增测试 | ① 纯求解层单测（给定 `J`、`e` 求 `dq`，不碰机器人）：良态构型下 $J\Delta q \approx e$，奇异构型下 `dq` 有界；② 收敛性：多组随机可达目标位姿，迭代到 $\|e\| <$ tol，统计迭代次数分布；③ **不可达目标必须优雅失败**（返回失败码 + 最后一次残差），不能死循环、不能返回 NaN；④ 注入错误：λ 置 0 后奇异构型的单测必须变红 |
| 奇异性曲线（计划书 5.3 第5条 + 5.4 扩展点1） | 沿一条**扫过奇异构型**的笛卡尔路径记录：$\sigma_{\min}$、条件数、生效的 λ、`dq` 峰值、跟踪误差。产物是一张图进 `results/`，不是一句"处理了奇异性" |
| 预期要讲的概念 | DLS 为什么等价于带正则项的最小二乘（$\min \|J\Delta q - e\|^2 + \lambda^2\|\Delta q\|^2$）；伪逆在奇异点为什么爆炸而 DLS 不会（SVD 视角：$\sigma/(\sigma^2+\lambda^2)$ 的上界）；零空间投影 $(I - J^+J)$ 在 7 自由度上具体多了什么自由；为什么要 clamp 误差范数（一步跨太大时线性化失效） |
| 明确不做（stretch） | 计划书 5.4 扩展点1 的完整对照（"伪逆后裁剪" vs "约束内求解"的路径偏差）——有余力再做，没余力进悬挂清单，不挤 Stage N |

### 2.4 Stage N — `DiffIkWaypointSource`：FSM 一行不改地换掉查表

| | 内容 |
|---|---|
| 改动 | ① `task_executor` 新增 `DiffIkWaypointSource : WaypointSource`：从 `object_pose` 构造各阶段的 TCP 目标位姿（pregrasp = 物体上方 h、grasp = 物体位姿加抓取偏移、…），调 Stage M 的求解器得关节目标；② 节点参数选择用哪个 `WaypointSource`（默认切到 IK，保留查表可切回——这是本周唯一的回归对照）；③ `fsm.cpp` **目标是一行不改** |
| **本 stage 最实质的设计决策** | diff-IK 有两种用法：**(a) 离线迭代到收敛，发一个关节目标给 position servo**（与现有 `~/joint_command` 接口和 FSM 的"到位判据"完全兼容，本质是把 diff-IK 当 IK 求解器用）；**(b) 每 tick 发一次增量，做在线笛卡尔伺服**（才是真正的 differential IK 闭环，但会和 position servo 的稳态误差、FSM 的到位判据直接打架，且需要重新想"到位"是什么意思）。**先做 (a)，但要在笔记里论证清楚 (b) 差在哪、什么时候必须换**，不能默认选了 (a) 就以为 diff-IK 只有这一种形态 |
| 已知会翻车的地方 | 运动学解到不了：week2 实测 grasp 构型下 `joint2` 稳态差 0.12~0.17 rad 且**命令更远无效**（forcerange 饱和于自重）。三条出路——放宽判据（治标）、前馈重力补偿、MJCF 开 `gravcomp`。**本周选哪条要显式记录理由**，因为它直接决定第9/10周控制模块的起点 |
| 必须重测的常数 | `close_settle_s`/`lift_settle_grace_s`/`grasp_position_epsilon_rad`/`place_x_m`/`place_y_m`——全部建在"手调查表 + 离散目标"这个场景上（[architecture.md 第7节](../../docs/architecture.md)）。`place_x/y` 当前是"IK 前的妥协值"（实测能到的点，不是 marker 名义位置 `(0.5, 0.3)`），**IK 接上后能不能回到名义位置，是这次替换有没有真正兑现的直接证据** |
| 验收 | ① 固定物体位姿下 20 次连跑仍然全绿（**回归**：不是新能力，是证明没弄坏）；② **至少 1 组新的物体位姿**，查表实现必然失败、IK 实现成功——这是本周唯一的"新能力"证据；③ 逐阶段日志新增：TCP 目标位姿、IK 迭代次数、最终残差、$\sigma_{\min}$（补上计划书 5.3 第5条最后那个缺口） |
| 预期要讲的概念 | 抓取位姿怎么从物体位姿构造（approach 方向、绕 TCP z 轴的自由度怎么定）；IK 有多解时怎么选（离当前构型最近 / 限位裕量最大）；"接口留缝"的收益这次到底兑现了多少——`fsm.cpp` 真改了几行，改的那几行说明当初哪里没想到 |

### 2.5 Stage O — Cartesian waypoint 与执行后端解耦（本周必做，持续复盘）

| | 内容 |
|---|---|
| 动机 | Stage N 的 `DiffIkWaypointSource` 仍直接持有 `KeyframeWaypointSource`，用关键帧的关节目标经过 FK 取得 TCP 姿态。这样可以验证离线 IK，但会把“任务几何”和“关节执行”绑在一起，后续接 MoveIt 或 Cartesian servo 时需要重新拆分 |
| 初步决策 | 新增 message-free 的 `CartesianWaypoint` 和 `CartesianWaypointSource`：输出 world frame 的 `hand_tcp` 目标、阶段和夹爪命令；离线 IK、MoveIt motion planning、在线 Cartesian servo 都作为后端适配器消费它 |
| 实际适配器 | `PickPlaceCartesianWaypointSource` 从物体和场景几何直接产生 Cartesian waypoint；`DiffIkWaypointSource` 改成 `Cartesian waypoint -> solveIk() -> JointTarget` 的适配器，不再直接依赖 `KeyframeWaypointSource`。最初草案中的 `KeyframeCartesianWaypointSource` 会保留被否定的依赖，因此没有实现 |
| 明确不做 | 本 stage 不接入 MoveIt planner，不把 `JointTrajectory` 执行器提前塞进 FSM，不做随机障碍实验。MoveIt 规划和控制执行留到后续 stage；但必须完成真实 Cartesian waypoint 到离线 IK 的适配，不能只停留在接口草图 |
| 需要验证 | FSM 行为和 Stage N 的 20 次固定场景回归不变；Cartesian waypoint 的 frame、夹爪时序和 TCP 目标与当前实现一致；旧 `waypoint_source:=keyframe` 对照仍可用 |
| 预期要讲的概念 | MoveIt 的 pose target、Cartesian path、IK、碰撞检查和全局规划分别负责什么；为什么 MoveIt 最终通常输出 `JointTrajectory`，而不是直接成为 FSM 的单个 `JointTarget`；任务 waypoint、规划结果和控制参考的接口边界 |

## 3. 本周验收标准

从计划书 [5.3](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md) 里挑出本周该收的（第 2、3、5 条是 week2 明确推过来的），加两条自定：

| # | 标准 | 出处 | 本周范围 |
|---|---|---|---|
| 1 | Jacobian 解析值与有限差分在容差内一致 | 5.3 第3条（week2 推来） | **本周做**（Stage K） |
| 2 | MuJoCo 与 MoveIt FK 一致性通过固定样例测试 | 5.3 第2条（week2 推来） | **本周做**（Stage L，且升级成三方对照） |
| 3 | 日志显示每阶段、目标 frame、末端误差、**最小奇异值**、失败原因 | 5.3 第5条（week2 只做了一半） | **本周补齐**（Stage M 产生该量，Stage N 打进日志） |
| 4 | 实现 damped least-squares differential IK，记录最小奇异值、关节限位裕量、tracking error | 5.2 第5条 | **本周做**（Stage M + N） |
| 5 | 固定物体位姿下连续成功 20 次，无非预期碰撞 | 5.3 第4条 | **回归**：week2 已达成，本周换了求目标的方式，必须不退化（Stage O） |
| 6 | 约束 differential IK（速度/位置限制、joint centering、奇异区减速）与"伪逆后裁剪"的路径偏差对照 | 5.4 扩展点1 | **部分**：限制与 centering 做（Stage M），**对照实验列为 stretch** |
| 7 | 随机物体位姿下的泛化评测 | 2.1 | **推到第4周**（见 [2.0](#20-为什么是五个-stage)） |
| 8 | Cartesian waypoint 与离线 IK 后端解耦 | 本周新增 Stage O | **本周必须完成**，先完成接口和回归，不接 MoveIt |

自加两条：

- **错误注入后单测必须变红**（延续 week2）。本周三类注入见 [2.1](#21-stage-k--自写-fkjacobian-纯函数库新包-arm_kinematics) 和 [2.3](#23-stage-m--damped-least-squares-diff-ik-与奇异性观测)。
- **一致性阈值从 TBD 变成实测数字并写回 [architecture.md 第3节](../../docs/architecture.md)**。挂了两周的 TBD 本周必须落地，否则"禁止继续集成"这条门槛等于不存在。

## 4. 要回头解锁的 Week 1/2 悬挂项

按 [STUDY_NOTES_GUIDE.md 4.5](../../STUDY_NOTES_GUIDE.md)："解锁节点到了要主动回头"。

| 条目 | 本周能做到哪一步 |
|---|---|
| [architecture.md 第4节](../../docs/architecture.md) 第一个自动 FK 测试（5组 `q`，MuJoCo vs MoveIt） | **本周完成**（Stage L），且扩成三方 |
| [architecture.md 第4节](../../docs/architecture.md) `hand_tcp` 的 `0.1034` 搬进 MJCF `<site>` | **本周必须决定**（Stage L）：不搬则 MuJoCo 侧 Jacobian 要手动传点。搬就要配 ADR（破坏"不改 vendor"） |
| [architecture.md 第4节](../../docs/architecture.md) URDF/MJCF 碰撞几何是否同一份简化 | Stage L 顺带核对（成本低，此刻正好两边模型都加载着） |
| [week2 6.1](week2.md#61-清单) `close_settle_s`/`lift_settle_grace_s` 换 diff-IK 后重测 | **本周解锁**（Stage N，解锁条件写的就是这一周） |
| [week2 6.2](week2.md#62-反向清单现在就该做的) `extractArmState`/`extractGripperWidth` 未剥纯函数 | Stage N 会动这块代码，**顺手评估**是否现在剥（判据用 week2 2.1.1 那条） |
| [week2 6.2](week2.md#62-反向清单现在就该做的) `ament_lint_auto` 全量债务 | 旧包债务已在 `9951aa7` 收口；新包 `arm_kinematics` 从第一天起保持绿（Stage K） |
| [week1 13.1 #4](week1.md#131-清单) 100Hz `/joint_states` 够不够 | **部分解锁**：diff-IK 的 (b) 在线伺服形态第一次真正关心反馈频率。本周走 (a)，所以只在讨论 (b) 时记录，不做实验 |
| [week1 13.1 #5](week1.md#131-清单) TF 由仿真发是否常规 / `robot_state_publisher` 的 remap | **不解锁**：本周只用 `moveit_core` 库，不起 `move_group`、不起 rsp，那条冲突还没发生 |
| [week1 13.1 #7](week1.md#131-清单) `home` vs `ready` 谁是权威 | **不定权威**（仍推第6/8周），但 Stage L 会把两个构型都作为测试样例，第一次让它们在同一处出现 |
| [week1 13.1 #2](week1.md#131-清单) timestep = 0.002 凭什么 | **继续挂着**：本周不增加接触复杂度 |
| [week2 6.1](week2.md#61-清单) `classifyGrasp` 没有"宽度-only"降级判据（标注"现在就可以做"） | **本周不插队**，进本周反向清单。它和 IK 无依赖关系，硬塞进来会打断主线 |

## 5. 开周提示及处理状态（历史记录）

按 [STUDY_NOTES_GUIDE.md 第5节](../../STUDY_NOTES_GUIDE.md)，这些是开周时主动指出的问题。它们保留在这里作为提问过程记录；已经在 Stage K~O 或后续审阅中回答的内容，以对应 stage 的实测和边界为准。

1. **Jacobian 的三处约定不一致时，没有任何东西会报错**（C 类）— 参考点（TCP / hand / body 质心）、表达 frame（world / body）、6 维向量里线速度和角速度谁在前。MuJoCo（`jacp`/`jacr` 分开两个数组、world frame）、MoveIt（`getJacobian()` 有默认参考点）、自写实现三套默认值大概率不同。**值得在写第一行代码前先把三边约定各自查清写下来**，否则数值对不上时是三选一甚至八选一的排错。
2. **"三方对照"其实只有两方数据独立**（E/F 类，已核实）— [1.1 #3](#11-开周核对出来的新事实本周才第一次相关)：`fer.urdf.xacro` 直接读 `kinematics.yaml`。所以自写 ↔ MoveIt 的一致**只证明两个实现都没写错，不证明参数对**。值得问的是：**那"参数抄错"这类错误，本周有哪一条测试真的能抓到？** 如果答案是"只有和 MJCF 的对照"，那这个对照对的容差就不能和另一对用同一个数。
3. **diff-IK 的"失败"在单测里长什么样**（E 类）— 不可达目标、奇异构型、超限位，这三种"算不出来"要不要用同一个失败码？更要紧的一问：**如果求解器返回了 NaN 并被发到 `~/joint_command`，现在的系统会怎样？** position servo 会把机械臂甩飞、被 `ctrlrange` 截住、还是 MuJoCo 整个炸掉——现在不知道，而这是 IK 接进执行链后**新出现的**一类输入。
4. **$\sigma_{\min}$ 作为奇异度量的适用边界**（F 类）— 它的数值依赖 `J` 里米和弧度的混合加权（[2.3](#23-stage-m--damped-least-squares-diff-ik-与奇异性观测)），换个加权同一个构型会得到不同的 $\sigma_{\min}$。所以"$\sigma_{\min} < 0.01$ 即接近奇异"这类阈值**不是构型的固有属性**。值得在写进日志之前就说清它相对于什么而言，否则这个数会被后面几周当成绝对量引用。

## 6. 悬挂问题（本周新增）

### 6.1 清单

- **MoveIt 与 MuJoCo 的碰撞判定是否需要跨引擎一致性测试**：Stage L 已确认两边碰撞几何并非全身同一表示，因此不能把“相同 `q` 必须得到完全相同接触集合”当作普遍断言。当前缺的是规划器真正如何消费 MoveIt 碰撞结果的参照系。解锁条件：进入 MoveIt 运动规划集成时，加入远离接触边界的明确无碰撞/自碰撞/环境碰撞样例，核对 SRDF Allowed Collision Matrix、padding 与 MuJoCo `contype`/`conaffinity` 后，再决定保留哪些跨引擎正负例。Manipulation control 阶段只继续补持续接触、允许接触和抓取接触语义，不重新承担模型一致性。

### 6.2 反向清单：现在就该做的

> 第1、2周遗留、本周不打算做的条目**不要**复制过来，仍以 [week1.md 13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) 和 [week2.md 6.2](week2.md#62-反向清单现在就该做的) 为权威。本节只放本周新产生的。开周先记两条：

- **历史 ADR 仍需补齐**：开周时 `docs/adr/` 为空，Stage O 已新增 [ADR 001](../../docs/adr/001-cartesian-task-waypoints.md)，记录 Cartesian 任务几何的归属。此前同目录 `<include>`、可视化从 RViz 回退到 GLFW、episode 边界改成话题触发的历史决策仍未独立成文，不能因为已有一份 ADR 就视为全部完成。
- **新包 `arm_kinematics` 的 lint 从第一天起就要绿**（版权头、include 顺序、uncrustify 格式）。旧包的 lint 债务已经收口；新包不能再积累同类债务。

## 7. Stage K：自写 FK/Jacobian 纯函数库

### 7.0 一句话总结

建立了独立的 `arm_kinematics` C++17 包：用 `franka_description` 的 `kinematics.yaml` 和 `joint_limits.yaml` 运行时加载模型参数，在不依赖 ROS、MuJoCo 或 MoveIt 的纯 Eigen 核心中计算七轴机械臂的 FK 和 `hand_tcp` 几何 Jacobian。容易混淆的末端链是 `link7 -> link8 -> hand -> hand_tcp`：`link7` 是第七关节后的活动 link，`link8` 是再沿 z 平移 `0.107m` 的固定法兰 frame，`hand` 相对它绕 z 旋转 `-45°`，`hand_tcp` 再沿 hand 的 z 轴平移 `0.1034m` 且不增加旋转；MJCF 把 `link8` 和 `-45°` 折进了 `link7 -> hand`，所以其中没有独立的 `link8`。解析 Jacobian 已通过中心有限差分的步长扫描验证，且新包从第一天起 lint 全绿。

### 7.1 改动清单与验证结果

| 项 | 结果 |
|---|---|
| 新包 | `src/arm_kinematics/`，导出 `arm_kinematics` 静态库 |
| 参数来源 | `/opt/ros/humble/share/franka_description/robots/fer/kinematics.yaml` 与 `joint_limits.yaml`；loader 接收显式路径，不把 ROS package lookup 混进数学核心 |
| FK 输出 | `link0..link8`、`hand`、`hand_tcp` 的 world-frame `Eigen::Isometry3d` |
| Jacobian | `6x7` 几何 Jacobian，`[vx, vy, vz, wx, wy, wz]`，world 表达，参考点为 `hand_tcp`，满足 `twist_world = J(q) * qdot` |
| 固定变换 | `link7 -> link8` 读取 joint8；`link8 -> hand` 使用 URDF 的 `-45°`；`hand -> hand_tcp` 使用 URDF 的 `0.1034m` 纯 z 平移 |
| 单测 | 8 项 gtest：参数读取及非法 YAML、FK 链复合/求逆、NaN/越界输入策略、Jacobian 中心差分步长扫描、TCP 参考点、关节限位、近奇异构型有限性 |
| lint | `copyright`、`cpplint`、`uncrustify`、`lint_cmake`、`xmllint` 全部通过；cppcheck 因环境中的已知慢版本跳过 |
| 实际验证 | `colcon test --packages-select arm_kinematics`：7/7 通过，0 失败 |

### 7.2 本阶段冻结的数学约定

1. `q` 顺序固定为 `joint1..joint7`，单位为 rad；夹爪不进入这套 7-DoF Jacobian。
2. 所有 FK 输出都是 world-frame 绝对变换；`world` 与 MJCF 的 `link0` 重合，URDF 的 `base` 与 `fer_link0` 通过固定单位边对应。
3. Jacobian 是 world-frame 几何 Jacobian，行顺序固定为线速度前三行、角速度后三行，参考点固定为 `hand_tcp`。
4. `hand_tcp` 相对 `hand` 是 `[0, 0, 0.1034]m` 的 identity-rotation 固定变换；`-45°` 属于 `link8 -> hand`，不能重复加到 TCP。
5. 姿态比较使用旋转矩阵的相对角度，不逐分量比较四元数；后续姿态误差使用轴角/log map，而不是四元数分量差。

### 7.3 JD 写“了解 FK/Jacobian”通常要求到什么程度

> 如果 JD 上要求了解正向运动学和 Jacobian，到底要什么程度？我现在基本知道意思，知道输入输出和怎么使用，但是不清楚具体计算公式、规则和实现细节，这样是否足够？

目前已经达到“会使用”的入门层，但对多数机器人软件、运动规划或控制岗位来说，还差一层可解释、可排错的理解。JD 只写“了解”时，通常不要求背出 Franka 七轴机械臂的完整展开式，也不要求从零实现通用运动学库；合理下限是能说明下面两条公式、在简单机械臂上推导一次，并能看懂项目中的实现。

#### 7.3.1 正向运动学应掌握到的程度

正向运动学的输入是关节位置 $q$，输出是各 link 或末端相对基准坐标系的位姿。核心规则是沿运动链依次复合坐标变换：

$$
T^0_n=T^0_1T^1_2\cdots T^{n-1}_n
$$

每一节包含两部分：关节安装位置带来的固定变换，以及当前关节角带来的旋转。当前实现对应：

```cpp
current = current * model.joints[i].origin *
  jointRotation(model.joints[i], q(i));
```

其中 `origin` 是父 link 到关节 frame 的固定变换，`jointRotation(...)` 是绕该关节轴旋转 $q_i$。循环得到 `link1..link7` 后，再依次追加固定的 `link7 -> link8 -> hand -> hand_tcp` 变换。

需要理解矩阵乘法顺序不能交换，并能通过 frame 名判断相邻变换能否相乘；不需要记住 Franka 每个关节的尺寸、角度或手算七轴最终矩阵。面试若给一个二维两连杆机械臂，应当能根据两段长度和两个关节角写出或推导末端位置。

#### 7.3.2 Jacobian 应掌握到的程度

Jacobian 描述当前构型附近，关节速度如何映射为末端瞬时速度：

$$
\begin{bmatrix}v\\\omega\end{bmatrix}=J(q)\dot q
$$

七轴机械臂的几何 Jacobian 是 $6\times7$：每一列表示“只让一个关节以单位速度运动时，TCP 会产生什么线速度和角速度”。对旋转关节，第 $i$ 列为：

$$
J_i=
\begin{bmatrix}
a_i\times(p_{tcp}-p_i)\\
a_i
\end{bmatrix}
$$

这里 $a_i$ 是关节轴、$p_i$ 是关节原点、$p_{tcp}$ 是 TCP 位置，并且三者必须表达在同一个 frame。当前代码直接对应这个公式：

```cpp
const Eigen::Vector3d axis_world =
  joint_frame.linear() * model.joints[i].axis.normalized();
result.block<3, 1>(0, i) =
  axis_world.cross(tcp_position - joint_position);
result.block<3, 1>(3, i) = axis_world;
```

还需要知道三个常见约定会改变数值：参考点是 `hand` 还是 `hand_tcp`、结果表达在 world 还是 body frame、六维向量是线速度在前还是角速度在前。当前接口选择 `hand_tcp`、world frame、线速度在前。

面试中的合理下限是：能解释 $\dot x=J\dot q$，看懂上面的叉乘公式，知道 Jacobian 丢秩意味着奇异构型，并知道可以把 FK 对关节角做有限差分来检查 Jacobian。不要求默写 Franka 的完整 $6\times7$ 数值表达式。

#### 7.3.3 最小补足路径

针对 JD，最有效的补足顺序是：

1. 用二维两连杆机械臂手推一次 FK。
2. 对末端位置分别求 $q_1,q_2$ 的偏导，得到二维 Jacobian。
3. 理解三维旋转关节的叉乘公式是同一局部导数关系的几何表达。
4. 回到本项目，能逐行解释 `fk()` 和 `jacobian()` 中每个量属于哪个 frame。
5. 能说明有限差分能验证实现自洽，但不能证明共享的模型参数正确。

达到这一步，可以合理地说“了解 FK/Jacobian”。若 JD 写的是“熟悉运动学”“实现 IK/控制器”或“掌握机器人建模与控制”，才需要继续深入伪逆、阻尼最小二乘、奇异值、冗余与关节限位等内容；这些属于本周 Stage M，而不是 Stage K 必须一次学完的前置条件。

#### 7.3.4 二维、三维旋转与一般位姿矩阵

二维绕原点旋转 $\theta$ 的矩阵是：

$$
R(\theta)=
\begin{bmatrix}
\cos\theta&-\sin\theta\\
\sin\theta&\cos\theta
\end{bmatrix}
$$

三维旋转需要说明绕哪条轴。绕 $x,y,z$ 轴旋转时分别为：

$$
R_x=\begin{bmatrix}1&0&0\\0&c&-s\\0&s&c\end{bmatrix},\quad
R_y=\begin{bmatrix}c&0&s\\0&1&0\\-s&0&c\end{bmatrix},\quad
R_z=\begin{bmatrix}c&-s&0\\s&c&0\\0&0&1\end{bmatrix}
$$

其中 $c=\cos\theta,s=\sin\theta$。一般三维姿态可由多个轴旋转复合；本项目读取 URDF RPY 时使用 $R=R_z(yaw)R_y(pitch)R_x(roll)$，顺序不能交换。

加入平移后，用齐次矩阵把旋转和平移统一表示。二维位姿是 $3\times3$：

$$
T=\begin{bmatrix}R_{2\times2}&p_{2\times1}\\0\;0&1\end{bmatrix}
=\begin{bmatrix}c&-s&x\\s&c&y\\0&0&1\end{bmatrix}
$$

三维位姿是 $4\times4$：

$$
T=\begin{bmatrix}R_{3\times3}&p_{3\times1}\\0\;0\;0&1\end{bmatrix}
$$

它作用于齐次坐标时得到 $p'=Rp+t$。代码中的 `Eigen::Isometry3d` 就是这种三维刚体位姿；`linear()` 取 $R$，`translation()` 取 $t$。

#### 7.3.5 从物理意义理解 Jacobian 的一列

旋转关节第 $i$ 列为：

$$
J_i=\begin{bmatrix}a_i\times(p_{tcp}-p_i)\\a_i\end{bmatrix}
$$

前三项是 TCP 的线速度。叉积 $a_i\times(p_{tcp}-p_i)$ 的方向同时垂直于旋转轴和“关节到 TCP”的半径，因此正好沿圆周切线；它的大小是轴角速度乘以 TCP 到旋转轴的垂直距离，距离轴越远，线速度越大。

后三项是 TCP 的角速度。只让该关节以单位角速度旋转时，所有下游刚体都绕同一根轴转，所以末端角速度方向就是 world frame 中的关节轴 $a_i$。乘上实际 $\dot q_i$ 后，该关节贡献的角速度为 $a_i\dot q_i$。

#### 7.3.6 `model_loader` 的定位与测试路径

`model_loader.cpp` 不是 MJCF 或通用 URDF 解析器，而是针对 Franka FER 的轻量适配器：它读取 `kinematics.yaml` 中 `joint1..joint8` 的固定变换和 `joint_limits.yaml` 中 `joint1..joint7` 的限位，再补上 YAML 没有表达的模型假设。需要注意的假设包括：`joint1..7` 都绕自身 z 轴旋转、world 与 `link0` 重合，以及手写的 `link8 -> hand` 的 $-45^\circ$ 和 `hand -> hand_tcp` 的 `0.1034m`。RPY 按 $R_z(yaw)R_y(pitch)R_x(roll)$ 组合，顺序写错不会产生类型错误。

测试中的 `KINEMATICS_YAML_PATH` 和 `JOINT_LIMITS_YAML_PATH` 不是头文件变量，而是 `CMakeLists.txt` 通过 `target_compile_definitions(test_arm_kinematics PRIVATE ...)` 只注入测试目标的字符串宏。CMake 先从 `franka_description_DIR` 得到包的 share 目录，再生成实际编译参数；当前环境展开后分别指向 `/opt/ros/humble/share/franka_description/robots/fer/kinematics.yaml` 和 `joint_limits.yaml`。库接口本身仍要求调用者显式传路径，因此数学库不依赖 ROS package lookup。

输入策略在本阶段明确为：`fk()` 和 `jacobian()` 对 NaN/Inf 抛出 `std::invalid_argument`，避免坏数静默传播；对有限但超出关节限位的角度仍然计算，因为运动学公式本身有定义，物理上能否执行由 `withinJointLimits()` 单独判断。YAML 缺字段、包含 NaN/Inf 或出现下限大于上限时，loader 立即抛异常；三类情况均有负向测试。

### 7.4 这组测试证明了什么

当前测试证明的是自写实现内部自洽，以及解析 Jacobian 与自身 FK 的数值导数一致。有限差分扫描使用 `h=1e-2、5e-3、2.5e-3`，误差随 `h^2` 下降；这能抓住关节轴、参考点、线/角速度块等实现错误。

它还没有证明 URDF/MJCF 两套模型一致，也没有证明 `kinematics.yaml` 的物理参数正确。因为自写实现和未来的 MoveIt `RobotState` 都会间接使用同一份 URDF/YAML 参数，二者对上只说明实现不同但解释一致。模型来源的独立验证留给 Stage L 的 MuJoCo 三方对照。

### 7.5 失败模式与验证手段

- 把某个 joint origin 的 roll 符号改反：有限差分仍可能和错误解析式一致，因此需要 Stage L 的 MuJoCo 对照抓住“参数错但实现自洽”的情况。
- 把 Jacobian 参考点改成 `hand`：`hand_tcp` 的有限差分测试会失败，因为固定 TCP 偏移会改变线速度块。
- 交换线速度和角速度块：有限差分的 6 维逐块比较会失败。
- 把 position servo 的实际稳态关节角拿来做 FK 对照：会把控制误差误判成模型误差；Stage L 必须直接写 `mjData::qpos` 后调用 `mj_forward`。

### 7.6 本阶段边界与后续

Stage K 至此结束，本阶段不加入 MoveIt、MuJoCo API 或 diff-IK。下一阶段需要构造 `moveit::core::RobotModel/RobotState`，明确 `fer_*` 与 MJCF 原生 frame 的映射，并用至少五组固定 `q` 做 FK/Jacobian 三方对照；届时还要决定 MuJoCo 侧用 `mj_jac` 手动传 TCP 点，还是给模型增加 `hand_tcp` site。

## 8. Stage L：MuJoCo、MoveIt 与自写运动学三方一致性测试

### 8.0 一句话总结

安装 MoveIt Core 后，在 `mujoco_bridge` 的测试层建立了不依赖运行中仿真的三方模型门禁：同一组关节位置分别送入自写 `arm_kinematics`、MoveIt `RobotState` 和直接加载 `panda.xml` 的 MuJoCo，再逐 link 比较 FK、逐元素比较 `hand_tcp` Jacobian。测试覆盖 7 组有明确目的的构型，首次实测最大位置误差 `6.87e-16m`、姿态误差 `3.49e-8rad`、Jacobian 元素误差 `1.78e-15`；最终全仓 5 个包构建成功，224 项测试 0 失败。Stage D 曾测到的毫米级差值因此被明确归因于运行中 position servo 的稳态误差，而不是 URDF/MJCF 模型不一致。

### 8.1 改动清单与验证结果

| 项 | 结果 |
|---|---|
| 环境依赖 | 安装 `ros-humble-moveit-core`、`ros-humble-moveit-kinematics` 2.5.10；apt 实际新增 36 个包、约 82.5MB；已同步 [CLAUDE.md](../../CLAUDE.md) |
| 新测试 | [test_model_consistency.cpp](../../src/mujoco_bridge/test/test_model_consistency.cpp)：自行加载三套模型，不启动 `move_group`、ROS 节点或仿真循环 |
| MuJoCo API | [mujoco_dl.hpp](../../src/mujoco_bridge/include/mujoco_bridge/mujoco_dl.hpp)/[mujoco_dl.cpp](../../src/mujoco_bridge/src/mujoco_dl.cpp) 增加 `mj_jac` 与 `mj_jacBody`，继续走 `RTLD_LOCAL | RTLD_DEEPBIND` 隔离 |
| 构建接线 | [CMakeLists.txt](../../src/mujoco_bridge/CMakeLists.txt) 在测试配置中展开 `fer.urdf.xacro`/`fer.srdf.xacro`；MoveIt、xacro 和 `arm_kinematics` 都是测试依赖，不进入 bridge 节点的生产依赖 |
| 依赖声明 | [package.xml](../../src/mujoco_bridge/package.xml) 增加 `arm_kinematics`、`franka_description`、`moveit_core`、`xacro` 的 `test_depend` |
| 权威事实 | 门槛、实测误差、碰撞几何差异与 `hand_tcp` 决策已同步 [architecture.md 第3节](../../docs/architecture.md) |
| 最终构建 | `colcon build --symlink-install`：5/5 包成功 |
| 最终测试 | `colcon test` + `colcon test-result --all --verbose`：224 tests，0 errors，0 failures，32 skipped；skipped 均为环境中已知慢版 cppcheck |

测试构型不是为了凑够 5 组，而是各自覆盖一类风险：

| 构型 | 目的 |
|---|---|
| 全零 | 纯代数零位；即使 joint4 超出物理限位，三套 FK 仍应对有限输入给出相同数学结果 |
| MJCF `home` | 覆盖仿真 reset 使用的 vendor keyframe |
| SRDF `ready` | 覆盖 Franka/MoveIt 生态的命名状态，并防止把它误当成 `home` |
| `near_singular` | 覆盖接近伸直的构型，为 Stage M 奇异性处理留基线 |
| `near_limits` | 七个关节靠近交替限位，扩大角度和姿态覆盖 |
| `random_a/b` | 两组固定内部样例，扩大一般构型覆盖且保证回归可复现 |

门禁与首次实测：

| 量 | 断言门槛 | 首次最大误差 |
|---|---:|---:|
| FK 位置范数 | `< 1e-6m` | `6.866350198e-16m` |
| FK 相对旋转角 | `< 1e-6rad` | `3.491982864e-8rad` |
| Jacobian 单元素绝对误差 | `< 1e-8` | `1.776356839e-15` |

姿态残差集中在 MJCF `hand`：其中表示 $-45^\circ$ 的四元数只写到 `0.9238795 ... -0.3826834`，因此不会得到严格机器零，但仍远低于门槛。

### 8.2 三方测试是怎样工作的

测试夹具的 `SetUp()` 为每个 `TEST_F` 建立一套干净实验环境：

1. `arm_kinematics::loadFrankaFerModel()` 从官方 YAML 建自写模型。
2. 解析构建期展开的 URDF/SRDF，构造 `moveit::core::RobotModel`；`JointModelGroup("fer_arm")` 选出七轴组，`RobotState` 保存当前 `q` 和变换缓存。
3. `mj_loadXML()` 直接加载 `panda.xml` 并创建 `mjData`，按名字解析每个关节的 `jnt_qposadr` 与 `jnt_dofadr`。
4. 每组构型同时写入 MoveIt `RobotState` 和 MuJoCo `qpos`；MuJoCo 随后调用 `mj_forward` 刷新 `xpos`、`xmat`、Jacobian 等派生量，不推进时间。
5. `link0..link7`、`hand`、`hand_tcp` 做三方逐对 FK 比较；URDF 独有的 `link8` 做自写与 MoveIt 比较。最后把三套 `hand_tcp` Jacobian 拉平为 world 表达、线速度在前、`joint1..7` 列顺序后比较。

这里不能经过 actuator 或读取运行中仿真：那样测到的会同时包含 position servo、重力和稳态误差。Stage L 要回答的是“相同精确 `q` 下三套运动学是否一致”，所以必须直接写 `qpos` 后 `mj_forward`。

### 8.3 `RobotModel`、`RobotState` 与 `JointModelGroup`

`RobotModel` 是从 URDF/SRDF 构造出的相对稳定模型：link/joint 拓扑、固定变换、限位以及 SRDF 规划组等语义都住在这里。`JointModelGroup` 是 `RobotModel` 内部的一张只读视图，本阶段的 `fer_arm` 把七个活动关节按 MoveIt 认可的顺序组织起来。`RobotState` 则是一份可变状态：当前关节值以及由它们计算出的全局 link 变换缓存。

因此本阶段只需要链接 `moveit_core`，不需要运行 `move_group`。`move_group` 是提供规划服务、插件和场景管理的进程；这里使用的是底层 C++ 数据结构和运动学函数。

### 8.4 测试层为什么没有破坏此前的隔离

> 我希望知道这个程序为什么这样设计，背后的思想是什么，希望能够讨论变量和函数的作用域、权限、名称空间等等问题，并且讨论一下我们先前做的隔离机制（比如自写FK不依赖mujoco）在这里起到了什么好的作用吗？还有些新东西比如SCOPED_TRACE，为什么cout会在这里的函数中使用（运行测试的时候可以看到它们吗）。

纯 C++ 机制（GoogleTest、fixture、`protected`、匿名 namespace、智能指针所有权、`SCOPED_TRACE` 和 `cout` 捕获规则）的完整讲解分流到 [cpp_concepts.md：GoogleTest fixture、作用域与所有权](cpp_concepts.md#googletest-fixture作用域与所有权以-stage-l-为例)。本节只记录工程边界。

Stage K 把自写 FK/Jacobian 留成只依赖 Eigen/yaml-cpp 的纯库，这次带来四个直接收益：

1. 三方测试可以把它当普通函数调用，不需要构造 ROS 节点、伪造消息或把数据绕一圈话题。
2. MoveIt 和 MuJoCo 只在测试层汇合；`arm_kinematics` 的生产依赖没有反向长出 MoveIt/MuJoCo。
3. `BUILD_TESTING=OFF` 时，bridge 节点也不会因为这项验证而依赖 MoveIt。
4. 三个入口保留不同实现路径，使“比较一致”仍然有证据价值；若把自写 FK 改成内部直接调用 MoveIt，再拿它和 MoveIt 比较就只是在比较同一实现两次。

独立性仍有边界：自写实现和 MoveIt 都从 `franka_description` 的同一份 `kinematics.yaml` 获得参数，所以二者对上只验证实现解释一致；只有与独立 MJCF 的对照能抓住共享参数错误或两份描述漂移。

### 8.5 `hand_tcp` 不写进 MJCF 时怎样取得 Jacobian

> 对于TCP Jacobian偏移，还是不必讲解数学底层原理了。

本阶段保留工程层结论，不展开速度搬移的数学推导：

- MJCF 中没有 `hand_tcp` body/site；它一直由 `hand` 位姿加局部 z 轴 `0.1034m` 合成。
- 测试先合成 TCP 的 world-space 坐标，然后调用 `mj_jac(model, data, ..., point, hand_id)`。这个 API 能直接计算固定在指定 body 上任意世界点的 Jacobian。
- 当前实现**不是**先调用 `mj_jacBody(hand)` 再在项目代码中手动变换 Jacobian；参考点搬移由 MuJoCo 内部完成。
- MoveIt 的 URDF 本来就有 `fer_hand_tcp` link，因此对该 link 原点调用 `getJacobian()`；自写实现则直接以 `p_tcp` 为参考点计算。
- 第二个 gtest 把 `mj_jac` 的任意点恰好设为 hand 原点，并断言它与 `mj_jacBody(hand)` 相同，用来钉住两个 API 的参考点语义。

最终决定是不直接修改 vendor `panda.xml`。好处是 vendor 文件能继续和上游原样比较；代价是 `0.1034` 仍是手抄副本。三方门禁将它从“漂移后无人发现”变成“漂移后测试失败”。若以后有真实的 MuJoCo site 消费者，再用项目自有 overlay MJCF 增加 site，而不是直接改 vendor 文件。

### 8.6 碰撞几何核对与测试边界

> 碰撞相关的内容是在哪里进行测试的？多讲讲碰撞测试相关的内容。
>
> 碰撞测试应该在后面处理manipulation control的week中开展你觉得对吗？还是说没必要做？

Stage L 没有实现 MoveIt 与 MuJoCo 的自动碰撞结果一致性测试。本阶段完成的是**静态资源核对**：对 collision mesh 做 SHA256 和结构检查。结果为 `link0..link4`、`link6`、`link7`、`hand` 两边 STL 完全相同；`link5` 在 URDF 是一份 STL、MJCF 是三个 OBJ；手指在 URDF 是 4 个 box，MJCF 是 `finger_0` mesh 加 5 个 fingertip box。因此不能建立“同一 `q` 下两边完整接触集合必须相同”的普遍断言。

现有 [test_grasp_state.cpp](../../src/mujoco_bridge/test/test_grasp_state.cpp) 使用最小 `contact_probe.xml` 验证 `mj_forward` 后 `mjData::contact` 的 body-pair 提取、参数顺序无关性和单/双指接触。它测的是 **MuJoCo 接触读取代码**，不是跨引擎碰撞等价。

合理的后续分层是：

| 时机 | 应验证什么 |
|---|---|
| Stage L | 记录碰撞资源是否同源、明确差异和知识边界；已完成 |
| MoveIt 规划集成 | 明确无碰撞、自碰撞、桌面/物体碰撞样例；验证 FCL、SRDF Allowed Collision Matrix 和 padding；选少量远离边界的样例与 MuJoCo 对照 |
| Manipulation control | 持续接触、允许接触、抓取接触、非预期碰撞和接触抖动等任务语义 |

因此不是“没必要做”，也不应全部推到 manipulation control。决定轨迹可执行性的 MoveIt 碰撞门禁应随规划集成完成；control 周只承担接触随时间演化的部分。该事项已写入 [6.1](#61-清单)，解锁条件是 MoveIt 规划真正开始消费碰撞结果。

### 8.7 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现或防住 |
|---|---|---|
| 把 `jnt_qposadr` 当成 Jacobian 列号 | 当前纯 hinge Panda 可能碰巧通过；加入 free joint 后列整体错位 | `qpos` 写入只用 `jnt_qposadr`，Jacobian 提取只用 `jnt_dofadr`；既有 `free_body.xml` 单测已证明二者会分离 |
| TCP 偏移副本漂移 | hand FK 仍一致，但 TCP 位置和 Jacobian 都偏 | 实际把 `0.1034` 注入为 `0.1044`：测试变红，最大位置误差 `1e-3m`、Jacobian 元素误差约 `9.63e-4` |
| 用 position servo 的稳态状态做模型断言 | 把控制误差误判为模型误差，容差被迫放宽到毫米/厘米级 | 测试直接写 `mjData::qpos` 后调用 `mj_forward`，不启动 actuator 或仿真循环 |
| 四元数逐元素比较 | 同一旋转的 `q`/`-q` 被误报，或分量误差没有清晰物理单位 | 用相对旋转矩阵的轴角大小比较，单位为 rad |
| 三套 Jacobian 的参考点、frame 或行顺序不同 | 数值形状相同但内容系统性对不上，没有 API 报错 | 统一为 `hand_tcp`、world、线速度在前；三方逐元素门禁 `<1e-8` |
| 把自写 ↔ MoveIt 一致误当成参数独立验证 | 两边共享的 YAML 同时写错仍然全绿 | 把 MuJoCo 对照视为跨数据源证据；三对结果不等权解释 |
| fixture 间共享可变 `RobotState`/`mjData` | 测试顺序改变结果，单跑通过、全跑失败 | 当前每个 `TEST_F` 都重新 `SetUp()`；不为约 0.4s 的测试时间引入 suite 级共享状态 |
| 碰撞 mesh 部分相同就推断全身碰撞一致 | link5/手指附近出现不同碰撞判定 | 权威文档明确记录差异；规划阶段只选远离边界的跨引擎样例，不要求临界点完全一致 |

### 8.8 排查记录：首次编译与 lint 反馈

首次编译失败在 URDF 指针的静态类型：成员保存为 `urdf::ModelInterfaceSharedPtr`，接口类型没有派生类 `urdf::Model::initString()`。修复不是强转，而是先用 `std::shared_ptr<urdf::Model>` 完成解析，再赋给接口指针供 SRDF/MoveIt 使用。这保留了多态接口，同时把派生类专有初始化限制在局部变量作用域。

功能测试第一次即通过；完整 `colcon test` 随后暴露 4 个 include-order cpplint 报告和 1 个 uncrustify 换行报告。按工具建议调整后，单独 lint 与最终全仓测试均全绿。这个过程再次说明“gtest 通过”只证明行为断言，通过完整 `colcon test` 才包含项目约定的静态质量门禁。

### 8.9 你没问但值得注意的

1. **上游模型版本变化如何归因**（C/E 类）：测试每次构建都展开系统 apt 中的 `franka_description`。未来 apt 升级导致门禁变红时，需要把已安装包版本和模型内容 hash 一起记录，才能区分项目回归与上游数据变化。
2. **成功测试的逐 link 数值目前只在 stdout/CTest 日志**（C 类）：它足够定位单次失败，但不适合观察多次 CI 之间的缓慢漂移；建立 CI 时应决定是否保存结构化 CSV/JUnit artifact。
3. **MuJoCo 裸资源尚未使用 RAII 包装**（E 类）：正常 GoogleTest 流程会执行 `TearDown()`，但构造中途发生非致命之外的 C++ 异常时，带自定义 deleter 的 `unique_ptr` 会比手写清理更稳健。是否重构应等第二个生产消费者出现或异常清理成为真实问题，不在本阶段扩范围。
4. **`moveit_kinematics` 当前没有直接被代码调用**（F 类）：本阶段只需要 `moveit_core`；它是按 Stage M/N 后续插件路径预装的环境依赖，不应误写成 `mujoco_bridge` 当前的直接 `package.xml` 依赖。

### 8.10 本阶段边界与后续

Stage L 至此结束：运动学三方一致性、门槛、`hand_tcp` 方案和碰撞几何事实已经落地。没有新增 MoveIt 碰撞运行测试、没有启动 `move_group`，也没有开始 IK。下一阶段 Stage M 只在 `arm_kinematics` 中实现 DLS differential IK、奇异性观测、限位与零空间处理；Stage L 的三方门禁将作为其模型前提持续回归。

## 9. Stage M：damped least-squares diff-IK 与奇异性观测

### 9.0 一句话总结

Stage M 在只依赖 Eigen 的 `arm_kinematics` 中加入了加权 damped least-squares differential IK。`differentialIkStep()` 负责单步求解、误差裁剪、阻尼、关节中心化和限位投影；`solveIk()` 反复调用单步求解器，作为离线位姿 IK 使用。实现没有显式构造矩阵逆，而是用 LDLT 分解解线性方程。新增 8 个 gtest、奇异性扫线工具和 CSV/PNG 产物；最终全仓 5 个包构建成功，249 项测试 0 失败。

### 9.1 改动清单与验证结果

| 改动 | 内容 |
|---|---|
| 求解接口 | [`differential_ik.hpp`](../../src/arm_kinematics/include/arm_kinematics/differential_ik.hpp)：`TaskVector`、参数、单步结果、IK 状态和最终结果 |
| 求解实现 | [`differential_ik.cpp`](../../src/arm_kinematics/src/differential_ik.cpp)：加权 DLS、最小奇异值、阻尼、joint centering、误差/步长裁剪和限位投影 |
| 自动测试 | [`test_differential_ik.cpp`](../../src/arm_kinematics/test/test_differential_ik.cpp)：良态、奇异、限位、冗余、可达/不可达目标和非法输入 |
| 奇异性实验 | [`singularity_sweep.cpp`](../../src/arm_kinematics/tools/singularity_sweep.cpp) + [`plot_singularity_sweep.py`](../../scripts/plot_singularity_sweep.py) |
| 实验产物 | [`stage_m_singularity_sweep.csv`](../../results/stage_m_singularity_sweep.csv)、[`stage_m_singularity_sweep.png`](../../results/stage_m_singularity_sweep.png) |

最终实测：4 个可达目标分别在 5、6、6、7 次迭代内收敛；不可达 `(5, 5, 5)m` 目标在 80 次后返回 `kMaxIterations`，位置残差 `7.54997m`，状态和残差均有限。奇异性扫线 81 个样本中，加权 `sigma_min` 最低约 `3.86e-4`，条件数峰值约 `1649`，自适应阻尼最高约 `0.05`，固定 `1mm` 笛卡尔命令产生的最大单步关节增量低于 `0.0032rad`。

单元测试按风险分层，而不是只测一个成功案例：

| 测试层 | 代表测试 | 主要验证 |
|---|---|---|
| 公式约定 | `PoseError` | world frame、平移差、轴角旋转误差和分量顺序 |
| 良态单步 | 人工构造单位 Jacobian | DLS 公式、矩阵维度、主任务跟踪 |
| 奇异保护 | 最后一行缩放到 `1e-9` | `sigma_min` 变小、阻尼增大、`delta_q` 仍有界 |
| 约束行为 | 大误差、接近上限的关节 | 误差裁剪、单步限制、关节限位投影 |
| 冗余处理 | 第 7 关节作为冗余方向 | joint centering 确实沿近似零空间起作用 |
| 完整可达 IK | FK 生成的 4 个目标 | 多轮迭代收敛、误差达标、结果仍在限位内 |
| 不可达/非法输入 | `(5,5,5)m`、NaN seed、越界 seed | 有限失败、最大迭代退出、输入拒绝 |
| 错误注入 | 强制 `lambda=0` | 奇异单测必须变红，证明测试能捕获关键退化 |

### 9.2 加权 DLS、误差向量与数值求解

> 加权 DLS 是什么，公式里每一项是什么意思？`e_w` 是世界坐标系路径点之间的差、代表世界坐标速度吗？当前方法实际上就是矩阵逆的解析解吗？

当前任务向量采用 world frame 和 `[vx, vy, vz, wx, wy, wz]` 的顺序。对位姿 IK 来说，误差写成：

$$
e = \begin{bmatrix}
\Delta x & \Delta y & \Delta z & r_x & r_y & r_z
\end{bmatrix}^T
$$

前三项是目标 TCP 与当前 TCP 的世界坐标位置差，后三项是相对旋转的轴角向量。它是**位姿误差**，不是速度；只有除以控制周期 `dt` 后，才可以近似解释为期望空间速度：

```cpp
// Offline IK: e is a pose error.
TaskVector e = poseError(current_tcp, target_tcp);

// Online servo would need an explicit time scale.
TaskVector desired_twist = e / dt;
```

因为米和弧度的数值尺度不同，代码显式构造权重矩阵：

$$
W=\operatorname{diag}(w_t,w_t,w_t,w_r,w_r,w_r),\qquad
e_w=We,\qquad J_w=WJ
$$

对应实现是：

```cpp
const Eigen::DiagonalMatrix<double, 6> weights = taskWeights(parameters);
const Jacobian weighted_jacobian = weights * geometric_jacobian;
const TaskVector weighted_error = weights * bounded_error;
```

加权 DLS 单步公式是：

$$
\Delta q = J_w^T(J_wJ_w^T+\lambda^2I)^{-1}e_w
$$

它等价于带 Tikhonov 正则项的最小二乘问题：

$$
\arg\min_{\Delta q}
\left\|J_w\Delta q-e_w\right\|^2
 +\lambda^2\left\|\Delta q\right\|^2
$$

这里 `J` 是 `6x7`，因为 Panda 有 7 个关节、末端任务有 6 个自由度。数学表达式确实是这个优化问题的闭式解，但工程代码不显式调用 `inverse()`：

```cpp
TaskMatrix normal = weighted_jacobian * weighted_jacobian.transpose();
normal.diagonal().array() += damping * damping;
const Eigen::LDLT<TaskMatrix> factorization(normal);
const auto damped_inverse =
  weighted_jacobian.transpose() * factorization.solve(TaskMatrix::Identity());
result.delta_q = damped_inverse * weighted_error;
```

所以应准确表述为“使用解析形式、通过 LDLT 数值分解求解”，而不是“显式计算矩阵逆”。显式 `A.inverse()` 更容易放大病态数值误差，也没有必要。

### 9.3 阻尼、奇异值与奇异性诊断

> 阻尼是哪来的？为什么可以和最小奇异值自适应？为什么要诊断奇异值？

没有阻尼时，伪逆在奇异值方向上的增益近似为 `1/sigma`。`sigma` 越小，末端的小误差和噪声越容易被放大成巨大的关节增量。DLS 把增益改为：

$$
\frac{\sigma}{\sigma^2+\lambda^2}
$$

这来自目标函数中的正则项 `lambda^2 ||Delta q||^2`，不是任意添加的安全常数。代码根据加权 Jacobian 的最小奇异值选择阻尼：

```cpp
if (minimum_singular_value >= parameters.damping_threshold) {
  return 0.0;
}
const double ratio = minimum_singular_value / parameters.damping_threshold;
return parameters.maximum_damping * (1.0 - ratio * ratio);
```

远离奇异区时保持准确跟踪；接近奇异区时牺牲一部分跟踪误差，换取有界的 `delta_q`。诊断 `sigma_min` 是为了区分“目标不可达”“当前构型病态”和“执行器没跟上”，并为日志中的阻尼、条件数和关节步长提供因果线索。注意：`sigma_min` 依赖 Jacobian 的米/弧度权重，因此不是脱离权重约定的绝对构型属性。

#### 9.3.1 奇异性扫线的读法

> 扫奇异性曲线的原理是什么，横轴是什么意思，条件数的解释以及它的变化趋势能够说明什么，`error/step` 那一张图表示的是什么？

扫线工具不启动 MuJoCo，也不运行 position servo；它沿一条预先指定的关节构型路径逐点计算 Jacobian：

```cpp
const double fraction = static_cast<double>(sample) / (kSamples - 1U);
const JointVector q = (1.0 - fraction) * start_q + fraction * near_singular_q;
const Jacobian geometric_jacobian = jacobian(model, q);
```

因此横轴 `fraction` 是**关节构型路径的归一化进度**，不是时间，也不是 TCP 距离：`0` 是 `start_q`，`1` 是预先选定的 `near_singular_q`，中间值是两组关节角的线性插值。

对加权 Jacobian 做 SVD：

$$
J_w=U\Sigma V^T,\qquad
\kappa(J_w)=\frac{\sigma_{\max}}{\sigma_{\min}}
$$

条件数表示不同笛卡尔方向的可控性差异：`sigma_min` 越小，至少有一个方向越难由关节运动产生；条件数越大，Jacobian 越病态，误差和噪声越容易被放大成关节动作。曲线中的尖峰说明这条指定路径经过局部病态构型；条件数随后下降则表示离开该构型后局部可控性恢复。这个结论只针对这条路径和当前的米/弧度权重，不能推广成所有路径的绝对阈值。

第三张 `error/step` 图使用一个固定的 `1mm` 世界 `x` 方向任务：

```cpp
const TaskVector cartesian_command =
  (TaskVector() << 0.001, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
const DifferentialIkStep step = differentialIkStep(model, q, J, cartesian_command, parameters);
const double tracking_error = (J * step.delta_q - cartesian_command).norm();
```

图中的 `error / step` 只是“误差与步长画在同一张图”的简写，**不是 error 除以 step**。

蓝线 `linearized tracking error` 计算的是 `||J delta_q - e_command||`。它表示把求出的 `delta_q` 代回一阶 Jacobian 模型后，与请求的 `1mm` 任务之间还剩多少残差。远离奇异区且 `lambda=0` 时，满秩 Jacobian 几乎能精确实现该命令，所以蓝线接近机器精度；进入阻尼区后，DLS 主动放弃部分难以稳定实现的任务分量，因此蓝线上升。它是 6 维线性化任务残差的范数，其中平移分量是 m、旋转分量是 rad，只能作为当前约定下的诊断量。它也不是实际 MuJoCo TCP tracking error，因为实验没有包含重力、执行器饱和、position servo 或时间推进。

橙线 `max |dq| [rad]` 计算的是 `max_i |delta_q_i|`，也就是 7 个关节增量中绝对值最大的一个，不是 7 个关节之和，也不是末端误差。对同一个 `1mm` 命令，它表示这一步中最忙的关节需要转多少。接近奇异区时橙线没有爆炸、甚至下降，是因为增大的阻尼宁愿少完成任务，也不产生巨大的关节动作；这不表示奇异区反而更容易运动。

两条线画在同一个对数纵轴面板，是为了一起观察 DLS 的稳定性权衡：接近奇异区时，蓝线所代表的局部任务残差增加，而橙线所代表的最大关节步长仍然有界。两条线单位不同，不能直接比较数值大小。

### 9.4 关节中心化项与冗余自由度

> 关节中心化项是直接加到 `delta q` 上吗？

是，但它先经过近似零空间投影：

$$
\Delta q = J^\#e_w +
\alpha(I-J^\#J_w)v_\text{center}
$$

代码对应：

```cpp
const auto nullspace =
  Eigen::Matrix<double, kArmDof, kArmDof>::Identity() -
  damped_inverse * weighted_jacobian;
result.delta_q = damped_inverse * weighted_error;
result.delta_q += parameters.joint_centering_gain *
  nullspace * centeringDirection(model, q);
```

`v_center` 指向各关节的中间位置。Panda 是 7 自由度，而任务空间是 6 维，因此存在一个冗余方向可以用于远离限位。由于这里使用的是阻尼逆而非严格伪逆，阻尼较大时投影只是近似零空间，中心化项会有少量主任务泄漏；随后还会经过步长裁剪和限位投影。

#### 9.4.1 一维冗余不等于只作用于一个关节

> 关节冗余是可以真正反映到某个关节上的吗？零空间投影的关节居中最后只能作用在一个关节上吗？我以为冗余是一个抽象维度，最后可以同时对所有关节产生一定效果。

后一种理解才是一般情况。Panda 的 Jacobian 是 `6x7`；在满秩构型下：

$$
\dim\mathcal{N}(J)=7-\operatorname{rank}(J)=1
$$

“一维”表示只有一个独立冗余运动方向，不表示只有一个关节能动。这个方向是一个 7 维关节向量：

$$
n=\begin{bmatrix}n_1&n_2&n_3&n_4&n_5&n_6&n_7\end{bmatrix}^T,
\qquad Jn=0
$$

沿 `n` 运动时，一阶近似的末端变化为零，但 `n` 的多个关节分量通常同时非零。因此真实 Panda 的 joint centering 一般会协调改变多个关节：

```cpp
JointVector preferred = centeringDirection(model, q);  // 7 个关节各自想回中点
JointVector projected = nullspace * preferred;          // 保留尽量不动 TCP 的组合
result.delta_q += joint_centering_gain * projected;
```

单元测试中看起来只有 joint7 运动，是因为测试刻意构造 `J=[I6 | 0]`。这个人工模型的零空间恰好是 `[0,0,0,0,0,0,1]^T`，方便给出明确断言；它不是生产 Panda Jacobian 的结构。接近奇异构型时 `rank(J)` 还可能下降，零空间维数会超过一维；当前阻尼投影又只是近似零空间，因此中心化可能轻微改变 TCP。

### 9.5 裁剪标准与关节限位

> 你的裁剪标准是如何确定的？关节限位指的是什么？

关节限位是每个关节角允许的范围：

$$
q_i^\text{lower}\le q_i\le q_i^\text{upper}
$$

`lower`/`upper` 是从官方 `joint_limits.yaml` 读取的实际角度上下限，**不是** `1e-4`。`joint_limit_margin=1e-4rad` 只是把有效区间稍微内缩，避免浮点数恰好落在边界：

```cpp
const double lower = model.joints[i].limits.lower + parameters.joint_limit_margin;
const double upper = model.joints[i].limits.upper - parameters.joint_limit_margin;
const double projected = std::clamp(q(index) + result.delta_q(index), lower, upper);
result.delta_q(index) = projected - q(index);
```

三种裁剪各自对应不同理由：

| 裁剪 | 当前默认值 | 理由与边界 |
|---|---:|---|
| 平移误差范数 | `0.05m` | 保持 FK/Jacobian 的局部线性近似；工程初值，需在 Stage N 重测 |
| 旋转误差范数 | `0.2rad` | 同上，避免单步跨过太大的姿态变化 |
| 单关节步长 | `0.12rad` | 抑制奇异区和大误差下的巨大关节跳变 |
| 限位 margin | `1e-4rad` | 数值边界保护，不是机械安全距离 |

这些数目前证明了“行为有界且测试可重复”，还没有证明“对真实伺服最优”。Stage N 接入 position servo 后必须重新测量。位置限位也不等于速度、加速度、力矩或碰撞约束；本阶段只实现位置限位和单步裁剪。

### 9.6 最小二乘 IK 的计划边界

> 计划中有实现最小二乘 IK 的部分吗？以后还会做吗？

当前 `solveIk()` 已经是最小二乘 IK：它反复计算位姿误差、Jacobian 和 DLS 单步，直到收敛。计划书和本周 Stage M 都要求解释 DLS 与正则化最小二乘的等价关系。

尚未实现的是“把约束直接放进优化器”的版本。当前流程是：

```text
先求 DLS 最小二乘解 -> 再裁剪 delta_q -> 再投影到关节限位
```

计划书 5.4 的 stretch 项才是：

```text
把速度/位置约束直接放进 QP 或其他约束最小二乘求解器
```

后续 Chap 6 会比较自写 DLS、MoveIt IK/规划器和碰撞筛选；不预设生产系统必须继续手写完整约束优化器。

#### 9.6.1 面试辨析：Jacobian 与最小二乘 IK 的区别

> 面试会问 Jacobian 和最小二乘 IK 的区别。是否可以回答二者没有本质区别，只是后者能把约束显式加入 QP？

不能回答“没有本质区别”，因为两者不在同一层级：

| 概念 | 定位 | 典型表达 |
|---|---|---|
| Jacobian | 当前构型附近的局部微分模型/工具 | `Delta x approximately equals J(q) Delta q` |
| 最小二乘 IK | 使用局部模型求关节增量的一类求解方法 | `min ||J Delta q - e||^2` |
| DLS | 带关节增量正则项的最小二乘 IK | `min ||J Delta q-e||^2 + lambda^2 ||Delta q||^2` |
| QP | 可在二次目标上显式加入线性约束的优化形式 | joint position/velocity bounds 等 |

无约束最小二乘 IK 已经成立，可以用伪逆、SVD、QR 或 DLS 求解，不需要 QP。只有需要把位置、速度等约束直接放进优化问题时，才进一步写成例如：

$$
\begin{aligned}
\min_{\Delta q}\quad &
\left\|J\Delta q-e\right\|^2+\lambda^2\left\|\Delta q\right\|^2\\
\text{s.t.}\quad &
\Delta q_{\min}\le\Delta q\le\Delta q_{\max}\\
&q_{\min}\le q+\Delta q\le q_{\max}
\end{aligned}
$$

面试中的精简回答可以是：

> Jacobian 不是 IK 算法，而是关节空间到任务空间的局部微分映射。最小二乘 IK 利用 Jacobian，把末端误差转化为 `min ||J Delta q-e||^2` 的优化问题。无约束时可用伪逆、SVD 或 DLS 求解；需要显式处理关节限位和速度等约束时，可以进一步写成 QP。由于 Jacobian 只是局部线性化，完整位姿 IK 通常还需要反复更新 Jacobian 并迭代收敛。

知识边界：IK 不一定依赖 Jacobian，解析 IK、几何 IK 等是其他路线；使用 Jacobian 的通常是数值迭代 IK 或 differential IK。因此应记成：

```text
Jacobian：模型/工具
最小二乘 IK：使用该模型的一类求解方法
QP：在二次目标上显式加入约束的一种求解形式
```

#### 9.6.2 `solveIk()`、在线反馈 diff-IK 与 MPC

> 当前的 `DifferentialIkStep` 是求解单步 IK。那么 `solveIk()` 呢？它一次直接求解到底吗？难道不应该使用 MPC 范式，每步读取反馈并调用 `differentialIkStep()`，也就是和 MuJoCo 联调吗？

`differentialIkStep()` 只完成一次局部线性更新；`solveIk()` 在一次函数调用内部反复执行 FK、位姿误差、Jacobian 和单步更新，直到收敛或失败：

```cpp
JointVector q = seed;
while (!converged) {
  const auto current = fk(model, q).hand_tcp;
  const TaskVector error = poseError(current, target);
  const Jacobian J = jacobian(model, q);
  q += differentialIkStep(model, q, J, error).delta_q;
}
return q;
```

所以它是“用 differential IK 迭代实现的离线 full-pose IK 求解器”。每轮会重新线性化，但状态反馈来自自写运动学模型预测的 `q`，不是 MuJoCo 或真机实际状态。它能证明模型中存在可收敛的关节解，不能修正 position servo 稳态误差、重力下垂、actuator 饱和、接触扰动或模型偏差。

在线反馈式 diff-IK 则应每个控制周期读取实际状态：

```cpp
while (running) {
  const JointVector q_actual = readJointState();
  const auto current = fk(model, q_actual).hand_tcp;
  const TaskVector error = poseError(current, target);
  const auto step = differentialIkStep(
    model, q_actual, jacobian(model, q_actual), error);
  sendJointCommand(q_actual + step.delta_q);
}
```

这通常称为 resolved-rate control、在线 differential IK 或 Cartesian servo。它虽然每轮反馈重算，但不是严格 MPC：MPC 还需要预测多个未来 timestep、联合优化一段控制序列并显式处理动态和约束，然后只执行第一个控制量。

| 对比 | 当前 `solveIk()` | 在线反馈 diff-IK |
|---|---|---|
| 状态来源 | 内部模型预测的 `q` | MuJoCo/真机实际 `q` |
| 输出 | 一个最终关节目标 | 每周期一个增量目标 |
| 现有 FSM | 兼容“发目标、等待到位” | 需要重新定义运动中、到位和超时 |
| 执行误差修正 | 不能 | 可以 |
| 新增要求 | 很少 | 反馈频率、延迟、稳定性和命令接口 |

Stage N 先选择离线 `solveIk()`，目的是只替换 waypoint 生成方式，保留 FSM 和 position servo，从而把运动学错误与控制错误分开定位。这个分层合理，但知识边界必须写清：Stage N 只能证明“模型中算得到并能作为离散目标执行”，不能宣称已经完成在线 differential IK 控制。后续 manipulation control 阶段需要增加反馈式 Cartesian servo，并比较实际 TCP 误差、奇异区行为和执行稳定性。

### 9.7 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现或防住 |
|---|---|---|
| 把位姿误差当速度使用 | 在线控制随周期改变而速度尺度错误 | 离线 `solveIk()` 不除以 `dt`；在线形态必须显式引入控制周期 |
| 米和弧度不加权 | 平移或旋转任务被数值尺度单方面支配 | 参数中显式保存两个权重；改变权重时同步解释 `sigma_min` |
| 奇异区不加阻尼 | `delta_q` 突然变成数弧度，测试或执行发生跳变 | 奇异单测；关闭阻尼的错误注入使测试失败 |
| 把 `joint_limit_margin` 当作关节下限 | 合法关节状态被错误拒绝，或有效区间缩成极小范围 | `withinJointLimits()` 使用官方 lower/upper；margin 只在更新投影时加入 |
| 先求解后裁剪造成路径偏差 | TCP 线性跟踪误差在限位附近变大 | 记录 `sigma_min`、tracking error、限位命中次数；完整约束求解列为 stretch |
| 把 `sigma_min` 当成无单位绝对阈值 | 换权重后错误地比较奇异性 | 日志记录权重约定；曲线只在同一权重下比较 |
| 不可达目标死循环或返回 NaN | 执行端收到非法关节命令 | `max_iterations`、停滞检测、有限性断言和不可达目标测试 |

### 9.8 排查记录：阻尼错误注入与实验路径修正

首次奇异性实验工具错误地在不同路径样本之间复用上一个样本的关节状态，导致样本结果相互污染。发现后改为每个样本从同一个 seed 开始，并将实验改为直接沿关节插值路径记录局部 Jacobian 响应，从而确保曲线确实经过预定的近奇异构型。

随后临时把 `adaptiveDamping()` 强制返回 `0.0` 做错误注入。奇异单测按预期变红：`delta_q.norm()` 从应小于 `1e-6` 膨胀到 `2.1815rad`。恢复阻尼实现后，Stage M 的 8 个 gtest、lint 和全仓测试全部通过。

### 9.9 你没问但值得注意的

1. **阻尼零空间不是严格零空间**（E/F 类）：当前实现用 DLS 逆近似 `J#`，阻尼越大，joint centering 对 TCP 主任务的泄漏越明显；需要后续单测量化这个泄漏上界。
2. **裁剪不是约束优化**（E 类）：当前方法保证 `q` 合法和步长有界，但不保证裁剪后的 `delta_q` 仍是最小二乘意义下的最佳解；QP 对照是明确的 stretch 项。
3. **奇异性扫线是运动学实验，不是执行实验**（C 类）：曲线记录的是固定精确 `q` 下的 Jacobian，不包含 position servo、重力和 actuator 饱和；Stage N 需要分别记录求解残差和实际末端残差。
4. **失败状态还可以细分**（C/E 类）：当前区分 `kConverged`、`kMaxIterations` 和 `kStalled`，尚未单独报告“限位阻塞”和“奇异区阻塞”；接入 waypoint 日志后再判断是否需要扩展错误码。

### 9.10 本阶段边界与后续

Stage M 完成了加权 DLS differential IK、奇异性观测、误差/步长裁剪、关节限位投影和零空间 joint centering。Stage N 已将离线求解出的关节目标接入现有 waypoint 接口，并重新测量执行端稳态误差、settle 时间和放置位置。

## 10. Stage N：`DiffIkWaypointSource` 接入 FSM

### 10.0 一句话总结

Stage N 新增 `DiffIkWaypointSource`，默认把 `task_executor` 的固定关节查表替换成基于 `arm_kinematics::solveIk()` 的离线 waypoint 求解。物体在 `PREGRASP` 首次观测时锁定，抓取侧目标使用相对默认 box 中心的平移偏移；每个阶段只求解一次并缓存目标，FSM 的 `fsm.cpp` 保持不变。默认场景 20 次全部成功，物体沿 `+y` 移动 4cm 时 IK 成功而查表实现失败；实际放置仍受 position servo 重力下垂和接触误差限制。

### 10.1 改动清单与验证结果

| 改动 | 内容 |
|---|---|
| waypoint 实现 | [`diff_ik_waypoint_source.hpp`](../../src/task_executor/include/task_executor/diff_ik_waypoint_source.hpp)、[`diff_ik_waypoint_source.cpp`](../../src/task_executor/src/diff_ik_waypoint_source.cpp)：模型 FK、阶段目标构造、离线 IK、阶段缓存和诊断 |
| 节点接入 | [`task_executor_node.cpp`](../../src/task_executor/src/task_executor_node.cpp)：`waypoint_source` 参数、seed 设置、episode/retry 缓存重置、IK 失败处理和执行误差日志 |
| 构建依赖 | [`task_executor/CMakeLists.txt`](../../src/task_executor/CMakeLists.txt)、[`package.xml`](../../src/task_executor/package.xml)：链接 `arm_kinematics`，使用 `ament_index_cpp` 和 `franka_description` |
| 测试 | [`test_diff_ik_waypoint_source.cpp`](../../src/task_executor/test/test_diff_ik_waypoint_source.cpp)：物体平移、阶段缓存、episode 重新锁定和不可达目标 |
| 对照场景 | [`stage_n_shifted_scene.xml`](../../robot_description/mujoco/franka_emika_panda/stage_n_shifted_scene.xml)：box 的 y 坐标从 0 移到 0.04m |
| 实验结果 | [`stage_n_fixed_20.csv`](../../results/stage_n_fixed_20.csv)、[`stage_n_shifted_ik.csv`](../../results/stage_n_shifted_ik.csv)、[`stage_n_shifted_keyframe.csv`](../../results/stage_n_shifted_keyframe.csv)、[`stage_n_strict_place.csv`](../../results/stage_n_strict_place.csv) |

实测命令和结果：

| 验证 | 结果 |
|---|---|
| `colcon build --symlink-install` | 5 个包成功 |
| `colcon test` | 265 tests，0 failures，39 skipped（lint/测试框架的 skipped 项保留） |
| 默认固定场景，`DiffIkWaypointSource` | 20/20 成功，0 重试；episode 时长约 6.65--6.75s |
| box y=0.04m，IK | 1/1 成功，6.68s |
| 同一偏移场景，`waypoint_source:=keyframe` | 0/1；重试 3 次后 `GRASP_EMPTY` |
| marker 中心 `(0.5, 0.3)`，验收半径 0.03m | 0/1；重试 3 次后 `PLACE_MISSED` |

### 10.2 waypoint 的构造和缓存

`DiffIkWaypointSource` 没有直接把 object pose 当作完整抓取姿态。它先使用 `KeyframeWaypointSource` 的阶段关节构型作为姿态参考，通过自写 FK 得到对应 `hand_tcp` 位姿，再把抓取侧阶段相对默认 box 中心 `(0.5, 0, 0.241)m` 的平移偏移加入 TCP 目标。当前受支持的是物体平移；物体偏航角没有改变夹爪旋转，也没有做候选抓取或碰撞规划。

`PREGRASP` 第一次调用时保存 `grasp_object_pose_`。`PREGRASP`、`GRASP`、`CLOSE`、`LIFT` 使用这个锁定值；`PREPLACE`、`PLACE`、`OPEN`、`RETRACT` 使用固定放置侧参考目标。阶段第一次调用 `solveIk()`，结果写入 `cached_target_`；同一阶段后续 tick 返回缓存。episode 开始或恢复重试时清除缓存并重新设置 seed，避免上一轮的物体位置或关节目标泄漏。

这个缓存不是为了优化而优化。若每个 20Hz tick 都使用最新观测重新求解，接触造成的 box 微动和观测噪声会让关节命令持续变化，FSM 的“发目标、等待到位”语义也会变成追逐移动目标。另一方面，锁定物体位姿意味着物体在抓取后发生较大滑移时不会自动纠正；这正是后续在线反馈控制需要解决的边界。

### 10.3 为什么 Stage N 先使用离线 IK

当前链路是：

```text
实测关节 seed + TCP 目标
        -> solveIk() 内部反复 FK/Jacobian/DLS
        -> 一个最终关节目标
        -> 现有 MuJoCo position servo
```

这保持了现有 joint command 接口和 FSM 的阶段判据，能够单独验证“物体位置变化是否进入 waypoint 计算”。它不是在线 Cartesian servo：`solveIk()` 内部的反馈是自写模型中的迭代状态，不是 MuJoCo 实际 TCP 状态。

在线 differential IK 会在每个控制周期重新读取实际关节状态，再计算 FK、误差和增量。它可以修正伺服误差和外部扰动，但需要重新定义控制周期、延迟、速度/加速度限制、阶段到位和两层闭环的稳定性。MPC 还需要预测多个未来步并优化一段控制序列；“每 tick 调 `differentialIkStep()`”本身不是 MPC。当前选择离线形式是为了保持 FSM 和控制接口稳定，不代表在线控制已经完成。

### 10.4 诊断信号：模型残差和执行残差必须分开

每个阶段首次求解记录 world frame 下 TCP 目标位置、四元数、IK 迭代次数、模型位置/姿态残差和加权 `sigma_min`。阶段退出时额外从 `world -> hand_tcp` TF 计算实际 TCP 位置、姿态误差以及关节最大绝对误差。

这两个残差回答不同问题：

| 信号 | 回答的问题 |
|---|---|
| IK position/orientation error | 自写运动学模型能否找到目标解 |
| joint error | position servo 是否接近命令关节角 |
| actual TCP error | 执行后的末端是否真的到达目标 |
| `sigma_min` | 当前加权 Jacobian 是否接近病态构型 |

默认实验中，`GRASP -> CLOSE` 的 IK 模型残差接近零，但实际 TCP 平移误差约 `0.111m`、姿态误差约 `0.393rad`；`PLACE -> OPEN` 的实际 TCP 平移误差约 `0.006m`。这说明“求解器收敛”不能替代“执行器到位”。查表模式没有 IK 目标诊断，日志中的 TCP 误差会保持为 `NaN`，不伪造零值。

### 10.5 验收结果和失败边界

固定场景的 20/20 回归证明替换 waypoint 源没有破坏现有任务链。偏移场景的对照证明新能力来自 IK，而不是 FSM 逻辑：IK 能把物体的 y 偏移传到抓取 waypoint，查表仍然前往旧位置并最终 `GRASP_EMPTY`。

放置实验揭示了另一条边界。使用默认放置验收中心 `(0.43, 0.31)m` 和半径 `0.08m` 时任务成功，但最终 box 均值约为 `(0.43226, 0.30902)m`；把目标改为 marker 中心 `(0.5, 0.3)m` 且半径收紧到 `0.03m`，任务经过 3 次重试后返回 `PLACE_MISSED`。因此宽松成功不能解释成 marker 中心的精确放置，`place_x_m`/`place_y_m` 仍是当前执行系统的实测验收点。

### 10.6 失败模式与验证手段

| 失败模式 | 现象 | 验证或防护 |
|---|---|---|
| IK 不收敛 | 阶段输出 `IK_FAILED`，不发布该解 | 检查 `IkStatus`、迭代次数和有限残差；不可达目标 gtest |
| 阶段缓存未清理 | 新 episode 继承旧物体位置或目标 | `beginEpisode()` 单测和连续 episode 实跑 |
| 每 tick 追逐 object pose | 命令随接触微动抖动 | `PREGRASP` 锁定物体位置、阶段缓存 |
| 物体偏航未处理 | 平移场景成功，旋转物体可能抓取失败 | 当前明确记录为能力边界；后续增加偏航测试 |
| 奇异区步长放大 | 关节目标突然跳变 | 记录 `sigma_min`；复用 Stage M 阻尼和有界步长测试 |
| IK 模型收敛但 servo 跟不上 | 模型残差小，实际 TCP 误差大 | 同时记录模型残差、关节误差和 TF TCP 误差 |
| 放置半径过宽 | 偏离 marker 仍报告成功 | 用 0.03m 严格实验；实际返回 `PLACE_MISSED` |
| 查表源误用于偏移物体 | 机械臂仍去旧位置，抓取为空 | `waypoint_source:=keyframe` 对照实验 |

### 10.7 你没问但值得注意的

1. **物体姿态是否应进入抓取目标的旋转部分？** 当前只传播平移；物体偏航、夹爪方向和抓取对称性仍未验证，值得在第 4 周随机化前先定义验收范围。
2. **IK 目标是否需要碰撞检查？** 当前 `solveIk()` 只处理运动学和关节限位，不能保证从一个阶段到下一个阶段的路径不碰撞；MoveIt/规划器接入时要把 waypoint 可达和路径可行分开记录。
3. **seed 取当前关节状态是否总是合适？** 它有利于选择靠近当前构型的解，但在错误恢复或奇异区可能把求解带到不理想的局部结果；后续可比较固定 ready seed、上一阶段目标和多 seed 策略。
4. **`mutable` 缓存是否应该继续留在 `const` 查询接口里？** 当前是兼容既有 `WaypointSource` 契约的折中；如果将来 waypoint 源需要更复杂的生命周期，应该重新评估 `preparePhase()` 等非 `const` 接口，而不是继续扩大具体类型分支。

### 10.8 排查记录：构建、运行和严格放置验证

首次构建失败原因是 `arm_kinematics` 的导出目标名为 `arm_kinematics::arm_kinematics`，而 `task_executor` 只链接了未限定的 `arm_kinematics`，导致 Eigen 头文件路径没有通过 imported target 传递。改为链接导出目标后构建通过。

第一次真实启动时，executor 因消息类型支持库未在同一个已 source 的 workspace 环境中加载而退出；错误是找不到 `libmanipulation_interfaces__rosidl_typesupport_fastrtps_cpp.so`。随后统一在同一个 shell 中 source `/opt/ros/humble/setup.bash` 和 `install/setup.bash`，直接启动两个真实可执行文件，episode 成功。

新增测试最初只验证了功能，lint 还发现 include 顺序和 uncrustify 格式问题；修复后 `task_executor` 的 84 项测试全绿，最终全仓 265 项测试全绿。

最后用 marker 中心和 3cm 半径做严格验证，任务连续 4 次尝试均在 `VERIFY` 以 `PLACE_MISSED` 退出。日志显示 `PLACE` 阶段的模型 IK 残差约为微米级，但实际 TCP 仍有毫米级误差，且物体最终落点约为 `(0.428, 0.314)m`。这次实验把“IK 目标正确”和“物理执行达到严格放置标准”明确分开。

### 10.9 后续边界

Stage N 完成了离线 IK waypoint 接入、固定场景回归、平移物体对照、阶段诊断和执行误差测量。它没有完成在线 Cartesian servo、物体偏航抓取、碰撞规划或精确 marker 中心放置。随机物体批量泛化评测仍推迟到第 4 周，与 RGB-D 感知一起进行。

### 10.10 Stage N 设计复盘：Keyframe 反推 Cartesian 目标是不合理的过渡

> 现在 `tcpTargetFor()` 是不是把 `KeyframeWaypointSource` 的构型做了一次 FK？Keyframe 本来就是不精确的扫描结果，放置误差是不是大概率来自这里？

是的。当前 `tcpTargetFor()` 先取 `KeyframeWaypointSource` 的手调关节目标，再通过自写 FK 得到一个理想 `hand_tcp` 位姿，然后把这个位姿交给 `solveIk()`。这验证了“一个关节目标可以经过 FK/IK 往返”，但没有从真正的任务世界坐标定义抓取或放置目标。

Keyframe 中的数字不是纯几何数据，而是运行 position servo 后手工调出的经验命令，已经混入重力下垂、执行器饱和、接触和放置妥协。把它们再做理想 FK，会把执行端补偿误认为任务几何。实测证据是：Stage N 的 `PLACE` IK TCP 目标约为 `(0.434, 0.310)m`，而不是 marker 中心 `(0.5, 0.3)m`；最终 box 约落在 `(0.432, 0.309)m`。因此 6--7cm 的系统性放置偏移主要在目标构造阶段就已经存在，毫米级差异才是 position servo 和接触执行误差。

> `joint error` 的 ground truth 是不是 Keyframe waypoint？`IK position/orientation error` 又是什么？

不是。`joint error` 是 MuJoCo 实际 `/joint_states` 与当前源产生的关节命令之间的最大绝对差：`diff_ik` 模式比较实际状态和 IK 输出，`keyframe` 模式才比较实际状态和 Keyframe 命令。Keyframe 是命令/经验参考，不是关节 ground truth；仿真中的实际 `qpos` 才是当前执行状态。

`IK position/orientation error` 是自写模型内部的残差：比较 `fk(model, q_ik).hand_tcp` 和 `tcpTargetFor()` 生成的目标位姿。它不读取 MuJoCo 实际 TF、重力或 position servo，因此只说明数学模型找到了一个解，不能说明仿真中的末端已经到位。当前需要同时看三类量：IK 模型残差、关节跟踪误差、实际 TF TCP 误差。

> 现在的物体坐标仍然是 oracle 吗？加入深度相机后当前结构够不够 generic？MoveIt 的动态路径点能否直接兼容？

当前订阅的 `/mujoco_bridge/ground_truth/object_pose` 确实仍是 oracle。`WaypointSource` 的纯 C++ `ObjectPose` 参数与数据来源无关，因此视觉节点如果输出同样语义的 world-frame 位姿，下游数学代码可以复用；但当前数据契约还没有 frame id、时间戳、置信度、协方差和观测失败状态，不能称为完整的感知接口。相机外参和 camera-to-world 变换应由感知/TF 层处理，不能让 IK 源读取相机坐标。

MoveIt 通常最终输出带时间的 `JointTrajectory`，而当前接口只输出一个 `JointTarget`。MoveIt 可以组合 pose target、IK、碰撞检查和轨迹规划，但 Cartesian path、全局路径规划和轨迹执行不是同一个接口。因此随机障碍和 MoveIt 结果不能直接塞进当前 `WaypointSource`，需要规划器和轨迹执行适配层。

> 是否应该现在增加世界坐标轨迹抽象？

复盘结论是应该增加，但范围放在 Stage O：引入 message-free 的 `CartesianWaypoint`/`CartesianWaypointSource`，让任务层输出 world-frame `hand_tcp` 目标和夹爪命令；离线 IK、MoveIt 和在线 Cartesian servo 分别消费该契约。Stage O 将把 `DiffIkWaypointSource` 改成 Cartesian waypoint 到 JointTarget 的适配器，并去掉它对 `KeyframeWaypointSource` 的直接依赖。Keyframe 可以保留为旧版关节执行和回归对照，但不能再作为任务 Cartesian 目标的权威来源。

因此 Stage N 的结论被修正为：代码、构建、单测和回归实验有效，但设计验收不完整。Stage N 不能作为本周最终完成的标志；必须完成 Stage O 的 Cartesian 抓取/放置目标和离线 IK 适配，并重新通过固定场景回归后，本周工作才算完成。MoveIt 本身和随机障碍规划仍留到后续 stage。

### 10.11 用户否定 Stage N 后的正式决策记录

> Stage N 的设计实在不合理。Keyframe 里的点本来就是不精确的扫描结果；要等到 Stage O 的笛卡尔坐标 IK 抓取放置完成之后，这个 week 的工作才能够算正式完成。

这不是对 Stage N 实验结果的否定，而是对目标构造层的否定。Stage N 已经证明了离线 `solveIk()` 能接入现有 FSM，也证明了物体平移可以让 IK 产生不同关节目标；但它把手调的执行命令错误地当成了任务几何输入，因而不能证明抓取/放置目标在 world frame 中是正确的。放置目标已经在进入 IK 前偏离 marker 中心，继续调 DLS 参数或 position servo 不能修复这个架构错误。

因此本周验收状态明确改为：

1. Stage K/L/M 的数学库、模型一致性和 DLS 实验保持有效。
2. Stage N 作为“离线 IK 接入和失败模式暴露”保留，但不作为 Week 3 的最终完成标志。
3. Stage O 必须完成真实的 Cartesian 抓取/放置目标、离线 IK 适配和固定场景回归后，Week 3 才能收周。
4. 第 4 周的 RGB-D 和随机化评测建立在 Stage O 的 Cartesian 契约之上，不再从 Keyframe 反推目标。

Stage O 的具体计划：

| 步骤 | 计划交付 | 验收方式 |
|---|---|---|
| 1 | 定义 message-free `CartesianWaypoint`：阶段、world-frame `hand_tcp` pose、夹爪命令、必要的到位容差/目标有效性 | 单测检查 frame 约定、四元数和阶段夹爪时序 |
| 2 | 定义 `CartesianWaypointSource`，直接从任务物体位姿和抓取/放置几何构造目标 | 测试目标随物体 world 平移变化，放置目标直接等于指定 world 坐标 |
| 3 | 提取 `KeyframeCartesianWaypointSource` 作为过渡几何源，但不再让它充当关节命令的 FK 反推结果 | 检查 `DiffIkWaypointSource` 不再依赖 `KeyframeWaypointSource` 的 joint target |
| 4 | 将离线 IK 改成 Cartesian waypoint 的执行适配器，输出现有 `JointTarget` | 固定场景 20 次回归、偏移物体成功、严格放置目标误差可解释 |
| 5 | 保留 `waypoint_source:=keyframe` 作为旧版 joint-space 回归入口 | 旧实现仍能单独运行，但不参与 Cartesian 目标正确性的证明 |
| 6 | 记录模型 IK 残差、关节跟踪误差、实际 TCP 误差和任务放置误差四个层次 | 日志和 CSV 能区分目标构造、IK、servo、接触四类问题 |

Stage O 暂不包含 MoveIt planner、随机障碍、完整 `JointTrajectory` 执行器或在线 Cartesian servo。它先建立正确的 Cartesian 任务契约；后续后端可以分别接离线 IK、MoveIt 或在线控制，而不会再次修改任务几何层。

---

## 11. Stage O：Cartesian 任务几何与离线 IK 适配

### 11.0 一句话总结

任务层现在直接给出 world-frame `hand_tcp` 位姿和夹爪命令，`DiffIkWaypointSource` 消费这个目标求关节解。放置 XY 默认就是 marker 中心 `(0.5, 0.3)m`，不再从手调关节关键帧反推目标。初版仍把 `atan2(y, x)` 偷放进姿态构造，用户指出后改成显式的固定工具朝向，并重新完成 20 次严格放置回归。当前结果证明固定场景的抓取和位置放置可用；尚未根据物体旋转生成抓取姿态，也不验收最终物体朝向。

### 11.1 改动清单与验证结果

| 文件 | 本阶段改动 |
|---|---|
| [cartesian_waypoint_source.hpp](../../src/task_executor/include/task_executor/cartesian_waypoint_source.hpp)、[实现](../../src/task_executor/src/cartesian_waypoint_source.cpp) | 新增 `CartesianWaypoint`、`CartesianWaypointSource`、`PickPlaceGeometry` 和直接构造任务几何的实现；校验有限数值及单位四元数 |
| [diff_ik_waypoint_source.cpp](../../src/task_executor/src/diff_ik_waypoint_source.cpp) | 注入 Cartesian source，保留实测关节 seed、首次 PREGRASP 锁定物体观测和阶段内缓存；移除 Keyframe 依赖 |
| [task_executor_node.cpp](../../src/task_executor/src/task_executor_node.cpp) | 接入几何参数、按模式设定验收中心、检查输入 frame、记录分层误差，等待 reset 完成再继续 |
| [EpisodeOutcome.msg](../../src/manipulation_interfaces/msg/EpisodeOutcome.msg)、[episode_runner.py](../../scripts/episode_runner.py) | 将每阶段 TCP 目标、模型 IK 残差、关节跟踪误差、实际 TCP 误差和最终放置误差写进 CSV |
| [Cartesian 测试](../../src/task_executor/test/test_cartesian_waypoint_source.cpp)、[适配器测试](../../src/task_executor/test/test_diff_ik_waypoint_source.cpp) | 检查平移响应、姿态独立、绝对放置目标、夹爪时序、非法输入与适配器行为 |
| [ADR 001](../../docs/adr/001-cartesian-task-waypoints.md)、[architecture 第 10 节](../../docs/architecture.md#10-stage-o-cartesian-任务目标与离线-ik-适配) | 同步目标归属、姿态约定、参数边界和实测事实 |

计划中的 `KeyframeCartesianWaypointSource` 最终命名为 `PickPlaceCartesianWaypointSource`，准确表达其输入是任务几何。`CartesianWaypoint` 当前包含 phase、pose 和夹爪宽度；到位容差仍在 `FsmParams`，有效性通过输入检查和异常报告，未新增逐 waypoint 容差字段。

姿态修正后完成 `colcon build --packages-select task_executor` 和 `colcon test --packages-select task_executor`。随后查询工作区已有测试结果：

```text
colcon test-result --all
Summary: 283 tests, 0 errors, 0 failures, 42 skipped
```

这包含其他包此前留下的结果，不表示此次重新执行了全仓测试；42 个 skipped 来自 cppcheck 结果。`task_executor` 的 Cartesian source 有 4 个 gtest，IK adapter 有 4 个 gtest，均通过。姿态参数 `0.25rad` 的单测证明目标方向可以独立于位置变化，不代表该角度已通过完整抓取实验。

| 实验 | 条件 | 实测结果 | 原始记录 |
|---|---|---|---|
| 固定场景 IK | 修正后的固定工具方向；`verify.place_region_radius_m:=0.03` | **20/20 成功，零重试**；最终 box XY 均值 `(0.49742, 0.29532)m`；落点误差均值 **5.35mm**、最大 **6.07mm** | [stage_o_fixed_20.csv](../../results/stage_o_fixed_20.csv) |
| box 沿 +y 偏移 4cm，IK | bridge 加载 `stage_n_shifted_scene.xml`；验收半径 3cm | **1/1 成功，零重试**；落点误差 **5.60mm** | [stage_o_shifted_ik.csv](../../results/stage_o_shifted_ik.csv) |
| 固定场景 keyframe | 旧模式默认验收中心 `(0.43, 0.31)m` | **1/1 成功，零重试** | [stage_o_fixed_keyframe.csv](../../results/stage_o_fixed_keyframe.csv) |
| 偏移场景 keyframe | 与偏移 IK 相同的初始 box 位置 | **0/1 成功，三次重试，`GRASP_EMPTY`** | [stage_o_shifted_keyframe.csv](../../results/stage_o_shifted_keyframe.csv) |

固定 IK 的 PLACE 阶段：模型 IK 位置残差约 **8µm**；每次取最大关节绝对误差后求均值，得到 **0.00666rad**；实际 TCP 位置误差均值 **6.82mm**。CSV 中误差值保留 6 位小数，8µm 是该精度下的统计。实际 TCP 来自 bridge 发布的 TF；IK 残差来自 FK 计算，两者不能互相替代。

复现实验时先确认无遗留 bridge，再分别启动安装目录中的 bridge 和 executor 真实可执行文件，加载 ROS 环境和工作区 `install/setup.bash`。固定 IK 的关键参数与 runner 命令为：

```bash
./install/task_executor/lib/task_executor/task_executor_node --ros-args \
  -p use_sim_time:=true -p waypoint_source:=diff_ik \
  -p target.tool_yaw_rad:=0.0 -p verify.place_region_radius_m:=0.03
# 另一个已加载同样环境的终端：
/usr/bin/python3 scripts/episode_runner.py --episodes 20 \
  --out results/stage_o_fixed_20.csv
```

如果只是想看 MuJoCo 正在运行的实时渲染窗口，可在图形会话中执行：

```bash
distrobox enter robotics-dev -- bash -ic '
  source /opt/ros/humble/setup.bash
  cd /home/anby/robotics/robotic_manipulation_practice
  source install/setup.bash
  ros2 launch mujoco_bridge demo.launch.py \
    enable_debug_viewer:=true debug_viewer_rate_hz:=5.0
'
```

这会打开 `mujoco_bridge debug viewer` GLFW 窗口，并同时启动现有 demo 的 executor 和 RViz；窗口直接读取 bridge 进程正在步进的那份 `mjData`，所以显示的是当前仿真状态，不是另开一份物理仿真。`demo.launch.py` 已为 bridge 和 RViz 设置 `LIBGL_ALWAYS_SOFTWARE=1`，适配当前容器的 X11/GL 转发环境；关闭时在启动终端按 `Ctrl-C`。

偏移实验仅将 bridge 的 `model_path` 指向 [stage_n_shifted_scene.xml](../../robot_description/mujoco/franka_emika_panda/stage_n_shifted_scene.xml)，runner 次数改为 1。keyframe 对照切换 executor 的 `waypoint_source:=keyframe` 并使用其旧默认验收参数，因此固定 keyframe 成功不代表通过了 marker 中心的 3cm 验收。验证后已清理启动的进程，复查无遗留 bridge。

### 11.2 目标怎样从任务几何进入 IK

调用链是 `ObjectPose + PickPlaceGeometry -> CartesianWaypoint -> solveIk(seed, pose) -> JointTarget`。`world_to_hand_tcp` 的含义是将 TCP 坐标转换到 world；它指定机械臂应达到的工具位姿。几何源不读取关节关键帧，也不通过 FK 获取任务目标。

| 阶段 | TCP 平移目标 | 夹爪总开口 |
|---|---|---|
| HOME | 固定安全点 `(0.5545, 0, 0.5211)m` | 0.08m |
| PREGRASP | 物体中心上方 0.15m | 0.08m |
| GRASP / CLOSE | 物体中心 | GRASP 为 0.08m，CLOSE 为 0 |
| LIFT | 锁定的物体中心上方 0.15m | 0 |
| PREPLACE / RETRACT | 放置 XY，桌面上表面 + box 半高 + 0.15m，即 z=0.39m | PREPLACE 为 0，RETRACT 为 0.08m |
| PLACE / OPEN | 放置 XY，桌面上表面 + box 半高 + 0.05m，即 z=0.29m | PLACE 为 0，OPEN 为 0.08m |

PREGRASP 首次锁定物体观测后，GRASP/CLOSE/LIFT 使用同一位置；否则 box 被抬起时，LIFT 目标也随观测上升，就会一直追逐更高的目标。每个阶段首次求 IK，阶段内复用缓存；reset 或 retry 清除缓存。物体中心作为抓取 TCP、释放时高出中心 5cm、固定安全点和固定朝向，都仍是这个场景的显式任务约定。

### 11.3 工具朝向不能从目标的径向位置推出

> 1. downwardPose函数是否隐藏了任务约定？凭什么直接用y和x来计算yaw？我们的box放置姿态未必是和原点径向吧？虽然我知道这种约定在这里都是oracle，会在后面的week被实际视觉和抓取的模块替换掉。

是，初版在工具函数里隐藏了未经任务定义支持的约定。`atan2(y, x)` 计算的是目标相对 world 原点的方位角；位置本身不决定物体朝向、夹爪闭合方向或期望放置姿态。抓取点接近 `(0.5, 0)` 时这个角度为 0，而放置点 `(0.5, 0.3)` 会变成约 `0.5404rad`，于是在搬运中额外要求工具转动约 31°。这既不是 IK 推出的约束，也不是 oracle 位姿数据所要求的动作。

已将姿态改成独立的 `PickPlaceGeometry::tool_yaw_rad`，由启动参数 `target.tool_yaw_rad` 覆盖，默认 0。构造公式是：

```text
R_world_tcp = Rz(tool_yaw_rad) * R_down
R_down = Rotation(pi, normalize([1, 1, 0]))
```

`R_down` 把 TCP 的 z 轴指向 world 的 -z，TCP 的 x/y 分别指向 world 的 +y/+x。因而参数为 0 表示这个基准朝下姿态；**不表示 TCP x 轴指向 world +x，也不是标准 ZYX Euler yaw 等于 0**。参数表示绕 world z 轴对该基准姿态附加的旋转。抓取与放置使用同一参数，不因 XY 改变而转动工具。

固定朝向是当前轴对齐方形 box 的任务假设，绝不是任意 box 姿态的解法。更一般的目标应由物体位姿和选定的物体到工具抓取变换组成：`T_world_tcp = T_world_object * T_object_tcp`；若指定最终物体朝向，还要结合抓取后保持的相对变换构造放置 TCP。视觉负责估计物体位姿，抓取生成负责选择相对变换，放置任务负责给出目标物体位姿，三者各自有明确输入。

当前代码虽检查物体四元数是否有效，**并未用它计算工具朝向**。最终 VERIFY 也没有物体姿态误差判据，所以 20/20 成功不能证明“箱体按要求的朝向放好了”。对应回归已断言：只平移物体时工具旋转矩阵不变；相同配置的抓取与放置方向一致；显式改变 `tool_yaw_rad` 才改变方向。

### 11.4 MJCF 初始位置、TCP 目标与验收参数的归属

> 2. 现在box的位置是hardcode在xml文件里的吗？我注意到task executor的胶水代码里还可以通过declare\_param来修改？

**初始位姿和 reset 位姿是 MJCF 里的固定值；仿真运行后，实际位姿由动力学更新。** [pick_place_scene.xml](../../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml) 中 `<body name="box" pos="0.5 0 0.241">` 给出初始位置；`pick_place_home` keyframe 的 `qpos` 最后 7 项 `0.5 0 0.241 1 0 0 0` 给出 reset 时的平移和 wxyz 四元数。episode 开始会请求 reset，因此只改 body、漏改 keyframe，会在 reset 后回到旧位置。偏移实验的 XML 已把两处都改为 `y=0.04`。

| 配置入口 | 实际控制什么 | 不会自动发生的事 |
|---|---|---|
| bridge 的 `model_path` | 加载哪份 MJCF 场景 | 不会读取 executor 的几何参数 |
| MJCF box body / reset keyframe | 初始 / reset 后的真实 box 位姿 | 不会改变放置 TCP 目标 |
| `target.place_x_m/y_m` | IK 模式下放置 TCP 的 world XY | 不会移动初始 box 或 XML 中的 marker |
| `target.tool_yaw_rad` | 工具的固定朝向偏置 | 不会直接设置物体姿态 |
| `verify.place_x_m/y_m` | FSM 判断最终 box 是否落入目标圆的圆心 | 不会改变机械臂指令 |
| `verify.place_region_radius_m` | 验收圆半径，默认 0.08m；严格实验覆盖为 0.03m | 不会改变夹爪运动 |

`declare_parameter(name, default)` 声明参数并返回启动覆盖值或默认值，随后代码把它拷贝到 `geometry` / `params_` 中。当前无参数更新回调，运行中 `ros2 param set` 即使更新了参数服务器中的数值，也不会更新已构造的几何源或 FSM 参数。应在启动时通过 `--ros-args -p ...` 覆盖。

例如要改放置位置，需在启动时同时配置 `target.place_x_m/y_m` 和 `verify.place_x_m/y_m`；两组值目前互不联动。marker 是独立 XML 几何体，如需可视化标记同步，还要改 marker。`waypoint_source:=keyframe` 不消费 `target.*`，仍使用旧关节目标。

另一个观测细节：runner 的 `place_error_m` 是最终 box XY 到所记录 PLACE TCP XY 的距离；FSM 成功标志和节点 `final placement` 日志使用 `verify.*`。本次 IK 实验两组 XY 都是 `(0.5, 0.3)`，所以度量相同；若以后分开配置，就不能把两者当成同一个误差。keyframe 无 Cartesian 目标，相关诊断为 NaN，CSV 的 `place_error_m` 留空。

### 11.5 讲解内容的删减与流程修正

> 3. 你的讲解里，“设计权衡”的部分全是以前有的内容，所以去掉这块... “为什么没有直接把 FSM 改成 Cartesian”也感觉挺多余的。

删除这两块重复讲解。此次需要讲清的是新增几何源到底承诺什么，以及把位置错误地当成朝向依据的问题；旧的纯函数分层、离线 IK 和 FSM 讨论已有记录，不为套固定骨架再讲一遍。姿态选择的依据、替代的物体相对抓取变换及其验证边界已放在 11.3。

用户此前也指出“流程不对。太简略了”，要求参考 Week 2 和本周先前章节。修正后的记录应包含具体目标构造、代码与数据链接、原始问题、错误结论的更正和验证限制，不能只留下抽象接口清单或一句“测试通过”。

### 11.6 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现或防住；当前证据 |
|---|---|---|
| 用 `atan2(y,x)` 生成工具朝向 | 移动物体或目标就无端改变工具旋转；成功位置验收掩盖姿态约定错误 | 用户审查发现；已移除该规则，用平移不改变旋转的单测防回归，并重跑固定/偏移 IK |
| 只改 XML body，没改 reset keyframe | 加载后看似新位置，episode reset 又回旧位置 | 同时核对两处定义，reset 后看 ground truth；偏移 XML 两处一致，遗漏本身未做故障注入 |
| 改 `target.*`，遗漏 `verify.*` | 机械臂去新目标，FSM 仍按旧圆心验收 | 查看目标日志与 `final placement` 中的 verify XY；代码已核对其独立性，未新增参数错配实跑 |
| reset 尚未完成就读取 IK seed | 可能消费前一 episode 的关节状态并缓存 HOME 解 | 等 reset 响应、清除本地观测、等待 0.1s 仿真时间；连续 20 次无失败，但延时不是严格的新鲜度屏障 |
| Cartesian 输入不是 world / 非有限数 / 非单位四元数 | 错误目标或求解失败 | 节点检查 frame；纯函数对数值和四元数抛异常，非法输入单测已通过；frame 分支未单独集成注入 |
| 把 IK 残差当作最终精度 | 模型误差只有 µm，真实 TCP 和 box 仍偏移数 mm | 本次 CSV 分别量到 8µm、6.82mm、5.35mm，保留关节误差辅助定位 |
| 将固定方向解释成任意姿态抓取 | 旋转物体后可能抓空，或方向不符合要求但仍判成功 | 当前不读取 box yaw、不验收箱体朝向；任意朝向实验尚未做，不作泛化结论 |

### 11.7 排查记录与被推翻的结论

**reset 时序修正。** 本阶段接入实测 seed 后，需要区分“reset 请求已经发出”和“新场景状态已经可以使用”。旧代码异步请求 reset 后继续运行，存在从缓存观测求 HOME 解的窗口。当前增加 `reset_pending_` / `reset_request_sent_` 门控；成功响应后清空本地关节、物体和接触缓存，等待 0.1s 仿真时间再恢复决策。修正后的连续 20 次记录无 IK 失败或重试。此修复并未引入消息时间戳屏障：清空本地缓存不能清空 DDS 队列，TF 也仍取 latest；不要把这个结果写成“严格同步已经解决”。保留的正式 CSV 是修正后的结果，不能从它重建初版竞态的发生率。

**径向 yaw 被用户指出。** 初版把“工具朝下”与“朝向目标相对原点的径向”一起塞进 `downwardPose()`，并曾获得位置验收成功。用户的问题揭示了验证缺口：位置成功既不能提供姿态选择的任务依据，也不能证明物体朝向正确。修正时把位置和朝向输入分开，增加姿态独立的断言，再重跑固定场景 20 次和偏移场景 1 次。初版径向 yaw 的平均放置误差 `4.62mm` 已被当前固定朝向版本的 `5.35mm` 取代；不能继续引用旧数字给新代码背书。

**构建和 lint。** 本阶段出现过新测试代码格式未满足 uncrustify 的反馈，修正格式后重新构建和运行测试，最终结果见 11.1。这属于代码规范失败，不能与 IK 求解或物理实验失败混记。

### 11.8 补充核查与后续边界

讲解后额外指出并通过现有代码核查了三项：参数是否动态生效、目标与验收圆心是否联动、成功标志是否覆盖物体朝向。前两项结论写在 11.4，后一项写在 11.3；都不需要等待后续周次才能回答，也不再作为悬挂问题。

当前可测试性已有直接断言：平移不改变工具方向，放置 XY 不依赖抓取物体位置，非零朝向参数独立生效。当前可观测性保留四层误差，但 CSV 未保存物体最终四元数；需要姿态验收时，应先补这一观测，再定义考虑物体对称性的误差判据。

仍可继续追问的实现边界包括 reset 的样本新鲜度和物体到工具相对变换。前者已有明确代码缺口，不是缺少参照系的悬挂题；后者在引入旋转物体或抓取生成时必须成为显式契约。现有 20 次固定回归和单次平移实验不足以替代这些验证，也不据此直接进入下一 stage 或宣告整周收尾。

## 12. 本周总结与第 4 周交接

### 12.0 本周完成情况

Week 3 完成了从模型数学到任务执行的第一条可验证闭环：

1. Stage K 建立纯 C++/Eigen 的 FK、几何 Jacobian 和模型加载库，并用有限差分、MoveIt 与 MuJoCo 交叉核对约定。
2. Stage L 明确了三方对照的独立性边界、frame/参考点约定和碰撞几何并非完全相同。
3. Stage M 实现带阻尼、关节限位、joint centering 和奇异性诊断的离线 DLS IK；明确它不是在线 Cartesian servo。
4. Stage N 将离线 IK 接入原有 FSM，暴露了“从手调 keyframe 反推任务目标”的设计错误和执行端误差层次。
5. Stage O 把任务几何改成直接输出 world-frame Cartesian waypoint，固定场景在 3cm 验收半径下 20/20 成功，平移 box 的 IK 实验 1/1 成功；旧 keyframe 模式保留为对照，并在偏移场景 0/1 失败，说明它没有泛化能力。

本周最重要的结论不是“20 次成功”，而是把四类误差分开：任务目标、IK 模型残差、关节跟踪/TCP 执行误差、最终物体落点。固定场景中 PLACE 的 IK 残差约 8µm，但实际 TCP 位置误差约 6.82mm，最终 box 落点误差均值约 5.35mm；这证明后续不能继续用调 IK 的方式解决执行器和接触误差。

### 12.1 文档审阅结论

本周文档现在有明确的事实来源和过程记录：`docs/architecture.md` 保存当前模型、frame、参数和实验事实；`docs/adr/001-cartesian-task-waypoints.md` 保存 Cartesian 任务几何的架构决策；本文件保存 Stage K~O 的推导、追问、失败模式和实测过程。Stage N 的旧结论没有被删除，而是明确标为“实验有效、设计验收不完整”，Stage O 作为修正后的收束。

审阅时修正了两处容易误读的地方：20 次回归属于 Stage O 的新目标构造验证；第 5 节的主动提示现在标明是历史记录，不再伪装成尚未回答的问题。仍需在后续周次独立补 ADR 的历史决策包括 vendor `<include>` 约定、GLFW debug viewer 和 episode 边界从进程改为话题触发。

### 12.2 第 4 周入口和边界

第 4 周可以在 Stage O 的 Cartesian 契约上接 RGB-D/视觉估计和随机物体位姿，但必须先决定视觉输出的语义：world-frame 还是 camera-frame、是否带时间戳/置信度、物体四元数是否参与抓取。当前 ground-truth oracle 只能作为仿真验证源，不能伪装成视觉接口。

第 4 周不应默认扩大 Stage O 的职责。MoveIt planner、碰撞规划、在线 Cartesian servo、真实抓取姿态生成和物体朝向验收分别是后续能力，只有触发条件到来时才进入接口重构。

## 13. 重构讨论与触发条件

### 13.0 现在的判断

当前不做大范围重写。Stage O 刚建立 `CartesianWaypointSource -> DiffIkWaypointSource -> JointTarget` 的边界，先让视觉和随机化实验使用它，才能知道哪些字段是真正稳定的契约。过早把接口改成 ROS action、轨迹对象或在线 servo，会把尚未观测到的需求写死。

### 13.1 现在值得做的局部重构

以下改动可以在不改变任务行为的前提下进行，但每项都应伴随纯函数测试或节点契约测试：

| 改动 | 触发理由 | 验收方式 |
|---|---|---|
| 从 `TaskExecutorNode` 提取参数解析、观测快照和 episode 日志结构 | 当前节点同时负责 ROS 回调、reset 门控、FSM 驱动、IK 诊断和 CSV 字段拼装，已接近“胶水层过厚” | 提取后 `fsm.cpp` 行为不变；参数默认值快照测试；episode outcome 字段长度一致 |
| 把 `target.*`、`verify.*` 和 marker 位置集中成显式 `PlacementGoal` 或配置结构 | 两组 XY 当前允许不一致，容易出现“机械臂去 A、FSM 验收 B” | 单测覆盖一致配置、故意不一致配置和启动日志；不改变现有 keyframe 兼容模式 |
| 把 reset 后的观测新鲜度从固定 `0.1s` 改成带时间/序号的快照条件 | 现有等待只能降低概率，不能证明拿到的是 reset 后样本；随机化和视觉接入会放大该风险 | 给 bridge/观测加 reset generation 或时间戳，测试旧样本不会驱动 HOME/GRASP |

这些是“复杂度已经阻碍测试或造成错误”的重构，不是为了追求更少的行数。建议先做参数/日志提取，再做 reset 新鲜度；`PlacementGoal` 在视觉输出协议确定后一起定型。

#### 13.1.1 第一层：提取配置、观测快照和 episode telemetry

> 启动第一层修改吧

这一层只整理职责，不改变 ROS 接口和任务语义。当前 [`TaskExecutorNode`](../../src/task_executor/src/task_executor_node.cpp) 同时负责参数声明、消息解析、reset 门控、IK 调用、FSM 输入组装和 episode 日志。第一层把它拆成三个数据边界：

| 边界 | 当前混在何处 | 建议抽出的内容 | 不改变的行为 |
|---|---|---|---|
| `TaskExecutorConfig` | 构造函数中 `declare_parameter()` 和 `waypoint_mode` 分支 | `FsmParams`、`PickPlaceGeometry`、验收配置和模式；另有集中校验函数 | 参数名、默认值、`diff_ik`/`keyframe` 选择规则 |
| `ObservationSnapshot` | `onTimer()` 直接读取多个 `latest_*` 成员并拼装 `FsmInputs` | arm、gripper、object pose、接触信号和 `world -> hand_tcp` 的一次 tick 快照 | joint name 映射、world frame 检查、缺数据时等待 |
| `EpisodeTelemetry` | 多组 `phase_names_log_`、`target_tcp_*`、误差数组 | 一个阶段一个结构体，最后再转换成已有 `EpisodeOutcome` | 消息字段、CSV 字段、阶段顺序和 NaN 语义 |

**配置抽取。** 可以增加 `loadTaskExecutorConfig(rclcpp::Node &)` 和 `validateTaskExecutorConfig()`。前者只做 ROS 参数到结构体的映射，后者是可单测的纯校验：高度和半径必须为有限正数，yaw 不能是 NaN，模式只能是 `diff_ik` 或 `keyframe`。构造函数随后只负责根据配置创建 waypoint source。这样以后新增参数不会继续扩大节点构造函数，也能直接测试默认配置和 keyframe 的兼容默认值。

**观测抽取。** 增加 `collectObservation()`，成功时返回一个完整 `ObservationSnapshot`，缺少任意必要输入时返回空。按 joint name 查找和两个手指宽度检查放进可单测的 `makeObservationSnapshot()`；`world` frame 检查和 TF 查询仍在节点中。`onTimer()` 先取得一份快照，再用它驱动 IK 和 FSM。这只避免同一 tick 的后续代码重复读取可变缓存，**并不保证 arm、box、contact 与 TF 来自同一仿真时刻**；多线程 executor 或随机化场景需要第三层的新鲜度协议。

**episode telemetry 抽取。** 当前正常阶段转换和 IK 失败路径各自手工 push 多组并行数组；新增诊断字段时容易造成数组长度错位。建议使用 `PhaseTelemetry` 保存阶段名、耗时、TCP 目标、IK 残差、关节误差和实际 TCP 误差，节点只调用 `recordPhase()`。`makeEpisodeOutcome()` 在 episode 结束时把结构体展开成现有 ROS message，因此暂时不需要修改 `EpisodeOutcome.msg` 或 `episode_runner.py`。

这一层的验收不是“文件变短”，而是：配置默认值单测通过；joint name 任意排序仍能得到相同快照；缺少输入不会调用 FSM；成功和 IK 失败路径生成的所有并行数组长度一致；固定场景 20/20、keyframe 对照和 CSV 字段保持不变。

**第一层实施与排查记录。** `TaskExecutorConfig`、`ObservationSnapshot`、`EpisodeTelemetry` 已分别落到独立头文件和实现文件；节点保留 ROS 回调、TF 查询、reset 门控、IK/FSM 驱动。配置测试覆盖默认 IK、keyframe 历史验收中心、参数覆盖以及非法几何；观测测试覆盖 joint name 逆序和缺手指/速度数据；telemetry 测试覆盖阶段顺序、字段长度与失败阶段 NaN。第一次实跑的首个 episode 出现 `PLACE_MISSED`、3 次重试，虽然最终 box 与验收中心只差 5.67mm。排查发现 `verify.*` 只写入新建的 `PlacementVerification`，漏写 `FsmParams.place_*`，FSM 实际仍按 `(0, 0)` 验收；日志却从新配置打印 `(0.5, 0.3)`，两者分叉。修复后同时断言两组字段相等，再做实跑回归。这个故障说明结构抽取中的同值复制也是行为契约，不能只比较参数声明的名字和默认值。

最终代码在 `robotics-dev` 环境用 `colcon build --packages-select task_executor --symlink-install` 和 `colcon test --packages-select task_executor` 验证；`colcon test-result --test-result-base build/task_executor/test_results --all --verbose` 报告 **135 tests、0 errors、0 failures、24 skipped**（其中 skipped 为静态检查中未启用的 cppcheck 项）。启动真实 bridge/executor 前 `ps -eo pid,comm` 未发现遗留 bridge；运行时 `/clock` 是 **1 个 publisher**。进程均直接使用 `install/*/lib/*/*_node` 可执行文件，结束后再查无遗留进程。实跑 CSV 保存在 `/tmp`，没有覆盖 Stage O 的基线文件。

| 最终代码验证 | 结果 | 证据 |
|---|---|---|
| 固定场景 IK，3cm 验收半径，20 次 | **20/20 成功、0 重试**；平均 episode 6.853s，平均放置误差 5.25mm、最大 5.67mm | `/tmp/first_layer_final_fixed_20.csv` |
| keyframe 旧模式，1 次 | **1/1 成功、0 重试**；日志中的验收中心仍是 `(0.43, 0.31)m` | `/tmp/first_layer_final_keyframe.csv` |
| 故意不可达的 `target.place_x_m:=10.0`，1 次 | `0/1`，`IK_FAILED`，失败阶段 `PREPLACE`，其 IK 残差字段为 `nan` | `/tmp/first_layer_final_ik_failed.csv` |
| CSV 结构 | 三组结果的所有阶段数组长度均等于 `phase_names` 长度；18 个列名及顺序与 `stage_o_fixed_20.csv` 完全相同 | 用 `/usr/bin/python3` 的 `csv` 解析后逐行检查 |

**机制与权衡。** 每次 tick 由节点检查消息是否齐备、查询 TF，再由 `makeObservationSnapshot()` 按 joint name 对齐 7 个关节和 2 个手指。IK 与 FSM 消费同一份快照；配置解析只在构造时进行；阶段退出时追加一条 `PhaseTelemetry`，发布结果时统一展开成 ROS 消息。保留 `EpisodeOutcome` 的并行数组，是为保持 runner 和历史 CSV 契约；内部改成每阶段一个结构体，使新增字段时不必同时维护多组 push。代价是发布时仍需一次转换，并且 `PlacementVerification` 到 `FsmParams` 有一份必须测试的同值映射。

**失败模式与边界。** 配置漏映射会造成日志目标与 FSM 判据不一致，第一次实跑已复现并修复；缺任一必要 joint/手指数据时快照返回空，节点不会调用 FSM；不可达 IK 保留已完成阶段并追加带 NaN 诊断的失败阶段。当前快照没有各话题的时间戳一致性检查，reset 后仍依靠原有 0.1s 等待；这是第三层的范围，不能把第一层的快照称为严格同步观测。

#### 13.1.2 第二层：把任务目标和验收目标变成显式配置

> 开始下一层级的重构

当前 `target.place_x_m/y_m` 控制 IK 的 TCP 目标，`verify.place_x_m/y_m` 控制 FSM 的最终落点圆心。它们默认相同，但允许分别覆盖，因此可以出现“机械臂去 A、FSM 在 B 判成功”。这不是当前实验已经发生的失败，而是配置语义没有被类型表达出来。

第二层不应简单删除 `verify.*`：keyframe 基线的历史落点约为 `(0.43, 0.31)m`，而 IK 任务目标是 marker 中心 `(0.5, 0.3)m`；执行误差实验也可能故意让命令目标和验收目标不同。应该把两者改成不同名字、不同字段：

```text
PlacementTask       { tcp_target_xy, tcp_target_height, tool_yaw }
PlacementVerification { box_target_xy, radius }
```

启动时打印两组值，并在 `diff_ik` 模式下默认检查它们是否一致；若实验确实需要不一致，应增加显式的允许开关，而不是依靠用户记住两组参数的关系。keyframe 模式保留旧默认值和旧行为。

验收需要覆盖四个配置组合：IK 同目标、IK 故意不一致、keyframe 旧默认值、两组显式覆盖。每次 episode 的日志同时打印 task target 和 verification target，避免只看最终成功布尔值。这个重构应在视觉输出协议确定前完成，因为视觉接入后需要明确“观测到的物体、生成的抓取目标、指定的放置目标、最终验收目标”分别是谁负责。

**实施机制。** `PlacementTask` 持有 TCP 目标 XY、高度和工具旋转，在创建 `PickPlaceCartesianWaypointSource` 时转换为旧 `PickPlaceGeometry`；`PlacementVerification` 持有 box 目标圆心与半径，加载时映射到 `FsmParams`。`diff_ik` 模式若两组 XY 不同，默认抛出配置错误；只有显式设置 `verify.allow_target_mismatch:=true` 才允许故意不一致。`keyframe` 不消费 Cartesian 任务目标，因此保留默认 `(0.43, 0.31)m`，无需这个开关。参数和 CSV 列未改；节点在启动和每次 episode 开始时同时打印两组目标。决策见 [ADR 002](../../docs/adr/002-placement-target-verification-contract.md)。

**权衡与失败模式。** 保留两组目标可继续做 keyframe 对照和故意不一致实验，但多了一个显式允许参数。只检查 XY 是当前固定朝下、无物体姿态验收的任务约定；不意味着一般物体的 TCP 与 box 目标一定同点。漏设开关时 IK 节点启动即失败，比跑完才报 `PLACE_MISSED` 更易定位；误开开关时日志会同时暴露两组值，但仍需实验者核对目标语义。`FsmParams` 仍需从 `PlacementVerification` 拷贝三个验收字段，加载测试对这三项做断言，防止第一层的漏映射回归。

**实测结果。** 在无遗留 bridge 的环境启动安装目录中的真实可执行文件，运行时 `/clock` 有 1 个 publisher。最终代码执行 `colcon build --packages-select task_executor --symlink-install`、`colcon test --packages-select task_executor`，测试汇总为 **139 tests、0 errors、0 failures、24 skipped**；结束后无遗留 bridge/executor 进程。CSV 存在 `/tmp`，没有覆盖既有基线。

| 配置 | 实跑 | 目标与验收日志 / 结果 |
|---|---|---|
| 默认 IK，`verify.place_region_radius_m:=0.03` | 固定场景 **20/20 成功、零重试** | 任务 TCP XY 与 box 验收中心均为 `(0.50, 0.30)m`；`/tmp/second_layer_final_fixed_20.csv` 的平均 `place_error_m` 为 5.22mm |
| keyframe 默认值 | **1/1 成功、零重试** | 验收中心仍是 `(0.43, 0.31)m`；`/tmp/second_layer_final_keyframe.csv` 的 `place_error_m` 仍为空，因为 keyframe 不产生 TCP 目标 |
| 两组显式覆盖到 `(0.51, 0.30)m` | **1/1 成功、零重试** | 任务 TCP 与验收中心都改变；`/tmp/second_layer_final_matching_override.csv` 的放置误差为 5.43mm |
| 任务 `(0.50, 0.30)m`、验收 `(0.51, 0.30)m`，显式允许 | **1/1 成功、零重试** | 启动和 episode 日志同时打印两组值；`/tmp/second_layer_final_allowed_mismatch.csv` 相对任务 TCP 的放置误差为 5.22mm，而节点日志相对验收圆心的误差为 13.39mm |
| 同上但未设置允许参数 | 启动拒绝，退出码 **1** | `Startup failed: IK task and verification XY differ; set verify.allow_target_mismatch:=true for an intentional mismatch`；`/tmp/second_layer_rejected.log` |

四个实际运行结果的 18 个 CSV 列名和顺序均与 `stage_o_fixed_20.csv` 相同，每行所有诊断数组长度都等于 `phase_names` 长度。`place_error_m` 是 runner 按 `PLACE` 阶段的任务 TCP XY 计算的，**不是 FSM 相对 `verify.*` 的验收误差**。因此故意不一致实验不能只看 CSV 的这一列；要同时看节点日志中的 `verify_box_xy` 与 `final placement`。

**排查记录。** 第一轮启动拒绝测试确实检查出不一致配置，但旧 `main()` 对构造异常未捕获，进程以 `Aborted`（退出码 134）结束。该错误如今是一个预期配置路径，因此在构造节点时捕获异常、打印 FATAL 原因并以退出码 1 结束；后续异常不被误标为启动错误。第一次 CSV 校验脚本又把 keyframe 的空 `place_error_m` 当浮点数解析而报错；按字段可用性重做统计后四组结果全部通过，这个空值本身是旧 CSV 语义，并非执行失败。

#### 13.1.3 第三层：把 reset 后等待改成可证明的新鲜度门控

> 开始第三层吧

当前实现已经有 `reset_pending_`、`reset_request_sent_` 和 reset 后 `0.1s` 等待。它解决了“reset 回调成功后立即使用旧缓存”的已知竞态，但它只能降低风险：清空本地指针不能清空 DDS 队列，TF 仍取 latest，`0.1s` 也不是样本来源证明。

第三层分两步做。第一步先抽出 `ResetGate`，把 begin、服务请求中、reset 成功、等待观测和 ready 状态集中管理；节点的 `onTimer()` 只询问 gate 是否允许决策。第二步在随机化或视觉接入前引入 reset generation 或等价的样本序号：bridge 每次 reset 产生新 generation，joint/object/contact/TF 观测带上时间或 generation，executor 只接受当前 episode 的样本。这样才能测试“reset 前的样本不会驱动 HOME seed、抓取目标或 VERIFY 判据”。

第三层的验收必须包含故意延迟旧消息、连续 reset、retry reset 和正常 episode 四类场景。只看到 20 次成功不能证明时序已经严格同步；应记录被接受的 generation/timestamp，失败时明确报告“没有新鲜观测”，而不是继续等到 phase timeout。

三层的顺序是：先做第一层的数据结构抽取，再做第二层的目标语义整理，最后在随机化和视觉需要时做第三层的 generation 协议。第一层和第二层可以保持现有实验行为；第三层会改变观测契约，必须单独记录 ADR 和回归结果。

**实现后的数据契约。** bridge 保留旧 `~/reset` Trigger 和 `/joint_states`、TF、ground-truth 话题，另外提供返回会话标识和 generation 的 `~/reset_with_generation` 服务，以及 `~/episode_observation` 同步观测包。每次 bridge 启动生成新会话，成功 reset 才递增 generation；每个观测包来自同一次 `mj_step`，携带会话、generation、单调的 `sample_sequence`、同一仿真时间戳的关节、物体、world-frame TCP 和双指接触。executor 改为只订阅这份包，用纯逻辑 `ResetGate` 管理 reset 请求、回执、等待、ready、失败。观测可解析、会话及 generation 等于回执且序号递增后才调用 IK/FSM；旧会话、旧 generation、重复序号被拒绝，同会话下更高 generation 表示外部 reset 取代本次 episode。5s 墙钟看门狗区分 `RESET_UNAVAILABLE`、`RESET_FAILED`、`OBSERVATION_STALE`、`RESET_SUPERSEDED`；FSM 的阶段耗时仍用仿真时间。详见 [ADR 003](../../docs/adr/003-reset-generation-observation.md)。

**构建、单测与实跑。** `colcon build --packages-up-to task_executor --symlink-install` 构建了接口、bridge、executor 及其依赖；`colcon test --packages-select mujoco_bridge task_executor` 汇总分别为 **106 tests、0 failures、18 skipped** 与 **159 tests、0 failures、27 skipped**。`ResetGate` 单测覆盖故意延迟旧 generation、重复序号、连续 begin、retry reset、bridge 重启后同 generation 的旧会话、观测中断、服务失败和超时。启动实际进程前 `ps -eo pid,comm` 无遗留 bridge，运行中 `/clock` 为 1 个 publisher；执行完再次检查无遗留 bridge/executor。实跑结果保存在 `/tmp`，未覆盖已有基线。

| 场景 | 实测 | 证据 |
|---|---|---|
| 固定 IK、3cm 验收半径 | **20/20 成功、零重试**；日志依次接受 generation 1 到 20 | `/tmp/third_layer_session_fixed_20.csv`、`/tmp/third_layer_session_fixed_20.log` |
| keyframe 旧模式 | **1/1 成功、零重试** | `/tmp/third_layer_session_keyframe.csv` |
| 故意把 keyframe 验收中心设为 `(0,0)m`，`fsm.max_retries:=1` | `0/1`，`PLACE_MISSED`，**1 次 retry**；reset generation 1 → 2，两次各接受对应 generation 的观测 | `/tmp/third_layer_session_retry_one.csv`、`/tmp/third_layer_session_retry_one.log` |
| 延迟旧观测集成探针 | reset 回执为 generation 2，连续发布 generation 1 的完整包；约 5s 后 outcome=`OBSERVATION_STALE`，**0 条关节命令** | `/usr/bin/python3 src/task_executor/test/reset_generation_probe.py` |

三组 CSV 的 18 个列名和顺序均与 `stage_o_fixed_20.csv` 相同，每行诊断数组长度等于 `phase_names` 长度。正确参数的 retry 实验在 bridge 与 executor 同时启动、runner 立即请求 episode 的条件下进行：第一次 reset 回执 generation=1 后首份被接受观测为 generation=1、stamp=0.090s；重试后接受 generation=2、stamp=12.390s；没有先把旧样本作为 HOME IK seed。

**机制权衡与排查。** 固定等待 0.1s 无法证明 DDS 队列和 TF 缓存已更新；同一步观测包加会话/generation 可检查来源和内部一致性，代价是桥与 executor 新增共享接口，且同一物理数据会在旧话题和新包中各发布一次。最初只比较 generation，复核发现 bridge 重启后计数会复用，于是加入启动时生成的会话标识，并补“旧会话与新会话同为 generation=1”单测。首次编译因 `collectObservation()` 条件块漏一处花括号失败，补齐后构建通过；静态格式检查先后指出日志整数格式、枚举换行及 Python 探针版权头，逐项修正。正确参数的 retry 初跑又在首条 `/clock` 到来前触发了 episode：仿真时间仍为零，旧的 `phase_start_time_==0` idle 哨兵把已启动 episode 误判为空闲，runner 120s 超时。改用 `ResetGate::kIdle` 判断后重建和测试通过，同样的即时启动方式得到 `PLACE_MISSED`、一次 retry 和两个正确 generation。集成探针证明旧观测不会驱动运动命令；20/20 只证明正常链路没有退化，不能代替延迟注入实验。当前仍依赖默认单线程 executor 和 bridge 的互斥回调组；改成多线程时需为 reset、观测缓存和 `mjData` 加同步。

#### 13.1.3.1 哪些组件能复用，时间戳对齐通常从哪里来

> 这一层的组件，未来是否有复用的可能？我注意到它处理了时间戳对齐的问题？在通常的practice中，这种时间戳对齐的问题通常出自哪里又是如何解决的？

可复用的是时序约束，不是 MuJoCo 的 ground truth 字段本身。[ResetGate](../../src/task_executor/src/reset_gate.cpp) 不依赖 ROS，只检查 reset 回执的会话、generation 与观测序号；以后若仍有“重置场景后开始任务”的流程，可沿用这一门控规则。[BridgeObservation.msg](../../src/manipulation_interfaces/msg/BridgeObservation.msg) 体现“把一次决策所需的同源观测作为一份数据交付”的思路，但它含有仿真 oracle 和接触真值，不能原样充当真机感知协议。

这里要区分两类时间问题：generation 回答“属于哪次 reset”，仿真时间戳回答“物理上是什么时候”。旧节点分别接收不同速率、不同到达顺序的关节、box、无 stamp 的接触消息，再从 TF buffer 取 latest，拼出的状态未必属于同一物理步。新包由 bridge 在同一次 `mj_step` 后读取同一份 `mjData`；executor 检查会话、generation、递增序号，**没有**运行通用的多传感器时间戳匹配算法。实测旧 generation 持续到达时，约 5s 报 `OBSERVATION_STALE` 且没有关节命令；它验证 reset 新鲜度，不证明真机传感器已同步。

实际机器人上的不同采样频率、采集延迟、传输延迟以及时钟不一致，都会造成图像、关节状态、力传感器与 TF 错位。通常先统一时钟并保留采集时刻的 stamp，再用精确/近似时间同步配对离散消息、对连续关节状态插值、用目标观测时刻查询 TF，并限定最大时间差和等待时长。视觉接入时应记录数据年龄、同步误差和丢弃原因；不能仅把几个异步消息塞进一个 ROS message 就称为对齐。

#### 13.1.3.2 旧 onReset 是否还在使用，如何处理兼容接口

> 1. 就现在来看，原本的OnReset没有被任何地方使用对吗？我们通常对这样的函数or实现是如何处理或者标记的？

仓库内运行时已没有 `/mujoco_bridge/reset` 的客户端：executor 改调 `/mujoco_bridge/reset_with_generation`。但 [onReset()](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp) 仍注册为旧 Trigger 服务的回调，外部 `ros2 service call` 仍能触发；week1、week2 也记录过手动调用。它不是死代码。两个回调共用 `resetScene()`，旧服务成功调用同样推进 generation，使正在运行的 episode 能发现被外部 reset 取代。

这是服务名层面的兼容接口。通常先在文档说明“新客户端使用带 generation 的服务，旧服务供现有手动/外部调用”，确认外部依赖并定移除时点后再删除。只给 C++ 回调加 `[[deprecated]]` 无法通知调用 ROS 服务的用户；贸然删除则会让已有外部调用直接失效。当前仓库搜索只能证明没有仓库内运行时客户端，不能证明外部无人使用。

#### 13.1.3.3 BridgeObservation 与独立 TF、旧话题的关系

> 2. 我仍然好奇BridgeObservation的真正作用。它把好几个obs ground truth封装在一起，这样就可以搞定时间戳对齐的问题？但是TF数据仍然是分开发的？而且为什么继publishObjectPose和publishGripperContact还要再发一遍？

包内数据能对齐，原因是**生产时**从同一份 `mjData`、同一次 `mj_step` 后采样，且 bridge 默认互斥回调不会在发布之间再次步进；不是“封装”本身具有同步能力。[onTimer()](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp) 在同一个回调中发布旧话题与新包。新包还内嵌一份从 `mjData` 计算的 world-frame `hand_tcp` 变换，executor 只用这份 TCP，不查 TF buffer 的 latest。`/tf` 仍独立发布父子 frame 变换供 RViz 等使用；其他只订阅旧话题或 `/tf` 的节点**不会**因此自动获得原子快照。

重复发布是兼容旧消费者的代价：`/joint_states` 与 `/tf` 供现有 ROS 可视化链路使用，[episode_runner.py](../../scripts/episode_runner.py) 仍读独立 object pose 写 CSV，旧接触话题也保留原接口。当前代码确实重复填充 joint state、box pose 和接触数据，有额外计算与传输成本；若需要优化，可先从一次采样填充两类输出，再根据实际消费者决定是否移除旧话题。这样做仍需守住“同一次物理步”的不变量，不能让两类输出在重构后从不同时间的缓存取值。

### 13.2 暂时不要做的重构

- 不要现在把 `CartesianWaypoint` 改成完整 ROS message。视觉真正接入、需要跨进程传输 frame、时间戳和置信度时再决定 message/action 的字段。
- 不要现在把 FSM 改成 MoveIt `JointTrajectory` 或在线 Cartesian controller。当前 20Hz 离散目标正是验证任务几何的实验后端，替换它会混淆任务目标和控制执行问题。
- 不要现在删除 `KeyframeWaypointSource`。它仍是旧行为回归基线；等 Cartesian 后端在固定、平移、旋转和失败恢复场景都有覆盖后再移除，并保留一次历史对照数据。
- 不要仅因为 `task_executor_node.cpp` 较长就拆成多个 ROS 节点。节点边界应由通信、线程或生命周期需求驱动，而不是按文件长度切分。

### 13.3 何时必须重构

满足下列任一条件时，重构从“可选整理”变成“下一阶段的前置工作”：

1. 视觉输出需要跨节点传递 `frame_id`、时间戳、置信度或观测失败状态；当前 `ObjectPose` 无法表达这些信息。
2. `target.*` 与 `verify.*` 在随机化实验中出现过一次配置漂移，导致目标和验收中心不一致。
3. reset 竞态再次导致旧样本进入 IK seed、目标缓存或抓取判据。
4. 第二个 Cartesian 后端（MoveIt 或在线 servo）接入时，现有接口需要加入后端专属字段；此时应先稳定任务层，再加适配器，不能把后端字段塞进任务 waypoint。
5. `TaskExecutorNode` 的单元测试需求已经无法通过纯函数提取完成，必须启动 ROS graph 才能验证关键行为；此时增加节点级测试，而不是继续扩大未测试的 glue code。

重构完成的判据是：行为变化有独立实验说明，旧模式仍有可比基线，接口边界有单测，文档和 ADR 同步更新。仅“代码变短”“类更多”不算重构收益。
