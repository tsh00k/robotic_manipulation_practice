# Week 3 学习笔记

> **本文件当前是开周计划稿**，只有第 1~6 节（继承事实、stage 计划、验收标准、要回头解锁的悬挂项、开周提示、本周悬挂清单）是现在写下的。第 7 节起的 stage 节随每个 stage 完成后就地扩写，骨架按 [STUDY_NOTES_GUIDE.md](../../STUDY_NOTES_GUIDE.md) 2.2（`N.0 一句话总结 → N.1 改动清单与验证结果 → N.x 概念讲解 → N.x 失败模式与验证手段 → N.x 排查记录`）。
>
> Stage 编号延续第1周的 A~E、第2周的 F~J，本周是 **K~N**（四个，[范围选择见 2.0](#20-为什么是四个-stage不是五个)）。

## 学习重点范围

本周对应计划书[第5节 Chap 3 模块](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md)里第2周没做完的后半段（5.2 第 4、5 条）与 Roadmap 第3周（"FK/Jacobian/damped diff-IK、模型一致性测试 → 数学单测、奇异性曲线、MoveIt/MuJoCo FK 对照"）。

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
  - [2.0 为什么是四个 stage，不是五个](#20-为什么是四个-stage不是五个)
  - [2.1 Stage K — 自写 FK/Jacobian 纯函数库（新包 `arm_kinematics`）](#21-stage-k--自写-fkjacobian-纯函数库新包-arm_kinematics)
  - [2.2 Stage L — 模型一致性测试：MuJoCo ↔ URDF ↔ 自写三方对照](#22-stage-l--模型一致性测试mujoco--urdf--自写三方对照)
  - [2.3 Stage M — damped least-squares diff-IK 与奇异性观测](#23-stage-m--damped-least-squares-diff-ik-与奇异性观测)
  - [2.4 Stage N — `DiffIkWaypointSource`：FSM 一行不改地换掉查表](#24-stage-n--diffikwaypointsourcefsm-一行不改地换掉查表)
- [3. 本周验收标准](#3-本周验收标准)
- [4. 要回头解锁的 Week 1/2 悬挂项](#4-要回头解锁的-week-12-悬挂项)
- [5. 开周就该注意的（Claude 提示，尚未讨论）](#5-开周就该注意的claude-提示尚未讨论)
- [6. 悬挂问题（本周新增）](#6-悬挂问题本周新增)
  - [6.1 清单](#61-清单)
  - [6.2 反向清单：现在就该做的](#62-反向清单现在就该做的)
- [7. Stage K：自写 FK/Jacobian 纯函数库](#7-stage-k自写-fkjacobian-纯函数库)
  - [7.0 一句话总结](#70-一句话总结)
  - [7.1 改动清单与验证结果](#71-改动清单与验证结果)
  - [7.2 本阶段冻结的数学约定](#72-本阶段冻结的数学约定)
  - [7.3 JD 写“了解 FK/Jacobian”通常要求到什么程度](#73-jd-写了解-fkjacobian通常要求到什么程度)
  - [7.4 这组测试证明了什么](#74-这组测试证明了什么)
  - [7.5 失败模式与验证手段](#75-失败模式与验证手段)
  - [7.6 本阶段边界与后续](#76-本阶段边界与后续)

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

四个 stage，**每个都按五步走不跳步**（写代码 → build + 实跑验证 → 讲解 → 追问 → 写进本文件），见 [STUDY_NOTES_GUIDE.md 第3节](../../STUDY_NOTES_GUIDE.md)。

### 2.0 为什么是四个 stage，不是五个

计划书 Roadmap 第3周只写了"FK/Jacobian/damped diff-IK、模型一致性测试"，没有要求接进任务链。但**只做数学、不换掉 `KeyframeWaypointSource`，第3周结束时系统能力和第2周完全一样**（还是手调查表），而且第2周为这次替换专门留的接口会晾一周，晾着的接口通常会长歪。所以 Stage N 是必做。

砍掉的是**随机物体位姿的批量泛化评测**（原本会是 Stage O）：它才是 IK 真正的价值证明（查表必然失败、IK 能过），但它依赖 Stage N 的物理调参先稳定，而第4条学习重点（运动学解 vs 伺服执行的系统性偏差）大概率会吃掉整块时间。**推到第4周**，和 Chap 4 的 RGB-D 一起——那时物体位姿本来就要随机化，合并做一次比这周赶出来更省。

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

## 3. 本周验收标准

从计划书 [5.3](../5.%20视觉机械臂%20Pick-and-Place%20项目计划书.md) 里挑出本周该收的（第 2、3、5 条是 week2 明确推过来的），加两条自定：

| # | 标准 | 出处 | 本周范围 |
|---|---|---|---|
| 1 | Jacobian 解析值与有限差分在容差内一致 | 5.3 第3条（week2 推来） | **本周做**（Stage K） |
| 2 | MuJoCo 与 MoveIt FK 一致性通过固定样例测试 | 5.3 第2条（week2 推来） | **本周做**（Stage L，且升级成三方对照） |
| 3 | 日志显示每阶段、目标 frame、末端误差、**最小奇异值**、失败原因 | 5.3 第5条（week2 只做了一半） | **本周补齐**（Stage M 产生该量，Stage N 打进日志） |
| 4 | 实现 damped least-squares differential IK，记录最小奇异值、关节限位裕量、tracking error | 5.2 第5条 | **本周做**（Stage M + N） |
| 5 | 固定物体位姿下连续成功 20 次，无非预期碰撞 | 5.3 第4条 | **回归**：week2 已达成，本周换了求目标的方式，必须不退化（Stage N） |
| 6 | 约束 differential IK（速度/位置限制、joint centering、奇异区减速）与"伪逆后裁剪"的路径偏差对照 | 5.4 扩展点1 | **部分**：限制与 centering 做（Stage M），**对照实验列为 stretch** |
| 7 | 随机物体位姿下的泛化评测 | 2.1 | **推到第4周**（见 [2.0](#20-为什么是四个-stage不是五个)） |

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

## 5. 开周就该注意的（Claude 提示，尚未讨论）

按 [STUDY_NOTES_GUIDE.md 第5节](../../STUDY_NOTES_GUIDE.md)，这几条是"你还没问但值得注意的"。**点出来，不自问自答**，等到对应 stage 再展开或决定忽略。

1. **Jacobian 的三处约定不一致时，没有任何东西会报错**（C 类）— 参考点（TCP / hand / body 质心）、表达 frame（world / body）、6 维向量里线速度和角速度谁在前。MuJoCo（`jacp`/`jacr` 分开两个数组、world frame）、MoveIt（`getJacobian()` 有默认参考点）、自写实现三套默认值大概率不同。**值得在写第一行代码前先把三边约定各自查清写下来**，否则数值对不上时是三选一甚至八选一的排错。
2. **"三方对照"其实只有两方数据独立**（E/F 类，已核实）— [1.1 #3](#11-开周核对出来的新事实本周才第一次相关)：`fer.urdf.xacro` 直接读 `kinematics.yaml`。所以自写 ↔ MoveIt 的一致**只证明两个实现都没写错，不证明参数对**。值得问的是：**那"参数抄错"这类错误，本周有哪一条测试真的能抓到？** 如果答案是"只有和 MJCF 的对照"，那这个对照对的容差就不能和另一对用同一个数。
3. **diff-IK 的"失败"在单测里长什么样**（E 类）— 不可达目标、奇异构型、超限位，这三种"算不出来"要不要用同一个失败码？更要紧的一问：**如果求解器返回了 NaN 并被发到 `~/joint_command`，现在的系统会怎样？** position servo 会把机械臂甩飞、被 `ctrlrange` 截住、还是 MuJoCo 整个炸掉——现在不知道，而这是 IK 接进执行链后**新出现的**一类输入。
4. **$\sigma_{\min}$ 作为奇异度量的适用边界**（F 类）— 它的数值依赖 `J` 里米和弧度的混合加权（[2.3](#23-stage-m--damped-least-squares-diff-ik-与奇异性观测)），换个加权同一个构型会得到不同的 $\sigma_{\min}$。所以"$\sigma_{\min} < 0.01$ 即接近奇异"这类阈值**不是构型的固有属性**。值得在写进日志之前就说清它相对于什么而言，否则这个数会被后面几周当成绝对量引用。

## 6. 悬挂问题（本周新增）

### 6.1 清单

> 待各 stage 结束后按 [4.3 的判据](../../STUDY_NOTES_GUIDE.md) 分流填入（缺参照系 → 这里）。

### 6.2 反向清单：现在就该做的

> 第1、2周遗留、本周不打算做的条目**不要**复制过来，仍以 [week1.md 13.2](week1.md#132-反向清单现在就该做的属于缺一次推演) 和 [week2.md 6.2](week2.md#62-反向清单现在就该做的) 为权威。本节只放本周新产生的。开周先记两条：

- **`docs/adr/` 至今是空目录**，而 [STUDY_NOTES_GUIDE.md 第6节](../../STUDY_NOTES_GUIDE.md) 的收周清单每周都要求"重大架构决策补一份 ADR"。已经攒了至少三件够格的：keyframe 长度静默补零后定下的同目录 `<include>` 约定、可视化从 RViz 回退到同进程 GLFW viewer、episode 边界从"进程边界"改成话题触发。本周还会再产生至少一件（`hand_tcp` 要不要改 vendor MJCF）。**缺一次推演**：不是不知道怎么写，是没定"够格"的判据——值得先定判据再补写，否则会变成把笔记复制一遍。
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
