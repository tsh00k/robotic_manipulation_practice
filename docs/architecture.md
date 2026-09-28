# Architecture & Conventions

> 对应 Job_guides 第15节任务2：固定 Panda 关节命名、world/base/tool/camera/object frame 约定。
> 任何后续模块都不得引入与本文档不一致的第二套约定。

## 0. 模型来源

不在本仓库 vendor 机器人描述文件，统一走 apt 安装的系统 ROS2 包，通过 `get_package_share_directory()` 引用：

| 用途 | 包名 | 安装方式 | 关键路径 |
| --- | --- | --- | --- |
| 官方 URDF/SRDF/mesh | `franka_description` (1.0.1, Humble) | `sudo apt install ros-humble-franka-description` | `share/franka_description/robots/fer/fer.urdf.xacro`（7-DoF 臂，官方新命名 `fer`，即原 "Panda"）、`share/franka_description/end_effectors/franka_hand/franka_hand.urdf.xacro`（平行夹爪） |
| MoveIt 2 现成配置（SRDF、kinematics、joint_limits、OMPL/Pilz/CHOMP planning yaml） | `moveit_resources_panda_moveit_config` (2.0.7, Humble) | `sudo apt install ros-humble-moveit-resources-panda-moveit-config` | `share/moveit_resources_panda_moveit_config/config/` |
| MuJoCo MJCF + mesh | 无 apt 包，从 [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) `franka_emika_panda/` 子目录复制 | vendor 进本仓库 | `robot_description/mujoco/franka_emika_panda/panda.xml`（含夹爪）、`panda_nohand.xml`（仅7轴臂） |

### 0.1 MJCF 组合规则（Stage G 新增）

- **`<include>` 与每个被包含文件自己的 `meshdir`，无论嵌套多深，都相对"顶层主文件"的目录解析，不是相对"直接包含它的文件"**。这不是本仓库的假设，是 MuJoCo 编译器本身的行为（[mujoco#974](https://github.com/google-deepmind/mujoco/issues/974)）。实测：把新场景文件放进独立目录 `mujoco_models/` 并用 `<include file="../franka_emika_panda/scene.xml"/>` 引用，`compile` 报错找不到 mesh（路径被拼接成三层重复）。**结论：任何要 `<include>` `panda.xml`/`scene.xml` 的新文件，必须和它们放在同一目录**，这也是为什么 vendor 自己的 `mjx_single_cube.xml` 采取同目录布局——不是随意的组织方式，是被这条限制逼出来的唯一可行结构。`pick_place_scene.xml` 因此放在 `franka_emika_panda/` 目录内，是新增文件、不是修改 vendor 文件。
- **keyframe 名字冲突 和 keyframe 长度不匹配，是两件独立的失效模式，不要混为一谈**：
  - 名字冲突（两个 `<key>` 同名，即使不同文件里定义、通过 `<include>` 合并到同一模型）→ **编译期硬错误**：`Error: repeated name 'home' in key`。
  - 长度不匹配（同一模型里，`<key>` 的 `qpos` 长度 ≠ 当前 `nq`）→ **不报错，MuJoCo 静默把缺的部分补 0**（对 freejoint 的四元数分量，补出来的是 `(0,0,0,1,0,0,0)`，即世界原点+单位旋转,不是"沿用旧值"或"截断报错"）。实测：把 `pick_place_scene.xml`（nq=16）的 reset 目标改回 vendor 的 9 长度 `home` keyframe，`~/reset` 返回 `success=True`（不报错），box 被静默传送到世界原点附近并砸向机械臂底座。**这比编译期报错危险得多**——它只在两个 keyframe 恰好重名时才会被前一种错误挡住；只要改成不同名字，长度不匹配就会在运行时安静发生。详见 [week2.md 8.8](../Job_guides/my_study/week2.md#88-排查记录keyframe-名字冲突与长度不匹配是两件独立的事结论被推翻)。

选型理由：
- Franka 官方仓库已把经典 "Panda" 型号重命名为 `fer`（Franka Emika Robot），物理几何/DH 参数与原 Panda 完全一致，`joint_limits.yaml` 数值可交叉验证。
- MJCF 没有对应 apt 包，且体积不大（~33MB mesh），vendor 进仓库比运行时依赖一个未打包的第三方仓库更可复现；已在 `.gitignore` 外单独保留（不受 build/install/log 忽略规则影响）。
- `moveit_resources_panda_moveit_config` 虽然包名仍叫 `panda`（历史遗留测试资源包名，未随官方改名同步），但里面的 joint_limits/kinematics 数值与 `franka_description` 的 `fer` 一致，可直接作为 Chap 6 `motion_planner` 包的参考配置起点。

## 1. Frame 约定

**权威规则：TF 里的 frame 名 = MJCF 的原生 body 名**，唯一例外是合成的 `hand_tcp`。`mujoco_bridge` 是仿真侧唯一的 ground-truth 发布者，它从 `mjModel` 自动推导整棵树，不做任何名字翻译（决策与理由见 week1.md 9.9、9.10）。**URDF 侧的名字（`fer_` 前缀、`link8`、`base`、`leftfinger`）不出现在 TF 里**，见下方"URDF 侧命名差异"。

| Frame | 父 frame | 说明 | 发布方式 |
| --- | --- | --- | --- |
| `world` | - | 全局固定 frame，仿真与真实世界坐标原点。MuJoCo 的隐式 body 0，也是 TF 树的唯一根 | - （作为根，不作为任何变换的 child） |
| `link0` | `world` | 机器人基座，固定于桌面。**注意不叫 `base_link`**——MJCF 里这个 body 就叫 `link0`，URDF 侧对应 `fer_link0`（其父是 URDF 独有的 `base`，TF 里不存在） | static（当前为单位变换） |
| `link1` .. `link7` | 链式（`link{i}` 的父为 `link{i-1}`） | Panda 7-DoF 关节链，命名沿用 `franka_description` 的无前缀形式 | dynamic（`mujoco_bridge` 按仿真步发布） |
| `hand` | **`link7`** | 夹爪基座。**TF 里没有 `link8`**：MJCF 把 URDF 的 `link7→link8→hand` 两步合并成一步（`pos="0 0 0.107"` + 绕 z 转 −45°），数学上等价（已逐项核对，见 week1.md 4.3.3） | static（相对 `link7` 固定） |
| `hand_tcp` | `hand` | 工具中心点（TCP），抓取位姿以此为参考 | static，由 `mujoco_bridge` **合成发布**（见下方说明，MJCF 里没有这个 frame） |
| `left_finger` / `right_finger` | `hand` | 两指夹爪指尖 body，关节为 `finger_joint1`/`finger_joint2` | dynamic |
| `camera_link` / `camera_optical_frame` | `world` 或固定支架 link | RGB-D 相机外参；只能由 TF 发布一份 | static |
| `object` | `world` | 目标物体 ground-truth/估计位姿 | dynamic（free joint 的 body，由 `mujoco_bridge` 自动推导，无需额外代码） |

规则：

- **每条 TF 边只允许有一个发布者。** tf2 不会对重复发布的同一条边报错，只会按到达顺序反复覆盖，表现为位姿抖动/跳变。具体两条后果：
  - 相机外参只允许在 `mujoco_bridge` 中以一份 static TF 发布，禁止感知节点手写第二套外参（对应第3.2节）。
  - **不得让 `robot_state_publisher` 发布 `/tf`**（它默认会发 `link0..link7` 这几条边，和 `mujoco_bridge` 直接冲突）。第6周接 MoveIt 时需要 `/robot_description`，届时应 remap 掉 rsp 的 `/tf`、`/tf_static`。职责划分：**URDF 负责"长什么样和怎么规划"，MuJoCo 负责"现在在哪"**。
- ground truth 与视觉估计的 `object` frame 使用同一命名，但通过不同 topic 区分（oracle vs vision），不得混用。
- 抓取/规划模块统一以 `hand_tcp` 作为末端参考 frame，不直接用 `hand`（TCP 已经把夹爪长度的偏移量算进去，避免每个模块各自加一遍）。

**`hand_tcp` 合成说明**：MuJoCo 的 `panda.xml` 里**没有** `hand_tcp` 这个 body/site。这个 frame 来自 URDF 侧 `franka_description/end_effectors/common/franka_hand.xacro` 的 `hand_tcp_joint`，是一个**空 link**（无 visual/collision/inertial），即纯粹的命名坐标系：不参与动力学、不参与碰撞、不占 DoF。偏移量 `xyz="0 0 0.1034"`, `rpy="0 0 0"`（相对 `hand`）。`mujoco_bridge` 额外手动合成一条 `hand -> hand_tcp` static TF，这样下游抓取/规划模块仍能拿到 `hand_tcp`。

> **更正（Stage C）**：本文档此前写"TCP 已经把夹爪长度和**默认 45° 旋转**的偏移量算进去"，**这条是错的**。`tcp_rpy` 默认为 `0 0 0`，`hand_tcp` 是**纯 103.4mm 平移**；那个 −45° 的手腕旋转在 `hand_joint`（URDF 的 `link8→hand`）上，MJCF 里折进了 `hand` body 自己的 `quat`。

**`0.1034` 是一份手抄的副本**：它的权威出处是 `franka_description` 的 xacro（`tcp_xyz` 默认值），而 `franka_description`（apt）和 `panda.xml`（vendor）之间没有任何构建步骤，上游改了这里不会有任何东西告警。当前代码里以具名常量 `kHandToTcpZ` 出现（[mujoco_bridge_node.cpp](../src/mujoco_bridge/src/mujoco_bridge_node.cpp)），注释内附核对命令：

```bash
xacro $(ros2 pkg prefix franka_description)/share/franka_description/robots/fer/fer.urdf.xacro
```

**Stage L 决定不修改 vendor MJCF**：模型一致性测试用 `mj_jac` 对 `hand` body 上的任意 world-space 点求 Jacobian，TCP 点由 `hand` 位姿和局部 `[0, 0, 0.1034]` 合成；运行时 TF 继续沿用同一合成方式。这样保留 vendor 文件可直接和上游比较，代价是 `0.1034` 仍有代码副本。自动三方测试会在该副本漂移时失败，因此风险从“无人发现”降为“测试门禁可见”。若以后有 MuJoCo 原生 site 的运行时消费者，再用项目自有 overlay MJCF 增加 site，而不是直接修改 vendor 文件。

**URDF 侧命名差异（已知上游差异，不是 bug）**：展开 `fer.urdf.xacro` 后，URDF 侧和 MJCF/TF 侧的对应关系是：

| MJCF / TF（权威） | URDF (`fer.urdf.xacro` 展开后) | 差异 |
| --- | --- | --- |
| `world`（树根） | `base`（树根） | 名字不同 |
| `link0` .. `link7` | `fer_link0` .. `fer_link7` | `fer_` 前缀 |
| —（不存在） | `fer_link8` | URDF 多一级法兰 frame |
| `hand` | `fer_hand` | 前缀 |
| `hand_tcp`（合成） | `fer_hand_tcp` | 前缀 |
| `left_finger` / `right_finger` | `fer_leftfinger` / `fer_rightfinger` | 前缀 **且**无下划线 |

任何需要跟 URDF 侧工具（MoveIt 配置等）对照的代码**必须显式处理这张表**，不能假设两边字符串相同。注意 `hand:=true` 参数下 URDF 会强制加 `fer_` 前缀而不管 `no_prefix` 设置。

## 2. 关节命名与顺序

以 `franka_description`（`fer.urdf.xacro`）与 MuJoCo Menagerie（`panda.xml`）交叉核对，两者关节命名和限位数值完全一致：

| 关节 | 类型 | 下限 (rad) | 上限 (rad) | 最大速度 (rad/s) | 最大力矩 (N·m) | 备注 |
| --- | --- | --- | --- | --- | --- | --- |
| `joint1` | revolute | -2.8973 | 2.8973 | 2.175 | 87 | |
| `joint2` | revolute | -1.7628 | 1.7628 | 2.175 | 87 | |
| `joint3` | revolute | -2.8973 | 2.8973 | 2.175 | 87 | |
| `joint4` | revolute | -3.0718 | -0.0698 | 2.175 | 87 | 全负区间，注意零位不在区间中点 |
| `joint5` | revolute | -2.8973 | 2.8973 | 2.61 | 12 | |
| `joint6` | revolute | -0.0175 | 3.7525 | 2.61 | 12 | 几乎全正区间 |
| `joint7` | revolute | -2.8973 | 2.8973 | 2.61 | 12 | |
| `finger_joint1` | prismatic (slide) | 0 | 0.04 | - | - | 夹爪左指，MuJoCo 用 `class="finger"`，轴 `0 1 0` |
| `finger_joint2` | prismatic (slide) | 0 | 0.04 | - | - | 夹爪右指，与 finger_joint1 通常做镜像/等值驱动 |

- **关节顺序**：URDF `JointState.name` 顺序与 MuJoCo `qpos`/`ctrl` 顺序均为 `joint1, joint2, ..., joint7`（再加 `finger_joint1, finger_joint2`），两边一致，不需要额外映射表。若后续 `arm_prefix` 参数被设为非空（多臂场景），需要重新核对。
- **零位 / 初始位姿**：**手边存在三处互不一致的"初始位姿"定义，见下面第 2.1 节。** `mujoco_bridge` 的 `~/reset` 复位到 MJCF `keyframe` 的 `home`。
  > 更正记录（Stage D）：本条原先写的是"即 Franka 官方标准 home 姿态"，**这句是错的**。Franka 生态里的标准起始位姿是 SRDF 的 `ready`，和 MJCF 的 `home` 是两个不同构型（joint7 连符号都相反）。发现方式是 Stage D 讲解时去核对 SRDF，详见 [week1.md 10.3.2](../Job_guides/my_study/week1.md#1032-三处初始位姿互不一致第6周会咬人)。
- **轴方向**：所有主关节在 MuJoCo 里默认 `axis="0 0 1"`（各 link 自身局部 Z 轴），与 URDF xacro 里的 joint `axis` 定义在同一约定下应一致；后续第3周做 FK 对照测试时需要用有限差分/解析 Jacobian 交叉验证，不能只凭文档假设。

### 2.1 三处"初始位姿"定义互不一致（权威对照表）

| 来源 | 文件 | joint1 | joint2 | joint3 | joint4 | joint5 | joint6 | joint7 | 手指 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **MJCF** `<key name="home">` | `robot_description/mujoco/franka_emika_panda/panda.xml:281` | 0 | **0** | 0 | **−1.5708** | 0 | 1.5708 | **−0.7853** | 0.04 |
| **SRDF** `group_state name="ready"` | `/opt/ros/humble/share/moveit_resources_panda_moveit_config/config/panda.srdf:20` | 0 | **−0.785** | 0 | **−2.356** | 0 | 1.571 | **+0.785** | 0.035（`open`） |
| **`ros2_control` fake system** | 同目录 `initial_positions.yaml` | 0 | −0.785 | 0 | −2.356 | 0 | 1.571 | 0.785 | —— |
| **URDF** | `fer.urdf.xacro` | 没有这个概念（URDF 不含状态；隐含约定是全零位） | | | | | | | |

**当前权威**：`mujoco_bridge` 的 `~/reset` 用 MJCF keyframe。Stage G 后带 box 的默认场景使用 **`pick_place_home`**，由 `reset_keyframe_name` 参数选择；表中 vendor 的 `home` 是仅机械臂模型的历史基准。当前场景的 keyframe 在相同机械臂构型后补足 box 的 7 维 freejoint 位姿。

**已知后果**：第6周接 MoveIt 后，MoveIt 的 `setNamedTarget("ready")` 和我们的 `~/reset` 指向两个不同构型，**不会有任何报错**，只表现为"点了 reset，MoveIt 说当前不在 ready 位姿"。第6周必须显式定权威（倾向对齐 SRDF 的 `ready`，因为它是真机生态的约定俗成），并补一份 ADR。已进第4节待办。

**另外两条关于 keyframe 的事实**（Stage D 实测，详见 [week1.md 10.3.1](../Job_guides/my_study/week1.md#1031-keyframe-是一组完整的状态快照不只是-qpos)）：

- `<key>` 除 `qpos`（9 个，= `nq`）外还带 `ctrl="0 0 0 -1.57079 0 1.57079 -0.7853 255"`（8 个，= `nu`）。`mj_resetDataKeyframe` **会一并恢复 `ctrl`**，所以复位后伺服目标与新 `qpos` 自洽，不会把机械臂拽回旧目标。
- `ctrl` 末位 `255` 是夹爪 actuator 被上游重映射后的 `ctrlrange`（`0..255`，不是 `0..0.04`），对应 `qpos` 里的 `0.04 0.04`。**写 `ctrl` 必须按 actuator id 索引，不能用 joint id。**

### 2.2 reset 的语义边界

`mujoco_bridge` 的 `~/reset`（`std_srvs/srv/Trigger`）定义为：

| | 行为 |
| --- | --- |
| **复位** | `qpos`、`qvel`、`act`、`ctrl`、mocap（即 `mj_resetDataKeyframe` 的全部作用域） |
| **不复位** | `mjData::time`。代码显式存旧值再写回，**sim time 保持单调** |
| **附带** | 复位后调 `mj_forward` 刷新派生量（`xpos`/`xquat`/`qfrc_*`） |

**为什么不复位时间**：本节点是全系统 `/clock` 的唯一来源，回退 `/clock` 等于对全图做一次时间倒流（tf2 buffer 清空、stamp 变成"未来"），而且是静默的。对照 Gazebo 的 `/reset_world`（只复位状态）vs `/reset_simulation`（连时间一起清零）——我们实现的是前者。若将来需要后者，应是**另一个 service**，不是给这个加字段。

**这是仿真专有接口**：真机上没有语义对应物（关节不能瞬移）。`~/reset` 用私有名（解析成 `/mujoco_bridge/reset`）而非全局 `/reset`，目的就是让换到真机时**立刻失败于"服务不存在"**。真机上"回到初始位姿"的正确形态是 action（规划一条轨迹、可取消、有 feedback），不是 service。详见 [week1.md 10.4](../Job_guides/my_study/week1.md#104-真机上误用-reset-会发生什么)。

## 3. MuJoCo 与 MoveIt 模型一致性

Stage L 已在 [test_model_consistency.cpp](../src/mujoco_bridge/test/test_model_consistency.cpp) 建立自动三方门禁。测试展开官方 `fer.urdf.xacro`/`fer.srdf.xacro`，直接构造 `moveit::core::RobotModel`/`RobotState`；MuJoCo 侧自行加载 `panda.xml`，把同一组 `q` 直接写入各关节的 `jnt_qposadr`，再调用 `mj_forward`。它不启动仿真节点、不经过 actuator 或 position servo，因此只测模型和运动学实现。

- 固定样例：全零、MJCF `home`、SRDF `ready`、近伸直、贴近限位，以及两组固定随机构型，共 7 组。
- FK 覆盖：逐构型比较 `link0..link7`、URDF 独有的 `link8`、`hand` 和合成 `hand_tcp`；三个可用数据源之间逐对比较。姿态比较使用相对旋转角，不逐分量比较四元数。
- Jacobian 约定：`hand_tcp` 参考点、world 表达、`[vx, vy, vz, wx, wy, wz]`；MuJoCo 侧按 `jnt_dofadr` 取 7 个臂关节列，而不是假设列号等于 `qpos` 地址。
- 门禁阈值：位置误差 `< 1e-6 m`，姿态误差 `< 1e-6 rad`，Jacobian 任一元素绝对误差 `< 1e-8`。
- 首次实测最大值：位置 `6.87e-16 m`，姿态 `3.49e-8 rad`，Jacobian 元素 `1.78e-15`。姿态残差集中在 MJCF `hand` 的截断四元数 `0.9238795 ... -0.3826834`；它远低于门槛，但不是严格的机器零。
- **已有的一次手工交叉验证（Stage D）**：用运行中 position servo 的状态测得 `link4` 差 2mm / 0.3°、`hand_tcp` 差 7mm。Stage L 的直接状态写入结果证明那些差值来自伺服稳态误差，不是模型不一致。运行态验证和模型验证不能共用容差或输入方法。

碰撞几何不是全模型统一的一份简化：`link0..link4`、`link6`、`link7` 和 `hand` 的 collision STL 在 apt `franka_description` 与 vendor MJCF 中逐文件 SHA256 相同；`link5` 在 URDF 中是一份 `link5.stl`，MJCF 中则拆成三个 `link5_collision_*.obj`；手指在 URDF 中由 4 个 box 组成，MJCF 使用 `finger_0` mesh 加 5 个 fingertip box。结论是主臂多数 link 共享同源 mesh，但不能据此假设 MoveIt 与 MuJoCo 的全身碰撞结果完全一致。

## 4. 待办

- [x] 选定 Panda URDF 来源：`franka_description`（apt，`fer` 型号）
- [x] 选定 MJCF 来源：MuJoCo Menagerie `franka_emika_panda/`（vendor 进 `robot_description/mujoco/`）
- [x] 填写关节命名表、零位、限位（见第2节）
- [x] 定下 TF frame 命名权威规则（MJCF 原生名 + 合成 `hand_tcp`），并与 `mujoco_bridge` 实现核对一致（Stage C）
- [x] 决定 `hand_tcp` 不直接写进 vendor MJCF；Stage L 用 `mj_jac` 任意点接口并由三方测试监控 `0.1034` 副本漂移（见第1、3节）
- [ ] 第6周接 MoveIt 时决定 `robot_state_publisher` 的 TF remap 方案，并补一份 `docs/adr/`
- [ ] **第6周决定"初始位姿"以 MJCF `home` 还是 SRDF `ready` 为权威**（见第 2.1 节），同样需要 ADR
- [x] 定下 `~/reset` 的语义边界（复位状态不复位时间、仿真专有接口用私有名）——见第 2.2 节（Stage D）
- [ ] 记录相机外参数值来源与标定方式（相机型号/安装位置尚未选定）
- [x] 确认 URDF 与 MJCF 碰撞几何并非全身同一表示；相同与不同部分见第3节
- [x] 编写自动三方测试：7组固定 `q`，逐 link 对比 MuJoCo、MoveIt 与自写 FK/Jacobian（Stage L）
- [ ] keyframe 长度不匹配会被静默补零（不报错，见第 0.1 节）——需要至少一个 gtest 防止手滑改错 `qpos` 长度却没人发现（Stage G 实测，见 [week2.md 8.8](../Job_guides/my_study/week2.md#88-排查记录keyframe-名字冲突与长度不匹配是两件独立的事结论被推翻)）
- [ ] `gripper_actuator_id_`/`kHandBodyName="hand"`（`mujoco_bridge_node.cpp`）目前是隐式单机械臂假设：模型里有第二个夹爪/第二个 `hand` body 会静默覆盖或找不到，不报错。解锁条件：真正引入第二条机械臂（第 2.2 节"暂不进入 MVP"包含双臂，当前不修）
- [x] Stage N 已用默认 `close_settle_s=2.0s`、`lift_settle_grace_s=2.0s` 和 `grasp_position_epsilon_rad=0.3rad` 重跑 20 次固定场景，20/20 成功、零重试；这只验证了当前组合，没有证明这些阈值是最小可行值或适用于在线伺服
- [ ] `task_executor` 的 `CMakeLists.txt`/`package.xml` 直接 `find_package(mujoco_bridge REQUIRED)`，只为了拿 `grasp_criteria` 这段纯函数库——历史顺序造成的（`classifyGrasp()` 在 `task_executor` 包存在之前就已经长在 `mujoco_bridge` 里），不是刻意设计。仓库里已有空占位包 `manipulation_interfaces` 可以承载这类两边共享的纯函数，当初没有搬。解锁条件：第6周 `mujoco_bridge` 真的被换/重命名成真机驱动包时，评估要不要把 `grasp_criteria` 挪进 `manipulation_interfaces`

## 5. Pick-and-place 场景（Stage G）

新增 [pick_place_scene.xml](../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml)（新文件，不修改任何 vendor 文件，见第 0.1 节的同目录约束），`<include>` 上 vendor 的 `scene.xml`（连带 `panda.xml`）。

| 项 | 数值 | 说明 |
| --- | --- | --- |
| 桌面 | `box` geom，`size="0.3 0.4 0.02"`，`pos="0.5 0 0.2"` | 无 body 包裹（结构上等价于 `floor`），顶面 z=0.22，不产生 TF 帧 |
| box（可操作物体） | `size="0.02 0.02 0.02"`（4cm 边长立方体），`mass="0.05"`（50g），`pos="0.5 0 0.241"` | 带 `freejoint`，父级为 world；`body_jntnum!=0` 使其自动获得 `world -> box` 动态 TF（零代码改动，Stage C 结构判据的首次真实检验） |
| 接触参数 | `condim="3"`，`friction="1 0.03 0.003"`，`solref="0.01 1"` | 抄自 vendor `mjx_single_cube.xml`，未针对本场景重新验证；三个摩擦系数含义是（滑动、扭转、滚动），`condim=3` 下只有滑动系数生效，扭转/滚动当前是死代码（升级 `condim` 到 4/6 才会激活，届时需要重新调参） |
| 放置区标记 | `place_marker` geom，`type="cylinder"`，`pos="0.5 0.3 0.221"`，`contype="0" conaffinity="0"` | 纯视觉参考，无碰撞，不是判据的一部分（判据在 Stage H 定义） |
| reset keyframe | `pick_place_home`（不是 `home`，见第 0.1 节的名字冲突） | `qpos` 长度 16（=nq），顺序为 [7 臂关节, 2 手指关节, 3 box 平移, 4 box 四元数]——这个顺序由 body 在合并后 worldbody 里的**文档顺序**决定（深度优先遍历），不是硬性规则；`pick_place_scene.xml` 自己的 `<worldbody>` 块写在 `<include>` 之后，所以 box 排在臂的关节之后 |
| 模型加载路径 | `model_path` 参数（默认指向本文件） | `mujoco_bridge_node` 的 ROS 参数，可覆盖回 `panda.xml` |
| reset keyframe 名 | `reset_keyframe_name` 参数（默认 `pick_place_home`） | 换模型必须同时覆盖这个参数，但两个方向的错法后果完全不同：把 `model_path` 换回 `panda.xml` 却忘记把这个也改回 `home` → `~/reset` 找不到 `pick_place_home` 这个名字，**显式失败**（`~/reset` 返回 `success=false`）；反过来，`model_path` 留在 `pick_place_scene.xml` 却把这个改回（或忘了从）`home` → `home` 这个名字**确实存在**（vendor 的 keyframe 被 `<include>` 原样带入），只是长度不对，`~/reset` 返回 `success=true`，box 被静默传送到补零位置（见第 0.1 节）。**只有第一个方向会显式报错**，第二个方向完全没有报错 |

**ground-truth oracle 接口**（对应计划书"ground truth 必须走独立接口"的硬要求）：

| 项 | 值 |
| --- | --- |
| topic | `~/ground_truth/object_pose`（解析为 `/mujoco_bridge/ground_truth/object_pose`） |
| 类型 | `geometry_msgs/msg/PoseStamped` |
| frame_id | `world`（绝对位姿，不是父相对——因为 box 的父级本来就是 world，见 [week2.md 8.4](../Job_guides/my_study/week2.md#84-qpos-与-xposxquat为什么-oracle-发布读后者)） |
| 数据来源 | `mjData::xpos`/`xquat`（不是 `qpos`——理由见 week2.md 8.4：`qpos` 只在"父级恰好是 world"时才等于世界坐标，这个假设一旦物体被放进可移动容器就会静默失效） |
| 发布频率 | 与 `/tf` 共享 `tf_decimation_`（不是独立频率），目的是让这条话题和 `world -> box` 的 TF 帧互相印证同一物理步；已实测两者数值一致（差异仅浮点噪声量级） |
| 存在性 | 仅当模型里有名为 `box` 的 body 时才创建；`panda.xml` 单独加载时该话题不存在（日志显式说明，不是空话题） |
| 这是唯一合法的 oracle 源 | 第4周接视觉估计后，感知节点的输出必须发布到不同的话题，禁止复用这个名字或把估计值伪装成 ground truth |

## 6. 技术选型与项目定位：MuJoCo vs Gazebo，复用 vs 自建

### 6.1 为什么选 MuJoCo 而不是 Gazebo

- 对照过的 JD（`/media/anby/Data2/Docs/obsidian/robotics_work/JDs/md/_doc/`）中，涉及仿真平台的岗位点名的都是 **MuJoCo / Isaac Sim / Isaac Gym / Genesis / Pinocchio**，没有一份点名 Gazebo。Gazebo 在 ROS 生态里的定位更偏"导航/感知集成"的默认仿真器；MuJoCo 是 RL / sim2real / 具身智能这条线的事实标准，接触力学求解器的精度和速度都明显更好。
- 反证见 6.3：公开的 Gazebo 版 pick-and-place 项目普遍靠 `gazebo_ros_link_attacher` 之类的插件"运动学焊接"物体到夹爪上（离得够近就 attach 成一个刚体），绕过真实接触力学判定抓取是否成功。这不是 Gazebo 不能用，而是它的接触精度不足以支撑"基于真实接触力判定抓取成功"这件事——而这正是本项目 `grasp_criteria`/`grasp_state` 要做的。
- ROS2 + MuJoCo 确实比 ROS2 + Gazebo 多一层桥接工作（Gazebo 有现成 `ros_control`/`gazebo_plugins`，MuJoCo 没有），但这层工作本身就是目标能力（可复现仿真、仿真与真机校准闭环），不是可以省略的额外开销。

### 6.2 复用 vs 自建的判断标准

| 判断 | 举例 | 处理方式 |
| --- | --- | --- |
| 成熟库已把正确性和性能做到位，JD 不要求手推 | 标准 FK、常规运动规划 | 直接用 MuJoCo API / MoveIt / Pinocchio，不重写进 `mujoco_bridge` |
| JD 明确要求理解底层数学（如 TAKS 岗位要求 FK/IK/雅可比/DLS/零空间/奇异处理） | FK/IK 推导 | 用独立的小练习验证理解，不嵌入主系统 |
| 涉及物理仿真保真度、可测试性、C++/ROS2 系统集成，且没有现成库覆盖"这个桥接层" | `grasp_criteria`/`grasp_state`、接触力探针、FSM | 自己写，并配单元测试——这是本项目相对"调库拼图"类项目的差异化部分 |
| 成熟的算法/生成器已经解决且没有验证需求 | 抓取姿态生成（GraspIt 类）、点云物体识别 | 调库，把精力放在"能否正确、可验证地接入系统"而不是重新实现算法本身 |

### 6.3 对照：三个公开 pick-and-place 项目的取舍

| 项目 | 架构 | 抓取判定 | 备注 |
| --- | --- | --- | --- |
| [Salman-H/pick-place-robot](https://github.com/Salman-H/pick-place-robot) | Gazebo + RViz + MoveIt，核心自写代码只有一个 IK_server（Sympy 解析解 KUKA KR210） | 依赖仿真器固定流程，无自定义抓取验证 | 唯一"自建"的部分是 IK，而这恰好是 JD 通常不要求在生产系统里手写的部分 |
| [pietrolechthaler/UR5-Pick-and-Place-Simulation](https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation) | vision（YOLOv5）+ motion_planning + `gazebo_ros_link_attacher` | 靠 link-attacher 插件运动学"焊接"，非真实接触力学判定 | "结构简单"的代价是抓取成功与否完全没有物理验证 |
| [gstavrinos/ez_pick_and_place](https://github.com/gstavrinos/ez_pick_and_place) | MoveIt + GraspIt 的胶水代码（`ez_tools.py`） | 无自定义判定，README 让用户自己看源码 | 作者本人在 moveit 仓库报过 `/compute_ik` 一直失败且未修的 issue——生产代码里留着未解决的集成 bug，缺测试覆盖 |

三者的共同点：都合理地复用了 MoveIt/GraspIt 做规划和抓取姿态生成，但都把"抓取成功"这个最难验证、最容易出 bug 的环节跳过或简化掉了。本项目在 `mujoco_bridge` 上投入的时间，对应的正是这个被普遍跳过的验证层，而不是重新实现 MoveIt/GraspIt 已经做好的规划或抓取姿态生成能力。

## 7. `task_executor` 任务状态机（Stage I）

新增 [task_executor](../src/task_executor/)（C++ 节点，`use_sim_time=true`）。状态机固定顺序：`HOME → PREGRASP → GRASP → CLOSE → LIFT → PREPLACE → PLACE → OPEN → RETRACT → VERIFY → DONE`，异常出口 `RECOVER`（重试，回 `HOME`）→ 耗尽 `max_retries` 后 `FAILED`。核心决策函数 `step()`（[fsm.hpp/cpp](../src/task_executor/include/task_executor/fsm.hpp)）不依赖 `rclcpp`/`mjModel`，是 Stage F 定义的 Layer 1，19 个 gtest。

**`grasp_criteria` 库的复用**：`mujoco_bridge` 的 `classifyGrasp()` 拆成独立 CMake 库目标 `mujoco_bridge::grasp_criteria` 并导出（`ament_export_targets`），`task_executor` 直接链接、不重新实现。Stage I 时节点从 `/joint_states` 与 `~/ground_truth/*` 拼 `GraspSignals`；第三层 reset 重构后改从同一步 `BridgeObservation` 生成信号。仍**没有**新增 `GraspOutcome` ROS topic：谁掌握阶段信息就拥有分类计算权。

**`WaypointSource` 抽象**：`jointTargetFor(Phase, ObjectPose) -> JointTarget` 接口。`KeyframeWaypointSource` 是固定查表并忽略 `object_pose`；Stage O 的默认 `DiffIkWaypointSource` 从实测关节状态作 seed，消费独立的 `CartesianWaypointSource` 并调用离线 `solveIk()`，阶段内缓存目标。节点参数 `waypoint_source:=keyframe` 可切回查表；`fsm.cpp` 未修改。节点需要选择源、设置 seed、处理 IK 失败及记录诊断，所以“节点也不需要改”的旧预期并未成立。

**Stage O 后的第一层职责整理**：`TaskExecutorConfig` 集中声明和校验 `fsm.*`、`grasp.*`、`target.*`、`verify.*` 及 waypoint 模式；`verify.*` 同时映射到 `FsmParams` 的 VERIFY 判据。`ObservationSnapshot` 从 joint name 对齐的关节状态、物体 pose、接触布尔值和 `world -> hand_tcp` TF 组成一次决策输入；缺任何必要观测则等待。`EpisodeTelemetry` 按阶段存一条记录，在发布时转换为原有 `EpisodeOutcome` 并行数组。话题、消息、参数默认值和 keyframe 的历史验收点 `(0.43, 0.31)m` 不变。快照仅保证本次 20 Hz tick 后续使用同一组已读值；各 ROS 话题仍可能来自不同仿真时刻，reset 后的新鲜度协议尚未实现。

**第二层目标配置**：`TaskExecutorConfig.task`（`PlacementTask`）表达 world-frame TCP 放置目标 XY、高度与工具旋转；`TaskExecutorConfig.verification`（`PlacementVerification`）表达 box 落点验收圆心和半径。`target.*` 与 `verify.*` 的参数名和默认值保持不变。`diff_ik` 模式启动时默认要求两组 XY 相等；故意不一致的实验必须显式设置 `verify.allow_target_mismatch:=true`，否则配置加载抛错。`keyframe` 不消费任务 Cartesian 目标，继续使用历史验收中心 `(0.43, 0.31)m`，免除此检查。启动和每个 episode 的日志均打印两组 XY；CSV 契约不变。决策见 [ADR 002](adr/002-placement-target-verification-contract.md)。

**第三层 reset/观测契约**：`/mujoco_bridge/reset_with_generation`（`ResetScene.srv`）在每次成功 reset 后返回 bridge 会话标识和严格递增的 generation；原 `/mujoco_bridge/reset` Trigger 仍可用，也会使 generation 递增。会话标识在每次 bridge 启动时重建，防止重启后计数复用。`/mujoco_bridge/episode_observation`（`BridgeObservation.msg`）在一个物理步后打包会话、generation、单调样本序号、同一仿真时间戳的 9 关节状态、box pose、world-frame `hand_tcp` 变换和双指接触。executor 不再从旧话题及 TF buffer 混合取样；P2 起由 `EpisodeController` 只放行本次 reset 会话与 generation 且序号递增的完整观测，旧 `ResetGate` 已删除。旧会话或 generation 被拒绝，同会话下更高 generation 报 `RESET_SUPERSEDED`；服务或观测等待、运行中断流在 5s 墙钟看门狗下分别报 `RESET_UNAVAILABLE`、`RESET_FAILED` 或 `OBSERVATION_STALE`。旧观测话题继续给 RViz 和其他消费者使用。原协议决策见 [ADR 003](adr/003-reset-generation-observation.md)。

`task_executor` 是新 reset 服务的仓库内客户端；旧 Trigger 服务仍注册，供手动及可能的外部调用，两个服务共用同一 reset 实现。观测包的“同一步”来自 bridge 在一次物理步后读取同一份 `mjData`，不是对异步传感器做时间戳匹配。executor 使用包内的 world-frame TCP，不读取单独发布的 `/tf`；`/tf` 和 `/joint_states` 继续服务可视化链路，`episode_runner.py` 仍从独立的 `~/ground_truth/object_pose` 记录最终 box pose。保留这些旧输出会重复发布部分物理量，但旧话题的其他订阅者不会自动获得观测包的原子性保证。

**手测出来的关节空间 waypoint**（`KeyframeWaypointSource`，单位 rad，顺序 joint1..joint7；`box` 初始位姿见第5节表格）：

| 阶段 | joint1 | joint2 | joint4 | 其余关节 | 夹爪宽度 (m) |
| --- | --- | --- | --- | --- | --- |
| HOME | 0 | 0 | -1.5708 | joint3=0, joint5=0, joint6=1.5708, joint7=-0.7853（= MJCF `pick_place_home` 的臂部分） | 0.08（开） |
| PREGRASP | 0 | 0.2 | -1.6 | 同上 | 0.08 |
| GRASP / CLOSE | 0 | 0.4 | -2.0 | 同上 | GRASP=0.08，CLOSE=0.0（命令全闭，见下） |
| LIFT | 0 | 0.2 | -1.6 | 同上 | 0.0（保持闭合） |
| PREPLACE | 0.62 | 0.2 | -1.6 | 同上 | 0.0 |
| PLACE | 0.62 | 0.27 | -1.75 | 同上 | 0.0 |
| OPEN / RETRACT | 0.62 | 0.27→0.2 | -1.75→-1.6 | 同上 | 0.08（松开） |

**`kClosedWidthM` 为什么是 0.0（命令全闭）而不是"比 box 宽度窄一点"（0.03，常规写法）**：0.03（比 box 的 0.04m 窄 1cm）能撑住纯垂直的 `LIFT`，但撑不住 `PREPLACE` 需要的 `joint1` 旋转（一次侧向摆动）——实测看到接触从双指变成单指、box 绕着剩下的接触点转出去、跌回桌面。0.0（伺服朝"完全闭合"尽力推，被 box 挡住后停在比 0.03 更紧的挤压力度）实测能撑过同一段摆动，一路验证到 `PLACE`→松开→box 落在 `(0.42, 0.31)` 附近。这是纯物理调参，不是逻辑修正——见 [week2.md 10.10.5](../Job_guides/my_study/week2.md#10105-一次独立的物理调参夹爪闭合力度不够撑不住-kpreplace-的侧向摆动)。

**到位判据的三层等待时间**（`FsmParams`，详见 [week2.md 10.6](../Job_guides/my_study/week2.md#106-到位判据为什么分层三条独立的等一等不是同一件事)）：

| 常数 | 默认值 | 回答的问题 |
| --- | --- | --- |
| `min_settle_s` | 0.5s | 这一 tick 是不是上一阶段留下的旧读数？（所有阶段通用） |
| `close_settle_s` | 2.0s | `kClose` 专用：夹爪是不是刚开始收紧、伺服还没收敛？ |
| `lift_settle_grace_s` | 2.0s | `kLift` 专用：手臂已到位，box（独立仿真体，靠摩擦耦合）有没有跟上？实测滞后接近 1s |

**Stage I 的历史验收标准**：`place_x_m`/`place_y_m` 当时默认 `(0.43, 0.31)`——不是 `pick_place_scene.xml` 里 `place_marker` 几何体的名义位置 `(0.5, 0.3)`，是 `KeyframeWaypointSource` 的 `PLACE` waypoint 实际能送到的位置。Stage O 后仅 `waypoint_source:=keyframe` 保留这一默认值；IK 模式默认验证 marker 中心，见第 10 节。`place_region_radius_m` 仍默认 0.08m，Stage O 的严格回归显式设为 0.03m。Stage I 已验证一次完整成功 episode（全 11 阶段到 `DONE`）和一次人为制造的必然失败（`place_x_m/y_m` 设为不可达值，`RECOVER` 重试 3 次后正确落到 `FAILED`）。

- [ ] `close_settle_s`/`lift_settle_grace_s` 是"改到实测通过为止"定的，不是从物理量推出来的；第3周换 diff-IK 后需要重新测（见第4节延伸）

## 8. `arm_kinematics` differential IK（Stage M）

`arm_kinematics` 继续保持纯 C++/Eigen/yaml-cpp 边界，不依赖 ROS、MoveIt 或 MuJoCo。Stage M 新增两层接口：`differentialIkStep()` 计算一次加权 DLS 关节增量，`solveIk()` 在内部反复执行 FK/Jacobian/DLS，返回一个离线求得的最终关节目标。

**冻结的任务空间约定**：参考点为 `hand_tcp`，表达 frame 为 world，六维顺序为 `[x, y, z, rx, ry, rz]`；平移误差单位为 m，旋转误差为轴角向量、单位 rad。加权 Jacobian 使用 `translation_weight=1.0`、`rotation_weight=0.2`，因此 `sigma_min`、条件数和阻尼阈值只能在这套权重下比较，不能作为跨权重或跨机器人的绝对奇异性指标。

**默认单步参数**：`damping_threshold=0.08`、`maximum_damping=0.05`、`joint_centering_gain=0.02`、平移误差裁剪 `0.05m`、旋转误差裁剪 `0.2rad`、单关节最大步长 `0.12rad`、关节限位内缩量 `1e-4rad`。其中 `joint_limit_margin` 只是在官方 `lower/upper` 角度限位基础上的数值内缩量，不是关节下限或机械安全距离。这些值已通过数学单测证明行为有界，但尚未经过 position servo 闭环调参；Stage N 接入执行链后必须重测。

`IkStatus` 当前只有 `kConverged`、`kMaxIterations` 和 `kStalled`。不可达目标返回最后一个有限关节状态和残差，不返回名义成功或 NaN。当前约束实现是“先求 DLS，再裁剪步长并投影到位置限位”，不是把约束直接放进 QP；因此裁剪后的解不保证仍是约束最小二乘最优解。

**能力边界**：`solveIk()` 的迭代反馈来自自写运动学模型，不读取 MuJoCo/真机实际状态。它是用 differential IK 实现的离线 full-pose IK 求解器，不是在线 Cartesian servo，也不是 MPC；不能修正重力下垂、actuator 饱和、接触扰动或 position servo 稳态误差。Stage N 先用它生成离散 waypoint，以便把运动学求解和执行控制分层验证。

**Stage O 已实现**：Stage N 复盘确认，由手调关节目标 FK 反推 Cartesian 目标会把执行端经验补偿混入任务几何。Stage O 新增 message-free 的 `CartesianWaypoint`/`CartesianWaypointSource`，表达 world-frame `hand_tcp` 目标、阶段和夹爪命令；`PickPlaceCartesianWaypointSource` 直接从任务几何构造目标，`DiffIkWaypointSource` 作为离线 IK 适配器消费它，不再包含 `KeyframeWaypointSource` 或由关节目标求任务 pose。旧 keyframe 模式仍可单独运行。MoveIt 规划、`JointTrajectory` 执行器、随机障碍和在线 Cartesian servo 均不在本阶段。决策见 [ADR 001](adr/001-cartesian-task-waypoints.md)。

## 9. Stage N 离线 IK waypoint 与执行验证

`DiffIkWaypointSource` 使用 Stage I 关键帧经自写 FK 得到的 TCP 位姿作为基准；在 `PREGRASP` 首次观测时锁定物体位置，把相对默认 box 中心 `(0.5, 0, 0.241)m` 的平移加到抓取侧 `PREGRASP/GRASP/CLOSE/LIFT` 目标。`PREPLACE/PLACE/OPEN/RETRACT` 保持固定放置侧目标。抓取姿态仍是基准关键帧姿态，当前不随物体偏航角变化；未做碰撞规划。`HOME` 和每次重试重置锁定位置与缓存。每阶段首个命令求解一次，后续 20 Hz tick 重发同一关节目标；不可收敛则不发布解并返回 `IK_FAILED` episode outcome。

阶段日志分别记录 world frame 下 TCP 目标位置/四元数、IK 迭代数、模型内残差、加权 `sigma_min`、实测关节误差及 TF 测得的 TCP 误差。默认场景 20/20 次成功、零重试，最终 box 均值 `(0.43226, 0.30902)m`；向 `+y` 移动 4cm 的场景中 IK 1/1 成功，查表 0/1、三次重试后 `GRASP_EMPTY`。结果见 `results/stage_n_*.csv`；偏移场景在 `stage_n_shifted_scene.xml`。

当前 `close_settle_s=2.0s`、`lift_settle_grace_s=2.0s`、`grasp_position_epsilon_rad=0.3rad` 组合通过 20 次回归，但尚未测最小可行阈值。`GRASP -> CLOSE` 时关节最大误差约 `0.190rad`，实际 TCP 平移误差约 `0.111m`；`PLACE -> OPEN` 时实际 TCP 平移误差约 `0.006m`。以 marker 中心 `(0.5, 0.3)m` 和原有 0.08m 半径验收会通过，但最终 box 约 `(0.4323, 0.3090)m`；半径改为 0.03m 则 `PLACE_MISSED`，三次重试后失败。故默认验收中心仍为实测 `(0.43, 0.31)m`，不能把宽半径成功解释为 marker 中心精确放置。

## 10. Stage O Cartesian 任务目标与离线 IK 适配

`CartesianWaypoint` 含阶段、`world -> hand_tcp` 位姿和夹爪总开口宽度；无 ROS 消息或关节目标。`PickPlaceCartesianWaypointSource` 用物体中心 `(x, y, z)` 产生 `GRASP/CLOSE` TCP 目标，`PREGRASP/LIFT` 高出中心 `0.15m`。抓取侧物体位姿在首次 `PREGRASP` 锁定。放置侧 XY 独立于物体观测，默认为 marker 中心 `(0.5, 0.3)m`；桌面上表面 `z=0.22m`、box 半高 `0.02m`，`PLACE/OPEN` TCP 为放置 box 中心上方 `0.05m`，即 `z=0.29m`，`PREPLACE/RETRACT` 为该中心上方 `0.15m`，即 `z=0.39m`。工具 z 轴朝下，绕 world z 的旋转由独立参数 `target.tool_yaw_rad`（默认 0）指定，抓取与放置保持同一方向；**不再**从目标 `(x,y)` 相对 world 原点的径向角推算 yaw。当前方形 box 不读取其偏航角，也不验证最终箱体姿态。`CLOSE/LIFT/PREPLACE/PLACE` 夹爪命令全闭，其余阶段张开。`HOME` 使用 world 中已知安全位姿 `(0.5545, 0, 0.5211)m`。`target.place_x_m/y_m`、两个高度参数及 `target.tool_yaw_rad` 可覆盖几何值；默认 `verify.place_x_m/y_m` 在 IK 模式为 marker 中心，旧 keyframe 模式仍为实测 `(0.43, 0.31)m`。box 的初始与 reset 位姿来自 MJCF 的 `<body>`/`<keyframe>`，这些 executor 参数不会移动 box。

姿态的精确定义是 `R_world_tcp = Rz(tool_yaw_rad) * R_down`，其中 `R_down` 绕 `(1,1,0)/sqrt(2)` 旋转 π；参数为 0 时 TCP x/y/z 分别指向 world +y/+x/-z。该参数是相对基准姿态绕 world z 的旋转，不是 ZYX Euler yaw。节点仅在构造时读取 `target.*` 和 `verify.*`，无动态更新回调；两组 XY 默认值独立，不会彼此联动，也不会修改 XML marker。IK 模式下的覆盖值若不同，必须显式允许。旧 keyframe 模式不消费 `target.*`。

固定场景以 **3cm** 验收半径连续 20/20 成功、零重试，最终 box 均值 `(0.49742, 0.29532)m`，距 marker 中心均值 `5.35mm`、最大 `6.07mm`。`PLACE` 模型 IK 残差约 `8µm`、关节跟踪最大绝对误差均值约 `0.00666rad`、实际 TCP 位置误差均值约 `6.82mm`；这些量和最终落点误差分别记录，不能互相代替。box 沿 `+y` 移动 `4cm` 时 IK 1/1 成功、最终放置误差 `5.60mm`；同场景 keyframe 0/1，重试三次后 `GRASP_EMPTY`。固定场景 keyframe 1/1 成功。结果见 `results/stage_o_*.csv`，其中每阶段目标、IK 残差、关节误差和实际 TCP 误差与最终 box 落点同列记录；keyframe 没有 Cartesian 诊断时写 `NaN`，最终 `place_error_m` 留空，不伪造零误差。runner 的 `place_error_m` 相对于记录的 PLACE TCP XY，FSM 成功判据和节点最终落点日志相对于 `verify.*`；本次 IK 实验这两个中心一致。这里的 20 次结果来自移除径向 yaw 后的固定 world-frame yaw 版本；早先径向 yaw 版本的均值 `4.62mm` 不再是当前代码的验收数据。验证证明当前固定场景可工作，不证明随机姿态、碰撞路径或真机精度。

Stage O 的最终 smoke 曾暴露 episode reset 竞态：reset 服务虽已成功回复，节点仍可能用 reset 前排队的 `/joint_states` 作 HOME IK seed，实测报 `IK seed is outside the joint limits`。当时靠清本地缓存并等待 0.1s 仿真时间缓解，第三层已改成上述 generation 协议；不再把固定等待视为新鲜度证明。若以后改用多线程 executor，reset 回调与 tick 的状态访问需要重新加同步。

## 11. Week 3.5 episode 编排重构前基线

P0 在未修改 `task_executor` 生产代码的情况下冻结了 episode 编排基线。当前 `onTimer()` 在调用 FSM `step()` 前发布当前 phase 的关节/夹爪目标；phase 未变化时继续重发目标，phase 变化时追加一条 telemetry；`DONE`/`FAILED` 后停止决策和发布，下一次 `start_episode` 才重新进入 `HOME`。episode 运行中再次收到 `start_episode` 也会立即清空本地状态、重新 reset 并从 `HOME` 开始；实测在 `CLOSE` 阶段抢占时 generation 从 `1` 增到 `2`。IK 异常直接发布 `IK_FAILED` 且不增加 retry；retry 会重新 reset 并使 generation 从旧值递增。

实测基线为：默认 diff-IK 固定场景 **20/20 成功、0 retry**，最终 box 均值 `(0.497417, 0.295315)m`，放置误差均值 `5.352mm`；keyframe 固定场景 **1/1 成功**；物体沿 `+y` 偏移 `4cm` 时 diff-IK **1/1 成功**，keyframe **0/1** 并在 3 次 retry 后 `GRASP_EMPTY`；不可达放置目标在 `PREPLACE` 返回 `IK_FAILED`；错误验收中心的 retry 实验使 generation `1 → 2` 后以 `PLACE_MISSED` 失败；旧 generation 观测只得到 `OBSERVATION_STALE` 且没有关节命令。详细 CSV、命令和源码行为契约见 [week3.5 P0](../Job_guides/my_study/week3.5.md#10-p0现有行为基线)。

### P1：ROS-free episode 控制对象

P1 新增了 [`EpisodeController`](../src/task_executor/include/task_executor/episode_controller.hpp)。它接收 start、reset 发送确认与回执、带 session/generation/sequence 的观测和 timer tick，返回无 ROS 的 `EpisodeActions`。`ObservationFrame`、`ResetReceipt` 和 `EpisodeTelemetry` 数据位于不包含 ROS message 的头文件中。P1 尚未接入节点时的边界见 [week3.5 P1](../Job_guides/my_study/week3.5.md#11-p1最小-episode-控制对象)。

### P2：reset 与观测生命周期接管

节点已将 start、reset service 发送/回执、观测准入、5s 墙钟看门狗和 retry 计数交给 `EpisodeController`，并删除 `ResetGate` 的源码、测试和构建引用。service 未就绪时 controller 持续提供同一个 reset token；节点仅在 `async_send_request()` 成功提交后确认发送，旧 token 的回执不能修改当前 episode。节点仍按原 20 Hz timer 消费最新完整观测、设置 IK seed、求目标、执行 FSM、发布命令和填充阶段 telemetry；这些决策将在 P3 迁入 controller。P2 使用 `tickLifecycle()` 处理生命周期而不重复求 waypoint。ROS 话题、服务、参数与 `EpisodeOutcome` 字段未改；实跑覆盖固定成功、错误验收中心 retry、旧观测、service 失败/不可用、连续 start 和不可达 IK。详见 [week3.5 P2](../Job_guides/my_study/week3.5.md#12-p2接管-reset-与观测生命周期)。

### P3：目标、FSM 与阶段诊断接管

`EpisodeController::tick()` 在接受一份未消费的新观测后设置 diff-IK seed、求当前 phase 目标、调用纯 `step()` 并记录迁移 telemetry。动作批次可同时携带当前 phase 的 `TargetCommand`、迁移后的 `PhaseTransition`、reset 意图或一次性 outcome；ROS 节点按目标、日志、reset/outcome 的顺序执行。无新观测时不重发旧目标；每份新鲜观测在 phase 未变时仍会重发当前目标。controller 唯一持有 phase、retry、失败原因和 telemetry；节点仅做消息转换、service/topic 调用和日志格式化。阶段耗时采用 ROS 仿真时间，看门狗采用 steady clock；观测准入会先检查超时，再更新新鲜度。实跑覆盖正常 diff-IK、keyframe retry、不可达 IK 和旧 generation；详见 [week3.5 P3](../Job_guides/my_study/week3.5.md#13-p3迁移-tick-决策)。

### P4：ROS 适配层收窄

`TaskExecutorNode` 的动作执行已集中到 `executeActions()`。节点只负责 ROS entity、消息到 `ObservationEnvelope` 的转换、service/topic 调用、定时触发和日志格式化；它不再直接调用 `step()` 或 `jointTargetFor()`，也不再持有 phase、retry 或 telemetry。动作执行顺序保持为当前目标、迁移日志、reset、outcome，外部 topic、service、参数和 `EpisodeOutcome` 契约不变。正常 episode 与旧 generation 探针在 P4 后复测通过；详见 [week3.5 P4](../Job_guides/my_study/week3.5.md#14-p4收窄-ros-节点)。当前仍以单线程 executor 的串行回调为前提。

### P5：最终回归与架构记录

P5 新增 [ADR 004](adr/004-episode-controller-orchestration.md)，并更新重构前后架构图：[规格](task_executor_episode_orchestration.json) 和 [HTML](task_executor_episode_orchestration.html)。最终全工作区构建和测试为 5 packages、375 tests、0 failures；默认 diff-IK 固定场景连续 20 次全部成功、0 retry，平均放置误差为 5.251mm。旧 generation 探针仍返回 `OBSERVATION_STALE` 且不发布关节命令。该回归结果支持控制器迁移没有改变固定场景行为，但不代表随机姿态、视觉输入或真机线程模型已经验证。

`EpisodeState` 是 reset/观测生命周期状态（Idle、ResetPending、AwaitingResetResponse、AwaitingObservation、Ready、Finished、Failed）；抓取任务的 `Phase` 是另一层状态，只在 Ready 的 FSM tick 中推进。`startEpisode()` 可以从任意状态开始新 reset，terminal 状态只产生一次 outcome。完整转移及动作见 [week3.5 的状态图](../Job_guides/my_study/week3.5.md#42-episodestate-整体状态转移)。
