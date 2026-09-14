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

计划中的根治方案：给 vendor 的 MJCF 的 `hand` body 加 `<site name="hand_tcp" pos="0 0 0.1034"/>`，让数值住在描述层、代码从模型读。

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

**当前权威**：`mujoco_bridge` 的 `~/reset` 用 **MJCF 的 `home`**（`mj_resetDataKeyframe` 按名查 `kResetKeyframeName = "home"`）。

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

对应第3.2节要求：对至少5组固定关节构型比较 MuJoCo 与 MoveIt 2 的末端 FK，位置与姿态误差超过阈值即禁止继续集成。

- 阈值：TBD（建议初始位置误差 < 5mm，姿态误差 < 1°，后续按实测调整）
- 测试位置：`test/`（第一个自动测试，对应第15节任务5）
- **已有的一次手工交叉验证（Stage D）**：用 `robot_state_publisher` + `franka_description` URDF（KDL FK，独立于 MuJoCo 的实现和描述）对 `home` 位姿做了对照，`link4` 差 2mm / 0.3°，`hand_tcp` 差 7mm。**差值来源是 position servo 的稳态误差，不是模型不一致**——伺服靠位置误差产生力矩对抗重力，误差为零就没有力矩。方法和完整数据见 [week1.md 10.7](../Job_guides/my_study/week1.md#107-怎么快速做一次独立的-fk-验证)。
  - 对自动测试的两条约束：**(1) 断言输入必须用实际读到的 `qpos`，不能用 keyframe 标称值**，否则容差得放宽到厘米级；**(2) 四元数不能逐分量比**——`q` 和 `−q` 是同一个旋转，必须比 `|q1·q2| ≈ 1` 或转成角度差。
- 碰撞几何简化记录：TBD——`franka_description` 的 collision geometry 与 MJCF 的 `*_c` collision mesh 都是简化过的凸包/近似几何，两者是否用同一份简化尚待确认，第3周对照测试时一并核对。

## 4. 待办

- [x] 选定 Panda URDF 来源：`franka_description`（apt，`fer` 型号）
- [x] 选定 MJCF 来源：MuJoCo Menagerie `franka_emika_panda/`（vendor 进 `robot_description/mujoco/`）
- [x] 填写关节命名表、零位、限位（见第2节）
- [x] 定下 TF frame 命名权威规则（MJCF 原生名 + 合成 `hand_tcp`），并与 `mujoco_bridge` 实现核对一致（Stage C）
- [ ] 把 `hand_tcp` 的 `0.1034` 搬进 vendor 的 MJCF（`<site>`），消除手抄副本（见第1节）
- [ ] 第6周接 MoveIt 时决定 `robot_state_publisher` 的 TF remap 方案，并补一份 `docs/adr/`
- [ ] **第6周决定"初始位姿"以 MJCF `home` 还是 SRDF `ready` 为权威**（见第 2.1 节），同样需要 ADR
- [x] 定下 `~/reset` 的语义边界（复位状态不复位时间、仿真专有接口用私有名）——见第 2.2 节（Stage D）
- [ ] 记录相机外参数值来源与标定方式（相机型号/安装位置尚未选定）
- [ ] 确认 URDF 与 MJCF 碰撞几何是否为同一份简化（第3节）
- [ ] 编写第一个自动测试：5组固定 `q`，对比 MuJoCo 与 MoveIt FK（对应第15节任务5，需等 `motion_planner`/MoveIt 配置接入后才能跑）
