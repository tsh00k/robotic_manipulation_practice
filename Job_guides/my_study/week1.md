# Week 1 学习笔记

## 学习重点范围

重点是把工程地基打稳。需要重点学习/掌握以下几块：

1. **ROS2 C++ 包体系与 colcon 工作流** — `package.xml`/`CMakeLists.txt`/`ament_cmake` 的分工；一个最小 ROS2 C++ node 怎么写（`rclcpp::Node`、发布者/订阅者、参数、launch 文件）；`colcon build` 的工作区结构（`src/` → `build/` → `install/`），以及为什么用 `source install/setup.bash`。
2. **TF2（坐标系变换系统）** — `tf2_ros::TransformBroadcaster`/`static_transform_broadcaster`；`world → base → tool0 → camera` 这类 frame tree 怎么定义、发布、在 RViz2 里可视化验证。这是计划书反复强调的重点：第2周之前必须把 frame 约定钉死，否则后面感知/规划模块的坐标系错误会很难排查。
3. **URDF/Xacro 基础** — 关节类型、link/joint 层级、原点(origin)与轴(axis)的含义；怎么给 Panda 写/复用 URDF，并搞清楚零位(home position)、关节顺序、限位。这直接对应计划书第3.2节要求的"记录关节名、关节顺序、零位、轴方向、限位"。
4. **MuJoCo 的 C/C++ API 基础** — `mjModel`/`mjData` 的关系，仿真步进 `mj_step`；怎么读取 joint position/velocity、怎么下发 actuator 命令；MJCF 格式（和 URDF 是两套不同的模型描述，第3.2节强调这是必须显式对齐的两个模型源）。
5. **消息类型与发布频率的基本概念** — `sensor_msgs/JointState`、`/clock`（仿真时间 sim time vs wall time）、`rosgraph_msgs/Clock`；为什么要"固定仿真步长"、控制频率和消息频率的关系。

## 目录

- [1. ROS2 包体系与工程约定](#1-ros2-包体系与工程约定)
  - [1.1 share 目录与 PROJECT_NAME 约定](#11-share-目录与-project_name-约定)
  - [1.2 CMakeLists.txt 逐段解析（mujoco_bridge 样板）](#12-cmakeliststxt-逐段解析mujoco_bridge-样板)
- [2. TF / Frame 约定](#2-tf--frame-约定)
  - [2.1 为什么必须先约定 Frame，关键规则有哪些](#21-为什么必须先约定-frame关键规则有哪些)
- [3. 机器人描述模型的来源](#3-机器人描述模型的来源)
  - [3.1 apt 装 vs 自己 vendor，MoveIt 的 SRDF 从哪来](#31-apt-装-vs-自己-vendormoveit-的-srdf-从哪来)
  - [3.2 vendor 是什么意思](#32-vendor-是什么意思)
- [4. 开发环境：VSCode + distrobox](#4-开发环境vscode--distrobox)
  - [4.1 `#include <rclcpp/rclcpp.hpp>` 标红怎么办](#41-include-rclcppcclcpphpp-标红怎么办)
  - [4.2 已经 attach 容器了仍然标红](#42-已经-attach-容器了仍然标红)
- [5. 运动学（FK）](#5-运动学fk)
  - [5.1 FK 实践中可用的组件、需要自己写的部分、构型鲁棒写法](#51-fk-实践中可用的组件需要自己写的部分构型鲁棒写法)
- [6. Stage A：mujoco_bridge_node 最小实现（模型加载 + 物理步进定时器）](#6-stage-amujoco_bridge_node-最小实现模型加载--物理步进定时器)
  - [6.0 一句话总结](#60-一句话总结)
  - [6.1 Stage A 具体做了什么，涉及哪些概念](#61-stage-a-具体做了什么涉及哪些概念)
  - [6.2 调试时踩到的段错误根因，为什么改成 dlopen](#62-调试时踩到的段错误根因为什么改成-dlopen)
  - [6.3 `main` 函数介绍](#63-main-函数介绍)
  - [6.4 `onTimer` 为什么是回调函数，注册过程，其它回调注册方式](#64-ontimer-为什么是回调函数注册过程其它回调注册方式)
  - [6.5 单进程 embed vs 分进程 + IPC：为什么、实现上差在哪](#65-单进程-embed-vs-分进程--ipc为什么实现上差在哪)
- [7. 后续计划与 Session 交接](#7-后续计划与-session-交接)
  - [Stage B — /clock + /joint_states](#stage-b--clock--joint_states)
  - [Stage C — TF（static + dynamic）](#stage-c--tfstatic--dynamic)
  - [Stage D — reset service](#stage-d--reset-service)
  - [Stage E — 命令订阅 + sine 测试脚本 + demo launch/rviz](#stage-e--命令订阅--sine-测试脚本--demo-launchrviz)

---

## 1. ROS2 包体系与工程约定

### 1.1 share 目录与 PROJECT_NAME 约定

> Q: 现在这个 CMakeLists 是由你完成的，DESTINATION 到 share 里是通常的约定吗？`$PROJECT_NAME` 在哪定义的。

**`share/` 是不是通常约定** — 是的，这是 ROS2（继承自 ROS1）的标准约定，不是随意选的：

- ROS2 的包安装遵循 Linux FHS 风格布局：可执行文件在 `lib/<pkg>/`，共享的非二进制资源（launch 文件、config、urdf、mesh、rviz 配置等）统一放在 `share/<pkg>/`。
- 这个约定被 `ament_index_cpp`/`ament_index_python` 的 `get_package_share_directory()` 依赖——它就是去查 `install/share/<pkg_name>/` 这个固定路径。如果不放在 `share/` 下，那个查找函数就找不到文件。
- 所以任何 ROS2 包想让别的包能用统一 API 找到自己的资源文件，几乎都必须装到 `share/${PROJECT_NAME}/` 下，这不是这个包的特殊选择，是生态里的硬约定。

**`${PROJECT_NAME}` 在哪定义** — 是 CMake 自己的内建变量，来自 CMakeLists 里的：

```cmake
project(robot_description)
```

CMake 的 `project()` 命令一旦被调用，就会自动设置好几个变量，其中就包括 `PROJECT_NAME`（值为 `robot_description`）、`PROJECT_SOURCE_DIR`、`PROJECT_VERSION` 等。这不是 ament 特有的东西，是纯 CMake 语义。写 `${PROJECT_NAME}` 而不是硬编码 `robot_description` 字符串，纯粹是为了防止将来改包名时漏改一处——两处保持一致。

所以整体效果是：`colcon build` 之后，这些文件会被安装到

```
install/robot_description/share/robot_description/{urdf,meshes,mujoco}/...
```

之后其他包用 `get_package_share_directory("robot_description")` 拿到这个路径，再拼接子路径去读取具体文件。

### 1.2 CMakeLists.txt 逐段解析（mujoco_bridge 样板）

> Q: 将 mujoco_bridge 的 CMake 文件 copy 到 week1.md 里，然后作为一个样板，解释一下这个 makefile 的各处细节。

```cmake
cmake_minimum_required(VERSION 3.16)
project(mujoco_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mujoco_bridge_node src/mujoco_bridge_node.cpp)
target_include_directories(mujoco_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(mujoco_bridge_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  tf2_ros
  std_srvs
)

install(TARGETS mujoco_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

逐段解释：

| 行 | 内容 | 解释 |
|---|---|---|
| 1-2 | `cmake_minimum_required` / `project()` | 声明脚本使用的 CMake 语法/行为版本下限，避免用了新版本才有的命令却在旧 CMake 上跑出诡异错误。`project(mujoco_bridge)` 声明包名，同时自动设置 `PROJECT_NAME` 等内建变量（见 [1.1](#11-share-目录与-project_name-约定)）。 |
| 4-7 | C++ 标准 | 指定用 C++17 编译（计划书第3.1节定的语言标准）。外面包一层 `if(NOT ...)` 是防御性写法：如果上层（比如父 workspace 脚本）已经统一设置过 `CMAKE_CXX_STANDARD`，这里就不覆盖，避免多个包之间标准冲突。`CXX_STANDARD_REQUIRED ON` 表示编译器不支持 C++17 就直接报错，而不是静默降级。 |
| 9-16 | `find_package` | 每一行去系统里找一个已经安装好的 ROS2 库/包，并把它的头文件路径、库文件路径等信息导入当前 CMake 环境。`ament_cmake` 是 ROS2 用的 CMake 扩展工具（提供 `ament_package()`、`ament_target_dependencies()` 等命令），必须最先找到。`REQUIRED` 表示找不到就直接报错终止，而不是继续跑到链接阶段才失败。这里列的是 `mujoco_bridge_node.cpp` 会用到的消息类型和库：`rclcpp`（节点框架）、`sensor_msgs`（JointState/Image）、`geometry_msgs`（位姿/坐标变换消息）、`trajectory_msgs`/`control_msgs`（轨迹和控制命令）、`tf2_ros`（发布 TF）、`std_srvs`（reset service 用的标准服务类型）。 |
| 18 | `add_executable` | 声明要编译出一个可执行文件，取名 `mujoco_bridge_node`。这一行才是真正触发"编译"这件事的命令——`robot_description` 包里没有这一行，因为它不产出可执行文件，只安装资源文件。 |
| 19-21 | `target_include_directories` | 告诉编译器去哪里找这个 target 用 `#include` 引用的头文件——这里加的是本包自己的 `include/` 目录。`PRIVATE` 表示这个 include 路径只对 `mujoco_bridge_node` 自己编译时生效，不会传递给依赖这个包的其他包（要给别人用的头文件应该用 `PUBLIC` 并且也 install 出去）。`$<BUILD_INTERFACE:...>` 是"生成器表达式"，意思是"这条 include 路径只在构建阶段（本地 build 目录）生效"，对应的还有 `$<INSTALL_INTERFACE:...>`（安装后从 `install/include/` 找），这里没写后者，因为目前没有对外导出头文件的需求。 |
| 22-30 | `ament_target_dependencies` | `ament_cmake` 提供的便捷命令，等价于把上面 `find_package` 找到的每个库的 include 路径和链接库都绑定到 `mujoco_bridge_node` 这个 target 上。不用这个命令的话需要手写更繁琐的 `target_link_libraries` + `target_include_directories`；ROS2 生态里几乎所有 C++ 包都用这个命令代替。 |
| 32-34 | `install(TARGETS ...)` | `colcon build` 编译完成后，把生成的可执行文件从 `build/` 目录复制到 `install/mujoco_bridge/lib/mujoco_bridge/` 下。这也是 ROS2 的硬约定：可执行文件装在 `lib/<pkg_name>/` 下（不是常见的 `bin/`），这样 `ros2 run mujoco_bridge mujoco_bridge_node` 才能按包名+可执行文件名找到它。 |
| 36-38 | `install(DIRECTORY include/ ...)` | 把本包的头文件目录也装到 `install/mujoco_bridge/include/` 下，方便将来如果有其他包想 `#include <mujoco_bridge/xxx.hpp>` 复用这里的头文件。 |
| 40-43 | `BUILD_TESTING` | `BUILD_TESTING` 是 colcon/CTest 的标准开关变量，默认开启。这几行只在"测试模式"下生效，作用是自动跑 ROS2 生态标准的静态检查（代码风格、版权声明格式等 lint 检查），检查项来自 `package.xml` 里声明的 `test_depend`（`ament_lint_auto`、`ament_lint_common`）。这不是业务逻辑测试，是代码规范检查；后续真正的 gtest 单元测试（比如第3周的 FK/Jacobian 数值测试）会在这里加 `ament_add_gtest(...)`。 |
| 45 | `ament_package()` | 必须放在文件最后一行调用。`ament_cmake` 的收尾函数，负责生成包的元数据、导出 `find_package(mujoco_bridge)` 需要的信息、注册到 ament 索引，让这个包能被其他 ROS2 包和工具正确发现。忘记调用这一行，包基本上无法被其他包依赖或被 `ros2 pkg list` 正确识别。 |

---

## 2. TF / Frame 约定

### 2.1 为什么必须先约定 Frame，关键规则有哪些

> Q: 为什么需要先约定好 Frame，一般会要有哪些关键规则。

**为什么必须先钉死 frame 约定**

- 系统里每个模块（感知、抓取、规划、控制）都要在坐标系之间传递位姿。如果 `world`/`base_link`/`tool0`/`camera` 这些 frame 的定义在模块之间不一致（比如一个模块认为 `tool0` 是法兰面中心，另一个认为是指尖），位姿会带着一个隐藏的固定偏差传下去——数值上"看起来正常"，但抓取会系统性偏移，且很难从报错里定位到问题。
- Frame 错误和算法错误（比如 IK 解错、ICP 没收敛）在现象上常常长得很像（末端位置不对、抓取失败），但根因完全不同、排查方式完全不同。先把 frame 钉死，等于先排除一整类"看起来是算法 bug，其实是坐标系 bug"的情况，这也是计划书反复强调"第2周前必须完成"的原因。
- 一旦下游模块（感知、规划）开始基于某个 frame 定义写代码，事后再改 frame 命名/原点定义，等于要求所有已写的模块跟着改，成本远高于一开始就定好。

**关键规则**（对应 [architecture.md](../../docs/architecture.md) 里记录的内容）：

1. **每个 frame 只能有一个权威来源**：比如相机外参只能由 `mujoco_bridge` 发一份 static TF，任何感知节点都不能自己再定义/硬编码一套外参，否则两套数值一旦不一致，误差会看起来像感知算法的问题。
2. **frame tree 要有清晰的父子链**，不能出现环或者游离 frame：`world → base_link → panda_link0..7 → tool0`，`world → camera_link`。每个非根 frame 只能有一个父 frame（TF2 树的硬性要求）。
3. **区分 static 和 dynamic frame**：不随时间变化的（相机支架、桌面、base）用 `static_transform_broadcaster` 一次性发布；随时间变化的（关节链、物体位姿）持续以固定频率发布，且要和对应数据的时间戳对齐。
4. **oracle 和真实估计不能共用同一条 topic/frame 但换着塞不同数据**：ground-truth 物体位姿和视觉估计位姿用同一个 frame 命名（比如都叫 `object`），但通过不同 topic 区分，方便做 oracle vs vision 的对照实验，而不是在同一个 frame 里"悄悄"混用两种来源。
5. **命名和方向要与模型文件一致**：URDF 里的 link/joint 名字、MuJoCo 里的 body/joint 名字，如果命名不同或者轴方向定义不同，必须显式写一张映射表（对应计划书第3.2节要求的"记录关节名、关节顺序、零位、轴方向、限位"），不能靠"应该是对应关系"去猜。

---

## 3. 机器人描述模型的来源

### 3.1 apt 装 vs 自己 vendor，MoveIt 的 SRDF 从哪来

> Q1: 关于 Panda 模型：直接安装到系统会有什么区别，原来 robot_description 的 CMake 和 package 文件是否要改动；MoveIt 所需的 description 文件是怎么来的，它不能直接使用 URDF 吗，它可以直接从 URDF 生成吗？

**"直接装到系统" vs 自己 vendor 一份，区别在哪**

- apt 装的 `ros-humble-franka-description` 是一个完整、独立的 ROS2 包，装完之后躺在 `/opt/ros/humble/share/franka_description/` 下，和这个仓库（`robot_description`）是两个不同的包，谁也不包含谁。
- 好处：不用把 Franka 官方几十个 mesh 文件、多种末端执行器变体、多种机型（`fer`/`fr3`/`fp3`...）的 xacro 全部塞进自己仓库；官方更新（比如修 bug、加新型号）时只要 `apt upgrade` 就跟着更新，不需要手动同步 GitHub。
- 代价：这个仓库现在多了一条"外部依赖"——换一台新机器/新容器时，如果没装这个 apt 包，`robot_description` 里引用它的 launch/xacro 就会找不到文件。这也是为什么要把它写进 `package.xml` 的 `exec_depend`（见下面第二点），并记录进 [CLAUDE.md](../../CLAUDE.md)。

**`robot_description` 的 CMakeLists.txt 需要改吗**

不需要改。[CMakeLists.txt](../../robot_description/CMakeLists.txt) 只负责把**本仓库自己的** `urdf/`、`meshes/`、`mujoco/` 目录安装到 `share/robot_description/` 下，这几个目录现在装的是：
- `mujoco/`：vendor 进来的 MJCF（因为没有 apt 包，只能自己管）
- `urdf/`、`meshes/`：暂时是空的——因为 URDF 直接用 apt 装的 `franka_description` 提供，本仓库不需要再放一份

**`package.xml` 需要改，而且已经改了**——加了两行 `exec_depend`：

```xml
<exec_depend>franka_description</exec_depend>
<exec_depend>moveit_resources_panda_moveit_config</exec_depend>
```

`exec_depend` 的意思是"运行时依赖"（不是编译时链接的库，而是启动 launch 文件、找 xacro 文件时需要这个包已经装好）。这样声明之后，`rosdep`（ROS2 的依赖检查工具）能自动识别出这台机器缺这两个包，而不是等 `ros2 launch` 跑到一半才报"文件找不到"。

**MoveIt 用的 description 文件是从哪来的，能不能直接用 URDF，能不能从 URDF 生成**

三个问题分开答：

1. **MoveIt 需要哪些文件**：MoveIt 2 除了 URDF（几何、link/joint 结构）之外，还需要 **SRDF**（Semantic Robot Description Format）—— 这是 URDF 没有的语义信息：哪些 link 组成一个"规划组"（比如 `panda_arm` 这7个关节算一组，`panda_hand` 的两个指头算另一组）、哪些 link 对之间要禁用自碰撞检测（比如物理上相邻但几何上会误报碰撞的 link）、末端执行器是哪个 link、预定义姿态（比如 "home"、"ready"）等。除此之外还需要一堆 YAML 配置：`joint_limits.yaml`（规划时用的限位，可以比 URDF 里的物理限位更保守）、`kinematics.yaml`（IK 求解器选择和参数）、`ompl_planning.yaml`（OMPL 规划器参数）等。
2. **能不能直接用 URDF，不要 SRDF**：不能。MoveIt 的核心数据结构 `PlanningScene`/`RobotModel` 在初始化时就要求 URDF+SRDF 一起加载——没有 SRDF，MoveIt 不知道"手臂"和"夹爪"是两个独立可控的组，也不知道该对哪些 link 跳过自碰撞检测（严格按几何检测会把正常姿态下本来就贴近的连杆误判为碰撞）。
3. **SRDF 能不能从 URDF"自动生成"**：能，但不是全自动、不需要人看的那种自动。MoveIt 官方提供一个叫 **MoveIt Setup Assistant** 的 GUI 工具：你喂给它一份 URDF，它会加载模型、让你在界面上手动圈定规划组、手动勾选哪些 link 对不做自碰撞检测（它会先跑一遍碰撞采样给你建议，但最终需要人确认）、设置预定义姿态，然后一键导出一整套 SRDF + YAML 配置包。所以准确说法是"辅助生成"而不是"纯自动生成"——几何关系是自动读的，但规划组划分和自碰撞豁免这些语义信息需要人的判断介入。

我们现在的情况：`ros-humble-moveit-resources-panda-moveit-config` 这个 apt 包已经是官方帮你跑完 Setup Assistant 之后打包好的成品（`panda_arm.srdf.xacro`、`joint_limits.yaml`、`kinematics.yaml` 等全都在里面），所以这一步的人工工作已经被官方做掉了，我们只是复用现成结果。第6周真正接入 MoveIt 2 时，需要核对这份配置里的规划组定义、限位数值是否符合我们项目的实际需求（比如它是不是把夹爪也算进了同一个 group，会不会需要按项目实际情况调整）。

### 3.2 vendor 是什么意思

> Q2: vendor 是什么意思？

"vendor"（动词用法，"vendor 进仓库"）是软件工程里的常用说法，指**把一份外部依赖的源代码/资源文件直接复制一份，纳入自己项目的版本控制（比如 git）里，而不是在构建/运行时去动态拉取或依赖外部安装**。

对比一下三种常见的"引入外部东西"的方式：

1. **系统包依赖**（这次给 `franka_description`/`moveit_resources_panda_moveit_config` 采用的方式）：不复制文件，只记录"这台机器需要装有这个包"，实际文件由包管理器（apt）管理，装在系统路径下（`/opt/ros/humble/share/...`），不进 git。
2. **vendor 进仓库**（这次给 MuJoCo MJCF 采用的方式）：把文件实实在在复制一份到 `robot_description/mujoco/franka_emika_panda/`，跟着这个仓库一起提交进 git、一起被 clone、一起打包。
3. **构建时拉取**（这次没用，但常见于其他项目）：比如用包管理器的 lock 文件（`package.json`、`Cargo.toml`）或者 CMake 的 `FetchContent`，在 build 阶段自动去网络上下载指定版本的依赖，不长期存放在仓库里，也不需要预先装在系统上。

选 vendor（方式2）而不是方式1（系统包）的原因，通常是：
- 这个依赖没有现成的包管理器安装方式（MuJoCo Menagerie 没有 apt 包）
- 想要更强的可复现性——别人 clone 这个仓库下来就能直接用，不需要额外一步"先装什么东西"（缺点是仓库体积变大，这次多了约33MB）
- 需要对这份文件做本地修改，又不想依赖上游随时变动的版本

这次的选择逻辑是：**能装系统包就装系统包（避免重复维护一份文件），没有包管理方式的才 vendor 进仓库**。这也是为什么 [CLAUDE.md](../../CLAUDE.md) 里把两种情况分开记录——它们对"如何在新机器上复现环境"这件事的处理方式完全不同：系统包需要额外一步 `apt install`，vendor 的文件只要 `git clone` 这个仓库就自带了。

---

## 4. 开发环境：VSCode + distrobox

### 4.1 `#include <rclcpp/rclcpp.hpp>` 标红怎么办

> Q：我正在使用 vscode，目前可以看到 `mujoco_bridge_node.cpp` 里的 `#include <rclcpp/rclcpp.hpp>` 会在编辑器标红，表示检测不到。如何才能让编辑器知道要去哪里找？

**根源**：VSCode 现在是运行在宿主机上，而 `rclcpp` 的头文件（`/opt/ros/humble/include/rclcpp/...`）只存在于 `robotics-dev` 这个 distrobox 容器里，宿主机文件系统里根本没有这条路径。C/C++ 插件的 IntelliSense 在宿主机文件系统里当然找不到，所以标红是符合预期的——不是配置错了，而是编辑器"看错了机器"。

**方案 A（推荐）：让 VSCode 直接在容器里跑**

distrobox 底层是 podman/docker，VSCode 的 **Dev Containers** 扩展可以直接 attach 到一个正在运行的容器：

1. 安装扩展 `ms-vscode-remote.remote-containers`（Dev Containers）
2. 确保容器在运行：
   ```bash
   distrobox enter robotics-dev
   ```
   （进一次就会启动它，之后可以退出，容器仍在后台运行）
3. `Cmd/Ctrl+Shift+P` → `Dev Containers: Attach to Running Container...` → 选 `robotics-dev`
4. attach 成功后，VSCode 会开一个新窗口，这个窗口的文件系统视角就是容器内部——用 `File > Open Folder` 打开仓库在容器内的实际路径（比如 `~/robotics/robotic_manipulation_practice`）

这样之后，C/C++ 插件的 IntelliSense 天然就能看到 `/opt/ros/humble/include`，不需要手动配置 `includePath`。这是最彻底的方案，后面调试（断点、`gdb`）也依赖这个，迟早要配。

### 4.2 已经 attach 容器了仍然标红

> Q：我已经在 container 运行了 vscode，但是仍然标红？

attach 到容器本身没问题（容器内确认 `rclcpp` 头文件确实存在于 `/opt/ros/humble/include/rclcpp`），标红通常是下面几个原因之一：

1. **C/C++ 插件是否在"容器窗口"里单独装的**：Remote/Dev Containers 里扩展分本地和远程两类，attach 到容器后打开的新窗口需要**在容器里重新安装一遍** `C/C++`（`ms-vscode.cpptools`）扩展，宿主机装过不算数。检查方法：左下角远程指示器应显示 `Dev Container: robotics-dev`，打开扩展面板看 `C/C++` 是否显示 "Install in Dev Container"。
2. **装了插件但没告诉它去哪找头文件**：让 colcon 生成真实编译参数，而不是手写 `includePath`：
   ```bash
   colcon build --packages-select mujoco_bridge --symlink-install \
     --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
   ```
   会在 `build/mujoco_bridge/compile_commands.json` 生成每个源文件真实的 `-I`/`-D` 参数。然后在 `.vscode/settings.json` 加：
   ```json
   {
     "C_Cpp.default.compileCommands": "${workspaceFolder}/build/mujoco_bridge/compile_commands.json"
   }
   ```
   加完后 `Cmd/Ctrl+Shift+P` → `C/C++: Reset IntelliSense Database` 强制刷新。
3. **打开的 Folder 是不是仓库根目录**：确认标题栏路径是容器内的 `~/robotics/robotic_manipulation_practice`，不是宿主机残留路径，否则 `${workspaceFolder}` 会指错地方。

**结果：方案 2（`compile_commands.json` + `C_Cpp.default.compileCommands`）解决了问题。** 说明插件本身已经在容器里正确安装，只是缺少真实的编译参数信息——colcon 生成的 `compile_commands.json` 把每个源文件实际用到的 `-I` 路径（`/opt/ros/humble/include/...`）喂给了 IntelliSense。这也印证了一个通用经验：ROS2/colcon 这类多包工作区，`compile_commands.json` 比手写 `c_cpp_properties.json` 里的 `includePath` 更可靠——手写容易漏掉某个包的依赖路径，而编译参数是构建系统自己算出来的，不会漏。

---

## 5. 运动学（FK）

### 5.1 FK 实践中可用的组件、需要自己写的部分、构型鲁棒写法

> Q：FK（正向运动学）的坐标变换是获得末端世界坐标的方法，实践中可以用的坐标变换组件通常是什么，哪些是需要自己编码的（没有现成的包可用的），会如何根据机器人构型的变化而变化，是否存在对构型鲁棒的写法？

**FK 本质**

FK 就是从关节角 q 到末端 world 系位姿的一条刚体变换链：

```
T_world_end = T_world_base · T_base_1(q1) · T_1_2(q2) · ... · T_n-1_end
```

每一节由两部分组成：URDF/MJCF 里固定的 joint origin（父子 link 间静态偏移），加上该 joint 类型（revolute/prismatic/fixed）和当前关节值决定的可变部分。

**实践中可直接用的现成组件**

在这个项目（Panda + ROS2 Humble + MoveIt + MuJoCo）里，FK 几乎不需要手写：

- **moveit_core::RobotState**（依赖 `panda_moveit_config` 的 SRDF/kinematics 配置，底层是 KDL 或 Pinocchio 之类的运动学树求解器）：给 joint 值调 `updateLinkTransforms()`，直接拿到任意 link 的 world 位姿。这是 ROS 侧标准做法。
- **robot_state_publisher + tf2**：订阅 `joint_states`，用 URDF 建运动学树自动发布 tf tree，可以查询任意两 link 间变换。注意 tf2 本身只是发布/查询机制，真正算 FK 的是 KDL（robot_state_publisher 内部用的）。
- **MuJoCo 自身**：`mj_forward()` 走的是 MJCF 定义的 body 树，之后直接读 `d->xpos[body_id]` / `d->xquat[body_id]` 就是该 body 的 world 位姿——这是 mujoco_bridge 里最直接能用的现成 FK，完全不用自己实现。
- 通用替代：Pinocchio、Orocos KDL，都能从 URDF 自动构建运动学树。

**需要自己写的部分**

- **静态 offset frame**：比如末端 flange 到夹爪指尖接触点之间的偏移，URDF/MJCF 通常不会精确到你需要的具体特征点，需要自己定义一个固定 4x4 变换叠加在 FK 结果之后。
- **传感器外参**（相机/力传感器到 base_link）：没有"内置于运动学库"的东西，需要自己标定、手写 static transform。
- **ROS ↔ MuJoCo 的关节映射**：ROS 侧 joint 顺序和 MuJoCo `qpos` 数组顺序不一定一致，这个 name→index 的映射需要自己在 bridge 代码里维护（这是 `mujoco_bridge_node.cpp` 里实际存在的耦合点）。
- 如果真要脱离所有库手写 FK（比如教学目的），才需要自己写 DH 参数表或者手动做变换矩阵连乘。

**随构型变化的影响与鲁棒写法**

不鲁棒的写法：把某个具体机器人的关节数、DH 参数、变换链长度硬编码在代码里（比如写死"7 次矩阵乘法对应 Panda 7 关节"）。一旦加装夹爪、改变 link 长度、换机型，就必须重新推导并改代码。

鲁棒的写法：模型驱动（model-driven），FK 函数不关心具体链长和参数，而是：
- 输入是"运动学树描述"（parent-child + joint type + axis + origin，来自 URDF/MJCF 解析）+ 当前 joint 值的 name→value map；
- 输出通过遍历树结构动态组合变换链。

这正是 KDL/Pinocchio/moveit_core/MuJoCo 全都采用的方式——它们都是从描述文件（URDF/MJCF）自动生成运动学树，代码本身不含具体机器人的结构信息。所以对这个项目而言：**改 URDF 或 MJCF（比如换夹爪、加 link）不需要改任何 FK 调用代码**，前提是自己写的粘合代码（比如上面提到的关节映射）也用 name 查找而不是写死 index/顺序——写死顺序会打破这种鲁棒性，应该用 joint name 做 key。

---

## 6. Stage A：mujoco_bridge_node 最小实现（模型加载 + 物理步进定时器）

### 6.0 一句话总结

初步搭起了 ROS 和 MuJoCo 仿真器的桥接雏形：用 C++ 通过 `dlopen`/`dlsym` 调用 MuJoCo 的 API，实际读取了机器人的 MJCF 描述文件，并用 `mj_step`（而不是 `mj_forward`——两者的区别见 [6.1](#61-stage-a-具体做了什么涉及哪些概念)）做定长步进积分来推进仿真。具体做法：设置一个周期等于 `timestep_s` 的软件定时器，每次触发都调一次 `mj_step` 并累计步数，每隔一定步数读一次 `mjData::time`（MuJoCo 自己按"步数 × timestep"累计的仿真时长）打日志。

验证结果：MuJoCo 里读到的 `data_->time` 和"软件步数 × 定时器周期"数值上完全吻合。但这个吻合**不是墙钟时间决定了仿真时间**，而是因为定时器周期被故意设成了和 MJCF 的 `timestep` 相等（都是 0.002s）——`data_->time` 全程只由"步数 × timestep"决定，跟定时器周期本身无关，只是这次两个数字凑成了同一个值，才看起来像是"对上了"。

### 6.1 Stage A 具体做了什么，涉及哪些概念

> Q: Stage A 具体做了什么，涉及哪些概念？

**代码结构**（`src/mujoco_bridge/src/mujoco_bridge_node.cpp`）：一个 `MujocoBridgeNode : public rclcpp::Node`，构造函数里加载 MJCF、创建 `mjData`、启动一个定时器；定时器回调里每次调一次 `mj_step`；析构函数释放 `mjData`/`mjModel`。`main()` 就是标准的 `rclcpp::init` → `spin` → `shutdown` 三段式（详见 [6.3](#63-main-函数介绍)）。

**`mjModel` / `mjData` 的关系**：`mjModel` 是"编译后的模型描述"——从 MJCF 解析出来的静态结构（有多少个 body/joint/actuator、它们的连接关系、惯量、几何形状……），加载一次之后不会变。`mjData` 是"当前仿真状态"——`qpos`（关节位置）、`qvel`（关节速度）、`xpos`/`xquat`（每个 body 的 world 位姿）、`ctrl`（下发给 actuator 的控制量）等，每步物理都会变。这个分离的好处：同一个 `mjModel` 可以配多份 `mjData`（比如做并行仿真/MPC rollout），模型本身不用重复解析。

**`mj_step` 每次前进多少**：固定前进 `mjModel->opt.timestep` 这么多仿真时间，这个值来自 MJCF 里 `<option timestep="0.002".../>`（Panda 这份模型里是 0.002s）。**不是**每次调用时按真实经过的墙钟时间前进——如果那样做，物理积分步长会随系统调度抖动变化，数值积分（尤其接触/摩擦）在变步长下会不稳定甚至发散。所以正确做法是：仿真时间 = 步数 × 固定 timestep，跟墙钟解耦。代码里 `data_->time` 就是 MuJoCo 自己维护的这个累加值，日志里 `sim_time` 直接读它，不是自己算的。

**定时器多久触发一次**：`create_wall_timer` 的周期设成跟 `timestep` 一样（0.002s → 500Hz），这样"墙钟触发频率"和"仿真步长"数值上凑成 1:1，日志里能看到 1 秒真实时间对应恰好 1.000s 仿真时间。但这只是尽量让两者同步、方便观察，本质上二者是两个独立的量——如果某次定时器回调被系统调度延迟触发（真实间隔变成比如 3ms），仿真时间依然只前进一个 timestep（0.002s），不会跟着墙钟的延迟一起变化，下一次回调也不会"追赶"。这就是上一段说的解耦：真正决定仿真时间的永远是"步数 × timestep"，墙钟只是决定"多久调用一次 `mj_step`"这个节奏，不参与仿真时间的计算。

**为什么用 `create_wall_timer` 而不是 `spin` 里裸写循环**：`rclcpp::spin(node)` 本身是一个事件循环，负责处理所有回调（定时器、订阅者、服务）——不能在 `main` 里再写一个 `while` 循环手动调 `mj_step`，那样会跟 `spin` 抢执行权、也没法同时处理 ROS 的其他回调（后面 Stage D 的 reset service 就是靠 `spin` 调度进来的）。`create_wall_timer` 把"周期性执行"这件事注册给 `spin` 的事件循环去调度，是 ROS2 里做周期性任务的标准方式（回调注册机制详见 [6.4](#64-ontimer-为什么是回调函数注册过程其它回调注册方式)）。

**验证结果**：`ros2 run mujoco_bridge mujoco_bridge_node` 跑 3 秒多，日志显示：
```
Loaded .../panda.xml (nq=9, timestep=0.0020s)
step=500  sim_time=1.000s
step=1000 sim_time=2.000s
step=1500 sim_time=3.000s
```
`nq=9`：Panda 7 个臂关节 + 2 个夹爪指关节，和 architecture.md 里记录的关节数一致。500 步 = 1 秒仿真时间，和 0.002s×500=1.0s 对得上。

### 6.2 调试时踩到的段错误根因，为什么改成 dlopen

> Q: 调试 Stage A 时踩到一个很深的段错误，根因是什么？为什么最终改成 `dlopen` 而不是直接 `target_link_libraries(mujoco::mujoco)`？

**现象**：一开始按计划直接链接 `mujoco::mujoco`（正常的 CMake `target_link_libraries`），只要可执行文件里同时"链接了 MuJoCo"和"构造了一个 `rclcpp::Node`"，进程就必定在构造 `rclcpp::Node()` 那一行段错误——即使完全没调用任何 `mj_*` 函数,光链接就会崩，且必须两个条件同时满足才崩（只链接不崩、只构造 Node 不链接也不崩）。

**根因**：MuJoCo 的共享库自己内置打包了一份 tinyxml2（XML 解析库），并且这份 tinyxml2 的符号是"默认可见"（没有做符号隐藏）。ROS2 的 RMW（DDS 中间层）在 `rclcpp::init`/构造第一个 Node 时会去 `dlopen` 加载 `libfastrtps.so`，而 fastrtps **自己也内置打包了一份 tinyxml2**，同样是默认可见符号。两份 tinyxml2 的函数名（mangled symbol）完全一样，但**内部实现/内存布局并不保证兼容**（版本、编译选项可能不同）。

Linux 动态链接器维护一个全局符号表：MuJoCo 是可执行文件的直接链接依赖，进程启动时就把它的符号（包括这份 tinyxml2）注册进了全局符号表；fastrtps 是后来才被 `dlopen` 进来的。当 fastrtps 内部代码调用它自己的 tinyxml2 函数时，动态链接器按"全局符号表里第一个匹配的名字"解析，结果解析到了 MuJoCo 那份 tinyxml2 的实现——但 fastrtps 传进去的对象内存布局是按它自己那份 tinyxml2 的 ABI 排的。于是变成"用 A 库的函数处理 B 库排布的对象"，读到错位的虚函数表指针，最终变成对空指针的写操作（`dmesg` 里的 `error 14` = write fault + user mode + page-not-present，正好对应这个模式）——这叫 **DSO 符号冲突/symbol interposition**，是两个库各自静态打包同名第三方库、又都不做符号隐藏时的经典坑。

> 补充定性（见 [6.5](#65-单进程-embed-vs-分进程--ipc为什么实现上差在哪)）：这属于纯 Linux 动态链接/ELF 层面的通用系统编程知识，跟机器人本身无关，任何语言/工具链选型不当的 C++ 项目都可能碰到；作为系统编程经验值得记录，但不该占用"机器人相关深度"的追问预算。

**修复**：不再让 MuJoCo 参与进程启动时的正常链接（不再 `target_link_libraries(mujoco::mujoco)`），而是用 `dlopen("libmujoco.so...", RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND)` 在运行时手动加载：
- `RTLD_LOCAL`：这个库里的符号（包括它那份 tinyxml2）**不注册进全局符号表**，别的库（fastrtps）不可能解析到它头上；
- `RTLD_DEEPBIND`（glibc 扩展）：这个库自己内部的函数调用，优先绑定到"自己库内部的定义"，而不是去外部全局符号表找——双重保险，即使全局表里恰好有同名符号也不受影响。

代价是不能再用编译期链接的 `mj_loadXML(...)` 直接调用，得用 `dlsym` 把每个用到的函数解析成函数指针。为此写了 `mujoco_bridge/mujoco_dl.hpp` + `mujoco_dl.cpp`：定义一个 `MujocoApi` 结构体（每个字段是一个函数指针），`loadMujocoApi()` 内部做 `dlopen` + 一串 `dlsym`，返回一个全局唯一的 `MujocoApi&`。业务代码（`mujoco_bridge_node.cpp`）里全部通过 `api_.loadXML(...)`、`api_.step(...)` 这种形式调用，看起来跟直接调 `mj_*`差别不大，只是多了一层间接。（这几个语法/实现细节的进一步拆解见 [cpp_concepts.md](cpp_concepts.md)。）

**对已批准计划的影响**：计划里 Change 2 原本写的是"直接 `target_link_libraries(mujoco::mujoco)`"，现在改成"只用 `find_package(mujoco)` 拿头文件路径（`get_target_property(... INTERFACE_INCLUDE_DIRECTORIES)`），不链接库本体，库本体运行时 `dlopen`"。后续所有 Stage（B/C/D/E）用到的 `mj_*` 函数，都要先加进 `MujocoApi` 结构体、在 `loadMujocoApi()` 里 `dlsym` 解析，再通过 `api_.xxx` 调用，不能再假设可以直接调用裸的 `mj_*` 名字。

### 6.3 `main` 函数介绍

> Q: `mujoco_bridge_node.cpp` 里的 `main` 函数介绍一下？

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mujoco_bridge::MujocoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
```

这是 ROS2 C++ 节点标准的三段式结构，只有三行：

- **`rclcpp::init(argc, argv)`**：初始化整个进程级的 rclcpp 上下文，解析 ROS2 专用命令行参数（`--ros-args -r ...` 这类重映射），并启动底层 DDS（rmw/fastrtps）。这一步或紧接着的 Node 构造，正是 [6.2](#62-调试时踩到的段错误根因为什么改成-dlopen) 那次段错误发生的地方——`libfastrtps.so` 在这里被 `dlopen` 进来。必须在创建任何 `rclcpp::Node` 之前调用，且整个进程只调用一次。
- **`rclcpp::spin(...)`**：先构造 `MujocoBridgeNode`（触发构造函数里的 MJCF 加载、`mj_makeData`、注册 500Hz 定时器），再把节点交给 ROS2 执行器（executor），进入一个**阻塞的事件循环**——不断检查有没有到期的定时器/新消息/服务请求，有就调用对应回调（目前只有 Stage A 注册的 `onTimer`）。会一直阻塞直到收到关闭信号（如 `Ctrl+C`/SIGINT），这也是为什么验证时要用 `timeout 4 ros2 run ...` 才能让它自动退出。
- **`rclcpp::shutdown()`**：`spin` 返回后做全局清理，释放 rclcpp 的进程级资源，跟 `init` 一一对应。

节点自己的资源清理（`mjData`/`mjModel` 释放）不在 `main` 里，而是在 `MujocoBridgeNode` 的析构函数里——`spin` 返回后，`make_shared` 创建的 `shared_ptr` 在 `main` 结束时自动析构触发清理，标准 RAII，不用手写释放代码。

### 6.4 `onTimer` 为什么是回调函数，注册过程，其它回调注册方式

> Q: 为什么 `onTimer` 是回调函数？这个注册过程是怎么发生的？还有其它常见的回调注册方法吗？

**回调 vs 直接调用**：`onTimer` 是"回调函数"，核心在于我们的代码从来没有直接调用它，而是把它"交给"别人，由别人在将来某个时刻决定何时调用。

**注册过程**：

```cpp
timer_ = create_wall_timer(
  std::chrono::duration_cast<std::chrono::nanoseconds>(period),
  std::bind(&MujocoBridgeNode::onTimer, this));
```

1. `std::bind(&MujocoBridgeNode::onTimer, this)`：`onTimer` 是非静态成员函数，光有函数地址不够，调用时还需要知道"是哪个对象的 `onTimer`"（也就是 `this`）。`std::bind` 把函数指针和 `this` 打包成一个无参数可调用对象（等价于一个 `std::function<void()>`）。也可以写成 lambda：`[this]() { onTimer(); }`，效果一样，是更常见的现代写法。
2. `create_wall_timer(period, callback)` 是 `rclcpp::Node` 的成员函数，内部会创建一个 `rclcpp::WallTimer`（包装底层 `rcl_timer_t`，依赖操作系统时钟维护"下次到期时间"），注册进节点内部的 timer 列表（归入某个回调组，未指定则用默认组），返回 `TimerBase::SharedPtr` 赋给 `timer_`。这一步只是让节点持有定时器的生命周期，跟"回调什么时候被调用"无关。
3. 真正触发调用的是 `rclcpp::spin(node)`：它创建一个执行器（默认 `SingleThreadedExecutor`），进入循环，向操作系统申请"等待这组事件里任意一个就绪"（底层基于 `rcl`/`rmw` 的 wait set，类似 `epoll`/`timerfd`），一旦某个定时器到期（或有消息/请求到达）就同步调用对应回调。

所以整条链路是：我们只是把"要做什么"（回调）和"什么时候做"的条件（周期）注册进节点，真正"什么时候调用"的决策权交给了 `spin()` 里的执行器。这也是为什么 `main` 里必须调用 `rclcpp::spin`——不调用的话，所有注册的回调永远不会被触发。

**其它常见的回调注册方式**（同样的模式：注册一个 callback + 一个触发条件，交给执行器调度）：

| 方式 | 触发条件 |
|---|---|
| `create_subscription<MsgType>(topic, qos, callback)` | 收到话题消息时，回调参数是收到的消息本身 |
| `create_service<SrvType>(name, callback)` | 收到服务请求时，回调里填写响应（Stage D 的 reset service 就是这种） |
| `create_client<SrvType>` + `async_send_request(request, callback)` | 客户端异步调用，拿到响应时触发 |
| `rclcpp_action::create_server(...)` | 动作服务器，需要注册多个回调（收到目标请求、取消请求、目标被接受后执行） |
| `add_on_set_parameters_callback(callback)` | 节点参数被外部修改时 |

共同点：都是"告诉 ROS2 触发条件是什么、触发时该调用哪个函数"，具体调用发生在 `spin()` 的事件循环里，而不是自己代码里某处直接调用。

### 6.5 单进程 embed vs 分进程 + IPC：为什么、实现上差在哪

> Q: 为什么很多真实项目把仿真器/机器人控制器和 ROS 桥接节点分成两个进程，而不是像 Stage A 这样单进程 embed？实现上具体差在哪？

**先搞清楚"进程"**：跑 `ros2 run mujoco_bridge mujoco_bridge_node` 时，操作系统创建**一个进程**——一块独立内存空间 + 正在执行的代码。这一个进程里同时装着自己写的 `MujocoBridgeNode`、MuJoCo 库（`mjModel`/`mjData`/`mj_step`）、ROS2 库（`rclcpp`），全部挤在**同一块内存**里，互相读对方的变量就跟读普通变量一样——`onTimer()` 里 `data_->qpos[i]` 就是直接去内存里那个地址取数字，没有任何中间步骤。这是 Stage A 现在的做法，称为**单进程 embed**。

**换成分进程**：假设把 MuJoCo 单独编译成另一个独立可执行文件，开两个终端各跑一个：

```
终端1: ./mujoco_sim_server     ← 只管算物理，不知道 ROS 是什么
终端2: ros2 run mujoco_bridge bridge_node   ← 只管跟 ROS 打交道
```

这两个进程**各自有独立内存**，进程1 算出来的 `qpos`，进程2 天生看不到——操作系统专门把进程之间隔离开，防止一个进程乱改另一个进程的内存（安全机制，不是缺陷）。所以必须专门想办法把进程1 的数字"传"给进程2：比如进程1 每算完一步，把 `qpos` 打包成一段二进制/文字，通过网络（socket）发给进程2，进程2 收到后解析出来用。这个"打包发送→接收解析"的步骤，单进程模式完全不需要，分进程模式必须自己写。

**为什么宁愿多写这一步也要分进程**：

1. **进程1 根本不是你自己的代码，没法塞进自己进程里**——最常见、最硬的原因。这个项目未来真要对接的 Franka 机械臂就是这样：`libfranka` 的实时控制循环跑在机械臂自带的控制电脑上，以 1kHz 通过网络协议（FCI）对外提供接口；`franka_ros2` 是另一个进程，通过这个网络协议跟它对话，再包装成 ROS2 的 topic/action。ROS 这边完全没法把 `libfranka` 的控制循环"embed"进自己进程——它根本不是给你链接用的东西，且很可能跑在另一台机器上。Gazebo classic 的 `gzserver` 也是独立进程，ROS 侧通过 Gazebo 自己的 transport/插件去接。
2. **故障隔离**：底层控制/仿真崩溃时，不希望把上层规划、感知这些代码一起拖死；反过来上层的 bug 也不该有机会搞崩硬实时控制循环。真实机器人系统里，"硬实时底层控制"和"ROS 应用层"几乎总是刻意分进程甚至分机器。
3. **生命周期/更新率不同**：底层常年固定高频跑（1kHz 甚至更高），上层 ROS 节点可能重启、调试、挂断点，分进程后两者互不拖累。
4. **多个消费者共享同一份仿真/机器人状态**：如果好几个 ROS 节点都要读同一份状态，让每个都 embed 一份仿真引擎既浪费又难保证一致，不如仿真只跑一份，其他人订阅它发出来的数据。
5. **语言/运行时不匹配**：仿真器是 Python 重度（很多 RL 环境）或者 MATLAB/Simulink，没法简单链接进 C++ 进程。

**Stage A 选单进程是合理的，不是错误选择**——前提是我们自己拥有仿真器源码（MuJoCo 是个普通 C 库，可以直接 dlopen/链接进自己进程），且需要每个 timestep 高频读 `qpos`/`qvel`/`xpos`，单进程的零拷贝直接访问在这个前提下更合适，分进程反而是不必要的开销。[6.2](#62-调试时踩到的段错误根因为什么改成-dlopen) 那次段错误（tinyxml2 符号冲突）正是"单进程 embed"模式特有的隐藏代价——分进程的方案从设计上就不可能出现"两个库在同一地址空间打架"的问题，因为它们压根不共享地址空间。这是为了拿到"零拷贝直接读 mjData"的好处，顺带买单的风险。

一句话：**是否分进程，本质是在"零延迟直接访问"和"故障隔离+进程独立生命周期+能对接不受自己控制的黑盒程序"之间做选择**。Stage A 现在这样做没问题，但要清楚这是一次有意识的取舍——面试被问"如果仿真器换成一个你没法链接的商业仿真器/真实机器人控制器，架构要怎么改"时，这条就是标准答案的骨架。

---

## 7. 后续计划与 Session 交接

Stage A 已完成、build+run 验证通过、讲解已记录（见第6节）。以下是 Stage B-E 的简要计划，供下一个 session 接着做（原始完整计划见 `.claude/plans` 下已批准的计划文件，这里只摘录关键点方便快速回忆上下文）。

**执行方式约定**（沿用 Stage A 的模式，不要跳过）：每个 stage 写完代码后先 `colcon build` + 实际跑一遍验证，验证通过再讲解涉及的概念，讲解完追问/确认理解后再写进本文件，最后才进入下一个 stage。每个 stage 建议除了"这段代码怎么工作"之外，再补至少一条**权衡/替代方案对比**和一条**失败模式/怎么验证**类问题（这是第 4 轮反思后定下的规矩，避免只深挖单段代码语法）。

**硬约束（Stage A 踩坑后定下，后续都要遵守）**：不能再 `target_link_libraries(mujoco::mujoco)` 直接链接 MuJoCo（会和 ROS2 的 fastrtps 因为 tinyxml2 符号冲突段错误，见 [6.2](#62-调试时踩到的段错误根因为什么改成-dlopen)）。任何新用到的 `mj_*` 函数，都要先加进 `mujoco_bridge/include/mujoco_bridge/mujoco_dl.hpp` 里的 `MujocoApi` 结构体字段，再到 `mujoco_dl.cpp` 的 `loadMujocoApi()` 里加一行 `resolve(handle, "mj_xxx", api.xxx)`，业务代码统一通过 `api_.xxx(...)` 调用。

### Stage B — `/clock` + `/joint_states`

- 新增依赖：`rosgraph_msgs`（`/clock` 消息类型），`package.xml` 和 `CMakeLists.txt` 都要加。
- `/clock`：每个 `onTimer` 里额外发布一次 `rosgraph_msgs::msg::Clock`，用 `data_->time` 换算成 ROS 的时间戳格式。这个节点是全局唯一的 `/clock` 发布者（sim time 权威来源），不会有别的节点也发 `/clock` 造成冲突。
- `/joint_states`：按 MJCF 声明的关节顺序（`joint1..7`、`finger_joint1/2`）遍历，从 `mjData::qpos`/`qvel` 里按 index 取值，组装成 `sensor_msgs::msg::JointState` 发布。**要点**：joint name→qpos index 的映射必须用 `mj_name2id` 按名字查（`MujocoApi` 里已经有这个函数指针了），不能硬编码 index 顺序——这是 [5.1](#51-fk-实践中可用的组件需要自己写的部分构型鲁棒写法) 里强调过的"构型鲁棒写法"原则的具体应用点。
- 需要讲解的概念：sim time vs wall time、`use_sim_time` 参数、`sensor_msgs/JointState` 的字段布局（`name`/`position`/`velocity`/`effort`）、为什么这里没有单独维护发布频率（发布和物理步进目前是同一个定时器触发，是简化，不是最优设计——值得追问"如果要解耦发布频率和物理步进频率，应该怎么改"）。

### Stage C — TF（static + dynamic）

- 新增依赖：无新增 ROS 包（`tf2_ros` 已在 CMakeLists 里），需要 `#include <tf2_ros/transform_broadcaster.h>` 和 `<tf2_ros/static_transform_broadcaster.h>`。
- 动态 TF：遍历 MJCF 里所有带关节的 body，用 `xpos`/`xquat` + `body_parentid` 算出父子相对变换（不能直接发 `xpos`/`xquat`，那是相对 world 的绝对位姿，TF 需要的是相对父 frame 的变换），通过 `tf2_ros::TransformBroadcaster` 发布。
- 静态 TF（启动时发一次）：`world -> base_link`，`link8`、`hand` 这类 fixed joint 的 body，以及**合成的** `hand -> hand_tcp`（偏移量 `xyz=(0,0,0.1034)`，来自 `docs/architecture.md` 已经记录的说明，MJCF 本身没有这个 frame）。
- 需要讲解的概念：`TransformBroadcaster` vs `StaticTransformBroadcaster` 的区别和为什么要分开、frame tree 必须是树（单一父节点）、`body_parentid` 怎么用、指尖命名差异（`left_finger`/`right_finger` vs URDF 的 `leftfinger`/`rightfinger`，见 architecture.md）。

### Stage D — reset service

- 新增依赖：无（`std_srvs` 已在）。
- `std_srvs::srv::Trigger` 类型的 `~/reset` service，回调里调 `mj_resetDataKeyframe(model_, data_, keyframe_id)`。`keyframe_id` 要在构造函数里用 `mj_name2id(model_, mjOBJ_KEY, "home")` 按名字查一次并缓存，不要硬编码成 `0`。
- 需要讲解的概念：service（请求/响应）vs topic（发布/订阅）的适用场景区别、MJCF 里 keyframe 是什么（预定义的一组 `qpos` 值）、reset 之后要不要额外调 `mj_forward`（把 `xpos`/`xquat` 等派生量刷新，不然发出去的第一帧 TF/JointState 可能还是旧值——这是个值得验证的失败模式）。

### Stage E — 命令订阅 + sine 测试脚本 + demo launch/rviz

- 新增依赖：`control_msgs`/`trajectory_msgs` 已在 CMakeLists 里。
- 订阅 `~/joint_command`（`trajectory_msgs::msg::JointTrajectory`），收到后把第一个 trajectory point 的 `positions` 按 joint name 写进 `mjData::ctrl`（MJCF 里的 position-servo actuator 会自己把 `ctrl` 值当目标位置去伺服，不是直接改 `qpos`）。
- 新建 `src/mujoco_bridge/launch/demo.launch.py`（起 `mujoco_bridge_node` + `rviz2`）和 `src/mujoco_bridge/rviz/demo.rviz`（先只放 TF + Grid，不接 RobotModel），`CMakeLists.txt` 加 `install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})`。
- 新建 `scripts/sine_joint_test.py`（rclpy 脚本），发一个缓慢正弦振荡的 `JointTrajectory` 给 `~/joint_command`，人工在 RViz 里确认对应关节的 TF 帧在动。
- 需要讲解的概念：position-servo actuator 的本质是什么（是一种简化的 PD 控制器，MJCF 里怎么定义增益）、`ctrl` 写入和真实机器人控制器接口的对应关系、launch 文件基础语法。

### 验收标准（沿用已批准计划里的定义）

1. `ros2 launch mujoco_bridge demo.launch.py` 后确认 `/clock`、`/joint_states`、`/tf`、`/tf_static` 都在发布（`ros2 topic list` / `ros2 topic hz /joint_states`），RViz 里 TF 树非爆炸、根节点是 `world`。
2. `ros2 service call /mujoco_bridge/reset std_srvs/srv/Trigger {}` 后 `/joint_states` 数值应该跳回 home pose。
3. 跑 `scripts/sine_joint_test.py`，RViz 里对应关节的 frame 应该能看到周期性摆动。
