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
- [2. ROS2 节点基础设施：Node、Publisher、QoS、消息类型](#2-ros2-节点基础设施nodepublisherqos消息类型)
  - [2.1 `rclcpp::Node` 基类：常用属性与方法](#21-rclcppnode-基类常用属性与方法)
  - [2.2 Publisher 与 QoS](#22-publisher-与-qos)
  - [2.3 内置消息类型 vs 自定义消息](#23-内置消息类型-vs-自定义消息)
  - [2.4 为什么 `timer_` / `clock_pub_` / `joint_state_pub_` 都是 `SharedPtr`](#24-为什么-timer_--clock_pub_--joint_state_pub_-都是-sharedptr)
  - [2.5 `declare_parameter`：ROS 参数到底是什么，为什么不用 `const double`](#25-declare_parameterros-参数到底是什么为什么不用-const-double)
- [3. TF / Frame 约定](#3-tf--frame-约定)
  - [3.1 为什么必须先约定 Frame，关键规则有哪些](#31-为什么必须先约定-frame关键规则有哪些)
- [4. 机器人描述模型的来源](#4-机器人描述模型的来源)
  - [4.1 apt 装 vs 自己 vendor，MoveIt 的 SRDF 从哪来](#41-apt-装-vs-自己-vendormoveit-的-srdf-从哪来)
  - [4.2 vendor 是什么意思](#42-vendor-是什么意思)
  - [4.3 怎么检查和分析 MJCF / URDF，以及怎么据此判断代码正误](#43-怎么检查和分析-mjcf--urdf以及怎么据此判断代码正误)
    - [4.3.1 两边都要先"展平"，否则等于没读](#431-两边都要先展平否则等于没读)
    - [4.3.2 该按什么顺序看（MJCF）](#432-该按什么顺序看mjcf)
    - [4.3.3 URDF ↔ MJCF 交叉验证：这是判断代码正误的主要手段](#433-urdf--mjcf-交叉验证这是判断代码正误的主要手段)
    - [4.3.4 "每个关节的状态变量应该是什么样"，以及一个 Stage E 地雷](#434-每个关节的状态变量应该是什么样以及一个-stage-e-地雷)
  - [4.4 `hand_tcp` 是什么，以及 `0.1034` 这个 magic number](#44-hand_tcp-是什么以及-01034-这个-magic-number)
    - [4.4.1 它是一个纯粹的命名坐标系，不是零件](#441-它是一个纯粹的命名坐标系不是零件)
    - [4.4.2 为什么这个数没有自动共享](#442-为什么这个数没有自动共享)
- [5. 开发环境：VSCode + distrobox](#5-开发环境vscode--distrobox)
  - [5.1 `#include <rclcpp/rclcpp.hpp>` 标红怎么办](#51-include-rclcpprclcpphpp-标红怎么办)
  - [5.2 已经 attach 容器了仍然标红](#52-已经-attach-容器了仍然标红)
- [6. 运动学（FK）](#6-运动学fk)
  - [6.1 FK 实践中可用的组件、需要自己写的部分、构型鲁棒写法](#61-fk-实践中可用的组件需要自己写的部分构型鲁棒写法)
  - [6.2 广义坐标：`qpos` / `qvel` 的表示方法，以及为什么维度不相等](#62-广义坐标qpos--qvel-的表示方法以及为什么维度不相等)
  - [6.3 四元数基础：从半角到位姿复合](#63-四元数基础从半角到位姿复合)
    - [6.3.1 一个单位四元数就是一个旋转](#631-一个单位四元数就是一个旋转)
    - [6.3.2 三个运算](#632-三个运算)
    - [6.3.3 位姿（不只是旋转）怎么复合](#633-位姿不只是旋转怎么复合)
- [7. Stage A：mujoco_bridge_node 最小实现（模型加载 + 物理步进定时器）](#7-stage-amujoco_bridge_node-最小实现模型加载--物理步进定时器)
  - [7.0 一句话总结](#70-一句话总结)
  - [7.1 Stage A 具体做了什么，涉及哪些概念](#71-stage-a-具体做了什么涉及哪些概念)
  - [7.2 调试时踩到的段错误：符号冲突与 dlopen 隔离](#72-调试时踩到的段错误符号冲突与-dlopen-隔离)
    - [7.2.1 前置：C++ 从编译到运行，符号是怎么被解析的](#721-前置c-从编译到运行符号是怎么被解析的)
    - [7.2.2 这次为什么会冲突（本项目的实际情况）](#722-这次为什么会冲突本项目的实际情况)
    - [7.2.3 这类冲突常见吗](#723-这类冲突常见吗)
    - [7.2.4 处理原理：dlopen 的三个 flag 各自关掉了什么](#724-处理原理dlopen-的三个-flag-各自关掉了什么)
    - [7.2.5 处理模式唯一吗：六种方案对比](#725-处理模式唯一吗六种方案对比)
    - [7.2.6 当前方案的扩展性代价，以及怎么改善](#726-当前方案的扩展性代价以及怎么改善)
  - [7.3 `main` 函数介绍](#73-main-函数介绍)
  - [7.4 `onTimer` 为什么是回调函数，注册过程，其它回调注册方式](#74-ontimer-为什么是回调函数注册过程其它回调注册方式)
  - [7.5 单进程 embed vs 分进程 + IPC：为什么、实现上差在哪](#75-单进程-embed-vs-分进程--ipc为什么实现上差在哪)
- [8. Stage B：`/clock` + `/joint_states`](#8-stage-bclock--joint_states)
  - [8.0 一句话总结](#80-一句话总结)
  - [8.1 改动清单与验证结果](#81-改动清单与验证结果)
  - [8.2 sim time vs wall time，以及 `use_sim_time`](#82-sim-time-vs-wall-time以及-use_sim_time)
  - [8.3 `sensor_msgs/JointState` 字段布局](#83-sensor_msgsjointstate-字段布局)
  - [8.4 关节列表为什么从模型推导而不是写死](#84-关节列表为什么从模型推导而不是写死)
  - [8.5 权衡：发布频率和物理步进频率怎么解耦](#85-权衡发布频率和物理步进频率怎么解耦)
    - [8.5.1 为什么是 5 倍，decimation 到底管什么](#851-为什么是-5-倍decimation-到底管什么)
    - [8.5.2 对齐到设备的任意 fps：动机和做法](#852-对齐到设备的任意-fps动机和做法)
  - [8.6 失败模式与验证手段](#86-失败模式与验证手段)
  - [8.7 排查记录：环境不干净导致的两次误判](#87-排查记录环境不干净导致的两次误判)
    - [第一次：`ros2 topic hz` 读到 1500Hz](#第一次ros2-topic-hz-读到-1500hz)
    - [第二次：RTF 读到 33x（以及"正确做法"其实不正确）](#第二次rtf-读到-33x以及正确做法其实不正确)
    - [留下的经验](#留下的经验)
  - [8.8 墙钟定时器与 sim time 的关系：为什么不可能严格一致，不一致会怎样](#88-墙钟定时器与-sim-time-的关系为什么不可能严格一致不一致会怎样)
    - [8.8.1 确实做不到，而且 rclcpp 的定时器比想象的更"软"](#881-确实做不到而且-rclcpp-的定时器比想象的更软)
    - [8.8.2 能保证的性质：仿真时间自洽 + 无累积漂移](#882-能保证的性质仿真时间自洽--无累积漂移)
    - [8.8.3 不一致会怎样：多数情况下什么都不会](#883-不一致会怎样多数情况下什么都不会)
- [9. Stage C：TF（static + dynamic）](#9-stage-ctfstatic--dynamic)
  - [9.0 一句话总结](#90-一句话总结)
  - [9.1 改动清单与验证结果](#91-改动清单与验证结果)
  - [9.2 动机：TF 以后在哪些环节被用](#92-动机tf-以后在哪些环节被用)
  - [9.3 为什么必须是两个 broadcaster](#93-为什么必须是两个-broadcaster)
  - [9.4 static / dynamic 怎么划分：用结构而不是名字](#94-static--dynamic-怎么划分用结构而不是名字)
  - [9.5 从 `xpos`/`xquat` 到父相对变换：推导与代码对应](#95-从-xposxquat-到父相对变换推导与代码对应)
  - [9.6 四元数分量顺序：一次讲错的更正](#96-四元数分量顺序一次讲错的更正)
  - [9.7 为什么先建索引，而不在 publish 里直接迭代 `model_`](#97-为什么先建索引而不在-publish-里直接迭代-model_)
  - [9.8 frame tree 必须是树](#98-frame-tree-必须是树)
  - [9.9 命名：我们发 MJCF 的原生名](#99-命名我们发-mjcf-的原生名)
  - [9.10 权衡：为什么不用 `robot_state_publisher`](#910-权衡为什么不用-robot_state_publisher)
  - [9.11 权衡：`tf_rate_hz` 独立于 `joint_state_rate_hz`](#911-权衡tf_rate_hz-独立于-joint_state_rate_hz)
  - [9.12 失败模式与验证手段](#912-失败模式与验证手段)
  - [9.13 排查记录：第三次遗留进程（同一个坑的第三次）](#913-排查记录第三次遗留进程同一个坑的第三次)
- [10. 后续计划与 Session 交接](#10-后续计划与-session-交接)
  - [Stage B — `/clock` + `/joint_states` ✅ 已完成](#stage-b--clock--joint_states--已完成)
  - [Stage C — TF（static + dynamic）✅ 已完成](#stage-c--tfstatic--dynamic-已完成)
  - [Stage D — reset service](#stage-d--reset-service)
  - [Stage E — 命令订阅 + sine 测试脚本 + demo launch/rviz](#stage-e--命令订阅--sine-测试脚本--demo-launchrviz)
  - [验收标准（沿用已批准计划里的定义）](#验收标准沿用已批准计划里的定义)
  - [第6周接 MoveIt 时的待决项（Stage C 识别出来的）](#第6周接-moveit-时的待决项stage-c-识别出来的)
- [11. 悬挂问题（等有了参照系再回来）](#11-悬挂问题等有了参照系再回来)
  - [11.0 为什么要单开这一节](#110-为什么要单开这一节)
  - [11.1 清单](#111-清单)
  - [11.2 反向清单：现在就该做的（属于"缺一次推演"）](#112-反向清单现在就该做的属于缺一次推演)
  - [11.3 维护约定](#113-维护约定)

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

## 2. ROS2 节点基础设施：Node、Publisher、QoS、消息类型

这一节是使用 `rclcpp` 时反复要用到的几个基础构件。放在这里（而不是并进 Stage 笔记里）是因为它们跟具体 stage 无关，后面每个 stage 都会引用。

### 2.1 `rclcpp::Node` 基类：常用属性与方法

> Q: 我没有看到 `use_sim_time` 的定义在哪？是否继承自 `Node`？如果是，请把 `Node` 基类的常用属性和方法列表简要介绍一下。

**`use_sim_time` 不在我们代码里，也不是"属性"，而是 `Node` 构造时自动声明的一个参数。** 实测：

```
$ ros2 param list /mujoco_bridge
  joint_state_rate_hz              ← 我们自己 declare_parameter 的
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time                     ← 没写一行代码，它自己就在
$ ros2 param get /mujoco_bridge use_sim_time
Boolean value is: False
```

机制：`rclcpp::Node` 的构造函数内部会创建一个 `NodeTimeSource`（`node.hpp` 里的 `node_time_source_` 成员），它内部持有一个 `rclcpp::TimeSource`。`TimeSource` 在 attach 到节点时声明 `use_sim_time` 这个 bool 参数并注册回调：
- `use_sim_time = false`（默认）→ 节点的 `get_clock()` 走 `RCL_SYSTEM_TIME`，读真实系统时钟；
- `use_sim_time = true` → `TimeSource` 自动**订阅 `/clock` 话题**，用收到的值去驱动节点所有 attach 过的 `rclcpp::Clock`，于是 `get_clock()->now()` 返回仿真时间。

几个推论值得记住：

- **这个订阅是 `TimeSource` 自动建的，不需要你写 subscriber**。这正是 [8.2](#82-sim-time-vs-wall-time以及-use_sim_time) 说"下游节点只要设 `use_sim_time:=true` 就自动对齐"的实现机制。
- `TimeSource` 的 `/clock` 订阅 QoS 可以用 `qos_overrides./clock.*` 参数调（见 `time_source.hpp` 的文档注释）。
- 所以 `use_sim_time` 严格来说不是"继承来的成员变量"，而是"基类构造时替你声明的一个 ROS 参数"。代码里搜不到它是正常的。

**`Node` 常用方法/属性速查**（只列实际会用到的；`Node` 本身有 100+ 个方法，大部分是各种重载和 interface getter）

| 类别 | 成员 | 说明 |
|---|---|---|
| **构造** | `Node(name)` / `Node(name, namespace)` / `Node(name, NodeOptions)` | `NodeOptions` 可设自动声明参数、context、intra-process 通信等 |
| **身份** | `get_name()`、`get_namespace()`、`get_fully_qualified_name()` | 日志/调试用 |
| **日志** | `get_logger()` | 配合 `RCLCPP_INFO/WARN/ERROR/DEBUG(logger, fmt, ...)` 宏。还有 `_ONCE`、`_THROTTLE`、`_SKIPFIRST` 变体——高频循环里打日志**必须**用 `_THROTTLE`，否则 500Hz 会把日志刷爆 |
| **时间** | `get_clock()`、`now()` | `now()` 是 `get_clock()->now()` 的快捷方式。受 `use_sim_time` 影响。**本节点刻意不用它**，理由见 [8.2](#82-sim-time-vs-wall-time以及-use_sim_time) |
| **参数** | `declare_parameter<T>(name, default)`、`get_parameter(name)`、`set_parameter()`、`has_parameter()` | Humble 起**必须先 `declare` 才能 `get`**（否则抛异常），这是相对 ROS1 的一个重要变化。`declare_parameter` 返回声明后的值，所以可以写成 `double x = declare_parameter("x", 1.0);`——本节点的 `joint_state_rate_hz` 就是这个写法 |
| 参数回调 | `add_on_set_parameters_callback(cb)` | 外部改参数时触发，可以在回调里校验并拒绝 |
| **发布/订阅** | `create_publisher<Msg>(topic, qos)`、`create_subscription<Msg>(topic, qos, cb)` | 见 [2.2](#22-publisher-与-qos) |
| **服务** | `create_service<Srv>(name, cb)`、`create_client<Srv>(name)` | Stage D 用 |
| **定时器** | `create_wall_timer(period, cb)`、`create_timer(clock, period, cb)` | 前者绑墙钟，后者可绑任意时钟（含 sim time）。区别见 [8.8](#88-墙钟定时器与-sim-time-的关系为什么不可能严格一致不一致会怎样) |
| **生命周期** | `shared_from_this()` | 拿自己的 `shared_ptr`。**构造函数里不能调用**（那时还没有 `shared_ptr` 管理它），需要在构造后调用的场合要用两阶段初始化 |
| **内部接口** | `get_node_base_interface()`、`get_node_clock_interface()` 等一系列 | 把节点的某个能力单独传给别的组件（比如 `tf2_ros::TransformListener` 要的就是这些 interface 而不是整个 Node）。Stage C 会用到 |

**为什么继承 `Node` 而不是持有一个 `Node`**：两种写法都有人用。继承（本项目的写法）代码更短，`create_publisher` 这些直接就能调；组合（成员里放 `rclcpp::Node::SharedPtr`）则更灵活、便于测试。ROS2 官方更推荐**组合 + component**（`rclcpp_components`，可以把节点动态加载进同一个进程），但对单一可执行文件的桥接节点来说继承足够了。

### 2.2 Publisher 与 QoS

> Q: publisher 是否就是对应一个 topic 的发起组件？但 publisher 就是 bridge node 本身？QoS 是什么？

**Publisher 是什么，和 Node 是什么关系**

不是同一个东西，是**归属**关系：

```
进程 (mujoco_bridge_node 可执行文件)
└── Node "mujoco_bridge"                    ← 一个图节点，有名字、参数、日志
    ├── Publisher<Clock>      → "/clock"     ← 一个发布端点
    ├── Publisher<JointState> → "/joint_states"
    └── WallTimer(2ms)
```

- **Node** 是 ROS 计算图里的一个**参与者**，是名字、参数、日志、时钟的宿主。
- **Publisher** 是一个**具体话题上的发送端点**。一个 Publisher 绑定一个话题名 + 一个消息类型 + 一套 QoS，一经创建不可改。
- 一个 Node 可以有任意多个 Publisher（本节点有 2 个），也可以在同一个话题上建多个 Publisher（虽然通常没必要）。

所以"publisher 就是 bridge node 本身"这个说法不准确。准确说法是：**这个 node *拥有* 两个 publisher**。在 `ros2 topic info` 的输出里能清楚看到这层归属——话题的 endpoint 记录了它属于哪个 node：

```
$ ros2 topic info /clock --verbose
Publisher count: 1
  Node name: mujoco_bridge        ← 端点归属的节点
  Endpoint type: PUBLISHER
  GID: 01.0f.22.90....            ← 这个端点的全局唯一 ID
  QoS profile: ...
```

`publish(msg)` 做的事：把消息序列化，交给底层 DDS，DDS 负责发给所有匹配的订阅端点。**发布是异步的、不阻塞的**，`publish()` 返回不代表任何人收到了，甚至不代表有人在听——没有订阅者时消息直接被丢弃（`Volatile` durability 下）。这也是为什么 Stage B 验证必须用 `ros2 topic echo`/`hz` 实际接一下，光看节点不报错说明不了什么。

**QoS（Quality of Service）是什么**

QoS 是**这条话题的传输策略合同**。ROS1 只有"TCP 还是 UDP"这一个粗糙选择，ROS2 建在 DDS 上，把传输行为拆成一组可配置策略。常用的几条：

| 策略 | 取值 | 含义 |
|---|---|---|
| **Reliability** | `RELIABLE` / `BEST_EFFORT` | `RELIABLE`：丢包会重传，保证送达（代价：可能阻塞、有延迟尖峰）。`BEST_EFFORT`：发出去就不管，丢了算了 |
| **History + Depth** | `KEEP_LAST(n)` / `KEEP_ALL` | 发送/接收队列保留多少条。`KEEP_LAST(1)` = 只留最新一条，新的覆盖旧的 |
| **Durability** | `VOLATILE` / `TRANSIENT_LOCAL` | `TRANSIENT_LOCAL`：publisher 缓存最后 n 条，**晚加入的 subscriber 也能立刻收到**（"latched"）。static TF、robot_description 这类"一次性配置量"必须用它 |
| Deadline / Lifespan / Liveliness | 时长 | 分别是"期望的最大间隔"、"消息过期时间"、"对方还活着吗"。工业场景用得多，本项目暂时不碰 |

**关键约束：QoS 必须兼容，否则连不上**。发布者和订阅者的 QoS 不匹配时，DDS **不会建立连接，而且默认不报错**——现象是"话题在，两边 count 都是 1，但 echo 收不到任何东西"。最常见的踩法是 publisher 用 `BEST_EFFORT` 而 subscriber 要求 `RELIABLE`（订阅方要求的可靠性高于发布方提供的 → 不兼容）。反过来（publisher `RELIABLE`，subscriber `BEST_EFFORT`）是兼容的。排查手段就是 `ros2 topic info --verbose` 对比两边的 QoS profile。

**rclcpp 提供的预设 profile**（都在 `rclcpp/qos.hpp`）：

| 预设 | History | Reliability | 用途 |
|---|---|---|---|
| `rclcpp::QoS(10)` / `SystemDefaultsQoS` | KeepLast(10) | RELIABLE | 通用默认 |
| `SensorDataQoS` | KeepLast(5) | **BEST_EFFORT** | 高频传感器流：图像、点云、IMU。丢一帧无所谓，不能让重传拖慢 |
| `ClockQoS` | **KeepLast(1)** | **BEST_EFFORT** | `/clock` 专用 |
| `ParametersQoS` / `ServicesQoS` | KeepLast(1000) / KeepLast(10) | RELIABLE | 参数和服务，不能丢 |
| `TransientLocalQoS` | — | + TRANSIENT_LOCAL | latched 配置量 |

**本节点的两个选择及理由**：

```cpp
clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));
```

实测确认：

```
/clock         → Reliability: BEST_EFFORT, Durability: VOLATILE   (ClockQoS = KeepLast(1) + BestEffort)
/joint_states  → Reliability: RELIABLE,    Durability: VOLATILE
```

`/clock` 用 `ClockQoS`（best-effort + 深度 1）的道理很值得体会：**一个需要重传才能到达的时间戳，到达时已经过期了**，重传它没有意义，只会在 500Hz 下堆积 backpressure。深度 1 也是同理——旧的时间值永远不如新的有用，队列里留着只会让订阅方处理到过时数据。这是"实时性优先于完整性"的典型场景。

（*笔记留档：我最初在代码注释里把 `ClockQoS` 写成了 "reliable"，是记错了。查 `qos.hpp` 的文档注释和 `ros2 topic info --verbose` 都确认是 best-effort，注释已修正。教训：QoS 这种"默认值型"知识不要凭印象写，`--verbose` 一查就有。*）

`/joint_states` 保持 RELIABLE 的理由：100Hz 不算高频，而且下游（`robot_state_publisher`、MoveIt）做状态估计时丢样本会造成可见的抖动。如果以后频率提上去、或者发现网络成为瓶颈，可以考虑换 `SensorDataQoS`——关节状态本质上确实是传感器数据。

### 2.3 内置消息类型 vs 自定义消息

> Q: 讲讲 ROS2 内置 msgs 的知识（这里是 `rosgraph_msgs` 和 `sensor_msgs`），为什么这里不用自定义 msgs？两种方式的应用场景有什么不同？

**ROS2 的标准消息包生态**

消息类型定义在 `.msg` 文件里，由 `rosidl` 在编译期生成 C++/Python 等各语言的结构体。常用的标准包：

| 包 | 内容 | 例子 |
|---|---|---|
| `std_msgs` | 最基础的裸类型包装 | `String`、`Float64`、`Header` |
| `builtin_interfaces` | 时间原语 | `Time`、`Duration` |
| `geometry_msgs` | 几何量 | `Point`、`Quaternion`、`Pose`、`Transform`、`Twist`、`Wrench` |
| `sensor_msgs` | 传感器数据 | `JointState`、`Image`、`PointCloud2`、`Imu`、`CameraInfo`、`LaserScan` |
| `rosgraph_msgs` | ROS 图基础设施自身用的 | `Clock`、`Log` |
| `trajectory_msgs` | 轨迹 | `JointTrajectory`、`JointTrajectoryPoint` |
| `control_msgs` | 控制器接口 | `JointJog`、`FollowJointTrajectory`（action） |
| `std_srvs` | 标准服务 | `Trigger`、`SetBool`、`Empty` |
| `nav_msgs`、`visualization_msgs`、`shape_msgs`、`moveit_msgs` | 导航/RViz 标记/形状/MoveIt | `Odometry`、`Marker` |

**为什么这里必须用内置类型，不能自定义**

核心理由一句话：**消息类型就是接口契约，用标准类型才能免费接上整个生态**。具体到本节点：

- `rosgraph_msgs/Clock` —— 这个**根本没有选择权**。`use_sim_time` 的实现（`rclcpp::TimeSource`）硬编码了订阅 `/clock` 话题、类型是 `rosgraph_msgs/msg/Clock`。自定义一个 `MySimClock` 消息，`TimeSource` 认不出来，整个 sim time 机制直接失效。这是**框架级约定**。
- `sensor_msgs/JointState` —— 技术上可以自定义，但代价极大。用标准类型的直接收益：`robot_state_publisher` 能直接订阅它算 TF、RViz 的 JointState 面板能直接显示、MoveIt 能直接拿它做当前状态、`ros2 bag` 录下来别人能直接解析、`rqt_plot` 能直接画曲线。**全都是零代码接入**。换成自定义类型，上面每一项都要写一个转换节点。

**那什么时候该自定义消息？**

| 场景 | 该用什么 | 理由 |
|---|---|---|
| 已有标准类型语义完全匹配 | **标准类型** | 免费接生态，别人也看得懂 |
| 语义匹配但字段多余 | **标准类型** | 多余字段留空，不值得为省几个字节切断生态兼容 |
| 项目特有的复合结构 | **自定义** | 例：`GraspCandidate{ Pose pose; float score; string strategy; }`——没有任何标准类型表达"带评分的抓取候选" |
| 需要把多路数据严格绑成一个原子单元 | **自定义** | 例：RGB + Depth + 相机位姿必须同一时刻。分三个话题要做时间同步（`message_filters`），合成一个消息就天然原子 |
| 内部调试/中间量 | 看情况 | 短期可以用 `std_msgs/Float64MultiArray` 凑，但可读性差，长期该自定义 |

**自定义消息的实际成本**（不只是写个 `.msg` 文件）：需要单独一个 `*_msgs` 包（`rosidl_default_generators` + `ament_export_dependencies`），所有用它的包都要依赖这个包，改字段就是改 ABI、所有依赖方要重编。**经验法则：先找标准类型，找不到再自定义；自定义消息里的字段尽量复用标准类型**（比如自己的消息里放 `geometry_msgs/Pose` 而不是 7 个裸 float，这样 RViz/tf2 的工具链还能部分复用）。

本项目后面会碰到真正需要自定义的地方：第5周的抓取候选（`GraspCandidate`）、感知结果（带协方差和置信度的物体位姿）——那些确实没有标准类型能表达。

### 2.4 为什么 `timer_` / `clock_pub_` / `joint_state_pub_` 都是 `SharedPtr`

> Q: 为什么这三个都选择共享指针设计？（假设我对智能指针知识还比较浅薄）

**先补智能指针的最小背景**

C 风格管理资源：`new` 申请 → 用 → `delete` 释放。问题是"释放"这一步靠人记得，忘了就泄漏，重复释放或释放后仍使用（use-after-free）就是段错误。C++ 的解法是 **RAII**：把资源的生命周期绑到一个栈对象上，对象析构时自动释放。智能指针就是这个思路的通用封装：

| 类型 | 所有权语义 | 什么时候释放 |
|---|---|---|
| `std::unique_ptr<T>` | **独占**。不可拷贝，只能移动 | 持有者析构时 |
| `std::shared_ptr<T>` | **共享**。可拷贝，内部维护一个**引用计数** | 最后一个 `shared_ptr` 析构时（计数归零） |
| `std::weak_ptr<T>` | **观察但不拥有** | 不影响释放；用前要 `lock()` 检查对象是否还活着 |

`shared_ptr` 的机制：拷贝一份计数 +1，析构一份计数 -1，归零时删除对象。代价是多一次间接和原子计数操作（线程安全的计数），以及**循环引用会泄漏**（A 持有 B、B 持有 A，计数永不归零——这是 `weak_ptr` 存在的原因）。

对比本项目里 MuJoCo 的 `mjModel*`/`mjData*`：那是纯 C API 的裸指针，必须在析构函数里手动 `api_.deleteData(data_)`。这正好反衬出智能指针的价值——那两行手动释放就是"忘了就泄漏"的风险点（[7.3](#73-main-函数介绍) 提过 `MujocoBridgeNode` 自己是靠 `make_shared` + RAII 自动清理的）。

**为什么 rclcpp 的这三个用 `shared_ptr`**

不是我们选的，是 `create_wall_timer` / `create_publisher` 的**返回类型**就是 `SharedPtr`，我们只能照接。ROS2 这样设计的原因：

1. **所有权真的是共享的**。`create_publisher` 内部把 publisher 注册进了节点的 `NodeTopics`，执行器（`spin`）也要访问它。也就是说这个对象**至少有两个持有方**：你的成员变量，和 rclcpp 内部结构。定时器更明显——执行器的 wait set 必须持有它才能等它到期。`unique_ptr` 表达不了"多方共同持有"，裸指针则说不清谁负责删。

2. **回调期间对象必须保证存活**。执行器正在调 `onTimer()` 的时候，如果定时器对象被别处删掉，就是 use-after-free。引用计数天然解决这个问题：只要执行器还持有一份 `shared_ptr`，对象就活着。

3. **"持有即启用"的生命周期语义**。这一条最容易踩：

   ```cpp
   // 错误写法——定时器立刻就死了
   create_wall_timer(2ms, cb);            // 返回值没接住，shared_ptr 立刻析构，回调永远不触发
   // 正确写法
   timer_ = create_wall_timer(2ms, cb);   // 存进成员变量，节点活着它就活着
   ```

   Publisher 同理：`publish()` 要通过 `clock_pub_` 调，不存下来根本没法用。所以这三个成员变量的真正作用是**把这些对象的生命周期锚定到节点上**——节点析构时，成员变量析构，计数 -1；rclcpp 内部那份也随节点清理掉，计数归零，资源自动释放。

   **一句话**：成员变量不只是"为了以后能调用它"，它同时是"让它保持存活"的手段。

4. **`SharedPtr` 这个名字**：rclcpp 里几乎每个类都有 `using SharedPtr = std::shared_ptr<T>;` 这样的嵌套别名（`RCLCPP_SMART_PTR_DEFINITIONS` 宏生成的），所以 `rclcpp::TimerBase::SharedPtr` 就是 `std::shared_ptr<rclcpp::TimerBase>`。写成嵌套别名纯粹是为了短，以及万一将来换指针类型不用改所有用户代码。

**顺带解释 `main` 里的 `std::make_shared`**：

```cpp
rclcpp::spin(std::make_shared<mujoco_bridge::MujocoBridgeNode>());
```

`spin()` 需要一个 `Node::SharedPtr`（它要在 spin 期间保证节点存活，也要 `shared_from_this` 之类的操作）。用 `make_shared` 而不是 `shared_ptr<T>(new T)` 的理由：一次内存分配就同时放下对象和引用计数块（后者要两次），而且异常安全。

`api_` 是个例外——它是 `MujocoApi &`（引用，不是指针）。因为 `loadMujocoApi()` 返回的是函数内 `static` 单例的引用，那个对象的生命周期是整个程序，不需要任何所有权管理，用引用最直接（详见 [cpp_concepts.md](cpp_concepts.md) 的单例小节）。

### 2.5 `declare_parameter`：ROS 参数到底是什么，为什么不用 `const double`

> Q: 我看到 `declare_parameter` 这里还需要接收 param 参数，为什么，而且我还不知道 `declare_parameter` 在这里除了返回一个 double 还能做什么？那为什么不直接用 `const double`？

**第一个参数是"参数名"，一个字符串 key。** ROS 参数是**节点持有的一个运行时 `string → value` 字典**。C++ 变量名在进程外完全不可见，外界（命令行、launch 文件、YAML、其它节点）只能通过这个字符串指代它。所以名字必须显式给出。

这就是和 `const double joint_state_rate_hz = 100.0;` 的本质区别：**后者根本不存在于进程外**。

**除了返回一个 double，它还做了这些**（以下全部实测，节点确认 `Publisher count: 1`）：

```
$ ros2 param list /mujoco_bridge
  joint_state_rate_hz
  tf_rate_hz
  use_sim_time
  qos_overrides./tf.publisher.depth          ← 附带得到的
  qos_overrides./tf.publisher.durability
  ...
```

| 能力 | 实测 |
|---|---|
| 注册进节点参数表，可枚举 | `ros2 param list` 见上 |
| 可远程读 | `ros2 param get /mujoco_bridge tf_rate_hz` → `Double value is: 40.0` |
| **启动时覆盖，不用重编译** | `--ros-args -p tf_rate_hz:=40.0` → 日志 `/tf every 13 steps (40.0 Hz requested, 38.5 Hz actual)`，`ros2 topic hz /tf` 实测 `38.462` ✅ |
| 类型被默认值钉住 | 默认值写 `100.0`（double），传非数值会被拒 |
| 变更事件广播 | 这就是 `ros2 topic list` 里 `/parameter_events` 的来源 |
| 重复声明同名会抛异常 | `ParameterAlreadyDeclaredException`，防同名冲突 |
| 顺带暴露 QoS 覆盖 | 上面那堆 `qos_overrides.*`，可在运行时调话题 QoS 而不改代码 |

**为什么不用 `const double`**：`const double` 是编译期常量，改发布频率要重新 `colcon build` + 重装。参数的整个意义是**让运行这个节点的人（launch 文件、调试中的自己）在不碰源码的前提下调**。这是 ROS 里暴露可调旋钮的标准做法。

**顺带答掉 [2.1](#21-rclcppnode-基类常用属性与方法) 留下的一个问题**：`use_sim_time` 出现在 `ros2 param list` 里，而我们从没声明它——它是 `rclcpp::Node` **基类构造时自己声明的**。当时那条"我没看到 `use_sim_time` 定义在哪"，答案就是这个。

**一个必须记下的缺陷（实测）**：

```
$ ros2 param set /mujoco_bridge tf_rate_hz 10.0
Set parameter successful          ← 报告成功
$ ros2 topic hz /tf
average rate: 38.457              ← 频率一点没变
```

因为构造函数里**读一次就算成 `tf_decimation_` 存起来了**，运行时改参数只改了字典里那个 double，没人再去看它。这是典型的 C 类问题：**接口报告成功、实际无效果、无任何告警**。要真支持热改需要 `add_on_set_parameters_callback` 重算 decimation。现在没做，进了 [11.2](#112-反向清单现在就该做的属于缺一次推演) 反向清单。

---

## 3. TF / Frame 约定

### 3.1 为什么必须先约定 Frame，关键规则有哪些

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

## 4. 机器人描述模型的来源

### 4.1 apt 装 vs 自己 vendor，MoveIt 的 SRDF 从哪来

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

### 4.2 vendor 是什么意思

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

### 4.3 怎么检查和分析 MJCF / URDF，以及怎么据此判断代码正误

> Q: 目前的代码，以及讲解，有多处涉及 URDF/MJCF 等描述文件，但我还不知道应该如何去检查分析它们，如何根据我的检查和分析判断代码正误，比如每个关节的状态变量应该是什么样。

#### 4.3.1 两边都要先"展平"，否则等于没读

**MJCF 有继承，必须展平。** `panda.xml` 里写的是：

```xml
<joint name="joint1"/>          <!-- 什么属性都没有 -->
```

它的 `range` / `damping` / `armature` / `axis` 全部来自上面的 `<default class="panda">`。直接读 body 段**什么都看不出来**。MuJoCo 自带的 `compile` 工具会把所有 default 继承展开：

```bash
compile panda.xml /tmp/panda_flat.xml     # 在 $MUJOCO_DIR/bin 里
```

展平后（实跑结果）：

```xml
<joint name="joint1" pos="0 0 0" axis="0 0 1"/>                        <!-- range 从 default 继承 -->
<joint name="joint4" pos="0 0 0" axis="0 0 1" range="-3.0718 -0.0698"/>
<joint name="finger_joint1" class="finger" pos="0 0 0" axis="0 1 0"/>
```

**URDF 本身是扁平的**（没有继承），但 `.xacro` 有宏和参数，所以先展开：

```bash
xacro /opt/ros/humble/share/franka_description/robots/fer/fer.urdf.xacro > /tmp/fer.urdf
```

之后 URDF 就是一串扁平的 `<joint>`，一遍 ElementTree 就能打出整棵树（[4.3.3](#433-urdf--mjcf-交叉验证这是判断代码正误的主要手段) 那张表就是这么来的）。

另有交互式手段：`simulate panda.xml` 看 MJCF，`joint_state_publisher_gui` + RViz 拖 URDF 关节。

#### 4.3.2 该按什么顺序看（MJCF）

| 看什么 | 为什么它排在前面 |
|---|---|
| `<compiler angle="radian">` | **决定所有角度数字的含义**。如果是 `degree`，下面每个数的意思都变 |
| `<default>` 链 | 关节的**有效**属性来源，不看等于没读 |
| body 嵌套 + `pos`/`quat` | 这就是运动学树 + 每条边的静止变换 |
| `<joint>` 的 `type`（缺省 hinge）/ `axis` / `range` | 决定它在 `qpos` 里占几位、单位是什么 |
| `<actuator>` | **控制接口的真实形状**。见 [4.3.4](#434-每个关节的状态变量应该是什么样以及一个-stage-e-地雷)，这里有坑 |
| `<equality>` / `<tendon>` | 关节间的耦合。`qpos` 的维度会骗人 |
| `<keyframe>` | 预定义的 `qpos`/`ctrl` 组合（Stage D 的 reset 用它） |

#### 4.3.3 URDF ↔ MJCF 交叉验证：这是判断代码正误的主要手段

两份描述来自**两个互不知道的上游**（`mujoco_menagerie` 和 `franka_description`）。如果它们几何参数一致，说明我们信任的那份没被改坏。逐行核对结果——13 条边全对：

| URDF joint (`origin xyz / rpy`) | MJCF body (`pos` / `quat`) | 一致？ |
|---|---|---|
| `joint1` `0 0 0.333` / `0 0 0` | `link1 pos="0 0 0.333"` | ✅ |
| `joint2` `0 0 0` / `-π/2 0 0` | `link2 quat="1 -1 0 0"` = −90° 绕 x | ✅ |
| `joint3` `0 -0.316 0` / `π/2 0 0` | `link3 pos="0 -0.316 0" quat="1 1 0 0"` | ✅ |
| `joint4` `0.0825 0 0` / `π/2 0 0` | `link4 pos="0.0825 0 0" quat="1 1 0 0"` | ✅ |
| `joint5` `-0.0825 0.384 0` / `-π/2 0 0` | `link5 pos="-0.0825 0.384 0" quat="1 -1 0 0"` | ✅ |
| `joint6` `0 0 0` / `π/2 0 0` | `link6 quat="1 1 0 0"` | ✅ |
| `joint7` `0.088 0 0` / `π/2 0 0` | `link7 pos="0.088 0 0" quat="1 1 0 0"` | ✅ |
| `joint8` `0 0 0.107` **然后** `hand_joint` `0 0 0` / `0 0 -π/4` | `hand pos="0 0 0.107" quat="0.9238795 0 0 -0.3826834"` = −45° 绕 z | ✅ 合并写法 |
| `hand_tcp_joint` `0 0 0.1034` / `0 0 0` | **MJCF 里没有** → 我们合成 | — |
| `finger_joint1` `0 0 0.0584` | `left_finger pos="0 0 0.0584"` | ✅ |
| `finger_joint2` `0 0 0.0584` / `0 0 π` | `right_finger pos="0 0 0.0584" quat="0 0 0 1"` = 180° 绕 z | ✅ |

（怎么把 `1 1 0 0` 读成"绕 x 转 90°"，见 [6.3](#63-四元数基础从半角到位姿复合)。）

这张表直接支撑了 [9.1](#91-改动清单与验证结果) 的 `tf2_echo` 验证：`link4→link5` 实测 `[-0.082, 0.384, 0]` + RPY `[-90°,0,0]`，**两份独立描述都说该是这个数**，所以那不是自证。

**顺带发现 `architecture.md` 两处要改**（已改）：
1. URDF 侧的根是 `base`，所有名字带 `fer_` 前缀（`fer_link0`、`fer_leftfinger`），**没有 `world`**。
2. 原文写"TCP 已经把夹爪长度和**默认 45° 旋转**的偏移量算进去"——**这条错了**。`tcp_rpy` 默认是 `0 0 0`，`hand_tcp` 是**纯 103.4mm 平移**；那个 −45° 在 `hand_joint`（`link8→hand`）上。

#### 4.3.4 "每个关节的状态变量应该是什么样"，以及一个 Stage E 地雷

这个问题的答案不在 `<joint>` 里，在 `<actuator>` 里。展平后：

```xml
<general name="actuator1" joint="joint1" gainprm="4500" biasprm="0 -4500 -450"/>
...
<general name="actuator7" joint="joint7" gainprm="2000" biasprm="0 -2000 -200"/>
<!-- Remap original ctrlrange (0, 0.04) to (0, 255): 0.04*100/255 = 0.01568627451 -->
<general name="actuator8" tendon="split" forcerange="-100 100" ctrlrange="0 255"
         gainprm="0.01568627451 0 0" biasprm="0 -100 -10"/>
```

加上：

```xml
<tendon><fixed name="split">
  <joint joint="finger_joint1" coef="0.5"/><joint joint="finger_joint2" coef="0.5"/>
</fixed></tendon>
<equality><joint joint1="finger_joint1" joint2="finger_joint2" polycoef="0 1 0 0 0"/></equality>
```

于是本模型的真实形状是：

| 量 | 值 | 含义 |
|---|---|---|
| `njnt` / `nq` / `nv` | 9 / 9 / 9 | 7 个臂 hinge + 2 个指 slide，全是 1-DoF |
| **`nu`** | **8** | 7 个臂 actuator + **1 个**夹爪 actuator |
| `neq` | 1 | `finger_joint1 == finger_joint2`（`polycoef="0 1 0 0 0"` 就是这个等式） |
| `ntendon` | 1 | `split`，把一个 ctrl 摊到两指 |

**这是 Stage E 的三颗地雷**（计划里写的是"把 `positions` 按 joint name 写进 `mjData::ctrl`"）：

1. **`ctrl` 长度是 `nu`=8，不是 `nq`=9**，索引是 **actuator id，不是 joint id**。`ctrl[joint_index]` 对 7 个臂关节碰巧对（actuator 顺序恰好同 joint 顺序），到夹爪就错位/越界。正确做法：`mj_name2id(m, mjOBJ_ACTUATOR, name)` 建一份 joint→actuator 映射，或用 `actuator_trnid`。
2. **`actuator8` 的 `ctrlrange` 是 `0..255`，不是 `0..0.04`**（上游注释明说是为了对齐真机夹爪的 0-255 接口重映射过）。直接把 `finger_joint1` 的目标角 `0.04` 写进去，夹爪会收到"0.04/255"的指令，几乎全闭。`keyframe` 里 `ctrl="... 255"` 印证了这点。
3. **两指不能独立指令**。物理上只有一个可控自由度。

三条都是"读描述文件读出来的"，不读就要在 Stage E 靠调试撞出来。

### 4.4 `hand_tcp` 是什么，以及 `0.1034` 这个 magic number

> Q: 比如 `0.1034` 这个 magic number（为什么不统一写成一个公共的常量）（`hand_tcp` 不在哪又在哪，意义是什么）。

#### 4.4.1 它是一个纯粹的命名坐标系，不是零件

URDF 里它长这样，完整、没省略（`franka_description/end_effectors/common/franka_hand.xacro`）：

```xml
<link name="fer_hand_tcp" />                            ← 空的
<joint name="fer_hand_tcp_joint" type="fixed">
  <origin xyz="0 0 0.1034" rpy="0 0 0" />
  <parent link="fer_hand" /><child link="fer_hand_tcp" />
</joint>
```

`<link>` 里**没有 `<visual>`、没有 `<collision>`、没有 `<inertial>`**。所以：

**`hand_tcp` 不参与动力学、不参与碰撞、不占 DoF，它只是一个约定**——"抓取点在这儿"。

这就是它"不在 MJCF 里"的原因：MJCF 对应的概念是 `<site>`（同样是无质量标记点），而 menagerie 这份模型**只是没定义**，不是 MuJoCo 表达不了。

它的**意义是消除重复**：夹爪长度 103.4mm 这个偏移，如果不定义 `hand_tcp`，抓取模块、规划模块、可视化各自都要在自己代码里加一遍——三份副本、三个出错机会。定义成一个 frame 后大家统一说"我要 `hand_tcp` 到这儿"。这就是 [architecture.md](../../docs/architecture.md) 里"统一以 `hand_tcp` 作为末端参考 frame"的含义。

#### 4.4.2 为什么这个数没有自动共享

因为**两份描述来自两个互不知道的上游、之间没有任何构建步骤**：`franka_description` 是 apt 装的、`panda.xml` 是 vendor 进来的，谁都不读谁。这个数在我们这儿**必然是一份手抄的副本**。

已经做的改善（[mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp)）——具名常量 + 注释写清上游出处和核对命令：

```cpp
constexpr const char * kHandBodyName = "hand";
constexpr const char * kTcpFrameName = "hand_tcp";
constexpr double kHandToTcpZ = 0.1034;
```

**但要说清它解决了什么**：具名常量只消除了"一个进程内有几份副本"，**没有**解决"上游改了我们不知道"。真正的修法两条：

- **(a) 把这个数搬进 vendor 的 MJCF**：给 `hand` body 加 `<site name="hand_tcp" pos="0 0 0.1034"/>`，代码从模型里读。数值就住在**描述层**、和其它几何参数待在一起，代码里零 magic number。**倾向这个做法**，还顺手能消掉 `kHandBodyName` 那个硬编码字符串（改成"找 site 存不存在"）。
- **(b) 写测试**：展开 xacro、断言 `tcp_xyz == 0.1034`。能抓上游漂移，但要跑 xacro，比较重。

没在 Stage C 做，因为改 vendor 的模型文件是个独立决定。进 [11.2](#112-反向清单现在就该做的属于缺一次推演)。

---

## 5. 开发环境：VSCode + distrobox

### 5.1 `#include <rclcpp/rclcpp.hpp>` 标红怎么办

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

### 5.2 已经 attach 容器了仍然标红

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

## 6. 运动学（FK）

### 6.1 FK 实践中可用的组件、需要自己写的部分、构型鲁棒写法

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

### 6.2 广义坐标：`qpos` / `qvel` 的表示方法，以及为什么维度不相等

> Q: 补充一些 FK 基础知识：`qpos` 和 `qvel` 的表示方法，包含为什么 pos 比 vel 要多一个维度。
>
> Q（相关）：我不太理解 free joint，它和其它 joint 有什么区别？

这两个问题是同一件事的两面，一起讲。

**广义坐标（generalized coordinates）的概念**

刻画一个机械系统的状态，不需要记录每个刚体的完整 6 自由度位姿——关节已经把相对运动约束住了。Panda 的 7 个连杆各有 6 个自由度（共 42），但因为 7 个转动关节把它们串成一条链，**整个手臂的构型只需要 7 个数**（每个关节转了多少弧度）就完全确定。这 7 个数就是广义坐标。

MuJoCo 把它们存成两个平铺的数组：

- `mjData::qpos`，长度 `mjModel::nq` —— **位置**（构型）
- `mjData::qvel`，长度 `mjModel::nv` —— **速度**

FK 做的事就是：给定 `qpos`，沿运动学树把每一节的变换连乘起来，算出每个 body 的 world 位姿（结果落在 `mjData::xpos`/`xquat` 里）。所以 `qpos` 是 FK 的**输入**，`xpos`/`xquat` 是**输出**。

**四种关节类型，各占多少维**

MuJoCo 的关节类型定义在 `mjmodel.h`，注释里直接标了维度：

```c
typedef enum mjtJoint_ {          // type of degree of freedom
  mjJNT_FREE  = 0,   // global position and orientation (quat)      (7)
  mjJNT_BALL,        // orientation (quat) relative to parent       (4)
  mjJNT_SLIDE,       // sliding distance along body-fixed axis      (1)
  mjJNT_HINGE        // rotation angle (rad) around body-fixed axis (1)
} mjtJoint;
```

整理成表（关键在于 `qpos` 和 `qvel` 占的维度**不一定相同**）：

| 类型 | 物理含义 | `qpos` 维度 | `qvel` 维度 | 表示方式 |
|---|---|---|---|---|
| `HINGE` | 绕固定轴转动（Panda 的 7 个臂关节） | 1 | 1 | 角度 (rad) / 角速度 (rad/s) |
| `SLIDE` | 沿固定轴平移（Panda 的 2 个夹爪指） | 1 | 1 | 位移 (m) / 线速度 (m/s) |
| `BALL` | 球关节，相对父体的任意朝向 | **4** | **3** | 单位四元数 / 角速度向量 |
| `FREE` | 完全自由的 6 自由度刚体（掉在桌上的待抓物体） | **7** | **6** | 位置(3) + 单位四元数(4) / 线速度(3) + 角速度(3) |

所以 `nq` 和 `nv` 是两个独立的数字：

```c
int nq;   // number of generalized coordinates = dim(qpos)
int nv;   // number of degrees of freedom      = dim(qvel)
```

Panda 全是 hinge/slide，所以 `nq == nv == 9`，两者恰好相等——**这个巧合很有欺骗性**，见下面。

**为什么 position 比 velocity 多一维（四元数的本质原因）**

关键在于**朝向（rotation）的表示**。三维旋转本身只有 **3 个自由度**（可以用绕 x/y/z 的三个角度描述），但用 3 个数去表示旋转**位置**会出问题：

- 欧拉角（roll-pitch-yaw）用 3 个数，但有**万向锁（gimbal lock）**：在某些姿态下两个轴退化成同一个，失去一个自由度，且附近数值剧烈不稳定。
- 旋转向量（轴×角）用 3 个数，但在角度接近 π 时有奇异性，而且插值/复合运算很别扭。

**用 4 个数的单位四元数就没有奇异性**，代价是多带一个数，并且必须满足约束 ‖q‖ = 1 —— 也就是说这 4 个数只有 3 个是独立的，第 4 个被归一化约束掉了。这是"4 维表示 3 维流形"的标准技巧：**多一个数换取全局无奇异**。

而**角速度**不需要这个技巧。角速度是一个实实在在的 3 维向量 ω（旋转轴方向 × 转动快慢），它活在切空间里，没有奇异性问题，3 个数就够。

所以：

```
FREE joint:  qpos = [x, y, z, qw, qx, qy, qz]   ← 3 位置 + 4 四元数 = 7
             qvel = [vx, vy, vz, ωx, ωy, ωz]    ← 3 线速度 + 3 角速度 = 6
BALL joint:  qpos = [qw, qx, qy, qz]            ← 4
             qvel = [ωx, ωy, ωz]                ← 3
```

**用数学语言说**：构型空间不是一个平坦的向量空间，而是一个**流形**（manifold）。朝向的构型空间是 SO(3)，它是 3 维的但没法用 3 个数全局无奇异地参数化，只能嵌进 4 维（单位四元数 = S³ 球面）。速度活在流形某一点的**切空间**里，切空间是真正平坦的 3 维向量空间。所以"位置多一维、速度不多"不是编码上的随意选择，而是**流形和它的切空间维度本来就可以不同**。

这也解释了一个实践上的重要后果：**不能对 `qpos` 做朴素的加减和插值**。`qpos_a - qpos_b` 对 hinge 关节是有意义的角度差，但对 free joint 的四元数部分毫无意义（两个单位四元数相减不是单位四元数）。同理积分也不是 `qpos += qvel * dt` —— MuJoCo 内部对四元数部分做的是流形上的指数映射更新，这是 `mj_step` 里 `mj_integratePos` 负责的事。**自己写代码碰到 free/ball joint 时，任何"把 qpos 当普通数组做算术"的写法都是 bug。**

**Free joint 和其它 joint 的区别（回答 Q7）**

概念上的区别：**普通关节是"约束"，free joint 是"没有约束"**。

- `HINGE`/`SLIDE`/`BALL` 描述的是**父体和子体之间的相对运动被限制成什么样**。Panda 的 `joint2` 说的是"link2 相对 link1 只能绕某个固定轴转动，范围 ±1.76 rad"——它**减少**自由度（从 6 减到 1）。
- `FREE` 说的是"这个 body 相对 world **完全自由**，6 个自由度一个都不限制"。它不约束任何东西，只是给这个刚体在广义坐标里**分配 6 个自由度的存储**。

用途：场景里**不连在机器人上的独立物体**。待抓的方块、桌上的杯子、掉落的零件——它们的运动完全由重力和接触力决定，没有"关节"把它们连到什么东西上。MJCF 里通常写成 `<freejoint/>`。

（实测：现在的 `panda.xml` 里 `freejoint` 数量为 0 —— 场景里还只有机器人本体。第4周往场景加待抓物体时才会出现。）

对本项目的四个具体影响，都已经在代码里处理了：

1. **`nq != nv`**。加一个 free joint 物体后，`nq` 变 16、`nv` 变 15。所以 `qpos` 的索引和 `qvel` 的索引**不再是同一套**，必须分别用 `jnt_qposadr[i]` 和 `jnt_dofadr[i]` 查（[8.3](#83-sensor_msgsjointstate-字段布局) 强调过）。Panda 上两者数值恰好相同，**这个 bug 现在写下去完全看不出来，加物体的那天才会炸**——这是"写对但暂时无从验证"的典型，值得特别小心。
2. **不该进 `JointState`**。free joint 没有"一个关节角"，7 个数里混着位置和四元数，塞进 `position[]` 数组下游无法解释。它的状态应该走 TF（`world → object`）。所以 `buildJointIndex()` 里显式 `if (type != mjJNT_HINGE && type != mjJNT_SLIDE) continue;`（[8.4](#84-关节列表为什么从模型推导而不是写死)）。
3. **不该被当作可控关节**。free joint 没有 actuator，不能下发命令，只受物理规律驱动。Stage E 的命令订阅按 name 查 actuator，天然不会碰到它。
4. **oracle 位姿的来源**。ground-truth 物体位姿就是读这个 free joint 对应 body 的 `xpos`/`xquat`（或直接读 `qpos` 那 7 个数）。这是第4-5周做 oracle vs vision 对照实验的数据来源，对应 [3.1](#31-为什么必须先约定-frame关键规则有哪些) 规则4 里说的"同一个 frame 名、不同 topic 区分来源"。

**`ctrl` 是第三个维度，别和上面两个混了**：`mjData::ctrl` 长度是 `mjModel::nu`（actuator 个数），既不等于 `nq` 也不等于 `nv`。Panda 这份 MJCF 有 8 个 actuator（7 个臂关节 + 1 个夹爪，两个指头由同一个 actuator 驱动），所以 `nu = 8` 而 `nq = 9`。Stage E 写 `ctrl` 时必须按 actuator 名字查 `mj_name2id(m, mjOBJ_ACTUATOR, name)`，**不能用关节索引去索引 `ctrl`**——这是个会静默错位的坑（MJCF 里 `home` keyframe 的 `ctrl` 正好是 8 个数，可以用来交叉验证）。Stage C 阶段读 `<actuator>` 段时又挖出两条相关的坑（`ctrlrange` 被重映射成 `0..255`、两指不可独立指令），见 [4.3.4](#434-每个关节的状态变量应该是什么样以及一个-stage-e-地雷)。

### 6.3 四元数基础：从半角到位姿复合

> Q: 我需要四元数运算的基础知识，以便了解 TF publish 函数到底在干什么，发送的是什么。

这一节是 [9.5](#95-从-xposxquat-到父相对变换推导与代码对应) 的前置，同时也是读懂 MJCF 里那些 `quat="1 1 0 0"` 的前提。

#### 6.3.1 一个单位四元数就是一个旋转

$q = (w, x, y, z)$，满足 $w^2+x^2+y^2+z^2 = 1$。它编码"绕单位轴 $\mathbf{n}$ 转 $\theta$"：

$$w = \cos\frac{\theta}{2},\qquad (x,y,z) = \mathbf{n}\sin\frac{\theta}{2}$$

**注意是半角。** 学会这个就能徒手核对描述文件——MJCF 里那些数字的真实含义：

| MJCF 写的 | 归一化后 | $\theta = 2\arccos w$ | 轴 | 读作 |
|---|---|---|---|---|
| `1 1 0 0` | $(0.7071, 0.7071, 0, 0)$ | $2 \times 45° = 90°$ | $+x$ | 绕 x 转 +90° |
| `1 -1 0 0` | $(0.7071, -0.7071, 0, 0)$ | $90°$ | $-x$ | 绕 x 转 −90° |
| `0 0 0 1` | $(0,0,0,1)$ | $2 \times 90° = 180°$ | $+z$ | 绕 z 转 180° |
| `0.9238795 0 0 -0.3826834` | 已归一 | $2 \times 22.5° = 45°$ | $-z$ | 绕 z 转 −45° |
| `1 0 0 0` | — | $0°$ | — | 单位（不转） |

（`quat` 在 MJCF 里不要求已归一化，编译器会归一。`w=0` ⟺ 恰好 180°。）

**一个陷阱**：$q$ 和 $-q$ 表示**同一个旋转**（半角的后果，叫 double cover）。所以**"逐字段比较两个四元数相等"的断言可能对正确值报失败**——要比 $|q_1 \cdot q_2| \approx 1$，或先把 $w$ 的符号归一。这是将来写 TF 单测的第一个坑。

#### 6.3.2 三个运算

| 运算 | 数学 | MuJoCo |
|---|---|---|
| **共轭 = 逆**（单位四元数） | $q^* = (w, -x, -y, -z)$ | `mju_negQuat(res, q)` |
| **复合** | $q_1 \otimes q_2$（Hamilton 积） | `mju_mulQuat(res, q1, q2)` |
| **旋转一个向量** | $(0,\mathbf{v}') = q \otimes (0,\mathbf{v}) \otimes q^*$ | `mju_rotVecQuat(res, v, q)` = $R(q)\mathbf{v}$ |

复合的关键性质两条：

$$R(q_1 \otimes q_2) = R(q_1)\,R(q_2) \qquad\text{且}\qquad q_1 \otimes q_2 \ne q_2 \otimes q_1$$

**不可交换**。所以 `mulQuat` 的参数顺序是承重的，写反了不会报错、只会得到错的姿态。

最有用的读法是**坐标系复合**：若 $q_1 = q_{A \leftarrow B}$（B 系在 A 系里的姿态）、$q_2 = q_{B \leftarrow C}$，则

$$q_1 \otimes q_2 = q_{A \leftarrow C}$$

"下标像分数一样约掉"。记住这个，参数顺序就不用背。

#### 6.3.3 位姿（不只是旋转）怎么复合

一个位姿 $T = (\mathbf{p}, q)$ 的含义是：**body 系里的点 $\mathbf{v}$ 映射到父系的 $R(q)\mathbf{v} + \mathbf{p}$**。这就是 `mjData::xpos`/`xquat` 的含义（父系 = world），也是 `TransformStamped` 的含义（父系 = `header.frame_id`）。

两条规则：

$$(\mathbf{p}_1, q_1) \circ (\mathbf{p}_2, q_2) = \big(\mathbf{p}_1 + R(q_1)\mathbf{p}_2,\ \ q_1 \otimes q_2\big)$$

$$(\mathbf{p}, q)^{-1} = \big(-R(q^*)\,\mathbf{p},\ \ q^*\big)$$

[9.5](#95-从-xposxquat-到父相对变换推导与代码对应) 就是把这两条代进一个具体场景。

## 7. Stage A：mujoco_bridge_node 最小实现（模型加载 + 物理步进定时器）

### 7.0 一句话总结

初步搭起了 ROS 和 MuJoCo 仿真器的桥接雏形：用 C++ 通过 `dlopen`/`dlsym` 调用 MuJoCo 的 API，实际读取了机器人的 MJCF 描述文件，并用 `mj_step`（而不是 `mj_forward`——两者的区别见 [7.1](#71-stage-a-具体做了什么涉及哪些概念)）做定长步进积分来推进仿真。具体做法：设置一个周期等于 `timestep_s` 的软件定时器，每次触发都调一次 `mj_step` 并累计步数，每隔一定步数读一次 `mjData::time`（MuJoCo 自己按"步数 × timestep"累计的仿真时长）打日志。

验证结果：MuJoCo 里读到的 `data_->time` 和"软件步数 × 定时器周期"数值上完全吻合。但这个吻合**不是墙钟时间决定了仿真时间**，而是因为定时器周期被故意设成了和 MJCF 的 `timestep` 相等（都是 0.002s）——`data_->time` 全程只由"步数 × timestep"决定，跟定时器周期本身无关，只是这次两个数字凑成了同一个值，才看起来像是"对上了"。

### 7.1 Stage A 具体做了什么，涉及哪些概念

> Q: Stage A 具体做了什么，涉及哪些概念？

**代码结构**（`src/mujoco_bridge/src/mujoco_bridge_node.cpp`）：一个 `MujocoBridgeNode : public rclcpp::Node`，构造函数里加载 MJCF、创建 `mjData`、启动一个定时器；定时器回调里每次调一次 `mj_step`；析构函数释放 `mjData`/`mjModel`。`main()` 就是标准的 `rclcpp::init` → `spin` → `shutdown` 三段式（详见 [7.3](#73-main-函数介绍)）。

**`mjModel` / `mjData` 的关系**：`mjModel` 是"编译后的模型描述"——从 MJCF 解析出来的静态结构（有多少个 body/joint/actuator、它们的连接关系、惯量、几何形状……），加载一次之后不会变。`mjData` 是"当前仿真状态"——`qpos`（关节位置）、`qvel`（关节速度）、`xpos`/`xquat`（每个 body 的 world 位姿）、`ctrl`（下发给 actuator 的控制量）等，每步物理都会变。这个分离的好处：同一个 `mjModel` 可以配多份 `mjData`（比如做并行仿真/MPC rollout），模型本身不用重复解析。

**`mj_step` 每次前进多少**：固定前进 `mjModel->opt.timestep` 这么多仿真时间，这个值来自 MJCF 里 `<option timestep="0.002".../>`（Panda 这份模型里是 0.002s）。**不是**每次调用时按真实经过的墙钟时间前进——如果那样做，物理积分步长会随系统调度抖动变化，数值积分（尤其接触/摩擦）在变步长下会不稳定甚至发散。所以正确做法是：仿真时间 = 步数 × 固定 timestep，跟墙钟解耦。代码里 `data_->time` 就是 MuJoCo 自己维护的这个累加值，日志里 `sim_time` 直接读它，不是自己算的。

**定时器多久触发一次**：`create_wall_timer` 的周期设成跟 `timestep` 一样（0.002s → 500Hz），这样"墙钟触发频率"和"仿真步长"数值上凑成 1:1，日志里能看到 1 秒真实时间对应恰好 1.000s 仿真时间。但这只是尽量让两者同步、方便观察，本质上二者是两个独立的量——如果某次定时器回调被系统调度延迟触发（真实间隔变成比如 3ms），仿真时间依然只前进一个 timestep（0.002s），不会跟着墙钟的延迟一起变化，下一次回调也不会"追赶"。这就是上一段说的解耦：真正决定仿真时间的永远是"步数 × timestep"，墙钟只是决定"多久调用一次 `mj_step`"这个节奏，不参与仿真时间的计算。

**为什么用 `create_wall_timer` 而不是 `spin` 里裸写循环**：`rclcpp::spin(node)` 本身是一个事件循环，负责处理所有回调（定时器、订阅者、服务）——不能在 `main` 里再写一个 `while` 循环手动调 `mj_step`，那样会跟 `spin` 抢执行权、也没法同时处理 ROS 的其他回调（后面 Stage D 的 reset service 就是靠 `spin` 调度进来的）。`create_wall_timer` 把"周期性执行"这件事注册给 `spin` 的事件循环去调度，是 ROS2 里做周期性任务的标准方式（回调注册机制详见 [7.4](#74-ontimer-为什么是回调函数注册过程其它回调注册方式)）。

**验证结果**：`ros2 run mujoco_bridge mujoco_bridge_node` 跑 3 秒多，日志显示：
```
Loaded .../panda.xml (nq=9, timestep=0.0020s)
step=500  sim_time=1.000s
step=1000 sim_time=2.000s
step=1500 sim_time=3.000s
```
`nq=9`：Panda 7 个臂关节 + 2 个夹爪指关节，和 architecture.md 里记录的关节数一致。500 步 = 1 秒仿真时间，和 0.002s×500=1.0s 对得上。

### 7.2 调试时踩到的段错误：符号冲突与 dlopen 隔离

> Q: 调试 Stage A 时踩到一个很深的段错误，根因是什么？为什么最终改成 `dlopen` 而不是直接 `target_link_libraries(mujoco::mujoco)`？希望从 C++ 编译链接运行开始讲，讲清楚为什么会冲突、这种冲突常见吗、处理原理是什么、处理模式唯一吗，以及当前处理方式的扩展性。

**现象**：一开始按计划直接链接 `mujoco::mujoco`（正常的 CMake `target_link_libraries`），只要可执行文件里同时"链接了 MuJoCo"和"构造了一个 `rclcpp::Node`"，进程就必定在构造 `rclcpp::Node()` 那一行段错误——即使完全没调用任何 `mj_*` 函数，光链接就会崩；且必须两个条件同时满足才崩（只链接不构造 Node 不崩、只构造 Node 不链接 MuJoCo 也不崩）。`dmesg` 里是 `error 14`（= write fault + user mode + page-not-present），典型的"通过一个垃圾指针往外写"。

这个"两个东西单独都没问题、放一起就崩，而且崩在跟两者都无关的第三行代码上"的形态，是符号冲突的典型指纹。要讲清楚它，得先把 C++ 从源码到运行的符号解析链路过一遍。

#### 7.2.1 前置：C++ 从编译到运行，符号是怎么被解析的

**四个阶段，符号在每个阶段的状态不同**

1. **预处理**：`#include` 被替换成头文件的文本。头文件里通常只有**声明**（`void mj_step(const mjModel*, mjData*);`），没有函数体。此时编译器只知道"有这么个名字、长这个签名"。
2. **编译**：每个 `.cpp` 单独编译成 `.o`（目标文件）。编译器遇到 `mj_step(...)` 的调用，因为不知道它在哪，就在 `.o` 里留一个**未定义符号（undefined symbol，`nm` 里标 `U`）**，同时在调用点留一个"待填的地址"。反之，本文件里定义了函数体的，就在 `.o` 里记成**已定义符号（`T`/`D`）**。

   C++ 在这一步会做 **name mangling（名字修饰）**：把命名空间、类名、参数类型全部编码进符号名，好让重载/模板/命名空间能共存。所以 `tinyxml2::XMLDocument::Identify(char*, XMLNode**)` 在符号表里长这样：

   ```
   _ZN8tinyxml211XMLDocument8IdentifyEPcPPNS_7XMLNodeE
   ```

   `_ZN` = 嵌套名开始，`8tinyxml2` = 长度 8 的 `tinyxml2`，`11XMLDocument` = 长度 11 的类名，`8Identify` = 方法名，`E` 后面是参数类型（`Pc` = `char*`，`PPNS_7XMLNodeE` = `XMLNode**`）。C 语言不 mangle，`mj_step` 在符号表里就叫 `mj_step`。

   **这里埋下了本次事故的第一颗种子**：mangled 名字里编码了签名，但**没有编码版本、也没有编码这个 `tinyxml2` 来自哪个库**。符号名是全局扁平的命名空间，谁都能定义 `_ZN8tinyxml2...`。
3. **链接（link）**：链接器（`ld`）把若干 `.o` 和库拼成可执行文件，负责把每个 `U` 符号对上某处的定义。这里分两种库：
   - **静态库（`.a`）**：本质是一堆 `.o` 的打包。链接时把用到的 `.o` **原样复制进最终可执行文件**，符号解析在此刻完成、地址焊死。
   - **动态/共享库（`.so`）**：**不复制代码**。链接器只做两件事：确认符号确实存在于某个 `.so` 里，然后在可执行文件里记一条 `DT_NEEDED "libfoo.so.1"`（"我运行时需要这个库"），把真正的地址解析推迟到运行时。

   所以 `target_link_libraries(mujoco::mujoco)` 的效果不是"把 MuJoCo 塞进我的可执行文件"，而是"在我的 ELF 头里写一条 `DT_NEEDED libmujoco.so.3.3.7`"。
4. **运行（加载 + 动态链接）**：内核加载可执行文件，把控制权交给**动态链接器**（`ld-linux-x86-64.so.2`）。它做：
   - 按 `DT_NEEDED` 列表**递归**加载所有依赖的 `.so` 到进程地址空间；
   - 把所有还没解析的符号逐个对上地址（`RTLD_NOW` 语义）或推迟到第一次调用（惰性绑定，默认）；
   - 之后进程才真正开始跑 `main`。

**关键机制：全局符号作用域（global symbol scope）与 interposition**

动态链接器维护一个进程级的**全局符号查找列表**。它是**有序**的，顺序大致是：可执行文件本身 → 按 `DT_NEEDED` 广度优先顺序加载的各个 `.so`。

解析一个未定义符号时，动态链接器**从头往后扫，取第一个匹配的定义就停**。这条规则叫 **first-wins / symbol interposition（符号插入）**。

注意它的三个性质，都是本次事故的必要条件：

- **只按名字匹配**。不检查版本、不检查是哪个库导出的、不检查 ABI 是否兼容。名字一样就认。
- **是进程全局的，不是每个库私有的**。库 A 内部调用自己的函数，也走这张全局表——它并**不**天然优先绑定到自己身上。这一点最反直觉，也是坑的核心。
- **顺序敏感**。谁先进列表谁赢。`LD_PRELOAD` 就是故意利用这个机制（插到最前面）来替换 `malloc`、做 mock 的。所以 interposition 本身是**特性不是 bug**，只是这次被意外触发了。

到此为止的链路已经足够解释这次的崩溃了。

#### 7.2.2 这次为什么会冲突（本项目的实际情况）

**先纠正一个之前记错的结论。** 这一节原来写的是"MuJoCo 和 fastrtps **各自都内置打包了**一份 tinyxml2"。实际用 `nm`/`objdump` 查过之后，情况不是这样——是更不对称的一种：

```bash
# MuJoCo：静态链进了 tinyxml2，并且全部 default visibility 再导出
$ nm -D --defined-only libmujoco.so.3.3.7 | grep -c tinyxml2
250
$ objdump -p libmujoco.so.3.3.7 | grep NEEDED
  NEEDED  libm.so.6 / libstdc++.so.6 / libgcc_s.so.1 / libc.so.6 / ld-linux...
  # ← 没有 libtinyxml2，证明是静态链进去的

# fastrtps：自己一个 tinyxml2 符号都不定义，是动态依赖系统那份
$ nm -D --defined-only libfastrtps.so | grep -c ' _ZN8tinyxml2'
0
$ nm -D --undefined-only libfastrtps.so | grep -c tinyxml2
16
$ objdump -p libfastrtps.so | grep NEEDED
  NEEDED  libtinyxml2.so.9      # ← 明确要系统的那份
  ...
$ dpkg -l libtinyxml2-9
ii  libtinyxml2-9:amd64  9.0.0+dfsg-3
```

所以真实格局是：

| | tinyxml2 从哪来 | 导出 tinyxml2 符号？ |
|---|---|---|
| `libmujoco.so` | **静态链入**一份自己 patch 过的 | **导出 250 个**（default visibility） |
| `libfastrtps.so` | **动态依赖** `libtinyxml2.so.9` | 不导出，**import 16 个** |

**冲突的成立条件**：fastrtps 需要的那 16 个符号，MuJoCo 是不是也都导出了？逐个查，答案是 16/16 全中：

```
MUJOCO SYSLIB SYMBOL
1      1      _ZN8tinyxml211XMLDocument5ParseEPKcm
1      1      _ZN8tinyxml211XMLDocument8LoadFileEPKc
1      1      _ZN8tinyxml211XMLDocumentC1EbNS_10WhitespaceE
1      1      _ZN8tinyxml211XMLDocumentD1Ev
1      1      _ZNK8tinyxml210XMLElement7GetTextEv
...（共 16 行，两列全是 1）
```

**于是事故链条是**：

1. `target_link_libraries(mujoco::mujoco)` → 可执行文件的 `DT_NEEDED` 里有 `libmujoco.so`。
2. 进程启动，动态链接器按 `DT_NEEDED` 加载 MuJoCo，它那 250 个 tinyxml2 符号**进入全局符号作用域**，而且位置很靠前（可执行文件的直接依赖）。
3. `main` 开始跑，`rclcpp::init` / 构造第一个 `Node` 时，rclcpp 去 **`dlopen`** DDS 后端 `libfastrtps.so`（RMW 是运行时可插拔的，所以必然是 `dlopen` 而非 `DT_NEEDED`）。
4. 加载 fastrtps 时要解析它那 16 个 tinyxml2 未定义符号。动态链接器扫全局符号作用域，**MuJoCo 的定义排在 `libtinyxml2.so.9` 前面**（MuJoCo 早就在表里了），于是 16 个全部绑到了 MuJoCo 那份实现上。fastrtps 的 `DT_NEEDED libtinyxml2.so.9` 照样加载了，但它的定义在表里排后面，一个也没被用到。
5. fastrtps 开始解析 DDS 的 XML profile，它在**自己的栈上按系统 tinyxml2 9.0.0 的类布局**构造 `XMLDocument` 对象，然后把对象指针传给（它以为是系统的、实际是 MuJoCo 的）`XMLDocument::Parse`。
6. 两份 tinyxml2 **ABI 不兼容**。这是硬证据——同名类的方法签名都不一样：

   ```
   $ nm -DC libmujoco.so.3.3.7   | grep 'XMLDocument::Identify'
     tinyxml2::XMLDocument::Identify(char*, tinyxml2::XMLNode**, bool)   ← 3 个参数
   $ nm -DC libtinyxml2.so.9     | grep 'XMLDocument::Identify'
     tinyxml2::XMLDocument::Identify(char*, tinyxml2::XMLNode**)         ← 2 个参数

   $ nm -DC libmujoco.so.3.3.7   | grep 'XMLPrinter::XMLPrinter'
     tinyxml2::XMLPrinter::XMLPrinter(_IO_FILE*, bool, int, XMLPrinter::EscapeAposCharsInAttributes)
   $ nm -DC libtinyxml2.so.9     | grep 'XMLPrinter::XMLPrinter'
     tinyxml2::XMLPrinter::XMLPrinter(_IO_FILE*, bool, int)
   ```

   符号总数也不同（MuJoCo 223 vs 系统 229），符号集合互有出入。MuJoCo vendor 的是一份改过的、更新的 tinyxml2。**多出来的参数/成员意味着类的成员布局和虚函数表布局都变了**。
7. 于是变成"用 A 版本的方法去操作 B 版本布局的对象"：按错误的偏移量去取成员、按错位的 vtable 槽位去取虚函数地址，拿到一个垃圾值当指针写进去 → `error 14` 段错误。

**为什么崩在"构造 Node"而不是"调用 `mj_*`"**：因为崩溃跟 MuJoCo 的功能完全无关，纯粹是它的**存在**污染了符号表。触发点是 fastrtps 第一次用 tinyxml2 的时刻，也就是 Node 构造时；而 MuJoCo 只要被链接就已经完成了污染。这解释了最开始那个诡异现象——两个条件必须同时满足，而崩溃点在第三个地方。

一句话概括：**MuJoCo 静态链入并导出了一份改过的 tinyxml2，无意中给整个进程"LD_PRELOAD"了它，把 fastrtps 应该绑到系统 tinyxml2 的调用劫持走了。**

#### 7.2.3 这类冲突常见吗

**"两个库都静态打包同名第三方库"这种局面**在 C++ 生态里相当常见，因为静态打包依赖是发行预编译二进制的标准做法（避免用户装一堆依赖）。常见组合：`protobuf`、`abseil`、`zlib`、`libpng`、`jsoncpp`、`tinyxml2`、各种 BLAS。CUDA/TensorRT、OpenCV 的第三方发行版、游戏引擎 SDK 都是重灾区。

**但真正炸出来需要好几个条件同时满足**，所以日常遇到的频率远低于前者：同名符号 + 至少一方导出（没做 visibility 隐藏）+ 版本 ABI 不兼容 + 运行时真的走到那段代码。少任何一条就只是"侥幸没事"。这也是它难查的原因：换个机器、换个库版本、换个链接顺序，现象就变了。

ROS 生态特别容易撞，因为 RMW 是 `dlopen` 进来的——它**晚于**你的正常依赖入场，天然处于"被 interpose"的下风位置。反过来说，`rclcpp` + 任何静态打包了常见库的第三方 SDK，都值得先警惕一下。

#### 7.2.4 处理原理：dlopen 的三个 flag 各自关掉了什么

修复思路不是"改代码逻辑"，而是**不让 MuJoCo 进入全局符号作用域**——从根上拆掉 6.2.1 里那三个必要条件中的"进程全局 + 顺序靠前"。

做法：从 `DT_NEEDED` 里彻底去掉 MuJoCo（不再 `target_link_libraries`），改成运行时自己 `dlopen`，并显式指定作用域：

```cpp
void * handle = dlopen(kMujocoLibPath, RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
```

三个 flag 分别解决一件事：

- **`RTLD_LOCAL`（核心修复）**：这个库导出的符号**不加入全局符号作用域**，只对"通过这个 handle 做 `dlsym`"可见。于是 MuJoCo 那 250 个 tinyxml2 符号对 fastrtps 彻底不可见，fastrtps 的 16 个 import 正常绑到 `libtinyxml2.so.9`。这一条就够治本了。

  （对照：`RTLD_GLOBAL` 是把符号加进全局作用域，等价于普通链接的效果。`dlopen` 默认就是 `RTLD_LOCAL`，这里显式写出来是为了表达意图——这是**安全相关的设计决定，不能依赖默认值**。）
- **`RTLD_DEEPBIND`（反向保险）**：`RTLD_LOCAL` 挡住的是"别人看到 MuJoCo 的符号"，`RTLD_DEEPBIND` 挡的是**反方向**——MuJoCo 内部调用它自己的 tinyxml2 时，优先在自己的依赖树里查，而不是先扫全局作用域。没有它的话，如果进程里别处（比如某个 ROS 包）已经把系统 tinyxml2 用 `RTLD_GLOBAL` 放进全局表，MuJoCo 自己的 XML 解析反而会被劫持成系统那份，变成对称的另一半事故。

  代价要知道：`RTLD_DEEPBIND` 是 glibc 扩展（不可移植，musl/macOS 没有），而且会**一并**改变 `malloc`/`operator new` 等符号的绑定，理论上可能让库用上和主程序不同的分配器——如果跨边界传递需要 `free` 的内存就会出事。这里安全，因为 MuJoCo 的内存都由它自己的 `mj_makeData`/`mj_deleteData` 配对管理，不跨边界。
- **`RTLD_NOW`（不解决冲突，只改善可诊断性）**：加载时立刻解析库内所有符号，而不是惰性到第一次调用。让"符号缺失/版本不对"在 `dlopen` 当场失败并能报出原因，而不是跑到半小时后某个冷路径才崩。纯粹是可观测性选择。

**为什么 `dlopen` 能做到、普通链接做不到**：本质区别在于普通链接（`DT_NEEDED`）**没有地方表达"作用域"这个概念**——它只能说"我需要这个库"，符号一律进全局表。`dlopen` 是运行时 API，多了一个 flag 参数，这才有机会把符号可见性降级成 handle-local。所以从 `DT_NEEDED` 迁到 `dlopen`，换来的就是这个控制权。

代价是**符号不再由链接器自动对接，得手动接线**：不能再写 `mj_loadXML(...)`（编译期无从解析），必须 `dlsym` 拿地址存成函数指针。于是有了 `mujoco_dl.hpp`/`mujoco_dl.cpp`：一个 `MujocoApi` 结构体（每个字段一个函数指针），`loadMujocoApi()` 内部 `dlopen` + 一串 `dlsym`，返回全局唯一的 `MujocoApi&`（Meyer's Singleton）。业务代码统一走 `api_.loadXML(...)` / `api_.step(...)`，只多一层指针间接。（`dlopen`/`dlsym`/`resolve` 模板/单例这些**语法层面**的拆解见 [cpp_concepts.md](cpp_concepts.md)，本节只讲冲突原理。）

#### 7.2.5 处理模式唯一吗：六种方案对比

不唯一。按"改谁"分成三类：

| 方案 | 做法 | 为什么这次没选 |
|---|---|---|
| **① `dlopen` + `RTLD_LOCAL`**（采用） | 运行时隔离加载，`dlsym` 接线 | —— 唯一一个"不需要重编译任何第三方库、也不需要改 ROS"的方案 |
| ② 调整链接顺序 | 让 `libtinyxml2.so.9` 排在 MuJoCo 前面（如 `-ltinyxml2` 提前，或 `LD_PRELOAD=libtinyxml2.so.9`） | 治不了本：反过来变成系统 tinyxml2 劫持 MuJoCo 自己的 XML 解析（MuJoCo 那份多了参数，缺符号会直接崩）。而且依赖脆弱的顺序约定，`LD_PRELOAD` 还得污染运行环境 |
| ③ 从源码重编 MuJoCo，加 visibility 隐藏 | `-fvisibility=hidden` + 只显式导出 `mj_*`，或用 version script（`--version-script`）把 tinyxml2 符号标 `local` | 最干净的**根治**方案，符号压根不导出就无从冲突。但要自己维护一份 MuJoCo 构建，放弃 `/opt` 那份预编译库，升级成本高。**如果 MuJoCo 是自己项目的一部分，这才是正确解** |
| ④ 让 MuJoCo 静态链接（`.a`）进可执行文件 | 链接静态库而非 `.so` | 静态库的符号默认同样进全局表，不加 `--exclude-libs` / 局部化处理照样冲突。只是换个地方出问题 |
| ⑤ 换掉一方 | 换 DDS 后端（`rmw_cyclonedds` 不依赖 tinyxml2），或换不 vendor tinyxml2 的 MuJoCo 版本 | 能绕开，但把一个链接问题变成了运行时中间件选型约束——为了修 bug 去换 DDS 实现，代价和风险都不对等。可作为临时验证手段 |
| ⑥ 分进程 | MuJoCo 单独一个进程，和 ROS 侧走 IPC | **从设计上根除**：两个库不共享地址空间就不可能符号冲突。代价是 IPC 的复杂度和拷贝开销，详见 [7.5](#75-单进程-embed-vs-分进程--ipc为什么实现上差在哪) |

选 ① 的理由：在"不改第三方库、不改 ROS、不重构架构"的约束下，它是唯一可行的；代价（手动接线）是局部的、可控的、可被封装的。

值得记住的**通用判断框架**：符号冲突的修复手段，本质都是在动那三个必要条件之一——**要么让符号不导出**（③④）、**要么让符号不同名/不共存**（⑤）、**要么让符号不在同一个作用域**（①②⑥）。遇到新的符号冲突，照这三类去找解法就不会漏。

#### 7.2.6 当前方案的扩展性代价，以及怎么改善

**代价是实打实的**：每个新用到的 `mj_*` 函数要改三处——

1. `mujoco_dl.hpp` 的 `MujocoApi` 里加一个函数指针字段（**手抄一遍签名**）；
2. `mujoco_dl.cpp` 的 `loadMujocoApi()` 里加一行 `resolve(handle, "mj_xxx", api.xxx)`；
3. 调用点从 `mj_xxx(...)` 改成 `api_.xxx(...)`。

Stage B 加 `mj_id2name` 就走了一遍这个流程。这是 O(用到的 API 数量) 的持续摊销成本，而且有几个**真实的风险点**：

- **签名手抄错了编译器不会报错**。`reinterpret_cast` 是无条件的，`dlsym` 只按名字查、不校验类型。抄错参数个数或类型 → 编译通过、`dlsym` 成功、调用时栈错乱崩在别处。**这是当前方案最危险的地方**，比"多写一行"严重得多。缓解办法是从 MuJoCo 头文件直接拷签名，别凭记忆写。
- **拼错符号名要到运行时才发现**（`dlsym` 返回 `nullptr` → `resolve` 抛异常）。已经算好的情况：`RTLD_NOW` + 启动时一次性全部 `resolve`，所以是**启动即失败**而不是冷路径才崩。
- **函数指针字段没初始化**：`MujocoApi` 是裸结构体，漏了一行 `resolve` 的字段是未初始化的野指针，调用即崩。

**能怎么改善（按代价递增）**

| 手段 | 做法 | 收益 / 代价 |
|---|---|---|
| **宏 + X-macro 列表**（性价比最高） | 把 API 列表集中写成一张 `MJ_API_LIST(X)` 宏表，结构体字段和 `resolve` 调用都由宏展开生成 | 三处改动降到一处，且**结构体和 resolve 列表不可能不同步**（消灭"漏一行"和"字段未初始化"）。签名仍然要手抄 |
| `decltype` 推签名 | 字段写成 `decltype(&mj_step) step;`——头文件已经 `#include <mujoco/mujoco.h>`，声明就在手边 | **彻底消灭手抄签名的风险**，编译器保证类型和官方头文件一致。这是对上面那个"最危险的地方"的直接解药，应该优先做 |
| 结构体零初始化 + 断言 | `MujocoApi api{};` 让漏掉的字段是 `nullptr` 而非野指针，再在 `loadMujocoApi()` 末尾断言所有字段非空 | 把"漏一行"从随机崩溃变成启动期明确报错 |
| 代码生成 | 脚本解析 `mujoco.h` 自动生成整个 `mujoco_dl.*` | 零手工，但引入构建期依赖和生成代码，当前 API 用量（个位数）远不值得 |
| 换隔离层次 | 回到 6.2.5 的方案 ③（重编 MuJoCo 隐藏符号）或 ⑥（分进程） | 从根上不需要这层胶水，但成本高得多 |

**结论**：`decltype(&mj_xxx)` + X-macro 两个加起来大约 20 行改动，就能把当前方案的三处改动降到一处、并且让签名错误变成编译错误。目前 API 只有 8 个、手工维护还不痛，所以没做；但**如果 Stage C/D/E 之后 `MujocoApi` 涨到十几二十个字段，就该做这次重构**——触发条件写在这里，免得以后凭感觉拖。

> 补充定性（见 [7.5](#75-单进程-embed-vs-分进程--ipc为什么实现上差在哪)）：这属于纯 Linux 动态链接/ELF 层面的通用系统编程知识，跟机器人本身无关，任何 C++ 项目选型不当都可能碰到；作为系统编程经验值得记录，但不该占用"机器人相关深度"的追问预算。

**对已批准计划的影响**：计划里 Change 2 原本写的是"直接 `target_link_libraries(mujoco::mujoco)`"，现在改成"只用 `find_package(mujoco)` 拿头文件路径（`get_target_property(... INTERFACE_INCLUDE_DIRECTORIES)`），不链接库本体，库本体运行时 `dlopen`"。后续所有 Stage（B/C/D/E）用到的 `mj_*` 函数，都要先加进 `MujocoApi` 结构体、在 `loadMujocoApi()` 里 `dlsym` 解析，再通过 `api_.xxx` 调用，不能再假设可以直接调用裸的 `mj_*` 名字。

### 7.3 `main` 函数介绍

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

- **`rclcpp::init(argc, argv)`**：初始化整个进程级的 rclcpp 上下文，解析 ROS2 专用命令行参数（`--ros-args -r ...` 这类重映射），并启动底层 DDS（rmw/fastrtps）。这一步或紧接着的 Node 构造，正是 [7.2](#72-调试时踩到的段错误符号冲突与-dlopen-隔离) 那次段错误发生的地方——`libfastrtps.so` 在这里被 `dlopen` 进来。必须在创建任何 `rclcpp::Node` 之前调用，且整个进程只调用一次。
- **`rclcpp::spin(...)`**：先构造 `MujocoBridgeNode`（触发构造函数里的 MJCF 加载、`mj_makeData`、注册 500Hz 定时器），再把节点交给 ROS2 执行器（executor），进入一个**阻塞的事件循环**——不断检查有没有到期的定时器/新消息/服务请求，有就调用对应回调（目前只有 Stage A 注册的 `onTimer`）。会一直阻塞直到收到关闭信号（如 `Ctrl+C`/SIGINT），这也是为什么验证时要用 `timeout 4 ros2 run ...` 才能让它自动退出。
- **`rclcpp::shutdown()`**：`spin` 返回后做全局清理，释放 rclcpp 的进程级资源，跟 `init` 一一对应。

节点自己的资源清理（`mjData`/`mjModel` 释放）不在 `main` 里，而是在 `MujocoBridgeNode` 的析构函数里——`spin` 返回后，`make_shared` 创建的 `shared_ptr` 在 `main` 结束时自动析构触发清理，标准 RAII，不用手写释放代码。

### 7.4 `onTimer` 为什么是回调函数，注册过程，其它回调注册方式

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

### 7.5 单进程 embed vs 分进程 + IPC：为什么、实现上差在哪

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

**Stage A 选单进程是合理的，不是错误选择**——前提是我们自己拥有仿真器源码（MuJoCo 是个普通 C 库，可以直接 dlopen/链接进自己进程），且需要每个 timestep 高频读 `qpos`/`qvel`/`xpos`，单进程的零拷贝直接访问在这个前提下更合适，分进程反而是不必要的开销。[7.2](#72-调试时踩到的段错误符号冲突与-dlopen-隔离) 那次段错误（tinyxml2 符号冲突）正是"单进程 embed"模式特有的隐藏代价——分进程的方案从设计上就不可能出现"两个库在同一地址空间打架"的问题，因为它们压根不共享地址空间。这是为了拿到"零拷贝直接读 mjData"的好处，顺带买单的风险。

一句话：**是否分进程，本质是在"零延迟直接访问"和"故障隔离+进程独立生命周期+能对接不受自己控制的黑盒程序"之间做选择**。Stage A 现在这样做没问题，但要清楚这是一次有意识的取舍——面试被问"如果仿真器换成一个你没法链接的商业仿真器/真实机器人控制器，架构要怎么改"时，这条就是标准答案的骨架。

---

## 8. Stage B：`/clock` + `/joint_states`

### 8.0 一句话总结

让桥接节点开始真正"对外说话"：在 Stage A 的物理步进定时器里加了两个发布者——`/clock`（每个物理步发一次，把 `mjData::time` 换算成 ROS 时间戳，是整个系统 sim time 的唯一权威来源）和 `/joint_states`（按 100Hz 抽取发布，从 `mjData::qpos`/`qvel`/`qfrc_actuator` 里取 9 个关节的位置/速度/力矩）。关节列表不是写死的，而是启动时遍历 `mjModel` 自动推导出来的。

### 8.1 改动清单与验证结果

**改动**：

- [mujoco_dl.hpp](../../src/mujoco_bridge/include/mujoco_bridge/mujoco_dl.hpp) / [mujoco_dl.cpp](../../src/mujoco_bridge/src/mujoco_dl.cpp)：新增 `mj_id2name` 函数指针（遵守 [7.2](#72-调试时踩到的段错误符号冲突与-dlopen-隔离) 定下的硬约束：新用到的 `mj_*` 必须先进 `MujocoApi` 再 `resolve`）
- [package.xml](../../src/mujoco_bridge/package.xml) / [CMakeLists.txt](../../src/mujoco_bridge/CMakeLists.txt)：加 `rosgraph_msgs` 依赖
- [mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp)：新增 `buildJointIndex()`、`simTime()`、`publishJointState()`，两个 publisher，以及 decimation 计数

**验证结果**：

```
Loaded .../panda.xml (nq=9, timestep=0.0020s)
9 actuated joints: joint1..joint7, finger_joint1, finger_joint2
/joint_states every 5 steps (100.0 Hz requested, 100.0 Hz actual)
```

| 检查项 | 结果 |
|---|---|
| `ros2 topic list` | `/clock`、`/joint_states` 都在 |
| `ros2 topic hz /clock` | 499.97 Hz（= 1/0.002s，每个物理步一发） |
| `ros2 topic hz /joint_states` | 99.97 Hz（decimation = 5） |
| `header.stamp` | `3.300 / 3.310 / 3.320 / 3.330` —— 精确 10ms 间隔 |
| `name` 顺序 | 与 MJCF 声明顺序一致，且是从模型自动读出来的 |

**一个顺带发现**：启动后 `position` 不是全零（`joint4 = -0.0696`、`joint6 = +0.192`），且 `effort[joint6] = -12.0`，正好是该 actuator 的力上限，说明它饱和了。原因是启动时 `ctrl` 全零，position-servo 把目标设成 q=0（手臂竖直伸展的构型），但这个构型下重力矩太大、servo 增益顶不住，手臂就下垂了一点，一个关节直接打满力矩。这不是 bug，而是 Stage D 要用 `home` keyframe 做 reset 的动机之一——MJCF 里 `<key name="home">` 同时给了 `qpos` 和配套的 `ctrl`，是一个自洽的静止状态。

**踩到的小坑**：`stamp` 一开始输出成 `sec: 4, nanosec: 857999999`（本该是 4.858s）。`mjData::time` 是一个 double 累加值，`步数 × timestep` 会落在精确值下方一点点，`static_cast<int64_t>(t * 1e9)` 的截断就把它变成了 `...999999`。改成 `std::llround` 之后输出是精确的 10ms 倍数。差 1ns 本身无害，但下游如果做时间戳相等判断或者按整数周期对齐就会出问题。

### 8.2 sim time vs wall time，以及 `use_sim_time`

两个完全独立的时间轴：

- **wall time（墙钟时间）**：操作系统的真实时钟。`create_wall_timer` 用的是这个，它决定"多久调一次 `mj_step`"。
- **sim time（仿真时间）**：`步数 × timestep`，由 MuJoCo 累加在 `mjData::time` 里。它决定"物理世界过了多久"。

两者只是在这个项目里被刻意调成了 1:1（定时器周期 = timestep = 0.002s）。如果机器卡顿、某次回调晚了 5ms 才触发，sim time 依然只前进 0.002s，也不会"追赶"（这一点 [7.1](#71-stage-a-具体做了什么涉及哪些概念) 已经说过，Stage B 只是把它显式发布了出去）。

**`use_sim_time`** 是 ROS2 的节点级参数：设成 `true` 后，该节点的 `get_clock()->now()` 不再读系统时钟，而是返回它从 `/clock` 话题收到的最新值。这是整个系统"时间同步"的机制——所有下游节点（`robot_state_publisher`、RViz2、MoveIt）都要设 `use_sim_time:=true`，它们的 TF 缓存查询、超时判断、轨迹插值才会和仿真时间对齐。否则仿真跑得比实时慢 3 倍时，它们会按墙钟认为"所有数据都过期了"，表现为 TF extrapolation 错误。

**但这个节点自己不设 `use_sim_time`**，而且代码里刻意没有调用 `get_clock()->now()`，改用 `simTime()` 直接从 `data_->time` 换算。理由：这个节点**就是** `/clock` 的源头。它去读自己的时钟，`use_sim_time=false` 时拿到的是墙钟（错的时间轴），`use_sim_time=true` 时拿到的是它上一步刚发出去、绕了一圈 DDS 回来的值（多一跳延迟，还可能丢）。权威数据就在手边的 `data_->time` 里，没有理由绕道。

一条推论：**`/clock` 的发布者必须全局唯一**。多个源互相覆盖会让所有 `use_sim_time=true` 的节点看到时间倒流——现象和"代码算错了"长得一模一样，极难排查（见 [8.7](#87-排查记录环境不干净导致的两次误判)，这个坑当场就踩到了）。

### 8.3 `sensor_msgs/JointState` 字段布局

四个平行数组 + 一个 header：

| 字段 | 含义 | 这里填的值 |
|---|---|---|
| `header.stamp` | 这组测量对应的时刻 | sim time（`simTime()`） |
| `name[]` | 关节名 | `mj_id2name(model, mjOBJ_JOINT, i)` |
| `position[]` | 位置（revolute 用 rad，prismatic 用 m） | `qpos[jnt_qposadr[i]]` |
| `velocity[]` | 速度 | `qvel[jnt_dofadr[i]]` |
| `effort[]` | 力/力矩 | `qfrc_actuator[jnt_dofadr[i]]` |

三个关键点：

1. **靠下标对齐，靠名字解释**。`position[3]` 属于哪个关节，完全由 `name[3]` 决定——消息本身没有规定的关节顺序。所以任何消费方**必须按 name 查**，不能假设顺序。这是 [6.1](#61-fk-实践中可用的组件需要自己写的部分构型鲁棒写法) 里"构型鲁棒写法"原则在消息层面的体现。
2. **`velocity`/`effort` 允许留空**（长度 0）。真机上如果没有力矩传感器就发空数组，消费方需要处理这种情况。仿真里三个都能填满。
3. **`qpos` 索引 ≠ 关节索引**。`qpos` 是按自由度排布的：hinge/slide 各占 1 个，ball 占 4 个（四元数），free 占 7 个（位置+四元数）。所以必须用 `jnt_qposadr[i]` 查地址。而且 `qvel` 的索引（`jnt_dofadr`）和 `qpos` 的**不是同一套**——free joint 在 `qpos` 里占 7 个但在 `qvel` 里只占 6 个（角速度是 3 维而不是 4 维）。Panda 全是 hinge/slide，两套索引目前数值恰好相同，但写法上必须分开，否则以后往场景里扔一个带 free joint 的待抓物体就会静默错位。

选 `qfrc_actuator` 作为 `effort` 的理由：它是 actuator 实际施加的广义力，最接近真实机器人关节力矩传感器报的量。MuJoCo 里还有 `qfrc_applied`（外部施加力）、`qfrc_constraint`（约束反力）等，含义都不同。

### 8.4 关节列表为什么从模型推导而不是写死

`buildJointIndex()` 遍历 `model_->njnt`，只收 `mjJNT_HINGE` / `mjJNT_SLIDE`，跳过 free/ball，把 `{name, jnt_qposadr, jnt_dofadr}` 存成一张表。原始计划里写的是"按 MJCF 声明的关节顺序遍历 `joint1..7`、`finger_joint1/2`"，实现时改成了全自动推导——只多花一个 `mj_id2name` 符号，但换掉夹爪、换机型、往场景加物体都不用改这段代码。

跳过 free joint 是有意的语义选择，不是偷懒：待抓物体的 free joint 有 7 个 `qpos`，它没有"一个关节角"可言，它的状态属于 TF（Stage C 的 `world → object`），塞进 `JointState` 只会让下游困惑。

另外 `name` 数组在构造时填一次就不再改，`position`/`velocity`/`effort` 预先 `resize`，每次发布只覆写数字——避免 500Hz 下反复分配字符串。

### 8.5 权衡：发布频率和物理步进频率怎么解耦

Stage A 只有一个定时器，物理步进和（当时还不存在的）发布天然同频。Stage B 需要 `/joint_states` 比 500Hz 慢，有三种做法：

| 方案 | 做法 | 代价 |
|---|---|---|
| 每步都发 | 500Hz 发 `JointState` | 下游白白多处理 5 倍消息。`/clock` 保留这种方式，因为它需要细粒度 |
| **整数抽取（采用）** | 物理定时器里 `step_count_ % N == 0` 才发 | 发布频率只能是 `500/N`，取不到任意值 |
| 独立的第二个定时器 | 再 `create_wall_timer(10ms)` 单独负责发布 | 采样时刻和物理步不再对齐，`stamp` 和 `qpos` 的对应关系变模糊；多线程执行器下还要给 `mjData` 加锁；**而且挂在错误的时间轴上**（见下面第4点） |

选整数抽取的理由：**每个发布出去的样本都精确对应某一个物理步的完整状态**，`stamp = 步数 × timestep` 严格成立，没有插值、没有状态撕裂、不需要锁。

#### 8.5.1 为什么是 5 倍，decimation 到底管什么

> Q: 为什么选择 5 倍的 Hz 差异？`joint_state_decimation_` 只是用来调整定时器中的 publish 频率的吗？为什么需要区分 requested 和 actual？为什么 actual 是这样计算的？

**`joint_state_decimation_` 管什么：只管发布，完全不碰物理。** 它在 `onTimer` 里只出现在一个地方：

```cpp
void onTimer()
{
  api_.step(model_, data_);          // ← 物理：每次回调都走，跟 decimation 无关
  ++step_count_;
  clock_pub_->publish(clock_msg);    // ← /clock：每次回调都发

  if (step_count_ % joint_state_decimation_ == 0) {   // ← 只有这里用到
    publishJointState();
  }
}
```

就是一个**计数取模的闸门**。物理步进频率、`/clock` 频率都不受它影响；把它从 5 改成 50，仿真的物理行为**一模一样**，只是 `/joint_states` 变成 10Hz。所以它不是"调整定时器频率"——定时器周期始终是 `timestep`，动不了（要动的话就是改物理步长，那是完全不同的一件事，会改变数值积分的精度和稳定性）。

**为什么是 5（即 100Hz）**，两层理由：

1. **100Hz 够用，500Hz 浪费**。下游消费者的实际需求：`robot_state_publisher` 算 TF 给 RViz 看，人眼 30~60Hz 就够；MoveIt 做规划和执行监控，典型 50~100Hz；真机上 `ros2_control` 的常见状态发布频率也在 100Hz 量级（Franka 的 FCI 底层是 1kHz，但暴露给 ROS 侧的状态话题通常降频）。500Hz 的 `JointState` 意味着每条消息 9 个关节 × 3 个数组，下游要多做 5 倍的反序列化和回调，换不来任何可观测收益。
2. **5 是整数**。这是抽取方案的硬约束：`500 / 100 = 5` 正好整除。选 100Hz 而不是 120Hz 就是因为前者整除、后者不整除（500/120 = 4.17）。

**为什么要区分 requested 和 actual**

因为**抽取倍数必须是整数，所以用户要的频率不一定拿得到**。代码：

```cpp
const double joint_state_rate_hz = declare_parameter("joint_state_rate_hz", 100.0);
joint_state_decimation_ = std::max(
  1, static_cast<int>(std::lround(1.0 / (joint_state_rate_hz * timestep_s))));
RCLCPP_INFO(get_logger(), "/joint_states every %d steps (%.1f Hz requested, %.1f Hz actual)",
  joint_state_decimation_, joint_state_rate_hz,
  1.0 / (joint_state_decimation_ * timestep_s));
```

举例说明为什么必须把两个都打出来：

| 请求 | `1/(rate × timestep)` | 取整后 N | 实际频率 `1/(N × timestep)` |
|---|---|---|---|
| 100 Hz | 5.00 | 5 | **100.0 Hz** ✅ 正好 |
| 60 Hz | 8.33 | 8 | **62.5 Hz** ← 差了 4% |
| 30 Hz | 16.67 | 17 | **29.4 Hz** |
| 1000 Hz | 0.5 | 1（被 `max` 兜住） | **500 Hz** ← 请求超过物理频率，不可能满足 |

如果只打 requested，日志会**撒谎**——用户设了 60Hz，日志说 60Hz，实际是 62.5Hz，而这个 4% 的差异要等到别人用 `ros2 topic hz` 对不上、或者做时序分析时才发现。把两个都打出来，量化误差**在启动瞬间就可见**。这是"宁可难看也不要静默"的设计取向，和 [8.6](#86-失败模式与验证手段) 里那些失败模式的思路一致：**能在启动时暴露的问题，绝不留到运行时**。

**为什么 actual 这样算**

`actual = 1 / (N × timestep)`：选定整数 N 之后，两次发布之间真实间隔就是 N 个物理步 = `N × timestep` 秒，取倒数就是频率。

注意**没有**写成 `500.0 / N`。虽然这次数值相同，但 500 是从 `timestep = 0.002` 推出来的，硬编码它就等于假设 timestep 永远是 0.002s。MJCF 里改一行 `<option timestep="0.001">`，硬编码版本会静默报错误的频率，而 `1/(N × timestep)` 自动正确。同理 `std::max(1, ...)` 也不是装饰——请求频率高于物理频率时 `lround` 会得到 0，`% 0` 是除零，直接崩。

（`lround` 而非截断：60Hz 时 8.33 两种都得 8，但 55Hz 时 9.09 → `lround` 得 9（55.6Hz），截断也得 9；而 52Hz 时 9.6 → `lround` 得 10（50Hz，误差 4%），截断得 9（55.6Hz，误差 7%）。四舍五入总是取更接近的那个。）

#### 8.5.2 对齐到设备的任意 fps：动机和做法

> Q: 关于"对齐到实际设备的任意 fps"这块，我仍不理解它的动机和做法。

**动机：真实传感器的帧率不是我们能选的**

前面所有讨论都假设"想要的频率可以挑一个整除的"。但接 Stage F/第4周的 RGB-D 相机时，这个自由度消失了：

- RealSense D435 输出 30 fps 或 60 fps；
- 工业相机可能是 25 fps；
- 数据集回放的时间戳是别人录的。

**为什么必须是 30 而不能凑成 31.25**，三个具体场景：

1. **sim-to-real 对照实验**。第5周要做的事：同一套感知/抓取代码，先在仿真里跑出成功率，再上真机跑。如果仿真喂 31.25 fps、真机喂 30 fps，两边的结果差异里就混进了"帧率不同"这个额外变量。做对照实验最忌讳的就是多一个没控制住的变量——尤其当感知算法里有任何依赖帧间隔的成分（光流、跟踪、滤波器的 dt）时。
2. **多传感器时间同步的确定性**。相机 30Hz + 关节状态 100Hz，下游用 `message_filters::ApproximateTime` 做配对。如果相机帧落在 1/31.25 的网格上、关节状态落在 1/100 的网格上，两个网格的相对相位会**缓慢漂移**，配对结果时好时坏，出 bug 时极难复现。都落在各自的标称网格上，相位关系是固定的、可推算的。
3. **回放/录包的可比性**。仿真录的 bag 想和真机 bag 用同一套分析脚本，标称帧率对不上会让"每秒应该有 30 帧"这类断言全部失效。

**做法：把发布时刻挂在 sim time 上，累加阈值而不是数步数**

```cpp
// 构造函数
image_period_ = 1.0 / 30.0;              // 33.333... ms，不需要整除 timestep
next_image_time_ = image_period_;

// onTimer 里，mj_step 之后
if (data_->time >= next_image_time_) {
  publishImage();
  next_image_time_ += image_period_;     // 累加，不是 = data_->time + period
}
```

关键是**两个细节**：

- **判断依据是 `data_->time`（仿真时间），不是墙钟、也不是步数**。
- **`next_image_time_ += period` 而不是 `= data_->time + period`**。前者把发布时刻钉在绝对网格 `k/30` 上；后者会把每次的量化误差累加进去，**造成漂移**（每帧晚一点，一小时后差出好几帧）。这跟 [8.8](#88-墙钟定时器与-sim-time-的关系为什么不可能严格一致不一致会怎样) 里 rcl 定时器"下次 = 上次**预期** + 周期"是完全同一个技巧。

**效果和代价**，用 30fps / timestep=0.002s 举例（1/30 = 0.03333s = 16.67 步）：

| 帧 | 理想时刻 | 实际发布时刻（必须落在 2ms 步边界上） | 误差 |
|---|---|---|---|
| 1 | 0.03333 | 0.034（第 17 步） | +0.67ms |
| 2 | 0.06667 | 0.068（第 34 步） | +1.33ms |
| 3 | 0.10000 | 0.100（第 50 步） | 0 |
| 4 | 0.13333 | 0.134（第 67 步） | +0.67ms |

所以两种方案的取舍是清晰的：

| | 整数抽取 | sim-time 阈值累加 |
|---|---|---|
| 平均频率 | 量化到 `500/N` | **精确等于标称值** |
| 单帧抖动 | **零**（完全周期） | 最多一个 timestep（0~2ms） |
| 长期漂移 | 无 | 无（靠 `+=` 保证） |

**两个都想要是不可能的**，除非 timestep 正好整除目标周期。物理步长是最小时间粒度，发布只能发生在步边界上——这是离散仿真的固有限制，不是实现缺陷。实践上选哪个：**周期性比标称值重要就抽取（关节状态），标称值比周期性重要就阈值累加（传感器）**。2ms 的抖动对 30fps 相机来说是 6% 的帧间隔，通常无所谓，因为消息**带着精确的 `stamp`**，下游用时间戳而不是"第几帧"来做计算。

**为什么不能用第二个 wall timer 来做这件事**（前面表格里第3方案的真正致命之处）：墙钟定时器挂在**错误的时间轴**上。假设仿真只能跑到 0.5x 实时，一个 30Hz 的墙钟定时器每墙钟秒发 30 帧，而这一墙钟秒里只过了 0.5 仿真秒——于是**每仿真秒发了 60 帧**，相机在仿真世界里变成了 60fps。传感器帧率是仿真世界的物理属性，必须定义在仿真时间轴上。这一条在仿真跑不满实时的时候差别巨大，而跑不满实时恰恰是常态（[8.8](#88-墙钟定时器与-sim-time-的关系为什么不可能严格一致不一致会怎样)）。

（真要用定时器而非在物理循环里判断，正确做法是 `create_timer(get_clock(), period, cb)` 并设 `use_sim_time=true` —— 绑仿真时钟而不是墙钟。但那样又回到了"采样时刻和物理步不对齐"的问题，所以在物理循环里判断仍然更好。）

### 8.6 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现 / 怎么防住 |
|---|---|---|
| 关节名/顺序假设错 | 下游 FK 算出的末端位置离谱，但不报任何错 | 已用 name 查而非写死 index；验证时对比 `ros2 topic echo` 的 name 列表和 [architecture.md](../../docs/architecture.md) |
| `qpos` 和 `qvel` 索引混用 | Panda 上完全看不出来，加 free joint 物体后静默错位 | 已分开用 `jnt_qposadr` / `jnt_dofadr`（[6.2](#62-广义坐标qpos--qvel-的表示方法以及为什么维度不相等)） |
| double 截断 | `stamp` 是 `4.857999999` 而非 `4.858`，下游做时间相等判断时失败 | 改用 `std::llround`；验证输出是精确 10ms 倍数（[8.1](#81-改动清单与验证结果)） |
| 有第二个 `/clock` 发布者 | sim time 来回跳，TF 报 extrapolation 错误 | `ros2 topic info /clock` 确认 `Publisher count: 1`。**这条踩到了两次，见 [8.7](#87-排查记录环境不干净导致的两次误判)** |
| 请求频率不整除 / 超过物理频率 | 实际频率和预期差几个百分点，或 `% 0` 除零崩溃 | requested/actual 双打印 + `std::max(1, ...)`（[8.5.1](#851-为什么是-5-倍decimation-到底管什么)） |
| 定时器回调超时（`mj_step` 比 2ms 慢） | sim time 落后墙钟，但**不报任何错**，整个系统只是"慢放" | 实测 RTF，见 [8.8](#88-墙钟定时器与-sim-time-的关系为什么不可能严格一致不一致会怎样) |

最后一条是当前代码的真实缺口：**没有常驻的实时率（RTF）监控**。测的时候是临时打了个补丁，测完撤掉了。补上大约 6 行（`steady_clock` 记起点 + 周期性对比 `data_->time`），是 [8.8](#88-墙钟定时器与-sim-time-的关系为什么不可能严格一致不一致会怎样) 那类问题唯一的可观测手段，等哪次真的开始掉帧了会需要它。

### 8.7 排查记录：环境不干净导致的两次误判

同一个坑踩了两次，第二次还是栽在自己刚写下的"正确做法"上，所以完整记下来。

#### 第一次：`ros2 topic hz` 读到 1500Hz

**现象**：同一份代码，测出 `/clock` = 1500.2 Hz、`/joint_states` = 200.0 Hz，正好是预期值（500 / 100）的 3 倍和 2 倍，而且两个倍数还不一样。

**第一条线索**是 `min: 0.000s` —— 单一发布者的定时器抖动不可能产生间隔为 0 的相邻消息。这直接指向"多个发布者的消息几乎同时到达"，而不是"代码把频率算错了"。

**确认**：

```
$ ros2 topic info /clock
Publisher count: 3
$ ros2 topic info /joint_states
Publisher count: 2
$ ros2 topic info /mujoco_bridge/joint_states   # 改名前那版代码用的私有话题
Publisher count: 1
```

3 个 `/clock` 发布者 × 500Hz = 1500Hz，对上了。`/joint_states` 只有 2 个发布者，因为其中一个僵尸进程是**话题改名前**编译的那版二进制，它还在往旧的私有话题发——这解释了为什么两个倍数不一致。

**根因**：`ros2 run` 是一个 Python wrapper，它 `fork`/`exec` 出真正的 C++ 可执行文件作为**子进程**。用 `ros2 run ... &` 起节点、再 `kill` wrapper 的 PID，杀掉的只是 Python 那层，C++ 子进程被 init 收养后继续在后台跑。`timeout N ros2 run ...` 有同样的问题。

#### 第二次：RTF 读到 33x（以及"正确做法"其实不正确）

第一次之后，我在笔记里写下的清理方法是 `setsid ros2 run ... &` + `kill -- -$!`。后来测实时率时读到 **RTF = 33**（仿真比墙钟快 33 倍），显然荒谬——定时器周期 2ms，不可能跑出 33 倍。

**两个错误叠在一起**：

1. **`setsid ... &` + `kill -- -$!` 不可靠**。`setsid` 在自己已经是进程组 leader 时会先 `fork` 再让子进程开新会话，所以 `$!` 拿到的是 `setsid` 自己的 PID（它马上就退出了），**不是**新进程组的 PGID。`kill -- -$!` 打在一个不存在的进程组上，静默失败。结果又攒了 4 个孤儿。
2. **`ros2 topic echo --once` 在多发布者下抓到的是"随便哪一个"**。4 个实例启动时刻不同，各自的 `data_->time` 毫无关系。测量取的两个采样点来自不同实例，差值（385 秒）纯粹是两个实例启动时间差，和仿真速度毫无关系。

**真正可靠的做法**——绕开 wrapper，直接跑可执行文件，这样 `$!` 就是节点本身：

```bash
./install/mujoco_bridge/lib/mujoco_bridge/mujoco_bridge_node > /tmp/n.log 2>&1 &
PID=$!
sleep 30
kill $PID; wait $PID 2>/dev/null
```

**还有一个反复咬人的细节**：`pkill -f mujoco_bridge_node` 会**杀掉自己所在的 shell**——`-f` 匹配完整命令行，而这条命令自己的命令行里就含有这个字符串，自匹配。（`ps | grep` 的经典 `[d]` 括号技巧对 `pkill -f` 无效，因为命令行里别处还可能出现该字符串。）可靠写法是先取 PID 再按数字杀：

```bash
P=$(ps -eo pid,comm | awk '$2 ~ /^mujoco_bridge/ {print $1}')
[ -n "$P" ] && kill $P
```

#### 留下的经验

1. **`ros2 topic hz` 测的是话题上所有发布者的合计频率**，不是单个节点的发布频率。数字是预期值的整数倍时，第一反应应该是"多了几个发布者"。
2. **`min: 0.000s` 或异常大的 std dev 是多发布者的特征信号**。
3. **`echo --once` 在多发布者下没有意义**——它抓到哪个实例的数据是不确定的。任何"从话题上采两个点算差值"的测量，前提都是确认 `Publisher count: 1`。
4. **每次验证前先确认环境干净**。可靠的检查：`ps -eo pid,comm | awk '$2 ~ /^mujoco/'`，或 `ros2 node list`。`/clock` 尤其致命——它是 [8.2](#82-sim-time-vs-wall-time以及-use_sim_time) 说的全局唯一权威时间源。
5. **测量类的结论要先做量级合理性检查**。RTF = 33 本该在看到的第一秒就被否决（定时器 2ms，物理不可能快 33 倍）。荒谬的数字说明测量方法坏了，不是被测对象坏了——**先怀疑尺子，再怀疑物体**。
6. 这也说明 [8.6](#86-失败模式与验证手段) 那张失败模式表不是纸上练习：表里第四条（多 `/clock` 发布者）在写完表之后几分钟内就真实发生了，而且当时列的检查手段（`ros2 topic info` 看 publisher count）正是两次都定位它的手段。

### 8.8 墙钟定时器与 sim time 的关系：为什么不可能严格一致，不一致会怎样

> Q: 高级程序语言总是可能慢一点或者快一点，这里的定时器回调又和单片机的定时器中断不同，怎么可能总是保证墙钟和 sim time 的一致性？而且如果它们不一致，会出现什么问题——如果所有 node 都使用来自 MuJoCo api 的 clock 的话？

这个问题问到了整个设计的要点上。分三层答：**做不到严格一致；但有一个比"严格一致"弱得多、也够用得多的性质是能保证的；而且最后一问的答案是——大部分情况下什么问题都没有，这正是发布 `/clock` 的意义。**

#### 8.8.1 确实做不到，而且 rclcpp 的定时器比想象的更"软"

`rcl` 的头文件把这一点说得很直白：

```c
/* A timer simply holds state and does not automatically call callbacks.
 * It does not create any threads, register interrupts, or consume signals.
 * For blocking behavior it can be used in conjunction with a wait set and rcl_wait().
 * When rcl_timer_is_ready() returns true, the timer must still be called
 * explicitly using rcl_timer_call(). */
```

**和单片机定时器中断的区别，正是这段话**：

| | MCU 定时器中断 | rclcpp `create_wall_timer` |
|---|---|---|
| 触发机制 | 硬件计数器溢出 → **CPU 硬件中断**，强制打断当前代码 | 只是一个"到期时间"状态位；执行器在 `rcl_wait()` 里等，醒来后**主动查询**谁到期了 |
| 抢占性 | 有。正在跑的代码被打断 | **无**。上一个回调没返回，下一个不会开始 |
| 时延上界 | 确定的（几个时钟周期） | **没有上界**。取决于 Linux 调度、其他回调、页错误、CPU 竞争 |
| 谁调用回调 | 硬件向量表 | `spin()` 里的执行器，单线程顺序执行 |

所以一连串误差源都在：普通 `SCHED_OTHER` 优先级（不是实时调度）、单线程执行器里其他回调会阻塞它、`mj_step` 本身耗时波动（接触多的时候更慢）、`publish` 走 DDS 有不确定开销、内存缺页、缓存抖动。**任何一次回调都可能晚到，这是设计上就接受的。**

#### 8.8.2 能保证的性质：仿真时间自洽 + 无累积漂移

关键认识：**我们从来不需要「墙钟 == sim time」**。真正需要的只有两条，而这两条都是**构造上就成立**的：

1. **sim time 严格自洽、严格单调、步长精确**。`data_->time` 永远等于 `步数 × timestep`，不管每步之间墙钟过了 2ms 还是 200ms。物理积分拿到的 dt 永远是精确的 0.002s。**仿真世界内部的物理是完全正确的**，墙钟只决定"这个世界以多快的速度上演给你看"。
2. **所有节点共用同一个时间轴**。这就是 `/clock` + `use_sim_time` 的作用（[2.1](#21-rclcppnode-基类常用属性与方法)）。

墙钟定时器在这里的角色只是**节拍器（pacer）**——它决定 `mj_step` 被调用的节奏，不参与任何时间计算。

**实测：稳态下没有累积漂移**（临时加了 6 行诊断，跑 60 秒）：

```
   step      sim     wall  deficit  interval RTF
   5000   10.000   10.318    0.318
  10000   20.000   20.318    0.318      1.00000
  15000   30.000   30.318    0.318      1.00000
  20000   40.000   40.319    0.319      0.99990
  25000   50.000   50.319    0.319      1.00000
  30000   60.000   60.319    0.319      1.00000
```

两个读数方式的区别要看清：

- **累计 RTF**（`sim/wall`，从进程启动算）是 0.97 → 0.995，慢慢趋近 1。
- **区间 RTF**（相邻两行之间）是 **1.00000**。

那个 0.318s 的亏损**不累积**——它是**一次性**的启动开销（`mj_loadXML` 读 33MB mesh + DDS 初始化，发生在 `t0_` 记录之后、第一次定时器触发之前），之后 60 秒一步没再多欠。

**为什么不累积**（这是 `rcl` 定时器的一个重要设计）：下次到期时刻 = **上次预期时刻 + 周期**，而不是「现在 + 周期」。所以某次回调晚了 0.5ms，下次的截止时间不会跟着推迟 0.5ms——误差不复利。如果晚得超过一整个周期，定时器会立刻就绪、连续触发来补，直到追上。

（这和 [8.5.2](#852-对齐到设备的任意-fps动机和做法) 里 `next_image_time_ += period` 是**完全相同的技巧**，值得当成一条通用模式记住：**周期性任务的下次时刻要从「上次预期时刻」累加，绝不从「当前时刻」累加**，否则量化误差和调度抖动会攒成漂移。）

**什么时候会真正掉队**：如果 `mj_step` + `publish` 的平均耗时**超过** timestep，那就不是抖动而是能力不足，定时器永远追不上，亏损单调增长，RTF 稳定小于 1。这个模型现在离上限还远（RTF ≈ 1.0，说明单步远快于 2ms），但加了接触丰富的抓取场景、或者加上渲染 RGB-D 之后完全可能撞上。

**反过来也有个当前设计的限制**：墙钟定时器同时是**速度上限**——哪怕 `mj_step` 只要 60µs，也得等墙钟 2ms 才跑下一步，RTF 封顶在 1.0。想做"比实时快"的批量实验（RL 训练、大规模评测），必须**去掉墙钟定时器**，改成在一个紧循环里连续 `mj_step`，靠 `/clock` 驱动整个系统。那时 sim time 跑得比墙钟快好几倍，而系统依然正确——**这恰恰证明了"墙钟和 sim time 无需一致"**。

#### 8.8.3 不一致会怎样：多数情况下什么都不会

如果**所有**节点都 `use_sim_time:=true`，那么仿真跑 0.5x 或 5x，对系统来说都是透明的：

- 关节状态之间的间隔仍是精确 10ms（sim time）；
- 控制器算 dt 用消息时间戳，拿到的还是 10ms；
- TF 查询、轨迹插值、滤波器全部在 sim 时间轴上，一致。

**整个仿真世界只是相对你的手表被慢放或快放了，内部物理和时序关系完全不变。** 这就是问题最后一句的答案：如果所有节点都用 MuJoCo 来的 clock，不一致**不构成问题**。

真正出问题的是下面这些情况，**全部都是"有东西没在 sim 时间轴上"**：

| 场景 | 会怎样 | 为什么 |
|---|---|---|
| **某个节点漏设 `use_sim_time`** | TF extrapolation 错误、消息被判定为"来自未来/过期"、超时误触发 | 它按墙钟判断新鲜度，而数据戳的是 sim time。两个时间轴数值毫无关系，**这是最常见的实际故障** |
| 代码里直接用 `std::chrono::now()` / `create_wall_timer` 做控制逻辑 | 同上，而且更隐蔽——参数设对了也没用 | `use_sim_time` 只影响 `get_clock()`，管不了绕过它的墙钟调用。控制类周期任务应该用 `create_timer(get_clock(), ...)` |
| 硬件在环（真机、真相机接进来） | 真实设备按墙钟走，仿真按 sim time 走，**真实的失步** | 这时"墙钟 ≈ sim time"从"无所谓"变成了硬需求，必须保证 RTF ≈ 1 并监控它 |
| 人在环（遥操、手动示教） | 操作者的手按墙钟动 | 同上 |
| `ros2 topic hz` / `rqt` 等诊断工具 | 报出的频率是**墙钟频率**。RTF=0.5 时 `/joint_states` 显示 50Hz 而非 100Hz | 只是诊断困惑，不是系统故障。但排查时会误导——要记得 `hz` 的分母永远是墙钟 |
| 服务调用超时、`spin_until_future_complete(timeout)` | 多数超时参数走墙钟 | RTF 很低时，仿真里还没过去多久，墙钟超时就先到了 |

**结论性的一句话**：目标从来不是"墙钟和 sim time 一致"，而是**"所有参与者都在同一条时间轴上"**。`/clock` + `use_sim_time` 就是实现后者的机制；墙钟定时器只是让这条时间轴大致按人类可观看的速度推进的节拍器。只有当系统里存在**无法接受被慢放的真实世界组件**（真机、真传感器、人）时，RTF ≈ 1 才从"锦上添花"变成"硬约束"——那时就需要 [8.6](#86-失败模式与验证手段) 里提到的那个还没补上的 RTF 监控。

---

## 9. Stage C：TF（static + dynamic）

### 9.0 一句话总结

从 `mjModel`/`mjData` 自动推导出整棵 TF 树并发布：没有关节的 body（焊死的）走 `/tf_static` 发一次，有关节的 body 走 `/tf` 按 100Hz 重发。核心计算是把 MuJoCo 的**绝对**位姿（`xpos`/`xquat`，相对 world）换算成 TF 要的**父相对**变换。额外合成一条 MJCF 里不存在的 `hand -> hand_tcp`。时间戳继续走 `simTime()`，和 `/joint_states` 落在同一根时间轴上。

### 9.1 改动清单与验证结果

| 文件 | 改动 |
|---|---|
| [mujoco_dl.hpp](../../src/mujoco_bridge/include/mujoco_bridge/mujoco_dl.hpp) | `MujocoApi` 加三个字段：`negQuat`/`mulQuat`/`rotVecQuat` |
| [mujoco_dl.cpp](../../src/mujoco_bridge/src/mujoco_dl.cpp) | 对应三行 `resolve(handle, "mju_...", ...)` |
| [mujoco_bridge_node.cpp](../../src/mujoco_bridge/src/mujoco_bridge_node.cpp) | `buildFrameIndex()`、`makeTransform()`、`publishTransforms()`、`decimationFor()`（把 Stage B 的抽取逻辑提成复用函数）、两个 broadcaster、常量 `kHandToTcpZ` 等、新参数 `tf_rate_hz` |

`CMakeLists.txt` / `package.xml` **没动**——`tf2_ros`、`geometry_msgs` 早就在里面了。新增的 `mju_*` 按 [第10节](#10-后续计划与-session-交接) 那条硬约束走了 `MujocoApi` 结构体，没有直接链接 MuJoCo。

**启动日志**：

```
9 actuated joints: joint1..joint7, finger_joint1, finger_joint2
TF: 3 static, 9 dynamic frames
/joint_states every 5 steps (100.0 Hz requested, 100.0 Hz actual)
/tf every 5 steps (100.0 Hz requested, 100.0 Hz actual)
```

**实测表**（跑前已确认 `ros2 topic info /tf` → `Publisher count: 1`）：

| 验证项 | 命令 | 结果 |
|---|---|---|
| 话题存在 | `ros2 topic list` | `/tf`、`/tf_static` 都在 ✅ |
| 动态频率 | `ros2 topic hz /tf` | `99.98`，`std dev 0.0002s` ✅ |
| static 内容 | `ros2 topic echo /tf_static --once` | 3 条：`world->link0`(单位)、`link7->hand`(z=0.107, quat z=−0.3827)、`hand->hand_tcp`(z=0.1034) ✅ |
| TF 树形状 | `ros2 run tf2_tools view_frames` | 12 个 frame，每个恰好一个父，`world` 是唯一根 ✅ |
| 参数覆盖生效 | `-p tf_rate_hz:=40.0` | 日志 `every 13 steps (38.5 Hz actual)`，`hz` 实测 `38.462` ✅ |

**相对变换的数值验证**（最关键的一条）——此时 `ctrl=0`、机器人稳定在 `qpos≈0` 附近，所以每条父子变换应等于 MJCF 的 `body_pos`/`body_quat` 原值。而这些原值刚刚在 [4.3.3](#433-urdf--mjcf-交叉验证这是判断代码正误的主要手段) 里用**另一份独立的 URDF** 交叉验证过，所以这不是自证：

| `tf2_echo` | 实测 | MJCF 里写的 | |
|---|---|---|---|
| `link0 link1` | `[0, 0, 0.333]`，旋转单位 | `pos="0 0 0.333"` | ✅ |
| `link4 link5` | `[-0.082, 0.384, 0.000]`，RPY `[-90°, 0, 0]` | `pos="-0.0825 0.384 0" quat="1 -1 0 0"` | ✅ |
| `link5 link6` | `[0,0,0]`，RPY `[90°, -11.009°, 0]` | `quat="1 1 0 0"` ∘ rot_z(joint6) | ✅（`joint6=0.19214 rad = 11.009°`） |
| `world hand_tcp` | `[0.141, 0.000, 0.840]`，yaw `45.16°` | 整链 + 手的 −45° | ✅ 量级合理 |

这张表里**第二行是最强的一条测试**：`link4` 在 world 系里是歪的，如果 `rotVecQuat(delta, negQuat(parent_quat))` 那步的逆旋转写错，平移分量不可能恰好落回 `(-0.0825, 0.384, 0)`。**第三行验证的是四元数乘法顺序**（`parent_inv * child`），靠 `joint6=0.192` 是唯一一个偏离零位够多的关节撞上——这属于偶然覆盖，不是设计好的测试（见 [9.12](#912-失败模式与验证手段)）。

### 9.2 动机：TF 以后在哪些环节被用

> Q: Stage C 的动机。发布 TF，即父子关系之间的变换关系，在以后的什么环节会被使用。

按周排，全是具体用途：

| 何时 | 怎么用 |
|---|---|
| **第3周 FK/Jacobian** | TF 是**参考答案**。自己算出 `link0→hand_tcp`，和 `lookupTransform("link0","hand_tcp")` 比。没有 TF，FK 的单测没有东西可断言。**这同时是 [9.10](#910-权衡为什么不用-robot_state_publisher) 不能用 `robot_state_publisher` 的决定性理由** |
| **第4周 感知** | 相机给的点云在 `camera_optical_frame` 里，规划要的是 `world`/`link0` 里的物体位姿。这个换算链条**就是 TF**；`tf2_ros::Buffer::transform()` 对 `PointCloud2`/`PoseStamped` 一行搞定 |
| **第5周 抓取** | 抓取位姿是**相对物体**生成的，要变成"`hand_tcp` 在机器人基座系里的目标"。又是一条 TF 链 |
| **第6周 MoveIt** | MoveIt 的 planning scene 建立在 TF 上；`world→link0` 告诉它机器人装在哪；规划轨迹要用当前状态做起点 |
| **RViz（Stage E 起）** | RViz 里**一切东西的位置都由 TF 决定**。发一个 `frame_id: hand_tcp` 的 Marker，它自动出现在正确位置，不用自己算 |
| **随时调试** | `tf2_echo` / `view_frames` 是**最便宜的跨模块健全性检查**，[9.1](#91-改动清单与验证结果) 的验证表就是它 |

再往深一层，两个更本质的动机：

**① TF 是全项目的共享坐标词汇表。** 没有它，任意两个模块之间要**双边约定**一个变换、各自硬编码一份——这正是 [3.1](#31-为什么必须先约定-frame关键规则有哪些) 规则1（相机外参只允许发一份、禁止感知节点手写第二套）要禁的东西。有了 TF，每条边有唯一权威发布者，其他人按**名字 + 时间戳**查。

**② TF 在时间上解耦。** 感知节点处理一帧图像花了 200ms，它可以问"`hand_tcp` 在**那个时间戳**时在哪"，tf2 会在缓冲区里插值给出答案。这是为什么每条 TF 必须带正确的 stamp，也是为什么 **Stage B 的 sim time 一致性是 Stage C 的前置条件**（[8.2](#82-sim-time-vs-wall-time以及-use_sim_time)）——如果 TF 用墙钟、JointState 用 sim time，这个查询就落在两个时间轴上，插值结果没有意义。

### 9.3 为什么必须是两个 broadcaster

不是 API 洁癖，是**两个不同的话题、不同的 QoS、不同的语义**：

| | `/tf` | `/tf_static` |
|---|---|---|
| 发布者类 | `tf2_ros::TransformBroadcaster` | `tf2_ros::StaticTransformBroadcaster` |
| Durability | volatile | **transient_local（latched）** |
| 发布次数 | 每周期都要重发 | 启动时发一次 |
| tf2 怎么用时间戳 | 按 stamp 插值，超出缓冲区（默认 10s）就查不到 | **完全忽略 stamp，视为对任意时刻都有效** |

两条关键推论：

1. **static 只发一次也不会丢**。`transient_local` 让 DDS 替我们缓存最后一条样本，5 分钟后才启动的 RViz 依然收得到。这也是为什么 `view_frames` 里那三条的 `rate` 显示成 `10000.000`、`buffer_length: 0.000`——它只见过一个样本，那个数没有意义。
2. **static 的 frame 不占缓冲区、不会过期**。如果把 `world->link0` 塞进 `/tf`，任何一次 `lookupTransform` 只要时间戳落在缓冲窗口外就**整条链**查不到；放进 `/tf_static` 就没这个问题。

### 9.4 static / dynamic 怎么划分：用结构而不是名字

判据只有一行：

```cpp
if (model_->body_jntnum[i] == 0) { /* static */ } else { /* dynamic */ }
```

**一个没有任何关节的 body 是焊在父 body 上的**，那么它相对父的变换永远等于 MJCF 里写的 `body_pos`/`body_quat`——这两个数组本来就是"父相对"表示，直接搬走即可。有至少一个关节的 body 相对父会动，必须每周期从 `xpos`/`xquat` 现算。

这个判据的好处是**零硬编码名字**，延续了 [8.4](#84-关节列表为什么从模型推导而不是写死) 的路子：第4周往场景里丢一个带 free joint 的待抓物体，它的父是 `world`、`body_jntnum=1`，会自动出现在 dynamic 列表里变成 `world -> object`，代码一行不用改。

还有一个不明显的收益：**用 `body_pos` 而不是 `xpos` 算 static，就不需要在构造函数里调 `mj_forward`**。`mj_makeData` 之后 `xpos` 还是全 0（要走一遍前向运动学才会填），如果从 `xpos` 推 static 变换，构造期发出去的三条 static TF 全是零——而且因为 static 是 latched、只发一次，**这个错会永久生效、不会被后面任何一帧纠正**。（Stage D 的 reset 之后要不要补 `mj_forward`，是同一个坑的另一面。）

**这条判据的已知反例：mocap body。** MuJoCo 的 mocap body 没有任何关节，但位姿由用户每步直接写 `mjData::mocap_pos`/`mocap_quat`——它会动。现在的代码会把它判成 static、发一次就再也不更新。正确判据应该是 `body_jntnum[i] == 0 && body_mocapid[i] < 0`。当前 MJCF 里没有 mocap body，所以这是"写着的假设暂时成立"，进 [11.1](#111-清单) 的知识边界条目。

### 9.5 从 `xpos`/`xquat` 到父相对变换：推导与代码对应

`mjData::xpos[3*i]` / `xquat[4*i]` 是 body `i` 在 **world 系**里的位姿。TF 的每条边是"child 在 parent 系里的位姿"。父 id 从 `model_->body_parentid[i]` 拿——MJCF 的 body 嵌套结构在编译期就被压成了这个整型数组。

用 [6.3.3](#633-位姿不只是旋转怎么复合) 的两条规则推。已知 $T_{W \leftarrow C}$（child 的 `xpos`/`xquat`）和 $T_{W \leftarrow P}$（parent 的），要求：

$$T_{P \leftarrow C} = T_{W \leftarrow P}^{-1} \circ T_{W \leftarrow C} = T_{P \leftarrow W} \circ T_{W \leftarrow C}$$

（下标约掉 $W$。）代入复合与求逆规则：

$$q_{P \leftarrow C} = q_P^* \otimes q_C \qquad\qquad \mathbf{p}_{P \leftarrow C} = R(q_P^*)\,(\mathbf{p}_C - \mathbf{p}_P)$$

逐字对应代码：

```cpp
api_.negQuat(parent_quat_inv, parent_quat);           // q_P*
delta = child_pos - parent_pos;                        // (p_C − p_P)，注意还在 world 系里
api_.rotVecQuat(rel_pos,  delta, parent_quat_inv);    // R(q_P*)·delta  → 转进 parent 系
api_.mulQuat  (rel_quat, parent_quat_inv, child_quat); // q_P* ⊗ q_C
```

两个必须说清的点：

- **`delta` 先减、后转**。相减得到的是"从父原点指向子原点的向量"，但它的**分量还是用 world 的坐标轴表达的**；`rotVecQuat` 那一步才把它换成用 parent 的坐标轴表达。顺序反过来（先转再减）是错的。
- **`mulQuat(rel, parent_inv, child)` 不能交换**。按 [6.3.2](#632-三个运算) 的下标约法：$q_{P \leftarrow W} \otimes q_{W \leftarrow C} = q_{P \leftarrow C}$ ✓；写成 `(child, parent_inv)` 得到 $q_{C \leftarrow W} \otimes q_{W \leftarrow P}$，那是**反向**的变换。

**如果偷懒直接发绝对位姿会怎样**：对父是 `world` 的 body（这里只有 `link0`）恰好正确，其余全错。而且错得很隐蔽——RViz 里会看到一个形状离谱但仍然"连着"的机器人，不会有任何报错。这就是 [9.1](#91-改动清单与验证结果) 那张数值表存在的理由。

**所以每周期发出去的到底是什么**：9 条 dynamic 边，每条是 7 个数 +1 个时间戳：

```
header.frame_id  = "link4"        ← 参考系
child_frame_id   = "link5"        ← 被描述的系
translation      = (x, y, z)      ← link5 的原点，在 link4 系里的坐标
rotation         = (x, y, z, w)   ← link5 的三根坐标轴，相对 link4 的朝向
header.stamp     = sim time       ← 这组数在什么时刻成立
```

一句话：**"在时刻 $t$，child 的原点在 parent 系里的这个位置，且 child 的坐标轴相对 parent 是这个朝向。"** tf2 收到一堆这样的边拼成树，然后能回答任意两个 frame 之间、任意时刻的变换。

另外是**一条消息装所有变换**，不是每个 frame 发一条：`/tf` 的类型是 `tf2_msgs/TFMessage`（一个 `transforms` 数组），正是为了让发布者能在单个样本里送出一个**一致的快照**。

### 9.6 四元数分量顺序：一次讲错的更正

> Q: 你在 `makeTransform` 前面标注 quat 在 mujoco 和 geo msgs 里的顺序不一致性，但是你下面的 tf 的 rotation 分量用的都是结构体成员表示，这有啥顺序在？我认为会出错的应该是 memcpy 写法才对。

**这条追问是对的，我原来的注释和讲解把风险放错了侧。** 留档原文：我最初写的是"`quat` is MuJoCo order (w,x,y,z); geometry_msgs is (x,y,z,w)"，读起来像是在说这段赋值代码有顺序风险。

实际上：

- **输出侧（`geometry_msgs`）没有顺序**。`tf.transform.rotation` 是四个**具名字段** `.x/.y/.z/.w`，写 `rotation.w = ...` 编译器认字段名，不可能"顺序写混"。
- **输入侧（MuJoCo）才有顺序**。`quat` 是一个裸 `mjtNum[4]`，`quat[0]` 是 **w**。读的人必须知道这件事，否则会以为 `quat[0]` 是 x。

所以那段注释真正的作用是**告诉读者怎么读那个裸数组**，而**正是"逐字段具名赋值"这个写法让顺序错误不可能发生**。真会出错的确实是 `memcpy` / `std::copy` 那类整块搬四个 double 的写法——那时 MuJoCo 的 w 会落到 ROS 的 x 上，而且**结果模长仍是 1**，tf2 不会报 `invalid quaternion`，只会安静地把机器人摆成一个错姿态。

源码注释已按这个版本改掉（[STUDY_NOTES_GUIDE.md 2.3 的约定](../../STUDY_NOTES_GUIDE.md)：笔记改结论，代码注释要一起改）。

### 9.7 为什么先建索引，而不在 publish 里直接迭代 `model_`

> Q: 为什么我们选择先构建 JointIndex，而不是在 publish 的时候直接读取 `model_` 里的地址数据（我知道关节角的数值会变，但是地址应该不会变化吧）。对于 FrameIndex 也是如此，后面的 publish 函数里面也只是用了 id 来找地址而已，为什么不直接迭代 `model_`。

**前提完全正确**：`jnt_qposadr` 这些地址在 `mj_loadXML` 之后就固定了，publish 里每次读 `model_->jnt_qposadr[i]` 拿到的是同一个数。所以**这不是为了缓存会变的东西**。真正的理由，按重要性排：

**① `name` 字段必须和数值字段严格同序——这是个不变量，最好在结构上保证。**

`JointState` 是四个平行数组（[8.3](#83-sensor_msgsjointstate-字段布局)），第 `i` 项必须描述同一个关节。如果 publish 里现场迭代 `njnt`、现场 `push_back(name)`，"筛选逻辑"就出现在**两个地方**（建名字时、填数值时），将来任何人改动其中一处，两个数组就错位——而且**错位后消息完全合法**，`ros2 topic echo` 看着一切正常，只是 `joint4` 的角度被标成了 `joint5`。现在的写法让 `name` 只在构造期写一次、publish 只能改数值，这类错位**在结构上不可能发生**。

**② 筛选和取名字是真正的工作，不是"取地址"。** publish 路径要做的不只是索引：`jnt_type` 的过滤（`njnt` 里可能有 free/ball 关节）、`mj_id2name` 返回 `const char *` 塞进 `std::string` 要**堆分配 + 拷贝**。9 个关节 × 100Hz = 每秒 900 次本可避免的 string 构造；TF 那边是 9 frame × 2 个名字 = 每秒 1800 次。

**③ Fail fast + 日志只打一次。** `throw std::runtime_error("no hinge/slide joints found")` 放构造期意味着模型不对时节点**启动就死**；放 publish 里就变成"节点跑起来了，一直发空消息"。同理 `RCLCPP_WARN("body id %d has no name")`：构造期打一条，publish 期就是每秒 100 条刷屏。

**④ vector 只 resize 一次。** `position.resize(9)` 构造期做完，publish 是纯覆写，无 realloc。

**对 FrameIndex 额外还有两条，而且这两条不是优化、是逻辑上必须：**

**⑤ static/dynamic 的划分是一个"决策"，不是一次查询。** static 变换**根本没有 publish 循环**——构造期算完、发一次、靠 DDS 的 latch 活着（[9.3](#93-为什么必须是两个-broadcaster)）。所以 `buildFrameIndex` 不是"把 publish 的活提前做了"，**它是那三条 static TF 唯一存在的地方**。如果 publish 里迭代 `nbody`，还得每周期把 static 那些跳过去（又一次重复筛选逻辑），而且得另外找地方处理 static。

**⑥ 父子关系要配对使用。** publish 里每个 frame 需要 `body_id` **和** `parent_id` 两个 id（算 $T_P^{-1}T_C$），以及对应的两个名字。把这四样打成一个 `DynamicFrame` 存起来，比每周期做 `parent = body_parentid[i]; parent_name = id2name(...)` 更难写错。

**诚实的反面**：对 9 个 body / 100Hz 这个规模，②④ 的性能论证**完全不重要**（几微秒）。真正立得住的是 ①③⑤⑥——**不变量、失败时机、和"static 只能在这里做"**。多出的一次间接（`joints_[i].qpos_adr` vs `model_->jnt_qposadr[i]`）是真实代价。

### 9.8 frame tree 必须是树

tf2 要求每个 frame **恰好一个父**，且整体连通无环。我们天然满足：MJCF 的 body 层级本身就是树，`body_parentid` 是一个到父的单射，一条 body 一条边。`view_frames` 的输出（12 frame / 11 条边 / 只有 `world` 从不作为 child 出现）就是这个性质的直接体现。

真会破坏它的是**"第二个发布者也发同一条边"**——比如 Stage E 起了 `robot_state_publisher`（[9.10](#910-权衡为什么不用-robot_state_publisher)），它也要发 `link0->link1`，那条边就有两个说法在打架，tf2 会按到达顺序反复覆盖，表现为**机器人抖动/跳变**而不是报错。[3.1](#31-为什么必须先约定-frame关键规则有哪些) 里"相机外参只允许在 `mujoco_bridge` 发一份"是同一个规则。

### 9.9 命名：我们发 MJCF 的原生名

这是 Stage C 开工前定的一个决策（三个选项：全按 MJCF / 全对齐 `architecture.md` / 只补 `link8`），选了**MJCF 原生 + 仅合成 `hand_tcp`**：

- 基座是 **`link0`**，不是 `base_link`。
- **没有 `link8`**。MJCF 直接写了 `link7 -> hand`（`pos="0 0 0.107"` + 绕 z −45°），正好是 URDF 侧 `link7->link8->hand` 两步的合并（[4.3.3](#433-urdf--mjcf-交叉验证这是判断代码正误的主要手段) 验证过是同一个变换，拆开不引入误差）。
- 指尖是 **`left_finger`/`right_finger`**（下划线），URDF 侧是 `leftfinger`/`rightfinger`。
- `hand_tcp` 是唯一的合成例外（[4.4](#44-hand_tcp-是什么以及-01034-这个-magic-number)）。

**理由**：代码里零硬编码名字，全从模型推导（延续 [9.4](#94-static--dynamic-怎么划分用结构而不是名字) 的判据）。**代价**是 `architecture.md` 第1节的 frame 表和实现对不上（写了 `base_link` 和 `link8`）——已经回头改掉了。按 [STUDY_NOTES_GUIDE.md](../../STUDY_NOTES_GUIDE.md) 第1节，`architecture.md` 是权威来源，不能留着和代码不一致。

另两个选项的代价是**代码里要出现 2~3 处硬编码的 name 特判**（合成一个单位变换的 `link0->base_link` 别名、把 `link7->hand` 人为拆成两步），通用推导循环被打断。

### 9.10 权衡：为什么不用 `robot_state_publisher`

这是 Stage C 最重要的替代方案，而且是 ROS 圈里的**标准做法**。在**今天这个模型上确实"两者都行"**（[4.3.3](#433-urdf--mjcf-交叉验证这是判断代码正误的主要手段) 刚逐行验证了两份描述几何一致），但"都行"是这个特定模型的巧合，不是这两个方案的性质。

**两者回答的是不同的问题：**

| | `robot_state_publisher` | 我们（Stage C） |
|---|---|---|
| TF 的来源 | $\text{TF} = \text{FK}(\text{URDF},\ /joint\_states)$ | $\text{TF} = $ MuJoCo 的 `xpos`/`xquat` |
| 回答的问题 | "**给定这些关节角和这个运动学模型，各 link 应该在哪？**" | "**仿真里各 body 实际在哪？**" |
| 性质 | **派生量**（用第二个模型算出来的） | **原始量**（物理引擎的状态本身） |

四个它们会分叉的地方：

**① 第3周的 FK 测试会变成循环论证（决定性的一条）。** 第3周要写"我的 FK 实现 vs 仿真真值"的 gtest。如果 TF 本身是 `robot_state_publisher` 用 URDF 做一遍 FK 得到的，那就是在**拿自己的 FK 和别人的 FK 比**——两边都对不上真值也能互相对上，测试毫无意义。TF 必须来自物理引擎，测试才有参照。

**② 两份模型的一致性没有任何东西在守。** [4.3.3](#433-urdf--mjcf-交叉验证这是判断代码正误的主要手段) 那张表是**手工跑出来的、没进 CI**。任何人动一下 vendor 的 `panda.xml`（比如把机器人装到桌子上，改 `link0` 的 `pos`），`robot_state_publisher` 会**继续按旧 URDF 报告**，TF 和物理静静地分叉，没有报错。用 `xpos` 不存在这个接缝——按定义一致。

**③ `robot_state_publisher` 表达不了 MJCF 里的一大类东西。** 它的输入只有 `/joint_states`，所以只能摆放"位姿是关节角的函数"的 link。摆不了：第4周要丢进场景的待抓物体（free joint，位姿不是任何关节角的函数）、mocap body、第二个机器人、任何没写进 URDF 的东西。这些都得**另开一个发布者**——于是同时维护两套机制，还得小心别发同一条边（[9.8](#98-frame-tree-必须是树)）。我们的 `body_jntnum` 判据对这些是零改动自动支持。

**④ 命名会从 MJCF 切到 URDF。** URDF 侧是 `fer_link0`、`fer_leftfinger`、有 `fer_link8`、根是 `base`；MJCF 侧是 `link0`、`left_finger`、无 `link8`、根是 `world`。用 rsp 意味着 TF 里的名字**不再是我们用来索引 `mjData` 的 body 名**，任何"从 TF frame 反查仿真 body"的代码都要过一张翻译表。

**一条论断更正（留档）**：我最初讲这条权衡时写的是"代价是 RViz 里只能看 TF 坐标轴、看不到机器人网格"。**不准确。** `robot_state_publisher` 实际上捆了两件事：(1) 发布 `/robot_description`（URDF 字符串，transient_local）；(2) 发布 FK 推出来的 TF。RViz 的 `RobotModel` 显示只需要 **(1)** 拿网格、再**按 link 名去 TF 里查位置**——所以要看到网格需要的是 (1)，**不需要 (2)**。

于是第6周接 MoveIt 时（MoveIt 无论如何都要 `/robot_description`）的正确做法是：**让 URDF 进图，但不让 rsp 发 TF**（remap 掉它的 `/tf`、`/tf_static`，或自己发那个 String）。职责就干净了：**URDF 负责"长什么样和怎么规划"，MuJoCo 负责"现在在哪"**。这条已记进 [第10节](#10-后续计划与-session-交接) 第6周待决项。

### 9.11 权衡：`tf_rate_hz` 独立于 `joint_state_rate_hz`

把 Stage B 的抽取逻辑（[8.5](#85-权衡发布频率和物理步进频率怎么解耦)）提成了 `decimationFor()`，然后给 `/tf` 开了独立参数（默认同样 100Hz）。理由是两者的下游消费者不同——`/tf` 给 RViz 和坐标变换查询，`/joint_states` 给控制器和 MoveIt，需求可以不一样。

**但默认值故意设成相同**：一旦不同，下游"拿 `/joint_states` 的 stamp 去 `lookupTransform`"就会命中 tf2 的插值/外推路径，得到一个两个采样点之间的数**而不是任何真实的仿真状态**。

诚实地说，这是我**自己引进来的一个可配置不一致源**，而且设成不同值不会有任何告警。属于 C 类问题，进 [11.2](#112-反向清单现在就该做的属于缺一次推演)（备选解：构造期加一条 `RCLCPP_WARN`，或者干脆共用一个参数）。

另外 [2.5](#25-declare_parameterros-参数到底是什么为什么不用-const-double) 记录的那个缺陷同样适用于这两个参数：**运行时 `ros2 param set` 会报成功但无效果**。

### 9.12 失败模式与验证手段

| 失败模式 | 现象 | 怎么发现 / 防住 | 这个 stage 验过？ |
|---|---|---|---|
| 发绝对位姿而非父相对 | RViz 里机器人形状离谱但"连着"，无报错 | `tf2_echo` 在 `q≈0` 时比对 MJCF 的 `body_pos`（[9.1](#91-改动清单与验证结果)） | ✅ 验过 |
| 四元数分量顺序写混（memcpy 类写法） | 姿态乱但模长合法，tf2 **不报** `invalid quaternion` | 逐字段具名赋值从结构上避开；数值上看 RPY 是否等于 MJCF 的 `quat` | ✅ 结构上避开 + 数值验过 |
| `mulQuat` 参数顺序反 | **仅在关节角非零处**偏差 | 需要一个明显偏离零位的关节 | ⚠️ 靠 `joint6=0.192` **偶然**覆盖 |
| static 从 `xpos` 算而非 `body_pos` | 三条 static 全零，且因 latched **永不纠正** | 用 `body_pos` 从根上避开；或构造期调 `mj_forward` | ✅ 结构上避开 |
| 双发布者发同一条边 | 机器人抖动/跳变，无报错 | `ros2 topic info /tf --verbose` 看 publisher count | ✅ 本 stage 只有一个 |
| TF 树断成两棵 | `lookupTransform` 抛 `ConnectivityException` | `view_frames` 看根节点数 | ✅ 验过 |
| 未命名 body | 它的**子 body 会被静默重挂到错误的父上** | 代码里 `RCLCPP_WARN` + `continue` | ⚠️ 只有日志，没测过 |
| mocap body 被判成 static | 位姿永不更新，无报错 | 判据应加 `body_mocapid[i] < 0`（[9.4](#94-static--dynamic-怎么划分用结构而不是名字)） | ❌ 当前模型无 mocap body |
| `/tf` 和 `/joint_states` 频率不同 | 下游静默走 tf2 插值路径 | 无任何告警（[9.11](#911-权衡tf_rate_hz-独立于-joint_state_rate_hz)） | ❌ 未防 |
| `0.1034` 上游漂移 | TCP 位置错 103.4mm 量级，无报错 | 具名常量 + 注释里的核对命令（[4.4.2](#442-为什么这个数没有自动共享)） | ⚠️ 只靠人工核对 |

**反查一遍**（按 [STUDY_NOTES_GUIDE.md](../../STUDY_NOTES_GUIDE.md) 5.1 的要求）：这张表里有 3 条 ⚠️ 和 3 条 ❌。**整套相对变换数学的验证全是人眼看 `tf2_echo`，没有一条自动化断言**，这是 E 类的空缺，见 [11.2](#112-反向清单现在就该做的属于缺一次推演)。

### 9.13 排查记录：第三次遗留进程（同一个坑的第三次）

**现象**：验证 `tf_rate_hz` 参数覆盖时，日志明明写着 `38.5 Hz actual`，但 `ros2 topic hz /tf` 读到：

```
average rate: 138.539
	min: 0.000s max: 0.010s std dev: 0.00330s
```

而且 `ros2 param get /mujoco_bridge tf_rate_hz` 返回 `100.0`——和刚传进去的 `40.0` 不符。

**线索**：`138.5 = 100 + 38.5`。按 [3.2 的规则](../../STUDY_NOTES_GUIDE.md)（荒谬的数字先怀疑尺子），这个和式立刻指向两个发布者。`min: 0.000s` 和偏大的 std dev 也是多发布者的特征信号。

**根因**：启动前的检查打印了 `STALE: 277928`——上一轮验证的节点没清掉。`param get` 打到的也是那个旧节点（同名 `/mujoco_bridge`，服务名冲突，谁应答不确定）。

**修复**：按数字 PID 循环清理，确认 `ros2 topic info /tf` → `Publisher count: 1` 后重测，得到 `38.462` ✅。

**留下的经验（新增两条，前两次见 [8.7](#87-排查记录环境不干净导致的两次误判)）**：

1. **`ros2 param get` 在多实例下和 `topic echo --once` 一样不可信**。同名节点的参数服务会冲突，读到的是哪个实例不确定。此前 8.7 的结论只覆盖了话题，**服务/参数也适用同一条规则**。
2. **`ps -eo pid,args | grep 'lib/mujoco_bridg[e]'` 会匹配到自己的命令行**——这次清理后它报了 `DIRTY`，其实是假警报（grep 匹配到了那条 bash `-c` 的完整命令字符串）。这是 [3.1](../../STUDY_NOTES_GUIDE.md) 里 `pkill -f` 自匹配那个坑的 grep 版本。**用 `ps -eo pid,comm`（只匹配可执行文件名）更可靠**：
   ```bash
   ps -eo pid,comm | awk '$2 ~ /^mujoco_bridge/ {print $1}'
   ```

---

## 10. 后续计划与 Session 交接

Stage A（[第7节](#7-stage-amujoco_bridge_node-最小实现模型加载--物理步进定时器)）、Stage B（[第8节](#8-stage-bclock--joint_states)）、Stage C（[第9节](#9-stage-ctfstatic--dynamic)）均已完成、build+run 验证通过、讲解已记录。以下是 Stage D-E 的简要计划，供下一个 session 接着做（原始完整计划见 `.claude/plans` 下已批准的计划文件，这里只摘录关键点方便快速回忆上下文）。

**执行方式约定**（沿用 Stage A 的模式，不要跳过）：每个 stage 写完代码后先 `colcon build` + 实际跑一遍验证，验证通过再讲解涉及的概念，讲解完追问/确认理解后再写进本文件，最后才进入下一个 stage。每个 stage 建议除了"这段代码怎么工作"之外，再补至少一条**权衡/替代方案对比**和一条**失败模式/怎么验证**类问题（这是第 4 轮反思后定下的规矩，避免只深挖单段代码语法）。

**验证环境卫生（Stage B 踩坑后补的）**：每次跑验证前先 `ros2 node list` 确认没有遗留节点；起后台节点用 `setsid ros2 run ... &` + `kill -- -$PGID`（或 `pkill -f mujoco_bridge_node`），**不要** `kill` `ros2 run` 的 PID——那只杀 Python wrapper，C++ 子进程会变成孤儿继续发话题，导致 `ros2 topic hz` 读到成倍的频率。完整排查记录见 [8.7](#87-排查记录环境不干净导致的两次误判)。

**硬约束（Stage A 踩坑后定下，后续都要遵守）**：不能再 `target_link_libraries(mujoco::mujoco)` 直接链接 MuJoCo（会和 ROS2 的 fastrtps 因为 tinyxml2 符号冲突段错误，见 [7.2](#72-调试时踩到的段错误符号冲突与-dlopen-隔离)）。任何新用到的 `mj_*` 函数，都要先加进 `mujoco_bridge/include/mujoco_bridge/mujoco_dl.hpp` 里的 `MujocoApi` 结构体字段，再到 `mujoco_dl.cpp` 的 `loadMujocoApi()` 里加一行 `resolve(handle, "mj_xxx", api.xxx)`，业务代码统一通过 `api_.xxx(...)` 调用。

### Stage B — `/clock` + `/joint_states` ✅ 已完成

详见第7节。相对原计划的三处偏离，后续 stage 需要知道：

1. **关节列表改成从 `mjModel` 全自动推导**（遍历 `njnt` + `mj_id2name`，只收 hinge/slide），不再是按名字查 `joint1..7`/`finger_joint1/2` 的固定列表。新增了 `mj_id2name` 到 `MujocoApi`。见 [8.4](#84-关节列表为什么从模型推导而不是写死)。
2. **话题名是全局 `/joint_states`，不是 `~/joint_states`**——`robot_state_publisher` 和 RViz2 都默认订阅全局名，Stage C/E 接它们时不用 remap。
3. **`/joint_states` 用整数抽取降到 100Hz**（参数 `joint_state_rate_hz`，默认 100.0），`/clock` 仍然每个物理步发一次。见 [8.5](#85-权衡发布频率和物理步进频率怎么解耦)。

另外时间戳统一走 `simTime()`（`std::llround(data_->time * 1e9)`），**不要**用 `get_clock()->now()`——理由见 [8.2](#82-sim-time-vs-wall-time以及-use_sim_time)。Stage C 的 TF 时间戳也必须用它，否则 TF 和 JointState 会落在两个时间轴上。

### Stage C — TF（static + dynamic）✅ 已完成

详见[第9节](#9-stage-ctfstatic--dynamic)。相对原计划的三处偏离，后续 stage 需要知道：

1. **frame 名全部用 MJCF 原生名**（[9.9](#99-命名我们发-mjcf-的原生名)）：基座是 `link0` 不是 `base_link`，**没有 `link8`**（MJCF 把 `link7->link8->hand` 合并成一步）。原计划写的 `world -> base_link` 和 `link8` 都不存在。`docs/architecture.md` 第1节的 frame 表已回头改正。
2. **static/dynamic 的划分是 `body_jntnum[i] == 0`，不是按名字列举**（[9.4](#94-static--dynamic-怎么划分用结构而不是名字)）。第4周加 free joint 物体会自动变成 `world -> object`，零代码改动。**已知反例：mocap body**（判据该加 `body_mocapid[i] < 0`）。
3. **新增参数 `tf_rate_hz`**（默认 100.0，故意和 `joint_state_rate_hz` 相同，见 [9.11](#911-权衡tf_rate_hz-独立于-joint_state_rate_hz)）。`MujocoApi` 新增了 `mju_negQuat`/`mju_mulQuat`/`mju_rotVecQuat` 三个字段（现在共 11 个，[7.2.6](#726-当前方案的扩展性代价以及怎么改善) 说的"涨到十几二十个就该上 `decltype`"的触发点在接近）。

### Stage D — reset service

- 新增依赖：无（`std_srvs` 已在）。
- `std_srvs::srv::Trigger` 类型的 `~/reset` service，回调里调 `mj_resetDataKeyframe(model_, data_, keyframe_id)`。`keyframe_id` 要在构造函数里用 `mj_name2id(model_, mjOBJ_KEY, "home")` 按名字查一次并缓存，不要硬编码成 `0`。
- 需要讲解的概念：service（请求/响应）vs topic（发布/订阅）的适用场景区别、MJCF 里 keyframe 是什么（预定义的一组 `qpos` 值）、reset 之后要不要额外调 `mj_forward`（把 `xpos`/`xquat` 等派生量刷新，不然发出去的第一帧 TF/JointState 可能还是旧值——这是个值得验证的失败模式）。
- **Stage C 补充的一条**：`mj_forward` 这个问题现在更值得验证了，因为 TF 直接读 `xpos`/`xquat`。[9.4](#94-static--dynamic-怎么划分用结构而不是名字) 已经踩到它的另一面（构造期 `xpos` 全 0）。注意 `keyframe` 里除了 `qpos` 还有 `ctrl="0 0 0 -1.57079 0 1.57079 -0.7853 255"`（8 个数，对应 `nu=8`）——reset 时**要不要一起恢复 `ctrl`** 是个必须定的问题：只 reset `qpos` 而 `ctrl` 留着旧目标，机器人会立刻被伺服拉回旧位姿。

### Stage E — 命令订阅 + sine 测试脚本 + demo launch/rviz

- 新增依赖：`control_msgs`/`trajectory_msgs` 已在 CMakeLists 里。
- 订阅 `~/joint_command`（`trajectory_msgs::msg::JointTrajectory`），收到后把第一个 trajectory point 的 `positions` 按 joint name 写进 `mjData::ctrl`（MJCF 里的 position-servo actuator 会自己把 `ctrl` 值当目标位置去伺服，不是直接改 `qpos`）。
- **⚠️ Stage C 读 `<actuator>` 段时挖出三颗地雷，动手前必读 [4.3.4](#434-每个关节的状态变量应该是什么样以及一个-stage-e-地雷)**：(a) `ctrl` 长度是 `nu`=8 不是 `nq`=9，索引是 **actuator id 不是 joint id**，臂关节碰巧对齐、夹爪会错位；(b) `actuator8`（夹爪）的 `ctrlrange` 被上游重映射成 **`0..255`**，不是 `0..0.04`，直接写关节角进去会几乎全闭；(c) 两指由一个 tendon + 一个 equality 约束耦合，**物理上不能独立指令**。
- **RViz 的 `RobotModel` 显示需要 `/robot_description`**，不只是 TF（[9.10](#910-权衡为什么不用-robot_state_publisher) 更正过一次的论断）。`demo.rviz` 先只放 TF + Grid 的计划不变，但原因不是"做不到"，而是这个 stage 不想引入 URDF。
- 新建 `src/mujoco_bridge/launch/demo.launch.py`（起 `mujoco_bridge_node` + `rviz2`）和 `src/mujoco_bridge/rviz/demo.rviz`（先只放 TF + Grid，不接 RobotModel），`CMakeLists.txt` 加 `install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})`。
- 新建 `scripts/sine_joint_test.py`（rclpy 脚本），发一个缓慢正弦振荡的 `JointTrajectory` 给 `~/joint_command`，人工在 RViz 里确认对应关节的 TF 帧在动。
- 需要讲解的概念：position-servo actuator 的本质是什么（是一种简化的 PD 控制器，MJCF 里怎么定义增益）、`ctrl` 写入和真实机器人控制器接口的对应关系、launch 文件基础语法。

### 验收标准（沿用已批准计划里的定义）

1. `ros2 launch mujoco_bridge demo.launch.py` 后确认 `/clock`、`/joint_states`、`/tf`、`/tf_static` 都在发布（`ros2 topic list` / `ros2 topic hz /joint_states`），RViz 里 TF 树非爆炸、根节点是 `world`。← 话题部分 Stage C 已单独验过（[9.1](#91-改动清单与验证结果)），剩 launch 文件本身。
2. `ros2 service call /mujoco_bridge/reset std_srvs/srv/Trigger {}` 后 `/joint_states` 数值应该跳回 home pose。
3. 跑 `scripts/sine_joint_test.py`，RViz 里对应关节的 frame 应该能看到周期性摆动。

### 第6周接 MoveIt 时的待决项（Stage C 识别出来的）

MoveIt 无论如何都需要 `/robot_description`（URDF），而 `robot_state_publisher` **同时**发 URDF 和 TF。届时必须显式决定**谁发 `link0->link7` 这几条边**，否则出现 [9.8](#98-frame-tree-必须是树) 说的双发布者打架（表现为机器人抖动，不报错）。

倾向的做法：**让 URDF 进图，但 remap 掉 rsp 的 `/tf`、`/tf_static`**（或干脆自己发那个 String）。职责划分是 **URDF 负责"长什么样和怎么规划"，MuJoCo 负责"现在在哪"**。理由见 [9.10](#910-权衡为什么不用-robot_state_publisher)。同时要处理 URDF 侧的 `fer_` 前缀和 `leftfinger`/`left_finger` 命名差异。

这个决定值得在第6周补一份 `docs/adr/`。
---

## 11. 悬挂问题（等有了参照系再回来）

这一节专门存放**现在还答不了、或者答了也存不住的问题**。

### 11.0 为什么要单开这一节

> Q: 作为初学者，在缺少横向对比和真机经验的情况下，很难问出设计决策的问题。如果认为"还无法提问，说明还不是现在这个阶段应该解决的"，这个思路对吗？

**对一半，而且那一半很重要。** 但要先把"无法提问"拆成两种成因，因为它们的处理方式相反：

| 成因 | 例子 | 怎么办 |
|---|---|---|
| **缺少参照系** | timestep 凭什么取 0.002；真机力矩传感器读数和 `qfrc_actuator` 可不可比；这个架构在真实项目里合不合理 | **推迟，而且理直气壮**。现在问，拿到的是一条别人给的结论 + 无法验证的理由，只会存成教条 |
| **缺少一次推演** | 这段代码怎么测试；`effort` 填 `qfrc_actuator` 和填 `qfrc_constraint` 差在哪；`buildJointIndex` 遇到没名字的 body 会怎样 | **现在就做**。答案全在手边的代码、头文件、或跑一次实验里 |

判断方法很粗暴但够用：**这个问题的答案，有没有可能在我手边的代码、头文件、和一次实验里？** 是 → 现在问；否 → 进这份清单。

**支持"推迟"的一个理由**（比"问不出来"更积极）：同样的知识，来得太早反而更难扎根。现在问"timestep 该取多少"，只能听结论；等第4周真的加了接触丰富的抓取场景、真的看到穿透或抖动，那时同一个问题的答案会长在自己的观察上。

**但判据用得太宽就危险**：第二类（缺一次推演）如果也被归进"以后再说"，就没人会替你做了——那恰恰是现阶段唯一能自己走完的部分。

**一个不需要经验也能用的替代动作**：设计决策问题的门槛在于需要横向对比，但有个低成本替身——

> 不要问"这样设计对吗"（需要参照系），改问**"如果这里写错了 / 被改了，我怎么知道？"**（只需要顺着眼前的代码推演一步）。

这个问法不需要你先有答案或标准，而且产出和设计决策问题高度重叠：**一个"改了没人发现"的地方，几乎总是一个"设计上没交代清楚"的地方**。

回头看，Stage B 这轮最好的两个追问已经是这个形状了：[8.8](#88-墙钟定时器与-sim-time-的关系为什么不可能严格一致不一致会怎样) 的后半句"如果它们不一致会出现什么问题"就是它；[2.1](#21-rclcppnode-基类常用属性与方法) 的"我没看到 `use_sim_time` 定义在哪"是它的变体。所以这个动作不是新技能，是把已经在用的东西显式化、可复用化。

**Stage C 的补充观察**：这一轮最有产出的追问是另一个形状——**"我不知道怎么检查这个东西"**（[4.3](#43-怎么检查和分析-mjcf--urdf以及怎么据此判断代码正误) 那问）。它既不是 A 类（不是"这是什么"）也不是 C 类，而是在问**工具和方法**。产出：一条 URDF↔MJCF 交叉验证流程、`architecture.md` 的两处错误、以及 Stage E 的三颗地雷（[4.3.4](#434-每个关节的状态变量应该是什么样以及一个-stage-e-地雷)）。值得记下来的是——**这三颗地雷全部是在"学怎么读文件"的过程中顺手捡到的，不是在找 bug 时找到的**。另外 [9.6](#96-四元数分量顺序一次讲错的更正) 和 [9.10](#910-权衡为什么不用-robot_state_publisher) 各纠正了讲解里的一处错误论断，都是质疑论断（[4.4 提问方式](../../STUDY_NOTES_GUIDE.md)）问出来的。

### 11.1 清单

格式：问题 / 为什么现在答不了 / 什么时候回来。第 1~4 条是 Stage B 识别的，第 5~6 条是 Stage C 新增的。

| # | 问题 | 现在答不了的原因 | 解锁条件 |
|---|---|---|---|
| 1 | **物理步进凭什么放在 ROS 定时器回调里？** 桥接节点该不该同时拥有物理循环？ | 需要见过别的架构（Gazebo 的 `gzserver` 分进程、Isaac 的 Python 主循环、`ros2_control` 的 `update()`）才能比较。[7.5](#75-单进程-embed-vs-分进程--ipc为什么实现上差在哪) 已经讲了单进程 vs 分进程的取舍，但"物理循环归谁所有"是更上一层的问题 | 第6周接 MoveIt 之后；或任何一次读别人的 sim bridge 源码 |
| 2 | **timestep = 0.002 凭什么？** 谁定的、和控制频率的关系、什么时候必须调小 | 现在场景里只有机器人本体、没有接触，改 timestep 看不出任何差别，无从验证任何结论 | 第4周加入待抓物体和桌面接触之后。那时可以实验：调大 timestep 直到出现穿透/抖动 |
| 3 | **`effort` 填 `qfrc_actuator` 对吗？** 它和真机关节力矩传感器的读数可比吗？ | 需要真机 `franka_ros2` 的 `effort` 字段实际数值做对照。仿真里无法自证 | 上真机之后；或找到 `franka_ros2` 里 effort 来源的文档/源码 |
| 4 | **100Hz 的 `/joint_states` 够吗？** 下游 MoveIt / 控制器的真实需求是多少 | [8.5.1](#851-为什么是-5-倍decimation-到底管什么) 里列的 50~100Hz 是我查来的经验值，不是自己测出来的 | 第6周 MoveIt 实际跑起来，可以实验：降到 20Hz 看规划执行是否退化 |
| 5 | **TF 由仿真直接发、而不用 `robot_state_publisher`，在真实项目里是常规做法还是异类？** | [9.10](#910-权衡为什么不用-robot_state_publisher) 的四条理由都立得住，但"别人怎么做"需要参照系。Gazebo 的 `gazebo_ros2_control`、Isaac Sim 的 ROS bridge 各自怎么处理这个职责划分，我没读过 | 第6周接 MoveIt 时必须面对（见[第10节](#10-后续计划与-session-交接)待决项）；或任何一次读别人的 sim bridge 源码。和本清单第 1 条是同一类问题 |
| 6 | **`body_jntnum == 0 → static` 这个判据的完整例外集合有多大？** | 已知 mocap body 是一个反例（[9.4](#94-static--dynamic-怎么划分用结构而不是名字)）。但"MuJoCo 里还有哪些让 body 动起来的机制不经过 joint"需要对 MuJoCo 更全面的了解，现在只能逐个撞上 | 第4~5周真用到 mocap（遥操作/给定末端目标）时；届时至少要把判据改成 `body_jntnum == 0 && body_mocapid < 0`。这条也是 F 类（知识边界）的典型 |

### 11.2 反向清单：现在就该做的（属于"缺一次推演"）

这些不进悬挂清单，是待办。前四条 Stage B 留下，后五条 Stage C 新增：

- [ ] **`jnt_qposadr` / `jnt_dofadr` 写混了没有任何东西会告警**——Panda 上两者数值相同，测不出来；等第4周加 free joint 物体才炸。该写一个用**带 free joint 的最小 MJCF** 做的单元测试，现在就能写，不需要任何新知识（对应 [6.2](#62-广义坐标qpos--qvel-的表示方法以及为什么维度不相等) 第1条影响）。
- [ ] **`MujocoApi` 的签名手抄错了编译器不会报错**——解药是 `decltype(&mj_xxx)`，约 20 行改动，触发条件已写在 [7.2.6](#726-当前方案的扩展性代价以及怎么改善)。
- [ ] **没有常驻 RTF 监控**——约 6 行，见 [8.6](#86-失败模式与验证手段) 末尾。
- [ ] **Stage B 的验证全是人眼看 `echo`/`hz`，没有一条自动化断言**。第3周要写 FK/Jacobian 的 gtest，届时"这段桥接代码怎么单元测试"会变成主要矛盾（难点：`mjModel` 加载依赖文件路径、`rclcpp::Node` 构造依赖 DDS）。
- [ ] **`publishTransforms` 的相对变换数学没有任何自动化断言**（E 类，Stage C）。[9.1](#91-改动清单与验证结果) 全是人眼看 `tf2_echo`，而且恰好挑在 `q≈0` 附近；`mulQuat` 参数顺序只靠 `joint6=0.192` **偶然**覆盖到（[9.12](#912-失败模式与验证手段)）。这段其实是整个节点里**最容易写成纯函数单测**的一块——给一组 `xpos`/`xquat` 和父子关系，断言输出的相对变换。难点和上一条（第3周 FK gtest）**完全重叠**，一起写更划算。注意断言四元数要比 $|q_1 \cdot q_2| \approx 1$，不能逐字段比（[6.3.1](#631-一个单位四元数就是一个旋转) 的 double cover）。
- [ ] **`mj_forward` 在构造期/reset 后的必要性**（Stage C 识别，Stage D 就会撞上）。[9.4](#94-static--dynamic-怎么划分用结构而不是名字) 靠改用 `body_pos` 绕开了构造期的 `xpos` 全 0，但 Stage D 的 reset 之后没有东西绕得开。属于"跑一次实验就能答"。
- [ ] **`tf_rate_hz` 和 `joint_state_rate_hz` 设成不同值没有任何告警**（C 类，[9.11](#911-权衡tf_rate_hz-独立于-joint_state_rate_hz)）。这是 Stage C 自己引进来的不一致源。约 3 行：构造期比一下，不等就 `RCLCPP_WARN`。
- [ ] **`ros2 param set` 报成功但无效果**（C 类，[2.5](#25-declare_parameterros-参数到底是什么为什么不用-const-double)）。触发条件写明：等真的需要在跑动中调频率时，用 `add_on_set_parameters_callback` 重算 decimation。现在不做的理由是"当前够用"，不是"不该做"。
- [ ] **把 `0.1034` 搬进 vendor 的 MJCF**（`<site name="hand_tcp" pos="0 0 0.1034"/>`），代码改成从模型读（[4.4.2](#442-为什么这个数没有自动共享)）。顺手能消掉 `kHandBodyName` 那个硬编码字符串。这是个改 vendor 文件的独立决定，值得配一份 `docs/adr/`。

### 11.3 维护约定

- 每个 stage 结束时过一遍：新攒的问题按 11.0 的判据分流——进 11.1（等参照系）还是 11.2（现在就做）。
- **解锁条件满足时要回头看**。第4周加物体、第6周接 MoveIt、上真机，这三个节点各自对应上表里几行，到时候主动回来答，别等它自然遗忘。
- 答掉的问题从清单里删掉，答案写进对应的 stage 小节（不要堆在这一节里，这节只放"未答"）。

