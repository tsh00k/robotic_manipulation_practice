# ADR 007：Stage Q 使用 image_geometry 与 PCL 完成几何感知

## Status

已采用（2026-09-29）。

## Context

`mujoco_bridge_node` 已同时承担 MuJoCo 物理步进、ROS 状态发布、RGB-D 渲染、reset 和 debug viewer。Stage Q 还需要深度反投影、桌面去除、聚类、刚体配准、姿态质量评估和生命周期同步。把这些算法继续放进 bridge 会让视觉处理阻塞物理线程，并使算法单测依赖 MuJoCo/OpenGL。

本项目的学习重点是观测契约和任务接入，不是重新实现点云库。容器已提供 PCL 1.12、`image_geometry`、`pcl_conversions`、`message_filters` 和 tf2。

## Decision

新增独立的 `mujoco_perception` 包。`mujoco_bridge` 保留仿真传感器源和原有 oracle 接口；感知节点只订阅标准 RGB-D、CameraInfo、静态 TF 和 `BridgeObservation`，按完全相等的仿真时间戳组成一帧。

算法组件如下：

- `image_geometry::PinholeCameraModel`：深度像素反投影；
- PCL `PassThrough`：world ROI（Region of Interest，感兴趣区域）和已知桌面高度的退化过滤；
- PCL `SACSegmentation`：水平支持平面；
- PCL `EuclideanClusterExtraction`：目标聚类；
- PCL `TransformationEstimationSVD`：已知对应点刚体变换；
- PCL `MomentOfInertiaEstimation`：已知盒体的 OBB（Oriented Bounding Box，有向包围盒）基线。

保留 SVD 是为了给未来的模型点集配准提供成熟的刚体变换求解器，避免项目维护重复的 Kabsch 实现。它要求上游已经建立正确的逐点对应关系，并且不能自行处理外点、遮挡、匹配和对称性；因此当前俯视立方体的默认运行路径仍使用 OBB，SVD 作为经过退化检查的可复用入口，而不是当前单帧估计的主算法。

项目代码仅负责参数、TF/时间戳、generation、置信度和拒绝码。输出使用新的 `VisionObjectPose` 消息，不能覆盖 ground-truth topic。

俯视 RGB-D 只能稳定观察盒子的顶面。因此在已知桌面和盒体尺寸时，姿态估计只把两个可见主尺寸作为尺寸证据，z 位置由桌面高度和已知半高补齐；方盒 yaw 显式标记为对称不确定。错误尺寸、点数不足、退化对应、低 inlier ratio 和高残差都会拒绝。

## Alternatives

继续自写 voxel 聚类和 Kabsch SVD 可以减少依赖，但会重复成熟库的边界处理，增加维护和学习范围。把视觉算法直接放到 bridge 可以省一个 ROS 节点，却会把渲染/算法延迟和 physics step 耦合，难以离线 fixture 重放。

`depth_image_proc` 和 `pcl_ros` 是常见 ROS 运行时组件，但当前容器没有安装它们；因此使用已安装的 `image_geometry` 和 PCL C++ API，保持同一算法库而不引入未验证的运行时依赖。

## Consequences

bridge 的 `mjData` 所有权、单线程 reset/step 前提和既有 RGB-D 话题保持不变。感知包可脱离 MuJoCo 对固定点云 fixture 做单测，也可以以后替换 RGB-D 驱动。代价是新增 `mujoco_perception` 包和 `VisionObjectPose` 接口，PCL 的参数和版本成为可记录的运行时依赖。

## Verification

包级测试覆盖 PCL SVD 正确变换、点数不匹配、共线退化、桌面去除/聚类、完整立方体 OBB、顶面可见但厚度隐藏和错误尺寸拒绝。真实 bridge + estimator 运行时验证得到 `accepted=true`、180 个目标点、约 0.824 置信度、约 0.68 mm 残差，位置约为 `(0.5000, 0.0002, 0.2400)m`；运行结束后无遗留 bridge 或 estimator 进程。

尚未完成的验证包括随机物体位姿、深度噪声、动态遮挡、误分割和长时间延迟统计。这些属于 Stage Q 的后续评测，不由当前静态 smoke test 代替。
