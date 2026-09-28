# ADR 006：交付容器使用专用 home 和显式 workspace mount

## Status

已采用（2026-09-28）。

## Context

当前 `robotics-dev` 是 Distrobox 开发环境，默认与宿主机共享 home。这样可以方便访问源码和个人配置，但会让构建结果隐式依赖宿主机的 `.bashrc`、pyenv、缓存、凭据和路径。它不适合作为公开项目的复现条件。

项目还需要为 perception 和 policy 依赖提供可选 profile；LeRobot、PyTorch 和模型权重不应进入所有使用者都必须安装的 core 环境。

## Decision

正式容器交付使用可审查的 Containerfile/Dockerfile 和固定镜像标签，不使用长期运行容器快照作为来源。

Distrobox 开发入口必须：

- 使用 `distrobox create --home <dedicated-home>` 指定容器专用 home；
- 使用 `--volume <repo>:/workspace:rw` 显式挂载工作区；
- 不挂载宿主机完整 home，不依赖作者个人 `.bashrc`、pyenv 或缓存；
- 将 core、perception、policy 依赖拆成独立 profile 或镜像标签。

CI 和最小复现验证优先使用 Docker/Podman 的显式 workspace mount。GUI、GPU 和相机设备按运行场景单独映射，不以重新共享完整 home 为代价。

## Alternatives

继续使用 Distrobox 默认共享 home 可以减少开发命令，但构建会读取宿主机状态，无法证明镜像自足。

把源码和构建产物全部烘焙进一个巨型镜像可以减少挂载，但不利于开发迭代、增加 policy 依赖的发布成本，也会把个人或实验 artifact 混入运行环境。

直接 `docker commit`/`podman commit` 当前容器不能记录依赖来源、版本和安装步骤，不能作为正式交付来源。

## Consequences

换机器时需要显式 clone 仓库并创建 workspace mount；首次命令比默认 Distrobox 多一些参数。作为交换，构建、测试和 demo 可以在空 home 中复现，policy 依赖也不会污染 core 镜像。

## Verification

首次容器交付必须在空 home、专用 home 和显式 `/workspace` mount 下完成 `colcon build`、`colcon test`、模型加载和 demo smoke test，并确认没有读取作者个人路径。

