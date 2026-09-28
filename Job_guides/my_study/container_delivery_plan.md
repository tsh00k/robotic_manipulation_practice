# 容器化交付计划

本文记录如何把当前由人工维护的 Distrobox 开发环境，逐步转成可在 GitHub 上分发、由其他人复现和直接使用的容器环境。

当前 `robotics-dev` 仍是个人开发环境，不是可交付产物。Distrobox 默认会与宿主机共享 home，因此容器里看到的仓库、个人 `.bashrc`、pyenv、缓存和凭据可能来自宿主机挂载，不能假定它们会进入镜像，也不能把当前容器快照当作环境的唯一来源。正式验收必须使用专用容器 home，并把工作区作为显式 volume 挂载；不能把默认共享 home 作为复现前提。该决策见 [ADR 006](../../docs/adr/006-container-home-isolation.md)。

## 1. 目标交付形态

交付分成两个互补部分：

1. GitHub 仓库保存源码、依赖声明、容器构建配方、验收脚本和 CI 配置。
2. GitHub Container Registry（GHCR）保存 CI 构建并验证过的 OCI 镜像。

预期结构：

```text
GitHub repository
├── source code
├── Containerfile / Dockerfile
├── package.xml
├── container smoke-test script
└── GitHub Actions workflow
          │ build + test + push
          ▼
GitHub Container Registry
└── ghcr.io/<owner>/<project>:<version>
          │
          ▼
distrobox create --image ...
```

大体积镜像不提交进 Git 历史。Git 仓库存放可审查、可重建的配方，GHCR 存放构建产物。

## 2. 依赖的三层职责

### 2.1 ROS 包依赖：`package.xml` 和 `CMakeLists.txt`

每个包必须声明自己直接使用的依赖。以 Stage L 为例，真正使用 MoveIt Core API 的包应在 `package.xml` 中声明相应依赖，并在 `CMakeLists.txt` 中使用 `find_package(moveit_core REQUIRED)`。

这样可以通过以下命令安装 ROS 可解析依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

只在某个已有容器里执行过一次 `apt install`，不算完成依赖声明。

### 2.2 系统环境：`Containerfile` 或 `Dockerfile`

容器构建文件负责描述基础发行版、ROS2、编译工具、系统库和 ROS 二进制包如何安装。Stage L 当前计划安装的最小 MoveIt 集合为：

```text
ros-humble-moveit-core
ros-humble-moveit-kinematics
```

最终构建文件还应包含项目已经依赖的 `franka_description`、`control_msgs`、xacro、colcon 等工具。安装后清理 apt 列表，避免把无用缓存带进镜像。

MuJoCo 3.3.7 不是当前系统的 apt 依赖，不能只依赖个人容器里已有的 `/opt/mujoco-3.3.7`。容器配方应固定下载来源和版本、校验 SHA256，再安装到固定路径。

### 2.3 个人开发配置：不进入项目契约

个人 `.bashrc`、pyenv 配置、编辑器设置、缓存和凭据仍属于使用者自己的 home，不应成为构建成功的前提，也不应被复制进镜像。

MuJoCo 等必要环境变量应由镜像自身提供，例如通过 `ENV` 或镜像内的 `/etc/profile.d/robotics.sh` 设置：

```text
MUJOCO_DIR=/opt/mujoco-3.3.7
PATH=/opt/mujoco-3.3.7/bin:<existing PATH>
LD_LIBRARY_PATH=/opt/mujoco-3.3.7/lib
CMAKE_PREFIX_PATH=/opt/mujoco-3.3.7:<existing CMAKE_PREFIX_PATH>
```

配置中不得写死 `/home/anby/...` 等个人路径。

### 2.4 镜像 profile 与职责

“分层”描述的是镜像职责，不是把所有依赖安装到同一个长期运行的 `robotics-dev` 容器：

| profile | 内容 | 适用场景 |
| --- | --- | --- |
| `core` | ROS2、MuJoCo、C++、MoveIt、模型和测试工具 | 构建、单测、传统仿真和基础 demo |
| `perception` | `core` 加 OpenCV/PCL 和 RGB-D 处理依赖 | Week 4 视觉实验 |
| `policy` | `core`/`perception` 加 PyTorch、LeRobot 和具体模型依赖 | IL/RL/VLA 数据转换和推理 |

`core` 必须能在没有 Python 深度学习依赖时独立构建和测试。`policy` profile 的版本、CUDA 运行时和模型权重单独锁定，不成为基础项目的安装前置条件。

## 3. 开发镜像与运行镜像

第一阶段优先提供开发镜像。镜像包含工具链和系统依赖，但不烘焙工作区源码，也不依赖宿主机完整 home。使用者在宿主机 clone 仓库，给 Distrobox 指定独立 home，并只把仓库映射到 `/workspace`：

```bash
distrobox create \
  --name robotics-dev \
  --image ghcr.io/<owner>/robotic-manipulation-practice:<version> \
  --home "$HOME/.local/share/robotic-manipulation-practice/robotics-dev-home" \
  --volume "$PWD:/workspace:rw"

distrobox enter robotics-dev
cd /workspace
colcon build --symlink-install
```

这里的 `--home` 是容器专用 home；它可以位于宿主机，但不能指向宿主机的完整 `$HOME`。源码、构建产物和实验 artifact 通过显式 volume 管理，镜像内的 shell 配置和环境变量由镜像自身提供。GUI、GPU 和相机设备映射可以按运行场景增加，但不应重新引入完整 home 挂载。

CI 和最小复现测试优先直接使用 Docker/Podman 的显式 workspace mount，不依赖 Distrobox 的默认 home 行为。Distrobox 主要作为开发者体验入口。

如果以后需要无需源码即可启动 Demo，再增加单独的运行镜像。运行镜像只携带已构建的 `install/` 工作区和运行时依赖，不携带编译器、测试工具或个人开发配置。开发镜像与运行镜像不混为一个交付目标。

## 4. CI、版本与发布

GitHub Actions 最终应执行以下流水线：

1. 从 `Containerfile` 构建全新镜像。
2. 在镜像内执行 `rosdep` 检查、`colcon build --symlink-install`、`colcon test` 和 `colcon test-result --all --verbose`。
3. 执行不依赖人工交互的容器 smoke test，包括模型加载和关键节点启动检查。
4. Stage L 完成后，把固定样例的 MuJoCo、MoveIt 与自写 FK/Jacobian 一致性测试纳入镜像验收。
5. 验证空 home、专用 home 和显式 `/workspace` 挂载下都不读取作者个人路径。
6. 全部通过后推送到 GHCR。

镜像应提供不可变或语义清晰的版本标签，例如：

```text
humble-mujoco-3.3.7-v1
```

可以额外维护 `latest` 作为便利入口，但文档和 CI 验收应引用固定标签，不能只依赖会漂移的 `latest`。

## 5. 为什么不直接导出当前 Distrobox

`docker commit`、`podman commit` 或同类容器快照只能作为临时备份，不能作为正式交付来源，原因包括：

- Distrobox 挂载进来的共享 home 不会可靠成为镜像内容；
- 快照没有记录依赖为什么存在以及如何安装；
- 容易带入 apt 缓存、临时文件和机器特有配置；
- 难以审查、升级和从零重建；
- 可能意外携带凭据或其他隐私数据。

`distrobox-export` 的主要用途是把容器中的应用或服务暴露给宿主机，不等同于生成可复现开发镜像。

因此，当前容器只作为验证环境；`Containerfile` 才是环境构造的权威配方，CI 从空白基础镜像重建的结果才是可交付产物。

## 6. 从 Stage L 开始执行的纪律

每次新增环境依赖都按以下顺序处理：

1. 在 `robotics-dev` 中安装最小依赖集合并完成当前 stage，避免在尚未确认需求时过早固化大包集合。
2. 确认真正使用到的直接依赖，并同步更新消费包的 `package.xml` 和 `CMakeLists.txt`。
3. 把系统安装步骤和必要环境变量写入容器构建配方。
4. 从全新镜像重建项目，不能用当前长期运行容器的残留状态代替验证。
5. 运行完整构建、测试和 smoke test。
6. CI 通过后发布带固定版本标签的 GHCR 镜像。

Stage L 安装 MoveIt 时先执行第 1、2 步。首次建立容器交付基础设施时，再集中完成构建文件、CI 和 GHCR 发布；在此之前，新增包及其安装原因必须持续记录，避免后续反向猜测当前容器是怎样形成的。

## 7. 首次落地验收标准

容器化交付第一次完成时，至少满足：

- 一台没有现有 `robotics-dev` 容器的新环境能从构建配方创建镜像；
- 不读取原作者 `.bashrc`、pyenv、完整 home 或 `/home/anby/...` 路径也能构建；
- 使用专用容器 home 和显式 `/workspace` volume 可以完成构建和 demo；
- `colcon build`、`colcon test` 和测试结果汇总全部通过；
- MuJoCo 动态库、模型文件和 MoveIt Core 能被实际加载；
- Stage L 的模型一致性测试通过；
- 镜像中不包含 GitHub token、SSH key、shell history 或其他个人凭据；
- README 给出从拉取镜像、创建 Distrobox 到完成第一次构建的完整命令；
- 使用固定镜像标签可以重复得到相同的工具链版本和测试结果。
