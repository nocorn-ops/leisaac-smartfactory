# 环境安装说明（从零到能跑）

> 这份文档讲**怎么把这套环境装起来**。装完之后怎么用（建图 / 导航 / 遥操 / 数采 / 训练 / 推理）看
> **操作指南**：**发给赛队/目标机的是 `README.md`**（Ubuntu 22.04 原生 Humble 版）；
> 开发机自己那份（宿主机 + Docker ROS2）是 `README1.md`。
>
> **验证状态**：本文所有版本号、路径、体积、命令都在这台开发机（Ubuntu 26.04 + RTX 5070 Ti）上**实测核对过**；
> 唯一没在仓库内验证的是 §4.2 的 Docker 镜像构建模板（开发机用的是现成镜像，我实测列出了它的内容）。

---

## 目录

| 章 | 内容 |
|---|---|
| [0](#0-装什么不装什么) | 装什么 / 不装什么 |
| [1](#1-硬件与系统要求) | 硬件与系统要求 |
| [2](#2-快速路径一键脚本) | **快速路径：一键脚本** |
| [3](#3-手动路径逐步与脚本一一对应) | **手动路径：逐步**（排错用） |
| [4](#4-ros2-humble脚本不含这一步) | **ROS2 Humble**（脚本不含这一步） |
| [5](#5-验收清单) | 验收清单 |
| [6](#6-路径覆盖localenv-机制) | 路径覆盖：`local.env` 机制 |
| [7](#7-磁盘规划) | 磁盘规划 |
| [8](#8-常见安装问题) | 常见安装问题 |
| [9](#9-重装--清理) | 重装 / 清理 |

---

## 0. 装什么、不装什么

| 组件 | 版本 | 装它干什么 | 一键脚本管吗 |
|---|---|---|---|
| **NVIDIA 驱动** | ≥ 580.95.05（Linux） | Isaac Sim 的 GPU 前提 | ❌（只检查 + 打印命令） |
| **Isaac Sim** | **6.0.1** standalone | 仿真本体（场景 / 物理 / 渲染 / 传感器） | ✅ |
| **IsaacLab** | **3.0.0**（git submodule） | 仿真框架（env / manager / 传感器抽象） | ✅ |
| **本项目 `leisaac`** | 仓库源码（editable） | 比赛任务、桥接、遥操设备、策略客户端 | ✅ |
| **lerobot 环境**（conda） | lerobot **0.4.2** + torch **2.7.1+cu128** | 数据转换 / ACT 训练 / 策略服务端 | ✅ |
| **ROS2 Humble** | Humble | 底盘遥控 / 建图（SLAM）/ 导航（Nav2） | ❌（见 §4） |

> ⚠️ **不要把 ROS2 和 Isaac Sim 装在同一个 Python 里**。Isaac Sim 用自带的 Python 3.12，
> ROS2 Humble 用系统 Python 3.10，两者依赖互相冲突。本项目通过 **TCP 桥接**让它们通信，
> 所以**物理上同机、逻辑上两个终端**即可（详见两份操作指南的 §0）。
>
> ⚠️ **为什么锁 Isaac Sim 6.0.1 而不是上游文档写的 5.1.0**：5.1.0 的 RTX 渲染器在 RTX 50 系
> （Blackwell sm_120）上必崩（驱动 595 段错误 / 驱动 580 时 `vkCreateRayTracingPipelinesKHR` 失败），
> 属软硬件代际不兼容、配置解决不了。迁移记录见 `docs_archive/MIGRATION_ISAACSIM6.md`。

---

## 1. 硬件与系统要求

**Isaac Sim 6.0 官方要求**（[官网 Requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html)，已逐项核对）：

| 项 | 最低 | 建议 | 本开发机（实测能跑全流程） |
|---|---|---|---|
| 系统 | Ubuntu **22.04 / 24.04**、Windows 11 | 同左 | Ubuntu 26.04（**超官方范围**，靠 `reproduce/compat_libs_isaac6` 顶住） |
| CPU | Intel i7 7 代 / Ryzen 5，4 核 | i7 9 代 / Ryzen 7，8 核 | i9-13900HX，24 核 |
| 内存 | **32 GB** | 64 GB | 30 GB（够，但别同时开太多） |
| 磁盘 | 50 GB SSD | 500 GB SSD | ≥150 GB 建议（见 §7） |
| GPU | **GeForce RTX 4080** | RTX 5080 | RTX 5070 Ti Laptop |
| 显存 | **16 GB** | 16 GB+ | **12 GB 也能跑通**（仿真 + Nav2 + RViz 同开；headless 更省） |
| 驱动（Linux） | **580.95.05** | 580+ | 580.178.04 |

**关于显卡的结论**：

- **RTX 40 系（Ada）可以用，不用改代码** —— 4080/4090 完全达标；4060/4070（8–12 GB）低于官方最低显存，
  但本平台在 12 GB 上已跑通全流程，建议全程 `--headless`、必要时不开 RViz。
- **RTX 50 系（Blackwell sm_120）**：torch 必须是 **cu128**（cu126 及以下没有 sm_120 内核）。
  一键脚本默认就装 cu128。
- 40 系想省点下载量可以用 `--torch-cu cu126`（cu128 也兼容 40 系）。

**非 22.04 的系统**（Ubuntu 24.04 / 26.04 等）：Isaac Sim 6.0.1 依赖一些旧 ABI 库
（`libxml2.so.2` / `libicu*.so.74`）。仓库里带了 `reproduce/compat_libs_isaac6/`，
启动脚本**只在库文件真实存在时**才把它前置进 `LD_LIBRARY_PATH`，所以：

- **22.04**：自带 `libxml2.so.2` → 自动跳过，**不需要**这套兼容库；
- **26.04**：需要 → 自动生效。

---

## 2. 快速路径：一键脚本

```bash
cd <仓库根目录>

bash reproduce/install_all.sh --dry-run     # ① 先看它要做什么（不动任何东西）
bash reproduce/install_all.sh               # ② 真装
```

脚本会依次做 6 件事（每一步都会打印它在干什么）：

```
[1/6] 前置自检        系统 / GPU+驱动 / 磁盘 / 工具 / conda
[2/6] Isaac Sim       探测已装的 → 本地 zip → 自动下载（约 12.1 GB，断点续传）
[3/6] IsaacLab        子模块 + _isaac_sim 软链 + ./isaaclab.sh --install + pip install -e source/leisaac
[4/6] lerobot 环境    conda create + lerobot[async]==0.4.2 + torch cu128 + h5py/av + 自检
[5/6] local.env       把探测到的路径写进 reproduce/local.env
[6/6] 验收            headless 冒烟测试
```

### 2.1 常用开关

| 开关 | 作用 |
|---|---|
| `--dry-run` / `-n` | **只打印要执行的命令，什么都不改**（首次强烈建议先跑这个） |
| `-y` / `--yes` | 无人值守：自动同意（**包括 12 GB 下载**）。非交互环境下不给 `-y` 就不会自动下载 |
| `--isaacsim-dir PATH` | 已经装好 Isaac Sim，直接用 |
| `--isaacsim-zip PATH` | 有本地 zip，解压它 |
| `--isaacsim-dest DIR` | 解压到哪（默认 `<仓库>/../isaac-sim-6.0.1`） |
| `--isaacsim-url URL` | 换镜像源下载 |
| `--no-download` | 缺 Isaac Sim 时**报错退出**，不下载 |
| `--lerobot-env NAME` | conda 环境名（默认 `leisaac-lerobot-sim`） |
| `--conda-dir DIR` | conda 安装位置（默认自动探测） |
| `--torch-cu cu128\|cu126` | torch 的 CUDA 版本（默认 `cu128`） |
| `--skip-isaacsim` / `--skip-isaaclab` / `--skip-lerobot` / `--skip-smoke` | 跳过对应步骤 |
| `--install-driver` | 顺带 `apt install nvidia-driver-580-open`（需要 sudo，装完**必须重启**） |
| `--min-free-gb N` | 磁盘下限（默认 60） |

### 2.2 脚本的三个性质（为什么可以放心跑）

1. **只增不删** —— 脚本里没有任何针对用户数据的删除动作。解压 Isaac Sim 时如果目标目录**非空**，
   它会**拒绝**而不是覆盖。
2. **幂等** —— 每步先检测：Isaac Sim 在就跳过、子模块在就跳过、conda 环境在就跳过。
   **任何一步失败，修好后原样重跑即可**，已完成的步骤会自动跳过。
3. **不碰系统** —— 驱动、ROS2 都不动，只打印该敲什么。

### 2.3 典型用法

```bash
# 已经下载好 zip（省一次 12 GB 下载）
bash reproduce/install_all.sh --isaacsim-zip ~/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64.zip

# Isaac Sim 装在别处
bash reproduce/install_all.sh --isaacsim-dir /opt/isaac-sim-6.0.1

# 只想补建 lerobot 环境（Isaac Sim / IsaacLab 都好了）
bash reproduce/install_all.sh --skip-isaacsim --skip-isaaclab

# 40 系想用 cu126
bash reproduce/install_all.sh --torch-cu cu126
```

### 2.4 安装要多久

| 阶段 | 时间（参考） |
|---|---|
| 下载 Isaac Sim（12.1 GB） | 取决于网速（千兆 ≈ 2 分钟；百兆 ≈ 20 分钟） |
| 解压 Isaac Sim（→ 38 GB） | 3–8 分钟 |
| `isaaclab.sh --install` | 5–20 分钟（视网速，要下 PyTorch 等一堆包） |
| 建 lerobot 环境 | 5–15 分钟 |
| 冒烟测试 | 约 20 秒～1 分钟（首次编译 shader 可能到 1 分钟） |

---

## 3. 手动路径：逐步（与脚本一一对应）

**只在脚本失败、或者你想知道每一步到底发生了什么时才需要看这节。**

### 3.1 系统与驱动

```bash
# 系统：Ubuntu 22.04 / 24.04（官方支持）
lsb_release -ds

# 驱动 ≥ 580.95.05；装完必须重启
sudo apt install -y nvidia-driver-580-open
sudo reboot

# 验收：应打印显卡名 / 显存 / 驱动版本，且**没有** NVML mismatch
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv
```

> 顺带装好基础工具：`sudo apt install -y git unzip curl`。

### 3.2 Isaac Sim 6.0.1 standalone

官方直链（**已实测 HTTP 200，且响应体字节数与开发机上那份 zip 完全一致**）：

```
https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.0.1-linux-x86_64.zip
```

- 大小 **13,018,774,928 字节（12.1 GB）**；解压后 **38 GB**
- 服务器支持 `Range` → 可以断点续传：`curl -L -C - -o <zip> <url>`
- ⚠️ **zip 顶层没有文件夹**（`isaac-sim.sh` / `python.sh` 直接在根），所以要**解压到一个你指定的目录**里

```bash
export ISAACSIM_DIR=$HOME/isaac-sim-6.0.1
mkdir -p "$ISAACSIM_DIR"
unzip -q -n ~/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64.zip -d "$ISAACSIM_DIR"

# 验收：能打印 Python 版本（应是 3.12.x）
"$ISAACSIM_DIR/python.sh" -c "import sys; print(sys.version)"
```

> 也可以用 NVIDIA 官方的 Compatibility Checker 先自查：
> `"$ISAACSIM_DIR/isaac-sim.compatibility_check.sh"`（见官网 Workstation Installation 页）。

### 3.3 IsaacLab 3.0.0 + 本项目 `leisaac`

```bash
cd <仓库根目录>

# ① 跑之前先退出 conda/venv —— IsaacLab 3.0 拒绝"下载版 Isaac Sim + 虚拟环境"的组合
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE VIRTUAL_ENV

# ② 拉子模块（必须是 release/3.0.0）
git submodule update --init --recursive
cat dependencies/IsaacLab/VERSION        # 期望输出 3.0.0

# ③ 把 Isaac Sim 链到 IsaacLab 期望的位置
ln -sfn "$ISAACSIM_DIR" dependencies/IsaacLab/_isaac_sim

# ④ ★ 用 IsaacLab 自带的安装入口（会装**全部 14 个子包**）
cd dependencies/IsaacLab && ./isaaclab.sh --install && cd ../..

# ⑤ 装本项目（editable）
"$ISAACSIM_DIR/python.sh" -m pip install -e source/leisaac
```

**为什么用 `isaaclab.sh --install` 而不是手写几个 `pip install -e`**：
`--install`（等价 `-i`，不带值 = `all`）会装 **14 个 editable 包**：

```
核心 12：isaaclab, isaaclab_ppisp, isaaclab_contrib, isaaclab_newton, isaaclab_physx,
         isaaclab_assets, isaaclab_experimental, isaaclab_ov, isaaclab_tasks,
         isaaclab_tasks_experimental, isaaclab_rl, isaaclab_visualizers
可选 2：isaaclab_mimic, isaaclab_teleop
```

> ⚠️ **别只手动装 `isaaclab` / `isaaclab_assets` / `isaaclab_tasks` 三个就完事** ——
> `leisaac` 还**直接 import 了 `isaaclab_physx`**（`source/leisaac/leisaac/utils/physx_compat.py`），
> 少了它 PhysX 参数会静默走到 2.x 回退分支、不生效。
> IsaacLab 的 CLI 自带重试（3 次）和依赖顺序处理，比手写 pip 稳。

验收：

```bash
"$ISAACSIM_DIR/python.sh" -c "import isaaclab, isaaclab_physx, leisaac; print('ok', isaaclab.__version__)"
```

### 3.4 lerobot 环境（conda）

```bash
# ① 建环境（python 3.12）
#    ⚠️ 坑 1：直接 conda create 会先要求接受 Anaconda 默认频道的 ToS
#       → 用 --override-channels -c conda-forge 绕开
conda create -n leisaac-lerobot-sim python=3.12 pip -y --override-channels -c conda-forge

LR=$HOME/miniconda3/envs/leisaac-lerobot-sim/bin/python

# ② lerobot（[async] 会带上 grpcio，lerobot.async_inference 需要）
"$LR" -m pip install "lerobot[async]==0.4.2"

# ③ ★ 坑 2：pip 装来的 torch 是 +cu126，**不含 sm_120 内核**（50 系会报
#    "CUDA error: no kernel image is available for execution on the device"）。
#    必须**显式写本地版本号**，否则 pip 认为 +cu126 已满足、不换。
"$LR" -m pip install --index-url https://download.pytorch.org/whl/cu128 \
    "torch==2.7.1+cu128" "torchvision==0.22.1+cu128"

# ④ 本项目额外需要的两个包
#    h5py → 读写 HDF5；av（PyAV）→ lerobot 的视频解码后端（--dataset.video_backend=pyav）
"$LR" -m pip install h5py av

# ⑤ 自检（不需要 checkpoint）
"$LR" reproduce/check_lerobot_env.py --deps-only
```

> ⚠️ **坑 3：不要 `conda create --clone <别人的环境>`** —— 克隆会把对方的 **editable 安装**一起继承
> （`.pth` 文件指向别人的源码目录），"新环境"其实不独立。
>
> 依赖上的硬约束（改版本前先看）：`draccus==0.10.0`（0.11 的 `ChoiceRegistry` API 变了）、
> `huggingface-hub<0.36.0`（与新版 transformers 冲突）→ **策略服务端只能用这个环境启动**。

实测这套装出来的关键版本：

| 包 | 版本 |
|---|---|
| lerobot | 0.4.2 |
| torch / torchvision | 2.7.1+cu128 / 0.22.1+cu128 |
| h5py / av | 3.16.0 / 15.1.0 |
| numpy / pillow | 2.2.6 / 12.3.0 |
| datasets / grpcio / protobuf | 4.1.1 / 1.73.1 / 6.31.0 |

### 3.5 冒烟测试

```bash
cd <仓库根目录>
export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y
bash reproduce/verify_env.sh --steps 60
```

期望：`[smoke] OK: LeIsaac-SmartFactory-v0 构建并稳定运行 60 步 (...) ，环境复现通过。`，退出码 0。

> 首次跑会看到一堆「警告」（`modify_rigid_body_properties` 改不动、`viewer is deprecated`、
> `Skipping unsupported non-NVIDIA GPU` 等）—— **都无害**，逐条解释见操作指南 §1 的表。

---

## 4. ROS2 Humble（脚本不含这一步）

一键脚本**故意不装 ROS2**：它涉及 `sudo apt`、仓库源、以及和系统 Python 的绑定，交给你自己控制更安全。

### 4.1 方式 A：apt 原生安装（**推荐**，也最省事）

只装本项目需要的包（不是全套 desktop）：

```bash
# ① 加 ROS2 apt 源（如果还没加过）
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ② 装包
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2
```

| 包 | 干什么用 |
|---|---|
| `ros-humble-ros-base` | ros2 CLI + **rclpy** + tf2（最小底座；桥接客户端是 rclpy 脚本，必须要） |
| `ros-humble-navigation2` | Nav2 全家桶：AMCL / NavFn 规划 / MPPI 控制 / costmap / behavior / map_server |
| `ros-humble-nav2-bringup` | Nav2 的标准 launch（保险起见显式列出） |
| `ros-humble-slam-toolbox` | 建图 |
| `ros-humble-teleop-twist-keyboard` | 底盘键盘遥控 |
| `ros-humble-rviz2` | 看地图 / 点 **2D Goal Pose** |

**★ 不需要 `colcon build`**：本仓库的 launch 都是**按路径启动**的
（`ros2 launch scripts/nav2_config/ops/xxx.launch.py`），不是 ament 包 —— 不用建工作空间、不用编译。

验收：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E "^(nav2_bringup|slam_toolbox|rviz2|teleop_twist_keyboard)$"   # 4 个都在
python3 -c "import rclpy, sys; print('rclpy OK', sys.version.split()[0])"             # 应是 3.10
```

> ⚠️ **两个终端，别混**：ROS2 终端里不要跑 Isaac Sim 的 `python.sh`，Isaac Sim 终端里不要
> `source /opt/ros/humble/setup.bash` —— 两边 `PYTHONPATH` / `LD_LIBRARY_PATH` 会互相污染。
> 详见操作指南 §0.1（原生版）。

### 4.2 方式 B：Docker（想在别的系统上跑 ROS2 时）

> ⚠️ **本仓库不提供 `ros2-humble-dev` 的 Dockerfile。** 开发机上的 `ros2-humble-dev:latest`
> 是本地现成镜像（**没进仓库、无法从这里复现构建过程**）。下面是它的**实测内容**和我给的**构建模板**
> —— 模板**未在仓库内验证过**，用它之前请自己跑一遍验收。

**参考镜像的实测内容**（`docker inspect` + 容器内检查）：

| 项 | 值 |
|---|---|
| 基础系统 | Ubuntu **22.04.5 LTS** |
| ROS | **Humble**（`/opt/ros/humble`） |
| Python | **3.10.12** |
| 已装 | `nav2_bringup`、`nav2_msgs`、`slam_toolbox`、`rviz2`、`teleop_twist_keyboard`、`tf2_ros` 全部命中 |
| 大小 | 6.82 GB |
| Entrypoint / Cmd | **无 entrypoint**，Cmd = `/bin/bash` |
| 默认用户 | **`rosdev`（uid=1000, gid=1000）** |
| 环境变量 | `DISPLAY=:0`、`LANG=zh_CN.UTF-8` |
| ⚠️ 没有 `/ros_entrypoint.sh` | 所以**每次都要**手动 `source /opt/ros/humble/setup.bash` |

**构建模板**（从官方 ROS 镜像起，装同一组包）：

```dockerfile
# 保存成 Dockerfile.ros2-dev，然后：
#   docker build -f Dockerfile.ros2-dev -t ros2-humble-dev:latest .
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rviz2 \
        ros-humble-tf2-ros \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 官方 ros 镜像自带 /ros_entrypoint.sh 会 source ROS；本项目习惯显式 source，所以不设 entrypoint
CMD ["/bin/bash"]
```

### 4.3 ★ 容器的 uid 坑（**很关键，会让人以为"存图失败是 bug"**）

参考镜像里默认用户是 `rosdev`，**uid=1000**。开发机上宿主机用户也是 uid 1000 → 挂载进去的仓库可写，
一切正常。**但如果你的宿主机 uid ≠ 1000**，容器里的 `rosdev` 就**写不进挂载的仓库**，症状是：

- 建图能走、`/map` 也发得出来，但**存图时写 `.pgm/.yaml` 失败**；
- 训练/转换往 `datasets/` 写文件失败。

**补救**：让容器用你宿主机的 uid/gid（实测可用）：

```bash
docker run -it --rm --network host --gpus all \
  --user "$(id -u):$(id -g)" \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v <仓库>:/work -w /work \
  ros2-humble-dev:latest bash
```

> 另外三条**运行期**（不是安装期）的 Docker 注意事项，写在开发版 `README1.md` §3.1：
> RViz 必须 `--gpus all`；键盘必须 `docker run -it`；跨容器通信要 `--ipc=host`
> （**最省事是所有容器侧东西跑在一个容器里，用 `docker exec` 开第二个终端**）。

---

## 5. 验收清单

逐条跑，每条都有明确判定：

| # | 检查项 | 命令 | 期望 |
|---|---|---|---|
| 1 | 系统版本 | `lsb_release -ds` | `Ubuntu 22.04.x LTS`（或 24.04） |
| 2 | 驱动 | `nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv` | 有显卡名/显存，`driver ≥ 580`，**无 NVML 报错** |
| 3 | Isaac Sim Python | `"$ISAACSIM_DIR/python.sh" -c "import sys;print(sys.version)"` | `3.12.x` |
| 4 | 缺共享库 | `"$ISAACSIM_DIR/python.sh" -c "print(1)" 2>&1 \| grep "cannot open shared object"` | **无输出**（22.04 上不该缺 `libxml2.so.2`） |
| 5 | IsaacLab 版本 | `cat dependencies/IsaacLab/VERSION` | `3.0.0` |
| 6 | IsaacLab 可导入 | `"$ISAACSIM_DIR/python.sh" -c "import isaaclab, isaaclab_physx;print('ok')"` | `ok`（失败常见原因：conda 没 unset） |
| 7 | 本项目任务注册 | `"$ISAACSIM_DIR/python.sh" -c "import leisaac;print('ok')"` | `ok` |
| 8 | **冒烟测试** | `bash reproduce/verify_env.sh --steps 60` | 退出码 0，出现 `[smoke] OK` |
| 9 | **lerobot 依赖** | `"$LR" reproduce/check_lerobot_env.py --deps-only` | 9 项全 ✅、子模块 3 项全 ✅ |
| 10 | lerobot 可推理 | `"$LR" reproduce/check_lerobot_env.py --device cuda` | 返回 `(100,12)` 动作块（**需要仓库里有 checkpoint**） |
| 11 | ROS2 包 | `ros2 pkg list \| grep -E "^(nav2_bringup\|slam_toolbox\|rviz2\|teleop_twist_keyboard)$"` | 4 个都在 |
| 12 | rclpy 版本 | `python3 -c "import rclpy,sys;print(sys.version)"` | 3.10（Humble） |

一次跑完（把上面前 9 条串起来）：

```bash
cd <仓库根目录>
export ISAACSIM_DIR=<你的 Isaac Sim>  LR=<你的 lerobot python>
lsb_release -ds
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv
"$ISAACSIM_DIR/python.sh" -c "import sys, isaaclab, isaaclab_physx, leisaac; print('py', sys.version.split()[0], '| all imports ok')"
cat dependencies/IsaacLab/VERSION
bash reproduce/verify_env.sh --steps 60
"$LR" reproduce/check_lerobot_env.py --deps-only
```

---

## 6. 路径覆盖：`local.env` 机制

**装完之后所有脚本都不用改源码**就能找到 Isaac Sim / lerobot / conda，靠的是
`reproduce/leisaac_env.sh` 的三级覆盖：

```
export 的环境变量  >  reproduce/local.env  >  自动探测
```

一键脚本会在第 5 步写一份 `reproduce/local.env`（**已存在则不覆盖**）：

```bash
# 本机路径配置 —— 由 reproduce/install_all.sh 自动生成
ISAACSIM_DIR=/path/to/isaac-sim-6.0.1
LEISAAC_LR_PY=/path/to/miniconda3/envs/leisaac-lerobot-sim/bin/python
CONDA_DIR=/path/to/miniconda3
```

- 这个文件**不进 git、不进发布包**，是机器相关的私人配置。
- 自动探测的候选目录：`$ISAACSIM_DIR` → `<仓库>/../isaac-sim-*` → `$HOME/isaac-sim-*` →
  `$HOME/WorkStation/isaac-sim-*` → `$HOME/Downloads/isaac-sim-*` → `/opt/isaac-sim-*` → `/isaac-sim-*`；
  lerobot 解释器在若干候选里用 `import lerobot` 试。
- 想确认探测结果：

```bash
LEISAAC_VERBOSE=1 bash -c 'source reproduce/leisaac_env.sh'
# [env] 仓库     REPO=...
# [env] Isaac Sim ISAACSIM_DIR=...
# [env] lerobot   LR_PY=...
```

---

## 7. 磁盘规划

各组件实测体积：

| 组件 | 体积 |
|---|---|
| Isaac Sim 的 zip（装完可删） | **12.1 GB** |
| Isaac Sim 解压后 | **38 GB** |
| lerobot conda 环境 | **7.9 GB** |
| 仓库本体（不含 `datasets/` `outputs/`） | 8.7 GB |
| `dependencies/IsaacLab` | 101 MB |
| （可选）示例数据集 `datasets/kitchen_biarm.hdf5` | **17 GB** |
| （可选）转换产物 / checkpoint | 各 1–10 GB |

- 一键脚本要求目标分区**至少 60 GB 空闲**（`--min-free-gb` 可调）。
- **建议预留 150 GB**：38（Isaac Sim）+ 8（conda）+ 9（仓库）+ 17（数据集，若要用）
  + zip 12（装完可删）+ 余量。
- 装完可以删掉那个 zip 省 12 GB（脚本不替你删）。

---

## 8. 常见安装问题

| 现象 | 原因 / 处理 |
|---|---|
| `Downloaded Isaac Sim packages cannot be combined with a Python virtual environment` | 有 conda/venv 处于激活状态。**先 `conda deactivate`**，并 `unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE VIRTUAL_ENV` |
| `libxml2.so.2: cannot open shared object file` | 你的系统（26.04 等）没有 `libxml2.so.2`。确认 `reproduce/compat_libs_isaac6/libxml2.so.2` 存在；启动脚本会自动前置（22.04 不需要） |
| 下载 12 GB 中断 / `curl: (56) OpenSSL SSL_read: ... unexpected eof` | 这是**大文件被 CDN 掐断**的典型错误（不是你没网）。**原样重跑 `bash reproduce/install_all.sh` 即可自动续传** —— 脚本会发现那个残缺 zip、精确算出已有字节数、用 `curl -C -` 从断点接着下（并自动重试最多 5 轮，`--retry-all-errors` 已覆盖 error 56）。**不需要你删文件**。手动续传：`curl -L -C - -o <那个zip的路径> https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.0.1-linux-x86_64.zip` |
| zip 下完了但大小不对 | 官方大小是 **13,018,774,928** 字节。脚本会 `unzip -t` 做完整性校验：**残缺的不会拿去解压**，而是自动续传直到字节数精确匹配。若你放的是**别的版本**的完整 zip（能通过 `unzip -t`），脚本也会用，并提示"非官方字节数" |
| 解压后 `python.sh` 找不到 | zip 顶层没有文件夹，必须解压到**你指定的空目录**。脚本遇到非空目标会拒绝覆盖（不会替你删） |
| `isaaclab.sh --install` 中途网络失败 | IsaacLab CLI 自带 3 次重试；仍失败就原样重跑（幂等） |
| `import isaaclab_physx` 失败 | 只手动装了 3 个子包。用 `./isaaclab.sh --install` 装全 14 个（见 §3.3） |
| `conda create` 报 ToS / `CondaToSNonInteractiveError` | 加 `--override-channels -c conda-forge`（脚本已带） |
| `CUDA error: no kernel image is available for execution on the device` | torch 是 cu126，50 系需要 cu128。**显式**装 `torch==2.7.1+cu128`（只写 `2.7.1` pip 不会换） |
| `draccus` 报 `ChoiceRegistry` 相关错误 | draccus 版本不对，必须 **0.10.0**（lerobot 0.4.2 的约束） |
| 训练/推理报 `libavutil.so.59` 加载失败 | `torchcodec` 缺 FFmpeg 库。**用 PyAV**：加载数据集传 `video_backend="pyav"`，训练加 `--dataset.video_backend=pyav` |
| `nvidia-smi` 报 `Failed to initialize NVML: Driver/library version mismatch` | 内核模块和用户态驱动版本不一致（常见于 `unattended-upgrades` 自动升级后）。**重启机器**即可 |
| 容器里存图写不进去 | 容器 uid 和你宿主机 uid 不一致（见 §4.3），加 `--user "$(id -u):$(id -g)"` |
| `ros2: command not found` | 没 `source /opt/ros/humble/setup.bash`（该镜像没有 entrypoint，`docker exec` 进新终端也要重新 source） |
| 装完一跑仿真就 OOM | 12 GB 显存下：`--headless` + `--camera_view off`，别同时开 RViz 和训练 |

---

## 9. 重装 / 清理

**本说明只描述"该删什么"，不会替你删。** 想重装时按下面来（**先确认路径再执行**）：

| 想清掉 | 位置 | 说明 |
|---|---|---|
| lerobot 环境 | `conda env remove -n leisaac-lerobot-sim` | 7.9 GB。删完重跑脚本第 4 步即可 |
| Isaac Sim | 你解压出来的那个目录（如 `~/isaac-sim-6.0.1`） | 38 GB。注意先确认里面**只有** Isaac Sim |
| Isaac Sim 的 zip | 你下载的位置（12.1 GB） | 装完就没用了 |
| **下了一半的 zip** | 同上那个路径 | 脚本会拿它当**续传起点**，**别急着删** —— 先原样重跑一次，它会从断点接着下。只有当你确认这个文件已损坏、想从头下时才删 |
| **解压到一半的目录** | 你 `--isaacsim-dest` 指定的那个目录 | 脚本会**拒绝**往非空目录里解压（保护你的数据）。确认里面没有你要留的东西后删掉它，再重跑 |
| IsaacLab 的 editable 安装 | `"$ISAACSIM_DIR/python.sh" -m pip uninstall -y <包名>` | 一般不用清，重跑 `isaaclab.sh --install` 会覆盖 |
| `local.env` | `reproduce/local.env` | 只是路径配置；删掉会退回自动探测 |

> 重装 = 删掉对应的东西 → 原样重跑 `bash reproduce/install_all.sh`（幂等，已完成的会自动跳过）。
>
> ⚠️ **本脚本自己从不删你的文件** —— 上面所有"删"都是要你自己动手的；脚本只会在遇到
> 非空目录/残缺包时**停下来告诉你怎么办**。
>
> ⚠️ **别删仓库本体**：`assets/` 里有场景与机器人 USD、`datasets/` 里有你采的数据、
> `outputs/` 里有训练产物 —— 这些都是**结果**，不是环境。

---

## 附：相关文档

| 想了解 | 看 |
|---|---|
| **本文（安装）** | `INSTALL.md` |
| 装完怎么用（宿主机 + Docker ROS2，开发机） | `README1.md` |
| 装完怎么用（Ubuntu 22.04 原生 Humble，**发布给赛队的那份**） | `README.md` |
| 一键安装脚本本身 | `reproduce/install_all.sh --help` |
| 依赖查证过程与上机验收清单 | `../环境依赖查证_Ubuntu22.04_Humble.md` |
| 项目整体架构与全部踩坑 | `../HANDOFF.md` |
| 过时文档（归档） | `docs_archive/` |
