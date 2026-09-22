# 智慧工厂仿真平台 · 操作指南 —— **本机开发版**（`README1.md`）

> 📌 **本文是开发机自己的指南**：宿主机跑 Isaac Sim，ROS2 装在 `ros2-humble-dev` Docker 容器里。
> **发布给赛队的是 `README.md`**（Ubuntu 22.04 原生 Humble 那份）；
> 两份内容一一对应，**上肢遥操 / 数采 / 转换 / 训练 / 推理五段完全一样**。
>
> **本机环境：宿主机 Isaac Sim + Docker ROS2。** 这份文档的目标是：**照着做，就能把项目现在
> 已有的全部功能复现出来**（起仿真 → 建图 → 2D Goal Pose 导航 → 遥操 → 数采 → 转换 → 训练 → 推理）。
>
> - **不包含环境安装/依赖配置** —— 假设已经装好了（安装与依赖见 `../环境依赖查证_Ubuntu22.04_Humble.md`、
>   历史交接见 `../HANDOFF.md`）。
> - 过时/被本文取代的文档都归档在 **`docs_archive/`**（含旧的 `OPERATIONS.md`、`START_HERE.md`、
>   `PROJECT_README.md`、`README_TECHNICAL.md`、上游 doc 站点等）。**冲突时：本机环境以本文（`README1.md`）为准；发布给赛队的以 `README.md` 为准。**
>
> 最后核对：2026-09-19。

---

## 目录

| 章 | 内容 |
|---|---|
| [0](#0-先读本机环境与分工) | 本机环境与分工（宿主机 / 容器各自干什么） |
| [1](#1-五分钟最小闭环) | 五分钟最小闭环（先确认平台是活的） |
| [2](#2-启动仿真唯一入口) | **启动仿真**（唯一入口 + 三种模式） |
| [3](#3-底盘建图slam) | **建图**（手动 / 无人值守） |
| [4](#4-底盘2d-goal-pose-导航) | **2D Goal Pose 导航**（RViz 手点 + 命令行） |
| [5](#5-上肢遥操四种方式) | **上肢遥操**（四种方式 + 键位表） |
| [6](#6-上肢数据采集hdf5) | **数据采集**（HDF5） |
| [7](#7-数据转换hdf5--lerobot-v3) | **数据转换**（HDF5 → LeRobot v3） |
| [8](#8-模型训练act) | **模型训练**（ACT） |
| [9](#9-模型推理) | **推理**（策略服务端 + 仿真侧） |
| [10](#10-全流程串一遍复制粘贴版) | 全流程串一遍（复制粘贴版） |
| [11](#11-坐标系与关键常量速查) | 坐标系与关键常量速查 |
| [12](#12-常见问题排查) | 常见问题排查 |
| [13](#13-能力边界能做什么不能做什么) | 能力边界（能做什么 / 不能做什么） |

---

## 0. 先读：本机环境与分工

### 0.0 如果这台机器还没装环境（可选）

> **完整安装说明见 `INSTALL.md`**（硬件要求 / 一键脚本 / 手动分步 / ROS2 / 验收清单 / 排错）。

仓库带了一个**一键安装脚本**，装齐「除 ROS2 以外」的全部环境
（Isaac Sim 6.0.1 standalone、IsaacLab 3.0.0、本项目 `leisaac`、lerobot 环境）：

```bash
bash reproduce/install_all.sh --dry-run     # 先看它要做什么（不动任何东西）
bash reproduce/install_all.sh               # 真装（没有 Isaac Sim 会自动下载 ~12 GB，支持断点续传）
```

- 装完它会写 `reproduce/local.env` 把探测到的路径记下来，之后所有脚本自动使用。
- **ROS2 不在其中**（本机是 Docker 镜像 `ros2-humble-dev:latest`；原生 22.04 用 apt 装）。
- 脚本**幂等**：任何一步失败，修好后原样重跑即可，已完成的步骤会自动跳过。
- 常用开关：`--isaacsim-dir/-zip`（用现成的）、`--skip-isaacsim/--skip-isaaclab/--skip-lerobot`、
  `--torch-cu cu128|cu126`、`--lerobot-env NAME`、`--no-download`、`-y`。`--help` 看全部。

### 0.1 本机事实（本文所有命令都基于这些路径）

| 项 | 值 |
|---|---|
| 仓库 | `/home/vedal/WorkStation/homework1/leisaac` |
| Isaac Sim | `/home/vedal/WorkStation/isaac-sim-6.0.1`（standalone，用自带 `python.sh`） |
| 仿真框架 | IsaacLab 3.0.0（`dependencies/IsaacLab` submodule） |
| GPU | RTX 5070 Ti Laptop，**12227 MiB（12 GB）**，驱动 580.178.04 |
| ROS2 镜像 | `ros2-humble-dev:latest`（已装 Nav2 / slam_toolbox / RViz / teleop_twist_keyboard） |
| lerobot 环境 | `/home/vedal/miniconda3/envs/leisaac-lerobot-sim/bin/python`（lerobot 0.4.2 + torch 2.7.1+cu128） |
| 默认任务 | `LeIsaac-SmartFactory-v0`（智慧工厂比赛场地，移动双臂机器人） |
| 现成产物 | `datasets/kitchen_biarm.hdf5`（17 GB / 52 demos）、`outputs/train/kitchen_biarm_act_300k/checkpoints/300000/pretrained_model`、`datasets/lerobot/kitchen_biarm_local`（已转好的 v3 数据集，2 集 / 1582 帧） |

**建议先把这几个变量 export 到每个终端里**（下面命令直接用它们）：

```bash
export REPO=/home/vedal/WorkStation/homework1/leisaac
export ISAACSIM=/home/vedal/WorkStation/isaac-sim-6.0.1
export LR=/home/vedal/miniconda3/envs/leisaac-lerobot-sim/bin/python
export IMG=ros2-humble-dev:latest
```

### 0.2 分工：什么跑在宿主机，什么跑在容器

```
┌─────────────── 宿主机 ───────────────┐        ┌──────────── 容器（ROS2 Humble）────────────┐
│ Isaac Sim（场地 + 双臂 + 3 相机 +     │  TCP   │ ros2_leisaac_bridge.py                     │
│            2D 激光 + 运动学底盘）      │ :5560  │   → 发 /odom /scan /tf                     │
│ SimBridgeServer（TCP 服务端）          │◄──────►│   ← 收 /cmd_vel、/leisaac/reset            │
│ ChassisController（底盘运动学）        │        │ Nav2 / slam_toolbox / RViz / 键盘          │
│ 上肢：hold / 主手 / 策略               │        │                                            │
│ 遥操设备、HDF5 录制                    │        │                                            │
└───────────────────────────────────────┘        └────────────────────────────────────────────┘
     ↑ 上肢与仿真：宿主机                              ↑ 底盘与导航：容器
```

三条硬规则，先记住：

1. **仿真启动入口只有一个**：`reproduce/ros2_chassis_teleop.sh`，用途靠 `--mode` 切。
   `teleop_left_arm.sh` 之类都只是它的薄封装。
2. **桥接同一时刻只能有一个客户端**。容器里所有东西（键盘 / SLAM / Nav2 / RViz）都通过
   **话题**和这唯一一根"网线"说话 —— 所以"建图的终端 2 + 键盘的终端 3"是正常的，但
   **键盘那个别再起桥接**。最省事的做法是**所有容器侧的东西跑在同一个容器里**，用 `docker exec` 开第二个终端。
3. **仿真侧只负责"显示 + 执行 + 回传观测"**；所有算法（SLAM / Nav2 / 策略）都在容器或独立进程里。

---

## 1. 五分钟最小闭环

确认平台是活的（headless、约 1 分钟）：

```bash
cd $REPO
bash reproduce/verify_env.sh --steps 60
```

期望输出末尾：`[smoke] OK: ... 环境复现通过。`，退出码 0。
（要看相机加 `--cameras`；这个脚本**固定 headless**，不需要窗口。）

> **2026-09-18 在本机实测**：`--steps 60` → `[smoke] OK ... 60 步 (0.5s)`，退出码 0；
> 加 `--cameras --steps 30` → `[smoke] OK ... 30 步 (0.3s)`，退出码 0（日志里有
> `Created new renderer for simulation: IsaacRtxRenderer`，说明相机渲染通道起来了）。

**跑起来会看到这些「警告」—— 都是无害的，不用管**（下表全部来自上面的实测输出）：

| 日志 | 为什么无害 |
|---|---|
| `Could not perform 'modify_rigid_body_properties' on any prims under '/World/envs/env_0/Body'` | 躯干是**实例化的纯视觉**模型（本来就没挂刚体/碰撞），改不动是正常的 |
| `env_cfg.viewer is deprecated ... automatically forwarded` | IsaacLab 3.0 把 `viewer` 换成 `default_visualizer_cfg`；旧写法仍生效 |
| `Seed not set for the environment` | 没传 `--seed`；同机同配置照样可复现 |
| `Skipping unsupported non-NVIDIA GPU: Intel(R) Graphics` | 笔记本带核显，Isaac Sim 正确选了 NVIDIA 独显 |
| `CPU performance profile is set to powersave` / `PCIe link width current (8) and maximum (16) ... don't match` / `IOMMU is enabled` | 性能提示（省电模式 / PCIe 降宽 / IOMMU），**只影响速度、不影响正确性** |
| `Encountered USD Warnings but USD Diagnostics are currently muted` | USD 诊断被静音了，不是错误 |
| `PhysicsUSD: Parse collision ... falling back to convexHull approximation` | 场景资产固有，会刷屏但无害（只会拖慢并撑大日志） |
| `DLSS increasing input dimensions: Render resolution of (213, 160) is below minimal input resolution of 300` | 相机分辨率小，DLSS 自动放大输入；不影响图像内容 |

然后开一次仿真看窗口：

```bash
bash reproduce/ros2_chassis_teleop.sh
```

窗口出现、终端打印下面这些就算就绪（GUI 约 11 秒，headless 约 6 秒）：

```
[SimBridge] Listening on 127.0.0.1:5560
...
 pos=(0.450, 2.550) yaw=0.0°   ...   clients=False
```

`pos=(0.450, 2.550)` 就是**起点区**（场地坐标），车头朝 **+X**。
`clients=True` 表示容器里的桥接客户端连上了（第 2 节才连）。
用 `Ctrl+C` 退出，或 `bash reproduce/stop_all.sh` 一键收工。

---

## 2. 启动仿真（唯一入口，**只有一个模式**）

### 2.1 所有操控通道同时在线（2026-09-18 起）

**不再需要 `--mode` 选"底盘/上肢"** —— 一次仿真里所有通道同时可用，互不干扰：

| 通道 | 怎么"打开" | 谁在驱动 |
|---|---|---|
| **底盘** | 容器里跑键控（`ros2_keyboard_container.sh`）或 Nav2 发目标 | ROS2 `/cmd_vel`，**随时可用** |
| **上肢：遥操** | 主手使能 / 键盘按 `B` | 遥操设备（`--teleop_device`，默认双臂主手） |
| **上肢：策略** | 加 `--enable_policy`（或旧的 `--mode policy`） | 策略；**人一按 `B` 就交给人**，`R`/`N` 后交回策略 |
| **上肢：保持** | 谁都没接管时 | 自动保持位姿（IK 项零动作 + 臂重力已关，不会塌） |
| **录制** | 默认就挂着 | 手工：`R` = 失败重录，`N` = 成功保存；**推荐用 §6 的 `collect.sh` 一条命令录一场** |

也就是：**打开底盘遥操就能开底盘，打开机械臂遥操就能操作机械臂，两者可以同时进行**
（Nav2 边开边让策略做上肢；或者一边微调车一边调臂都行）。

- `--mode nav` / `--mode teleop` **仍然接受**，但只是兼容别名（行为完全一样，会打印一行提示）
- 真要把底盘锁住（怕误碰底盘键）加 `--lock_chassis`
- 上肢设备**建不起来也不会挡住仿真**：没插主手/串口不对时只打印一条警告，
  仿真照常起（底盘 + 策略可用）；要键盘遥操加 `--teleop_device bi-keyboard`

#### 数采："怎么把机器人弄到柜子前面"

现在**三条路都不用重启仿真、也不用切模式**：

```bash
# ★ 办法 1（推荐）：直接从作业位起仿真，根本不用开过去
bash reproduce/ros2_chassis_teleop.sh --teleop_device bi-keyboard \
     --start_at shelf --enable_cameras            # shelf = 收纳架前 (2.85, 0.85)，面向架子开口

# 办法 2：底盘本来就自由 —— 容器里用键控/Nav2 开过去，停稳后再操作上肢
bash reproduce/ros2_chassis_teleop.sh --teleop_device bi-keyboard --enable_cameras

# 办法 3：不想用预设点就自己写坐标（覆盖 --start_at 的对应分量）
bash reproduce/ros2_chassis_teleop.sh --teleop_device bi-keyboard \
     --start_x 2.85 --start_y 0.85 --start_yaw 3.1416 --enable_cameras
```

- `--start_at` 可选 `home`（起点区）/ `shelf`（收纳架前）/ `transfer`（转运架前）/ `park`（停放区），
  坐标定义在 `smart_factory_layout.py::WORK_ZONES`，改尺寸只动那一处
- **按 `N` 保存一条 demo 后，机器人留在原地**（只有手臂回初始位姿）——
  一个作业位能连着录完所有 demo，不用每次开过去
- ⚠️ 采数据中途**别用容器里的"复位"节点**（`reset_sim`）：那会把底盘送回起点

#### 运行中切换（**仿真全程不用重启**）—— 你要的"打开/关掉某个程序"就是这个

上肢由谁驱动、要不要录制，**都可以在仿真跑着的时候从容器侧切**（各是一条命令）：

```bash
# 上肢交给遥操设备（主手/键盘，一按 B 就接管）—— 数采前
ros2 topic pub --once /leisaac/arm_source std_msgs/String "{data: device}"

# 上肢交给策略（**懒加载**：这时才去连策略服务端）；换端口/换模型都不用重启仿真
ros2 topic pub --once /leisaac/arm_source std_msgs/String "{data: policy}"
ros2 topic pub --once /leisaac/arm_source std_msgs/String \
  "{data: '{"'"'"'arm_source'"'"'":'"'"'policy'"'"','"'"'policy_port'"'"':5557}'}"

# 谁也不动，保持位姿（切回来 / 收工）
ros2 topic pub --once /leisaac/arm_source std_msgs/String "{data: hold}"

# 录制：开始 / 本条成功保存 / 放弃本条
ros2 topic pub --once /leisaac/record std_msgs/String "{data: begin}"
ros2 topic pub --once /leisaac/record std_msgs/String "{data: success}"
ros2 topic pub --once /leisaac/record std_msgs/String "{data: fail}"
```

- 仿真窗口里也有快捷键：**`P` = 在 auto / policy 之间切**（`B` 仍然是把控制权抢回到设备）
- 优先级：`policy` 时策略驱动，人一按 `B` 立刻抢回；`device` 只认设备；`hold` 谁都不动
- 切到 `policy` 但策略服务端没起 → **只警告并退回 `auto`**，仿真和遥控都不受影响

**对应的完整流程**（仿真开一次，容器里按需开关程序）：

| 步骤 | 宿主机（终端 1，全程不动） | 容器（终端 2） |
|---|---|---|
| ① 起仿真 | `bash reproduce/ros2_chassis_teleop.sh`（默认已含相机 + 录制 + 设备探测；激光急停默认**关**） | — |
| ② 建图 / 或开到柜子前 | （同一个仿真） | `ros2 launch scripts/nav2_config/ops/teleop.launch.py` → 开完 **`Ctrl+C` 关掉它** |
| ③ 数采 | （同上） | `ros2 topic pub --once /leisaac/arm_source std_msgs/String "{data: device}"` + `... /leisaac/record ... "{data: begin}"`；每条做完发 `success` |
| ④ 关仿真、后台训练 | `Ctrl+C`（或 `stop_all.sh`） | 见 §7/§8（转换 + `lerobot-train`） |
| ⑤ 重开仿真 → 开过去 → 推理 | `bash reproduce/ros2_chassis_teleop.sh`（要改策略端口才需要加参数） | 先 `teleop.launch.py` 开到柜子前并 `Ctrl+C`，再 `ros2 topic pub --once /leisaac/arm_source std_msgs/String "{data: policy}"` → 或直接在仿真窗口按 **`P`** |

### 2.1.1 冲突检测（脚本之间互相感知，不用人猜）

| 冲突 | 现在的行为 |
|---|---|
| 开了**两个仿真** | 第二个在起 Isaac Sim **之前**就被拦住（锁文件 `/tmp/leisaac_sim.lock` + 桥接端口预检），并提示 `bash reproduce/stop_all.sh` |
| 开了**两个 ROS2 桥接客户端** | 仿真侧**明确拒绝**第二个并回一行 JSON 说明；被拒的客户端打红色错误（以前是"键盘能按、车不动"，极难查） |
| `/cmd_vel` 上**多个发布者**（键控 + Nav2 同时开） | 桥接客户端每 2 秒查一次，>1 就警告并列出节点名（两边抢方向盘 = 抖动/画龙） |
| 上肢**设备与策略**都想驱动 | 按优先级串行：设备启动时设备优先，`R`/`N` 后交回策略 —— 结构上不可能同时驱动 |

### 2.2 起仿真（宿主机，终端 1）

```bash
cd $REPO

# 建图 / 导航（想防穿墙加急停；默认关 —— 0.45 在场地角落会让人以为"车动不了"）
bash reproduce/ros2_chassis_teleop.sh --lidar_stop_distance 0.25

# 无窗口 + 15 分钟后自动收工（省 GPU）
bash reproduce/ros2_chassis_teleop.sh --headless --max_seconds 900

# 遥操 / 数采
bash reproduce/ros2_chassis_teleop.sh \
     --left_arm_port /dev/ttyACM0 --right_arm_port /dev/ttyACM1 --enable_cameras

# 推理（先在另一个终端起策略服务端，见 §9）
bash reproduce/ros2_chassis_teleop.sh --enable_policy --policy_type local-act \
     --policy_host 127.0.0.1 --policy_port 5556 --policy_action_horizon 100 --enable_cameras
```

### 2.3 常用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--mode {nav,teleop,policy}` | 无 | **已废弃**：唯一的兼容别名（nav/teleop 等同"统一行为"；policy = 顺便打开策略） |
| `--start_at {home,shelf,transfer,park}` | 无 | 直接以**作业位**起仿真（`shelf` = 收纳架前） |
| `--start_x / --start_y / --start_yaw` | 无 | 自定义起始位姿（覆盖 `--start_at` 对应分量） |
| `--lock_chassis` | 关 | 锁住底盘（忽略 `/cmd_vel`）。默认**不锁** |
| `--enable_policy` | 关 | 启动时就打开策略驱动上肢 |
| `--teleop_device NAME` | **自动探测** | 有两条主手串口 → `bi-so101leader`；没有 → `bi-keyboard`；也可手动指定 `so101leader-one` 等。**建不起来只是警告，不挡仿真** |
| `--headless` | 关 | 不开窗（省显存，推荐配合 RViz 时用） |
| `--max_seconds N` | 0=不自动退 | N 秒后自动收工 |
| `--lidar_stop_distance D` | **0（关）** | 前方 ±40° 净空 < D 米就禁止继续前进（运动学底盘会穿墙）。**默认关**：4×3m 场地**角落**里前向锥内 0.70m 就打到围栏，设 0.45 会让"想往前开却一动不动"（实测）。要防穿墙设 0.25~0.3 |
| `--enable_cameras` | GUI 下自动开；headless 关 | 打开三路相机（headless 里录数据/推理要显式给） |
| `--record` / `--no_record` | **开** | recorder 一直挂着（按 N / 容器指令才真正写）；自动命名落在 `datasets/sessions/`；纯建图会话可 `--no_record` |
| `--dataset_file X` | 自动 `datasets/sessions/session_<月日>_<时分>.hdf5` | 指定录制文件（不写就自动命名，所以同一条命令能反复跑） |
| `--camera_view {panel,windows,off}` | `panel` | 三路相机显示方式（`panel` 约 +4 ms/步，`windows` 约 +29 ms/步） |
| `--hud / --no_hud` | 开 | 窗口左上角状态行 |
| `--lidar_vis` | 关 | 画出激光射线 |
| `--collision_force_threshold N` | 20 | 接触力超过它且正往障碍里顶 → 停止平移 |
| `--task NAME` | `LeIsaac-SmartFactory-v0` | 换任务（跑厨房要 `LeIsaac-LeRobot-Kitchen-v0`） |

### 2.4 一键停止

```bash
bash reproduce/stop_all.sh --status     # 先看有什么在跑
bash reproduce/stop_all.sh              # 停（会先列出来问你）
bash reproduce/stop_all.sh -y           # 不问直接停
bash reproduce/stop_all.sh --containers-stopped   # 连已退出的容器也删
```

---

## 3. 底盘：建图（SLAM）

**前提**：宿主机仿真已经在跑（§2.2）。

### 3.1 开容器（终端 2）

先给 X 授权（**宿主机执行一次**）：

```bash
xhost +SI:localuser:$USER
```

然后起容器。**注意 `-it` 是必须的**（建图入口自带键盘，需要 tty；`-i` 不够）：

```bash
docker run -it --rm --name leisaac-map \
  --network host --gpus all \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $REPO:/work -w /work \
  $IMG bash
# 进容器后：
source /opt/ros/humble/setup.bash
```

> `--gpus all` 不是可选项：容器里 RViz 缺 `/dev/dri` 会报
> `Invalid parentWindowHandle ... GLXWindow::create` 并崩（软件渲染也救不了，llvmpipe 同样要 `/dev/dri`）。

### 3.2 手动建图（推荐第一次这么做）

**一条命令 = 桥接 + slam_toolbox + RViz + 键盘**：

```bash
ros2 launch scripts/nav2_config/ops/mapping.launch.py rviz:=true
```

- 键盘就在**这个终端**里（`i` 前进 / `,` 后退 / `j` `l` 左右转 / `k` 停 / `q` `z` 调速）。
- **停止发指令 = 立即停车**（桥接侧 deadman 设计）。
- RViz 里应该看到：地图（白=空闲/黑=障碍/灰=未知）、红色激光点、`Fixed Frame = map`。
- 开着车在场地里**贴着墙走一圈**（转一圈把四面围栏都扫到）。停一会儿让地图稳定。

走完在**同一个容器**里开第二个终端存图 + 归位（另开一个宿主机终端）：

```bash
docker exec -it leisaac-map bash -lc "source /opt/ros/humble/setup.bash && \
    ros2 launch scripts/nav2_config/ops/save_map.launch.py"
docker exec -it leisaac-map bash -lc "source /opt/ros/humble/setup.bash && \
    ros2 launch scripts/nav2_config/ops/reset_sim.launch.py"
```

**顺序不能反**：先存图再归位（反了 slam 会收到位姿跳变，地图会歪）。

### 3.3 无人值守建图（自动探索 → 存图 → 归位 → 退出）

```bash
# 容器里（不需要 -it，因为 auto 模式强制关掉键盘）
docker run --rm --network host -v $REPO:/work -w /work $IMG \
  bash -lc "source /opt/ros/humble/setup.bash && \
            ros2 launch scripts/nav2_config/ops/mapping.launch.py auto:=true"
```

约 150 秒，全自动退出（容器 `Exited (0)`）。实测：场地扫出 **4.02×3.03 m** 地图、空闲约 87%。

### 3.4 ★ 地图文件怎么挑（最容易踩的坑）

地图存在 `scripts/nav2_config/maps/`。挑选规则是 **"文件名是纯数字的里面取最大"**，没有数字名才取最近修改。

**当前该目录里已经有 `0.yaml` / `1.yaml`（数字名）**，所以：

| 你怎么写 | 实际加载 |
|---|---|
| `map` 留空 | **`1.yaml`**（数字最大） |
| `map:=arena` | `arena.yaml`（**M3 验证过的场地图**） |
| `map_name:=auto`（存图时） | 下一个数字名 = `2` |

> **要跑到那张验证过的场地图，必须显式写 `map:=arena`**，不要依赖"留空自动挑"。
> 每次启动导航都会打印 `[nav] 使用地图: <绝对路径>`，**发目标前先看这一行**。

---

## 4. 底盘：2D Goal Pose 导航

**前提**：仿真在跑 + **已经有一张地图**（第 3 章）+ 机器人回到起点。

### 4.1 用 RViz 的 "2D Goal Pose" 手点目标（最直观）

容器里（和建图同一个容器最省事，或新开一个）：

```bash
ros2 launch scripts/nav2_config/ops/navigation.launch.py \
    rviz:=true map:=arena localization:=odom
```

这条命令会：**先归位**（`reset_first:=true`，等 3 秒）→ 起 Nav2（map_server + 规划 + 控制）→ 起 RViz。

在 RViz 里：

1. 左侧 `Fixed Frame` 保持 **`map`**。
2. 顶栏点 **`2D Goal Pose`** 按钮。
3. 在地图上**点一下**（目标位置）→ **按住拖出箭头**（目标朝向）→ 松手。
4. 会看到绿色全局路径 `/plan`、黄色局部路径 `/local_plan`、红色激光点；机器人开始走。

> **不需要点 "2D Pose Estimate"** —— `amcl.set_initial_pose=true` 且初始位姿写死 `(0,0,0)`，
> 因为"建图与导航都从同一个物理起点开始"。

### 4.2 用命令行发目标（无人值守，可脚本化）

```bash
# 一条命令走完：先归位 → 起 Nav2 → 等 active → 发目标 → 报结果 → 退出
ros2 launch scripts/nav2_config/ops/navigation.launch.py \
    goal:="2.40 -1.70 0" localization:=odom map:=arena
```

`goal` 三个数是 **map 坐标系的 `x y yaw_deg`**（yaw 可省，默认 0）。
输出会打印实时剩余距离，最后给 `SUCCEEDED ✅` / `ABORTED ❌`。

**map 坐标怎么来的 —— 记住这一个换算**：

```
map 坐标 = 场地坐标 − (0.45, 2.55)
```

| 位置 | map 坐标 | 场地坐标 |
|---|---|---|
| 起点（机器人初始在这，车头朝 +x） | `(0.00, 0.00)` | `(0.45, 2.55)` |
| 停放区 | `(3.10, 0.00)` | `(3.55, 2.55)` |
| 收纳架前（面向货架） | `(2.40, −1.70)` | `(2.85, 0.85)` |
| 转运架前 | `(0.30, −1.70)` | `(0.75, 0.85)` |

### 4.3 定位模式二选一（`localization:=`）

| 模式 | 做法 | 实测终点误差 | 什么时候用 |
|---|---|---|---|
| `odom`（**比赛推荐**） | 把 `map→odom` 固定成单位变换（"里程计即真值"） | **≈1.6 cm** | 仿真里脚本化跑动最稳 |
| `amcl`（默认） | AMCL 粒子滤波在已有地图上做激光定位 | ≈0.2 m | 演示标准 ROS 定位流程 |

> `odom` 模式要求 **机器人现在就在起点**（所以有 `reset_first`）。地图也必须是从同一起点建的。

### 4.4 赛队自己写"自动定点导航脚本"要用的接口

平台不限制你怎么写，标准接口都开着（参考实现可直接抄）：

| 接口 | 类型 | 用途 |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose`（action，frame `map`） | **定点导航主接口** |
| `/bt_navigator/get_state` | lifecycle 服务 | 判断 Nav2 是否真的能接目标 |
| `/cmd_vel` | `geometry_msgs/Twist` | 不用 Nav2，直接驱动底盘（麦轮，支持 `vy`） |
| `/odom` | `nav_msgs/Odometry` | 里程计（起点为原点、初始朝向为 +x；**误差 ~cm 级**） |
| `/scan` | `sensor_msgs/LaserScan` | 360° 激光（z≈0.26 单平面） |
| `/tf` | TF | `odom→base_footprint`、静态 `base_footprint→base_link/base_lidar_link` |
| `/leisaac/reset` | `std_msgs/Empty` | 底盘瞬移回起点 |

**两个必知的坑**（自己写脚本时一定会遇到）：

1. **发目标前必须等 `bt_navigator` 生命周期进入 ACTIVE**。只等动作服务器"出现"是不够的 ——
   它在节点一启动就存在，那时还是 inactive，目标会被**直接拒绝**。参考实现：
   `scripts/nav2_config/nodes/goal_client_node.py`（轮询 `/bt_navigator/get_state`，然后发目标、等结果、打印进度）。
2. **纯 `/cmd_vel` 的写法也有参考**：`scripts/nav2_config/nodes/auto_drive_node.py`
   （用 `/odom` 闭环走"距离/转角"，用 `/scan` 判净空提前停）。

---

## 5. 上肢：遥操（四种方式）

上肢的遥操**都在仿真进程里**（宿主机），不需要容器。共同规则：

- **必须先按 `B` 才开始控制**。不按 `B`，从手不动，看起来像"卡住"。
- `N` = 本次演示成功并保存（数采时用）；`R` = 放弃/重置当前这条。
- 按 `R` / `N` 之后要**再按一次 `B`** 才能继续。（如果同时开了 `--enable_policy`，
  `R`/`N` 之后那段时间由**策略**接管，再按 `B` 才回到你手里。）
- 要先**点一下 Isaac Sim 窗口**让它拿到键盘焦点（键盘设备读的是窗口焦点）。
- 真主手**第一次用要标定**：加 `--recalibrate`（交互式，会自动备份原标定 JSON）。

### 5.1 双臂真主手 `bi-so101leader`（推荐，12 维关节角）

```bash
ls /dev/ttyACM*                     # 先确认两条主手都在
bash reproduce/ros2_chassis_teleop.sh \
     --left_arm_port /dev/ttyACM0 --right_arm_port /dev/ttyACM1 \
     --enable_cameras
```

### 5.2 单臂主手 `so101leader-one`（只有一条主手时）

受控那半边跟主手，**另一只手臂锁定在当前位姿**；动作空间**仍是 12 维**（录出的数据与双臂一致）。

```bash
# 统一入口写法
bash reproduce/ros2_chassis_teleop.sh \
     --teleop_device so101leader-one --port /dev/ttyACM0 --arm_side left --enable_cameras

# 或者用它的一行薄封装（默认就是智慧工厂 + 左臂）
bash reproduce/teleop_left_arm.sh --arm_side left
```

> ⚠️ 按 `B` 之前，**先把主手掰成与仿真里那只手臂相近的姿势**（都是"伸直朝前"）。
> 差得太多（默认 >20°）从手不会动，终端会提示差最多的关节；摆近后**自动接管**，不用再按 `B`。
> 门槛太严可以加 `--engage_threshold 0.6`（弧度，约 34°）。

### 5.3 双臂键盘 `bi-keyboard`（没有真手时的兜底）

```bash
bash reproduce/ros2_chassis_teleop.sh --teleop_device bi-keyboard --enable_cameras
```

一次驱动**一条**臂，按 `T` 在左/右之间切换（做不到双手同时动）。

| 键 | 作用 | 每按一下 |
|---|---|---|
| `T` | **切换当前操作的臂**（LEFT ⇄ RIGHT，HUD 显示） | — |
| `W` / `S` | 末端沿**自身 Z** 前 / 后 | 1 cm |
| `Q` / `E` | 末端沿**自身 X** 上 / 下 | 1 cm |
| `J` / `L` | 腕部俯仰（pitch） | ≈8.6° |
| `K` / `I` | 腕部滚转（roll） | ≈8.6° |
| `A` / `D` | 该臂 `shoulder_pan` 左 / 右转 | 0.15 rad/次 |
| `U` / `O` | 该臂夹爪 开 / 合 | 0.15 rad/次 |

**两个必须知道的坑**：

1. **动作维度是 16，不是 12。** 键盘是"位姿增量 + IK"，每臂 8 维（肩部 1 + 夹爪 1 + IK 6 个增量）。
   转换脚本能自动识别 16 维（`dim_i` 命名 + 原样存，不套"弧度→电机量值"），所以键盘数据**能转能训**
   （见 §7/§8）。**但一份数据集里不要混两种设备** —— 12 维和 16 维动作**语义不同**。
2. **不按键时臂不会下塌**（2026-09-18 修）：键盘走相对模式 IK，"零动作 = 目标跟住当前位姿"，
   原本会被重力匀速拽着往下掉 —— 现在 `bi-keyboard`/`bi-gamepad` 会**自动关掉机械臂的重力**
   （和上游对单臂 `keyboard`、对 `bi_so101_state_machine` 的处理一致）。
   实测：不按键 30 步的下沉从 **−15.4 cm 变成 −0.0000 m**；按住 `Q` 就是干净的 **+11.0 cm** 抬升。
   想保留重力（"臂有重量"的对比实验）加 `--arm_gravity`。

**实测记录**（`reproduce/verify_bi_keyboard_events.py`，注入真实按键事件走 carb 输入管线）：
按 `B` 启动 ✅ / `Q` 抬臂 ✅ / `T` 换到右臂 ✅ / `W` 右臂末端移动 11.9 cm ✅ /
`O` 夹爪闭合 0.17 rad ✅ / `R`、`N` 回调 ✅。

### 5.4 单臂键盘 `keyboard`（**仅单臂任务**可用）

```bash
bash reproduce/ros2_chassis_teleop.sh --teleop_device keyboard \
     --task LeIsaac-SO101-CleanToyTable-v0
```

⚠️ **它不能用在双臂任务上**（场地/厨房）—— 键盘设备的动作配置写死了 `asset_name="robot"`，
而双臂场景只有 `left_arm`/`right_arm`，会直接报 `Scene entry with key 'robot' not found`。
双臂任务要用键盘就用 §5.3 的 `bi-keyboard`。

### 5.5 遥操时看相机画面

默认 `--camera_view panel`：一个窗口里并排显示 `left_wrist` / `right_wrist` / `front`
（**就是录进数据集的那三张图**）。

```bash
--camera_view panel     # 默认，+4 ms/步
--camera_view windows   # 每台相机一个可拖动视口窗口，+29 ms/步
--camera_view off       # 不显示（图像照常录）
```

---

## 6. 上肢：数据采集（HDF5）—— **一条命令，一场录完**

借鉴 `A1Z-LinkerHand` 的数采方式：**参数提前写进配置文件，然后一条命令把整场录完**
（定时分集 + 热键），**不需要 GUI**，也不需要记一堆参数。

### 6.1 用法

```bash
# ① 改参数（一次性）：reproduce/collect.env
#    EPISODES / EPISODE_TIME / RESET_TIME / TASK / DATASET / TASK_DESC / START_AT / TELEOP_DEVICE …
vi reproduce/collect.env

# ② 录
bash reproduce/collect.sh

# ③ 想临时改几个就加参数（命令行优先于配置文件）
bash reproduce/collect.sh --episodes 10 --episode_time 45 --dataset datasets/pick_eggplant_v2.hdf5
```

**这条命令自己会把仿真起好**（按配置摆到作业位）、录完、**优雅退出并刷盘**，最后打印转换命令。
如果仿真**已经在跑**（你自己起的、车已经开到柜子前了），它会**直接挂上去录，不重启、车不动**。

### 6.2 ★ 操作机械臂的两步（不点窗口 / 不按 B 就"像收不到键盘"）

```
① 用鼠标点一下仿真窗口       ← 键盘事件只发给**当前聚焦**的窗口
② 在仿真窗口按 B 开始控制     ← 不按 B，从手不动（看着就像没反应）
   双臂键盘再按 T 切换左/右臂；N 保存本条 / R 丢弃本条
```

`collect.sh` 开头的提示、仿真终端启动时的 `[自检]` 段、以及容器发 `record=begin` 时都会把这两步再念一遍。
若仿真跑在 `--headless`（没有窗口）下，键盘设备**收不到任何按键**，请去掉 `--headless`，或改用主手/策略。

### 6.3 会话里发生什么（定时分集 + 热键）

```
第 k/N 条： 复位场景 → 你操作（主手 / 双臂键盘）→ 到 EPISODE_TIME 秒自动保存
         → 停 RESET_TIME 秒（摆好下一轮）→ 下一条 ……
```

| 按键（在 collect.sh 那个终端里按） | 作用 |
|---|---|
| `→`（右箭头） | **提前结束当前阶段**：录制中 = 立刻保存这条；重置中 = 立刻开始下一条 |
| `←`（左箭头） | **丢弃当前这条并重录**（那条会被标成 `success=False`，转换时自动跳过） |
| `q` | 结束会话（已保存的都在） |

- **集与集之间的空转一帧都不录**：开始一条时仿真会把场景复位（每条起点一致），
  保存后立刻关掉录制开关 → 采出来的数据干净、没有开头一大段静止帧
- **每条以 `success=True` 落盘**，丢弃的那条是 `success=False`
- 录完 **优雅退出**（`env.close()` 刷盘）—— 直接杀进程会丢最后一条的元数据（踩过，已修）

### 6.4 `reproduce/collect.env` 配置项

| 键 | 默认 | 说明 |
|---|---|---|
| `EPISODES` | 30 | 录几条 |
| `EPISODE_TIME` | 60 | 每条多少秒（到点自动保存） |
| `RESET_TIME` | 20 | 条间停多少秒 |
| `TASK` | `LeIsaac-SmartFactory-v0` | 任务（跑厨房改成 `LeIsaac-LeRobot-Kitchen-v0`） |
| `DATASET` | `datasets/sf_pick_eggplant.hdf5` | 输出 HDF5（**必须不存在**；同名会拒绝，防止误覆盖） |
| `TASK_DESC` | — | 一句话任务描述，转换/训练用（同一个模型所有数据用同一句） |
| `START_AT` | `shelf` | 作业位（`shelf`/`transfer`/`home`/空=场景初始位姿） |
| `TELEOP_DEVICE` | 空=自动探测 | `bi-so101leader` / `bi-keyboard` / `so101leader-one` … |
| `LEFT_ARM_PORT` / `RIGHT_ARM_PORT` | `/dev/ttyACM0` / `ACM1` | 真主手串口 |
| `LIDAR_STOP` | 0（关） | 激光前向急停（别设 0.45，柜子前净空只有 0.38m） |
| `STOP_SIM_AT_END` | 1 | 会话结束后是否把（脚本起的）仿真关掉 |

### 6.5 手工数采（不想用脚本时）

仿真侧 recorder **默认就挂着**，所以直接按窗口键也能录：

```
B 开始控制 → 演示 → N 标记成功并保存 → R 丢弃 → 再按 B 录下一条
```

- 手动方式下同样"保存后即停止累计"，但**不会**自动复位场景（脚本方式会）
- `--num_demos N` 可让手动方式录够 N 条自动退出

### 6.6 产物

```
datasets/sf_pick_eggplant.hdf5
└── data/demo_0/
    ├── obs/left_joint_pos   (T,6) float32        ← 左臂 6 关节（弧度）
    ├── obs/right_joint_pos  (T,6) float32
    ├── obs/left_wrist / right_wrist / front  (T,240,320,3) uint8
    ├── actions              (T,12 或 T,16) float32   ← 主手 12 维 / 双臂键盘 16 维
    └── attrs: success / num_samples / seed
```

> ⚠️ **务必带相机**：`collect.sh` 默认 GUI 起仿真，相机是开的。headless 下相机默认关，
> 要录数据得自己加 `--enable_cameras`（否则 HDF5 里没有图像，训练/推理用不了）。

---

## 7. 数据转换：HDF5 → LeRobot v3

**只依赖 `h5py + lerobot`**，不需要 isaaclab、不用起仿真。在 lerobot 环境里跑：

```bash
cd $REPO
$LR scripts/convert/hdf5_to_lerobot_v3.py \
    --hdf5    datasets/sf_biarm.hdf5 \
    --repo_id sf_biarm_v1 \
    --root    $REPO/datasets/lerobot \
    --task    "把茄子放进收纳盒" \
    --fps 30
```

产物：`datasets/lerobot/sf_biarm_v1/`（`meta/info.json` 里 `codebase_version = v3.0`）。

### 7.1 动作空间自动识别（12 维 / 16 维都能转）

| 录制设备 | `action` 维度 | feature 名 | 数值处理 |
|---|---|---|---|
| `bi-so101leader`（真主手） | 12 | `left_*/right_*.pos` 关节名 | 弧度 → 电机量值 |
| `bi-keyboard`（双臂键盘） | **16** | `dim_0 … dim_15` | **原样存，不转换** |

`observation.state` **恒为 12 维**关节角→电机量值（与录制设备无关）。LeRobot 允许 action 与 state 维度不同，
所以两种数据都能直接训。

### 7.2 转完自检

```bash
$LR -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset(repo_id='sf_biarm_v1',
                    root='$REPO/datasets/lerobot/sf_biarm_v1',
                    video_backend='pyav')
print('episodes', ds.num_episodes, 'frames', ds.num_frames)"
```

**动作空间契约验收**（12 维主手 / 16 维键盘都测一遍，纯数据操作、不用起仿真）：

```bash
$LR reproduce/verify_convert_action_space.py
# 期望结尾：✅ 动作空间契约全部通过（12 维主手 / 16 维键盘都能进 LeRobot 训练）
```

它把两份合成数据各转一遍并逐条核对：feature 维度与命名、**16 维动作 == HDF5 原值（零转换）**、
12 维动作 == 弧度→电机量值、state 恒为 12 维、`dataset_to_policy_features` 认出
ACTION(16)/STATE(12)/3 路相机、DataLoader 能拼 batch。

### 7.3 三个要点

1. **每份数据转一次，`--repo_id` 别重复** —— 它就是训练时 `--dataset.repo_id` 的那个名字。
2. **`--task` 那句话会写进每一条 frame**，训练时作为语言指令进观测。**同一个模型的数据用同一句话**，
   推理时 `--policy_language_instruction` 要对得上。
3. **转换脚本内部已经调了 `dataset.finalize()`，别绕过它** —— 漏了的话 `meta/` 不完整，
   lerobot 会认为"本地没有这个数据集"，然后去连 huggingface.co（本机连不上就报错）。

### 7.4 现成可用的数据集（不想从头采就用它）

`datasets/lerobot/kitchen_biarm_local` —— 已转好的 v3 数据集：**2 集 / 1582 帧 / 30 fps**，
action 12 维、state 12 维、三路 240×320 相机。第 8 章的"最小训练验证"直接用它。

---

## 8. 模型训练（ACT）

用 lerobot 自带的 `lerobot-train`，在 **`leisaac-lerobot-sim`** 环境里跑：

```bash
export PATH=/home/vedal/miniconda3/envs/leisaac-lerobot-sim/bin:$PATH
cd $REPO

lerobot-train \
    --dataset.repo_id=sf_biarm_v1 \
    --dataset.root=datasets/lerobot/sf_biarm_v1 \
    --dataset.video_backend=pyav \
    --policy.type=act \
    --policy.device=cuda \
    --output_dir=outputs/train/sf_biarm_act \
    --job_name=sf_biarm_act \
    --steps=100000 \
    --batch_size=4 \
    --wandb.enable=false \
    --policy.push_to_hub=false
```

### 8.1 最容易踩的三个参数

| 参数 | 说明 |
|---|---|
| `--dataset.root` | 必须指向**直接含 `meta/` 的那个目录**（`.../lerobot/<数据集名>`）。指到上一层就找不到 |
| `--dataset.video_backend=pyav` | **必须写**。本环境的 `torchcodec` 缺 FFmpeg 共享库，lerobot 只看"包在不在"会误判，不写会解码失败 |
| `--batch_size` | 4（16–20 GB 显存）/ 8（24–32 GB）。**本机 12 GB，用 4**（或更小） |

其余按需：`--steps`（30 条数据 100k–200k 步够用）、`--policy.vision_backbone`（默认 `resnet18`）。

### 8.2 看什么判断训练正常

- `loss`：行为克隆损失，正常从 5~8 降到 1~2。**不降就是数据或分段有问题，别急着加步数。**
- `grdn`：梯度范数；`updt_s`：每步更新时间（< 0.05 s 正常）。

### 8.3 最小验证（不采数据，5 分钟看链路通不通）

用现成数据集跑 3 步，确认"数据集 → 训练 → checkpoint"这条路是通的：

```bash
export PATH=/home/vedal/miniconda3/envs/leisaac-lerobot-sim/bin:$PATH
cd $REPO
lerobot-train \
    --dataset.repo_id=kitchen_biarm_local \
    --dataset.root=datasets/lerobot/kitchen_biarm_local \
    --dataset.video_backend=pyav \
    --policy.type=act --policy.device=cuda \
    --output_dir=/tmp/act_smoke --job_name=act_smoke \
    --steps=3 --batch_size=4 --wandb.enable=false --policy.push_to_hub=false
```

期望：loss 从几十降到几十（3 步即可），退出码 0，`/tmp/act_smoke/checkpoints/000003/pretrained_model/` 有 `model.safetensors`。

### 8.4 接着训（resume）

```bash
lerobot-train \
  --config_path=outputs/train/sf_biarm_act/checkpoints/<步数>/pretrained_model/train_config.json \
  --resume=true          # ⚠️ 必须写成 =true（小写）
```

训完的目录结构（**推理时指 `pretrained_model` 那一层**）：

```
outputs/train/sf_biarm_act/checkpoints/100000/
├── pretrained_model/     ← 推理 / 策略服务端指这个
└── training_state/
```

---

## 9. 模型推理

分两个进程：**策略服务端**跑模型，**仿真侧**发观测、收动作。

### 9.1 终端 1：起策略服务端

```bash
cd $REPO
$LR scripts/evaluation/act_action_server.py \
    --checkpoint_path outputs/train/sf_biarm_act/checkpoints/100000/pretrained_model \
    --device cuda --port 5556
```

看到 `[Server] Listening on 127.0.0.1:5556` 即可。

- `--checkpoint_path` 指到 **`pretrained_model`** 那一层，不是 `checkpoints/<步数>`。
- TCP + JSON 协议（不走 gRPC）。显存紧张就 `--device cpu`（实测 100 步动作块 88–139 ms，够用）。
- **多个模型就换端口起多个服务端**（5556 / 5557 / 5558…），推理时换 `--policy_port` 即可。

### 9.2 终端 2：仿真侧推理（宿主机）

```bash
cd $REPO
bash reproduce/ros2_chassis_teleop.sh --enable_policy \
     --policy_type local-act --policy_host 127.0.0.1 --policy_port 5556 \
     --policy_action_horizon 100 \
     --policy_language_instruction "把茄子放进收纳盒" \
     --enable_cameras
```

- **动作安全层默认开**：位置限幅（关节行程内缩 2°）+ 每步增量限幅（0.1 rad/步）+ NaN/形状守卫。
  模型输出越界时它兜住，不会把仿真炸掉；对比实验可加 `--no_action_safety` 关掉（结束时打印拦截统计）。
- **底盘不锁** —— 所以可以 §4 让 Nav2 开着车、同时策略操作上肢（人在遥操上肢时也一样）。
- 动作块用完会自动请求下一块（`--policy_action_horizon`，默认 100，与 checkpoint 的 `n_action_steps` 对齐）。
- **`--policy_language_instruction` 要和 §7 转换时 `--task` 写的一致**。

### 9.3 用现成 checkpoint 立刻验证推理链路

仓库里有一份 300k 步的 ACT checkpoint（12 维双臂 + 三路 320×240）：

```bash
# 终端 1
cd $REPO
$LR scripts/evaluation/act_action_server.py \
    --checkpoint_path outputs/train/kitchen_biarm_act_300k/checkpoints/300000/pretrained_model \
    --device cuda --port 5556
```

```bash
# 终端 2（语言指令必须与它训练时的一致）
bash reproduce/ros2_chassis_teleop.sh --enable_policy \
     --policy_type local-act --policy_port 5556 --policy_action_horizon 100 \
     --policy_language_instruction "Pick oranges with the leRobot bi-arm manipulator." \
     --enable_cameras
```

> 这份 checkpoint 是用**厨房场景**的数据训的，在场地里跑只是验证"观测→模型→动作→仿真"这条链路通，
> **不表示它会把茄子抓对**。要它干场地的活，得按 §6→§7→§8 用场地数据重训。

### 9.4 环境自检（不用起仿真）

```bash
$LR reproduce/check_lerobot_env.py --device cuda
```

会打印 Python/lerobot/torch 版本、能否 `from_pretrained`、`make_pre_post_processors`、以及一次
`select_action` 的输出形状。

### 9.5 各环节验收脚本（想确认"真能用"就跑这个）

| 脚本 | 验什么 | 要不要仿真 |
|---|---|---|
| `reproduce/verify_bi_keyboard.py` | 双臂键盘设备的**动作语义**（16 维、T 切臂、只有激活臂动） | 要（headless） |
| `reproduce/verify_bi_keyboard_events.py` | **真实按键事件**链路：注键 → carb → 设备 → 物理（B/Q/T/W/O/R/N 全部实测） | 要（**开窗**） |
| `reproduce/verify_convert_action_space.py` | HDF5→LeRobot v3 的**动作空间契约**（12 维 / 16 维都能转能训） | 不要 |
| `reproduce/verify_smart_factory_props.py` | 场地货物落位与层高 | 要（headless） |

> ⚠️ 这些脚本的**退出码不可信**：`simulation_app.close()` 会直接把进程结束掉（码 0），
> 所以判据看输出里最后那一行 `✅ / ❌`。脚本内部已用 `os._exit()` 尽量给出真实退出码。

---

## 10. 全流程串一遍（复制粘贴版）

> 这一段假设：**已有两条 SO101 主手**（`/dev/ttyACM0`、`/dev/ttyACM1`）。
> 没有主手的话：遥操那步改用 `--teleop_device bi-keyboard`。

```bash
# ── 变量 ──────────────────────────────────────────────────────────────
export REPO=/home/vedal/WorkStation/homework1/leisaac
export ISAACSIM=/home/vedal/WorkStation/isaac-sim-6.0.1
export LR=/home/vedal/miniconda3/envs/leisaac-lerobot-sim/bin/python
export IMG=ros2-humble-dev:latest
export NAME=sf_biarm_v1                 # 数据集/模型名字，三段用同一个

# ── ① 采数据（宿主机，终端 1）：一条命令录一场（见 §6）──────────────
cd $REPO
vi reproduce/collect.env          # 改 DATASET / TASK_DESC / EPISODES / EPISODE_TIME …
bash reproduce/collect.sh         # 自己起仿真 → 定时分集 → 存盘 → 优雅退出
#    会话中：→ 提前结束保存   ← 丢弃重录   q 结束

# ── ② 转 LeRobot v3（终端 2；不需要仿真）──────────────────────────────
cd $REPO
$LR scripts/convert/hdf5_to_lerobot_v3.py \
    --hdf5 datasets/$NAME.hdf5 --repo_id $NAME \
    --root $REPO/datasets/lerobot --task "把茄子放进收纳盒" --fps 30

# ── ③ 训练（终端 2；12 GB 显存用 batch_size=4）───────────────────────
export PATH=/home/vedal/miniconda3/envs/leisaac-lerobot-sim/bin:$PATH
cd $REPO
lerobot-train \
    --dataset.repo_id=$NAME --dataset.root=datasets/lerobot/$NAME \
    --dataset.video_backend=pyav --policy.type=act --policy.device=cuda \
    --output_dir=outputs/train/${NAME}_act --job_name=${NAME}_act \
    --steps=100000 --batch_size=4 --wandb.enable=false --policy.push_to_hub=false

# ── ④ 推理：终端 3 起服务端 ──────────────────────────────────────────
cd $REPO
$LR scripts/evaluation/act_action_server.py \
    --checkpoint_path outputs/train/${NAME}_act/checkpoints/100000/pretrained_model \
    --device cuda --port 5556

# ── ⑤ 推理：终端 4 起仿真 ────────────────────────────────────────────
cd $REPO
bash reproduce/ros2_chassis_teleop.sh --enable_policy \
     --policy_type local-act --policy_host 127.0.0.1 --policy_port 5556 \
     --policy_action_horizon 100 \
     --policy_language_instruction "把茄子放进收纳盒" --enable_cameras

# ── ⑥ 收工 ───────────────────────────────────────────────────────────
bash reproduce/stop_all.sh
```

---

## 11. 坐标系与关键常量速查

| 项 | 值 |
|---|---|
| 场地内区 | `X∈[0,4.0] m`、`Y∈[0,3.0] m`、地面 `z=0`（4000×3000 mm） |
| 机器人起点 | 场地 `(0.45, 2.55)`，车头 yaw=0°（朝 +X） |
| `/odom` 原点 | **机器人起点**，初始朝向 = **map +x** |
| map ↔ 场地 | `map = 场地 − (0.45, 2.55)`（yaw=0 使两套坐标轴平行） |
| 围栏高 / 激光平面 | 围栏 0.35 m；2D 激光扫描平面 **z≈0.26 m（单平面）** |
| 收纳架 | 中心 `(2.27, 0.85)`，长边沿 Y，开口朝 +X；上层台面 z=0.58 |
| 转运架 | 中心 `(0.75, 0.85)`，长边沿 Y，开口朝 +X；可放面 z≈0.30 |
| 货物（全在上层台面 0.58） | 茄子 y=0.47 / 收纳盒 y=0.85 / 香蕉 y=1.23（沿 Y 从 −Y 到 +Y） |
| 相机 | `left_wrist` / `right_wrist` / `front`，均 **320×240 @30Hz**，仅 RGB（**无深度**） |
| 动作空间 | 真主手 **12** 维关节角；双臂键盘 **16** 维位姿增量 |
| 运动学参数 | 底盘外廓 0.4354×0.4563 m；Nav2 `robot_radius=0.26`；MPPI `vx_max=0.3 / vy_max=0.15` |

> ⚠️ **场地尺寸是按官方平面示意图 + 手臂可达性估的**（文档写明"实际场景后续群内公布"）。
> 拿到官方图纸后只改一个文件：`source/leisaac/leisaac/assets/scenes/smart_factory_layout.py`。

---

## 12. 常见问题排查

| 现象 | 原因 / 处理 |
|---|---|
| **键盘终端每按一个键就刷日志** | **已修**：桥接客户端原来每收到一条 `cmd_vel` 就打一行（键盘 10Hz 发布 → 每秒 10 行）。现在那个终端只有键位表；`currently: speed/turn` 只在你按 `q/z/w/x/e/c` 调速度时出现 |
| **键盘能按、车不动（但日志里 cmd 非零）** | ① 开了激光急停（`LiDAR急停=0.xx`）：场地**角落**里前向 ±40° 锥内 0.70m 就是围栏，`0.45` 会直接禁止前进 → 去掉参数或降到 0.25；② 可能是"被接触保险杠拦住"（日志有 `⛔撞到东西，已停`）→ 后退/转向即可 |
| **键盘能按、车不动（日志里 cmd 全是 0）** | 起了**两个桥接客户端** —— 现在仿真侧会**明确拒绝第二个**并在终端/客户端报出来（以前是静默排队）。把多余的关掉（含 `ros2_mapping.sh`/`ros2_navigation.sh` 起的）；最稳是同一个容器里 `docker exec` 开第二个终端 |
| **"打开仿真闪退"（窗口开了又退出）** | 看终端最后几行。最常见是**录制文件已存在**（同一分钟内起两次会撞名，已修：现在精确到秒 + 自动加 `_2` 后缀）。显式 `--dataset_file` 撞了名现在会**启动前 0.2s** 就报错并给三个选项（`--resume` / 换名 / `--no_record`） |
| **起第二个仿真被拦住** | 这是**故意的**（单实例锁 `/tmp/leisaac_sim.lock` + 桥接端口预检）。`bash reproduce/stop_all.sh --status` 看谁在跑；确实要并行就 `rm /tmp/leisaac_sim.lock`（但会抢显存） |
| **机器人抖动/画龙/走不动** | `/cmd_vel` 上**多个发布者**（键控 + Nav2 同时开）。桥接客户端会警告并列出节点名；手动遥控时先把 Nav2 的 `controller_server` 停掉 |
| 键盘报 `termios.error: (25, ...)` | 容器没用 **`docker run -it`**（`-i` 不够） |
| 跨容器"话题看得到、指令传不过去" | 两边都缺 `--ipc=host`。**最省事：所有容器侧东西跑在一个容器里** |
| RViz 报 `Invalid parentWindowHandle ... GLXWindow` | 容器缺 **`--gpus all`**（软件渲染也救不了） |
| 存图报"等不到 `/slam_toolbox/save_map`" | 建图进程已退出，或不在同一个容器里执行 |
| 存图报"还没收到地图" | 机器人一步没走 —— 没扫到东西 slam 就不出图 |
| **导航加载了错的地图** | 数字名优先：现在 `map` 留空会选 `1.yaml`。**显式写 `map:=arena`**，并看日志 `[nav] 使用地图: ...` |
| 导航终点来回微调 | 终点容差已放宽到 0.05 m / 0.10 rad；仍抖动查 `scripts/nav2_config/nav2.yaml` |
| 目标点被直接拒绝 | 发太早了 —— Nav2 的 `bt_navigator` 还没进 ACTIVE。用 `ops/navigation.launch.py goal:=` 或参考 `nodes/goal_client_node.py` |
| 仿真"没有窗口" | IsaacLab 3.0 不选可视化器就强制 headless。统一入口已自动加 `--visualizer kit`；手跑脚本时自己加 |
| 窗口出来了但视口空白 | 开了相机时 env 级 RTX 分区与 Kit 视口冲突。统一入口已设 `ISAAC_LAB_ENABLE_ISAAC_RTX_PER_SCENE_PARTITION=0` |
| 掰主手不动 | **没按 `B`**。按 `R`/`N` 之后要**再按一次 `B`** |
| 单臂键盘在双臂任务上报 `key 'robot' not found` | `keyboard` 只能用于单臂任务；双臂用 `bi-keyboard` |
| 双臂键盘松手后手臂下沉 | **已修**：`bi-keyboard` 会自动关闭机械臂重力（零动作不再下塌）。想保留重力加 `--arm_gravity` |
| **窗口打开后立刻退出**（终端有 Traceback） | 环境建好、窗口出来之后才进主循环，**任何异常都在这一步暴露**。先看终端最后 15 行；`bash reproduce/ros2_chassis_teleop.sh --headless` 能最快看到同样的错误 |
| **改完入口脚本先跑冒烟** | `timeout 60 bash reproduce/ros2_chassis_teleop.sh --headless --max_seconds 10` —— 看到 `[chassis] t=  ...` 心跳就说明"建环境 → 起设备 → 进主循环"整条通了 |
| 训练报找不到数据集 → 去连 huggingface | 转换时漏了 `finalize()`，或 `--dataset.root` 指错了层级 |
| 训练报文件锁 / `BlockingIOError` | 别同时转换和训练同一个数据集 |
| 推理首帧全白 | 静置步数不够，相机 render product 没更新（统一入口有 `--settle_steps`） |
| 显存不够 / 跑起来很卡 | 先 `nvidia-smi`；**别和训练抢显存**。headless + `--camera_view off` 最省 |
| 想一键停掉所有相关进程 | `bash reproduce/stop_all.sh --status` 先看，再 `bash reproduce/stop_all.sh` |

---

## 13. 能力边界（能做什么、不能做什么）

**平台提供（已实测）**：

| 能力 | 状态 |
|---|---|
| 场地 + 移动双臂机器人 + 三路相机 + 2D 激光（360°） | ✅ |
| 统一仿真入口（nav / teleop / policy 三模式 + TCP 桥接） | ✅ |
| 底盘遥控 / 建图（4.02×3.03 m，空闲 ~87%）/ 导航（SUCCEEDED，误差 1.6–3.0 cm） | ✅ |
| 上肢遥操四种方式 + HDF5 数采（12 维 / 16 维） | ✅ 真主手链路需插主手 |
| HDF5 → LeRobot v3 转换（12 维 / 16 维都验证过） | ✅ |
| ACT 训练（12 维、16 维都实跑通过）+ 推理（12 维，TCP 直连） | ✅ |
| 动作安全层（限幅 / 限速 / NaN 守卫） | ✅ |

**平台不提供 / 目前做不到（如实说明）**：

| 缺什么 | 说明 |
|---|---|
| **抓取算法、任务编排、成败判定** | **平台只提供能力**，策略与任务逻辑由各赛队自己实现、裁判判定 |
| **成功率指标** | 厨房/场地任务只有 `time_out`，没有 `success` 终止项。要评估得自己定义判据 |
| **深度相机** | 三路相机只有 RGB → "选取对应层高"没有距离信息 |
| **相机图像进 ROS2** | 桥接只发 `/odom` `/scan` `/tf`，**不发图像话题** |
| **16 维键盘模型的推理** | 推理侧仍按"6 维一段"反转换，16 维数据能录/能转/能训，**但还不能回仿真跑** |
| **底盘碰撞响应** | 运动学底盘**会穿墙**，静态避障只能靠 `--lidar_stop_distance` 或上层规划 |
| **单平面激光看不到台面货物** | 激光 z≈0.26，货物在 z≥0.58 → 抓取只能靠相机 |
| **任务级状态机（M4）** | 只有 `StateMachineBase` 骨架，没有场地任务状态机、没有裁判演示、没有 15 分钟运行模式 |
| **场地尺寸的可能偏差** | 现为按平面图估算，待官方图纸 |

---

## 附：归档与权威文档

| 想了解 | 看 |
|---|---|
| **本机全流程操作**（宿主机 Isaac Sim + Docker ROS2，本文） | `README1.md` |
| **Ubuntu 22.04 原生 Humble 版**（发布给赛队的那份） | `README.md` |
| **一键装环境**（除 ROS2：Isaac Sim / IsaacLab / 本项目 / lerobot） | `reproduce/install_all.sh`（`--help` 看用法；`--dry-run` 先试） |
| **安装说明**（硬件要求 / 手动分步 / ROS2 / 验收清单 / 排错） | `INSTALL.md` |
| 项目整体架构与原理 | `../HANDOFF.md`（**最权威**，历史交接 + 全部踩坑） |
| ROS2 launch 组合的细节、参数表 | `scripts/nav2_config/ops/README.md` |
| 场地本身（尺寸/分区/货物/摆位/待确认项） | `source/leisaac/leisaac/tasks/smart_factory/README.md` |
| 目标机依赖查证（Ubuntu 22.04 + Humble） | `../环境依赖查证_Ubuntu22.04_Humble.md` |
| **过时文档（归档）** | `docs_archive/` —— 旧 `OPERATIONS.md`、`START_HERE.md`、`PROJECT_README.md`、`README_TECHNICAL.md`、`OPERATIONS_NATIVE_HUMBLE.md`、`reproduce_README.md`、`MIGRATION_ISAACSIM6.md`、上游 doc 站点 |
| 撰写风格（写技术报告时参考） | `CLAUDE1.md` |
| AI 协作约定与开发须知 | `CLAUDE.md` |
