# `ops/` —— duojin01 风格的 launch 组合入口

## 这是什么

原来的操作方式是一堆 shell 脚本，每个都自己复制一份"起桥接 + 等 20 秒看 Connected"，
还要靠人记住"哪一步起桥接、别再起第二个"。参考 duojin01（`duojin01_bringup`）的做法，
把**一个操作 = 一条 `ros2 launch` 命令**。

> ⚠️ 这是**新增**的一套入口。原来的 8 个 shell 脚本和 `mapping.launch.py` /
> `navigation.launch.py` **一行没改**，随时可回退对照。关系见第 5 节。

---

## 0. 从 duojin01 借鉴了什么（重点看建图链）

| # | duojin01 的做法 | 这里怎么落地 | 解决的痛点 |
|---|---|---|---|
| 1 | **公共组件唯一化**：`base.launch.py` 被所有入口 include | `bridge.launch.py` 是唯一知道怎么起桥接的地方，其它入口 include 它，用 `bridge:=true/false` 决定带不带 | 三个 shell 各复制一份"起桥接 + 等 Connected"，且要人记"别再起第二个" |
| 2 | **入口自己带遥控**：`sim_mapping.launch.py` 固定 `use_teleop:=true` | `ops/mapping.launch.py` 的 `teleop` **默认 true** → 一条命令 = 桥接 + SLAM + RViz + 键盘 | 手动建图要开第二个终端，还要记得 `bridge:=false` |
| 3 | **模式由入口决定**：`sim_navigation.launch.py` 固定 `use_teleop:=false` | `auto:=true` 时**强制**关掉遥控（两者都在发 `/cmd_vel`，实测已互斥） | 自动扫场和人工遥控撞车 |
| 4 | **存图调 slam_toolbox 自带服务**：`duojin01_slam_tools/save_map_client_node.cpp` | `nodes/save_map_node.py` 复刻同样语义：等 `/slam_toolbox/save_map` → 传**绝对路径前缀** → 查返回码 | 原来 subprocess 调 `map_saver_cli`，QoS 要猜；slam 还没出图时报 `Failed to spin map subscription`，原因不明 |
| 5 | **自动递增命名**：`next_numeric_map_name()`，输出到 `maps/` | 同（`map_name:=auto` → `0` → `1` → `2` …） | 不覆盖历史地图 |
| 6 | **辅助操作单独一个小 launch**：`save_map.launch.py` | `ops/save_map.launch.py` / `ops/reset_sim.launch.py` / `ops/auto_drive.launch.py` | 存图/归位/自动行驶原来只能 `bash xxx.sh` |
| 7 | **参数统一命名 + 都有默认值** | 见第 3 节参数表，`--show-args` 可查 | `--file` vs `--map`、`--auto` 两处语义不同 |
| 8 | **能自动解析的绝不让用户敲**：`map_paths.py` 自动选图 | `map_paths.py` **完全照搬**：数字名优先；`map` 留空自动挑；另加 `map:=arena` 简写（不用写路径和后缀） | 每次都要敲 `map:=maps/xxx.yaml` |

> **建图请一律用 `map_name:=auto`**（数字递增）。只要 `maps/` 里存在数字图，命名图就不会被自动选中 ——
> 命名图只作为兼容 / 临时用途（详见第 4 节）。

---

## 1. 命令一览

容器里（`docker run -it ... -v <仓库>:/work -w /work`，`source /opt/ros/humble/setup.bash`）：

> ★ **如果赛队机器是 Ubuntu 22.04 + 原装 ROS2 Humble（不用 docker）**：
> 下面每一条命令都**去掉 `docker run … bash -lc "…"` 这层包装**，改成
> `source /opt/ros/humble/setup.bash` 然后直接敲即可 —— 本目录的 launch 文件**一行都不用改**。
> （原生路线的历史说明见 `docs_archive/OPERATIONS_NATIVE_HUMBLE.md`，按假设写、未实测。）

```bash
# ① 遥控（桥接 + 键盘）—— 容器必须 -it，原因见第 6 节①
ros2 launch scripts/nav2_config/ops/teleop.launch.py

# ② 建图 · 手动：**一条命令** = 桥接 + SLAM + RViz + 键盘，开着车走一圈
ros2 launch scripts/nav2_config/ops/mapping.launch.py rviz:=true
#    走完在另一个终端（同一容器 docker exec）存图 + 归位 ↓
ros2 launch scripts/nav2_config/ops/save_map.launch.py
ros2 launch scripts/nav2_config/ops/reset_sim.launch.py

# ③ 建图 · 无人值守：自动探索扫场 → 存图 → 归位 → 自动退出
ros2 launch scripts/nav2_config/ops/mapping.launch.py auto:=true

# ④ 不要键盘（遥控在别处 / 容器没 -it）
ros2 launch scripts/nav2_config/ops/mapping.launch.py teleop:=false rviz:=true

# ⑤ 导航 · 前台：起 Nav2 + RViz，自己在 RViz 点 "2D Goal Pose"
ros2 launch scripts/nav2_config/ops/navigation.launch.py rviz:=true

# ⑥ 导航 · 无人值守：归位 → 起 Nav2 → 等 active → 发目标点 → 报结果
ros2 launch scripts/nav2_config/ops/navigation.launch.py goal:="-0.7 0.0 0"
ros2 launch scripts/nav2_config/ops/navigation.launch.py goal:="2.40 -1.70 0" localization:=odom

# ⑦ 自动行驶（建图扫场 / 定点测试）
ros2 launch scripts/nav2_config/ops/auto_drive.launch.py pattern:=explore
ros2 launch scripts/nav2_config/ops/auto_drive.launch.py dist:=-0.5

# ⑧ 只起桥接（不带任何算法）
ros2 launch scripts/nav2_config/bridge.launch.py
```

所有入口都支持 `--show-args` 查参数：

```bash
ros2 launch scripts/nav2_config/ops/navigation.launch.py --show-args
```

### 宿主机侧（仿真由它启动）

```bash
# 默认就是智慧工厂场地（2026-09-16 起默认任务已从厨房改成 LeIsaac-SmartFactory-v0）
bash reproduce/ros2_chassis_teleop.sh --headless --max_seconds 600 --lidar_stop_distance 0.45

# 要看窗口就把 --headless 去掉
bash reproduce/ros2_chassis_teleop.sh --lidar_stop_distance 0.45

# 想跑别的任务（例如厨房）才需要显式指定
bash reproduce/ros2_chassis_teleop.sh --task LeIsaac-LeRobot-Kitchen-v0
```

> 默认任务改在了这两处：`reproduce/ros2_chassis_teleop.sh` 的 `TASK` 和
> `scripts/environments/ros2_chassis_teleop.py` 的 argparse 默认值；
> `reproduce/` 下其它工具脚本与 `scripts/` 下几个调试脚本的默认任务也一起改了。

---

## 2. 统一参数表

| 参数 | 出现在 | 默认 | 说明 |
|---|---|---|---|
| `sim_host` | 全部 | `127.0.0.1` | 桥接服务端地址（宿主机跑 `ros2_chassis_teleop.sh`） |
| `sim_port` | 全部 | `5560` | 桥接服务端端口 |
| `bridge` | 全部 | 主入口 `true` / 辅助 `false` | 是否启动桥接。**主入口**（teleop/mapping/navigation）默认自己起；**辅助**（save_map/reset_sim/auto_drive）默认不起，插进已有图里用 |
| `rviz` | mapping / navigation | `false` | 要看图就加 `rviz:=true`（容器里需 `--gpus all` + X11 授权） |
| `use_sim_time` | mapping / navigation | `false` | 仿真侧不发 `/clock`，保持 false |
| `map` | navigation | `""` = 自动挑 | 见第 4 节；也可只写名字（`map:=arena`） |
| `localization` | navigation | `amcl` | `amcl`=粒子滤波；`odom`=固定 `map→odom`（零误差，比赛脚本化跑动推荐） |
| `reset_first` | navigation | `true` | 起 Nav2 前先归位（地图从起点建的，定位按 (0,0,0) 起步） |
| `nav_delay_sec` | navigation | `3.0` | `reset_first` 时等多久再起 Nav2 |
| `goal` | navigation | `""` = 不发 | `"x y [yaw_deg]"`，map 坐标系 |
| **`teleop`** | mapping | **`true`** | 手动建图时是否把键盘一起拉起来（`auto:=true` 时强制忽略） |
| `teleop_speed` / `teleop_turn` | mapping | `0.5` / `1.0` | 建图入口里键盘的线速度 m/s / 角速度 rad/s |
| `auto` | mapping | `false` | 无人值守：自动探索 → 存图 → 归位 |
| `pattern` | mapping / auto_drive | `explore` / `none` | `explore`=贪心挑最开阔方向；`map`=原地转 360°×4 + 后退 |
| `save` / `reset` | mapping | `true` / `true` | `auto:=true` 时走完是否自动存图 / 归位 |
| `exit_when_done` | mapping | `true` | `auto` 全流程走完自动退出整个 launch |
| `map_name` | mapping / save_map | `auto` | `auto` = 数字递增（`0` → `1` → `2` …），不覆盖历史地图 |
| `output_dir` | mapping / save_map | `scripts/nav2_config/maps` | 输出目录（对齐 duojin01 的 `output_dir`） |
| `wait_timeout` | mapping / save_map | `30.0` | 等 `/slam_toolbox/save_map` 和等存完的超时 s |
| `clearance` / `speed` / `spin` | auto_drive | `0.35` / `0.20` / `0.60` | 安全净空 m / 平移速度 m/s / 转速 rad/s |
| `speed` / `turn` | teleop | `0.5` / `1.0` | 键盘线速度 / 角速度 |

> 数值参数写成整数也**不会**崩（例如 `turn:=90`、`wait_timeout:=8`）：节点里的浮点参数
> 都用 `dynamic_typing` 声明。不加这个的话 ROS 会把 `90` 判成 INTEGER，而声明是 DOUBLE，
> 启动即 `InvalidParameterTypeException`（这条是实测踩出来的）。

---

## 3. 存图：走 slam_toolbox 自带服务（duojin01 的做法）

`nodes/save_map_node.py` 复刻 `duojin01_slam_tools/save_map_client_node.cpp` 的语义：

```
解析 output_dir → map_name=auto 时取"最大数字名 + 1" → 等 /slam_toolbox/save_map
  → 用**绝对路径前缀**当 name 发请求 → 检查返回码（0=SUCCESS / 1=还没收到地图 / 255=未定义失败）
  → 校验 .pgm/.yaml 落盘 → 打印地图统计
```

leisaac 的 `slam_toolbox.yaml` 里本来就开着 `use_map_saver: true`，服务是现成的（已实测）。

返回码被翻译成人话，实测输出：

```
[leisaac_save_map]: 等 /slam_toolbox/save_map 服务 ...（超时 8s）
[leisaac_save_map]: 保存地图到 /work/scripts/nav2_config/maps/2.{pgm,yaml} ...
[leisaac_save_map]: ❌ 存图失败：还没收到地图（slam_toolbox 起来了吗？机器人走过一圈了吗？）（返回码 1）
```

对比原来的 `map_saver_cli` 路线：它自己订阅 `/map`、QoS 要猜，slam 还没出图时只会报
`Failed to spin map subscription`，看不出到底哪儿不对。

---

## 4. 地图自动挑选 / 自动命名（`map_paths.py`）

完全照搬 duojin01 的 `pick_latest_map_yaml`：

- **`map` 留空**：优先取**文件名是纯数字**里最大的（`1.yaml` > `0.yaml`）；
  没有数字图才取**最近修改**的那张 → `navigation.launch.py` 不用带路径。
- **跳过 `_` / `.` 开头的 yaml**：它们是 scratch / 隐藏文件（例如自测留下的 `_selftest.yaml`）。
- **`map_name:=auto`**：取"最大数字名 + 1"，从 `0` 开始，不覆盖历史地图。
- **`map` 可以只给名字**：`map:=arena` → `maps/arena.yaml`（自动补 `.yaml`/`.yml` 后缀）。
- 传了路径就按路径用；相对路径依次尝试「相对 `nav2_config/`」「相对 `maps/`」「相对仓库根」。

### 当前实际状态（2026-09-18 校核）

`maps/` 里现在有 **4 套图**（曾经清理到只剩两张，之后又跑过建图，数字图又出现了）：

| 文件 | 尺寸 | 空闲占比 | 说明 |
|---|---|---|---|
| `0.pgm/yaml` | 134×101 = 4.02×3.03 m | 86.9% | 数字名（自动流程生成的**场地图**） |
| `1.pgm/yaml` | 134×101 = 4.02×3.03 m | 79.1% | 数字名（同上，较新） |
| `arena.pgm/yaml` | 134×101 = 4.02×3.03 m | 77.9% | **M3 验证过的场地图**；`tasks/smart_factory/README.md` 引用了它 |
| `kitchen.pgm/yaml` | 433×536 ≈ 13×16 m | 2.0% | 旧厨房图；未改动的 `reproduce/ros2_navigation.sh:19` 把它写成默认值 |

⚠️ **因为数字名优先，`map` 留空现在会选中 `1.yaml`，而不是 `arena.yaml`**：

| 调用 | 结果 |
|---|---|
| `map` 留空 | **`1.yaml`**（数字名最大者） |
| `map:=arena` / `map:=kitchen` | 各自解析成功 |
| `map_name:=auto`（存图） | 下一个名字 = **`2`** |

> 想确保跑到**场地那张图**，请**显式写 `map:=arena`**（或 `map:=maps/arena.yaml`），
> 不要依赖"留空自动挑"。`navigation.launch.py` 每次都会打印
> `[nav] 使用地图: <绝对路径>`，发目标前先看这一行。

---

## 5. 新旧入口对照

| 新入口（ops/） | 旧方式 | 备注 |
|---|---|---|
| `ops/teleop.launch.py` | `reproduce/ros2_keyboard_container.sh` | 桥接逻辑改为 include `bridge.launch.py` |
| `ops/mapping.launch.py [auto:=true]` | `reproduce/ros2_mapping.sh [--auto]` | **自带键盘**；复用 `nav2_config/mapping.launch.py`（slam+RViz） |
| `ops/navigation.launch.py [goal:=...]` | `reproduce/ros2_navigation.sh [--auto X Y YAW]` | 复用 `nav2_config/navigation.launch.py`（Nav2） |
| `ops/save_map.launch.py` | `reproduce/ros2_save_map.sh` | **改走 slam_toolbox 服务** + 自动递增命名 |
| `ops/reset_sim.launch.py` | `reproduce/ros2_reset_sim.sh` | 逻辑一致 |
| `ops/auto_drive.launch.py` | `reproduce/ros2_auto_drive.sh` | heredoc 提成了正式节点 `nodes/auto_drive_node.py` |
| `ops/navigation.launch.py goal:=...` | `reproduce/ros2_send_goal.sh X Y YAW` | 不用再开第二个终端 |
| `bridge.launch.py` | （三个脚本各自内嵌一份） | 唯一化 |
| （宿主机）`reproduce/ros2_chassis_teleop.sh` | 同 | **未改动**，仿真仍由它启动 |

文件布局：

```
scripts/nav2_config/
  bridge.launch.py         公共组件：唯一一处起桥接
  launch_utils.py          ops/*.launch.py 共用小工具（含键盘进程、路径推导）
  map_paths.py             地图自动挑选 / 自动命名
  mapping.launch.py        ← 原有，未改（slam + RViz 组件）
  navigation.launch.py     ← 原有，未改（Nav2 组件）
  nodes/                   auto_drive_node.py / reset_sim_node.py
                           goal_client_node.py / save_map_node.py
  ops/                     6 个入口 launch（本目录 README 描述的对象）
```

---

## 6. 两个非显而易见的设计（都是实测踩出来的）

### ① 键盘必须用 `< /dev/tty`，不能用 `Node(...)`

ROS2 Humble 的 launch 用 asyncio 的 subprocess 起子进程，**stdin 默认是新建的 pipe**：
`osrf_pycommon/.../async_execute_process_asyncio/impl.py` 里
`_async_execute_process_pty = _async_execute_process_nopty`，而 nopty 不传 `stdin`，
于是吃 asyncio 的默认 `stdin=PIPE`。所以无论 `emulate_tty` 设 True 还是 False，
launch 起的子进程 `sys.stdin.isatty()` **都是 False**（实测两种都是 False）。
而 `teleop_twist_keyboard` 第 121 行会 `termios.tcgetattr(sys.stdin)` → 直接
`termios.error: (25, 'Inappropriate ioctl for device')` 挂掉。

绕法（实现在 `launch_utils.keyboard_process()`，teleop 与 mapping 两个入口共用）：
让子进程自己重开控制终端 —— `ExecuteProcess(cmd=["bash","-c","exec ros2 run ... < /dev/tty"])`，
实测 `isatty()` 变 True，键位图正常打印。

**因此只要用到键盘（`ops/teleop.launch.py`、`ops/mapping.launch.py` 默认的 `teleop:=true`），
容器就必须用 `docker run -it`**（`-i` 不够，`-t` 才会给容器分配 tty）。

### ② 发目标点必须等 `bt_navigator` 生命周期 ACTIVE

只等 `/navigate_to_pose` **动作服务器出现**是不够的：动作服务器在 `bt_navigator`
节点一启动就存在，那时它还只是 inactive，目标会被**直接拒绝**
（实测：launch 起来 3 秒就发 → 立刻 "目标点被拒绝"）。
原来的 `ros2_navigation.sh` 用 `ros2 lifecycle get /bt_navigator` 轮询到 active 才发；
`nodes/goal_client_node.py` 用等价的 `/bt_navigator/get_state` 服务轮询（纯 rclpy，不依赖 CLI）。

### ③ RViz 必须 `--gpus all`（**软件渲染也救不了**）

容器里 RViz 报：

```
RenderingAPIException: Invalid parentWindowHandle (wrong server or screen) in GLXWindow::create
... Unable to create the rendering window after 100 tries
```

**根因是容器没有 GPU 设备节点**，不是 X 授权问题。实测（2026-09-16）：

| 检查 | 无 `--gpus all` | 有 `--gpus all` |
|---|---|---|
| `docker inspect` 的 `DeviceRequests` | `[]` | 有 nvidia 请求 |
| 容器内 `/dev/dri` | **不存在** | `card1` / `renderD128` |
| 容器内 `nvidia-smi -L` | `Permission denied` | 看到 RTX 5070 Ti |
| 容器内 `xdpyinfo -display :0` | **成功** ← 会误导人 | 成功 |
| RViz | GLX 建窗失败 100 次后崩溃 | **`OpenGl version: 4.6 (GLSL 4.6)`** ✅ |

★ **`xdpyinfo` 能连上是假象**：`--network host` 共享了网络命名空间，XWayland 的
**abstract socket** 恰好可达，所以 X 连接没问题、只有 GLX 渲染建不起来 ——
报错会「晚一步」出现在渲染阶段，容易被误判成 X 授权问题。
★ **别指望软件渲染绕过**：试过 duojin01 那套
（`LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe QT_OPENGL=software LIBGL_DRI3_DISABLE=1`），
镜像里 mesa DRI 也装了（`libgl1-mesa-dri` / `kms_swrast_dri.so`），**依然报 `Unable to create glx visual`** ——
因为 **llvmpipe 自己也要 `/dev/dri`**。所以软件渲染方案在本项目**无效**，只能 `--gpus all`。

正确写法（三样缺一不可）：

```bash
docker run -it --rm --name leisaac-nav --network host --gpus all \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v <仓库>:/work -w /work ros2-humble-dev:latest bash
```

⚠️ 镜像是**烤死了 `DISPLAY=:0`** 的 —— 宿主机 DISPLAY 恰好也是 `:0` 才对得上，
所以永远显式写 `-e DISPLAY=$DISPLAY`，别靠镜像里的默认值。

---

## 7. 验证状态（如实说明）

**已在容器内（`ros2-humble-dev:latest`，**没有**跑 Isaac Sim）验证 ✅**

- 日志组合与参数：6 个入口 + `bridge.launch.py` 的 `--show-args` 全部正常；`bridge:=true/false` 开关
- **建图入口自带键盘**：`mapping.launch.py`（`teleop` 默认 true，`-it`）→ 一条命令起
  bridge + slam + RViz + 键盘，键盘 `Moving around:` 键位图正常打印 ✅
- **`auto`/`teleop` 互斥**：`auto:=true teleop:=true` → 键盘进程出现 **0** 次，只有 bridge+slam+auto_drive ✅
- **存图走 slam 服务**：找到 `/slam_toolbox/save_map`、自动命名 `2`、返回码 1 被翻译成
  "还没收到地图（机器人走过一圈了吗？）" ✅
- **串行事件链 + 自动退出**：`auto_drive`（15s 超时）退出 → `save_map` 退出 → `reset_sim` 退出
  → 整个 launch 自行退出、容器 `Exited (0)` ✅
  （实测时序：t=0 bridge+slam+auto_drive，t=15s save_map，t=30s reset_sim，t=32s 全退）
  ★ 中途修过一次：原来把 save/reset 一起塞进 `auto_drive` 的 `on_exit`，两者是**并行**起来的
  （实测 pid 60/62 同时启动）—— 存图还没落盘就归位，slam 会收到位姿跳变。现在是真正的串行链。
- **导航编排**：`reset_sim` → 3 秒后 Nav2 全部节点起来 → `goal_client` 等 bt_navigator active ✅
- **地图挑选/命名**：数字名优先规则（duojin01 原样）；`map:=arena`/`kitchen` 都能解析、
  下一个自动名递增 ✅
- **删图（历史动作）**：当时删掉 `0`/`1`/`_selftest` 共 6 个文件（已核对无任何引用），
  保留 `arena`（M3 验证过）与 `kitchen`（旧脚本默认值）✅
  ⚠️ **但之后又跑过建图，`0`/`1` 已经重新出现**（见第 4 节"当前实际状态"）——
  所以现在 `map` 留空会选中 `1.yaml`，**要用场地图请显式 `map:=arena`**。
- 报错路径：`goal` 格式写错、浮点参数写成整数、存图等不到服务 ✅

**未验证（需要真的起 Isaac Sim + 桥接）⚠️**

- 遥控实际驱动底盘、建图产出完整地图、Nav2 真正走到目标点
- RViz 在容器里的显示（需要 `--gpus all` + `xhost`）
- 新存图路径**成功**的分支（实测到的是"还没收到地图"那一支；成功分支需要 slam 真出图）
- 建议第一次用按第 1 节的 ② → ③ → ⑥ 各跑一遍确认

> 补记（2026-09-16）：上述"遥控/建图/导航"三项**后来都实测通过了** ——
> 场地建图出图 4.02×3.03 m、空闲 86.9%；导航两次 SUCCEEDED，终点误差 3.0 cm / 1.6 cm。
> 详见 `../HANDOFF.md` §9 和操作指南 §3 / §4（发布包 `README.md`；开发机 `README1.md`）。

---

## 8. 配套的两件事

### 8.1 宿主机侧的仿真入口：**只有一个模式**（2026-09-18 起）

容器侧这些 launch 都假设"宿主机已经起了一个带桥接的仿真"。那个入口
（`reproduce/ros2_chassis_teleop.sh`）现在**不再分模式** —— 所有操控通道同时在线：

| 通道 | 怎么打开 | 谁在驱动 |
|---|---|---|
| 底盘 | 本目录的键控 / Nav2 / 任何 `/cmd_vel` 发布者 | ROS2，**随时可用**（默认不锁） |
| 上肢·遥操 | 主手使能 / 键盘按 `B` | 遥操设备 |
| 上肢·策略 | `--enable_policy` | 策略（人按 `B` 接管，`R`/`N` 后交回） |
| 上肢·保持 | 谁都没接管时 | 自动保持位姿 |
| 录制 | `--record` | `R` = 失败重录，`N` = 成功保存 |

所以**本目录的 ①遥控 / ②建图 / ⑤导航 与上肢遥操/数采可以在同一个仿真里同时用**，
不再有"遥操时键盘无效"这回事。`--mode nav|teleop` 只是兼容别名（会打印废弃提示）；
真要把底盘锁住加 `--lock_chassis`。

**冲突检测**（脚本之间互相感知，见操作指南 §2.1.1）：
第二个仿真会被锁文件 + 端口预检拦住；桥接同一时刻只服务一个 ROS2 客户端（第二个被明确拒绝）；
`/cmd_vel` 上多于一个发布者时，本目录起的桥接客户端会警告并列出节点名。

### 8.2 一键停止所有相关进程

```bash
bash reproduce/stop_all.sh              # 列出要停谁 → 确认 → SIGTERM → 5s → SIGKILL
bash reproduce/stop_all.sh --status     # 只看有什么在跑（排查"是不是还有残留"最有用）
bash reproduce/stop_all.sh -n           # dry-run
bash reproduce/stop_all.sh -y           # 不问直接停
bash reproduce/stop_all.sh --containers-stopped   # 连已退出的容器也删
```

覆盖容器侧的东西（`ros2 launch`/`ros2 run`/`rviz2`/`slam_toolbox`/nav2 各节点/
`teleop_twist_keyboard`/`ros2_leisaac_bridge.py`）、宿主机侧的 Isaac Sim 与各仿真入口脚本、
以及 `ros2 daemon`。**只按明确的脚本名匹配并排除自身父进程链**，不会误伤终端/桌面。

> 它在容器侧**之前**先杀宿主机进程会被一起带走吗？——不会：脚本杀的是宿主机的进程和 docker **容器**，
> 容器里的节点随容器 stop 一起结束。

### 8.3 全链路操作指南

从底盘建图/导航 → 上肢遥操/数采 → HDF5→LeRobot 转换 → ACT 训练 → 推理，
见 **操作指南**（发布包 `README.md` / 开发机 `README1.md`；含每步命令、坐标对照表、避坑清单、能力边界）。
