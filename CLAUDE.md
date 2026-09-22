# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 安全规则（必须遵守）

1. **严禁删除本地文件**：不得执行任何可能删除或覆盖本地文件的指令（`rm`、`git rm`、`git reset --hard`、`git clean`、`git stash drop`、重定向覆盖 `>` 等）。即使是为了修复问题，也必须先向用户说明哪些文件会被删除/修改，征得同意后再执行。
2. **严禁推送大文件到 GitHub**：不得将 USD/USDZ/STL/STEP/zip/png/jpg/hdr/fbx 等二进制资源文件推送到 GitHub 仓库。仅推送代码（`.py`、`.cpp`、`.hpp`）、配置（`.yaml`、`.json`、`.toml`、`.xml`、`.xacro`、`.urdf`）、文档（`.md`）和启动脚本（`.launch.py`、`.sh`）。大文件通过 Git LFS 管理，但推送前需确认 LFS 对象完整。
3. 涉及破坏性 git 操作（`--force` push、删除分支等）需要先征得用户确认。

## ★ 先读这一节：本仓库当前实际环境（2026-09 起）

> 本文下面的 Project Overview / Prerequisites / Build & Install 是**上游 LeIsaac 原版的说明**，
> 描述的是 **Isaac Sim 5.1.0 + IsaacLab 2.3.0** 那套栈。**那套栈在本机已经不可用、也已放弃**，
> 照它安装会装出一个跑不起来的组合。
>
> **本仓库现在实际跑的是：**
>
> | 项 | 值 |
> |---|---|
> | 仿真 | **Isaac Sim 6.0.1 standalone**（解压版，用自带的 `python.sh`，Python 3.12） |
> | 仿真框架 | **IsaacLab 3.0.0**（git submodule `dependencies/IsaacLab`，`VERSION=3.0.0`） |
> | 学习环境 | conda `leisaac-lerobot-sim`（lerobot 0.4.2 + torch 2.7.1+cu128） |
> | ROS2 | **Humble**（Ubuntu 22.04 原生，或 `ros2-humble-dev` 容器） |
> | 默认任务 | `LeIsaac-SmartFactory-v0`（跑厨房要 `--task LeIsaac-LeRobot-Kitchen-v0`） |
>
> **为什么从 5.1.0 迁到 6.0.1**：5.1.0 的 RTX 渲染器在 RTX 5070 Ti（Blackwell sm_120）上必崩
> （驱动 595 段错误 / 驱动 580 时 `vkCreateRayTracingPipelinesKHR` 失败），属软硬件代际不兼容，
> 配置无法解决；5.1.0 只能 headless。迁移记录见 `docs_archive/MIGRATION_ISAACSIM6.md`。
>
> **权威文档（按优先级）**：
> 1. 操作指南（**唯一权威**）：开发机 = `README1.md`（宿主机+Docker ROS2）；
   发布包/目标机 = `README.md`（= 原生 Humble 版）：起仿真 → 建图 → 2D Goal Pose 导航 → 遥操 → 数采 → 转换 → 训练 → 推理
> 2. `../HANDOFF.md` —— 历史交接与全部踩坑（**最权威**的项目整体说明）
> 3. `../环境依赖查证_Ubuntu22.04_Humble.md` —— 目标机依赖查证
> 4. `scripts/nav2_config/ops/README.md` —— ROS2 launch 细节与完整参数表
> 5. `source/leisaac/leisaac/tasks/smart_factory/README.md` —— 场地本身
>
> **过时文档已归档到 `docs_archive/`**（旧 `OPERATIONS.md`、`START_HERE.md`、`PROJECT_README.md`、
> `README_TECHNICAL.md`、`OPERATIONS_NATIVE_HUMBLE.md`、`reproduce_README.md`、
> `MIGRATION_ISAACSIM6.md`、上游 doc 站点）—— 那些内容基于旧的 5.1.0/2.3.0 栈，只作历史参考，
> **遇到冲突以操作指南（`README1.md` / 发布包 `README.md`）与 HANDOFF.md 为准**。归档清单见 `docs_archive/README.md`。

## Project Overview

LeIsaac provides teleoperation in [IsaacLab](https://isaac-sim.github.io/IsaacLab/main/index.html) using the SO101 leader arm (from [LeRobot](https://github.com/huggingface/lerobot)), plus data collection, data conversion to LeRobot format, and policy inference (GR00T N1.5/N1.6, LeRobot, OpenPI). Upstream runs inside NVIDIA Isaac Sim 5.1.0; **this repository runs Isaac Sim 6.0.1 + IsaacLab 3.0.0** (see the section above).

## Prerequisites

> ⚠️ 以下是**上游原版**的前提条件（5.1.0 栈）。本仓库用 6.0.1 standalone，**不要**拉 5.1.0 镜像。

- **Isaac Sim 5.1.0**（上游原版）— 本仓库改用 **6.0.1 standalone**，路径由 `ISAACSIM_DIR` 或 `reproduce/local.env` 指定
- **IsaacLab 2.3.0**（上游原版）— 本仓库 submodule 已切到 **3.0.0**
- Python ≥ 3.10（6.0.1 自带 3.12）

## Build & Install

```bash
# Install IsaacLab from submodule
cd dependencies/IsaacLab
ln -s ../isaac-sim _isaac_sim   # symlink to Isaac Sim installation
./isaaclab.sh --install

# Install LeIsaac (editable)
pip install -e source/leisaac

# Optional dependency groups (see pyproject.toml):
pip install -e "source/leisaac[gr00t,remote,lerobot,openpi]"
```

## Code Architecture

The package lives at `source/leisaac/leisaac/` and is registered as a Gymnasium environment extension to IsaacLab:

- **`devices/`** — Teleoperation device interfaces. `DeviceBase` defines the abstract device; concrete implementations for keyboard, gamepad, SO101 leader (local + remote via ZMQ), bi-arm SO101, and LeKiwi (leader/keyboard/gamepad). Devices produce action dicts that the env processes via `preprocess_device_action()`.

- **`tasks/`** — Task/environment configurations. Registered at import time via `import_packages()` in `tasks/__init__.py`, which scans subpackages recursively. Each task follows a two-layer structure:
  - `template/` — Base configs (`SingleArmTaskEnvCfg`, `BiArmTaskEnvCfg`, `LeKiwiEnvCfg`) extended by all concrete tasks
  - Concrete tasks (e.g., `pick_orange/`, `lift_cube/`, `fold_cloth/`) each have an `mdp/` subpackage (ManagerBasedRLEnv terms) and optionally a `direct/` subpackage (DirectRLEnv terms)

- **`enhance/`** — Extensions on top of IsaacLab for recording:
  - `envs/` — `RecorderEnhanceDirectRLEnv` and `RecorderEnhanceManagerBasedRLEnv` inject recorder hooks into the step/reset lifecycle
  - `managers/` — `StreamingRecorderManager` streams episodes to HDF5 incrementally (avoids OOM); `LeRobotRecorderManager` records directly to the LeRobot dataset format
  - `datasets/` — HDF5 and LeRobot dataset file handlers with streaming write support

- **`policy/`** — Policy inference clients. `ZMQServicePolicy` (GR00T N1.5/N1.6), gRPC-based `LeRobotServicePolicyClient`, and `WebsocketServicePolicy` (OpenPI). All inherit from the abstract `Policy` base; `get_action()` returns `torch.Tensor` of shape `(action_horizon, num_envs, action_dim)`.

- **`datagen/`** — Programmatic data generation via state machines (`StateMachineBase`). The state machine drives the env step-by-step (setup → get_action → step → advance → check_success) without a human operator.

- **`assets/`** — USD scene definitions (`scenes/`) and robot articulation configs (`robots/lerobot.py` with `SO101_FOLLOWER_CFG`).

- **`utils/`** — Math utilities (`math_utils.py`), joint-space conversions (`robot_utils.py` — `convert_leisaac_action_to_lerobot` / `convert_lerobot_action_to_leisaac`), domain randomization helpers, and asset resolution.

## Common Commands

> ★ **仿真启动入口只有一个**：`reproduce/ros2_chassis_teleop.sh`（用 `--mode` 切换用途）。
> 下面这些是最初的上游命令，**不是**本仓库对外的入口 —— 日常请用：
>
> | 用途 | 命令 |
> |---|---|
> | 建图 / 验证导航 | `bash reproduce/ros2_chassis_teleop.sh --mode nav [--lidar_stop_distance 0.45]` |
> | 遥操 / 数采 | `bash reproduce/ros2_chassis_teleop.sh --mode teleop [--record --dataset_file ...]` |
> | 验证推理 | `bash reproduce/ros2_chassis_teleop.sh --mode policy --policy_type local-act ...` |
>
> `scripts/environments/teleoperation/teleop_se3_agent.py` 与
> `scripts/evaluation/policy_inference.py` 是**内部实现 / 单一功能工具**
> （后者多了 `--eval_rounds N` 批量评测），只在统一入口覆盖不到的场合直接用。

```bash
# Linting & formatting
pre-commit install            # install git hooks (once per clone)
pre-commit run --all-files    # run all checks manually

# Teleoperation (the main entry point)
python scripts/environments/teleoperation/teleop_se3_agent.py \
    --task LeIsaac-Pick-Orange-SO101-v0 \
    --teleop_device keyboard

# Teleop with recording to HDF5
python scripts/environments/teleoperation/teleop_se3_agent.py \
    --task LeIsaac-Pick-Orange-SO101-v0 \
    --teleop_device so101leader --port /dev/ttyACM0 \
    --record --dataset_file ./datasets/demo.hdf5

# Teleop with LeRobot-format recording
python scripts/environments/teleoperation/teleop_se3_agent.py \
    --task LeIsaac-Pick-Orange-SO101-v0 \
    --teleop_device so101leader --port /dev/ttyACM0 \
    --record --use_lerobot_recorder --lerobot_dataset_repo_id my_org/my_dataset

# Remote teleoperation (leader on another machine)
# On remote machine:
python scripts/environments/teleoperation/so101_joint_state_server.py --port 5556
# On simulation machine:
python scripts/environments/teleoperation/teleop_se3_agent.py \
    --task LeIsaac-Pick-Orange-SO101-v0 \
    --teleop_device so101leader --remote_endpoint tcp://192.168.1.10:5556

# Data conversion: HDF5 → LeRobot Dataset v2
python scripts/convert/isaaclab2lerobot.py \
    --task_name LeIsaac-Pick-Orange-SO101-v0 \
    --hdf5_root ./datasets --repo_id my_org/my_lerobot_dataset

# Data conversion: LeRobot → IsaacLab HDF5
python scripts/convert/lerobot2isaaclab.py \
    --lerobot_dataset_repo_id my_org/my_lerobot_dataset

# Programmatic data generation (state machine)
python scripts/datagen/state_machine/generate.py --task LeIsaac-Pick-Orange-SO101-v0

# Policy inference / evaluation
python scripts/evaluation/policy_inference.py \
    --task LeIsaac-Pick-Orange-SO101-v0 \
    --policy_type gr00tn1.5 --policy_host localhost --policy_port 5555 \
    --policy_language_instruction "Pick the oranges and place them in the plate" \
    --eval_rounds 10
```

## Testing

This project does not have a local test suite (the `tests/` directory is gitignored). Testing is done by running the teleoperation or inference scripts inside the Isaac Sim container. Pre-commit hooks handle formatting and lint enforcement; CI runs `pre-commit` on PRs.

## Key Conventions

- **Task naming**: `LeIsaac-<TaskName>-<RobotType>-v0` (e.g., `LeIsaac-Pick-Orange-SO101-v0`, `LeIsaac-Clean-Toy-Table-BiArm-v0`). The `Direct` suffix (e.g., `LeIsaac-Lift-Cube-Direct-SO101-v0`) denotes a DirectRLEnv task.
- **Action space**: Actions are LeRobot-format joint positions (6 DoF for single arm: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper). Conversion functions handle IsaacLab ↔ LeRobot format.
- **Device action flow**: Device produces dict → `env.cfg.preprocess_device_action(action, device)` → tensor → `env.step()`
- **Configuration**: Uses `@configclass` dataclasses (IsaacLab convention). Task configs extend template base configs and compose scene, observations, actions, events, rewards, and terminations.
- **Black format**: line length 120, `--unstable` flag enabled
- **Editor**: The `.vscode/` directory is checked in for VS Code workspace settings

## Docker Quickstart

```bash
./start_leisaac_docker.sh
```
This mounts the repo into the Isaac Sim container, installs the package editable, and launches keyboard teleop.

## Practical Guides

> ⚠️ **下面 §9–§15 是历史记录**（厨房任务 + 云端训练阶段写的），里面的路径多来自**原作者机器**
> （`/home/anno/...`）或**云端算力平台**（`/root/gpufree-data/...`）。这些经验（DOF 顺序、
> 自碰撞、刚度、PhysX 显存、踩坑表）**依然有效**，但**命令里的路径不要照抄**。
> 当前本机可用的命令见 `README1.md`（开发机指南）；§16/§17 是最新的（本机路径正确）。

### URDF → USD Conversion: Use GUI, Not CLI

The command-line `URDFParseAndImportFile` generates multi-layer USD files that are hard to debug and edit. **Always use the Isaac Sim GUI for URDF→USD conversion.**

```bash
# 本仓库用 Isaac Sim 6.0.1 standalone；兼容库在 reproduce/compat_libs_isaac6
LD_LIBRARY_PATH="$PWD/reproduce/compat_libs_isaac6:${LD_LIBRARY_PATH:-}" \
  "${ISAACSIM_DIR:?先设 ISAACSIM_DIR}/python.sh" scripts/isaac_gui.py
```

Then:
1. `File` → `Import` → file type dropdown select **URDF** → select the `.urdf` file
2. Verify the robot renders correctly (all meshes visible, joints correct)
3. `File` → `Export` → file type select **USD** (or **USDZ** for bundled assets)
4. The exported USD is a single self-contained file — no sublayer path issues.

**Two export formats compared:**

| Format | Output | 网络数据 | 推荐 |
|--------|--------|:---:|:---:|
| USDZ | `main.usdc` + `SubUSDs/` (86MB base含全部网格) | 单zip | 路径100%可靠 |
| Direct USD | `robot.usd` + `configuration/` (86MB base) | 分层文件 | 名更干净 |

**Important:** After export, set `defaultPrim` on the USD:
```python
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
from pxr import Usd
stage = Usd.Stage.Open('path/to/robot.usd')
stage.SetDefaultPrim(stage.GetPrimAtPath('/robot_name'))
stage.GetRootLayer().Save()
app.close()
```

### Modifying Object Positions in Existing Scenes

Kitchen scene objects (oranges, plates) have their positions defined in `scene.usd` as scene-level transforms. **Do not use binary file editing** — use the USD API:

```python
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.Open('path/to/scene.usd')

for prim in stage.TraverseAll():
    if prim.GetName() not in ['Orange002', 'Orange003']:
        continue
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if 'translate' in op.GetOpName() and abs(op.Get()[0]) > 1.0:
            # abs(X) > 1.0 identifies scene-level translate (vs local mesh offset at X=0)
            v = op.Get()
            op.Set(Gf.Vec3d(v[0], v[1], new_z_value))
            break

stage.GetRootLayer().Save()
app.close()
```

**Key insight:** USDC objects may have MULTIPLE `translate` attributes at different composition levels:
- Scene-level (X≈2.x, Y≈-0.x) — the one that controls placement in the world
- Local offset (X=0, Y=0) — mesh-internal offset, do not modify

Always check with `abs(op.Get()[0]) > 1.0` to identify the scene-level translate.

### Adjusting Robot Position in LeIsaac

Set in task config `__post_init__`. Position is world coordinates, rotation is quaternion (w, x, y, z):

```python
def __post_init__(self):
    self.scene.robot.init_state.pos = (x, y, z)        # meters
    self.scene.robot.init_state.rot = (w, x, y, z)     # quaternion
```

To find desired coordinates: open the task in GUI, pause physics, drag robot with W key, read Transform from Property panel. Note the convention: `rot` format is `(w, x, y, z)`.

### Environment Setup Notes

- **别再用旧的 `/home/anno/...` 路径**（那是原作者机器上的 conda `leisaac` 环境，本机已失效）。
  当前仿真用 **Isaac Sim 6.0.1 自带的 `python.sh`**（Python 3.12，不走 conda）。
- **跑仿真前先 `unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE VIRTUAL_ENV`** —— 否则
  IsaacLab 3.0 会报 `Downloaded Isaac Sim packages cannot be combined with a Python virtual environment`。
  `reproduce/ros2_chassis_teleop.sh` / `reproduce/launch_gui.sh` 已自动处理。
- **兼容库**：Ubuntu 26.04 上需要 `reproduce/compat_libs_isaac6/`（`libxml2.so.2` / `libicu*.so.74` 软链）
  前置到 `LD_LIBRARY_PATH`；**Ubuntu 22.04 自带 `libxml2.so.2`，这些软链会悬空，可不用**。
- **学习环境**用 conda `leisaac-lerobot-sim`（Python 3.12 + lerobot 0.4.2 + torch 2.7.1+cu128），
  见下面 §16；旧的 conda 环境 `leisaac`（5.1.0）**已无法跑本项目仿真**。
- 如果包报 SIGBUS，用 conda（不是 pip）重装以保 ABI 一致：`conda install -c conda-forge pyarrow datasets`

### 自定义多关节机器人集成指南（双臂移动机器人实战）

#### 1. 单 Articulation 多关节的 DOF 顺序

自定义机器人往往将全部关节放在一个 articulation 中（如 13 DOF：腰 1 + 左臂 6 + 右臂 6）。PhysX 的 DOF 顺序是**交替排列**的（按 USD 关节出现顺序），不是分组排列的：

```
DOF  0: waist_lift
DOF  1: left_shoulder_pan     ← 不是全部左臂连续！
DOF  2: right_shoulder_pan
DOF  3: left_shoulder_lift
DOF  4: right_shoulder_lift
DOF  5: left_elbow_flex
DOF  6: right_elbow_flex
...
```

因此 `JointPositionActionCfg` 必须设置 **`preserve_order=True`**，确保 action tensor 索引按指定顺序映射到关节，而不是按 articulation 内部顺序：

```python
self.actions.arm_action = JointPositionActionCfg(
    asset_name="robot",
    joint_names=[  # 按左臂全部、右臂全部分组排列
        "left_shoulder_pan", "left_shoulder_lift", "left_elbow_flex",
        "left_wrist_flex", "left_wrist_roll", "left_gripper",
        "right_shoulder_pan", "right_shoulder_lift", "right_elbow_flex",
        "right_wrist_flex", "right_wrist_roll", "right_gripper",
    ],
    scale=1.0, preserve_order=True,  # 关键！
)
```

**验证方法**：运行 `scripts/debug_actuators.py` 查看 articulation 的 `find_joints()` 返回的 `joint_ids` 是否与预期一致。

#### 2. 双臂自碰撞（最常见的抖动根因）

同一 articulation 内的两个机械臂在零位时**碰撞网格可能重叠**。PhysX 碰撞力远超 PD 控制器 effort limit（`effort_limit_sim=10`），导致关节被推往反方向，表现为：
- 特定关节不受遥控控制
- 往同一方向偏移（双臂都往左偏）
- 来回抖动

**诊断**：比较关闭/开启自碰撞时的关节响应：
```python
# 关闭自碰撞
env_cfg.scene.robot.spawn.articulation_props.enabled_self_collisions = False
```

**修复**：在 `LEROBOT_ROBOT_CFG` 中设置 `enabled_self_collisions=False`。注意不影响标准 SO101 独立 articulation 的双臂配置。

#### 3. 腰部棱柱关节刚度

移动机器人的腰部（prismatic joint）支撑整个上半身+双臂重量，需要**远大于旋转关节的刚度**：

| 参数 | 错误值 | 正确值 | 说明 |
|------|--------|--------|------|
| stiffness | 17.8 | 500 | 棱柱关节线性刚度 (N/m)，旋转关节 17.8 Nm/rad |
| damping | 0.6 | 100 | 需足够大的阻尼防止震荡 |
| effort_limit | 10 | 500 | 需足够支撑上半身重量 |

刚度太低 → 腰部下沉 → 肩关节轴倾斜 → 重力扭矩导致 shoulder_pan 不受控。

#### 4. 遥操脚本跳过 LeRobot 任务的动作覆盖

`teleop_se3_agent.py` 默认调用 `env_cfg.use_teleop_device()` 覆盖动作配置。LeRobot 任务在 `__post_init__` 已自定义动作，需跳过：

```python
# teleop_se3_agent.py 中添加：
if "LeRobot" not in args_cli.task:
    env_cfg.use_teleop_device(args_cli.teleop_device)
```

#### 5. USD 导出后必须设 defaultPrim

GUI 导出 USDZ 格式后，`main.usdc` 的 `defaultPrim` 可能未设置，导致 IsaacLab 无法正确解析 articulation。修复：

```python
from pxr import Usd
stage = Usd.Stage.Open('path/to/main.usdc')
stage.SetDefaultPrim(stage.GetPrimAtPath('/robot_name'))
stage.GetRootLayer().Save()
```

#### 6. 数据录制维度

录制时不包括腰部关节，仅双臂 12 关节。在 `build_lerobot_frame` 中跳过 `waist_lift`（DOF 0）：

```python
state_12 = raw_state[1:]  # skip waist_lift (DOF 0)
```

#### 7. 调试工具脚本

| 脚本 | 用途 |
|------|------|
| `scripts/debug_actuators.py` | 查看 articulation DOF 顺序、action term 的 joint_ids、default_joint_pos |
| `scripts/test_single_arm.py` | 逐一测试 12 个关节是否按 target 方向移动 |
| `scripts/test_nogravity.py` | 隔离测试重力 vs 自碰撞对关节的影响 |
| `scripts/compare_usd_joints.py` | 对比官方和自定义 USD 的关节 axis/limit |
| `scripts/deep_compare.py` | 对比关节世界空间轴方向（遍历父链计算） |
| `scripts/convert/isaaclab2lerobotv3.py` | HDF5 → LeRobot v3 数据集转换（改过 headless/recorders/actions 跳过） |

#### 8. 踩坑总结

| 问题 | 根因 | 修复 |
|------|------|------|
| shoulder_pan 不受控、抖动 | 双臂自碰撞 | `enabled_self_collisions=False` |
| 关节映射错乱 | DOF 交替排列 + `preserve_order=False` | `preserve_order=True` |
| 腰部导致手臂抖动 | 棱柱关节刚度太低 | stiffness 50→500 |
| LeRobot 任务无法遥操 | `use_teleop_device` 覆盖动作 | skip LeRobot task |
| 新导出的 USD 无法加载 | defaultPrim 未设 | GUI 导出后用脚本设 defaultPrim |
| 橙子穿透桌面 | 场景整体加载无碰撞 + 橙子无碰撞 mesh | `parse_usd_and_create_subassets` 提取为独立刚体 |
| 转换脚本 CUDA OOM (8GB) | `headless=True+enable_cameras=True` 触发 offscreen rendering + PhysX 碰撞对缓冲过大 | `headless=False` + 降 `gpu_found_lost_aggregate_pairs_capacity` |
| 转换时 BlockingIOError | 默认 recorder 创建 HDF5 文件锁冲突 | 加 `env_cfg.recorders = None` |
| 转换时 KeyError: 'actions' | HDF5 中有中断录制的残缺 episode | 跳过无 actions 字段的 episode |

### 9. 双臂碰撞体方案（官方 SO101 + 自建躯干）

**最终成功方案**: 用两个官方 `SO101_FOLLOWER_CFG` articulation + 一个静态躯干视觉模型。

**为什么**: 自定义 URDF→USD 的碰撞 mesh 路径在 USDZ 实例代理中无法被 PhysX 识别。官方 SO101 USD 的碰撞 100% 正常。

**架构**:
```python
# 场景 = 厨房 + 左官方臂 + 右官方臂 + 躯干模型
left_arm = SO101_FOLLOWER_CFG   # 碰撞正常
right_arm = SO101_FOLLOWER_CFG  # 碰撞正常
body = AssetBaseCfg(...)        # 纯视觉，无碰撞
```

**躯干 USD 制作**:
1. 从 URDF 中删除所有臂相关的 link、joint、transmission
2. GUI 导入裁剪后的 URDF → 导出 USDZ
3. 用 SubUSDs 中的 base 文件（含所有 mesh）作为躯干视觉模型

**机械臂位置** (厨房场景，2026-07-02 最终值):
```python
# 左臂
self.scene.left_arm.init_state.pos = (2.14527, -0.76399, 0.54)
self.scene.left_arm.init_state.rot = (0.0, 0.0, 0.0, 1.0)
# 右臂
self.scene.right_arm.init_state.pos = (2.48527, -0.79399, 0.54)
self.scene.right_arm.init_state.rot = (0.0, 0.0, 0.0, 1.0)
# 躯干
body.init_state.pos = (2.29177, -0.84691, 0.01)
body.init_state.rot = (0.7323, 0.0, 0.0, 0.6810)
```

**双臂间距避碰**: 左右臂 Y 间距需 ≥ 0.03m 以上避免碰撞网格重叠。

**之后机械臂随躯干移动**: 
```python
# 底盘移动时，每帧更新手臂 articulation root pose
chassis_pos, chassis_rot = get_chassis_world_pose()
left_arm.write_root_pose_to_sim(chassis_pos + left_offset, chassis_rot)
right_arm.write_root_pose_to_sim(chassis_pos + right_offset, chassis_rot)
```

**当前配置关键参数**:
- 任务: `LeIsaac-LeRobot-Kitchen-v0`，基于 `BiArmTaskEnvCfg`
- 动作: 标准 `init_action_cfg(device="bi-so101leader")`（4-term 双臂结构）
- 录制: HDF5 → `isaaclab2lerobotv3.py` → LeRobot v3
- 12 关节（不含腰），双臂各 6
- 躯干无物理碰撞，纯视觉

### 10. HDF5 数据录制与 LeRobot v3 转换（完整流程）

**录制**:
```bash
python scripts/environments/teleoperation/teleop_se3_agent.py \
    --task LeIsaac-LeRobot-Kitchen-v0 \
    --teleop_device bi-so101leader \
    --left_arm_port /dev/ttyACM0 --right_arm_port /dev/ttyACM1 \
    --enable_cameras \
    --record \
    --dataset_file ./datasets/kitchen_biarm.hdf5 \
    --num_demos 35
```
- `R` 开始录制 → 操作 → `N` 标记成功并保存 → `R` 下一条
- 输出为 `./datasets/kitchen_biarm.hdf5`（17GB / 35条）

**转换为 LeRobot v3**（★ 现在用**离线转换器**，只依赖 h5py + lerobot，不需要 isaaclab、不用起仿真）:
```bash
/home/vedal/miniconda3/envs/leisaac-lerobot-sim/bin/python scripts/convert/hdf5_to_lerobot_v3.py \
    --hdf5 datasets/kitchen_biarm.hdf5 \
    --repo_id kitchen_biarm_v1 \
    --root datasets/lerobot \
    --task "Pick oranges with the leRobot bi-arm manipulator." \
    --fps 30
```
- 旧的 `scripts/convert/isaaclab2lerobotv3.py` 仍保留，但它**要求同一个解释器里同时有 lerobot 和 isaaclab**
  （会 `gym.make()` 起仿真）—— 那个组合在本机凑不出来，所以日常用上面的离线版。
- 数据集输出到 `--root/<repo_id>/`（`meta/info.json` 里 `codebase_version=v3.0`）；重转前先清掉旧目录

**转换常见问题**:
| 问题 | 原因 | 修复 |
|------|------|------|
| CUDA OOM (640MB 分配失败) | `headless=True + enable_cameras=True` 触发 offscreen rendering | 改 `headless=False` |
| BlockingIOError (文件锁) | 默认 recorder 创建 HDF5 文件冲突 | 转换脚本加 `env_cfg.recorders = None` |
| KeyError: 'actions' | HDF5 中有录制时中断的残缺 episode | 脚本跳过无 actions 字段的 episode |
| KeyError: 'episode_X' | pyc 缓存不一致 | 清理 `__pycache__` |

### 11. 复杂厨房场景的 GPU 显存优化（8GB 显卡）

**问题根因**: 厨房 USD 内建数百个 PhysX 碰撞体（柜子、抽屉、门），PhysX 默认 `gpu_found_lost_aggregate_pairs_capacity=2^25`（640MB），加上相机 offscreen rendering，8GB 显存不够。

**AppLauncher 经验文件加载机制**（关键知识点）:
```
headless + enable_cameras → isaaclab.python.headless.rendering.kit（最重，offline render targets per camera）
!headless + enable_cameras → isaaclab.python.rendering.kit（GUI 渲染，较轻）
headless + !enable_cameras → isaaclab.python.headless.kit（最轻，无渲染）
```

录制脚本默认 `enable_cameras=False, headless=False`，所以不触发 offscreen rendering。转换脚本原来强制 `enable_cameras=True, headless=True` 导致 OOM。

**修复方案**（在 `lerobot_kitchen_env_cfg.py` 的 `__post_init__` 中）:
```python
# 降低 GPU 物理内存（默认值对复杂厨房场景过高）
self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 2**20   # 默认 2^25(640MB) → 20MB
self.sim.physx.gpu_max_rigid_contact_count = 2**19               # 默认 2^23
```

**转换脚本修改**（`scripts/convert/isaaclab2lerobotv3.py`）:
```python
# headless=True → headless=False（避免加载 headless.rendering.kit）
default_args = {
    "headless": False,
    "enable_cameras": True,
}
# 添加（避免 recorder 文件锁冲突）
env_cfg.recorders = None
```

### 12. 场景物体的碰撞体添加方案（桌子+橙子）

**问题**: 厨房场景作为整块 `AssetBaseCfg` 加载时，桌子和橙子的 PhysX 碰撞不生效。

**方案**: 用 `parse_usd_and_create_subassets` 提取特定物体为独立刚体：
```python
from leisaac.utils.general_assets import parse_usd_and_create_subassets
from leisaac.assets.scenes.kitchen import KITCHEN_WITH_ORANGE_USD_PATH

parse_usd_and_create_subassets(
    KITCHEN_WITH_ORANGE_USD_PATH,
    self,
    specific_name_list=["Table038", "Orange001", "Orange002", "Orange003", "Plate"],
)
# 桌子设为静态碰撞体
if hasattr(self.scene, "Table038"):
    self.scene.Table038.spawn = RigidObjectSpawnerCfg(
        func=self.scene.Table038.spawn.func,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=False),
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    )
```
- 桌子设为静态碰撞体（`rigid_body_enabled=False, collision_enabled=True`）
- 橙子和盘子由 `parse_usd_and_create_subassets` 自动创建为动态刚体

### 13. 机器人位置批量位移方法

当躯干移动到新位置，机械臂按相同 delta 位移：

| 对象 | 旧位置 | 新位置 | Delta |
|------|--------|--------|-------|
| 躯干 | (2.5765, -1.89292, 0.01) | (2.29177, -0.84691, 0.01) | Δx=-0.285, Δy=+1.046 |
| 左臂 | (2.43, -1.81, 0.54) | (2.14527, -0.76399, 0.54) | 同上 |
| 右臂 | (2.77, -1.84, 0.54) | (2.48527, -0.79399, 0.54) | 同上 |

同时更新 `self.viewer.eye` 和 `self.viewer.lookat` 保持视角一致。

### 14. ACT 模型训练（LeRobot）

**数据集要求**:
- LeRobot v3 格式，本地路径结构为 `{root}/{repo_id}/meta/info.json`
- 上传到算力平台后解压，确保目录层级正确

**训练命令**（模板，根据平台路径调整 `--dataset.root`）:
```bash
lerobot-train \
    --dataset.repo_id=kitchen_biarm_v1 \
    --dataset.root=/root/gpufree-data/doulei741wsy/kitchen_biarm_v1 \
    --dataset.streaming=False \
    --dataset.revision=v3.0 \
    --policy.type=act \
    --output_dir=/root/gpufree-data/lerobot/outputs/act/kitchen_biarm_act \
    --job_name=act_kitchen_biarm_v1 \
    --policy.device=cuda \
    --wandb.enable=false \
    --policy.push_to_hub=false \
    --steps=300000 \
    --batch_size=4
```

**关键参数**:
| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `--dataset.repo_id` | 数据集名，对应 root 下的子目录 | `kitchen_biarm_v1` |
| `--dataset.root` | 数据集**所在目录**（直接包含 meta/info.json 的路径） | 必须指向 meta 的父目录 |
| `--policy.type` | 策略类型 | `act` |
| `--batch_size` | 批次大小 | 4（16-20GB）、8（24-32GB） |
| `--steps` | 训练步数 | 33条数据建议 100k-200k |
| `--policy.vision_backbone` | 图像编码器 | `resnet18`（默认） |
| `--policy.use_vae` | ACT VAE | `true`（默认） |

**验证数据集能否被 LeRobot 加载**:
```bash
python3 -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset(repo_id='kitchen_biarm_v1', root='/your/dataset/path')
print(f'episodes={ds.num_episodes}, frames={ds.num_frames}')
"
```

**训练期间监控指标**:
- `loss`: 行为克隆损失，正常从 5-8 降到 1-2
- `grdn`: 梯度范数，反映训练稳定性
- `epch`: epoch 数，33 条数据每个 epoch = 全部过一遍
- `updt_s`: 每步更新时间（< 0.05s 正常）

**本次训练实际数据**:
| 指标 | 值 |
|------|-----|
| 数据集 | 33 episodes, 25,584 frames |
| 相机 | 3 路 320×240（left_wrist, right_wrist, front） |
| 动作/状态 | 12 维（双臂各 6） |
| 模型参数 | 52M |
| 初始 loss | ~8.1 → 2k步降到 ~1.9 |
| 平台 | 算力自由 (gpufree) |

**继续训练（resume）**:
```bash
lerobot-train \
    --config_path=/root/gpufree-data/lerobot/outputs/act/kitchen_biarm_act/checkpoints/180000/pretrained_model/train_config.json \
    --resume=true
```
- 注意：gpufree 版 lerobot 的 `--resume` 必须用 `=true`（小写），不能是 `--resume` 或 `--resume=True`

### 15. 本地 ACT 模型推理（完整管线）

**架构**: gRPC policy server（lerobot 环境，GPU/CPU 推理）+ 仿真 client（leisaac 环境）

**启动**:
```bash
# 终端 1 — policy server（lerobot 环境）
conda activate lerobot
python -m lerobot.async_inference.policy_server --host=127.0.0.1 --port=5555 --fps=30

# 终端 2 — 仿真推理（leisaac 环境）
conda activate leisaac
export LD_LIBRARY_PATH=/home/anno/miniconda3/envs/leisaac/lib:$LD_LIBRARY_PATH
python scripts/evaluation/policy_inference.py \
    --task LeIsaac-LeRobot-Kitchen-v0 --enable_cameras \
    --policy_type lerobot-act --policy_host 127.0.0.1 --policy_port 5555 \
    --policy_checkpoint_path <checkpoint_path> \
    --policy_action_horizon 100 \
    --policy_language_instruction "Pick oranges with the leRobot bi-arm manipulator." \
    --policy_device cuda --eval_rounds 1
```

**已添加的关键修复**:
| 修复 | 文件 | 说明 |
|------|------|------|
| 双臂 12 维动作转换 | `robot_utils.py` | `convert_lerobot_action_to_leisaac` 支持 offset=6 处理右臂 |
| 双臂 state/observation | `service_policy_clients.py` | 添加 `bi-so101leader` 分支，12 DOF |
| pickle 模块路径 | `policy/lerobot/__init__.py` | 匹配服务器端 `lerobot.async_inference.helpers` 路径 |
| `must_go=True` | `service_policy_clients.py` | 防止观测被 `_obs_sanity_checks` 过滤 |
| RemotePolicyConfig | `policy/lerobot/helpers.py` | 添加 `rename_map` 字段兼容 |
| PD 刚度提升 | `lerobot_kitchen_env_cfg.py` | effort_limit 10→50, stiffness 17.8→35, damping 0.6→1.2 |
| PhysX 显存优化 | `lerobot_kitchen_env_cfg.py` | gpu_found_lost_aggregate_pairs 2^25→2^20 |
| protobuf 兼容 | `transport/` | 使用 lerobot 包自带 pb2 文件 |
| `recorders=None` | `isaaclab2lerobotv3.py` | 避免转换脚本文件锁冲突 |
| 跳过无 actions 数据 | `isaaclab2lerobotv3.py` | 跳过 HDF5 残缺 episode |

**SSH OpenSSL 修复**: 系统 ssh 报 `OpenSSL version mismatch` 时，用 conda 安装的 openssh：
```bash
/home/anno/miniconda3/bin/conda install -y -c conda-forge openssh -p /home/anno/miniconda3/envs/leisaac
# 然后使用 /home/anno/miniconda3/envs/leisaac/bin/ssh 和 scp
```

### 16. 推理环境与依赖问题

**★ 本项目专用环境（2026-09-12 起）：`leisaac-lerobot-sim`** = `/home/vedal/miniconda3/envs/leisaac-lerobot-sim`
（Python 3.12 + **lerobot 0.4.2** + torch 2.7.1**+cu128** + grpcio 1.73.1 + draccus 0.10.0）。
实测可加载 `outputs/train/kitchen_biarm_act_300k` 的 checkpoint 并服务动作（自检脚本
`reproduce/check_lerobot_env.py`）。重建步骤与三个坑见 `../HANDOFF.md` §2：
① conda 默认频道要先接受 ToS（用 `--override-channels -c conda-forge` 绕开）；
② **torch 必须 cu128**，cu126 的 wheel 不含 sm_120 内核，RTX 5070 Ti 上会报
"CUDA error: no kernel image is available for execution on the device"；
③ 不要 `conda create --clone` 别人的环境（会继承它的 editable 安装，不独立）。
⚠️ 不要再借用 `lerobot-a1z`（"星海图 A1Z"项目的环境，其 lerobot 是 editable 指向该项目的源码）。

**conda 环境分离**: policy server 必须在 **lerobot** conda 环境运行（lerobot 0.4.2 依赖与 Isaac Sim 冲突），仿真在 **leisaac** 环境运行。两者通过 gRPC (localhost:5555) 通信。**当前实际使用的是 TCP 直连**
（`scripts/evaluation/act_action_server.py` + `policy_inference.py --policy_type local-act`，已验证返回
`(100,12)` 动作块）；gRPC 路线在依赖上已具备（0.4.2 + grpcio），但客户端 pb2 与 0.4.2 是否匹配尚未验证。

**依赖版本兼容**:
- lerobot 0.4.2 要求 `draccus==0.10.0`（不能是 0.11.x，`ChoiceRegistry` API 变了）
- lerobot 0.4.2 要求 `datasets<4.2.0,>=4.0.0`, `deepdiff<9.0.0,>=7.0.1`
- lerobot 的 `huggingface-hub<0.36.0` 与新版 transformers 的 `huggingface-hub>=1.5.0` 冲突 → **只能用 lerobot 环境启动 server**
- protobuf 文件必须与 server 端版本匹配 → 直接复制 lerobot 包的 pb2 文件到 leisaac 项目中

**8GB 显存方案**: 仿真用 GPU，policy server 用 CPU：
```bash
# 推理端加 --policy_device cpu
python scripts/evaluation/policy_inference.py ... --policy_device cpu
```

### 17. 备用方案：TCP 直连跳过 gRPC（**当前实际在用**）

当 gRPC 通信有问题时，可用 `scripts/evaluation/act_action_server.py`（TCP socket 直连，JSON 协议）替代 `lerobot.async_inference.policy_server`：
```bash
# 终端 1: TCP server（用本项目专用环境 leisaac-lerobot-sim；CPU/GPU 都行）
/home/vedal/miniconda3/envs/leisaac-lerobot-sim/bin/python scripts/evaluation/act_action_server.py \
    --checkpoint_path outputs/train/kitchen_biarm_act_300k/checkpoints/300000/pretrained_model \
    --device cuda --port 5556

# 终端 2: 推理（Isaac Sim 的 python.sh，不是 conda 环境）
python scripts/evaluation/policy_inference.py \
    ... --policy_type local-act --policy_host 127.0.0.1 --policy_port 5556
```

### 18. 技术文档

完整的系统架构、数据格式、转换逻辑、训练和推理细节见 `../HANDOFF.md`；
面向本机的可执行操作步骤见 **`README1.md`**。
（旧的技术手册 `README_TECHNICAL.md` 已归档到 `docs_archive/`。）

### 19. 移动底盘集成（duojin01 → Isaac Sim）

#### 架构
将 duojin01 麦克纳姆轮底盘集成到 Isaac Sim 厨房场景中，使双臂随底盘移动。未来通过 ROS2 桥接接入 Nav2 导航。

**当前状态（Phase 1）**：
- 躯干 body 从静态 `AssetBaseCfg` 改为 `RigidObjectCfg`（`kinematic_enabled=True`）
- 底盘位姿通过 `body.write_root_pose_to_sim()` 直接控制
- 机械臂通过 `arm.write_root_pose_to_sim()` 跟随底盘
- 测试脚本：`scripts/test_chassis_move.py`

#### Body 配置
```python
body: RigidObjectCfg = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/Body",
    spawn=sim_utils.UsdFileCfg(
        usd_path="...lerobot_robot_no_arms_base.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            kinematic_enabled=True,  # 运动学模式
        ),
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(2.29, -0.85, 0.01), rot=(0.73, 0, 0, 0.68)),
)
```

#### 机械臂随动
```python
# 获取底盘当前位姿
body_pos = body.data.root_link_pose_w[:, :3]
body_yaw = quaternion_to_yaw(body.data.root_link_pose_w[:, 3:])
# 将臂偏移旋转到世界坐标系
arm_offset_world = rotate_2d(arm_offset_local, body_yaw)
# 更新臂根位姿
arm.write_root_pose_to_sim(body_pos + arm_offset_world + quat)
```

#### 机械臂相对底盘偏移
| 臂 | 局部偏移 (dx, dy, dz) |
|----|----------------------|
| 左臂 | (-0.1465, 0.0829, 0.53) |
| 右臂 | (0.1935, 0.0529, 0.53) |

#### 测试
```bash
python scripts/test_chassis_move.py --task LeIsaac-LeRobot-Kitchen-v0
# W/S: 前进/后退  A/D: 左/右平移  Q/E: 旋转  Shift: 加速
```

#### 下一阶段（Phase 2-3）
- Isaac Sim 侧 TCP 桥接服务（`sim_bridge_server.py`）
- ROS2 侧桥接客户端（`ros2_leisaac_bridge.py`，运行在系统 Python）
- LiDAR 模拟（RayCaster）
- Nav2 + SLAM 导航
- 参考 duojin01 项目：`/home/anno/learning/duojin01-main/`
