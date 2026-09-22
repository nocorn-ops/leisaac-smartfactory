#!/usr/bin/env python
"""ROS2 键盘遥控底盘（sim 侧入口）—— 开窗 + 起 TCP 桥接 + 把 /cmd_vel 作用到底盘。

分工（键盘在 ROS2 侧，不在 Isaac Sim 里）：
    ┌─ 宿主机（本脚本）: Isaac Sim GUI + SimBridgeServer(tcp 5560) + 底盘运动学 ─┐
    │                                                                            │
    └─ Docker(ROS2 Humble, --network host) ─────────────────────────────────────┘
         ros2 run teleop_twist_keyboard teleop_twist_keyboard   → /cmd_vel
         scripts/ros2_leisaac_bridge.py --sim_host 127.0.0.1     → TCP ↔ 仿真

用法（宿主机）::

    # 带窗口（推荐，能看到机器人动）
    bash reproduce/ros2_keyboard_teleop.sh
    # 或手动
    python.sh scripts/environments/ros2_chassis_teleop.py --visualizer kit

容器侧（两条终端）::

    docker run -it --rm --network host -v $PWD:/work -w /work ros2-humble-dev:latest bash
    # A: source /opt/ros/humble/setup.bash && python3 scripts/ros2_leisaac_bridge.py --sim_host 127.0.0.1
    # B: source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard
    #    然后按住 i/j/l/, 等键（teleop_twist_keyboard 的标准按键）

说明：
- 底盘是**运动学**模型（写 USD 变换 + 机械臂跟随，没有轮子物理、没有碰撞响应），
  所以"遥控"是精确可控的，但避障要靠上层（LiDAR/Nav2）。
- 脚本会**去掉 time_out / success 终止项**：长时间遥控不会中途重置场景。
- 状态 HUD（窗口左上角）：运行时长、底盘位姿、收到的 cmd_vel、桥接客户端是否连上、LiDAR 最近距离。
"""
from __future__ import annotations

import argparse
import math
import multiprocessing
import os
import sys
import time

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)

# ── 让本仓库的 `leisaac` 包可被 import（不依赖 editable 安装 / 外部 PYTHONPATH）──
# 打包给赛队时仓库可能被解压到任意目录、也不一定跑过 `pip install -e source/leisaac`；
# 这里从本文件往上找到含 `source/leisaac` 的仓库根，自己补 sys.path（不依赖 CWD）。
_p = os.path.dirname(os.path.abspath(__file__))
while _p != os.path.dirname(_p):
    if os.path.isdir(os.path.join(_p, "source", "leisaac")):
        sys.path.insert(0, os.path.join(_p, "source", "leisaac"))
        break
    _p = os.path.dirname(_p)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="ROS2 键盘遥控底盘（Isaac Sim 侧）")
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0", help="任务名（决定场景与机器人）")
parser.add_argument("--step_hz", type=int, default=30, help="主循环频率")
parser.add_argument("--bridge_port", type=int, default=5560, help="TCP 桥接端口（ROS2 侧连这个）")
parser.add_argument("--max_seconds", type=float, default=0.0, help="运行多少秒后自动退出（0=一直跑到手动关闭）")
parser.add_argument("--settle_steps", type=int, default=60, help="开始前的物理静置步数")
# ── 起始/作业位：数采时直接从柜子前起仿真，不用先开车过去 ──────────────────────
parser.add_argument(
    "--start_at",
    type=str,
    default=None,
    help="直接用场地里的**作业位**起仿真（省得先开车过去）。可选："
    "home 起点区 / shelf 收纳架前（上肢数采用这个）/ transfer 转运架前 / park 停放区",
)
parser.add_argument("--start_x", type=float, default=None, help="起始世界坐标 x（米），覆盖 --start_at")
parser.add_argument("--start_y", type=float, default=None, help="起始世界坐标 y（米），覆盖 --start_at")
parser.add_argument("--start_yaw", type=float, default=None, help="起始绝对朝向（弧度，0=朝 +X），覆盖 --start_at")
parser.add_argument(
    "--camera_view", type=str, default="panel", choices=["panel", "windows", "off"], help="相机画面显示方式"
)
parser.add_argument("--lidar_vis", action="store_true", help="打开 LiDAR 射线可视化（能看到避障扫描）")
parser.add_argument(
    "--lidar_stop_distance",
    type=float,
    default=0.0,
    help="LiDAR 前向急停距离（米）：前方这个距离内有障碍就不允许继续前进；**默认 0 = 关**。"
    "⚠️ 别默认给 0.45：4x3m 场地**角落里**（起点、停放区）前向 ±40° 锥内 0.70m 就打到围栏，"
    "开了急停会让「想往前开却一动不动」（实测踩坑）。建图/导航想要防穿墙再加 0.3。"
    "运动学底盘不会被静态障碍物理挡住，需要「不撞墙」时开它",
)
parser.add_argument(
    "--collision_force_threshold",
    type=float,
    default=20.0,
    help="接触保险杠阈值（N）：推开/顶到动态物体时超过它就停止平移（允许转向/后退），默认 20",
)
parser.add_argument("--hud", dest="hud", action="store_true", default=True, help="显示状态 HUD（默认开）")
parser.add_argument("--no_hud", dest="hud", action="store_false", help="关闭状态 HUD")
parser.add_argument(
    "--odom_frame",
    type=str,
    default="start",
    choices=["start", "world"],
    help="发给 ROS2 的 /odom 用哪个坐标系：start=以**起始位姿**为原点、初始朝向为 +x（ROS 惯例，"
    "SLAM 建出的地图原点就是机器人起点，AMCL 初始位姿 (0,0,0) 直接可用）；world=Isaac Sim 世界坐标。",
)

# ── 统一入口：**只有一个模式**（所有操控通道同时在线）────────────────────────────
#
# 2026-09-18 起不再用 `--mode` 区分"底盘模式/上肢模式"——一个仿真里同时具备：
#   · 底盘：随时接受 ROS2 `/cmd_vel`（容器里的键控 / Nav2 / 任何发布者）
#   · 上肢：遥操设备一"启动"就接管（主手使能 / 键盘按 B），没启动时保持位姿
#   · 上肢：配了策略就由策略驱动，**人一按 B 就交给人**（R / N 之后交回策略）
#   · 录制：`--record` 随时可用（R=失败重录 / N=成功保存）
# `--mode` 保留只为**兼容旧命令**：`nav`/`teleop` 等同于统一行为（会提示），
# `policy` 等于统一行为 + 启动时就打开策略。
parser.add_argument(
    "--mode",
    type=str,
    default=None,
    choices=[None, "auto", "nav", "teleop", "policy"],
    help="【已废弃，仅为兼容旧命令】统一入口不再需要它：nav/teleop 都等同统一行为；"
    "policy = 统一行为 + 启动时打开策略（也可用 --enable_policy）",
)
parser.add_argument(
    "--lock_chassis",
    action="store_true",
    default=False,
    help="锁住底盘（忽略 /cmd_vel）。默认**不锁** —— 统一入口里底盘随时可被 ROS2 驱动；"
    "只在「怕误碰底盘键」的场合才加",
)
parser.add_argument(
    "--allow_chassis_during_teleop",
    action="store_true",
    default=False,
    help="【已废弃】旧参数，现在底盘默认就是放开的，加不加都一样",
)
parser.add_argument("--enable_policy", action="store_true", default=False, help="启动时就打开策略驱动上肢")

# ── teleop 模式：设备（语义与 teleop_se3_agent.py 一致）──────────────────────
parser.add_argument(
    "--teleop_device",
    type=str,
    default=None,
    help="上肢设备 / 动作空间。不给则按任务推导（双臂任务 = bi-so101leader）。"
    "可选：bi-so101leader（真主手，双臂，12 维关节角）/ bi-keyboard（**双臂键盘**，T 切换左右臂，"
    "16 维增量，给没有真主手的队伍）/ so101leader-one（只有一条主手）/ keyboard（**单臂任务**才可用）"
    "/ gamepad / so101leader / lekiwi-*。policy 模式也认这个参数：用键盘采的数据训出来的策略"
    "必须写 --teleop_device bi-keyboard 才能对上 16 维动作空间。",
)
parser.add_argument("--sensitivity", type=float, default=1.0)
parser.add_argument("--port", type=str, default=None, help="单臂主手串口")
parser.add_argument("--remote_endpoint", type=str, default=None, help="远程主手 tcp://host:port")
parser.add_argument("--recalibrate", action="store_true", default=False)
parser.add_argument("--left_arm_port", type=str, default=None, help="左主手串口（bi-so101leader）")
parser.add_argument("--right_arm_port", type=str, default=None, help="右主手串口（bi-so101leader）")
parser.add_argument("--arm_side", type=str, default="left", choices=["left", "right"])
parser.add_argument("--relative_one_arm", action="store_true", default=False)
parser.add_argument("--engage_threshold", type=float, default=None)
parser.add_argument(
    "--arm_gravity",
    action="store_true",
    default=False,
    help="键盘/手柄这类 IK 设备默认**关掉机械臂重力**（否则不按键时整条臂被重力拽着往下塌）。"
    "加这个开关可保留重力，用于「臂有重量」的对比实验",
)

# ── teleop 模式：录制（语义与 teleop_se3_agent.py 一致：R 开始 / N 标成功保存）──
parser.add_argument(
    "--record",
    dest="record",
    action="store_true",
    default=True,
    help="**默认就开**：recorder 一直挂着，按 N / 容器指令才真正写这一条。"
    "不想挂就加 --no_record（省一点内存/IO）",
)
parser.add_argument("--no_record", dest="record", action="store_false",
                    help="不挂 recorder（纯建图/导航会话可用）")
parser.add_argument(
    "--dataset_file",
    type=str,
    default=None,
    help="录制输出 HDF5。**不写就自动生成**：datasets/session_<月日>_<时分>.hdf5 —— "
    "所以同一条命令可以反复跑，不会撞上「文件已存在」（撞了就 --resume）",
)
parser.add_argument("--num_demos", type=int, default=0, help="录够这么多条就自动退出，0=不限")
parser.add_argument("--resume", action="store_true", default=False)
parser.add_argument("--use_lerobot_recorder", action="store_true", default=False)
parser.add_argument("--lerobot_dataset_repo_id", type=str, default=None)
parser.add_argument("--lerobot_dataset_fps", type=int, default=30)

# ── policy 模式（语义与 policy_inference.py 一致）────────────────────────────
parser.add_argument("--policy_type", type=str, default="local-act",
                    help="local-act（本地 TCP ACT）/ lerobot-<model> / gr00tn1.5 / gr00tn1.6 / openpi")
parser.add_argument("--policy_host", type=str, default="127.0.0.1")
parser.add_argument("--policy_port", type=int, default=5556)
parser.add_argument("--policy_timeout_ms", type=int, default=15000)
parser.add_argument("--policy_action_horizon", type=int, default=100)
parser.add_argument("--policy_language_instruction", type=str, default=None)
parser.add_argument("--policy_checkpoint_path", type=str, default=None)
parser.add_argument("--policy_device", type=str, default="cpu")
parser.add_argument("--action_safety", dest="action_safety", action="store_true", default=True,
                    help="动作安全层（位置限幅 + 每步增量限幅 + NaN 守卫），默认开")
parser.add_argument("--no_action_safety", dest="action_safety", action="store_false")
parser.add_argument("--action_safety_margin_deg", type=float, default=2.0)
parser.add_argument("--action_safety_max_delta", type=float, default=0.1)
parser.add_argument("--policy_progress_every", type=int, default=30)

# IsaacLab 3.0 起 --headless / --enable_cameras 是 AppLauncher 的配置键、不能当 argparse 字段名，
# 用别名 dest 保留命令行写法，解析后写回（与 teleop_se3_agent.py 一致）。
parser.add_argument("--enable_cameras", dest="enable_cameras_flag", action="store_true", default=False)
parser.add_argument("--headless", dest="headless_flag", action="store_true", default=False)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# ── 默认值补全：录制文件名自动带时间戳（**必须保证不重名**）─────────────────────
# 为什么：`_configure_recorder()` 里有断言"目标 HDF5 不能已存在"（除非 --resume）。
# 自动命名一旦撞名，仿真会在**窗口已经打开之后**抛 AssertionError 闪退 ——
# 实测：同一分钟内起两次就撞（原来精确到分钟，就是这么挂的）。
# 所以精确到秒，**再兜一层自增后缀**，怎么起都不会重名。
if args_cli.record and not args_cli.dataset_file:
    # 放到 datasets/sessions/ 子目录：裸起仿真（建图/导航）也会挂一个 recorder，
    # 自动命名的会话文件集中在子目录里，不会把 datasets/ 顶层塞满空文件。
    _sess_dir = os.path.join("datasets", "sessions")
    _stamp = time.strftime("%m%d_%H%M%S")
    _sess_path = os.path.join(_sess_dir, f"session_{_stamp}.hdf5")
    _dup = 2
    while os.path.exists(_sess_path):
        _sess_path = os.path.join(_sess_dir, f"session_{_stamp}_{_dup}.hdf5")
        _dup += 1
    args_cli.dataset_file = _sess_path
    _AUTO_SESSION_FILE = True
else:
    _AUTO_SESSION_FILE = False

# ★ 录制文件"已存在"要**提前**报错：不然要等 Isaac Sim 起完（30s、窗口都开了）才在
#   `_configure_recorder()` 的 assert 上炸掉，看起来就是"打开仿真闪退"（用户实测）。
if args_cli.record and args_cli.dataset_file and not args_cli.resume:
    if os.path.exists(args_cli.dataset_file):
        raise SystemExit(
            f"\n[ERROR] 录制文件已存在：{os.path.abspath(args_cli.dataset_file)}\n"
            f"        · 想接着往里录：加 --resume\n"
            f"        · 想新开一份：  换名字（--dataset_file datasets/别的名字.hdf5）\n"
            f"        · 不想挂录制：  加 --no_record\n"
        )


# ── 冲突检测 ①：同一台机器只允许一个仿真（否则抢显存 + 抢桥接端口）────────────────
def _claim_single_instance(lock_path: str, port: int) -> None:
    """抢单实例锁；已经有一个仿真正在跑就带说明退出。

    为什么在起 Isaac Sim **之前**做：起一次要 ~35 s + 几 GB 显存，
    等跑起来才发现端口被占（或者两个人各自开了半个仿真互相抢 GPU），代价太大。
    """
    import os as _os
    import socket as _socket

    # ① 桥接端口被占 → 十有八九是另一个仿真（或它的残留进程）
    probe = _socket.socket()
    probe.settimeout(0.5)
    try:
        port_busy = probe.connect_ex(("127.0.0.1", port)) == 0
    finally:
        probe.close()

    # ② 锁文件里的 pid 还活着 → 另一个仿真在跑
    holder = None
    if _os.path.isfile(lock_path):
        try:
            with open(lock_path) as fh:
                holder = int(fh.read().strip() or 0)
        except (OSError, ValueError):
            holder = None
        if holder:
            try:
                _os.kill(holder, 0)  # 只探活，不发信号
            except OSError:
                holder = None  # 陈旧锁（进程已死）→ 直接接管

    if holder or port_busy:
        why = (
            f"另一个仿真正在跑（pid={holder}）" if holder
            else f"桥接端口 {port} 已被占用（可能是残留进程）"
        )
        raise SystemExit(
            f"\n[ERROR] 检测到{why} —— **同一台机器只允许一个仿真**。\n"
            f"        · 先看看有什么在跑：bash reproduce/stop_all.sh --status\n"
            f"        · 需要清干净：      bash reproduce/stop_all.sh\n"
            f"        · 确实要并行（换端口做对比实验）：先删锁文件 rm {lock_path}\n"
        )

    try:
        with open(lock_path, "w") as fh:
            fh.write(str(_os.getpid()))
    except OSError:
        pass  # 锁写不下去（/tmp 只读之类）不算致命，端口检查仍然有效


_SIM_LOCK = "/tmp/leisaac_sim.lock"
_claim_single_instance(_SIM_LOCK, args_cli.bridge_port)

# ⚠️ AppLauncher 会把这两个键从传入的 dict（即 args_cli.__dict__）里**取走**，
#    之后再读 args_cli.enable_cameras 会 AttributeError → 先存成局部常量。
HEADLESS = bool(args_cli.headless_flag)
# GUI 下如果要显示相机画面，就自动开相机（否则相机被剔掉 → 面板是空的，踩过）
ENABLE_CAMERAS = bool(args_cli.enable_cameras_flag) or (args_cli.camera_view != "off" and not HEADLESS)
if ENABLE_CAMERAS and not args_cli.enable_cameras_flag and args_cli.camera_view != "off":
    print("[ros2_chassis_teleop] 要显示相机画面 → 自动启用 --enable_cameras")
args_cli.enable_cameras = ENABLE_CAMERAS
args_cli.headless = True if HEADLESS else None
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import dataclasses  # noqa: E402

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.chassis import ChassisController, send_lidar_scan  # noqa: E402
from leisaac.utils.env_utils import dynamic_reset_gripper_effort_limit_sim, get_task_type  # noqa: E402
from leisaac.utils.sim_modes import (  # noqa: E402
    build_device,
    build_policy,
    ensure_observations,
    preprocess_obs_dict,
)
from leisaac.utils.sim_bridge_server import SimBridgeServer  # noqa: E402


_RESOLVED_DEVICE_CACHE: str | None = None


def resolve_device_name() -> str:
    """本次运行实际用的遥操设备名。

    · 显式给了 `--teleop_device` → 用它（命令行永远优先）
    · 没给 → 先按任务推断（双臂任务 = `bi-so101leader`），**再看主手是否真的插着**：
      两个串口都在就用双臂主手；否则**自动退回 `bi-keyboard`**（双臂键盘，不需要硬件），
      并打印一行说明。这样"没插主手的人"也能用同一条命令把仿真跑起来操作机械臂。

    ★ 集中到一处的原因：以前 `build_env()` 里算了一个**局部** `device_name`，而 `main()`
    里也引用它（起设备 / 打印 / HUD），于是 `--mode teleop` 一进 `main()` 就
    `NameError: name 'device_name' is not defined` —— 那正是"窗口打开后立刻退出"的原因。
    """
    if args_cli.teleop_device:
        return args_cli.teleop_device
    inferred = get_task_type(args_cli.task)
    global _RESOLVED_DEVICE_CACHE
    if _RESOLVED_DEVICE_CACHE is not None:
        return _RESOLVED_DEVICE_CACHE
    if inferred in ("bi-so101leader", "so101leader", "so101leader-one"):
        lp = args_cli.left_arm_port or "/dev/ttyACM0"
        rp = args_cli.right_arm_port or "/dev/ttyACM1"
        if getattr(args_cli, "remote_endpoint", None) or getattr(args_cli, "port", None):
            return inferred  # 用户指了串口/远端，别自作聪明
        if os.path.exists(lp) and os.path.exists(rp):
            _RESOLVED_DEVICE_CACHE = inferred
            return inferred
        print(
            f"[ros2_chassis_teleop] 没找到主手串口（{lp} / {rp}）→ 自动改用 **bi-keyboard**"
            f"（双臂键盘，按 T 切左右臂；要用主手就插好线并加 --teleop_device {inferred}）"
        )
        _RESOLVED_DEVICE_CACHE = "bi-keyboard"
        return _RESOLVED_DEVICE_CACHE
    _RESOLVED_DEVICE_CACHE = inferred
    return inferred


def _strip_cameras(env_cfg) -> int:
    """没开 `--enable_cameras` 时把相机传感器与图像观测词条剔掉。

    Isaac Sim 6.0.1 下"相机传感器存在但渲染未启用"会报
    `Invalid object in Py_Graph in getWrappedGraphFromNode`（见 HANDOFF §1/§6）。
    """
    from isaaclab.sensors import TiledCameraCfg

    removed = 0

    def is_image_term(term):
        func = getattr(term, "func", None)
        if func is not None and "image" in str(func):
            return True
        params = getattr(term, "params", None)
        return isinstance(params, dict) and "data_type" in params

    def walk(obj):
        nonlocal removed
        if obj is None or not dataclasses.is_dataclass(obj):
            return
        for f in dataclasses.fields(obj):
            try:
                val = getattr(obj, f.name)
            except Exception:  # noqa: BLE001
                continue
            if val is None:
                continue
            if is_image_term(val):
                setattr(obj, f.name, None)
                removed += 1
            elif dataclasses.is_dataclass(val):
                walk(val)

    for attr in ("observations", "events"):
        if hasattr(env_cfg, attr):
            walk(getattr(env_cfg, attr))
    for name in list(vars(env_cfg.scene).keys()):
        if isinstance(getattr(env_cfg.scene, name), TiledCameraCfg):
            setattr(env_cfg.scene, name, None)
            removed += 1
    return removed


def build_env():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)

    # 动作配置：三种模式都用 `--teleop_device`（显式给了就用它，否则按任务推导）。
    # teleop 模式的默认推导值就是 bi-so101leader，所以不写参数时行为与以前一致；
    # policy 模式也认这个参数 —— 用键盘采的数据训出来的策略是 16 维动作空间，
    # 推理时必须同样写 `--teleop_device bi-keyboard` 才能对上（见操作指南 §5）。
    device_name = resolve_device_name()

    # ★ 设备与场景的**臂数**必须匹配，否则会在 IsaacLab 深处抛难懂的 KeyError。提前给一句人话。
    _SINGLE_ARM_KEYBOARDS = ("keyboard", "gamepad", "lekiwi-keyboard", "lekiwi-gamepad")
    _BI_ARM_KEYBOARDS = ("bi-keyboard", "bi-gamepad")
    has_single_arm_entity = hasattr(env_cfg.scene, "robot")
    has_bi_arm_entities = hasattr(env_cfg.scene, "left_arm") and hasattr(env_cfg.scene, "right_arm")

    if device_name in _SINGLE_ARM_KEYBOARDS and not has_single_arm_entity:
        raise SystemExit(
            f"\n[ERROR] 任务 {args_cli.task} 是**双臂**场景，不支持 --teleop_device {device_name}。\n"
            f"        原因：单臂键盘/手柄的动作配置写的是 asset_name='robot'（单臂任务才有这个名字），\n"
            f"        双臂场景里只有 'left_arm' / 'right_arm' → IsaacLab 会抛 "
            f"KeyError: Scene entity with key 'robot' not found。\n"
            f"        ★ 双臂任务用键盘遥操请改加 --teleop_device bi-keyboard（T 键切换左右臂）；\n"
            f"        有真主手时：\n"
            f"          --teleop_device bi-so101leader --left_arm_port <左> --right_arm_port <右>\n"
            f"        只有一条主手时：\n"
            f"          --teleop_device so101leader-one --port <主手串口> --arm_side left\n"
        )
    if device_name in _BI_ARM_KEYBOARDS and not has_bi_arm_entities:
        raise SystemExit(
            f"\n[ERROR] 任务 {args_cli.task} 不是**双臂**场景，不支持 --teleop_device {device_name}。\n"
            f"        原因：双臂键盘的设备要找 'left_arm' / 'right_arm' 两个实体，"
            f"本任务里没有。\n"
            f"        单臂任务用键盘请改加 --teleop_device keyboard。\n"
        )

    env_cfg.use_teleop_device(device_name)

    # 键盘/手柄这类 IK 设备下，模板已把机械臂重力关掉（否则"不按键"时整条臂被重力匀速往下塌，
    # 见 `bi_arm_env_cfg.py::use_teleop_device`）。`--arm_gravity` 反转这个默认值。
    # ★ 采样窗口要建环境**之前**设，之后改 cfg 不生效。
    if args_cli.arm_gravity:
        for side in ("left_arm", "right_arm"):
            arm_cfg = getattr(env_cfg.scene, side, None)
            if arm_cfg is not None and getattr(arm_cfg, "spawn", None) is not None:
                arm_cfg.spawn.rigid_props.disable_gravity = False
        print("[ros2_chassis_teleop] --arm_gravity：已**保留**机械臂重力"
              "（键盘遥操下不按键会让臂缓缓下塌，属预期）")
    elif device_name in ("bi-keyboard", "bi-gamepad"):
        print(f"[ros2_chassis_teleop] {device_name}：已关闭机械臂重力（不按键时臂不会下塌）"
              "；要保留重力加 --arm_gravity")

    # 终止项：三种模式都删掉 time_out，避免长时间运行中途重置场景
    for name in ("time_out", "success"):
        if hasattr(env_cfg.terminations, name):
            setattr(env_cfg.terminations, name, None)

    # 观测组：录制与策略推理需要；场地任务的观测是空的，这里按双臂模板补上（不改任务文件）
    # （统一入口后不再看 --mode：只要 --record 或要跑策略就注入）
    if args_cli.record or args_cli.enable_policy or args_cli.mode == "policy":
        if ensure_observations(env_cfg):
            print("[ros2_chassis_teleop] 任务没有观测组 → 已按双臂模板注入一个"
                  "（left/right 关节 + left_wrist/right_wrist/front 三路相机）")

    # ★ 相机/图像观测项必须在**注入观测组之后**才剔 —— 否则注入进来的
    #   left_wrist/right_wrist/front 词条会指向已被删掉的相机传感器，建环境直接报
    #   `The scene entity 'left_wrist' does not exist`（实测：--record 但没给
    #   --enable_cameras 时必崩）。
    if not ENABLE_CAMERAS:
        n = _strip_cameras(env_cfg)
        print(f"[ros2_chassis_teleop] 未开 --enable_cameras，已剔除 {n} 个相机/图像观测项"
              "（注意：这样录出的 HDF5 **没有图像**，训练/推理都要相机，采数据请加 --enable_cameras）")

    # recorder：只有 --record 才要；不录就保持 None（省内存/省 I/O）
    if args_cli.record:
        _configure_recorder(env_cfg)
    else:
        env_cfg.recorders = None

    if args_cli.lidar_vis and hasattr(env_cfg.scene, "lidar"):
        env_cfg.scene.lidar.debug_vis = True
    return env_cfg


def _configure_recorder(env_cfg) -> None:
    """按 teleop_se3_agent.py 的语义配置 recorder（含 R 开始 / N 标成功的假 success 项）。"""
    from isaaclab.managers import DatasetExportMode
    from isaaclab.managers.manager_term_cfg import TerminationTermCfg

    from leisaac.enhance.managers.recorder_manager import EnhanceDatasetExportMode

    dataset_file = os.path.abspath(args_cli.dataset_file)
    output_dir = os.path.dirname(dataset_file)
    output_file_name = os.path.splitext(os.path.basename(dataset_file))[0]
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    if args_cli.use_lerobot_recorder:
        if args_cli.resume:
            env_cfg.recorders.dataset_export_mode = EnhanceDatasetExportMode.EXPORT_SUCCEEDED_ONLY_RESUME
        else:
            env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_ONLY
    else:
        if args_cli.resume:
            env_cfg.recorders.dataset_export_mode = EnhanceDatasetExportMode.EXPORT_ALL_RESUME
            assert os.path.exists(
                args_cli.dataset_file
            ), "the dataset file does not exist, please don't use '--resume' if you want to record a new dataset"
        else:
            env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_ALL
            assert not os.path.exists(
                args_cli.dataset_file
            ), "the dataset file already exists, please use '--resume' to resume recording"
    env_cfg.recorders.dataset_export_dir_path = output_dir
    env_cfg.recorders.dataset_filename = output_file_name

    # 录制时挂一个**永远为 False** 的假 success：operator 按 N → manual_terminate(env, True)
    # 才把它翻成 True，recorder 才把这条 demo 标为成功（原项目就是这么设计的）
    if not hasattr(env_cfg.terminations, "success"):
        setattr(env_cfg.terminations, "success", None)
    env_cfg.terminations.success = TerminationTermCfg(
        func=lambda env: torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    )


def _install_streaming_recorder(env, env_cfg) -> None:
    """把 env.recorder_manager 换成流式版（避免长 episode 把内存吃爆）—— 同 teleop_se3_agent.py。"""
    del env.recorder_manager
    if args_cli.use_lerobot_recorder:
        from leisaac.enhance.datasets.lerobot_dataset_handler import LeRobotDatasetCfg
        from leisaac.enhance.managers.lerobot_recorder_manager import LeRobotRecorderManager

        dataset_cfg = LeRobotDatasetCfg(
            repo_id=args_cli.lerobot_dataset_repo_id,
            fps=args_cli.lerobot_dataset_fps,
        )
        env.recorder_manager = LeRobotRecorderManager(env_cfg.recorders, dataset_cfg, env)
    else:
        from leisaac.enhance.managers.recorder_manager import StreamingRecorderManager

        env.recorder_manager = StreamingRecorderManager(env_cfg.recorders, env)
        env.recorder_manager.flush_steps = 100
        env.recorder_manager.compression = "lzf"


def manual_terminate(env, success: bool):
    """手动终止当前 episode（遥操按 N 标成功 / 按 R 弃掉）—— 照搬 teleop_se3_agent.py。

    录制时 `success` 终止项是个永远 False 的假项；按 N 就把它换成 True，
    recorder 才把这条 demo 记成成功。
    """
    from isaaclab.managers import TerminationTermCfg

    if hasattr(env, "termination_manager"):
        flag = torch.ones if success else torch.zeros
        env.termination_manager.set_term_cfg(
            "success",
            TerminationTermCfg(func=lambda env: flag(env.num_envs, dtype=torch.bool, device=env.device)),
        )
        env.termination_manager.compute()
    elif hasattr(env, "_get_dones"):
        env.cfg.return_success_status = success


class Hud:
    """左上角状态面板（omni.ui）。取不到 UI 时静默降级。

    ⚠️ **面板上的文本一律用 ASCII 英文**：omni.ui 的默认字体**没有中文字形**，
    中文会渲染成一串 `?`（2026-09-16 实测：窗口标题 + 每一行标签全变问号）。
    终端里的 `print()` 中文**不受影响**（终端有中文字体），所以那边照旧用中文。
    """

    def __init__(self, enabled: bool):
        self.labels = {}
        self.window = None
        if not enabled:
            return
        try:
            import omni.ui as ui

            self.window = ui.Window("LeIsaac status", width=390, height=190)
            with self.window.frame:
                with ui.VStack(spacing=4):
                    for key, text in (
                        ("time", "uptime: -"),
                        ("pose", "chassis pose: -"),
                        ("cmd", "cmd_vel: -"),
                        ("bridge", "ROS2 bridge: not connected"),
                        ("lidar", "LiDAR: -"),
                        ("mode", "mode: -"),
                        ("bump", "bump: -"),
                    ):
                        self.labels[key] = ui.Label(text)
        except Exception as exc:  # noqa: BLE001
            print(f"[hud] 无法创建状态面板（{exc}），只用终端输出")
            self.window = None

    def update(self, **values):
        for key, val in values.items():
            label = self.labels.get(key)
            if label is not None:
                label.text = val


def main() -> int:
    env_cfg = build_env()
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    # --record：把默认 recorder 换成流式 recorder（照搬 teleop_se3_agent.py）
    if args_cli.record:
        _install_streaming_recorder(env, env_cfg)

    env.reset()
    for _ in range(max(args_cli.settle_steps, 0)):
        env.sim.step(render=True)
        env.scene.update(env.physics_dt)

    chassis = ChassisController(
        env,
        collision_force_threshold=args_cli.collision_force_threshold,
        lidar_stop_distance=args_cli.lidar_stop_distance,
    )
    print(
        f"[ros2_chassis_teleop] 底盘碰撞体={chassis.has_collider} "
        f"接触保险杠阈值={chassis.collision_force_threshold}N "
        f"LiDAR急停={chassis.lidar_stop_distance or '关'}"
    )

    # ── 起始/作业位：`--start_at <区名>` 或 `--start_x/--start_y/--start_yaw` ──────────
    # 数采时不必先开车去柜子前：直接从作业位起仿真即可（`--start_at shelf`）。
    start_x, start_y, start_yaw = args_cli.start_x, args_cli.start_y, args_cli.start_yaw
    if args_cli.start_at:
        from leisaac.assets.scenes.smart_factory_layout import WORK_ZONES

        if args_cli.start_at not in WORK_ZONES:
            raise SystemExit(
                f"[ERROR] --start_at 只支持 {sorted(WORK_ZONES)}，收到 {args_cli.start_at!r}"
            )
        zx, zy, zyaw = WORK_ZONES[args_cli.start_at]
        if start_x is None:
            start_x = zx
        if start_y is None:
            start_y = zy
        if start_yaw is None:
            start_yaw = zyaw
    if start_x is not None or start_y is not None or start_yaw is not None:
        chassis.set_pose(start_x, start_y, start_yaw)
        x, y, yaw = chassis.pose
        print(
            f"[ros2_chassis_teleop] 起始位姿已设为 "
            f"x={x:+.2f} y={y:+.2f} yaw={yaw * 57.2958:+.1f}°"
            + (f"（--start_at {args_cli.start_at}）" if args_cli.start_at else "")
        )
        # 摆位后让物理/渲染跟上一帧，免得第一帧还在老位置
        for _ in range(3):
            env.sim.step(render=True)
            env.scene.update(env.physics_dt)

    bridge = SimBridgeServer(host="127.0.0.1", port=args_cli.bridge_port)
    bridge.start()

    # 相机画面
    camera_panel = None
    if args_cli.camera_view != "off" and not HEADLESS:
        if args_cli.camera_view == "windows":
            from leisaac.utils.camera_view import create_camera_view_windows

            create_camera_view_windows(env)
        else:
            from leisaac.utils.camera_view import CameraImagePanel

            camera_panel = CameraImagePanel(env)

    hud = Hud(args_cli.hud and not HEADLESS)

    # 机械臂保持初始位姿（固定目标，避免 PD 误差为 0 时慢慢下沉）
    #
    # ⚠️ 只有"绝对关节目标"类 action term（`JointPositionActionCfg`，真主手用）才谈得上
    #    "保持某个关节角"：它的 action_dim == len(joint_names)（左臂 5 + 夹爪 1）。
    #    IK / 增量类 term（键盘、手柄：`DifferentialInverseKinematicsActionCfg`）维度是
    #    **6（末端位姿增量）** 而 `joint_names` 只有 4 个、`RelativeJointPositionActionCfg`
    #    是 2 个关节 —— 维度天然对不上（曾经在这里直接 RuntimeError 崩掉），而且
    #    `use_relative_mode=True` 下"零动作 = 目标跟住当前位姿"，塞成关节角毫无意义。
    #    所以按 term 类型分别处理：对得上的填当前关节角，对不上的填 0（增量语义下的"保持"）。
    hold = torch.zeros(1, env.action_manager.total_action_dim, device=env.device)
    held_terms: list[str] = []
    zero_terms: list[str] = []
    offset = 0
    for term_name in env.action_manager.active_terms:
        term = env.action_manager.get_term(term_name)
        cfg = term.cfg
        dim = term.action_dim
        jn = list(getattr(term, "_joint_names", []) or getattr(cfg, "joint_names", []) or [])
        asset = env.scene[cfg.asset_name]
        ids, _ = asset.find_joints(jn)
        if jn and len(ids) == dim:  # 绝对关节目标：直接填当前关节角
            pos = asset.data.joint_pos
            pos = pos.torch if hasattr(pos, "torch") else pos
            hold[:, offset : offset + dim] = pos[:, ids]
            held_terms.append(term_name)
        else:  # IK / 增量：零动作即"保持原状"
            zero_terms.append(f"{term_name}(动作{dim}维/关节{len(ids)}个)")
        offset += dim
    print(f"[ros2_chassis_teleop] 保持位姿：绝对关节目标项 {held_terms or '无'}")
    if zero_terms:
        print(f"[ros2_chassis_teleop] 保持位姿：增量/IK 项用零动作 {zero_terms}"
              f"（相对 IK 下重力会让臂缓慢下沉，属已知行为）")

    # ── 上肢动作源：**一个循环里全部留出来**，每帧按优先级取 ──────────────────
    #
    #   ① 遥操设备：一"启动"就接管（主手使能 / 键盘按 B）
    #   ② 策略    ：配了 `--mode policy` / `--enable_policy` 就跑；人按 B 就交给人
    #   ③ 都没有  ：`hold` 保持位姿（IK 类 term 用零动作，模板已关掉臂的重力 → 不会塌）
    #
    # 底盘与上肢**互不干扰**：底盘永远听 ROS2 `/cmd_vel`（除非 `--lock_chassis`）。
    teleop_interface = None
    policy = None
    action_limiter = None
    policy_obs = None
    policy_horizon_left = 0
    policy_actions = None
    should_reset_recording_instance = False
    should_reset_task_success = False
    start_record_state = False
    current_recorded_demo_count = 0
    mode = args_cli.mode
    policy_step_in_episode = 0
    # 本次实际使用的设备名（起设备 / 打印 / HUD 都要用；见 resolve_device_name 的注释）
    device_name = resolve_device_name()
    #: 是不是"键盘类"设备（提示语不一样：键盘要先点窗口再按 B，主手要掰到相近姿势）
    _device_is_keyboard = "keyboard" in device_name or "gamepad" in device_name

    # ── 上肢驱动源：可在**运行中**切换（仿真不用重启）────────────────────────────
    #   auto（默认）：设备启动了就用设备，否则有策略就跑策略，否则保持位姿
    #   device      ：只用设备（策略不掺和）
    #   policy      ：强制策略（按需**懒加载**：第一次切过去时才连策略服务端）
    #   hold        ：谁也不动，保持位姿
    # 切换途径：① 仿真窗口按 P（在 auto / policy 之间切）；② 容器侧发
    #   `ros2 topic pub --once /leisaac/arm_source std_msgs/String "{data: policy}"`
    arm_source_mode = "auto"

    # ── 录制"分集"控制 ────────────────────────────────────────────────────────
    # 为什么需要：IsaacLab 的流式录制器**每帧都在写盘**，只在 env.reset() 时收尾一条。
    # 如果把"集与集之间的空转（车没动、臂在初始位姿）"也累计进去，采出来的每条前面
    # 都会拖一长段静止帧（而且会导出成一堆假 episode）。这里的做法是：
    #   · 空闲时把 recorder 的 terms 清空 → 一帧都不记
    #   · begin（容器指令 / 键盘 B）时复位场景并重新打开 terms → 这条从"干净状态"开始
    #   · success/fail（容器指令 / 键盘 N / R）时收尾导出，然后立刻回到空闲
    _ALL_RECORDER_TERMS = list(getattr(env.recorder_manager, "active_terms", []) or [])

    def _set_recording_active(on: bool) -> None:
        if not _ALL_RECORDER_TERMS:
            return
        try:
            env.recorder_manager._term_names = list(_ALL_RECORDER_TERMS) if on else []
        except Exception as exc:  # noqa: BLE001
            print(f"[record] ⚠ 切换录制开关失败（{type(exc).__name__}: {exc}）", flush=True)

    def _begin_episode(why: str) -> None:
        """开始新的一条：**复位场景**（保证每条起点一致）+ 打开录制。"""
        env.reset()
        chassis.apply_pose()  # 复位只把手臂/货物写回初始位姿，底盘留在原地
        if args_cli.record:
            manual_terminate(env, False)  # 新的一条先标"未成功"
            _set_recording_active(True)
        print(f"[record] 第 {current_recorded_demo_count + 1} 条开始（场景已复位；{why}）", flush=True)

    def _try_build_policy(overrides: dict | None = None):
        """懒加载策略 + 动作安全层。失败只警告并返回 None（仿真照跑）。"""
        ov = overrides or {}
        ptype = str(ov.get("policy_type", args_cli.policy_type))
        pport = int(ov.get("policy_port", args_cli.policy_port))
        phost = str(ov.get("policy_host", args_cli.policy_host))
        try:
            pol = build_policy(
                env,
                policy_type=ptype,
                task_type=get_task_type(args_cli.task),
                host=phost,
                port=pport,
                timeout_ms=args_cli.policy_timeout_ms,
                action_horizon=args_cli.policy_action_horizon,
                language_instruction=args_cli.policy_language_instruction,
                checkpoint_path=ov.get("policy_checkpoint_path", args_cli.policy_checkpoint_path),
                policy_device=str(ov.get("policy_device", args_cli.policy_device)),
            )
            limiter = None
            if args_cli.action_safety:
                from leisaac.utils.action_safety import ActionSafetyLimiter, install_declarative_clip

                limiter = ActionSafetyLimiter(
                    env,
                    margin_deg=args_cli.action_safety_margin_deg,
                    max_delta=args_cli.action_safety_max_delta,
                )
                install_declarative_clip(env, margin_deg=args_cli.action_safety_margin_deg)
            print(f"[arm] 策略已连接：{ptype} @ {phost}:{pport}", flush=True)
            return pol, limiter
        except Exception as exc:  # noqa: BLE001
            print(
                f"[ros2_chassis_teleop] ⚠ 策略没起来（{type(exc).__name__}: {exc}）\n"
                f"    → 仿真继续跑：上肢先保持位姿，遥控/底盘不受影响。\n"
                f"    → 策略服务端起了吗？见 README §9.1（`scripts/evaluation/act_action_server.py`）。",
                flush=True,
            )
            return None, None

    # 底盘默认**不锁**（统一入口的要点：底盘和上肢可以同时被操作）
    chassis_locked = bool(args_cli.lock_chassis)
    if args_cli.allow_chassis_during_teleop:
        print("[ros2_chassis_teleop] --allow_chassis_during_teleop 已废弃：底盘现在默认就是放开的")
    if mode in ("nav", "teleop"):
        print(f"[ros2_chassis_teleop] --mode {mode} 已废弃：统一入口不再分模式"
              f"（行为与不加 --mode 完全相同；底盘自由 + 上肢设备待命）")
    policy_wanted = bool(args_cli.enable_policy or mode == "policy")

    # 设备：**容错构建** —— 没插主手也必须能把仿真跑起来（至少底盘 + 策略可用）。
    # 以前只有 teleop 模式才建设备，所以 nav 模式不需要硬件；统一之后若还硬要求主手，
    # 建图/导航的人会被一条串口错误挡在门外，所以这里降级而不是抛异常。
    try:
        teleop_interface = build_device(
            env,
            device=device_name,  # ← 用解析后的名字（--teleop_device 没给时 = 任务推导值）
            sensitivity=args_cli.sensitivity,
            port=args_cli.port,
            remote_endpoint=args_cli.remote_endpoint,
            recalibrate=args_cli.recalibrate,
            left_arm_port=args_cli.left_arm_port,
            right_arm_port=args_cli.right_arm_port,
            arm_side=args_cli.arm_side,
            relative_one_arm=args_cli.relative_one_arm,
            engage_threshold=args_cli.engage_threshold,
        )
    except Exception as exc:  # noqa: BLE001
        print(
            f"[ros2_chassis_teleop] ⚠ 遥操设备 {device_name} 没能建起来"
            f"（{type(exc).__name__}: {exc}）\n"
            f"    → 本次仿真仍然可用：**底盘**照常听 /cmd_vel、**策略**照常能跑，"
            f"只是没有上肢遥操。\n"
            f"    → 没插主手/没接串口时：加 `--teleop_device bi-keyboard` 用双臂键盘，"
            f"或干脆不遥操上肢。"
        )

    if teleop_interface is not None:
        def _reset_recording_instance():
            nonlocal should_reset_recording_instance
            should_reset_recording_instance = True

        def _reset_task_success():
            nonlocal should_reset_task_success
            should_reset_task_success = True
            _reset_recording_instance()

        def _toggle_policy():
            """`P` 键：在 auto / policy 之间切换上肢驱动源（运行中，不用重启仿真）。"""
            nonlocal arm_source_mode, policy, action_limiter
            if arm_source_mode == "policy":
                arm_source_mode = "auto"
                print("[arm] P 键 → 交回遥操设备/保持（arm_source=auto）", flush=True)
                return
            if policy is None:
                _pol, _lim = _try_build_policy()
                if _pol is None:
                    return  # 连不上策略服务端，_try_build_policy 已经打印原因
                policy, action_limiter = _pol, _lim
            arm_source_mode = "policy"
            print("[arm] P 键 → 策略接管（arm_source=policy；再按 B 可随时抢回来）", flush=True)

        teleop_interface.add_callback("R", _reset_recording_instance)
        teleop_interface.add_callback("N", _reset_task_success)
        teleop_interface.add_callback("P", _toggle_policy)
        teleop_interface.display_controls()
        teleop_interface.reset()
        if args_cli.record:
            current_recorded_demo_count = (
                env.recorder_manager._dataset_file_handler.get_num_episodes() if args_cli.resume else 0
            )

    if args_cli.record:
        _set_recording_active(False)
        print("[record] 录制**待命**：按 B（或用容器指令 record=begin）才开始累计这一条，"
              "N/success 保存、R/fail 丢弃；集与集之间的空转不会被录进去")

    if policy_wanted:
        _pol, _lim = _try_build_policy()
        if _pol is not None:
            policy, action_limiter = _pol, _lim
            arm_source_mode = "policy"  # 启动就打开策略（仍可被 P 键 / 容器指令切走）

    print("\n" + "=" * 64)
    print("  LeIsaac 统一仿真入口  ——  **单模式：所有操控通道同时在线**")
    print(f"  任务: {args_cli.task}")
    print(f"  桥接: tcp://127.0.0.1:{args_cli.bridge_port}  ← ROS2 侧 ros2_leisaac_bridge.py 连它")
    print("  底盘: ROS2 /cmd_vel（容器侧键控 / Nav2 / 任何发布者）"
          + ("  🔒 已按 --lock_chassis 锁住" if chassis_locked else ""))
    if teleop_interface is not None:
        print(f"  上肢: {device_name}" + "（一启动就接管；" +
              ("键盘按 B" if "keyboard" in device_name or "gamepad" in device_name else "主手使能") + "）")
    else:
        print("  上肢: ⚠ 遥操设备不可用（没建成）—— 只剩策略/保持位姿")
    if policy is not None:
        print(f"  上肢: 策略 {args_cli.policy_type} @ {args_cli.policy_host}:{args_cli.policy_port}"
              "（设备未启动时由它驱动；人按 B 接管，R/N 之后交回）"
              + (f"  安全层 margin={args_cli.action_safety_margin_deg}° "
                 f"delta={args_cli.action_safety_max_delta}" if args_cli.action_safety else "  安全层已关"))
    if args_cli.record:
        print(f"  录制: {os.path.abspath(args_cli.dataset_file)}   R=失败重录  N=成功保存"
              f"（--num_demos {args_cli.num_demos or '不限'}）")
    print("=" * 64)

    # ── 遥操自检（用户反馈"没办法遥操机械臂"：多半是没点窗口 / 没按 B）────────────
    _is_kb = teleop_interface is not None and _device_is_keyboard
    print("[自检] 上肢遥操 = " + (device_name if teleop_interface is not None else "无（设备没建成）"))
    if _is_kb:
        if HEADLESS:
            print("[自检] ⚠ 现在是 **headless（没有窗口）** → 键盘设备收不到任何按键！")
            print("        · 要用键盘遥操：去掉 --headless 重起（键盘必须有窗口）")
            print("        · 或者用主手 --teleop_device bi-so101leader / 策略 --enable_policy")
        else:
            print("[自检] 键盘遥操两步走：① **先用鼠标点一下仿真窗口**（按键事件只发给聚焦的窗口）")
            print("                       ② 按 **B** 开始控制（不按 B 从手不动，看着就像没反应）")
            print("       键位：T 切左右臂 | W/S 前后 · Q/E 上下 · A/D 肩转 · J/L K/I 调姿 · U/O 开合爪")
            print("       录制：N 本条成功保存 / R 丢弃本条（保存完要继续录就**再按一次 B**）")
    elif teleop_interface is not None:
        print("[自检] 主手遥操：先把主手掰到与仿真里那条臂相近的姿势，接近后自动使能")
        print("       第一次用需要标定：加 --recalibrate")
    if args_cli.record:
        print("[自检] 录制**待命**：按 B（或容器指令 record=begin）才开始累计这一条")
    print("=" * 64 + "\n")

    target_dt = 1.0 / args_cli.step_hz  # 节流用的**目标**步长（固定）
    dt = target_dt  # 底盘积分步长：循环里会换成**实测**墙钟间隔（见下）
    t0 = time.time()
    last_t = t0
    step = 0
    last_cmd = (0.0, 0.0, 0.0)
    last_arm_source = None  # 上肢控制源变化时打一行日志（设备接管 / 交回策略 / 回到保持）
    want_shutdown = False   # 收到 shutdown 请求后，等本帧跑完再退出（保证最后一条已导出刷盘）
    # 终端日志节流（避免每秒刷屏；HUD 上有实时状态）
    _log_key = None
    _log_pose = chassis.pose
    _log_t = -99.0

    # /odom 的参考原点：默认用"起始位姿"（ROS 惯例 —— 里程计原点 = 机器人起点，初始朝向 = +x）。
    # 这样 slam_toolbox 建出的地图原点就是机器人起点、朝向就是初始朝向，AMCL 的 initial_pose=(0,0,0) 直接对得上。
    # 注意底盘 yaw 是**世界系绝对角**（含初始朝向 85.8°），所以这里要做一次刚体变换，不只是相减。
    odom_x0, odom_y0, odom_yaw0 = chassis.pose

    def to_odom(x: float, y: float, yaw: float):
        if args_cli.odom_frame == "world":
            return x, y, yaw
        c, s = math.cos(odom_yaw0), math.sin(odom_yaw0)
        dx, dy = x - odom_x0, y - odom_y0
        # 世界 → 初始朝向坐标系（+x = 初始朝前，+y = 初始朝左）
        ox = dx * c + dy * s
        oy = -dx * s + dy * c
        oyaw = math.atan2(math.sin(yaw - odom_yaw0), math.cos(yaw - odom_yaw0))
        return ox, oy, oyaw

    print(
        f"[chassis] /odom 坐标系 = {args_cli.odom_frame}"
        + (f"（起点世界位姿 x={odom_x0:.3f} y={odom_y0:.3f} yaw={odom_yaw0 * 57.2958:.1f}° → odom(0,0,0)）"
           if args_cli.odom_frame == "start" else "")
    )
    while simulation_app.is_running():
        # ★ dt 用**实测的墙钟间隔**，不是固定的 1/step_hz。
        #   为什么：这个循环每步要跑 env.step + 4 台相机 + 激光，实际只有 10~25 Hz，
        #   若按 1/30 积分，机器人走的距离只有指令的 1/3 —— 对遥操是"手感发飘"，
        #   对 Nav2 是致命的：MPPI 内部模型按实时速度预测，实际慢 3 倍 →
        #   控制器认为"走不动"→ 一直报 Failed to make progress / 发极小速度 → 卡死。
        #   （厨房场景里只是慢，4×3m 的场地里就直接卡住了，实测。）
        now = time.time()
        dt = min(max(now - last_t, 1.0 / 240.0), 0.25)
        last_t = now
        with torch.inference_mode():
            if bridge.consume_reset():
                # ROS2 侧发 {"reset": true} → 底盘瞬移回起始位姿（建图完 → 导航前归位，不用重启仿真）
                chassis.reset()
                last_cmd = (0.0, 0.0, 0.0)
                print("[chassis] 收到复位请求 → 底盘回到初始位姿", flush=True)
            vx, vy, wz = bridge.get_cmd_vel()
            if chassis_locked:
                # ★ 互斥：人在遥操上肢时底盘不动。指令照样消费掉（不堆积），只提示一次/2秒。
                if (abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4) and step % 60 == 0:
                    print("[chassis] 🔒 上肢遥操中，已忽略 /cmd_vel"
                          "（要放开加 --allow_chassis_during_teleop）", flush=True)
                last_cmd = (0.0, 0.0, 0.0)
                chassis.update_sensors()
            else:
                moving = abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4
                if moving:
                    chassis.step(vx, vy, wz, dt)
                    last_cmd = (vx, vy, wz)
                else:
                    chassis.update_sensors()  # 静止时也刷新接触力/前方净空（状态显示用）

            # ── 运行中控制（容器侧发的 arm_source / record / shutdown）：先处理，再决定上肢怎么动 ──
            # ★ 退出请求不能让本帧提前 break：同一批里往往还带着 `record=success`，
            #   提前 break 会让**最后一条 episode 来不及导出**（实测：变成 success=False 且帧数被截断）。
            #   所以这里只打标记，等这一帧完整跑完（含导出/复位）再退。
            for ctl in bridge.consume_control():
                if ctl.get("shutdown"):
                    want_shutdown = True
                    continue
                if "arm_source" in ctl:
                    want = str(ctl["arm_source"]).strip().lower()
                    if want not in ("auto", "device", "policy", "hold"):
                        print(f"[arm] 忽略未知 arm_source={want!r}（可选 auto/device/policy/hold）")
                    else:
                        arm_source_mode = want
                        if want == "policy" and policy is None:
                            _pol, _lim = _try_build_policy(ctl)
                            if _pol is not None:
                                policy, action_limiter = _pol, _lim
                            else:
                                # 连不上策略服务端：退回 auto（设备启动就用设备，否则保持位姿），
                                # 不要把上肢卡成 hold —— 人还能用主手/键盘接管
                                arm_source_mode = "auto"
                        print(f"[arm] 容器指令 → arm_source = {arm_source_mode}", flush=True)
                if "record" in ctl:
                    act = str(ctl["record"]).strip().lower()
                    if act == "begin":
                        start_record_state = True
                        if not args_cli.record:
                            print("[record] ⚠ 本次仿真**启动时没挂 recorder**（`--no_record`）→ 录不了。\n"
                                  "         请重起仿真（不要 --no_record），再跑 collect.sh", flush=True)
                        _begin_episode("容器指令 record=begin")
                        if teleop_interface is not None and not teleop_interface.started:
                            if _device_is_keyboard:
                                print("[arm] ⏳ 现在等你在**仿真窗口**里操作："
                                      "① 用鼠标点一下窗口 ② 按 B 开始控制（不按 B 从手不动）",
                                      flush=True)
                            else:
                                print("[arm] ⏳ 现在等你把**主手**掰到与仿真里那条臂相近的姿势"
                                      "（接近后自动使能）", flush=True)
                    elif act in ("success", "fail", "stop"):
                        if act == "success":
                            print("Task Success!!!", flush=True)
                            if args_cli.record:
                                manual_terminate(env, True)
                        should_reset_recording_instance = True
                        print(f"[record] 容器指令 → {act}（等价按 {'N' if act == 'success' else 'R'}）",
                              flush=True)
                    else:
                        print(f"[record] 忽略未知 record={act!r}（可选 begin/success/fail/stop）")

            # ── 上肢动作：**一个循环里按优先级取**（设备 > 策略 > 保持）──────────
            # ★ 底盘已经在上面按 /cmd_vel 走过了 —— 两件事互不干扰，这就是"单模式"的意义。
            actions = None
            device_active = False
            if teleop_interface is not None and arm_source_mode in ("auto", "device", "policy"):
                dev_action = teleop_interface.advance()
                if dev_action is not None and not isinstance(dev_action, dict):
                    actions = dev_action
                    device_active = True
                    if arm_source_mode == "policy":
                        # 切到 policy 之后人又按了 B：人优先（自动退回 auto）
                        arm_source_mode = "auto"
                        print("[arm] 检测到遥操设备接管 → arm_source 自动回到 auto", flush=True)

            # R / N（设备按的键）：任何臂源下都要处理，否则录制/复位会失灵
            if should_reset_task_success:
                print("Task Success!!!")
                should_reset_task_success = False
                if args_cli.record:
                    manual_terminate(env, True)
            if should_reset_recording_instance:
                env.reset()
                # ★ 换一条 demo 时机械臂必须还贴在底盘上：env.reset() 会把两条手臂写回
                #   场景初始位姿（实测臂-底盘间距 0.20 m → **2.78 m**，底盘自己不动），
                #   不摆回来的话机器人"底盘在柜子前、胳膊在起点飘着"，数采直接废掉。
                chassis.apply_pose()
                should_reset_recording_instance = False
                if start_record_state:
                    if args_cli.record:
                        print("Stop Recording!!!")
                    start_record_state = False
                if args_cli.record:
                    manual_terminate(env, False)
                    # 这条已经收尾导出 → 立刻关掉录制，别把"集与集之间的空转"也记进去
                    _set_recording_active(False)
                if args_cli.record:
                    done = env.recorder_manager.exported_successful_episode_count
                    if done > current_recorded_demo_count:
                        current_recorded_demo_count = done
                        print(f"Recorded {current_recorded_demo_count} successful demonstrations.")
                    if args_cli.num_demos > 0 and done >= args_cli.num_demos:
                        print(f"All {args_cli.num_demos} demonstrations recorded. Exiting the app.")
                        break

            use_policy = policy is not None and (
                arm_source_mode == "policy" or (arm_source_mode == "auto" and actions is None)
            )
            if arm_source_mode in ("device", "hold"):
                use_policy = False  # 明确要求只用设备 / 只保持位姿时，策略不掺和
            if use_policy and actions is None:
                # 策略驱动（设备没启动时 / 或容器明确切到 policy）
                if policy_actions is None or policy_horizon_left <= 0:
                    obs_in = preprocess_obs_dict(
                        policy_obs["policy"],
                        args_cli.policy_type,
                        args_cli.policy_language_instruction,
                    )
                    policy_actions = policy.get_action(obs_in).to(env.device)
                    policy_horizon_left = min(args_cli.policy_action_horizon, policy_actions.shape[0])
                idx = policy_actions.shape[0] - policy_horizon_left
                actions = policy_actions[idx, :, :]
                policy_horizon_left -= 1
                if action_limiter is not None:
                    actions = action_limiter.filter(actions)
                if env.cfg.dynamic_reset_gripper_effort_limit:
                    dynamic_reset_gripper_effort_limit_sim(env, get_task_type(args_cli.task))
                policy_step_in_episode += 1
                if args_cli.policy_progress_every and policy_step_in_episode % args_cli.policy_progress_every == 0:
                    print(f"[policy] 已推理 {policy_step_in_episode} 步"
                          f"（chunk 剩 {policy_horizon_left}）", flush=True)

            source = "device" if device_active else ("policy" if use_policy else "hold")
            if actions is None:
                actions = hold  # 谁都没接管 → 保持位姿（IK 项为 0；模板已关掉臂的重力）
            if source != last_arm_source:
                names = {"device": f"遥操设备 {device_name}", "policy": f"策略 {args_cli.policy_type}",
                         "hold": "保持位姿（等设备启动）"}
                print(f"[arm] 上肢控制源 → {names[source]}", flush=True)
                last_arm_source = source
            if device_active and not start_record_state:
                if args_cli.record:
                    print("Start Recording!!!")
                    _begin_episode(f"键盘 B / 设备使能（{device_name}）")
                start_record_state = True
            policy_obs, _, _, _, _ = env.step(actions)
            # 每 3 步把位姿 + LiDAR 发给 ROS2（静止也发，SLAM/里程计需要）
            if step % 3 == 0:
                x, y, yaw = chassis.pose
                bridge.send_pose(*to_odom(x, y, yaw))
                send_lidar_scan(env, bridge)
            if step % 30 == 0:
                x, y, yaw = chassis.pose
                ox, oy, oyaw = to_odom(x, y, yaw)
                elapsed = time.time() - t0
                # ★ 终端**不要**每秒刷一行（用户要求：遥操时只要键位说明，别刷屏）。
                #   只在"状态真的变了"或"跑了一段路"时才打一行：
                #     · 客户端连上/断开、cmd_vel 变化、被挡住
                #     · 或者离上次打印走了 >5cm / 转了 >3°，且距上次打印 >8s
                _key = (
                    round(last_cmd[0], 2), round(last_cmd[1], 2), round(last_cmd[2], 2),
                    bool(chassis.blocked), bool(bridge.client_connected),
                )
                _moved = math.hypot(x - _log_pose[0], y - _log_pose[1]) > 0.05 or abs(
                    math.degrees(yaw - _log_pose[2])
                ) > 3.0
                if _key != _log_key or (_moved and elapsed - _log_t > 8.0):
                    print(
                        f"[chassis] t={elapsed:6.1f}s pos=({x:.3f}, {y:.3f}) yaw={yaw * 57.2958:6.1f}° "
                        f"odom=({ox:+.2f}, {oy:+.2f}, {oyaw * 57.2958:+.0f}°) "
                        f"cmd=({last_cmd[0]:+.2f}, {last_cmd[1]:+.2f}, {last_cmd[2]:+.2f}) "
                        f"clients={bridge.client_connected} "
                        f"接触力={chassis.last_contact_force:6.1f}N 前方净空={chassis.last_forward_clearance:.2f}m "
                        f"{'⛔撞到东西，已停' if chassis.blocked else ''}",
                        flush=True,
                    )
                    _log_key, _log_pose, _log_t = _key, (x, y, yaw), elapsed
                if camera_panel is not None:
                    camera_panel.update()
                # ⚠️ HUD 文本一律 ASCII 英文（omni.ui 没有中文字形，中文会变 `?`）；
                #    终端 print() 里的中文不受影响。
                hud.update(
                    time=f"uptime: {elapsed:6.1f} s",
                    pose=(
                        f"pose: x={x:+.3f} y={y:+.3f} yaw={yaw * 57.2958:+.1f} deg"
                        f"   |   odom: x={ox:+.2f} y={oy:+.2f} yaw={oyaw * 57.2958:+.0f} deg"
                    ),
                    cmd=f"cmd_vel: vx={last_cmd[0]:+.2f} vy={last_cmd[1]:+.2f} wz={last_cmd[2]:+.2f}",
                    mode=(
                        f"mode: {mode}"
                        + f"  |  arm: {last_arm_source or 'hold'}({device_name})"
                        + (f"  |  policy: {args_cli.policy_type}" if policy is not None else "")
                        + (f"  |  recorded: {current_recorded_demo_count}" if args_cli.record else "")
                        + ("  |  chassis LOCKED" if chassis_locked else "")
                    ),
                    bump=(
                        f"bump: BLOCKED (contact {chassis.last_contact_force:.0f} N)"
                        if chassis.blocked
                        else (f"bump: contact {chassis.last_contact_force:.0f} N"
                              f"  |  front clearance {chassis.last_forward_clearance:.2f} m")
                    ),
                    bridge=(
                        "ROS2 bridge: connected"
                        if getattr(bridge, "client_connected", False)
                        else "ROS2 bridge: waiting for client..."
                    ),
                )
            step += 1
            if args_cli.max_seconds > 0 and time.time() - t0 >= args_cli.max_seconds:
                print(f"[chassis] 到达 --max_seconds={args_cli.max_seconds}，退出")
                break
            if want_shutdown or bridge.consume_shutdown():
                want_shutdown = True
                if bridge.has_control():
                    # 同一批里 `record=success` 和 `shutdown` 一起到 → 这一帧还没轮到它们。
                    # 再走一帧把它们处理完（会顺带导出最后一条 episode），然后再退。
                    continue
                print("[ros2_chassis_teleop] 收到优雅退出请求 → 收尾刷盘后退出", flush=True)
                break
            if args_cli.step_hz > 0:
                # ⚠️ 节流必须用**固定的 target_dt**：dt 上面已经被换成"实测间隔"，
                #    再拿它乘 (step-1) 会算出天文数字的 sleep → 主循环直接睡死
                #    （表现：日志停住、GPU 0%、进程却还活着。这是实测踩到的坑。）
                time.sleep(max(0.0, target_dt - (time.time() - t0 - (step - 1) * target_dt)))

    # 自动命名的会话文件：一条 episode 都没录到 → 是个空壳，直接删掉（别在 datasets/sessions/ 堆垃圾）。
    # ⚠️ 只删**本次自动命名**的文件；显式 --dataset_file 给的绝不碰。
    if args_cli.record and _AUTO_SESSION_FILE and os.path.isfile(args_cli.dataset_file):
        try:
            _n_ok = env.recorder_manager.exported_successful_episode_count
            _n_bad = env.recorder_manager.exported_failed_episode_count
            if _n_ok == 0 and _n_bad == 0:
                os.remove(args_cli.dataset_file)
                print(f"[record] 本次没录到任何 episode → 已删掉空会话文件 {args_cli.dataset_file}")
        except Exception as _exc:  # noqa: BLE001
            print(f"[record] （空会话文件清理跳过：{type(_exc).__name__}）")

    bridge.stop()
    print(f"\n[chassis] 结束：共 {step} 步，最终位姿 {chassis.pose}")
    try:  # 释放单实例锁（下一个仿真才能起来）
        import os as _os

        _os.remove(_SIM_LOCK)
    except OSError:
        pass
    env.close()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
