#!/usr/bin/env python
"""用 Isaac Sim 6.0.1 standalone（不依赖 IsaacLab）打开厨房场景 USD。

用途：当 IsaacLab 3.0 的 ``--visualizer kit`` 路线窗口无响应时，仍能把场景显示出来。
默认只加载厨房 USD；加 ``--with-robots`` 会额外把躯干 + 两条 SO101 机械臂
（与 LeIsaac 厨房任务相同的资产和位姿）以 USD reference 方式贴进场景，
所以**不需要 IsaacLab 也能看到"厨房 + 双臂"**。

用法（在仓库根目录执行，需图形界面）:

    bash reproduce/open_kitchen_gui.sh                       # 厨房场景
    bash reproduce/open_kitchen_gui.sh --with-robots         # 厨房 + 躯干 + 双臂
    bash reproduce/open_kitchen_gui.sh --seconds 60          # 60 秒后自动退出

参数:
    --usd PATH     要打开的 USD（默认 assets/scenes/kitchen_with_orange/scene.usd）
    --with-robots  额外引用躯干 + 左右 SO101 机械臂
    --headless     不开窗口（仅验证调用链/资源可用性）
    --seconds N    打开后 N 秒自动退出（0 = 一直开到手动关窗），便于无人值守测试
"""
import argparse
import os
import sys
import time

from isaacsim import SimulationApp

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 与 source/leisaac/leisaac/tasks/lerobot_kitchen/lerobot_kitchen_env_cfg.py 保持一致
BODY_USD = os.path.join(REPO, "lerobot_robot_1", "SubUSDs", "lerobot_robot_no_arms_base.usd")
ARM_USD = os.path.join(REPO, "assets", "robots", "so101_follower.usd")
BODY_POSE = dict(pos=(2.29177, -0.84691, 0.01), rot=(0.7323, 0.0, 0.0, 0.6810))  # (w,x,y,z)
LEFT_ARM_POSE = dict(pos=(2.14527, -0.76399, 0.54), rot=(0.0, 0.0, 0.0, 1.0))
RIGHT_ARM_POSE = dict(pos=(2.48527, -0.79399, 0.54), rot=(0.0, 0.0, 0.0, 1.0))
VIEWER_EYE = (1.11527, 0.14601, 1.2)
VIEWER_LOOKAT = (1.71527, 0.54601, 1.0)

parser = argparse.ArgumentParser(description="Isaac Sim 6.0.1 打开场景 USD")
parser.add_argument(
    "--usd",
    default=os.path.join(REPO, "assets/scenes/kitchen_with_orange/scene.usd"),
)
parser.add_argument("--with-robots", action="store_true", help="额外引用躯干 + 双臂")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--seconds", type=float, default=0.0, help="0 = 一直运行到手动关窗")
parser.add_argument("--eye", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"), help="初始视角位置")
parser.add_argument("--lookat", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"), help="初始视角注视点")
# SimulationApp 只接受 "= 形式" 的 kit 参数；这里先摘掉自定义参数再交给 kit
args_cli, kit_argv = parser.parse_known_args()
sys.argv = [sys.argv[0]] + kit_argv

usd_path = os.path.abspath(args_cli.usd)
if not os.path.isfile(usd_path):
    print(f"[open] ERROR: USD 不存在: {usd_path}", flush=True)
    sys.exit(2)

print(f"[open] 启动 Isaac Sim 6.0.1 (headless={args_cli.headless}) ...", flush=True)
app = SimulationApp({"headless": args_cli.headless})

import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402


def _add_reference(stage, prim_path: str, source_usd: str, pose: dict) -> bool:
    """在 ``prim_path`` 下挂一个 Xform（带位姿），把 ``source_usd`` 引用到它的子 prim。

    分成「父 Xform 定位姿 + 子 prim 引用」两层，避免本地 xformOp 与被引用 USD 的
    defaultPrim 变换互相覆盖。
    """
    if not os.path.isfile(source_usd):
        print(f"[open] WARN: 资产不存在，跳过 {prim_path}: {source_usd}", flush=True)
        return False
    parent = UsdGeom.Xform.Define(stage, prim_path)
    parent.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pose["pos"]))
    w, x, y, z = pose["rot"]
    parent.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
    child = UsdGeom.Xform.Define(stage, prim_path + "/model")
    child.GetPrim().GetReferences().AddReference(source_usd)
    # 自检：引用是否真的把网格合进来了（引用失败/资产为空时这里会是 0）。
    # 注意要用 TraverseInstanceProxies —— 被引用的机器人子树里有 instance proxy，
    # 默认的 stage.Traverse() 看不到它们，会误报 0。
    rang = Usd.PrimRange(child.GetPrim(), Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
    n_mesh = sum(1 for p in rang if p.IsA(UsdGeom.Mesh))
    print(
        f"[open] 已引用 {prim_path} <- {os.path.relpath(source_usd, REPO)} (mesh={n_mesh})",
        flush=True,
    )
    return n_mesh > 0


t0 = time.time()
ok = omni.usd.get_context().open_stage(usd_path)
print(f"[open] open_stage({usd_path}) -> {ok} ({time.time() - t0:.1f}s)", flush=True)

if not ok:
    print("[open] ERROR: 场景打开失败", flush=True)
    app.close()
    sys.exit(1)

def _set_view(eye, target) -> None:
    """把视口相机摆到 eye → target（不同版本的函数位置不一样，都试一遍）。"""
    set_camera_view = None
    for mod, fn in (
        ("isaacsim.core.utils.viewports", "set_camera_view"),
        ("omni.isaac.core.utils.viewports", "set_camera_view"),
    ):
        try:
            set_camera_view = getattr(__import__(mod, fromlist=[fn]), fn)
            break
        except Exception:  # noqa: BLE001
            continue
    if set_camera_view is None:
        print("[open] 提示: 未找到 set_camera_view，可在窗口里按 F 键取景", flush=True)
        return
    try:
        set_camera_view(eye=eye, target=target)
        print(f"[open] 视角已设为 eye={tuple(eye)} lookat={tuple(target)}", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[open] WARN: 设置视角失败({e})，可在窗口里按 F 键取景", flush=True)


if args_cli.with_robots:
    stage = omni.usd.get_context().get_stage()
    _add_reference(stage, "/World/Body", BODY_USD, BODY_POSE)
    _add_reference(stage, "/World/Left_Robot", ARM_USD, LEFT_ARM_POSE)
    _add_reference(stage, "/World/Right_Robot", ARM_USD, RIGHT_ARM_POSE)
    _set_view(VIEWER_EYE, VIEWER_LOOKAT)

if args_cli.eye is not None and args_cli.lookat is not None:
    _set_view(tuple(args_cli.eye), tuple(args_cli.lookat))

print("[open] 场景已打开，开始渲染循环 ...", flush=True)
t0 = time.time()
frames = 0
while app.is_running():
    app.update()
    frames += 1
    if args_cli.seconds and time.time() - t0 > args_cli.seconds:
        print(f"[open] 到达 --seconds {args_cli.seconds}，自动退出", flush=True)
        break
    if frames % 600 == 0:
        fps = frames / max(time.time() - t0, 1e-6)
        print(f"[open] frames={frames} ({fps:.1f} fps)", flush=True)

print(f"[open] 循环结束，共 {frames} 帧", flush=True)
app.close()
