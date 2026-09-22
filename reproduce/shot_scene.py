#!/usr/bin/env python
"""场景多视角截图 —— 无人值守地把任意任务场景渲染成一组 PNG（用来"看"我搭的场景对不对）。

为什么不用 view_task_nocams.py 的 --capture：那个抓的是 **GUI 的活动 viewport**，
headless 下没有 viewport。这里改成直接建一个 `TiledCamera` 传感器，
用 `set_world_poses_from_view(eye, lookat)` 摆机位（IsaacLab 自带 API，不用手算四元数），
渲染几帧后把 RGB 存成 PNG —— headless 可用（`--enable_cameras`）。

用法（仓库根目录）:
    reproduce/shot_scene.py --task LeIsaac-SmartFactory-v0 --out /tmp/sf
    reproduce/shot_scene.py --task LeIsaac-SmartFactory-v0 --out /tmp/sf \\
        --view top=2.0,1.5,5.5,2.0,1.5,0.0 --view rack=3.4,2.6,1.3,2.4,0.8,0.5

    --view 格式: 名字=eye_x,eye_y,eye_z,lookat_x,lookat_y,lookat_z
    不给 --view 就用任务自带的默认机位（下面 DEFAULT_VIEWS 按任务名匹配，取不到就用全景机位）。
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="把任务场景渲染成多视角 PNG")
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
parser.add_argument("--out", type=str, default="/tmp/shot_scene")
parser.add_argument("--view", action="append", default=None, metavar="NAME=ex,ey,ez,tx,ty,tz")
parser.add_argument("--width", type=int, default=960)
parser.add_argument("--height", type=int, default=720)
parser.add_argument("--warmup", type=int, default=12, help="每个机位渲染前先步进几帧")
parser.add_argument("--settle_steps", type=int, default=0, help="先纯物理静置 N 步（看落位后的样子）")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True  # 渲染必须有相机
args_cli.headless = True  # 无窗口
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import gymnasium as gym  # noqa: E402
import leisaac  # noqa: E402, F401  注册任务
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from PIL import Image  # noqa: E402

#: 各任务的默认机位（名字 → (eye, lookat)）；--view 会覆盖/追加
DEFAULT_VIEWS: dict[str, list[tuple[str, tuple, tuple]]] = {
    "LeIsaac-SmartFactory-v0": [
        # ⚠️ 正下方俯视与场景 up 轴（Z）共线，look-at 的滚转是退化的（实测图会随机转 180°），
        #    所以给 eye 一点点 Y 偏移，等效"几乎正俯视"。
        ("top", (2.0, 1.30, 5.60), (2.0, 1.50, 0.0)),
        ("bird", (4.70, -1.80, 2.10), (1.95, 1.45, 0.30)),  # 斜俯视全场
        # 两个架子长边沿 Y、开口朝 +X → 正面特写要从**东侧（+X）**看
        ("pickup", (3.60, 0.85, 1.10), (2.28, 0.85, 0.52)),  # 收纳区双层架正视图
        ("transfer", (2.15, 0.85, 1.10), (0.78, 0.85, 0.40)),  # 转运架正视图
        ("startzone", (3.90, 2.75, 1.85), (1.20, 0.55, 0.35)),  # 从起点区一侧斜看：两个架子的**开口**都朝东
    ],
}


def _parse_view(text: str) -> tuple[str, tuple, tuple]:
    name, _, nums = text.partition("=")
    vals = [float(v) for v in nums.replace(" ", "").split(",")]
    if len(vals) != 6:
        raise SystemExit(f"--view 需要 6 个数（eye 3 + lookat 3），收到: {text!r}")
    return name.strip(), tuple(vals[:3]), tuple(vals[3:])


views = [(n, e, t) for n, e, t in DEFAULT_VIEWS.get(args_cli.task, [(("bird"), (4.0, -3.0, 3.0), (1.5, 1.5, 0.3))])]
if args_cli.view:
    views = [_parse_view(v) for v in args_cli.view]

env_cfg = parse_env_cfg(args_cli.task, device="cuda:0", num_envs=1)
env_cfg.recorders = None
env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()
if args_cli.settle_steps > 0:
    print(f"[shot] 物理静置 {args_cli.settle_steps} 步 ...")
    for _ in range(args_cli.settle_steps):
        env.sim.step(render=False)
        env.scene.update(env.physics_dt)

# 用**场景里**的预览相机：IsaacLab 3.0 的传感器要在 physics-ready 事件里初始化，
# 事后单独 new 一个 TiledCamera 拿不到初始化（实测 is_initialized=False）。
sensors = getattr(env.scene, "sensors", {}) or {}
cam_name = "preview" if "preview" in sensors else next(iter(sensors), None)
if cam_name is None:
    raise SystemExit("场景里没有相机传感器：请在任务场景 cfg 里加一个 TiledCameraCfg（例如 preview）")
cam = sensors[cam_name]
print(f"[shot] 使用场景相机 '{cam_name}' @ {cam.cfg.prim_path} is_initialized={cam.is_initialized}")

# ── M1 自检：场地部件是否真的都生成了、有没有碰撞 API（"搭出来了"的硬证据）──
def _verify_arena(root: str = "/World/envs/env_0/Arena") -> None:
    import omni.usd
    from pxr import Usd, UsdPhysics
    from leisaac.assets.scenes.smart_factory_layout import arena_boxes

    stage = omni.usd.get_context().get_stage()
    missing, no_col, unexpect_col = [], [], []
    for box in arena_boxes():
        prim = stage.GetPrimAtPath(f"{root}/{box.name}")
        if not prim or not prim.IsValid():
            missing.append(box.name)
            continue
        # ⚠️ 不能只看 CollisionAPI 是否存在：IsaacLab 的 collision_enabled=False 是
        #    把 API 挂上但 `physics:collisionEnabled=false`（属性存在、碰撞关闭）。
        #    所以要看**属性值**。
        has_col = False
        for p_ in Usd.PrimRange(prim):
            if not p_.HasAPI("PhysicsCollisionAPI"):
                continue
            attr = p_.GetAttribute("physics:collisionEnabled")
            if (not attr or not attr.IsValid()) or bool(attr.Get()):
                has_col = True
                break
        if box.collision and not has_col:
            no_col.append(box.name)
        if (not box.collision) and has_col:
            unexpect_col.append(box.name)
    print(
        f"[verify] 场地部件 {len(arena_boxes())} 个: 缺失={missing or '无'} "
        f"该有碰撞却没有={no_col or '无'} 不该有碰撞却开了={unexpect_col or '无'}"
    )
    print(f"[verify] {'✅ 全部生成且碰撞属性正确' if not (missing or no_col or unexpect_col) else '❌ 见上'}")


_verify_arena()

os.makedirs(args_cli.out, exist_ok=True)
for name, eye, target in views:
    cam.set_world_poses_from_view(
        torch.tensor([eye], device=env.device, dtype=torch.float32),
        torch.tensor([target], device=env.device, dtype=torch.float32),
    )
    for _ in range(args_cli.warmup):
        env.sim.render()
        env.scene.update(env.physics_dt)
    out = cam.data.output
    rgb = out.get("rgb") if isinstance(out, dict) else None
    if rgb is None:
        print(f"[shot] {name}: ❌ 没有 rgb 输出")
        continue
    arr = rgb[0].detach().cpu().numpy()
    finite = np.isfinite(arr)
    print(
        f"[shot] {name}: eye={eye} lookat={target} shape={arr.shape} "
        f"min={np.nanmin(arr):.3f} max={np.nanmax(arr):.3f} mean={np.nanmean(arr):.3f} "
        f"非有限像素={int((~finite).sum())}"
    )
    if arr.dtype != np.uint8:
        arr = (np.nan_to_num(arr[..., :3]).clip(0.0, 1.0) * 255.0).astype(np.uint8)
    path = os.path.join(args_cli.out, f"{name}.png")
    Image.fromarray(arr[..., :3]).save(path)
    print(f"[shot] {name}: 已保存 {path}")

env.close()
simulation_app.close()
